using LanguageExt.Common;
using Microsoft.AspNetCore.Identity;
using Microsoft.EntityFrameworkCore;
using Microsoft.Extensions.Configuration;
using Microsoft.IdentityModel.Tokens;
using SwarmBackend.Entities;
using SwarmBackend.Helpers;
using SwarmBackend.Interfaces;
using SwarmBackend.Models;
using System.Data;
using System.IdentityModel.Tokens.Jwt;
using System.Net;
using System.Security.Claims;
using System.Security.Cryptography;
using System.Text.Json;
using System.Text.RegularExpressions;
using Npgsql;
using BC = BCrypt.Net.BCrypt;


namespace SwarmBackend.Services;

public class AccountService : IAccountService
{
    private readonly DataContext _dataContext;
    private readonly IConfiguration configuration;

    public AccountService(DataContext dataContext, IConfiguration configuration)
    {
        _dataContext = dataContext;
        this.configuration = configuration;
    }

    public async Task<Result<AuthenticateResponse>> Authenticate(string email, string password, string? ipAddress)
    {
        await using var transaction = await _dataContext.Database.BeginTransactionAsync(
            IsolationLevel.Serializable);
        try
        {
            var account = await _dataContext.Accounts
                .Where(x => x.Email == email)
                .FirstOrDefaultAsync();

            if (account == null)
            {
                return new Result<AuthenticateResponse>(new Exception("Usuario no encontrado"));
            }

            if (!account.Enabled)
            {
                return new Result<AuthenticateResponse>(new Exception("Cuenta deshabilitada"));
            }

            if (!BC.Verify(password, account.PasswordHash))
            {
                return new Result<AuthenticateResponse>(new Exception("Contraseña invalida"));
            }

            var token = CreateToken(account);
            var refreshToken = GenerateRefreshToken(ipAddress);
            account.RefreshTokens.Add(refreshToken);
            await _dataContext.SaveChangesAsync();
            await transaction.CommitAsync();

            return AuthenticateResponse.From(account, token, refreshToken.Token);
        }
        catch (PostgresException exception)
            when (exception.SqlState == PostgresErrorCodes.SerializationFailure)
        {
            return AuthenticationChanged();
        }
        catch (DbUpdateException exception)
            when (exception.InnerException is PostgresException
            {
                SqlState: PostgresErrorCodes.SerializationFailure
            })
        {
            return AuthenticationChanged();
        }
    }

    public async Task<Result<AccountResponse>> Create(AccountRequest request)
    {
        return await Create(request, Role.User);
    }

    public async Task<Result<AccountResponse>> Create(AccountRequest request, Role role)
    {
        string emailRegex = @"^[a-zA-Z0-9._%+-]+@[a-zA-Z0-9.-]+\.[a-zA-Z]{2,}$";

        if (!Enum.IsDefined(role))
        {
            return new Result<AccountResponse>(new Exception("Invalid account role."));
        }

        if (string.IsNullOrEmpty(request.Email))
        {
            return new Result<AccountResponse>(new Exception("Email cannot be empty."));
        }

        if (!Regex.IsMatch(request.Email, emailRegex))
        {
            return new Result<AccountResponse>(new Exception("Invalid email format."));
        }

        var existing = await _dataContext.Accounts.AnyAsync(x => x.Email == request.Email);
        if (existing)
        {
            return new Result<AccountResponse>(new Exception("Correo en uso"));
        }

        var account = new Account
        {
            Email = request.Email,
            Created = DateTime.Now,
            FirstName = request.FirstName,
            Enabled = true,
            LastName = request.LastName,
            Verified = DateTime.Now,
            PasswordHash = BC.HashPassword(request.Password),
            Role = role,
        };

        _dataContext.Accounts.Add(account);
        await _dataContext.SaveChangesAsync();

        return AccountResponse.From(account);
    }

    public async Task<Result<AuthenticateResponse>> RefreshTokenAsync(string refreshToken, string? ipAddress)
    {
        await using var transaction = await _dataContext.Database.BeginTransactionAsync(
            IsolationLevel.Serializable);
        try
        {
            var storedToken = await _dataContext.RefreshTokens
                .Include(x => x.Account)
                .SingleOrDefaultAsync(x => x.Token == refreshToken);

            if (storedToken?.Account == null
                || !storedToken.IsActive
                || !storedToken.Account.Enabled)
            {
                return InvalidRefreshToken();
            }

            var account = storedToken.Account;
            var newAccessToken = CreateToken(account);
            var newRefreshToken = GenerateRefreshToken(ipAddress);

            storedToken.Revoked = DateTime.Now;
            storedToken.RevokedByIp = ipAddress ?? string.Empty;
            storedToken.ReplacedByToken = newRefreshToken.Token;
            account.RefreshTokens.Add(newRefreshToken);
            await _dataContext.SaveChangesAsync();
            await transaction.CommitAsync();

            return AuthenticateResponse.From(account, newAccessToken, newRefreshToken.Token);
        }
        catch (PostgresException exception)
            when (exception.SqlState == PostgresErrorCodes.SerializationFailure)
        {
            return InvalidRefreshToken();
        }
        catch (DbUpdateException exception)
            when (exception.InnerException is PostgresException
            {
                SqlState: PostgresErrorCodes.SerializationFailure
            })
        {
            return InvalidRefreshToken();
        }
    }

    private static Result<AuthenticateResponse> InvalidRefreshToken()
    {
        return new Result<AuthenticateResponse>(new Exception("Refresh token inválido"));
    }

    private static Result<AuthenticateResponse> AuthenticationChanged()
    {
        return new Result<AuthenticateResponse>(
            new Exception("La cuenta cambió durante el inicio de sesión. Intente nuevamente."));
    }


    private string CreateToken(Account account)
    {
        var signingCredentials = GetSigningCredentials();
        var claims = GetClaims(account);
        var tokenOptions = GenerateTokenOptions(signingCredentials, claims);
        return new JwtSecurityTokenHandler().WriteToken(tokenOptions);
    }

    private SigningCredentials GetSigningCredentials()
    {
        var secret = new SymmetricSecurityKey(GetSigningKey());
        return new SigningCredentials(secret, SecurityAlgorithms.HmacSha256);
    }

    private static List<Claim> GetClaims(Account account)
    {
        var claims = new List<Claim>
            {
                new(ClaimTypes.Name, account.FirstName),
                new("id", account.Id.ToString()),
                new(ClaimTypes.Role, account.Role.ToString()),
                new("account_version", (account.Updated?.Ticks ?? 0L).ToString()),
            };

        return claims;
    }

    private JwtSecurityToken GenerateTokenOptions(SigningCredentials signingCredentials, List<Claim> claims)
    {
        var jwtSettings = configuration.GetSection("AppSettings");
        var tokenOptions = new JwtSecurityToken
        (
            issuer: jwtSettings["Issuer"],
            audience: jwtSettings["Issuer"],
            claims: claims,
            expires: DateTime.Now.AddMinutes(Convert.ToDouble(jwtSettings["ExpiresIn"])),
            signingCredentials: signingCredentials
        );
        return tokenOptions;
    }

    private RefreshToken GenerateRefreshToken(string? ipAddress)
    {
        var randomNumber = new byte[64];
        using var rng = RandomNumberGenerator.Create();
        rng.GetBytes(randomNumber);
        var token = Convert.ToBase64String(randomNumber);
        var jwtSettings = configuration.GetSection("AppSettings");

        return new RefreshToken
        {
            CreatedByIp = ipAddress ?? string.Empty,
            Created = DateTime.Now,
            Expires = DateTime.Now.AddMinutes(Convert.ToDouble(jwtSettings["ExpiresIn"])),
            Token = token,
        };
    }

    public ClaimsPrincipal? GetPrincipalFromExpiredToken(string? token)
    {
        var tokenValidationParameters = new TokenValidationParameters
        {
            ValidateAudience = false,
            ValidateIssuer = false,
            ValidateIssuerSigningKey = true,
            IssuerSigningKey = new SymmetricSecurityKey(GetSigningKey()),
            ValidateLifetime = false
        };

        var tokenHandler = new JwtSecurityTokenHandler();
        var principal = tokenHandler.ValidateToken(token, tokenValidationParameters, out SecurityToken securityToken);
        if (securityToken is not JwtSecurityToken jwtSecurityToken || !jwtSecurityToken.Header.Alg.Equals(SecurityAlgorithms.HmacSha256, StringComparison.InvariantCultureIgnoreCase))
            throw new SecurityTokenException("Invalid token");

        return principal;

    }

    private byte[] GetSigningKey()
    {
        var secret = configuration["AppSettings:Secret"];
        if (string.IsNullOrWhiteSpace(secret))
        {
            throw new InvalidOperationException("AppSettings:Secret is required.");
        }

        return Convert.FromBase64String(secret);
    }

    public async Task<IEnumerable<AccountResponse>> GetAll(int? accountId, Role? role)
    {
        // If user is not admin and we have an accountId, only return their own account
        if (role.HasValue && role != Role.Admin && accountId.HasValue)
        {
            return await _dataContext.Accounts
                .Where(x => x.Id == accountId.Value)
                .Select(x => AccountResponse.From(x))
                .ToListAsync();
        }

        // Admin or internal operations (null role) can see all accounts
        return await _dataContext.Accounts
            .Select(x => AccountResponse.From(x))
            .ToListAsync();
    }

    public async Task<Result<AccountResponse>> GetById(int accountId)
    {
        var account = await _dataContext.Accounts
            .Where(x => x.Id == accountId)
            .FirstOrDefaultAsync();

        if (account == null)
        {
            return new Result<AccountResponse>(new Exception("Cuenta no encontrada"));
        }

        return AccountResponse.From(account);
    }

    public async Task<Result<AccountResponse>> Update(int accountId, AccountRequest request)
    {
        await using var transaction = await _dataContext.Database.BeginTransactionAsync(
            IsolationLevel.Serializable);
        var account = await _dataContext.Accounts
            .Include(x => x.RefreshTokens)
            .Where(x => x.Id == accountId)
            .FirstOrDefaultAsync();

        if (account == null)
        {
            return new Result<AccountResponse>(new Exception("Cuenta no encontrada"));
        }

        account.FirstName = request.FirstName;
        account.LastName = request.LastName;
        account.Email = request.Email;
        account.PasswordHash = BC.HashPassword(request.Password);
        account.Updated = DateTime.UtcNow;
        RevokeRefreshTokens(account, account.Updated.Value);
        await _dataContext.SaveChangesAsync();
        await transaction.CommitAsync();

        return AccountResponse.From(account);
    }

    public async Task<Result<AccountResponse>> Update(int accountId, AccountPatchRequest request)
    {
        await using var transaction = await _dataContext.Database.BeginTransactionAsync(
            IsolationLevel.Serializable);
        var account = await _dataContext.Accounts
          .Include(x => x.RefreshTokens)
          .Where(x => x.Id == accountId)
          .FirstOrDefaultAsync();

        if (account == null)
        {
            return new Result<AccountResponse>(new Exception("Cuenta no encontrada"));
        }

        var securityChanged = request.Password != null
            || request.Role.HasValue && request.Role.Value != account.Role;
        if (account.Enabled
            && account.Role == Role.Admin
            && request.Role.HasValue
            && request.Role.Value != Role.Admin)
        {
            var enabledAdminCount = await _dataContext.Accounts.CountAsync(candidate =>
                candidate.Enabled && candidate.Role == Role.Admin);
            if (enabledAdminCount <= 1)
            {
                return new Result<AccountResponse>(
                    new Exception("The last enabled administrator cannot be demoted."));
            }
        }

        account.FirstName = request.FirstName ?? account.FirstName;
        account.LastName = request.LastName ?? account.LastName;
        account.Email = request.Email ?? account.Email;
        account.Role = request.Role ?? account.Role;
        account.PasswordHash = request.Password != null
            ? BC.HashPassword(request.Password)
            : account.PasswordHash;
        if (securityChanged)
        {
            account.Updated = DateTime.UtcNow;
            RevokeRefreshTokens(account, account.Updated.Value);
        }

        await _dataContext.SaveChangesAsync();
        await transaction.CommitAsync();

        return AccountResponse.From(account);
    }

    public async Task<bool> Delete(int accountId)
    {
        await using var transaction = await _dataContext.Database.BeginTransactionAsync(
            IsolationLevel.Serializable);
        var account = await _dataContext.Accounts
          .Include(x => x.RefreshTokens)
          .Where(x => x.Id == accountId)
          .FirstOrDefaultAsync();

        if (account == null)
        {
            return false;
        }

        if (account.Enabled && account.Role == Role.Admin)
        {
            var enabledAdminCount = await _dataContext.Accounts.CountAsync(candidate =>
                candidate.Enabled && candidate.Role == Role.Admin);
            if (enabledAdminCount <= 1)
            {
                return false;
            }
        }

        var now = DateTime.UtcNow;
        account.Enabled = false;
        account.Updated = now;
        RevokeRefreshTokens(account, now);

        var leases = await _dataContext.ViewerLeases
            .Where(lease => lease.AccountId == accountId && !lease.RevokedAt.HasValue)
            .ToListAsync();
        foreach (var lease in leases)
        {
            lease.RevokedAt = now;
        }

        var sessions = await _dataContext.SimulationSessions
            .Include(session => session.Commands)
            .Where(session => session.AccountId == accountId
                && (session.State < SimulationSessionState.Stopped
                    || session.State == SimulationSessionState.Failed))
            .ToListAsync();
        foreach (var session in sessions)
        {
            if (session.State == SimulationSessionState.Queued
                || !session.ComputeWorkerId.HasValue)
            {
                session.State = SimulationSessionState.Stopped;
                session.StoppedAt ??= now;
                session.UpdatedAt = now;
                session.Revision++;
                continue;
            }

            if (session.State < SimulationSessionState.Stopped
                && session.State != SimulationSessionState.Stopping)
            {
                session.State = SimulationSessionState.Stopping;
                session.UpdatedAt = now;
                session.Revision++;
            }

            var stopAlreadyQueued = session.Commands.Any(command =>
                command.Type == WorkerCommandType.StopSession
                && command.State is WorkerCommandState.Pending
                    or WorkerCommandState.Dispatched
                    or WorkerCommandState.Acknowledged
                    or WorkerCommandState.Running);
            if (stopAlreadyQueued)
            {
                continue;
            }

            var attempt = session.Commands.Count(command =>
                command.Type == WorkerCommandType.StopSession) + 1;
            var sequence = session.Commands.Count == 0
                ? 1
                : session.Commands.Max(command => command.Sequence) + 1;
            session.Commands.Add(new WorkerCommand
            {
                ComputeWorkerId = session.ComputeWorkerId,
                Type = WorkerCommandType.StopSession,
                State = WorkerCommandState.Pending,
                IdempotencyKey = $"sys:account-disabled:{session.Id:N}:{attempt}",
                Sequence = sequence,
                Payload = JsonDocument.Parse("{}"),
                CreatedAt = now,
                UpdatedAt = now
            });
        }

        await _dataContext.SaveChangesAsync();
        await transaction.CommitAsync();
        return true;
    }

    private static void RevokeRefreshTokens(Account account, DateTime revokedAt)
    {
        foreach (var token in account.RefreshTokens.Where(token => token.IsActive))
        {
            token.Revoked = revokedAt;
            token.RevokedByIp = "account-security-change";
        }
    }
}
