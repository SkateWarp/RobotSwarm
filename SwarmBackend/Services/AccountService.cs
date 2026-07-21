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
        var normalizedEmail = AccountRequestValidator.CanonicalEmail(email ?? string.Empty);
        await using var transaction = await _dataContext.Database.BeginTransactionAsync(
            IsolationLevel.Serializable);
        try
        {
            var account = await FindAccountWithEmail(
                normalizedEmail,
                CancellationToken.None);

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
        return await CreateStandalone(request, Role.User, CancellationToken.None);
    }

    public async Task<Result<AccountResponse>> Create(AccountRequest request, Role role)
    {
        return await CreateStandalone(request, role, CancellationToken.None);
    }

    private async Task<Result<AccountResponse>> CreateStandalone(
        AccountRequest request,
        Role role,
        CancellationToken cancellationToken)
    {
        await using var transaction = await _dataContext.Database.BeginTransactionAsync(
            IsolationLevel.ReadCommitted,
            cancellationToken);
        await AccountResourceLock.AcquireEmailMutation(
            _dataContext,
            cancellationToken);

        var result = await CreateCore(request, role, cancellationToken);
        if (Succeeded(result))
        {
            await transaction.CommitAsync(cancellationToken);
        }

        return result;
    }

    public async Task<Result<AccountResponse>> CreateAuthorized(
        int actorAccountId,
        ClaimsPrincipal principal,
        AccountRequest request,
        Role role,
        CancellationToken cancellationToken = default)
    {
        await using var transaction = await _dataContext.Database.BeginTransactionAsync(
            IsolationLevel.ReadCommitted,
            cancellationToken);
        // This overload is used by the administrator-only creation route.
        // Join the same global order even when the new account is a user.
        await AccountResourceLock.AcquireAdministratorMutation(
            _dataContext,
            cancellationToken);
        await AccountResourceLock.AcquireEmailMutation(
            _dataContext,
            cancellationToken);

        var actorIsCurrent = await AccountResourceLock.AcquireSharedAndValidate(
            _dataContext,
            actorAccountId,
            principal,
            cancellationToken);
        if (!actorIsCurrent)
        {
            return UnauthorizedMutation<AccountResponse>();
        }

        var result = await CreateCore(request, role, cancellationToken);
        if (Succeeded(result))
        {
            await transaction.CommitAsync(cancellationToken);
        }
        return result;
    }

    private async Task<Result<AccountResponse>> CreateCore(
        AccountRequest request,
        Role role,
        CancellationToken cancellationToken)
    {
        if (!Enum.IsDefined(role))
        {
            return new Result<AccountResponse>(new Exception("Invalid account role."));
        }

        if (!AccountRequestValidator.TryNormalize(
                request,
                out var normalized,
                out var errors))
        {
            return InvalidAccount(errors);
        }

        var existing = await EmailIsInUse(
            normalized.Email,
            accountId: null,
            cancellationToken);
        if (existing)
        {
            return EmailInUse();
        }

        var account = new Account
        {
            Email = normalized.Email,
            Created = DateTime.Now,
            FirstName = normalized.FirstName,
            Enabled = true,
            LastName = normalized.LastName,
            Verified = DateTime.Now,
            PasswordHash = BC.HashPassword(normalized.Password),
            Role = role,
        };

        _dataContext.Accounts.Add(account);
        try
        {
            await _dataContext.SaveChangesAsync(cancellationToken);
        }
        catch (DbUpdateException exception) when (IsEmailUniqueViolation(exception))
        {
            return EmailInUse();
        }

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
        var refreshTokenLifetime = configuration.GetValue<double?>(
            "AppSettings:RefreshTokenExpiresInDays") ?? 7;
        if (double.IsNaN(refreshTokenLifetime)
            || double.IsInfinity(refreshTokenLifetime)
            || refreshTokenLifetime <= 0)
        {
            throw new InvalidOperationException(
                "AppSettings:RefreshTokenExpiresInDays must be greater than zero.");
        }

        return new RefreshToken
        {
            CreatedByIp = ipAddress ?? string.Empty,
            Created = DateTime.Now,
            Expires = DateTime.Now.AddDays(refreshTokenLifetime),
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

    public Task<Result<AccountResponse>> Update(int accountId, AccountRequest request)
    {
        return UpdateCore(
            actorAccountId: null,
            principal: null,
            accountId,
            request,
            CancellationToken.None);
    }

    public Task<Result<AccountResponse>> UpdateAuthorized(
        int actorAccountId,
        ClaimsPrincipal principal,
        int accountId,
        AccountRequest request,
        CancellationToken cancellationToken = default)
    {
        return UpdateCore(
            actorAccountId,
            principal,
            accountId,
            request,
            cancellationToken);
    }

    private async Task<Result<AccountResponse>> UpdateCore(
        int? actorAccountId,
        ClaimsPrincipal? principal,
        int accountId,
        AccountRequest request,
        CancellationToken cancellationToken)
    {
        if (!AccountRequestValidator.TryNormalize(
                request,
                out var normalized,
                out var errors))
        {
            return InvalidAccount(errors);
        }

        request = normalized;
        await using var transaction = await _dataContext.Database.BeginTransactionAsync(
            IsolationLevel.ReadCommitted,
            cancellationToken);
        if (actorAccountId.HasValue)
        {
            if (PrincipalIsAdministrator(principal))
            {
                await AccountResourceLock.AcquireAdministratorMutation(
                    _dataContext,
                    cancellationToken);
            }
        }

        await AccountResourceLock.AcquireEmailMutation(
            _dataContext,
            cancellationToken);

        if (actorAccountId.HasValue)
        {
            if (principal == null
                || !await AccountResourceLock.AcquireActorAndTarget(
                    _dataContext,
                    actorAccountId.Value,
                    principal,
                    accountId,
                    cancellationToken))
            {
                return UnauthorizedMutation<AccountResponse>();
            }
        }
        else
        {
            await AccountResourceLock.AcquireExclusive(
                _dataContext,
                accountId,
                cancellationToken);
        }

        Account? account;
        if (_dataContext.Database.IsRelational())
        {
            account = await _dataContext.Accounts
                .FromSqlInterpolated($"""
                    SELECT *
                    FROM "Accounts"
                    WHERE "Id" = {accountId}
                    FOR UPDATE
                    """)
                .SingleOrDefaultAsync();
            if (account != null)
            {
                await _dataContext.Entry(account)
                    .Collection(candidate => candidate.RefreshTokens)
                    .LoadAsync();
            }
        }
        else
        {
            account = await _dataContext.Accounts
                .Include(candidate => candidate.RefreshTokens)
                .SingleOrDefaultAsync(candidate => candidate.Id == accountId);
        }

        if (account == null)
        {
            return new Result<AccountResponse>(new Exception("Cuenta no encontrada"));
        }

        if (await EmailIsInUse(request.Email, accountId, cancellationToken))
        {
            return EmailInUse();
        }

        account.FirstName = request.FirstName;
        account.LastName = request.LastName;
        account.Email = request.Email;
        account.PasswordHash = BC.HashPassword(request.Password);
        account.Updated = DateTime.UtcNow;
        RevokeRefreshTokens(account, account.Updated.Value);
        try
        {
            await _dataContext.SaveChangesAsync(cancellationToken);
        }
        catch (DbUpdateException exception) when (IsEmailUniqueViolation(exception))
        {
            return EmailInUse();
        }
        await transaction.CommitAsync(cancellationToken);

        return AccountResponse.From(account);
    }

    public Task<Result<AccountResponse>> Update(int accountId, AccountPatchRequest request)
    {
        return UpdateCore(
            actorAccountId: null,
            principal: null,
            accountId,
            request,
            CancellationToken.None);
    }

    public Task<Result<AccountResponse>> UpdateAuthorized(
        int actorAccountId,
        ClaimsPrincipal principal,
        int accountId,
        AccountPatchRequest request,
        CancellationToken cancellationToken = default)
    {
        return UpdateCore(
            actorAccountId,
            principal,
            accountId,
            request,
            cancellationToken);
    }

    private async Task<Result<AccountResponse>> UpdateCore(
        int? actorAccountId,
        ClaimsPrincipal? principal,
        int accountId,
        AccountPatchRequest request,
        CancellationToken cancellationToken)
    {
        if (!AccountRequestValidator.TryNormalize(
                request,
                out var normalized,
                out var errors))
        {
            return InvalidAccount(errors);
        }

        request = normalized;
        await using var transaction = await _dataContext.Database.BeginTransactionAsync(
            IsolationLevel.ReadCommitted,
            cancellationToken);
        if (request.Role.HasValue
            || actorAccountId.HasValue && PrincipalIsAdministrator(principal))
        {
            await AccountResourceLock.AcquireAdministratorMutation(
                _dataContext,
                cancellationToken);
        }

        if (request.Email != null)
        {
            await AccountResourceLock.AcquireEmailMutation(
                _dataContext,
                cancellationToken);
        }

        if (actorAccountId.HasValue)
        {
            if (principal == null
                || !await AccountResourceLock.AcquireActorAndTarget(
                    _dataContext,
                    actorAccountId.Value,
                    principal,
                    accountId,
                    cancellationToken))
            {
                return UnauthorizedMutation<AccountResponse>();
            }
        }
        else
        {
            await AccountResourceLock.AcquireExclusive(
                _dataContext,
                accountId,
                cancellationToken);
        }

        Account? account;
        if (_dataContext.Database.IsRelational())
        {
            List<Account> lockedAccounts;
            if (request.Role.HasValue)
            {
                lockedAccounts = await _dataContext.Accounts
                    .FromSqlInterpolated($"""
                        SELECT *
                        FROM "Accounts"
                        WHERE "Id" = {accountId}
                           OR ("Enabled" = TRUE AND "Role" = {(int)Role.Admin})
                        ORDER BY "Id"
                        FOR UPDATE
                        """)
                    .ToListAsync();
            }
            else
            {
                lockedAccounts = await _dataContext.Accounts
                    .FromSqlInterpolated($"""
                        SELECT *
                        FROM "Accounts"
                        WHERE "Id" = {accountId}
                        FOR UPDATE
                        """)
                    .ToListAsync();
            }

            account = lockedAccounts.SingleOrDefault(candidate => candidate.Id == accountId);
            if (account != null)
            {
                await _dataContext.Entry(account)
                    .Collection(candidate => candidate.RefreshTokens)
                    .LoadAsync();
            }
        }
        else
        {
            account = await _dataContext.Accounts
                .Include(candidate => candidate.RefreshTokens)
                .SingleOrDefaultAsync(candidate => candidate.Id == accountId);
        }

        if (account == null)
        {
            return new Result<AccountResponse>(new Exception("Cuenta no encontrada"));
        }

        if (request.Email != null
            && await EmailIsInUse(request.Email, accountId, cancellationToken))
        {
            return EmailInUse();
        }

        var securityChanged = request.Password != null
            || request.Email != null
                && !AccountRequestValidator.CanonicalEmail(account.Email)
                    .Equals(request.Email, StringComparison.Ordinal)
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

        try
        {
            await _dataContext.SaveChangesAsync(cancellationToken);
        }
        catch (DbUpdateException exception) when (IsEmailUniqueViolation(exception))
        {
            return EmailInUse();
        }
        await transaction.CommitAsync(cancellationToken);

        return AccountResponse.From(account);
    }

    public async Task<bool> Delete(int accountId)
    {
        var outcome = await DeleteCore(
            actorAccountId: null,
            principal: null,
            accountId,
            CancellationToken.None);
        return outcome.Deleted;
    }

    public async Task<Result<bool>> DeleteAuthorized(
        int actorAccountId,
        ClaimsPrincipal principal,
        int accountId,
        CancellationToken cancellationToken = default)
    {
        var outcome = await DeleteCore(
            actorAccountId,
            principal,
            accountId,
            cancellationToken);
        return outcome.Authorized
            ? outcome.Deleted
            : UnauthorizedMutation<bool>();
    }

    private async Task<(bool Authorized, bool Deleted)> DeleteCore(
        int? actorAccountId,
        ClaimsPrincipal? principal,
        int accountId,
        CancellationToken cancellationToken)
    {
        await using var transaction = await _dataContext.Database.BeginTransactionAsync(
            IsolationLevel.ReadCommitted,
            cancellationToken);
        await AccountResourceLock.AcquireAdministratorMutation(
            _dataContext,
            cancellationToken);
        if (actorAccountId.HasValue)
        {
            if (principal == null
                || !await AccountResourceLock.AcquireActorAndTarget(
                    _dataContext,
                    actorAccountId.Value,
                    principal,
                    accountId,
                    cancellationToken))
            {
                return (Authorized: false, Deleted: false);
            }
        }
        else
        {
            await AccountResourceLock.AcquireExclusive(
                _dataContext,
                accountId,
                cancellationToken);
        }

        Account? account;
        if (_dataContext.Database.IsRelational())
        {
            // Lock administrators in a stable order as well as the requested
            // account. This keeps the last-admin rule intact without making
            // session cleanup depend on a broad serializable snapshot.
            var lockedAccounts = await _dataContext.Accounts
                .FromSqlInterpolated($"""
                    SELECT *
                    FROM "Accounts"
                    WHERE "Id" = {accountId}
                       OR ("Enabled" = TRUE AND "Role" = {(int)Role.Admin})
                    ORDER BY "Id"
                    FOR UPDATE
                    """)
                .ToListAsync();
            account = lockedAccounts.SingleOrDefault(candidate => candidate.Id == accountId);
            if (account != null)
            {
                await _dataContext.Entry(account)
                    .Collection(candidate => candidate.RefreshTokens)
                    .LoadAsync();
            }
        }
        else
        {
            account = await _dataContext.Accounts
                .Include(candidate => candidate.RefreshTokens)
                .SingleOrDefaultAsync(candidate => candidate.Id == accountId);
        }

        if (account == null)
        {
            return (Authorized: true, Deleted: false);
        }

        if (account.Enabled && account.Role == Role.Admin)
        {
            var enabledAdminCount = await _dataContext.Accounts.CountAsync(candidate =>
                candidate.Enabled && candidate.Role == Role.Admin);
            if (enabledAdminCount <= 1)
            {
                return (Authorized: true, Deleted: false);
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

        List<SimulationSession> sessions;
        if (_dataContext.Database.IsRelational())
        {
            sessions = await _dataContext.SimulationSessions
                .FromSqlInterpolated($"""
                    SELECT *
                    FROM "SimulationSessions"
                    WHERE "AccountId" = {accountId}
                      AND ("State" < {(int)SimulationSessionState.Stopped}
                        OR "State" = {(int)SimulationSessionState.Failed}
                        OR "State" = {(int)SimulationSessionState.Expired})
                    ORDER BY "Id"
                    FOR UPDATE
                    """)
                .ToListAsync();
            foreach (var session in sessions)
            {
                await _dataContext.Entry(session)
                    .Collection(candidate => candidate.Commands)
                    .LoadAsync();
            }
        }
        else
        {
            sessions = await _dataContext.SimulationSessions
                .Include(session => session.Commands)
                .Where(session => session.AccountId == accountId
                    && (session.State < SimulationSessionState.Stopped
                        || session.State == SimulationSessionState.Failed
                        || session.State == SimulationSessionState.Expired))
                .ToListAsync();
        }
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

            var resourceKnownPresent = session.State < SimulationSessionState.Stopped;
            if (session.State < SimulationSessionState.Stopped
                && session.State != SimulationSessionState.Stopping)
            {
                session.State = SimulationSessionState.Stopping;
                session.UpdatedAt = now;
                session.Revision++;
            }

            var cleanupCommand = TerminalCleanupPolicy.TryQueue(
                session,
                now,
                "sys:account-disabled",
                resourceKnownPresent);
            if (cleanupCommand == null)
            {
                continue;
            }

            _dataContext.WorkerCommands.Add(cleanupCommand);
        }

        await _dataContext.SaveChangesAsync();
        await transaction.CommitAsync(cancellationToken);
        return (Authorized: true, Deleted: true);
    }

    private static Result<T> UnauthorizedMutation<T>()
    {
        return new Result<T>(new UnauthorizedAccessException(
            "The account token is no longer active."));
    }

    private static bool PrincipalIsAdministrator(ClaimsPrincipal? principal)
    {
        return principal?.FindFirst(ClaimTypes.Role)?.Value
            == Role.Admin.ToString();
    }

    private static void RevokeRefreshTokens(Account account, DateTime revokedAt)
    {
        foreach (var token in account.RefreshTokens.Where(token => token.IsActive))
        {
            token.Revoked = revokedAt;
            token.RevokedByIp = "account-security-change";
        }
    }

    private async Task<Account?> FindAccountWithEmail(
        string normalizedEmail,
        CancellationToken cancellationToken)
    {
        if (_dataContext.Database.ProviderName
            == "Npgsql.EntityFrameworkCore.PostgreSQL")
        {
            return await _dataContext.Accounts.FirstOrDefaultAsync(account =>
                account.NormalizedEmail == normalizedEmail,
                cancellationToken);
        }

        var accounts = await _dataContext.Accounts.ToListAsync(cancellationToken);
        return accounts.FirstOrDefault(account =>
            AccountRequestValidator.CanonicalEmail(account.Email) == normalizedEmail);
    }

    private async Task<bool> EmailIsInUse(
        string normalizedEmail,
        int? accountId,
        CancellationToken cancellationToken)
    {
        if (_dataContext.Database.ProviderName
            == "Npgsql.EntityFrameworkCore.PostgreSQL")
        {
            return await _dataContext.Accounts.AnyAsync(account =>
                    account.NormalizedEmail == normalizedEmail
                    && (!accountId.HasValue || account.Id != accountId.Value),
                cancellationToken);
        }

        var accounts = await _dataContext.Accounts
            .AsNoTracking()
            .ToListAsync(cancellationToken);
        return accounts.Any(account =>
            (!accountId.HasValue || account.Id != accountId.Value)
            && AccountRequestValidator.CanonicalEmail(account.Email) == normalizedEmail);
    }

    private static bool Succeeded<T>(Result<T> result)
    {
        return result.Match(_ => true, _ => false);
    }

    private static Result<AccountResponse> InvalidAccount(
        Dictionary<string, string[]> errors)
    {
        var message = errors.Values
            .SelectMany(messages => messages)
            .FirstOrDefault() ?? "Los datos de la cuenta no son válidos.";
        return new Result<AccountResponse>(new Exception(message));
    }

    private static Result<AccountResponse> EmailInUse()
    {
        return new Result<AccountResponse>(new Exception("Correo en uso."));
    }

    private static bool IsEmailUniqueViolation(Exception exception)
    {
        for (var current = exception; current != null; current = current.InnerException)
        {
            if (current is PostgresException postgres
                && postgres.SqlState == PostgresErrorCodes.UniqueViolation
                && postgres.ConstraintName == "IX_Accounts_NormalizedEmail")
            {
                return true;
            }
        }

        return false;
    }
}
