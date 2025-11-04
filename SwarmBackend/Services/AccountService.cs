using LanguageExt.Common;
using Microsoft.AspNetCore.Identity;
using Microsoft.EntityFrameworkCore;
using Microsoft.Extensions.Configuration;
using Microsoft.IdentityModel.Tokens;
using SwarmBackend.Entities;
using SwarmBackend.Helpers;
using SwarmBackend.Interfaces;
using SwarmBackend.Models;
using System.IdentityModel.Tokens.Jwt;
using System.Net;
using System.Security.Claims;
using System.Security.Cryptography;
using System.Text;
using System.Text.RegularExpressions;
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
        await AddAccountRefreshToken(account, refreshToken);

        return AuthenticateResponse.From(account, token, refreshToken.Token);
    }

    public async Task<Result<AccountResponse>> Create(AccountRequest request)
    {
        string emailRegex = @"^[a-zA-Z0-9._%+-]+@[a-zA-Z0-9.-]+\.[a-zA-Z]{2,}$";

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
            Role = Role.Admin,
        };

        _dataContext.Accounts.Add(account);
        await _dataContext.SaveChangesAsync();

        return AccountResponse.From(account);
    }

    public async Task<Result<AuthenticateResponse>> RefreshTokenAsync(string refreshToken, string? ipAddress)
    {
        var account = await _dataContext.RefreshTokens
            .Include(x => x.Account)
            .Where(x => x.Token == refreshToken)
            .Select(x => x.Account)
            .FirstOrDefaultAsync();

        if (account == null)
        {
            return new Result<AuthenticateResponse>(new Exception("Cuenta no encontrada"));
        }

        var newAccessToken = CreateToken(account);
        var newRefreshToken = GenerateRefreshToken(ipAddress);

        await AddAccountRefreshToken(account, newRefreshToken);
        return AuthenticateResponse.From(account, newAccessToken, newRefreshToken.Token);

    }

    private async Task AddAccountRefreshToken(Account account, RefreshToken refreshToken)
    {
        account.RefreshTokens.Add(refreshToken);
        _dataContext.Accounts.Update(account);
        await _dataContext.SaveChangesAsync();

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
        var jwtConfig = configuration.GetSection("AppSettings");
        byte[] dataDecoded = Convert.FromBase64String(jwtConfig["Secret"]!);
        var keyString = Encoding.UTF8.GetString(dataDecoded);

        var key = Encoding.UTF8.GetBytes(keyString);
        var secret = new SymmetricSecurityKey(key);
        return new SigningCredentials(secret, SecurityAlgorithms.HmacSha256);
    }

    private static List<Claim> GetClaims(Account account)
    {
        var claims = new List<Claim>
            {
                new(ClaimTypes.Name, account.FirstName),
                new("id", account.Id.ToString()),
                new(ClaimTypes.Role, account.Role.ToString()),
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
            IssuerSigningKey = new SymmetricSecurityKey(Encoding.UTF8.GetBytes(Encoding.UTF8.GetString(Convert.FromBase64String(configuration["AppSettings:Secret"])))),
            ValidateLifetime = false
        };

        var tokenHandler = new JwtSecurityTokenHandler();
        var principal = tokenHandler.ValidateToken(token, tokenValidationParameters, out SecurityToken securityToken);
        if (securityToken is not JwtSecurityToken jwtSecurityToken || !jwtSecurityToken.Header.Alg.Equals(SecurityAlgorithms.HmacSha256, StringComparison.InvariantCultureIgnoreCase))
            throw new SecurityTokenException("Invalid token");

        return principal;

    }

    public async Task<IEnumerable<AccountResponse>> GetAll(int? accountId, Role? role)
    {
        // If user is not admin, only return their own account
        if (role != Role.Admin && accountId.HasValue)
        {
            return await _dataContext.Accounts
                .Where(x => x.Id == accountId.Value)
                .Select(x => AccountResponse.From(x))
                .ToListAsync();
        }

        // Admin can see all accounts
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
        var account = await _dataContext.Accounts
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
        _dataContext.Accounts.Update(account);
        await _dataContext.SaveChangesAsync();

        return AccountResponse.From(account);
    }

    public async Task<Result<AccountResponse>> Update(int accountId, AccountPatchRequest request)
    {
        var account = await _dataContext.Accounts
          .Where(x => x.Id == accountId)
          .FirstOrDefaultAsync();

        if (account == null)
        {
            return new Result<AccountResponse>(new Exception("Cuenta no encontrada"));
        }

        account.FirstName = request.FirstName ?? account.FirstName;
        account.LastName = request.LastName ?? account.LastName;
        account.Email = request.Email ?? account.Email;
        account.Role = request.Role ?? account.Role;
        account.PasswordHash = request.Password != null ? BC.HashPassword(request.Password) : account.PasswordHash;
        _dataContext.Accounts.Update(account);
        await _dataContext.SaveChangesAsync();

        return AccountResponse.From(account);
    }

    public async Task<bool> Delete(int accountId)
    {
        var account = await _dataContext.Accounts
          .Where(x => x.Id == accountId)
          .FirstOrDefaultAsync();

        if (account == null)
        {
            return false;
        }

        account.Enabled = false;
        _dataContext.Accounts.Update(account);
        _dataContext.SaveChanges();
        return true;
    }
}
