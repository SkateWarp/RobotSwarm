using LanguageExt.Common;
using SwarmBackend.Models;

namespace SwarmBackend.Interfaces;

public interface IAccountService
{

    Task<Result<AuthenticateResponse>> Authenticate(string email, string password, string? ipAddress);
    Task<Result<AccountResponse>> Create(AccountRequest request);

    Task<Result<AuthenticateResponse>> RefreshTokenAsync(string refreshToken, string? ipAddress);

    Task<IEnumerable<AccountResponse>> GetAll();

    Task<Result<AccountResponse>> Update(int accountId, AccountRequest request);

}
