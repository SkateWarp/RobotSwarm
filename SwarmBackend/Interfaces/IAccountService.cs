using LanguageExt.Common;
using SwarmBackend.Entities;
using SwarmBackend.Models;

namespace SwarmBackend.Interfaces;

public interface IAccountService
{

    Task<Result<AuthenticateResponse>> Authenticate(string email, string password, string? ipAddress);
    Task<Result<AccountResponse>> Create(AccountRequest request);

    Task<Result<AuthenticateResponse>> RefreshTokenAsync(string refreshToken, string? ipAddress);

    Task<IEnumerable<AccountResponse>> GetAll(int? accountId, Role? role);

    Task<Result<AccountResponse>> GetById(int accountId);

    Task<Result<AccountResponse>> Update(int accountId, AccountRequest request);

    Task<Result<AccountResponse>> Update(int accountId, AccountPatchRequest request);

    Task<bool> Delete(int accountId);
}
