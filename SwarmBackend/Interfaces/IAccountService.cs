using LanguageExt.Common;
using SwarmBackend.Entities;
using SwarmBackend.Models;
using System.Security.Claims;

namespace SwarmBackend.Interfaces;

public interface IAccountService
{

    Task<Result<AuthenticateResponse>> Authenticate(string email, string password, string? ipAddress);
    Task<Result<AccountResponse>> Create(AccountRequest request);
    Task<Result<AccountResponse>> Create(AccountRequest request, Role role);
    Task<Result<AccountResponse>> CreateAuthorized(
        int actorAccountId,
        ClaimsPrincipal principal,
        AccountRequest request,
        Role role,
        CancellationToken cancellationToken = default);

    Task<Result<AuthenticateResponse>> RefreshTokenAsync(string refreshToken, string? ipAddress);

    Task<IEnumerable<AccountResponse>> GetAll(int? accountId, Role? role);

    Task<Result<AccountResponse>> GetById(int accountId);

    Task<Result<AccountResponse>> Update(int accountId, AccountRequest request);
    Task<Result<AccountResponse>> UpdateAuthorized(
        int actorAccountId,
        ClaimsPrincipal principal,
        int accountId,
        AccountRequest request,
        CancellationToken cancellationToken = default);

    Task<Result<AccountResponse>> Update(int accountId, AccountPatchRequest request);
    Task<Result<AccountResponse>> UpdateAuthorized(
        int actorAccountId,
        ClaimsPrincipal principal,
        int accountId,
        AccountPatchRequest request,
        CancellationToken cancellationToken = default);

    Task<Result<AccountResponse>> ReactivateAuthorized(
        int actorAccountId,
        ClaimsPrincipal principal,
        int accountId,
        CancellationToken cancellationToken = default);

    Task<bool> Delete(int accountId);
    Task<Result<bool>> DeleteAuthorized(
        int actorAccountId,
        ClaimsPrincipal principal,
        int accountId,
        CancellationToken cancellationToken = default);
}
