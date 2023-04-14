using SwarmBackend.Models;

namespace SwarmBackend.Interfaces;

public interface IAccountService
{

    Task<AccountResponse> Authenticate(string email, string password);
}
