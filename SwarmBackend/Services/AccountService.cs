using Microsoft.EntityFrameworkCore;
using SwarmBackend.Helpers;
using SwarmBackend.Interfaces;
using SwarmBackend.Models;

namespace SwarmBackend.Services;

public class AccountService : IAccountService
{
    private readonly DataContext _dataContext;

    public AccountService(DataContext dataContext)
    {
        _dataContext = dataContext;
    }

    public async Task<AccountResponse> Authenticate(string email, string password)
    {
        var account = await _dataContext.Accounts
            .Where(x => x.Email == email)
            .FirstOrDefaultAsync();

        if (account == null)
        {
            throw new Exception("Usuario no encontrado");
        }


        return AccountResponse.From(account);
    }
}
