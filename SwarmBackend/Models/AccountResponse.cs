namespace SwarmBackend.Models;

public record AuthenticateResponse(int Id, string FirstName, string LastName, string Email, string JwtToken, string RefreshToken)
{
    public static AuthenticateResponse From(Entities.Account account, string token, string refreshToken) => new(account.Id, account.FirstName, account.LastName, account.Email, token, refreshToken);

}


public record AccountResponse(int Id, string FirstName, string LastName, string Email)
{
    public static AccountResponse From(Entities.Account account) => new(account.Id, account.FirstName, account.LastName, account.Email);
}


public record AccountRequest(string FirstName, string LastName, string Email, string Password);
public record AuthenticateRequest(string Email, string Password);