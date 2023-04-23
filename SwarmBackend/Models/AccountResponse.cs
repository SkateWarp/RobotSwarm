namespace SwarmBackend.Models;

public record AuthenticateResponse(int Id, string FirstName, string LastName, string Email, string Role, string JwtToken, string RefreshToken)
{
    public static AuthenticateResponse From(Entities.Account account, string token, string refreshToken) => 
        new(account.Id, account.FirstName, account.LastName, account.Email, account.Role.ToString(), token, refreshToken);

}


public record AccountResponse(int Id, string FirstName, string LastName, string Email, string Role)
{
    public static AccountResponse From(Entities.Account account) => new(account.Id, account.FirstName, account.LastName, account.Email, account.Role.ToString());
}


public record AccountRequest(string FirstName, string LastName, string Email, string Password);
public record AuthenticateRequest(string Email, string Password);