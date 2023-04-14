namespace SwarmBackend.Models;

public record AccountResponse(int Id)
{
    public static AccountResponse From(Entities.Account account) => new(account.Id);
}
