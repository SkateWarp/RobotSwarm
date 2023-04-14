namespace SwarmBackend.Entities;

public class Account
{

    public int Id { get; set; }

    public DateTime DateCreated { get; set; } = DateTime.Now;

    public string FirstName { get; set; } = null!;
    public string LastName { get; set; } = null!;
    public string Email { get; set; } = null!;

    public string? PasswordHash { get; set; }

    public string? VerificationToken { get; set; }
    public DateTime? Verified { get; set; }
    public bool IsVerified => Verified.HasValue || PasswordReset.HasValue;
    public string? ResetToken { get; set; }
    public DateTime? ResetTokenExpires { get; set; }
    public DateTime? PasswordReset { get; set; }
    public DateTime Created { get; set; }
    public DateTime? Updated { get; set; }
    public bool Enabled { get; set; }
    public List<RefreshToken> RefreshTokens { get; set; } = new();

    public bool OwnsToken(string token)
    {
        return RefreshTokens.Find(x => x.Token == token) != null;
    }

}
