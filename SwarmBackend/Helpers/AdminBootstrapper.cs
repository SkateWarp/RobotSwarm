using Microsoft.EntityFrameworkCore;
using SwarmBackend.Entities;

namespace SwarmBackend.Helpers;

public static class AdminBootstrapper
{
    public static async Task EnsureAdminExists(
        DataContext dataContext,
        IConfiguration configuration,
        ILogger logger,
        CancellationToken cancellationToken = default)
    {
        var email = configuration["BootstrapAdmin:Email"];
        var password = configuration["BootstrapAdmin:Password"];
        if (string.IsNullOrWhiteSpace(email) && string.IsNullOrWhiteSpace(password))
        {
            return;
        }

        if (string.IsNullOrWhiteSpace(email) || string.IsNullOrWhiteSpace(password))
        {
            throw new InvalidOperationException(
                "BootstrapAdmin:Email and BootstrapAdmin:Password must be configured together.");
        }

        if (!AccountRequestValidator.TryCanonicalizeEmail(
                email,
                out var normalizedEmail))
        {
            throw new InvalidOperationException("BootstrapAdmin:Email is not a valid email address.");
        }

        if (password.Length < 12)
        {
            throw new InvalidOperationException(
                "BootstrapAdmin:Password must contain at least 12 characters.");
        }

        if (await dataContext.Accounts.AnyAsync(
                account => account.Role == Role.Admin && account.Enabled,
                cancellationToken))
        {
            return;
        }

        Account? existingAccount;
        if (dataContext.Database.ProviderName
            == "Npgsql.EntityFrameworkCore.PostgreSQL")
        {
            existingAccount = await dataContext.Accounts.SingleOrDefaultAsync(
                account => account.NormalizedEmail == normalizedEmail,
                cancellationToken);
        }
        else
        {
            var accounts = await dataContext.Accounts.ToListAsync(cancellationToken);
            existingAccount = accounts.SingleOrDefault(account =>
                AccountRequestValidator.CanonicalEmail(account.Email)
                    == normalizedEmail);
        }
        if (existingAccount != null)
        {
            if (existingAccount.Role != Role.Admin)
            {
                throw new InvalidOperationException(
                    "The bootstrap admin email already belongs to a non-admin account.");
            }

            var reenabledAt = DateTime.UtcNow;
            existingAccount.Enabled = true;
            existingAccount.PasswordHash = BCrypt.Net.BCrypt.HashPassword(password);
            existingAccount.Updated = reenabledAt;
            await dataContext.SaveChangesAsync(cancellationToken);
            logger.LogWarning(
                "Re-enabled the bootstrap administrator account {AdminEmail}. Remove the bootstrap secrets now.",
                normalizedEmail);
            return;
        }

        var now = DateTime.UtcNow;
        dataContext.Accounts.Add(new Account
        {
            FirstName = "RobotSwarm",
            LastName = "Admin",
            Email = normalizedEmail,
            PasswordHash = BCrypt.Net.BCrypt.HashPassword(password),
            Role = Role.Admin,
            Enabled = true,
            Verified = now,
            Created = now,
            DateCreated = now
        });
        await dataContext.SaveChangesAsync(cancellationToken);
        logger.LogWarning(
            "Created the one-time bootstrap administrator account {AdminEmail}. Remove the bootstrap secrets now.",
            normalizedEmail);
    }
}
