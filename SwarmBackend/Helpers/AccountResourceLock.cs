using System.Security.Claims;
using Microsoft.EntityFrameworkCore;

namespace SwarmBackend.Helpers;

/// <summary>
/// Orders account disablement before session and viewer mutations.
/// PostgreSQL advisory locks are transaction-scoped, so callers must begin
/// their transaction before acquiring one.
/// </summary>
public static class AccountResourceLock
{
    private const long LockNamespace = 0x5253000000000000;
    private const long AdministratorSetLock = 0x525341444D494E00;
    private const long EmailSetLock = 0x5253454D41494C00;

    public static Task AcquireAdministratorMutation(
        DataContext dataContext,
        CancellationToken cancellationToken = default)
    {
        return AcquireKey(
            dataContext,
            AdministratorSetLock,
            shared: false,
            cancellationToken: cancellationToken);
    }

    public static Task AcquireEmailMutation(
        DataContext dataContext,
        CancellationToken cancellationToken = default)
    {
        return AcquireKey(
            dataContext,
            EmailSetLock,
            shared: false,
            cancellationToken: cancellationToken);
    }

    public static Task AcquireShared(
        DataContext dataContext,
        int accountId,
        CancellationToken cancellationToken = default)
    {
        return Acquire(
            dataContext,
            accountId,
            shared: true,
            cancellationToken: cancellationToken);
    }

    public static async Task<bool> AcquireSharedAndValidate(
        DataContext dataContext,
        int accountId,
        ClaimsPrincipal principal,
        CancellationToken cancellationToken = default)
    {
        await AcquireShared(dataContext, accountId, cancellationToken);

        return await ValidatePrincipal(
            dataContext,
            accountId,
            principal,
            cancellationToken);
    }

    /// <summary>
    /// Locks the actor and target in account-id order. When both ids are the
    /// same, one exclusive lock is enough for validation and mutation.
    /// </summary>
    public static async Task<bool> AcquireActorAndTarget(
        DataContext dataContext,
        int actorAccountId,
        ClaimsPrincipal principal,
        int targetAccountId,
        CancellationToken cancellationToken = default)
    {
        if (actorAccountId == targetAccountId)
        {
            await AcquireExclusive(dataContext, targetAccountId, cancellationToken);
            return await ValidatePrincipal(
                dataContext,
                actorAccountId,
                principal,
                cancellationToken);
        }

        if (actorAccountId < targetAccountId)
        {
            var actorIsCurrent = await AcquireSharedAndValidate(
                dataContext,
                actorAccountId,
                principal,
                cancellationToken);
            if (!actorIsCurrent)
            {
                return false;
            }

            await AcquireExclusive(dataContext, targetAccountId, cancellationToken);
            return true;
        }

        await AcquireExclusive(dataContext, targetAccountId, cancellationToken);
        return await AcquireSharedAndValidate(
            dataContext,
            actorAccountId,
            principal,
            cancellationToken);
    }

    private static async Task<bool> ValidatePrincipal(
        DataContext dataContext,
        int accountId,
        ClaimsPrincipal principal,
        CancellationToken cancellationToken)
    {

        var roleValue = principal.FindFirst(ClaimTypes.Role)?.Value;
        var versionValue = principal.FindFirst("account_version")?.Value;
        if (string.IsNullOrWhiteSpace(roleValue)
            || !long.TryParse(versionValue, out var accountVersion))
        {
            return false;
        }

        var accounts = dataContext.Accounts.AsNoTracking();
        if (dataContext.Database.ProviderName
            == "Npgsql.EntityFrameworkCore.PostgreSQL")
        {
            // The advisory-lock statement can establish a SERIALIZABLE snapshot
            // while it waits for an account writer. A row lock makes PostgreSQL
            // reject that stale snapshot with 40001, so the route can retry and
            // validate the committed account version instead of accepting it.
            accounts = dataContext.Accounts
                .FromSqlInterpolated($"""
                    SELECT *
                    FROM "Accounts"
                    WHERE "Id" = {accountId}
                    FOR SHARE
                    """)
                .AsNoTracking();
        }

        var account = await accounts
            .Where(candidate => candidate.Id == accountId)
            .Select(candidate => new
            {
                candidate.Enabled,
                candidate.Role,
                candidate.Updated
            })
            .SingleOrDefaultAsync(cancellationToken);
        return account != null
            && account.Enabled
            && roleValue.Equals(account.Role.ToString(), StringComparison.Ordinal)
            && accountVersion == (account.Updated?.Ticks ?? 0L);
    }

    public static Task AcquireExclusive(
        DataContext dataContext,
        int accountId,
        CancellationToken cancellationToken = default)
    {
        return Acquire(
            dataContext,
            accountId,
            shared: false,
            cancellationToken: cancellationToken);
    }

    private static async Task Acquire(
        DataContext dataContext,
        int accountId,
        bool shared,
        CancellationToken cancellationToken)
    {
        var key = LockNamespace | (uint)accountId;
        await AcquireKey(dataContext, key, shared, cancellationToken);
    }

    private static async Task AcquireKey(
        DataContext dataContext,
        long key,
        bool shared,
        CancellationToken cancellationToken)
    {
        if (!dataContext.Database.IsRelational())
        {
            return;
        }

        if (shared)
        {
            await dataContext.Database.ExecuteSqlInterpolatedAsync(
                $"SELECT pg_advisory_xact_lock_shared({key})",
                cancellationToken);
            return;
        }

        await dataContext.Database.ExecuteSqlInterpolatedAsync(
            $"SELECT pg_advisory_xact_lock({key})",
            cancellationToken);
    }
}
