using Microsoft.EntityFrameworkCore;
using SwarmBackend.Entities;
using SwarmBackend.Helpers;

namespace SwarmBackend.Services;

public sealed class ViewerControlReconciler(
    IServiceScopeFactory scopeFactory,
    ViewerControlRegistry registry,
    ILogger<ViewerControlReconciler> logger) : BackgroundService
{
    internal static readonly TimeSpan ReconciliationInterval = TimeSpan.FromSeconds(1);
    internal const int MaximumConcurrentDrains = 4;
    private const int MaximumLeaseIdsPerQuery = 500;

    protected override async Task ExecuteAsync(CancellationToken stoppingToken)
    {
        using var timer = new PeriodicTimer(ReconciliationInterval);
        while (!stoppingToken.IsCancellationRequested)
        {
            try
            {
                await using var scope = scopeFactory.CreateAsyncScope();
                var dataContext = scope.ServiceProvider.GetRequiredService<DataContext>();
                await ReconcileOnceAsync(
                    dataContext,
                    registry,
                    DateTimeOffset.UtcNow,
                    stoppingToken);
            }
            catch (OperationCanceledException) when (stoppingToken.IsCancellationRequested)
            {
                break;
            }
            catch (Exception exception)
            {
                logger.LogError(exception, "Viewer control reconciliation failed.");
            }

            try
            {
                if (!await timer.WaitForNextTickAsync(stoppingToken))
                {
                    break;
                }
            }
            catch (OperationCanceledException) when (stoppingToken.IsCancellationRequested)
            {
                break;
            }
        }
    }

    internal static async Task ReconcileOnceAsync(
        DataContext dataContext,
        ViewerControlRegistry registry,
        DateTimeOffset now,
        CancellationToken cancellationToken)
    {
        var snapshots = registry.GetSnapshots();
        if (snapshots.Count == 0)
        {
            registry.PruneIdleLeaseStates(now);
            return;
        }

        var validationByLease = new Dictionary<Guid, LeaseValidation>();
        var leaseIds = snapshots
            .Select(snapshot => snapshot.LeaseId)
            .Distinct()
            .ToArray();
        foreach (var leaseIdBatch in leaseIds.Chunk(MaximumLeaseIdsPerQuery))
        {
            var validations = await dataContext.ViewerLeases
                .AsNoTracking()
                .Where(lease => leaseIdBatch.Contains(lease.Id))
                .Select(lease => new LeaseValidation(
                    lease.Id,
                    lease.SimulationSessionId,
                    lease.AccountId,
                    lease.SimulationSession.AccountId,
                    lease.Account.Enabled,
                    lease.ExpiresAt,
                    lease.RevokedAt,
                    lease.SimulationSession.State,
                    lease.SimulationSession.ComputeWorkerId))
                .ToListAsync(cancellationToken);
            foreach (var validation in validations)
            {
                validationByLease[validation.LeaseId] = validation;
            }
        }

        var invalidSnapshots = snapshots
            .Where(snapshot => !IsValid(snapshot, validationByLease, now));
        await Parallel.ForEachAsync(
            invalidSnapshots,
            new ParallelOptions
            {
                CancellationToken = cancellationToken,
                MaxDegreeOfParallelism = MaximumConcurrentDrains
            },
            async (snapshot, token) =>
            {
                await registry.DrainVersionAsync(snapshot, token);
            });

        registry.PruneIdleLeaseStates(now);
    }

    private static bool IsValid(
        ViewerControlGrantSnapshot snapshot,
        IReadOnlyDictionary<Guid, LeaseValidation> validationByLease,
        DateTimeOffset now)
    {
        if (snapshot.ReleaseRequested
            || snapshot.AuthorizedUntil <= now
            || !validationByLease.TryGetValue(snapshot.LeaseId, out var validation))
        {
            return false;
        }

        return validation.SessionId == snapshot.SessionId
            && validation.AccountId == snapshot.AccountId
            && validation.SessionAccountId == snapshot.AccountId
            && validation.AccountEnabled
            && validation.RevokedAt is null
            && validation.ExpiresAt > now.UtcDateTime
            && validation.SessionState is SimulationSessionState.Ready
                or SimulationSessionState.Active
                or SimulationSessionState.Paused
            && validation.WorkerId == snapshot.WorkerId;
    }

    private sealed record LeaseValidation(
        Guid LeaseId,
        Guid SessionId,
        int AccountId,
        int SessionAccountId,
        bool AccountEnabled,
        DateTime ExpiresAt,
        DateTime? RevokedAt,
        SimulationSessionState SessionState,
        Guid? WorkerId);
}
