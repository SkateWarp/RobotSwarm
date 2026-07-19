using System.Collections.Concurrent;
using Microsoft.AspNetCore.SignalR;
using SwarmBackend.Helpers;
using SwarmBackend.Models;

namespace SwarmBackend.Services;

public enum ViewerControlCheck
{
    Authorized,
    Missing,
    Mismatched,
    Expired,
    RateLimited
}

public enum ViewerControlAuthorizationStatus
{
    Authorized,
    Occupied,
    ReleaseFailed
}

public enum ViewerControlDrainStatus
{
    Removed,
    MissingOrReplaced,
    ReleaseFailed
}

public sealed record ViewerControlAuthorizationResult(
    ViewerControlAuthorizationStatus Status,
    DateTimeOffset AuthorizedUntil,
    long Version);

public sealed record ViewerControlGrantSnapshot(
    string ConnectionId,
    long Version,
    int AccountId,
    Guid SessionId,
    Guid LeaseId,
    Guid WorkerId,
    DateTimeOffset AuthorizedUntil,
    bool ReleaseRequested);

public sealed class ViewerControlRegistry(
    IHubContext<WorkerHub> workerContext,
    ILogger<ViewerControlRegistry> logger)
{
    public const int MaximumEventsPerSecond = 120;
    public static readonly TimeSpan MaximumAuthorizationLifetime = TimeSpan.FromSeconds(30);
    public static readonly TimeSpan ReleaseGracePeriod = TimeSpan.FromSeconds(5);
    internal static readonly TimeSpan WorkerSendTimeout = TimeSpan.FromSeconds(2);

    private static readonly TimeSpan DisconnectedConnectionRetention =
        MaximumAuthorizationLifetime + ReleaseGracePeriod + TimeSpan.FromSeconds(5);
    private static readonly TimeSpan RevocationMarkerRetention =
        MaximumAuthorizationLifetime + ReleaseGracePeriod + TimeSpan.FromMinutes(1);

    private readonly record struct LeaseKey(Guid SessionId, Guid LeaseId);
    private sealed class RevocationMarker(long version, DateTimeOffset createdAt)
    {
        public long Version { get; } = version;
        public DateTimeOffset CreatedAt { get; set; } = createdAt;
        public int ActiveReferences { get; set; } = 1;
        public bool Committed { get; set; }
    }

    private sealed class LeaseState(DateTimeOffset createdAt)
    {
        public SemaphoreSlim Gate { get; } = new(1, 1);
        public ConnectionAuthorization? Controller { get; set; }
        public DateTimeOffset RateWindowStartedAt { get; set; } = createdAt;
        public int RateWindowCount { get; set; }
        public bool FailSafeReleaseUsed { get; set; }
        public DateTimeOffset IdleSince { get; set; } = createdAt;
        public bool Retired { get; set; }
    }

    private sealed record ConnectionAuthorization(
        string ConnectionId,
        long Version,
        int AccountId,
        LeaseKey Lease,
        Guid WorkerId,
        DateTimeOffset AuthorizedUntil,
        bool ReleaseForwarded,
        LeaseState State);

    private readonly ConcurrentDictionary<string, ConnectionAuthorization> _connections =
        new(StringComparer.Ordinal);
    private readonly ConcurrentDictionary<LeaseKey, LeaseState> _leases = new();
    private readonly ConcurrentDictionary<string, DateTimeOffset> _disconnectedConnections =
        new(StringComparer.Ordinal);
    private readonly ConcurrentDictionary<string, DateTimeOffset> _pendingReleases =
        new(StringComparer.Ordinal);
    private readonly object _revocationLock = new();
    private readonly Dictionary<LeaseKey, RevocationMarker> _revokingLeases = new();
    private readonly Dictionary<Guid, RevocationMarker> _revokingSessions = new();
    private long _nextVersion;

    public async Task<ViewerControlAuthorizationResult> AuthorizeAsync(
        string connectionId,
        int accountId,
        Guid sessionId,
        Guid leaseId,
        Guid workerId,
        DateTimeOffset leaseExpiresAt,
        DateTimeOffset now,
        CancellationToken cancellationToken)
    {
        ArgumentException.ThrowIfNullOrWhiteSpace(connectionId);
        if (accountId <= 0
            || sessionId == Guid.Empty
            || leaseId == Guid.Empty
            || workerId == Guid.Empty)
        {
            throw new ArgumentException("Viewer control identifiers cannot be empty.");
        }

        if (leaseExpiresAt <= now)
        {
            throw new ArgumentOutOfRangeException(
                nameof(leaseExpiresAt),
                "An expired viewer lease cannot authorize input.");
        }

        var lease = new LeaseKey(sessionId, leaseId);
        if (_disconnectedConnections.ContainsKey(connectionId)
            || _pendingReleases.ContainsKey(connectionId)
            || IsRevoking(lease))
        {
            return new ViewerControlAuthorizationResult(
                ViewerControlAuthorizationStatus.ReleaseFailed,
                default,
                0);
        }

        if (_connections.TryGetValue(connectionId, out var previous)
            && previous.Lease != lease)
        {
            var releaseStatus = await DrainVersionAsync(
                ToSnapshot(previous),
                cancellationToken);
            if (releaseStatus == ViewerControlDrainStatus.ReleaseFailed)
            {
                return new ViewerControlAuthorizationResult(
                    ViewerControlAuthorizationStatus.ReleaseFailed,
                    default,
                    0);
            }
        }

        var authorizedUntil = DateTimeOffset.Compare(
                leaseExpiresAt,
                now + MaximumAuthorizationLifetime) < 0
            ? leaseExpiresAt
            : now + MaximumAuthorizationLifetime;

        while (true)
        {
            var state = _leases.GetOrAdd(lease, _ => new LeaseState(now));
            await state.Gate.WaitAsync(cancellationToken);
            try
            {
                if (_disconnectedConnections.ContainsKey(connectionId)
                    || _pendingReleases.ContainsKey(connectionId)
                    || IsRevoking(lease))
                {
                    return new ViewerControlAuthorizationResult(
                        ViewerControlAuthorizationStatus.ReleaseFailed,
                        default,
                        0);
                }

                if (state.Retired)
                {
                    continue;
                }

                var current = state.Controller;
                if (current is not null)
                {
                    if (!_connections.TryGetValue(
                            current.ConnectionId,
                            out var registered)
                        || !ReferenceEquals(registered.State, state))
                    {
                        state.Controller = null;
                        state.IdleSince = now;
                        current = null;
                    }
                    else if (!ReferenceEquals(current, registered))
                    {
                        state.Controller = registered;
                        current = registered;
                    }
                }

                if (current is not null && current.ConnectionId != connectionId)
                {
                    if (current.AuthorizedUntil > now)
                    {
                        return new ViewerControlAuthorizationResult(
                            ViewerControlAuthorizationStatus.Occupied,
                            current.AuthorizedUntil,
                            current.Version);
                    }

                    if (!await TrySendReleaseAsync(current, cancellationToken))
                    {
                        return new ViewerControlAuthorizationResult(
                            ViewerControlAuthorizationStatus.ReleaseFailed,
                            default,
                            0);
                    }

                    RemoveUnderGate(current, now);
                    current = null;
                }

                if (current is not null
                    && (current.WorkerId != workerId || current.AuthorizedUntil <= now))
                {
                    if (!await TrySendReleaseAsync(current, cancellationToken))
                    {
                        return new ViewerControlAuthorizationResult(
                            ViewerControlAuthorizationStatus.ReleaseFailed,
                            default,
                            0);
                    }

                    RemoveUnderGate(current, now);
                    current = null;
                }

                var authorization = new ConnectionAuthorization(
                    connectionId,
                    NextVersion(),
                    accountId,
                    lease,
                    workerId,
                    authorizedUntil,
                    current?.ReleaseForwarded ?? false,
                    state);
                state.Controller = authorization;
                state.IdleSince = now;
                _connections[connectionId] = authorization;
                return new ViewerControlAuthorizationResult(
                    ViewerControlAuthorizationStatus.Authorized,
                    authorizedUntil,
                    authorization.Version);
            }
            finally
            {
                state.Gate.Release();
            }
        }
    }

    public async Task<ViewerControlCheck> DispatchAsync(
        string connectionId,
        Guid sessionId,
        Guid leaseId,
        ViewerInputEvent input,
        DateTimeOffset now,
        CancellationToken cancellationToken)
    {
        if (!_connections.TryGetValue(connectionId, out var authorization))
        {
            return ViewerControlCheck.Missing;
        }

        var state = authorization.State;
        await state.Gate.WaitAsync(cancellationToken);
        try
        {
            if (!_connections.TryGetValue(connectionId, out var current)
                || !ReferenceEquals(state.Controller, current))
            {
                return ViewerControlCheck.Missing;
            }

            if (_disconnectedConnections.ContainsKey(connectionId))
            {
                return ViewerControlCheck.Missing;
            }

            if (current.Lease.SessionId != sessionId || current.Lease.LeaseId != leaseId)
            {
                return ViewerControlCheck.Mismatched;
            }

            var isRelease = input.Type == "releaseAll";
            if ((IsRevoking(current.Lease)
                    || _pendingReleases.ContainsKey(connectionId))
                && !isRelease)
            {
                return ViewerControlCheck.Expired;
            }

            if (!isRelease && current.AuthorizedUntil <= now)
            {
                return ViewerControlCheck.Expired;
            }

            if (isRelease)
            {
                if (now > current.AuthorizedUntil + ReleaseGracePeriod)
                {
                    return ViewerControlCheck.Expired;
                }

                if (current.ReleaseForwarded)
                {
                    return ViewerControlCheck.Authorized;
                }

                ResetRateWindowIfNeeded(state, now);
                if (state.RateWindowCount >= MaximumEventsPerSecond)
                {
                    if (state.FailSafeReleaseUsed)
                    {
                        return ViewerControlCheck.RateLimited;
                    }

                }

                state.RateWindowCount++;
            }
            else
            {
                ResetRateWindowIfNeeded(state, now);
                if (state.RateWindowCount >= MaximumEventsPerSecond)
                {
                    return ViewerControlCheck.RateLimited;
                }

                state.RateWindowCount++;
            }

            try
            {
                await SendInputWithTimeoutAsync(
                    current,
                    new ViewerInputEnvelope(sessionId, leaseId, input),
                    cancellationToken);
            }
            catch
            {
                if (isRelease)
                {
                    state.RateWindowCount--;
                    _pendingReleases.TryAdd(connectionId, DateTimeOffset.UtcNow);
                }
                else
                {
                    // The connection must re-establish control after an ambiguous
                    // non-release send. Reconciliation will issue the matching
                    // release before this connection can control another input.
                    _disconnectedConnections.TryAdd(connectionId, DateTimeOffset.UtcNow);
                }
                throw;
            }

            if (isRelease && state.RateWindowCount > MaximumEventsPerSecond)
            {
                // Consume the one overflow release only after SignalR accepted it.
                state.FailSafeReleaseUsed = true;
            }

            if (isRelease)
            {
                _pendingReleases.TryRemove(connectionId, out _);
            }

            var updated = current with { ReleaseForwarded = isRelease };
            state.Controller = updated;
            _connections[connectionId] = updated;
            return ViewerControlCheck.Authorized;
        }
        finally
        {
            state.Gate.Release();
        }
    }

    public Task<ViewerControlDrainStatus> ReleaseConnectionAsync(
        string connectionId,
        CancellationToken cancellationToken)
    {
        if (!_connections.TryGetValue(connectionId, out var authorization))
        {
            return Task.FromResult(ViewerControlDrainStatus.MissingOrReplaced);
        }

        return DrainVersionAsync(ToSnapshot(authorization), cancellationToken);
    }

    public async Task<ViewerControlDrainStatus> ReleaseDisconnectedConnectionAsync(
        string connectionId,
        DateTimeOffset now,
        CancellationToken cancellationToken)
    {
        ArgumentException.ThrowIfNullOrWhiteSpace(connectionId);
        _disconnectedConnections.TryAdd(connectionId, now);

        while (_connections.TryGetValue(connectionId, out var authorization))
        {
            var status = await DrainVersionAsync(ToSnapshot(authorization), cancellationToken);
            if (status != ViewerControlDrainStatus.MissingOrReplaced)
            {
                return status;
            }
        }

        return ViewerControlDrainStatus.MissingOrReplaced;
    }

    public async Task<long> BeginLeaseRevocationAsync(
        Guid sessionId,
        Guid leaseId,
        DateTimeOffset now,
        CancellationToken cancellationToken)
    {
        var lease = new LeaseKey(sessionId, leaseId);
        var marker = AddRevocationMarker(_revokingLeases, lease, now);
        try
        {
            await WaitForLeaseBarrierAsync(lease, cancellationToken);
            return marker.Version;
        }
        catch
        {
            CancelRevocationMarker(_revokingLeases, lease, marker.Version);
            throw;
        }
    }

    public void CancelLeaseRevocation(Guid sessionId, Guid leaseId, long version)
    {
        var lease = new LeaseKey(sessionId, leaseId);
        CancelRevocationMarker(_revokingLeases, lease, version);
    }

    public void ConfirmLeaseRevocation(Guid sessionId, Guid leaseId, long version)
    {
        ConfirmRevocationMarker(_revokingLeases, new LeaseKey(sessionId, leaseId), version);
    }

    public async Task<long> BeginSessionRevocationAsync(
        Guid sessionId,
        DateTimeOffset now,
        CancellationToken cancellationToken)
    {
        var marker = AddRevocationMarker(_revokingSessions, sessionId, now);
        try
        {
            var leases = _leases.Keys
                .Where(lease => lease.SessionId == sessionId)
                .OrderBy(lease => lease.LeaseId)
                .ToArray();
            foreach (var lease in leases)
            {
                await WaitForLeaseBarrierAsync(lease, cancellationToken);
            }

            return marker.Version;
        }
        catch
        {
            CancelRevocationMarker(_revokingSessions, sessionId, marker.Version);
            throw;
        }
    }

    public void CancelSessionRevocation(Guid sessionId, long version)
    {
        CancelRevocationMarker(_revokingSessions, sessionId, version);
    }

    public void ConfirmSessionRevocation(Guid sessionId, long version)
    {
        ConfirmRevocationMarker(_revokingSessions, sessionId, version);
    }

    public async Task<ViewerControlDrainStatus> DrainVersionAsync(
        ViewerControlGrantSnapshot snapshot,
        CancellationToken cancellationToken)
    {
        if (!_connections.TryGetValue(snapshot.ConnectionId, out var authorization)
            || authorization.Version != snapshot.Version
            || authorization.Lease.SessionId != snapshot.SessionId
            || authorization.Lease.LeaseId != snapshot.LeaseId)
        {
            return ViewerControlDrainStatus.MissingOrReplaced;
        }

        var state = authorization.State;
        await state.Gate.WaitAsync(cancellationToken);
        try
        {
            if (!_connections.TryGetValue(snapshot.ConnectionId, out var current)
                || current.Version != snapshot.Version
                || current.Lease.SessionId != snapshot.SessionId
                || current.Lease.LeaseId != snapshot.LeaseId
                || !ReferenceEquals(state.Controller, current))
            {
                return ViewerControlDrainStatus.MissingOrReplaced;
            }

            if (!await TrySendReleaseAsync(current, cancellationToken))
            {
                return ViewerControlDrainStatus.ReleaseFailed;
            }

            RemoveUnderGate(current, DateTimeOffset.UtcNow);
            return ViewerControlDrainStatus.Removed;
        }
        finally
        {
            state.Gate.Release();
        }
    }

    public Task<ViewerControlDrainStatus> DrainVersionAsync(
        string connectionId,
        long version,
        CancellationToken cancellationToken)
    {
        if (!_connections.TryGetValue(connectionId, out var authorization)
            || authorization.Version != version)
        {
            return Task.FromResult(ViewerControlDrainStatus.MissingOrReplaced);
        }

        return DrainVersionAsync(ToSnapshot(authorization), cancellationToken);
    }

    public async Task<ViewerControlDrainStatus> DrainLeaseAsync(
        Guid sessionId,
        Guid leaseId,
        CancellationToken cancellationToken)
    {
        if (!_leases.TryGetValue(new LeaseKey(sessionId, leaseId), out var state))
        {
            return ViewerControlDrainStatus.MissingOrReplaced;
        }

        await state.Gate.WaitAsync(cancellationToken);
        try
        {
            var current = state.Controller;
            if (current is null)
            {
                return ViewerControlDrainStatus.MissingOrReplaced;
            }

            if (!await TrySendReleaseAsync(current, cancellationToken))
            {
                return ViewerControlDrainStatus.ReleaseFailed;
            }

            RemoveUnderGate(current, DateTimeOffset.UtcNow);
            return ViewerControlDrainStatus.Removed;
        }
        finally
        {
            state.Gate.Release();
        }
    }

    public async Task<int> DrainSessionAsync(
        Guid sessionId,
        CancellationToken cancellationToken)
    {
        var snapshots = GetSnapshots()
            .Where(snapshot => snapshot.SessionId == sessionId)
            .ToArray();
        var removed = 0;
        foreach (var snapshot in snapshots)
        {
            if (await DrainVersionAsync(snapshot, cancellationToken)
                == ViewerControlDrainStatus.Removed)
            {
                removed++;
            }
        }

        return removed;
    }

    public IReadOnlyList<ViewerControlGrantSnapshot> GetSnapshots()
    {
        return _connections.Values
            .Select(ToSnapshot)
            .ToArray();
    }

    public bool IsCurrent(string connectionId, long version)
    {
        return _connections.TryGetValue(connectionId, out var authorization)
            && authorization.Version == version;
    }

    public void PruneIdleLeaseStates(DateTimeOffset now)
    {
        foreach (var pair in _leases)
        {
            var state = pair.Value;
            if (!state.Gate.Wait(0))
            {
                continue;
            }

            try
            {
                if (state.Controller is not null
                    || state.Retired
                    || now - state.IdleSince < ReleaseGracePeriod)
                {
                    continue;
                }

                state.Retired = true;
                ((ICollection<KeyValuePair<LeaseKey, LeaseState>>)_leases)
                    .Remove(pair);
            }
            finally
            {
                state.Gate.Release();
            }
        }

        foreach (var disconnected in _disconnectedConnections)
        {
            if (!_connections.ContainsKey(disconnected.Key)
                && now - disconnected.Value >= DisconnectedConnectionRetention)
            {
                ((ICollection<KeyValuePair<string, DateTimeOffset>>)_disconnectedConnections)
                    .Remove(disconnected);
            }
        }

        foreach (var pending in _pendingReleases)
        {
            if (!_connections.ContainsKey(pending.Key))
            {
                _pendingReleases.TryRemove(pending.Key, out _);
            }
        }

        PruneRevocationMarkers(_revokingLeases, now);
        PruneRevocationMarkers(_revokingSessions, now);
    }

    private async Task<bool> TrySendReleaseAsync(
        ConnectionAuthorization authorization,
        CancellationToken cancellationToken)
    {
        try
        {
            await SendWithTimeoutAsync(
                token => workerContext.Clients
                    .Group(ControlPlaneGroups.Worker(authorization.WorkerId))
                    .SendAsync(
                        "ViewerInputRelease",
                        new ViewerInputReleaseEnvelope(
                            authorization.Lease.SessionId,
                            authorization.Lease.LeaseId),
                        token),
                cancellationToken);
            return true;
        }
        catch (Exception exception) when (exception is not OutOfMemoryException)
        {
            logger.LogWarning(
                exception,
                "Unable to release viewer input for connection {ConnectionId}.",
                authorization.ConnectionId);
            return false;
        }
    }

    private void RemoveUnderGate(
        ConnectionAuthorization authorization,
        DateTimeOffset now)
    {
        var state = authorization.State;
        if (!ReferenceEquals(state.Controller, authorization))
        {
            return;
        }

        ((ICollection<KeyValuePair<string, ConnectionAuthorization>>)_connections)
            .Remove(new KeyValuePair<string, ConnectionAuthorization>(
                authorization.ConnectionId,
                authorization));
        state.Controller = null;
        state.IdleSince = now;
        _pendingReleases.TryRemove(authorization.ConnectionId, out _);
    }

    private bool IsRevoking(LeaseKey lease)
    {
        lock (_revocationLock)
        {
            return _revokingLeases.ContainsKey(lease)
                || _revokingSessions.ContainsKey(lease.SessionId);
        }
    }

    private async Task WaitForLeaseBarrierAsync(
        LeaseKey lease,
        CancellationToken cancellationToken)
    {
        if (!_leases.TryGetValue(lease, out var state))
        {
            return;
        }

        await state.Gate.WaitAsync(cancellationToken);
        state.Gate.Release();
    }

    private RevocationMarker AddRevocationMarker<TKey>(
        Dictionary<TKey, RevocationMarker> markers,
        TKey key,
        DateTimeOffset now) where TKey : notnull
    {
        lock (_revocationLock)
        {
            if (markers.TryGetValue(key, out var existing))
            {
                existing.ActiveReferences++;
                if (now > existing.CreatedAt)
                {
                    existing.CreatedAt = now;
                }
                return existing;
            }

            var marker = new RevocationMarker(NextVersion(), now);
            markers.Add(key, marker);
            return marker;
        }
    }

    private void CancelRevocationMarker<TKey>(
        Dictionary<TKey, RevocationMarker> markers,
        TKey key,
        long version) where TKey : notnull
    {
        lock (_revocationLock)
        {
            if (!markers.TryGetValue(key, out var marker) || marker.Version != version)
            {
                return;
            }

            if (marker.ActiveReferences > 0)
            {
                marker.ActiveReferences--;
            }
            if (marker.ActiveReferences == 0 && !marker.Committed)
            {
                markers.Remove(key);
            }
        }
    }

    private void ConfirmRevocationMarker<TKey>(
        Dictionary<TKey, RevocationMarker> markers,
        TKey key,
        long version) where TKey : notnull
    {
        lock (_revocationLock)
        {
            if (!markers.TryGetValue(key, out var marker) || marker.Version != version)
            {
                return;
            }

            marker.Committed = true;
            if (marker.ActiveReferences > 0)
            {
                marker.ActiveReferences--;
            }
        }
    }

    private void PruneRevocationMarkers<TKey>(
        Dictionary<TKey, RevocationMarker> markers,
        DateTimeOffset now) where TKey : notnull
    {
        lock (_revocationLock)
        {
            var expired = markers
                .Where(pair => pair.Value.ActiveReferences == 0
                    && now - pair.Value.CreatedAt >= RevocationMarkerRetention)
                .Select(pair => pair.Key)
                .ToArray();
            foreach (var key in expired)
            {
                markers.Remove(key);
            }
        }
    }

    private static void ResetRateWindowIfNeeded(LeaseState state, DateTimeOffset now)
    {
        if (now < state.RateWindowStartedAt
            || now - state.RateWindowStartedAt >= TimeSpan.FromSeconds(1))
        {
            state.RateWindowStartedAt = now;
            state.RateWindowCount = 0;
            state.FailSafeReleaseUsed = false;
        }
    }

    private long NextVersion() => Interlocked.Increment(ref _nextVersion);

    private async Task SendInputWithTimeoutAsync(
        ConnectionAuthorization authorization,
        ViewerInputEnvelope input,
        CancellationToken cancellationToken)
    {
        using var timeout = CancellationTokenSource.CreateLinkedTokenSource(cancellationToken);
        timeout.CancelAfter(WorkerSendTimeout);
        var sendTask = workerContext.Clients
            .Group(ControlPlaneGroups.Worker(authorization.WorkerId))
            .SendAsync("ViewerInput", input, timeout.Token);
        try
        {
            await sendTask.WaitAsync(WorkerSendTimeout, cancellationToken);
        }
        catch
        {
            timeout.Cancel();
            _ = ReleaseAfterAmbiguousInputAsync(sendTask, authorization);
            throw;
        }
    }

    private async Task ReleaseAfterAmbiguousInputAsync(
        Task sendTask,
        ConnectionAuthorization authorization)
    {
        try
        {
            await sendTask;
        }
        catch (Exception exception) when (exception is not OutOfMemoryException)
        {
            logger.LogDebug(
                exception,
                "Ambiguous viewer input settled with an error for connection {ConnectionId}.",
                authorization.ConnectionId);
        }

        if (!await TrySendReleaseAsync(authorization, CancellationToken.None))
        {
            logger.LogWarning(
                "The follow-up viewer input release failed for connection {ConnectionId}.",
                authorization.ConnectionId);
        }
    }

    private static async Task SendWithTimeoutAsync(
        Func<CancellationToken, Task> send,
        CancellationToken cancellationToken)
    {
        using var timeout = CancellationTokenSource.CreateLinkedTokenSource(cancellationToken);
        timeout.CancelAfter(WorkerSendTimeout);
        try
        {
            await send(timeout.Token).WaitAsync(WorkerSendTimeout, cancellationToken);
        }
        catch
        {
            timeout.Cancel();
            throw;
        }
    }

    private ViewerControlGrantSnapshot ToSnapshot(
        ConnectionAuthorization authorization) =>
        new(
            authorization.ConnectionId,
            authorization.Version,
            authorization.AccountId,
            authorization.Lease.SessionId,
            authorization.Lease.LeaseId,
            authorization.WorkerId,
            authorization.AuthorizedUntil,
            _disconnectedConnections.ContainsKey(authorization.ConnectionId)
                || _pendingReleases.ContainsKey(authorization.ConnectionId)
                || IsRevoking(authorization.Lease));
}
