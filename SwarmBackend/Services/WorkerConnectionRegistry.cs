using System.Collections.Concurrent;

namespace SwarmBackend.Services;

internal enum WorkerConnectionClaim
{
    Accepted,
    Replaced,
    Rejected
}

public sealed class WorkerConnectionRegistry(
    ILogger<WorkerConnectionRegistry> logger)
{
    private readonly ConcurrentDictionary<Guid, ActiveConnection> _connections = new();

    internal WorkerConnectionClaim Claim(
        Guid workerId,
        Guid? agentInstanceId,
        string connectionId,
        Action abort)
    {
        var candidate = new ActiveConnection(
            agentInstanceId,
            connectionId,
            abort);

        while (true)
        {
            if (_connections.TryAdd(workerId, candidate))
            {
                return WorkerConnectionClaim.Accepted;
            }

            if (!_connections.TryGetValue(workerId, out var current))
            {
                continue;
            }

            // A legacy client has no process incarnation. It can own an empty
            // slot, but cannot safely prove that a duplicate is its reconnect.
            if (!agentInstanceId.HasValue
                || !current.AgentInstanceId.HasValue
                || current.AgentInstanceId != agentInstanceId)
            {
                return WorkerConnectionClaim.Rejected;
            }

            if (current.ConnectionId == connectionId)
            {
                return WorkerConnectionClaim.Accepted;
            }

            if (!_connections.TryUpdate(workerId, candidate, current))
            {
                continue;
            }

            AbortSafely(workerId, current);
            return WorkerConnectionClaim.Replaced;
        }
    }

    internal bool IsCurrent(Guid workerId, string connectionId)
    {
        return _connections.TryGetValue(workerId, out var current)
            && current.ConnectionId == connectionId;
    }

    internal bool Release(Guid workerId, string connectionId)
    {
        if (!_connections.TryGetValue(workerId, out var current)
            || current.ConnectionId != connectionId)
        {
            return false;
        }

        return _connections.TryRemove(
            new KeyValuePair<Guid, ActiveConnection>(workerId, current));
    }

    public bool Invalidate(Guid workerId)
    {
        if (!_connections.TryRemove(workerId, out var current))
        {
            return false;
        }

        AbortSafely(workerId, current);
        return true;
    }

    private void AbortSafely(Guid workerId, ActiveConnection connection)
    {
        try
        {
            connection.Abort();
        }
        catch (Exception exception)
        {
            logger.LogWarning(
                exception,
                "Unable to abort superseded connection {ConnectionId} for worker {WorkerId}.",
                connection.ConnectionId,
                workerId);
        }
    }

    private sealed record ActiveConnection(
        Guid? AgentInstanceId,
        string ConnectionId,
        Action Abort);
}
