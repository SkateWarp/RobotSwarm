using Microsoft.Extensions.Logging.Abstractions;
using SwarmBackend.Helpers;
using SwarmBackend.Services;

namespace SwarmBackend.Tests;

public sealed class WorkerConnectionRegistryTests
{
    [Fact]
    public void ReconnectFromTheSameAgentReplacesAndAbortsTheOldConnection()
    {
        var registry = Registry();
        var workerId = Guid.NewGuid();
        var agentInstanceId = Guid.NewGuid();
        var oldAborted = false;

        Assert.Equal(
            WorkerConnectionClaim.Accepted,
            registry.Claim(
                workerId,
                agentInstanceId,
                "old-connection",
                () => oldAborted = true));

        Assert.Equal(
            WorkerConnectionClaim.Replaced,
            registry.Claim(
                workerId,
                agentInstanceId,
                "new-connection",
                () => { }));

        Assert.True(oldAborted);
        Assert.False(registry.IsCurrent(workerId, "old-connection"));
        Assert.True(registry.IsCurrent(workerId, "new-connection"));
    }

    [Fact]
    public void DifferentAgentCannotDisplaceTheCurrentWorkerProcess()
    {
        var registry = Registry();
        var workerId = Guid.NewGuid();

        Assert.Equal(
            WorkerConnectionClaim.Accepted,
            registry.Claim(
                workerId,
                Guid.NewGuid(),
                "current-connection",
                () => { }));

        Assert.Equal(
            WorkerConnectionClaim.Rejected,
            registry.Claim(
                workerId,
                Guid.NewGuid(),
                "duplicate-connection",
                () => { }));

        Assert.True(registry.IsCurrent(workerId, "current-connection"));
        Assert.False(registry.IsCurrent(workerId, "duplicate-connection"));
    }

    [Fact]
    public void LegacyClientKeepsTheOldFailClosedDuplicatePolicy()
    {
        var registry = Registry();
        var workerId = Guid.NewGuid();

        Assert.Equal(
            WorkerConnectionClaim.Accepted,
            registry.Claim(
                workerId,
                agentInstanceId: null,
                "legacy-current",
                () => { }));
        Assert.Equal(
            WorkerConnectionClaim.Rejected,
            registry.Claim(
                workerId,
                agentInstanceId: null,
                "legacy-duplicate",
                () => { }));
        Assert.Equal(
            WorkerConnectionClaim.Rejected,
            registry.Claim(
                workerId,
                Guid.NewGuid(),
                "versioned-duplicate",
                () => { }));

        Assert.True(registry.Release(workerId, "legacy-current"));
        Assert.Equal(
            WorkerConnectionClaim.Accepted,
            registry.Claim(
                workerId,
                Guid.NewGuid(),
                "versioned-after-release",
                () => { }));
    }

    [Fact]
    public void LateDisconnectFromReplacedConnectionCannotRemoveTheNewOwner()
    {
        var registry = Registry();
        var workerId = Guid.NewGuid();
        var agentInstanceId = Guid.NewGuid();

        registry.Claim(
            workerId,
            agentInstanceId,
            "old-connection",
            () => { });
        registry.Claim(
            workerId,
            agentInstanceId,
            "new-connection",
            () => { });

        Assert.False(registry.Release(workerId, "old-connection"));
        Assert.True(registry.IsCurrent(workerId, "new-connection"));
    }

    [Fact]
    public void ConcurrentReconnectsLeaveExactlyOneCurrentConnection()
    {
        var registry = Registry();
        var workerId = Guid.NewGuid();
        var agentInstanceId = Guid.NewGuid();
        var connectionIds = Enumerable.Range(0, 32)
            .Select(index => $"connection-{index}")
            .ToArray();
        var abortCount = 0;
        var claims = new WorkerConnectionClaim[connectionIds.Length];

        Parallel.For(
            0,
            connectionIds.Length,
            index =>
            {
                claims[index] = registry.Claim(
                    workerId,
                    agentInstanceId,
                    connectionIds[index],
                    () => Interlocked.Increment(ref abortCount));
            });

        Assert.DoesNotContain(WorkerConnectionClaim.Rejected, claims);
        Assert.Equal(connectionIds.Length - 1, abortCount);
        Assert.Single(
            connectionIds,
            connectionId => registry.IsCurrent(workerId, connectionId));
    }

    [Fact]
    public void CredentialInvalidationRemovesAndAbortsTheCurrentConnection()
    {
        var registry = Registry();
        var workerId = Guid.NewGuid();
        var aborted = false;
        registry.Claim(
            workerId,
            Guid.NewGuid(),
            "current-connection",
            () => aborted = true);

        Assert.True(registry.Invalidate(workerId));

        Assert.True(aborted);
        Assert.False(registry.IsCurrent(workerId, "current-connection"));
        Assert.False(registry.Invalidate(workerId));
    }

    [Theory]
    [InlineData(null, true)]
    [InlineData("", true)]
    [InlineData("not-a-guid", false)]
    [InlineData("00000000-0000-0000-0000-000000000000", false)]
    public void AgentInstanceParserKeepsLegacyButRejectsMalformedValues(
        string? value,
        bool accepted)
    {
        Assert.Equal(
            accepted,
            WorkerCredentialAuthenticationHandler.TryParseAgentInstanceId(
                value,
                out var parsed));
        if (string.IsNullOrWhiteSpace(value))
        {
            Assert.Null(parsed);
        }
    }

    [Fact]
    public void AgentInstanceParserKeepsTheExactIdentifier()
    {
        var value = Guid.NewGuid();

        Assert.True(
            WorkerCredentialAuthenticationHandler.TryParseAgentInstanceId(
                value.ToString("D"),
                out var parsed));

        Assert.Equal(value, parsed);
    }

    private static WorkerConnectionRegistry Registry() =>
        new(NullLogger<WorkerConnectionRegistry>.Instance);
}
