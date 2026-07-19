using SwarmWorker.Runtime;
using SwarmWorker.Services;

namespace SwarmWorker.Tests;

public sealed class WorkerAgentTests
{
    [Fact]
    public void HeartbeatIncludesStoppedManagedContainersUntilTheyAreRemoved()
    {
        var runningSessionId = Guid.Parse("aaaaaaaa-aaaa-aaaa-aaaa-aaaaaaaaaaaa");
        var stoppedSessionId = Guid.Parse("bbbbbbbb-bbbb-bbbb-bbbb-bbbbbbbbbbbb");
        var sessions = new[]
        {
            ManagedSession(runningSessionId, running: true),
            ManagedSession(stoppedSessionId, running: false),
            ManagedSession(stoppedSessionId, running: false)
        };

        var reported = WorkerAgent.ManagedSessionIdsForHeartbeat(sessions);

        Assert.Equal(new[] { runningSessionId, stoppedSessionId }, reported);
    }

    private static ManagedSessionInfo ManagedSession(Guid sessionId, bool running)
    {
        return new ManagedSessionInfo(
            sessionId,
            $"container-{sessionId:N}",
            $"robotswarm-session-{sessionId:N}",
            "robotswarm:test",
            running,
            running ? "running" : "exited",
            new Dictionary<string, string>());
    }
}
