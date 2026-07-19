using SwarmWorker.Configuration;
using SwarmWorker.Runtime;

namespace SwarmWorker.Tests;

public sealed class DockerArgumentBuilderTests
{
    [Fact]
    public void CreateContainerUsesIsolatedSecureDefaults()
    {
        var workerId = Guid.NewGuid();
        var sessionId = Guid.NewGuid();
        var options = ValidOptions();
        var specification = new SessionContainerSpec(
            workerId,
            sessionId,
            "arena-v1",
            options.SessionImage,
            "test",
            options.MaxRobotsPerSession);

        var arguments = DockerArgumentBuilder.BuildCreateContainer(specification, options);

        Assert.Contains("--read-only", arguments);
        Assert.Contains("--cap-drop", arguments);
        Assert.Contains("ALL", arguments);
        Assert.Contains("no-new-privileges:true", arguments);
        Assert.Contains("--pids-limit", arguments);
        var pidsFlag = Array.IndexOf(arguments.ToArray(), "--pids-limit");
        Assert.Equal("1024", arguments[pidsFlag + 1]);
        Assert.Contains("--memory", arguments);
        Assert.Contains("--cpus", arguments);
        var cpuFlag = Array.IndexOf(arguments.ToArray(), "--cpus");
        Assert.Equal("12", arguments[cpuFlag + 1]);
        Assert.Contains("--gpus", arguments);
        Assert.DoesNotContain("--privileged", arguments);
        Assert.DoesNotContain("--network=host", arguments);
        Assert.DoesNotContain("--publish", arguments);
        Assert.DoesNotContain("-p", arguments);
        Assert.Contains(SessionResourceNames.Network(sessionId), arguments);
        Assert.Contains("start_legacy_bridge:=false", arguments);
        Assert.Contains("gui:=false", arguments);
        Assert.DoesNotContain(
            arguments,
            argument => argument.StartsWith("DISPLAY=", StringComparison.Ordinal));
        Assert.DoesNotContain(
            arguments,
            argument => argument.Contains("/tmp/.X11-unix", StringComparison.Ordinal));
    }

    [Fact]
    public void ViewerPublishingKeepsTheRosServerHeadless()
    {
        var options = ValidOptions();
        options.Viewer.Enabled = true;
        var specification = new SessionContainerSpec(
            options.WorkerId,
            Guid.NewGuid(),
            "arena-v1",
            options.SessionImage,
            "test",
            options.MaxRobotsPerSession);

        var arguments = DockerArgumentBuilder.BuildCreateContainer(specification, options);

        Assert.Contains("gui:=false", arguments);
        Assert.Contains("NVIDIA_DRIVER_CAPABILITIES=compute,graphics,utility,video", arguments);
        Assert.DoesNotContain(
            arguments,
            argument => argument.StartsWith("DISPLAY=", StringComparison.Ordinal));
    }

    [Fact]
    public void SessionNetworkIsInternalAndLabeled()
    {
        var workerId = Guid.NewGuid();
        var sessionId = Guid.NewGuid();

        var arguments = DockerArgumentBuilder.BuildCreateNetwork(workerId, sessionId);

        Assert.Contains("--internal", arguments);
        Assert.Contains($"{SessionLabels.WorkerId}={workerId:D}", arguments);
        Assert.Contains($"{SessionLabels.SessionId}={sessionId:D}", arguments);
        Assert.Contains(SessionResourceNames.Network(sessionId), arguments);
    }

    private static WorkerOptions ValidOptions() =>
        new()
        {
            BackendUrl = "https://backend.example.test",
            WorkerId = Guid.NewGuid(),
            WorkerSecret = new string('a', 32),
            Name = "test-worker",
            SessionImage = "robotswarm/ros-noetic:test",
            MaxConcurrentSessions = 4,
            MaxRobotsPerSession = 10
        };
}
