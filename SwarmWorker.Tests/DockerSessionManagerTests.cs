using System.Text.Json;
using Microsoft.Extensions.Logging.Abstractions;
using Microsoft.Extensions.Options;
using SwarmWorker.Configuration;
using SwarmWorker.Contracts;
using SwarmWorker.Infrastructure;
using SwarmWorker.Runtime;

namespace SwarmWorker.Tests;

public sealed class DockerSessionManagerTests
{
    [Fact]
    public async Task ProvisionRejectsACompleteRosterWhenGazeboStoppedAfterSpawn()
    {
        var workerId = Guid.NewGuid();
        var sessionId = Guid.NewGuid();
        var robotIds = FleetRosterParser.GenerateRobotIds(10);
        var roster = $"data: {JsonSerializer.Serialize(string.Join(',', robotIds))}\n";
        var docker = new RecordingDockerCli(
            new DockerCommandResult(0, "container-1\n", string.Empty),
            new DockerCommandResult(
                0,
                BuildInspection(workerId, sessionId),
                string.Empty),
            new DockerCommandResult(0, roster, string.Empty),
            new DockerCommandResult(0, "ready", string.Empty),
            new DockerCommandResult(0, roster, string.Empty),
            new DockerCommandResult(1, string.Empty, "Gazebo clock did not advance"));
        var manager = CreateManager(docker, workerId);
        var payload = new FleetCommandPayload(
            10,
            "arena-v1",
            "grid",
            robotIds);

        var error = await Assert.ThrowsAsync<DockerCliException>(
            () => manager.ProvisionAsync(
                sessionId,
                payload,
                CancellationToken.None));

        Assert.Equal("verify Gazebo fleet liveness", error.Operation);
        var probe = docker.Calls[^1];
        Assert.Contains("/gazebo/model_states", probe[4]);
        Assert.Contains("/clock", probe[4]);
        Assert.Equal(robotIds, probe.Skip(6));
    }

    [Fact]
    public async Task ProvisionDoesNotReuseAContainerWithTheObsoletePidsLimit()
    {
        var workerId = Guid.NewGuid();
        var sessionId = Guid.NewGuid();
        var docker = new RecordingDockerCli(
            new DockerCommandResult(0, "container-1\n", string.Empty),
            new DockerCommandResult(
                0,
                BuildInspection(workerId, sessionId, pidsLimit: 512),
                string.Empty));
        var manager = CreateManager(docker, workerId);
        var payload = new FleetCommandPayload(
            1,
            "arena-v1",
            "grid",
            new[] { "tb3_0" });

        var error = await Assert.ThrowsAsync<InvalidOperationException>(
            () => manager.ProvisionAsync(
                sessionId,
                payload,
                CancellationToken.None));

        Assert.Contains("does not match", error.Message);
        Assert.DoesNotContain(
            docker.Calls,
            arguments => arguments.Count > 0 && arguments[0] == "exec");
    }

    [Theory]
    [InlineData("")]
    [InlineData("not ready\n")]
    public async Task GazeboFleetProbeRequiresAnExplicitReadyResult(string output)
    {
        var docker = new RecordingDockerCli(
            new DockerCommandResult(0, output, string.Empty));
        var manager = CreateManager(docker, Guid.NewGuid());

        var error = await Assert.ThrowsAsync<InvalidOperationException>(
            () => manager.VerifyGazeboFleetAsync(
                "container-1",
                new[] { "tb3_0" },
                CancellationToken.None));

        Assert.Contains("without confirming liveness", error.Message);
        Assert.Equal(TimeSpan.FromSeconds(20), docker.Timeouts.Single());
    }

    [Fact]
    public async Task PublishesControlHeartbeatThroughTheFixedRosTopic()
    {
        var workerId = Guid.NewGuid();
        var sessionId = Guid.NewGuid();
        var docker = new RecordingDockerCli(
            new DockerCommandResult(0, string.Empty, string.Empty));
        var manager = new DockerSessionManager(
            docker,
            new RecordingViewerPublisher(),
            Options.Create(new WorkerOptions
            {
                BackendUrl = "https://backend.example.test",
                WorkerId = workerId,
                WorkerSecret = new string('a', 32),
                Name = "test-worker",
                SessionImage = "robotswarm/ros-noetic:test"
            }),
            NullLogger<DockerSessionManager>.Instance);
        var session = new ManagedSessionInfo(
            sessionId,
            "container-1",
            SessionResourceNames.Container(sessionId),
            "robotswarm/ros-noetic:test",
            1024,
            true,
            "running",
            new Dictionary<string, string>
            {
                [SessionLabels.WorkerId] = workerId.ToString("D")
            });

        await manager.PublishControlHeartbeatAsync(
            session,
            123.5,
            CancellationToken.None);

        var arguments = Assert.Single(docker.Calls);
        Assert.Equal("exec", arguments[0]);
        Assert.Equal("/usr/bin/timeout", arguments[2]);
        Assert.Equal("--signal=KILL", arguments[3]);
        Assert.Equal(
            $"{WorkerOptions.ControlHeartbeatPublishTimeoutSeconds}s",
            arguments[4]);
        Assert.Equal("/bin/bash", arguments[5]);
        Assert.Equal("-lc", arguments[6]);
        Assert.Contains("/proc/uptime", arguments[7]);
        Assert.Contains(
            "rostopic pub -1 /swarm/control_heartbeat std_msgs/Float64",
            arguments[7]);
        Assert.Contains("\"data: $1\"", arguments[7]);
        Assert.Equal("swarm-worker", arguments[8]);
        Assert.Equal("123.5", arguments[9]);
        Assert.Equal(
            TimeSpan.FromSeconds(
                WorkerOptions.ControlHeartbeatDockerExecTimeoutSeconds),
            docker.Timeouts.Single());
    }

    [Theory]
    [InlineData(double.NaN)]
    [InlineData(double.PositiveInfinity)]
    [InlineData(double.NegativeInfinity)]
    public async Task RejectsANonFiniteControlHeartbeatDeadline(double deadline)
    {
        var workerId = Guid.NewGuid();
        var sessionId = Guid.NewGuid();
        var docker = new RecordingDockerCli();
        var manager = CreateManager(docker, workerId);
        var session = new ManagedSessionInfo(
            sessionId,
            "container-1",
            SessionResourceNames.Container(sessionId),
            "robotswarm/ros-noetic:test",
            1024,
            true,
            "running",
            new Dictionary<string, string>
            {
                [SessionLabels.WorkerId] = workerId.ToString("D")
            });

        await Assert.ThrowsAsync<ArgumentOutOfRangeException>(
            () => manager.PublishControlHeartbeatAsync(
                session,
                deadline,
                CancellationToken.None));

        Assert.Empty(docker.Calls);
    }

    [Fact]
    public async Task HeartbeatDiscoveryUsesOneBoundedRunningContainerList()
    {
        var workerId = Guid.NewGuid();
        var sessionId = Guid.NewGuid();
        var docker = new RecordingDockerCli(
            new DockerCommandResult(
                0,
                $"container-1\trobotswarm-test\trobotswarm:test\t{sessionId:D}\n",
                string.Empty));
        var manager = CreateManager(docker, workerId);

        var session = Assert.Single(
            await manager.GetHeartbeatSessionsAsync(CancellationToken.None));

        Assert.Equal(sessionId, session.SessionId);
        Assert.Equal("container-1", session.ContainerId);
        Assert.Equal("robotswarm-test", session.ContainerName);
        Assert.True(session.Running);
        var arguments = Assert.Single(docker.Calls);
        Assert.Equal("ps", arguments[0]);
        Assert.DoesNotContain("-a", arguments);
        Assert.Equal(
            TimeSpan.FromSeconds(
                WorkerOptions.ControlHeartbeatDiscoveryTimeoutSeconds),
            docker.Timeouts.Single());
    }

    [Fact]
    public async Task ReadsSwarmStatusWithoutRosYamlFolding()
    {
        var workerId = Guid.NewGuid();
        var sessionId = Guid.NewGuid();
        var taskRunId = Guid.NewGuid();
        var statusJson = JsonSerializer.Serialize(new
        {
            task = new
            {
                task_id = taskRunId,
                status = "running",
                progress = 0.25
            },
            robots = Enumerable.Range(0, 20).Select(index => new
            {
                id = $"tb3_{index}",
                ranges = Enumerable.Repeat(3.5, 360)
            })
        });
        var docker = new RecordingDockerCli(
            new DockerCommandResult(
                0,
                $"data: {JsonSerializer.Serialize(statusJson)}\n",
                string.Empty));
        var manager = new DockerSessionManager(
            docker,
            new RecordingViewerPublisher(),
            Options.Create(new WorkerOptions
            {
                BackendUrl = "https://backend.example.test",
                WorkerId = workerId,
                WorkerSecret = new string('a', 32),
                Name = "test-worker",
                SessionImage = "robotswarm/ros-noetic:test"
            }),
            NullLogger<DockerSessionManager>.Instance);
        var session = new ManagedSessionInfo(
            sessionId,
            "container-1",
            SessionResourceNames.Container(sessionId),
            "robotswarm/ros-noetic:test",
            1024,
            true,
            "running",
            new Dictionary<string, string>
            {
                [SessionLabels.WorkerId] = workerId.ToString("D")
            });

        var status = await manager.ReadTaskStatusAsync(
            session,
            CancellationToken.None);

        Assert.NotNull(status);
        Assert.Equal(taskRunId, status.TaskRunId);
        Assert.Equal("Running", status.State);
        Assert.Equal(0.25, status.Progress);

        var arguments = Assert.Single(docker.Calls);
        Assert.Contains("rospy.wait_for_message", arguments[4]);
        Assert.Contains("timeout=5.0", arguments[4]);
        Assert.Contains("json.dumps(message.data)", arguments[4]);
        Assert.DoesNotContain(
            "rostopic echo -n 1 /swarm/status",
            arguments[4]);
    }

    [Fact]
    public async Task TaskCancellationWaitsForTheCorrelatedRosTerminalState()
    {
        var workerId = Guid.NewGuid();
        var sessionId = Guid.NewGuid();
        var taskRunId = Guid.NewGuid();
        var staleStatus = JsonSerializer.Serialize(new
        {
            task = new
            {
                task_id = Guid.NewGuid(),
                status = "stopped",
                progress = 0
            }
        });
        var confirmedStatus = JsonSerializer.Serialize(new
        {
            task = new
            {
                task_id = taskRunId,
                status = "stopped",
                progress = 0
            }
        });
        var docker = new RecordingDockerCli(
            new DockerCommandResult(
                0,
                $"data: {JsonSerializer.Serialize(staleStatus)}\n",
                string.Empty),
            new DockerCommandResult(
                0,
                $"data: {JsonSerializer.Serialize(confirmedStatus)}\n",
                string.Empty));
        var manager = new DockerSessionManager(
            docker,
            new RecordingViewerPublisher(),
            Options.Create(new WorkerOptions
            {
                WorkerId = workerId,
                SessionImage = "robotswarm/ros-noetic:test",
                TaskCancellationTimeoutSeconds = 2
            }),
            NullLogger<DockerSessionManager>.Instance);
        var session = new ManagedSessionInfo(
            sessionId,
            "container-1",
            SessionResourceNames.Container(sessionId),
            "robotswarm/ros-noetic:test",
            1024,
            true,
            "running",
            new Dictionary<string, string>
            {
                [SessionLabels.WorkerId] = workerId.ToString("D")
            });

        await manager.WaitForTaskStoppedAsync(
            session,
            taskRunId,
            CancellationToken.None);

        Assert.Equal(2, docker.Calls.Count);
        Assert.All(docker.Calls, arguments =>
            Assert.Contains("rospy.wait_for_message", arguments[4]));
    }

    [Fact]
    public async Task ReconciliationIgnoresAnExplicitlyMissingContainer()
    {
        var workerId = Guid.NewGuid();
        var docker = new RecordingDockerCli(
            new DockerCommandResult(0, "container-1\n", string.Empty),
            new DockerCommandResult(
                1,
                string.Empty,
                "Error: No such object: container-1"));
        var manager = CreateManager(docker, workerId);

        var sessions = await manager.GetManagedSessionsAsync(
            CancellationToken.None);

        Assert.Empty(sessions);
    }

    [Fact]
    public async Task ManagedSessionInventoryIncludesStoppedContainers()
    {
        var workerId = Guid.NewGuid();
        var runningSessionId = Guid.NewGuid();
        var stoppedSessionId = Guid.NewGuid();
        var docker = new RecordingDockerCli(
            new DockerCommandResult(
                0,
                "container-running\ncontainer-stopped\n",
                string.Empty),
            new DockerCommandResult(
                0,
                BuildInspection(
                    workerId,
                    runningSessionId,
                    "container-running",
                    running: true),
                string.Empty),
            new DockerCommandResult(
                0,
                BuildInspection(
                    workerId,
                    stoppedSessionId,
                    "container-stopped",
                    running: false),
                string.Empty));
        var manager = CreateManager(docker, workerId);

        var sessions = await manager.GetManagedSessionsAsync(
            CancellationToken.None);

        Assert.Contains(sessions, session =>
            session.SessionId == runningSessionId && session.Running);
        Assert.Contains(sessions, session =>
            session.SessionId == stoppedSessionId && !session.Running);
        Assert.Contains("-a", docker.Calls[0]);
    }

    [Theory]
    [InlineData("permission denied while contacting the Docker daemon")]
    [InlineData("network plugin not found while inspecting container metadata")]
    public async Task ReconciliationReportsTransientContainerInspectFailure(
        string standardError)
    {
        var workerId = Guid.NewGuid();
        var docker = new RecordingDockerCli(
            new DockerCommandResult(0, "container-1\n", string.Empty),
            new DockerCommandResult(
                1,
                string.Empty,
                standardError));
        var manager = CreateManager(docker, workerId);

        var error = await Assert.ThrowsAsync<DockerCliException>(
            () => manager.GetManagedSessionsAsync(CancellationToken.None));

        Assert.Equal("inspect managed container", error.Operation);
    }

    [Fact]
    public async Task PublishesSwarmJsonAsASeparateDockerArgument()
    {
        var workerId = Guid.NewGuid();
        var sessionId = Guid.NewGuid();
        var docker = new RecordingDockerCli(
            new DockerCommandResult(0, "container-1\n", string.Empty),
            new DockerCommandResult(
                0,
                BuildInspection(workerId, sessionId),
                string.Empty),
            new DockerCommandResult(0, string.Empty, string.Empty));
        var options = new WorkerOptions
        {
            BackendUrl = "https://backend.example.test",
            WorkerId = workerId,
            WorkerSecret = new string('a', 32),
            Name = "test-worker",
            SessionImage = "robotswarm/ros-noetic:test"
        };
        var manager = new DockerSessionManager(
            docker,
            new RecordingViewerPublisher(),
            Options.Create(options),
            NullLogger<DockerSessionManager>.Instance);
        var command = JsonSerializer.SerializeToElement(new
        {
            command = "start_task",
            parameters = new
            {
                task_id = Guid.NewGuid(),
                label = "\"; touch /tmp/should-not-run; #"
            }
        });

        await manager.PublishSwarmCommandAsync(
            sessionId,
            command,
            CancellationToken.None);

        var publishArguments = docker.Calls[2];
        Assert.Equal("exec", publishArguments[0]);
        Assert.Equal("container-1", publishArguments[1]);
        Assert.Contains(
            "rostopic pub -1 /swarm/commands std_msgs/String \"$1\"",
            publishArguments[4]);
        Assert.DoesNotContain("touch /tmp/should-not-run", publishArguments[4]);
        Assert.Contains("touch /tmp/should-not-run", publishArguments[6]);
    }

    [Fact]
    public async Task StartsViewerPublisherForTheOwnedRunningSession()
    {
        var workerId = Guid.NewGuid();
        var sessionId = Guid.NewGuid();
        var docker = new RecordingDockerCli(
            new DockerCommandResult(0, "container-1\n", string.Empty),
            new DockerCommandResult(
                0,
                BuildInspection(workerId, sessionId),
                string.Empty));
        var publisher = new RecordingViewerPublisher();
        var manager = new DockerSessionManager(
            docker,
            publisher,
            Options.Create(new WorkerOptions
            {
                WorkerId = workerId,
                SessionImage = "robotswarm/ros-noetic:test"
            }),
            NullLogger<DockerSessionManager>.Instance);
        var command = SceneCommand(sessionId);

        var result = await manager.SetViewerSourceAsync(
            sessionId,
            command,
            CancellationToken.None);

        Assert.True(result.Ready);
        var request = Assert.Single(publisher.PublishRequests);
        Assert.Equal(sessionId, request.SessionId);
        Assert.Equal("container-1", request.ContainerId);
        Assert.Equal(command, request.Command);
    }

    [Fact]
    public async Task StopsViewerPublisherEvenWhenSessionContainerIsGone()
    {
        var sessionId = Guid.NewGuid();
        var docker = new RecordingDockerCli(
            new DockerCommandResult(0, string.Empty, string.Empty),
            new DockerCommandResult(1, string.Empty, "No such network"));
        var publisher = new RecordingViewerPublisher();
        var manager = new DockerSessionManager(
            docker,
            publisher,
            Options.Create(new WorkerOptions
            {
                WorkerId = Guid.NewGuid(),
                SessionImage = "robotswarm/ros-noetic:test"
            }),
            NullLogger<DockerSessionManager>.Instance);

        await manager.StopAsync(sessionId, CancellationToken.None);

        Assert.Equal(new[] { sessionId }, publisher.StoppedSessions);
    }

    [Fact]
    public async Task ContinuesDockerCleanupWhenViewerStopFails()
    {
        var sessionId = Guid.NewGuid();
        var docker = new RecordingDockerCli(
            new DockerCommandResult(0, string.Empty, string.Empty),
            new DockerCommandResult(1, string.Empty, "No such network"));
        var publisher = new RecordingViewerPublisher(
            new InvalidOperationException("viewer stop failed"));
        var manager = new DockerSessionManager(
            docker,
            publisher,
            Options.Create(new WorkerOptions
            {
                WorkerId = Guid.NewGuid(),
                SessionImage = "robotswarm/ros-noetic:test"
            }),
            NullLogger<DockerSessionManager>.Instance);

        var error = await Assert.ThrowsAsync<InvalidOperationException>(
            () => manager.StopAsync(sessionId, CancellationToken.None));

        Assert.Equal("viewer stop failed", error.Message);
        Assert.Equal(2, docker.Calls.Count);
        Assert.Equal("ps", docker.Calls[0][0]);
        Assert.Equal(new[] { "network", "inspect", SessionResourceNames.Network(sessionId) },
            docker.Calls[1]);
    }

    [Fact]
    public async Task CallerCancellationIsReportedAfterBoundedCleanup()
    {
        var workerId = Guid.NewGuid();
        var sessionId = Guid.NewGuid();
        var docker = new RecordingDockerCli(
            new DockerCommandResult(0, "container-1\n", string.Empty),
            new DockerCommandResult(
                0,
                BuildInspection(workerId, sessionId),
                string.Empty),
            new DockerCommandResult(0, string.Empty, string.Empty),
            new DockerCommandResult(0, string.Empty, string.Empty),
            new DockerCommandResult(
                0,
                BuildNetworkInspection(workerId, sessionId),
                string.Empty),
            new DockerCommandResult(0, string.Empty, string.Empty));
        var manager = CreateManager(docker, workerId);
        using var cancellation = new CancellationTokenSource();
        cancellation.Cancel();

        await Assert.ThrowsAnyAsync<OperationCanceledException>(
            () => manager.StopAsync(sessionId, cancellation.Token));

        Assert.Contains(
            docker.Calls,
            arguments => arguments.SequenceEqual(
                new[] { "rm", "--force", "container-1" }));
        Assert.Contains(
            docker.Calls,
            arguments => arguments.SequenceEqual(
                new[] { "network", "rm", "network-1" }));
    }

    [Fact]
    public async Task ForcesRemovalWhenGracefulContainerStopFails()
    {
        var workerId = Guid.NewGuid();
        var sessionId = Guid.NewGuid();
        var docker = new RecordingDockerCli(
            new DockerCommandResult(0, "container-1\n", string.Empty),
            new DockerCommandResult(
                0,
                BuildInspection(workerId, sessionId),
                string.Empty),
            new DockerCommandResult(1, string.Empty, "stop failed"),
            new DockerCommandResult(0, string.Empty, string.Empty),
            new DockerCommandResult(1, string.Empty, "No such network"));
        var manager = CreateManager(docker, workerId);

        await manager.StopAsync(sessionId, CancellationToken.None);

        Assert.Equal(
            new[] { "stop", "--time", "30", "container-1" },
            docker.Calls[2]);
        Assert.Equal(
            new[] { "rm", "--force", "container-1" },
            docker.Calls[3]);
        Assert.Equal(TimeSpan.FromSeconds(30), docker.Timeouts[3]);
    }

    [Fact]
    public async Task ForcesRemovalWhenGracefulContainerStopTimesOut()
    {
        var workerId = Guid.NewGuid();
        var sessionId = Guid.NewGuid();
        var docker = new RecordingDockerCli(
            new DockerCommandResult(0, "container-1\n", string.Empty),
            new DockerCommandResult(
                0,
                BuildInspection(workerId, sessionId),
                string.Empty),
            new TimeoutException("docker stop timed out"),
            new DockerCommandResult(0, string.Empty, string.Empty),
            new DockerCommandResult(1, string.Empty, "No such network"));
        var manager = CreateManager(docker, workerId);

        await manager.StopAsync(sessionId, CancellationToken.None);

        Assert.Equal(
            new[] { "rm", "--force", "container-1" },
            docker.Calls[3]);
    }

    [Fact]
    public async Task AcceptsFailedForceRemoveOnlyAfterContainerIsVerifiedGone()
    {
        var workerId = Guid.NewGuid();
        var sessionId = Guid.NewGuid();
        var docker = new RecordingDockerCli(
            new DockerCommandResult(0, "container-1\n", string.Empty),
            new DockerCommandResult(
                0,
                BuildInspection(workerId, sessionId),
                string.Empty),
            new DockerCommandResult(1, string.Empty, "stop failed"),
            new DockerCommandResult(1, string.Empty, "remove raced"),
            new DockerCommandResult(1, string.Empty, "No such object: container-1"),
            new DockerCommandResult(1, string.Empty, "No such network"));
        var manager = CreateManager(docker, workerId);

        await manager.StopAsync(sessionId, CancellationToken.None);

        Assert.Equal(
            new[] { "inspect", "container-1" },
            docker.Calls[4]);
    }

    [Fact]
    public async Task ReportsFailedForceRemoveWhenOwnedContainerStillExists()
    {
        var workerId = Guid.NewGuid();
        var sessionId = Guid.NewGuid();
        var docker = new RecordingDockerCli(
            new DockerCommandResult(0, "container-1\n", string.Empty),
            new DockerCommandResult(
                0,
                BuildInspection(workerId, sessionId),
                string.Empty),
            new DockerCommandResult(1, string.Empty, "stop failed"),
            new DockerCommandResult(1, string.Empty, "remove failed"),
            new DockerCommandResult(
                0,
                BuildInspection(workerId, sessionId),
                string.Empty),
            new DockerCommandResult(1, string.Empty, "No such network"));
        var manager = CreateManager(docker, workerId);

        var error = await Assert.ThrowsAsync<DockerCliException>(
            () => manager.StopAsync(sessionId, CancellationToken.None));

        Assert.Equal("remove session container", error.Operation);
        Assert.DoesNotContain(
            docker.Calls,
            arguments => arguments.Count >= 2
                         && arguments[0] == "network"
                         && arguments[1] == "rm");
    }

    [Fact]
    public async Task DoesNotStopOrRemoveContainerOwnedByAnotherWorker()
    {
        var workerId = Guid.NewGuid();
        var sessionId = Guid.NewGuid();
        var docker = new RecordingDockerCli(
            new DockerCommandResult(0, "container-1\n", string.Empty),
            new DockerCommandResult(
                0,
                BuildInspection(Guid.NewGuid(), sessionId),
                string.Empty),
            new DockerCommandResult(1, string.Empty, "No such network"));
        var manager = CreateManager(docker, workerId);

        await Assert.ThrowsAsync<InvalidOperationException>(
            () => manager.StopAsync(sessionId, CancellationToken.None));

        Assert.DoesNotContain(
            docker.Calls,
            arguments => arguments.Count > 0
                         && arguments[0] is "stop" or "rm");
    }

    [Fact]
    public async Task AcceptsFailedNetworkRemoveOnlyAfterNetworkIsVerifiedGone()
    {
        var workerId = Guid.NewGuid();
        var sessionId = Guid.NewGuid();
        var docker = new RecordingDockerCli(
            new DockerCommandResult(0, string.Empty, string.Empty),
            new DockerCommandResult(
                0,
                BuildNetworkInspection(workerId, sessionId),
                string.Empty),
            new DockerCommandResult(1, string.Empty, "remove raced"),
            new DockerCommandResult(1, string.Empty, "No such network"));
        var manager = CreateManager(docker, workerId);

        await manager.StopAsync(sessionId, CancellationToken.None);

        Assert.Equal(
            new[] { "network", "rm", "network-1" },
            docker.Calls[2]);
        Assert.Equal(
            new[] { "network", "inspect", "network-1" },
            docker.Calls[3]);
    }

    [Fact]
    public async Task RemovesTheExactInspectedNetworkId()
    {
        var workerId = Guid.NewGuid();
        var sessionId = Guid.NewGuid();
        const string networkId = "immutable-network-id";
        var docker = new RecordingDockerCli(
            new DockerCommandResult(0, string.Empty, string.Empty),
            new DockerCommandResult(
                0,
                BuildNetworkInspection(workerId, sessionId, networkId),
                string.Empty),
            new DockerCommandResult(0, string.Empty, string.Empty));
        var manager = CreateManager(docker, workerId);

        await manager.StopAsync(sessionId, CancellationToken.None);

        Assert.Equal(
            new[] { "network", "rm", networkId },
            docker.Calls[2]);
    }

    [Fact]
    public async Task ReportsFailedNetworkRemoveWhenOwnedNetworkStillExists()
    {
        var workerId = Guid.NewGuid();
        var sessionId = Guid.NewGuid();
        var networkInspection = BuildNetworkInspection(workerId, sessionId);
        var docker = new RecordingDockerCli(
            new DockerCommandResult(0, string.Empty, string.Empty),
            new DockerCommandResult(0, networkInspection, string.Empty),
            new DockerCommandResult(1, string.Empty, "network is still in use"),
            new DockerCommandResult(0, networkInspection, string.Empty));
        var manager = CreateManager(docker, workerId);

        var error = await Assert.ThrowsAsync<DockerCliException>(
            () => manager.StopAsync(sessionId, CancellationToken.None));

        Assert.Equal("remove session network", error.Operation);
    }

    [Fact]
    public async Task ReportsNetworkInspectionFailureInsteadOfAssumingCleanup()
    {
        var workerId = Guid.NewGuid();
        var sessionId = Guid.NewGuid();
        var docker = new RecordingDockerCli(
            new DockerCommandResult(0, string.Empty, string.Empty),
            new DockerCommandResult(1, string.Empty, "daemon unavailable"));
        var manager = CreateManager(docker, workerId);

        var error = await Assert.ThrowsAsync<DockerCliException>(
            () => manager.StopAsync(sessionId, CancellationToken.None));

        Assert.Equal("inspect session network", error.Operation);
    }

    private static DockerSessionManager CreateManager(
        IDockerCli docker,
        Guid workerId)
    {
        return new DockerSessionManager(
            docker,
            new RecordingViewerPublisher(),
            Options.Create(new WorkerOptions
            {
                WorkerId = workerId,
                SessionImage = "robotswarm/ros-noetic:test"
            }),
            NullLogger<DockerSessionManager>.Instance);
    }

    private static ViewerSourceCommand SceneCommand(Guid sessionId)
    {
        var sourceId = $"scene-{sessionId:N}";
        return new ViewerSourceCommand(
            Guid.NewGuid(),
            DateTimeOffset.UtcNow.AddMinutes(5),
            "AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA",
            ViewerSourceKind.Scene,
            null,
            sourceId,
            $"session/{sessionId:N}/{sourceId}");
    }

    private static string BuildInspection(
        Guid workerId,
        Guid sessionId,
        string containerId = "container-1",
        bool running = true,
        int pidsLimit = 1024)
    {
        return JsonSerializer.Serialize(new[]
        {
            new
            {
                Id = containerId,
                Name = $"/{SessionResourceNames.Container(sessionId)}",
                Config = new
                {
                    Image = "robotswarm/ros-noetic:test",
                    Labels = new Dictionary<string, string>
                    {
                        [SessionLabels.Managed] = "true",
                        [SessionLabels.WorkerId] = workerId.ToString("D"),
                        [SessionLabels.SessionId] = sessionId.ToString("D"),
                        [SessionLabels.ArenaVersion] = "arena-v1",
                        [SessionLabels.ImageVersion] = string.Empty,
                        [SessionLabels.MaxRobots] = "10"
                    }
                },
                HostConfig = new
                {
                    PidsLimit = pidsLimit
                },
                State = new
                {
                    Running = running,
                    Status = running ? "running" : "exited"
                }
            }
        });
    }

    private static string BuildNetworkInspection(
        Guid workerId,
        Guid sessionId,
        string networkId = "network-1")
    {
        return JsonSerializer.Serialize(new[]
        {
            new
            {
                Id = networkId,
                Labels = new Dictionary<string, string>
                {
                    [SessionLabels.WorkerId] = workerId.ToString("D"),
                    [SessionLabels.SessionId] = sessionId.ToString("D")
                }
            }
        });
    }

    private sealed class RecordingDockerCli : IDockerCli
    {
        private readonly Queue<object> _results;

        public RecordingDockerCli(params object[] results)
        {
            _results = new Queue<object>(results);
        }

        public List<IReadOnlyList<string>> Calls { get; } = new();
        public List<TimeSpan?> Timeouts { get; } = new();

        public Task<DockerCommandResult> RunAsync(
            IReadOnlyList<string> arguments,
            CancellationToken cancellationToken,
            TimeSpan? timeout = null)
        {
            cancellationToken.ThrowIfCancellationRequested();
            Calls.Add(arguments.ToArray());
            Timeouts.Add(timeout);
            var next = _results.Dequeue();
            if (next is Exception exception)
            {
                return Task.FromException<DockerCommandResult>(exception);
            }

            return Task.FromResult((DockerCommandResult)next);
        }
    }

    private sealed class RecordingViewerPublisher : IViewerPublisher
    {
        private readonly Exception? _stopFailure;

        public RecordingViewerPublisher(Exception? stopFailure = null)
        {
            _stopFailure = stopFailure;
        }

        public ViewerPublisherAvailability Availability { get; } =
            new(
                true,
                new[] { ViewerSourceKind.Scene },
                "H264",
                "ready");

        public List<ViewerPublishRequest> PublishRequests { get; } = new();
        public List<Guid> StoppedSessions { get; } = new();
        public List<(Guid SessionId, Guid LeaseId)> StoppedLeases { get; } = new();

        public Task RefreshAvailabilityAsync(CancellationToken cancellationToken)
        {
            return Task.CompletedTask;
        }

        public Task<ViewerPublishResult> PublishAsync(
            ViewerPublishRequest request,
            CancellationToken cancellationToken)
        {
            PublishRequests.Add(request);
            return Task.FromResult(new ViewerPublishResult(
                request.SessionId,
                request.Command.LeaseId,
                request.Command.ExpiresAt,
                request.Command.Source,
                request.Command.RobotRuntimeId,
                request.Command.SourceId,
                request.Command.StreamPath,
                Ready: true,
                "H264"));
        }

        public Task StopSessionAsync(
            Guid sessionId,
            CancellationToken cancellationToken)
        {
            cancellationToken.ThrowIfCancellationRequested();
            StoppedSessions.Add(sessionId);
            if (_stopFailure is not null)
            {
                return Task.FromException(_stopFailure);
            }

            return Task.CompletedTask;
        }

        public Task SendInputAsync(
            ViewerInputEnvelope request,
            CancellationToken cancellationToken) => Task.CompletedTask;

        public Task ReleaseInputAsync(
            Guid sessionId,
            Guid leaseId,
            CancellationToken cancellationToken) => Task.CompletedTask;

        public Task ReleaseAllInputsAsync(CancellationToken cancellationToken) =>
            Task.CompletedTask;

        public Task<bool> StopLeaseAsync(
            Guid sessionId,
            Guid leaseId,
            CancellationToken cancellationToken)
        {
            cancellationToken.ThrowIfCancellationRequested();
            StoppedLeases.Add((sessionId, leaseId));
            return Task.FromResult(true);
        }
    }
}
