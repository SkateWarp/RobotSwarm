using System.Collections.Concurrent;
using System.Text.Json;
using Microsoft.Extensions.Logging.Abstractions;
using Microsoft.Extensions.Options;
using SwarmWorker.Configuration;
using SwarmWorker.Contracts;
using SwarmWorker.Infrastructure;
using SwarmWorker.Runtime;
using SwarmWorker.Services;

namespace SwarmWorker.Tests;

public sealed class BoundedCommandExecutorTests
{
    [Theory]
    [InlineData(false, true)]
    [InlineData(true, false)]
    public async Task CancelledStaleEnvelopeNeverReachesTheRuntime(
        bool acceptAcknowledgement,
        bool acceptRunningTransition)
    {
        var sessionId = Guid.NewGuid();
        var docker = new RecordingDockerCli();
        var publisher = new RecordingViewerPublisher();
        var hub = new RecordingWorkerCommandHub
        {
            AcceptAcknowledgement = acceptAcknowledgement,
            AcceptRunningTransition = acceptRunningTransition
        };
        var executor = CreateExecutor(docker, publisher, hub);
        var command = SessionControlCommand(sessionId, "CancelTask");

        await executor.StartAsync(CancellationToken.None);
        try
        {
            Assert.True(await executor.EnqueueAsync(command, CancellationToken.None));
            var failure = await hub.WaitForFailureAsync();

            Assert.Equal(command.Id, failure.CommandId);
            Assert.Empty(hub.CompletedCommands);
            Assert.Empty(hub.SessionEvents);
            Assert.Empty(docker.Calls);
            Assert.Empty(publisher.StoppedSessions);
        }
        finally
        {
            await executor.StopAsync(CancellationToken.None);
        }
    }

    [Fact]
    public async Task LocalEmergencyStopRejectsQueuedFleetUpdateBeforeAcknowledgement()
    {
        var sessionId = Guid.NewGuid();
        var docker = new RecordingDockerCli();
        var publisher = new RecordingViewerPublisher();
        var hub = new RecordingWorkerCommandHub();
        var executor = CreateExecutor(docker, publisher, hub);
        var command = UpdateFleetCommand(sessionId);

        await executor.StartAsync(CancellationToken.None);
        try
        {
            executor.RequestLocalEmergencyStop(sessionId);
            Assert.True(await executor.EnqueueAsync(command, CancellationToken.None));

            var failure = await hub.WaitForFailureAsync();

            Assert.Equal(command.Id, failure.CommandId);
            Assert.Contains("local emergency stop", failure.Error);
            Assert.Empty(hub.AcknowledgedCommands);
            Assert.Empty(hub.RunningCommands);
            Assert.Empty(hub.CompletedCommands);
            Assert.Empty(hub.SessionEvents);
            Assert.Empty(docker.Calls);
            Assert.Empty(publisher.StoppedSessions);
        }
        finally
        {
            await executor.StopAsync(CancellationToken.None);
        }
    }

    [Theory]
    [InlineData("ResetEmergencyStop")]
    [InlineData("StopSession")]
    public async Task NewEmergencyStopSurvivesConcurrentLatchClearingCommand(
        string clearingCommandType)
    {
        var sessionId = Guid.NewGuid();
        var workerId = Guid.NewGuid();
        var dockerResults = clearingCommandType == "ResetEmergencyStop"
            ? new[]
            {
                new DockerCommandResult(0, "container-1\n", string.Empty),
                new DockerCommandResult(
                    0,
                    SessionInspection(workerId, sessionId),
                    string.Empty),
                new DockerCommandResult(0, string.Empty, string.Empty),
                new DockerCommandResult(0, "container-1\n", string.Empty),
                new DockerCommandResult(
                    0,
                    SessionInspection(workerId, sessionId),
                    string.Empty),
                new DockerCommandResult(
                    0,
                    EmergencyStopStatus(active: false),
                    string.Empty)
            }
            : new[]
            {
                new DockerCommandResult(0, string.Empty, string.Empty),
                new DockerCommandResult(1, string.Empty, "No such network")
            };
        var docker = new RecordingDockerCli(dockerResults);
        var hub = new RecordingWorkerCommandHub();
        var options = Options.Create(new WorkerOptions
        {
            WorkerId = workerId,
            SessionImage = "robotswarm/ros-noetic:test",
            MaxQueuedCommands = 4,
            MaxParallelCommands = 1,
            ShutdownDrainSeconds = 5
        });
        var sessions = new DockerSessionManager(
            docker,
            new RecordingViewerPublisher(),
            options,
            NullLogger<DockerSessionManager>.Instance);
        var executor = new BoundedCommandExecutor(
            new SessionCommandHandler(sessions, options),
            hub,
            new TaskStatusTracker(),
            options,
            NullLogger<BoundedCommandExecutor>.Instance);
        var clearingCommand = SessionControlCommand(
            sessionId,
            clearingCommandType);
        hub.BlockCompletion(clearingCommand.Id);

        await executor.StartAsync(CancellationToken.None);
        try
        {
            executor.RequestLocalEmergencyStop(sessionId);
            Assert.True(await executor.EnqueueAsync(
                clearingCommand,
                CancellationToken.None));
            await hub.WaitForBlockedCompletionAsync();

            executor.RequestLocalEmergencyStop(sessionId);
            hub.ReleaseBlockedCompletion();
            await WaitUntilAsync(() => hub.CompletedCommands.Any(
                completion => completion.CommandId == clearingCommand.Id));

            var fleetUpdate = UpdateFleetCommand(sessionId);
            Assert.True(await executor.EnqueueAsync(
                fleetUpdate,
                CancellationToken.None));
            var failure = await hub.WaitForFailureAsync();

            Assert.Equal(fleetUpdate.Id, failure.CommandId);
            Assert.Contains("local emergency stop", failure.Error);
            Assert.Equal(dockerResults.Length, docker.Calls.Count);
        }
        finally
        {
            hub.ReleaseBlockedCompletion();
            await executor.StopAsync(CancellationToken.None);
        }
    }

    [Fact]
    public async Task FleetUpdateDockerFailureStillCleansUpAndFailsTheSession()
    {
        var sessionId = Guid.NewGuid();
        var docker = new RecordingDockerCli(
            new DockerCommandResult(1, string.Empty, "Docker daemon unavailable"),
            new DockerCommandResult(0, string.Empty, string.Empty),
            new DockerCommandResult(1, string.Empty, "network not found"));
        var publisher = new RecordingViewerPublisher();
        var hub = new RecordingWorkerCommandHub();
        var executor = CreateExecutor(docker, publisher, hub);
        var command = UpdateFleetCommand(sessionId);

        await executor.StartAsync(CancellationToken.None);
        try
        {
            Assert.True(await executor.EnqueueAsync(command, CancellationToken.None));

            var failure = await hub.WaitForFailureAsync();

            Assert.Equal(command.Id, failure.CommandId);
            Assert.Contains("Docker", failure.Error);
            Assert.Equal(new[] { command.Id }, hub.AcknowledgedCommands);
            Assert.Equal(new[] { command.Id }, hub.RunningCommands);
            Assert.Empty(hub.CompletedCommands);
            var sessionEvent = Assert.Single(hub.SessionEvents);
            Assert.Equal(sessionId, sessionEvent.SessionId);
            Assert.Equal("Failed", sessionEvent.State);
            Assert.Equal(3, docker.Calls.Count);
            Assert.Equal(new[] { sessionId }, publisher.StoppedSessions);
        }
        finally
        {
            await executor.StopAsync(CancellationToken.None);
        }
    }

    [Fact]
    public async Task BackendDisconnectFailSafeLatchesCommandsBeforeRosFallback()
    {
        var workerId = Guid.NewGuid();
        var sessionId = Guid.NewGuid();
        var options = Options.Create(new WorkerOptions
        {
            WorkerId = workerId,
            SessionImage = "robotswarm/ros-noetic:test",
            MaxQueuedCommands = 4,
            MaxParallelCommands = 1,
            ShutdownDrainSeconds = 5,
            BackendDisconnectEmergencyStopSeconds = 10
        });
        var docker = new RecordingDockerCli(
            new DockerCommandResult(0, "container-1\n", string.Empty),
            new DockerCommandResult(
                0,
                SessionInspection(workerId, sessionId),
                string.Empty),
            new DockerCommandResult(1, string.Empty, "ROS is unavailable"),
            new DockerCommandResult(0, string.Empty, string.Empty));
        var publisher = new RecordingViewerPublisher();
        var sessions = new DockerSessionManager(
            docker,
            publisher,
            options,
            NullLogger<DockerSessionManager>.Instance);
        var hub = new RecordingWorkerCommandHub(
            DateTime.UtcNow.AddMinutes(-1),
            isConnected: false);
        var executor = new BoundedCommandExecutor(
            new SessionCommandHandler(sessions, options),
            hub,
            new TaskStatusTracker(),
            options,
            NullLogger<BoundedCommandExecutor>.Instance);
        var monitor = new BackendDisconnectSafetyMonitor(
            sessions,
            hub,
            executor,
            options,
            NullLogger<BackendDisconnectSafetyMonitor>.Instance);
        var command = UpdateFleetCommand(sessionId);

        await executor.StartAsync(CancellationToken.None);
        await monitor.StartAsync(CancellationToken.None);
        try
        {
            await WaitUntilAsync(() => docker.Calls.Count >= 4);
            Assert.True(await executor.EnqueueAsync(command, CancellationToken.None));

            var failure = await hub.WaitForFailureAsync();

            Assert.Equal(command.Id, failure.CommandId);
            Assert.Contains("local emergency stop", failure.Error);
            Assert.Empty(hub.AcknowledgedCommands);
            Assert.Empty(hub.RunningCommands);
            Assert.Empty(hub.SessionEvents);
            Assert.Equal(4, docker.Calls.Count);
        }
        finally
        {
            using var stop = new CancellationTokenSource(TimeSpan.FromSeconds(5));
            await monitor.StopAsync(stop.Token);
            await executor.StopAsync(stop.Token);
        }
    }

    [Fact]
    public async Task BackendDisconnectFailSafeRetriesLatchedSessionAfterContactRecovers()
    {
        var workerId = Guid.NewGuid();
        var sessionId = Guid.NewGuid();
        var options = Options.Create(new WorkerOptions
        {
            WorkerId = workerId,
            SessionImage = "robotswarm/ros-noetic:test",
            MaxQueuedCommands = 4,
            MaxParallelCommands = 1,
            ShutdownDrainSeconds = 5,
            BackendDisconnectEmergencyStopSeconds = 10
        });
        var confirmedEmergencyStop = JsonSerializer.Serialize(new
        {
            emergency_stop = true,
            task = new { status = "idle" }
        });
        var docker = new RecordingDockerCli(
            new DockerCommandResult(0, "container-1\n", string.Empty),
            new DockerCommandResult(
                0,
                SessionInspection(workerId, sessionId),
                string.Empty),
            new DockerCommandResult(1, string.Empty, "ROS is unavailable"),
            new DockerCommandResult(1, string.Empty, "Docker stop failed"),
            new DockerCommandResult(0, "container-1\n", string.Empty),
            new DockerCommandResult(
                0,
                SessionInspection(workerId, sessionId),
                string.Empty),
            new DockerCommandResult(0, string.Empty, string.Empty),
            new DockerCommandResult(
                0,
                $"data: {JsonSerializer.Serialize(confirmedEmergencyStop)}\n---\n",
                string.Empty));
        var sessions = new DockerSessionManager(
            docker,
            new RecordingViewerPublisher(),
            options,
            NullLogger<DockerSessionManager>.Instance);
        var hub = new RecordingWorkerCommandHub(
            DateTime.UtcNow.AddMinutes(-1),
            isConnected: false);
        var executor = new BoundedCommandExecutor(
            new SessionCommandHandler(sessions, options),
            hub,
            new TaskStatusTracker(),
            options,
            NullLogger<BoundedCommandExecutor>.Instance);
        var monitor = new BackendDisconnectSafetyMonitor(
            sessions,
            hub,
            executor,
            options,
            NullLogger<BackendDisconnectSafetyMonitor>.Instance);

        await executor.StartAsync(CancellationToken.None);
        await monitor.StartAsync(CancellationToken.None);
        try
        {
            await WaitUntilAsync(() => docker.Calls.Count >= 4);
            hub.MarkSuccessfulContact();

            var report = await hub.WaitForEmergencyStopReportAsync();

            Assert.Equal(sessionId, report.SessionId);
            Assert.True(report.Active);
            Assert.Equal(8, docker.Calls.Count);
        }
        finally
        {
            using var stop = new CancellationTokenSource(TimeSpan.FromSeconds(5));
            await monitor.StopAsync(stop.Token);
            await executor.StopAsync(stop.Token);
        }
    }

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public async Task BackendDisconnectFailSafeReportsLatchedContainerThatIsNoLongerRunning(
        bool disappeared)
    {
        var workerId = Guid.NewGuid();
        var sessionId = Guid.NewGuid();
        var options = Options.Create(new WorkerOptions
        {
            WorkerId = workerId,
            SessionImage = "robotswarm/ros-noetic:test",
            MaxQueuedCommands = 4,
            MaxParallelCommands = 1,
            ShutdownDrainSeconds = 5,
            BackendDisconnectEmergencyStopSeconds = 10
        });
        var dockerResults = new List<DockerCommandResult>
        {
            new(0, "container-1\n", string.Empty),
            new(0, SessionInspection(workerId, sessionId), string.Empty),
            new(1, string.Empty, "ROS is unavailable"),
            new(1, string.Empty, "Docker stop failed"),
            new(0, disappeared ? string.Empty : "container-1\n", string.Empty)
        };
        if (!disappeared)
        {
            dockerResults.Add(new DockerCommandResult(
                0,
                SessionInspection(workerId, sessionId, running: false),
                string.Empty));
        }

        var docker = new RecordingDockerCli(dockerResults.ToArray());
        var sessions = new DockerSessionManager(
            docker,
            new RecordingViewerPublisher(),
            options,
            NullLogger<DockerSessionManager>.Instance);
        var hub = new RecordingWorkerCommandHub(
            DateTime.UtcNow.AddMinutes(-1),
            isConnected: false);
        var executor = new BoundedCommandExecutor(
            new SessionCommandHandler(sessions, options),
            hub,
            new TaskStatusTracker(),
            options,
            NullLogger<BoundedCommandExecutor>.Instance);
        var monitor = new BackendDisconnectSafetyMonitor(
            sessions,
            hub,
            executor,
            options,
            NullLogger<BackendDisconnectSafetyMonitor>.Instance);

        await executor.StartAsync(CancellationToken.None);
        await monitor.StartAsync(CancellationToken.None);
        try
        {
            await WaitUntilAsync(() => docker.Calls.Count >= 4);
            hub.MarkSuccessfulContact();
            await WaitUntilAsync(() => hub.SessionEvents.Count == 1);

            var report = Assert.Single(hub.SessionEvents);
            Assert.Equal(sessionId, report.SessionId);
            Assert.Equal("Failed", report.State);
            Assert.True(report.Payload?.GetProperty("ContainerStopped").GetBoolean());
            Assert.Equal(disappeared ? 5 : 6, docker.Calls.Count);
        }
        finally
        {
            using var stop = new CancellationTokenSource(TimeSpan.FromSeconds(5));
            await monitor.StopAsync(stop.Token);
            await executor.StopAsync(stop.Token);
        }
    }

    private static BoundedCommandExecutor CreateExecutor(
        IDockerCli docker,
        IViewerPublisher publisher,
        IWorkerCommandHub hub)
    {
        var options = Options.Create(new WorkerOptions
        {
            WorkerId = Guid.NewGuid(),
            SessionImage = "robotswarm/ros-noetic:test",
            MaxQueuedCommands = 4,
            MaxParallelCommands = 1,
            ShutdownDrainSeconds = 5
        });
        var sessions = new DockerSessionManager(
            docker,
            publisher,
            options,
            NullLogger<DockerSessionManager>.Instance);

        return new BoundedCommandExecutor(
            new SessionCommandHandler(sessions, options),
            hub,
            new TaskStatusTracker(),
            options,
            NullLogger<BoundedCommandExecutor>.Instance);
    }

    private static WorkerCommandEnvelope UpdateFleetCommand(Guid sessionId)
    {
        return new WorkerCommandEnvelope(
            Guid.NewGuid(),
            sessionId,
            "UpdateFleet",
            $"fleet-{Guid.NewGuid():N}",
            Guid.NewGuid(),
            1,
            JsonSerializer.SerializeToElement(new
            {
                desiredRobotCount = 3,
                spawnPattern = "grid"
            }),
            DateTime.UtcNow);
    }

    private static WorkerCommandEnvelope SessionControlCommand(
        Guid sessionId,
        string commandType)
    {
        return new WorkerCommandEnvelope(
            Guid.NewGuid(),
            sessionId,
            commandType,
            $"control-{Guid.NewGuid():N}",
            Guid.NewGuid(),
            1,
            JsonSerializer.SerializeToElement(new { }),
            DateTime.UtcNow);
    }

    private static string EmergencyStopStatus(bool active)
    {
        var json = JsonSerializer.Serialize(new
        {
            emergency_stop = active,
            task = new { status = "idle" }
        });
        return $"data: {JsonSerializer.Serialize(json)}\n---\n";
    }

    private static string SessionInspection(
        Guid workerId,
        Guid sessionId,
        bool running = true)
    {
        return JsonSerializer.Serialize(new[]
        {
            new
            {
                Id = "container-1",
                Name = $"/{SessionResourceNames.Container(sessionId)}",
                Config = new
                {
                    Image = "robotswarm/ros-noetic:test",
                    Labels = new Dictionary<string, string>
                    {
                        [SessionLabels.Managed] = "true",
                        [SessionLabels.WorkerId] = workerId.ToString("D"),
                        [SessionLabels.SessionId] = sessionId.ToString("D")
                    }
                },
                HostConfig = new
                {
                    PidsLimit = 1024
                },
                State = new
                {
                    Running = running,
                    Status = running ? "running" : "exited"
                }
            }
        });
    }

    private static async Task WaitUntilAsync(Func<bool> condition)
    {
        var deadline = DateTime.UtcNow.AddSeconds(5);
        while (!condition())
        {
            if (DateTime.UtcNow >= deadline)
            {
                throw new TimeoutException("The background safety monitor did not run.");
            }

            await Task.Delay(20);
        }
    }

    private sealed class RecordingDockerCli : IDockerCli
    {
        private readonly Queue<DockerCommandResult> _results;

        public RecordingDockerCli(params DockerCommandResult[] results)
        {
            _results = new Queue<DockerCommandResult>(results);
        }

        public ConcurrentQueue<IReadOnlyList<string>> Calls { get; } = new();

        public Task<DockerCommandResult> RunAsync(
            IReadOnlyList<string> arguments,
            CancellationToken cancellationToken,
            TimeSpan? timeout = null)
        {
            Calls.Enqueue(arguments.ToArray());
            if (_results.Count == 0)
            {
                throw new InvalidOperationException("The safety-rejected command reached Docker.");
            }

            return Task.FromResult(_results.Dequeue());
        }
    }

    private sealed class RecordingViewerPublisher : IViewerPublisher
    {
        public ViewerPublisherAvailability Availability { get; } =
            new(false, Array.Empty<ViewerSourceKind>(), null, "not needed");

        public List<Guid> StoppedSessions { get; } = new();

        public Task RefreshAvailabilityAsync(CancellationToken cancellationToken) =>
            Task.CompletedTask;

        public Task<ViewerPublishResult> PublishAsync(
            ViewerPublishRequest request,
            CancellationToken cancellationToken) =>
            throw new InvalidOperationException("The test does not publish a viewer stream.");

        public Task StopSessionAsync(
            Guid sessionId,
            CancellationToken cancellationToken)
        {
            StoppedSessions.Add(sessionId);
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
            CancellationToken cancellationToken) => Task.FromResult(false);
    }

    private sealed class RecordingWorkerCommandHub : IWorkerCommandHub
    {
        private readonly TaskCompletionSource<WorkerCommandFailureRequest> _failure =
            new(TaskCreationOptions.RunContinuationsAsynchronously);
        private readonly TaskCompletionSource<WorkerEmergencyStopReport> _emergencyStopReport =
            new(TaskCreationOptions.RunContinuationsAsynchronously);
        private readonly TaskCompletionSource _blockedCompletion =
            new(TaskCreationOptions.RunContinuationsAsynchronously);
        private readonly TaskCompletionSource _releaseCompletion =
            new(TaskCreationOptions.RunContinuationsAsynchronously);

        private long _lastSuccessfulContactTicks;
        private int _isConnected;
        private Guid? _blockedCompletionCommandId;

        public RecordingWorkerCommandHub(
            DateTime? lastSuccessfulContactUtc = null,
            bool isConnected = true)
        {
            _lastSuccessfulContactTicks =
                (lastSuccessfulContactUtc ?? DateTime.UtcNow).Ticks;
            _isConnected = isConnected ? 1 : 0;
        }

        public bool IsConnected => Volatile.Read(ref _isConnected) == 1;
        public DateTime LastSuccessfulContactUtc => new(
            Interlocked.Read(ref _lastSuccessfulContactTicks),
            DateTimeKind.Utc);
        public ConcurrentQueue<Guid> AcknowledgedCommands { get; } = new();
        public ConcurrentQueue<Guid> RunningCommands { get; } = new();
        public ConcurrentQueue<WorkerCommandCompletionRequest> CompletedCommands { get; } = new();
        public ConcurrentQueue<SessionEventReport> SessionEvents { get; } = new();

        public bool AcceptAcknowledgement { get; set; } = true;
        public bool AcceptRunningTransition { get; set; } = true;

        public Task AcknowledgeCommandAsync(
            Guid commandId,
            CancellationToken cancellationToken)
        {
            AcknowledgedCommands.Enqueue(commandId);
            return AcceptAcknowledgement
                ? Task.CompletedTask
                : Task.FromException(new InvalidOperationException(
                    "The backend rejected a stale command acknowledgement."));
        }

        public Task MarkCommandRunningAsync(
            Guid commandId,
            CancellationToken cancellationToken)
        {
            RunningCommands.Enqueue(commandId);
            return AcceptRunningTransition
                ? Task.CompletedTask
                : Task.FromException(new InvalidOperationException(
                    "The backend rejected a stale command execution."));
        }

        public async Task CompleteCommandAsync(
            WorkerCommandCompletionRequest request,
            CancellationToken cancellationToken)
        {
            if (_blockedCompletionCommandId == request.CommandId)
            {
                _blockedCompletion.TrySetResult();
                await _releaseCompletion.Task;
            }

            CompletedCommands.Enqueue(request);
        }

        public Task FailCommandAsync(
            WorkerCommandFailureRequest request,
            CancellationToken cancellationToken)
        {
            _failure.TrySetResult(request);
            return Task.CompletedTask;
        }

        public Task ReportEmergencyStopAsync(
            WorkerEmergencyStopReport report,
            CancellationToken cancellationToken)
        {
            _emergencyStopReport.TrySetResult(report);
            return Task.CompletedTask;
        }

        public Task ReportSessionEventAsync(
            SessionEventReport report,
            CancellationToken cancellationToken)
        {
            SessionEvents.Enqueue(report);
            return Task.CompletedTask;
        }

        public Task ReportTaskEventAsync(
            TaskEventReport report,
            CancellationToken cancellationToken) => Task.CompletedTask;

        public Task<WorkerCommandFailureRequest> WaitForFailureAsync()
        {
            return _failure.Task.WaitAsync(TimeSpan.FromSeconds(5));
        }

        public Task<WorkerEmergencyStopReport> WaitForEmergencyStopReportAsync()
        {
            return _emergencyStopReport.Task.WaitAsync(TimeSpan.FromSeconds(5));
        }

        public void MarkSuccessfulContact()
        {
            Interlocked.Exchange(
                ref _lastSuccessfulContactTicks,
                DateTime.UtcNow.Ticks);
            Volatile.Write(ref _isConnected, 1);
        }

        public void BlockCompletion(Guid commandId)
        {
            _blockedCompletionCommandId = commandId;
        }

        public Task WaitForBlockedCompletionAsync()
        {
            return _blockedCompletion.Task.WaitAsync(TimeSpan.FromSeconds(5));
        }

        public void ReleaseBlockedCompletion()
        {
            _releaseCompletion.TrySetResult();
        }
    }
}
