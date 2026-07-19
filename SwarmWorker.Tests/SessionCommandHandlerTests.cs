using System.Text.Json;
using Microsoft.Extensions.Logging.Abstractions;
using Microsoft.Extensions.Options;
using SwarmWorker.Configuration;
using SwarmWorker.Contracts;
using SwarmWorker.Infrastructure;
using SwarmWorker.Runtime;
using SwarmWorker.Services;

namespace SwarmWorker.Tests;

public sealed class SessionCommandHandlerTests
{
    private const string PublishToken =
        "AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA";

    [Fact]
    public async Task SetViewerSourceReachesPublisherAndReturnsReadyResult()
    {
        var workerId = Guid.NewGuid();
        var sessionId = Guid.NewGuid();
        var options = Options.Create(new WorkerOptions
        {
            WorkerId = workerId,
            Viewer = new ViewerPublisherOptions { MaximumLeaseMinutes = 30 }
        });
        var docker = new RecordingDockerCli(
            new DockerCommandResult(0, "container-1\n", string.Empty),
            new DockerCommandResult(
                0,
                Inspection(workerId, sessionId),
                string.Empty));
        var publisher = new RecordingViewerPublisher();
        var manager = new DockerSessionManager(
            docker,
            publisher,
            options,
            NullLogger<DockerSessionManager>.Instance);
        var handler = new SessionCommandHandler(manager, options);

        var execution = await handler.ExecuteAsync(
            Command(sessionId, Payload(sessionId)),
            CancellationToken.None);

        Assert.True(execution.Result.GetProperty("Ready").GetBoolean());
        Assert.Equal("Scene", execution.Result.GetProperty("source").GetString());
        Assert.Single(publisher.Requests);
        Assert.Equal(sessionId, publisher.Requests[0].SessionId);
    }

    [Fact]
    public async Task SetViewerSourceRejectsAnotherSessionsPathBeforeDockerAccess()
    {
        var assignedSession = Guid.NewGuid();
        var otherSession = Guid.NewGuid();
        var options = Options.Create(new WorkerOptions());
        var docker = new RecordingDockerCli();
        var publisher = new RecordingViewerPublisher();
        var handler = new SessionCommandHandler(
            new DockerSessionManager(
                docker,
                publisher,
                options,
                NullLogger<DockerSessionManager>.Instance),
            options);

        await Assert.ThrowsAsync<InvalidOperationException>(
            () => handler.ExecuteAsync(
                Command(assignedSession, Payload(otherSession)),
                CancellationToken.None));

        Assert.Empty(docker.Calls);
        Assert.Empty(publisher.Requests);
    }

    private static WorkerCommandEnvelope Command(Guid sessionId, JsonElement payload)
    {
        return new WorkerCommandEnvelope(
            Guid.NewGuid(),
            sessionId,
            "SetViewerSource",
            "viewer-test",
            Guid.NewGuid(),
            1,
            payload,
            DateTime.UtcNow);
    }

    private static JsonElement Payload(Guid sessionId)
    {
        var sourceId = $"scene-{sessionId:N}";
        return JsonSerializer.SerializeToElement(new
        {
            leaseId = Guid.NewGuid(),
            expiresAt = DateTimeOffset.UtcNow.AddMinutes(5),
            publishToken = PublishToken,
            source = "Scene",
            robotRuntimeId = (string?)null,
            sourceId,
            streamPath = $"session/{sessionId:N}/{sourceId}"
        });
    }

    private static string Inspection(Guid workerId, Guid sessionId)
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
                        [SessionLabels.WorkerId] = workerId.ToString("D"),
                        [SessionLabels.SessionId] = sessionId.ToString("D")
                    }
                },
                HostConfig = new { PidsLimit = 1024 },
                State = new { Running = true, Status = "running" }
            }
        });
    }

    private sealed class RecordingDockerCli : IDockerCli
    {
        private readonly Queue<DockerCommandResult> _results;

        public RecordingDockerCli(params DockerCommandResult[] results)
        {
            _results = new Queue<DockerCommandResult>(results);
        }

        public List<IReadOnlyList<string>> Calls { get; } = new();

        public Task<DockerCommandResult> RunAsync(
            IReadOnlyList<string> arguments,
            CancellationToken cancellationToken,
            TimeSpan? timeout = null)
        {
            Calls.Add(arguments);
            return Task.FromResult(_results.Dequeue());
        }
    }

    private sealed class RecordingViewerPublisher : IViewerPublisher
    {
        public ViewerPublisherAvailability Availability { get; } =
            new(
                true,
                new[] { ViewerSourceKind.Scene },
                "H264",
                "ready");

        public List<ViewerPublishRequest> Requests { get; } = new();

        public Task RefreshAvailabilityAsync(CancellationToken cancellationToken) =>
            Task.CompletedTask;

        public Task<ViewerPublishResult> PublishAsync(
            ViewerPublishRequest request,
            CancellationToken cancellationToken)
        {
            Requests.Add(request);
            return Task.FromResult(new ViewerPublishResult(
                request.SessionId,
                request.Command.LeaseId,
                request.Command.ExpiresAt,
                request.Command.Source,
                request.Command.RobotRuntimeId,
                request.Command.SourceId,
                request.Command.StreamPath,
                Ready: true,
                VideoCodec: "H264"));
        }

        public Task StopSessionAsync(
            Guid sessionId,
            CancellationToken cancellationToken) => Task.CompletedTask;

        public Task SendInputAsync(
            ViewerInputEnvelope request,
            CancellationToken cancellationToken) => Task.CompletedTask;

        public Task ReleaseInputAsync(
            Guid sessionId,
            Guid leaseId,
            CancellationToken cancellationToken) => Task.CompletedTask;

        public Task ReleaseAllInputsAsync(CancellationToken cancellationToken) =>
            Task.CompletedTask;
    }
}
