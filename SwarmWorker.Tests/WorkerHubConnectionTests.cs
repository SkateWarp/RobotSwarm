using Microsoft.Extensions.Logging.Abstractions;
using Microsoft.Extensions.Options;
using SwarmWorker.Configuration;
using SwarmWorker.Contracts;
using SwarmWorker.Runtime;
using SwarmWorker.Services;

namespace SwarmWorker.Tests;

public sealed class WorkerHubConnectionTests
{
    [Fact]
    public async Task DisposeCanBeCalledMoreThanOnce()
    {
        var publisher = new RecordingViewerPublisher();
        var connection = new WorkerHubConnection(
            Options.Create(new WorkerOptions
            {
                BackendUrl = "https://robot.example.test",
                WorkerId = Guid.Parse("11111111-1111-1111-1111-111111111111"),
                WorkerSecret = "abcdefghijklmnopqrstuvwxyzABCDEF_123456"
            }),
            publisher,
            NullLogger<WorkerHubConnection>.Instance);

        await connection.DisposeAsync();
        await connection.DisposeAsync();
    }

    [Fact]
    public async Task ViewerInputIsForwardedToThePublisherWithoutChangingItsLease()
    {
        var publisher = new RecordingViewerPublisher();
        await using var connection = new WorkerHubConnection(
            Options.Create(new WorkerOptions
            {
                BackendUrl = "https://robot.example.test",
                WorkerId = Guid.Parse("11111111-1111-1111-1111-111111111111"),
                WorkerSecret = "abcdefghijklmnopqrstuvwxyzABCDEF_123456"
            }),
            publisher,
            NullLogger<WorkerHubConnection>.Instance);
        var request = new ViewerInputEnvelope(
            Guid.NewGuid(),
            Guid.NewGuid(),
            new ViewerInputEvent("pointerMove", 0.25, 0.75));

        await connection.DispatchViewerInputAsync(request, CancellationToken.None);

        Assert.Same(request, Assert.Single(publisher.Inputs));
    }

    [Fact]
    public async Task StaleViewerInputIsDroppedWithoutFailingTheHubCallback()
    {
        var publisher = new RecordingViewerPublisher
        {
            InputFailure = new InvalidOperationException("lease was replaced")
        };
        await using var connection = new WorkerHubConnection(
            Options.Create(new WorkerOptions
            {
                BackendUrl = "https://robot.example.test",
                WorkerId = Guid.Parse("11111111-1111-1111-1111-111111111111"),
                WorkerSecret = "abcdefghijklmnopqrstuvwxyzABCDEF_123456"
            }),
            publisher,
            NullLogger<WorkerHubConnection>.Instance);

        await connection.DispatchViewerInputAsync(
            new ViewerInputEnvelope(
                Guid.NewGuid(),
                Guid.NewGuid(),
                new ViewerInputEvent("keyUp", Code: "KeyW")),
            CancellationToken.None);

        Assert.Single(publisher.Inputs);
    }

    [Fact]
    public async Task TrustedViewerReleaseIsForwardedWithItsExactLease()
    {
        var publisher = new RecordingViewerPublisher();
        await using var connection = new WorkerHubConnection(
            Options.Create(new WorkerOptions
            {
                BackendUrl = "https://robot.example.test",
                WorkerId = Guid.Parse("11111111-1111-1111-1111-111111111111"),
                WorkerSecret = "abcdefghijklmnopqrstuvwxyzABCDEF_123456"
            }),
            publisher,
            NullLogger<WorkerHubConnection>.Instance);
        var request = new ViewerInputReleaseEnvelope(Guid.NewGuid(), Guid.NewGuid());

        await connection.DispatchViewerInputReleaseAsync(
            request,
            CancellationToken.None);

        Assert.Equal((request.SessionId, request.LeaseId), Assert.Single(publisher.Releases));
    }

    [Fact]
    public async Task ViewerReleaseWithEmptyIdentifiersIsDropped()
    {
        var publisher = new RecordingViewerPublisher();
        await using var connection = new WorkerHubConnection(
            Options.Create(new WorkerOptions
            {
                BackendUrl = "https://robot.example.test",
                WorkerId = Guid.Parse("11111111-1111-1111-1111-111111111111"),
                WorkerSecret = "abcdefghijklmnopqrstuvwxyzABCDEF_123456"
            }),
            publisher,
            NullLogger<WorkerHubConnection>.Instance);

        await connection.DispatchViewerInputReleaseAsync(
            new ViewerInputReleaseEnvelope(Guid.Empty, Guid.NewGuid()),
            CancellationToken.None);

        Assert.Empty(publisher.Releases);
    }

    [Fact]
    public async Task HubOutageReleasesAllViewerInputsOnlyOnce()
    {
        var publisher = new RecordingViewerPublisher();
        await using var connection = new WorkerHubConnection(
            Options.Create(new WorkerOptions
            {
                BackendUrl = "https://robot.example.test",
                WorkerId = Guid.Parse("11111111-1111-1111-1111-111111111111"),
                WorkerSecret = "abcdefghijklmnopqrstuvwxyzABCDEF_123456"
            }),
            publisher,
            NullLogger<WorkerHubConnection>.Instance);

        await Task.WhenAll(
            connection.ReleaseViewerInputsForOutageAsync(),
            connection.ReleaseViewerInputsForOutageAsync());

        Assert.Equal(1, publisher.ReleaseAllCount);
    }

    [Fact]
    public async Task ReconnectingAndClosedCoalesceWhileReleaseIsInFlight()
    {
        var release = new TaskCompletionSource<bool>(
            TaskCreationOptions.RunContinuationsAsynchronously);
        var releaseStarted = new TaskCompletionSource<bool>(
            TaskCreationOptions.RunContinuationsAsynchronously);
        var publisher = new RecordingViewerPublisher();
        publisher.ReleaseAllHandler = (_, _) =>
        {
            releaseStarted.TrySetResult(true);
            return release.Task;
        };
        await using var connection = new WorkerHubConnection(
            Options.Create(new WorkerOptions
            {
                BackendUrl = "https://robot.example.test",
                WorkerId = Guid.Parse("11111111-1111-1111-1111-111111111111"),
                WorkerSecret = "abcdefghijklmnopqrstuvwxyzABCDEF_123456"
            }),
            publisher,
            NullLogger<WorkerHubConnection>.Instance);

        var reconnecting = connection.ReleaseViewerInputsForOutageAsync();
        await releaseStarted.Task.WaitAsync(TimeSpan.FromSeconds(1));
        await connection.ReleaseViewerInputsForOutageAsync();
        Assert.Equal(1, publisher.ReleaseAllCount);

        release.TrySetResult(true);
        await reconnecting;

        Assert.Equal(1, publisher.ReleaseAllCount);
    }

    [Fact]
    public async Task ClosedCanRetryAfterReconnectingReleaseFailsImmediately()
    {
        var publisher = new RecordingViewerPublisher
        {
            ReleaseAllHandler = (attempt, _) => attempt == 1
                ? Task.FromException(new IOException("publisher input pipe failed"))
                : Task.CompletedTask
        };
        await using var connection = new WorkerHubConnection(
            Options.Create(new WorkerOptions
            {
                BackendUrl = "https://robot.example.test",
                WorkerId = Guid.Parse("11111111-1111-1111-1111-111111111111"),
                WorkerSecret = "abcdefghijklmnopqrstuvwxyzABCDEF_123456"
            }),
            publisher,
            NullLogger<WorkerHubConnection>.Instance);

        await connection.ReleaseViewerInputsForOutageAsync();
        await connection.ReleaseViewerInputsForOutageAsync();

        Assert.Equal(2, publisher.ReleaseAllCount);
    }

    [Fact]
    public async Task ClosedQueuesOneRetryWhenReconnectingReleaseTimesOut()
    {
        var firstRelease = new TaskCompletionSource<bool>(
            TaskCreationOptions.RunContinuationsAsynchronously);
        var retryStarted = new TaskCompletionSource<bool>(
            TaskCreationOptions.RunContinuationsAsynchronously);
        var publisher = new RecordingViewerPublisher
        {
            ReleaseAllHandler = (attempt, _) =>
            {
                if (attempt == 1)
                {
                    return firstRelease.Task;
                }

                retryStarted.TrySetResult(true);
                return Task.CompletedTask;
            }
        };
        await using var connection = new WorkerHubConnection(
            Options.Create(new WorkerOptions
            {
                BackendUrl = "https://robot.example.test",
                WorkerId = Guid.Parse("11111111-1111-1111-1111-111111111111"),
                WorkerSecret = "abcdefghijklmnopqrstuvwxyzABCDEF_123456"
            }),
            publisher,
            NullLogger<WorkerHubConnection>.Instance);

        await connection.ReleaseViewerInputsForOutageAsync(
            TimeSpan.FromMilliseconds(25));
        await connection.ReleaseViewerInputsForOutageAsync(
            TimeSpan.FromMilliseconds(25));
        Assert.Equal(1, publisher.ReleaseAllCount);

        firstRelease.TrySetException(new IOException("late fail-closed failure"));
        await retryStarted.Task.WaitAsync(TimeSpan.FromSeconds(1));

        Assert.Equal(2, publisher.ReleaseAllCount);
    }

    private sealed class RecordingViewerPublisher : IViewerPublisher
    {
        public ViewerPublisherAvailability Availability { get; } =
            ViewerPublisherAvailability.Unavailable("not used");
        public List<ViewerInputEnvelope> Inputs { get; } = new();
        public List<(Guid SessionId, Guid LeaseId)> Releases { get; } = new();
        public Exception? InputFailure { get; init; }
        private int _releaseAllCount;

        public Func<int, CancellationToken, Task>? ReleaseAllHandler { get; set; }
        public int ReleaseAllCount => Volatile.Read(ref _releaseAllCount);

        public Task RefreshAvailabilityAsync(CancellationToken cancellationToken) =>
            Task.CompletedTask;

        public Task<ViewerPublishResult> PublishAsync(
            ViewerPublishRequest request,
            CancellationToken cancellationToken) =>
            throw new InvalidOperationException("The test does not publish.");

        public Task SendInputAsync(
            ViewerInputEnvelope request,
            CancellationToken cancellationToken)
        {
            Inputs.Add(request);
            return InputFailure is null
                ? Task.CompletedTask
                : Task.FromException(InputFailure);
        }

        public Task ReleaseInputAsync(
            Guid sessionId,
            Guid leaseId,
            CancellationToken cancellationToken)
        {
            Releases.Add((sessionId, leaseId));
            return InputFailure is null
                ? Task.CompletedTask
                : Task.FromException(InputFailure);
        }

        public Task ReleaseAllInputsAsync(CancellationToken cancellationToken)
        {
            var attempt = Interlocked.Increment(ref _releaseAllCount);
            return ReleaseAllHandler?.Invoke(attempt, cancellationToken)
                ?? Task.CompletedTask;
        }

        public Task StopSessionAsync(
            Guid sessionId,
            CancellationToken cancellationToken) => Task.CompletedTask;

        public Task<bool> StopLeaseAsync(
            Guid sessionId,
            Guid leaseId,
            CancellationToken cancellationToken) => Task.FromResult(false);
    }
}
