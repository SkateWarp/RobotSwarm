using System.Text.Json;
using Microsoft.AspNetCore.SignalR;
using Microsoft.Extensions.Logging.Abstractions;
using SwarmBackend.Entities;
using SwarmBackend.Helpers;
using SwarmBackend.Models;
using SwarmBackend.Services;

namespace SwarmBackend.Tests;

public sealed class ViewerInputControlTests
{
    [Theory]
    [InlineData("{\"type\":\"pointerMove\",\"x\":0,\"y\":1}", "pointerMove")]
    [InlineData("{\"type\":\"pointerDown\",\"x\":0.5,\"y\":0.25,\"button\":0}", "pointerDown")]
    [InlineData("{\"type\":\"pointerUp\",\"x\":1,\"y\":0,\"button\":2}", "pointerUp")]
    [InlineData("{\"type\":\"wheel\",\"x\":0.5,\"y\":0.5,\"deltaX\":-25,\"deltaY\":100}", "wheel")]
    [InlineData("{\"type\":\"keyDown\",\"code\":\"KeyW\"}", "keyDown")]
    [InlineData("{\"type\":\"keyUp\",\"code\":\"Escape\"}", "keyUp")]
    [InlineData("{\"type\":\"releaseAll\"}", "releaseAll")]
    public void NormalizerAcceptsOnlyCanonicalViewerEvents(string json, string type)
    {
        using var document = JsonDocument.Parse(json);

        var valid = ViewerInputNormalizer.TryNormalize(
            document.RootElement,
            out var input,
            out var error);

        Assert.True(valid, error);
        Assert.NotNull(input);
        Assert.Equal(type, input.Type);
    }

    [Fact]
    public void ReleaseSerializesWithoutOptionalInputFields()
    {
        var serialized = JsonSerializer.Serialize(
            new ViewerInputEvent("releaseAll"),
            new JsonSerializerOptions { PropertyNamingPolicy = JsonNamingPolicy.CamelCase });

        Assert.Equal("{\"type\":\"releaseAll\"}", serialized);
    }

    [Theory]
    [InlineData("null")]
    [InlineData("{\"type\":\"pointerMove\",\"x\":-0.1,\"y\":0}")]
    [InlineData("{\"type\":\"pointerMove\",\"x\":0,\"y\":1.1}")]
    [InlineData("{\"type\":\"pointerDown\",\"x\":0,\"y\":0,\"button\":3}")]
    [InlineData("{\"type\":\"wheel\",\"x\":0,\"y\":0,\"deltaX\":0,\"deltaY\":1001}")]
    [InlineData("{\"type\":\"keyDown\",\"code\":\"Untrusted arbitrary key\"}")]
    [InlineData("{\"type\":\"keyUp\",\"code\":\"KeyW\",\"x\":0}")]
    [InlineData("{\"type\":\"PointerMove\",\"x\":0,\"y\":0}")]
    [InlineData("{\"type\":\"pointerMove\",\"x\":0,\"y\":0,\"extra\":true}")]
    [InlineData("{\"type\":\"pointerMove\",\"type\":\"pointerUp\",\"x\":0,\"y\":0}")]
    [InlineData("{\"type\":\"releaseAll\",\"code\":\"KeyW\"}")]
    [InlineData("{\"type\":\"releaseAll\",\"extra\":true}")]
    public void NormalizerRejectsMalformedOrExpandedEvents(string json)
    {
        using var document = JsonDocument.Parse(json);

        Assert.False(ViewerInputNormalizer.TryNormalize(
            document.RootElement,
            out _,
            out var error));
        Assert.NotEmpty(error);
    }

    [Fact]
    public async Task LeaseHasOneControllerUntilItsReleaseIsEnqueued()
    {
        var workerContext = new RecordingWorkerHubContext();
        var registry = Registry(workerContext);
        var sessionId = Guid.NewGuid();
        var leaseId = Guid.NewGuid();
        var workerId = Guid.NewGuid();
        var now = Time("2026-07-19T12:00:00Z");

        var first = await Authorize(
            registry,
            "connection-a",
            sessionId,
            leaseId,
            workerId,
            now);
        var occupied = await Authorize(
            registry,
            "connection-b",
            sessionId,
            leaseId,
            workerId,
            now);

        Assert.Equal(ViewerControlAuthorizationStatus.Authorized, first.Status);
        Assert.Equal(ViewerControlAuthorizationStatus.Occupied, occupied.Status);
        Assert.Equal(
            ViewerControlCheck.Missing,
            await registry.DispatchAsync(
                "connection-b",
                sessionId,
                leaseId,
                PointerMove(),
                now,
                CancellationToken.None));

        Assert.Equal(
            ViewerControlDrainStatus.Removed,
            await registry.ReleaseConnectionAsync(
                "connection-a",
                CancellationToken.None));
        Assert.Equal("ViewerInputRelease", Assert.Single(workerContext.Messages).Method);

        var second = await Authorize(
            registry,
            "connection-b",
            sessionId,
            leaseId,
            workerId,
            now);
        Assert.Equal(ViewerControlAuthorizationStatus.Authorized, second.Status);
    }

    [Fact]
    public async Task LeaseRateWindowSurvivesImmediateControllerReconnect()
    {
        var workerContext = new RecordingWorkerHubContext();
        var registry = Registry(workerContext);
        var sessionId = Guid.NewGuid();
        var leaseId = Guid.NewGuid();
        var workerId = Guid.NewGuid();
        var now = Time("2026-07-19T12:00:00Z");
        await Authorize(registry, "connection-a", sessionId, leaseId, workerId, now);

        for (var index = 0;
             index < ViewerControlRegistry.MaximumEventsPerSecond;
             index++)
        {
            Assert.Equal(
                ViewerControlCheck.Authorized,
                await registry.DispatchAsync(
                    "connection-a",
                    sessionId,
                    leaseId,
                    PointerMove(),
                    now,
                    CancellationToken.None));
        }

        await registry.ReleaseConnectionAsync("connection-a", CancellationToken.None);
        var replacement = await Authorize(
            registry,
            "connection-b",
            sessionId,
            leaseId,
            workerId,
            now);

        Assert.Equal(ViewerControlAuthorizationStatus.Authorized, replacement.Status);
        Assert.Equal(
            ViewerControlCheck.RateLimited,
            await registry.DispatchAsync(
                "connection-b",
                sessionId,
                leaseId,
                PointerMove(),
                now,
                CancellationToken.None));
        Assert.Equal(
            ViewerControlCheck.Authorized,
            await registry.DispatchAsync(
                "connection-b",
                sessionId,
                leaseId,
                PointerMove(),
                now.AddSeconds(1),
                CancellationToken.None));
    }

    [Fact]
    public async Task ReleaseIsDeduplicatedAndStillPassesAfterTheInputLimit()
    {
        var workerContext = new RecordingWorkerHubContext();
        var registry = Registry(workerContext);
        var sessionId = Guid.NewGuid();
        var leaseId = Guid.NewGuid();
        var workerId = Guid.NewGuid();
        var now = Time("2026-07-19T12:00:00Z");
        await Authorize(registry, "connection-a", sessionId, leaseId, workerId, now);

        for (var index = 0;
             index < ViewerControlRegistry.MaximumEventsPerSecond;
             index++)
        {
            Assert.Equal(
                ViewerControlCheck.Authorized,
                await registry.DispatchAsync(
                    "connection-a",
                    sessionId,
                    leaseId,
                    PointerMove(),
                    now,
                    CancellationToken.None));
        }

        Assert.Equal(
            ViewerControlCheck.RateLimited,
            await registry.DispatchAsync(
                "connection-a",
                sessionId,
                leaseId,
                PointerMove(),
                now,
                CancellationToken.None));
        Assert.Equal(
            ViewerControlCheck.Authorized,
            await registry.DispatchAsync(
                "connection-a",
                sessionId,
                leaseId,
                new ViewerInputEvent("releaseAll"),
                now,
                CancellationToken.None));
        Assert.Equal(
            ViewerControlCheck.Authorized,
            await registry.DispatchAsync(
                "connection-a",
                sessionId,
                leaseId,
                new ViewerInputEvent("releaseAll"),
                now,
                CancellationToken.None));

        var clientReleases = workerContext.Messages.Count(message =>
            message.Method == "ViewerInput"
            && Assert.IsType<ViewerInputEnvelope>(Assert.Single(message.Arguments)).Input.Type
                == "releaseAll");
        Assert.Equal(1, clientReleases);
    }

    [Fact]
    public async Task FailedOverflowReleaseCanBeRetried()
    {
        var releaseAttempts = 0;
        var workerContext = new RecordingWorkerHubContext
        {
            OnSend = (method, arguments, _) =>
            {
                if (method != "ViewerInput"
                    || Assert.IsType<ViewerInputEnvelope>(Assert.Single(arguments)).Input.Type
                        != "releaseAll")
                {
                    return Task.CompletedTask;
                }

                releaseAttempts++;
                return releaseAttempts == 1
                    ? Task.FromException(new IOException("first release send failed"))
                    : Task.CompletedTask;
            }
        };
        var registry = Registry(workerContext);
        var sessionId = Guid.NewGuid();
        var leaseId = Guid.NewGuid();
        var workerId = Guid.NewGuid();
        var now = Time("2026-07-19T12:00:00Z");
        await Authorize(registry, "connection-a", sessionId, leaseId, workerId, now);

        for (var index = 0;
             index < ViewerControlRegistry.MaximumEventsPerSecond;
             index++)
        {
            Assert.Equal(
                ViewerControlCheck.Authorized,
                await registry.DispatchAsync(
                    "connection-a",
                    sessionId,
                    leaseId,
                    PointerMove(),
                    now,
                    CancellationToken.None));
        }

        await Assert.ThrowsAsync<IOException>(() => registry.DispatchAsync(
            "connection-a",
            sessionId,
            leaseId,
            new ViewerInputEvent("releaseAll"),
            now,
            CancellationToken.None));
        Assert.Equal(
            ViewerControlCheck.Authorized,
            await registry.DispatchAsync(
                "connection-a",
                sessionId,
                leaseId,
                new ViewerInputEvent("releaseAll"),
                now,
                CancellationToken.None));
        Assert.Equal(
            ViewerControlCheck.Authorized,
            await registry.DispatchAsync(
                "connection-a",
                sessionId,
                leaseId,
                new ViewerInputEvent("releaseAll"),
                now,
                CancellationToken.None));

        Assert.Equal(2, releaseAttempts);
    }

    [Fact]
    public async Task AlternatingInputAndReleaseCannotExceedOneFailSafeOverflow()
    {
        var workerContext = new RecordingWorkerHubContext();
        var registry = Registry(workerContext);
        var sessionId = Guid.NewGuid();
        var leaseId = Guid.NewGuid();
        var workerId = Guid.NewGuid();
        var now = Time("2026-07-19T12:00:00Z");
        await Authorize(registry, "connection-a", sessionId, leaseId, workerId, now);

        for (var index = 0; index < 59; index++)
        {
            Assert.Equal(
                ViewerControlCheck.Authorized,
                await registry.DispatchAsync(
                    "connection-a",
                    sessionId,
                    leaseId,
                    PointerMove(),
                    now,
                    CancellationToken.None));
            Assert.Equal(
                ViewerControlCheck.Authorized,
                await registry.DispatchAsync(
                    "connection-a",
                    sessionId,
                    leaseId,
                    new ViewerInputEvent("releaseAll"),
                    now,
                    CancellationToken.None));
        }

        Assert.Equal(
            ViewerControlCheck.Authorized,
            await registry.DispatchAsync(
                "connection-a",
                sessionId,
                leaseId,
                PointerMove(),
                now,
                CancellationToken.None));
        Assert.Equal(
            ViewerControlCheck.Authorized,
            await registry.DispatchAsync(
                "connection-a",
                sessionId,
                leaseId,
                PointerMove(),
                now,
                CancellationToken.None));
        Assert.Equal(
            ViewerControlCheck.Authorized,
            await registry.DispatchAsync(
                "connection-a",
                sessionId,
                leaseId,
                new ViewerInputEvent("releaseAll"),
                now,
                CancellationToken.None));
        Assert.Equal(
            ViewerControlCheck.RateLimited,
            await registry.DispatchAsync(
                "connection-a",
                sessionId,
                leaseId,
                PointerMove(),
                now,
                CancellationToken.None));
        Assert.Equal(
            ViewerControlCheck.Authorized,
            await registry.DispatchAsync(
                "connection-a",
                sessionId,
                leaseId,
                new ViewerInputEvent("releaseAll"),
                now,
                CancellationToken.None));

        Assert.Equal(ViewerControlRegistry.MaximumEventsPerSecond + 1, workerContext.Messages.Count);
        Assert.Equal(
            ViewerControlCheck.Authorized,
            await registry.DispatchAsync(
                "connection-a",
                sessionId,
                leaseId,
                PointerMove(),
                now.AddSeconds(1),
                CancellationToken.None));
    }

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public async Task InputDispatchCompletesBeforeDisconnectOrRevocationRelease(bool revokeLease)
    {
        var inputStarted = new TaskCompletionSource<bool>(
            TaskCreationOptions.RunContinuationsAsynchronously);
        var allowInput = new TaskCompletionSource<bool>(
            TaskCreationOptions.RunContinuationsAsynchronously);
        var workerContext = new RecordingWorkerHubContext
        {
            OnSend = async (method, _, cancellationToken) =>
            {
                if (method != "ViewerInput")
                {
                    return;
                }

                inputStarted.TrySetResult(true);
                await allowInput.Task.WaitAsync(cancellationToken);
            }
        };
        var registry = Registry(workerContext);
        var sessionId = Guid.NewGuid();
        var leaseId = Guid.NewGuid();
        var workerId = Guid.NewGuid();
        var now = Time("2026-07-19T12:00:00Z");
        await Authorize(registry, "connection-a", sessionId, leaseId, workerId, now);

        var dispatch = registry.DispatchAsync(
            "connection-a",
            sessionId,
            leaseId,
            new ViewerInputEvent("keyDown", Code: "KeyW"),
            now,
            CancellationToken.None);
        await inputStarted.Task;
        var drain = revokeLease
            ? registry.DrainLeaseAsync(sessionId, leaseId, CancellationToken.None)
            : registry.ReleaseConnectionAsync("connection-a", CancellationToken.None);

        Assert.False(drain.IsCompleted);
        allowInput.TrySetResult(true);
        Assert.Equal(ViewerControlCheck.Authorized, await dispatch);
        Assert.Equal(ViewerControlDrainStatus.Removed, await drain);
        Assert.Equal(
            new[] { "ViewerInput", "ViewerInputRelease" },
            workerContext.Messages.Select(message => message.Method));
        Assert.Empty(registry.GetSnapshots());
    }

    [Fact]
    public async Task TimedOutDisconnectIsRetriedAndBlocksRenewal()
    {
        await using var context = TestDataContext.Create();
        var seeded = await SeedLease(context);
        var inputStarted = new TaskCompletionSource<bool>(
            TaskCreationOptions.RunContinuationsAsynchronously);
        var allowInput = new TaskCompletionSource<bool>(
            TaskCreationOptions.RunContinuationsAsynchronously);
        var workerContext = new RecordingWorkerHubContext
        {
            OnSend = async (method, _, _) =>
            {
                if (method == "ViewerInput")
                {
                    inputStarted.TrySetResult(true);
                    await allowInput.Task;
                }
            }
        };
        var registry = Registry(workerContext);
        var now = DateTimeOffset.UtcNow;
        await Authorize(
            registry,
            "connection-a",
            seeded.Owner.Id,
            seeded.Session.Id,
            seeded.Lease.Id,
            seeded.Worker.Id,
            now);

        var dispatch = registry.DispatchAsync(
            "connection-a",
            seeded.Session.Id,
            seeded.Lease.Id,
            new ViewerInputEvent("keyDown", Code: "KeyW"),
            now,
            CancellationToken.None);
        await inputStarted.Task;

        await SessionHub.ReleaseDisconnectedViewerControlAsync(
            "connection-a",
            registry,
            TimeSpan.FromMilliseconds(40));

        Assert.True(Assert.Single(registry.GetSnapshots()).ReleaseRequested);
        Assert.Equal(
            ViewerControlAuthorizationStatus.ReleaseFailed,
            (await Authorize(
                registry,
                "connection-a",
                seeded.Owner.Id,
                seeded.Session.Id,
                seeded.Lease.Id,
                seeded.Worker.Id,
                now.AddSeconds(1))).Status);

        allowInput.TrySetResult(true);
        Assert.Equal(ViewerControlCheck.Authorized, await dispatch);
        await ViewerControlReconciler.ReconcileOnceAsync(
            context,
            registry,
            now.AddSeconds(1),
            CancellationToken.None);

        Assert.Empty(registry.GetSnapshots());
        Assert.Equal(
            new[] { "ViewerInput", "ViewerInputRelease" },
            workerContext.Messages.Select(message => message.Method));
    }

    [Fact]
    public async Task OneSlowReleaseDoesNotBlockReconciliationOfAnotherLease()
    {
        var firstSessionId = Guid.NewGuid();
        var firstLeaseId = Guid.NewGuid();
        var secondSessionId = Guid.NewGuid();
        var secondLeaseId = Guid.NewGuid();
        var firstReleaseStarted = new TaskCompletionSource<bool>(
            TaskCreationOptions.RunContinuationsAsynchronously);
        var secondReleaseStarted = new TaskCompletionSource<bool>(
            TaskCreationOptions.RunContinuationsAsynchronously);
        var allowFirstRelease = new TaskCompletionSource<bool>(
            TaskCreationOptions.RunContinuationsAsynchronously);
        var workerContext = new RecordingWorkerHubContext
        {
            OnSend = async (method, arguments, _) =>
            {
                if (method != "ViewerInputRelease")
                {
                    return;
                }

                var release = Assert.IsType<ViewerInputReleaseEnvelope>(
                    Assert.Single(arguments));
                if (release.LeaseId == firstLeaseId)
                {
                    firstReleaseStarted.TrySetResult(true);
                    await allowFirstRelease.Task;
                }
                else if (release.LeaseId == secondLeaseId)
                {
                    secondReleaseStarted.TrySetResult(true);
                }
            }
        };
        var registry = Registry(workerContext);
        var workerId = Guid.NewGuid();
        var now = DateTimeOffset.UtcNow;
        await Authorize(
            registry,
            "connection-a",
            firstSessionId,
            firstLeaseId,
            workerId,
            now);
        await Authorize(
            registry,
            "connection-b",
            secondSessionId,
            secondLeaseId,
            workerId,
            now);
        await using var context = TestDataContext.Create();

        var reconciliation = ViewerControlReconciler.ReconcileOnceAsync(
            context,
            registry,
            now.AddSeconds(1),
            CancellationToken.None);
        await firstReleaseStarted.Task.WaitAsync(TimeSpan.FromSeconds(1));
        await secondReleaseStarted.Task.WaitAsync(TimeSpan.FromSeconds(1));
        allowFirstRelease.TrySetResult(true);
        await reconciliation;

        Assert.Empty(registry.GetSnapshots());
    }

    [Fact]
    public async Task ReconciliationBoundsParallelReleaseSends()
    {
        var sync = new object();
        var activeSends = 0;
        var maximumSends = 0;
        var firstBatchStarted = new TaskCompletionSource<bool>(
            TaskCreationOptions.RunContinuationsAsynchronously);
        var allowReleases = new TaskCompletionSource<bool>(
            TaskCreationOptions.RunContinuationsAsynchronously);
        var workerContext = new RecordingWorkerHubContext
        {
            OnSend = async (method, _, _) =>
            {
                if (method != "ViewerInputRelease")
                {
                    return;
                }

                lock (sync)
                {
                    activeSends++;
                    maximumSends = Math.Max(maximumSends, activeSends);
                    if (activeSends == ViewerControlReconciler.MaximumConcurrentDrains)
                    {
                        firstBatchStarted.TrySetResult(true);
                    }
                }

                await allowReleases.Task;
                lock (sync)
                {
                    activeSends--;
                }
            }
        };
        var registry = Registry(workerContext);
        var workerId = Guid.NewGuid();
        var now = DateTimeOffset.UtcNow;
        for (var index = 0; index < ViewerControlReconciler.MaximumConcurrentDrains * 2; index++)
        {
            await Authorize(
                registry,
                $"connection-{index}",
                Guid.NewGuid(),
                Guid.NewGuid(),
                workerId,
                now);
        }
        await using var context = TestDataContext.Create();

        var reconciliation = ViewerControlReconciler.ReconcileOnceAsync(
            context,
            registry,
            now.AddSeconds(1),
            CancellationToken.None);
        await firstBatchStarted.Task.WaitAsync(TimeSpan.FromSeconds(1));

        lock (sync)
        {
            Assert.Equal(ViewerControlReconciler.MaximumConcurrentDrains, maximumSends);
        }
        allowReleases.TrySetResult(true);
        await reconciliation;

        Assert.Empty(registry.GetSnapshots());
    }

    [Fact]
    public async Task WorkerReleaseSendHasAnInternalTimeout()
    {
        var workerContext = new RecordingWorkerHubContext
        {
            OnSend = (method, _, _) => method == "ViewerInputRelease"
                ? new TaskCompletionSource<bool>().Task
                : Task.CompletedTask
        };
        var registry = Registry(workerContext);
        var sessionId = Guid.NewGuid();
        var leaseId = Guid.NewGuid();
        var workerId = Guid.NewGuid();
        var now = DateTimeOffset.UtcNow;
        await Authorize(registry, "connection-a", sessionId, leaseId, workerId, now);

        var drain = await registry.ReleaseConnectionAsync(
                "connection-a",
                CancellationToken.None)
            .WaitAsync(ViewerControlRegistry.WorkerSendTimeout + TimeSpan.FromSeconds(1));

        Assert.Equal(ViewerControlDrainStatus.ReleaseFailed, drain);
        Assert.Single(registry.GetSnapshots());
    }

    [Fact]
    public async Task LeaseRevocationFenceBlocksInputAndCanBeRolledBack()
    {
        var workerContext = new RecordingWorkerHubContext();
        var registry = Registry(workerContext);
        var sessionId = Guid.NewGuid();
        var leaseId = Guid.NewGuid();
        var workerId = Guid.NewGuid();
        var now = DateTimeOffset.UtcNow;
        await Authorize(registry, "connection-a", sessionId, leaseId, workerId, now);

        var marker = await registry.BeginLeaseRevocationAsync(
            sessionId,
            leaseId,
            now,
            CancellationToken.None);

        Assert.True(Assert.Single(registry.GetSnapshots()).ReleaseRequested);
        Assert.Equal(
            ViewerControlCheck.Expired,
            await registry.DispatchAsync(
                "connection-a",
                sessionId,
                leaseId,
                PointerMove(),
                now,
                CancellationToken.None));
        Assert.Equal(
            ViewerControlAuthorizationStatus.ReleaseFailed,
            (await Authorize(
                registry,
                "connection-a",
                sessionId,
                leaseId,
                workerId,
                now.AddSeconds(1))).Status);
        Assert.Equal(
            ViewerControlCheck.Authorized,
            await registry.DispatchAsync(
                "connection-a",
                sessionId,
                leaseId,
                new ViewerInputEvent("releaseAll"),
                now.AddSeconds(1),
                CancellationToken.None));

        registry.CancelLeaseRevocation(sessionId, leaseId, marker);
        Assert.Equal(
            ViewerControlCheck.Authorized,
            await registry.DispatchAsync(
                "connection-a",
                sessionId,
                leaseId,
                PointerMove(),
                now.AddSeconds(1),
                CancellationToken.None));
    }

    [Fact]
    public async Task FailedClientReleaseIsRetriedByReconciliation()
    {
        await using var context = TestDataContext.Create();
        var seeded = await SeedLease(context);
        var failClientRelease = true;
        var workerContext = new RecordingWorkerHubContext
        {
            OnSend = (method, arguments, _) =>
            {
                if (method == "ViewerInput"
                    && Assert.IsType<ViewerInputEnvelope>(Assert.Single(arguments)).Input.Type
                        == "releaseAll"
                    && failClientRelease)
                {
                    failClientRelease = false;
                    return Task.FromException(new IOException("client release send failed"));
                }

                return Task.CompletedTask;
            }
        };
        var registry = Registry(workerContext);
        var now = DateTimeOffset.UtcNow;
        await Authorize(
            registry,
            "connection-a",
            seeded.Owner.Id,
            seeded.Session.Id,
            seeded.Lease.Id,
            seeded.Worker.Id,
            now);

        await Assert.ThrowsAsync<IOException>(() => registry.DispatchAsync(
            "connection-a",
            seeded.Session.Id,
            seeded.Lease.Id,
            new ViewerInputEvent("releaseAll"),
            now,
            CancellationToken.None));
        Assert.True(Assert.Single(registry.GetSnapshots()).ReleaseRequested);

        await ViewerControlReconciler.ReconcileOnceAsync(
            context,
            registry,
            now.AddMilliseconds(100),
            CancellationToken.None);

        Assert.Empty(registry.GetSnapshots());
        Assert.Contains(workerContext.Messages, message => message.Method == "ViewerInputRelease");
    }

    [Fact]
    public async Task LateInputIsFollowedByAnotherRelease()
    {
        var delivered = new List<string>();
        var allowLateInput = new TaskCompletionSource<bool>(
            TaskCreationOptions.RunContinuationsAsynchronously);
        var followUpRelease = new TaskCompletionSource<bool>(
            TaskCreationOptions.RunContinuationsAsynchronously);
        var releaseCount = 0;
        var workerContext = new RecordingWorkerHubContext
        {
            OnSend = async (method, _, _) =>
            {
                if (method == "ViewerInput")
                {
                    await allowLateInput.Task;
                    lock (delivered)
                    {
                        delivered.Add("input");
                    }
                    return;
                }

                if (method == "ViewerInputRelease")
                {
                    lock (delivered)
                    {
                        delivered.Add("release");
                    }
                    if (Interlocked.Increment(ref releaseCount) >= 2)
                    {
                        followUpRelease.TrySetResult(true);
                    }
                }
            }
        };
        var registry = Registry(workerContext);
        var sessionId = Guid.NewGuid();
        var leaseId = Guid.NewGuid();
        var workerId = Guid.NewGuid();
        var now = DateTimeOffset.UtcNow;
        await Authorize(registry, "connection-a", sessionId, leaseId, workerId, now);

        await Assert.ThrowsAsync<TimeoutException>(() => registry.DispatchAsync(
            "connection-a",
            sessionId,
            leaseId,
            new ViewerInputEvent("keyDown", Code: "KeyW"),
            now,
            CancellationToken.None));
        Assert.True(Assert.Single(registry.GetSnapshots()).ReleaseRequested);

        Assert.Equal(
            ViewerControlDrainStatus.Removed,
            await registry.ReleaseDisconnectedConnectionAsync(
                "connection-a",
                now.AddSeconds(1),
                CancellationToken.None));
        allowLateInput.TrySetResult(true);
        await followUpRelease.Task.WaitAsync(TimeSpan.FromSeconds(1));

        lock (delivered)
        {
            Assert.Equal(new[] { "release", "input", "release" }, delivered);
        }
    }

    [Fact]
    public async Task RevocationFenceWaitsForEarlierInputAndRejectsLaterInput()
    {
        var inputStarted = new TaskCompletionSource<bool>(
            TaskCreationOptions.RunContinuationsAsynchronously);
        var allowInput = new TaskCompletionSource<bool>(
            TaskCreationOptions.RunContinuationsAsynchronously);
        var workerContext = new RecordingWorkerHubContext
        {
            OnSend = async (method, _, _) =>
            {
                if (method == "ViewerInput")
                {
                    inputStarted.TrySetResult(true);
                    await allowInput.Task;
                }
            }
        };
        var registry = Registry(workerContext);
        var sessionId = Guid.NewGuid();
        var leaseId = Guid.NewGuid();
        var workerId = Guid.NewGuid();
        var now = DateTimeOffset.UtcNow;
        await Authorize(registry, "connection-a", sessionId, leaseId, workerId, now);

        var earlierInput = registry.DispatchAsync(
            "connection-a",
            sessionId,
            leaseId,
            new ViewerInputEvent("keyDown", Code: "KeyW"),
            now,
            CancellationToken.None);
        await inputStarted.Task;
        var fence = registry.BeginLeaseRevocationAsync(
            sessionId,
            leaseId,
            now,
            CancellationToken.None);
        var laterInput = registry.DispatchAsync(
            "connection-a",
            sessionId,
            leaseId,
            PointerMove(),
            now,
            CancellationToken.None);

        Assert.False(fence.IsCompleted);
        Assert.False(laterInput.IsCompleted);
        allowInput.TrySetResult(true);
        Assert.Equal(ViewerControlCheck.Authorized, await earlierInput);
        var marker = await fence;
        Assert.Equal(ViewerControlCheck.Expired, await laterInput);
        registry.CancelLeaseRevocation(sessionId, leaseId, marker);
    }

    [Fact]
    public async Task SessionRevocationFenceBlocksEveryLeaseInTheSession()
    {
        var workerContext = new RecordingWorkerHubContext();
        var registry = Registry(workerContext);
        var sessionId = Guid.NewGuid();
        var firstLeaseId = Guid.NewGuid();
        var secondLeaseId = Guid.NewGuid();
        var workerId = Guid.NewGuid();
        var now = DateTimeOffset.UtcNow;
        await Authorize(
            registry,
            "connection-a",
            sessionId,
            firstLeaseId,
            workerId,
            now);
        await Authorize(
            registry,
            "connection-b",
            sessionId,
            secondLeaseId,
            workerId,
            now);

        var marker = await registry.BeginSessionRevocationAsync(
            sessionId,
            now,
            CancellationToken.None);

        Assert.All(registry.GetSnapshots(), snapshot => Assert.True(snapshot.ReleaseRequested));
        Assert.Equal(
            ViewerControlCheck.Expired,
            await registry.DispatchAsync(
                "connection-a",
                sessionId,
                firstLeaseId,
                PointerMove(),
                now,
                CancellationToken.None));
        Assert.Equal(
            ViewerControlCheck.Expired,
            await registry.DispatchAsync(
                "connection-b",
                sessionId,
                secondLeaseId,
                PointerMove(),
                now,
                CancellationToken.None));

        registry.CancelSessionRevocation(sessionId, marker);
        Assert.All(registry.GetSnapshots(), snapshot => Assert.False(snapshot.ReleaseRequested));
    }

    [Fact]
    public async Task CommittedFenceSurvivesAnotherTransactionRollingBack()
    {
        var workerContext = new RecordingWorkerHubContext();
        var registry = Registry(workerContext);
        var sessionId = Guid.NewGuid();
        var leaseId = Guid.NewGuid();
        var workerId = Guid.NewGuid();
        var now = DateTimeOffset.UtcNow;
        await Authorize(registry, "connection-a", sessionId, leaseId, workerId, now);

        var first = await registry.BeginLeaseRevocationAsync(
            sessionId,
            leaseId,
            now,
            CancellationToken.None);
        var second = await registry.BeginLeaseRevocationAsync(
            sessionId,
            leaseId,
            now.AddMilliseconds(1),
            CancellationToken.None);

        Assert.Equal(first, second);
        registry.ConfirmLeaseRevocation(sessionId, leaseId, first);
        registry.CancelLeaseRevocation(sessionId, leaseId, second);

        Assert.True(Assert.Single(registry.GetSnapshots()).ReleaseRequested);
        Assert.Equal(
            ViewerControlCheck.Expired,
            await registry.DispatchAsync(
                "connection-a",
                sessionId,
                leaseId,
                PointerMove(),
                now,
                CancellationToken.None));
    }

    [Fact]
    public async Task FailedReleaseKeepsOwnershipAndFailsClosed()
    {
        var workerContext = new RecordingWorkerHubContext
        {
            OnSend = (method, _, _) => method == "ViewerInputRelease"
                ? Task.FromException(new IOException("worker send failed"))
                : Task.CompletedTask
        };
        var registry = Registry(workerContext);
        var sessionId = Guid.NewGuid();
        var leaseId = Guid.NewGuid();
        var workerId = Guid.NewGuid();
        var now = Time("2026-07-19T12:00:00Z");
        await Authorize(registry, "connection-a", sessionId, leaseId, workerId, now);

        Assert.Equal(
            ViewerControlDrainStatus.ReleaseFailed,
            await registry.ReleaseConnectionAsync(
                "connection-a",
                CancellationToken.None));
        Assert.Equal(
            ViewerControlAuthorizationStatus.Occupied,
            (await Authorize(
                registry,
                "connection-b",
                sessionId,
                leaseId,
                workerId,
                now)).Status);
        Assert.True(registry.IsCurrent("connection-a", Assert.Single(registry.GetSnapshots()).Version));
    }

    [Fact]
    public async Task StaleSnapshotCannotRemoveARenewedGrant()
    {
        var workerContext = new RecordingWorkerHubContext();
        var registry = Registry(workerContext);
        var sessionId = Guid.NewGuid();
        var leaseId = Guid.NewGuid();
        var workerId = Guid.NewGuid();
        var now = Time("2026-07-19T12:00:00Z");
        var first = await Authorize(
            registry,
            "connection-a",
            sessionId,
            leaseId,
            workerId,
            now);
        var stale = Assert.Single(registry.GetSnapshots());
        var renewed = await Authorize(
            registry,
            "connection-a",
            sessionId,
            leaseId,
            workerId,
            now.AddSeconds(1));

        Assert.NotEqual(first.Version, renewed.Version);
        Assert.Equal(
            ViewerControlDrainStatus.MissingOrReplaced,
            await registry.DrainVersionAsync(stale, CancellationToken.None));
        Assert.True(registry.IsCurrent("connection-a", renewed.Version));
        Assert.Empty(workerContext.Messages);
    }

    [Fact]
    public async Task ReconcilerRevokesDatabaseGrantAndSendsExactRelease()
    {
        await using var context = TestDataContext.Create();
        var seeded = await SeedLease(context);
        var workerContext = new RecordingWorkerHubContext();
        var registry = Registry(workerContext);
        var now = DateTimeOffset.UtcNow;
        await Authorize(
            registry,
            "connection-a",
            seeded.Owner.Id,
            seeded.Session.Id,
            seeded.Lease.Id,
            seeded.Worker.Id,
            now);

        seeded.Lease.RevokedAt = now.UtcDateTime;
        await context.SaveChangesAsync();
        await ViewerControlReconciler.ReconcileOnceAsync(
            context,
            registry,
            now.AddSeconds(1),
            CancellationToken.None);

        Assert.Empty(registry.GetSnapshots());
        var message = Assert.Single(workerContext.Messages);
        Assert.Equal("ViewerInputRelease", message.Method);
        var release = Assert.IsType<ViewerInputReleaseEnvelope>(Assert.Single(message.Arguments));
        Assert.Equal(seeded.Session.Id, release.SessionId);
        Assert.Equal(seeded.Lease.Id, release.LeaseId);
    }

    [Fact]
    public async Task ReconcilerDoesNotRemoveVersionRenewedAfterItsSnapshot()
    {
        await using var context = TestDataContext.Create();
        var seeded = await SeedLease(context);
        var workerContext = new RecordingWorkerHubContext();
        var registry = Registry(workerContext);
        var now = DateTimeOffset.UtcNow;
        await Authorize(
            registry,
            "connection-a",
            seeded.Owner.Id,
            seeded.Session.Id,
            seeded.Lease.Id,
            seeded.Worker.Id,
            now);
        var stale = Assert.Single(registry.GetSnapshots());
        var renewed = await Authorize(
            registry,
            "connection-a",
            seeded.Owner.Id,
            seeded.Session.Id,
            seeded.Lease.Id,
            seeded.Worker.Id,
            now.AddSeconds(1));

        seeded.Lease.RevokedAt = now.UtcDateTime;
        await context.SaveChangesAsync();
        Assert.Equal(
            ViewerControlDrainStatus.MissingOrReplaced,
            await registry.DrainVersionAsync(stale, CancellationToken.None));
        Assert.True(registry.IsCurrent("connection-a", renewed.Version));

        await ViewerControlReconciler.ReconcileOnceAsync(
            context,
            registry,
            now.AddSeconds(2),
            CancellationToken.None);
        Assert.Empty(registry.GetSnapshots());
        Assert.Equal("ViewerInputRelease", Assert.Single(workerContext.Messages).Method);
    }

    [Fact]
    public async Task GrantRequiresTheActiveLeaseSessionAndAccountToMatch()
    {
        await using var context = TestDataContext.Create();
        var seeded = await SeedLease(context);
        var now = DateTimeOffset.UtcNow;

        var grant = await SessionHub.FindViewerControlGrant(
            context,
            seeded.Owner.Id,
            seeded.Session.Id,
            seeded.Lease.Id,
            now,
            CancellationToken.None);
        var anotherOwner = await SessionHub.FindViewerControlGrant(
            context,
            seeded.Owner.Id + 1,
            seeded.Session.Id,
            seeded.Lease.Id,
            now,
            CancellationToken.None);

        Assert.NotNull(grant);
        Assert.Equal(seeded.Worker.Id, grant.WorkerId);
        Assert.Null(anotherOwner);

        seeded.Lease.RevokedAt = DateTime.UtcNow;
        await context.SaveChangesAsync();
        Assert.Null(await SessionHub.FindViewerControlGrant(
            context,
            seeded.Owner.Id,
            seeded.Session.Id,
            seeded.Lease.Id,
            now,
            CancellationToken.None));

        seeded.Lease.RevokedAt = null;
        seeded.Session.State = SimulationSessionState.Stopping;
        await context.SaveChangesAsync();
        Assert.Null(await SessionHub.FindViewerControlGrant(
            context,
            seeded.Owner.Id,
            seeded.Session.Id,
            seeded.Lease.Id,
            now,
            CancellationToken.None));
    }

    private static ViewerControlRegistry Registry(RecordingWorkerHubContext workerContext) =>
        new(workerContext, NullLogger<ViewerControlRegistry>.Instance);

    private static Task<ViewerControlAuthorizationResult> Authorize(
        ViewerControlRegistry registry,
        string connectionId,
        Guid sessionId,
        Guid leaseId,
        Guid workerId,
        DateTimeOffset now) =>
        Authorize(registry, connectionId, 72, sessionId, leaseId, workerId, now);

    private static Task<ViewerControlAuthorizationResult> Authorize(
        ViewerControlRegistry registry,
        string connectionId,
        int accountId,
        Guid sessionId,
        Guid leaseId,
        Guid workerId,
        DateTimeOffset now) =>
        registry.AuthorizeAsync(
            connectionId,
            accountId,
            sessionId,
            leaseId,
            workerId,
            now.AddMinutes(5),
            now,
            CancellationToken.None);

    private static ViewerInputEvent PointerMove() => new("pointerMove", 0.25, 0.75);

    private static DateTimeOffset Time(string value) => DateTimeOffset.Parse(value);

    private static async Task<SeededLease> SeedLease(TestDataContext context)
    {
        var worker = new ComputeWorker
        {
            Id = Guid.NewGuid(),
            Name = "viewer-worker"
        };
        var owner = new Account
        {
            Id = 72,
            FirstName = "Viewer",
            LastName = "Owner",
            Email = $"viewer-control-{Guid.NewGuid():N}@example.test",
            Enabled = true
        };
        var session = new SimulationSession
        {
            Id = Guid.NewGuid(),
            Account = owner,
            AccountId = owner.Id,
            ComputeWorker = worker,
            ComputeWorkerId = worker.Id,
            State = SimulationSessionState.Ready,
            DesiredRobotCount = 3
        };
        var lease = new ViewerLease
        {
            Id = Guid.NewGuid(),
            SimulationSession = session,
            SimulationSessionId = session.Id,
            Account = owner,
            AccountId = owner.Id,
            TokenHash = Guid.NewGuid().ToString("N"),
            ExpiresAt = DateTime.UtcNow.AddMinutes(5)
        };
        context.AddRange(worker, owner, session, lease);
        await context.SaveChangesAsync();
        return new SeededLease(worker, owner, session, lease);
    }

    private sealed record SeededLease(
        ComputeWorker Worker,
        Account Owner,
        SimulationSession Session,
        ViewerLease Lease);

    private sealed class RecordingWorkerHubContext : IHubContext<WorkerHub>
    {
        private readonly RecordingHubClients _clients;

        public RecordingWorkerHubContext()
        {
            _clients = new RecordingHubClients(this);
        }

        public Func<string, object?[], CancellationToken, Task>? OnSend { get; init; }
        public List<RecordedMessage> Messages { get; } = new();
        public IHubClients Clients => _clients;
        public IGroupManager Groups => null!;

        private sealed class RecordingHubClients(RecordingWorkerHubContext owner) : IHubClients
        {
            public IClientProxy All => new RecordingClientProxy(owner, string.Empty);
            public IClientProxy AllExcept(IReadOnlyList<string> excludedConnectionIds) => All;
            public IClientProxy Client(string connectionId) => All;
            public IClientProxy Clients(IReadOnlyList<string> connectionIds) => All;
            public IClientProxy Group(string groupName) =>
                new RecordingClientProxy(owner, groupName);
            public IClientProxy GroupExcept(
                string groupName,
                IReadOnlyList<string> excludedConnectionIds) => Group(groupName);
            public IClientProxy Groups(IReadOnlyList<string> groupNames) => All;
            public IClientProxy User(string userId) => All;
            public IClientProxy Users(IReadOnlyList<string> userIds) => All;
        }

        private sealed class RecordingClientProxy(
            RecordingWorkerHubContext owner,
            string groupName) : IClientProxy
        {
            public async Task SendCoreAsync(
                string method,
                object?[] args,
                CancellationToken cancellationToken = default)
            {
                owner.Messages.Add(new RecordedMessage(groupName, method, args));
                if (owner.OnSend is not null)
                {
                    await owner.OnSend(method, args, cancellationToken);
                }
            }
        }
    }

    private sealed record RecordedMessage(
        string GroupName,
        string Method,
        object?[] Arguments);
}
