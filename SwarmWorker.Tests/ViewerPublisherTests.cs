using System.Diagnostics;
using Microsoft.Extensions.Logging;
using Microsoft.Extensions.Logging.Abstractions;
using Microsoft.Extensions.Options;
using SwarmWorker.Configuration;
using SwarmWorker.Contracts;
using SwarmWorker.Runtime;

namespace SwarmWorker.Tests;

public sealed class ViewerPublisherTests
{
    private const string PublishToken =
        "AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA";

    [Fact]
    public async Task DisposeCanBeCalledMoreThanOnce()
    {
        var publisher = Publisher(new ViewerPublisherOptions());

        await publisher.DisposeAsync();
        await publisher.DisposeAsync();
    }

    [Fact]
    public async Task DisabledPublisherRemainsUnavailable()
    {
        await using var publisher = Publisher(new ViewerPublisherOptions
        {
            Enabled = false
        });

        await publisher.RefreshAvailabilityAsync(CancellationToken.None);

        Assert.False(publisher.Availability.IsAvailable);
        Assert.Empty(publisher.Availability.Sources);
    }

    [Fact]
    public async Task MissingExecutableDoesNotAdvertiseCapabilities()
    {
        await using var publisher = Publisher(new ViewerPublisherOptions
        {
            Enabled = true,
            PublisherExecutable = "/definitely/missing/robotswarm-viewer-publisher",
            PublishBaseUrl = "rtsp://media.example.test:8554"
        });

        await publisher.RefreshAvailabilityAsync(CancellationToken.None);

        Assert.False(publisher.Availability.IsAvailable);
        Assert.Contains(
            "probe failed",
            publisher.Availability.Reason,
            StringComparison.OrdinalIgnoreCase);
    }

    [Fact]
    public async Task InvalidPublishUrlFailsBeforeStartingAProcess()
    {
        await using var publisher = Publisher(new ViewerPublisherOptions
        {
            Enabled = true,
            PublisherExecutable = "/bin/false",
            PublishBaseUrl = "https://media.example.test/viewer?token=secret"
        });

        await publisher.RefreshAvailabilityAsync(CancellationToken.None);

        Assert.False(publisher.Availability.IsAvailable);
        Assert.Contains("RTSP", publisher.Availability.Reason, StringComparison.Ordinal);
    }

    [Fact]
    public async Task ProbedHelperPublishesAndGracefullyStopsOneSceneProcess()
    {
        if (!OperatingSystem.IsLinux())
        {
            return;
        }

        var directory = Directory.CreateTempSubdirectory("robotswarm-viewer-test-");
        var executable = Path.Combine(directory.FullName, "viewer-publisher");
        var terminationMarker = Path.Combine(directory.FullName, "term.marker");
        var inputMarker = Path.Combine(directory.FullName, "input.marker");
        try
        {
            await File.WriteAllTextAsync(
                executable,
                """
                #!/bin/sh
                set -eu
                if test "$1" = "probe"; then
                  test -z "${ROBOTSWARM_MEDIA_TOKEN:-}"
                  test -z "${ROBOTSWARM_WORKER_ID:-}"
                  test -z "${Worker__WorkerSecret:-}"
                  if IFS= read -r unexpected; then exit 8; fi
                  echo '{"protocolVersion":2,"ready":true,"videoCodec":"H264","sources":["Scene"],"interactive":true}'
                  exit 0
                fi
                test "$1" = "publish"
                worker_id=""
                previous=""
                for argument in "$@"; do
                  if test "$previous" = "--worker-id"; then worker_id="$argument"; fi
                  previous="$argument"
                done
                case "$worker_id" in
                  ????????-????-????-????-????????????) ;;
                  *) exit 10 ;;
                esac
                test -z "${ROBOTSWARM_MEDIA_TOKEN:-}"
                IFS= read -r media_token
                test "$media_token" = 'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA'
                test -z "${ROBOTSWARM_WORKER_ID:-}"
                test -z "${Worker__WorkerSecret:-}"
                case "$*" in
                  *"$media_token"*) exit 9 ;;
                esac
                marker="$(dirname "$0")/term.marker"
                trap 'printf "%s" TERM > "$marker"; exit 0' TERM
                echo READY
                IFS= read -r viewer_input
                printf '%s' "$viewer_input" > "$(dirname "$0")/input.marker"
                while :; do sleep 0.1; done
                """);
            File.SetUnixFileMode(
                executable,
                UnixFileMode.UserRead
                    | UnixFileMode.UserWrite
                    | UnixFileMode.UserExecute);

            var previousWorkerSecret = Environment.GetEnvironmentVariable(
                "Worker__WorkerSecret");
            Environment.SetEnvironmentVariable(
                "Worker__WorkerSecret",
                "must-not-reach-publisher");
            try
            {
                await using var publisher = Publisher(new ViewerPublisherOptions
                {
                    Enabled = true,
                    PublisherExecutable = executable,
                    PublishBaseUrl = "rtsp://media.example.test:8554",
                    ProbeTimeoutSeconds = 2,
                    StartupTimeoutSeconds = 2,
                    StopTimeoutSeconds = 2
                });
                await publisher.RefreshAvailabilityAsync(CancellationToken.None);
                var sessionId = Guid.NewGuid();
                var sourceId = $"scene-{sessionId:N}";
                var command = new ViewerSourceCommand(
                    Guid.NewGuid(),
                    DateTimeOffset.UtcNow.AddMinutes(1),
                    PublishToken,
                    ViewerSourceKind.Scene,
                    null,
                    sourceId,
                    $"session/{sessionId:N}/{sourceId}");

                var result = await publisher.PublishAsync(
                    new ViewerPublishRequest(
                        sessionId,
                        "container-1",
                        SessionResourceNames.Container(sessionId),
                        command),
                    CancellationToken.None);

                Assert.True(publisher.Availability.IsAvailable);
                Assert.Equal(
                    new[] { ViewerSourceKind.Scene },
                    publisher.Availability.Sources);
                Assert.True(result.Ready);
                Assert.Equal(command.LeaseId, result.LeaseId);

                await publisher.SendInputAsync(
                    new ViewerInputEnvelope(
                        sessionId,
                        command.LeaseId,
                        new ViewerInputEvent("pointerMove", 0.25, 0.75)),
                    CancellationToken.None);
                await WaitUntilAsync(
                    () => File.Exists(inputMarker),
                    TimeSpan.FromSeconds(2));
                Assert.Equal(
                    "{\"type\":\"pointerMove\",\"x\":0.25,\"y\":0.75}",
                    await File.ReadAllTextAsync(inputMarker));

                await Assert.ThrowsAsync<InvalidOperationException>(() =>
                    publisher.SendInputAsync(
                        new ViewerInputEnvelope(
                            sessionId,
                            Guid.NewGuid(),
                            new ViewerInputEvent("keyDown", Code: "KeyW")),
                        CancellationToken.None));

                await publisher.StopSessionAsync(sessionId, CancellationToken.None);
                Assert.Equal(
                    "TERM",
                    await File.ReadAllTextAsync(terminationMarker));
            }
            finally
            {
                Environment.SetEnvironmentVariable(
                    "Worker__WorkerSecret",
                    previousWorkerSecret);
            }
        }
        finally
        {
            directory.Delete(recursive: true);
        }
    }

    [Fact]
    public async Task CancelledForcedStopReleasesTheProcessGate()
    {
        if (!OperatingSystem.IsLinux())
        {
            return;
        }

        var directory = Directory.CreateTempSubdirectory("robotswarm-viewer-test-");
        var executable = Path.Combine(directory.FullName, "viewer-publisher");
        try
        {
            await File.WriteAllTextAsync(
                executable,
                """
                #!/bin/sh
                set -eu
                if test "$1" = "probe"; then
                  echo '{"protocolVersion":2,"ready":true,"videoCodec":"H264","sources":["Scene"],"interactive":true}'
                  exit 0
                fi
                IFS= read -r media_token
                trap '' TERM
                echo READY
                while :; do sleep 0.1; done
                """);
            File.SetUnixFileMode(
                executable,
                UnixFileMode.UserRead
                    | UnixFileMode.UserWrite
                    | UnixFileMode.UserExecute);

            await using var publisher = Publisher(new ViewerPublisherOptions
            {
                Enabled = true,
                PublisherExecutable = executable,
                PublishBaseUrl = "rtsp://media.example.test:8554",
                ProbeTimeoutSeconds = 2,
                StartupTimeoutSeconds = 2,
                StopTimeoutSeconds = 30
            });
            await publisher.RefreshAvailabilityAsync(CancellationToken.None);
            var sessionId = Guid.NewGuid();
            var sourceId = $"scene-{sessionId:N}";
            var command = new ViewerSourceCommand(
                Guid.NewGuid(),
                DateTimeOffset.UtcNow.AddMinutes(1),
                PublishToken,
                ViewerSourceKind.Scene,
                null,
                sourceId,
                $"session/{sessionId:N}/{sourceId}");
            await publisher.PublishAsync(
                new ViewerPublishRequest(
                    sessionId,
                    "container-1",
                    SessionResourceNames.Container(sessionId),
                    command),
                CancellationToken.None);

            using var stopCancellation = new CancellationTokenSource(
                TimeSpan.FromMilliseconds(100));
            await Assert.ThrowsAnyAsync<OperationCanceledException>(
                () => publisher.StopSessionAsync(
                    sessionId,
                    stopCancellation.Token));

            using var gateDeadline = new CancellationTokenSource(
                TimeSpan.FromSeconds(1));
            await publisher.StopSessionAsync(sessionId, gateDeadline.Token);
        }
        finally
        {
            directory.Delete(recursive: true);
        }
    }

    [Fact]
    public async Task BlockedInputForOneSessionDoesNotDelayAnotherSession()
    {
        if (!OperatingSystem.IsLinux())
        {
            return;
        }

        var directory = Directory.CreateTempSubdirectory("robotswarm-viewer-test-");
        var executable = Path.Combine(directory.FullName, "viewer-publisher");
        var blockedSession = Guid.NewGuid();
        var freeSession = Guid.NewGuid();
        var blockedWriteStarted = new TaskCompletionSource<bool>(
            TaskCreationOptions.RunContinuationsAsynchronously);
        var continueBlockedWrite = new TaskCompletionSource<bool>(
            TaskCreationOptions.RunContinuationsAsynchronously);
        var freeSessionWrites = 0;
        try
        {
            await File.WriteAllTextAsync(
                executable,
                """
                #!/bin/sh
                set -eu
                if test "$1" = "probe"; then
                  echo '{"protocolVersion":2,"ready":true,"videoCodec":"H264","sources":["Scene"],"interactive":true}'
                  exit 0
                fi
                IFS= read -r media_token
                trap 'exit 0' TERM
                echo READY
                while :; do sleep 0.1; done
                """);
            File.SetUnixFileMode(
                executable,
                UnixFileMode.UserRead
                    | UnixFileMode.UserWrite
                    | UnixFileMode.UserExecute);

            async Task WriteInputAsync(
                Guid sessionId,
                Process process,
                string line,
                CancellationToken cancellationToken)
            {
                _ = process;
                _ = line;
                _ = cancellationToken;
                if (sessionId == blockedSession)
                {
                    blockedWriteStarted.TrySetResult(true);
                    await continueBlockedWrite.Task;
                    return;
                }

                Interlocked.Increment(ref freeSessionWrites);
            }

            await using var publisher = Publisher(
                new ViewerPublisherOptions
                {
                    Enabled = true,
                    PublisherExecutable = executable,
                    PublishBaseUrl = "rtsp://media.example.test:8554",
                    ProbeTimeoutSeconds = 2,
                    StartupTimeoutSeconds = 2,
                    StopTimeoutSeconds = 1
                },
                inputWriter: WriteInputAsync);
            await publisher.RefreshAvailabilityAsync(CancellationToken.None);
            var blockedCommand = CommandFor(blockedSession);
            var freeCommand = CommandFor(freeSession);
            await publisher.PublishAsync(
                RequestFor(blockedSession, blockedCommand),
                CancellationToken.None);
            await publisher.PublishAsync(
                RequestFor(freeSession, freeCommand),
                CancellationToken.None);

            var blockedInput = publisher.SendInputAsync(
                new ViewerInputEnvelope(
                    blockedSession,
                    blockedCommand.LeaseId,
                    new ViewerInputEvent("keyDown", Code: "KeyW")),
                CancellationToken.None);
            await blockedWriteStarted.Task.WaitAsync(TimeSpan.FromSeconds(1));

            using (var freeSessionDeadline = new CancellationTokenSource(
                       TimeSpan.FromSeconds(1)))
            {
                await publisher.SendInputAsync(
                    new ViewerInputEnvelope(
                        freeSession,
                        freeCommand.LeaseId,
                        new ViewerInputEvent("pointerMove", 0.4, 0.6)),
                    freeSessionDeadline.Token);
                await publisher.ReleaseInputAsync(
                    freeSession,
                    freeCommand.LeaseId,
                    freeSessionDeadline.Token);
                await publisher.StopSessionAsync(
                    freeSession,
                    freeSessionDeadline.Token);
            }

            Assert.Equal(2, Volatile.Read(ref freeSessionWrites));
            continueBlockedWrite.TrySetResult(true);
            await blockedInput;
            await publisher.StopSessionAsync(blockedSession, CancellationToken.None);
        }
        finally
        {
            continueBlockedWrite.TrySetResult(true);
            directory.Delete(recursive: true);
        }
    }

    [Fact]
    public async Task InputPipeFailureStopsOnlyTheMatchingPublisher()
    {
        if (!OperatingSystem.IsLinux())
        {
            return;
        }

        var directory = Directory.CreateTempSubdirectory("robotswarm-viewer-test-");
        var executable = Path.Combine(directory.FullName, "viewer-publisher");
        var failedSession = Guid.NewGuid();
        var disposedSession = Guid.NewGuid();
        var healthySession = Guid.NewGuid();
        var failedWriteStarted = new TaskCompletionSource<bool>(
            TaskCreationOptions.RunContinuationsAsynchronously);
        var failFirstWrite = new TaskCompletionSource<bool>(
            TaskCreationOptions.RunContinuationsAsynchronously);
        var failedSessionWrites = 0;
        var healthyWrites = 0;
        try
        {
            await File.WriteAllTextAsync(
                executable,
                """
                #!/bin/sh
                set -eu
                if test "$1" = "probe"; then
                  echo '{"protocolVersion":2,"ready":true,"videoCodec":"H264","sources":["Scene"],"interactive":true}'
                  exit 0
                fi
                IFS= read -r media_token
                trap 'exit 0' TERM
                echo READY
                while :; do sleep 0.1; done
                """);
            File.SetUnixFileMode(
                executable,
                UnixFileMode.UserRead
                    | UnixFileMode.UserWrite
                    | UnixFileMode.UserExecute);

            async Task WriteInputAsync(
                Guid sessionId,
                Process process,
                string line,
                CancellationToken cancellationToken)
            {
                _ = process;
                _ = line;
                _ = cancellationToken;
                if (sessionId == failedSession)
                {
                    Interlocked.Increment(ref failedSessionWrites);
                    failedWriteStarted.TrySetResult(true);
                    await failFirstWrite.Task;
                    throw new IOException("simulated closed input pipe");
                }

                if (sessionId == disposedSession)
                {
                    throw new ObjectDisposedException("simulated disposed input writer");
                }

                Interlocked.Increment(ref healthyWrites);
            }

            await using var publisher = Publisher(
                new ViewerPublisherOptions
                {
                    Enabled = true,
                    PublisherExecutable = executable,
                    PublishBaseUrl = "rtsp://media.example.test:8554",
                    ProbeTimeoutSeconds = 2,
                    StartupTimeoutSeconds = 2,
                    StopTimeoutSeconds = 1
                },
                inputWriter: WriteInputAsync);
            await publisher.RefreshAvailabilityAsync(CancellationToken.None);
            var failedCommand = CommandFor(failedSession);
            var disposedCommand = CommandFor(disposedSession);
            var healthyCommand = CommandFor(healthySession);
            await publisher.PublishAsync(
                RequestFor(failedSession, failedCommand),
                CancellationToken.None);
            await publisher.PublishAsync(
                RequestFor(disposedSession, disposedCommand),
                CancellationToken.None);
            await publisher.PublishAsync(
                RequestFor(healthySession, healthyCommand),
                CancellationToken.None);

            var firstFailedInput = publisher.SendInputAsync(
                new ViewerInputEnvelope(
                    failedSession,
                    failedCommand.LeaseId,
                    new ViewerInputEvent("keyDown", Code: "KeyW")),
                CancellationToken.None);
            await failedWriteStarted.Task.WaitAsync(TimeSpan.FromSeconds(1));
            var queuedFailedInput = publisher.SendInputAsync(
                new ViewerInputEnvelope(
                    failedSession,
                    failedCommand.LeaseId,
                    new ViewerInputEvent("keyUp", Code: "KeyW")),
                CancellationToken.None);
            failFirstWrite.TrySetResult(true);

            await Assert.ThrowsAsync<IOException>(async () => await firstFailedInput);
            await Assert.ThrowsAsync<InvalidOperationException>(() =>
                queuedFailedInput);
            Assert.Equal(1, Volatile.Read(ref failedSessionWrites));

            await Assert.ThrowsAsync<IOException>(() => publisher.SendInputAsync(
                new ViewerInputEnvelope(
                    disposedSession,
                    disposedCommand.LeaseId,
                    new ViewerInputEvent("keyDown", Code: "KeyD")),
                CancellationToken.None));
            await Assert.ThrowsAsync<InvalidOperationException>(() =>
                publisher.SendInputAsync(
                    new ViewerInputEnvelope(
                        disposedSession,
                        disposedCommand.LeaseId,
                        new ViewerInputEvent("keyUp", Code: "KeyD")),
                    CancellationToken.None));

            await publisher.SendInputAsync(
                new ViewerInputEnvelope(
                    healthySession,
                    healthyCommand.LeaseId,
                    new ViewerInputEvent("keyDown", Code: "KeyA")),
                CancellationToken.None);
            Assert.Equal(1, Volatile.Read(ref healthyWrites));
            await publisher.StopSessionAsync(healthySession, CancellationToken.None);
        }
        finally
        {
            failFirstWrite.TrySetResult(true);
            directory.Delete(recursive: true);
        }
    }

    [Fact]
    public async Task StopRetriesProcessGroupReapingWhenRecoveryIsCancelled()
    {
        if (!OperatingSystem.IsLinux())
        {
            return;
        }

        var directory = Directory.CreateTempSubdirectory("robotswarm-viewer-test-");
        var executable = Path.Combine(directory.FullName, "viewer-publisher");
        var firstReapStarted = new TaskCompletionSource<bool>(
            TaskCreationOptions.RunContinuationsAsynchronously);
        var reapAttempts = 0;
        try
        {
            await File.WriteAllTextAsync(
                executable,
                """
                #!/bin/sh
                set -eu
                if test "$1" = "probe"; then
                  echo '{"protocolVersion":2,"ready":true,"videoCodec":"H264","sources":["Scene"],"interactive":true}'
                  exit 0
                fi
                IFS= read -r media_token
                echo READY
                sleep 0.1
                exit 17
                """);
            File.SetUnixFileMode(
                executable,
                UnixFileMode.UserRead
                    | UnixFileMode.UserWrite
                    | UnixFileMode.UserExecute);

            async Task<bool> ReapProcessGroupAsync(
                int processId,
                CancellationToken cancellationToken)
            {
                _ = processId;
                var attempt = Interlocked.Increment(ref reapAttempts);
                if (attempt == 1)
                {
                    firstReapStarted.TrySetResult(true);
                    await Task.Delay(Timeout.InfiniteTimeSpan, cancellationToken);
                }

                return true;
            }

            await using var publisher = Publisher(
                new ViewerPublisherOptions
                {
                    Enabled = true,
                    PublisherExecutable = executable,
                    PublishBaseUrl = "rtsp://media.example.test:8554",
                    ProbeTimeoutSeconds = 2,
                    StartupTimeoutSeconds = 2,
                    StopTimeoutSeconds = 1
                },
                inputWriter: (_, _, _, _) => Task.CompletedTask,
                processGroupReaper: ReapProcessGroupAsync);
            await publisher.RefreshAvailabilityAsync(CancellationToken.None);
            var sessionId = Guid.NewGuid();
            var command = CommandFor(sessionId);
            await publisher.PublishAsync(
                RequestFor(sessionId, command),
                CancellationToken.None);
            await firstReapStarted.Task.WaitAsync(TimeSpan.FromSeconds(2));

            await publisher.StopSessionAsync(sessionId, CancellationToken.None);

            Assert.Equal(2, Volatile.Read(ref reapAttempts));
        }
        finally
        {
            directory.Delete(recursive: true);
        }
    }

    [Fact]
    public async Task UnexpectedPublisherExitIsRecoveredWithinTheActiveLease()
    {
        if (!OperatingSystem.IsLinux())
        {
            return;
        }

        var directory = Directory.CreateTempSubdirectory("robotswarm-viewer-test-");
        var executable = Path.Combine(directory.FullName, "viewer-publisher");
        var startsMarker = Path.Combine(directory.FullName, "starts.marker");
        var terminationMarker = Path.Combine(directory.FullName, "term.marker");
        try
        {
            await File.WriteAllTextAsync(
                executable,
                """
                #!/bin/sh
                set -eu
                if test "$1" = "probe"; then
                  echo '{"protocolVersion":2,"ready":true,"videoCodec":"H264","sources":["Scene"],"interactive":true}'
                  exit 0
                fi
                test "$1" = "publish"
                test -z "${ROBOTSWARM_MEDIA_TOKEN:-}"
                IFS= read -r media_token
                test "$media_token" = 'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA'
                directory="$(dirname "$0")"
                starts=0
                if test -f "$directory/starts.marker"; then
                  starts="$(cat "$directory/starts.marker")"
                fi
                starts=$((starts + 1))
                printf '%s' "$starts" > "$directory/starts.marker"
                echo READY
                if test "$starts" = 1; then
                  sleep 0.2
                  printf 'transient failure for token %s\n' "$media_token" >&2
                  exit 17
                fi
                trap 'printf "%s" TERM > "$directory/term.marker"; exit 0' TERM
                while :; do sleep 0.1; done
                """);
            File.SetUnixFileMode(
                executable,
                UnixFileMode.UserRead
                    | UnixFileMode.UserWrite
                    | UnixFileMode.UserExecute);

            var logger = new RecordingLogger<ExternalViewerPublisher>();
            await using var publisher = Publisher(
                new ViewerPublisherOptions
                {
                    Enabled = true,
                    PublisherExecutable = executable,
                    PublishBaseUrl = "rtsp://media.example.test:8554",
                    ProbeTimeoutSeconds = 2,
                    StartupTimeoutSeconds = 2,
                    StopTimeoutSeconds = 2
                },
                logger);
            await publisher.RefreshAvailabilityAsync(CancellationToken.None);
            var sessionId = Guid.NewGuid();
            var sourceId = $"scene-{sessionId:N}";
            var command = new ViewerSourceCommand(
                Guid.NewGuid(),
                DateTimeOffset.UtcNow.AddMinutes(1),
                PublishToken,
                ViewerSourceKind.Scene,
                null,
                sourceId,
                $"session/{sessionId:N}/{sourceId}");
            var request = new ViewerPublishRequest(
                sessionId,
                "container-1",
                SessionResourceNames.Container(sessionId),
                command);

            await publisher.PublishAsync(request, CancellationToken.None);
            await WaitUntilAsync(
                () => File.Exists(startsMarker)
                    && File.ReadAllText(startsMarker) == "2",
                TimeSpan.FromSeconds(5));

            var repeated = await publisher.PublishAsync(
                request,
                CancellationToken.None);
            Assert.True(repeated.Ready);
            Assert.Equal("2", await File.ReadAllTextAsync(startsMarker));
            Assert.DoesNotContain(PublishToken, logger.MessagesText);
            Assert.Contains("[REDACTED]", logger.MessagesText);
            Assert.Contains(
                "recovered on restart 1",
                logger.MessagesText,
                StringComparison.OrdinalIgnoreCase);

            await publisher.StopSessionAsync(sessionId, CancellationToken.None);
            Assert.Equal(
                "TERM",
                await File.ReadAllTextAsync(terminationMarker));
        }
        finally
        {
            directory.Delete(recursive: true);
        }
    }

    [Fact]
    public async Task HardKilledPublisherReapsItsOrphansBeforeRecovery()
    {
        if (!OperatingSystem.IsLinux())
        {
            return;
        }

        var directory = Directory.CreateTempSubdirectory("robotswarm-viewer-test-");
        var executable = Path.Combine(directory.FullName, "viewer-publisher");
        var startsMarker = Path.Combine(directory.FullName, "starts.marker");
        var publisherPidMarker = Path.Combine(directory.FullName, "publisher.pid");
        var orphanPidMarker = Path.Combine(directory.FullName, "orphan.pid");
        var terminationMarker = Path.Combine(directory.FullName, "term.marker");
        try
        {
            await File.WriteAllTextAsync(
                executable,
                """
                #!/bin/sh
                set -eu
                if test "$1" = "probe"; then
                  echo '{"protocolVersion":2,"ready":true,"videoCodec":"H264","sources":["Scene"],"interactive":true}'
                  exit 0
                fi
                if test "${VIEWER_TEST_GROUPED:-0}" != 1; then
                  export VIEWER_TEST_GROUPED=1
                  exec setsid "$0" "$@"
                fi
                test -z "${ROBOTSWARM_MEDIA_TOKEN:-}"
                IFS= read -r media_token
                test "$media_token" = 'AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA'
                directory="$(dirname "$0")"
                starts=0
                if test -f "$directory/starts.marker"; then
                  starts="$(cat "$directory/starts.marker")"
                fi
                starts=$((starts + 1))
                if test "$starts" = 2; then
                  orphan_pid="$(cat "$directory/orphan.pid")"
                  if kill -0 "$orphan_pid" 2>/dev/null; then
                    echo 'orphan is still running' >&2
                    exit 1
                  fi
                fi
                printf '%s' "$starts" > "$directory/starts.marker"
                if test "$starts" = 1; then
                  (
                    trap 'exit 0' TERM
                    while :; do sleep 0.1; done
                  ) &
                  printf '%s' "$!" > "$directory/orphan.pid"
                  printf '%s' "$$" > "$directory/publisher.pid"
                  echo READY
                  while :; do sleep 0.1; done
                fi
                trap 'printf "%s" TERM > "$directory/term.marker"; exit 0' TERM
                echo READY
                while :; do sleep 0.1; done
                """);
            File.SetUnixFileMode(
                executable,
                UnixFileMode.UserRead
                    | UnixFileMode.UserWrite
                    | UnixFileMode.UserExecute);

            await using var publisher = Publisher(new ViewerPublisherOptions
            {
                Enabled = true,
                PublisherExecutable = executable,
                PublishBaseUrl = "rtsp://media.example.test:8554",
                ProbeTimeoutSeconds = 2,
                StartupTimeoutSeconds = 2,
                StopTimeoutSeconds = 2
            });
            await publisher.RefreshAvailabilityAsync(CancellationToken.None);
            var sessionId = Guid.NewGuid();
            var sourceId = $"scene-{sessionId:N}";
            var command = new ViewerSourceCommand(
                Guid.NewGuid(),
                DateTimeOffset.UtcNow.AddMinutes(1),
                PublishToken,
                ViewerSourceKind.Scene,
                null,
                sourceId,
                $"session/{sessionId:N}/{sourceId}");
            var request = new ViewerPublishRequest(
                sessionId,
                "container-1",
                SessionResourceNames.Container(sessionId),
                command);

            await publisher.PublishAsync(request, CancellationToken.None);
            await WaitUntilAsync(
                () => File.Exists(publisherPidMarker),
                TimeSpan.FromSeconds(2));
            var orphanProcessId = int.Parse(
                await File.ReadAllTextAsync(orphanPidMarker),
                System.Globalization.CultureInfo.InvariantCulture);
            Assert.True(ProcessExists(orphanProcessId));
            await KillAsync(int.Parse(
                await File.ReadAllTextAsync(publisherPidMarker),
                System.Globalization.CultureInfo.InvariantCulture));

            await WaitUntilAsync(
                () => File.Exists(startsMarker)
                    && File.ReadAllText(startsMarker) == "2",
                TimeSpan.FromSeconds(5));
            Assert.False(ProcessExists(orphanProcessId));

            await publisher.StopSessionAsync(sessionId, CancellationToken.None);
            Assert.Equal(
                "TERM",
                await File.ReadAllTextAsync(terminationMarker));
        }
        finally
        {
            directory.Delete(recursive: true);
        }
    }

    [Fact]
    public async Task PublisherFailureRedactsTheLeaseToken()
    {
        if (!OperatingSystem.IsLinux())
        {
            return;
        }

        var directory = Directory.CreateTempSubdirectory("robotswarm-viewer-test-");
        var executable = Path.Combine(directory.FullName, "viewer-publisher");
        try
        {
            await File.WriteAllTextAsync(
                executable,
                """
                #!/bin/sh
                set -eu
                if test "$1" = "probe"; then
                  echo '{"protocolVersion":2,"ready":true,"videoCodec":"H264","sources":["Scene"],"interactive":true}'
                  exit 0
                fi
                test -z "${ROBOTSWARM_MEDIA_TOKEN:-}"
                IFS= read -r media_token
                printf 'publisher rejected token %s\n' "$media_token" >&2
                exit 1
                """);
            File.SetUnixFileMode(
                executable,
                UnixFileMode.UserRead
                    | UnixFileMode.UserWrite
                    | UnixFileMode.UserExecute);

            await using var publisher = Publisher(new ViewerPublisherOptions
            {
                Enabled = true,
                PublisherExecutable = executable,
                PublishBaseUrl = "rtsp://media.example.test:8554",
                ProbeTimeoutSeconds = 2,
                StartupTimeoutSeconds = 2
            });
            await publisher.RefreshAvailabilityAsync(CancellationToken.None);
            var sessionId = Guid.NewGuid();
            var sourceId = $"scene-{sessionId:N}";
            var command = new ViewerSourceCommand(
                Guid.NewGuid(),
                DateTimeOffset.UtcNow.AddMinutes(1),
                PublishToken,
                ViewerSourceKind.Scene,
                null,
                sourceId,
                $"session/{sessionId:N}/{sourceId}");

            var exception = await Assert.ThrowsAsync<InvalidOperationException>(
                () => publisher.PublishAsync(
                    new ViewerPublishRequest(
                        sessionId,
                        "container-1",
                        SessionResourceNames.Container(sessionId),
                        command),
                    CancellationToken.None));

            Assert.DoesNotContain(PublishToken, exception.Message);
            Assert.Contains("[REDACTED]", exception.Message);
        }
        finally
        {
            directory.Delete(recursive: true);
        }
    }

    private static async Task WaitUntilAsync(
        Func<bool> condition,
        TimeSpan timeout)
    {
        var deadline = DateTimeOffset.UtcNow + timeout;
        while (DateTimeOffset.UtcNow < deadline)
        {
            if (condition())
            {
                return;
            }

            await Task.Delay(50);
        }

        Assert.Fail("The expected viewer publisher state was not reached in time.");
    }

    private static async Task KillAsync(int processId)
    {
        using var process = Process.GetProcessById(processId);
        process.Kill();
        await process.WaitForExitAsync();
    }

    private static bool ProcessExists(int processId)
    {
        try
        {
            using var process = Process.GetProcessById(processId);
            return !process.HasExited;
        }
        catch (ArgumentException)
        {
            return false;
        }
    }

    private static ExternalViewerPublisher Publisher(
        ViewerPublisherOptions viewer,
        ILogger<ExternalViewerPublisher>? logger = null,
        Func<Guid, Process, string, CancellationToken, Task>? inputWriter = null,
        Func<int, CancellationToken, Task<bool>>? processGroupReaper = null)
    {
        var options = Options.Create(new WorkerOptions
            {
                WorkerId = Guid.NewGuid(),
                WorkerSecret = new string('a', 32),
                Viewer = viewer
            });
        var actualLogger = logger ?? NullLogger<ExternalViewerPublisher>.Instance;
        return inputWriter is null && processGroupReaper is null
            ? new ExternalViewerPublisher(options, actualLogger)
            : new ExternalViewerPublisher(
                options,
                actualLogger,
                inputWriter ?? throw new ArgumentNullException(nameof(inputWriter)),
                processGroupReaper);
    }

    private static ViewerSourceCommand CommandFor(Guid sessionId)
    {
        var sourceId = $"scene-{sessionId:N}";
        return new ViewerSourceCommand(
            Guid.NewGuid(),
            DateTimeOffset.UtcNow.AddMinutes(1),
            PublishToken,
            ViewerSourceKind.Scene,
            null,
            sourceId,
            $"session/{sessionId:N}/{sourceId}");
    }

    private static ViewerPublishRequest RequestFor(
        Guid sessionId,
        ViewerSourceCommand command) =>
        new(
            sessionId,
            $"container-{sessionId:N}",
            SessionResourceNames.Container(sessionId),
            command);

    private sealed class RecordingLogger<T> : ILogger<T>
    {
        private readonly System.Collections.Concurrent.ConcurrentQueue<string> _messages =
            new();

        public string MessagesText => string.Join(Environment.NewLine, _messages);

        public IDisposable? BeginScope<TState>(TState state)
            where TState : notnull => null;

        public bool IsEnabled(LogLevel logLevel) => true;

        public void Log<TState>(
            LogLevel logLevel,
            EventId eventId,
            TState state,
            Exception? exception,
            Func<TState, Exception?, string> formatter)
        {
            _messages.Enqueue(formatter(state, exception));
        }
    }
}
