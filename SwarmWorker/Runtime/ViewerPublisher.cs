using System.ComponentModel;
using System.Diagnostics;
using System.Runtime.InteropServices;
using System.Text.Json;
using Microsoft.Extensions.Options;
using SwarmWorker.Configuration;

namespace SwarmWorker.Runtime;

public sealed record ViewerPublisherAvailability(
    bool IsAvailable,
    IReadOnlyList<ViewerSourceKind> Sources,
    string? VideoCodec,
    string Reason)
{
    public static ViewerPublisherAvailability Unavailable(string reason) =>
        new(false, Array.Empty<ViewerSourceKind>(), null, reason);
}

public sealed record ViewerPublishRequest(
    Guid SessionId,
    string ContainerId,
    string ContainerName,
    ViewerSourceCommand Command);

public sealed record ViewerPublishResult(
    Guid SessionId,
    Guid LeaseId,
    DateTimeOffset ExpiresAt,
    ViewerSourceKind Source,
    string? RobotRuntimeId,
    string SourceId,
    string StreamPath,
    bool Ready,
    string VideoCodec);

public interface IViewerPublisher
{
    ViewerPublisherAvailability Availability { get; }

    Task RefreshAvailabilityAsync(CancellationToken cancellationToken);

    Task<ViewerPublishResult> PublishAsync(
        ViewerPublishRequest request,
        CancellationToken cancellationToken);

    Task StopSessionAsync(Guid sessionId, CancellationToken cancellationToken);
}

public sealed class ExternalViewerPublisher : IViewerPublisher, IAsyncDisposable
{
    private const int ProtocolVersion = 1;
    private const int TerminationSignal = 15;
    private const int KillSignal = 9;
    private const int SignalCheck = 0;
    private const int NoSuchProcessError = 3;
    private const int MaximumUnexpectedExitRestarts = 3;

    private sealed record ActivePublisher(
        ViewerPublishRequest Request,
        Process Process,
        Task<string> ErrorOutput,
        CancellationTokenSource ExpiryCancellation,
        int RestartCount);

    private readonly ViewerPublisherOptions _options;
    private readonly ILogger<ExternalViewerPublisher> _logger;
    private readonly SemaphoreSlim _probeGate = new(1, 1);
    private readonly SemaphoreSlim _processGate = new(1, 1);
    private readonly Dictionary<Guid, ActivePublisher> _active = new();
    private ViewerPublisherAvailability _availability =
        ViewerPublisherAvailability.Unavailable("Viewer publishing has not been probed.");

    public ExternalViewerPublisher(
        IOptions<WorkerOptions> options,
        ILogger<ExternalViewerPublisher> logger)
    {
        _options = options.Value.Viewer;
        _logger = logger;
    }

    public ViewerPublisherAvailability Availability =>
        Volatile.Read(ref _availability);

    public async Task RefreshAvailabilityAsync(CancellationToken cancellationToken)
    {
        await _probeGate.WaitAsync(cancellationToken);
        try
        {
            if (!_options.Enabled)
            {
                SetAvailability(ViewerPublisherAvailability.Unavailable(
                    "Viewer publishing is disabled by worker configuration."));
                return;
            }

            if (string.IsNullOrWhiteSpace(_options.PublisherExecutable))
            {
                SetAvailability(ViewerPublisherAvailability.Unavailable(
                    "The publisher executable is not configured."));
                return;
            }

            if (!TryGetPublishBaseUri(out _, out var configurationError))
            {
                SetAvailability(ViewerPublisherAvailability.Unavailable(configurationError));
                return;
            }

            try
            {
                SetAvailability(await ProbeAsync(cancellationToken));
            }
            catch (OperationCanceledException) when (cancellationToken.IsCancellationRequested)
            {
                throw;
            }
            catch (Exception exception)
                when (exception is Win32Exception
                      or IOException
                      or InvalidOperationException
                      or JsonException
                      or TimeoutException
                      or UnauthorizedAccessException)
            {
                SetAvailability(ViewerPublisherAvailability.Unavailable(
                    $"Publisher probe failed: {Sanitize(exception.Message)}"));
            }
        }
        finally
        {
            _probeGate.Release();
        }
    }

    public async Task<ViewerPublishResult> PublishAsync(
        ViewerPublishRequest request,
        CancellationToken cancellationToken)
    {
        if (request.Command.ExpiresAt <= DateTimeOffset.UtcNow)
        {
            throw new InvalidOperationException(
                "The viewer lease expired before publishing could start.");
        }

        var availability = Availability;
        if (!availability.IsAvailable
            || availability.VideoCodec is null
            || !availability.Sources.Contains(request.Command.Source))
        {
            throw new InvalidOperationException(
                "The configured viewer publisher is not ready for this source.");
        }

        if (!TryGetPublishBaseUri(out var publishBaseUri, out var error))
        {
            throw new InvalidOperationException(error);
        }

        await _processGate.WaitAsync(cancellationToken);
        try
        {
            if (_active.TryGetValue(request.SessionId, out var existing))
            {
                if (!existing.Process.HasExited && existing.Request == request)
                {
                    return BuildResult(request, availability.VideoCodec);
                }

                await StopPublisherAsync(existing, cancellationToken);
                _active.Remove(request.SessionId);
            }

            var publisher = await StartPublisherAsync(
                request,
                publishBaseUri,
                cancellationToken);
            if (request.Command.ExpiresAt <= DateTimeOffset.UtcNow)
            {
                await StopPublisherAsync(publisher, CancellationToken.None);
                throw new InvalidOperationException(
                    "The viewer lease expired while publishing was starting.");
            }

            _active[request.SessionId] = publisher;
            WatchPublisher(publisher);
            return BuildResult(request, availability.VideoCodec);
        }
        finally
        {
            _processGate.Release();
        }
    }

    public async Task StopSessionAsync(
        Guid sessionId,
        CancellationToken cancellationToken)
    {
        await _processGate.WaitAsync(cancellationToken);
        try
        {
            if (_active.Remove(sessionId, out var publisher))
            {
                await StopPublisherAsync(publisher, cancellationToken);
            }
        }
        finally
        {
            _processGate.Release();
        }
    }

    public async ValueTask DisposeAsync()
    {
        await _processGate.WaitAsync();
        try
        {
            foreach (var publisher in _active.Values)
            {
                await StopPublisherAsync(publisher, CancellationToken.None);
            }

            _active.Clear();
        }
        finally
        {
            _processGate.Release();
            _processGate.Dispose();
            _probeGate.Dispose();
        }
    }

    private async Task<ViewerPublisherAvailability> ProbeAsync(
        CancellationToken cancellationToken)
    {
        using var process = CreateProcess();
        process.StartInfo.ArgumentList.Add("probe");
        process.StartInfo.ArgumentList.Add("--protocol-version");
        process.StartInfo.ArgumentList.Add(ProtocolVersion.ToString(
            System.Globalization.CultureInfo.InvariantCulture));
        process.StartInfo.ArgumentList.Add("--publish-base-url");
        process.StartInfo.ArgumentList.Add(_options.PublishBaseUrl.TrimEnd('/'));
        StartProcess(process);
        process.StandardInput.Close();
        var standardOutput = process.StandardOutput.ReadToEndAsync();
        var standardError = process.StandardError.ReadToEndAsync();

        using var timeout = CancellationTokenSource.CreateLinkedTokenSource(cancellationToken);
        timeout.CancelAfter(TimeSpan.FromSeconds(_options.ProbeTimeoutSeconds));
        try
        {
            await process.WaitForExitAsync(timeout.Token);
        }
        catch (OperationCanceledException)
        {
            TryKill(process);
            if (cancellationToken.IsCancellationRequested)
            {
                throw;
            }

            throw new TimeoutException("The viewer publisher probe timed out.");
        }

        var output = await standardOutput;
        var error = await standardError;
        if (process.ExitCode != 0)
        {
            throw new InvalidOperationException(
                $"Publisher probe exited with code {process.ExitCode}: {Sanitize(error)}");
        }

        using var document = JsonDocument.Parse(output);
        var root = document.RootElement;
        if (root.ValueKind != JsonValueKind.Object
            || !root.TryGetProperty("protocolVersion", out var protocol)
            || !protocol.TryGetInt32(out var protocolVersion)
            || protocolVersion != ProtocolVersion
            || !root.TryGetProperty("ready", out var ready)
            || ready.ValueKind != JsonValueKind.True
            || !root.TryGetProperty("videoCodec", out var codec)
            || !string.Equals(codec.GetString(), "H264", StringComparison.OrdinalIgnoreCase)
            || !root.TryGetProperty("sources", out var sourcesElement)
            || sourcesElement.ValueKind != JsonValueKind.Array)
        {
            throw new InvalidOperationException(
                "Publisher probe did not confirm protocol 1 with H.264 output.");
        }

        var sources = new List<ViewerSourceKind>();
        foreach (var sourceElement in sourcesElement.EnumerateArray())
        {
            var source = sourceElement.ValueKind == JsonValueKind.String
                ? sourceElement.GetString()
                : null;
            if (string.Equals(source, "Scene", StringComparison.OrdinalIgnoreCase))
            {
                sources.Add(ViewerSourceKind.Scene);
            }
            else
            {
                throw new InvalidOperationException(
                    "Publisher probe returned an unsupported viewer source; this image currently supports Scene only.");
            }
        }

        var distinctSources = sources.Distinct().Order().ToArray();
        if (distinctSources.Length == 0)
        {
            throw new InvalidOperationException(
                "Publisher probe did not expose any viewer source.");
        }

        return new ViewerPublisherAvailability(
            true,
            distinctSources,
            "H264",
            "Publisher probe succeeded.");
    }

    private async Task<ActivePublisher> StartPublisherAsync(
        ViewerPublishRequest request,
        Uri publishBaseUri,
        CancellationToken cancellationToken)
    {
        var process = CreateProcess();
        AddPublishArguments(process.StartInfo, request, publishBaseUri);

        try
        {
            StartProcess(process);
            await process.StandardInput.WriteLineAsync(
                request.Command.PublishToken.AsMemory(),
                cancellationToken);
            await process.StandardInput.FlushAsync(cancellationToken);
            process.StandardInput.Close();
            var errorOutput = process.StandardError.ReadToEndAsync();
            var readyLine = process.StandardOutput.ReadLineAsync();

            string? firstLine;
            try
            {
                firstLine = await readyLine.WaitAsync(
                    TimeSpan.FromSeconds(_options.StartupTimeoutSeconds),
                    cancellationToken);
            }
            catch (TimeoutException)
            {
                TryKill(process);
                throw new TimeoutException(
                    "The viewer publisher did not report readiness before the timeout.");
            }

            if (!string.Equals(firstLine, "READY", StringComparison.Ordinal)
                || process.HasExited)
            {
                TryKill(process);
                var error = await ReadErrorAfterExit(errorOutput);
                throw new InvalidOperationException(
                    $"The viewer publisher did not become ready: {Sanitize(error, request.Command.PublishToken)}");
            }

            _ = DrainOutputAsync(process, request.SessionId);
            return new ActivePublisher(
                request,
                process,
                errorOutput,
                new CancellationTokenSource(),
                RestartCount: 0);
        }
        catch
        {
            TryKill(process);
            process.Dispose();
            throw;
        }
    }

    private Process CreateProcess()
    {
        var process = new Process
        {
            StartInfo = new ProcessStartInfo
            {
                FileName = _options.PublisherExecutable,
                UseShellExecute = false,
                RedirectStandardInput = true,
                RedirectStandardOutput = true,
                RedirectStandardError = true,
                CreateNoWindow = true
            }
        };
        RemoveWorkerCredentials(process.StartInfo);
        return process;
    }

    private static void RemoveWorkerCredentials(ProcessStartInfo startInfo)
    {
        foreach (var name in startInfo.Environment.Keys.ToArray())
        {
            var normalized = name.ToLowerInvariant();
            if (normalized.StartsWith("worker__", StringComparison.Ordinal)
                || normalized is "robotswarm_media_token"
                    or "robotswarm_worker_id"
                    or "robotswarm_worker_secret"
                    or "robotswarm_worker_token")
            {
                startInfo.Environment.Remove(name);
            }
        }
    }

    private static void AddPublishArguments(
        ProcessStartInfo startInfo,
        ViewerPublishRequest request,
        Uri publishBaseUri)
    {
        AddArgument(startInfo, "publish");
        AddOption(startInfo, "--protocol-version", ProtocolVersion.ToString(
            System.Globalization.CultureInfo.InvariantCulture));
        AddOption(startInfo, "--session-id", request.SessionId.ToString("D"));
        AddOption(startInfo, "--lease-id", request.Command.LeaseId.ToString("D"));
        AddOption(
            startInfo,
            "--expires-at",
            request.Command.ExpiresAt.ToUniversalTime().ToString("O"));
        AddOption(startInfo, "--container-id", request.ContainerId);
        AddOption(startInfo, "--container-name", request.ContainerName);
        AddOption(startInfo, "--source", request.Command.Source.ToString());
        if (request.Command.RobotRuntimeId is not null)
        {
            AddOption(
                startInfo,
                "--robot-runtime-id",
                request.Command.RobotRuntimeId);
        }

        AddOption(startInfo, "--source-id", request.Command.SourceId);
        AddOption(startInfo, "--stream-path", request.Command.StreamPath);
        AddOption(startInfo, "--publish-base-url", publishBaseUri.AbsoluteUri.TrimEnd('/'));
    }

    private static void AddOption(
        ProcessStartInfo startInfo,
        string name,
        string value)
    {
        AddArgument(startInfo, name);
        AddArgument(startInfo, value);
    }

    private static void AddArgument(ProcessStartInfo startInfo, string value)
    {
        startInfo.ArgumentList.Add(value);
    }

    private static void StartProcess(Process process)
    {
        if (!process.Start())
        {
            throw new InvalidOperationException(
                "The configured viewer publisher process did not start.");
        }
    }

    private async Task StopPublisherAsync(
        ActivePublisher publisher,
        CancellationToken cancellationToken)
    {
        publisher.ExpiryCancellation.Cancel();
        try
        {
            if (!publisher.Process.HasExited)
            {
                var requestedGracefulStop = TryRequestGracefulStop(publisher.Process);
                if (requestedGracefulStop)
                {
                    using var timeout = CancellationTokenSource.CreateLinkedTokenSource(
                        cancellationToken);
                    timeout.CancelAfter(TimeSpan.FromSeconds(_options.StopTimeoutSeconds));
                    try
                    {
                        await publisher.Process.WaitForExitAsync(timeout.Token);
                        return;
                    }
                    catch (OperationCanceledException)
                    {
                        // A helper gets one bounded chance to clean up its display and
                        // encoder. Cancellation still forces cleanup before propagating.
                    }
                }

                TryKill(publisher.Process);
                using var forcedStopTimeout =
                    CancellationTokenSource.CreateLinkedTokenSource(
                        cancellationToken);
                forcedStopTimeout.CancelAfter(
                    TimeSpan.FromSeconds(_options.StopTimeoutSeconds));
                try
                {
                    await publisher.Process.WaitForExitAsync(
                        forcedStopTimeout.Token);
                }
                catch (OperationCanceledException)
                    when (!cancellationToken.IsCancellationRequested)
                {
                    throw new TimeoutException(
                        "The viewer publisher did not exit after its forced stop.");
                }

                cancellationToken.ThrowIfCancellationRequested();
            }
        }
        catch (InvalidOperationException)
        {
            // The process exited between the state check and stop request.
        }
        finally
        {
            publisher.Process.Dispose();
            publisher.ExpiryCancellation.Dispose();
        }
    }

    private bool TryRequestGracefulStop(Process process)
    {
        if (!OperatingSystem.IsLinux() && !OperatingSystem.IsMacOS())
        {
            return false;
        }

        try
        {
            if (process.HasExited)
            {
                return true;
            }

            if (SendUnixSignal(process.Id, TerminationSignal) == 0)
            {
                return true;
            }

            _logger.LogDebug(
                "Unable to send SIGTERM to viewer publisher {ProcessId}; native error {ErrorCode}. Falling back to a forced stop.",
                process.Id,
                Marshal.GetLastPInvokeError());
        }
        catch (InvalidOperationException)
        {
            // The process exited between the state check and signal request.
            return true;
        }

        return false;
    }

    private async Task StopAtExpiryAsync(ActivePublisher publisher)
    {
        try
        {
            var delay = publisher.Request.Command.ExpiresAt - DateTimeOffset.UtcNow;
            if (delay > TimeSpan.Zero)
            {
                await Task.Delay(delay, publisher.ExpiryCancellation.Token);
            }

            await _processGate.WaitAsync(publisher.ExpiryCancellation.Token);
            try
            {
                if (_active.TryGetValue(
                        publisher.Request.SessionId,
                        out var current)
                    && ReferenceEquals(current, publisher))
                {
                    _active.Remove(publisher.Request.SessionId);
                    await StopPublisherAsync(current, CancellationToken.None);
                }
            }
            finally
            {
                _processGate.Release();
            }
        }
        catch (OperationCanceledException)
        {
            // The publisher was replaced or stopped before its lease expired.
        }
        catch (ObjectDisposedException)
        {
            // Worker shutdown disposed the expiry token after stopping the process.
        }
        catch (Exception exception)
        {
            _logger.LogError(
                exception,
                "Unable to stop viewer publisher at lease expiry for session {SessionId}.",
                publisher.Request.SessionId);
        }
    }

    private void WatchPublisher(ActivePublisher publisher)
    {
        _ = StopAtExpiryAsync(publisher);
        _ = RecoverUnexpectedExitAsync(publisher);
    }

    private async Task RecoverUnexpectedExitAsync(ActivePublisher publisher)
    {
        try
        {
            await publisher.Process.WaitForExitAsync(
                publisher.ExpiryCancellation.Token);
        }
        catch (OperationCanceledException)
        {
            // An intentional stop or replacement cancels the process watcher.
            return;
        }
        catch (ObjectDisposedException)
        {
            // The normal stop path owns process disposal.
            return;
        }

        if (publisher.ExpiryCancellation.IsCancellationRequested
            || publisher.Request.Command.ExpiresAt <= DateTimeOffset.UtcNow)
        {
            return;
        }

        var exitCode = TryGetExitCode(publisher.Process);
        bool processGroupReaped;
        try
        {
            processGroupReaped = await ReapProcessGroupAsync(
                publisher.Process.Id,
                publisher.ExpiryCancellation.Token);
        }
        catch (OperationCanceledException)
        {
            return;
        }

        // A surviving child can keep the redirected error pipe open after its
        // parent exits. Reap the group before reading diagnostics so recovery
        // is not delayed by an orphan that we are about to stop anyway.
        var diagnostic = Sanitize(
            await ReadErrorAfterExit(publisher.ErrorOutput),
            publisher.Request.Command.PublishToken);
        _logger.LogWarning(
            "Viewer publisher for session {SessionId} exited unexpectedly with code {ExitCode}: {Diagnostic}",
            publisher.Request.SessionId,
            exitCode,
            diagnostic);

        if (!processGroupReaped)
        {
            await RetireExitedPublisherAsync(publisher);
            _logger.LogError(
                "Viewer publisher recovery stopped for session {SessionId} because its process group could not be reaped safely.",
                publisher.Request.SessionId);
            return;
        }

        await RecoverPublisherAsync(publisher, diagnostic);
    }

    private async Task RecoverPublisherAsync(
        ActivePublisher failedPublisher,
        string initialDiagnostic)
    {
        var diagnostic = initialDiagnostic;
        for (var restartCount = failedPublisher.RestartCount + 1;
             restartCount <= MaximumUnexpectedExitRestarts;
             restartCount++)
        {
            try
            {
                await Task.Delay(
                    RestartDelay(restartCount),
                    failedPublisher.ExpiryCancellation.Token);
            }
            catch (OperationCanceledException)
            {
                return;
            }
            catch (ObjectDisposedException)
            {
                return;
            }

            try
            {
                await _processGate.WaitAsync(
                    failedPublisher.ExpiryCancellation.Token);
            }
            catch (OperationCanceledException)
            {
                return;
            }
            catch (ObjectDisposedException)
            {
                return;
            }

            try
            {
                if (!_active.TryGetValue(
                        failedPublisher.Request.SessionId,
                        out var current)
                    || !ReferenceEquals(current, failedPublisher))
                {
                    return;
                }

                if (failedPublisher.Request.Command.ExpiresAt <= DateTimeOffset.UtcNow)
                {
                    _active.Remove(failedPublisher.Request.SessionId);
                    DisposeExitedPublisher(failedPublisher);
                    return;
                }

                if (!TryGetPublishBaseUri(out var publishBaseUri, out var error))
                {
                    diagnostic = error;
                    break;
                }

                try
                {
                    var replacement = await StartPublisherAsync(
                        failedPublisher.Request,
                        publishBaseUri,
                        failedPublisher.ExpiryCancellation.Token);
                    replacement = replacement with { RestartCount = restartCount };
                    _active[failedPublisher.Request.SessionId] = replacement;
                    DisposeExitedPublisher(failedPublisher);
                    WatchPublisher(replacement);
                    _logger.LogInformation(
                        "Viewer publisher for session {SessionId} recovered on restart {RestartCount}.",
                        failedPublisher.Request.SessionId,
                        restartCount);
                    return;
                }
                catch (OperationCanceledException)
                {
                    return;
                }
                catch (Exception exception)
                    when (exception is Win32Exception
                          or IOException
                          or InvalidOperationException
                          or TimeoutException
                          or UnauthorizedAccessException)
                {
                    diagnostic = Sanitize(
                        exception.Message,
                        failedPublisher.Request.Command.PublishToken);
                    _logger.LogWarning(
                        "Viewer publisher restart {RestartCount} of {MaximumRestarts} failed for session {SessionId}: {Diagnostic}",
                        restartCount,
                        MaximumUnexpectedExitRestarts,
                        failedPublisher.Request.SessionId,
                        diagnostic);
                }
            }
            finally
            {
                _processGate.Release();
            }
        }

        await RetireExitedPublisherAsync(failedPublisher);

        _logger.LogError(
            "Viewer publisher for session {SessionId} stopped after {MaximumRestarts} recovery attempts. Last error: {Diagnostic}",
            failedPublisher.Request.SessionId,
            MaximumUnexpectedExitRestarts,
            diagnostic);
    }

    private static TimeSpan RestartDelay(int restartCount)
    {
        return TimeSpan.FromSeconds(1 << Math.Min(restartCount - 1, 2));
    }

    private static async Task<bool> ReapProcessGroupAsync(
        int processId,
        CancellationToken cancellationToken)
    {
        if (!OperatingSystem.IsLinux() && !OperatingSystem.IsMacOS())
        {
            return true;
        }

        SendProcessGroupSignal(processId, TerminationSignal);
        if (await WaitForProcessGroupExitAsync(
                processId,
                TimeSpan.FromSeconds(1),
                cancellationToken))
        {
            return true;
        }

        SendProcessGroupSignal(processId, KillSignal);
        return await WaitForProcessGroupExitAsync(
            processId,
            TimeSpan.FromSeconds(2),
            cancellationToken);
    }

    private static async Task<bool> WaitForProcessGroupExitAsync(
        int processId,
        TimeSpan timeout,
        CancellationToken cancellationToken)
    {
        var deadline = DateTimeOffset.UtcNow + timeout;
        while (ProcessGroupExists(processId) && DateTimeOffset.UtcNow < deadline)
        {
            await Task.Delay(TimeSpan.FromMilliseconds(50), cancellationToken);
        }

        return !ProcessGroupExists(processId);
    }

    private static void SendProcessGroupSignal(int processId, int signal)
    {
        _ = SendUnixSignal(-processId, signal);
    }

    private static bool ProcessGroupExists(int processId)
    {
        if (SendUnixSignal(-processId, SignalCheck) == 0)
        {
            return true;
        }

        return Marshal.GetLastPInvokeError() != NoSuchProcessError;
    }

    private async Task RetireExitedPublisherAsync(ActivePublisher publisher)
    {
        try
        {
            await _processGate.WaitAsync();
            try
            {
                if (_active.TryGetValue(
                        publisher.Request.SessionId,
                        out var current)
                    && ReferenceEquals(current, publisher))
                {
                    _active.Remove(publisher.Request.SessionId);
                    DisposeExitedPublisher(publisher);
                }
            }
            finally
            {
                _processGate.Release();
            }
        }
        catch (ObjectDisposedException)
        {
            // Worker shutdown already owns publisher cleanup.
        }
    }

    private static int? TryGetExitCode(Process process)
    {
        try
        {
            return process.HasExited ? process.ExitCode : null;
        }
        catch (InvalidOperationException)
        {
            return null;
        }
    }

    private static void DisposeExitedPublisher(ActivePublisher publisher)
    {
        publisher.ExpiryCancellation.Cancel();
        publisher.Process.Dispose();
        publisher.ExpiryCancellation.Dispose();
    }

    private async Task DrainOutputAsync(Process process, Guid sessionId)
    {
        try
        {
            while (await process.StandardOutput.ReadLineAsync() is not null)
            {
                // Drain output so a chatty helper cannot block. Helper output
                // is deliberately not logged because the helper handles the
                // lease's publish token in memory after reading standard input.
            }
        }
        catch (Exception exception)
            when (exception is IOException or InvalidOperationException)
        {
            _logger.LogDebug(
                exception,
                "Viewer publisher output closed for session {SessionId}.",
                sessionId);
        }
    }

    private static async Task<string> ReadErrorAfterExit(Task<string> errorOutput)
    {
        try
        {
            return await errorOutput.WaitAsync(TimeSpan.FromSeconds(2));
        }
        catch (TimeoutException)
        {
            return "No diagnostic output was available.";
        }
    }

    private bool TryGetPublishBaseUri(out Uri uri, out string error)
    {
        if (!Uri.TryCreate(_options.PublishBaseUrl, UriKind.Absolute, out uri!)
            || uri.Scheme is not ("rtsp" or "rtsps")
            || string.IsNullOrWhiteSpace(uri.Host)
            || !string.IsNullOrEmpty(uri.UserInfo)
            || !string.IsNullOrEmpty(uri.Query)
            || !string.IsNullOrEmpty(uri.Fragment))
        {
            error =
                "Worker:Viewer:PublishBaseUrl must be an RTSP(S) URL without credentials, query, or fragment.";
            return false;
        }

        error = string.Empty;
        return true;
    }

    private void SetAvailability(ViewerPublisherAvailability availability)
    {
        var previous = Volatile.Read(ref _availability);
        if (SameAvailability(previous, availability))
        {
            return;
        }

        Interlocked.Exchange(ref _availability, availability);

        if (availability.IsAvailable)
        {
            _logger.LogInformation(
                "Viewer publisher is available for {Sources} with {VideoCodec}.",
                string.Join(',', availability.Sources),
                availability.VideoCodec);
        }
        else
        {
            _logger.LogWarning(
                "Viewer publishing is unavailable: {Reason}",
                availability.Reason);
        }
    }

    private static ViewerPublishResult BuildResult(
        ViewerPublishRequest request,
        string videoCodec)
    {
        return new ViewerPublishResult(
            request.SessionId,
            request.Command.LeaseId,
            request.Command.ExpiresAt,
            request.Command.Source,
            request.Command.RobotRuntimeId,
            request.Command.SourceId,
            request.Command.StreamPath,
            Ready: true,
            VideoCodec: videoCodec);
    }

    private static string Sanitize(string? value, params string[] sensitiveValues)
    {
        var text = string.IsNullOrWhiteSpace(value)
            ? "No diagnostic output was available."
            : value.ReplaceLineEndings(" ").Trim();

        foreach (var sensitiveValue in sensitiveValues)
        {
            if (!string.IsNullOrEmpty(sensitiveValue))
            {
                text = text.Replace(
                    sensitiveValue,
                    "[REDACTED]",
                    StringComparison.Ordinal);
            }
        }

        return text.Length <= 500 ? text : text[..500];
    }

    private static bool SameAvailability(
        ViewerPublisherAvailability left,
        ViewerPublisherAvailability right)
    {
        return left.IsAvailable == right.IsAvailable
            && string.Equals(left.VideoCodec, right.VideoCodec, StringComparison.Ordinal)
            && string.Equals(left.Reason, right.Reason, StringComparison.Ordinal)
            && left.Sources.SequenceEqual(right.Sources);
    }

    private static void TryKill(Process process)
    {
        try
        {
            if (!process.HasExited)
            {
                process.Kill(entireProcessTree: true);
            }
        }
        catch (InvalidOperationException)
        {
            // The process exited between the state check and kill request.
        }
    }

#pragma warning disable SYSLIB1054 // This small POSIX call does not need generated marshalling.
    [DllImport("libc", EntryPoint = "kill", SetLastError = true)]
    private static extern int SendUnixSignal(int processId, int signal);
#pragma warning restore SYSLIB1054
}
