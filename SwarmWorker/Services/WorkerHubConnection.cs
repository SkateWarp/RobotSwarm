using System.Net.WebSockets;
using System.Text.Json;
using Microsoft.AspNetCore.Http.Connections;
using Microsoft.AspNetCore.SignalR;
using Microsoft.AspNetCore.SignalR.Client;
using Microsoft.Extensions.Options;
using SwarmWorker.Configuration;
using SwarmWorker.Contracts;
using SwarmWorker.Runtime;

namespace SwarmWorker.Services;

public interface IWorkerCommandHub
{
    bool IsConnected { get; }
    DateTime LastSuccessfulContactUtc { get; }

    Task AcknowledgeCommandAsync(Guid commandId, CancellationToken cancellationToken);
    Task MarkCommandRunningAsync(Guid commandId, CancellationToken cancellationToken);
    Task CompleteCommandAsync(
        WorkerCommandCompletionRequest request,
        CancellationToken cancellationToken);
    Task FailCommandAsync(
        WorkerCommandFailureRequest request,
        CancellationToken cancellationToken);
    Task ReportEmergencyStopAsync(
        WorkerEmergencyStopReport report,
        CancellationToken cancellationToken);
    Task ReportSessionEventAsync(
        SessionEventReport report,
        CancellationToken cancellationToken);
    Task ReportTaskEventAsync(
        TaskEventReport report,
        CancellationToken cancellationToken);
}

public sealed class WorkerHubConnection : IWorkerCommandHub, IAsyncDisposable
{
    private static readonly TimeSpan ViewerInputReleaseTimeout = TimeSpan.FromSeconds(12);
    private static readonly TimeSpan[] ReconnectDelays =
    {
        TimeSpan.Zero,
        TimeSpan.FromSeconds(2),
        TimeSpan.FromSeconds(5),
        TimeSpan.FromSeconds(10),
        TimeSpan.FromSeconds(30)
    };

    private readonly HubConnection _connection;
    private readonly SemaphoreSlim _connectionGate = new(1, 1);
    private readonly SemaphoreSlim _commandSignal = new(0, 1);
    private readonly ILogger<WorkerHubConnection> _logger;
    private readonly IViewerPublisher _viewerPublisher;
    private readonly object _viewerInputReleaseGate = new();
    private int _disposeStarted;
    private bool _viewerInputsReleasedForOutage;
    private bool _viewerInputReleaseRetryRequested;
    private Task<bool>? _viewerInputReleaseAttempt;
    private long _viewerInputReleaseGeneration;
    private long _connectionVersion;
    private long _lastSuccessfulContactTicks = DateTime.UtcNow.Ticks;

    public WorkerHubConnection(
        IOptions<WorkerOptions> options,
        IViewerPublisher viewerPublisher,
        ILogger<WorkerHubConnection> logger)
    {
        var workerOptions = options.Value;
        _viewerPublisher = viewerPublisher;
        _logger = logger;

        _connection = new HubConnectionBuilder()
            .WithUrl(
                workerOptions.GetWorkerHubUri(),
                connectionOptions =>
                {
                    connectionOptions.Transports =
                        HttpTransportType.WebSockets | HttpTransportType.LongPolling;
                    connectionOptions.AccessTokenProvider =
                        () => Task.FromResult<string?>(workerOptions.GetAccessToken());
                })
            .WithAutomaticReconnect(ReconnectDelays)
            .AddJsonProtocol(protocolOptions =>
            {
                protocolOptions.PayloadSerializerOptions.PropertyNamingPolicy =
                    JsonNamingPolicy.CamelCase;
                protocolOptions.PayloadSerializerOptions.PropertyNameCaseInsensitive = true;
            })
            .Build();

        _connection.On(
            "CommandAvailable",
            () =>
            {
                NotifyCommandAvailable();
                return Task.CompletedTask;
            });
        _connection.On<ViewerInputEnvelope>(
            "ViewerInput",
            request => DispatchViewerInputAsync(request, CancellationToken.None));
        _connection.On<ViewerInputReleaseEnvelope>(
            "ViewerInputRelease",
            request => DispatchViewerInputReleaseAsync(request, CancellationToken.None));
        _connection.Reconnecting += async exception =>
        {
            _logger.LogWarning(
                "Worker hub connection lost; SignalR is reconnecting ({ErrorType}).",
                exception?.GetType().Name ?? "unknown");
            await ReleaseViewerInputsForOutageAsync();
        };
        _connection.Reconnected += connectionId =>
        {
            ResetViewerInputReleaseState();
            Interlocked.Increment(ref _connectionVersion);
            MarkSuccessfulContact();
            NotifyCommandAvailable();
            _logger.LogInformation(
                "Worker hub reconnected with connection {ConnectionId}.",
                connectionId);
            return Task.CompletedTask;
        };
        _connection.Closed += async exception =>
        {
            _logger.LogWarning(
                "Worker hub connection closed; the worker loop will reconnect ({ErrorType}).",
                exception?.GetType().Name ?? "unknown");
            await ReleaseViewerInputsForOutageAsync();
        };
    }

    public long ConnectionVersion => Interlocked.Read(ref _connectionVersion);
    public bool IsConnected => _connection.State == HubConnectionState.Connected;
    public DateTime LastSuccessfulContactUtc =>
        new(Interlocked.Read(ref _lastSuccessfulContactTicks), DateTimeKind.Utc);

    internal async Task DispatchViewerInputAsync(
        ViewerInputEnvelope request,
        CancellationToken cancellationToken)
    {
        try
        {
            await _viewerPublisher.SendInputAsync(request, cancellationToken);
        }
        catch (Exception exception)
            when (exception is InvalidOperationException
                  or IOException
                  or ObjectDisposedException
                  or TimeoutException)
        {
            _logger.LogDebug(
                "Dropped viewer input because its active publisher no longer accepts it ({ErrorType}).",
                exception.GetType().Name);
        }
    }

    internal async Task DispatchViewerInputReleaseAsync(
        ViewerInputReleaseEnvelope request,
        CancellationToken cancellationToken)
    {
        if (request.SessionId == Guid.Empty || request.LeaseId == Guid.Empty)
        {
            _logger.LogWarning(
                "Dropped viewer input release because its identifiers are invalid.");
            return;
        }

        try
        {
            await _viewerPublisher.ReleaseInputAsync(
                request.SessionId,
                request.LeaseId,
                cancellationToken);
        }
        catch (Exception exception)
            when (exception is InvalidOperationException
                  or IOException
                  or ObjectDisposedException
                  or TimeoutException)
        {
            _logger.LogDebug(
                "Dropped viewer input release because its active publisher no longer accepts it ({ErrorType}).",
                exception.GetType().Name);
        }
    }

    internal async Task ReleaseViewerInputsForOutageAsync(
        TimeSpan? timeoutOverride = null)
    {
        var timeoutDuration = timeoutOverride ?? ViewerInputReleaseTimeout;
        Task<bool> attempt;
        lock (_viewerInputReleaseGate)
        {
            if (_viewerInputsReleasedForOutage)
            {
                return;
            }

            if (_viewerInputReleaseAttempt is not null)
            {
                // Reconnecting and Closed may report the same outage. Coalesce
                // them, but remember that the later event should retry if the
                // attempt already in progress ultimately fails.
                _viewerInputReleaseRetryRequested = true;
                return;
            }

            var completion = new TaskCompletionSource<bool>(
                TaskCreationOptions.RunContinuationsAsynchronously);
            attempt = completion.Task;
            _viewerInputReleaseAttempt = attempt;
            var generation = _viewerInputReleaseGeneration;
            _ = RunViewerInputReleaseAttemptAsync(
                generation,
                completion,
                timeoutDuration);
        }

        using var callbackTimeout = new CancellationTokenSource(timeoutDuration);
        try
        {
            await attempt.WaitAsync(callbackTimeout.Token);
        }
        catch (OperationCanceledException)
            when (callbackTimeout.IsCancellationRequested)
        {
            _logger.LogError(
                "Viewer input fail-closed cleanup is still running after the hub-outage timeout.");
        }
    }

    private async Task RunViewerInputReleaseAttemptAsync(
        long generation,
        TaskCompletionSource<bool> completion,
        TimeSpan timeoutDuration)
    {
        await Task.Yield();
        var succeeded = false;
        using var timeout = new CancellationTokenSource(timeoutDuration);
        try
        {
            await _viewerPublisher.ReleaseAllInputsAsync(timeout.Token);
            succeeded = true;
        }
        catch (Exception exception)
        {
            _logger.LogError(
                exception,
                "Unable to release viewer input after losing the worker hub connection.");
        }

        var retry = false;
        lock (_viewerInputReleaseGate)
        {
            if (generation == _viewerInputReleaseGeneration
                && ReferenceEquals(_viewerInputReleaseAttempt, completion.Task))
            {
                _viewerInputReleaseAttempt = null;
                if (succeeded)
                {
                    _viewerInputsReleasedForOutage = true;
                    _viewerInputReleaseRetryRequested = false;
                }
                else
                {
                    retry = _viewerInputReleaseRetryRequested;
                    _viewerInputReleaseRetryRequested = false;
                }
            }
        }

        completion.TrySetResult(succeeded);
        if (retry)
        {
            _ = ReleaseViewerInputsForOutageAsync();
        }
    }

    private void ResetViewerInputReleaseState()
    {
        lock (_viewerInputReleaseGate)
        {
            _viewerInputReleaseGeneration++;
            _viewerInputsReleasedForOutage = false;
            _viewerInputReleaseRetryRequested = false;
            _viewerInputReleaseAttempt = null;
        }
    }

    public async Task EnsureConnectedAsync(CancellationToken cancellationToken)
    {
        while (_connection.State != HubConnectionState.Connected)
        {
            cancellationToken.ThrowIfCancellationRequested();

            await _connectionGate.WaitAsync(cancellationToken);
            try
            {
                if (_connection.State == HubConnectionState.Disconnected)
                {
                    try
                    {
                        await _connection.StartAsync(cancellationToken);
                        ResetViewerInputReleaseState();
                        Interlocked.Increment(ref _connectionVersion);
                        MarkSuccessfulContact();
                        NotifyCommandAvailable();
                        _logger.LogInformation("Connected to the worker control hub.");
                    }
                    catch (Exception exception) when (IsTransient(exception))
                    {
                        _logger.LogWarning(
                            "Unable to connect to the worker hub; retrying ({ErrorType}).",
                            exception.GetType().Name);
                    }
                }
            }
            finally
            {
                _connectionGate.Release();
            }

            if (_connection.State != HubConnectionState.Connected)
            {
                await Task.Delay(TimeSpan.FromSeconds(2), cancellationToken);
            }
        }
    }

    public Task<WorkerRegistrationResponse> RegisterAsync(
        WorkerRegistrationRequest request,
        CancellationToken cancellationToken) =>
        InvokeReliableAsync<WorkerRegistrationResponse>(
            "Register",
            new object?[] { request },
            cancellationToken);

    public Task<WorkerRegistrationResponse> HeartbeatAsync(
        WorkerHeartbeatRequest request,
        CancellationToken cancellationToken) =>
        InvokeReliableAsync<WorkerRegistrationResponse>(
            "Heartbeat",
            new object?[] { request },
            cancellationToken);

    public Task<IReadOnlyList<WorkerCommandEnvelope>> PullPendingCommandsAsync(
        int maxCount,
        CancellationToken cancellationToken) =>
        InvokeReliableAsync<IReadOnlyList<WorkerCommandEnvelope>>(
            "PullPendingCommands",
            new object?[] { maxCount },
            cancellationToken);

    public Task AcknowledgeCommandAsync(Guid commandId, CancellationToken cancellationToken) =>
        InvokeReliableAsync(
            "AcknowledgeCommand",
            new object?[] { commandId },
            cancellationToken);

    public Task MarkCommandRunningAsync(Guid commandId, CancellationToken cancellationToken) =>
        InvokeReliableAsync(
            "MarkCommandRunning",
            new object?[] { commandId },
            cancellationToken);

    public Task CompleteCommandAsync(
        WorkerCommandCompletionRequest request,
        CancellationToken cancellationToken) =>
        InvokeReliableAsync(
            "CompleteCommand",
            new object?[] { request },
            cancellationToken);

    public Task FailCommandAsync(
        WorkerCommandFailureRequest request,
        CancellationToken cancellationToken) =>
        InvokeReliableAsync(
            "FailCommand",
            new object?[] { request },
            cancellationToken);

    public async Task ReportEmergencyStopAsync(
        WorkerEmergencyStopReport report,
        CancellationToken cancellationToken)
    {
        _ = await InvokeReliableAsync<JsonElement>(
            "ReportEmergencyStop",
            new object?[] { report },
            cancellationToken);
    }

    public async Task ReportSessionEventAsync(
        SessionEventReport report,
        CancellationToken cancellationToken)
    {
        _ = await InvokeReliableAsync<JsonElement>(
            "ReportSessionEvent",
            new object?[] { report },
            cancellationToken);
    }

    public async Task ReportTaskEventAsync(
        TaskEventReport report,
        CancellationToken cancellationToken)
    {
        _ = await InvokeReliableAsync<JsonElement>(
            "ReportTaskEvent",
            new object?[] { report },
            cancellationToken);
    }

    public async Task<bool> WaitForCommandSignalAsync(
        TimeSpan timeout,
        CancellationToken cancellationToken)
    {
        return await _commandSignal.WaitAsync(timeout, cancellationToken);
    }

    public async ValueTask DisposeAsync()
    {
        if (Interlocked.Exchange(ref _disposeStarted, 1) != 0)
        {
            return;
        }

        try
        {
            if (_connection.State != HubConnectionState.Disconnected)
            {
                await _connection.StopAsync();
            }
        }
        finally
        {
            await _connection.DisposeAsync();
            _connectionGate.Dispose();
            _commandSignal.Dispose();
        }
    }

    private async Task<T> InvokeReliableAsync<T>(
        string methodName,
        object?[] arguments,
        CancellationToken cancellationToken)
    {
        while (true)
        {
            await EnsureConnectedAsync(cancellationToken);
            try
            {
                var result = await _connection.InvokeCoreAsync<T>(
                    methodName,
                    arguments,
                    cancellationToken);
                MarkSuccessfulContact();
                return result;
            }
            catch (HubException)
            {
                throw;
            }
            catch (Exception exception) when (IsTransient(exception))
            {
                _logger.LogWarning(
                    "Transient failure invoking worker hub method {Method}; retrying ({ErrorType}).",
                    methodName,
                    exception.GetType().Name);
                await Task.Delay(TimeSpan.FromSeconds(2), cancellationToken);
            }
        }
    }

    private async Task InvokeReliableAsync(
        string methodName,
        object?[] arguments,
        CancellationToken cancellationToken)
    {
        while (true)
        {
            await EnsureConnectedAsync(cancellationToken);
            try
            {
                await _connection.InvokeCoreAsync(
                    methodName,
                    arguments,
                    cancellationToken);
                MarkSuccessfulContact();
                return;
            }
            catch (HubException)
            {
                throw;
            }
            catch (Exception exception) when (IsTransient(exception))
            {
                _logger.LogWarning(
                    "Transient failure invoking worker hub method {Method}; retrying ({ErrorType}).",
                    methodName,
                    exception.GetType().Name);
                await Task.Delay(TimeSpan.FromSeconds(2), cancellationToken);
            }
        }
    }

    private void NotifyCommandAvailable()
    {
        try
        {
            _commandSignal.Release();
        }
        catch (SemaphoreFullException)
        {
            // A pending wake-up is already queued.
        }
    }

    private void MarkSuccessfulContact()
    {
        Interlocked.Exchange(
            ref _lastSuccessfulContactTicks,
            DateTime.UtcNow.Ticks);
    }

    private static bool IsTransient(Exception exception)
    {
        return exception is HttpRequestException
            or IOException
            or TimeoutException
            or WebSocketException
            or InvalidOperationException
            or OperationCanceledException;
    }
}
