using System.Net.WebSockets;
using System.Text.Json;
using Microsoft.AspNetCore.Http.Connections;
using Microsoft.AspNetCore.SignalR;
using Microsoft.AspNetCore.SignalR.Client;
using Microsoft.Extensions.Options;
using SwarmWorker.Configuration;
using SwarmWorker.Contracts;

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
    private long _connectionVersion;
    private long _lastSuccessfulContactTicks = DateTime.UtcNow.Ticks;

    public WorkerHubConnection(
        IOptions<WorkerOptions> options,
        ILogger<WorkerHubConnection> logger)
    {
        var workerOptions = options.Value;
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
        _connection.Reconnecting += exception =>
        {
            _logger.LogWarning(
                "Worker hub connection lost; SignalR is reconnecting ({ErrorType}).",
                exception?.GetType().Name ?? "unknown");
            return Task.CompletedTask;
        };
        _connection.Reconnected += connectionId =>
        {
            Interlocked.Increment(ref _connectionVersion);
            MarkSuccessfulContact();
            NotifyCommandAvailable();
            _logger.LogInformation(
                "Worker hub reconnected with connection {ConnectionId}.",
                connectionId);
            return Task.CompletedTask;
        };
        _connection.Closed += exception =>
        {
            _logger.LogWarning(
                "Worker hub connection closed; the worker loop will reconnect ({ErrorType}).",
                exception?.GetType().Name ?? "unknown");
            return Task.CompletedTask;
        };
    }

    public long ConnectionVersion => Interlocked.Read(ref _connectionVersion);
    public bool IsConnected => _connection.State == HubConnectionState.Connected;
    public DateTime LastSuccessfulContactUtc =>
        new(Interlocked.Read(ref _lastSuccessfulContactTicks), DateTimeKind.Utc);

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
