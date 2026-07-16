using System.Collections.Concurrent;
using System.Text.Json;
using System.Threading.Channels;
using Microsoft.Extensions.Options;
using SwarmWorker.Configuration;
using SwarmWorker.Contracts;
using SwarmWorker.Runtime;

namespace SwarmWorker.Services;

public sealed class BoundedCommandExecutor : IHostedService
{
    private sealed record ActiveSessionOperation(
        string CommandType,
        CancellationTokenSource Cancellation);

    private readonly Channel<WorkerCommandEnvelope> _commands;
    private readonly ConcurrentDictionary<string, byte> _scheduled =
        new(StringComparer.Ordinal);
    private readonly ConcurrentDictionary<Guid, SemaphoreSlim> _sessionLocks = new();
    private readonly ConcurrentDictionary<Guid, SemaphoreSlim> _emergencyLocks = new();
    private readonly ConcurrentDictionary<Guid, ActiveSessionOperation> _activeOperations = new();
    private readonly ConcurrentDictionary<Guid, Task> _urgentCommands = new();
    private readonly ConcurrentDictionary<Guid, byte> _emergencyRequested = new();
    private readonly CancellationTokenSource _abort = new();
    private readonly SessionCommandHandler _handler;
    private readonly WorkerHubConnection _hub;
    private readonly TaskStatusTracker _taskStatusTracker;
    private readonly WorkerOptions _options;
    private readonly ILogger<BoundedCommandExecutor> _logger;
    private Task[] _consumers = Array.Empty<Task>();
    private volatile bool _accepting;

    public BoundedCommandExecutor(
        SessionCommandHandler handler,
        WorkerHubConnection hub,
        TaskStatusTracker taskStatusTracker,
        IOptions<WorkerOptions> options,
        ILogger<BoundedCommandExecutor> logger)
    {
        _handler = handler;
        _hub = hub;
        _taskStatusTracker = taskStatusTracker;
        _options = options.Value;
        _logger = logger;
        _commands = Channel.CreateBounded<WorkerCommandEnvelope>(
            new BoundedChannelOptions(_options.MaxQueuedCommands)
            {
                FullMode = BoundedChannelFullMode.Wait,
                SingleWriter = false,
                SingleReader = _options.MaxParallelCommands == 1,
                AllowSynchronousContinuations = false
            });
    }

    public Task StartAsync(CancellationToken cancellationToken)
    {
        _accepting = true;
        _consumers = Enumerable.Range(0, _options.MaxParallelCommands)
            .Select(index => ConsumeAsync(index, _abort.Token))
            .ToArray();
        return Task.CompletedTask;
    }

    public async Task StopAsync(CancellationToken cancellationToken)
    {
        _accepting = false;
        _commands.Writer.TryComplete();

        try
        {
            await WaitForWorkToFinish().WaitAsync(
                TimeSpan.FromSeconds(_options.ShutdownDrainSeconds),
                cancellationToken);
        }
        catch (Exception exception)
            when (exception is TimeoutException or OperationCanceledException)
        {
            _logger.LogWarning(
                "Command executor did not drain within {DrainSeconds} seconds; cancelling remaining work.",
                _options.ShutdownDrainSeconds);
            _abort.Cancel();
            try
            {
                await WaitForWorkToFinish();
            }
            catch (OperationCanceledException)
            {
                // Expected after the drain deadline.
            }
        }
        finally
        {
            foreach (var sessionLock in _sessionLocks.Values)
            {
                sessionLock.Dispose();
            }

            foreach (var emergencyLock in _emergencyLocks.Values)
            {
                emergencyLock.Dispose();
            }

            _abort.Dispose();
        }
    }

    public async Task<bool> EnqueueAsync(
        WorkerCommandEnvelope command,
        CancellationToken cancellationToken)
    {
        if (!_accepting)
        {
            return false;
        }

        var deduplicationKey = GetDeduplicationKey(command);
        if (!_scheduled.TryAdd(deduplicationKey, 0))
        {
            return false;
        }

        try
        {
            if (command.Type == "EmergencyStop")
            {
                _emergencyRequested[command.SessionId] = 0;
                CancelActiveOperation(command.SessionId);
                var task = ExecuteEmergencyCommandAsync(command, _abort.Token);
                _urgentCommands[command.Id] = task;
                _ = ObserveUrgentCommandAsync(command, task);
                return true;
            }

            await _commands.Writer.WriteAsync(command, cancellationToken);
            return true;
        }
        catch
        {
            _scheduled.TryRemove(deduplicationKey, out _);
            throw;
        }
    }

    private async Task ConsumeAsync(int consumerId, CancellationToken cancellationToken)
    {
        await foreach (var command in _commands.Reader.ReadAllAsync(cancellationToken))
        {
            var sessionLock = _sessionLocks.GetOrAdd(
                command.SessionId,
                static _ => new SemaphoreSlim(1, 1));

            await sessionLock.WaitAsync(cancellationToken);
            using var operationCancellation =
                CancellationTokenSource.CreateLinkedTokenSource(cancellationToken);
            var operation = new ActiveSessionOperation(
                command.Type,
                operationCancellation);
            if (!_activeOperations.TryAdd(command.SessionId, operation))
            {
                operationCancellation.Dispose();
                sessionLock.Release();
                throw new InvalidOperationException(
                    $"Session {command.SessionId} already has an active command.");
            }

            try
            {
                await ExecuteCommandAsync(
                    command,
                    consumerId,
                    operationCancellation.Token,
                    cancellationToken);
            }
            finally
            {
                _activeOperations.TryRemove(command.SessionId, out _);
                _scheduled.TryRemove(GetDeduplicationKey(command), out _);
                sessionLock.Release();
            }
        }
    }

    private async Task ExecuteEmergencyCommandAsync(
        WorkerCommandEnvelope command,
        CancellationToken cancellationToken)
    {
        var emergencyLock = _emergencyLocks.GetOrAdd(
            command.SessionId,
            static _ => new SemaphoreSlim(1, 1));
        await emergencyLock.WaitAsync(cancellationToken);
        try
        {
            await ExecuteCommandAsync(
                command,
                consumerId: -1,
                cancellationToken,
                cancellationToken);
        }
        finally
        {
            emergencyLock.Release();
        }
    }

    private async Task ObserveUrgentCommandAsync(
        WorkerCommandEnvelope command,
        Task task)
    {
        try
        {
            await task;
        }
        catch (OperationCanceledException) when (_abort.IsCancellationRequested)
        {
            // Expected during worker shutdown.
        }
        catch (Exception exception)
        {
            _logger.LogCritical(
                exception,
                "Urgent command {CommandId} terminated outside the normal reporting path.",
                command.Id);
        }
        finally
        {
            _urgentCommands.TryRemove(command.Id, out _);
            _scheduled.TryRemove(GetDeduplicationKey(command), out _);
        }
    }

    private async Task<bool> ExecuteCommandAsync(
        WorkerCommandEnvelope command,
        int consumerId,
        CancellationToken executionToken,
        CancellationToken reportingToken)
    {
        _logger.LogInformation(
            "Consumer {ConsumerId} executing command {CommandId} ({CommandType}) for session {SessionId}.",
            consumerId,
            command.Id,
            command.Type,
            command.SessionId);

        try
        {
            await _hub.AcknowledgeCommandAsync(command.Id, executionToken);
            await _hub.MarkCommandRunningAsync(command.Id, executionToken);

            if (_emergencyRequested.ContainsKey(command.SessionId)
                && command.Type is not "EmergencyStop"
                    and not "ResetEmergencyStop"
                    and not "StopSession")
            {
                throw new InvalidOperationException(
                    "The command was blocked because an emergency stop is active.");
            }

            var startingState = GetStartingState(command.Type);
            if (startingState is not null)
            {
                await _hub.ReportSessionEventAsync(
                    new SessionEventReport(
                        command.SessionId,
                        startingState,
                        null,
                        JsonSerializer.SerializeToElement(new
                        {
                            commandId = command.Id,
                            command.Sequence
                        })),
                    executionToken);
            }

            var execution = await _handler.ExecuteAsync(command, executionToken);

            if (execution.CompletedSessionState is not null)
            {
                await _hub.ReportSessionEventAsync(
                    new SessionEventReport(
                        command.SessionId,
                        execution.CompletedSessionState,
                        null,
                        execution.Result),
                    executionToken);
            }

            if (execution.ImmediateTaskEvent is not null)
            {
                await _hub.ReportTaskEventAsync(
                    execution.ImmediateTaskEvent,
                    executionToken);
                _taskStatusTracker.Record(execution.ImmediateTaskEvent);
            }

            await _hub.CompleteCommandAsync(
                new WorkerCommandCompletionRequest(command.Id, execution.Result),
                executionToken);

            if (command.Type == "StopSession")
            {
                _taskStatusTracker.RemoveSession(command.SessionId);
                _emergencyRequested.TryRemove(command.SessionId, out _);
            }
            else if (command.Type == "ResetEmergencyStop")
            {
                _emergencyRequested.TryRemove(command.SessionId, out _);
            }

            _logger.LogInformation(
                "Command {CommandId} completed for session {SessionId}.",
                command.Id,
                command.SessionId);
            return true;
        }
        catch (OperationCanceledException) when (reportingToken.IsCancellationRequested)
        {
            throw;
        }
        catch (Exception exception)
        {
            var error = exception is OperationCanceledException
                        && executionToken.IsCancellationRequested
                ? "Command was preempted by an emergency stop."
                : SanitizeError(exception);
            _logger.LogError(
                exception,
                "Command {CommandId} failed for session {SessionId}.",
                command.Id,
                command.SessionId);

            try
            {
                if (command.Type is "ProvisionSession"
                    or "UpdateFleet"
                    or "EmergencyStop")
                {
                    try
                    {
                        await _handler.CleanupSessionAsync(
                            command.SessionId,
                            reportingToken);
                    }
                    catch (Exception cleanupException)
                        when (cleanupException is not OperationCanceledException)
                    {
                        _logger.LogError(
                            cleanupException,
                            "Failed to clean up a partially provisioned session {SessionId}.",
                            command.SessionId);
                    }
                }

                if (IsSessionFatal(command.Type))
                {
                    await _hub.ReportSessionEventAsync(
                        new SessionEventReport(
                            command.SessionId,
                            "Failed",
                            error,
                            JsonSerializer.SerializeToElement(new { commandId = command.Id })),
                        reportingToken);
                }

                await _hub.FailCommandAsync(
                    new WorkerCommandFailureRequest(command.Id, error),
                    reportingToken);
                return false;
            }
            catch (Exception reportingException)
                when (reportingException is not OperationCanceledException)
            {
                _logger.LogCritical(
                    reportingException,
                    "Unable to report failure for command {CommandId}; the backend will redeliver it.",
                    command.Id);
                return false;
            }
        }
    }

    private void CancelActiveOperation(Guid sessionId)
    {
        if (!_activeOperations.TryGetValue(sessionId, out var operation)
            || operation.CommandType == "StopSession")
        {
            return;
        }

        try
        {
            operation.Cancellation.Cancel();
        }
        catch (ObjectDisposedException)
        {
            // The command finished between the lookup and cancellation.
        }
    }

    private async Task WaitForWorkToFinish()
    {
        await Task.WhenAll(_consumers);
        while (!_urgentCommands.IsEmpty)
        {
            await Task.WhenAll(_urgentCommands.Values.ToArray());
        }
    }

    private static string GetDeduplicationKey(WorkerCommandEnvelope command)
    {
        var key = string.IsNullOrWhiteSpace(command.IdempotencyKey)
            ? command.Id.ToString("D")
            : command.IdempotencyKey;
        return $"{command.SessionId:D}:{key}";
    }

    private static string? GetStartingState(string commandType)
    {
        return commandType switch
        {
            "ProvisionSession" => "Provisioning",
            _ => null
        };
    }

    private static bool IsSessionFatal(string commandType)
    {
        return commandType is "ProvisionSession"
            or "UpdateFleet"
            or "EmergencyStop"
            or "StopSession";
    }

    private static string SanitizeError(Exception exception)
    {
        var message = exception.Message.ReplaceLineEndings(" ").Trim();
        return message.Length <= 2000 ? message : message[..2000];
    }
}
