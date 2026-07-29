using System.Text.Json;
using Microsoft.AspNetCore.SignalR;
using Microsoft.Extensions.Options;
using SwarmWorker.Configuration;
using SwarmWorker.Contracts;
using SwarmWorker.Infrastructure;
using SwarmWorker.Runtime;

namespace SwarmWorker.Services;

public sealed class BackendDisconnectSafetyMonitor : BackgroundService
{
    private static readonly TimeSpan CheckInterval = TimeSpan.FromSeconds(2);
    private const string ContainerNoLongerRunningReason =
        "Backend contact was lost, so the worker applied its local safety stop. "
        + "The latched session container is no longer running.";

    private readonly DockerSessionManager _sessions;
    private readonly IWorkerCommandHub _hub;
    private readonly BoundedCommandExecutor _commands;
    private readonly WorkerOptions _options;
    private readonly ILogger<BackendDisconnectSafetyMonitor> _logger;
    private readonly Dictionary<Guid, string> _latchedContainers = new();
    private readonly Dictionary<Guid, string> _handledContainers = new();
    private readonly Dictionary<Guid, FailSafeOperationResult> _pendingReports = new();
    private bool _outageActive;

    public BackendDisconnectSafetyMonitor(
        DockerSessionManager sessions,
        IWorkerCommandHub hub,
        BoundedCommandExecutor commands,
        IOptions<WorkerOptions> options,
        ILogger<BackendDisconnectSafetyMonitor> logger)
    {
        _sessions = sessions;
        _hub = hub;
        _commands = commands;
        _options = options.Value;
        _logger = logger;
    }

    protected override async Task ExecuteAsync(CancellationToken stoppingToken)
    {
        while (!stoppingToken.IsCancellationRequested)
        {
            try
            {
                var disconnectedFor = _hub.LastSuccessfulContactAge;
                var disconnectThreshold = TimeSpan.FromSeconds(
                    _options.BackendDisconnectEmergencyStopSeconds);
                var contactUnavailable = disconnectedFor >= disconnectThreshold;
                if (contactUnavailable && !_outageActive)
                {
                    _outageActive = true;
                    _logger.LogCritical(
                        "Backend contact has been unavailable for {Seconds:F0} seconds; applying the local session fail-safe.",
                        disconnectedFor.TotalSeconds);
                }

                if (_outageActive)
                {
                    await ApplyFailSafeToSessions(
                        includeUnlatchedSessions: contactUnavailable,
                        stoppingToken);
                }

                if (_pendingReports.Count > 0 && _hub.IsConnected)
                {
                    await ReportPendingResults(stoppingToken);
                }

                if (_outageActive
                    && !contactUnavailable
                    && _pendingReports.Count == 0
                    && AllLatchedContainersHandled())
                {
                    _outageActive = false;
                    _latchedContainers.Clear();
                    _handledContainers.Clear();
                    _logger.LogInformation(
                        "Backend contact recovered after the local session fail-safe.");
                }
            }
            catch (OperationCanceledException) when (stoppingToken.IsCancellationRequested)
            {
                break;
            }
            catch (Exception exception)
            {
                _logger.LogError(
                    exception,
                    "The backend disconnect safety monitor failed; it will retry.");
            }

            await Task.Delay(CheckInterval, stoppingToken);
        }
    }

    private async Task ApplyFailSafeToSessions(
        bool includeUnlatchedSessions,
        CancellationToken cancellationToken)
    {
        var runningSessions = await _sessions.GetManagedSessionsAsync(cancellationToken);
        foreach (var latched in _latchedContainers.ToArray())
        {
            if (_handledContainers.TryGetValue(
                    latched.Key,
                    out var handledContainer)
                && handledContainer == latched.Value)
            {
                continue;
            }

            var exactContainer = runningSessions.SingleOrDefault(session =>
                session.SessionId == latched.Key
                && session.ContainerId == latched.Value);
            if (exactContainer is not null && exactContainer.Running)
            {
                continue;
            }

            _handledContainers[latched.Key] = latched.Value;
            _pendingReports[latched.Key] = new FailSafeOperationResult(
                latched.Key,
                EmergencyStopConfirmed: false,
                ContainerStopped: true,
                ContainerNoLongerRunningReason);
        }

        var candidates = runningSessions
            .Where(session => session.Running
                && (includeUnlatchedSessions
                    || _latchedContainers.ContainsKey(session.SessionId))
                && (!_handledContainers.TryGetValue(
                        session.SessionId,
                        out var handledContainer)
                    || handledContainer != session.ContainerId))
            .ToArray();

        foreach (var session in candidates)
        {
            _latchedContainers[session.SessionId] = session.ContainerId;
            _commands.RequestLocalEmergencyStop(session.SessionId);
            try
            {
                var result = await _sessions.ApplyDisconnectFailSafeAsync(
                    session,
                    cancellationToken);
                _handledContainers[session.SessionId] = session.ContainerId;
                _pendingReports[session.SessionId] = result;
            }
            catch (Exception exception)
                when (exception is DockerCliException
                      or TimeoutException
                      or InvalidOperationException)
            {
                _logger.LogCritical(
                    exception,
                    "Could not apply the disconnect fail-safe to session {SessionId}; retrying on the next check.",
                    session.SessionId);
            }
        }
    }

    private bool AllLatchedContainersHandled()
    {
        return _latchedContainers.All(latched =>
            _handledContainers.TryGetValue(latched.Key, out var handledContainer)
            && handledContainer == latched.Value);
    }

    private async Task ReportPendingResults(CancellationToken cancellationToken)
    {
        foreach (var result in _pendingReports.Values.ToArray())
        {
            using var timeout = CancellationTokenSource.CreateLinkedTokenSource(
                cancellationToken);
            timeout.CancelAfter(TimeSpan.FromSeconds(8));

            try
            {
                if (result.EmergencyStopConfirmed)
                {
                    await _hub.ReportEmergencyStopAsync(
                        new WorkerEmergencyStopReport(
                            result.SessionId,
                            Active: true,
                            result.Reason),
                        timeout.Token);
                }
                else
                {
                    await _hub.ReportSessionEventAsync(
                        new SessionEventReport(
                            result.SessionId,
                            "Failed",
                            result.Reason,
                            JsonSerializer.SerializeToElement(new
                            {
                                failSafe = true,
                                result.ContainerStopped
                            })),
                        timeout.Token);
                }

                _pendingReports.Remove(result.SessionId);
            }
            catch (HubException exception)
            {
                _pendingReports.Remove(result.SessionId);
                _logger.LogWarning(
                    exception,
                    "Backend no longer accepts the fail-safe report for session {SessionId}.",
                    result.SessionId);
            }
            catch (OperationCanceledException) when (!cancellationToken.IsCancellationRequested)
            {
                return;
            }
        }
    }
}
