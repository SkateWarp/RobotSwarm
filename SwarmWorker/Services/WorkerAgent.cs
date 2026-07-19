using System.Text.Json;
using Microsoft.AspNetCore.SignalR;
using Microsoft.Extensions.Options;
using SwarmWorker.Configuration;
using SwarmWorker.Contracts;
using SwarmWorker.Runtime;

namespace SwarmWorker.Services;

public sealed class WorkerAgent : BackgroundService
{
    private static readonly TimeSpan HeartbeatInterval = TimeSpan.FromSeconds(10);

    private readonly WorkerHubConnection _hub;
    private readonly BoundedCommandExecutor _executor;
    private readonly DockerSessionManager _sessions;
    private readonly IViewerPublisher _viewerPublisher;
    private readonly WorkerOptions _options;
    private readonly ILogger<WorkerAgent> _logger;

    public WorkerAgent(
        WorkerHubConnection hub,
        BoundedCommandExecutor executor,
        DockerSessionManager sessions,
        IViewerPublisher viewerPublisher,
        IOptions<WorkerOptions> options,
        ILogger<WorkerAgent> logger)
    {
        _hub = hub;
        _executor = executor;
        _sessions = sessions;
        _viewerPublisher = viewerPublisher;
        _options = options.Value;
        _logger = logger;
    }

    protected override async Task ExecuteAsync(CancellationToken stoppingToken)
    {
        while (!stoppingToken.IsCancellationRequested)
        {
            try
            {
                await _sessions.ReconcileAsync(stoppingToken);
                break;
            }
            catch (OperationCanceledException) when (stoppingToken.IsCancellationRequested)
            {
                return;
            }
            catch (Exception exception)
            {
                _logger.LogError(
                    exception,
                    "Docker is not ready; session reconciliation will retry.");
                await Task.Delay(TimeSpan.FromSeconds(5), stoppingToken);
            }
        }

        var registeredConnectionVersion = -1L;
        var readyConnectionVersion = -1L;
        var nextHeartbeat = DateTime.MinValue;

        while (!stoppingToken.IsCancellationRequested)
        {
            try
            {
                await _hub.EnsureConnectedAsync(stoppingToken);
                if (registeredConnectionVersion != _hub.ConnectionVersion
                    || DateTime.UtcNow >= nextHeartbeat)
                {
                    await _viewerPublisher.RefreshAvailabilityAsync(stoppingToken);
                }

                if (registeredConnectionVersion != _hub.ConnectionVersion)
                {
                    var registration = await _hub.RegisterAsync(
                        new WorkerRegistrationRequest(
                            _options.ImageVersion,
                            BuildCapabilities()),
                        stoppingToken);
                    ValidateRegistration(registration);
                    registeredConnectionVersion = _hub.ConnectionVersion;
                    nextHeartbeat = DateTime.MinValue;
                }

                if (DateTime.UtcNow >= nextHeartbeat)
                {
                    await SendHeartbeatAsync(stoppingToken);
                    if (readyConnectionVersion != _hub.ConnectionVersion)
                    {
                        readyConnectionVersion = _hub.ConnectionVersion;
                        _logger.LogInformation(
                            "Worker ready for control-plane commands. ImageVersion={ImageVersion}",
                            _options.ImageVersion);
                    }

                    nextHeartbeat = DateTime.UtcNow + HeartbeatInterval;
                }

                var commands = await _hub.PullPendingCommandsAsync(
                    _options.PullBatchSize,
                    stoppingToken);
                foreach (var command in commands)
                {
                    await _executor.EnqueueAsync(command, stoppingToken);
                }
            }
            catch (HubException exception)
            {
                _logger.LogError(
                    exception,
                    "Worker hub rejected a control-plane operation.");
                await Task.Delay(TimeSpan.FromSeconds(5), stoppingToken);
            }
            catch (OperationCanceledException) when (stoppingToken.IsCancellationRequested)
            {
                break;
            }
            catch (Exception exception)
            {
                _logger.LogError(
                    exception,
                    "Worker control loop failed; retrying without stopping managed sessions.");
                await Task.Delay(TimeSpan.FromSeconds(5), stoppingToken);
            }

            var untilHeartbeat = nextHeartbeat <= DateTime.UtcNow
                ? TimeSpan.Zero
                : nextHeartbeat - DateTime.UtcNow;
            var wait = TimeSpan.FromSeconds(_options.CommandPollIntervalSeconds);
            if (untilHeartbeat > TimeSpan.Zero && untilHeartbeat < wait)
            {
                wait = untilHeartbeat;
            }

            if (wait > TimeSpan.Zero)
            {
                await _hub.WaitForCommandSignalAsync(wait, stoppingToken);
            }
        }
    }

    private async Task SendHeartbeatAsync(CancellationToken cancellationToken)
    {
        var managedSessionIds = ManagedSessionIdsForHeartbeat(
            await _sessions.GetManagedSessionsAsync(cancellationToken));

        var response = await _hub.HeartbeatAsync(
            new WorkerHeartbeatRequest(
                _options.ImageVersion,
                BuildCapabilities(),
                managedSessionIds),
            cancellationToken);
        ValidateRegistration(response);
    }

    internal static Guid[] ManagedSessionIdsForHeartbeat(
        IEnumerable<ManagedSessionInfo> managedSessions)
    {
        return managedSessions
            .Select(session => session.SessionId)
            .Distinct()
            .Order()
            .ToArray();
    }

    private JsonElement BuildCapabilities()
    {
        return WorkerCapabilityBuilder.Build(
            _options,
            _viewerPublisher.Availability);
    }

    private void ValidateRegistration(WorkerRegistrationResponse response)
    {
        if (response.WorkerId != _options.WorkerId)
        {
            throw new InvalidOperationException(
                "Backend registration returned a different worker identity.");
        }

        if (!response.Name.Equals(_options.Name, StringComparison.Ordinal))
        {
            _logger.LogWarning(
                "Configured worker name '{ConfiguredName}' differs from enrolled name '{EnrolledName}'.",
                _options.Name,
                response.Name);
        }

        if (response.MaxConcurrentSessions != _options.MaxConcurrentSessions)
        {
            _logger.LogWarning(
                "Configured capacity {ConfiguredCapacity} differs from enrolled capacity {EnrolledCapacity}; "
                + "the worker enforces its configured limit while backend scheduling uses the enrolled limit.",
                _options.MaxConcurrentSessions,
                response.MaxConcurrentSessions);
        }
    }
}
