using Microsoft.Extensions.Options;
using SwarmWorker.Configuration;
using SwarmWorker.Infrastructure;
using SwarmWorker.Runtime;

namespace SwarmWorker.Services;

public sealed class SessionControlHeartbeat : BackgroundService
{
    private readonly DockerSessionManager _sessions;
    private readonly WorkerHubConnection _hub;
    private readonly WorkerOptions _options;
    private readonly ILogger<SessionControlHeartbeat> _logger;
    private bool _leaseSuspended;

    public SessionControlHeartbeat(
        DockerSessionManager sessions,
        WorkerHubConnection hub,
        IOptions<WorkerOptions> options,
        ILogger<SessionControlHeartbeat> logger)
    {
        _sessions = sessions;
        _hub = hub;
        _options = options.Value;
        _logger = logger;
    }

    protected override async Task ExecuteAsync(CancellationToken stoppingToken)
    {
        while (!stoppingToken.IsCancellationRequested)
        {
            try
            {
                var backendContactAge = DateTime.UtcNow - _hub.LastSuccessfulContactUtc;
                if (backendContactAge > TimeSpan.FromSeconds(
                        _options.ControlHeartbeatBackendLeaseSeconds))
                {
                    if (!_leaseSuspended)
                    {
                        _leaseSuspended = true;
                        _logger.LogCritical(
                            "Session heartbeat pulses are suspended because the backend lease is stale.");
                    }

                    await DelayNextHeartbeat(stoppingToken);
                    continue;
                }

                if (_leaseSuspended)
                {
                    _leaseSuspended = false;
                    _logger.LogInformation(
                        "Session heartbeat pulses resumed after backend contact recovered.");
                }

                var runningSessions = (await _sessions.GetManagedSessionsAsync(stoppingToken))
                    .Where(session => session.Running)
                    .ToArray();
                await Task.WhenAll(runningSessions.Select(
                    session => PublishHeartbeat(session, stoppingToken)));
            }
            catch (OperationCanceledException) when (stoppingToken.IsCancellationRequested)
            {
                break;
            }
            catch (Exception exception)
                when (exception is DockerCliException
                      or TimeoutException
                      or InvalidOperationException)
            {
                _logger.LogDebug(
                    exception,
                    "Docker is unavailable while publishing session control heartbeats.");
            }

            await DelayNextHeartbeat(stoppingToken);
        }
    }

    private Task DelayNextHeartbeat(CancellationToken cancellationToken)
    {
        return Task.Delay(
            TimeSpan.FromSeconds(_options.ControlHeartbeatIntervalSeconds),
            cancellationToken);
    }

    private async Task PublishHeartbeat(
        ManagedSessionInfo session,
        CancellationToken cancellationToken)
    {
        try
        {
            await _sessions.PublishControlHeartbeatAsync(session, cancellationToken);
        }
        catch (Exception exception)
            when (exception is DockerCliException
                  or TimeoutException
                  or InvalidOperationException)
        {
            _logger.LogDebug(
                exception,
                "ROS is not ready for the control heartbeat in session {SessionId}.",
                session.SessionId);
        }
    }
}
