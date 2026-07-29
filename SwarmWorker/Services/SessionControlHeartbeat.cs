using System.Diagnostics;
using Microsoft.Extensions.Options;
using SwarmWorker.Configuration;
using SwarmWorker.Infrastructure;
using SwarmWorker.Runtime;

namespace SwarmWorker.Services;

public sealed class SessionControlHeartbeat : BackgroundService
{
    private readonly DockerSessionManager _sessions;
    private readonly IWorkerCommandHub _hub;
    private readonly WorkerOptions _options;
    private readonly ILogger<SessionControlHeartbeat> _logger;
    private ManagedSessionInfo[] _runningSessions =
        Array.Empty<ManagedSessionInfo>();
    private bool _leaseSuspended;

    public SessionControlHeartbeat(
        DockerSessionManager sessions,
        IWorkerCommandHub hub,
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
            var cycleStarted = Stopwatch.GetTimestamp();
            try
            {
                if (BackendLeaseExpired())
                {
                    SuspendLease();
                    await DelayNextHeartbeat(cycleStarted, stoppingToken);
                    continue;
                }

                ResumeLease();
                try
                {
                    _runningSessions = (await _sessions
                            .GetHeartbeatSessionsAsync(stoppingToken))
                        .ToArray();
                }
                catch (Exception exception)
                    when (exception is DockerCliException
                          or TimeoutException
                          or InvalidOperationException)
                {
                    _logger.LogDebug(
                        exception,
                        "Docker discovery is unavailable; publishing to the last known running sessions.");
                }

                var publications = new List<Task>(_runningSessions.Length);
                foreach (var session in _runningSessions)
                {
                    // Discovery has its own short timeout, but the backend
                    // lease still has to be fresh when each publication starts.
                    if (BackendLeaseExpired())
                    {
                        SuspendLease();
                        break;
                    }

                    publications.Add(PublishHeartbeat(
                        session,
                        BackendLeaseDeadline(),
                        stoppingToken));
                }

                await Task.WhenAll(publications);
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

            await DelayNextHeartbeat(cycleStarted, stoppingToken);
        }
    }

    private Task DelayNextHeartbeat(
        long cycleStarted,
        CancellationToken cancellationToken)
    {
        var interval = TimeSpan.FromSeconds(
            _options.ControlHeartbeatIntervalSeconds);
        var remaining = interval - Stopwatch.GetElapsedTime(cycleStarted);
        return remaining > TimeSpan.Zero
            ? Task.Delay(remaining, cancellationToken)
            : Task.CompletedTask;
    }

    private bool BackendLeaseExpired()
    {
        return CurrentMonotonicSeconds() >= BackendLeaseDeadline();
    }

    private double BackendLeaseDeadline()
    {
        return _hub.LastSuccessfulContactMonotonicSeconds
            + _options.ControlHeartbeatBackendLeaseSeconds
            - WorkerOptions.ControlHeartbeatDeadlineGuardSeconds;
    }

    private static double CurrentMonotonicSeconds()
    {
        return SharedMonotonicClock.GetSeconds();
    }

    private void SuspendLease()
    {
        if (_leaseSuspended)
        {
            return;
        }

        _leaseSuspended = true;
        _logger.LogCritical(
            "Session heartbeat pulses are suspended because the backend lease is stale.");
    }

    private void ResumeLease()
    {
        if (!_leaseSuspended)
        {
            return;
        }

        _leaseSuspended = false;
        _logger.LogInformation(
            "Session heartbeat pulses resumed after backend contact recovered.");
    }

    private async Task PublishHeartbeat(
        ManagedSessionInfo session,
        double deadline,
        CancellationToken cancellationToken)
    {
        try
        {
            await _sessions.PublishControlHeartbeatAsync(
                session,
                deadline,
                cancellationToken);
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
