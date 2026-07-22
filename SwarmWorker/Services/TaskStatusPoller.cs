using Microsoft.AspNetCore.SignalR;
using Microsoft.Extensions.Options;
using SwarmWorker.Configuration;
using SwarmWorker.Contracts;
using SwarmWorker.Infrastructure;
using SwarmWorker.Runtime;

namespace SwarmWorker.Services;

public sealed class TaskStatusPoller : BackgroundService
{
    private readonly DockerSessionManager _sessions;
    private readonly TaskStatusTracker _tracker;
    private readonly BoundedCommandExecutor _commandExecutor;
    private readonly IWorkerCommandHub _hub;
    private readonly WorkerOptions _options;
    private readonly ILogger<TaskStatusPoller> _logger;

    public TaskStatusPoller(
        DockerSessionManager sessions,
        TaskStatusTracker tracker,
        BoundedCommandExecutor commandExecutor,
        IWorkerCommandHub hub,
        IOptions<WorkerOptions> options,
        ILogger<TaskStatusPoller> logger)
    {
        _sessions = sessions;
        _tracker = tracker;
        _commandExecutor = commandExecutor;
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
                await PollOnceAsync(stoppingToken);
            }
            catch (OperationCanceledException) when (stoppingToken.IsCancellationRequested)
            {
                break;
            }
            catch (Exception exception)
            {
                _logger.LogWarning(
                    exception,
                    "ROS task status polling failed; the next interval will retry.");
            }

            await Task.Delay(
                TimeSpan.FromSeconds(_options.TaskStatusPollIntervalSeconds),
                stoppingToken);
        }
    }

    internal async Task PollOnceAsync(CancellationToken cancellationToken)
    {
        var trackedTasks = _tracker.Snapshot();
        IReadOnlyList<ManagedSessionInfo> managedSessions;
        try
        {
            managedSessions = await _sessions.GetManagedSessionsAsync(cancellationToken);
        }
        catch (Exception exception)
            when (exception is DockerCliException
                  or TimeoutException
                  or InvalidOperationException)
        {
            _logger.LogDebug(
                exception,
                "Docker is unavailable while polling ROS task status.");
            return;
        }

        var sessionsById = managedSessions
            .GroupBy(session => session.SessionId)
            .ToDictionary(group => group.Key, group => group.ToArray());

        var trackedPolls = trackedTasks.Select(
            trackedTask => PollSessionAsync(
                trackedTask,
                sessionsById,
                cancellationToken));
        var discoveryPolls = sessionsById.Values
            .Where(matches => matches.Length == 1
                && matches[0].Running
                && !_tracker.IsTracking(matches[0].SessionId))
            .Select(matches => DiscoverSessionTaskAsync(
                matches[0],
                cancellationToken));

        await Task.WhenAll(trackedPolls.Concat(discoveryPolls));
    }

    private async Task PollSessionAsync(
        TrackedTaskStatus trackedTask,
        IReadOnlyDictionary<Guid, ManagedSessionInfo[]> sessionsById,
        CancellationToken cancellationToken)
    {
        if (_commandExecutor.IsStartTaskActive(trackedTask.SessionId))
        {
            return;
        }

        if (!sessionsById.TryGetValue(trackedTask.SessionId, out var matches)
            || matches.Length != 1
            || !matches[0].Running)
        {
            return;
        }

        try
        {
            var status = await _sessions.ReadTaskStatusAsync(
                matches[0],
                cancellationToken);
            if (status is null
                || _commandExecutor.IsStartTaskActive(trackedTask.SessionId))
            {
                return;
            }

            if (!_tracker.TryCreateReport(
                    trackedTask,
                    status,
                    _options.TaskProgressReportStep,
                    out var report))
            {
                return;
            }

            try
            {
                await _hub.ReportTaskEventAsync(report, cancellationToken);
            }
            catch (HubException exception)
            {
                _tracker.TryRecordRejected(trackedTask, report);
                _logger.LogWarning(
                    exception,
                    "Backend rejected ROS status for task {TaskRunId}; the worker will retry it with bounded backoff.",
                    report.TaskRunId);
                return;
            }

            _tracker.TryRecord(trackedTask, report);
            _logger.LogDebug(
                "Reported ROS task {TaskRunId} state {State} at progress {Progress}.",
                report.TaskRunId,
                report.State,
                report.Progress);
        }
        catch (OperationCanceledException) when (cancellationToken.IsCancellationRequested)
        {
            throw;
        }
        catch (Exception exception)
            when (exception is DockerCliException
                  or TimeoutException
                  or FormatException
                  or InvalidOperationException)
        {
            _logger.LogDebug(
                exception,
                "ROS status is unavailable for session {SessionId}; polling will retry.",
                trackedTask.SessionId);
        }
    }

    private async Task DiscoverSessionTaskAsync(
        ManagedSessionInfo session,
        CancellationToken cancellationToken)
    {
        if (_commandExecutor.IsStartTaskActive(session.SessionId))
        {
            return;
        }

        try
        {
            var status = await _sessions.ReadTaskStatusAsync(session, cancellationToken);
            if (status is null
                || _commandExecutor.IsStartTaskActive(session.SessionId))
            {
                return;
            }

            var report = new TaskEventReport(
                session.SessionId,
                status.TaskRunId,
                status.State,
                status.Progress,
                status.Result,
                status.State == "Failed" ? status.Error : null);
            try
            {
                await _hub.ReportTaskEventAsync(report, cancellationToken);
            }
            catch (HubException)
            {
                // The backend may have already closed this task while the
                // worker was offline. Retry a few times in case the rejection
                // was transient, then retain a watermark to prevent spam.
                _tracker.TryRecordDiscoveredRejected(report);
                return;
            }

            _tracker.TryRecordDiscovered(report);

            _logger.LogInformation(
                IsTerminal(status.State)
                    ? "Recovered terminal ROS task {TaskRunId} for session {SessionId}."
                    : "Recovered active ROS task {TaskRunId} for session {SessionId}.",
                report.TaskRunId,
                report.SessionId);
        }
        catch (OperationCanceledException) when (cancellationToken.IsCancellationRequested)
        {
            throw;
        }
        catch (Exception exception)
            when (exception is DockerCliException
                  or TimeoutException
                  or FormatException
                  or InvalidOperationException)
        {
            _logger.LogDebug(
                exception,
                "ROS task discovery is unavailable for session {SessionId}.",
                session.SessionId);
        }
    }

    private static bool IsTerminal(string state)
    {
        return state is "Completed" or "Cancelled" or "Failed";
    }
}
