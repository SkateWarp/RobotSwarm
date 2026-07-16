using SwarmWorker.Contracts;

namespace SwarmWorker.Runtime;

public sealed record TrackedTaskStatus(
    Guid SessionId,
    Guid TaskRunId,
    string LastState,
    double? LastProgress);

public sealed class TaskStatusTracker
{
    private readonly object _gate = new();
    private readonly Dictionary<Guid, TrackedTaskStatus> _tasks = new();

    public IReadOnlyList<TrackedTaskStatus> Snapshot()
    {
        lock (_gate)
        {
            return _tasks.Values
                .OrderBy(task => task.SessionId)
                .ToArray();
        }
    }

    public bool IsTracking(Guid sessionId)
    {
        lock (_gate)
        {
            return _tasks.ContainsKey(sessionId);
        }
    }

    public void Record(TaskEventReport report)
    {
        lock (_gate)
        {
            if (IsTerminal(report.State))
            {
                if (_tasks.TryGetValue(report.SessionId, out var current)
                    && current.TaskRunId == report.TaskRunId)
                {
                    _tasks.Remove(report.SessionId);
                }

                return;
            }

            var progress = report.Progress;
            if (!progress.HasValue
                && _tasks.TryGetValue(report.SessionId, out var existing)
                && existing.TaskRunId == report.TaskRunId)
            {
                progress = existing.LastProgress;
            }

            _tasks[report.SessionId] = new TrackedTaskStatus(
                report.SessionId,
                report.TaskRunId,
                report.State,
                progress);
        }
    }

    public bool TryCreateReport(
        TrackedTaskStatus snapshot,
        RosTaskStatus status,
        double progressStep,
        out TaskEventReport report)
    {
        lock (_gate)
        {
            if (!_tasks.TryGetValue(snapshot.SessionId, out var current)
                || current != snapshot
                || status.TaskRunId != current.TaskRunId)
            {
                report = default!;
                return false;
            }

            if (status.State == "Accepted"
                && current.LastState is "Running" or "Paused")
            {
                report = default!;
                return false;
            }

            var stateChanged = !status.State.Equals(
                current.LastState,
                StringComparison.OrdinalIgnoreCase);
            var progressChanged = status.Progress.HasValue
                                  && (!current.LastProgress.HasValue
                                      || Math.Abs(
                                          status.Progress.Value
                                          - current.LastProgress.Value) >= progressStep);
            if (!stateChanged && !progressChanged)
            {
                report = default!;
                return false;
            }

            report = new TaskEventReport(
                current.SessionId,
                current.TaskRunId,
                status.State,
                status.Progress,
                null,
                status.State == "Failed" ? status.Error : null);
            return true;
        }
    }

    public void RemoveSession(Guid sessionId, Guid? taskRunId = null)
    {
        lock (_gate)
        {
            if (!_tasks.TryGetValue(sessionId, out var current)
                || taskRunId.HasValue && current.TaskRunId != taskRunId.Value)
            {
                return;
            }

            _tasks.Remove(sessionId);
        }
    }

    private static bool IsTerminal(string state)
    {
        return state is "Completed" or "Cancelled" or "Failed";
    }
}
