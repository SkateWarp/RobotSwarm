using System.Text.Json;
using SwarmWorker.Contracts;

namespace SwarmWorker.Runtime;

public sealed record TrackedTaskStatus(
    Guid SessionId,
    Guid TaskRunId,
    string LastState,
    double? LastProgress,
    JsonElement? LastResult,
    bool RestartPending = false,
    int RestartTerminalSamplesRemaining = 0,
    TaskEventReport? RejectedReport = null,
    int RejectionCount = 0,
    int RejectionPollsRemaining = 0,
    bool RejectionSuppressed = false);

public sealed class TaskStatusTracker
{
    private const int MaximumRejectedReportAttempts = 3;
    private const int RestartTerminalSamplesToIgnore = 2;

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
            _tasks.TryGetValue(report.SessionId, out var current);
            if (current is not null
                && current.TaskRunId == report.TaskRunId
                && IsTerminal(current.LastState)
                && !IsTerminal(report.State)
                && !report.State.Equals("Accepted", StringComparison.OrdinalIgnoreCase))
            {
                // A late ROS sample must not reopen a finished task. The
                // Accepted event emitted by an explicit StartTask is the one
                // deliberate exception.
                return;
            }

            var acceptedStart = report.State.Equals(
                "Accepted",
                StringComparison.OrdinalIgnoreCase);
            var explicitRestart = current is not null
                                  && current.TaskRunId == report.TaskRunId
                                  && acceptedStart
                                  && !current.LastState.Equals(
                                      "Accepted",
                                      StringComparison.OrdinalIgnoreCase);
            var keepRestartPending = current is not null
                                     && current.RestartPending
                                     && current.TaskRunId == report.TaskRunId
                                     && acceptedStart;
            _tasks[report.SessionId] = CreateSnapshot(
                report,
                explicitRestart ? null : current,
                restartPending: explicitRestart || keepRestartPending,
                restartTerminalSamplesRemaining: explicitRestart
                    ? RestartTerminalSamplesToIgnore
                    : keepRestartPending
                        ? current!.RestartTerminalSamplesRemaining
                        : 0);
        }
    }

    public bool TryRecord(
        TrackedTaskStatus expected,
        TaskEventReport report)
    {
        lock (_gate)
        {
            if (!_tasks.TryGetValue(expected.SessionId, out var current)
                || !ReferenceEquals(current, expected)
                || report.SessionId != expected.SessionId)
            {
                return false;
            }

            _tasks[report.SessionId] = CreateSnapshot(report, current);
            return true;
        }
    }

    public bool TryRecordRejected(
        TrackedTaskStatus expected,
        TaskEventReport report)
    {
        lock (_gate)
        {
            if (!_tasks.TryGetValue(expected.SessionId, out var current)
                || !ReferenceEquals(current, expected)
                || report.SessionId != expected.SessionId)
            {
                return false;
            }

            var sameRejectedTask = current.RejectedReport?.TaskRunId
                                   == report.TaskRunId;
            var rejectionCount = sameRejectedTask
                ? current.RejectionCount + 1
                : 1;
            if (rejectionCount >= MaximumRejectedReportAttempts)
            {
                // A permanently rejected ROS snapshot must eventually become a
                // watermark, otherwise it is sent on every poll forever.
                _tasks[report.SessionId] = CreateSnapshot(report, current) with
                {
                    RejectedReport = CloneReport(report),
                    RejectionCount = rejectionCount,
                    RejectionSuppressed = true
                };
                return true;
            }

            _tasks[report.SessionId] = current with
            {
                RestartPending = current.RestartPending
                                 && (current.TaskRunId != report.TaskRunId
                                     || IsTerminal(report.State)),
                RestartTerminalSamplesRemaining =
                    current.TaskRunId == report.TaskRunId
                    && !IsTerminal(report.State)
                        ? 0
                        : current.RestartTerminalSamplesRemaining,
                RejectedReport = CloneReport(report),
                RejectionCount = rejectionCount,
                RejectionPollsRemaining = 1 << (rejectionCount - 1)
            };
            return true;
        }
    }

    public bool TryRecordDiscovered(TaskEventReport report)
    {
        lock (_gate)
        {
            if (_tasks.ContainsKey(report.SessionId))
            {
                return false;
            }

            _tasks[report.SessionId] = CreateSnapshot(report, current: null);
            return true;
        }
    }

    public bool TryRecordDiscoveredRejected(TaskEventReport report)
    {
        lock (_gate)
        {
            if (_tasks.ContainsKey(report.SessionId))
            {
                return false;
            }

            _tasks[report.SessionId] = CreateSnapshot(report, current: null) with
            {
                RejectedReport = CloneReport(report),
                RejectionCount = 1,
                RejectionPollsRemaining = 1
            };
            return true;
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
                || !ReferenceEquals(current, snapshot))
            {
                report = default!;
                return false;
            }

            var sameTask = status.TaskRunId == current.TaskRunId;
            if (sameTask && current.RejectionSuppressed)
            {
                // The backend rejected this task run repeatedly. Suppress the
                // run as a whole even if ROS keeps changing its payload.
                report = default!;
                return false;
            }

            if (sameTask && current.RestartPending)
            {
                if (IsTerminal(status.State))
                {
                    if (current.RestartTerminalSamplesRemaining > 0)
                    {
                        // ROS may briefly retain the prior terminal sample
                        // after a restart that reuses the task run ID. The
                        // ambiguity budget is finite so a fast new run that
                        // finishes immediately is still reported.
                        _tasks[current.SessionId] = current with
                        {
                            RestartTerminalSamplesRemaining =
                                current.RestartTerminalSamplesRemaining - 1
                        };
                        report = default!;
                        return false;
                    }
                }

                if (status.State.Equals(
                        "Accepted",
                        StringComparison.OrdinalIgnoreCase))
                {
                    // Seeing Accepted from ROS proves it has consumed the new
                    // start, so a following terminal sample belongs to it.
                    _tasks[current.SessionId] = current with
                    {
                        RestartPending = false,
                        RestartTerminalSamplesRemaining = 0
                    };
                    report = default!;
                    return false;
                }
            }

            if (sameTask
                && status.State.Equals(
                    "Accepted",
                    StringComparison.OrdinalIgnoreCase)
                && current.LastState is "Running" or "Paused")
            {
                report = default!;
                return false;
            }

            if (sameTask
                && IsTerminal(current.LastState)
                && !IsTerminal(status.State))
            {
                report = default!;
                return false;
            }

            var candidate = CreateReport(current.SessionId, status);
            if (current.RejectedReport is not null
                && current.RejectedReport.TaskRunId == candidate.TaskRunId)
            {
                if (current.RejectionPollsRemaining > 0)
                {
                    _tasks[current.SessionId] = current with
                    {
                        RestartPending = current.RestartPending
                                         && (current.TaskRunId
                                             != candidate.TaskRunId
                                             || IsTerminal(candidate.State)),
                        RestartTerminalSamplesRemaining =
                            current.TaskRunId == candidate.TaskRunId
                            && !IsTerminal(candidate.State)
                                ? 0
                                : current.RestartTerminalSamplesRemaining,
                        RejectedReport = CloneReport(candidate),
                        RejectionPollsRemaining =
                            current.RejectionPollsRemaining - 1
                    };
                    report = default!;
                    return false;
                }

                report = candidate;
                return true;
            }

            if (!sameTask)
            {
                report = candidate;
                return true;
            }

            var stateChanged = !status.State.Equals(
                current.LastState,
                StringComparison.OrdinalIgnoreCase);
            var progressChanged = status.Progress.HasValue
                                  && (!current.LastProgress.HasValue
                                      || Math.Abs(
                                          status.Progress.Value
                                          - current.LastProgress.Value) >= progressStep);
            var resultChanged = status.Result.HasValue
                                && !ResultsMatch(
                                    current.LastResult,
                                    status.Result);
            if (!stateChanged && !progressChanged && !resultChanged)
            {
                report = default!;
                return false;
            }

            report = candidate;
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
        return state.Equals("Completed", StringComparison.OrdinalIgnoreCase)
               || state.Equals("Cancelled", StringComparison.OrdinalIgnoreCase)
               || state.Equals("Failed", StringComparison.OrdinalIgnoreCase);
    }

    private static TrackedTaskStatus CreateSnapshot(
        TaskEventReport report,
        TrackedTaskStatus? current,
        bool restartPending = false,
        int restartTerminalSamplesRemaining = 0)
    {
        var isSameTask = current?.TaskRunId == report.TaskRunId;
        var progress = report.Progress
                       ?? (isSameTask ? current?.LastProgress : null);
        JsonElement? result = report.Result.HasValue
            ? report.Result.Value.Clone()
            : isSameTask
                ? current?.LastResult
                : null;

        return new TrackedTaskStatus(
            report.SessionId,
            report.TaskRunId,
            report.State,
            progress,
            result,
            RestartPending: restartPending,
            RestartTerminalSamplesRemaining: restartTerminalSamplesRemaining);
    }

    private static TaskEventReport CreateReport(
        Guid sessionId,
        RosTaskStatus status,
        JsonElement? result = null)
    {
        return new TaskEventReport(
            sessionId,
            status.TaskRunId,
            status.State,
            status.Progress,
            result ?? (status.Result.HasValue ? status.Result.Value.Clone() : null),
            status.State.Equals("Failed", StringComparison.OrdinalIgnoreCase)
                ? status.Error
                : null);
    }

    private static bool ResultsMatch(JsonElement? left, JsonElement? right)
    {
        if (!left.HasValue || !right.HasValue)
        {
            return left.HasValue == right.HasValue;
        }

        return string.Equals(
            left.Value.GetRawText(),
            right.Value.GetRawText(),
            StringComparison.Ordinal);
    }

    private static TaskEventReport CloneReport(TaskEventReport report)
    {
        return report with
        {
            Result = report.Result.HasValue
                ? report.Result.Value.Clone()
                : null
        };
    }
}
