using System.Text.Json;
using SwarmWorker.Contracts;
using SwarmWorker.Runtime;

namespace SwarmWorker.Tests;

public sealed class TaskStatusParserTests
{
    [Theory]
    [InlineData("initializing", "Accepted")]
    [InlineData("running", "Running")]
    [InlineData("paused", "Paused")]
    [InlineData("completed", "Completed")]
    [InlineData("failed", "Failed")]
    [InlineData("stopped", "Cancelled")]
    public void MapsRosTaskStates(string rosState, string backendState)
    {
        var taskRunId = Guid.NewGuid();
        var output = BuildRosOutput(taskRunId, rosState, 1.4);

        var status = TaskStatusParser.Parse(output);

        Assert.NotNull(status);
        Assert.Equal(taskRunId, status.TaskRunId);
        Assert.Equal(backendState, status.State);
        Assert.Equal(1, status.Progress);
    }

    [Fact]
    public void IgnoresIdleStatusWithoutTaskId()
    {
        var json = JsonSerializer.Serialize(new
        {
            task = new
            {
                task_id = (string?)null,
                status = "idle",
                progress = 0
            }
        });

        Assert.Null(TaskStatusParser.Parse(ToRosOutput(json)));
    }

    [Fact]
    public void RejectsInvalidTaskId()
    {
        var json = JsonSerializer.Serialize(new
        {
            task = new
            {
                task_id = "not-a-uuid",
                status = "running",
                progress = 0.5
            }
        });

        Assert.Throws<FormatException>(
            () => TaskStatusParser.Parse(ToRosOutput(json)));
    }

    [Fact]
    public void PreservesTransportResultFromRosStatus()
    {
        var taskRunId = Guid.NewGuid();
        var json = JsonSerializer.Serialize(new
        {
            task = new
            {
                task_id = taskRunId,
                status = "running",
                progress = 0.15,
                result = new
                {
                    transport = new
                    {
                        phase = "SEARCH",
                        searching_robot_count = 10,
                        discovery = (object?)null
                    }
                }
            }
        });

        var status = TaskStatusParser.Parse(ToRosOutput(json));

        Assert.NotNull(status);
        Assert.True(status.Result.HasValue);
        var transport = status.Result.Value.GetProperty("transport");
        Assert.Equal("SEARCH", transport.GetProperty("phase").GetString());
        Assert.Equal(10, transport.GetProperty("searching_robot_count").GetInt32());
        Assert.Equal(JsonValueKind.Null, transport.GetProperty("discovery").ValueKind);
    }

    [Fact]
    public void TrackerReportsOnlyMeaningfulMatchingChanges()
    {
        var sessionId = Guid.NewGuid();
        var taskRunId = Guid.NewGuid();
        var tracker = new TaskStatusTracker();
        tracker.Record(new TaskEventReport(
            sessionId,
            taskRunId,
            "Running",
            0.10,
            null,
            null));
        var snapshot = Assert.Single(tracker.Snapshot());

        Assert.False(tracker.TryCreateReport(
            snapshot,
            new RosTaskStatus(taskRunId, "Running", 0.11, null, null),
            0.02,
            out _));
        Assert.True(tracker.TryCreateReport(
            snapshot,
            new RosTaskStatus(taskRunId, "Running", 0.13, null, null),
            0.02,
            out var progressReport));
        Assert.Equal(0.13, progressReport.Progress);
        Assert.True(tracker.TryCreateReport(
            snapshot,
            new RosTaskStatus(taskRunId, "Completed", 1, null, null),
            0.02,
            out var completedReport));
        Assert.Equal("Completed", completedReport.State);
    }

    [Fact]
    public void TrackerReportsResultOnlyChangesAndThenSuppressesDuplicates()
    {
        var sessionId = Guid.NewGuid();
        var taskRunId = Guid.NewGuid();
        var tracker = new TaskStatusTracker();
        var searching = JsonSerializer.SerializeToElement(new
        {
            transport = new
            {
                phase = "SEARCH",
                searching_robot_count = 3,
                discovery = (object?)null
            }
        });
        tracker.Record(new TaskEventReport(
            sessionId,
            taskRunId,
            "Running",
            0.1,
            searching,
            null));
        var snapshot = Assert.Single(tracker.Snapshot());
        var regrouping = JsonSerializer.SerializeToElement(new
        {
            transport = new
            {
                phase = "APPROACH",
                searching_robot_count = 0,
                discovery = new { finder = "tb3_1" }
            }
        });

        Assert.True(tracker.TryCreateReport(
            snapshot,
            new RosTaskStatus(
                taskRunId,
                "Running",
                0.1,
                regrouping,
                null),
            0.02,
            out var report));
        Assert.True(report.Result.HasValue);
        Assert.Equal(
            "tb3_1",
            report.Result.Value
                .GetProperty("transport")
                .GetProperty("discovery")
                .GetProperty("finder")
                .GetString());

        tracker.Record(report);
        var updatedSnapshot = Assert.Single(tracker.Snapshot());
        var sameRegrouping = JsonSerializer.Deserialize<JsonElement>(
            regrouping.GetRawText());
        Assert.False(tracker.TryCreateReport(
            updatedSnapshot,
            new RosTaskStatus(
                taskRunId,
                "Running",
                0.1,
                sameRegrouping,
                null),
            0.02,
            out _));
    }

    [Fact]
    public void TrackerRetainsTerminalWatermarkUntilSessionRemoval()
    {
        var sessionId = Guid.NewGuid();
        var taskRunId = Guid.NewGuid();
        var tracker = new TaskStatusTracker();
        tracker.Record(new TaskEventReport(
            sessionId,
            taskRunId,
            "Running",
            0,
            null,
            null));

        tracker.Record(new TaskEventReport(
            sessionId,
            taskRunId,
            "Failed",
            0.4,
            null,
            "controller failed"));

        var terminal = Assert.Single(tracker.Snapshot());
        Assert.Equal(taskRunId, terminal.TaskRunId);
        Assert.Equal("Failed", terminal.LastState);
        Assert.True(tracker.IsTracking(sessionId));
        Assert.False(tracker.TryCreateReport(
            terminal,
            new RosTaskStatus(
                taskRunId,
                "Failed",
                0.4,
                null,
                "controller failed"),
            0.02,
            out _));

        tracker.RemoveSession(sessionId);

        Assert.Empty(tracker.Snapshot());
    }

    [Fact]
    public void TrackerReplacesTerminalWatermarkWhenRosTaskIdChanges()
    {
        var sessionId = Guid.NewGuid();
        var finishedTaskId = Guid.NewGuid();
        var replacementTaskId = Guid.NewGuid();
        var tracker = new TaskStatusTracker();
        tracker.Record(new TaskEventReport(
            sessionId,
            finishedTaskId,
            "Completed",
            1,
            null,
            null));
        var terminal = Assert.Single(tracker.Snapshot());

        Assert.True(tracker.TryCreateReport(
            terminal,
            new RosTaskStatus(
                replacementTaskId,
                "Running",
                0.15,
                null,
                null),
            0.02,
            out var replacement));
        Assert.True(tracker.TryRecord(terminal, replacement));

        var current = Assert.Single(tracker.Snapshot());
        Assert.Equal(replacementTaskId, current.TaskRunId);
        Assert.Equal("Running", current.LastState);
    }

    [Fact]
    public void TrackerBoundsRejectionByTaskRunAndRetainsTheLatestReport()
    {
        var sessionId = Guid.NewGuid();
        var taskRunId = Guid.NewGuid();
        var tracker = new TaskStatusTracker();
        tracker.Record(new TaskEventReport(
            sessionId,
            taskRunId,
            "Running",
            0.8,
            null,
            null));
        RosTaskStatus FailedStatus(double progress, int revision) => new(
            taskRunId,
            "Failed",
            progress,
            JsonSerializer.SerializeToElement(new { revision }),
            $"error-{revision}");

        var snapshot = Assert.Single(tracker.Snapshot());
        Assert.True(tracker.TryCreateReport(
            snapshot,
            FailedStatus(0.4, 1),
            0.02,
            out var firstAttempt));
        Assert.True(tracker.TryRecordRejected(snapshot, firstAttempt));

        snapshot = Assert.Single(tracker.Snapshot());
        Assert.False(tracker.TryCreateReport(
            snapshot,
            FailedStatus(0.5, 2),
            0.02,
            out _));
        snapshot = Assert.Single(tracker.Snapshot());
        Assert.Equal(0.5, snapshot.RejectedReport?.Progress);
        Assert.Equal("error-2", snapshot.RejectedReport?.Error);
        Assert.True(tracker.TryCreateReport(
            snapshot,
            FailedStatus(0.6, 3),
            0.02,
            out var secondAttempt));
        Assert.True(tracker.TryRecordRejected(snapshot, secondAttempt));

        var skippedStatuses = new[]
        {
            FailedStatus(0.7, 4),
            FailedStatus(0.8, 5)
        };
        foreach (var skippedStatus in skippedStatuses)
        {
            snapshot = Assert.Single(tracker.Snapshot());
            Assert.False(tracker.TryCreateReport(
                snapshot,
                skippedStatus,
                0.02,
                out _));
        }

        snapshot = Assert.Single(tracker.Snapshot());
        Assert.Equal(0.8, snapshot.RejectedReport?.Progress);
        Assert.Equal("error-5", snapshot.RejectedReport?.Error);
        Assert.True(tracker.TryCreateReport(
            snapshot,
            FailedStatus(0.9, 6),
            0.02,
            out var finalAttempt));
        Assert.True(tracker.TryRecordRejected(snapshot, finalAttempt));

        var terminalWatermark = Assert.Single(tracker.Snapshot());
        Assert.Equal("Failed", terminalWatermark.LastState);
        Assert.Equal(0.9, terminalWatermark.LastProgress);
        Assert.Equal(
            6,
            terminalWatermark.LastResult?.GetProperty("revision").GetInt32());
        Assert.Equal("error-6", terminalWatermark.RejectedReport?.Error);
        Assert.True(terminalWatermark.RejectionSuppressed);
        Assert.False(tracker.TryCreateReport(
            terminalWatermark,
            FailedStatus(1, 7),
            0.02,
            out _));

        var nextTaskId = Guid.NewGuid();
        Assert.True(tracker.TryCreateReport(
            terminalWatermark,
            new RosTaskStatus(
                nextTaskId,
                "Running",
                0.1,
                null,
                null),
            0.02,
            out var nextTask));
        Assert.Equal(nextTaskId, nextTask.TaskRunId);
    }

    [Fact]
    public void TrackerRetriesARejectedDiscoveredTask()
    {
        var sessionId = Guid.NewGuid();
        var taskRunId = Guid.NewGuid();
        var tracker = new TaskStatusTracker();
        var report = new TaskEventReport(
            sessionId,
            taskRunId,
            "Completed",
            1,
            null,
            null);
        var status = new RosTaskStatus(
            taskRunId,
            "Completed",
            1,
            null,
            null);

        Assert.True(tracker.TryRecordDiscoveredRejected(report));
        var snapshot = Assert.Single(tracker.Snapshot());
        Assert.False(tracker.TryCreateReport(
            snapshot,
            status,
            0.02,
            out _));

        snapshot = Assert.Single(tracker.Snapshot());
        Assert.True(tracker.TryCreateReport(
            snapshot,
            status,
            0.02,
            out var retry));
        Assert.Equal(taskRunId, retry.TaskRunId);
        Assert.Equal("Completed", retry.State);
    }

    [Fact]
    public void AcceptedStartEventCanExplicitlyRestartTerminalTaskId()
    {
        var sessionId = Guid.NewGuid();
        var taskRunId = Guid.NewGuid();
        var tracker = new TaskStatusTracker();
        var oldResult = JsonSerializer.SerializeToElement(new { lap = 1 });
        tracker.Record(new TaskEventReport(
            sessionId,
            taskRunId,
            "Completed",
            1,
            oldResult,
            null));

        tracker.Record(new TaskEventReport(
            sessionId,
            taskRunId,
            "Running",
            0.25,
            null,
            null));
        Assert.Equal("Completed", Assert.Single(tracker.Snapshot()).LastState);

        tracker.Record(new TaskEventReport(
            sessionId,
            taskRunId,
            "Accepted",
            0,
            null,
            null));

        var restarted = Assert.Single(tracker.Snapshot());
        Assert.Equal("Accepted", restarted.LastState);
        Assert.Equal(0, restarted.LastProgress);
        Assert.False(restarted.LastResult.HasValue);
        Assert.True(restarted.RestartPending);
        Assert.Equal(2, restarted.RestartTerminalSamplesRemaining);

        Assert.False(tracker.TryCreateReport(
            restarted,
            new RosTaskStatus(
                taskRunId,
                "Completed",
                1,
                oldResult,
                null),
            0.02,
            out _));
        restarted = Assert.Single(tracker.Snapshot());
        Assert.True(tracker.TryCreateReport(
            restarted,
            new RosTaskStatus(
                taskRunId,
                "Running",
                0.1,
                null,
                null),
            0.02,
            out var running));
        Assert.True(tracker.TryRecord(restarted, running));

        var active = Assert.Single(tracker.Snapshot());
        Assert.Equal("Running", active.LastState);
        Assert.False(active.RestartPending);
    }

    [Fact]
    public void FastTerminalRestartIsReportedAfterBoundedDisambiguation()
    {
        var sessionId = Guid.NewGuid();
        var taskRunId = Guid.NewGuid();
        var tracker = new TaskStatusTracker();
        tracker.Record(new TaskEventReport(
            sessionId,
            taskRunId,
            "Completed",
            1,
            JsonSerializer.SerializeToElement(new { generation = "old" }),
            null));
        tracker.Record(new TaskEventReport(
            sessionId,
            taskRunId,
            "Accepted",
            0,
            null,
            null));
        var fastCompletion = new RosTaskStatus(
            taskRunId,
            "Completed",
            1,
            JsonSerializer.SerializeToElement(new { generation = "new" }),
            null);

        for (var staleSample = 0; staleSample < 2; staleSample++)
        {
            var pending = Assert.Single(tracker.Snapshot());
            Assert.False(tracker.TryCreateReport(
                pending,
                fastCompletion,
                0.02,
                out _));
        }

        var disambiguated = Assert.Single(tracker.Snapshot());
        Assert.True(tracker.TryCreateReport(
            disambiguated,
            fastCompletion,
            0.02,
            out var completed));
        Assert.Equal("Completed", completed.State);
        Assert.Equal(
            "new",
            completed.Result?.GetProperty("generation").GetString());
    }

    [Fact]
    public void RosAcceptedClearsRestartPendingBeforeARealTerminalStatus()
    {
        var sessionId = Guid.NewGuid();
        var taskRunId = Guid.NewGuid();
        var tracker = new TaskStatusTracker();
        tracker.Record(new TaskEventReport(
            sessionId,
            taskRunId,
            "Completed",
            1,
            null,
            null));
        tracker.Record(new TaskEventReport(
            sessionId,
            taskRunId,
            "Accepted",
            0,
            null,
            null));

        var restart = Assert.Single(tracker.Snapshot());
        Assert.False(tracker.TryCreateReport(
            restart,
            new RosTaskStatus(
                taskRunId,
                "Accepted",
                0,
                null,
                null),
            0.02,
            out _));

        var acceptedByRos = Assert.Single(tracker.Snapshot());
        Assert.False(acceptedByRos.RestartPending);
        Assert.True(tracker.TryCreateReport(
            acceptedByRos,
            new RosTaskStatus(
                taskRunId,
                "Completed",
                1,
                null,
                null),
            0.02,
            out var completed));
        Assert.Equal("Completed", completed.State);
    }

    [Fact]
    public void TrackerKnowsWhichSessionsNeedRecovery()
    {
        var sessionId = Guid.NewGuid();
        var tracker = new TaskStatusTracker();

        Assert.False(tracker.IsTracking(sessionId));

        tracker.Record(new TaskEventReport(
            sessionId,
            Guid.NewGuid(),
            "Running",
            0.2,
            null,
            null));

        Assert.True(tracker.IsTracking(sessionId));
    }

    private static string BuildRosOutput(
        Guid taskRunId,
        string state,
        double progress)
    {
        var json = JsonSerializer.Serialize(new
        {
            task = new
            {
                task_id = taskRunId,
                status = state,
                progress
            }
        });
        return ToRosOutput(json);
    }

    private static string ToRosOutput(string json) =>
        $"data: {JsonSerializer.Serialize(json)}\n---\n";
}
