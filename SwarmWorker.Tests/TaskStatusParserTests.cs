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
            new RosTaskStatus(taskRunId, "Running", 0.11, null),
            0.02,
            out _));
        Assert.False(tracker.TryCreateReport(
            snapshot,
            new RosTaskStatus(Guid.NewGuid(), "Completed", 1, null),
            0.02,
            out _));
        Assert.True(tracker.TryCreateReport(
            snapshot,
            new RosTaskStatus(taskRunId, "Running", 0.13, null),
            0.02,
            out var progressReport));
        Assert.Equal(0.13, progressReport.Progress);
        Assert.True(tracker.TryCreateReport(
            snapshot,
            new RosTaskStatus(taskRunId, "Completed", 1, null),
            0.02,
            out var completedReport));
        Assert.Equal("Completed", completedReport.State);
    }

    [Fact]
    public void TrackerRemovesTerminalTasks()
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

        Assert.Empty(tracker.Snapshot());
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
