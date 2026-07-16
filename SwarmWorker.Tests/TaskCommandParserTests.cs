using System.Text.Json;
using SwarmWorker.Runtime;

namespace SwarmWorker.Tests;

public sealed class TaskCommandParserTests
{
    [Theory]
    [InlineData("FollowLeader", "follow_leader")]
    [InlineData("Figure", "formation")]
    [InlineData("CollaborativeTransport", "transport")]
    public void MapsBackendTaskTypesAndPreservesParameters(
        string backendTaskType,
        string rosTaskType)
    {
        var taskRunId = Guid.NewGuid();
        using var document = JsonDocument.Parse(
            $$"""
              {
                "taskRunId": "{{taskRunId}}",
                "taskType": "{{backendTaskType}}",
                "parameters": {
                  "task_id": "00000000-0000-0000-0000-000000000001",
                  "task_type": "untrusted",
                  "config": {
                    "waypoints": [{"x": 1.25, "y": -2}],
                    "label": "alpha ' ; $(touch nope)"
                  },
                  "enabled": true
                }
              }
              """);

        var command = TaskCommandParser.Parse("StartTask", document.RootElement);
        var parameters = command.Envelope.GetProperty("parameters");

        Assert.Equal("start_task", command.Command);
        Assert.Equal(taskRunId, command.TaskRunId);
        Assert.Equal("Accepted", command.ImmediateState);
        Assert.Equal(0, command.ImmediateProgress);
        Assert.Equal(
            taskRunId.ToString("D"),
            parameters.GetProperty("task_id").GetString());
        Assert.Equal(rosTaskType, parameters.GetProperty("task_type").GetString());
        Assert.True(parameters.GetProperty("enabled").GetBoolean());
        Assert.Equal(
            "alpha ' ; $(touch nope)",
            parameters.GetProperty("config").GetProperty("label").GetString());
        Assert.Equal(
            1.25,
            parameters.GetProperty("config")
                .GetProperty("waypoints")[0]
                .GetProperty("x")
                .GetDouble());
    }

    [Fact]
    public void AcceptsCurrentBackendTypeAlias()
    {
        var taskRunId = Guid.NewGuid();
        using var document = JsonDocument.Parse(
            $$"""
              {
                "taskRunId": "{{taskRunId}}",
                "type": "Figure",
                "parameters": {}
              }
              """);

        var command = TaskCommandParser.Parse("StartTask", document.RootElement);

        Assert.Equal(
            "formation",
            command.Envelope.GetProperty("parameters")
                .GetProperty("task_type")
                .GetString());
    }

    [Theory]
    [InlineData("PauseTask", "pause_task", "Paused")]
    [InlineData("ResumeTask", "resume_task", "Running")]
    [InlineData("CancelTask", "stop_task", "Cancelled")]
    public void MapsTaskControlCommands(
        string workerCommand,
        string rosCommand,
        string immediateState)
    {
        var taskRunId = Guid.NewGuid();
        var payload = JsonSerializer.SerializeToElement(new { taskRunId });

        var command = TaskCommandParser.Parse(workerCommand, payload);

        Assert.Equal(rosCommand, command.Command);
        Assert.Equal(immediateState, command.ImmediateState);
        Assert.Equal(
            taskRunId.ToString("D"),
            command.Envelope.GetProperty("parameters")
                .GetProperty("task_id")
                .GetString());
    }

    [Theory]
    [InlineData("EmergencyStop", "emergency_stop")]
    [InlineData("ResetEmergencyStop", "reset_emergency_stop")]
    public void MapsSessionSafetyCommands(string workerCommand, string rosCommand)
    {
        var command = TaskCommandParser.Parse(
            workerCommand,
            JsonSerializer.SerializeToElement(new { }));

        Assert.Equal(rosCommand, command.Command);
        Assert.Null(command.TaskRunId);
        Assert.Null(command.ImmediateState);
        Assert.Empty(command.Envelope.GetProperty("parameters").EnumerateObject());
    }

    [Fact]
    public void RejectsUnknownTaskType()
    {
        var payload = JsonSerializer.SerializeToElement(new
        {
            taskRunId = Guid.NewGuid(),
            taskType = "Unknown",
            parameters = new { }
        });

        Assert.Throws<InvalidOperationException>(
            () => TaskCommandParser.Parse("StartTask", payload));
    }

    [Fact]
    public void RejectsAmbiguousTaskTypeAliases()
    {
        var payload = JsonSerializer.SerializeToElement(new
        {
            taskRunId = Guid.NewGuid(),
            taskType = "Figure",
            type = "FollowLeader",
            parameters = new { }
        });

        Assert.Throws<InvalidOperationException>(
            () => TaskCommandParser.Parse("StartTask", payload));
    }
}
