using System.Text.Json;

namespace SwarmWorker.Runtime;

public sealed record RosTaskCommand(
    string Command,
    Guid? TaskRunId,
    JsonElement Envelope,
    string? ImmediateState,
    double? ImmediateProgress);

public static class TaskCommandParser
{
    public static RosTaskCommand Parse(string commandType, JsonElement payload)
    {
        EnsureObject(payload);

        return commandType switch
        {
            "StartTask" => ParseStart(payload),
            "PauseTask" => ParseTaskControl(
                payload,
                "pause_task",
                "Paused"),
            "ResumeTask" => ParseTaskControl(
                payload,
                "resume_task",
                "Running"),
            "CancelTask" => ParseTaskControl(
                payload,
                "stop_task",
                "Cancelled"),
            "EmergencyStop" => ParseSessionControl(
                payload,
                "emergency_stop"),
            "ResetEmergencyStop" => ParseSessionControl(
                payload,
                "reset_emergency_stop"),
            _ => throw new NotSupportedException(
                $"Worker command type '{commandType}' is not a ROS task command.")
        };
    }

    private static RosTaskCommand ParseStart(JsonElement payload)
    {
        var taskRunId = ReadTaskRunId(payload);
        var taskType = ReadOptionalString(payload, "taskType");
        var legacyTaskType = ReadOptionalString(payload, "type");

        if (taskType is not null
            && legacyTaskType is not null
            && !taskType.Equals(legacyTaskType, StringComparison.OrdinalIgnoreCase))
        {
            throw new InvalidOperationException("taskType and type must identify the same task.");
        }

        var rosTaskType = (taskType ?? legacyTaskType) switch
        {
            string value when value.Equals(
                "FollowLeader",
                StringComparison.OrdinalIgnoreCase) => "follow_leader",
            string value when value.Equals(
                "Figure",
                StringComparison.OrdinalIgnoreCase) => "formation",
            string value when value.Equals(
                "CollaborativeTransport",
                StringComparison.OrdinalIgnoreCase) => "transport",
            _ => throw new InvalidOperationException(
                "taskType must be FollowLeader, Figure, or CollaborativeTransport.")
        };

        if (!TryGetProperty(payload, "parameters", out var parameters)
            || parameters.ValueKind != JsonValueKind.Object)
        {
            throw new InvalidOperationException("parameters is required and must be a JSON object.");
        }

        return new RosTaskCommand(
            "start_task",
            taskRunId,
            BuildStartEnvelope(taskRunId, rosTaskType, parameters),
            "Accepted",
            0);
    }

    private static RosTaskCommand ParseTaskControl(
        JsonElement payload,
        string rosCommand,
        string immediateState)
    {
        var taskRunId = ReadTaskRunId(payload);
        return new RosTaskCommand(
            rosCommand,
            taskRunId,
            JsonSerializer.SerializeToElement(new
            {
                command = rosCommand,
                parameters = new { task_id = taskRunId.ToString("D") }
            }),
            immediateState,
            null);
    }

    private static RosTaskCommand ParseSessionControl(
        JsonElement payload,
        string rosCommand)
    {
        if (payload.EnumerateObject().Any())
        {
            throw new InvalidOperationException(
                $"{rosCommand} payload must be an empty JSON object.");
        }

        return new RosTaskCommand(
            rosCommand,
            null,
            JsonSerializer.SerializeToElement(new
            {
                command = rosCommand,
                parameters = new { }
            }),
            null,
            null);
    }

    private static JsonElement BuildStartEnvelope(
        Guid taskRunId,
        string rosTaskType,
        JsonElement parameters)
    {
        using var stream = new MemoryStream();
        using (var writer = new Utf8JsonWriter(stream))
        {
            writer.WriteStartObject();
            writer.WriteString("command", "start_task");
            writer.WritePropertyName("parameters");
            writer.WriteStartObject();

            foreach (var property in parameters.EnumerateObject())
            {
                if (property.Name.Equals("task_id", StringComparison.OrdinalIgnoreCase)
                    || property.Name.Equals("task_type", StringComparison.OrdinalIgnoreCase))
                {
                    continue;
                }

                property.WriteTo(writer);
            }

            writer.WriteString("task_id", taskRunId.ToString("D"));
            writer.WriteString("task_type", rosTaskType);
            writer.WriteEndObject();
            writer.WriteEndObject();
        }

        using var document = JsonDocument.Parse(stream.ToArray());
        return document.RootElement.Clone();
    }

    private static Guid ReadTaskRunId(JsonElement payload)
    {
        var value = ReadOptionalString(payload, "taskRunId");
        if (!Guid.TryParse(value, out var taskRunId) || taskRunId == Guid.Empty)
        {
            throw new InvalidOperationException("taskRunId is required and must be a UUID.");
        }

        return taskRunId;
    }

    private static string? ReadOptionalString(JsonElement payload, string name)
    {
        return TryGetProperty(payload, name, out var value)
               && value.ValueKind == JsonValueKind.String
            ? value.GetString()
            : null;
    }

    private static bool TryGetProperty(
        JsonElement payload,
        string name,
        out JsonElement value)
    {
        foreach (var property in payload.EnumerateObject())
        {
            if (property.Name.Equals(name, StringComparison.OrdinalIgnoreCase))
            {
                value = property.Value;
                return true;
            }
        }

        value = default;
        return false;
    }

    private static void EnsureObject(JsonElement payload)
    {
        if (payload.ValueKind != JsonValueKind.Object)
        {
            throw new InvalidOperationException("Command payload must be a JSON object.");
        }
    }
}
