using System.Text.Json;

namespace SwarmWorker.Runtime;

public sealed record RosTaskStatus(
    Guid TaskRunId,
    string State,
    double? Progress,
    JsonElement? Result,
    string? Error);

public static class TaskStatusParser
{
    public static RosTaskStatus? Parse(string rostopicOutput)
    {
        var message = ReadRosString(rostopicOutput);

        JsonElement root;
        try
        {
            using var document = JsonDocument.Parse(message);
            root = document.RootElement.Clone();
        }
        catch (JsonException exception)
        {
            throw new FormatException("ROS swarm status did not contain valid JSON.", exception);
        }

        if (root.ValueKind != JsonValueKind.Object
            || !root.TryGetProperty("task", out var task)
            || task.ValueKind != JsonValueKind.Object)
        {
            throw new FormatException("ROS swarm status did not contain a task object.");
        }

        var rosState = task.TryGetProperty("status", out var stateElement)
                       && stateElement.ValueKind == JsonValueKind.String
            ? stateElement.GetString()
            : null;
        var state = MapState(rosState);
        if (state is null)
        {
            return null;
        }

        var taskIdText = task.TryGetProperty("task_id", out var taskIdElement)
                         && taskIdElement.ValueKind == JsonValueKind.String
            ? taskIdElement.GetString()
            : null;
        if (!Guid.TryParse(taskIdText, out var taskRunId) || taskRunId == Guid.Empty)
        {
            throw new FormatException("ROS swarm status contained an invalid task_id.");
        }

        double? progress = null;
        if (task.TryGetProperty("progress", out var progressElement)
            && progressElement.ValueKind is not JsonValueKind.Null
            and not JsonValueKind.Undefined)
        {
            if (progressElement.ValueKind != JsonValueKind.Number
                || !progressElement.TryGetDouble(out var value)
                || !double.IsFinite(value))
            {
                throw new FormatException("ROS swarm status contained invalid task progress.");
            }

            progress = Math.Clamp(value, 0, 1);
        }

        var error = task.TryGetProperty("error", out var taskError)
                    && taskError.ValueKind == JsonValueKind.String
            ? taskError.GetString()
            : root.TryGetProperty("error", out var rootError)
              && rootError.ValueKind == JsonValueKind.String
                ? rootError.GetString()
                : null;

        JsonElement? result = null;
        if (task.TryGetProperty("result", out var resultElement)
            && resultElement.ValueKind is not JsonValueKind.Null
            and not JsonValueKind.Undefined)
        {
            result = resultElement.Clone();
        }

        return new RosTaskStatus(taskRunId, state, progress, result, error);
    }

    private static string? MapState(string? state)
    {
        return state?.ToLowerInvariant() switch
        {
            "initializing" => "Accepted",
            "running" => "Running",
            "paused" => "Paused",
            "completed" => "Completed",
            "failed" => "Failed",
            "stopped" or "cancelled" or "canceled" => "Cancelled",
            _ => null
        };
    }

    private static string ReadRosString(string rostopicOutput)
    {
        var dataLine = rostopicOutput
            .Split('\n', StringSplitOptions.TrimEntries)
            .FirstOrDefault(line => line.StartsWith("data:", StringComparison.Ordinal));
        if (dataLine is null)
        {
            throw new FormatException("ROS swarm status output did not contain a data field.");
        }

        var value = dataLine["data:".Length..].Trim();
        if (value.StartsWith('"') && value.EndsWith('"'))
        {
            return JsonSerializer.Deserialize<string>(value)
                ?? throw new FormatException("ROS swarm status contained a null data value.");
        }

        if (value.StartsWith('\'') && value.EndsWith('\''))
        {
            return value[1..^1].Replace("''", "'", StringComparison.Ordinal);
        }

        return value;
    }
}
