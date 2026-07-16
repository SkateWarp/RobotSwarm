using System.Text.Json;

namespace SwarmWorker.Runtime;

public static class EmergencyStopStatusParser
{
    public static bool Parse(string rostopicOutput)
    {
        var dataLine = rostopicOutput
            .Split('\n', StringSplitOptions.TrimEntries)
            .FirstOrDefault(line => line.StartsWith("data:", StringComparison.Ordinal));
        if (dataLine is null)
        {
            throw new FormatException("ROS swarm status output did not contain a data field.");
        }

        var message = dataLine["data:".Length..].Trim();
        if (message.StartsWith('"') && message.EndsWith('"'))
        {
            message = JsonSerializer.Deserialize<string>(message)
                ?? throw new FormatException("ROS swarm status contained a null data value.");
        }
        else if (message.StartsWith('\'') && message.EndsWith('\''))
        {
            message = message[1..^1].Replace("''", "'", StringComparison.Ordinal);
        }

        try
        {
            using var document = JsonDocument.Parse(message);
            if (document.RootElement.ValueKind != JsonValueKind.Object
                || !document.RootElement.TryGetProperty("emergency_stop", out var value)
                || value.ValueKind is not JsonValueKind.True and not JsonValueKind.False)
            {
                throw new FormatException(
                    "ROS swarm status did not contain a boolean emergency_stop value.");
            }

            return value.GetBoolean();
        }
        catch (JsonException exception)
        {
            throw new FormatException("ROS swarm status did not contain valid JSON.", exception);
        }
    }
}
