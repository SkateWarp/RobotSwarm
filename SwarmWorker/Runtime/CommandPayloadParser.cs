using System.Text.Json;
using System.Text.RegularExpressions;
using SwarmWorker.Configuration;

namespace SwarmWorker.Runtime;

public sealed record FleetCommandPayload(
    int DesiredRobotCount,
    string ArenaVersion,
    string SpawnPattern,
    IReadOnlyList<string> RobotIds);

public static class CommandPayloadParser
{
    private static readonly Regex SafeNamePattern =
        new(@"^[A-Za-z0-9_.-]+$", RegexOptions.Compiled);

    private static readonly HashSet<string> SpawnPatterns =
        new(StringComparer.OrdinalIgnoreCase) { "grid", "circle", "line" };

    public static FleetCommandPayload ParseFleet(
        JsonElement payload,
        WorkerOptions options,
        bool requireArena)
    {
        var desiredRobotCount = ReadRequiredInt(
            payload,
            "desiredRobotCount",
            alternativeName: "robotCount");
        if (desiredRobotCount < 1 || desiredRobotCount > options.MaxRobotsPerSession)
        {
            throw new InvalidOperationException(
                $"Desired robot count must be between 1 and {options.MaxRobotsPerSession}.");
        }

        var arenaVersion = ReadOptionalString(payload, "arenaVersion") ?? "arena-v1";
        if (requireArena && string.IsNullOrWhiteSpace(arenaVersion))
        {
            throw new InvalidOperationException("arenaVersion is required.");
        }

        if (!SafeNamePattern.IsMatch(arenaVersion))
        {
            throw new InvalidOperationException("arenaVersion contains unsupported characters.");
        }

        var spawnPattern = ReadOptionalString(payload, "spawnPattern") ?? "grid";
        if (!SpawnPatterns.Contains(spawnPattern))
        {
            throw new InvalidOperationException("spawnPattern must be grid, circle, or line.");
        }

        var robotIds = ReadRobotIds(payload, desiredRobotCount)
            ?? FleetRosterParser.GenerateRobotIds(desiredRobotCount);

        return new FleetCommandPayload(
            desiredRobotCount,
            arenaVersion,
            spawnPattern.ToLowerInvariant(),
            robotIds);
    }

    private static IReadOnlyList<string>? ReadRobotIds(JsonElement payload, int expectedCount)
    {
        if (!TryGetProperty(payload, "robotIds", out var robotIdsElement)
            || robotIdsElement.ValueKind is JsonValueKind.Null or JsonValueKind.Undefined)
        {
            return null;
        }

        if (robotIdsElement.ValueKind != JsonValueKind.Array)
        {
            throw new InvalidOperationException("robotIds must be an array.");
        }

        var robotIds = robotIdsElement
            .EnumerateArray()
            .Select(element => element.GetString())
            .ToArray();

        if (robotIds.Length != expectedCount
            || robotIds.Any(string.IsNullOrWhiteSpace)
            || robotIds.Any(robotId => !FleetRosterParser.IsValidRobotId(robotId!))
            || robotIds.Distinct(StringComparer.Ordinal).Count() != robotIds.Length)
        {
            throw new InvalidOperationException(
                "robotIds must contain exactly one distinct tb3_N ID per desired robot.");
        }

        return robotIds
            .Select(robotId => robotId!)
            .OrderBy(robotId => int.Parse(
                robotId[4..],
                System.Globalization.CultureInfo.InvariantCulture))
            .ToArray();
    }

    private static int ReadRequiredInt(
        JsonElement payload,
        string name,
        string? alternativeName = null)
    {
        if (TryGetProperty(payload, name, out var value)
            || (alternativeName is not null && TryGetProperty(payload, alternativeName, out value)))
        {
            if (value.ValueKind == JsonValueKind.Number && value.TryGetInt32(out var number))
            {
                return number;
            }
        }

        throw new InvalidOperationException($"{name} is required and must be an integer.");
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
        if (payload.ValueKind == JsonValueKind.Object)
        {
            foreach (var property in payload.EnumerateObject())
            {
                if (property.Name.Equals(name, StringComparison.OrdinalIgnoreCase))
                {
                    value = property.Value;
                    return true;
                }
            }
        }

        value = default;
        return false;
    }
}
