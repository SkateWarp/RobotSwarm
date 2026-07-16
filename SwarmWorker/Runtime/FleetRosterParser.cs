using System.Text.Json;
using System.Text.RegularExpressions;

namespace SwarmWorker.Runtime;

public static class FleetRosterParser
{
    private static readonly Regex RobotIdPattern =
        new(@"^tb3_([0-9]+)$", RegexOptions.Compiled);

    public static IReadOnlyList<string> Parse(string rostopicOutput)
    {
        var dataLine = rostopicOutput
            .Split('\n', StringSplitOptions.TrimEntries)
            .FirstOrDefault(line => line.StartsWith("data:", StringComparison.Ordinal));

        if (dataLine is null)
        {
            throw new FormatException("ROS fleet roster output did not contain a data field.");
        }

        var value = dataLine["data:".Length..].Trim();
        if (value.Length == 0 || value is "''" or "\"\"")
        {
            return Array.Empty<string>();
        }

        if (value.StartsWith('"') && value.EndsWith('"'))
        {
            value = JsonSerializer.Deserialize<string>(value)
                ?? throw new FormatException("ROS fleet roster contained a null data value.");
        }
        else if (value.StartsWith('\'') && value.EndsWith('\''))
        {
            value = value[1..^1].Replace("''", "'", StringComparison.Ordinal);
        }

        if (string.IsNullOrWhiteSpace(value))
        {
            return Array.Empty<string>();
        }

        var robotIds = value
            .Split(',', StringSplitOptions.RemoveEmptyEntries | StringSplitOptions.TrimEntries)
            .Distinct(StringComparer.Ordinal)
            .ToList();

        if (robotIds.Any(robotId => !RobotIdPattern.IsMatch(robotId)))
        {
            throw new FormatException("ROS fleet roster contained an invalid TurtleBot3 runtime ID.");
        }

        robotIds.Sort(CompareRobotIds);
        return robotIds;
    }

    public static IReadOnlyList<string> GenerateRobotIds(int desiredRobotCount)
    {
        if (desiredRobotCount < 0)
        {
            throw new ArgumentOutOfRangeException(nameof(desiredRobotCount));
        }

        return Enumerable.Range(0, desiredRobotCount)
            .Select(index => $"tb3_{index}")
            .ToArray();
    }

    public static bool IsValidRobotId(string robotId) => RobotIdPattern.IsMatch(robotId);

    private static int CompareRobotIds(string left, string right)
    {
        var leftIndex = int.Parse(
            RobotIdPattern.Match(left).Groups[1].Value,
            System.Globalization.CultureInfo.InvariantCulture);
        var rightIndex = int.Parse(
            RobotIdPattern.Match(right).Groups[1].Value,
            System.Globalization.CultureInfo.InvariantCulture);
        return leftIndex.CompareTo(rightIndex);
    }
}
