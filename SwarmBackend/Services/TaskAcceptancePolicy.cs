using System.Text.Json;
using SwarmBackend.Entities;

namespace SwarmBackend.Services;

public static class TaskAcceptancePolicy
{
    private const double CompleteProgress = 0.999;

    public static bool TryAccept(
        TaskRun task,
        double? reportedProgress,
        JsonElement? result,
        bool requireCollaborativeTransportEvidence,
        out string? reason)
    {
        if (!reportedProgress.HasValue
            || !double.IsFinite(reportedProgress.Value)
            || reportedProgress.Value < CompleteProgress)
        {
            reason = "The task reported completion without reaching measurable full progress.";
            return false;
        }

        if (task.Type != SwarmTaskRunType.CollaborativeTransport)
        {
            reason = null;
            return true;
        }

        // Keep the compatibility switch explicit. During the backend-first
        // rollout, the old ROS image cannot produce the evidence contract.
        // Production enables this only after the matching worker is live.
        if (!requireCollaborativeTransportEvidence)
        {
            reason = null;
            return true;
        }

        var session = task.SimulationSession;
        var desiredRobotCount = session?.DesiredRobotCount ?? 0;
        var roster = session?.Robots
            .Select(robot => robot.RuntimeId)
            .Where(runtimeId => !string.IsNullOrWhiteSpace(runtimeId))
            .ToHashSet(StringComparer.Ordinal)
            ?? new HashSet<string>(StringComparer.Ordinal);
        if (desiredRobotCount < 1
            || roster.Count != desiredRobotCount
            || roster.Count != (session?.Robots.Count ?? 0))
        {
            reason =
                "Collaborative transport cannot be verified without a complete authoritative robot roster.";
            return false;
        }

        if (!TryReadTransportEvidence(
                result,
                task.Id,
                desiredRobotCount,
                roster,
                out reason))
        {
            return false;
        }

        reason = null;
        return true;
    }

    private static bool TryReadTransportEvidence(
        JsonElement? result,
        Guid taskRunId,
        int robotCount,
        HashSet<string> roster,
        out string? reason)
    {
        if (!result.HasValue
            || result.Value.ValueKind != JsonValueKind.Object
            || !TryGetProperty(result.Value, "transport", out var transport)
            || transport.ValueKind != JsonValueKind.Object
            || !TryGetString(transport, "phase", out var phase)
            || !phase.Equals("DONE", StringComparison.OrdinalIgnoreCase)
            || !TryGetProperty(transport, "discovery", out var discovery)
            || discovery.ValueKind != JsonValueKind.Object
            || !TryGetString(discovery, "event", out var eventName)
            || eventName != "payload_found"
            || !TryGetString(discovery, "task_id", out var reportedTaskId)
            || !Guid.TryParse(reportedTaskId, out var parsedTaskId)
            || parsedTaskId != taskRunId
            || !TryGetProperty(discovery, "announced", out var announced)
            || announced.ValueKind != JsonValueKind.True
            || !TryGetString(discovery, "finder", out var finder)
            || !TryGetProperty(discovery, "notified_robots", out var notified)
            || notified.ValueKind != JsonValueKind.Array)
        {
            reason =
                "Collaborative transport finished without correlated payload discovery and DONE evidence.";
            return false;
        }

        var notifiedRobots = new HashSet<string>(StringComparer.Ordinal);
        foreach (var element in notified.EnumerateArray())
        {
            if (element.ValueKind != JsonValueKind.String
                || string.IsNullOrWhiteSpace(element.GetString())
                || !notifiedRobots.Add(element.GetString()!))
            {
                reason =
                    "Collaborative transport discovery did not identify a distinct recipient fleet.";
                return false;
            }
        }

        if (!roster.Contains(finder))
        {
            reason =
                "Collaborative transport discovery identified a finder outside the authoritative roster.";
            return false;
        }

        var expectedRecipients = roster
            .Where(robotId => !robotId.Equals(finder, StringComparison.Ordinal))
            .ToHashSet(StringComparer.Ordinal);
        if (!notifiedRobots.SetEquals(expectedRecipients))
        {
            reason =
                "Collaborative transport discovery did not notify every companion robot.";
            return false;
        }

        if (!TryGetProperty(transport, "all_pushers_confirmed", out var confirmed)
            || confirmed.ValueKind != JsonValueKind.True
            || !TryGetProperty(
                transport,
                "useful_contributor_count",
                out var contributorCount)
            || contributorCount.ValueKind != JsonValueKind.Number
            || !contributorCount.TryGetInt32(out var contributors)
            || contributors < robotCount
            || contributors > roster.Count)
        {
            reason =
                "Collaborative transport did not prove that every robot contributed to the synchronized push.";
            return false;
        }

        if (!TryGetProperty(
                transport,
                "useful_contributor_ids",
                out var contributorIds)
            || contributorIds.ValueKind != JsonValueKind.Array)
        {
            reason =
                "Collaborative transport did not identify the robots that contributed to the synchronized push.";
            return false;
        }

        var contributorRobots = new HashSet<string>(StringComparer.Ordinal);
        foreach (var element in contributorIds.EnumerateArray())
        {
            if (element.ValueKind != JsonValueKind.String
                || string.IsNullOrWhiteSpace(element.GetString())
                || !contributorRobots.Add(element.GetString()!))
            {
                reason =
                    "Collaborative transport reported an invalid contributor identity list.";
                return false;
            }
        }

        if (contributorRobots.Count != contributors
            || !contributorRobots.SetEquals(roster))
        {
            reason =
                "Collaborative transport contributor identities did not match the authoritative robot roster.";
            return false;
        }

        reason = null;
        return true;
    }

    private static bool TryGetString(
        JsonElement element,
        string name,
        out string value)
    {
        if (TryGetProperty(element, name, out var property)
            && property.ValueKind == JsonValueKind.String
            && !string.IsNullOrWhiteSpace(property.GetString()))
        {
            value = property.GetString()!;
            return true;
        }

        value = string.Empty;
        return false;
    }

    private static bool TryGetProperty(
        JsonElement element,
        string name,
        out JsonElement value)
    {
        foreach (var property in element.EnumerateObject())
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
}
