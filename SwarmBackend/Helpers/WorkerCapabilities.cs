using System.Text.Json;
using SwarmBackend.Entities;

namespace SwarmBackend.Helpers;

public static class WorkerCapabilities
{
    public const int CollaborativeTransportEvidenceVersion = 1;

    public static bool SupportsCommand(ComputeWorker worker, string commandType)
    {
        return ContainsString(worker, "commandTypes", commandType);
    }

    public static bool SupportsViewerSource(ComputeWorker worker, string source)
    {
        return ContainsString(worker, "viewerSources", source);
    }

    public static bool SupportsCollaborativeTransportEvidence(ComputeWorker worker)
    {
        var root = worker.Capabilities.RootElement;
        if (root.ValueKind != JsonValueKind.Object
            || !TryGetProperty(root, "taskOutcomes", out var taskOutcomes)
            || taskOutcomes.ValueKind != JsonValueKind.Object
            || !TryGetProperty(
                taskOutcomes,
                "collaborativeTransportEvidenceVersion",
                out var version)
            || version.ValueKind != JsonValueKind.Number
            || !version.TryGetInt32(out var parsedVersion))
        {
            return false;
        }

        return parsedVersion >= CollaborativeTransportEvidenceVersion;
    }

    private static bool ContainsString(
        ComputeWorker worker,
        string propertyName,
        string requestedValue)
    {
        if (string.IsNullOrWhiteSpace(requestedValue))
        {
            return false;
        }

        var root = worker.Capabilities.RootElement;
        if (root.ValueKind != JsonValueKind.Object)
        {
            return false;
        }

        foreach (var property in root.EnumerateObject())
        {
            if (!property.Name.Equals(
                    propertyName,
                    StringComparison.OrdinalIgnoreCase)
                || property.Value.ValueKind != JsonValueKind.Array)
            {
                continue;
            }

            return property.Value.EnumerateArray().Any(item =>
                item.ValueKind == JsonValueKind.String
                && string.Equals(
                    item.GetString(),
                    requestedValue,
                    StringComparison.OrdinalIgnoreCase));
        }

        return false;
    }

    public static int GetMaxRobotsPerSession(ComputeWorker worker)
    {
        var root = worker.Capabilities.RootElement;
        if (root.ValueKind != JsonValueKind.Object)
        {
            return 0;
        }

        foreach (var property in root.EnumerateObject())
        {
            if (property.Name.Equals(
                    "maxRobotsPerSession",
                    StringComparison.OrdinalIgnoreCase)
                && property.Value.ValueKind == JsonValueKind.Number
                && property.Value.TryGetInt32(out var maximum)
                && maximum is >= 1 and <= 1000)
            {
                return maximum;
            }
        }

        return 0;
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
