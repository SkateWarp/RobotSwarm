using System.Text.Json;
using SwarmBackend.Entities;

namespace SwarmBackend.Helpers;

public static class WorkerCapabilities
{
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
}
