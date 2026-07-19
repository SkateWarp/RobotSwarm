using System.Text.Json;

namespace SwarmWorker.Runtime;

public enum ViewerSourceKind
{
    Scene,
    RobotCamera
}

public sealed record ViewerSourceCommand(
    Guid LeaseId,
    DateTimeOffset ExpiresAt,
    string PublishToken,
    ViewerSourceKind Source,
    string? RobotRuntimeId,
    string SourceId,
    string StreamPath);

public static class ViewerSourceCommandParser
{
    public static ViewerSourceCommand Parse(
        Guid sessionId,
        JsonElement payload,
        DateTimeOffset now,
        TimeSpan maximumLifetime)
    {
        if (sessionId == Guid.Empty)
        {
            throw new InvalidOperationException("Viewer commands require a session UUID.");
        }

        if (payload.ValueKind != JsonValueKind.Object)
        {
            throw new InvalidOperationException(
                "SetViewerSource payload must be a JSON object.");
        }

        if (maximumLifetime <= TimeSpan.Zero)
        {
            throw new ArgumentOutOfRangeException(nameof(maximumLifetime));
        }

        var leaseIdText = ReadRequiredString(payload, "leaseId");
        if (!Guid.TryParse(leaseIdText, out var leaseId) || leaseId == Guid.Empty)
        {
            throw new InvalidOperationException("leaseId must be a non-empty UUID.");
        }

        var publishToken = ReadRequiredString(payload, "publishToken");
        if (!IsCanonicalPublishToken(publishToken))
        {
            throw new InvalidOperationException(
                "publishToken must be a 32-byte Base64Url token.");
        }

        var expiresAtText = ReadRequiredString(payload, "expiresAt");
        if (!DateTimeOffset.TryParse(
                expiresAtText,
                System.Globalization.CultureInfo.InvariantCulture,
                System.Globalization.DateTimeStyles.AssumeUniversal
                    | System.Globalization.DateTimeStyles.AdjustToUniversal,
                out var expiresAt)
            || expiresAt <= now
            || expiresAt > now + maximumLifetime + TimeSpan.FromSeconds(30))
        {
            throw new InvalidOperationException(
                "expiresAt must be a future UTC timestamp within the worker lease limit.");
        }

        var sourceText = ReadRequiredString(payload, "source");
        var source = sourceText switch
        {
            string value when value.Equals(
                "Scene",
                StringComparison.OrdinalIgnoreCase) => ViewerSourceKind.Scene,
            string value when value.Equals(
                "RobotCamera",
                StringComparison.OrdinalIgnoreCase) => ViewerSourceKind.RobotCamera,
            _ => throw new InvalidOperationException(
                "Viewer source must be Scene or RobotCamera.")
        };

        var robotRuntimeId = ReadOptionalString(payload, "robotRuntimeId");
        if (source == ViewerSourceKind.Scene && robotRuntimeId is not null)
        {
            throw new InvalidOperationException(
                "robotRuntimeId is only valid for RobotCamera.");
        }

        if (source == ViewerSourceKind.RobotCamera
            && (robotRuntimeId is null
                || !IsCanonicalRobotRuntimeId(robotRuntimeId)))
        {
            throw new InvalidOperationException(
                "RobotCamera requires a canonical TurtleBot3 runtime ID.");
        }

        var sourceId = ReadRequiredString(payload, "sourceId");
        var streamPath = ReadRequiredString(payload, "streamPath");
        var expectedSourceId = source == ViewerSourceKind.Scene
            ? $"scene-{sessionId:N}"
            : $"robot-{sessionId:N}-{robotRuntimeId}";
        var expectedStreamPath = $"session/{sessionId:N}/{expectedSourceId}";

        if (!sourceId.Equals(expectedSourceId, StringComparison.Ordinal)
            || !streamPath.Equals(expectedStreamPath, StringComparison.Ordinal))
        {
            throw new InvalidOperationException(
                "Viewer source identifiers do not match the assigned session.");
        }

        return new ViewerSourceCommand(
            leaseId,
            expiresAt,
            publishToken,
            source,
            robotRuntimeId,
            sourceId,
            streamPath);
    }

    private static bool IsCanonicalPublishToken(string value)
    {
        if (value.Length != 43
            || value.Any(character => !char.IsAsciiLetterOrDigit(character)
                && character is not '_' and not '-'))
        {
            return false;
        }

        try
        {
            var padded = value.Replace('-', '+').Replace('_', '/') + "=";
            var decoded = Convert.FromBase64String(padded);
            var canonical = Convert.ToBase64String(decoded)
                .TrimEnd('=')
                .Replace('+', '-')
                .Replace('/', '_');
            return decoded.Length == 32
                && canonical.Equals(value, StringComparison.Ordinal);
        }
        catch (FormatException)
        {
            return false;
        }
    }

    private static bool IsCanonicalRobotRuntimeId(string value)
    {
        return value.StartsWith("tb3_", StringComparison.Ordinal)
            && int.TryParse(
                value.AsSpan(4),
                System.Globalization.NumberStyles.None,
                System.Globalization.CultureInfo.InvariantCulture,
                out var ordinal)
            && ordinal >= 0
            && value.Equals($"tb3_{ordinal}", StringComparison.Ordinal);
    }

    private static string ReadRequiredString(JsonElement payload, string name)
    {
        var value = ReadOptionalString(payload, name);
        if (value is null)
        {
            throw new InvalidOperationException($"{name} is required and must be a string.");
        }

        return value;
    }

    private static string? ReadOptionalString(JsonElement payload, string name)
    {
        if (!TryGetProperty(payload, name, out var value)
            || value.ValueKind is JsonValueKind.Null or JsonValueKind.Undefined)
        {
            return null;
        }

        if (value.ValueKind != JsonValueKind.String)
        {
            throw new InvalidOperationException($"{name} must be a string when provided.");
        }

        var text = value.GetString();
        if (string.IsNullOrWhiteSpace(text)
            || !text.Equals(text.Trim(), StringComparison.Ordinal))
        {
            throw new InvalidOperationException($"{name} must not be blank or padded.");
        }

        return text;
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
}
