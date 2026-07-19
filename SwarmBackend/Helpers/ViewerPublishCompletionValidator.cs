using System.Text.Json;

namespace SwarmBackend.Helpers;

public static class ViewerPublishCompletionValidator
{
    public static bool TryValidate(
        Guid sessionId,
        JsonElement commandPayload,
        JsonElement result,
        out string error)
    {
        if (commandPayload.ValueKind != JsonValueKind.Object
            || result.ValueKind != JsonValueKind.Object)
        {
            error = "Viewer completion payloads must be JSON objects.";
            return false;
        }

        if (TryGetProperty(result, "publishToken", out _))
        {
            error = "Viewer completion must not return the publish token.";
            return false;
        }

        if (!TryReadString(commandPayload, "publishToken", out var publishToken)
            || !ViewerPublishToken.TryHash(publishToken, out _))
        {
            error = "Viewer command did not contain a valid publish token.";
            return false;
        }

        if (!TryReadGuid(commandPayload, "leaseId", out var expectedLeaseId)
            || !TryReadGuid(result, "leaseId", out var leaseId)
            || expectedLeaseId != leaseId)
        {
            error = "Viewer completion did not match the issued lease.";
            return false;
        }

        if (!TryReadDateTime(commandPayload, "expiresAt", out var expectedExpiry)
            || !TryReadDateTime(result, "expiresAt", out var expiry)
            || expectedExpiry != expiry)
        {
            error = "Viewer completion did not match the lease expiry.";
            return false;
        }

        if (!TryReadGuid(result, "sessionId", out var completedSessionId)
            || completedSessionId != sessionId)
        {
            error = "Viewer completion did not match the assigned session.";
            return false;
        }

        if (!TryReadBoolean(result, "ready", out var ready) || !ready)
        {
            error = "Viewer publisher did not report readiness.";
            return false;
        }

        if (!TryReadString(result, "videoCodec", out var videoCodec)
            || !videoCodec.Equals("H264", StringComparison.OrdinalIgnoreCase))
        {
            error = "Viewer publisher did not confirm H.264 output.";
            return false;
        }

        foreach (var property in new[] { "source", "sourceId", "streamPath" })
        {
            if (!TryReadString(commandPayload, property, out var expected)
                || !TryReadString(result, property, out var actual)
                || !expected.Equals(actual, StringComparison.Ordinal))
            {
                error = $"Viewer completion did not match {property}.";
                return false;
            }
        }

        if (!TryReadOptionalString(
                commandPayload,
                "robotRuntimeId",
                out var expectedRobotRuntimeId)
            || !TryReadOptionalString(
                result,
                "robotRuntimeId",
                out var robotRuntimeId)
            || !string.Equals(
                expectedRobotRuntimeId,
                robotRuntimeId,
                StringComparison.Ordinal))
        {
            error = "Viewer completion did not match robotRuntimeId.";
            return false;
        }

        error = string.Empty;
        return true;
    }

    private static bool TryReadGuid(
        JsonElement element,
        string name,
        out Guid value)
    {
        value = Guid.Empty;
        return TryReadString(element, name, out var text)
            && Guid.TryParse(text, out value)
            && value != Guid.Empty;
    }

    private static bool TryReadDateTime(
        JsonElement element,
        string name,
        out DateTimeOffset value)
    {
        value = default;
        return TryGetProperty(element, name, out var property)
            && property.ValueKind == JsonValueKind.String
            && property.TryGetDateTimeOffset(out value);
    }

    private static bool TryReadBoolean(
        JsonElement element,
        string name,
        out bool value)
    {
        if (TryGetProperty(element, name, out var property)
            && property.ValueKind is JsonValueKind.True or JsonValueKind.False)
        {
            value = property.GetBoolean();
            return true;
        }

        value = false;
        return false;
    }

    private static bool TryReadString(
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

    private static bool TryReadOptionalString(
        JsonElement element,
        string name,
        out string? value)
    {
        if (!TryGetProperty(element, name, out var property)
            || property.ValueKind == JsonValueKind.Null)
        {
            value = null;
            return true;
        }

        if (property.ValueKind == JsonValueKind.String
            && !string.IsNullOrWhiteSpace(property.GetString()))
        {
            value = property.GetString();
            return true;
        }

        value = null;
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
