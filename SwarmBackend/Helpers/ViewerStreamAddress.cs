using SwarmBackend.Entities;

namespace SwarmBackend.Helpers;

public sealed record ViewerStreamAddress(
    Guid SessionId,
    ViewerSourceType Source,
    string? RobotRuntimeId)
{
    public string SourceId => Source switch
    {
        ViewerSourceType.Scene => $"scene-{SessionId:N}",
        ViewerSourceType.RobotCamera => $"robot-{SessionId:N}-{RobotRuntimeId}",
        _ => throw new InvalidOperationException("Unknown viewer source type.")
    };

    public string StreamPath => $"session/{SessionId:N}/{SourceId}";

    public bool Matches(ViewerLease lease)
    {
        return lease.SimulationSessionId == SessionId
            && lease.Source == Source
            && string.Equals(
                lease.RobotRuntimeId,
                RobotRuntimeId,
                StringComparison.Ordinal);
    }

    public static bool TryCreate(
        Guid sessionId,
        ViewerSourceType source,
        string? robotRuntimeId,
        out ViewerStreamAddress address)
    {
        address = null!;
        if (sessionId == Guid.Empty)
        {
            return false;
        }

        if (source == ViewerSourceType.Scene)
        {
            if (!string.IsNullOrWhiteSpace(robotRuntimeId))
            {
                return false;
            }

            address = new ViewerStreamAddress(sessionId, source, null);
            return true;
        }

        if (source != ViewerSourceType.RobotCamera
            || !IsValidRobotRuntimeId(robotRuntimeId))
        {
            return false;
        }

        address = new ViewerStreamAddress(sessionId, source, robotRuntimeId);
        return true;
    }

    public static bool TryParse(string? path, out ViewerStreamAddress address)
    {
        address = null!;
        var normalizedPath = path?.Trim('/');
        var parts = normalizedPath?.Split('/');
        if (parts is not { Length: 3 }
            || !string.Equals(parts[0], "session", StringComparison.Ordinal)
            || !Guid.TryParseExact(parts[1], "N", out var sessionId))
        {
            return false;
        }

        var sceneSourceId = $"scene-{sessionId:N}";
        if (string.Equals(parts[2], sceneSourceId, StringComparison.Ordinal))
        {
            return TryCreate(
                sessionId,
                ViewerSourceType.Scene,
                robotRuntimeId: null,
                out address)
                && string.Equals(
                    normalizedPath,
                    address.StreamPath,
                    StringComparison.Ordinal);
        }

        var robotPrefix = $"robot-{sessionId:N}-";
        if (!parts[2].StartsWith(robotPrefix, StringComparison.Ordinal))
        {
            return false;
        }

        return TryCreate(
            sessionId,
            ViewerSourceType.RobotCamera,
            parts[2][robotPrefix.Length..],
            out address)
            && string.Equals(
                normalizedPath,
                address.StreamPath,
                StringComparison.Ordinal);
    }

    public static string? BuildWhepUrl(
        string? publicBaseUrl,
        ViewerStreamAddress address)
    {
        return BuildPublicUrl(publicBaseUrl, address, "whep");
    }

    public static string? BuildHlsUrl(
        string? publicBaseUrl,
        ViewerStreamAddress address)
    {
        return BuildPublicUrl(publicBaseUrl, address, "index.m3u8");
    }

    private static bool IsValidRobotRuntimeId(string? value)
    {
        if (string.IsNullOrWhiteSpace(value) || value.Length > 100)
        {
            return false;
        }

        return value.All(character =>
            char.IsAsciiLetterOrDigit(character)
            || character is '-' or '_');
    }

    private static string? BuildPublicUrl(
        string? publicBaseUrl,
        ViewerStreamAddress address,
        string resource)
    {
        if (string.IsNullOrWhiteSpace(publicBaseUrl)
            || !Uri.TryCreate(publicBaseUrl, UriKind.Absolute, out var baseUri)
            || baseUri.Scheme is not ("http" or "https")
            || string.IsNullOrWhiteSpace(baseUri.Host)
            || !string.IsNullOrEmpty(baseUri.UserInfo)
            || !string.IsNullOrEmpty(baseUri.Query)
            || !string.IsNullOrEmpty(baseUri.Fragment))
        {
            return null;
        }

        return $"{publicBaseUrl.TrimEnd('/')}/{address.StreamPath}/{resource}";
    }
}
