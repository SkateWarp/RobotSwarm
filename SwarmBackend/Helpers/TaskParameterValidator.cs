using System.Text.Json;
using SwarmBackend.Entities;

namespace SwarmBackend.Helpers;

public static class TaskParameterValidator
{
    private static readonly HashSet<string> FollowModes = new(StringComparer.OrdinalIgnoreCase)
    {
        "circular", "square", "figure8"
    };

    private static readonly HashSet<string> FormationShapes = new(StringComparer.OrdinalIgnoreCase)
    {
        "triangle", "square", "circle", "line", "v", "v_formation", "diamond"
    };

    private const string FormationLetters = "ABCDEFGHIJKLMNOPRSTUVWXYZ";

    public static bool TryValidate(
        SwarmTaskRunType taskType,
        JsonElement parameters,
        out string? error)
    {
        if (parameters.ValueKind != JsonValueKind.Object)
        {
            error = "Parameters must be a JSON object.";
            return false;
        }

        if (!TryReadConfig(parameters, out var config, out error))
        {
            return false;
        }

        return taskType switch
        {
            SwarmTaskRunType.FollowLeader => ValidateFollowLeader(parameters, config, out error),
            SwarmTaskRunType.Figure => ValidateFormation(parameters, config, out error),
            SwarmTaskRunType.CollaborativeTransport =>
                ValidateTransport(parameters, config, out error),
            _ => Fail("The task type is not supported.", out error)
        };
    }

    private static bool ValidateFollowLeader(
        JsonElement parameters,
        JsonElement config,
        out string? error)
    {
        if (!TryReadAliasedString(
                parameters,
                config,
                "leader_mode",
                "circular",
                out var mode,
                out error))
        {
            return false;
        }

        if (!FollowModes.Contains(mode))
        {
            return Fail(
                "leader_mode must be circular, square, or figure8.",
                out error);
        }

        if (!TryReadNumber(config, "radius", 2, 0.5, 4, out _, out error)
            || !TryReadNumber(
                config,
                "follow_distance",
                0.7,
                0.35,
                2,
                out _,
                out error))
        {
            return false;
        }

        error = null;
        return true;
    }

    private static bool ValidateFormation(
        JsonElement parameters,
        JsonElement config,
        out string? error)
    {
        if (!TryReadAliasedString(
                parameters,
                config,
                "formation_type",
                "triangle",
                out var formationType,
                out error))
        {
            return false;
        }

        var isLetter = formationType.Length == 1
            && FormationLetters.Contains(formationType, StringComparison.OrdinalIgnoreCase);
        if (!isLetter && !FormationShapes.Contains(formationType))
        {
            return Fail("formation_type is not supported.", out error);
        }

        if (!TryReadAliasedString(
                parameters,
                config,
                "movement_mode",
                "static",
                out var movementMode,
                out error))
        {
            return false;
        }

        if (!movementMode.Equals("static", StringComparison.OrdinalIgnoreCase))
        {
            return Fail("movement_mode must be static.", out error);
        }

        return TryReadNumber(config, "spacing", 1, 0.35, 2, out _, out error);
    }

    private static bool ValidateTransport(
        JsonElement parameters,
        JsonElement config,
        out string? error)
    {
        if (!TryReadAliasedNumber(
                parameters,
                config,
                "target_x",
                3,
                -4,
                4,
                out _,
                out error)
            || !TryReadAliasedNumber(
                parameters,
                config,
                "target_y",
                3,
                -4,
                4,
                out _,
                out error))
        {
            return false;
        }

        if (!TryReadString(config, "transport_planner", "grf", out var planner, out error))
        {
            return false;
        }

        return planner.Equals("grf", StringComparison.OrdinalIgnoreCase)
            || Fail("transport_planner must be grf.", out error);
    }

    private static bool TryReadConfig(
        JsonElement parameters,
        out JsonElement config,
        out string? error)
    {
        if (!parameters.TryGetProperty("config", out config))
        {
            config = JsonSerializer.SerializeToElement(new { });
            error = null;
            return true;
        }

        if (config.ValueKind != JsonValueKind.Object)
        {
            return Fail("config must be a JSON object.", out error);
        }

        error = null;
        return true;
    }

    private static bool TryReadAliasedString(
        JsonElement parameters,
        JsonElement config,
        string name,
        string fallback,
        out string value,
        out string? error)
    {
        var hasParameter = parameters.TryGetProperty(name, out var parameterValue);
        var hasConfig = config.TryGetProperty(name, out var configValue);
        var parsedParameter = fallback;
        var parsedConfig = fallback;
        if (hasParameter
            && !TryReadStringValue(parameterValue, name, out parsedParameter, out error))
        {
            value = string.Empty;
            return false;
        }

        if (hasConfig
            && !TryReadStringValue(configValue, $"config.{name}", out parsedConfig, out error))
        {
            value = string.Empty;
            return false;
        }

        if (hasParameter
            && hasConfig
            && !parsedParameter.Equals(parsedConfig, StringComparison.OrdinalIgnoreCase))
        {
            value = string.Empty;
            return Fail($"{name} and config.{name} must match.", out error);
        }

        value = hasParameter ? parsedParameter : hasConfig ? parsedConfig : fallback;
        error = null;
        return true;
    }

    private static bool TryReadString(
        JsonElement source,
        string name,
        string fallback,
        out string value,
        out string? error)
    {
        if (!source.TryGetProperty(name, out var element))
        {
            value = fallback;
            error = null;
            return true;
        }

        return TryReadStringValue(element, name, out value, out error);
    }

    private static bool TryReadStringValue(
        JsonElement element,
        string label,
        out string value,
        out string? error)
    {
        if (element.ValueKind != JsonValueKind.String
            || string.IsNullOrWhiteSpace(element.GetString()))
        {
            value = string.Empty;
            return Fail($"{label} must be a non-empty string.", out error);
        }

        value = element.GetString()!.Trim();
        error = null;
        return true;
    }

    private static bool TryReadAliasedNumber(
        JsonElement parameters,
        JsonElement config,
        string name,
        double fallback,
        double minimum,
        double maximum,
        out double value,
        out string? error)
    {
        var hasParameter = parameters.TryGetProperty(name, out var parameterValue);
        var hasConfig = config.TryGetProperty(name, out var configValue);
        var parsedParameter = fallback;
        var parsedConfig = fallback;
        if (hasParameter
            && !TryReadNumberValue(
                parameterValue,
                name,
                minimum,
                maximum,
                out parsedParameter,
                out error))
        {
            value = default;
            return false;
        }

        if (hasConfig
            && !TryReadNumberValue(
                configValue,
                $"config.{name}",
                minimum,
                maximum,
                out parsedConfig,
                out error))
        {
            value = default;
            return false;
        }

        if (hasParameter && hasConfig && parsedParameter != parsedConfig)
        {
            value = default;
            return Fail($"{name} and config.{name} must match.", out error);
        }

        value = hasParameter ? parsedParameter : hasConfig ? parsedConfig : fallback;
        error = null;
        return true;
    }

    private static bool TryReadNumber(
        JsonElement source,
        string name,
        double fallback,
        double minimum,
        double maximum,
        out double value,
        out string? error)
    {
        if (!source.TryGetProperty(name, out var element))
        {
            value = fallback;
            error = null;
            return true;
        }

        return TryReadNumberValue(element, name, minimum, maximum, out value, out error);
    }

    private static bool TryReadNumberValue(
        JsonElement element,
        string label,
        double minimum,
        double maximum,
        out double value,
        out string? error)
    {
        if (element.ValueKind != JsonValueKind.Number
            || !element.TryGetDouble(out value)
            || !double.IsFinite(value))
        {
            value = default;
            return Fail($"{label} must be a finite number.", out error);
        }

        if (value < minimum || value > maximum)
        {
            return Fail($"{label} must be between {minimum} and {maximum}.", out error);
        }

        error = null;
        return true;
    }

    private static bool Fail(string message, out string? error)
    {
        error = message;
        return false;
    }
}
