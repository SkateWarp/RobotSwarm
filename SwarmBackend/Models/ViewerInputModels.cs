using System.Text.Json;
using System.Text.Json.Serialization;

namespace SwarmBackend.Models;

public sealed record ViewerInputEvent(
    string Type,
    [property: JsonIgnore(Condition = JsonIgnoreCondition.WhenWritingNull)] double? X = null,
    [property: JsonIgnore(Condition = JsonIgnoreCondition.WhenWritingNull)] double? Y = null,
    [property: JsonIgnore(Condition = JsonIgnoreCondition.WhenWritingNull)] int? Button = null,
    [property: JsonIgnore(Condition = JsonIgnoreCondition.WhenWritingNull)] double? DeltaX = null,
    [property: JsonIgnore(Condition = JsonIgnoreCondition.WhenWritingNull)] double? DeltaY = null,
    [property: JsonIgnore(Condition = JsonIgnoreCondition.WhenWritingNull)] string? Code = null);

public sealed record ViewerInputEnvelope(
    Guid SessionId,
    Guid LeaseId,
    ViewerInputEvent Input);

public sealed record ViewerInputReleaseEnvelope(
    Guid SessionId,
    Guid LeaseId);

public sealed record ViewerControlAuthorizationResponse(
    Guid SessionId,
    Guid LeaseId,
    DateTimeOffset AuthorizedUntil,
    int MaximumEventsPerSecond);

internal static class ViewerInputNormalizer
{
    internal const double MaximumWheelDelta = 1000;

    private static readonly HashSet<string> PointerMoveProperties =
        new(StringComparer.Ordinal) { "type", "x", "y" };
    private static readonly HashSet<string> PointerButtonProperties =
        new(StringComparer.Ordinal) { "type", "x", "y", "button" };
    private static readonly HashSet<string> WheelProperties =
        new(StringComparer.Ordinal) { "type", "x", "y", "deltaX", "deltaY" };
    private static readonly HashSet<string> KeyProperties =
        new(StringComparer.Ordinal) { "type", "code" };
    private static readonly HashSet<string> ReleaseProperties =
        new(StringComparer.Ordinal) { "type" };
    private static readonly HashSet<string> AllowedKeyCodes = BuildAllowedKeyCodes();

    public static bool TryNormalize(
        JsonElement value,
        out ViewerInputEvent? input,
        out string error)
    {
        input = null;
        error = "Viewer input must be a JSON object.";
        if (value.ValueKind != JsonValueKind.Object)
        {
            return false;
        }

        var properties = new Dictionary<string, JsonElement>(StringComparer.Ordinal);
        foreach (var property in value.EnumerateObject())
        {
            if (!properties.TryAdd(property.Name, property.Value))
            {
                error = $"Viewer input contains duplicate property '{property.Name}'.";
                return false;
            }
        }

        if (!TryReadString(properties, "type", out var type))
        {
            error = "Viewer input type is required.";
            return false;
        }

        var allowedProperties = type switch
        {
            "pointerMove" => PointerMoveProperties,
            "pointerDown" or "pointerUp" => PointerButtonProperties,
            "wheel" => WheelProperties,
            "keyDown" or "keyUp" => KeyProperties,
            "releaseAll" => ReleaseProperties,
            _ => null
        };
        if (allowedProperties is null)
        {
            error = "Viewer input type is not supported.";
            return false;
        }

        var unexpectedProperty = properties.Keys.FirstOrDefault(
            property => !allowedProperties.Contains(property));
        if (unexpectedProperty is not null)
        {
            error = $"Property '{unexpectedProperty}' is not valid for {type}.";
            return false;
        }

        if (type == "releaseAll")
        {
            input = new ViewerInputEvent(type);
            error = string.Empty;
            return true;
        }

        if (type is "keyDown" or "keyUp")
        {
            if (!TryReadString(properties, "code", out var code)
                || !AllowedKeyCodes.Contains(code))
            {
                error = "Keyboard code is not allowed.";
                return false;
            }

            input = new ViewerInputEvent(type, Code: code);
            error = string.Empty;
            return true;
        }

        if (!TryReadFiniteNumber(properties, "x", out var x)
            || !TryReadFiniteNumber(properties, "y", out var y)
            || x is < 0 or > 1
            || y is < 0 or > 1)
        {
            error = "Pointer coordinates must be finite numbers between 0 and 1.";
            return false;
        }

        if (type is "pointerDown" or "pointerUp")
        {
            if (!properties.TryGetValue("button", out var buttonValue)
                || !buttonValue.TryGetInt32(out var button)
                || button is < 0 or > 2)
            {
                error = "Pointer button must be an integer between 0 and 2.";
                return false;
            }

            input = new ViewerInputEvent(type, x, y, button);
            error = string.Empty;
            return true;
        }

        if (type == "wheel")
        {
            if (!TryReadFiniteNumber(properties, "deltaX", out var deltaX)
                || !TryReadFiniteNumber(properties, "deltaY", out var deltaY)
                || Math.Abs(deltaX) > MaximumWheelDelta
                || Math.Abs(deltaY) > MaximumWheelDelta)
            {
                error = $"Wheel deltas must be finite and between -{MaximumWheelDelta} and {MaximumWheelDelta}.";
                return false;
            }

            input = new ViewerInputEvent(type, x, y, DeltaX: deltaX, DeltaY: deltaY);
            error = string.Empty;
            return true;
        }

        input = new ViewerInputEvent(type, x, y);
        error = string.Empty;
        return true;
    }

    private static bool TryReadString(
        IReadOnlyDictionary<string, JsonElement> properties,
        string name,
        out string value)
    {
        value = string.Empty;
        return properties.TryGetValue(name, out var element)
            && element.ValueKind == JsonValueKind.String
            && !string.IsNullOrWhiteSpace(value = element.GetString()!);
    }

    private static bool TryReadFiniteNumber(
        IReadOnlyDictionary<string, JsonElement> properties,
        string name,
        out double value)
    {
        value = default;
        return properties.TryGetValue(name, out var element)
            && element.TryGetDouble(out value)
            && double.IsFinite(value);
    }

    private static HashSet<string> BuildAllowedKeyCodes()
    {
        var codes = new HashSet<string>(StringComparer.Ordinal)
        {
            "ArrowDown", "ArrowLeft", "ArrowRight", "ArrowUp",
            "Backspace", "Delete", "End", "Enter", "Escape", "Home",
            "Insert", "PageDown", "PageUp", "Space", "Tab",
            "ShiftLeft", "ShiftRight", "ControlLeft", "ControlRight",
            "AltLeft", "AltRight",
            "Backquote", "Backslash", "BracketLeft", "BracketRight",
            "Comma", "Equal", "Minus", "Period", "Quote", "Semicolon", "Slash"
        };
        for (var letter = 'A'; letter <= 'Z'; letter++)
        {
            codes.Add($"Key{letter}");
        }

        for (var digit = 0; digit <= 9; digit++)
        {
            codes.Add($"Digit{digit}");
        }

        for (var function = 1; function <= 12; function++)
        {
            codes.Add($"F{function}");
        }

        return codes;
    }
}
