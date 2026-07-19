using System.Text.Json;
using System.Text.Json.Serialization;

namespace SwarmWorker.Contracts;

public sealed record WorkerRegistrationRequest(
    string? ImageVersion,
    JsonElement Capabilities);

public sealed record WorkerHeartbeatRequest(
    string? ImageVersion,
    JsonElement Capabilities,
    // Wire name retained for compatibility; includes stopped containers until cleanup removes them.
    IReadOnlyList<Guid> ActiveSessionIds);

public sealed record WorkerRegistrationResponse(
    Guid WorkerId,
    string Name,
    string State,
    int MaxConcurrentSessions,
    DateTime ServerTime);

public sealed record WorkerCommandEnvelope(
    Guid Id,
    Guid SessionId,
    string Type,
    string IdempotencyKey,
    Guid CorrelationId,
    long Sequence,
    JsonElement Payload,
    DateTime CreatedAt);

public sealed record WorkerCommandCompletionRequest(
    Guid CommandId,
    JsonElement? Result);

public sealed record WorkerCommandFailureRequest(
    Guid CommandId,
    string Error);

public sealed record WorkerEmergencyStopReport(
    Guid SessionId,
    bool Active,
    string? Reason);

public sealed record SessionEventReport(
    Guid SessionId,
    string State,
    string? FailureReason,
    JsonElement? Payload);

public sealed record TaskEventReport(
    Guid SessionId,
    Guid TaskRunId,
    string State,
    double? Progress,
    JsonElement? Result,
    string? Error);

public sealed record ViewerInputEvent(
    string Type,
    [property: JsonIgnore(Condition = JsonIgnoreCondition.WhenWritingNull)] double? X = null,
    [property: JsonIgnore(Condition = JsonIgnoreCondition.WhenWritingNull)] double? Y = null,
    [property: JsonIgnore(Condition = JsonIgnoreCondition.WhenWritingNull)] int? Button = null,
    [property: JsonIgnore(Condition = JsonIgnoreCondition.WhenWritingNull)] double? DeltaX = null,
    [property: JsonIgnore(Condition = JsonIgnoreCondition.WhenWritingNull)] double? DeltaY = null,
    [property: JsonIgnore(Condition = JsonIgnoreCondition.WhenWritingNull)] string? Code = null)
{
    [JsonExtensionData]
    public IDictionary<string, JsonElement>? ExtraProperties { get; init; }
}

public sealed record ViewerInputEnvelope(
    Guid SessionId,
    Guid LeaseId,
    ViewerInputEvent Input);

public sealed record ViewerInputReleaseEnvelope(
    Guid SessionId,
    Guid LeaseId);

public static class ViewerInputEventValidator
{
    public const double MaximumWheelDelta = 1000;
    public static readonly TimeSpan ReleaseGracePeriod = TimeSpan.FromSeconds(5);

    private static readonly HashSet<string> AllowedKeyCodes = BuildAllowedKeyCodes();

    public static bool IsValid(ViewerInputEvent? input)
    {
        if (input is null || input.ExtraProperties is { Count: > 0 })
        {
            return false;
        }

        return input.Type switch
        {
            "pointerMove" => HasCoordinates(input)
                && input.Button is null
                && input.DeltaX is null
                && input.DeltaY is null
                && input.Code is null,
            "pointerDown" or "pointerUp" => HasCoordinates(input)
                && input.Button is >= 0 and <= 2
                && input.DeltaX is null
                && input.DeltaY is null
                && input.Code is null,
            "wheel" => HasCoordinates(input)
                && IsBoundedDelta(input.DeltaX)
                && IsBoundedDelta(input.DeltaY)
                && input.Button is null
                && input.Code is null,
            "keyDown" or "keyUp" => input.X is null
                && input.Y is null
                && input.Button is null
                && input.DeltaX is null
                && input.DeltaY is null
                && input.Code is not null
                && AllowedKeyCodes.Contains(input.Code),
            "releaseAll" => input.X is null
                && input.Y is null
                && input.Button is null
                && input.DeltaX is null
                && input.DeltaY is null
                && input.Code is null,
            _ => false
        };
    }

    public static bool IsReleaseAll(ViewerInputEvent? input) =>
        IsValid(input) && input!.Type == "releaseAll";

    private static bool HasCoordinates(ViewerInputEvent input) =>
        input.X is >= 0 and <= 1
        && input.Y is >= 0 and <= 1
        && double.IsFinite(input.X.Value)
        && double.IsFinite(input.Y.Value);

    private static bool IsBoundedDelta(double? value) =>
        value.HasValue
        && double.IsFinite(value.Value)
        && Math.Abs(value.Value) <= MaximumWheelDelta;

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
