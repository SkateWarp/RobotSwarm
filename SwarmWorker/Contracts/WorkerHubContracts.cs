using System.Text.Json;

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
