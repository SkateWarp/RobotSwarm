using System.Text.Json;

namespace SwarmBackend.Models;

public record EnrollComputeWorkerRequest(
    string Name,
    int MaxConcurrentSessions,
    JsonElement? Capabilities);

public record ComputeWorkerResponse(
    Guid Id,
    string Name,
    string State,
    int MaxConcurrentSessions,
    string? ImageVersion,
    JsonElement Capabilities,
    DateTime CreatedAt,
    DateTime UpdatedAt,
    DateTime? LastHeartbeatAt,
    DateTime? CredentialCreatedAt,
    DateTime? CredentialRevokedAt);

public record ComputeWorkerEnrollmentResponse(
    ComputeWorkerResponse Worker,
    string Credential);

public record WorkerRegistrationRequest(
    string? ImageVersion,
    JsonElement? Capabilities);

public record WorkerHeartbeatRequest(
    string? ImageVersion,
    JsonElement? Capabilities,
    // Wire name retained for compatibility; updated workers report every managed container.
    IReadOnlyList<Guid>? ActiveSessionIds);

public record WorkerRegistrationResponse(
    Guid WorkerId,
    string Name,
    string State,
    int MaxConcurrentSessions,
    DateTime ServerTime);

public record WorkerCommandEnvelope(
    Guid Id,
    Guid SessionId,
    string Type,
    string IdempotencyKey,
    Guid CorrelationId,
    long Sequence,
    JsonElement Payload,
    DateTime CreatedAt);

public record WorkerCommandCompletionRequest(
    Guid CommandId,
    JsonElement? Result);

public record WorkerCommandFailureRequest(
    Guid CommandId,
    string Error);

public record WorkerEmergencyStopReport(
    Guid SessionId,
    bool Active,
    string? Reason);

public record SessionEventReport(
    Guid SessionId,
    string State,
    string? FailureReason,
    JsonElement? Payload);

public record TaskEventReport(
    Guid SessionId,
    Guid TaskRunId,
    string State,
    double? Progress,
    JsonElement? Result,
    string? Error);

public record TaskRunEventResponse(
    Guid Id,
    Guid SessionId,
    string Type,
    string State,
    double Progress,
    JsonElement? Result,
    string? Error,
    DateTime UpdatedAt,
    DateTime? StartedAt,
    DateTime? CompletedAt);
