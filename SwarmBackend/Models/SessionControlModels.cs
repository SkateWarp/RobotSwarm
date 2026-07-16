using System.Text.Json;
using System.Text.Json.Serialization;

namespace SwarmBackend.Models;

public record UpdateFleetRequest(int RobotCount);

public record SessionRobotResponse(
    Guid Id,
    int Ordinal,
    string RuntimeId,
    string Namespace,
    string? Role,
    string State,
    DateTime CreatedAt,
    DateTime UpdatedAt);

public record CreateTaskRunRequest(
    string Type,
    JsonElement Parameters);

public record TaskRunResponse(
    Guid Id,
    Guid SessionId,
    string Type,
    string State,
    double Progress,
    JsonElement Parameters,
    JsonElement? Result,
    string? Error,
    DateTime CreatedAt,
    DateTime UpdatedAt,
    DateTime? StartedAt,
    DateTime? CompletedAt);

public record WorkerCommandResponse(
    Guid Id,
    Guid SessionId,
    Guid? TaskRunId,
    string Type,
    string State,
    string IdempotencyKey,
    Guid CorrelationId,
    long Sequence,
    DateTime CreatedAt);

public record FleetUpdateResponse(
    SimulationSessionResponse Session,
    WorkerCommandResponse Command);

public record TaskCommandResponse(
    TaskRunResponse Task,
    WorkerCommandResponse Command);

public record EmergencyStopResponse(
    Guid SessionId,
    bool IsEmergencyStopped,
    WorkerCommandResponse Command);

public record CreateViewerLeaseRequest(
    string Source,
    string? RobotRuntimeId);

public record ViewerLeaseResponse(
    Guid LeaseId,
    Guid SessionId,
    string Source,
    string? RobotRuntimeId,
    string Token,
    DateTime ExpiresAt,
    bool IsReady,
    string? SignalingUrl,
    WorkerCommandResponse? Command);

public class ViewerAuthRequest
{
    public string? Token { get; set; }
    public string? Action { get; set; }
    public string? Path { get; set; }
    public string? Protocol { get; set; }

    [JsonExtensionData]
    public Dictionary<string, JsonElement>? Extra { get; set; }
}
