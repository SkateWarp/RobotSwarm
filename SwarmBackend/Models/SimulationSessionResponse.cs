namespace SwarmBackend.Models;

public record CreateSimulationSessionRequest(int RobotCount);

public record SimulationSessionLimitsResponse(
    int MaxRobotsPerSession);

public record SimulationSessionResponse(
    Guid Id,
    string State,
    int DesiredRobotCount,
    int? QueuePosition,
    string ArenaVersion,
    Guid? ComputeWorkerId,
    string? ComputeWorkerName,
    bool IsEmergencyStopped,
    long Revision,
    string? FailureReason,
    DateTime CreatedAt,
    DateTime UpdatedAt,
    DateTime? StartedAt,
    DateTime? StoppedAt);
