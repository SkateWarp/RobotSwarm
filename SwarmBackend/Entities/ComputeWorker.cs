using System.Text.Json;

namespace SwarmBackend.Entities;

public class ComputeWorker
{
    public Guid Id { get; set; } = Guid.NewGuid();
    public string Name { get; set; } = null!;
    public ComputeWorkerState State { get; set; } = ComputeWorkerState.Offline;
    public int MaxConcurrentSessions { get; set; } = 1;
    public string? ImageVersion { get; set; }
    public JsonDocument Capabilities { get; set; } = JsonDocument.Parse("{}");
    public string? CredentialHash { get; set; }
    public DateTime? CredentialCreatedAt { get; set; }
    public DateTime? CredentialRevokedAt { get; set; }
    public DateTime CreatedAt { get; set; } = DateTime.UtcNow;
    public DateTime UpdatedAt { get; set; } = DateTime.UtcNow;
    public DateTime? LastHeartbeatAt { get; set; }
    public int? ReportedActiveSessionCount { get; set; }
    public DateTime? ActiveSessionsReportedAt { get; set; }
    public Guid? DrainLeaseId { get; set; }
    public string? DrainTargetRevision { get; set; }
    public DateTime? DrainRequestedAt { get; set; }
    public DateTime? DrainLeaseExpiresAt { get; set; }

    public ICollection<SimulationSession> Sessions { get; set; } = new List<SimulationSession>();
    public ICollection<WorkerCommand> Commands { get; set; } = new List<WorkerCommand>();
}
