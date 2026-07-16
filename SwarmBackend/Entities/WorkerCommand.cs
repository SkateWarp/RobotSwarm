using System.ComponentModel.DataAnnotations.Schema;
using System.Text.Json;

namespace SwarmBackend.Entities;

public class WorkerCommand
{
    public Guid Id { get; set; } = Guid.NewGuid();
    public Guid SimulationSessionId { get; set; }
    public SimulationSession SimulationSession { get; set; } = null!;
    public Guid? TaskRunId { get; set; }
    public TaskRun? TaskRun { get; set; }
    public Guid? ComputeWorkerId { get; set; }
    public ComputeWorker? ComputeWorker { get; set; }
    public WorkerCommandType Type { get; set; }
    public WorkerCommandState State { get; set; } = WorkerCommandState.Pending;
    public string IdempotencyKey { get; set; } = null!;
    public Guid CorrelationId { get; set; } = Guid.NewGuid();
    public long Sequence { get; set; }

    [Column(TypeName = "jsonb")]
    public JsonDocument Payload { get; set; } = JsonDocument.Parse("{}");

    [Column(TypeName = "jsonb")]
    public JsonDocument? Result { get; set; }

    public int RetryCount { get; set; }
    public string? LastError { get; set; }
    public DateTime CreatedAt { get; set; } = DateTime.UtcNow;
    public DateTime UpdatedAt { get; set; } = DateTime.UtcNow;
    public DateTime? DispatchedAt { get; set; }
    public DateTime? AcknowledgedAt { get; set; }
    public DateTime? CompletedAt { get; set; }
}
