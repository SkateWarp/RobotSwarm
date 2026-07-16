using System.ComponentModel.DataAnnotations.Schema;
using System.Text.Json;

namespace SwarmBackend.Entities;

public class TaskRun
{
    public Guid Id { get; set; } = Guid.NewGuid();
    public Guid SimulationSessionId { get; set; }
    public SimulationSession SimulationSession { get; set; } = null!;
    public SwarmTaskRunType Type { get; set; }
    public TaskRunState State { get; set; } = TaskRunState.Queued;
    public double Progress { get; set; }
    public long CommandRevision { get; set; }

    [Column(TypeName = "jsonb")]
    public JsonDocument Parameters { get; set; } = JsonDocument.Parse("{}");

    [Column(TypeName = "jsonb")]
    public JsonDocument? Result { get; set; }

    public string? Error { get; set; }
    public DateTime CreatedAt { get; set; } = DateTime.UtcNow;
    public DateTime UpdatedAt { get; set; } = DateTime.UtcNow;
    public DateTime? StartedAt { get; set; }
    public DateTime? CompletedAt { get; set; }

    public ICollection<WorkerCommand> Commands { get; set; } = new List<WorkerCommand>();
}
