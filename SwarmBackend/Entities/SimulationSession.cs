namespace SwarmBackend.Entities;

public class SimulationSession
{
    public Guid Id { get; set; } = Guid.NewGuid();
    public int AccountId { get; set; }
    public Account Account { get; set; } = null!;
    public Guid? ComputeWorkerId { get; set; }
    public ComputeWorker? ComputeWorker { get; set; }
    public SimulationSessionState State { get; set; } = SimulationSessionState.Queued;
    public int DesiredRobotCount { get; set; }
    public string ArenaVersion { get; set; } = "arena-v1";
    public string? WorkerImageVersion { get; set; }
    public bool IsEmergencyStopped { get; set; }
    public long Revision { get; set; }
    public string? FailureReason { get; set; }
    public DateTime CreatedAt { get; set; } = DateTime.UtcNow;
    public DateTime UpdatedAt { get; set; } = DateTime.UtcNow;
    public DateTime? StartedAt { get; set; }
    public DateTime? StoppedAt { get; set; }

    public ICollection<SessionRobot> Robots { get; set; } = new List<SessionRobot>();
    public ICollection<TaskRun> TaskRuns { get; set; } = new List<TaskRun>();
    public ICollection<WorkerCommand> Commands { get; set; } = new List<WorkerCommand>();
    public ICollection<ViewerLease> ViewerLeases { get; set; } = new List<ViewerLease>();
}
