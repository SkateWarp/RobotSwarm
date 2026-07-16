namespace SwarmBackend.Entities;

public class SessionRobot
{
    public Guid Id { get; set; } = Guid.NewGuid();
    public Guid SimulationSessionId { get; set; }
    public SimulationSession SimulationSession { get; set; } = null!;
    public int Ordinal { get; set; }
    public string RuntimeId { get; set; } = null!;
    public string Namespace { get; set; } = null!;
    public string? Role { get; set; }
    public SessionRobotState State { get; set; } = SessionRobotState.Provisioning;
    public DateTime CreatedAt { get; set; } = DateTime.UtcNow;
    public DateTime UpdatedAt { get; set; } = DateTime.UtcNow;
}
