namespace SwarmBackend.Entities;

public class ViewerLease
{
    public Guid Id { get; set; } = Guid.NewGuid();
    public Guid SimulationSessionId { get; set; }
    public SimulationSession SimulationSession { get; set; } = null!;
    public int AccountId { get; set; }
    public Account Account { get; set; } = null!;
    public ViewerSourceType Source { get; set; } = ViewerSourceType.Scene;
    public string? RobotRuntimeId { get; set; }
    public string? IdempotencyKey { get; set; }
    public string TokenHash { get; set; } = null!;
    public DateTime CreatedAt { get; set; } = DateTime.UtcNow;
    public DateTime ExpiresAt { get; set; }
    public DateTime? RevokedAt { get; set; }
}
