namespace SwarmBackend.Entities;

public class Robot
{
    public int Id { get; set; }
    public string Name { get; set; } = null!;
    public string? Description { get; set; }
    public string? Notes { get; set; }
    public DateTime DateCreated { get; set; } = DateTime.Now;

    public ICollection<Sensor>? Sensors { get; set; }

    public bool IsConnected { get; set; }
    public RobotStatus Status { get; set; } = RobotStatus.Idle;
    public bool IsPublic { get; set; } = true;

    public int? RobotGroupId { get; set; }
    public RobotGroup? RobotGroup { get; set; }

    public int? AccountId { get; set; }
    public Account? Account { get; set; }
}
