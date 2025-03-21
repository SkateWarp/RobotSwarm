namespace SwarmBackend.Entities;

public class RobotGroup
{
  public int Id { get; set; }
  public string Name { get; set; } = null!;
  public string? Description { get; set; }
  public DateTime DateCreated { get; set; } = DateTime.Now;

  public ICollection<Robot> Robots { get; set; } = [];
  public ICollection<TaskLog> TaskLogs { get; set; } = [];
}