namespace SwarmBackend.Entities;

public class Robot
{
    public int Id { get; set; }
    public string Name { get; set; } = null!;
    public string? Description { get; set; }
    public string? Notes { get; set; }
    public DateTime DateCreated { get; set; } = DateTime.Now;

    public ICollection<Sensor>? Sensors { get; set; }
}
