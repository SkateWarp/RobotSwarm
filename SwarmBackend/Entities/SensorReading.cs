namespace SwarmBackend.Entities;

public class SensorReading
{
    public int Id { get; set; }
    public string Value { get; set; } = string.Empty;
    public DateTime DateCreated { get; set; } = DateTime.Now;

    public int SensorId { get; set; }
    public virtual Sensor? Sensor { get; set; }
    public string? Notes { get; set; }
}
