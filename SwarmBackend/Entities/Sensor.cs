namespace SwarmBackend.Entities;

public class Sensor
{
    public int Id { get; set; }
    public DateTime DateCreated { get; set; } = DateTime.Now;
    public int RobotId { get; set; }
    public virtual Robot? Robot { get; set; }
    public SensorTypeEnum SensorType { get; set; } = SensorTypeEnum.None;
    public string? Description { get; set; }
    public string? Notes { get; set; }

}
