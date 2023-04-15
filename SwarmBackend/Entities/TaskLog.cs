namespace SwarmBackend.Entities;

public class TaskLog
{
    public int Id { get; set; }
    public DateTime DateCreated { get; set; } = DateTime.Now;
    public DateTime? DateFinished { get; set; }
    public int TaskTemplateId { get; set; }
    public TaskTemplate? TaskTemplate { get; set; }
    public int RobotId { get; set; }
    public Robot? Robot { get; set; }

}
