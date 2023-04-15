namespace SwarmBackend.Entities;

public class TaskTemplate
{

    public int Id { get; set; }

    public DateTime DateCreated { get; set; } = DateTime.Now;

    public TaskTypeEnum TaskType { get; set; }

    public string Name { get; set; } = null!;
}
