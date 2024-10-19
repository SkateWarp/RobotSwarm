
using System.ComponentModel.DataAnnotations.Schema;
using System.Text.Json;

namespace SwarmBackend.Entities;

public class TaskLog
{
    public int Id { get; set; }
    public DateTime DateCreated { get; set; } = DateTime.Now;
    public DateTime? DateFinished { get; set; }
    public DateTime? DateCancelled { get; set; }
    public int TaskTemplateId { get; set; }
    public TaskTemplate? TaskTemplate { get; set; }
    public int RobotId { get; set; }
    public Robot? Robot { get; set; }

    [Column(TypeName = "jsonb")]
    public JsonDocument Parameters { get; set; } = JsonDocument.Parse("{}");
}
