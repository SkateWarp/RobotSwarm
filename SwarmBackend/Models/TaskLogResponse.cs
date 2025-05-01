using System.Text.Json;
using SwarmBackend.Entities;

namespace SwarmBackend.Models;

public record TaskLogResponse(int Id, DateTime DateCreated, DateTime? DateFinished, int TaskTemplateId,
    TaskTemplateResponse? TaskTemplate, List<int> RobotIds, JsonDocument Parameters, DateTime? DateCancelled)
{
    public static TaskLogResponse From(TaskLog taskLog) => new(taskLog.Id, taskLog.DateCreated, taskLog.DateFinished,
        taskLog.TaskTemplateId, taskLog.TaskTemplate != null ? TaskTemplateResponse.From(taskLog.TaskTemplate) : null,
        [.. taskLog.Robots.Select(x => x.Id)], taskLog.Parameters, taskLog.DateCancelled);
}

public record TaskLogRequest(int TaskTemplateId, List<int> RobotIds, JsonElement Parameters);

public record RosTaskTemplateRequest(string TaskType, List<int> RobotIds, JsonElement Parameters);
