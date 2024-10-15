using System.Text.Json;
using SwarmBackend.Entities;

namespace SwarmBackend.Models;

public record TaskLogResponse(int Id, DateTime DateCreated, DateTime? DateFinished, int TaskTemplateId,
    TaskTemplateResponse? TaskTemplate, int RobotId, RobotResponse? Robot, JsonDocument Parameters)
{
    public static TaskLogResponse From(TaskLog taskLog) => new(taskLog.Id, taskLog.DateCreated, taskLog.DateFinished,
        taskLog.TaskTemplateId, taskLog.TaskTemplate != null ? TaskTemplateResponse.From(taskLog.TaskTemplate) : null,
        taskLog.RobotId, taskLog.Robot != null ? RobotResponse.From(taskLog.Robot) : null, taskLog.Parameters);
}

public record TaskLogRequest(int TaskTemplateId, int RobotId, JsonElement Parameters);
