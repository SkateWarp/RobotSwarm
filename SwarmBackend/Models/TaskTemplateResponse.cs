using SwarmBackend.Entities;
using SwarmBackend.Helpers;

namespace SwarmBackend.Models;

public record TaskTemplateResponse(int Id, DateTime DateCreated, TaskTypeEnum TaskType, string TaskTypeDescription, string Name)
{
    public static TaskTemplateResponse From(TaskTemplate task) => new(task.Id, task.DateCreated, task.TaskType,
        task.TaskType.GetDescriptionAttribute(), task.Name);
}

public record TaskTemplateRequest(string Name, TaskTypeEnum TaskType);