using SwarmBackend.Entities;
using SwarmBackend.Models;

namespace SwarmBackend.Helpers;

public static class TaskTemplateRequestValidator
{
    public const int MaximumNameLength = 100;

    public static bool TryValidate(
        TaskTemplateRequest request,
        out Dictionary<string, string[]> errors)
    {
        errors = new Dictionary<string, string[]>();

        if (string.IsNullOrWhiteSpace(request.Name))
        {
            errors[nameof(request.Name)] = ["Name is required."];
        }
        else if (request.Name.Trim().Length > MaximumNameLength)
        {
            errors[nameof(request.Name)] =
                [$"Name must not exceed {MaximumNameLength} characters."];
        }

        if (!Enum.IsDefined(request.TaskType) || request.TaskType == TaskTypeEnum.None)
        {
            errors[nameof(request.TaskType)] =
                ["TaskType must be Transport, FollowLeader, or Formation."];
        }

        return errors.Count == 0;
    }
}
