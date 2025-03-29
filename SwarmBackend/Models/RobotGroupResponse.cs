using SwarmBackend.Entities;

namespace SwarmBackend.Models;

public record RobotGroupResponse(
    int Id,
    string Name,
    string? Description,
    DateTime DateCreated,
    IEnumerable<RobotResponse> Robots,
    IEnumerable<TaskLogResponse> TaskLogs)
{
  public static RobotGroupResponse From(RobotGroup group) => new(
      group.Id,
      group.Name,
      group.Description,
      group.DateCreated,
      group.Robots.Select(RobotResponse.From),
      group.TaskLogs.Select(TaskLogResponse.From)
  );
}

public record RobotGroupRequest(string Name, string? Description);
public record RobotGroupUpdateRequest(string Name, string? Description);
public record AddRobotToGroupRequest(int RobotId, bool ForceTransfer = false);
public record AssignTaskToGroupRequest(int TaskTemplateId, JsonElement Parameters);
public record RobotGroupStatusResponse(bool IsInGroup, int? GroupId, string? GroupName);