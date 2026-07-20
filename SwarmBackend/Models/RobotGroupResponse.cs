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
      group.Robots.Select(RobotResponse.From).ToArray(),
      group.TaskLogs.Select(TaskLogResponse.From).ToArray()
  );
}

public record RobotGroupRequest(string Name, string? Description);
public record RobotGroupUpdateRequest(string Name, string? Description);
public record AddRobotToGroupRequest(int RobotId, bool ForceTransfer = false);
public record RobotGroupStatusResponse(bool IsInGroup, int? GroupId, string? GroupName);
