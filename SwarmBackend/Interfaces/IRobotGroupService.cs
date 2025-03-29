using LanguageExt.Common;
using SwarmBackend.Models;

namespace SwarmBackend.Interfaces;

public interface IRobotGroupService
{
  Task<IEnumerable<RobotGroupResponse>> GetAll();
  Task<Result<RobotGroupResponse>> GetById(int id);
  Task<Result<RobotGroupResponse>> Create(RobotGroupRequest request);
  Task<Result<RobotGroupResponse>> Update(int id, RobotGroupUpdateRequest request);
  Task<Result<bool>> Delete(int id);
  Task<Result<RobotGroupResponse>> AddRobot(int groupId, AddRobotToGroupRequest request);
  Task<Result<RobotGroupResponse>> RemoveRobot(int groupId, int robotId);
  Task<Result<RobotGroupResponse>> AssignTask(int groupId, AssignTaskToGroupRequest request);
  Task<Result<RobotGroupStatusResponse>> GetRobotGroupStatus(int robotId);
}