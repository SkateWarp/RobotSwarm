using Microsoft.AspNetCore.Mvc;
using SwarmBackend.Interfaces;
using SwarmBackend.Models;

namespace SwarmBackend.Routes;

public static class RobotGroupRoute
{
  public static RouteGroupBuilder MapRobotGroup(this RouteGroupBuilder group)
  {
    group.RequireAuthorization();

    group.MapGet("", GetAll);
    group.MapGet("/{id}", GetById);
    group.MapGet("/robot/{robotId}/status", GetRobotGroupStatus);
    group.MapPost("", Create);
    group.MapPut("/{id}", Update);
    group.MapDelete("/{id}", Delete);
    group.MapPost("/{groupId}/robots", AddRobot);
    group.MapDelete("/{groupId}/robots/{robotId}", RemoveRobot);
    group.MapPost("/{groupId}/tasks", AssignTask);

    return group;
  }

  private static async Task<IResult> GetAll(IRobotGroupService service)
  {
    return Results.Ok(await service.GetAll());
  }

  private static async Task<IResult> GetById(int id, IRobotGroupService service)
  {
    var response = await service.GetById(id);
    return response.Match(Results.Ok, Results.BadRequest);
  }

  private static async Task<IResult> GetRobotGroupStatus(int robotId, IRobotGroupService service)
  {
    var response = await service.GetRobotGroupStatus(robotId);
    return response.Match(Results.Ok, Results.BadRequest);
  }

  private static async Task<IResult> Create(RobotGroupRequest request, IRobotGroupService service)
  {
    var response = await service.Create(request);
    return response.Match(Results.Ok, Results.BadRequest);
  }

  private static async Task<IResult> Update(int id, RobotGroupUpdateRequest request, IRobotGroupService service)
  {
    var response = await service.Update(id, request);
    return response.Match(Results.Ok, Results.BadRequest);
  }

  private static async Task<IResult> Delete(int id, IRobotGroupService service)
  {
    var response = await service.Delete(id);
    return response.Match(Results.Ok, Results.BadRequest);
  }

  private static async Task<IResult> AddRobot(int groupId, AddRobotToGroupRequest request, IRobotGroupService service)
  {
    var response = await service.AddRobot(groupId, request);
    return response.Match(Results.Ok, Results.BadRequest);
  }

  private static async Task<IResult> RemoveRobot(int groupId, int robotId, IRobotGroupService service)
  {
    var response = await service.RemoveRobot(groupId, robotId);
    return response.Match(Results.Ok, Results.BadRequest);
  }

  private static async Task<IResult> AssignTask(int groupId, AssignTaskToGroupRequest request, IRobotGroupService service)
  {
    var response = await service.AssignTask(groupId, request);
    return response.Match(Results.Ok, Results.BadRequest);
  }
}