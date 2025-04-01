using LanguageExt.Common;
using Microsoft.EntityFrameworkCore;
using SwarmBackend.Entities;
using SwarmBackend.Helpers;
using SwarmBackend.Interfaces;
using SwarmBackend.Models;

namespace SwarmBackend.Services;

public class RobotGroupService(DataContext context) : IRobotGroupService
{
    public async Task<IEnumerable<RobotGroupResponse>> GetAll()
    {
        var groups = await context.RobotGroups
            .Include(x => x.Robots)
            .Include(x => x.TaskLogs)
            .ThenInclude(x => x.TaskTemplate)
            .ToListAsync();

        return groups.Select(RobotGroupResponse.From);
    }

    public async Task<Result<RobotGroupResponse>> GetById(int id)
    {
        var group = await context.RobotGroups
            .Include(x => x.Robots)
            .Include(x => x.TaskLogs)
            .ThenInclude(x => x.TaskTemplate)
            .FirstOrDefaultAsync(x => x.Id == id);

        return group == null
            ? new Result<RobotGroupResponse>(new Exception("Grupo no encontrado"))
            : RobotGroupResponse.From(group);
    }

    public async Task<Result<RobotGroupResponse>> Create(RobotGroupRequest request)
    {
        var group = new RobotGroup
        {
            Name = request.Name,
            Description = request.Description,
            DateCreated = DateTime.Now
        };

        context.RobotGroups.Add(group);
        await context.SaveChangesAsync();

        return await GetById(group.Id);
    }

    public async Task<Result<RobotGroupResponse>> Update(int id, RobotGroupUpdateRequest request)
    {
        var group = await context.RobotGroups.FindAsync(id);
        if (group == null)
        {
            return new Result<RobotGroupResponse>(new Exception("Grupo no encontrado"));
        }

        group.Name = request.Name;
        group.Description = request.Description;

        await context.SaveChangesAsync();

        return await GetById(id);
    }

    public async Task<Result<bool>> Delete(int id)
    {
        var group = await context.RobotGroups.FindAsync(id);
        if (group == null)
        {
            return new Result<bool>(new Exception("Grupo no encontrado"));
        }

        context.RobotGroups.Remove(group);
        await context.SaveChangesAsync();

        return true;
    }

    public async Task<Result<RobotGroupResponse>> AddRobot(int groupId, AddRobotToGroupRequest request)
    {
        var group = await context.RobotGroups
            .Include(x => x.Robots)
            .FirstOrDefaultAsync(x => x.Id == groupId);

        if (group == null)
        {
            return new Result<RobotGroupResponse>(new Exception("Grupo no encontrado"));
        }

        var robot = await context.Robots.FindAsync(request.RobotId);
        if (robot == null)
        {
            return new Result<RobotGroupResponse>(new Exception("Robot no encontrado"));
        }

        if (robot.RobotGroupId != null)
        {
            if (!request.ForceTransfer)
            {
                return new Result<RobotGroupResponse>(
                    new Exception(
                        "El robot ya pertenece a un grupo. Use ForceTransfer=true para moverlo a este grupo."));
            }

            // If force transfer is true and robot is in a different group, move it
            if (robot.RobotGroupId != groupId)
            {
                robot.RobotGroupId = groupId;
                await context.SaveChangesAsync();
            }
        }
        else
        {
            robot.RobotGroupId = groupId;
            await context.SaveChangesAsync();
        }

        return await GetById(groupId);
    }

    public async Task<Result<RobotGroupResponse>> RemoveRobot(int groupId, int robotId)
    {
        var robot = await context.Robots.FindAsync(robotId);
        if (robot == null)
        {
            return new Result<RobotGroupResponse>(new Exception("Robot no encontrado"));
        }

        if (robot.RobotGroupId != groupId)
        {
            return new Result<RobotGroupResponse>(new Exception("El robot no pertenece a este grupo"));
        }

        robot.RobotGroupId = null;
        await context.SaveChangesAsync();

        return await GetById(groupId);
    }

    public async Task<Result<RobotGroupResponse>> AssignTask(int groupId, AssignTaskToGroupRequest request)
    {
        var group = await context.RobotGroups
            .Include(x => x.Robots)
            .FirstOrDefaultAsync(x => x.Id == groupId);

        if (group == null)
        {
            return new Result<RobotGroupResponse>(new Exception("Grupo no encontrado"));
        }

        if (!group.Robots.Any())
        {
            return new Result<RobotGroupResponse>(new Exception("El grupo no tiene robots asignados"));
        }

        var template = await context.TaskTemplates.FindAsync(request.TaskTemplateId);
        if (template == null)
        {
            return new Result<RobotGroupResponse>(new Exception("Plantilla de tarea no encontrada"));
        }

        foreach (var robot in group.Robots)
        {
            var task = new TaskLog
            {
                RobotId = robot.Id,
                RobotGroupId = groupId,
                TaskTemplateId = request.TaskTemplateId,
                DateCreated = DateTime.Now,
                Parameters = System.Text.Json.JsonDocument.Parse(request.Parameters.GetRawText())
            };

            context.TaskLogs.Add(task);

            robot.Status = RobotStatus.Working;
        }

        await context.SaveChangesAsync();

        return await GetById(groupId);
    }

    public async Task<Result<RobotGroupStatusResponse>> GetRobotGroupStatus(int robotId)
    {
        var robot = await context.Robots
            .Include(x => x.RobotGroup)
            .FirstOrDefaultAsync(x => x.Id == robotId);

        if (robot == null)
        {
            return new Result<RobotGroupStatusResponse>(new Exception("Robot no encontrado"));
        }

        return new RobotGroupStatusResponse(
            IsInGroup: robot.RobotGroupId != null,
            GroupId: robot.RobotGroupId,
            GroupName: robot.RobotGroup?.Name
        );
    }
}