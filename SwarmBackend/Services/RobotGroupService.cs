using LanguageExt.Common;
using Microsoft.EntityFrameworkCore;
using SwarmBackend.Entities;
using SwarmBackend.Helpers;
using SwarmBackend.Interfaces;
using SwarmBackend.Models;

namespace SwarmBackend.Services;

public class RobotGroupService(DataContext context) : IRobotGroupService
{
    private const int MaximumNameLength = 80;

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
        var validation = await ValidateName(request.Name);
        if (validation != null)
        {
            return new Result<RobotGroupResponse>(validation);
        }

        var group = new RobotGroup
        {
            Name = request.Name.Trim(),
            Description = CleanDescription(request.Description),
            DateCreated = DateTime.UtcNow
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

        var validation = await ValidateName(request.Name, id);
        if (validation != null)
        {
            return new Result<RobotGroupResponse>(validation);
        }

        group.Name = request.Name.Trim();
        group.Description = CleanDescription(request.Description);

        await context.SaveChangesAsync();

        return await GetById(id);
    }

    public async Task<Result<bool>> Delete(int id)
    {
        var group = await context.RobotGroups
            .Include(item => item.Robots)
            .Include(item => item.TaskLogs)
            .FirstOrDefaultAsync(item => item.Id == id);
        if (group == null)
        {
            return new Result<bool>(new Exception("Grupo no encontrado"));
        }

        foreach (var robot in group.Robots)
        {
            robot.RobotGroupId = null;
        }

        group.TaskLogs.Clear();
        context.RobotGroups.Remove(group);
        await context.SaveChangesAsync();

        return true;
    }

    public async Task<IEnumerable<RobotResponse>> GetAvailableRobots()
    {
        return await context.Robots
            .AsNoTracking()
            .Include(robot => robot.Sensors)
            .Where(robot => robot.Status != RobotStatus.Disabled)
            .OrderBy(robot => robot.Name)
            .Select(robot => RobotResponse.From(robot))
            .ToListAsync();
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

        if (robot.RobotGroupId == groupId)
        {
            return await GetById(groupId);
        }

        if (robot.RobotGroupId != null)
        {
            if (!request.ForceTransfer)
            {
                return new Result<RobotGroupResponse>(
                    new Exception(
                        "El robot ya pertenece a un grupo. Use ForceTransfer=true para moverlo a este grupo."));
            }

            robot.RobotGroupId = groupId;
            await context.SaveChangesAsync();
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

    private async Task<Exception?> ValidateName(string? name, int? currentGroupId = null)
    {
        if (string.IsNullOrWhiteSpace(name))
        {
            return new Exception("El nombre del grupo es obligatorio");
        }

        var normalizedName = name.Trim();
        if (normalizedName.Length > MaximumNameLength)
        {
            return new Exception($"El nombre no puede superar {MaximumNameLength} caracteres");
        }

        var normalizedForComparison = normalizedName.ToUpper();
        var alreadyExists = await context.RobotGroups.AnyAsync(group =>
            group.Id != currentGroupId && group.Name.ToUpper() == normalizedForComparison);
        return alreadyExists
            ? new Exception("Ya existe un grupo con ese nombre")
            : null;
    }

    private static string? CleanDescription(string? description)
    {
        return string.IsNullOrWhiteSpace(description) ? null : description.Trim();
    }
}
