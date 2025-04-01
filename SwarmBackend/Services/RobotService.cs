using LanguageExt.Common;
using Microsoft.EntityFrameworkCore;
using SwarmBackend.Entities;
using SwarmBackend.Helpers;
using SwarmBackend.Interfaces;
using SwarmBackend.Models;

namespace SwarmBackend.Services;

public class RobotService : IRobotService
{
    private readonly DataContext context;

    public RobotService(DataContext context)
    {
        this.context = context;
    }

    public async Task<Result<RobotResponse>> Cancel(int id)
    {
        var robot = await context.Robots.FirstOrDefaultAsync(x => x.Id == id);
        if (robot == null)
        {
            return new Result<RobotResponse>(new Exception("Robot no encontrado"));
        }

        if (robot.Status != RobotStatus.Working)
        {
            return new Result<RobotResponse>(RobotResponse.From(robot));
        }

        robot.Status = RobotStatus.Idle;
        context.Robots.Update(robot);
        context.SaveChanges();

        return RobotResponse.From(robot);
    }

    public async Task<RobotResponse> Create(RobotRequest request)
    {
        var robot = new Robot
        {
            Name = request.Name,
            Description = request.Description,
            Notes = request.Notes,
            DateCreated = DateTime.Now,
            Status = RobotStatus.Idle,
            IsPublic = request.IsPublic,
            AccountId = request.AccountId
        };

        context.Robots.Add(robot);
        await context.SaveChangesAsync();

        return RobotResponse.From(robot);
    }

    public async Task<IEnumerable<RobotResponse>> GetAll(int? accountId = null)
    {
        var query = context.Robots
            .Include(x => x.Sensors)
            .Where(x => x.Status != RobotStatus.Disabled);

        if (accountId.HasValue)
        {
            query = query.Where(x => x.IsPublic || x.AccountId == accountId);
        }

        return await query
            .Select(x => RobotResponse.From(x))
            .ToListAsync();
    }

    public async Task<Result<RobotResponse>> GetById(int id, int? accountId = null)
    {
        var robot = await context.Robots
            .Include(x => x.Sensors)
            .FirstOrDefaultAsync(x => x.Id == id);

        if (robot == null)
        {
            return new Result<RobotResponse>(new Exception("Robot no encontrado"));
        }

        if (accountId.HasValue && !robot.IsPublic && robot.AccountId != accountId)
        {
            return new Result<RobotResponse>(new Exception("No tienes acceso a este robot"));
        }

        return RobotResponse.From(robot);
    }

    public async Task<Result<RobotResponse>> Update(int id, RobotRequest request, int? accountId = null)
    {
        var robot = await context.Robots.FirstOrDefaultAsync(x => x.Id == id);
        if (robot == null)
        {
            return new Result<RobotResponse>(new Exception("Robot no encontrado"));
        }

        if (accountId.HasValue && !robot.IsPublic && robot.AccountId != accountId)
        {
            return new Result<RobotResponse>(new Exception("No tienes acceso a este robot"));
        }

        robot.Name = request.Name;
        robot.Description = request.Description;
        robot.Notes = request.Notes;
        robot.Status = request.Status;
        robot.IsPublic = request.IsPublic;
        robot.AccountId = request.AccountId;

        await context.SaveChangesAsync();

        return RobotResponse.From(robot);
    }
}
