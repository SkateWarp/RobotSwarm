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
        };

        context.Robots.Add(robot);
        await context.SaveChangesAsync();

        return RobotResponse.From(robot);
    }

    public async Task<IEnumerable<RobotResponse>> GetAll()
    {
        return await context.Robots
            .Include(x => x.Sensors)
            .Where(x => x.Status != RobotStatus.Disabled)
            .Select(x => RobotResponse.From(x))
            .ToListAsync();
    }

    public async Task<Result<RobotResponse>> Update(int id, RobotRequest request)
    {
        var robot = await context.Robots.FirstOrDefaultAsync(x => x.Id == id);
        if (robot == null)
        {
            return new Result<RobotResponse>(new Exception("Robot no encontrado"));
        }

        context.Entry(robot).CurrentValues.SetValues(request);
        await context.SaveChangesAsync();

        return RobotResponse.From(robot);
    }
}
