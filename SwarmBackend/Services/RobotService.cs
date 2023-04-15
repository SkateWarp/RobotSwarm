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

    public async Task<RobotResponse> Create(RobotRequest request)
    {
        var robot = new Robot
        {
            Name = request.Name,
            Description = request.Description,
            Notes = request.Notes,
            DateCreated = DateTime.Now,
        };

        context.Robots.Add(robot);
        await context.SaveChangesAsync();

        return RobotResponse.From(robot);
    }

    public async Task<IEnumerable<RobotResponse>> GetAll()
    {
        return await context.Robots
            .Include(x => x.Sensors)
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
