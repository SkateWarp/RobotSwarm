using LanguageExt.Common;
using Microsoft.EntityFrameworkCore;
using SwarmBackend.Entities;
using SwarmBackend.Helpers;
using SwarmBackend.Interfaces;
using SwarmBackend.Models;

namespace SwarmBackend.Services;

public class SensorService : ISensorService
{
    private readonly DataContext context;

    public SensorService(DataContext context)
    {
        this.context = context;
    }

    public async Task<SensorResponse> Create(SensorRequest request)
    {
        var sensor = new Sensor
        {
            RobotId = request.RobotId,
            SensorType = request.SensorType,
            Notes = request.Notes,
            Description = request.Description,
            DateCreated = DateTime.Now,
        };

        context.Sensors.Add(sensor);
        await context.SaveChangesAsync();

        return SensorResponse.From(sensor);
    }

    public async Task<IEnumerable<SensorResponse>> GetAllByRobot(int robotId)
    {
        return await context.Sensors.Where(x => x.RobotId == robotId)
            .Select(x => SensorResponse.From(x, true))
            .ToListAsync();
    }

    public async Task<Result<SensorResponse>> Update(int id, SensorRequest request)
    {
        var sensor = await context.Sensors.FindAsync(id);
        if (sensor == null)
        {
            return new Result<SensorResponse>(new Exception("Sensor no encontrado"));
        }

        context.Entry(sensor).CurrentValues.SetValues(request);
        await context.SaveChangesAsync();

        return SensorResponse.From(sensor);
    }
}
