using LanguageExt.Common;
using Microsoft.EntityFrameworkCore;
using SwarmBackend.Entities;
using SwarmBackend.Helpers;
using SwarmBackend.Interfaces;
using SwarmBackend.Models;

namespace SwarmBackend.Services;

public class SensorService : ISensorService
{
    private readonly DataContext _context;

    public SensorService(DataContext context)
    {
        this._context = context;
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

        _context.Sensors.Add(sensor);
        await _context.SaveChangesAsync();

        return SensorResponse.From(sensor);
    }

    public async Task<IEnumerable<SensorResponse>> GetAllByRobot(int robotId)
    {
        return await _context.Sensors.Where(x => x.RobotId == robotId)
            .Select(x => SensorResponse.From(x, true))
            .ToListAsync();
    }

    public async Task<Result<SensorResponse>> Update(int id, SensorRequest request)
    {
        var sensor = await _context.Sensors.FindAsync(id);
        if (sensor == null)
        {
            return new Result<SensorResponse>(new Exception("Sensor no encontrado"));
        }

        _context.Entry(sensor).CurrentValues.SetValues(request);
        await _context.SaveChangesAsync();

        return SensorResponse.From(sensor);
    }
}
