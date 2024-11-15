using LanguageExt.Common;
using Microsoft.EntityFrameworkCore;
using SwarmBackend.Entities;
using SwarmBackend.Helpers;
using SwarmBackend.Interfaces;
using SwarmBackend.Models;

namespace SwarmBackend.Services;

public class SensorReadingService : ISensorReadingService
{
    private readonly DataContext context;

    public SensorReadingService(DataContext context)
    {
        this.context = context;
    }

    public async Task<SensorReadingResponse> Create(SensorReadingRequest request)
    {
        var reading = new SensorReading
        {
            Value = request.Value,
            SensorId = request.SensorId,
            Notes = request.Notes,
            DateCreated = DateTime.Now,
        };

        context.SensorReadings.Add(reading);
        await context.SaveChangesAsync();

        return SensorReadingResponse.From(reading);
    }

    public async Task<IEnumerable<SensorReadingResponse>> GetAllByRobot(int robotId, DateRangeRequest dateRange)
    {
        var query = context.SensorReadings
               .Include(x => x.Sensor)
               .Where(x => x.Sensor!.RobotId == robotId);

        if (dateRange.StartDate != null)
        {
            query = query.Where(x => x.DateCreated >= dateRange.StartDate);
        }

        if (dateRange.EndDate != null)
        {
            query = query.Where(x => x.DateCreated <= dateRange.EndDate);
        }
        return await query
               .Select(x => SensorReadingResponse.From(x))
               .ToListAsync();
    }

    public async Task<IEnumerable<SensorReadingResponse>> GetAllBySensor(int sensorId, DateRangeRequest dateRange)
    {
        var query = context.SensorReadings
                 .Where(x => x.SensorId == sensorId);

        if (dateRange.StartDate != null)
        {
            query = query.Where(x => x.DateCreated >= dateRange.StartDate);
        }

        if (dateRange.EndDate != null)
        {
            query = query.Where(x => x.DateCreated <= dateRange.EndDate);
        }

        return await query
        .Select(x => SensorReadingResponse.From(x))
                 .ToListAsync();
    }

    public async Task<Result<SensorReadingResponse>> Update(int id, SensorReadingRequest request)
    {
        var reading = await context.SensorReadings.FindAsync(id);
        if (reading == null)
        {
            return new Result<SensorReadingResponse>(new Exception("Lectura no encontrado"));
        }

        context.Entry(reading).CurrentValues.SetValues(request);
        await context.SaveChangesAsync();

        return SensorReadingResponse.From(reading);
    }
}
