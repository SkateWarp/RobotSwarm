using LanguageExt.Common;
using Microsoft.EntityFrameworkCore;
using SwarmBackend.Entities;
using SwarmBackend.Helpers;
using SwarmBackend.Interfaces;
using SwarmBackend.Models;

namespace SwarmBackend.Services;

public class SensorReadingService(DataContext context, ILogger<SensorReadingService> logger) : ISensorReadingService
{
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

    public async Task<SensorReadingResponse> Create(int robotId, RosSensorReadingRequest request)
    {
        if (!SensorTypeEnumUtils.TryParseFromString(request.SensorName, out var type))
        {
            logger.LogError("Sensor type {SensorName} not found for robot {RobotId}", request.SensorName, robotId);
            return new SensorReadingResponse(0, 0, DateTime.UtcNow, 0, "Sensor no encontrado");
        }

        var sensor = await context.Sensors.FirstOrDefaultAsync(x => x.RobotId == robotId && x.SensorType == type);
        if (sensor == null)
        {
            sensor = new Sensor
            {
                RobotId = robotId,
                SensorType = type,
                Notes = "",
                Description = "",
                DateCreated = DateTime.Now,
            };

            context.Sensors.Add(sensor);
            await context.SaveChangesAsync();
        }


        var sensorReading = new SensorReading
        {
            SensorId = sensor.Id,
            Value = request.Value,
            DateCreated = DateTime.UtcNow,
            Notes = request.Notes,
        };

        context.SensorReadings.Add(sensorReading);
        await context.SaveChangesAsync();

        return SensorReadingResponse.From(sensorReading);
    }

    public async Task<IEnumerable<SensorReadingResponse>> GetAllByRobot(int robotId, DateTime startDate,
        DateTime endDate)
    {
        return await context.SensorReadings
            .Include(x => x.Sensor)
            .Where(x => x.Sensor!.RobotId == robotId && x.DateCreated >= startDate && x.DateCreated <= endDate)
            .Select(x => SensorReadingResponse.From(x))
            .ToListAsync();
    }

    public async Task<IEnumerable<SensorReadingResponse>> GetAllBySensor(int sensorId, DateTime startDate,
        DateTime endDate)
    {
        return await context.SensorReadings
            .Where(x => x.SensorId == sensorId && x.DateCreated >= startDate && x.DateCreated <= endDate)
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

    public async Task<IEnumerable<SensorReadingResponse>> GetAllByRobot(int robotId, DateRangeRequest dateRange)
    {
        return await context.SensorReadings
            .Include(x => x.Sensor)
            .Where(x => x.Sensor!.RobotId == robotId && x.DateCreated >= dateRange.StartDate &&
                        x.DateCreated <= dateRange.EndDate)
            .Select(x => SensorReadingResponse.From(x))
            .ToListAsync();
    }

    public async Task<IEnumerable<SensorReadingResponse>> GetAllBySensor(int sensorId, DateRangeRequest dateRange)
    {
        return await context.SensorReadings
            .Where(x => x.SensorId == sensorId && x.DateCreated >= dateRange.StartDate &&
                        x.DateCreated <= dateRange.EndDate)
            .Select(x => SensorReadingResponse.From(x))
            .ToListAsync();
    }
    public IEnumerable<SensorReadingResponse> GetLastByRobotAndSensor(int sensorId, int robotId)
    {
        return context.SensorReadings
            .Include(x => x.Sensor)
            .Where(x => x.SensorId == sensorId && x.Sensor.RobotId == robotId)
            .OrderByDescending(x => x.DateCreated)
            .Take(1000)
            .AsEnumerable()
            .DistinctBy(x => new { x.Notes })
            .Select(SensorReadingResponse.From)
            .ToList();
    }

    public IEnumerable<SensorReadingResponse> GetLastByRobot(int robotId)
    {
        return context.SensorReadings
            .Include(x => x.Sensor)
            .Where(x => x.Sensor.RobotId == robotId)
            .OrderByDescending(x => x.DateCreated)
            .Take(1000)
            .AsEnumerable()
            .DistinctBy(x => new { x.Notes })
            .Select(SensorReadingResponse.From)
            .ToList();
    }
}