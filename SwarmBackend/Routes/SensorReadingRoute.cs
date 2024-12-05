using SwarmBackend.Interfaces;
using SwarmBackend.Models;

namespace SwarmBackend.Routes;

public static class SensorReadingRoute
{
    public static RouteGroupBuilder MapSensorReading(this RouteGroupBuilder group)
    {

        group.MapGet("/{robotId:int}", GetAllByRobot)
            .RequireAuthorization()
           .Produces<IEnumerable<SensorReadingResponse>>();

        group.MapGet("sensor/{sensorId:int}", GetAllBySensor)
         .RequireAuthorization()
        .Produces<IEnumerable<SensorReadingResponse>>();

        group.MapGet("last/{robotId}", GetLastByRobot)
            .RequireAuthorization()
           .Produces<IEnumerable<SensorResponse>>();

        group.MapGet("last/{sensorId}/{robotId}", GetLastByRobotAndSensor)
            .RequireAuthorization()
           .Produces<IEnumerable<SensorResponse>>();

        group.MapPost("", Create)
            .RequireAuthorization()
           .Produces<SensorReadingResponse>();
        
        group.MapPost("/{robotId:int}", CreateByRobot)
            .RequireAuthorization()
            .Produces<SensorReadingResponse>();

        group.MapPut("/{id:int}", Update)
          .RequireAuthorization()
         .Produces<SensorReadingResponse>();

        return group;
    }


    public static async Task<IResult> Create(SensorReadingRequest request, ISensorReadingService sensorService)
    {
        var response = await sensorService.Create(request);
        return Results.Ok(response);
    }

    public static async Task<IResult> GetAllByRobot(int robotId, ISensorReadingService sensorService, [AsParameters] DateRangeRequest dateRange)
    {
        var response = await sensorService.GetAllByRobot(robotId, dateRange);
        return Results.Ok(response);
    }

    public static async Task<IResult> GetAllBySensor(int sensorId, ISensorReadingService sensorService, [AsParameters] DateRangeRequest dateRange)
    {
        var response = await sensorService.GetAllBySensor(sensorId, dateRange);
        return Results.Ok(response);
    }


    public static async Task<IResult> Update(int id, SensorReadingRequest request, ISensorReadingService sensorService)
    {
        var response = await sensorService.Update(id, request);
        return response.Match(Results.Ok, Results.BadRequest);
    }

    private static async Task<IResult> CreateByRobot(int robotId, RosSensorReadingRequest request, ISensorReadingService sensorService)
    {
        var response = await sensorService.Create(robotId, request);
        return Results.Ok(response);
    }

    public static async Task<IResult> GetLastByRobot(int robotId, ISensorReadingService sensorService)
    {
        var response = await sensorService.GetLastByRobot(robotId);
        return Results.Ok(response);
    }

    public static async Task<IResult> GetLastByRobotAndSensor(int robotId, int sensorId, ISensorReadingService sensorService)
    {
        var response = await sensorService.GetLastByRobotAndSensor(robotId, sensorId);
        return Results.Ok(response);
    }
}
