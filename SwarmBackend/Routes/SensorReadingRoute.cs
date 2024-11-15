using SwarmBackend.Interfaces;
using SwarmBackend.Models;

namespace SwarmBackend.Routes;

public static class SensorReadingRoute
{
    public static RouteGroupBuilder MapSensorReading(this RouteGroupBuilder group)
    {

        group.MapGet("/{robotId}", GetAllByRobot)
            .RequireAuthorization()
           .Produces<IEnumerable<SensorReadingResponse>>();

        group.MapGet("sensor/{sensorId}", GetAllBySensor)
         .RequireAuthorization()
        .Produces<IEnumerable<SensorReadingResponse>>();

        group.MapPost("", Create)
            .RequireAuthorization()
           .Produces<SensorReadingResponse>();

        group.MapPut("/{id}", Update)
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
}
