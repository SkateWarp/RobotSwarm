using SwarmBackend.Interfaces;
using SwarmBackend.Models;

namespace SwarmBackend.Routes;

public static class SensorRoute
{
    public static RouteGroupBuilder MapSensor(this RouteGroupBuilder group)
    {

        group.MapGet("/{robotId}", GetAllByRobot)
            .RequireAuthorization()
           .Produces<IEnumerable<SensorResponse>>();

        group.MapPost("", Create)
            .RequireAuthorization()
           .Produces<SensorResponse>();

        group.MapPut("/{id}", Update)
          .RequireAuthorization()
         .Produces<SensorResponse>();

        return group;
    }


    public static async Task<IResult> Create(SensorRequest request, ISensorService sensorService)
    {
        var response = await sensorService.Create(request);
        return Results.Ok(response);
    }

    public static async Task<IResult> GetAllByRobot(int robotId, ISensorService sensorService)
    {
        var response = await sensorService.GetAllByRobot(robotId);
        return Results.Ok(response);
    }


    public static async Task<IResult> Update(int id, SensorRequest request, ISensorService sensorService)
    {
        var response = await sensorService.Update(id, request);
        return response.Match(Results.Ok, Results.BadRequest);
    }


}
