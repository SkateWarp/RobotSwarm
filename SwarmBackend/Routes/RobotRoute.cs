using SwarmBackend.Interfaces;
using SwarmBackend.Models;

namespace SwarmBackend.Routes;

public static class RobotRoute
{
    public static RouteGroupBuilder MapRobot(this RouteGroupBuilder group)
    {

        group.MapGet("", GetAll)
            .RequireAuthorization()
           .Produces<IEnumerable<RobotResponse>>();

        group.MapPost("", Create)
            .RequireAuthorization()
           .Produces<RobotResponse>();

        group.MapPut("/{id}", Update)
          .RequireAuthorization()
         .Produces<RobotResponse>();

        group.MapPut("/{id}/cancel", Cancel)
         .RequireAuthorization()
        .Produces<RobotResponse>();

        return group;
    }


    public static async Task<IResult> Create(RobotRequest request, IRobotService robotService)
    {
        var response = await robotService.Create(request);
        return Results.Ok(response);
    }

    public static async Task<IResult> GetAll(IRobotService robotService)
    {
        var response = await robotService.GetAll();
        return Results.Ok(response);
    }

    public static async Task<IResult> Update(int id, RobotRequest request, IRobotService robotService)
    {
        var response = await robotService.Update(id, request);
        return response.Match(Results.Ok, Results.BadRequest);
    }

    public static async Task<IResult> Cancel(int id, IRobotService service)
    {
        var response = await service.Cancel(id);
        return response.Match(Results.Ok, Results.BadRequest);
    }
}
