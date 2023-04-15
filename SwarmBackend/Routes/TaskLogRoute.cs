using SwarmBackend.Interfaces;
using SwarmBackend.Models;

namespace SwarmBackend.Routes;

public static class TaskLogRoute
{
    public static RouteGroupBuilder MapTaskLog(this RouteGroupBuilder group)
    {

        group.MapGet("", GetAll);
        group.MapPost("", Create);
        group.MapPut("/{id}", Update);

        return group;
    }

    public static async Task<IResult> Create(TaskLogRequest request, ITaskLogService service)
    {
        return Results.Ok(await service.Create(request));
    }

    public static async Task<IResult> GetAll(ITaskLogService service)
    {
        return Results.Ok(await service.GetAll());
    }

    public static async Task<IResult> Update(int id, TaskLogRequest request, ITaskLogService service)
    {
        var response = await service.Update(id, request);
        return response.Match(Results.Ok, Results.BadRequest);
    }
}
