using SwarmBackend.Interfaces;
using SwarmBackend.Models;

namespace SwarmBackend.Routes;

public static class TaskTemplateRoute
{
    public static RouteGroupBuilder MapTaskTemplate(this RouteGroupBuilder group)
    {
        group.MapGet("", GetAll);
        group.MapPut("/{id}", Update);

        return group;
    }

    public static async Task<IResult> GetAll(ITaskTemplateService service)
    {
        return Results.Ok(await service.GetAll());
    }

    public static async Task<IResult> Update(int id, TaskTemplateRequest request, ITaskTemplateService service)
    {
        var response = await service.Update(id, request);
        return response.Match(Results.Ok, Results.BadRequest);
    }
}
