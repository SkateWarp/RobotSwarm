using SwarmBackend.Entities;
using SwarmBackend.Helpers;
using SwarmBackend.Interfaces;
using SwarmBackend.Models;

namespace SwarmBackend.Routes;

public static class TaskTemplateRoute
{
    public static RouteGroupBuilder MapTaskTemplate(this RouteGroupBuilder group)
    {
        group.RequireAuthorization(policy => policy.RequireRole(Role.Admin.ToString()));

        group.MapGet("", GetAll)
            .Produces<IEnumerable<TaskTemplateResponse>>()
            .Produces(StatusCodes.Status401Unauthorized)
            .Produces(StatusCodes.Status403Forbidden);
        group.MapPut("/{id:int}", Update)
            .Produces<TaskTemplateResponse>()
            .ProducesValidationProblem()
            .Produces(StatusCodes.Status401Unauthorized)
            .Produces(StatusCodes.Status403Forbidden)
            .Produces(StatusCodes.Status404NotFound);

        return group;
    }

    public static async Task<IResult> GetAll(ITaskTemplateService service)
    {
        return Results.Ok(await service.GetAll());
    }

    public static async Task<IResult> Update(int id, TaskTemplateRequest request, ITaskTemplateService service)
    {
        if (!TaskTemplateRequestValidator.TryValidate(request, out var errors))
        {
            return Results.ValidationProblem(errors);
        }

        var normalizedRequest = request with { Name = request.Name.Trim() };
        var response = await service.Update(id, normalizedRequest);
        return response.Match(
            Results.Ok,
            error => Results.NotFound(new { message = error.Message }));
    }
}
