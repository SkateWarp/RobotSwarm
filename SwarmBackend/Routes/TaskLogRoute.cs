using System.Text.Json;
using Microsoft.AspNetCore.Mvc;
using SwarmBackend.Interfaces;
using SwarmBackend.Models;

namespace SwarmBackend.Routes;

public static class TaskLogRoute
{
    public static RouteGroupBuilder MapTaskLog(this RouteGroupBuilder group)
    {

        group.MapGet("", GetAll);
        group.MapPost("", Create);
        group.MapPost("/{robotId:int}", CreateByRobot);
        group.MapPut("/{id}", Update);

        return group;
    }

    public static async Task<IResult> Create(TaskLogRequest request, ITaskLogService service)
    {
        try
        {
            // Check if Parameters is an object
            if (request.Parameters.ValueKind != JsonValueKind.Object)
            {
                return Results.BadRequest(new { message = "Parameters must be a valid JSON object." });
            }

            // If valid, proceed to create the TaskLog
            var response = await service.Create(request);
            return response.Match(Results.Ok, Results.BadRequest);
        }
        catch (JsonException ex)
        {
            // Return a bad request response if JSON is invalid
            return Results.BadRequest(
                new { message = "Parameters must be a valid JSON object.", details = ex.Message }
            );
        }
    }

    public static async Task<IResult> GetAll(ITaskLogService service, [AsParameters] DateRangeRequest dateRange)
    {
        return Results.Ok(await service.GetAll(dateRange));
    }

    public static async Task<IResult> CreateByRobot(int robotId, RosTaskTemplateRequest request, ITaskLogService service)
    {
        var response = await service.Create(robotId, request);
        return response.Match(Results.Ok, Results.BadRequest);
    }
    public static async Task<IResult> Update(int id, TaskLogRequest request, ITaskLogService service)
    {
        var response = await service.Update(id, request);
        return response.Match(Results.Ok, Results.BadRequest);
    }


}
