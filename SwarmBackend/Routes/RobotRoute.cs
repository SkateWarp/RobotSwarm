using System.Security.Claims;
using Microsoft.AspNetCore.Authorization;
using Microsoft.AspNetCore.SignalR;
using SwarmBackend.Entities;
using SwarmBackend.Interfaces;
using SwarmBackend.Models;
using SwarmBackend.Services;

namespace SwarmBackend.Routes;

public static class RobotRoute
{
    public static RouteGroupBuilder MapRobot(this RouteGroupBuilder group)
    {
        group.MapGet("", GetAll)
           .Produces<IEnumerable<RobotResponse>>();

        group.MapGet("/{id}", GetById)
        .RequireAuthorization()
        .Produces<RobotResponse>();

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

    private static int? GetAccountId(HttpContext context)
    {
        var accountIdClaim = context.User.FindFirst("id");
        return accountIdClaim != null ? int.Parse(accountIdClaim.Value) : null;
    }

    private static Role? GetRole(HttpContext context)
    {
        var roleClaim = context.User.FindFirst(ClaimTypes.Role);
        return roleClaim != null && Enum.TryParse<Role>(roleClaim.Value, out var role) ? role : null;
    }

    public static async Task<IResult> Create(RobotRequest request, IRobotService robotService, IHubContext<RobotHub> hubContext, HttpContext context)
    {
        var accountId = GetAccountId(context);
        if (!accountId.HasValue)
        {
            return Results.Unauthorized();
        }

        var response = await robotService.Create(request, accountId.Value);
        return await response.Match(
            async robot =>
            {
                await hubContext.Clients.All.SendAsync("RobotCreated", robot);
                return (IResult)Results.Ok(robot);
            },
            error => Task.FromResult(BadRequest(error)));
    }

    public static async Task<IResult> GetAll(IRobotService robotService, HttpContext context, bool? isPublic = null)
    {
        var accountId = GetAccountId(context);
        var response = await robotService.GetAll(accountId, isPublic, GetRole(context));
        return Results.Ok(response);
    }

    public static async Task<IResult> GetById(int id, IRobotService robotService, HttpContext context)
    {
        var accountId = GetAccountId(context);
        var response = await robotService.GetById(id, accountId, GetRole(context));
        return response.Match(Results.Ok, BadRequest);
    }

    public static async Task<IResult> Update(int id, RobotRequest request, IRobotService robotService, HttpContext context)
    {
        var accountId = GetAccountId(context);
        if (!accountId.HasValue)
        {
            return Results.Unauthorized();
        }

        var role = GetRole(context);
        if (!role.HasValue)
        {
            return Results.Unauthorized();
        }

        var response = await robotService.Update(id, request, accountId.Value, role.Value);
        return response.Match(Results.Ok, BadRequest);
    }

    public static async Task<IResult> Cancel(int id, IRobotService service, HttpContext context)
    {
        var accountId = GetAccountId(context);
        if (!accountId.HasValue)
        {
            return Results.Unauthorized();
        }

        var role = GetRole(context);
        if (!role.HasValue)
        {
            return Results.Unauthorized();
        }

        var response = await service.Cancel(id, accountId.Value, role.Value);
        return response.Match(Results.Ok, BadRequest);
    }

    private static IResult BadRequest(Exception error)
    {
        return Results.BadRequest(new { message = error.Message });
    }
}
