using Microsoft.AspNetCore.Authorization;
using Microsoft.AspNetCore.Authorization.Infrastructure;
using Microsoft.AspNetCore.Builder;
using Microsoft.AspNetCore.Http;
using Microsoft.AspNetCore.Routing;
using Microsoft.Extensions.DependencyInjection;
using SwarmBackend.Entities;
using SwarmBackend.Interfaces;
using SwarmBackend.Models;
using SwarmBackend.Routes;
using SwarmBackend.Services;

namespace SwarmBackend.Tests;

public sealed class RobotGroupRouteTests
{
    [Fact]
    public async Task RoutesExposeRealMembershipOperationsOnlyToAdministrators()
    {
        var builder = WebApplication.CreateBuilder();
        builder.Services.AddScoped<IRobotGroupService>(_ => null!);
        await using var app = builder.Build();
        app.MapGroup("/RobotGroups").MapRobotGroup();

        var endpoints = ((IEndpointRouteBuilder)app).DataSources
            .SelectMany(source => source.Endpoints)
            .OfType<RouteEndpoint>()
            .ToArray();

        Assert.Equal(9, endpoints.Length);
        Assert.DoesNotContain(endpoints, endpoint =>
            endpoint.RoutePattern.RawText?.Contains("/tasks", StringComparison.Ordinal) == true);

        foreach (var endpoint in endpoints)
        {
            var policy = Assert.IsType<AuthorizationPolicy>(
                endpoint.Metadata.GetMetadata<AuthorizationPolicy>());
            var roles = Assert.Single(policy.Requirements.OfType<RolesAuthorizationRequirement>());
            Assert.Equal(new[] { Role.Admin.ToString() }, roles.AllowedRoles.ToArray());
        }
    }

    [Fact]
    public async Task ServiceTrimsNamesAndRejectsCaseInsensitiveDuplicates()
    {
        await using var dataContext = TestDataContext.Create();
        var service = new RobotGroupService(dataContext);

        var created = Success(await service.Create(
            new RobotGroupRequest("  Exploradores  ", "  Primera patrulla  ")));
        var duplicate = Failure(await service.Create(
            new RobotGroupRequest("exploradores", null)));

        Assert.Equal("Exploradores", created.Name);
        Assert.Equal("Primera patrulla", created.Description);
        Assert.Contains("Ya existe", duplicate.Message);
        Assert.Single(dataContext.RobotGroups);
    }

    [Fact]
    public async Task MovingRobotRequiresExplicitTransferAndAddingTwiceIsIdempotent()
    {
        await using var dataContext = TestDataContext.Create();
        var first = new RobotGroup { Name = "Primer grupo" };
        var second = new RobotGroup { Name = "Segundo grupo" };
        dataContext.AddRange(first, second);
        await dataContext.SaveChangesAsync();
        var robot = new Robot { Name = "tb3_0", RobotGroupId = first.Id };
        dataContext.Add(robot);
        await dataContext.SaveChangesAsync();
        var service = new RobotGroupService(dataContext);

        var sameGroup = Success(await service.AddRobot(
            first.Id,
            new AddRobotToGroupRequest(robot.Id)));
        var denied = Failure(await service.AddRobot(
            second.Id,
            new AddRobotToGroupRequest(robot.Id)));
        var transferred = Success(await service.AddRobot(
            second.Id,
            new AddRobotToGroupRequest(robot.Id, ForceTransfer: true)));

        Assert.Single(sameGroup.Robots);
        Assert.Contains("ForceTransfer=true", denied.Message);
        Assert.Contains(transferred.Robots, item => item.Id == robot.Id);
        Assert.Equal(second.Id, robot.RobotGroupId);
    }

    [Fact]
    public async Task DeletingGroupLeavesItsRobotsAvailable()
    {
        await using var dataContext = TestDataContext.Create();
        var group = new RobotGroup { Name = "Temporal" };
        dataContext.Add(group);
        await dataContext.SaveChangesAsync();
        var robot = new Robot { Name = "tb3_1", RobotGroupId = group.Id };
        dataContext.Add(robot);
        await dataContext.SaveChangesAsync();
        var service = new RobotGroupService(dataContext);

        var deleted = await service.Delete(group.Id);
        var deletedValue = deleted.Match(value => value, error => throw error);

        Assert.True(deletedValue);
        Assert.Null(robot.RobotGroupId);
        Assert.Empty(dataContext.RobotGroups);
        Assert.Single(await service.GetAvailableRobots());
    }

    [Fact]
    public async Task AdministratorInventoryIncludesActiveRobotsFromEveryOwner()
    {
        await using var dataContext = TestDataContext.Create();
        dataContext.Robots.AddRange(
            new Robot { Name = "publico", AccountId = 7, IsPublic = true },
            new Robot { Name = "privado", AccountId = 8, IsPublic = false },
            new Robot
            {
                Name = "deshabilitado",
                AccountId = 9,
                IsPublic = true,
                Status = RobotStatus.Disabled
            });
        await dataContext.SaveChangesAsync();
        var service = new RobotGroupService(dataContext);

        var robots = (await service.GetAvailableRobots())
            .Select(robot => robot.Name)
            .OrderBy(name => name)
            .ToArray();

        Assert.Equal(new[] { "privado", "publico" }, robots);
    }

    private static RobotGroupResponse Success(LanguageExt.Common.Result<RobotGroupResponse> result)
    {
        return result.Match(value => value, error => throw error);
    }

    private static Exception Failure(LanguageExt.Common.Result<RobotGroupResponse> result)
    {
        return result.Match<Exception>(
            _ => throw new InvalidOperationException("Se esperaba un error"),
            error => error);
    }
}
