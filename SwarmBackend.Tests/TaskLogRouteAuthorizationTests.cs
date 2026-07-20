using Microsoft.AspNetCore.Authorization;
using Microsoft.AspNetCore.Authorization.Infrastructure;
using Microsoft.AspNetCore.Builder;
using Microsoft.AspNetCore.Routing;
using Microsoft.Extensions.DependencyInjection;
using SwarmBackend.Entities;
using SwarmBackend.Interfaces;
using SwarmBackend.Routes;

namespace SwarmBackend.Tests;

public sealed class TaskLogRouteAuthorizationTests
{
    [Fact]
    public async Task EveryLegacyTaskLogEndpointRequiresAdministratorRole()
    {
        var builder = WebApplication.CreateBuilder();
        builder.Services.AddScoped<ITaskLogService>(_ => null!);
        await using var app = builder.Build();
        app.MapGroup("/TaskLog").MapTaskLog();

        var endpoints = ((IEndpointRouteBuilder)app).DataSources
            .SelectMany(source => source.Endpoints)
            .OfType<RouteEndpoint>()
            .ToArray();

        Assert.Equal(6, endpoints.Length);
        foreach (var endpoint in endpoints)
        {
            var policy = Assert.IsType<AuthorizationPolicy>(
                endpoint.Metadata.GetMetadata<AuthorizationPolicy>());
            var roleRequirement = Assert.Single(
                policy.Requirements.OfType<RolesAuthorizationRequirement>());
            Assert.Equal(new[] { Role.Admin.ToString() }, roleRequirement.AllowedRoles.ToArray());
        }
    }
}
