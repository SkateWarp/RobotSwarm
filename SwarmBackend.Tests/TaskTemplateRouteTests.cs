using System.Text.Json;
using Microsoft.AspNetCore.Builder;
using Microsoft.AspNetCore.Authorization;
using Microsoft.AspNetCore.Authorization.Infrastructure;
using Microsoft.AspNetCore.Http;
using Microsoft.AspNetCore.Routing;
using Microsoft.Extensions.DependencyInjection;
using SwarmBackend.Entities;
using SwarmBackend.Helpers;
using SwarmBackend.Interfaces;
using SwarmBackend.Models;
using SwarmBackend.Routes;
using SwarmBackend.Services;

namespace SwarmBackend.Tests;

public sealed class TaskTemplateRouteTests
{
    [Fact]
    public async Task RoutesOnlyExposeGetAndPutToAdministrators()
    {
        var builder = WebApplication.CreateBuilder();
        builder.Services.AddScoped<ITaskTemplateService, TaskTemplateService>();
        await using var app = builder.Build();
        app.MapGroup("/TaskTemplate").MapTaskTemplate();

        var endpoints = ((IEndpointRouteBuilder)app).DataSources
            .SelectMany(source => source.Endpoints)
            .OfType<RouteEndpoint>()
            .ToArray();

        Assert.Equal(2, endpoints.Length);
        Assert.Equal(
            new[] { "GET", "PUT" },
            endpoints
                .SelectMany(endpoint => endpoint.Metadata.GetMetadata<HttpMethodMetadata>()!.HttpMethods)
                .OrderBy(method => method)
                .ToArray());

        foreach (var endpoint in endpoints)
        {
            var policy = Assert.IsType<AuthorizationPolicy>(
                endpoint.Metadata.GetMetadata<AuthorizationPolicy>());
            var roles = Assert.Single(policy.Requirements.OfType<RolesAuthorizationRequirement>());
            Assert.Equal(new[] { Role.Admin.ToString() }, roles.AllowedRoles.ToArray());
        }
    }

    [Theory]
    [InlineData("", TaskTypeEnum.Transport)]
    [InlineData("   ", TaskTypeEnum.FollowLeader)]
    [InlineData("Valid name", TaskTypeEnum.None)]
    [InlineData("Valid name", (TaskTypeEnum)99)]
    public void ValidatorRejectsMissingNamesAndUnsupportedTypes(string name, TaskTypeEnum taskType)
    {
        var valid = TaskTemplateRequestValidator.TryValidate(
            new TaskTemplateRequest(name, taskType),
            out var errors);

        Assert.False(valid);
        Assert.NotEmpty(errors);
    }

    [Fact]
    public void ValidatorRejectsNamesLongerThanThePublishedLimit()
    {
        var valid = TaskTemplateRequestValidator.TryValidate(
            new TaskTemplateRequest(
                new string('A', TaskTemplateRequestValidator.MaximumNameLength + 1),
                TaskTypeEnum.Formation),
            out var errors);

        Assert.False(valid);
        Assert.Contains(nameof(TaskTemplateRequest.Name), errors.Keys);
    }

    [Fact]
    public async Task UpdateTrimsTheNameAndPersistsOnlyTheEditableFields()
    {
        await using var dataContext = TestDataContext.Create();
        var createdAt = DateTime.UtcNow.AddDays(-3);
        var template = new TaskTemplate
        {
            Id = 17,
            Name = "Old name",
            TaskType = TaskTypeEnum.Transport,
            DateCreated = createdAt,
            Enabled = true
        };
        dataContext.TaskTemplates.Add(template);
        await dataContext.SaveChangesAsync();
        var service = new TaskTemplateService(dataContext);

        var result = await TaskTemplateRoute.Update(
            template.Id,
            new TaskTemplateRequest("  Coordinated formation  ", TaskTypeEnum.Formation),
            service);
        var (statusCode, body) = await Execute(result);
        using var document = JsonDocument.Parse(body);

        Assert.Equal(StatusCodes.Status200OK, statusCode);
        Assert.Equal("Coordinated formation", template.Name);
        Assert.Equal(TaskTypeEnum.Formation, template.TaskType);
        Assert.Equal(createdAt, template.DateCreated);
        Assert.True(template.Enabled);
        Assert.Equal("Coordinated formation", document.RootElement.GetProperty("name").GetString());
    }

    [Fact]
    public async Task InvalidUpdateDoesNotChangeTheTemplate()
    {
        await using var dataContext = TestDataContext.Create();
        var template = new TaskTemplate
        {
            Id = 23,
            Name = "Keep me",
            TaskType = TaskTypeEnum.FollowLeader
        };
        dataContext.TaskTemplates.Add(template);
        await dataContext.SaveChangesAsync();
        var service = new TaskTemplateService(dataContext);

        var result = await TaskTemplateRoute.Update(
            template.Id,
            new TaskTemplateRequest(" ", TaskTypeEnum.None),
            service);

        Assert.Equal(StatusCodes.Status400BadRequest, (await Execute(result)).StatusCode);
        Assert.Equal("Keep me", template.Name);
        Assert.Equal(TaskTypeEnum.FollowLeader, template.TaskType);
    }

    [Fact]
    public async Task MissingTemplateReturnsNotFound()
    {
        await using var dataContext = TestDataContext.Create();
        var service = new TaskTemplateService(dataContext);

        var result = await TaskTemplateRoute.Update(
            404,
            new TaskTemplateRequest("Existing contract", TaskTypeEnum.Transport),
            service);

        Assert.Equal(StatusCodes.Status404NotFound, (await Execute(result)).StatusCode);
    }

    private static async Task<(int StatusCode, string Body)> Execute(IResult result)
    {
        var context = new DefaultHttpContext
        {
            RequestServices = new ServiceCollection()
                .AddLogging()
                .BuildServiceProvider()
        };
        context.Response.Body = new MemoryStream();

        await result.ExecuteAsync(context);
        context.Response.Body.Position = 0;
        using var reader = new StreamReader(context.Response.Body);
        return (context.Response.StatusCode, await reader.ReadToEndAsync());
    }
}
