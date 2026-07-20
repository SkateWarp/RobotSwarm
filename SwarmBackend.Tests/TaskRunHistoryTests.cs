using System.Security.Claims;
using System.Text.Json;
using Microsoft.AspNetCore.Http;
using Microsoft.Extensions.DependencyInjection;
using SwarmBackend.Entities;
using SwarmBackend.Routes;

namespace SwarmBackend.Tests;

public sealed class TaskRunHistoryTests
{
    [Fact]
    public async Task HistoryReturnsOnlyTheOwnersTasksInNewestFirstOrder()
    {
        await using var dataContext = TestDataContext.Create();
        var owner = Account(41, "history-owner@example.test");
        var other = Account(42, "history-other@example.test");
        var firstSession = Session(owner, 3);
        var secondSession = Session(owner, 7);
        var otherSession = Session(other, 10);
        var oldest = Task(firstSession, SwarmTaskRunType.Figure, DateTime.UtcNow.AddMinutes(-3));
        var middle = Task(firstSession, SwarmTaskRunType.FollowLeader, DateTime.UtcNow.AddMinutes(-2));
        var newest = Task(secondSession, SwarmTaskRunType.CollaborativeTransport, DateTime.UtcNow.AddMinutes(-1));
        var privateTask = Task(otherSession, SwarmTaskRunType.Figure, DateTime.UtcNow);
        dataContext.AddRange(
            owner,
            other,
            firstSession,
            secondSession,
            otherSession,
            oldest,
            middle,
            newest,
            privateTask);
        await dataContext.SaveChangesAsync();

        var result = await SessionControlRoute.GetTaskHistory(
            HttpContext(owner.Id),
            dataContext,
            CancellationToken.None,
            offset: 0,
            limit: 2);
        var (statusCode, body) = await Execute(result);
        using var document = JsonDocument.Parse(body);
        var root = document.RootElement;
        var items = root.GetProperty("items");

        Assert.Equal(StatusCodes.Status200OK, statusCode);
        Assert.Equal(3, root.GetProperty("total").GetInt32());
        Assert.Equal(2, items.GetArrayLength());
        Assert.Equal(newest.Id, items[0].GetProperty("id").GetGuid());
        Assert.Equal(middle.Id, items[1].GetProperty("id").GetGuid());
        Assert.DoesNotContain(privateTask.Id.ToString(), body, StringComparison.OrdinalIgnoreCase);
    }

    [Fact]
    public async Task HistoryRequiresAnOwnerAndRejectsUnboundedPages()
    {
        await using var dataContext = TestDataContext.Create();

        var unauthenticated = await SessionControlRoute.GetTaskHistory(
            new DefaultHttpContext(),
            dataContext,
            CancellationToken.None);
        var invalidPage = await SessionControlRoute.GetTaskHistory(
            HttpContext(7),
            dataContext,
            CancellationToken.None,
            offset: -1,
            limit: 101);

        Assert.Equal(StatusCodes.Status401Unauthorized, (await Execute(unauthenticated)).StatusCode);
        Assert.Equal(StatusCodes.Status400BadRequest, (await Execute(invalidPage)).StatusCode);
    }

    [Fact]
    public async Task HistoryFiltersByTheControlPlaneEnums()
    {
        await using var dataContext = TestDataContext.Create();
        var owner = Account(51, "history-filter@example.test");
        var session = Session(owner, 4);
        var transport = Task(session, SwarmTaskRunType.CollaborativeTransport, DateTime.UtcNow);
        var formation = Task(session, SwarmTaskRunType.Figure, DateTime.UtcNow.AddSeconds(-1));
        formation.OutcomeState = TaskOutcomeState.Failed;
        dataContext.AddRange(owner, session, transport, formation);
        await dataContext.SaveChangesAsync();

        var result = await SessionControlRoute.GetTaskHistory(
            HttpContext(owner.Id),
            dataContext,
            CancellationToken.None,
            type: "collaborativetransport",
            outcome: "succeeded");
        var (statusCode, body) = await Execute(result);
        using var document = JsonDocument.Parse(body);

        Assert.Equal(StatusCodes.Status200OK, statusCode);
        Assert.Equal(1, document.RootElement.GetProperty("total").GetInt32());
        Assert.Equal(
            transport.Id,
            document.RootElement.GetProperty("items")[0].GetProperty("id").GetGuid());
    }

    private static Account Account(int id, string email)
    {
        return new Account
        {
            Id = id,
            FirstName = "Task",
            LastName = "Owner",
            Email = email,
            Enabled = true
        };
    }

    private static SimulationSession Session(Account owner, int robotCount)
    {
        return new SimulationSession
        {
            AccountId = owner.Id,
            Account = owner,
            State = SimulationSessionState.Stopped,
            DesiredRobotCount = robotCount
        };
    }

    private static TaskRun Task(
        SimulationSession session,
        SwarmTaskRunType type,
        DateTime createdAt)
    {
        return new TaskRun
        {
            SimulationSessionId = session.Id,
            SimulationSession = session,
            Type = type,
            State = TaskRunState.Completed,
            OutcomeState = TaskOutcomeState.Succeeded,
            Progress = 1,
            CreatedAt = createdAt,
            UpdatedAt = createdAt,
            CompletedAt = createdAt
        };
    }

    private static DefaultHttpContext HttpContext(int accountId)
    {
        return new DefaultHttpContext
        {
            User = new ClaimsPrincipal(new ClaimsIdentity(
                new[] { new Claim("id", accountId.ToString()) },
                "test"))
        };
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
