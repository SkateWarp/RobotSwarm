using System.Security.Claims;
using System.Text.Json;
using Microsoft.AspNetCore.Http;
using Microsoft.Extensions.DependencyInjection;
using SwarmBackend.Entities;
using SwarmBackend.Models;
using SwarmBackend.Routes;

namespace SwarmBackend.Tests;

public sealed class ViewerLeaseStatusTests
{
    [Fact]
    public async Task StatusReportsViewerCloseFromQueuedThroughCompleted()
    {
        await using var dataContext = TestDataContext.Create();
        var owner = Account(10, "viewer-close-status@example.test");
        var session = Session(owner);
        const string sourceKey = "viewer-close-source";
        var lease = Lease(session, owner, sourceKey);
        lease.RevokedAt = DateTime.UtcNow;
        var sourceCommand = Command(session, sourceKey, WorkerCommandState.Completed);
        var closeCommand = new WorkerCommand
        {
            SimulationSessionId = session.Id,
            SimulationSession = session,
            Type = WorkerCommandType.StopViewer,
            State = WorkerCommandState.Pending,
            IdempotencyKey = $"sys:viewer-stop:{lease.Id:N}",
            Sequence = 2,
            Payload = JsonDocument.Parse($"{{\"leaseId\":\"{lease.Id:D}\"}}"),
            UpdatedAt = DateTime.UtcNow
        };
        dataContext.AddRange(owner, session, lease, sourceCommand, closeCommand);
        await dataContext.SaveChangesAsync();

        var queued = await ReadStatus(dataContext, owner.Id, session.Id, lease.Id);
        Assert.False(queued.GetProperty("isReady").GetBoolean());
        Assert.Equal(
            "Completed",
            queued.GetProperty("command").GetProperty("state").GetString());
        Assert.Equal(
            "Pending",
            queued.GetProperty("closeCommand").GetProperty("state").GetString());

        closeCommand.State = WorkerCommandState.Completed;
        closeCommand.CompletedAt = DateTime.UtcNow;
        closeCommand.UpdatedAt = closeCommand.CompletedAt.Value;
        await dataContext.SaveChangesAsync();

        var completed = await ReadStatus(dataContext, owner.Id, session.Id, lease.Id);
        Assert.Equal(
            closeCommand.Id,
            completed.GetProperty("closeCommand").GetProperty("id").GetGuid());
        Assert.Equal(
            "Completed",
            completed.GetProperty("closeCommand").GetProperty("state").GetString());
    }

    [Fact]
    public async Task OwnerCanReadTheViewerCommandFailureWithoutReceivingCredentials()
    {
        await using var dataContext = TestDataContext.Create();
        var owner = Account(11, "viewer-owner@example.test");
        var session = Session(owner);
        const string idempotencyKey = "viewer-status-test";
        var lease = Lease(session, owner, idempotencyKey);
        var command = Command(session, idempotencyKey, WorkerCommandState.Failed);
        command.LastError = "Gazebo did not expose a display.";
        dataContext.AddRange(owner, session, lease, command);
        await dataContext.SaveChangesAsync();

        var result = await SessionControlRoute.GetViewerLeaseStatus(
            session.Id,
            lease.Id,
            HttpContext(owner.Id),
            dataContext,
            CancellationToken.None);
        var (statusCode, body) = await Execute(result);
        using var document = JsonDocument.Parse(body);
        var root = document.RootElement;

        Assert.Equal(StatusCodes.Status200OK, statusCode);
        Assert.Equal(lease.Id, root.GetProperty("leaseId").GetGuid());
        Assert.False(root.GetProperty("isReady").GetBoolean());
        Assert.Equal(command.Id, root.GetProperty("command").GetProperty("id").GetGuid());
        Assert.Equal("Failed", root.GetProperty("command").GetProperty("state").GetString());
        Assert.Equal(
            command.LastError,
            root.GetProperty("command").GetProperty("error").GetString());
        Assert.DoesNotContain("token", body, StringComparison.OrdinalIgnoreCase);
        Assert.DoesNotContain("hash", body, StringComparison.OrdinalIgnoreCase);
        Assert.DoesNotContain("idempotency", body, StringComparison.OrdinalIgnoreCase);
    }

    [Fact]
    public async Task LeaseStatusIsNotVisibleToAnotherAccountOrSession()
    {
        await using var dataContext = TestDataContext.Create();
        var owner = Account(21, "first-owner@example.test");
        var otherAccount = Account(22, "second-owner@example.test");
        var session = Session(owner);
        var lease = Lease(session, owner, "private-viewer-status");
        var mismatchedLease = Lease(session, otherAccount, "mismatched-viewer-owner");
        dataContext.AddRange(owner, otherAccount, session, lease, mismatchedLease);
        await dataContext.SaveChangesAsync();

        var otherAccountResult = await SessionControlRoute.GetViewerLeaseStatus(
            session.Id,
            lease.Id,
            HttpContext(otherAccount.Id),
            dataContext,
            CancellationToken.None);
        var otherSessionResult = await SessionControlRoute.GetViewerLeaseStatus(
            Guid.NewGuid(),
            lease.Id,
            HttpContext(owner.Id),
            dataContext,
            CancellationToken.None);
        var mismatchedOwnerResult = await SessionControlRoute.GetViewerLeaseStatus(
            session.Id,
            mismatchedLease.Id,
            HttpContext(otherAccount.Id),
            dataContext,
            CancellationToken.None);

        Assert.Equal(StatusCodes.Status404NotFound, (await Execute(otherAccountResult)).StatusCode);
        Assert.Equal(StatusCodes.Status404NotFound, (await Execute(otherSessionResult)).StatusCode);
        Assert.Equal(StatusCodes.Status404NotFound, (await Execute(mismatchedOwnerResult)).StatusCode);
    }

    [Fact]
    public void StatusContractDoesNotExposeReadOrPublishCredentials()
    {
        var propertyNames = typeof(ViewerLeaseStatusResponse)
            .GetProperties()
            .Select(property => property.Name)
            .ToArray();

        Assert.DoesNotContain(
            propertyNames,
            name => name.Contains("Token", StringComparison.OrdinalIgnoreCase)
                || name.Contains("Hash", StringComparison.OrdinalIgnoreCase)
                || name.Contains("Idempotency", StringComparison.OrdinalIgnoreCase));
    }

    private static Account Account(int id, string email)
    {
        return new Account
        {
            Id = id,
            FirstName = "Viewer",
            LastName = "Owner",
            Email = email,
            Enabled = true
        };
    }

    private static SimulationSession Session(Account owner)
    {
        return new SimulationSession
        {
            AccountId = owner.Id,
            Account = owner,
            State = SimulationSessionState.Active,
            DesiredRobotCount = 3
        };
    }

    private static ViewerLease Lease(
        SimulationSession session,
        Account owner,
        string idempotencyKey)
    {
        return new ViewerLease
        {
            SimulationSessionId = session.Id,
            SimulationSession = session,
            AccountId = owner.Id,
            Account = owner,
            IdempotencyKey = idempotencyKey,
            TokenHash = new string('A', 64),
            PublishTokenHash = new string('B', 64),
            ExpiresAt = DateTime.UtcNow.AddMinutes(5)
        };
    }

    private static WorkerCommand Command(
        SimulationSession session,
        string idempotencyKey,
        WorkerCommandState state)
    {
        return new WorkerCommand
        {
            SimulationSessionId = session.Id,
            SimulationSession = session,
            Type = WorkerCommandType.SetViewerSource,
            State = state,
            IdempotencyKey = idempotencyKey,
            Sequence = 1,
            UpdatedAt = DateTime.UtcNow
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

    private static async Task<JsonElement> ReadStatus(
        TestDataContext dataContext,
        int accountId,
        Guid sessionId,
        Guid leaseId)
    {
        var result = await SessionControlRoute.GetViewerLeaseStatus(
            sessionId,
            leaseId,
            HttpContext(accountId),
            dataContext,
            CancellationToken.None);
        var (statusCode, body) = await Execute(result);
        Assert.Equal(StatusCodes.Status200OK, statusCode);
        using var document = JsonDocument.Parse(body);
        return document.RootElement.Clone();
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
