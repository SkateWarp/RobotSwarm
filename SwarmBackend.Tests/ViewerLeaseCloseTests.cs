using System.Security.Claims;
using System.Text.Json;
using Microsoft.AspNetCore.Http;
using Microsoft.AspNetCore.SignalR;
using Microsoft.EntityFrameworkCore;
using Microsoft.Extensions.DependencyInjection;
using Microsoft.Extensions.Logging.Abstractions;
using Npgsql;
using SwarmBackend.Entities;
using SwarmBackend.Routes;
using SwarmBackend.Services;

namespace SwarmBackend.Tests;

public sealed class ViewerLeaseCloseTests
{
    [Fact]
    public async Task OwnerCloseRevokesBeforeQueuingOneIdempotentStopCommand()
    {
        await using var context = TestDataContext.Create();
        var seeded = await SeedLease(context);
        var workerHub = new RecordingHubContext<WorkerHub>();
        var sessionHub = new RecordingHubContext<SessionHub>();
        var registry = new ViewerControlRegistry(
            workerHub,
            NullLogger<ViewerControlRegistry>.Instance);
        var now = DateTimeOffset.UtcNow;
        var authorization = await registry.AuthorizeAsync(
            "viewer-connection",
            seeded.Owner.Id,
            seeded.Session.Id,
            seeded.Lease.Id,
            seeded.Worker.Id,
            now.AddMinutes(5),
            now,
            CancellationToken.None);
        Assert.Equal(ViewerControlAuthorizationStatus.Authorized, authorization.Status);

        var first = await SessionControlRoute.CloseViewerLease(
            seeded.Session.Id,
            seeded.Lease.Id,
            HttpContext(seeded.Owner.Id),
            context,
            new WorkerCommandService(context),
            workerHub,
            sessionHub,
            registry,
            CancellationToken.None);
        Assert.Equal(StatusCodes.Status202Accepted, (await Execute(first)).StatusCode);

        var persistedLease = await context.ViewerLeases.SingleAsync();
        var command = await context.WorkerCommands.SingleAsync();
        Assert.NotNull(persistedLease.RevokedAt);
        Assert.Equal(WorkerCommandType.StopViewer, command.Type);
        Assert.Equal(
            seeded.Lease.Id,
            command.Payload.RootElement.GetProperty("leaseId").GetGuid());
        Assert.Empty(registry.GetSnapshots());
        Assert.Contains(workerHub.Messages, message => message.Method == "ViewerInputRelease");
        Assert.Contains(workerHub.Messages, message => message.Method == "CommandAvailable");

        var repeated = await SessionControlRoute.CloseViewerLease(
            seeded.Session.Id,
            seeded.Lease.Id,
            HttpContext(seeded.Owner.Id),
            context,
            new WorkerCommandService(context),
            workerHub,
            sessionHub,
            registry,
            CancellationToken.None);
        Assert.Equal(StatusCodes.Status202Accepted, (await Execute(repeated)).StatusCode);
        Assert.Equal(1, await context.WorkerCommands.CountAsync());
    }

    [Fact]
    public async Task FailedStopCanBeRetriedWithoutRevokingAgainOrStoppingTheSession()
    {
        await using var context = TestDataContext.Create();
        var seeded = await SeedLease(context);
        var workerHub = new RecordingHubContext<WorkerHub>();
        var registry = new ViewerControlRegistry(
            workerHub,
            NullLogger<ViewerControlRegistry>.Instance);

        var firstResult = await SessionControlRoute.CloseViewerLease(
            seeded.Session.Id,
            seeded.Lease.Id,
            HttpContext(seeded.Owner.Id),
            context,
            new WorkerCommandService(context),
            workerHub,
            new RecordingHubContext<SessionHub>(),
            registry,
            CancellationToken.None);
        Assert.Equal(StatusCodes.Status202Accepted, (await Execute(firstResult)).StatusCode);

        var revokedAt = (await context.ViewerLeases.SingleAsync()).RevokedAt;
        var firstCommand = await context.WorkerCommands.SingleAsync();
        firstCommand.State = WorkerCommandState.Failed;
        firstCommand.LastError = "Publisher did not stop in time.";
        firstCommand.CompletedAt = DateTime.UtcNow;
        firstCommand.UpdatedAt = firstCommand.CompletedAt.Value;
        await context.SaveChangesAsync();

        var retryResult = await SessionControlRoute.CloseViewerLease(
            seeded.Session.Id,
            seeded.Lease.Id,
            HttpContext(seeded.Owner.Id),
            context,
            new WorkerCommandService(context),
            workerHub,
            new RecordingHubContext<SessionHub>(),
            registry,
            CancellationToken.None);
        Assert.Equal(StatusCodes.Status202Accepted, (await Execute(retryResult)).StatusCode);

        var commands = await context.WorkerCommands
            .OrderBy(command => command.Sequence)
            .ToListAsync();
        Assert.Equal(2, commands.Count);
        Assert.Equal(WorkerCommandState.Failed, commands[0].State);
        Assert.Equal(WorkerCommandState.Pending, commands[1].State);
        Assert.EndsWith(":2", commands[1].IdempotencyKey, StringComparison.Ordinal);
        Assert.Equal(revokedAt, (await context.ViewerLeases.SingleAsync()).RevokedAt);
        Assert.Equal(SimulationSessionState.Ready, seeded.Session.State);

        var repeatedRetry = await SessionControlRoute.CloseViewerLease(
            seeded.Session.Id,
            seeded.Lease.Id,
            HttpContext(seeded.Owner.Id),
            context,
            new WorkerCommandService(context),
            workerHub,
            new RecordingHubContext<SessionHub>(),
            registry,
            CancellationToken.None);
        Assert.Equal(StatusCodes.Status202Accepted, (await Execute(repeatedRetry)).StatusCode);
        Assert.Equal(2, await context.WorkerCommands.CountAsync());
    }

    [Fact]
    public async Task RepeatedCloseReturnsOkAfterTheWorkerConfirmsCompletion()
    {
        await using var context = TestDataContext.Create();
        var seeded = await SeedLease(context);
        var workerHub = new RecordingHubContext<WorkerHub>();
        var registry = new ViewerControlRegistry(
            workerHub,
            NullLogger<ViewerControlRegistry>.Instance);

        var firstResult = await SessionControlRoute.CloseViewerLease(
            seeded.Session.Id,
            seeded.Lease.Id,
            HttpContext(seeded.Owner.Id),
            context,
            new WorkerCommandService(context),
            workerHub,
            new RecordingHubContext<SessionHub>(),
            registry,
            CancellationToken.None);
        Assert.Equal(StatusCodes.Status202Accepted, (await Execute(firstResult)).StatusCode);

        var command = await context.WorkerCommands.SingleAsync();
        command.State = WorkerCommandState.Completed;
        command.CompletedAt = DateTime.UtcNow;
        command.UpdatedAt = command.CompletedAt.Value;
        await context.SaveChangesAsync();

        var repeated = await SessionControlRoute.CloseViewerLease(
            seeded.Session.Id,
            seeded.Lease.Id,
            HttpContext(seeded.Owner.Id),
            context,
            new WorkerCommandService(context),
            workerHub,
            new RecordingHubContext<SessionHub>(),
            registry,
            CancellationToken.None);
        Assert.Equal(StatusCodes.Status200OK, (await Execute(repeated)).StatusCode);
        Assert.Equal(1, await context.WorkerCommands.CountAsync());
    }

    [Fact]
    public async Task AnotherOwnerCannotRevokeOrStopTheViewer()
    {
        await using var context = TestDataContext.Create();
        var seeded = await SeedLease(context);
        var workerHub = new RecordingHubContext<WorkerHub>();
        var result = await SessionControlRoute.CloseViewerLease(
            seeded.Session.Id,
            seeded.Lease.Id,
            HttpContext(seeded.Owner.Id + 1),
            context,
            new WorkerCommandService(context),
            workerHub,
            new RecordingHubContext<SessionHub>(),
            new ViewerControlRegistry(
                workerHub,
                NullLogger<ViewerControlRegistry>.Instance),
            CancellationToken.None);

        Assert.Equal(StatusCodes.Status404NotFound, (await Execute(result)).StatusCode);
        Assert.Null((await context.ViewerLeases.SingleAsync()).RevokedAt);
        Assert.Empty(context.WorkerCommands);
    }

    [Fact]
    public async Task ClosingAnAlreadyReplacedLeaseDoesNotQueueAStaleStop()
    {
        await using var context = TestDataContext.Create();
        var seeded = await SeedLease(context);
        seeded.Lease.RevokedAt = DateTime.UtcNow.AddSeconds(-1);
        context.ViewerLeases.Add(new ViewerLease
        {
            SimulationSession = seeded.Session,
            SimulationSessionId = seeded.Session.Id,
            Account = seeded.Owner,
            AccountId = seeded.Owner.Id,
            TokenHash = Guid.NewGuid().ToString("N"),
            ExpiresAt = DateTime.UtcNow.AddMinutes(5)
        });
        await context.SaveChangesAsync();
        var workerHub = new RecordingHubContext<WorkerHub>();

        var result = await SessionControlRoute.CloseViewerLease(
            seeded.Session.Id,
            seeded.Lease.Id,
            HttpContext(seeded.Owner.Id),
            context,
            new WorkerCommandService(context),
            workerHub,
            new RecordingHubContext<SessionHub>(),
            new ViewerControlRegistry(
                workerHub,
                NullLogger<ViewerControlRegistry>.Instance),
            CancellationToken.None);

        Assert.Equal(StatusCodes.Status200OK, (await Execute(result)).StatusCode);
        Assert.Empty(context.WorkerCommands);
        Assert.DoesNotContain(workerHub.Messages, message => message.Method == "CommandAvailable");
    }

    [Fact]
    public void StopCompletionMustMatchSessionLeaseAndBooleanResult()
    {
        var sessionId = Guid.NewGuid();
        var leaseId = Guid.NewGuid();
        var command = new WorkerCommand
        {
            SimulationSessionId = sessionId,
            Type = WorkerCommandType.StopViewer,
            IdempotencyKey = $"sys:viewer-stop:{leaseId:N}",
            Payload = JsonDocument.Parse($"{{\"leaseId\":\"{leaseId:D}\"}}")
        };

        Assert.True(WorkerHub.TryValidateViewerStopCompletion(
            command,
            JsonSerializer.SerializeToElement(new { sessionId, leaseId, stopped = false })));
        Assert.False(WorkerHub.TryValidateViewerStopCompletion(
            command,
            JsonSerializer.SerializeToElement(new
            {
                sessionId,
                leaseId = Guid.NewGuid(),
                stopped = true
            })));
    }

    [Fact]
    public async Task ConcurrentCloseRetriesTheDurableCommandUniqueConflict()
    {
        var attempts = 0;
        var resets = 0;

        var result = await SessionControlRoute.RetryViewerCloseConflicts(
            () =>
            {
                attempts++;
                return attempts == 1
                    ? Task.FromException<IResult>(new DbUpdateException(
                        "Concurrent viewer close.",
                        new PostgresException(
                            "duplicate command",
                            "ERROR",
                            "ERROR",
                            PostgresErrorCodes.UniqueViolation)))
                    : Task.FromResult<IResult>(Results.Accepted());
            },
            () => resets++,
            CancellationToken.None);

        Assert.Equal(StatusCodes.Status202Accepted, (await Execute(result)).StatusCode);
        Assert.Equal(2, attempts);
        Assert.Equal(1, resets);
    }

    private static async Task<SeededLease> SeedLease(TestDataContext context)
    {
        var owner = new Account
        {
            Id = 611,
            FirstName = "Viewer",
            LastName = "Owner",
            Email = $"viewer-close-{Guid.NewGuid():N}@example.test",
            Enabled = true
        };
        var worker = new ComputeWorker
        {
            Id = Guid.NewGuid(),
            Name = "viewer-worker",
            State = ComputeWorkerState.Online,
            Capabilities = JsonDocument.Parse(
                """
                {
                  "commandTypes": ["SetViewerSource", "StopViewer"],
                  "viewerSources": ["Scene"]
                }
                """)
        };
        var session = new SimulationSession
        {
            Id = Guid.NewGuid(),
            Account = owner,
            AccountId = owner.Id,
            ComputeWorker = worker,
            ComputeWorkerId = worker.Id,
            State = SimulationSessionState.Ready,
            DesiredRobotCount = 3
        };
        var lease = new ViewerLease
        {
            Id = Guid.NewGuid(),
            SimulationSession = session,
            SimulationSessionId = session.Id,
            Account = owner,
            AccountId = owner.Id,
            TokenHash = Guid.NewGuid().ToString("N"),
            ExpiresAt = DateTime.UtcNow.AddMinutes(5)
        };
        context.AddRange(owner, worker, session, lease);
        await context.SaveChangesAsync();
        return new SeededLease(owner, worker, session, lease);
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

    private sealed record SeededLease(
        Account Owner,
        ComputeWorker Worker,
        SimulationSession Session,
        ViewerLease Lease);

    private sealed class RecordingHubContext<THub> : IHubContext<THub>
        where THub : Hub
    {
        private readonly RecordingHubClients _clients;

        public RecordingHubContext()
        {
            _clients = new RecordingHubClients(this);
        }

        public List<RecordedMessage> Messages { get; } = new();
        public IHubClients Clients => _clients;
        public IGroupManager Groups => null!;

        private sealed class RecordingHubClients(RecordingHubContext<THub> owner) : IHubClients
        {
            public IClientProxy All => new RecordingClientProxy(owner, string.Empty);
            public IClientProxy AllExcept(IReadOnlyList<string> excludedConnectionIds) => All;
            public IClientProxy Client(string connectionId) => All;
            public IClientProxy Clients(IReadOnlyList<string> connectionIds) => All;
            public IClientProxy Group(string groupName) =>
                new RecordingClientProxy(owner, groupName);
            public IClientProxy GroupExcept(
                string groupName,
                IReadOnlyList<string> excludedConnectionIds) => Group(groupName);
            public IClientProxy Groups(IReadOnlyList<string> groupNames) => All;
            public IClientProxy User(string userId) => All;
            public IClientProxy Users(IReadOnlyList<string> userIds) => All;
        }

        private sealed class RecordingClientProxy(
            RecordingHubContext<THub> owner,
            string groupName) : IClientProxy
        {
            public Task SendCoreAsync(
                string method,
                object?[] args,
                CancellationToken cancellationToken = default)
            {
                owner.Messages.Add(new RecordedMessage(groupName, method, args));
                return Task.CompletedTask;
            }
        }
    }

    private sealed record RecordedMessage(
        string GroupName,
        string Method,
        object?[] Arguments);
}
