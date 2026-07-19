using System.Security.Cryptography;
using System.Text;
using System.Text.Json;
using Microsoft.AspNetCore.Http;
using Microsoft.EntityFrameworkCore;
using Microsoft.EntityFrameworkCore.Storage.ValueConversion;
using Microsoft.Extensions.Configuration;
using Microsoft.Extensions.DependencyInjection;
using SwarmBackend.Entities;
using SwarmBackend.Helpers;
using SwarmBackend.Models;
using SwarmBackend.Routes;

namespace SwarmBackend.Tests;

public sealed class ViewerAuthRouteTests
{
    [Fact]
    public async Task ViewerLeaseCanReadOnlyItsOwnSessionPath()
    {
        await using var context = CreateContext();
        var owner = CreateAccount(1, "owner@example.test");
        var otherOwner = CreateAccount(2, "other@example.test");
        var session = CreateSession(owner);
        var otherSession = CreateSession(otherOwner);
        const string token = "lease-token";
        context.AddRange(
            owner,
            otherOwner,
            session,
            otherSession,
            CreateLease(session, owner, token));
        await context.SaveChangesAsync();

        var ownResult = await AuthorizeRead(context, token, ScenePath(session.Id));
        var otherResult = await AuthorizeRead(
            context,
            token,
            ScenePath(otherSession.Id));

        Assert.Equal(StatusCodes.Status200OK, await StatusCode(ownResult));
        Assert.Equal(StatusCodes.Status401Unauthorized, await StatusCode(otherResult));
    }

    [Fact]
    public async Task ViewerLeaseAccountMustOwnTheSession()
    {
        await using var context = CreateContext();
        var sessionOwner = CreateAccount(1, "owner@example.test");
        var leaseOwner = CreateAccount(2, "lease@example.test");
        var session = CreateSession(sessionOwner);
        const string token = "mismatched-owner-token";
        context.AddRange(
            sessionOwner,
            leaseOwner,
            session,
            CreateLease(session, leaseOwner, token));
        await context.SaveChangesAsync();

        var result = await AuthorizeRead(context, token, ScenePath(session.Id));

        Assert.Equal(StatusCodes.Status401Unauthorized, await StatusCode(result));
    }

    [Fact]
    public async Task PublisherStaysDisabledWhenTheFeatureFlagIsOff()
    {
        await using var context = CreateContext();
        var owner = CreateAccount(1, "owner@example.test");
        var session = CreateSession(owner);
        var (publishToken, _) = ViewerPublishToken.Generate();
        context.AddRange(
            owner,
            session,
            CreateLease(session, owner, "read-token", publishToken));
        await context.SaveChangesAsync();

        var result = await AuthorizePublish(
            context,
            publishToken,
            ScenePath(session.Id),
            publishingEnabled: false);

        Assert.Equal(StatusCodes.Status401Unauthorized, await StatusCode(result));
    }

    [Fact]
    public async Task WorkerCredentialCannotPublishEvenForItsOwnSession()
    {
        await using var context = CreateContext();
        var owner = CreateAccount(1, "owner@example.test");
        var (worker, workerCredential) = CreateWorker();
        var session = CreateSession(owner, worker);
        var (publishToken, _) = ViewerPublishToken.Generate();
        context.AddRange(
            owner,
            worker,
            session,
            CreateLease(session, owner, "read-token", publishToken));
        await context.SaveChangesAsync();

        var result = await AuthorizePublish(
            context,
            workerCredential,
            ScenePath(session.Id),
            publishingEnabled: true);

        Assert.Equal(StatusCodes.Status401Unauthorized, await StatusCode(result));
    }

    [Fact]
    public async Task PublishLeaseCanPublishOnlyItsExactSourcePath()
    {
        await using var context = CreateContext();
        var owner = CreateAccount(1, "owner@example.test");
        var otherOwner = CreateAccount(2, "other@example.test");
        var session = CreateSession(owner);
        var otherSession = CreateSession(otherOwner);
        const string readToken = "read-token";
        var (publishToken, _) = ViewerPublishToken.Generate();
        context.AddRange(
            owner,
            otherOwner,
            session,
            otherSession,
            CreateLease(session, owner, readToken, publishToken));
        await context.SaveChangesAsync();

        var ownResult = await AuthorizePublish(
            context,
            publishToken,
            ScenePath(session.Id),
            publishingEnabled: true);
        var otherSessionResult = await AuthorizePublish(
            context,
            publishToken,
            ScenePath(otherSession.Id),
            publishingEnabled: true);
        var readTokenResult = await AuthorizePublish(
            context,
            readToken,
            ScenePath(session.Id),
            publishingEnabled: true);
        var publishTokenReadResult = await AuthorizeRead(
            context,
            publishToken,
            ScenePath(session.Id));

        Assert.Equal(StatusCodes.Status200OK, await StatusCode(ownResult));
        Assert.Equal(
            StatusCodes.Status401Unauthorized,
            await StatusCode(otherSessionResult));
        Assert.Equal(
            StatusCodes.Status401Unauthorized,
            await StatusCode(readTokenResult));
        Assert.Equal(
            StatusCodes.Status401Unauthorized,
            await StatusCode(publishTokenReadResult));
    }

    [Fact]
    public async Task RobotPublishLeaseDoesNotAuthorizeTheScenePath()
    {
        await using var context = CreateContext();
        var owner = CreateAccount(1, "owner@example.test");
        var session = CreateSession(owner);
        var (publishToken, _) = ViewerPublishToken.Generate();
        context.AddRange(
            owner,
            session,
            CreateLease(
                session,
                owner,
                "robot-read-token",
                publishToken,
                ViewerSourceType.RobotCamera,
                "tb3_0"));
        await context.SaveChangesAsync();

        var robotResult = await AuthorizePublish(
            context,
            publishToken,
            RobotPath(session.Id, "tb3_0"),
            publishingEnabled: true);
        var sceneResult = await AuthorizePublish(
            context,
            publishToken,
            ScenePath(session.Id),
            publishingEnabled: true);

        Assert.Equal(StatusCodes.Status200OK, await StatusCode(robotResult));
        Assert.Equal(
            StatusCodes.Status401Unauthorized,
            await StatusCode(sceneResult));
    }

    [Fact]
    public async Task RevokedExpiredAndStoppedLeasesCannotPublish()
    {
        await using var context = CreateContext();
        var owner = CreateAccount(1, "owner@example.test");
        var activeSession = CreateSession(owner);
        var stoppedSession = CreateSession(owner);
        stoppedSession.State = SimulationSessionState.Stopped;
        var (revokedToken, _) = ViewerPublishToken.Generate();
        var (expiredToken, _) = ViewerPublishToken.Generate();
        var (stoppedToken, _) = ViewerPublishToken.Generate();
        var revokedLease = CreateLease(
            activeSession,
            owner,
            "revoked-read-token",
            revokedToken);
        revokedLease.RevokedAt = DateTime.UtcNow;
        var expiredLease = CreateLease(
            activeSession,
            owner,
            "expired-read-token",
            expiredToken);
        expiredLease.ExpiresAt = DateTime.UtcNow.AddMinutes(-1);
        context.AddRange(
            owner,
            activeSession,
            stoppedSession,
            revokedLease,
            expiredLease,
            CreateLease(
                stoppedSession,
                owner,
                "stopped-read-token",
                stoppedToken));
        await context.SaveChangesAsync();

        foreach (var (token, path) in new[]
        {
            (revokedToken, ScenePath(activeSession.Id)),
            (expiredToken, ScenePath(activeSession.Id)),
            (stoppedToken, ScenePath(stoppedSession.Id))
        })
        {
            var result = await AuthorizePublish(
                context,
                token,
                path,
                publishingEnabled: true);
            Assert.Equal(
                StatusCodes.Status401Unauthorized,
                await StatusCode(result));
        }
    }

    [Fact]
    public void PublishTokensAreCanonicalAndOnlyTheirHashIsStored()
    {
        var (token, expectedHash) = ViewerPublishToken.Generate();

        Assert.Equal(43, token.Length);
        Assert.All(token, character => Assert.True(
            char.IsAsciiLetterOrDigit(character)
            || character is '_' or '-'));
        Assert.True(ViewerPublishToken.TryHash(token, out var hash));
        Assert.Equal(expectedHash, hash);
        Assert.Equal(64, hash.Length);
    }

    [Fact]
    public void ViewerLeaseResponseHasNoPublishCredentialField()
    {
        var propertyNames = typeof(ViewerLeaseResponse)
            .GetProperties()
            .Select(property => property.Name)
            .ToArray();

        Assert.DoesNotContain(
            propertyNames,
            name => name.Contains("PublishToken", StringComparison.OrdinalIgnoreCase));
    }

    private static DataContext CreateContext()
    {
        var options = new DbContextOptionsBuilder<DataContext>()
            .UseInMemoryDatabase(Guid.NewGuid().ToString("N"))
            .Options;
        return new ViewerTestContext(options);
    }

    private sealed class ViewerTestContext : DataContext
    {
        public ViewerTestContext(DbContextOptions<DataContext> options)
            : base(options)
        {
        }

        protected override void OnModelCreating(ModelBuilder modelBuilder)
        {
            base.OnModelCreating(modelBuilder);

            var jsonConverter = new ValueConverter<JsonDocument, string>(
                document => document.RootElement.GetRawText(),
                json => JsonDocument.Parse(json, new JsonDocumentOptions()));
            var nullableJsonConverter = new ValueConverter<JsonDocument?, string?>(
                document => document == null
                    ? null
                    : document.RootElement.GetRawText(),
                json => json == null
                    ? null
                    : JsonDocument.Parse(json, new JsonDocumentOptions()));

            modelBuilder.Entity<ComputeWorker>()
                .Property(worker => worker.Capabilities)
                .HasConversion(jsonConverter);
            modelBuilder.Entity<TaskRun>()
                .Property(task => task.Parameters)
                .HasConversion(jsonConverter);
            modelBuilder.Entity<TaskRun>()
                .Property(task => task.Result)
                .HasConversion(nullableJsonConverter);
            modelBuilder.Entity<WorkerCommand>()
                .Property(command => command.Payload)
                .HasConversion(jsonConverter);
            modelBuilder.Entity<WorkerCommand>()
                .Property(command => command.Result)
                .HasConversion(nullableJsonConverter);
            modelBuilder.Entity<TaskLog>()
                .Property(log => log.Parameters)
                .HasConversion(jsonConverter);
        }
    }

    private static Account CreateAccount(int id, string email)
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

    private static SimulationSession CreateSession(
        Account owner,
        ComputeWorker? worker = null)
    {
        return new SimulationSession
        {
            AccountId = owner.Id,
            Account = owner,
            ComputeWorkerId = worker?.Id,
            ComputeWorker = worker,
            State = SimulationSessionState.Active,
            DesiredRobotCount = 1
        };
    }

    private static ViewerLease CreateLease(
        SimulationSession session,
        Account leaseOwner,
        string token,
        string? publishToken = null,
        ViewerSourceType source = ViewerSourceType.Scene,
        string? robotRuntimeId = null)
    {
        return new ViewerLease
        {
            SimulationSessionId = session.Id,
            SimulationSession = session,
            AccountId = leaseOwner.Id,
            Account = leaseOwner,
            Source = source,
            RobotRuntimeId = robotRuntimeId,
            TokenHash = HashToken(token),
            PublishTokenHash = publishToken == null
                ? null
                : ViewerPublishToken.Hash(publishToken),
            ExpiresAt = DateTime.UtcNow.AddMinutes(5)
        };
    }

    private static (ComputeWorker Worker, string Credential) CreateWorker()
    {
        var (secret, hash) = WorkerCredential.Generate();
        var worker = new ComputeWorker
        {
            Name = "worker-" + Guid.NewGuid().ToString("N"),
            State = ComputeWorkerState.Online,
            CredentialHash = hash,
            Capabilities = JsonDocument.Parse(
                """
                {
                  "commandTypes": ["SetViewerSource"],
                  "viewerSources": ["Scene", "RobotCamera"]
                }
                """)
        };
        return (worker, $"{worker.Id}.{secret}");
    }

    private static async Task<IResult> AuthorizeRead(
        DataContext context,
        string token,
        string path)
    {
        return await ViewerAuthRoute.Authorize(
            new ViewerAuthRequest
            {
                Token = token,
                Action = "read",
                Path = path
            },
            context,
            Configuration(publishingEnabled: false),
            CancellationToken.None);
    }

    private static async Task<IResult> AuthorizePublish(
        DataContext context,
        string credential,
        string path,
        bool publishingEnabled)
    {
        return await ViewerAuthRoute.Authorize(
            new ViewerAuthRequest
            {
                Token = credential,
                Action = "publish",
                Path = path
            },
            context,
            Configuration(publishingEnabled),
            CancellationToken.None);
    }

    private static IConfiguration Configuration(bool publishingEnabled)
    {
        return new ConfigurationBuilder()
            .AddInMemoryCollection(new Dictionary<string, string?>
            {
                ["Viewer:WorkerPublishingEnabled"] = publishingEnabled.ToString()
            })
            .Build();
    }

    private static string ScenePath(Guid sessionId)
    {
        Assert.True(ViewerStreamAddress.TryCreate(
            sessionId,
            ViewerSourceType.Scene,
            robotRuntimeId: null,
            out var address));
        return address.StreamPath;
    }

    private static string RobotPath(Guid sessionId, string robotRuntimeId)
    {
        Assert.True(ViewerStreamAddress.TryCreate(
            sessionId,
            ViewerSourceType.RobotCamera,
            robotRuntimeId,
            out var address));
        return address.StreamPath;
    }

    private static string HashToken(string token)
    {
        return Convert.ToHexString(
            SHA256.HashData(Encoding.UTF8.GetBytes(token)));
    }

    private static async Task<int> StatusCode(IResult result)
    {
        var context = new DefaultHttpContext
        {
            RequestServices = new ServiceCollection()
                .AddLogging()
                .BuildServiceProvider()
        };
        await result.ExecuteAsync(context);
        return context.Response.StatusCode;
    }
}
