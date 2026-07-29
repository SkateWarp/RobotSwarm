using System.Data;
using System.Security.Claims;
using Microsoft.AspNetCore.Http;
using Microsoft.AspNetCore.Http.Features;
using Microsoft.AspNetCore.SignalR;
using Microsoft.EntityFrameworkCore;
using Microsoft.Extensions.Configuration;
using Microsoft.Extensions.Logging;
using Microsoft.Extensions.Logging.Abstractions;
using Npgsql;
using SwarmBackend.Entities;
using SwarmBackend.Helpers;
using SwarmBackend.Models;
using SwarmBackend.Routes;
using SwarmBackend.Services;

namespace SwarmBackend.Tests;

[CollectionDefinition(Name, DisableParallelization = true)]
public sealed class PostgresConcurrencyCollection
{
    public const string Name = "PostgreSQL concurrency";
}

[Collection(PostgresConcurrencyCollection.Name)]
public sealed class PostgresConcurrencyTests
{
    private const int AccountRaceIterations = 16;
    private const int TerminalRaceIterations = 12;

    [PostgresFact]
    [Trait("Category", "PostgreSQL")]
    public async Task ConcurrentCanonicalEmailCreationHasOneControlledWinner()
    {
        await PostgresTestDatabase.PrepareAsync();
        await PostgresTestDatabase.ResetAsync();
        var start = NewStartSignal();

        var first = RunTogether(start.Task, () => CreateAccount(
            "\t\r\n Shared.User@Example.Test \v\f",
            "First1!a"));
        var second = RunTogether(start.Task, () => CreateAccount(
            "shared.user@example.test",
            "Second2@a"));

        start.SetResult();
        var outcomes = await Task.WhenAll(first, second);

        Assert.Single(outcomes, outcome => outcome.Succeeded);
        var conflict = Assert.Single(outcomes, outcome => !outcome.Succeeded);
        Assert.Equal("Correo en uso.", conflict.Error);

        await using var verification = PostgresTestDatabase.OpenContext();
        var account = Assert.Single(await verification.Accounts
            .AsNoTracking()
            .ToListAsync());
        Assert.Equal("shared.user@example.test", account.Email);
        Assert.Equal("shared.user@example.test", account.NormalizedEmail);
    }

    [PostgresFact]
    [Trait("Category", "PostgreSQL")]
    public async Task CanonicalEmailIndexRejectsLegacyCaseAndWhitespaceDuplicates()
    {
        await PostgresTestDatabase.PrepareAsync();
        await PostgresTestDatabase.ResetAsync();
        await using var context = PostgresTestDatabase.OpenContext();

        context.Accounts.Add(Account(
            "\t\r\n Legacy.User@Example.Test \v\f",
            Role.User,
            DateTime.UtcNow));
        await context.SaveChangesAsync();
        context.Accounts.Add(Account(
            "legacy.user@example.test",
            Role.User,
            DateTime.UtcNow));

        var exception = await Assert.ThrowsAsync<DbUpdateException>(
            () => context.SaveChangesAsync());
        var postgres = Assert.IsType<PostgresException>(exception.InnerException);
        Assert.Equal(PostgresErrorCodes.UniqueViolation, postgres.SqlState);
        Assert.Equal("IX_Accounts_NormalizedEmail", postgres.ConstraintName);
    }

    [PostgresFact]
    [Trait("Category", "PostgreSQL")]
    public async Task CanonicalEmailColumnAndFormatConstraintUseTheAsciiContract()
    {
        await PostgresTestDatabase.PrepareAsync();
        await PostgresTestDatabase.ResetAsync();
        await using var context = PostgresTestDatabase.OpenContext();

        var valid = Account(
            " \t\n\v\f\rValid.User@Example.Test\r\f\v\n\t ",
            Role.User,
            DateTime.UtcNow);
        context.Accounts.Add(valid);
        await context.SaveChangesAsync();
        context.ChangeTracker.Clear();

        var stored = await context.Accounts
            .AsNoTracking()
            .SingleAsync(account => account.Id == valid.Id);
        Assert.Equal("valid.user@example.test", stored.NormalizedEmail);

        context.Accounts.Add(Account(
            "\u00a0invalid@example.test\u00a0",
            Role.User,
            DateTime.UtcNow));
        var exception = await Assert.ThrowsAsync<DbUpdateException>(
            () => context.SaveChangesAsync());
        var postgres = Assert.IsType<PostgresException>(exception.InnerException);
        Assert.Equal(PostgresErrorCodes.CheckViolation, postgres.SqlState);
        Assert.Equal("CK_Accounts_Email_SupportedFormat", postgres.ConstraintName);
    }

    [PostgresFact]
    [Trait("Category", "PostgreSQL")]
    public async Task ConcurrentAdminDemotionAndDisablementAlwaysLeaveAnAdministrator()
    {
        await PostgresTestDatabase.PrepareAsync();

        for (var iteration = 0; iteration < AccountRaceIterations; iteration++)
        {
            await PostgresTestDatabase.ResetAsync();
            var (adminAId, adminBId) = await SeedAdministrators(iteration);
            var start = NewStartSignal();

            var demotion = RunTogether(start.Task, async () =>
            {
                await using var context = PostgresTestDatabase.OpenContext();
                var service = new AccountService(context, Configuration());
                var result = await service.Update(
                    adminAId,
                    new AccountPatchRequest(null, null, null, null, Role.User));
                return result.Match(_ => true, _ => false);
            });
            var disablement = RunTogether(start.Task, async () =>
            {
                await using var context = PostgresTestDatabase.OpenContext();
                return await new AccountService(context, Configuration()).Delete(adminBId);
            });

            start.SetResult();
            var outcomes = await Task.WhenAll(demotion, disablement);

            await using var verification = PostgresTestDatabase.OpenContext();
            var enabledAdministrators = await verification.Accounts
                .AsNoTracking()
                .CountAsync(account => account.Enabled && account.Role == Role.Admin);

            Assert.Equal(1, enabledAdministrators);
            Assert.True(
                outcomes[0] ^ outcomes[1],
                $"Iteration {iteration + 1} should allow exactly one administrator mutation.");
        }
    }

    [PostgresFact]
    [Trait("Category", "PostgreSQL")]
    public async Task SessionCreationRetriesStaleSnapshotAndRejectsOldPrincipal()
    {
        await PostgresTestDatabase.PrepareAsync();
        await PostgresTestDatabase.ResetAsync();
        var seeded = await SeedSecurityVersionRace();
        var raceId = Guid.NewGuid().ToString("N");
        var putApplication = $"toctou-put-{raceId}";
        var requestApplication = $"toctou-request-{raceId}";

        await using var rowBlocker = PostgresTestDatabase.OpenContext(
            $"toctou-row-{raceId}");
        await using var rowBlockerTransaction = await rowBlocker.Database
            .BeginTransactionAsync(IsolationLevel.ReadCommitted);
        await rowBlocker.Accounts
            .FromSqlInterpolated($"""
                SELECT *
                FROM "Accounts"
                WHERE "Id" = {seeded.AccountId}
                FOR UPDATE
                """)
            .SingleAsync();

        var put = Task.Run(async () =>
        {
            await using var context = PostgresTestDatabase.OpenContext(putApplication);
            var result = await new AccountService(context, Configuration()).Update(
                seeded.AccountId,
                new AccountRequest(
                    "Changed",
                    "By Put",
                    seeded.Email,
                    seeded.NewPassword));
            return result.Match(_ => true, _ => false);
        });

        await WaitForAdvisoryLock(
            putApplication,
            mode: "ExclusiveLock",
            granted: true);
        Assert.False(put.IsCompleted);

        var oldRequest = Task.Run(async () =>
        {
            await using var context = PostgresTestDatabase.OpenContext(requestApplication);
            var httpContext = new DefaultHttpContext
            {
                User = seeded.OldPrincipal
            };
            var result = await SimulationSessionRoute.Create(
                new CreateSimulationSessionRequest(1),
                context,
                Configuration(),
                httpContext,
                CancellationToken.None);
            return Assert.IsAssignableFrom<IStatusCodeHttpResult>(result).StatusCode;
        });

        await WaitForAdvisoryLock(
            requestApplication,
            mode: "ShareLock",
            granted: false);
        Assert.False(oldRequest.IsCompleted);

        await rowBlockerTransaction.CommitAsync();
        Assert.True(await put);
        Assert.Equal(StatusCodes.Status401Unauthorized, await oldRequest);

        await using var verification = PostgresTestDatabase.OpenContext();
        var account = await verification.Accounts
            .AsNoTracking()
            .SingleAsync(candidate => candidate.Id == seeded.AccountId);
        Assert.NotNull(account.Updated);
        Assert.NotEqual(seeded.OldVersion, account.Updated);
        Assert.NotNull(account.PasswordHash);
        Assert.True(BCrypt.Net.BCrypt.Verify(
            seeded.NewPassword,
            account.PasswordHash));
        Assert.False(await verification.SimulationSessions.AnyAsync());
    }

    [PostgresFact]
    [Trait("Category", "PostgreSQL")]
    public async Task RevokedAdministratorCannotMutateAnotherAccount()
    {
        await PostgresTestDatabase.PrepareAsync();
        await PostgresTestDatabase.ResetAsync();
        var seeded = await SeedAdministratorAuthorizationRace();
        var raceId = Guid.NewGuid().ToString("N");
        var demotionApplication = $"admin-demotion-{raceId}";
        var requestApplication = $"stale-admin-{raceId}";

        await using var rowBlocker = PostgresTestDatabase.OpenContext(
            $"admin-row-{raceId}");
        await using var rowBlockerTransaction = await rowBlocker.Database
            .BeginTransactionAsync(IsolationLevel.ReadCommitted);
        await rowBlocker.Accounts
            .FromSqlInterpolated($"""
                SELECT *
                FROM "Accounts"
                WHERE "Id" = {seeded.ActorId}
                FOR UPDATE
                """)
            .SingleAsync();

        var demotion = Task.Run(async () =>
        {
            await using var context = PostgresTestDatabase.OpenContext(
                demotionApplication);
            var result = await new AccountService(context, Configuration()).Update(
                seeded.ActorId,
                new AccountPatchRequest(null, null, null, null, Role.User));
            return result.Match(_ => true, _ => false);
        });

        // A role mutation holds the global administrator lock and the actor's
        // account lock before it reaches the blocked row query.
        await WaitForAdvisoryLock(
            demotionApplication,
            mode: "ExclusiveLock",
            granted: true,
            minimumCount: 2);
        Assert.False(demotion.IsCompleted);

        var staleRequest = Task.Run(async () =>
        {
            await using var context = PostgresTestDatabase.OpenContext(
                requestApplication);
            var httpContext = new DefaultHttpContext
            {
                User = seeded.OldPrincipal
            };
            var result = await AccountRoute.Update(
                seeded.TargetId,
                new AccountRequest(
                    "Changed",
                    "By stale administrator",
                    "changed-target@example.test",
                    "Target1!"),
                new AccountService(context, Configuration()),
                httpContext);
            return Assert.IsAssignableFrom<IStatusCodeHttpResult>(result).StatusCode;
        });

        await WaitForAdvisoryLock(
            requestApplication,
            mode: "ExclusiveLock",
            granted: false);
        Assert.False(staleRequest.IsCompleted);

        await rowBlockerTransaction.CommitAsync();
        Assert.True(await demotion);
        Assert.Equal(StatusCodes.Status401Unauthorized, await staleRequest);

        await using var verification = PostgresTestDatabase.OpenContext();
        var actor = await verification.Accounts
            .AsNoTracking()
            .SingleAsync(account => account.Id == seeded.ActorId);
        var target = await verification.Accounts
            .AsNoTracking()
            .SingleAsync(account => account.Id == seeded.TargetId);
        Assert.Equal(Role.User, actor.Role);
        Assert.NotEqual(seeded.OldVersion, actor.Updated);
        Assert.Equal("Unchanged", target.FirstName);
        Assert.Equal("Target", target.LastName);
        Assert.Equal("target@example.test", target.Email);
        Assert.Equal("target-password-hash", target.PasswordHash);
    }

    [PostgresFact]
    [Trait("Category", "PostgreSQL")]
    public async Task HeartbeatAndAccountDisablementShareOneCleanupSequence()
    {
        await PostgresTestDatabase.PrepareAsync();

        for (var iteration = 0; iteration < TerminalRaceIterations; iteration++)
        {
            await PostgresTestDatabase.ResetAsync();
            var seeded = await SeedTerminalSession(iteration);
            var start = NewStartSignal();

            var heartbeat = RunTogether(
                start.Task,
                () => SendHeartbeat(seeded));
            var disablement = RunTogether(start.Task, async () =>
            {
                await using var context = PostgresTestDatabase.OpenContext();
                return await new AccountService(context, Configuration())
                    .Delete(seeded.AccountId);
            });

            start.SetResult();
            await WithoutUniqueViolation(heartbeat, "WorkerHub.Heartbeat");
            Assert.True(await WithoutUniqueViolation(
                disablement,
                "AccountService.Delete"));

            await using var verification = PostgresTestDatabase.OpenContext();
            var commands = await verification.WorkerCommands
                .AsNoTracking()
                .Where(command => command.SimulationSessionId == seeded.SessionId)
                .OrderBy(command => command.Sequence)
                .ToListAsync();
            var owner = await verification.Accounts
                .AsNoTracking()
                .SingleAsync(account => account.Id == seeded.AccountId);

            Assert.False(owner.Enabled);
            var cleanup = Assert.Single(commands);
            Assert.Equal(1, cleanup.Sequence);
            Assert.Equal(WorkerCommandType.StopSession, cleanup.Type);
            await AssertNoRepeatedSequence(verification);
        }
    }

    [PostgresFact]
    [Trait("Category", "PostgreSQL")]
    public async Task HeartbeatAndPhysicalSessionDeletionDoNotLeaveAConflictingCommand()
    {
        await PostgresTestDatabase.PrepareAsync();

        for (var iteration = 0; iteration < TerminalRaceIterations; iteration++)
        {
            await PostgresTestDatabase.ResetAsync();
            var seeded = await SeedTerminalSession(iteration);
            var start = NewStartSignal();

            var heartbeat = RunTogether(
                start.Task,
                () => SendHeartbeat(seeded));
            var deletion = RunTogether(start.Task, async () =>
            {
                await using var context = PostgresTestDatabase.OpenContext();
                return await context.SimulationSessions
                    .Where(session => session.Id == seeded.SessionId)
                    .ExecuteDeleteAsync();
            });

            start.SetResult();
            await WithoutUniqueViolation(heartbeat, "WorkerHub.Heartbeat");
            Assert.Equal(1, await WithoutUniqueViolation(
                deletion,
                "SimulationSession deletion"));

            await using var verification = PostgresTestDatabase.OpenContext();
            Assert.False(await verification.SimulationSessions
                .AnyAsync(session => session.Id == seeded.SessionId));
            Assert.False(await verification.WorkerCommands
                .AnyAsync(command => command.SimulationSessionId == seeded.SessionId));
            await AssertNoRepeatedSequence(verification);
        }
    }

    private static async Task<(int AdminAId, int AdminBId)> SeedAdministrators(
        int iteration)
    {
        await using var context = PostgresTestDatabase.OpenContext();
        var now = DateTime.UtcNow;
        var adminA = Account($"admin-a-{iteration}@example.test", Role.Admin, now);
        var adminB = Account($"admin-b-{iteration}@example.test", Role.Admin, now);
        context.Accounts.AddRange(adminA, adminB);
        await context.SaveChangesAsync();
        return (adminA.Id, adminB.Id);
    }

    private static async Task<TerminalRaceSeed> SeedTerminalSession(int iteration)
    {
        await using var context = PostgresTestDatabase.OpenContext();
        var now = DateTime.UtcNow;
        var owner = Account($"terminal-owner-{iteration}@example.test", Role.User, now);
        var credentialCreatedAt = new DateTime(
            2026,
            7,
            20,
            12,
            0,
            iteration,
            DateTimeKind.Utc);
        var worker = new ComputeWorker
        {
            Name = $"postgres-race-worker-{iteration}",
            State = ComputeWorkerState.Online,
            MaxConcurrentSessions = 2,
            CredentialHash = new string('a', 64),
            CredentialCreatedAt = credentialCreatedAt,
            LastHeartbeatAt = now,
            CreatedAt = now,
            UpdatedAt = now
        };
        var session = new SimulationSession
        {
            Account = owner,
            ComputeWorker = worker,
            State = SimulationSessionState.Expired,
            DesiredRobotCount = 2,
            FailureReason = "PostgreSQL concurrency acceptance fixture.",
            StoppedAt = now - TimeSpan.FromMinutes(1),
            CreatedAt = now - TimeSpan.FromMinutes(2),
            UpdatedAt = now - TimeSpan.FromMinutes(1)
        };

        context.AddRange(owner, worker, session);
        await context.SaveChangesAsync();
        return new TerminalRaceSeed(
            owner.Id,
            worker.Id,
            credentialCreatedAt,
            session.Id);
    }

    private static async Task<SecurityVersionRaceSeed> SeedSecurityVersionRace()
    {
        await using var context = PostgresTestDatabase.OpenContext();
        var oldVersion = new DateTime(
            2026,
            7,
            1,
            12,
            0,
            0,
            DateTimeKind.Utc);
        const string email = "old-principal@example.test";
        const string newPassword = "Racepass1!";
        var account = Account(email, Role.User, oldVersion);
        account.PasswordHash = "old-password-hash";
        account.Updated = oldVersion;
        context.Accounts.Add(account);
        await context.SaveChangesAsync();

        var principal = new ClaimsPrincipal(new ClaimsIdentity(new[]
        {
            new Claim("id", account.Id.ToString()),
            new Claim(ClaimTypes.Role, Role.User.ToString()),
            new Claim("account_version", oldVersion.Ticks.ToString())
        }, "postgres-test"));
        return new SecurityVersionRaceSeed(
            account.Id,
            email,
            newPassword,
            oldVersion,
            principal);
    }

    private static async Task<AdministratorAuthorizationRaceSeed>
        SeedAdministratorAuthorizationRace()
    {
        await using var context = PostgresTestDatabase.OpenContext();
        var oldVersion = new DateTime(
            2026,
            7,
            20,
            16,
            30,
            0,
            DateTimeKind.Utc);
        var actor = Account("actor-admin@example.test", Role.Admin, oldVersion);
        actor.PasswordHash = "actor-password-hash";
        actor.Updated = oldVersion;
        var peer = Account("peer-admin@example.test", Role.Admin, oldVersion);
        peer.PasswordHash = "peer-password-hash";
        var target = Account("target@example.test", Role.User, oldVersion);
        target.FirstName = "Unchanged";
        target.LastName = "Target";
        target.PasswordHash = "target-password-hash";
        context.Accounts.AddRange(actor, peer, target);
        await context.SaveChangesAsync();

        var principal = new ClaimsPrincipal(new ClaimsIdentity(new[]
        {
            new Claim("id", actor.Id.ToString()),
            new Claim(ClaimTypes.Role, Role.Admin.ToString()),
            new Claim("account_version", oldVersion.Ticks.ToString())
        }, "postgres-test"));
        return new AdministratorAuthorizationRaceSeed(
            actor.Id,
            target.Id,
            oldVersion,
            principal);
    }

    private static Account Account(string email, Role role, DateTime now)
    {
        return new Account
        {
            FirstName = "PostgreSQL",
            LastName = "Race",
            Email = email,
            Enabled = true,
            Verified = now,
            Created = now,
            DateCreated = now,
            Role = role
        };
    }

    private static async Task<WorkerRegistrationResponse> SendHeartbeat(
        TerminalRaceSeed seeded)
    {
        await using var context = PostgresTestDatabase.OpenContext();
        var logger = new PersistenceConflictLogger();
        var callerContext = new WorkerCallerContext(
            seeded.WorkerId,
            seeded.CredentialCreatedAt);
        var connections = new WorkerConnectionRegistry(
            NullLogger<WorkerConnectionRegistry>.Instance);
        Assert.Equal(
            WorkerConnectionClaim.Accepted,
            connections.Claim(
                seeded.WorkerId,
                agentInstanceId: null,
                callerContext.ConnectionId,
                callerContext.Abort));
        var hub = new WorkerHub(
            context,
            SilentSessionHubContext.Instance,
            connections,
            Configuration(),
            logger)
        {
            Context = callerContext
        };

        var response = await hub.Heartbeat(new WorkerHeartbeatRequest(
            ImageVersion: null,
            Capabilities: null,
            ActiveSessionIds: new[] { seeded.SessionId }));
        Assert.Empty(logger.UniqueViolations);
        return response;
    }

    private static async Task AssertNoRepeatedSequence(DataContext context)
    {
        var repeated = await context.WorkerCommands
            .AsNoTracking()
            .GroupBy(command => new
            {
                command.SimulationSessionId,
                command.Sequence
            })
            .Where(group => group.Count() > 1)
            .Select(group => new
            {
                group.Key.SimulationSessionId,
                group.Key.Sequence
            })
            .ToListAsync();
        Assert.Empty(repeated);
    }

    private static async Task<T> WithoutUniqueViolation<T>(
        Task<T> operation,
        string operationName)
    {
        try
        {
            return await operation;
        }
        catch (Exception exception)
        {
            var postgres = FindPostgresException(exception);
            Assert.False(
                postgres?.SqlState == PostgresErrorCodes.UniqueViolation,
                $"{operationName} surfaced PostgreSQL 23505 on constraint "
                + $"{postgres?.ConstraintName ?? "(unknown)"}.");
            throw;
        }
    }

    private static PostgresException? FindPostgresException(Exception exception)
    {
        for (var current = exception; current != null; current = current.InnerException!)
        {
            if (current is PostgresException postgres)
            {
                return postgres;
            }
        }

        return null;
    }

    private static async Task WaitForAdvisoryLock(
        string applicationName,
        string mode,
        bool granted,
        int minimumCount = 1)
    {
        await using var observer = PostgresTestDatabase.OpenConnection(
            $"lock-observer-{Guid.NewGuid():N}");
        await observer.OpenAsync();
        var deadline = DateTime.UtcNow + TimeSpan.FromSeconds(5);
        while (DateTime.UtcNow < deadline)
        {
            await using var command = observer.CreateCommand();
            command.CommandText = """
                SELECT COUNT(*) >= @minimum_count
                    FROM (
                    SELECT locks.pid
                    FROM pg_locks AS locks
                    INNER JOIN pg_stat_activity AS activity
                        ON activity.pid = locks.pid
                    WHERE activity.application_name = @application_name
                      AND locks.locktype = 'advisory'
                      AND locks.mode = @mode
                      AND locks.granted = @granted
                    ) AS matching_locks
                """;
            command.Parameters.AddWithValue("application_name", applicationName);
            command.Parameters.AddWithValue("mode", mode);
            command.Parameters.AddWithValue("granted", granted);
            command.Parameters.AddWithValue("minimum_count", minimumCount);
            if (await command.ExecuteScalarAsync() is true)
            {
                return;
            }

            await Task.Delay(TimeSpan.FromMilliseconds(20));
        }

        Assert.Fail(
            $"{applicationName} did not expose {minimumCount} expected {mode} "
            + $"advisory lock(s) (granted={granted}).");
    }

    private static Task<T> RunTogether<T>(Task start, Func<Task<T>> operation)
    {
        return Task.Run(async () =>
        {
            await start;
            return await operation();
        });
    }

    private static async Task<AccountCreationOutcome> CreateAccount(
        string email,
        string password)
    {
        await using var context = PostgresTestDatabase.OpenContext();
        var result = await new AccountService(context, Configuration()).Create(
            new AccountRequest("Concurrent", "Account", email, password));
        return result.Match(
            _ => new AccountCreationOutcome(true, null),
            error => new AccountCreationOutcome(false, error.Message));
    }

    private static TaskCompletionSource NewStartSignal()
    {
        return new TaskCompletionSource(TaskCreationOptions.RunContinuationsAsynchronously);
    }

    private static IConfiguration Configuration()
    {
        return new ConfigurationBuilder().Build();
    }

    private sealed record TerminalRaceSeed(
        int AccountId,
        Guid WorkerId,
        DateTime CredentialCreatedAt,
        Guid SessionId);

    private sealed record AccountCreationOutcome(bool Succeeded, string? Error);

    private sealed record SecurityVersionRaceSeed(
        int AccountId,
        string Email,
        string NewPassword,
        DateTime OldVersion,
        ClaimsPrincipal OldPrincipal);

    private sealed record AdministratorAuthorizationRaceSeed(
        int ActorId,
        int TargetId,
        DateTime OldVersion,
        ClaimsPrincipal OldPrincipal);

    private sealed class WorkerCallerContext : HubCallerContext
    {
        private readonly CancellationTokenSource aborted = new();

        public WorkerCallerContext(Guid workerId, DateTime credentialCreatedAt)
        {
            ConnectionId = $"postgres-race-{Guid.NewGuid():N}";
            User = new ClaimsPrincipal(new ClaimsIdentity(new[]
            {
                new Claim("worker_id", workerId.ToString()),
                new Claim(
                    "worker_credential_version",
                    credentialCreatedAt.Ticks.ToString())
            }, "postgres-test"));
        }

        public override string ConnectionId { get; }
        public override string? UserIdentifier => User?.FindFirst("worker_id")?.Value;
        public override ClaimsPrincipal? User { get; }
        public override IDictionary<object, object?> Items { get; } =
            new Dictionary<object, object?>();
        public override IFeatureCollection Features { get; } = new FeatureCollection();
        public override CancellationToken ConnectionAborted => aborted.Token;

        public override void Abort()
        {
            aborted.Cancel();
        }
    }

    private sealed class SilentSessionHubContext : IHubContext<SessionHub>
    {
        public static SilentSessionHubContext Instance { get; } = new();
        public IHubClients Clients { get; } = new SilentHubClients();
        public IGroupManager Groups => null!;

        private sealed class SilentHubClients : IHubClients
        {
            private static readonly IClientProxy ClientProxy = new SilentClientProxy();

            public IClientProxy All => ClientProxy;
            public IClientProxy AllExcept(IReadOnlyList<string> excludedConnectionIds) =>
                ClientProxy;
            public IClientProxy Client(string connectionId) => ClientProxy;
            public IClientProxy Clients(IReadOnlyList<string> connectionIds) => ClientProxy;
            public IClientProxy Group(string groupName) => ClientProxy;
            public IClientProxy GroupExcept(
                string groupName,
                IReadOnlyList<string> excludedConnectionIds) => ClientProxy;
            public IClientProxy Groups(IReadOnlyList<string> groupNames) => ClientProxy;
            public IClientProxy User(string userId) => ClientProxy;
            public IClientProxy Users(IReadOnlyList<string> userIds) => ClientProxy;
        }

        private sealed class SilentClientProxy : IClientProxy
        {
            public Task SendCoreAsync(
                string method,
                object?[] args,
                CancellationToken cancellationToken = default)
            {
                return Task.CompletedTask;
            }
        }
    }

    private sealed class PersistenceConflictLogger : ILogger<WorkerHub>
    {
        public List<string?> UniqueViolations { get; } = new();

        public IDisposable? BeginScope<TState>(TState state) where TState : notnull
        {
            return NullScope.Instance;
        }

        public bool IsEnabled(LogLevel logLevel)
        {
            return true;
        }

        public void Log<TState>(
            LogLevel logLevel,
            EventId eventId,
            TState state,
            Exception? exception,
            Func<TState, Exception?, string> formatter)
        {
            if (exception != null
                && FindPostgresException(exception) is
                {
                    SqlState: PostgresErrorCodes.UniqueViolation
                } postgres)
            {
                UniqueViolations.Add(postgres.ConstraintName);
            }
        }

        private sealed class NullScope : IDisposable
        {
            public static NullScope Instance { get; } = new();

            public void Dispose()
            {
            }
        }
    }
}

public sealed class PostgresFactAttribute : FactAttribute
{
    public PostgresFactAttribute()
    {
        if (!PostgresTestDatabase.IsConfigured)
        {
            Skip = $"Set {PostgresTestDatabase.ConnectionVariable} to run PostgreSQL races.";
        }
    }
}

internal static class PostgresTestDatabase
{
    internal const string ConnectionVariable =
        "ROBOTSWARM_POSTGRES_TEST_CONNECTION";
    private static readonly SemaphoreSlim PreparationLock = new(1, 1);
    private static bool prepared;

    static PostgresTestDatabase()
    {
        AppContext.SetSwitch("Npgsql.EnableLegacyTimestampBehavior", true);
    }

    internal static bool IsConfigured =>
        !string.IsNullOrWhiteSpace(Environment.GetEnvironmentVariable(ConnectionVariable));

    internal static DataContext OpenContext(string? applicationName = null)
    {
        var options = new DbContextOptionsBuilder<DataContext>()
            .UseNpgsql(ConnectionString(applicationName))
            .EnableDetailedErrors()
            .Options;
        return new DataContext(options);
    }

    internal static NpgsqlConnection OpenConnection(string applicationName)
    {
        return new NpgsqlConnection(ConnectionString(applicationName));
    }

    internal static async Task PrepareAsync()
    {
        await PreparationLock.WaitAsync();
        try
        {
            if (prepared)
            {
                return;
            }

            await using var context = OpenContext();
            await context.Database.MigrateAsync();
            prepared = true;
        }
        finally
        {
            PreparationLock.Release();
        }
    }

    internal static async Task ResetAsync()
    {
        await using var context = OpenContext();
        await context.Database.ExecuteSqlRawAsync(
            "TRUNCATE TABLE \"Accounts\", \"ComputeWorkers\" "
            + "RESTART IDENTITY CASCADE");
    }

    private static string ConnectionString(string? applicationName)
    {
        var connectionString = Environment.GetEnvironmentVariable(ConnectionVariable);
        if (string.IsNullOrWhiteSpace(connectionString))
        {
            throw new InvalidOperationException(
                $"{ConnectionVariable} is required for PostgreSQL integration tests.");
        }

        var builder = new NpgsqlConnectionStringBuilder(connectionString);
        if (!string.IsNullOrWhiteSpace(applicationName))
        {
            builder.ApplicationName = applicationName;
        }

        return builder.ConnectionString;
    }
}
