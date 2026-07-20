using System.Security.Claims;
using System.Text.Json;
using Microsoft.AspNetCore.Http;
using Microsoft.EntityFrameworkCore;
using Microsoft.Extensions.Configuration;
using Microsoft.Extensions.DependencyInjection;
using Npgsql;
using SwarmBackend.Entities;
using SwarmBackend.Models;
using SwarmBackend.Routes;
using SwarmBackend.Services;

namespace SwarmBackend.Tests;

public sealed class SessionSafetyTests
{
    [Fact]
    public async Task SessionCreationRetriesShortSerializationConflicts()
    {
        var attempts = 0;
        var resets = 0;

        var result = await SimulationSessionRoute.RetrySerializationFailures(
            () =>
            {
                attempts++;
                return attempts < 3
                    ? Task.FromException<IResult>(SerializationFailure())
                    : Task.FromResult(Results.Ok() as IResult);
            },
            () => resets++,
            CancellationToken.None);

        Assert.Equal(StatusCodes.Status200OK, await StatusCode(result));
        Assert.Equal(3, attempts);
        Assert.Equal(2, resets);
    }

    [Fact]
    public async Task SessionCreationReturnsConflictAfterRepeatedSerializationFailures()
    {
        var attempts = 0;

        var result = await SimulationSessionRoute.RetrySerializationFailures(
            () =>
            {
                attempts++;
                return Task.FromException<IResult>(SerializationFailure());
            },
            () => { },
            CancellationToken.None);

        Assert.Equal(StatusCodes.Status409Conflict, await StatusCode(result));
        Assert.Equal(3, attempts);
    }

    [Fact]
    public async Task SessionCreationHonoursCancellationAfterASerializationFailure()
    {
        using var cancellation = new CancellationTokenSource();
        cancellation.Cancel();

        await Assert.ThrowsAsync<OperationCanceledException>(() =>
            SimulationSessionRoute.RetrySerializationFailures(
                () => Task.FromException<IResult>(SerializationFailure()),
                () => { },
                cancellation.Token));
    }

    [Fact]
    public async Task ViewerLeaseCreationRetriesShortSerializationConflicts()
    {
        var attempts = 0;
        var resets = 0;

        var result = await SessionControlRoute.RetryViewerLeaseSerializationFailures(
            () =>
            {
                attempts++;
                return attempts < 3
                    ? Task.FromException<IResult>(SerializationFailure())
                    : Task.FromResult(Results.Ok() as IResult);
            },
            () => resets++,
            CancellationToken.None);

        Assert.Equal(StatusCodes.Status200OK, await StatusCode(result));
        Assert.Equal(3, attempts);
        Assert.Equal(2, resets);
    }

    [Fact]
    public async Task ViewerLeaseCreationReturnsConflictAfterRepeatedSerializationFailures()
    {
        var attempts = 0;

        var result = await SessionControlRoute.RetryViewerLeaseSerializationFailures(
            () =>
            {
                attempts++;
                return Task.FromException<IResult>(SerializationFailure());
            },
            () => { },
            CancellationToken.None);

        Assert.Equal(StatusCodes.Status409Conflict, await StatusCode(result));
        Assert.Equal(3, attempts);
    }

    [Fact]
    public async Task ViewerLeaseCreationHonoursCancellationAfterASerializationFailure()
    {
        using var cancellation = new CancellationTokenSource();
        cancellation.Cancel();

        await Assert.ThrowsAsync<OperationCanceledException>(() =>
            SessionControlRoute.RetryViewerLeaseSerializationFailures(
                () => Task.FromException<IResult>(SerializationFailure()),
                () => { },
                cancellation.Token));
    }

    [Fact]
    public async Task TaskCreationRetriesShortSerializationConflicts()
    {
        var attempts = 0;
        var resets = 0;

        var result = await SessionControlRoute.RetryTaskSerializationFailures(
            () =>
            {
                attempts++;
                return attempts < 3
                    ? Task.FromException<IResult>(SerializationFailure())
                    : Task.FromResult(Results.Accepted() as IResult);
            },
            () => resets++,
            CancellationToken.None);

        Assert.Equal(StatusCodes.Status202Accepted, await StatusCode(result));
        Assert.Equal(3, attempts);
        Assert.Equal(2, resets);
    }

    [Fact]
    public async Task TaskCreationReturnsConflictAfterRepeatedSerializationFailures()
    {
        var attempts = 0;

        var result = await SessionControlRoute.RetryTaskSerializationFailures(
            () =>
            {
                attempts++;
                return Task.FromException<IResult>(SerializationFailure());
            },
            () => { },
            CancellationToken.None);

        Assert.Equal(StatusCodes.Status409Conflict, await StatusCode(result));
        Assert.Equal(3, attempts);
    }

    [Fact]
    public async Task TaskCreationHonoursCancellationAfterASerializationFailure()
    {
        using var cancellation = new CancellationTokenSource();
        cancellation.Cancel();

        await Assert.ThrowsAsync<OperationCanceledException>(() =>
            SessionControlRoute.RetryTaskSerializationFailures(
                () => Task.FromException<IResult>(SerializationFailure()),
                () => { },
                cancellation.Token));
    }

    [Fact]
    public void SerializationFailureIsFoundInsideDatabaseUpdateWrappers()
    {
        var wrapped = new InvalidOperationException(
            "The transaction failed while saving the viewer lease.",
            new DbUpdateException(
                "The database update failed.",
                SerializationFailure()));

        Assert.True(SimulationSessionRoute.IsSerializationFailure(wrapped));
    }

    [Fact]
    public void OtherDatabaseUpdateFailuresAreNotSerializationFailures()
    {
        var wrapped = new InvalidOperationException(
            "The transaction failed while saving the viewer lease.",
            new DbUpdateException(
                "The database update failed.",
                new PostgresException(
                    "unique violation",
                    "ERROR",
                    "ERROR",
                    PostgresErrorCodes.UniqueViolation)));

        Assert.False(SimulationSessionRoute.IsSerializationFailure(wrapped));
    }

    [Theory]
    [InlineData("999")]
    [InlineData("Queued")]
    [InlineData("")]
    public void WorkerRejectsUnsupportedTaskReportStates(string state)
    {
        Assert.False(WorkerHub.TryParseTaskReportState(state, out _));
    }

    [Theory]
    [InlineData("999")]
    [InlineData("Queued")]
    [InlineData("Expired")]
    [InlineData("")]
    public void WorkerRejectsUnsupportedSessionReportStates(string state)
    {
        Assert.False(WorkerHub.TryParseSessionReportState(state, out _));
    }

    [Fact]
    public async Task SessionCreationStopsAtTheGlobalQueueCap()
    {
        await using var dataContext = TestDataContext.Create();
        var queuedOwner = Account(1, "queued@example.test");
        var requester = Account(2, "requester@example.test");
        dataContext.AddRange(
            queuedOwner,
            requester,
            Session(queuedOwner, SimulationSessionState.Queued));
        await dataContext.SaveChangesAsync();

        var result = await SimulationSessionRoute.Create(
            new CreateSimulationSessionRequest(3),
            dataContext,
            Configuration(new Dictionary<string, string?>
            {
                ["Sessions:MaxQueuedSessions"] = "1"
            }),
            HttpContext(requester.Id),
            CancellationToken.None);

        Assert.Equal(StatusCodes.Status503ServiceUnavailable, await StatusCode(result));
        Assert.Equal(1, dataContext.SimulationSessions.Count());
    }

    [Fact]
    public async Task FailedAssignedSessionKeepsItsAccountSlotUntilCleanup()
    {
        await using var dataContext = TestDataContext.Create();
        var owner = Account(1, "owner@example.test");
        var worker = new ComputeWorker { Name = "worker" };
        var failed = Session(owner, SimulationSessionState.Failed, worker);
        dataContext.AddRange(owner, worker, failed);
        await dataContext.SaveChangesAsync();

        var result = await SimulationSessionRoute.Create(
            new CreateSimulationSessionRequest(1),
            dataContext,
            Configuration(),
            HttpContext(owner.Id),
            CancellationToken.None);

        Assert.Equal(StatusCodes.Status409Conflict, await StatusCode(result));

        var occupiesSlot = SimulationSessionRoute.OccupiesAccountSlot(owner.Id).Compile();
        Assert.True(occupiesSlot(failed));
        failed.ComputeWorkerId = null;
        Assert.False(occupiesSlot(failed));
    }

    [Theory]
    [InlineData(SimulationSessionState.Failed, true)]
    [InlineData(SimulationSessionState.Expired, true)]
    [InlineData(SimulationSessionState.Stopped, false)]
    [InlineData(SimulationSessionState.Ready, false)]
    public void FailedAndExpiredSessionsUseConfirmedWorkerCleanup(
        SimulationSessionState state,
        bool expected)
    {
        Assert.Equal(expected, SimulationSessionRoute.RequiresWorkerCleanup(state));
    }

    [Fact]
    public void QueueAndAssignedSessionExpiryReleaseResourcesAndRequestCleanup()
    {
        var now = new DateTime(2026, 7, 18, 12, 0, 0, DateTimeKind.Utc);
        var queued = new SimulationSession
        {
            State = SimulationSessionState.Queued,
            DesiredRobotCount = 1
        };

        SimulationSessionScheduler.ExpireQueuedSession(queued, now);

        Assert.Equal(SimulationSessionState.Expired, queued.State);
        Assert.Equal(now, queued.StoppedAt);

        var workerId = Guid.NewGuid();
        var assigned = new SimulationSession
        {
            ComputeWorkerId = workerId,
            State = SimulationSessionState.Active,
            DesiredRobotCount = 2,
            TaskRuns =
            {
                new TaskRun { State = TaskRunState.Running }
            },
            Robots =
            {
                new SessionRobot { State = SessionRobotState.Active }
            },
            ViewerLeases =
            {
                new ViewerLease { ExpiresAt = now.AddMinutes(5) }
            }
        };
        assigned.Commands.Add(Command(
            WorkerCommandType.StartTask,
            WorkerCommandState.Running,
            1,
            assigned));

        var stop = SimulationSessionScheduler.ExpireAssignedSession(assigned, now);

        Assert.Equal(SimulationSessionState.Expired, assigned.State);
        Assert.Equal(TaskRunState.Failed, Assert.Single(assigned.TaskRuns).State);
        Assert.Equal(SessionRobotState.Removed, Assert.Single(assigned.Robots).State);
        Assert.Equal(now, Assert.Single(assigned.ViewerLeases).RevokedAt);
        Assert.Equal(
            WorkerCommandState.Cancelled,
            assigned.Commands.Single(command => command.Type == WorkerCommandType.StartTask).State);
        Assert.NotNull(stop);
        Assert.Equal(WorkerCommandType.StopSession, stop.Type);
        Assert.Equal(WorkerCommandState.Pending, stop.State);
        Assert.Equal(workerId, stop.ComputeWorkerId);
    }

    [Fact]
    public async Task PendingEmergencyTransitionIsVisibleUntilItIsTerminal()
    {
        await using var dataContext = TestDataContext.Create();
        var owner = Account(1, "owner@example.test");
        var session = Session(owner, SimulationSessionState.Ready);
        var command = Command(
            WorkerCommandType.EmergencyStop,
            WorkerCommandState.Dispatched,
            sequence: 1,
            session: session);
        dataContext.AddRange(owner, session, command);
        await dataContext.SaveChangesAsync();

        Assert.True(await SessionControlRoute.HasPendingEmergencyTransition(
            dataContext,
            session.Id));

        command.State = WorkerCommandState.Completed;
        await dataContext.SaveChangesAsync();

        Assert.False(await SessionControlRoute.HasPendingEmergencyTransition(
            dataContext,
            session.Id));
    }

    [Fact]
    public void DisconnectFailSafeCancelsStaleCommandsButKeepsStopControls()
    {
        var now = DateTime.UtcNow;
        var session = new SimulationSession();
        session.Commands.Add(Command(
            WorkerCommandType.UpdateFleet,
            WorkerCommandState.Pending,
            1,
            session));
        session.Commands.Add(Command(
            WorkerCommandType.ResetEmergencyStop,
            WorkerCommandState.Running,
            2,
            session));
        session.Commands.Add(Command(
            WorkerCommandType.EmergencyStop,
            WorkerCommandState.Dispatched,
            3,
            session));
        session.Commands.Add(Command(
            WorkerCommandType.StopSession,
            WorkerCommandState.Acknowledged,
            4,
            session));
        session.Commands.Add(Command(
            WorkerCommandType.StartTask,
            WorkerCommandState.Completed,
            5,
            session));

        var cancelled = WorkerHub.CancelCommandsAfterFailSafe(session, now);

        Assert.Equal(2, cancelled.Count);
        Assert.All(cancelled, command =>
        {
            Assert.Equal(WorkerCommandState.Cancelled, command.State);
            Assert.Equal(now, command.CompletedAt);
        });
        Assert.Equal(
            WorkerCommandState.Dispatched,
            session.Commands.Single(command =>
                command.Type == WorkerCommandType.EmergencyStop).State);
        Assert.Equal(
            WorkerCommandState.Acknowledged,
            session.Commands.Single(command =>
                command.Type == WorkerCommandType.StopSession).State);
    }

    [Fact]
    public void HeartbeatQueuesCleanupForTerminalSessionsOnlyOnce()
    {
        var now = new DateTime(2026, 7, 19, 12, 0, 0, DateTimeKind.Utc);
        var session = new SimulationSession
        {
            ComputeWorkerId = Guid.NewGuid(),
            State = SimulationSessionState.Stopped
        };
        var completed = Command(
            WorkerCommandType.StopSession,
            WorkerCommandState.Completed,
            1,
            session);
        completed.CompletedAt = now - WorkerHub.TerminalCleanupBaseDelay;
        completed.UpdatedAt = completed.CompletedAt.Value;
        session.Commands.Add(completed);

        Assert.False(WorkerHub.NeedsTerminalCleanupAttempt(
            session,
            reportedByWorker: false));
        Assert.True(WorkerHub.NeedsTerminalCleanupAttempt(
            session,
            reportedByWorker: true));
        var first = WorkerHub.QueueTerminalCleanup(session, now);
        var second = WorkerHub.QueueTerminalCleanup(session, now.AddSeconds(1));

        Assert.NotNull(first);
        Assert.Equal(2, first.Sequence);
        Assert.Equal(WorkerCommandState.Pending, first.State);
        Assert.Null(second);
    }

    [Fact]
    public void TerminalCleanupRetriesUseBackoffAndStopAtTheAttemptLimit()
    {
        var now = new DateTime(2026, 7, 19, 12, 0, 0, DateTimeKind.Utc);
        var session = new SimulationSession
        {
            ComputeWorkerId = Guid.NewGuid(),
            State = SimulationSessionState.Expired
        };
        var failed = Command(
            WorkerCommandType.StopSession,
            WorkerCommandState.Failed,
            1,
            session);
        failed.CompletedAt = now;
        failed.UpdatedAt = now;
        session.Commands.Add(failed);

        Assert.Null(WorkerHub.QueueTerminalCleanup(
            session,
            now + WorkerHub.TerminalCleanupBaseDelay - TimeSpan.FromMilliseconds(1)));
        Assert.NotNull(WorkerHub.QueueTerminalCleanup(
            session,
            now + WorkerHub.TerminalCleanupBaseDelay));

        while (session.Commands.Count(command =>
                   command.Type == WorkerCommandType.StopSession)
               < WorkerHub.MaximumTerminalCleanupAttempts)
        {
            var latest = session.Commands
                .Where(command => command.Type == WorkerCommandType.StopSession)
                .OrderByDescending(command => command.Sequence)
                .First();
            latest.State = WorkerCommandState.Failed;
            latest.CompletedAt = now.AddDays(-1);
            latest.UpdatedAt = latest.CompletedAt.Value;
            Assert.NotNull(WorkerHub.QueueTerminalCleanup(session, now));
        }

        var last = session.Commands
            .Where(command => command.Type == WorkerCommandType.StopSession)
            .OrderByDescending(command => command.Sequence)
            .First();
        last.State = WorkerCommandState.Failed;
        last.CompletedAt = now.AddDays(-1);
        last.UpdatedAt = last.CompletedAt.Value;
        Assert.Null(WorkerHub.QueueTerminalCleanup(session, now));
    }

    [Fact]
    public void CleanupAttemptCapIsSharedAcrossEveryEnqueueSource()
    {
        var now = new DateTime(2026, 7, 19, 12, 0, 0, DateTimeKind.Utc);
        var session = new SimulationSession
        {
            ComputeWorkerId = Guid.NewGuid(),
            State = SimulationSessionState.Stopping
        };
        var prefixes = new[]
        {
            "sys:session-delete",
            "sys:account-disabled",
            "sys:session-expired",
            "sys:orphan-cleanup",
            "sys:heartbeat-cleanup"
        };

        foreach (var prefix in prefixes)
        {
            var command = TerminalCleanupPolicy.TryQueue(
                session,
                now,
                prefix,
                resourceKnownPresent: true);
            Assert.NotNull(command);
            command.State = WorkerCommandState.Failed;
            command.CompletedAt = now.AddDays(-1);
            command.UpdatedAt = command.CompletedAt.Value;
        }

        Assert.Equal(
            TerminalCleanupPolicy.MaximumAttempts,
            session.Commands.Count(command =>
                command.Type == WorkerCommandType.StopSession));
        Assert.Null(TerminalCleanupPolicy.TryQueue(
            session,
            now,
            "sys:session-delete",
            resourceKnownPresent: true));
    }

    [Fact]
    public void AbsentContainerDoesNotSuppressRetryAfterNetworkCleanupFailure()
    {
        var now = new DateTime(2026, 7, 19, 12, 0, 0, DateTimeKind.Utc);
        var session = new SimulationSession
        {
            ComputeWorkerId = Guid.NewGuid(),
            State = SimulationSessionState.Expired,
            FailureReason = "Session expired."
        };
        var failedCleanup = Command(
            WorkerCommandType.StopSession,
            WorkerCommandState.Failed,
            1,
            session);
        failedCleanup.LastError = "Container removed, but network cleanup failed.";
        failedCleanup.CompletedAt = now - WorkerHub.TerminalCleanupBaseDelay;
        failedCleanup.UpdatedAt = failedCleanup.CompletedAt.Value;
        session.Commands.Add(failedCleanup);

        Assert.True(WorkerHub.ReconcileAbsentTerminalSession(session, now));
        Assert.Equal(SimulationSessionState.Stopped, session.State);
        Assert.True(WorkerHub.NeedsTerminalCleanupAttempt(
            session,
            reportedByWorker: false));

        var retry = WorkerHub.QueueTerminalCleanup(session, now);

        Assert.NotNull(retry);
        Assert.Equal(2, retry.Sequence);
        Assert.Equal(WorkerCommandState.Pending, retry.State);
        Assert.Equal(session.ComputeWorkerId, retry.ComputeWorkerId);
    }

    [Theory]
    [InlineData(SimulationSessionState.Failed)]
    [InlineData(SimulationSessionState.Expired)]
    public void MissingTerminalSessionIsReconciledWithoutLosingItsReason(
        SimulationSessionState state)
    {
        var now = new DateTime(2026, 7, 19, 12, 0, 0, DateTimeKind.Utc);
        var session = new SimulationSession
        {
            ComputeWorkerId = Guid.NewGuid(),
            State = state,
            FailureReason = "Original terminal reason."
        };

        Assert.True(WorkerHub.ReconcileAbsentTerminalSession(session, now));
        Assert.Equal(SimulationSessionState.Stopped, session.State);
        Assert.Equal("Original terminal reason.", session.FailureReason);
        Assert.Equal(now, session.StoppedAt);
        Assert.Equal(1, session.Revision);
    }

    [Fact]
    public void ReplayedFailSafeCancelsMutationThatArrivedAfterTheFirstReport()
    {
        var firstReportAt = new DateTime(2026, 7, 19, 12, 0, 0, DateTimeKind.Utc);
        var session = new SimulationSession
        {
            State = SimulationSessionState.Ready
        };

        Assert.Empty(WorkerHub.ApplyFailSafeTransition(session, firstReportAt));
        Assert.True(session.IsEmergencyStopped);

        var lateMutation = Command(
            WorkerCommandType.StartTask,
            WorkerCommandState.Pending,
            sequence: 1,
            session);
        session.Commands.Add(lateMutation);

        var cancelled = WorkerHub.ApplyFailSafeTransition(
            session,
            firstReportAt.AddSeconds(1));

        Assert.Equal(lateMutation, Assert.Single(cancelled));
        Assert.Equal(WorkerCommandState.Cancelled, lateMutation.State);
        Assert.Equal(firstReportAt.AddSeconds(1), lateMutation.CompletedAt);
    }

    [Fact]
    public void ExactTaskReportsAreNoOpsButChangedProgressIsNot()
    {
        var now = DateTime.UtcNow;
        var session = new SimulationSession
        {
            State = SimulationSessionState.Active
        };
        var task = new TaskRun
        {
            Id = Guid.NewGuid(),
            SimulationSessionId = session.Id,
            SimulationSession = session,
            State = TaskRunState.Running,
            Progress = 0.4,
            Result = JsonDocument.Parse("""{"status":"moving"}"""),
            StartedAt = now,
            UpdatedAt = now
        };
        using var duplicateResult = JsonDocument.Parse(
            """{"status":"moving"}""");
        var duplicate = new TaskEventReport(
            session.Id,
            task.Id,
            "Running",
            0.4,
            duplicateResult.RootElement,
            Error: null);
        var changed = duplicate with { Progress = 0.5 };

        Assert.True(WorkerHub.IsDuplicateTaskReport(
            task,
            TaskRunState.Running,
            duplicate));
        Assert.False(WorkerHub.IsDuplicateTaskReport(
            task,
            TaskRunState.Running,
            changed));
    }

    [Fact]
    public void RepeatedStoppedReportKeepsTheOriginalFailureReason()
    {
        var session = new SimulationSession
        {
            State = SimulationSessionState.Stopped,
            FailureReason = "Gazebo stopped unexpectedly."
        };

        var failureReason = WorkerHub.FailureReasonForSessionEvent(
            session,
            SimulationSessionState.Stopped,
            reportedReason: null);

        Assert.Equal(session.FailureReason, failureReason);
    }

    [Fact]
    public async Task DisablingAnAccountRetriesCleanupForAnAssignedExpiredSession()
    {
        await using var dataContext = TestDataContext.Create();
        var owner = Account(1, "expired-owner@example.test");
        var worker = new ComputeWorker { Name = "worker" };
        var session = Session(owner, SimulationSessionState.Expired, worker);
        var failedCleanup = Command(
            WorkerCommandType.StopSession,
            WorkerCommandState.Failed,
            sequence: 1,
            session);
        failedCleanup.CompletedAt = DateTime.UtcNow - TimeSpan.FromMinutes(1);
        failedCleanup.UpdatedAt = failedCleanup.CompletedAt.Value;
        session.Commands.Add(failedCleanup);
        dataContext.AddRange(owner, worker, session);
        await dataContext.SaveChangesAsync();
        dataContext.ChangeTracker.Clear();

        var service = new AccountService(dataContext, Configuration());

        Assert.True(await service.Delete(owner.Id));
        Assert.False((await dataContext.Accounts.SingleAsync()).Enabled);
        var cleanup = await dataContext.WorkerCommands.SingleAsync(command =>
            command.Type == WorkerCommandType.StopSession
            && command.State == WorkerCommandState.Pending);
        Assert.Equal(2, cleanup.Sequence);
        Assert.Equal(worker.Id, cleanup.ComputeWorkerId);
    }

    [Fact]
    public async Task RepeatedAccountDisableCannotExceedTheCleanupAttemptLimit()
    {
        await using var dataContext = TestDataContext.Create();
        var owner = Account(1, "retry-owner@example.test");
        var worker = new ComputeWorker { Name = "worker" };
        var session = Session(owner, SimulationSessionState.Expired, worker);
        dataContext.AddRange(owner, worker, session);
        await dataContext.SaveChangesAsync();
        dataContext.ChangeTracker.Clear();

        var service = new AccountService(dataContext, Configuration());
        for (var attempt = 1;
             attempt <= TerminalCleanupPolicy.MaximumAttempts;
             attempt++)
        {
            Assert.True(await service.Delete(owner.Id));
            var cleanup = await dataContext.WorkerCommands.SingleAsync(command =>
                command.Type == WorkerCommandType.StopSession
                && command.State == WorkerCommandState.Pending);
            Assert.Equal(attempt, cleanup.Sequence);
            cleanup.State = WorkerCommandState.Failed;
            cleanup.CompletedAt = DateTime.UtcNow - TimeSpan.FromDays(1);
            cleanup.UpdatedAt = cleanup.CompletedAt.Value;
            await dataContext.SaveChangesAsync();
            dataContext.ChangeTracker.Clear();
        }

        Assert.True(await service.Delete(owner.Id));
        Assert.Equal(
            TerminalCleanupPolicy.MaximumAttempts,
            await dataContext.WorkerCommands.CountAsync(command =>
                command.Type == WorkerCommandType.StopSession));
        Assert.DoesNotContain(
            await dataContext.WorkerCommands.ToListAsync(),
            command => command.State == WorkerCommandState.Pending);
    }

    private static Account Account(int id, string email)
    {
        return new Account
        {
            Id = id,
            FirstName = "Session",
            LastName = "Owner",
            Email = email,
            Enabled = true
        };
    }

    private static PostgresException SerializationFailure()
    {
        return new PostgresException(
            "concurrent session creation",
            "ERROR",
            "ERROR",
            PostgresErrorCodes.SerializationFailure);
    }

    private static SimulationSession Session(
        Account owner,
        SimulationSessionState state,
        ComputeWorker? worker = null)
    {
        return new SimulationSession
        {
            AccountId = owner.Id,
            Account = owner,
            ComputeWorkerId = worker?.Id,
            ComputeWorker = worker,
            State = state,
            DesiredRobotCount = 1
        };
    }

    private static WorkerCommand Command(
        WorkerCommandType type,
        WorkerCommandState state,
        long sequence,
        SimulationSession session)
    {
        return new WorkerCommand
        {
            SimulationSessionId = session.Id,
            SimulationSession = session,
            ComputeWorkerId = session.ComputeWorkerId,
            Type = type,
            State = state,
            IdempotencyKey = $"test:{sequence}",
            Sequence = sequence
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

    private static IConfiguration Configuration(
        Dictionary<string, string?>? values = null)
    {
        return new ConfigurationBuilder()
            .AddInMemoryCollection(values ?? new Dictionary<string, string?>())
            .Build();
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
