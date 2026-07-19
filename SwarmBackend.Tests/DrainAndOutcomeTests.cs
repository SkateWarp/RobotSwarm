using System.Security.Claims;
using System.Text.Json;
using Microsoft.AspNetCore.Http;
using Microsoft.EntityFrameworkCore;
using Microsoft.EntityFrameworkCore.Storage;
using SwarmBackend.Entities;
using SwarmBackend.Helpers;
using SwarmBackend.Models;
using SwarmBackend.Routes;
using SwarmBackend.Services;

namespace SwarmBackend.Tests;

public sealed class DrainAndOutcomeTests
{
    [Fact]
    public void HeartbeatKeepsAnActiveDrainAndClearsAnExpiredOne()
    {
        var now = new DateTime(2026, 7, 19, 12, 0, 0, DateTimeKind.Utc);
        var worker = DrainingWorker(now.AddMinutes(10));

        WorkerDrainLease.ApplyHeartbeatState(worker, now);

        Assert.Equal(ComputeWorkerState.Draining, worker.State);
        Assert.NotNull(worker.DrainLeaseId);

        WorkerDrainLease.ApplyHeartbeatState(worker, now.AddMinutes(11));

        Assert.Equal(ComputeWorkerState.Online, worker.State);
        Assert.Null(worker.DrainLeaseId);
        Assert.Null(worker.DrainTargetRevision);
    }

    [Fact]
    public async Task DrainNeedsAZeroContainerReportAfterSchedulingStops()
    {
        await using var dataContext = TestDataContext.Create();
        var worker = new ComputeWorker
        {
            Name = "gpu-worker",
            State = ComputeWorkerState.Online,
            ReportedActiveSessionCount = 0,
            ActiveSessionsReportedAt = DateTime.UtcNow.AddMinutes(-1)
        };
        dataContext.Add(worker);
        await dataContext.SaveChangesAsync();

        var result = await WorkerMaintenanceRoute.AcquireDrain(
            new AcquireWorkerDrainRequest(new string('a', 40)),
            dataContext,
            WorkerContext(worker.Id),
            CancellationToken.None);

        Assert.IsAssignableFrom<IResult>(result);
        Assert.Equal(ComputeWorkerState.Draining, worker.State);
        Assert.NotNull(worker.DrainLeaseId);
        var waiting = await WorkerMaintenanceRoute.BuildStatus(dataContext, worker);
        Assert.False(waiting.IsDrained);

        worker.ReportedActiveSessionCount = 0;
        worker.ActiveSessionsReportedAt = worker.DrainRequestedAt;
        await dataContext.SaveChangesAsync();

        var drained = await WorkerMaintenanceRoute.BuildStatus(dataContext, worker);
        Assert.True(drained.IsDrained);
        Assert.Equal(0, drained.TrackedSessionCount);
        Assert.Equal(0, drained.ReportedActiveSessionCount);
    }

    [Fact]
    public async Task StaleHeartbeatCannotOverwriteANewDrainLease()
    {
        var databaseName = Guid.NewGuid().ToString("N");
        var databaseRoot = new InMemoryDatabaseRoot();
        var workerId = Guid.NewGuid();

        await using (var seed = TestDataContext.Create(databaseName, databaseRoot))
        {
            seed.ComputeWorkers.Add(new ComputeWorker
            {
                Id = workerId,
                Name = "gpu-worker",
                State = ComputeWorkerState.Offline
            });
            await seed.SaveChangesAsync();
        }

        await using var staleHeartbeat = TestDataContext.Create(
            databaseName,
            databaseRoot);
        var staleWorker = await staleHeartbeat.ComputeWorkers
            .SingleAsync(worker => worker.Id == workerId);

        await using (var drain = TestDataContext.Create(databaseName, databaseRoot))
        {
            await WorkerMaintenanceRoute.AcquireDrain(
                new AcquireWorkerDrainRequest(new string('a', 40)),
                drain,
                WorkerContext(workerId),
                CancellationToken.None);
        }

        var heartbeatAt = DateTime.UtcNow;
        WorkerDrainLease.ApplyHeartbeatState(staleWorker, heartbeatAt);
        staleWorker.LastHeartbeatAt = heartbeatAt;
        staleWorker.UpdatedAt = heartbeatAt;
        await Assert.ThrowsAsync<DbUpdateConcurrencyException>(
            () => staleHeartbeat.SaveChangesAsync());

        staleHeartbeat.ChangeTracker.Clear();
        var reloadedWorker = await staleHeartbeat.ComputeWorkers
            .SingleAsync(worker => worker.Id == workerId);
        WorkerDrainLease.ApplyHeartbeatState(reloadedWorker, heartbeatAt);
        await staleHeartbeat.SaveChangesAsync();

        Assert.Equal(ComputeWorkerState.Draining, reloadedWorker.State);
        Assert.NotNull(reloadedWorker.DrainLeaseId);
    }

    [Fact]
    public void RelationalWorkerAvailabilityQueryGuardsTheDrainLease()
    {
        var options = new DbContextOptionsBuilder<DataContext>()
            .UseNpgsql(
                "Host=127.0.0.1;Database=translation_only;Username=test;Password=test")
            .Options;
        using var dataContext = new DataContext(options);
        var now = DateTime.UtcNow;

        var sql = SimulationSessionScheduler.AvailableWorkerQuery(
                dataContext,
                now,
                now.AddSeconds(-30))
            .ToQueryString();

        Assert.Contains("\"DrainLeaseId\" IS NULL", sql, StringComparison.Ordinal);
        Assert.Contains("\"DrainLeaseExpiresAt\" <=", sql, StringComparison.Ordinal);
    }

    [Fact]
    public async Task TrackedSessionKeepsDrainOpenEvenAfterAZeroReport()
    {
        await using var dataContext = TestDataContext.Create();
        var now = DateTime.UtcNow;
        var worker = DrainingWorker(now.AddMinutes(10));
        worker.DrainRequestedAt = now;
        worker.ReportedActiveSessionCount = 0;
        worker.ActiveSessionsReportedAt = now;
        var owner = new Account
        {
            Id = 1,
            FirstName = "Drain",
            LastName = "Test",
            Email = "drain@example.test",
            Enabled = true
        };
        var session = new SimulationSession
        {
            Account = owner,
            AccountId = owner.Id,
            ComputeWorker = worker,
            ComputeWorkerId = worker.Id,
            State = SimulationSessionState.Ready,
            DesiredRobotCount = 3
        };
        dataContext.AddRange(owner, worker, session);
        await dataContext.SaveChangesAsync();

        var status = await WorkerMaintenanceRoute.BuildStatus(dataContext, worker);

        Assert.False(status.IsDrained);
        Assert.Equal(1, status.TrackedSessionCount);
    }

    [Theory]
    [InlineData(SimulationSessionState.Stopped)]
    [InlineData(SimulationSessionState.Failed)]
    [InlineData(SimulationSessionState.Expired)]
    public async Task HistoricalTerminalSessionsDoNotBlockDrain(
        SimulationSessionState state)
    {
        await using var dataContext = TestDataContext.Create();
        var now = DateTime.UtcNow;
        var worker = DrainingWorker(now.AddMinutes(10));
        worker.DrainRequestedAt = now;
        worker.ReportedActiveSessionCount = 0;
        worker.ActiveSessionsReportedAt = now;
        var owner = Owner();
        var session = Session(owner, worker, state);
        dataContext.AddRange(owner, worker, session);
        await dataContext.SaveChangesAsync();

        var status = await WorkerMaintenanceRoute.BuildStatus(dataContext, worker);

        Assert.True(status.IsDrained);
        Assert.Equal(0, status.TrackedSessionCount);
    }

    [Fact]
    public async Task InFlightTerminalCleanupKeepsDrainOpen()
    {
        await using var dataContext = TestDataContext.Create();
        var now = DateTime.UtcNow;
        var worker = DrainingWorker(now.AddMinutes(10));
        worker.DrainRequestedAt = now;
        worker.ReportedActiveSessionCount = 0;
        worker.ActiveSessionsReportedAt = now;
        var owner = Owner();
        var session = Session(owner, worker, SimulationSessionState.Failed);
        session.Commands.Add(new WorkerCommand
        {
            SimulationSession = session,
            SimulationSessionId = session.Id,
            ComputeWorkerId = worker.Id,
            Type = WorkerCommandType.StopSession,
            State = WorkerCommandState.Running,
            IdempotencyKey = "sys:test-cleanup",
            Sequence = 1
        });
        dataContext.AddRange(owner, worker, session);
        await dataContext.SaveChangesAsync();

        var status = await WorkerMaintenanceRoute.BuildStatus(dataContext, worker);

        Assert.False(status.IsDrained);
        Assert.Equal(1, status.TrackedSessionCount);
    }

    [Fact]
    public async Task ActiveLeaseCannotBeReusedForAnotherRevision()
    {
        await using var dataContext = TestDataContext.Create();
        var worker = DrainingWorker(DateTime.UtcNow.AddMinutes(10));
        var originalLeaseId = worker.DrainLeaseId;
        dataContext.Add(worker);
        await dataContext.SaveChangesAsync();

        var result = await WorkerMaintenanceRoute.AcquireDrain(
            new AcquireWorkerDrainRequest(new string('b', 40)),
            dataContext,
            WorkerContext(worker.Id),
            CancellationToken.None);

        var conflict = Assert.IsAssignableFrom<IStatusCodeHttpResult>(result);
        Assert.Equal(StatusCodes.Status409Conflict, conflict.StatusCode);
        Assert.Equal(originalLeaseId, worker.DrainLeaseId);
        Assert.Equal(new string('a', 40), worker.DrainTargetRevision);
    }

    [Fact]
    public async Task ExpiredLeaseStartsASeparateDrainObservationWindow()
    {
        await using var dataContext = TestDataContext.Create();
        var worker = DrainingWorker(DateTime.UtcNow.AddMinutes(-1));
        var expiredLeaseId = worker.DrainLeaseId;
        var oldRequestTime = worker.DrainRequestedAt;
        dataContext.Add(worker);
        await dataContext.SaveChangesAsync();

        await WorkerMaintenanceRoute.AcquireDrain(
            new AcquireWorkerDrainRequest(new string('b', 40)),
            dataContext,
            WorkerContext(worker.Id),
            CancellationToken.None);

        Assert.NotEqual(expiredLeaseId, worker.DrainLeaseId);
        Assert.True(worker.DrainRequestedAt > oldRequestTime);
        Assert.Equal(new string('b', 40), worker.DrainTargetRevision);
    }

    [Fact]
    public void TerminalAcceptanceRequiresProgressAndTransportEvidence()
    {
        var follow = new TaskRun
        {
            Id = Guid.NewGuid(),
            Type = SwarmTaskRunType.FollowLeader
        };
        Assert.True(TaskAcceptancePolicy.TryAccept(
            follow,
            1,
            null,
            requireCollaborativeTransportEvidence: true,
            out _));
        Assert.False(TaskAcceptancePolicy.TryAccept(
            follow,
            0.8,
            null,
            requireCollaborativeTransportEvidence: true,
            out _));

        var transport = new TaskRun
        {
            Id = Guid.NewGuid(),
            Type = SwarmTaskRunType.CollaborativeTransport,
            SimulationSession = new SimulationSession
            {
                DesiredRobotCount = 3,
                Robots =
                {
                    new SessionRobot { RuntimeId = "tb3_0" },
                    new SessionRobot { RuntimeId = "tb3_1" },
                    new SessionRobot { RuntimeId = "tb3_2" }
                }
            }
        };
        Assert.True(TaskAcceptancePolicy.TryAccept(
            transport,
            1,
            null,
            requireCollaborativeTransportEvidence: false,
            out _));
        using var acceptedResult = JsonDocument.Parse($$"""
            {
              "transport": {
                "phase": "DONE",
                "all_pushers_confirmed": true,
                "useful_contributor_count": 3,
                "useful_contributor_ids": ["tb3_0", "tb3_1", "tb3_2"],
                "discovery": {
                  "event": "payload_found",
                  "task_id": "{{transport.Id}}",
                  "announced": true,
                  "finder": "tb3_0",
                  "notified_robots": ["tb3_1", "tb3_2"]
                }
              }
            }
            """);

        Assert.True(TaskAcceptancePolicy.TryAccept(
            transport,
            1,
            acceptedResult.RootElement,
            requireCollaborativeTransportEvidence: true,
            out _));
        using var incompleteResult = JsonDocument.Parse(
            """{"transport":{"phase":"DONE"}}""");
        Assert.False(TaskAcceptancePolicy.TryAccept(
            transport,
            1,
            incompleteResult.RootElement,
            requireCollaborativeTransportEvidence: true,
            out var reason));
        Assert.NotNull(reason);
        Assert.Contains("discovery", reason!, StringComparison.OrdinalIgnoreCase);

        using var missingPusherResult = JsonDocument.Parse($$"""
            {
              "transport": {
                "phase": "DONE",
                "all_pushers_confirmed": false,
                "useful_contributor_count": 3,
                "useful_contributor_ids": ["tb3_0", "tb3_1", "tb3_2"],
                "discovery": {
                  "event": "payload_found",
                  "task_id": "{{transport.Id}}",
                  "announced": true,
                  "finder": "tb3_0",
                  "notified_robots": ["tb3_1", "tb3_2"]
                }
              }
            }
            """);
        Assert.False(TaskAcceptancePolicy.TryAccept(
            transport,
            1,
            missingPusherResult.RootElement,
            requireCollaborativeTransportEvidence: true,
            out reason));
        Assert.Contains("every robot", reason!, StringComparison.OrdinalIgnoreCase);

        using var tooFewContributorsResult = JsonDocument.Parse($$"""
            {
              "transport": {
                "phase": "DONE",
                "all_pushers_confirmed": true,
                "useful_contributor_count": 2,
                "useful_contributor_ids": ["tb3_0", "tb3_1"],
                "discovery": {
                  "event": "payload_found",
                  "task_id": "{{transport.Id}}",
                  "announced": true,
                  "finder": "tb3_0",
                  "notified_robots": ["tb3_1", "tb3_2"]
                }
              }
            }
            """);
        Assert.False(TaskAcceptancePolicy.TryAccept(
            transport,
            1,
            tooFewContributorsResult.RootElement,
            requireCollaborativeTransportEvidence: true,
            out reason));
        Assert.Contains("every robot", reason!, StringComparison.OrdinalIgnoreCase);
    }

    [Fact]
    public void TransportAcceptanceRejectsInventedRosterIdsAndImpossibleCounts()
    {
        var task = new TaskRun
        {
            Id = Guid.NewGuid(),
            Type = SwarmTaskRunType.CollaborativeTransport,
            SimulationSession = new SimulationSession
            {
                DesiredRobotCount = 3,
                Robots =
                {
                    new SessionRobot { RuntimeId = "tb3_0" },
                    new SessionRobot { RuntimeId = "tb3_1" },
                    new SessionRobot { RuntimeId = "tb3_2" }
                }
            }
        };

        using var inventedFinder = TransportResult(
            task.Id,
            finder: "tb3_99",
            notified: new[] { "tb3_0", "tb3_1" },
            contributorCount: 3,
            contributors: new[] { "tb3_0", "tb3_1", "tb3_2" });
        Assert.False(TaskAcceptancePolicy.TryAccept(
            task,
            1,
            inventedFinder.RootElement,
            requireCollaborativeTransportEvidence: true,
            out var reason));
        Assert.Contains("finder", reason!, StringComparison.OrdinalIgnoreCase);

        using var inventedRecipient = TransportResult(
            task.Id,
            finder: "tb3_0",
            notified: new[] { "tb3_1", "tb3_99" },
            contributorCount: 3,
            contributors: new[] { "tb3_0", "tb3_1", "tb3_2" });
        Assert.False(TaskAcceptancePolicy.TryAccept(
            task,
            1,
            inventedRecipient.RootElement,
            requireCollaborativeTransportEvidence: true,
            out reason));
        Assert.Contains("companion", reason!, StringComparison.OrdinalIgnoreCase);

        using var excessiveCount = TransportResult(
            task.Id,
            finder: "tb3_0",
            notified: new[] { "tb3_1", "tb3_2" },
            contributorCount: 4,
            contributors: new[] { "tb3_0", "tb3_1", "tb3_2", "tb3_99" });
        Assert.False(TaskAcceptancePolicy.TryAccept(
            task,
            1,
            excessiveCount.RootElement,
            requireCollaborativeTransportEvidence: true,
            out reason));
        Assert.Contains("every robot", reason!, StringComparison.OrdinalIgnoreCase);

        using var inventedContributor = TransportResult(
            task.Id,
            finder: "tb3_0",
            notified: new[] { "tb3_1", "tb3_2" },
            contributorCount: 3,
            contributors: new[] { "tb3_0", "tb3_1", "tb3_99" });
        Assert.False(TaskAcceptancePolicy.TryAccept(
            task,
            1,
            inventedContributor.RootElement,
            requireCollaborativeTransportEvidence: true,
            out reason));
        Assert.Contains("roster", reason!, StringComparison.OrdinalIgnoreCase);
    }

    [Fact]
    public void BackendFirstRolloutCanSwitchFromLegacyToStrictTransportEvidence()
    {
        var task = new TaskRun
        {
            Id = Guid.NewGuid(),
            Type = SwarmTaskRunType.CollaborativeTransport,
            SimulationSession = new SimulationSession
            {
                DesiredRobotCount = 1,
                Robots =
                {
                    new SessionRobot { RuntimeId = "tb3_0" }
                }
            }
        };

        Assert.True(TaskAcceptancePolicy.TryAccept(
            task,
            1,
            result: null,
            requireCollaborativeTransportEvidence: false,
            out _));
        Assert.False(TaskAcceptancePolicy.TryAccept(
            task,
            1,
            result: null,
            requireCollaborativeTransportEvidence: true,
            out var reason));
        Assert.Contains("evidence", reason!, StringComparison.OrdinalIgnoreCase);
    }

    [Fact]
    public void TransportAcceptanceRejectsARosterLargerThanTheRequestedFleet()
    {
        var task = new TaskRun
        {
            Id = Guid.NewGuid(),
            Type = SwarmTaskRunType.CollaborativeTransport,
            SimulationSession = new SimulationSession
            {
                DesiredRobotCount = 2,
                Robots =
                {
                    new SessionRobot { RuntimeId = "tb3_0" },
                    new SessionRobot { RuntimeId = "tb3_1" },
                    new SessionRobot { RuntimeId = "tb3_2" }
                }
            }
        };
        using var result = TransportResult(
            task.Id,
            finder: "tb3_0",
            notified: new[] { "tb3_1", "tb3_2" },
            contributorCount: 3,
            contributors: new[] { "tb3_0", "tb3_1", "tb3_2" });

        Assert.False(TaskAcceptancePolicy.TryAccept(
            task,
            1,
            result.RootElement,
            requireCollaborativeTransportEvidence: true,
            out var reason));
        Assert.Contains("roster", reason!, StringComparison.OrdinalIgnoreCase);
    }

    [Fact]
    public void ProgressTimeoutDoesNotUseWorkerOrRecordHeartbeatTime()
    {
        var now = new DateTime(2026, 7, 19, 12, 0, 0, DateTimeKind.Utc);
        var task = new TaskRun
        {
            Type = SwarmTaskRunType.Figure,
            State = TaskRunState.Running,
            LastProgressAt = now.AddMinutes(-6),
            LastReportAt = now,
            UpdatedAt = now
        };

        var reason = TaskOutcomeMonitor.FindTimeoutReason(
            task,
            now,
            TimeSpan.FromMinutes(2),
            TimeSpan.FromSeconds(90),
            TimeSpan.FromMinutes(5));

        Assert.NotNull(reason);
        task.State = TaskRunState.Paused;
        Assert.Null(TaskOutcomeMonitor.FindTimeoutReason(
            task,
            now,
            TimeSpan.FromMinutes(2),
            TimeSpan.FromSeconds(90),
            TimeSpan.FromMinutes(5)));
    }

    [Fact]
    public void RepeatedAcceptedReportsCannotPostponeTheStartTimeout()
    {
        var now = new DateTime(2026, 7, 19, 12, 0, 0, DateTimeKind.Utc);
        var task = new TaskRun
        {
            State = TaskRunState.Accepted,
            LastProgressAt = now.AddMinutes(-2),
            LastReportAt = now,
            UpdatedAt = now
        };

        var reason = TaskOutcomeMonitor.FindTimeoutReason(
            task,
            now,
            TimeSpan.FromMinutes(2),
            TimeSpan.FromSeconds(90),
            TimeSpan.FromMinutes(5));

        Assert.NotNull(reason);
        Assert.Contains("did not begin", reason, StringComparison.OrdinalIgnoreCase);
    }

    [Fact]
    public void ContinuousFollowLeaderDoesNotUseACompletionProgressTimeout()
    {
        var now = DateTime.UtcNow;
        var task = new TaskRun
        {
            Type = SwarmTaskRunType.FollowLeader,
            State = TaskRunState.Running,
            LastProgressAt = now.AddHours(-1),
            UpdatedAt = now
        };

        var reason = TaskOutcomeMonitor.FindTimeoutReason(
            task,
            now,
            TimeSpan.FromMinutes(2),
            TimeSpan.FromSeconds(90),
            TimeSpan.FromMinutes(5));

        Assert.Null(reason);
    }

    [Fact]
    public void QueuedTimeoutCancelsTheStartBeforeItCanBePulled()
    {
        var now = DateTime.UtcNow;
        var worker = new ComputeWorker { Name = "gpu-worker" };
        var session = new SimulationSession
        {
            ComputeWorker = worker,
            ComputeWorkerId = worker.Id,
            State = SimulationSessionState.Ready,
            DesiredRobotCount = 3
        };
        var task = new TaskRun
        {
            Id = Guid.NewGuid(),
            SimulationSession = session,
            SimulationSessionId = session.Id,
            State = TaskRunState.Queued,
            CreatedAt = now.AddMinutes(-3),
            UpdatedAt = now
        };
        var start = new WorkerCommand
        {
            SimulationSession = session,
            SimulationSessionId = session.Id,
            TaskRun = task,
            TaskRunId = task.Id,
            Type = WorkerCommandType.StartTask,
            State = WorkerCommandState.Pending,
            IdempotencyKey = "test:start",
            Sequence = 1
        };
        task.Commands.Add(start);
        session.Commands.Add(start);

        var reason = TaskOutcomeMonitor.FindTimeoutReason(
            task,
            now,
            TimeSpan.FromMinutes(2),
            TimeSpan.FromSeconds(90),
            TimeSpan.FromMinutes(5));
        Assert.NotNull(reason);

        var cancel = TaskOutcomeMonitor.FailStalledTask(task, reason, now);

        Assert.Null(cancel);
        Assert.Equal(WorkerCommandState.Cancelled, start.State);
        Assert.Equal(TaskRunState.Failed, task.State);
        Assert.Equal(TaskOutcomeState.Failed, task.OutcomeState);
    }

    [Fact]
    public void StalledTaskFailsAndQueuesOneCorrelatedStop()
    {
        var now = DateTime.UtcNow;
        var worker = new ComputeWorker { Name = "gpu-worker" };
        var session = new SimulationSession
        {
            ComputeWorker = worker,
            ComputeWorkerId = worker.Id,
            State = SimulationSessionState.Active,
            DesiredRobotCount = 4,
            Revision = 7
        };
        var task = new TaskRun
        {
            Id = Guid.NewGuid(),
            SimulationSession = session,
            SimulationSessionId = session.Id,
            State = TaskRunState.Running
        };

        var command = TaskOutcomeMonitor.FailStalledTask(
            task,
            "No measurable progress.",
            now);

        Assert.NotNull(command);
        Assert.Equal(TaskRunState.Failed, task.State);
        Assert.Equal(TaskOutcomeState.Failed, task.OutcomeState);
        Assert.Equal(SimulationSessionState.Active, session.State);
        Assert.Equal(7, session.Revision);
        Assert.Equal(WorkerCommandType.CancelTask, command.Type);
        Assert.Equal(task.Id, command.TaskRunId);
        Assert.StartsWith("sys:task-monitor:", command.IdempotencyKey);

        session.TaskRuns.Add(task);
        session.Commands.Add(command);
        command.State = WorkerCommandState.Completed;

        Assert.True(WorkerHub.ReleaseSessionAfterConfirmedCancellation(
            session,
            task.Id,
            now.AddSeconds(1)));
        Assert.Equal(SimulationSessionState.Ready, session.State);
        Assert.Equal(8, session.Revision);
    }

    [Fact]
    public void MonitorVerdictIsImmutableAgainstLateTerminalReports()
    {
        var task = new TaskRun
        {
            State = TaskRunState.Failed,
            Error = "Progress watchdog timed out.",
            OutcomeState = TaskOutcomeState.Failed,
            OutcomeReason = "Progress watchdog timed out.",
            SimulationSession = new SimulationSession
            {
                State = SimulationSessionState.Active
            }
        };

        Assert.False(WorkerHub.CanApplyTaskTransition(
            task,
            TaskRunState.Failed));
        Assert.False(WorkerHub.CanApplyTaskTransition(
            task,
            TaskRunState.Completed));
    }

    [Fact]
    public void CancellationCompletionNeedsCorrelatedRosConfirmation()
    {
        var taskRunId = Guid.NewGuid();
        var command = new WorkerCommand
        {
            Type = WorkerCommandType.CancelTask,
            TaskRunId = taskRunId
        };
        var valid = JsonSerializer.SerializeToElement(new
        {
            taskRunId,
            taskCancellationConfirmed = true
        });
        var stale = JsonSerializer.SerializeToElement(new
        {
            taskRunId = Guid.NewGuid(),
            taskCancellationConfirmed = true
        });
        var unconfirmed = JsonSerializer.SerializeToElement(new
        {
            taskRunId,
            taskCancellationConfirmed = false
        });

        Assert.True(WorkerHub.TryValidateTaskCancellationCompletion(
            command,
            valid));
        Assert.False(WorkerHub.TryValidateTaskCancellationCompletion(
            command,
            stale));
        Assert.False(WorkerHub.TryValidateTaskCancellationCompletion(
            command,
            unconfirmed));
    }

    [Fact]
    public void FailedCancellationLeavesTheSessionClosedForSafety()
    {
        var now = DateTime.UtcNow;
        var session = new SimulationSession
        {
            State = SimulationSessionState.Active,
            Revision = 4
        };
        var task = new TaskRun
        {
            State = TaskRunState.Failed,
            SimulationSession = session
        };

        Assert.True(WorkerHub.FailSessionAfterCancellationFailure(
            task,
            "ROS confirmation timed out.",
            now));
        Assert.Equal(SimulationSessionState.Failed, session.State);
        Assert.Equal(now, session.StoppedAt);
        Assert.Equal(5, session.Revision);
        Assert.Contains(
            "ROS confirmation timed out",
            session.FailureReason,
            StringComparison.Ordinal);
    }

    [Fact]
    public async Task PendingTaskCancellationKeepsTheSessionNonStartable()
    {
        await using var dataContext = TestDataContext.Create();
        var owner = Owner();
        var worker = new ComputeWorker { Name = "gpu-worker" };
        var session = Session(owner, worker, SimulationSessionState.Ready);
        session.Commands.Add(new WorkerCommand
        {
            SimulationSession = session,
            SimulationSessionId = session.Id,
            ComputeWorker = worker,
            ComputeWorkerId = worker.Id,
            Type = WorkerCommandType.CancelTask,
            State = WorkerCommandState.Dispatched,
            IdempotencyKey = "sys:task-monitor:test",
            Sequence = 1
        });
        dataContext.AddRange(owner, worker, session);
        await dataContext.SaveChangesAsync();

        Assert.True(await SessionControlRoute.HasPendingTaskCancellation(
            dataContext,
            session.Id));

        session.Commands.Single().State = WorkerCommandState.Completed;
        await dataContext.SaveChangesAsync();

        Assert.False(await SessionControlRoute.HasPendingTaskCancellation(
            dataContext,
            session.Id));
    }

    [Fact]
    public async Task StaleTaskReportCannotOverwriteTheMonitoredFailure()
    {
        var databaseName = Guid.NewGuid().ToString("N");
        var databaseRoot = new InMemoryDatabaseRoot();
        var taskId = Guid.NewGuid();

        await using (var seed = TestDataContext.Create(databaseName, databaseRoot))
        {
            var owner = Owner();
            var worker = new ComputeWorker { Name = "gpu-worker" };
            var session = Session(owner, worker, SimulationSessionState.Active);
            session.TaskRuns.Add(new TaskRun
            {
                Id = taskId,
                SimulationSession = session,
                SimulationSessionId = session.Id,
                Type = SwarmTaskRunType.Figure,
                State = TaskRunState.Running
            });
            seed.AddRange(owner, worker, session);
            await seed.SaveChangesAsync();
        }

        await using var staleReporter = TestDataContext.Create(databaseName, databaseRoot);
        await using var monitor = TestDataContext.Create(databaseName, databaseRoot);
        var staleTask = await staleReporter.TaskRuns.SingleAsync(task => task.Id == taskId);
        var monitoredTask = await monitor.TaskRuns
            .Include(task => task.Commands)
            .Include(task => task.SimulationSession)
            .SingleAsync(task => task.Id == taskId);

        TaskOutcomeMonitor.FailStalledTask(
            monitoredTask,
            "No measurable progress.",
            DateTime.UtcNow);
        await monitor.SaveChangesAsync();

        staleTask.Progress = 0.5;
        staleTask.LastReportAt = DateTime.UtcNow;
        staleTask.UpdatedAt = DateTime.UtcNow;

        await Assert.ThrowsAsync<DbUpdateConcurrencyException>(
            () => staleReporter.SaveChangesAsync());
    }

    [Fact]
    public async Task StalePullCannotRedispatchACancelledCommand()
    {
        var databaseName = Guid.NewGuid().ToString("N");
        var databaseRoot = new InMemoryDatabaseRoot();
        var commandId = Guid.NewGuid();

        await using (var seed = TestDataContext.Create(databaseName, databaseRoot))
        {
            var owner = Owner();
            var worker = new ComputeWorker { Name = "gpu-worker" };
            var session = Session(owner, worker, SimulationSessionState.Ready);
            session.Commands.Add(new WorkerCommand
            {
                Id = commandId,
                SimulationSession = session,
                SimulationSessionId = session.Id,
                ComputeWorker = worker,
                ComputeWorkerId = worker.Id,
                Type = WorkerCommandType.StartTask,
                State = WorkerCommandState.Pending,
                IdempotencyKey = "test:stale-pull",
                Sequence = 1
            });
            seed.AddRange(owner, worker, session);
            await seed.SaveChangesAsync();
        }

        await using var stalePull = TestDataContext.Create(databaseName, databaseRoot);
        await using var monitor = TestDataContext.Create(databaseName, databaseRoot);
        var staleCommand = await stalePull.WorkerCommands
            .SingleAsync(command => command.Id == commandId);
        var cancelledCommand = await monitor.WorkerCommands
            .SingleAsync(command => command.Id == commandId);

        cancelledCommand.State = WorkerCommandState.Cancelled;
        cancelledCommand.UpdatedAt = DateTime.UtcNow;
        await monitor.SaveChangesAsync();

        staleCommand.State = WorkerCommandState.Dispatched;
        staleCommand.UpdatedAt = DateTime.UtcNow;

        await Assert.ThrowsAsync<DbUpdateConcurrencyException>(
            () => stalePull.SaveChangesAsync());
    }

    [Fact]
    public void TransportEvidenceCapabilityDistinguishesOldAndCommissionedWorkers()
    {
        var oldWorker = new ComputeWorker
        {
            Capabilities = JsonDocument.Parse(
                """{"commandTypes":["StartTask"]}""")
        };
        var commissionedWorker = new ComputeWorker
        {
            Capabilities = JsonDocument.Parse(
                """
                {
                  "taskOutcomes": {
                    "collaborativeTransportEvidenceVersion": 1
                  }
                }
                """)
        };

        Assert.False(
            WorkerCapabilities.SupportsCollaborativeTransportEvidence(oldWorker));
        Assert.True(
            WorkerCapabilities.SupportsCollaborativeTransportEvidence(
                commissionedWorker));
    }

    private static JsonDocument TransportResult(
        Guid taskId,
        string finder,
        string[] notified,
        int contributorCount,
        string[] contributors)
    {
        return JsonSerializer.SerializeToDocument(new
        {
            transport = new
            {
                phase = "DONE",
                all_pushers_confirmed = true,
                useful_contributor_count = contributorCount,
                useful_contributor_ids = contributors,
                discovery = new
                {
                    @event = "payload_found",
                    task_id = taskId,
                    announced = true,
                    finder,
                    notified_robots = notified
                }
            }
        });
    }

    private static ComputeWorker DrainingWorker(DateTime expiresAt)
    {
        return new ComputeWorker
        {
            Name = "gpu-worker",
            State = ComputeWorkerState.Draining,
            DrainLeaseId = Guid.NewGuid(),
            DrainTargetRevision = new string('a', 40),
            DrainRequestedAt = expiresAt.AddMinutes(-10),
            DrainLeaseExpiresAt = expiresAt
        };
    }

    private static Account Owner()
    {
        return new Account
        {
            Id = 1,
            FirstName = "Drain",
            LastName = "Owner",
            Email = "drain-owner@example.test",
            Enabled = true
        };
    }

    private static SimulationSession Session(
        Account owner,
        ComputeWorker worker,
        SimulationSessionState state)
    {
        return new SimulationSession
        {
            Account = owner,
            AccountId = owner.Id,
            ComputeWorker = worker,
            ComputeWorkerId = worker.Id,
            State = state,
            DesiredRobotCount = 3
        };
    }

    private static DefaultHttpContext WorkerContext(Guid workerId)
    {
        return new DefaultHttpContext
        {
            User = new ClaimsPrincipal(new ClaimsIdentity(
                new[] { new Claim("worker_id", workerId.ToString()) },
                "test"))
        };
    }
}
