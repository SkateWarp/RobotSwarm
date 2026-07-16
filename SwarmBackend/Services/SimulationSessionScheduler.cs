using System.Data;
using System.Text.Json;
using Microsoft.AspNetCore.SignalR;
using Microsoft.EntityFrameworkCore;
using Npgsql;
using SwarmBackend.Entities;
using SwarmBackend.Helpers;
using SwarmBackend.Models;

namespace SwarmBackend.Services;

public class SimulationSessionScheduler(
    IServiceScopeFactory scopeFactory,
    IHubContext<WorkerHub> workerHubContext,
    IHubContext<SessionHub> sessionHubContext,
    IConfiguration configuration,
    ILogger<SimulationSessionScheduler> logger) : BackgroundService
{
    private static readonly TimeSpan SchedulerInterval = TimeSpan.FromSeconds(5);
    private static readonly TimeSpan WorkerStaleAfter = TimeSpan.FromSeconds(30);

    protected override async Task ExecuteAsync(CancellationToken stoppingToken)
    {
        using var timer = new PeriodicTimer(SchedulerInterval);
        while (!stoppingToken.IsCancellationRequested)
        {
            try
            {
                await Schedule(stoppingToken);
            }
            catch (PostgresException exception)
                when (exception.SqlState == PostgresErrorCodes.SerializationFailure)
            {
                logger.LogDebug("Concurrent scheduler transaction detected; retrying on the next tick.");
            }
            catch (DbUpdateException exception)
                when (exception.InnerException is PostgresException
                {
                    SqlState: PostgresErrorCodes.SerializationFailure
                })
            {
                logger.LogDebug("Concurrent scheduler update detected; retrying on the next tick.");
            }
            catch (OperationCanceledException) when (stoppingToken.IsCancellationRequested)
            {
                break;
            }
            catch (Exception exception)
            {
                logger.LogError(exception, "Simulation session scheduler tick failed.");
            }

            try
            {
                await timer.WaitForNextTickAsync(stoppingToken);
            }
            catch (OperationCanceledException) when (stoppingToken.IsCancellationRequested)
            {
                break;
            }
        }
    }

    private async Task Schedule(CancellationToken cancellationToken)
    {
        await using var scope = scopeFactory.CreateAsyncScope();
        var dataContext = scope.ServiceProvider.GetRequiredService<DataContext>();
        await using var transaction = await dataContext.Database.BeginTransactionAsync(
            IsolationLevel.Serializable,
            cancellationToken);

        var now = DateTime.UtcNow;
        var staleBefore = now - WorkerStaleAfter;
        var staleWorkers = await dataContext.ComputeWorkers
            .Where(worker => worker.State == ComputeWorkerState.Online
                && (!worker.LastHeartbeatAt.HasValue || worker.LastHeartbeatAt < staleBefore))
            .ToListAsync(cancellationToken);

        foreach (var worker in staleWorkers)
        {
            worker.State = ComputeWorkerState.Offline;
            worker.UpdatedAt = now;
        }

        var orphanAfterSeconds = Math.Clamp(
            configuration.GetValue("Workers:OrphanAfterSeconds", 90),
            60,
            600);
        var orphanBefore = now - TimeSpan.FromSeconds(orphanAfterSeconds);
        var orphanedSessions = await dataContext.SimulationSessions
            .Include(session => session.ComputeWorker)
            .Include(session => session.TaskRuns)
            .Include(session => session.Robots)
            .Include(session => session.ViewerLeases)
            .Include(session => session.Commands)
            .Where(session => session.State < SimulationSessionState.Stopped
                && session.ComputeWorkerId.HasValue
                && session.ComputeWorker != null
                && (session.ComputeWorker.CredentialRevokedAt.HasValue
                    || !session.ComputeWorker.LastHeartbeatAt.HasValue
                    || session.ComputeWorker.LastHeartbeatAt < orphanBefore))
            .ToListAsync(cancellationToken);

        foreach (var session in orphanedSessions)
        {
            FailOrphanedSession(session, now);
        }

        var availableWorkers = await dataContext.ComputeWorkers
            .Where(worker => worker.State == ComputeWorkerState.Online
                && worker.LastHeartbeatAt.HasValue
                && worker.LastHeartbeatAt >= staleBefore
                && worker.CredentialHash != null
                && worker.CredentialCreatedAt.HasValue
                && !worker.CredentialRevokedAt.HasValue)
            .OrderBy(worker => worker.Name)
            .ThenBy(worker => worker.Id)
            .ToListAsync(cancellationToken);

        var activeCounts = await dataContext.SimulationSessions
            .Where(session => session.ComputeWorkerId.HasValue
                && (session.State >= SimulationSessionState.Provisioning
                    && session.State < SimulationSessionState.Stopped
                    || session.State == SimulationSessionState.Failed
                    && session.Commands.Any(command =>
                        command.Type == WorkerCommandType.StopSession
                        && (command.State == WorkerCommandState.Pending
                            || command.State == WorkerCommandState.Dispatched
                            || command.State == WorkerCommandState.Acknowledged
                            || command.State == WorkerCommandState.Running))))
            .GroupBy(session => session.ComputeWorkerId!.Value)
            .Select(group => new { WorkerId = group.Key, Count = group.Count() })
            .ToDictionaryAsync(item => item.WorkerId, item => item.Count, cancellationToken);

        var capacities = availableWorkers.ToDictionary(
            worker => worker.Id,
            worker => Math.Max(0, worker.MaxConcurrentSessions
                - activeCounts.GetValueOrDefault(worker.Id)));

        var queuedSessions = await dataContext.SimulationSessions
            .Where(session => session.State == SimulationSessionState.Queued
                && !session.ComputeWorkerId.HasValue)
            .OrderBy(session => session.CreatedAt)
            .ThenBy(session => session.Id)
            .ToListAsync(cancellationToken);

        var assignments = new List<ScheduledAssignment>();
        foreach (var session in queuedSessions)
        {
            var worker = availableWorkers
                .Where(candidate => capacities.GetValueOrDefault(candidate.Id) > 0)
                .Where(candidate =>
                    WorkerCapabilities.GetMaxRobotsPerSession(candidate)
                    >= session.DesiredRobotCount)
                .OrderBy(candidate => activeCounts.GetValueOrDefault(candidate.Id))
                .ThenBy(candidate => candidate.Name)
                .ThenBy(candidate => candidate.Id)
                .FirstOrDefault();

            if (worker == null)
            {
                continue;
            }

            session.ComputeWorkerId = worker.Id;
            session.ComputeWorker = worker;
            session.WorkerImageVersion = worker.ImageVersion;
            session.State = SimulationSessionState.Provisioning;
            session.UpdatedAt = now;
            session.Revision++;

            var idempotencyKey = $"sys:provision:{session.Id:N}";
            var commandExists = await dataContext.WorkerCommands.AnyAsync(
                command => command.SimulationSessionId == session.Id
                    && command.IdempotencyKey == idempotencyKey,
                cancellationToken);
            if (!commandExists)
            {
                var sequence = (await dataContext.WorkerCommands
                    .Where(command => command.SimulationSessionId == session.Id)
                    .Select(command => (long?)command.Sequence)
                    .MaxAsync(cancellationToken) ?? 0) + 1;
                dataContext.WorkerCommands.Add(new WorkerCommand
                {
                    SimulationSessionId = session.Id,
                    ComputeWorkerId = worker.Id,
                    Type = WorkerCommandType.ProvisionSession,
                    State = WorkerCommandState.Pending,
                    IdempotencyKey = idempotencyKey,
                    Sequence = sequence,
                    Payload = JsonSerializer.SerializeToDocument(new
                    {
                        desiredRobotCount = session.DesiredRobotCount,
                        arenaVersion = session.ArenaVersion
                    }),
                    CreatedAt = now,
                    UpdatedAt = now
                });
            }

            capacities[worker.Id]--;
            activeCounts[worker.Id] = activeCounts.GetValueOrDefault(worker.Id) + 1;
            assignments.Add(new ScheduledAssignment(session, worker));
        }

        await dataContext.SaveChangesAsync(cancellationToken);
        await transaction.CommitAsync(cancellationToken);

        foreach (var assignment in assignments)
        {
            await workerHubContext.Clients
                .Group(ControlPlaneGroups.Worker(assignment.Worker.Id))
                .SendAsync("CommandAvailable", cancellationToken);
            await sessionHubContext.Clients
                .Group(ControlPlaneGroups.Session(assignment.Session.Id))
                .SendAsync(
                    "SessionUpdated",
                    ToResponse(assignment.Session, assignment.Worker),
                    cancellationToken);
        }

        foreach (var session in orphanedSessions)
        {
            var group = sessionHubContext.Clients
                .Group(ControlPlaneGroups.Session(session.Id));
            await group.SendAsync(
                "SessionUpdated",
                ToResponse(session, session.ComputeWorker!),
                cancellationToken);
            await group.SendAsync(
                "SessionEvent",
                new
                {
                    sessionId = session.Id,
                    state = session.State.ToString(),
                    error = session.FailureReason,
                    eventType = "WorkerOrphaned",
                    timestamp = now
                },
                cancellationToken);
        }
    }

    private static void FailOrphanedSession(
        SimulationSession session,
        DateTime now)
    {
        const string failureReason =
            "The compute worker stopped reporting. The session was closed for safety.";

        session.State = SimulationSessionState.Failed;
        session.FailureReason = failureReason;
        session.IsEmergencyStopped = false;
        session.StoppedAt ??= now;
        session.UpdatedAt = now;
        session.Revision++;

        foreach (var task in session.TaskRuns.Where(task => task.State < TaskRunState.Completed))
        {
            task.State = TaskRunState.Failed;
            task.Error = failureReason;
            task.CompletedAt = now;
            task.UpdatedAt = now;
        }

        foreach (var robot in session.Robots.Where(
                     robot => robot.State != SessionRobotState.Removed))
        {
            robot.State = SessionRobotState.Removed;
            robot.UpdatedAt = now;
        }

        foreach (var lease in session.ViewerLeases.Where(lease => !lease.RevokedAt.HasValue))
        {
            lease.RevokedAt = now;
        }

        foreach (var command in session.Commands.Where(command =>
                     command.State is WorkerCommandState.Pending
                         or WorkerCommandState.Dispatched
                         or WorkerCommandState.Acknowledged
                         or WorkerCommandState.Running))
        {
            command.State = WorkerCommandState.Cancelled;
            command.LastError = failureReason;
            command.CompletedAt = now;
            command.UpdatedAt = now;
        }

        var sequence = session.Commands.Count == 0
            ? 1
            : session.Commands.Max(command => command.Sequence) + 1;
        session.Commands.Add(new WorkerCommand
        {
            SimulationSessionId = session.Id,
            ComputeWorkerId = session.ComputeWorkerId,
            Type = WorkerCommandType.StopSession,
            State = WorkerCommandState.Pending,
            IdempotencyKey = $"sys:orphan-cleanup:{session.Id:N}",
            Sequence = sequence,
            Payload = JsonDocument.Parse("{}"),
            CreatedAt = now,
            UpdatedAt = now
        });
    }

    private static SimulationSessionResponse ToResponse(
        SimulationSession session,
        ComputeWorker worker)
    {
        return new SimulationSessionResponse(
            session.Id,
            session.State.ToString(),
            session.DesiredRobotCount,
            QueuePosition: null,
            session.ArenaVersion,
            worker.Id,
            worker.Name,
            session.IsEmergencyStopped,
            session.Revision,
            session.FailureReason,
            session.CreatedAt,
            session.UpdatedAt,
            session.StartedAt,
            session.StoppedAt);
    }

    private sealed record ScheduledAssignment(
        SimulationSession Session,
        ComputeWorker Worker);
}
