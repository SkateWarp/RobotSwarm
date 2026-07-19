using System.Data;
using System.Text.Json;
using Microsoft.AspNetCore.SignalR;
using Microsoft.EntityFrameworkCore;
using Npgsql;
using SwarmBackend.Entities;
using SwarmBackend.Helpers;
using SwarmBackend.Models;

namespace SwarmBackend.Services;

public sealed class TaskOutcomeMonitor(
    IServiceScopeFactory scopeFactory,
    IHubContext<WorkerHub> workerHubContext,
    IHubContext<SessionHub> sessionHubContext,
    IConfiguration configuration,
    ILogger<TaskOutcomeMonitor> logger) : BackgroundService
{
    protected override async Task ExecuteAsync(CancellationToken stoppingToken)
    {
        var interval = TimeSpan.FromSeconds(Math.Clamp(
            configuration.GetValue("Tasks:MonitorIntervalSeconds", 5),
            2,
            30));
        using var timer = new PeriodicTimer(interval);

        while (!stoppingToken.IsCancellationRequested)
        {
            try
            {
                await CheckTasks(stoppingToken);
            }
            catch (PostgresException exception)
                when (exception.SqlState == PostgresErrorCodes.SerializationFailure)
            {
                logger.LogDebug(
                    "Task outcome monitor met a concurrent report; it will retry on the next tick.");
            }
            catch (DbUpdateException exception)
                when (exception.InnerException is PostgresException
                {
                    SqlState: PostgresErrorCodes.SerializationFailure
                })
            {
                logger.LogDebug(
                    "Task outcome update met a concurrent report; it will retry on the next tick.");
            }
            catch (DbUpdateConcurrencyException)
            {
                logger.LogDebug(
                    "Task outcome monitor met a concurrent command or task transition; it will retry on the next tick.");
            }
            catch (OperationCanceledException) when (stoppingToken.IsCancellationRequested)
            {
                break;
            }
            catch (Exception exception)
            {
                logger.LogError(exception, "Task outcome monitor tick failed.");
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

    private async Task CheckTasks(CancellationToken cancellationToken)
    {
        var acceptedTimeout = TimeSpan.FromSeconds(Math.Clamp(
            configuration.GetValue("Tasks:AcceptedTimeoutSeconds", 90),
            30,
            600));
        var queuedTimeout = TimeSpan.FromSeconds(Math.Clamp(
            configuration.GetValue("Tasks:QueuedTimeoutSeconds", 120),
            30,
            900));
        var progressTimeout = TimeSpan.FromSeconds(Math.Clamp(
            configuration.GetValue("Tasks:ProgressTimeoutSeconds", 300),
            30,
            1800));

        await using var scope = scopeFactory.CreateAsyncScope();
        var dataContext = scope.ServiceProvider.GetRequiredService<DataContext>();
        await using var transaction = await dataContext.Database.BeginTransactionAsync(
            IsolationLevel.Serializable,
            cancellationToken);

        var now = DateTime.UtcNow;
        var candidates = await dataContext.TaskRuns
            .Include(task => task.Commands)
            .Include(task => task.SimulationSession)
            .ThenInclude(session => session.ComputeWorker)
            .Include(task => task.SimulationSession)
            .ThenInclude(session => session.Commands)
            .Where(task => task.State == TaskRunState.Queued
                || task.State == TaskRunState.Accepted
                || (task.State == TaskRunState.Running
                    && task.Type != SwarmTaskRunType.FollowLeader))
            .OrderBy(task => task.UpdatedAt)
            .Take(100)
            .ToListAsync(cancellationToken);

        var failures = new List<MonitoredFailure>();
        foreach (var task in candidates)
        {
            var reason = FindTimeoutReason(
                task,
                now,
                queuedTimeout,
                acceptedTimeout,
                progressTimeout);
            if (reason == null)
            {
                continue;
            }

            var command = FailStalledTask(task, reason, now);
            if (command != null)
            {
                dataContext.WorkerCommands.Add(command);
            }

            failures.Add(new MonitoredFailure(task, command));
        }

        if (failures.Count == 0)
        {
            await transaction.CommitAsync(cancellationToken);
            return;
        }

        await dataContext.SaveChangesAsync(cancellationToken);
        await transaction.CommitAsync(cancellationToken);

        foreach (var failure in failures)
        {
            var task = failure.Task;
            if (failure.CancelCommand != null && task.SimulationSession.ComputeWorkerId.HasValue)
            {
                await workerHubContext.Clients
                    .Group(ControlPlaneGroups.Worker(
                        task.SimulationSession.ComputeWorkerId.Value))
                    .SendAsync("CommandAvailable", cancellationToken);
            }

            var group = sessionHubContext.Clients
                .Group(ControlPlaneGroups.Session(task.SimulationSessionId));
            await group.SendAsync("TaskUpdated", ToTaskResponse(task), cancellationToken);
            await group.SendAsync(
                "SessionUpdated",
                ToSessionResponse(task.SimulationSession),
                cancellationToken);
            await group.SendAsync("SessionEvent", new
            {
                sessionId = task.SimulationSessionId,
                taskRunId = task.Id,
                eventType = "TaskProgressTimedOut",
                error = task.OutcomeReason,
                detectionSource = "TaskProgress",
                timestamp = now
            }, cancellationToken);

            logger.LogWarning(
                "Task {TaskRunId} failed outcome monitoring: {Reason}",
                task.Id,
                task.OutcomeReason);
        }
    }

    internal static string? FindTimeoutReason(
        TaskRun task,
        DateTime now,
        TimeSpan queuedTimeout,
        TimeSpan acceptedTimeout,
        TimeSpan progressTimeout)
    {
        if (task.State == TaskRunState.Queued
            && task.CreatedAt <= now - queuedTimeout)
        {
            return "The task remained queued because the worker did not accept its start command before the queue timeout.";
        }

        if (task.State == TaskRunState.Accepted)
        {
            var waitingSince = task.LastProgressAt
                ?? task.StartedAt
                ?? task.UpdatedAt;
            if (waitingSince <= now - acceptedTimeout)
            {
                return "The task was accepted by the worker but ROS did not begin it before the acceptance timeout.";
            }
        }

        // Follow-leader is deliberately open-ended. Its safety comes from the
        // worker/control heartbeat and emergency-stop watchdog, not a fake
        // completion-progress deadline.
        if (task.State == TaskRunState.Running
            && task.Type != SwarmTaskRunType.FollowLeader)
        {
            var lastProgress = task.LastProgressAt
                ?? task.LastReportAt
                ?? task.UpdatedAt;
            if (lastProgress <= now - progressTimeout)
            {
                return "The task stopped making measurable progress before reaching its acceptance condition.";
            }
        }

        return null;
    }

    internal static WorkerCommand? FailStalledTask(
        TaskRun task,
        string reason,
        DateTime now)
    {
        if (task.State is not (TaskRunState.Queued
            or TaskRunState.Accepted
            or TaskRunState.Running))
        {
            return null;
        }

        var originalState = task.State;
        var startMayHaveReachedWorker = false;
        foreach (var startCommand in task.Commands.Where(candidate =>
                     candidate.Type == WorkerCommandType.StartTask
                     && (candidate.State is WorkerCommandState.Pending
                         or WorkerCommandState.Dispatched
                         or WorkerCommandState.Acknowledged
                         or WorkerCommandState.Running)))
        {
            startMayHaveReachedWorker |= startCommand.State != WorkerCommandState.Pending;
            startCommand.State = WorkerCommandState.Cancelled;
            startCommand.LastError = reason;
            startCommand.CompletedAt = now;
            startCommand.UpdatedAt = now;
        }

        var cancellationAlreadyInFlight = task.Commands.Any(command =>
            command.Type == WorkerCommandType.CancelTask
            && (command.State is WorkerCommandState.Pending
                or WorkerCommandState.Dispatched
                or WorkerCommandState.Acknowledged
                or WorkerCommandState.Running));
        var startCanBeDiscardedLocally = originalState == TaskRunState.Queued
            && !startMayHaveReachedWorker;
        var canQueueCancellation = task.SimulationSession.ComputeWorkerId.HasValue
            && task.SimulationSession.State < SimulationSessionState.Stopping;
        var cleanupMustFinish = cancellationAlreadyInFlight
            || (!startCanBeDiscardedLocally && canQueueCancellation);

        task.State = TaskRunState.Failed;
        task.Error = reason;
        task.OutcomeState = TaskOutcomeState.Failed;
        task.OutcomeReason = reason;
        task.CompletedAt = now;
        task.UpdatedAt = now;

        var session = task.SimulationSession;
        if (!cleanupMustFinish
            && startCanBeDiscardedLocally
            && session.State is SimulationSessionState.Active or SimulationSessionState.Paused)
        {
            session.State = SimulationSessionState.Ready;
            session.UpdatedAt = now;
            session.Revision++;
        }

        if (!session.ComputeWorkerId.HasValue
            || session.State >= SimulationSessionState.Stopping
            || startCanBeDiscardedLocally
            || cancellationAlreadyInFlight)
        {
            return null;
        }

        var sequence = session.Commands.Select(command => command.Sequence)
            .DefaultIfEmpty()
            .Max() + 1;
        var command = new WorkerCommand
        {
            SimulationSessionId = session.Id,
            SimulationSession = session,
            TaskRunId = task.Id,
            TaskRun = task,
            ComputeWorkerId = session.ComputeWorkerId,
            Type = WorkerCommandType.CancelTask,
            State = WorkerCommandState.Pending,
            IdempotencyKey = $"sys:task-monitor:{task.Id:N}",
            Sequence = sequence,
            Payload = JsonSerializer.SerializeToDocument(new { taskRunId = task.Id }),
            CreatedAt = now,
            UpdatedAt = now
        };
        task.CommandRevision = sequence;
        return command;
    }

    private static TaskRunEventResponse ToTaskResponse(TaskRun task)
    {
        return new TaskRunEventResponse(
            task.Id,
            task.SimulationSessionId,
            task.Type.ToString(),
            task.State.ToString(),
            task.Progress,
            ControlPlaneJson.ToNullableElement(task.Result),
            task.Error,
            task.OutcomeState.ToString(),
            task.OutcomeReason,
            task.UpdatedAt,
            task.LastReportAt,
            task.LastProgressAt,
            task.StartedAt,
            task.CompletedAt);
    }

    private static SimulationSessionResponse ToSessionResponse(SimulationSession session)
    {
        return new SimulationSessionResponse(
            session.Id,
            session.State.ToString(),
            session.DesiredRobotCount,
            QueuePosition: null,
            session.ArenaVersion,
            session.ComputeWorkerId,
            session.ComputeWorker?.Name,
            session.IsEmergencyStopped,
            session.Revision,
            session.FailureReason,
            session.CreatedAt,
            session.UpdatedAt,
            session.StartedAt,
            session.StoppedAt);
    }

    private sealed record MonitoredFailure(
        TaskRun Task,
        WorkerCommand? CancelCommand);
}
