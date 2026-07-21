using System.Collections.Concurrent;
using System.Data;
using System.Security.Claims;
using System.Text.Json;
using Microsoft.AspNetCore.Authorization;
using Microsoft.AspNetCore.SignalR;
using Microsoft.EntityFrameworkCore;
using Npgsql;
using SwarmBackend.Entities;
using SwarmBackend.Helpers;
using SwarmBackend.Models;

namespace SwarmBackend.Services;

[Authorize(AuthenticationSchemes = WorkerCredentialDefaults.AuthenticationScheme)]
public class WorkerHub(
    DataContext dataContext,
    IHubContext<SessionHub> sessionHubContext,
    IConfiguration configuration,
    ILogger<WorkerHub> logger) : Hub
{
    private static readonly ConcurrentDictionary<Guid, string> ActiveConnections = new();
    internal const int MaximumTerminalCleanupAttempts = TerminalCleanupPolicy.MaximumAttempts;
    internal static readonly TimeSpan TerminalCleanupBaseDelay =
        TerminalCleanupPolicy.BaseRetryDelay;
    private const int MaximumFailSafeTransactionAttempts = 3;
    private const int MaximumCommandTransitionAttempts = 3;

    public static void InvalidateConnection(Guid workerId)
    {
        ActiveConnections.TryRemove(workerId, out _);
    }

    public override async Task OnConnectedAsync()
    {
        var workerId = GetWorkerId();
        if (!ActiveConnections.TryAdd(workerId, Context.ConnectionId))
        {
            Context.Abort();
            throw new HubException("This compute worker already has an active connection.");
        }

        try
        {
            await Groups.AddToGroupAsync(
                Context.ConnectionId,
                ControlPlaneGroups.Worker(workerId),
                Context.ConnectionAborted);
            await base.OnConnectedAsync();
        }
        catch
        {
            RemoveActiveConnection(workerId, Context.ConnectionId);
            throw;
        }
    }

    public override async Task OnDisconnectedAsync(Exception? exception)
    {
        RemoveActiveConnection(GetWorkerId(), Context.ConnectionId);
        await base.OnDisconnectedAsync(exception);
    }

    public async Task<WorkerRegistrationResponse> Register(WorkerRegistrationRequest request)
    {
        for (var attempt = 1; attempt <= MaximumCommandTransitionAttempts; attempt++)
        {
            try
            {
                return await RegisterCore(request);
            }
            catch (DbUpdateConcurrencyException)
            {
                dataContext.ChangeTracker.Clear();
                logger.LogDebug(
                    "Worker registration met a concurrent drain transition (attempt {Attempt}).",
                    attempt);
            }
        }

        throw new HubException(
            "The worker drain state changed repeatedly during registration. Retry registration.");
    }

    private async Task<WorkerRegistrationResponse> RegisterCore(
        WorkerRegistrationRequest request)
    {
        var worker = await GetWorker();
        ApplyWorkerMetadata(worker, request.ImageVersion, request.Capabilities);

        var now = DateTime.UtcNow;
        WorkerDrainLease.ApplyHeartbeatState(worker, now);
        worker.LastHeartbeatAt = now;
        worker.UpdatedAt = now;
        await dataContext.SaveChangesAsync(Context.ConnectionAborted);

        logger.LogInformation("Compute worker {WorkerId} registered from connection {ConnectionId}.",
            worker.Id, Context.ConnectionId);
        return ToRegistrationResponse(worker, now);
    }

    public async Task<WorkerRegistrationResponse> Heartbeat(WorkerHeartbeatRequest request)
    {
        for (var attempt = 1; attempt <= MaximumCommandTransitionAttempts; attempt++)
        {
            try
            {
                return await HeartbeatCore(request);
            }
            catch (Exception exception) when (IsHeartbeatPersistenceConflict(exception))
            {
                dataContext.ChangeTracker.Clear();
                logger.LogDebug(
                    exception,
                    "Worker heartbeat met a concurrent drain transition (attempt {Attempt}).",
                    attempt);
            }
        }

        throw new HubException(
            "The worker drain state changed repeatedly during heartbeat. Retry the heartbeat.");
    }

    private async Task<WorkerRegistrationResponse> HeartbeatCore(
        WorkerHeartbeatRequest request)
    {
        await using var transaction = dataContext.Database.IsRelational()
            ? await dataContext.Database.BeginTransactionAsync(
                IsolationLevel.ReadCommitted,
                Context.ConnectionAborted)
            : null;
        var worker = await GetWorker();
        ApplyWorkerMetadata(worker, request.ImageVersion, request.Capabilities);

        var now = DateTime.UtcNow;
        WorkerDrainLease.ApplyHeartbeatState(worker, now);
        worker.LastHeartbeatAt = now;
        if (request.ActiveSessionIds != null)
        {
            worker.ReportedActiveSessionCount = request.ActiveSessionIds
                .Take(1000)
                .Distinct()
                .Count();
            worker.ActiveSessionsReportedAt = now;
        }

        worker.UpdatedAt = now;
        var reconciliation = await ReconcileTerminalSessions(
            worker.Id,
            request.ActiveSessionIds,
            now);
        await dataContext.SaveChangesAsync(Context.ConnectionAborted);
        if (transaction != null)
        {
            await transaction.CommitAsync(Context.ConnectionAborted);
        }

        foreach (var command in reconciliation.CleanupCommands)
        {
            await sessionHubContext.Clients
                .Group(ControlPlaneGroups.Session(command.SimulationSessionId))
                .SendAsync("SessionEvent", new
                {
                    sessionId = command.SimulationSessionId,
                    commandId = command.Id,
                    eventType = "CommandQueued",
                    state = command.State.ToString(),
                    timestamp = command.CreatedAt
                }, Context.ConnectionAborted);
        }

        foreach (var session in reconciliation.ReconciledSessions)
        {
            var group = sessionHubContext.Clients
                .Group(ControlPlaneGroups.Session(session.Id));
            await group.SendAsync(
                "SessionUpdated",
                ToSessionResponse(session),
                Context.ConnectionAborted);
            await group.SendAsync("SessionEvent", new
            {
                sessionId = session.Id,
                state = session.State.ToString(),
                error = session.FailureReason,
                eventType = "SessionReconciled",
                timestamp = now
            }, Context.ConnectionAborted);
        }

        return ToRegistrationResponse(worker, now);
    }

    public async Task<IReadOnlyList<WorkerCommandEnvelope>> PullPendingCommands(int maxCount = 25)
    {
        var workerId = (await GetWorker()).Id;
        maxCount = Math.Clamp(maxCount, 1, 100);

        for (var attempt = 1; attempt <= MaximumCommandTransitionAttempts; attempt++)
        {
            try
            {
                var commands = await dataContext.WorkerCommands
                    .Where(command => command.ComputeWorkerId == workerId
                        && (command.State == WorkerCommandState.Pending
                            || command.State == WorkerCommandState.Dispatched
                            || command.State == WorkerCommandState.Acknowledged
                            || command.State == WorkerCommandState.Running))
                    .OrderBy(command => command.CreatedAt)
                    .ThenBy(command => command.Sequence)
                    .Take(maxCount)
                    .ToListAsync(Context.ConnectionAborted);

                var now = DateTime.UtcNow;
                foreach (var command in commands.Where(command =>
                             command.State == WorkerCommandState.Pending))
                {
                    command.State = WorkerCommandState.Dispatched;
                    command.DispatchedAt = now;
                    command.UpdatedAt = now;
                }

                if (dataContext.ChangeTracker.HasChanges())
                {
                    await dataContext.SaveChangesAsync(Context.ConnectionAborted);
                }

                return commands.Select(ToEnvelope).ToList();
            }
            catch (DbUpdateConcurrencyException)
            {
                dataContext.ChangeTracker.Clear();
                if (attempt == MaximumCommandTransitionAttempts)
                {
                    break;
                }

                logger.LogDebug(
                    "Command pull met a concurrent cancellation; retrying attempt {Attempt}.",
                    attempt + 1);
            }
        }

        throw new HubException(
            "Pending commands changed repeatedly while they were being claimed. Retry the pull.");
    }

    public async Task AcknowledgeCommand(Guid commandId)
    {
        for (var attempt = 1; attempt <= MaximumCommandTransitionAttempts; attempt++)
        {
            try
            {
                var command = await GetOwnedCommand(commandId);
                if (IsTerminal(command.State))
                {
                    throw new HubException(
                        "The command was cancelled before acknowledgement.");
                }

                if (command.State is WorkerCommandState.Acknowledged
                    or WorkerCommandState.Running)
                {
                    return;
                }

                var now = DateTime.UtcNow;
                command.State = WorkerCommandState.Acknowledged;
                command.AcknowledgedAt = now;
                command.UpdatedAt = now;
                await dataContext.SaveChangesAsync(Context.ConnectionAborted);
                await PublishCommandUpdate(command);
                return;
            }
            catch (DbUpdateConcurrencyException)
            {
                dataContext.ChangeTracker.Clear();
                logger.LogDebug(
                    "Command {CommandId} changed while it was being acknowledged (attempt {Attempt}).",
                    commandId,
                    attempt);
            }
        }

        throw new HubException(
            "The command changed repeatedly before acknowledgement.");
    }

    public async Task MarkCommandRunning(Guid commandId)
    {
        for (var attempt = 1; attempt <= MaximumCommandTransitionAttempts; attempt++)
        {
            try
            {
                var command = await GetOwnedCommand(commandId);
                if (IsTerminal(command.State))
                {
                    throw new HubException(
                        "The command was cancelled before execution.");
                }

                if (command.State == WorkerCommandState.Running)
                {
                    return;
                }

                var now = DateTime.UtcNow;
                command.State = WorkerCommandState.Running;
                command.AcknowledgedAt ??= now;
                command.UpdatedAt = now;
                await dataContext.SaveChangesAsync(Context.ConnectionAborted);
                await PublishCommandUpdate(command);
                return;
            }
            catch (DbUpdateConcurrencyException)
            {
                dataContext.ChangeTracker.Clear();
                logger.LogDebug(
                    "Command {CommandId} changed while it was being started (attempt {Attempt}).",
                    commandId,
                    attempt);
            }
        }

        throw new HubException(
            "The command changed repeatedly before execution.");
    }

    public async Task CompleteCommand(WorkerCommandCompletionRequest request)
    {
        for (var attempt = 1; attempt <= MaximumCommandTransitionAttempts; attempt++)
        {
            try
            {
                await CompleteCommandCore(request);
                return;
            }
            catch (Exception exception) when (IsCommandCompletionConflict(exception))
            {
                dataContext.ChangeTracker.Clear();
                Context.ConnectionAborted.ThrowIfCancellationRequested();
                logger.LogDebug(
                    exception,
                    "Command {CommandId} completion conflicted with session state (attempt {Attempt}).",
                    request.CommandId,
                    attempt);
            }
        }

        throw new HubException(
            "The command completion conflicted repeatedly with session state. Retry the report.");
    }

    private async Task CompleteCommandCore(WorkerCommandCompletionRequest request)
    {
        await using var transaction = await dataContext.Database.BeginTransactionAsync(
            IsolationLevel.Serializable,
            Context.ConnectionAborted);
        var command = await GetOwnedCommand(request.CommandId);
        if (IsTerminal(command.State))
        {
            await transaction.CommitAsync(Context.ConnectionAborted);
            return;
        }

        var now = DateTime.UtcNow;
        SimulationSession? updatedSession = null;
        if (command.Type is WorkerCommandType.ProvisionSession
            or WorkerCommandType.UpdateFleet)
        {
            if (!request.Result.HasValue)
            {
                throw new HubException("Fleet command completion did not include a result.");
            }

            updatedSession = await SyncRobotRoster(
                command,
                request.Result.Value,
                now);
        }
        else if (command.Type is WorkerCommandType.EmergencyStop
                 or WorkerCommandType.ResetEmergencyStop)
        {
            if (!request.Result.HasValue
                || !TryGetProperty(
                    request.Result.Value,
                    "emergencyStop",
                    out var emergencyStopElement)
                || emergencyStopElement.ValueKind is not JsonValueKind.True
                    and not JsonValueKind.False)
            {
                throw new HubException(
                    "Emergency-stop completion did not include a confirmed state.");
            }

            var expectedState = command.Type == WorkerCommandType.EmergencyStop;
            if (emergencyStopElement.GetBoolean() != expectedState)
            {
                throw new HubException(
                    "Emergency-stop completion did not match the requested state.");
            }

            updatedSession = await dataContext.SimulationSessions
                .Include(session => session.ComputeWorker)
                .SingleAsync(
                    session => session.Id == command.SimulationSessionId,
                    Context.ConnectionAborted);
            if (updatedSession.State < SimulationSessionState.Stopped
                && updatedSession.IsEmergencyStopped != expectedState)
            {
                updatedSession.IsEmergencyStopped = expectedState;
                updatedSession.UpdatedAt = now;
                updatedSession.Revision++;
            }
        }
        else if (command.Type == WorkerCommandType.CancelTask)
        {
            if (!request.Result.HasValue
                || !TryValidateTaskCancellationCompletion(
                    command,
                    request.Result.Value))
            {
                throw new HubException(
                    "Task cancellation completion did not include correlated ROS stop confirmation.");
            }

            updatedSession = await dataContext.SimulationSessions
                .Include(session => session.ComputeWorker)
                .Include(session => session.TaskRuns)
                .Include(session => session.Commands)
                .SingleAsync(
                    session => session.Id == command.SimulationSessionId,
                    Context.ConnectionAborted);
        }
        else if (command.Type == WorkerCommandType.SetViewerSource)
        {
            var validationError = "Viewer command completion did not include a result.";
            if (!request.Result.HasValue
                || !ViewerPublishCompletionValidator.TryValidate(
                    command.SimulationSessionId,
                    command.Payload.RootElement,
                    request.Result.Value,
                    out validationError))
            {
                throw new HubException(validationError);
            }
        }
        else if (command.Type == WorkerCommandType.StopViewer)
        {
            if (!request.Result.HasValue
                || !TryValidateViewerStopCompletion(command, request.Result.Value))
            {
                throw new HubException(
                    "Viewer stop completion did not match the durable command.");
            }
        }

        command.State = WorkerCommandState.Completed;
        command.Result = request.Result.HasValue
            ? ControlPlaneJson.ToDocument(request.Result)
            : null;
        command.LastError = null;
        command.AcknowledgedAt ??= now;
        command.CompletedAt = now;
        command.UpdatedAt = now;

        if (command.Type == WorkerCommandType.CancelTask
            && updatedSession != null
            && !ReleaseSessionAfterConfirmedCancellation(
                updatedSession,
                command.TaskRunId,
                now))
        {
            updatedSession = null;
        }

        await dataContext.SaveChangesAsync(Context.ConnectionAborted);
        await transaction.CommitAsync(Context.ConnectionAborted);
        await PublishCommandUpdate(command);
        if (updatedSession != null)
        {
            await sessionHubContext.Clients
                .Group(ControlPlaneGroups.Session(updatedSession.Id))
                .SendAsync(
                    "SessionUpdated",
                    ToSessionResponse(updatedSession),
                    Context.ConnectionAborted);
        }
    }

    internal static bool IsCommandCompletionConflict(Exception exception)
    {
        return exception is DbUpdateConcurrencyException
            || IsSerializationFailure(exception);
    }

    public async Task FailCommand(WorkerCommandFailureRequest request)
    {
        for (var attempt = 1; attempt <= MaximumCommandTransitionAttempts; attempt++)
        {
            try
            {
                await FailCommandCore(request);
                return;
            }
            catch (DbUpdateConcurrencyException)
            {
                dataContext.ChangeTracker.Clear();
                logger.LogDebug(
                    "Command {CommandId} changed while its failure was being recorded (attempt {Attempt}).",
                    request.CommandId,
                    attempt);
            }
        }

        throw new HubException(
            "The command changed repeatedly while its failure was being recorded. Retry the report.");
    }

    private async Task FailCommandCore(WorkerCommandFailureRequest request)
    {
        var workerId = (await GetWorker()).Id;
        var command = await dataContext.WorkerCommands
            .Include(candidate => candidate.TaskRun)
            .ThenInclude(task => task!.SimulationSession)
            .ThenInclude(session => session.ComputeWorker)
            .SingleOrDefaultAsync(
                candidate => candidate.Id == request.CommandId
                    && candidate.ComputeWorkerId == workerId,
                Context.ConnectionAborted)
            ?? throw new HubException("Assigned worker command not found.");
        if (IsTerminal(command.State))
        {
            return;
        }

        var now = DateTime.UtcNow;
        var error = Truncate(
            request.Error ?? "Worker reported a command failure.",
            4000);
        command.State = WorkerCommandState.Failed;
        command.LastError = error;
        command.AcknowledgedAt ??= now;
        command.CompletedAt = now;
        command.UpdatedAt = now;

        var task = command.TaskRun;
        var sessionChanged = false;
        if (task != null && task.State < TaskRunState.Completed)
        {
            if (command.Type == WorkerCommandType.StartTask)
            {
                task.State = TaskRunState.Failed;
                task.OutcomeState = TaskOutcomeState.Failed;
                task.OutcomeReason = error;
                task.CompletedAt = now;
                sessionChanged = AlignSessionStateWithTask(task, now);
            }

            task.Error = error;
            task.UpdatedAt = now;
        }
        else if (task != null
                 && command.Type == WorkerCommandType.CancelTask
                 && FailSessionAfterCancellationFailure(task, error, now))
        {
            sessionChanged = true;
        }

        await dataContext.SaveChangesAsync(Context.ConnectionAborted);
        await PublishCommandUpdate(command);

        if (task != null)
        {
            var group = sessionHubContext.Clients
                .Group(ControlPlaneGroups.Session(task.SimulationSessionId));
            if (sessionChanged)
            {
                await group.SendAsync(
                    "SessionUpdated",
                    ToSessionResponse(task.SimulationSession),
                    Context.ConnectionAborted);
            }

            await group.SendAsync(
                "TaskUpdated",
                ToTaskResponse(task),
                Context.ConnectionAborted);
        }
    }

    public async Task<SimulationSessionResponse> ReportEmergencyStop(
        WorkerEmergencyStopReport report)
    {
        if (!report.Active)
        {
            throw new HubException(
                "Only a worker-initiated fail-safe emergency stop can be reported.");
        }

        var workerId = (await GetWorker()).Id;
        SimulationSession? session = null;
        IReadOnlyList<WorkerCommand> cancelledCommands = Array.Empty<WorkerCommand>();
        var now = DateTime.UtcNow;
        for (var attempt = 1; attempt <= MaximumFailSafeTransactionAttempts; attempt++)
        {
            await using var transaction = await dataContext.Database.BeginTransactionAsync(
                IsolationLevel.Serializable,
                Context.ConnectionAborted);
            try
            {
                session = await dataContext.SimulationSessions
                    .Include(candidate => candidate.ComputeWorker)
                    .Include(candidate => candidate.Commands)
                    .SingleOrDefaultAsync(
                        candidate => candidate.Id == report.SessionId
                            && candidate.ComputeWorkerId == workerId,
                        Context.ConnectionAborted)
                    ?? throw new HubException("Assigned simulation session not found.");
                if (session.State >= SimulationSessionState.Stopped)
                {
                    throw new HubException("The simulation session is already closed.");
                }

                var wasEmergencyStopped = session.IsEmergencyStopped;
                cancelledCommands = ApplyFailSafeTransition(session, now);
                if (!wasEmergencyStopped || cancelledCommands.Count > 0)
                {
                    await dataContext.SaveChangesAsync(Context.ConnectionAborted);
                }

                await transaction.CommitAsync(Context.ConnectionAborted);
                break;
            }
            catch (Exception exception)
                when (exception is DbUpdateConcurrencyException
                    || IsSerializationFailure(exception))
            {
                if (attempt < MaximumFailSafeTransactionAttempts)
                {
                    logger.LogDebug(
                        exception,
                        "Fail-safe transaction for session {SessionId} conflicted; retrying attempt {Attempt}.",
                        report.SessionId,
                        attempt + 1);
                }
                else
                {
                    logger.LogWarning(
                        exception,
                        "Fail-safe transaction for session {SessionId} exhausted its retry budget.",
                        report.SessionId);
                }

                dataContext.ChangeTracker.Clear();
                session = null;
                cancelledCommands = Array.Empty<WorkerCommand>();
            }
        }

        if (session == null)
        {
            throw new HubException(
                "The emergency-stop report conflicted repeatedly. Retry the report.");
        }

        foreach (var command in cancelledCommands)
        {
            await PublishCommandUpdate(command);
        }

        var response = ToSessionResponse(session);
        var group = sessionHubContext.Clients
            .Group(ControlPlaneGroups.Session(session.Id));
        await group.SendAsync("SessionUpdated", response, Context.ConnectionAborted);
        await group.SendAsync("SessionEvent", new
        {
            sessionId = session.Id,
            state = session.State.ToString(),
            error = Truncate(
                report.Reason ?? "The compute worker applied its disconnect fail-safe.",
                2000),
            eventType = "WorkerFailSafeEmergencyStop",
            timestamp = now
        }, Context.ConnectionAborted);
        return response;
    }

    public async Task<SimulationSessionResponse> ReportSessionEvent(SessionEventReport report)
    {
        var workerId = (await GetWorker()).Id;
        await using var transaction = dataContext.Database.IsRelational()
            ? await dataContext.Database.BeginTransactionAsync(
                IsolationLevel.ReadCommitted,
                Context.ConnectionAborted)
            : null;
        var session = dataContext.Database.IsRelational()
            ? await LockAssignedSession(report.SessionId, workerId)
            : await dataContext.SimulationSessions
                .Include(candidate => candidate.ComputeWorker)
                .SingleOrDefaultAsync(
                    candidate => candidate.Id == report.SessionId
                        && candidate.ComputeWorkerId == workerId,
                    Context.ConnectionAborted);

        if (session == null)
        {
            throw new HubException("Assigned simulation session not found.");
        }

        if (!TryParseSessionReportState(report.State, out var nextState))
        {
            throw new HubException("Unsupported simulation session state.");
        }

        if (!CanApplySessionTransition(session.State, nextState))
        {
            logger.LogWarning(
                "Ignored stale session transition {CurrentState} -> {NextState} for session {SessionId}.",
                session.State,
                nextState,
                session.Id);
            return ToSessionResponse(session);
        }

        var now = DateTime.UtcNow;
        var failureReason = FailureReasonForSessionEvent(
            session,
            nextState,
            report.FailureReason);
        var changed = session.State != nextState || session.FailureReason != failureReason;
        IReadOnlyList<TaskRun> finalizedTasks = Array.Empty<TaskRun>();

        if (changed)
        {
            session.State = nextState;
            session.FailureReason = failureReason;
            session.UpdatedAt = now;
            session.Revision++;

            if (nextState == SimulationSessionState.Active)
            {
                session.StartedAt ??= now;
            }

            if (nextState >= SimulationSessionState.Stopped)
            {
                session.StoppedAt ??= now;
                session.IsEmergencyStopped = false;
                finalizedTasks = await FinalizeSessionResources(
                    session.Id,
                    nextState == SimulationSessionState.Failed,
                    failureReason,
                    now);
            }

            await dataContext.SaveChangesAsync(Context.ConnectionAborted);
        }
        if (transaction != null)
        {
            await transaction.CommitAsync(Context.ConnectionAborted);
        }

        var response = ToSessionResponse(session);
        var group = sessionHubContext.Clients.Group(ControlPlaneGroups.Session(session.Id));
        await group.SendAsync("SessionUpdated", response, Context.ConnectionAborted);
        await group.SendAsync("SessionEvent", new
        {
            sessionId = session.Id,
            state = session.State.ToString(),
            payload = report.Payload,
            timestamp = now
        }, Context.ConnectionAborted);
        foreach (var task in finalizedTasks)
        {
            await group.SendAsync(
                "TaskUpdated",
                ToTaskResponse(task),
                Context.ConnectionAborted);
        }

        return response;
    }

    public async Task<TaskRunEventResponse> ReportTaskEvent(TaskEventReport report)
    {
        for (var attempt = 1; attempt <= MaximumCommandTransitionAttempts; attempt++)
        {
            try
            {
                return await ReportTaskEventCore(report);
            }
            catch (DbUpdateConcurrencyException)
            {
                dataContext.ChangeTracker.Clear();
                logger.LogDebug(
                    "Task {TaskRunId} changed while a worker report was being applied (attempt {Attempt}).",
                    report.TaskRunId,
                    attempt);
            }
        }

        throw new HubException(
            "The task changed repeatedly while its report was being applied. Retry the report.");
    }

    private async Task<TaskRunEventResponse> ReportTaskEventCore(TaskEventReport report)
    {
        var workerId = (await GetWorker()).Id;
        await using var transaction = dataContext.Database.IsRelational()
            ? await dataContext.Database.BeginTransactionAsync(
                IsolationLevel.ReadCommitted,
                Context.ConnectionAborted)
            : null;
        TaskRun? task;
        if (dataContext.Database.IsRelational())
        {
            var session = await LockAssignedSession(
                report.SessionId,
                workerId,
                loadRobots: true);
            task = session == null
                ? null
                : await dataContext.TaskRuns
                    .Include(candidate => candidate.Commands)
                    .SingleOrDefaultAsync(
                        candidate => candidate.Id == report.TaskRunId
                            && candidate.SimulationSessionId == session.Id,
                        Context.ConnectionAborted);
            if (task != null)
            {
                task.SimulationSession = session!;
            }
        }
        else
        {
            task = await dataContext.TaskRuns
                .Include(candidate => candidate.Commands)
                .Include(candidate => candidate.SimulationSession)
                .ThenInclude(session => session.ComputeWorker)
                .Include(candidate => candidate.SimulationSession)
                .ThenInclude(session => session.Robots)
                .SingleOrDefaultAsync(
                    candidate => candidate.Id == report.TaskRunId
                        && candidate.SimulationSessionId == report.SessionId
                        && candidate.SimulationSession.ComputeWorkerId == workerId,
                    Context.ConnectionAborted);
        }

        if (task == null)
        {
            throw new HubException("Assigned task run not found.");
        }

        if (!TryParseTaskReportState(report.State, out var nextState))
        {
            throw new HubException("Unsupported task run state.");
        }

        if (report.Progress.HasValue && !double.IsFinite(report.Progress.Value))
        {
            throw new HubException("Task progress must be finite.");
        }

        var effectiveReport = report;
        if (nextState == TaskRunState.Completed
            && !TaskAcceptancePolicy.TryAccept(
                task,
                report.Progress,
                report.Result,
                configuration.GetValue(
                    "Tasks:RequireCollaborativeTransportEvidence",
                    false),
                out var acceptanceFailure))
        {
            nextState = TaskRunState.Failed;
            effectiveReport = report with
            {
                State = TaskRunState.Failed.ToString(),
                Error = acceptanceFailure
            };
        }

        if (!CanApplyTaskTransition(task, nextState))
        {
            logger.LogWarning(
                "Ignored stale task transition {CurrentState} -> {NextState} for task {TaskRunId}.",
                task.State,
                nextState,
                task.Id);
            return ToTaskResponse(task);
        }

        if (IsDuplicateTaskReport(task, nextState, effectiveReport))
        {
            return ToTaskResponse(task);
        }

        var now = DateTime.UtcNow;
        var previousState = task.State;
        var previousProgress = task.Progress;
        task.State = nextState;
        task.Progress = effectiveReport.Progress.HasValue
            ? Math.Max(task.Progress, Math.Clamp(effectiveReport.Progress.Value, 0, 1))
            : task.Progress;
        task.Error = nextState == TaskRunState.Failed
            ? Truncate(effectiveReport.Error ?? "Worker reported a task failure.", 4000)
            : null;
        if (effectiveReport.Result.HasValue)
        {
            task.Result = ControlPlaneJson.ToDocument(effectiveReport.Result);
        }

        task.UpdatedAt = now;
        task.LastReportAt = now;
        if (task.Progress > previousProgress
            || (nextState is TaskRunState.Accepted or TaskRunState.Running
                && previousState != nextState))
        {
            task.LastProgressAt = now;
        }

        if (nextState == TaskRunState.Running)
        {
            task.StartedAt ??= now;
        }

        if (nextState >= TaskRunState.Completed)
        {
            task.StartedAt ??= InferTaskStartedAt(task);
            task.CompletedAt ??= now;
            if (nextState == TaskRunState.Completed)
            {
                task.Progress = 1;
            }
        }

        task.OutcomeState = nextState switch
        {
            TaskRunState.Completed => TaskOutcomeState.Succeeded,
            TaskRunState.Failed => TaskOutcomeState.Failed,
            TaskRunState.Cancelled => TaskOutcomeState.Cancelled,
            _ => TaskOutcomeState.Pending
        };
        task.OutcomeReason = nextState == TaskRunState.Failed
            ? task.Error
            : null;

        var sessionChanged = AlignSessionStateWithTask(task, now);
        await dataContext.SaveChangesAsync(Context.ConnectionAborted);
        if (transaction != null)
        {
            await transaction.CommitAsync(Context.ConnectionAborted);
        }

        var response = ToTaskResponse(task);
        var group = sessionHubContext.Clients.Group(ControlPlaneGroups.Session(task.SimulationSessionId));
        if (sessionChanged)
        {
            await group.SendAsync(
                "SessionUpdated",
                ToSessionResponse(task.SimulationSession),
                Context.ConnectionAborted);
        }

        await group.SendAsync("TaskUpdated", response, Context.ConnectionAborted);
        await group.SendAsync("SessionEvent", new
        {
            sessionId = task.SimulationSessionId,
            taskRunId = task.Id,
            eventType = "TaskUpdated",
            timestamp = now
        }, Context.ConnectionAborted);
        return response;
    }

    private Guid GetWorkerId()
    {
        var claim = Context.User?.FindFirst("worker_id")
            ?? Context.User?.FindFirst(ClaimTypes.NameIdentifier);
        if (claim == null || !Guid.TryParse(claim.Value, out var workerId))
        {
            throw new HubException("Authenticated worker identifier is missing.");
        }

        return workerId;
    }

    private async Task<SimulationSession?> LockAssignedSession(
        Guid sessionId,
        Guid workerId,
        bool loadRobots = false,
        bool loadCommands = false)
    {
        var session = await dataContext.SimulationSessions
            .FromSqlInterpolated($"""
                SELECT *
                FROM "SimulationSessions"
                WHERE "Id" = {sessionId} AND "ComputeWorkerId" = {workerId}
                FOR UPDATE
                """)
            .SingleOrDefaultAsync(Context.ConnectionAborted);
        if (session == null)
        {
            return null;
        }

        await dataContext.Entry(session)
            .Reference(candidate => candidate.ComputeWorker)
            .LoadAsync(Context.ConnectionAborted);
        if (loadRobots)
        {
            await dataContext.Entry(session)
                .Collection(candidate => candidate.Robots)
                .LoadAsync(Context.ConnectionAborted);
        }

        if (loadCommands)
        {
            await dataContext.Entry(session)
                .Collection(candidate => candidate.Commands)
                .LoadAsync(Context.ConnectionAborted);
        }

        return session;
    }

    private static void RemoveActiveConnection(Guid workerId, string connectionId)
    {
        if (ActiveConnections.TryGetValue(workerId, out var currentConnectionId)
            && currentConnectionId == connectionId)
        {
            ActiveConnections.TryRemove(workerId, out _);
        }
    }

    private async Task<ComputeWorker> GetWorker()
    {
        var workerId = GetWorkerId();
        var worker = await dataContext.ComputeWorkers.SingleOrDefaultAsync(
                worker => worker.Id == workerId,
                Context.ConnectionAborted);
        var credentialVersion = Context.User?.FindFirst("worker_credential_version")?.Value;
        if (worker?.CredentialHash == null
            || worker.CredentialRevokedAt.HasValue
            || credentialVersion != worker.CredentialCreatedAt?.Ticks.ToString())
        {
            RemoveActiveConnection(workerId, Context.ConnectionId);
            Context.Abort();
            throw new HubException("Compute worker credential is no longer active.");
        }

        return worker;
    }

    private async Task<HeartbeatReconciliation> ReconcileTerminalSessions(
        Guid workerId,
        IReadOnlyList<Guid>? activeSessionIds,
        DateTime now)
    {
        if (activeSessionIds == null)
        {
            return HeartbeatReconciliation.Empty;
        }

        var reportedIds = activeSessionIds
            .Take(1000)
            .Distinct()
            .ToArray();
        var reportedIdSet = reportedIds.ToHashSet();
        var reportIsComplete = activeSessionIds.Count <= 1000;
        List<SimulationSession> terminalSessions;
        if (dataContext.Database.IsRelational())
        {
            terminalSessions = await dataContext.SimulationSessions
                .FromSqlInterpolated($"""
                    SELECT *
                    FROM "SimulationSessions"
                    WHERE "ComputeWorkerId" = {workerId}
                      AND "State" IN (
                        {(int)SimulationSessionState.Stopped},
                        {(int)SimulationSessionState.Failed},
                        {(int)SimulationSessionState.Expired})
                    ORDER BY "Id"
                    FOR UPDATE
                    """)
                .ToListAsync(Context.ConnectionAborted);
            foreach (var session in terminalSessions)
            {
                await dataContext.Entry(session)
                    .Reference(candidate => candidate.ComputeWorker)
                    .LoadAsync(Context.ConnectionAborted);
                await dataContext.Entry(session)
                    .Collection(candidate => candidate.Commands)
                    .LoadAsync(Context.ConnectionAborted);
            }
        }
        else
        {
            terminalSessions = await dataContext.SimulationSessions
                .Include(session => session.ComputeWorker)
                .Include(session => session.Commands)
                .Where(session => session.ComputeWorkerId == workerId
                    && (session.State == SimulationSessionState.Stopped
                        || session.State == SimulationSessionState.Failed
                        || session.State == SimulationSessionState.Expired))
                .ToListAsync(Context.ConnectionAborted);
        }

        var sessions = terminalSessions
            .Where(session => IsTerminalReconciliationCandidate(
                session,
                reportedIdSet))
            .ToList();

        var queued = new List<WorkerCommand>();
        var reconciled = new List<SimulationSession>();
        foreach (var session in sessions)
        {
            var reportedByWorker = reportedIdSet.Contains(session.Id);
            if (reportIsComplete
                && !reportedByWorker
                && ReconcileAbsentTerminalSession(session, now))
            {
                reconciled.Add(session);
                logger.LogInformation(
                    "Worker {WorkerId} no longer reports terminal session {SessionId} as managed; released its capacity while cleanup remains command-confirmed.",
                    workerId,
                    session.Id);
            }

            if (!NeedsTerminalCleanupAttempt(session, reportedByWorker))
            {
                continue;
            }

            var command = QueueTerminalCleanup(
                session,
                now,
                resourceKnownPresent: reportedByWorker);
            if (command != null)
            {
                dataContext.WorkerCommands.Add(command);
                queued.Add(command);
                logger.LogWarning(
                    "Terminal session {SessionId} on worker {WorkerId} still needs cleanup; queued another bounded attempt.",
                    session.Id,
                    workerId);
            }
        }

        return new HeartbeatReconciliation(queued, reconciled);
    }

    private static bool IsTerminalReconciliationCandidate(
        SimulationSession session,
        HashSet<Guid> reportedSessionIds)
    {
        if (session.State is SimulationSessionState.Failed
            or SimulationSessionState.Expired)
        {
            return true;
        }

        if (session.State != SimulationSessionState.Stopped)
        {
            return false;
        }

        if (reportedSessionIds.Contains(session.Id))
        {
            return true;
        }

        var cleanupCommands = session.Commands
            .Where(command => command.Type == WorkerCommandType.StopSession)
            .OrderByDescending(command => command.Sequence)
            .ToList();
        if (cleanupCommands.Count >= MaximumTerminalCleanupAttempts)
        {
            return false;
        }

        var latest = cleanupCommands.FirstOrDefault();
        return latest == null
            || latest.State is WorkerCommandState.Failed
                or WorkerCommandState.Cancelled;
    }

    private async Task<WorkerCommand> GetOwnedCommand(Guid commandId)
    {
        var workerId = (await GetWorker()).Id;
        return await dataContext.WorkerCommands.SingleOrDefaultAsync(
                command => command.Id == commandId && command.ComputeWorkerId == workerId,
                Context.ConnectionAborted)
            ?? throw new HubException("Assigned worker command not found.");
    }

    private async Task<SimulationSession?> SyncRobotRoster(
        WorkerCommand command,
        JsonElement result,
        DateTime now)
    {
        if (!TryGetProperty(result, "robotIds", out var robotIdsElement)
            || robotIdsElement.ValueKind != JsonValueKind.Array)
        {
            throw new HubException("Fleet command result did not include robotIds.");
        }

        var robotIds = new List<string>();
        foreach (var item in robotIdsElement.EnumerateArray())
        {
            if (item.ValueKind != JsonValueKind.String
                || string.IsNullOrWhiteSpace(item.GetString()))
            {
                throw new HubException("Fleet command result contained an invalid robot roster.");
            }

            var robotId = item.GetString()!.Trim();
            if (!IsRuntimeRobotId(robotId))
            {
                throw new HubException("Fleet command result contained an invalid robot roster.");
            }

            robotIds.Add(robotId);
        }

        if (robotIds.Distinct(StringComparer.Ordinal).Count() != robotIds.Count)
        {
            throw new HubException("Fleet command result contained an invalid robot roster.");
        }

        if (!TryGetProperty(
                command.Payload.RootElement,
                "desiredRobotCount",
                out var desiredRobotCountElement)
            || desiredRobotCountElement.ValueKind != JsonValueKind.Number
            || !desiredRobotCountElement.TryGetInt32(out var desiredRobotCount)
            || desiredRobotCount < 1
            || robotIds.Count != desiredRobotCount)
        {
            throw new HubException(
                "Fleet command result did not match the command's desired robot count.");
        }

        SimulationSession? updatedSession = null;
        if (command.Type == WorkerCommandType.UpdateFleet)
        {
            updatedSession = await dataContext.SimulationSessions
                .Include(session => session.ComputeWorker)
                .SingleAsync(
                    session => session.Id == command.SimulationSessionId,
                    Context.ConnectionAborted);
            if (updatedSession.DesiredRobotCount != desiredRobotCount)
            {
                updatedSession.DesiredRobotCount = desiredRobotCount;
                updatedSession.UpdatedAt = now;
                updatedSession.Revision++;
            }
        }

        var currentRobots = await dataContext.SessionRobots
            .Where(robot => robot.SimulationSessionId == command.SimulationSessionId)
            .ToListAsync(Context.ConnectionAborted);
        if (currentRobots.Count > 0)
        {
            // PostgreSQL checks the unique ordinal index during each update.
            // Move tracked rows aside before assigning the final roster order.
            var usedOrdinals = currentRobots
                .Select(robot => robot.Ordinal)
                .ToHashSet();
            var temporaryOrdinal = int.MinValue;
            foreach (var robot in currentRobots)
            {
                while (usedOrdinals.Contains(temporaryOrdinal))
                {
                    temporaryOrdinal++;
                }

                robot.Ordinal = temporaryOrdinal;
                usedOrdinals.Add(temporaryOrdinal);
                temporaryOrdinal++;
            }

            await dataContext.SaveChangesAsync(Context.ConnectionAborted);
        }

        var byRuntimeId = currentRobots.ToDictionary(
            robot => robot.RuntimeId,
            StringComparer.Ordinal);
        var rosterIds = robotIds.ToHashSet(StringComparer.Ordinal);

        dataContext.SessionRobots.RemoveRange(
            currentRobots.Where(robot => !rosterIds.Contains(robot.RuntimeId)));

        for (var ordinal = 0; ordinal < robotIds.Count; ordinal++)
        {
            var runtimeId = robotIds[ordinal];
            if (!byRuntimeId.TryGetValue(runtimeId, out var robot))
            {
                robot = new SessionRobot
                {
                    SimulationSessionId = command.SimulationSessionId,
                    RuntimeId = runtimeId,
                    Namespace = $"/{runtimeId}",
                    CreatedAt = now
                };
                dataContext.SessionRobots.Add(robot);
            }

            robot.Ordinal = ordinal;
            robot.State = SessionRobotState.Ready;
            robot.UpdatedAt = now;
        }

        return updatedSession;
    }

    private static bool IsRuntimeRobotId(string value)
    {
        if (!value.StartsWith("tb3_", StringComparison.Ordinal)
            || !int.TryParse(value.AsSpan(4), out var ordinal)
            || ordinal < 0)
        {
            return false;
        }

        return value == $"tb3_{ordinal}";
    }

    internal static DateTime InferTaskStartedAt(TaskRun task)
    {
        if (task.StartedAt.HasValue)
        {
            return task.StartedAt.Value;
        }

        var startCommand = task.Commands
            .Where(command => command.Type == WorkerCommandType.StartTask)
            .OrderByDescending(command => command.Sequence)
            .FirstOrDefault();
        return startCommand?.CompletedAt
            ?? startCommand?.AcknowledgedAt
            ?? startCommand?.DispatchedAt
            ?? startCommand?.CreatedAt
            ?? task.CreatedAt;
    }

    private static bool AlignSessionStateWithTask(TaskRun task, DateTime now)
    {
        var session = task.SimulationSession;
        if (session.State >= SimulationSessionState.Stopping)
        {
            return false;
        }

        var nextState = task.State switch
        {
            TaskRunState.Running => SimulationSessionState.Active,
            TaskRunState.Paused => SimulationSessionState.Paused,
            TaskRunState.Completed
                or TaskRunState.Cancelled
                or TaskRunState.Failed => SimulationSessionState.Ready,
            _ => session.State
        };
        if (nextState == session.State)
        {
            return false;
        }

        session.State = nextState;
        session.UpdatedAt = now;
        session.Revision++;
        if (nextState == SimulationSessionState.Active)
        {
            session.StartedAt ??= now;
        }

        return true;
    }

    internal static bool ReleaseSessionAfterConfirmedCancellation(
        SimulationSession session,
        Guid? taskRunId,
        DateTime now)
    {
        if (!taskRunId.HasValue
            || session.State is not (SimulationSessionState.Active
                or SimulationSessionState.Paused)
            || session.TaskRuns.Any(task => task.State < TaskRunState.Completed)
            || !session.TaskRuns.Any(task =>
                task.Id == taskRunId.Value
                && task.State >= TaskRunState.Completed)
            || session.Commands.Any(command =>
                command.Type == WorkerCommandType.CancelTask
                && command.TaskRunId == taskRunId
                && IsInFlight(command.State)))
        {
            return false;
        }

        session.State = SimulationSessionState.Ready;
        session.UpdatedAt = now;
        session.Revision++;
        return true;
    }

    internal static bool FailSessionAfterCancellationFailure(
        TaskRun task,
        string error,
        DateTime now)
    {
        var session = task.SimulationSession;
        if (task.State < TaskRunState.Completed
            || session.State >= SimulationSessionState.Stopping)
        {
            return false;
        }

        session.State = SimulationSessionState.Failed;
        session.FailureReason = Truncate(
            $"The worker could not stop a timed-out task: {error}",
            2000);
        session.StoppedAt ??= now;
        session.UpdatedAt = now;
        session.Revision++;
        return true;
    }

    private static bool CanApplySessionTransition(
        SimulationSessionState current,
        SimulationSessionState next)
    {
        if (current == next)
        {
            return true;
        }

        if ((current == SimulationSessionState.Failed
                || current == SimulationSessionState.Expired)
            && next == SimulationSessionState.Stopped)
        {
            return true;
        }

        if (current >= SimulationSessionState.Stopped)
        {
            return false;
        }

        if (next is SimulationSessionState.Failed or SimulationSessionState.Stopped)
        {
            return true;
        }

        return current == SimulationSessionState.Provisioning
            && next == SimulationSessionState.Ready;
    }

    internal static bool TryParseSessionReportState(
        string state,
        out SimulationSessionState parsed)
    {
        return Enum.TryParse(state, true, out parsed)
               && Enum.IsDefined(parsed)
               && parsed is not SimulationSessionState.Queued
                   and not SimulationSessionState.Expired;
    }

    internal static bool TryParseTaskReportState(
        string state,
        out TaskRunState parsed)
    {
        return Enum.TryParse(state, true, out parsed)
               && Enum.IsDefined(parsed)
               && parsed != TaskRunState.Queued;
    }

    internal static string? FailureReasonForSessionEvent(
        SimulationSession session,
        SimulationSessionState nextState,
        string? reportedReason)
    {
        if (nextState == SimulationSessionState.Failed)
        {
            return Truncate(
                reportedReason ?? "Worker reported a session failure.",
                2000);
        }

        return nextState == SimulationSessionState.Stopped
            ? session.FailureReason
            : null;
    }

    internal static bool IsDuplicateTaskReport(
        TaskRun task,
        TaskRunState nextState,
        TaskEventReport report)
    {
        if (task.State != nextState)
        {
            return false;
        }

        var progress = report.Progress.HasValue
            ? Math.Max(task.Progress, Math.Clamp(report.Progress.Value, 0, 1))
            : task.Progress;
        if (nextState == TaskRunState.Completed)
        {
            progress = 1;
        }

        if (progress != task.Progress)
        {
            return false;
        }

        var error = nextState == TaskRunState.Failed
            ? Truncate(report.Error ?? "Worker reported a task failure.", 4000)
            : null;
        if (!string.Equals(task.Error, error, StringComparison.Ordinal))
        {
            return false;
        }

        if (report.Result.HasValue
            && (task.Result == null
                || !WorkerCommandService.SamePayload(
                    task.Result.RootElement,
                    report.Result.Value)))
        {
            return false;
        }

        if (nextState == TaskRunState.Running && !task.StartedAt.HasValue)
        {
            return false;
        }

        if (nextState >= TaskRunState.Completed && !task.CompletedAt.HasValue)
        {
            return false;
        }

        if (task.SimulationSession.State < SimulationSessionState.Stopping)
        {
            var expectedSessionState = nextState switch
            {
                TaskRunState.Running => SimulationSessionState.Active,
                TaskRunState.Paused => SimulationSessionState.Paused,
                TaskRunState.Completed
                    or TaskRunState.Cancelled
                    or TaskRunState.Failed => SimulationSessionState.Ready,
                _ => task.SimulationSession.State
            };
            if (task.SimulationSession.State != expectedSessionState)
            {
                return false;
            }
        }

        return true;
    }

    internal static IReadOnlyList<WorkerCommand> CancelCommandsAfterFailSafe(
        SimulationSession session,
        DateTime now)
    {
        const string reason =
            "Cancelled because the worker applied its disconnect fail-safe.";
        var cancelled = session.Commands
            .Where(command => IsInFlight(command.State)
                && command.Type is not WorkerCommandType.StopSession
                    and not WorkerCommandType.EmergencyStop
                    and not WorkerCommandType.StopViewer)
            .ToList();
        foreach (var command in cancelled)
        {
            command.State = WorkerCommandState.Cancelled;
            command.LastError = reason;
            command.CompletedAt = now;
            command.UpdatedAt = now;
        }

        return cancelled;
    }

    internal static IReadOnlyList<WorkerCommand> ApplyFailSafeTransition(
        SimulationSession session,
        DateTime now)
    {
        var cancelled = CancelCommandsAfterFailSafe(session, now);
        if (!session.IsEmergencyStopped)
        {
            session.IsEmergencyStopped = true;
            session.UpdatedAt = now;
            session.Revision++;
        }

        return cancelled;
    }

    internal static bool ReconcileAbsentTerminalSession(
        SimulationSession session,
        DateTime now)
    {
        if (!session.ComputeWorkerId.HasValue
            || session.State is not (SimulationSessionState.Failed
                or SimulationSessionState.Expired))
        {
            return false;
        }

        session.State = SimulationSessionState.Stopped;
        session.IsEmergencyStopped = false;
        session.StoppedAt ??= now;
        session.UpdatedAt = now;
        session.Revision++;
        return true;
    }

    internal static bool NeedsTerminalCleanupAttempt(
        SimulationSession session,
        bool reportedByWorker)
    {
        return TerminalCleanupPolicy.NeedsAttempt(session, reportedByWorker);
    }

    internal static WorkerCommand? QueueTerminalCleanup(
        SimulationSession session,
        DateTime now,
        bool resourceKnownPresent = true)
    {
        return TerminalCleanupPolicy.TryQueue(
            session,
            now,
            "sys:heartbeat-cleanup",
            resourceKnownPresent);
    }

    internal static TimeSpan GetTerminalCleanupRetryDelay(int completedAttempts)
    {
        return TerminalCleanupPolicy.GetRetryDelay(completedAttempts);
    }

    private static bool IsSerializationFailure(Exception exception)
    {
        return exception is PostgresException
        {
            SqlState: PostgresErrorCodes.SerializationFailure
        }
            || exception is DbUpdateException
            {
                InnerException: PostgresException
                {
                    SqlState: PostgresErrorCodes.SerializationFailure
                }
            };
    }

    internal static bool IsHeartbeatPersistenceConflict(Exception exception)
    {
        if (exception is DbUpdateConcurrencyException
            || IsSerializationFailure(exception))
        {
            return true;
        }

        var postgres = exception as PostgresException
            ?? (exception as DbUpdateException)?.InnerException as PostgresException;
        return postgres is
        {
            SqlState: PostgresErrorCodes.DeadlockDetected
        }
            || postgres is
            {
                SqlState: PostgresErrorCodes.UniqueViolation,
                ConstraintName: "IX_WorkerCommands_SimulationSessionId_Sequence"
            };
    }

    internal static bool CanApplyTaskTransition(TaskRun task, TaskRunState next)
    {
        var current = task.State;
        if (current >= TaskRunState.Completed)
        {
            return false;
        }

        if (current == next)
        {
            return true;
        }

        if (task.SimulationSession.State >= SimulationSessionState.Stopping
            && next < TaskRunState.Completed)
        {
            return false;
        }

        if (next >= TaskRunState.Completed)
        {
            return true;
        }

        var latestCommand = task.Commands
            .OrderByDescending(command => command.Sequence)
            .FirstOrDefault();
        if (current == TaskRunState.Paused && next == TaskRunState.Running)
        {
            return latestCommand?.Type == WorkerCommandType.ResumeTask;
        }

        if (current == TaskRunState.Running && next == TaskRunState.Paused)
        {
            return latestCommand?.Type == WorkerCommandType.PauseTask;
        }

        return current switch
        {
            TaskRunState.Queued => next is TaskRunState.Accepted or TaskRunState.Running,
            TaskRunState.Accepted => next is TaskRunState.Running or TaskRunState.Paused,
            TaskRunState.Running or TaskRunState.Paused =>
                next == TaskRunState.Cancelling
                && latestCommand?.Type == WorkerCommandType.CancelTask,
            _ => false
        };
    }

    private async Task<IReadOnlyList<TaskRun>> FinalizeSessionResources(
        Guid sessionId,
        bool failed,
        string? failureReason,
        DateTime now)
    {
        var tasks = await dataContext.TaskRuns
            .Where(task => task.SimulationSessionId == sessionId
                && task.State < TaskRunState.Completed)
            .ToListAsync(Context.ConnectionAborted);
        foreach (var task in tasks)
        {
            task.State = failed ? TaskRunState.Failed : TaskRunState.Cancelled;
            task.OutcomeState = failed
                ? TaskOutcomeState.Failed
                : TaskOutcomeState.Cancelled;
            task.Error = failed
                ? failureReason ?? "The simulation session stopped unexpectedly."
                : null;
            task.OutcomeReason = task.Error;
            task.CompletedAt = now;
            task.UpdatedAt = now;
        }

        var robots = await dataContext.SessionRobots
            .Where(robot => robot.SimulationSessionId == sessionId
                && robot.State != SessionRobotState.Removed)
            .ToListAsync(Context.ConnectionAborted);
        foreach (var robot in robots)
        {
            robot.State = SessionRobotState.Removed;
            robot.UpdatedAt = now;
        }

        var leases = await dataContext.ViewerLeases
            .Where(lease => lease.SimulationSessionId == sessionId
                && !lease.RevokedAt.HasValue)
            .ToListAsync(Context.ConnectionAborted);
        foreach (var lease in leases)
        {
            lease.RevokedAt = now;
        }

        return tasks;
    }

    private static bool TryGetProperty(
        JsonElement element,
        string name,
        out JsonElement value)
    {
        if (element.ValueKind == JsonValueKind.Object)
        {
            foreach (var property in element.EnumerateObject())
            {
                if (property.Name.Equals(name, StringComparison.OrdinalIgnoreCase))
                {
                    value = property.Value;
                    return true;
                }
            }
        }

        value = default;
        return false;
    }

    internal static bool TryValidateTaskCancellationCompletion(
        WorkerCommand command,
        JsonElement result)
    {
        return command.Type == WorkerCommandType.CancelTask
            && command.TaskRunId.HasValue
            && TryGetProperty(
                result,
                "taskCancellationConfirmed",
                out var confirmed)
            && confirmed.ValueKind == JsonValueKind.True
            && TryGetProperty(result, "taskRunId", out var taskRunId)
            && taskRunId.ValueKind == JsonValueKind.String
            && Guid.TryParse(taskRunId.GetString(), out var reportedTaskRunId)
            && reportedTaskRunId == command.TaskRunId.Value;
    }

    internal static bool TryValidateViewerStopCompletion(
        WorkerCommand command,
        JsonElement result)
    {
        return command.Type == WorkerCommandType.StopViewer
            && TryGetProperty(command.Payload.RootElement, "leaseId", out var expectedLease)
            && expectedLease.ValueKind == JsonValueKind.String
            && Guid.TryParse(expectedLease.GetString(), out var expectedLeaseId)
            && TryGetProperty(result, "sessionId", out var reportedSession)
            && reportedSession.ValueKind == JsonValueKind.String
            && Guid.TryParse(reportedSession.GetString(), out var reportedSessionId)
            && reportedSessionId == command.SimulationSessionId
            && TryGetProperty(result, "leaseId", out var reportedLease)
            && reportedLease.ValueKind == JsonValueKind.String
            && Guid.TryParse(reportedLease.GetString(), out var reportedLeaseId)
            && reportedLeaseId == expectedLeaseId
            && TryGetProperty(result, "stopped", out var stopped)
            && stopped.ValueKind is JsonValueKind.True or JsonValueKind.False;
    }

    private static void ApplyWorkerMetadata(
        ComputeWorker worker,
        string? imageVersion,
        JsonElement? capabilities)
    {
        worker.ImageVersion = string.IsNullOrWhiteSpace(imageVersion)
            ? worker.ImageVersion
            : Truncate(imageVersion.Trim(), 200);

        if (capabilities.HasValue
            && capabilities.Value.ValueKind is not JsonValueKind.Undefined
            and not JsonValueKind.Null)
        {
            if (capabilities.Value.ValueKind != JsonValueKind.Object)
            {
                throw new HubException("Capabilities must be a JSON object.");
            }

            worker.Capabilities = ControlPlaneJson.ToDocument(capabilities);
        }
    }

    private async Task PublishCommandUpdate(WorkerCommand command)
    {
        await sessionHubContext.Clients
            .Group(ControlPlaneGroups.Session(command.SimulationSessionId))
            .SendAsync("SessionEvent", new
            {
                sessionId = command.SimulationSessionId,
                commandId = command.Id,
                eventType = "CommandUpdated",
                state = command.State.ToString(),
                error = command.LastError,
                timestamp = command.UpdatedAt
            }, Context.ConnectionAborted);
    }

    private static bool IsTerminal(WorkerCommandState state)
    {
        return state is WorkerCommandState.Completed
            or WorkerCommandState.Failed
            or WorkerCommandState.Cancelled;
    }

    private static bool IsInFlight(WorkerCommandState state)
    {
        return state is WorkerCommandState.Pending
            or WorkerCommandState.Dispatched
            or WorkerCommandState.Acknowledged
            or WorkerCommandState.Running;
    }

    private sealed record HeartbeatReconciliation(
        IReadOnlyList<WorkerCommand> CleanupCommands,
        IReadOnlyList<SimulationSession> ReconciledSessions)
    {
        public static HeartbeatReconciliation Empty { get; } = new(
            Array.Empty<WorkerCommand>(),
            Array.Empty<SimulationSession>());
    }

    private static WorkerRegistrationResponse ToRegistrationResponse(
        ComputeWorker worker,
        DateTime serverTime)
    {
        return new WorkerRegistrationResponse(
            worker.Id,
            worker.Name,
            worker.State.ToString(),
            worker.MaxConcurrentSessions,
            serverTime);
    }

    private static WorkerCommandEnvelope ToEnvelope(WorkerCommand command)
    {
        return new WorkerCommandEnvelope(
            command.Id,
            command.SimulationSessionId,
            command.Type.ToString(),
            command.IdempotencyKey,
            command.CorrelationId,
            command.Sequence,
            ControlPlaneJson.ToElement(command.Payload),
            command.CreatedAt);
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

    private static string Truncate(string value, int maxLength)
    {
        return value.Length <= maxLength ? value : value[..maxLength];
    }
}
