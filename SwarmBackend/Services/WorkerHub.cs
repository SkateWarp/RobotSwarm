using System.Collections.Concurrent;
using System.Data;
using System.Security.Claims;
using System.Text.Json;
using Microsoft.AspNetCore.Authorization;
using Microsoft.AspNetCore.SignalR;
using Microsoft.EntityFrameworkCore;
using SwarmBackend.Entities;
using SwarmBackend.Helpers;
using SwarmBackend.Models;

namespace SwarmBackend.Services;

[Authorize(AuthenticationSchemes = WorkerCredentialDefaults.AuthenticationScheme)]
public class WorkerHub(
    DataContext dataContext,
    IHubContext<SessionHub> sessionHubContext,
    ILogger<WorkerHub> logger) : Hub
{
    private static readonly ConcurrentDictionary<Guid, string> ActiveConnections = new();

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
        var worker = await GetWorker();
        ApplyWorkerMetadata(worker, request.ImageVersion, request.Capabilities);

        var now = DateTime.UtcNow;
        if (worker.State != ComputeWorkerState.Draining)
        {
            worker.State = ComputeWorkerState.Online;
        }

        worker.LastHeartbeatAt = now;
        worker.UpdatedAt = now;
        await dataContext.SaveChangesAsync(Context.ConnectionAborted);

        logger.LogInformation("Compute worker {WorkerId} registered from connection {ConnectionId}.",
            worker.Id, Context.ConnectionId);
        return ToRegistrationResponse(worker, now);
    }

    public async Task<WorkerRegistrationResponse> Heartbeat(WorkerHeartbeatRequest request)
    {
        var worker = await GetWorker();
        ApplyWorkerMetadata(worker, request.ImageVersion, request.Capabilities);

        var now = DateTime.UtcNow;
        if (worker.State != ComputeWorkerState.Draining)
        {
            worker.State = ComputeWorkerState.Online;
        }

        worker.LastHeartbeatAt = now;
        worker.UpdatedAt = now;
        await dataContext.SaveChangesAsync(Context.ConnectionAborted);
        return ToRegistrationResponse(worker, now);
    }

    public async Task<IReadOnlyList<WorkerCommandEnvelope>> PullPendingCommands(int maxCount = 25)
    {
        var workerId = (await GetWorker()).Id;
        maxCount = Math.Clamp(maxCount, 1, 100);

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
        foreach (var command in commands.Where(command => command.State == WorkerCommandState.Pending))
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

    public async Task AcknowledgeCommand(Guid commandId)
    {
        var command = await GetOwnedCommand(commandId);
        if (IsTerminal(command.State)
            || command.State is WorkerCommandState.Acknowledged or WorkerCommandState.Running)
        {
            return;
        }

        var now = DateTime.UtcNow;
        command.State = WorkerCommandState.Acknowledged;
        command.AcknowledgedAt = now;
        command.UpdatedAt = now;
        await dataContext.SaveChangesAsync(Context.ConnectionAborted);
        await PublishCommandUpdate(command);
    }

    public async Task MarkCommandRunning(Guid commandId)
    {
        var command = await GetOwnedCommand(commandId);
        if (IsTerminal(command.State) || command.State == WorkerCommandState.Running)
        {
            return;
        }

        var now = DateTime.UtcNow;
        command.State = WorkerCommandState.Running;
        command.AcknowledgedAt ??= now;
        command.UpdatedAt = now;
        await dataContext.SaveChangesAsync(Context.ConnectionAborted);
        await PublishCommandUpdate(command);
    }

    public async Task CompleteCommand(WorkerCommandCompletionRequest request)
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

        command.State = WorkerCommandState.Completed;
        command.Result = request.Result.HasValue
            ? ControlPlaneJson.ToDocument(request.Result)
            : null;
        command.LastError = null;
        command.AcknowledgedAt ??= now;
        command.CompletedAt = now;
        command.UpdatedAt = now;

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

    public async Task FailCommand(WorkerCommandFailureRequest request)
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
                task.CompletedAt = now;
                sessionChanged = AlignSessionStateWithTask(task, now);
            }

            task.Error = error;
            task.UpdatedAt = now;
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
        var session = await dataContext.SimulationSessions
            .Include(candidate => candidate.ComputeWorker)
            .SingleOrDefaultAsync(
                candidate => candidate.Id == report.SessionId
                    && candidate.ComputeWorkerId == workerId,
                Context.ConnectionAborted)
            ?? throw new HubException("Assigned simulation session not found.");
        if (session.State >= SimulationSessionState.Stopped)
        {
            throw new HubException("The simulation session is already closed.");
        }

        var now = DateTime.UtcNow;
        if (!session.IsEmergencyStopped)
        {
            session.IsEmergencyStopped = true;
            session.UpdatedAt = now;
            session.Revision++;
            await dataContext.SaveChangesAsync(Context.ConnectionAborted);
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
        var session = await dataContext.SimulationSessions
            .Include(candidate => candidate.ComputeWorker)
            .SingleOrDefaultAsync(
                candidate => candidate.Id == report.SessionId
                    && candidate.ComputeWorkerId == workerId,
                Context.ConnectionAborted);

        if (session == null)
        {
            throw new HubException("Assigned simulation session not found.");
        }

        if (!Enum.TryParse<SimulationSessionState>(report.State, true, out var nextState)
            || nextState is SimulationSessionState.Queued or SimulationSessionState.Expired)
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

        var failedSessionCleanup =
            session.State == SimulationSessionState.Failed
            && nextState == SimulationSessionState.Stopped;
        var now = DateTime.UtcNow;
        var failureReason = nextState == SimulationSessionState.Failed
            ? Truncate(report.FailureReason ?? "Worker reported a session failure.", 2000)
            : failedSessionCleanup
                ? session.FailureReason
                : null;
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
        var workerId = (await GetWorker()).Id;
        var task = await dataContext.TaskRuns
            .Include(candidate => candidate.Commands)
            .Include(candidate => candidate.SimulationSession)
            .ThenInclude(session => session.ComputeWorker)
            .SingleOrDefaultAsync(
                candidate => candidate.Id == report.TaskRunId
                    && candidate.SimulationSessionId == report.SessionId
                    && candidate.SimulationSession.ComputeWorkerId == workerId,
                Context.ConnectionAborted);

        if (task == null)
        {
            throw new HubException("Assigned task run not found.");
        }

        if (!Enum.TryParse<TaskRunState>(report.State, true, out var nextState)
            || nextState == TaskRunState.Queued)
        {
            throw new HubException("Unsupported task run state.");
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

        if (report.Progress.HasValue && !double.IsFinite(report.Progress.Value))
        {
            throw new HubException("Task progress must be finite.");
        }

        var now = DateTime.UtcNow;
        task.State = nextState;
        task.Progress = report.Progress.HasValue
            ? Math.Max(task.Progress, Math.Clamp(report.Progress.Value, 0, 1))
            : task.Progress;
        task.Error = nextState == TaskRunState.Failed
            ? Truncate(report.Error ?? "Worker reported a task failure.", 4000)
            : null;
        if (report.Result.HasValue)
        {
            task.Result = ControlPlaneJson.ToDocument(report.Result);
        }

        task.UpdatedAt = now;
        if (nextState == TaskRunState.Running)
        {
            task.StartedAt ??= now;
        }

        if (nextState >= TaskRunState.Completed)
        {
            task.CompletedAt ??= now;
            if (nextState == TaskRunState.Completed)
            {
                task.Progress = 1;
            }
        }

        var sessionChanged = AlignSessionStateWithTask(task, now);
        await dataContext.SaveChangesAsync(Context.ConnectionAborted);

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

    private static bool CanApplySessionTransition(
        SimulationSessionState current,
        SimulationSessionState next)
    {
        if (current == next)
        {
            return true;
        }

        if (current == SimulationSessionState.Failed
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

    private static bool CanApplyTaskTransition(TaskRun task, TaskRunState next)
    {
        var current = task.State;
        if (current == next)
        {
            return true;
        }

        if (current >= TaskRunState.Completed)
        {
            return false;
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
            task.Error = failed
                ? failureReason ?? "The simulation session stopped unexpectedly."
                : null;
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
            task.UpdatedAt,
            task.StartedAt,
            task.CompletedAt);
    }

    private static string Truncate(string value, int maxLength)
    {
        return value.Length <= maxLength ? value : value[..maxLength];
    }
}
