using System.Data;
using System.Security.Cryptography;
using System.Text;
using System.Text.Json;
using Microsoft.AspNetCore.SignalR;
using Microsoft.AspNetCore.WebUtilities;
using Microsoft.EntityFrameworkCore;
using Npgsql;
using SwarmBackend.Entities;
using SwarmBackend.Helpers;
using SwarmBackend.Models;
using SwarmBackend.Services;

namespace SwarmBackend.Routes;

public static class SessionControlRoute
{
    public static RouteGroupBuilder MapSessionControl(this RouteGroupBuilder group)
    {
        group.RequireAuthorization();

        group.MapPatch("/{id:guid}/fleet", UpdateFleet)
            .Produces<FleetUpdateResponse>(StatusCodes.Status202Accepted);
        group.MapGet("/{id:guid}/robots", GetRobots)
            .Produces<IEnumerable<SessionRobotResponse>>();
        group.MapGet("/{id:guid}/tasks", GetTasks)
            .Produces<IEnumerable<TaskRunResponse>>();
        group.MapPost("/{id:guid}/tasks", CreateTask)
            .Produces<TaskCommandResponse>(StatusCodes.Status202Accepted);
        group.MapPost("/{id:guid}/tasks/{taskId:guid}/pause", PauseTask)
            .Produces<TaskCommandResponse>(StatusCodes.Status202Accepted);
        group.MapPost("/{id:guid}/tasks/{taskId:guid}/resume", ResumeTask)
            .Produces<TaskCommandResponse>(StatusCodes.Status202Accepted);
        group.MapPost("/{id:guid}/tasks/{taskId:guid}/cancel", CancelTask)
            .Produces<TaskCommandResponse>(StatusCodes.Status202Accepted);
        group.MapPost("/{id:guid}/emergency-stop", EmergencyStop)
            .Produces<EmergencyStopResponse>(StatusCodes.Status202Accepted);
        group.MapPost("/{id:guid}/reset-emergency-stop", ResetEmergencyStop)
            .Produces<EmergencyStopResponse>(StatusCodes.Status202Accepted);
        group.MapPost("/{id:guid}/viewer-lease", CreateViewerLease)
            .Produces<ViewerLeaseResponse>(StatusCodes.Status201Created);

        return group;
    }

    private static async Task<IResult> UpdateFleet(
        Guid id,
        UpdateFleetRequest request,
        HttpContext context,
        DataContext dataContext,
        WorkerCommandService commandService,
        IConfiguration configuration,
        IHubContext<WorkerHub> workerHub,
        IHubContext<SessionHub> sessionHub,
        CancellationToken cancellationToken)
    {
        if (!TryGetAccountId(context, out var accountId))
        {
            return Results.Unauthorized();
        }

        if (!TryGetIdempotencyKey(context, out var idempotencyKey, out var error))
        {
            return error!;
        }

        var maxRobotsPerSession = SessionLimits.GetMaxRobotsPerSession(configuration);
        if (request.RobotCount < 1 || request.RobotCount > maxRobotsPerSession)
        {
            return Results.ValidationProblem(new Dictionary<string, string[]>
            {
                [nameof(request.RobotCount)] =
                    new[] { $"RobotCount must be between 1 and {maxRobotsPerSession}." }
            });
        }

        await using var transaction = await dataContext.Database.BeginTransactionAsync(
            IsolationLevel.Serializable,
            cancellationToken);
        var session = await FindOwnedSession(dataContext, id, accountId, cancellationToken);
        if (session == null)
        {
            return Results.NotFound();
        }

        using var payload = JsonSerializer.SerializeToDocument(new
        {
            desiredRobotCount = request.RobotCount
        });
        var existing = await commandService.Find(id, idempotencyKey, cancellationToken);
        if (existing != null)
        {
            if (existing.Type != WorkerCommandType.UpdateFleet
                || !WorkerCommandService.SamePayload(existing.Payload, payload))
            {
                return IdempotencyConflict();
            }

            return Results.Accepted(
                $"/api/sessions/{id}",
                new FleetUpdateResponse(ToSessionResponse(session), WorkerCommandService.ToResponse(existing)));
        }

        if (await HasPendingEmergencyTransition(dataContext, id, cancellationToken))
        {
            return EmergencyTransitionConflict();
        }

        if (session.State != SimulationSessionState.Ready
            || !session.ComputeWorkerId.HasValue
            || session.IsEmergencyStopped)
        {
            return Results.Conflict(new { message = "The session cannot update its fleet in the current state." });
        }

        var hasActiveTask = await dataContext.TaskRuns.AnyAsync(
            task => task.SimulationSessionId == id
                && task.State < TaskRunState.Completed,
            cancellationToken);
        if (hasActiveTask)
        {
            return Results.Conflict(new
            {
                message = "Finish or cancel the current task before replacing the fleet."
            });
        }

        var workerMaximum = session.ComputeWorker == null
            ? 0
            : WorkerCapabilities.GetMaxRobotsPerSession(session.ComputeWorker);
        if (workerMaximum < request.RobotCount)
        {
            return Results.Conflict(new
            {
                message =
                    $"The assigned worker supports at most {workerMaximum} robots in this session."
            });
        }

        if (session.DesiredRobotCount == request.RobotCount)
        {
            return Results.Conflict(new { message = "The fleet already has the requested desired size." });
        }

        try
        {
            var (command, _) = await commandService.Queue(
                session,
                WorkerCommandType.UpdateFleet,
                idempotencyKey,
                JsonDocument.Parse(payload.RootElement.GetRawText()),
                taskRun: null,
                cancellationToken);
            await dataContext.SaveChangesAsync(cancellationToken);
            await transaction.CommitAsync(cancellationToken);

            await NotifyCommand(workerHub, sessionHub, session, command, cancellationToken);
            var response = new FleetUpdateResponse(
                ToSessionResponse(session),
                WorkerCommandService.ToResponse(command));
            await sessionHub.Clients.Group(ControlPlaneGroups.Session(id))
                .SendAsync("SessionUpdated", response.Session, cancellationToken);
            return Results.Accepted($"/api/sessions/{id}", response);
        }
        catch (Exception exception) when (IsCommandConflict(exception))
        {
            return RetryConflict();
        }
    }

    private static async Task<IResult> GetRobots(
        Guid id,
        HttpContext context,
        DataContext dataContext,
        CancellationToken cancellationToken)
    {
        if (!TryGetAccountId(context, out var accountId))
        {
            return Results.Unauthorized();
        }

        var ownsSession = await dataContext.SimulationSessions
            .AsNoTracking()
            .AnyAsync(session => session.Id == id && session.AccountId == accountId, cancellationToken);
        if (!ownsSession)
        {
            return Results.NotFound();
        }

        var robots = await dataContext.SessionRobots
            .AsNoTracking()
            .Where(robot => robot.SimulationSessionId == id)
            .OrderBy(robot => robot.Ordinal)
            .Select(robot => new SessionRobotResponse(
                robot.Id,
                robot.Ordinal,
                robot.RuntimeId,
                robot.Namespace,
                robot.Role,
                robot.State.ToString(),
                robot.CreatedAt,
                robot.UpdatedAt))
            .ToListAsync(cancellationToken);
        return Results.Ok(robots);
    }

    private static async Task<IResult> GetTasks(
        Guid id,
        HttpContext context,
        DataContext dataContext,
        CancellationToken cancellationToken)
    {
        if (!TryGetAccountId(context, out var accountId))
        {
            return Results.Unauthorized();
        }

        var ownsSession = await dataContext.SimulationSessions
            .AsNoTracking()
            .AnyAsync(session => session.Id == id && session.AccountId == accountId, cancellationToken);
        if (!ownsSession)
        {
            return Results.NotFound();
        }

        var tasks = await dataContext.TaskRuns
            .AsNoTracking()
            .Where(task => task.SimulationSessionId == id)
            .OrderByDescending(task => task.CreatedAt)
            .ToListAsync(cancellationToken);
        return Results.Ok(tasks.Select(ToTaskResponse));
    }

    private static async Task<IResult> CreateTask(
        Guid id,
        CreateTaskRunRequest request,
        HttpContext context,
        DataContext dataContext,
        WorkerCommandService commandService,
        IConfiguration configuration,
        IHubContext<WorkerHub> workerHub,
        IHubContext<SessionHub> sessionHub,
        CancellationToken cancellationToken)
    {
        if (!TryGetAccountId(context, out var accountId))
        {
            return Results.Unauthorized();
        }

        if (!TryGetIdempotencyKey(context, out var idempotencyKey, out var error))
        {
            return error!;
        }

        if (!TryParseTaskType(request.Type, out var taskType))
        {
            return Results.ValidationProblem(new Dictionary<string, string[]>
            {
                [nameof(request.Type)] =
                    new[] { "Type must be FollowLeader, Figure, or CollaborativeTransport." }
            });
        }

        if (request.Parameters.ValueKind != JsonValueKind.Object
            || request.Parameters.GetRawText().Length > 65_536)
        {
            return Results.ValidationProblem(new Dictionary<string, string[]>
            {
                [nameof(request.Parameters)] =
                    new[] { "Parameters must be a JSON object no larger than 64 KiB." }
            });
        }

        if (!TaskParameterValidator.TryValidate(
                taskType,
                request.Parameters,
                out var parameterError))
        {
            return Results.ValidationProblem(new Dictionary<string, string[]>
            {
                [nameof(request.Parameters)] = new[] { parameterError! }
            });
        }

        await using var transaction = await dataContext.Database.BeginTransactionAsync(
            IsolationLevel.Serializable,
            cancellationToken);
        var session = await FindOwnedSession(dataContext, id, accountId, cancellationToken);
        if (session == null)
        {
            return Results.NotFound();
        }

        var existing = await commandService.Find(id, idempotencyKey, cancellationToken);
        if (existing != null)
        {
            var sameRequest = existing.Type == WorkerCommandType.StartTask
                && existing.TaskRun?.Type == taskType
                && WorkerCommandService.SamePayload(
                    existing.TaskRun.Parameters.RootElement,
                    request.Parameters);
            if (!sameRequest || existing.TaskRun == null)
            {
                return IdempotencyConflict();
            }

            return Results.Accepted(
                $"/api/sessions/{id}/tasks/{existing.TaskRun.Id}",
                new TaskCommandResponse(
                    ToTaskResponse(existing.TaskRun),
                    WorkerCommandService.ToResponse(existing)));
        }

        if (await HasPendingEmergencyTransition(dataContext, id, cancellationToken))
        {
            return EmergencyTransitionConflict();
        }

        if (await HasPendingTaskCancellation(dataContext, id, cancellationToken))
        {
            return Results.Conflict(new
            {
                message = "Wait for the previous task cancellation to finish before starting another task."
            });
        }

        if (!CanControl(session) || session.IsEmergencyStopped)
        {
            return Results.Conflict(new { message = "The session cannot start a task in the current state." });
        }

        var requiresTransportEvidence = configuration.GetValue(
            "Tasks:RequireCollaborativeTransportEvidence",
            false);
        if (taskType == SwarmTaskRunType.CollaborativeTransport
            && requiresTransportEvidence
            && (session.ComputeWorker == null
                || !WorkerCapabilities.SupportsCollaborativeTransportEvidence(
                    session.ComputeWorker)))
        {
            return Results.Conflict(new
            {
                message =
                    "The assigned worker has not advertised the required collaborative-transport evidence contract."
            });
        }

        var hasActiveTask = await dataContext.TaskRuns.AnyAsync(
            task => task.SimulationSessionId == id && task.State < TaskRunState.Completed,
            cancellationToken);
        if (hasActiveTask)
        {
            return Results.Conflict(new { message = "The session already has an active task." });
        }

        var now = DateTime.UtcNow;
        var task = new TaskRun
        {
            SimulationSessionId = id,
            SimulationSession = session,
            Type = taskType,
            State = TaskRunState.Queued,
            Parameters = JsonDocument.Parse(request.Parameters.GetRawText()),
            CreatedAt = now,
            UpdatedAt = now
        };
        dataContext.TaskRuns.Add(task);
        using var payload = JsonSerializer.SerializeToDocument(new
        {
            taskRunId = task.Id,
            taskType = taskType.ToString(),
            parameters = request.Parameters
        });

        try
        {
            var (command, _) = await commandService.Queue(
                session,
                WorkerCommandType.StartTask,
                idempotencyKey,
                JsonDocument.Parse(payload.RootElement.GetRawText()),
                task,
                cancellationToken);
            task.CommandRevision = command.Sequence;
            await dataContext.SaveChangesAsync(cancellationToken);
            await transaction.CommitAsync(cancellationToken);

            await NotifyCommand(workerHub, sessionHub, session, command, cancellationToken);
            var response = new TaskCommandResponse(
                ToTaskResponse(task),
                WorkerCommandService.ToResponse(command));
            await sessionHub.Clients.Group(ControlPlaneGroups.Session(id))
                .SendAsync("TaskUpdated", response.Task, cancellationToken);
            return Results.Accepted($"/api/sessions/{id}/tasks/{task.Id}", response);
        }
        catch (Exception exception) when (IsCommandConflict(exception))
        {
            return RetryConflict();
        }
    }

    private static Task<IResult> PauseTask(
        Guid id,
        Guid taskId,
        HttpContext context,
        DataContext dataContext,
        WorkerCommandService commandService,
        IHubContext<WorkerHub> workerHub,
        IHubContext<SessionHub> sessionHub,
        CancellationToken cancellationToken)
    {
        return ChangeTaskState(
            id,
            taskId,
            WorkerCommandType.PauseTask,
            context,
            dataContext,
            commandService,
            workerHub,
            sessionHub,
            cancellationToken);
    }

    private static Task<IResult> ResumeTask(
        Guid id,
        Guid taskId,
        HttpContext context,
        DataContext dataContext,
        WorkerCommandService commandService,
        IHubContext<WorkerHub> workerHub,
        IHubContext<SessionHub> sessionHub,
        CancellationToken cancellationToken)
    {
        return ChangeTaskState(
            id,
            taskId,
            WorkerCommandType.ResumeTask,
            context,
            dataContext,
            commandService,
            workerHub,
            sessionHub,
            cancellationToken);
    }

    private static Task<IResult> CancelTask(
        Guid id,
        Guid taskId,
        HttpContext context,
        DataContext dataContext,
        WorkerCommandService commandService,
        IHubContext<WorkerHub> workerHub,
        IHubContext<SessionHub> sessionHub,
        CancellationToken cancellationToken)
    {
        return ChangeTaskState(
            id,
            taskId,
            WorkerCommandType.CancelTask,
            context,
            dataContext,
            commandService,
            workerHub,
            sessionHub,
            cancellationToken);
    }

    private static async Task<IResult> ChangeTaskState(
        Guid sessionId,
        Guid taskId,
        WorkerCommandType commandType,
        HttpContext context,
        DataContext dataContext,
        WorkerCommandService commandService,
        IHubContext<WorkerHub> workerHub,
        IHubContext<SessionHub> sessionHub,
        CancellationToken cancellationToken)
    {
        if (!TryGetAccountId(context, out var accountId))
        {
            return Results.Unauthorized();
        }

        if (!TryGetIdempotencyKey(context, out var idempotencyKey, out var error))
        {
            return error!;
        }

        await using var transaction = await dataContext.Database.BeginTransactionAsync(
            IsolationLevel.Serializable,
            cancellationToken);
        var session = await FindOwnedSession(dataContext, sessionId, accountId, cancellationToken);
        if (session == null)
        {
            return Results.NotFound();
        }

        var task = await dataContext.TaskRuns.SingleOrDefaultAsync(
            candidate => candidate.Id == taskId && candidate.SimulationSessionId == sessionId,
            cancellationToken);
        if (task == null)
        {
            return Results.NotFound();
        }

        using var payload = JsonSerializer.SerializeToDocument(new { taskRunId = taskId });
        var existing = await commandService.Find(sessionId, idempotencyKey, cancellationToken);
        if (existing != null)
        {
            if (existing.Type != commandType
                || existing.TaskRunId != taskId
                || !WorkerCommandService.SamePayload(existing.Payload, payload))
            {
                return IdempotencyConflict();
            }

            return Results.Accepted(
                $"/api/sessions/{sessionId}/tasks/{taskId}",
                new TaskCommandResponse(ToTaskResponse(task), WorkerCommandService.ToResponse(existing)));
        }

        if (await HasPendingEmergencyTransition(
                dataContext,
                sessionId,
                cancellationToken))
        {
            return EmergencyTransitionConflict();
        }

        if (!CanControl(session))
        {
            return Results.Conflict(new { message = "The session cannot control tasks in the current state." });
        }

        var legal = commandType switch
        {
            WorkerCommandType.PauseTask => task.State == TaskRunState.Running,
            WorkerCommandType.ResumeTask => task.State == TaskRunState.Paused
                && !session.IsEmergencyStopped,
            WorkerCommandType.CancelTask => task.State < TaskRunState.Completed
                && task.State != TaskRunState.Cancelling,
            _ => false
        };
        if (!legal)
        {
            return Results.Conflict(new { message = "The task cannot make this transition." });
        }

        try
        {
            var (command, _) = await commandService.Queue(
                session,
                commandType,
                idempotencyKey,
                JsonDocument.Parse(payload.RootElement.GetRawText()),
                task,
                cancellationToken);
            task.CommandRevision = command.Sequence;
            task.UpdatedAt = DateTime.UtcNow;
            await dataContext.SaveChangesAsync(cancellationToken);
            await transaction.CommitAsync(cancellationToken);

            await NotifyCommand(workerHub, sessionHub, session, command, cancellationToken);
            var response = new TaskCommandResponse(
                ToTaskResponse(task),
                WorkerCommandService.ToResponse(command));
            await sessionHub.Clients.Group(ControlPlaneGroups.Session(sessionId))
                .SendAsync("TaskUpdated", response.Task, cancellationToken);
            return Results.Accepted($"/api/sessions/{sessionId}/tasks/{taskId}", response);
        }
        catch (Exception exception) when (IsCommandConflict(exception))
        {
            return RetryConflict();
        }
    }

    private static Task<IResult> EmergencyStop(
        Guid id,
        HttpContext context,
        DataContext dataContext,
        WorkerCommandService commandService,
        IHubContext<WorkerHub> workerHub,
        IHubContext<SessionHub> sessionHub,
        CancellationToken cancellationToken)
    {
        return ChangeEmergencyStop(
            id,
            activate: true,
            context,
            dataContext,
            commandService,
            workerHub,
            sessionHub,
            cancellationToken);
    }

    private static Task<IResult> ResetEmergencyStop(
        Guid id,
        HttpContext context,
        DataContext dataContext,
        WorkerCommandService commandService,
        IHubContext<WorkerHub> workerHub,
        IHubContext<SessionHub> sessionHub,
        CancellationToken cancellationToken)
    {
        return ChangeEmergencyStop(
            id,
            activate: false,
            context,
            dataContext,
            commandService,
            workerHub,
            sessionHub,
            cancellationToken);
    }

    private static async Task<IResult> ChangeEmergencyStop(
        Guid id,
        bool activate,
        HttpContext context,
        DataContext dataContext,
        WorkerCommandService commandService,
        IHubContext<WorkerHub> workerHub,
        IHubContext<SessionHub> sessionHub,
        CancellationToken cancellationToken)
    {
        if (!TryGetAccountId(context, out var accountId))
        {
            return Results.Unauthorized();
        }

        if (!TryGetIdempotencyKey(context, out var idempotencyKey, out var error))
        {
            return error!;
        }

        await using var transaction = await dataContext.Database.BeginTransactionAsync(
            IsolationLevel.Serializable,
            cancellationToken);
        var session = await FindOwnedSession(dataContext, id, accountId, cancellationToken);
        if (session == null)
        {
            return Results.NotFound();
        }

        var commandType = activate
            ? WorkerCommandType.EmergencyStop
            : WorkerCommandType.ResetEmergencyStop;
        using var payload = JsonDocument.Parse("{}");
        var existing = await commandService.Find(id, idempotencyKey, cancellationToken);
        if (existing != null)
        {
            if (existing.Type != commandType
                || !WorkerCommandService.SamePayload(existing.Payload, payload))
            {
                return IdempotencyConflict();
            }

            return Results.Accepted(
                $"/api/sessions/{id}",
                new EmergencyStopResponse(
                    id,
                    session.IsEmergencyStopped,
                    WorkerCommandService.ToResponse(existing)));
        }

        var canStop = session.ComputeWorkerId.HasValue
            && session.State > SimulationSessionState.Queued
            && session.State < SimulationSessionState.Stopped;
        if (!canStop || session.IsEmergencyStopped == activate)
        {
            return Results.Conflict(new { message = "The emergency-stop state cannot make this transition." });
        }

        var emergencyCommandPending = await HasPendingEmergencyTransition(
            dataContext,
            id,
            cancellationToken);
        if (emergencyCommandPending)
        {
            return Results.Conflict(new
            {
                message = "An emergency-stop command is already being applied."
            });
        }

        try
        {
            var (command, _) = await commandService.Queue(
                session,
                commandType,
                idempotencyKey,
                JsonDocument.Parse("{}"),
                taskRun: null,
                cancellationToken);
            await dataContext.SaveChangesAsync(cancellationToken);
            await transaction.CommitAsync(cancellationToken);

            await NotifyCommand(workerHub, sessionHub, session, command, cancellationToken);
            var response = new EmergencyStopResponse(
                id,
                session.IsEmergencyStopped,
                WorkerCommandService.ToResponse(command));
            return Results.Accepted($"/api/sessions/{id}", response);
        }
        catch (Exception exception) when (IsCommandConflict(exception))
        {
            return RetryConflict();
        }
    }

    private static async Task<IResult> CreateViewerLease(
        Guid id,
        CreateViewerLeaseRequest request,
        HttpContext context,
        DataContext dataContext,
        WorkerCommandService commandService,
        IConfiguration configuration,
        IHubContext<WorkerHub> workerHub,
        IHubContext<SessionHub> sessionHub,
        CancellationToken cancellationToken)
    {
        if (!TryGetAccountId(context, out var accountId))
        {
            return Results.Unauthorized();
        }

        if (!TryGetIdempotencyKey(context, out var idempotencyKey, out var error))
        {
            return error!;
        }

        if (!TryParseViewerSource(request.Source, out var source))
        {
            return Results.ValidationProblem(new Dictionary<string, string[]>
            {
                [nameof(request.Source)] = new[] { "Source must be Scene or RobotCamera." }
            });
        }

        await using var transaction = await dataContext.Database.BeginTransactionAsync(
            IsolationLevel.Serializable,
            cancellationToken);
        var session = await FindOwnedSession(dataContext, id, accountId, cancellationToken);
        if (session == null)
        {
            return Results.NotFound();
        }

        if (!CanControl(session))
        {
            return Results.Conflict(new { message = "The session viewer is not available in the current state." });
        }

        if (await HasPendingEmergencyTransition(dataContext, id, cancellationToken))
        {
            return EmergencyTransitionConflict();
        }

        var workerSupportsViewer = session.ComputeWorker is not null
            && WorkerCapabilities.SupportsCommand(
                session.ComputeWorker,
                WorkerCommandType.SetViewerSource.ToString())
            && WorkerCapabilities.SupportsViewerSource(
                session.ComputeWorker,
                source.ToString());
        if (!workerSupportsViewer)
        {
            return Results.Conflict(new
            {
                message = "The assigned GPU worker does not support this viewer source."
            });
        }

        string? robotRuntimeId = null;
        if (source == ViewerSourceType.RobotCamera)
        {
            robotRuntimeId = request.RobotRuntimeId?.Trim();
            if (string.IsNullOrWhiteSpace(robotRuntimeId)
                || !await dataContext.SessionRobots.AnyAsync(
                    robot => robot.SimulationSessionId == id
                        && robot.RuntimeId == robotRuntimeId
                        && robot.State != SessionRobotState.Removed
                        && robot.State != SessionRobotState.Failed,
                    cancellationToken))
            {
                return Results.ValidationProblem(new Dictionary<string, string[]>
                {
                    [nameof(request.RobotRuntimeId)] =
                        new[] { "RobotRuntimeId must identify an available robot in this session." }
                });
            }
        }
        else if (!string.IsNullOrWhiteSpace(request.RobotRuntimeId))
        {
            return Results.ValidationProblem(new Dictionary<string, string[]>
            {
                [nameof(request.RobotRuntimeId)] =
                    new[] { "RobotRuntimeId is only valid for RobotCamera." }
            });
        }

        if (!ViewerStreamAddress.TryCreate(
                id,
                source,
                robotRuntimeId,
                out var streamAddress))
        {
            return Results.Conflict(new
            {
                message = "The session viewer source has an invalid runtime identifier."
            });
        }

        var leaseKeyAlreadyUsed = await dataContext.ViewerLeases
            .AsNoTracking()
            .AnyAsync(
                lease => lease.SimulationSessionId == id
                    && lease.IdempotencyKey == idempotencyKey,
                cancellationToken);
        if (leaseKeyAlreadyUsed
            || await commandService.Find(id, idempotencyKey, cancellationToken) != null)
        {
            return Results.Conflict(new
            {
                message = "This viewer lease was already issued. Use a new Idempotency-Key for a new one-time token."
            });
        }

        var now = DateTime.UtcNow;
        var leaseMinutes = int.TryParse(configuration["Viewer:LeaseMinutes"], out var configuredMinutes)
            ? Math.Clamp(configuredMinutes, 1, 30)
            : 5;
        var expiresAt = now.AddMinutes(leaseMinutes);
        var token = WebEncoders.Base64UrlEncode(RandomNumberGenerator.GetBytes(32));
        var tokenHash = Convert.ToHexString(SHA256.HashData(Encoding.UTF8.GetBytes(token)));

        var previousLeases = await dataContext.ViewerLeases
            .Where(lease => lease.SimulationSessionId == id
                && lease.AccountId == accountId
                && !lease.RevokedAt.HasValue
                && lease.ExpiresAt > now)
            .ToListAsync(cancellationToken);
        foreach (var lease in previousLeases)
        {
            lease.RevokedAt = now;
        }

        var viewerLease = new ViewerLease
        {
            SimulationSessionId = id,
            SimulationSession = session,
            AccountId = accountId,
            Source = source,
            RobotRuntimeId = robotRuntimeId,
            IdempotencyKey = idempotencyKey,
            TokenHash = tokenHash,
            CreatedAt = now,
            ExpiresAt = expiresAt
        };
        dataContext.ViewerLeases.Add(viewerLease);

        try
        {
            var workerPublishingEnabled =
                configuration.GetValue<bool>("Viewer:WorkerPublishingEnabled");
            var signalingUrl = workerPublishingEnabled && workerSupportsViewer
                ? ViewerStreamAddress.BuildWhepUrl(
                    configuration["Viewer:PublicBaseUrl"],
                    streamAddress)
                : null;
            var hlsUrl = workerPublishingEnabled
                && workerSupportsViewer
                && configuration.GetValue<bool>("Viewer:HlsProxyEnabled")
                    ? ViewerStreamAddress.BuildHlsUrl(
                        configuration["Viewer:HlsPublicBaseUrl"],
                        streamAddress)
                    : null;
            WorkerCommand? command = null;
            if (signalingUrl != null || hlsUrl != null)
            {
                var (publishToken, publishTokenHash) = ViewerPublishToken.Generate();
                viewerLease.PublishTokenHash = publishTokenHash;
                using var payload = JsonSerializer.SerializeToDocument(new
                {
                    leaseId = viewerLease.Id,
                    expiresAt,
                    source = source.ToString(),
                    robotRuntimeId,
                    sourceId = streamAddress.SourceId,
                    streamPath = streamAddress.StreamPath,
                    publishToken
                });
                (command, _) = await commandService.Queue(
                    session,
                    WorkerCommandType.SetViewerSource,
                    idempotencyKey,
                    JsonDocument.Parse(payload.RootElement.GetRawText()),
                    taskRun: null,
                    cancellationToken);
            }

            await dataContext.SaveChangesAsync(cancellationToken);
            await transaction.CommitAsync(cancellationToken);
            if (command != null)
            {
                await NotifyCommand(workerHub, sessionHub, session, command, cancellationToken);
            }

            var response = new ViewerLeaseResponse(
                viewerLease.Id,
                id,
                source.ToString(),
                robotRuntimeId,
                streamAddress.SourceId,
                streamAddress.StreamPath,
                token,
                expiresAt,
                IsReady: false,
                signalingUrl,
                hlsUrl,
                command == null ? null : WorkerCommandService.ToResponse(command));
            return Results.Created($"/api/sessions/{id}/viewer-lease", response);
        }
        catch (Exception exception) when (IsCommandConflict(exception))
        {
            return RetryConflict();
        }
    }

    private static async Task<SimulationSession?> FindOwnedSession(
        DataContext dataContext,
        Guid id,
        int accountId,
        CancellationToken cancellationToken)
    {
        return await dataContext.SimulationSessions
            .Include(session => session.ComputeWorker)
            .SingleOrDefaultAsync(
                session => session.Id == id && session.AccountId == accountId,
                cancellationToken);
    }

    private static bool CanControl(SimulationSession session)
    {
        return session.ComputeWorkerId.HasValue
            && session.State is SimulationSessionState.Ready
                or SimulationSessionState.Active
                or SimulationSessionState.Paused;
    }

    internal static Task<bool> HasPendingEmergencyTransition(
        DataContext dataContext,
        Guid sessionId,
        CancellationToken cancellationToken = default)
    {
        return dataContext.WorkerCommands.AnyAsync(
            command => command.SimulationSessionId == sessionId
                && (command.Type == WorkerCommandType.EmergencyStop
                    || command.Type == WorkerCommandType.ResetEmergencyStop)
                && (command.State == WorkerCommandState.Pending
                    || command.State == WorkerCommandState.Dispatched
                    || command.State == WorkerCommandState.Acknowledged
                    || command.State == WorkerCommandState.Running),
            cancellationToken);
    }

    internal static Task<bool> HasPendingTaskCancellation(
        DataContext dataContext,
        Guid sessionId,
        CancellationToken cancellationToken = default)
    {
        return dataContext.WorkerCommands.AnyAsync(
            command => command.SimulationSessionId == sessionId
                && command.Type == WorkerCommandType.CancelTask
                && (command.State == WorkerCommandState.Pending
                    || command.State == WorkerCommandState.Dispatched
                    || command.State == WorkerCommandState.Acknowledged
                    || command.State == WorkerCommandState.Running),
            cancellationToken);
    }

    private static IResult EmergencyTransitionConflict()
    {
        return Results.Conflict(new
        {
            message = "Wait for the emergency-stop transition to finish before changing the session."
        });
    }

    private static bool TryGetAccountId(HttpContext context, out int accountId)
    {
        accountId = 0;
        var claim = context.User.FindFirst("id");
        return claim != null && int.TryParse(claim.Value, out accountId);
    }

    private static bool TryGetIdempotencyKey(
        HttpContext context,
        out string idempotencyKey,
        out IResult? error)
    {
        idempotencyKey = context.Request.Headers["Idempotency-Key"].ToString().Trim();
        if (string.IsNullOrWhiteSpace(idempotencyKey) || idempotencyKey.Length > 200)
        {
            error = Results.ValidationProblem(new Dictionary<string, string[]>
            {
                ["Idempotency-Key"] =
                    new[] { "Idempotency-Key is required and cannot exceed 200 characters." }
            });
            return false;
        }

        if (idempotencyKey.StartsWith("sys:", StringComparison.OrdinalIgnoreCase))
        {
            error = Results.ValidationProblem(new Dictionary<string, string[]>
            {
                ["Idempotency-Key"] =
                    new[] { "Idempotency-Key cannot use the reserved sys: prefix." }
            });
            return false;
        }

        error = null;
        return true;
    }

    private static bool TryParseTaskType(string value, out SwarmTaskRunType taskType)
    {
        var normalized = value?.Replace("-", string.Empty).Replace("_", string.Empty).Trim();
        if (string.Equals(normalized, "FollowLeader", StringComparison.OrdinalIgnoreCase))
        {
            taskType = SwarmTaskRunType.FollowLeader;
            return true;
        }

        if (string.Equals(normalized, "Figure", StringComparison.OrdinalIgnoreCase))
        {
            taskType = SwarmTaskRunType.Figure;
            return true;
        }

        if (string.Equals(normalized, "CollaborativeTransport", StringComparison.OrdinalIgnoreCase)
            || string.Equals(normalized, "Transport", StringComparison.OrdinalIgnoreCase))
        {
            taskType = SwarmTaskRunType.CollaborativeTransport;
            return true;
        }

        taskType = default;
        return false;
    }

    private static bool TryParseViewerSource(string value, out ViewerSourceType source)
    {
        var normalized = value?.Replace("-", string.Empty).Replace("_", string.Empty).Trim();
        if (string.Equals(normalized, "Scene", StringComparison.OrdinalIgnoreCase))
        {
            source = ViewerSourceType.Scene;
            return true;
        }

        if (string.Equals(normalized, "RobotCamera", StringComparison.OrdinalIgnoreCase))
        {
            source = ViewerSourceType.RobotCamera;
            return true;
        }

        source = default;
        return false;
    }

    private static TaskRunResponse ToTaskResponse(TaskRun task)
    {
        return new TaskRunResponse(
            task.Id,
            task.SimulationSessionId,
            task.Type.ToString(),
            task.State.ToString(),
            task.Progress,
            ControlPlaneJson.ToElement(task.Parameters),
            ControlPlaneJson.ToNullableElement(task.Result),
            task.Error,
            task.OutcomeState.ToString(),
            task.OutcomeReason,
            task.CreatedAt,
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

    private static async Task NotifyCommand(
        IHubContext<WorkerHub> workerHub,
        IHubContext<SessionHub> sessionHub,
        SimulationSession session,
        WorkerCommand command,
        CancellationToken cancellationToken)
    {
        if (session.ComputeWorkerId.HasValue)
        {
            await workerHub.Clients.Group(ControlPlaneGroups.Worker(session.ComputeWorkerId.Value))
                .SendAsync("CommandAvailable", cancellationToken);
        }

        await sessionHub.Clients.Group(ControlPlaneGroups.Session(session.Id))
            .SendAsync("SessionEvent", new
            {
                sessionId = session.Id,
                command = WorkerCommandService.ToResponse(command),
                eventType = "CommandQueued",
                timestamp = command.CreatedAt
            }, cancellationToken);
    }

    private static IResult IdempotencyConflict()
    {
        return Results.Conflict(new
        {
            message = "The Idempotency-Key was already used for a different operation."
        });
    }

    private static IResult RetryConflict()
    {
        return Results.Conflict(new
        {
            message = "The session changed concurrently. Retry with the same Idempotency-Key."
        });
    }

    private static bool IsCommandConflict(Exception exception)
    {
        return exception is InvalidOperationException
            || exception is PostgresException
            {
                SqlState: PostgresErrorCodes.SerializationFailure
                    or PostgresErrorCodes.UniqueViolation
            }
            || exception is DbUpdateException
            {
                InnerException: PostgresException
                {
                    SqlState: PostgresErrorCodes.SerializationFailure
                        or PostgresErrorCodes.UniqueViolation
                }
            };
    }
}
