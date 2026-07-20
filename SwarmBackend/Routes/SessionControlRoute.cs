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
    private const int MaximumViewerLeaseAttempts = 3;
    private const int MaximumViewerStopAttempts = 3;

    public static RouteGroupBuilder MapSessionControl(this RouteGroupBuilder group)
    {
        group.RequireAuthorization();

        group.MapPatch("/{id:guid}/fleet", UpdateFleet)
            .Produces<FleetUpdateResponse>(StatusCodes.Status202Accepted);
        group.MapGet("/{id:guid}/robots", GetRobots)
            .Produces<IEnumerable<SessionRobotResponse>>();
        group.MapGet("/tasks/history", GetTaskHistory)
            .Produces<TaskRunHistoryResponse>();
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
        group.MapGet("/{id:guid}/viewer-lease/{leaseId:guid}", GetViewerLeaseStatus)
            .Produces<ViewerLeaseStatusResponse>();
        group.MapDelete("/{id:guid}/viewer-lease/{leaseId:guid}", CloseViewerLease)
            .Produces<ViewerLeaseCloseResponse>(StatusCodes.Status200OK)
            .Produces<ViewerLeaseCloseResponse>(StatusCodes.Status202Accepted);

        return group;
    }

    internal static async Task<IResult> GetViewerLeaseStatus(
        Guid id,
        Guid leaseId,
        HttpContext context,
        DataContext dataContext,
        CancellationToken cancellationToken)
    {
        if (!TryGetAccountId(context, out var accountId))
        {
            return Results.Unauthorized();
        }

        var lease = await dataContext.ViewerLeases
            .AsNoTracking()
            .SingleOrDefaultAsync(
                candidate => candidate.Id == leaseId
                    && candidate.SimulationSessionId == id
                    && candidate.AccountId == accountId
                    && candidate.SimulationSession.AccountId == accountId,
                cancellationToken);
        if (lease == null)
        {
            return Results.NotFound();
        }

        WorkerCommand? command = null;
        if (!string.IsNullOrWhiteSpace(lease.IdempotencyKey))
        {
            command = await dataContext.WorkerCommands
                .AsNoTracking()
                .SingleOrDefaultAsync(
                    candidate => candidate.SimulationSessionId == id
                        && candidate.IdempotencyKey == lease.IdempotencyKey
                        && candidate.Type == WorkerCommandType.SetViewerSource,
                    cancellationToken);
        }

        var stopKeys = ViewerStopIdempotencyKeys(leaseId);
        var closeCommand = await dataContext.WorkerCommands
            .AsNoTracking()
            .Where(candidate => candidate.SimulationSessionId == id
                && candidate.Type == WorkerCommandType.StopViewer
                && stopKeys.Contains(candidate.IdempotencyKey))
            .OrderByDescending(candidate => candidate.Sequence)
            .FirstOrDefaultAsync(cancellationToken);

        var commandStatus = command == null
            ? null
            : new ViewerLeaseCommandStatusResponse(
                command.Id,
                command.State.ToString(),
                command.LastError,
                command.UpdatedAt);
        var closeCommandStatus = closeCommand == null
            ? null
            : new ViewerLeaseCommandStatusResponse(
                closeCommand.Id,
                closeCommand.State.ToString(),
                closeCommand.LastError,
                closeCommand.UpdatedAt);
        return Results.Ok(new ViewerLeaseStatusResponse(
            lease.Id,
            lease.SimulationSessionId,
            lease.Source.ToString(),
            lease.RobotRuntimeId,
            lease.CreatedAt,
            lease.ExpiresAt,
            lease.RevokedAt,
            !lease.RevokedAt.HasValue
                && command?.State == WorkerCommandState.Completed,
            commandStatus,
            closeCommandStatus));
    }

    internal static Task<IResult> CloseViewerLease(
        Guid id,
        Guid leaseId,
        HttpContext context,
        DataContext dataContext,
        WorkerCommandService commandService,
        IHubContext<WorkerHub> workerHub,
        IHubContext<SessionHub> sessionHub,
        ViewerControlRegistry viewerControls,
        CancellationToken cancellationToken)
    {
        return RetryViewerCloseConflicts(
            () => CloseViewerLeaseAttempt(
                id,
                leaseId,
                context,
                dataContext,
                commandService,
                workerHub,
                sessionHub,
                viewerControls,
                cancellationToken),
            dataContext.ChangeTracker.Clear,
            cancellationToken);
    }

    private static async Task<IResult> CloseViewerLeaseAttempt(
        Guid id,
        Guid leaseId,
        HttpContext context,
        DataContext dataContext,
        WorkerCommandService commandService,
        IHubContext<WorkerHub> workerHub,
        IHubContext<SessionHub> sessionHub,
        ViewerControlRegistry viewerControls,
        CancellationToken cancellationToken)
    {
        if (!TryGetAccountId(context, out var accountId))
        {
            return Results.Unauthorized();
        }

        await using var transaction = await dataContext.Database.BeginTransactionAsync(
            IsolationLevel.Serializable,
            cancellationToken);
        var session = await FindOwnedSession(dataContext, id, accountId, cancellationToken);
        if (session == null)
        {
            return Results.NotFound();
        }

        var lease = await dataContext.ViewerLeases.SingleOrDefaultAsync(
            candidate => candidate.Id == leaseId
                && candidate.SimulationSessionId == id
                && candidate.AccountId == accountId,
            cancellationToken);
        if (lease == null)
        {
            return Results.NotFound();
        }

        using var stopPayload = JsonSerializer.SerializeToDocument(new { leaseId });
        var stopKeys = ViewerStopIdempotencyKeys(leaseId);
        var stopCommands = await dataContext.WorkerCommands
            .Where(candidate => candidate.SimulationSessionId == id
                && candidate.Type == WorkerCommandType.StopViewer
                && stopKeys.Contains(candidate.IdempotencyKey))
            .OrderByDescending(candidate => candidate.Sequence)
            .ToListAsync(cancellationToken);
        var existing = stopCommands.FirstOrDefault();
        if (existing != null)
        {
            if (!WorkerCommandService.SamePayload(existing.Payload, stopPayload))
            {
                return IdempotencyConflict();
            }

            var existingResponse = new ViewerLeaseCloseResponse(
                lease.Id,
                id,
                lease.RevokedAt ?? existing.CreatedAt,
                WorkerCommandService.ToResponse(existing));
            if (existing.State == WorkerCommandState.Completed)
            {
                return Results.Ok(existingResponse);
            }

            if (IsViewerStopInFlight(existing.State))
            {
                return Results.Accepted(
                    $"/api/sessions/{id}/viewer-lease/{leaseId}",
                    existingResponse);
            }

            if (existing.State is not (WorkerCommandState.Failed
                or WorkerCommandState.Cancelled))
            {
                return Results.Conflict(new
                {
                    message = "The viewer stop command is in an unsupported state."
                });
            }

            if (stopCommands.Count >= MaximumViewerStopAttempts)
            {
                return Results.Conflict(new
                {
                    message = "The GPU worker could not confirm the viewer close after three attempts."
                });
            }
        }

        if (lease.RevokedAt.HasValue && existing == null)
        {
            return Results.Ok(new ViewerLeaseCloseResponse(
                lease.Id,
                id,
                lease.RevokedAt.Value,
                Command: null));
        }

        WorkerCommand? command = null;
        if (CanControl(session))
        {
            if (session.ComputeWorker is null
                || !WorkerCapabilities.SupportsCommand(
                    session.ComputeWorker,
                    WorkerCommandType.StopViewer.ToString()))
            {
                return Results.Conflict(new
                {
                    message = "The assigned GPU worker cannot stop the viewer independently."
                });
            }

            var attempt = stopCommands.Count + 1;
            var idempotencyKey = ViewerStopIdempotencyKey(leaseId, attempt);
            (command, _) = await commandService.Queue(
                session,
                WorkerCommandType.StopViewer,
                idempotencyKey,
                JsonDocument.Parse(stopPayload.RootElement.GetRawText()),
                taskRun: null,
                cancellationToken);
        }
        else if (existing != null)
        {
            return Results.Conflict(new
            {
                message = "The GPU worker is no longer available to retry the viewer close."
            });
        }

        var now = DateTime.UtcNow;
        long? revocationVersion = null;
        if (!lease.RevokedAt.HasValue)
        {
            lease.RevokedAt = now;
            revocationVersion = await viewerControls.BeginLeaseRevocationAsync(
                id,
                leaseId,
                new DateTimeOffset(DateTime.SpecifyKind(now, DateTimeKind.Utc)),
                cancellationToken);
        }
        var revocationCommitted = false;
        try
        {
            await dataContext.SaveChangesAsync(cancellationToken);
            await transaction.CommitAsync(cancellationToken);
            if (revocationVersion.HasValue)
            {
                viewerControls.ConfirmLeaseRevocation(id, leaseId, revocationVersion.Value);
            }
            revocationCommitted = true;
            if (revocationVersion.HasValue)
            {
                await viewerControls.DrainLeaseAsync(id, leaseId, CancellationToken.None);
            }

            if (command != null)
            {
                await NotifyCommand(
                    workerHub,
                    sessionHub,
                    session,
                    command,
                    cancellationToken);
            }

            var response = new ViewerLeaseCloseResponse(
                lease.Id,
                id,
                lease.RevokedAt ?? now,
                command == null ? null : WorkerCommandService.ToResponse(command));
            return command == null
                ? Results.Ok(response)
                : Results.Accepted(
                    $"/api/sessions/{id}/viewer-lease/{leaseId}",
                    response);
        }
        finally
        {
            if (!revocationCommitted && revocationVersion.HasValue)
            {
                viewerControls.CancelLeaseRevocation(id, leaseId, revocationVersion.Value);
            }
        }
    }

    private static string ViewerStopIdempotencyKey(Guid leaseId, int attempt)
    {
        var prefix = $"sys:viewer-stop:{leaseId:N}";
        return attempt == 1 ? prefix : $"{prefix}:{attempt}";
    }

    private static string[] ViewerStopIdempotencyKeys(Guid leaseId)
    {
        return Enumerable.Range(1, MaximumViewerStopAttempts)
            .Select(attempt => ViewerStopIdempotencyKey(leaseId, attempt))
            .ToArray();
    }

    private static bool IsViewerStopInFlight(WorkerCommandState state)
    {
        return state is WorkerCommandState.Pending
            or WorkerCommandState.Dispatched
            or WorkerCommandState.Acknowledged
            or WorkerCommandState.Running;
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

    internal static async Task<IResult> GetTaskHistory(
        HttpContext context,
        DataContext dataContext,
        CancellationToken cancellationToken,
        int offset = 0,
        int limit = 25,
        string? type = null,
        string? state = null,
        string? outcome = null)
    {
        if (!TryGetAccountId(context, out var accountId))
        {
            return Results.Unauthorized();
        }

        if (offset < 0 || limit < 1 || limit > 100)
        {
            return Results.ValidationProblem(new Dictionary<string, string[]>
            {
                [nameof(offset)] = offset < 0
                    ? new[] { "Offset must be zero or greater." }
                    : Array.Empty<string>(),
                [nameof(limit)] = limit is < 1 or > 100
                    ? new[] { "Limit must be between 1 and 100." }
                    : Array.Empty<string>()
            }
            .Where(entry => entry.Value.Length > 0)
            .ToDictionary(entry => entry.Key, entry => entry.Value));
        }

        if (!TryParseOptionalEnum(type, out SwarmTaskRunType? taskType)
            || !TryParseOptionalEnum(state, out TaskRunState? taskState)
            || !TryParseOptionalEnum(outcome, out TaskOutcomeState? outcomeState))
        {
            return Results.ValidationProblem(new Dictionary<string, string[]>
            {
                ["filters"] = new[] { "Type, state, or outcome contains an unsupported value." }
            });
        }

        var query = dataContext.TaskRuns
            .AsNoTracking()
            .Where(task => task.SimulationSession.AccountId == accountId);
        if (taskType.HasValue)
        {
            query = query.Where(task => task.Type == taskType.Value);
        }
        if (taskState.HasValue)
        {
            query = query.Where(task => task.State == taskState.Value);
        }
        if (outcomeState.HasValue)
        {
            query = query.Where(task => task.OutcomeState == outcomeState.Value);
        }
        var total = await query.CountAsync(cancellationToken);
        var tasks = await query
            .OrderByDescending(task => task.CreatedAt)
            .ThenByDescending(task => task.Id)
            .Skip(offset)
            .Take(limit)
            .ToListAsync(cancellationToken);

        return Results.Ok(new TaskRunHistoryResponse(
            total,
            offset,
            limit,
            tasks.Select(ToTaskResponse).ToList()));
    }

    private static bool TryParseOptionalEnum<TEnum>(string? value, out TEnum? parsed)
        where TEnum : struct, Enum
    {
        if (string.IsNullOrWhiteSpace(value))
        {
            parsed = null;
            return true;
        }

        if (Enum.TryParse<TEnum>(value.Trim(), ignoreCase: true, out var result)
            && Enum.IsDefined(result))
        {
            parsed = result;
            return true;
        }

        parsed = null;
        return false;
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

    private static Task<IResult> CreateViewerLease(
        Guid id,
        CreateViewerLeaseRequest request,
        HttpContext context,
        DataContext dataContext,
        WorkerCommandService commandService,
        IConfiguration configuration,
        IHubContext<WorkerHub> workerHub,
        IHubContext<SessionHub> sessionHub,
        ViewerControlRegistry viewerControls,
        CancellationToken cancellationToken)
    {
        return RetryViewerLeaseSerializationFailures(
            () => CreateViewerLeaseAttempt(
                id,
                request,
                context,
                dataContext,
                commandService,
                configuration,
                workerHub,
                sessionHub,
                viewerControls,
                cancellationToken),
            dataContext.ChangeTracker.Clear,
            cancellationToken);
    }

    private static async Task<IResult> CreateViewerLeaseAttempt(
        Guid id,
        CreateViewerLeaseRequest request,
        HttpContext context,
        DataContext dataContext,
        WorkerCommandService commandService,
        IConfiguration configuration,
        IHubContext<WorkerHub> workerHub,
        IHubContext<SessionHub> sessionHub,
        ViewerControlRegistry viewerControls,
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

        var revocationMarkers = new List<(Guid LeaseId, long Version)>();
        var revocationCommitted = false;
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

            var revocationStartedAt = new DateTimeOffset(
                DateTime.SpecifyKind(now, DateTimeKind.Utc));
            foreach (var previousLease in previousLeases.OrderBy(lease => lease.Id))
            {
                var version = await viewerControls.BeginLeaseRevocationAsync(
                    id,
                    previousLease.Id,
                    revocationStartedAt,
                    cancellationToken);
                revocationMarkers.Add((previousLease.Id, version));
            }

            await dataContext.SaveChangesAsync(cancellationToken);
            await transaction.CommitAsync(cancellationToken);
            foreach (var marker in revocationMarkers)
            {
                viewerControls.ConfirmLeaseRevocation(id, marker.LeaseId, marker.Version);
            }
            revocationCommitted = true;
            foreach (var previousLease in previousLeases)
            {
                await viewerControls.DrainLeaseAsync(
                    id,
                    previousLease.Id,
                    CancellationToken.None);
            }

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
        catch (Exception exception) when (
            IsCommandConflict(exception)
            && !SimulationSessionRoute.IsSerializationFailure(exception))
        {
            return RetryConflict();
        }
        finally
        {
            if (!revocationCommitted)
            {
                foreach (var marker in revocationMarkers)
                {
                    viewerControls.CancelLeaseRevocation(id, marker.LeaseId, marker.Version);
                }
            }
        }
    }

    internal static async Task<IResult> RetryViewerLeaseSerializationFailures(
        Func<Task<IResult>> operation,
        Action resetContext,
        CancellationToken cancellationToken)
    {
        for (var attempt = 1; attempt <= MaximumViewerLeaseAttempts; attempt++)
        {
            try
            {
                return await operation();
            }
            catch (Exception exception)
                when (SimulationSessionRoute.IsSerializationFailure(exception))
            {
                resetContext();
                cancellationToken.ThrowIfCancellationRequested();
                if (attempt == MaximumViewerLeaseAttempts)
                {
                    break;
                }

                await Task.Delay(
                    TimeSpan.FromMilliseconds(25 * attempt),
                    cancellationToken);
            }
        }

        return RetryConflict();
    }

    internal static async Task<IResult> RetryViewerCloseConflicts(
        Func<Task<IResult>> operation,
        Action resetContext,
        CancellationToken cancellationToken)
    {
        for (var attempt = 1; attempt <= MaximumViewerLeaseAttempts; attempt++)
        {
            try
            {
                return await operation();
            }
            catch (Exception exception) when (
                SimulationSessionRoute.IsSerializationFailure(exception)
                || IsUniqueViolation(exception))
            {
                resetContext();
                cancellationToken.ThrowIfCancellationRequested();
                if (attempt == MaximumViewerLeaseAttempts)
                {
                    break;
                }

                await Task.Delay(
                    TimeSpan.FromMilliseconds(25 * attempt),
                    cancellationToken);
            }
        }

        return RetryConflict();
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

    private static bool IsUniqueViolation(Exception exception)
    {
        for (Exception? current = exception; current != null; current = current.InnerException)
        {
            if (current is PostgresException
                {
                    SqlState: PostgresErrorCodes.UniqueViolation
                })
            {
                return true;
            }
        }

        return false;
    }
}
