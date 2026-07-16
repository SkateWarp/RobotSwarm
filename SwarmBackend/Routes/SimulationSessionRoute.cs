using System.Data;
using System.Text.Json;
using Microsoft.AspNetCore.SignalR;
using Microsoft.EntityFrameworkCore;
using Npgsql;
using SwarmBackend.Entities;
using SwarmBackend.Helpers;
using SwarmBackend.Models;
using SwarmBackend.Services;

namespace SwarmBackend.Routes;

public static class SimulationSessionRoute
{
    public static RouteGroupBuilder MapSimulationSession(this RouteGroupBuilder group)
    {
        group.RequireAuthorization();

        group.MapPost("", Create)
            .Produces<SimulationSessionResponse>(StatusCodes.Status202Accepted)
            .Produces(StatusCodes.Status400BadRequest)
            .Produces(StatusCodes.Status401Unauthorized)
            .Produces(StatusCodes.Status409Conflict);

        group.MapGet("", GetAll)
            .Produces<IEnumerable<SimulationSessionResponse>>()
            .Produces(StatusCodes.Status401Unauthorized);

        group.MapGet("/limits", GetLimits)
            .Produces<SimulationSessionLimitsResponse>()
            .Produces(StatusCodes.Status401Unauthorized);

        group.MapGet("/{id:guid}", GetById)
            .Produces<SimulationSessionResponse>()
            .Produces(StatusCodes.Status401Unauthorized)
            .Produces(StatusCodes.Status404NotFound);

        group.MapDelete("/{id:guid}", Delete)
            .Produces<SimulationSessionResponse>()
            .Produces(StatusCodes.Status401Unauthorized)
            .Produces(StatusCodes.Status404NotFound);

        return group;
    }

    private static async Task<IResult> Create(
        CreateSimulationSessionRequest request,
        DataContext dataContext,
        IConfiguration configuration,
        HttpContext context,
        CancellationToken cancellationToken)
    {
        var accountId = GetAccountId(context);
        if (!accountId.HasValue)
        {
            return Results.Unauthorized();
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

        var hasLiveSession = await dataContext.SimulationSessions
            .AnyAsync(
                session => session.AccountId == accountId.Value
                    && session.State < SimulationSessionState.Stopped,
                cancellationToken);

        if (hasLiveSession)
        {
            return Results.Conflict(new
            {
                message = "The account already has an active or queued simulation session."
            });
        }

        var now = DateTime.UtcNow;
        var session = new SimulationSession
        {
            AccountId = accountId.Value,
            DesiredRobotCount = request.RobotCount,
            State = SimulationSessionState.Queued,
            CreatedAt = now,
            UpdatedAt = now
        };

        dataContext.SimulationSessions.Add(session);

        try
        {
            await dataContext.SaveChangesAsync(cancellationToken);
        }
        catch (DbUpdateException exception)
            when (exception.InnerException is PostgresException
            {
                SqlState: PostgresErrorCodes.UniqueViolation
            })
        {
            return Results.Conflict(new
            {
                message = "The account already has an active or queued simulation session."
            });
        }

        var queuePositions = await GetQueuePositions(dataContext, cancellationToken);
        var response = ToResponse(
            session,
            queuePositions.GetValueOrDefault(session.Id),
            computeWorkerName: null);

        return Results.Accepted($"/api/sessions/{session.Id}", response);
    }

    private static async Task<IResult> GetAll(
        DataContext dataContext,
        HttpContext context,
        CancellationToken cancellationToken)
    {
        var accountId = GetAccountId(context);
        if (!accountId.HasValue)
        {
            return Results.Unauthorized();
        }

        var sessions = await dataContext.SimulationSessions
            .AsNoTracking()
            .Include(session => session.ComputeWorker)
            .Where(session => session.AccountId == accountId.Value)
            .OrderByDescending(session => session.CreatedAt)
            .ThenByDescending(session => session.Id)
            .ToListAsync(cancellationToken);

        var queuePositions = await GetQueuePositions(dataContext, cancellationToken);
        var response = sessions.Select(session => ToResponse(
            session,
            queuePositions.GetValueOrDefault(session.Id),
            session.ComputeWorker?.Name));

        return Results.Ok(response);
    }

    private static IResult GetLimits(IConfiguration configuration)
    {
        return Results.Ok(new SimulationSessionLimitsResponse(
            SessionLimits.GetMaxRobotsPerSession(configuration)));
    }

    private static async Task<IResult> GetById(
        Guid id,
        DataContext dataContext,
        HttpContext context,
        CancellationToken cancellationToken)
    {
        var accountId = GetAccountId(context);
        if (!accountId.HasValue)
        {
            return Results.Unauthorized();
        }

        var session = await dataContext.SimulationSessions
            .AsNoTracking()
            .Include(candidate => candidate.ComputeWorker)
            .SingleOrDefaultAsync(
                candidate => candidate.Id == id && candidate.AccountId == accountId.Value,
                cancellationToken);

        if (session == null)
        {
            return Results.NotFound();
        }

        var queuePositions = await GetQueuePositions(dataContext, cancellationToken);
        return Results.Ok(ToResponse(
            session,
            queuePositions.GetValueOrDefault(session.Id),
            session.ComputeWorker?.Name));
    }

    private static async Task<IResult> Delete(
        Guid id,
        DataContext dataContext,
        WorkerCommandService commandService,
        IHubContext<WorkerHub> workerHub,
        IHubContext<SessionHub> sessionHub,
        HttpContext context,
        CancellationToken cancellationToken)
    {
        var accountId = GetAccountId(context);
        if (!accountId.HasValue)
        {
            return Results.Unauthorized();
        }

        await using var transaction = await dataContext.Database.BeginTransactionAsync(
            IsolationLevel.Serializable,
            cancellationToken);
        var session = await dataContext.SimulationSessions
            .Include(candidate => candidate.ComputeWorker)
            .SingleOrDefaultAsync(
                candidate => candidate.Id == id && candidate.AccountId == accountId.Value,
                cancellationToken);

        if (session == null)
        {
            return Results.NotFound();
        }

        if (session.State < SimulationSessionState.Stopped
            || session.State == SimulationSessionState.Failed)
        {
            var now = DateTime.UtcNow;
            WorkerCommand? command = null;
            if (session.State == SimulationSessionState.Queued
                || !session.ComputeWorkerId.HasValue)
            {
                session.State = SimulationSessionState.Stopped;
                session.StoppedAt = now;
                session.UpdatedAt = now;
                session.Revision++;
            }
            else
            {
                if (session.State == SimulationSessionState.Failed)
                {
                    command = await dataContext.WorkerCommands
                        .Where(candidate => candidate.SimulationSessionId == session.Id
                            && candidate.Type == WorkerCommandType.StopSession
                            && candidate.State != WorkerCommandState.Completed
                            && candidate.State != WorkerCommandState.Failed
                            && candidate.State != WorkerCommandState.Cancelled)
                        .OrderByDescending(candidate => candidate.Sequence)
                        .FirstOrDefaultAsync(cancellationToken);
                }

                if (command == null)
                {
                    var cleanupAttempt = session.State == SimulationSessionState.Failed
                        ? await dataContext.WorkerCommands.CountAsync(
                            candidate => candidate.SimulationSessionId == session.Id
                                && candidate.Type == WorkerCommandType.StopSession,
                            cancellationToken) + 1
                        : 0;
                    var commandKey = session.State == SimulationSessionState.Failed
                        ? $"sys:cleanup-session:{session.Id:N}:{cleanupAttempt}"
                        : $"sys:stop-session:{session.Id:N}";
                    (command, _) = await commandService.Queue(
                        session,
                        WorkerCommandType.StopSession,
                        commandKey,
                        JsonDocument.Parse("{}"),
                        taskRun: null,
                        cancellationToken);
                }

                if (session.State != SimulationSessionState.Failed
                    && session.State != SimulationSessionState.Stopping)
                {
                    session.State = SimulationSessionState.Stopping;
                    session.UpdatedAt = now;
                    session.Revision++;
                }
            }

            await dataContext.SaveChangesAsync(cancellationToken);
            await transaction.CommitAsync(cancellationToken);

            if (command != null)
            {
                await workerHub.Clients.Group(ControlPlaneGroups.Worker(session.ComputeWorkerId!.Value))
                    .SendAsync("CommandAvailable", cancellationToken);
                await sessionHub.Clients.Group(ControlPlaneGroups.Session(session.Id))
                    .SendAsync("SessionEvent", new
                    {
                        sessionId = session.Id,
                        command = WorkerCommandService.ToResponse(command),
                        eventType = "CommandQueued",
                        timestamp = command.CreatedAt
                    }, cancellationToken);
            }

            await sessionHub.Clients.Group(ControlPlaneGroups.Session(session.Id))
                .SendAsync(
                    "SessionUpdated",
                    ToResponse(session, queuePosition: null, session.ComputeWorker?.Name),
                    cancellationToken);
        }

        return Results.Ok(ToResponse(
            session,
            queuePosition: null,
            computeWorkerName: session.ComputeWorker?.Name));
    }

    private static int? GetAccountId(HttpContext context)
    {
        var accountIdClaim = context.User.FindFirst("id");
        return accountIdClaim != null && int.TryParse(accountIdClaim.Value, out var accountId)
            ? accountId
            : null;
    }

    private static async Task<IReadOnlyDictionary<Guid, int>> GetQueuePositions(
        DataContext dataContext,
        CancellationToken cancellationToken)
    {
        var queuedSessionIds = await dataContext.SimulationSessions
            .AsNoTracking()
            .Where(session => session.State == SimulationSessionState.Queued)
            .OrderBy(session => session.CreatedAt)
            .ThenBy(session => session.Id)
            .Select(session => session.Id)
            .ToListAsync(cancellationToken);

        return queuedSessionIds
            .Select((sessionId, index) => new { sessionId, position = index + 1 })
            .ToDictionary(item => item.sessionId, item => item.position);
    }

    private static SimulationSessionResponse ToResponse(
        SimulationSession session,
        int? queuePosition,
        string? computeWorkerName)
    {
        return new SimulationSessionResponse(
            session.Id,
            session.State.ToString(),
            session.DesiredRobotCount,
            session.State == SimulationSessionState.Queued ? queuePosition : null,
            session.ArenaVersion,
            session.ComputeWorkerId,
            computeWorkerName,
            session.IsEmergencyStopped,
            session.Revision,
            session.FailureReason,
            session.CreatedAt,
            session.UpdatedAt,
            session.StartedAt,
            session.StoppedAt);
    }
}
