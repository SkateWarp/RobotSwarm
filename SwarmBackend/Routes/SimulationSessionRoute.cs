using System.Data;
using System.Linq.Expressions;
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
            .RequireRateLimiting(AbuseProtection.SessionCreationPolicy)
            .Produces<SimulationSessionResponse>(StatusCodes.Status202Accepted)
            .Produces(StatusCodes.Status400BadRequest)
            .Produces(StatusCodes.Status401Unauthorized)
            .Produces(StatusCodes.Status409Conflict)
            .Produces(StatusCodes.Status429TooManyRequests)
            .Produces(StatusCodes.Status503ServiceUnavailable);

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

    internal static async Task<IResult> Create(
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

        await using var transaction = dataContext.Database.IsRelational()
            ? await dataContext.Database.BeginTransactionAsync(
                IsolationLevel.Serializable,
                cancellationToken)
            : null;

        var hasLiveSession = await dataContext.SimulationSessions
            .AnyAsync(OccupiesAccountSlot(accountId.Value), cancellationToken);

        if (hasLiveSession)
        {
            return Results.Conflict(new
            {
                message = "The account already has an active or queued simulation session."
            });
        }

        var queuedCount = await dataContext.SimulationSessions.CountAsync(
            session => session.State == SimulationSessionState.Queued,
            cancellationToken);
        if (queuedCount >= SessionLimits.GetMaxQueuedSessions(configuration))
        {
            context.Response.Headers.RetryAfter = "30";
            return Results.Problem(
                statusCode: StatusCodes.Status503ServiceUnavailable,
                title: "The simulation queue is full.",
                detail: "Try again after another queued session starts or expires.");
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
            if (transaction != null)
            {
                await transaction.CommitAsync(cancellationToken);
            }
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
        catch (PostgresException exception)
            when (exception.SqlState == PostgresErrorCodes.SerializationFailure)
        {
            return QueueChangedConflict();
        }
        catch (DbUpdateException exception)
            when (exception.InnerException is PostgresException
            {
                SqlState: PostgresErrorCodes.SerializationFailure
            })
        {
            return QueueChangedConflict();
        }

        var queuePositions = await GetQueuePositions(dataContext, cancellationToken);
        var response = ToResponse(
            session,
            queuePositions.GetValueOrDefault(session.Id),
            computeWorkerName: null);

        return Results.Accepted($"/api/sessions/{session.Id}", response);
    }

    internal static Expression<Func<SimulationSession, bool>> OccupiesAccountSlot(
        int accountId)
    {
        return session => session.AccountId == accountId
            && (session.State < SimulationSessionState.Stopped
                || (session.State == SimulationSessionState.Failed
                    || session.State == SimulationSessionState.Expired)
                && session.ComputeWorkerId.HasValue);
    }

    private static IResult QueueChangedConflict()
    {
        return Results.Conflict(new
        {
            message = "Simulation capacity changed while the request was queued. Try again."
        });
    }

    internal static bool RequiresWorkerCleanup(SimulationSessionState state)
    {
        return state is SimulationSessionState.Failed
            or SimulationSessionState.Expired;
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
            .Include(candidate => candidate.Commands)
            .SingleOrDefaultAsync(
                candidate => candidate.Id == id && candidate.AccountId == accountId.Value,
                cancellationToken);

        if (session == null)
        {
            return Results.NotFound();
        }

        if (session.State < SimulationSessionState.Stopped
            || session.State == SimulationSessionState.Failed
            || session.State == SimulationSessionState.Expired)
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
                var resourceKnownPresent = session.State < SimulationSessionState.Stopped;
                command = TerminalCleanupPolicy.TryQueue(
                    session,
                    now,
                    "sys:session-delete",
                    resourceKnownPresent);
                if (command != null)
                {
                    dataContext.WorkerCommands.Add(command);
                }

                if (resourceKnownPresent
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
