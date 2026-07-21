using System.Data;
using System.Linq.Expressions;
using System.Security.Claims;
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
    private const int MaximumCreateAttempts = 3;
    private const int MaximumDeleteAttempts = 3;

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
            .Produces(StatusCodes.Status404NotFound)
            .Produces(StatusCodes.Status409Conflict);

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

        return await RetrySerializationFailures(
            () => CreateSession(
                request,
                accountId.Value,
                dataContext,
                configuration,
                context,
                cancellationToken),
            dataContext.ChangeTracker.Clear,
            cancellationToken);
    }

    private static async Task<IResult> CreateSession(
        CreateSimulationSessionRequest request,
        int accountId,
        DataContext dataContext,
        IConfiguration configuration,
        HttpContext context,
        CancellationToken cancellationToken)
    {
        await using var transaction = dataContext.Database.IsRelational()
            ? await dataContext.Database.BeginTransactionAsync(
                IsolationLevel.Serializable,
                cancellationToken)
            : null;

        var accountIsCurrent = await AccountResourceLock.AcquireSharedAndValidate(
            dataContext,
            accountId,
            context.User,
            cancellationToken);
        if (!accountIsCurrent)
        {
            return Results.Unauthorized();
        }

        var hasLiveSession = await dataContext.SimulationSessions
            .AnyAsync(OccupiesAccountSlot(accountId), cancellationToken);

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
            AccountId = accountId,
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
        var queuePositions = await GetQueuePositions(dataContext, cancellationToken);
        var response = ToResponse(
            session,
            queuePositions.GetValueOrDefault(session.Id),
            computeWorkerName: null);

        return Results.Accepted($"/api/sessions/{session.Id}", response);
    }

    internal static async Task<IResult> RetrySerializationFailures(
        Func<Task<IResult>> operation,
        Action resetContext,
        CancellationToken cancellationToken)
    {
        for (var attempt = 1; attempt <= MaximumCreateAttempts; attempt++)
        {
            try
            {
                return await operation();
            }
            catch (Exception exception) when (IsSerializationFailure(exception))
            {
                // Two valid requests can both read the queue count before one
                // commits. PostgreSQL cancels one transaction so the limit is
                // never exceeded; retry that request with a fresh context.
                resetContext();
                cancellationToken.ThrowIfCancellationRequested();
                if (attempt == MaximumCreateAttempts)
                {
                    break;
                }

                await Task.Delay(
                    TimeSpan.FromMilliseconds(25 * attempt),
                    cancellationToken);
            }
        }

        return QueueChangedConflict();
    }

    internal static bool IsSerializationFailure(Exception exception)
    {
        for (Exception? current = exception; current != null; current = current.InnerException)
        {
            if (current is PostgresException
                {
                    SqlState: PostgresErrorCodes.SerializationFailure
                })
            {
                return true;
            }
        }

        return false;
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

    private static Task<IResult> Delete(
        Guid id,
        DataContext dataContext,
        IHubContext<WorkerHub> workerHub,
        IHubContext<SessionHub> sessionHub,
        ViewerControlRegistry viewerControls,
        HttpContext context,
        CancellationToken cancellationToken)
    {
        var accountId = GetAccountId(context);
        if (!accountId.HasValue)
        {
            return Task.FromResult<IResult>(Results.Unauthorized());
        }

        return RetryDeleteSerializationFailures(
            () => DeleteAttempt(
                id,
                accountId.Value,
                context.User,
                dataContext,
                workerHub,
                sessionHub,
                viewerControls,
                cancellationToken),
            dataContext.ChangeTracker.Clear,
            cancellationToken);
    }

    private static async Task<IResult> DeleteAttempt(
        Guid id,
        int accountId,
        ClaimsPrincipal principal,
        DataContext dataContext,
        IHubContext<WorkerHub> workerHub,
        IHubContext<SessionHub> sessionHub,
        ViewerControlRegistry viewerControls,
        CancellationToken cancellationToken)
    {
        await using var transaction = dataContext.Database.IsRelational()
            ? await dataContext.Database.BeginTransactionAsync(
                IsolationLevel.ReadCommitted,
                cancellationToken)
            : null;
        var accountIsCurrent = await AccountResourceLock.AcquireSharedAndValidate(
            dataContext,
            accountId,
            principal,
            cancellationToken);
        if (!accountIsCurrent)
        {
            return Results.Unauthorized();
        }

        SimulationSession? session;
        if (dataContext.Database.IsRelational())
        {
            // StartTask and Delete must serialize on the same row. Loading the
            // command history only after this lock also gives sequence numbers
            // a single, committed order.
            session = await dataContext.SimulationSessions
                .FromSqlInterpolated($"""
                    SELECT *
                    FROM "SimulationSessions"
                    WHERE "Id" = {id} AND "AccountId" = {accountId}
                    FOR UPDATE
                    """)
                .SingleOrDefaultAsync(cancellationToken);
            if (session != null)
            {
                await dataContext.Entry(session)
                    .Reference(candidate => candidate.ComputeWorker)
                    .LoadAsync(cancellationToken);
                await dataContext.Entry(session)
                    .Collection(candidate => candidate.Commands)
                    .LoadAsync(cancellationToken);
            }
        }
        else
        {
            session = await dataContext.SimulationSessions
                .Include(candidate => candidate.ComputeWorker)
                .Include(candidate => candidate.Commands)
                .SingleOrDefaultAsync(
                    candidate => candidate.Id == id && candidate.AccountId == accountId,
                    cancellationToken);
        }

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

            var revocationVersion = await viewerControls.BeginSessionRevocationAsync(
                session.Id,
                DateTimeOffset.UtcNow,
                cancellationToken);
            var revocationCommitted = false;
            try
            {
                await dataContext.SaveChangesAsync(cancellationToken);
                if (transaction != null)
                {
                    await transaction.CommitAsync(cancellationToken);
                }
                viewerControls.ConfirmSessionRevocation(session.Id, revocationVersion);
                revocationCommitted = true;
                await viewerControls.DrainSessionAsync(
                    session.Id,
                    CancellationToken.None);
            }
            finally
            {
                if (!revocationCommitted)
                {
                    viewerControls.CancelSessionRevocation(session.Id, revocationVersion);
                }
            }

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

    internal static async Task<IResult> RetryDeleteSerializationFailures(
        Func<Task<IResult>> operation,
        Action resetContext,
        CancellationToken cancellationToken)
    {
        for (var attempt = 1; attempt <= MaximumDeleteAttempts; attempt++)
        {
            try
            {
                return await operation();
            }
            catch (Exception exception) when (IsSerializationFailure(exception))
            {
                resetContext();
                cancellationToken.ThrowIfCancellationRequested();
                if (attempt == MaximumDeleteAttempts)
                {
                    break;
                }

                await Task.Delay(
                    TimeSpan.FromMilliseconds(25 * attempt),
                    cancellationToken);
            }
        }

        return Results.Conflict(new
        {
            code = "serialization_conflict",
            retryable = true,
            message = "The session changed while it was stopping. Try again."
        });
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
