using System.Data;
using System.Security.Claims;
using System.Text.RegularExpressions;
using Microsoft.EntityFrameworkCore;
using SwarmBackend.Entities;
using SwarmBackend.Helpers;
using SwarmBackend.Models;

namespace SwarmBackend.Routes;

public static partial class WorkerMaintenanceRoute
{
    private static readonly TimeSpan LeaseDuration = TimeSpan.FromHours(2);

    public static RouteGroupBuilder MapWorkerMaintenance(this RouteGroupBuilder group)
    {
        group.RequireAuthorization(policy => policy
            .AddAuthenticationSchemes(WorkerCredentialDefaults.AuthenticationScheme)
            .RequireAuthenticatedUser());

        group.MapPost("/drain", AcquireDrain)
            .Produces<WorkerDrainStatusResponse>()
            .Produces(StatusCodes.Status400BadRequest)
            .Produces(StatusCodes.Status409Conflict);
        group.MapGet("/drain/{leaseId:guid}", GetDrain)
            .Produces<WorkerDrainStatusResponse>()
            .Produces(StatusCodes.Status404NotFound);
        group.MapDelete("/drain/{leaseId:guid}", ReleaseDrain)
            .Produces(StatusCodes.Status204NoContent)
            .Produces(StatusCodes.Status404NotFound);

        return group;
    }

    internal static async Task<IResult> AcquireDrain(
        AcquireWorkerDrainRequest request,
        DataContext dataContext,
        HttpContext context,
        CancellationToken cancellationToken)
    {
        if (!TryGetWorkerId(context, out var workerId))
        {
            return Results.Unauthorized();
        }

        var targetRevision = request.TargetRevision?.Trim().ToLowerInvariant();
        if (targetRevision == null || !GitRevision().IsMatch(targetRevision))
        {
            return Results.ValidationProblem(new Dictionary<string, string[]>
            {
                [nameof(request.TargetRevision)] =
                    new[] { "TargetRevision must be a full 40-character Git SHA." }
            });
        }

        await using var transaction = await dataContext.Database.BeginTransactionAsync(
            IsolationLevel.Serializable,
            cancellationToken);
        var worker = await dataContext.ComputeWorkers.SingleOrDefaultAsync(
            candidate => candidate.Id == workerId,
            cancellationToken);
        if (worker == null)
        {
            return Results.NotFound();
        }

        var now = DateTime.UtcNow;
        var leaseIsActive = WorkerDrainLease.IsActive(worker, now);
        if (leaseIsActive
            && !string.Equals(
                worker.DrainTargetRevision,
                targetRevision,
                StringComparison.Ordinal))
        {
            return Results.Conflict(new
            {
                message = "The worker already has a drain lease for another revision."
            });
        }

        if (!leaseIsActive)
        {
            WorkerDrainLease.Clear(worker);
            worker.DrainLeaseId = Guid.NewGuid();
            worker.DrainRequestedAt = now;
        }

        worker.DrainTargetRevision = targetRevision;
        worker.DrainLeaseExpiresAt = now + LeaseDuration;
        worker.State = ComputeWorkerState.Draining;
        worker.UpdatedAt = now;
        await dataContext.SaveChangesAsync(cancellationToken);

        var response = await BuildStatus(dataContext, worker, cancellationToken);
        await transaction.CommitAsync(cancellationToken);
        return Results.Ok(response);
    }

    internal static async Task<IResult> GetDrain(
        Guid leaseId,
        DataContext dataContext,
        HttpContext context,
        CancellationToken cancellationToken)
    {
        var worker = await FindOwnedLease(
            leaseId,
            dataContext,
            context,
            cancellationToken);
        if (worker == null || !WorkerDrainLease.IsActive(worker, DateTime.UtcNow))
        {
            return Results.NotFound();
        }

        return Results.Ok(await BuildStatus(dataContext, worker, cancellationToken));
    }

    internal static async Task<IResult> ReleaseDrain(
        Guid leaseId,
        DataContext dataContext,
        HttpContext context,
        CancellationToken cancellationToken)
    {
        var worker = await FindOwnedLease(
            leaseId,
            dataContext,
            context,
            cancellationToken);
        if (worker == null)
        {
            return Results.NotFound();
        }

        var now = DateTime.UtcNow;
        WorkerDrainLease.Clear(worker);
        worker.State = worker.LastHeartbeatAt >= now - TimeSpan.FromSeconds(30)
            ? ComputeWorkerState.Online
            : ComputeWorkerState.Offline;
        worker.UpdatedAt = now;
        await dataContext.SaveChangesAsync(cancellationToken);
        return Results.NoContent();
    }

    internal static async Task<WorkerDrainStatusResponse> BuildStatus(
        DataContext dataContext,
        ComputeWorker worker,
        CancellationToken cancellationToken = default)
    {
        var trackedSessionCount = await dataContext.SimulationSessions.CountAsync(
            session => session.ComputeWorkerId == worker.Id
                && (session.State < SimulationSessionState.Stopped
                    || session.Commands.Any(command =>
                        command.Type == WorkerCommandType.StopSession
                        && (command.State == WorkerCommandState.Pending
                            || command.State == WorkerCommandState.Dispatched
                            || command.State == WorkerCommandState.Acknowledged
                            || command.State == WorkerCommandState.Running))),
            cancellationToken);
        var reportBelongsToDrain = worker.ActiveSessionsReportedAt
            >= worker.DrainRequestedAt;
        var isDrained = worker.State == ComputeWorkerState.Draining
            && WorkerDrainLease.IsActive(worker, DateTime.UtcNow)
            && trackedSessionCount == 0
            && worker.ReportedActiveSessionCount == 0
            && reportBelongsToDrain;

        return new WorkerDrainStatusResponse(
            worker.Id,
            worker.State.ToString(),
            worker.DrainLeaseId!.Value,
            worker.DrainTargetRevision!,
            AsUtc(worker.DrainRequestedAt!.Value),
            AsUtc(worker.DrainLeaseExpiresAt!.Value),
            trackedSessionCount,
            worker.ReportedActiveSessionCount,
            worker.ActiveSessionsReportedAt.HasValue
                ? AsUtc(worker.ActiveSessionsReportedAt.Value)
                : null,
            isDrained);
    }

    private static DateTime AsUtc(DateTime value)
    {
        return value.Kind switch
        {
            DateTimeKind.Utc => value,
            DateTimeKind.Local => value.ToUniversalTime(),
            _ => DateTime.SpecifyKind(value, DateTimeKind.Utc)
        };
    }

    private static async Task<ComputeWorker?> FindOwnedLease(
        Guid leaseId,
        DataContext dataContext,
        HttpContext context,
        CancellationToken cancellationToken)
    {
        if (!TryGetWorkerId(context, out var workerId))
        {
            return null;
        }

        return await dataContext.ComputeWorkers.SingleOrDefaultAsync(
            worker => worker.Id == workerId && worker.DrainLeaseId == leaseId,
            cancellationToken);
    }

    private static bool TryGetWorkerId(HttpContext context, out Guid workerId)
    {
        var claim = context.User.FindFirst("worker_id")
            ?? context.User.FindFirst(ClaimTypes.NameIdentifier);
        workerId = Guid.Empty;
        return claim != null && Guid.TryParse(claim.Value, out workerId);
    }

    [GeneratedRegex("^[a-f0-9]{40}$", RegexOptions.CultureInvariant)]
    private static partial Regex GitRevision();
}
