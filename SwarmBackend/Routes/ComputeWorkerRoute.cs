using System.Text.Json;
using Microsoft.EntityFrameworkCore;
using Npgsql;
using SwarmBackend.Entities;
using SwarmBackend.Helpers;
using SwarmBackend.Models;
using SwarmBackend.Services;

namespace SwarmBackend.Routes;

public static class ComputeWorkerRoute
{
    public static RouteGroupBuilder MapComputeWorker(this RouteGroupBuilder group)
    {
        group.RequireAuthorization(policy => policy.RequireRole(Role.Admin.ToString()));

        group.MapPost("", Enroll)
            .Produces<ComputeWorkerEnrollmentResponse>(StatusCodes.Status201Created)
            .Produces(StatusCodes.Status400BadRequest)
            .Produces(StatusCodes.Status409Conflict);
        group.MapGet("", GetAll)
            .Produces<IEnumerable<ComputeWorkerResponse>>();
        group.MapPost("/{id:guid}/rotate-credential", RotateCredential)
            .Produces<ComputeWorkerEnrollmentResponse>();
        group.MapDelete("/{id:guid}/credential", RevokeCredential)
            .Produces(StatusCodes.Status204NoContent)
            .Produces(StatusCodes.Status404NotFound);

        return group;
    }

    private static async Task<IResult> Enroll(
        EnrollComputeWorkerRequest request,
        DataContext dataContext,
        CancellationToken cancellationToken)
    {
        var validation = Validate(request);
        if (validation != null)
        {
            return validation;
        }

        var normalizedName = request.Name.Trim();
        if (await dataContext.ComputeWorkers.AnyAsync(
                worker => worker.Name == normalizedName,
                cancellationToken))
        {
            return Results.Conflict(new { message = "A compute worker with this name already exists." });
        }

        var now = DateTime.UtcNow;
        var (secret, credentialHash) = WorkerCredential.Generate();
        var worker = new ComputeWorker
        {
            Name = normalizedName,
            State = ComputeWorkerState.Offline,
            MaxConcurrentSessions = request.MaxConcurrentSessions,
            Capabilities = ControlPlaneJson.ToDocument(request.Capabilities),
            CredentialHash = credentialHash,
            CredentialCreatedAt = now,
            CreatedAt = now,
            UpdatedAt = now
        };

        dataContext.ComputeWorkers.Add(worker);
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
            return Results.Conflict(new { message = "A compute worker with this name already exists." });
        }

        var response = new ComputeWorkerEnrollmentResponse(
            ToResponse(worker),
            $"{worker.Id}.{secret}");
        return Results.Created($"/api/workers/{worker.Id}", response);
    }

    private static async Task<IResult> GetAll(
        DataContext dataContext,
        CancellationToken cancellationToken)
    {
        var workers = await dataContext.ComputeWorkers
            .AsNoTracking()
            .OrderBy(worker => worker.Name)
            .ToListAsync(cancellationToken);
        return Results.Ok(workers.Select(ToResponse));
    }

    private static async Task<IResult> RotateCredential(
        Guid id,
        DataContext dataContext,
        WorkerConnectionRegistry connectionRegistry,
        CancellationToken cancellationToken)
    {
        var worker = await dataContext.ComputeWorkers
            .SingleOrDefaultAsync(candidate => candidate.Id == id, cancellationToken);
        if (worker == null)
        {
            return Results.NotFound();
        }

        var now = DateTime.UtcNow;
        var (secret, credentialHash) = WorkerCredential.Generate();
        worker.CredentialHash = credentialHash;
        worker.CredentialCreatedAt = now;
        worker.CredentialRevokedAt = null;
        WorkerDrainLease.Clear(worker);
        worker.State = ComputeWorkerState.Offline;
        worker.UpdatedAt = now;
        await dataContext.SaveChangesAsync(cancellationToken);
        connectionRegistry.Invalidate(worker.Id);

        return Results.Ok(new ComputeWorkerEnrollmentResponse(
            ToResponse(worker),
            $"{worker.Id}.{secret}"));
    }

    private static async Task<IResult> RevokeCredential(
        Guid id,
        DataContext dataContext,
        WorkerConnectionRegistry connectionRegistry,
        CancellationToken cancellationToken)
    {
        var worker = await dataContext.ComputeWorkers
            .SingleOrDefaultAsync(candidate => candidate.Id == id, cancellationToken);
        if (worker == null)
        {
            return Results.NotFound();
        }

        var now = DateTime.UtcNow;
        worker.CredentialRevokedAt = now;
        WorkerDrainLease.Clear(worker);
        worker.State = ComputeWorkerState.Offline;
        worker.UpdatedAt = now;
        await dataContext.SaveChangesAsync(cancellationToken);
        connectionRegistry.Invalidate(worker.Id);
        return Results.NoContent();
    }

    private static IResult? Validate(EnrollComputeWorkerRequest request)
    {
        var errors = new Dictionary<string, string[]>();
        if (string.IsNullOrWhiteSpace(request.Name))
        {
            errors[nameof(request.Name)] = new[] { "Name is required." };
        }
        else if (request.Name.Trim().Length > 100)
        {
            errors[nameof(request.Name)] = new[] { "Name cannot exceed 100 characters." };
        }

        if (request.MaxConcurrentSessions is < 1 or > 32)
        {
            errors[nameof(request.MaxConcurrentSessions)] =
                new[] { "MaxConcurrentSessions must be between 1 and 32." };
        }

        if (request.Capabilities.HasValue
            && request.Capabilities.Value.ValueKind is not JsonValueKind.Object
            and not JsonValueKind.Null
            and not JsonValueKind.Undefined)
        {
            errors[nameof(request.Capabilities)] =
                new[] { "Capabilities must be a JSON object." };
        }

        return errors.Count == 0 ? null : Results.ValidationProblem(errors);
    }

    internal static ComputeWorkerResponse ToResponse(ComputeWorker worker)
    {
        return new ComputeWorkerResponse(
            worker.Id,
            worker.Name,
            worker.State.ToString(),
            worker.MaxConcurrentSessions,
            worker.ImageVersion,
            ControlPlaneJson.ToElement(worker.Capabilities),
            worker.CreatedAt,
            worker.UpdatedAt,
            worker.LastHeartbeatAt,
            worker.CredentialCreatedAt,
            worker.CredentialRevokedAt,
            worker.ReportedActiveSessionCount,
            worker.ActiveSessionsReportedAt,
            worker.DrainLeaseId,
            worker.DrainTargetRevision,
            worker.DrainRequestedAt,
            worker.DrainLeaseExpiresAt);
    }
}
