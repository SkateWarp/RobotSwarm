using System.Security.Cryptography;
using System.Text;
using Microsoft.EntityFrameworkCore;
using SwarmBackend.Entities;
using SwarmBackend.Helpers;
using SwarmBackend.Models;

namespace SwarmBackend.Routes;

public static class ViewerAuthRoute
{
    public static RouteGroupBuilder MapViewerAuth(this RouteGroupBuilder group)
    {
        group.MapPost("/auth", Authorize);
        return group;
    }

    private static async Task<IResult> Authorize(
        ViewerAuthRequest request,
        DataContext dataContext,
        CancellationToken cancellationToken)
    {
        if (string.IsNullOrWhiteSpace(request.Token)
            || !TryParsePath(request.Path, out var sessionId, out var stream))
        {
            return Results.Unauthorized();
        }

        if (string.Equals(request.Action, "read", StringComparison.OrdinalIgnoreCase))
        {
            return await AuthorizeRead(
                request.Token,
                sessionId,
                stream,
                dataContext,
                cancellationToken);
        }

        if (string.Equals(request.Action, "publish", StringComparison.OrdinalIgnoreCase))
        {
            return await AuthorizePublish(
                request.Token,
                sessionId,
                dataContext,
                cancellationToken);
        }

        return Results.Unauthorized();
    }

    private static async Task<IResult> AuthorizeRead(
        string token,
        Guid sessionId,
        string stream,
        DataContext dataContext,
        CancellationToken cancellationToken)
    {
        var tokenHash = Convert.ToHexString(SHA256.HashData(Encoding.UTF8.GetBytes(token)));
        var now = DateTime.UtcNow;
        var lease = await dataContext.ViewerLeases
            .AsNoTracking()
            .SingleOrDefaultAsync(
                candidate => candidate.TokenHash == tokenHash
                    && candidate.SimulationSessionId == sessionId
                    && candidate.Account.Enabled
                    && candidate.SimulationSession.State < SimulationSessionState.Stopped
                    && !candidate.RevokedAt.HasValue
                    && candidate.ExpiresAt > now,
                cancellationToken);

        if (lease == null)
        {
            return Results.Unauthorized();
        }

        var streamMatches = lease.Source == ViewerSourceType.Scene
            ? string.Equals(stream, "scene", StringComparison.Ordinal)
            : string.Equals(stream, lease.RobotRuntimeId, StringComparison.Ordinal);
        return streamMatches ? Results.Ok() : Results.Unauthorized();
    }

    private static async Task<IResult> AuthorizePublish(
        string token,
        Guid sessionId,
        DataContext dataContext,
        CancellationToken cancellationToken)
    {
        if (!WorkerCredential.TryParse(token, out var workerId, out var secret))
        {
            return Results.Unauthorized();
        }

        var worker = await dataContext.ComputeWorkers
            .AsNoTracking()
            .SingleOrDefaultAsync(candidate => candidate.Id == workerId, cancellationToken);
        if (worker?.CredentialHash == null
            || worker.CredentialRevokedAt.HasValue
            || !WorkerCredential.Verify(secret, worker.CredentialHash))
        {
            return Results.Unauthorized();
        }

        var ownsSession = await dataContext.SimulationSessions
            .AsNoTracking()
            .AnyAsync(
                session => session.Id == sessionId
                    && session.ComputeWorkerId == workerId
                    && session.State < SimulationSessionState.Stopped,
                cancellationToken);
        return ownsSession ? Results.Ok() : Results.Unauthorized();
    }

    private static bool TryParsePath(string? path, out Guid sessionId, out string stream)
    {
        sessionId = Guid.Empty;
        stream = string.Empty;
        var parts = path?.Trim('/').Split('/', StringSplitOptions.RemoveEmptyEntries);
        if (parts is not { Length: >= 3 }
            || !string.Equals(parts[0], "session", StringComparison.Ordinal)
            || !Guid.TryParseExact(parts[1], "N", out sessionId))
        {
            return false;
        }

        stream = parts[2];
        return !string.IsNullOrWhiteSpace(stream);
    }
}
