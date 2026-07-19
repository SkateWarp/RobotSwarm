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

    internal static async Task<IResult> Authorize(
        ViewerAuthRequest request,
        DataContext dataContext,
        IConfiguration configuration,
        CancellationToken cancellationToken)
    {
        if (string.IsNullOrWhiteSpace(request.Token)
            || !ViewerStreamAddress.TryParse(request.Path, out var streamAddress))
        {
            return Results.Unauthorized();
        }

        if (string.Equals(request.Action, "read", StringComparison.OrdinalIgnoreCase))
        {
            return await AuthorizeRead(
                request.Token,
                streamAddress,
                dataContext,
                cancellationToken);
        }

        if (string.Equals(request.Action, "publish", StringComparison.OrdinalIgnoreCase))
        {
            return await AuthorizePublish(
                request.Token,
                streamAddress,
                dataContext,
                configuration,
                cancellationToken);
        }

        return Results.Unauthorized();
    }

    private static async Task<IResult> AuthorizeRead(
        string token,
        ViewerStreamAddress streamAddress,
        DataContext dataContext,
        CancellationToken cancellationToken)
    {
        var tokenHash = Convert.ToHexString(SHA256.HashData(Encoding.UTF8.GetBytes(token)));
        var now = DateTime.UtcNow;
        var lease = await dataContext.ViewerLeases
            .AsNoTracking()
            .SingleOrDefaultAsync(
                candidate => candidate.TokenHash == tokenHash
                    && candidate.SimulationSessionId == streamAddress.SessionId
                    && candidate.AccountId == candidate.SimulationSession.AccountId
                    && candidate.Account.Enabled
                    && candidate.SimulationSession.State < SimulationSessionState.Stopped
                    && !candidate.RevokedAt.HasValue
                    && candidate.ExpiresAt > now,
                cancellationToken);

        if (lease == null)
        {
            return Results.Unauthorized();
        }

        return streamAddress.Matches(lease)
            ? Results.Ok()
            : Results.Unauthorized();
    }

    private static async Task<IResult> AuthorizePublish(
        string token,
        ViewerStreamAddress streamAddress,
        DataContext dataContext,
        IConfiguration configuration,
        CancellationToken cancellationToken)
    {
        if (!configuration.GetValue<bool>("Viewer:WorkerPublishingEnabled"))
        {
            return Results.Unauthorized();
        }

        if (!ViewerPublishToken.TryHash(token, out var tokenHash))
        {
            return Results.Unauthorized();
        }

        var now = DateTime.UtcNow;
        var lease = await dataContext.ViewerLeases
            .AsNoTracking()
            .SingleOrDefaultAsync(
                candidate => candidate.PublishTokenHash == tokenHash
                    && candidate.SimulationSessionId == streamAddress.SessionId
                    && candidate.AccountId == candidate.SimulationSession.AccountId
                    && candidate.Account.Enabled
                    && candidate.SimulationSession.State < SimulationSessionState.Stopped
                    && !candidate.RevokedAt.HasValue
                    && candidate.ExpiresAt > now,
                cancellationToken);
        return lease != null && streamAddress.Matches(lease)
            ? Results.Ok()
            : Results.Unauthorized();
    }
}
