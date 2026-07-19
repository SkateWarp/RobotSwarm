using System.Text.Json;
using Microsoft.AspNetCore.Authorization;
using Microsoft.AspNetCore.Authentication.JwtBearer;
using Microsoft.AspNetCore.SignalR;
using Microsoft.EntityFrameworkCore;
using SwarmBackend.Entities;
using SwarmBackend.Helpers;
using SwarmBackend.Models;

namespace SwarmBackend.Services;

[Authorize(AuthenticationSchemes = JwtBearerDefaults.AuthenticationScheme)]
public class SessionHub(
    DataContext dataContext,
    ViewerControlRegistry viewerControls) : Hub
{
    private static readonly TimeSpan DisconnectReleaseTimeout = TimeSpan.FromSeconds(1);

    public override async Task OnDisconnectedAsync(Exception? exception)
    {
        try
        {
            await ReleaseDisconnectedViewerControlAsync(
                Context.ConnectionId,
                viewerControls,
                DisconnectReleaseTimeout);
        }
        finally
        {
            await base.OnDisconnectedAsync(exception);
        }
    }

    internal static async Task ReleaseDisconnectedViewerControlAsync(
        string connectionId,
        ViewerControlRegistry registry,
        TimeSpan timeoutDuration)
    {
        using var timeout = new CancellationTokenSource(timeoutDuration);
        try
        {
            await registry.ReleaseDisconnectedConnectionAsync(
                connectionId,
                DateTimeOffset.UtcNow,
                timeout.Token);
        }
        catch (OperationCanceledException) when (timeout.IsCancellationRequested)
        {
            // The tombstone was recorded before waiting for the lease gate. The
            // reconciler can therefore finish the release without delaying SignalR.
        }
    }

    public async Task JoinSession(Guid sessionId)
    {
        var accountId = GetAccountId();
        var ownsSession = await dataContext.SimulationSessions
            .AsNoTracking()
            .AnyAsync(
                session => session.Id == sessionId && session.AccountId == accountId,
                Context.ConnectionAborted);

        if (!ownsSession)
        {
            throw new HubException("Simulation session not found.");
        }

        await Groups.AddToGroupAsync(
            Context.ConnectionId,
            ControlPlaneGroups.Session(sessionId),
            Context.ConnectionAborted);
    }

    public async Task LeaveSession(Guid sessionId)
    {
        var accountId = GetAccountId();
        var ownsSession = await dataContext.SimulationSessions
            .AsNoTracking()
            .AnyAsync(
                session => session.Id == sessionId && session.AccountId == accountId,
                Context.ConnectionAborted);

        if (!ownsSession)
        {
            throw new HubException("Simulation session not found.");
        }

        await Groups.RemoveFromGroupAsync(
            Context.ConnectionId,
            ControlPlaneGroups.Session(sessionId),
            Context.ConnectionAborted);
    }

    public async Task<ViewerControlAuthorizationResponse> BeginViewerControl(
        Guid sessionId,
        Guid leaseId)
    {
        var accountId = GetAccountId();
        var now = DateTimeOffset.UtcNow;
        var grant = await FindViewerControlGrant(
            dataContext,
            accountId,
            sessionId,
            leaseId,
            now,
            Context.ConnectionAborted);
        if (grant is null)
        {
            throw new HubException("Viewer control is not available for this session and lease.");
        }

        var authorization = await viewerControls.AuthorizeAsync(
            Context.ConnectionId,
            accountId,
            sessionId,
            leaseId,
            grant.WorkerId,
            grant.LeaseExpiresAt,
            now,
            Context.ConnectionAborted);
        if (authorization.Status == ViewerControlAuthorizationStatus.Occupied)
        {
            throw new HubException(
                "This viewer lease is already controlled by another connection.");
        }

        if (authorization.Status != ViewerControlAuthorizationStatus.Authorized)
        {
            throw new HubException(
                "The previous viewer input could not be released safely; retry authorization.");
        }

        // Revalidate after publishing the in-memory grant. Post-commit drains and
        // periodic reconciliation provide the remaining revocation coverage.
        var confirmedGrant = await FindViewerControlGrant(
            dataContext,
            accountId,
            sessionId,
            leaseId,
            DateTimeOffset.UtcNow,
            Context.ConnectionAborted);
        if (confirmedGrant is null
            || confirmedGrant.WorkerId != grant.WorkerId
            || !viewerControls.IsCurrent(Context.ConnectionId, authorization.Version))
        {
            await viewerControls.DrainVersionAsync(
                Context.ConnectionId,
                authorization.Version,
                Context.ConnectionAborted);
            throw new HubException("Viewer control authorization changed while it was being granted.");
        }

        return new ViewerControlAuthorizationResponse(
            sessionId,
            leaseId,
            authorization.AuthorizedUntil,
            ViewerControlRegistry.MaximumEventsPerSecond);
    }

    public async Task SendViewerInput(
        Guid sessionId,
        Guid leaseId,
        JsonElement inputEvent)
    {
        if (!ViewerInputNormalizer.TryNormalize(
                inputEvent,
                out var normalizedInput,
                out var error))
        {
            throw new HubException(error);
        }

        var authorization = await viewerControls.DispatchAsync(
            Context.ConnectionId,
            sessionId,
            leaseId,
            normalizedInput!,
            DateTimeOffset.UtcNow,
            Context.ConnectionAborted);
        if (authorization != ViewerControlCheck.Authorized)
        {
            throw new HubException(authorization switch
            {
                ViewerControlCheck.RateLimited =>
                    "Viewer input rate limit exceeded.",
                ViewerControlCheck.Expired =>
                    "Viewer control authorization expired; authorize it again.",
                _ => "Viewer control is not authorized for this connection."
            });
        }
    }

    internal sealed record ViewerControlGrant(
        Guid WorkerId,
        DateTimeOffset LeaseExpiresAt);

    internal static async Task<ViewerControlGrant?> FindViewerControlGrant(
        DataContext context,
        int accountId,
        Guid sessionId,
        Guid leaseId,
        DateTimeOffset now,
        CancellationToken cancellationToken)
    {
        var utcNow = now.UtcDateTime;
        var match = await context.ViewerLeases
            .AsNoTracking()
            .Where(lease => lease.Id == leaseId
                && lease.SimulationSessionId == sessionId
                && lease.AccountId == accountId
                && lease.SimulationSession.AccountId == accountId
                && lease.Account.Enabled
                && lease.RevokedAt == null
                && lease.ExpiresAt > utcNow
                && lease.SimulationSession.ComputeWorkerId != null
                && (lease.SimulationSession.State == SimulationSessionState.Ready
                    || lease.SimulationSession.State == SimulationSessionState.Active
                    || lease.SimulationSession.State == SimulationSessionState.Paused))
            .Select(lease => new
            {
                WorkerId = lease.SimulationSession.ComputeWorkerId!.Value,
                lease.ExpiresAt
            })
            .SingleOrDefaultAsync(cancellationToken);
        if (match is null)
        {
            return null;
        }

        var expiresAt = new DateTimeOffset(
            DateTime.SpecifyKind(match.ExpiresAt, DateTimeKind.Utc));
        return new ViewerControlGrant(match.WorkerId, expiresAt);
    }

    private int GetAccountId()
    {
        var accountIdClaim = Context.User?.FindFirst("id");
        if (accountIdClaim == null || !int.TryParse(accountIdClaim.Value, out var accountId))
        {
            throw new HubException("Authenticated account identifier is missing.");
        }

        return accountId;
    }
}
