using Microsoft.AspNetCore.Authorization;
using Microsoft.AspNetCore.Authentication.JwtBearer;
using Microsoft.AspNetCore.SignalR;
using Microsoft.EntityFrameworkCore;
using SwarmBackend.Helpers;

namespace SwarmBackend.Services;

[Authorize(AuthenticationSchemes = JwtBearerDefaults.AuthenticationScheme)]
public class SessionHub(DataContext dataContext) : Hub
{
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
