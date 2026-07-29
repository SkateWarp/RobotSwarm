using System.Security.Claims;
using System.Text.Encodings.Web;
using Microsoft.AspNetCore.Authentication;
using Microsoft.EntityFrameworkCore;
using Microsoft.Extensions.Options;

namespace SwarmBackend.Helpers;

public static class WorkerCredentialDefaults
{
    public const string AuthenticationScheme = "WorkerCredential";
    public const string AgentInstanceClaim = "worker_agent_instance_id";
    public const string AgentInstanceQueryParameter = "worker_instance_id";
}

public class WorkerCredentialAuthenticationHandler(
    IOptionsMonitor<AuthenticationSchemeOptions> options,
    ILoggerFactory logger,
    UrlEncoder encoder,
    DataContext dataContext)
    : AuthenticationHandler<AuthenticationSchemeOptions>(options, logger, encoder)
{
    protected override async Task<AuthenticateResult> HandleAuthenticateAsync()
    {
        var credential = GetCredential();
        if (string.IsNullOrWhiteSpace(credential)
            || !WorkerCredential.TryParse(credential, out var workerId, out var secret))
        {
            return AuthenticateResult.NoResult();
        }

        var worker = await dataContext.ComputeWorkers
            .AsNoTracking()
            .SingleOrDefaultAsync(candidate => candidate.Id == workerId, Context.RequestAborted);

        if (worker?.CredentialHash == null
            || worker.CredentialRevokedAt.HasValue
            || !WorkerCredential.Verify(secret, worker.CredentialHash))
        {
            return AuthenticateResult.Fail("Invalid worker credential.");
        }

        if (!TryParseAgentInstanceId(
                Request.Path.StartsWithSegments("/hubs/worker")
                    ? Request.Query[WorkerCredentialDefaults.AgentInstanceQueryParameter]
                        .ToString()
                    : null,
                out var agentInstanceId))
        {
            return AuthenticateResult.Fail("Invalid worker instance identifier.");
        }

        var claims = new List<Claim>
        {
            new Claim(ClaimTypes.NameIdentifier, worker.Id.ToString()),
            new Claim(ClaimTypes.Name, worker.Name),
            new Claim("worker_id", worker.Id.ToString()),
            new Claim(
                "worker_credential_version",
                worker.CredentialCreatedAt?.Ticks.ToString() ?? string.Empty)
        };
        if (agentInstanceId.HasValue)
        {
            claims.Add(new Claim(
                WorkerCredentialDefaults.AgentInstanceClaim,
                agentInstanceId.Value.ToString("D")));
        }

        var identity = new ClaimsIdentity(claims, Scheme.Name);
        var principal = new ClaimsPrincipal(identity);
        return AuthenticateResult.Success(new AuthenticationTicket(principal, Scheme.Name));
    }

    private string? GetCredential()
    {
        var authorization = Request.Headers.Authorization.ToString();
        if (authorization.StartsWith("Bearer ", StringComparison.OrdinalIgnoreCase))
        {
            return authorization["Bearer ".Length..].Trim();
        }

        if (Request.Path.StartsWithSegments("/hubs/worker")
            && Request.Query.TryGetValue("access_token", out var accessToken))
        {
            return accessToken.ToString();
        }

        return null;
    }

    internal static bool TryParseAgentInstanceId(
        string? value,
        out Guid? agentInstanceId)
    {
        agentInstanceId = null;
        if (string.IsNullOrWhiteSpace(value))
        {
            return true;
        }

        if (!Guid.TryParseExact(value, "D", out var parsed)
            || parsed == Guid.Empty)
        {
            return false;
        }

        agentInstanceId = parsed;
        return true;
    }
}
