using System.Security.Claims;
using System.Text.Encodings.Web;
using Microsoft.AspNetCore.Authentication;
using Microsoft.EntityFrameworkCore;
using Microsoft.Extensions.Options;

namespace SwarmBackend.Helpers;

public static class WorkerCredentialDefaults
{
    public const string AuthenticationScheme = "WorkerCredential";
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

        var claims = new[]
        {
            new Claim(ClaimTypes.NameIdentifier, worker.Id.ToString()),
            new Claim(ClaimTypes.Name, worker.Name),
            new Claim("worker_id", worker.Id.ToString()),
            new Claim(
                "worker_credential_version",
                worker.CredentialCreatedAt?.Ticks.ToString() ?? string.Empty)
        };
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
}
