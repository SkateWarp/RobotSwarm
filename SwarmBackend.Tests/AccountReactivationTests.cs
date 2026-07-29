using System.Security.Claims;
using Microsoft.AspNetCore.Authorization;
using Microsoft.AspNetCore.Authorization.Infrastructure;
using Microsoft.AspNetCore.Builder;
using Microsoft.AspNetCore.Routing;
using Microsoft.EntityFrameworkCore;
using Microsoft.Extensions.Configuration;
using Microsoft.Extensions.DependencyInjection;
using SwarmBackend.Entities;
using SwarmBackend.Interfaces;
using SwarmBackend.Models;
using SwarmBackend.Routes;
using SwarmBackend.Services;

namespace SwarmBackend.Tests;

public sealed class AccountReactivationTests
{
    [Fact]
    public async Task ReactivationRouteIsAnAdministratorOnlyPut()
    {
        var builder = WebApplication.CreateBuilder();
        builder.Services.AddScoped<IAccountService>(_ => null!);
        await using var app = builder.Build();
        app.MapGroup("/Accounts").MapAccount();

        var endpoint = ((IEndpointRouteBuilder)app).DataSources
            .SelectMany(source => source.Endpoints)
            .OfType<RouteEndpoint>()
            .Single(candidate =>
                candidate.RoutePattern.RawText == "/Accounts/{accountId:int}/reactivate");
        var method = Assert.Single(
            endpoint.Metadata.GetMetadata<HttpMethodMetadata>()!.HttpMethods);
        var policy = Assert.IsType<AuthorizationPolicy>(
            endpoint.Metadata.GetMetadata<AuthorizationPolicy>());
        var roles = Assert.Single(
            policy.Requirements.OfType<RolesAuthorizationRequirement>());

        Assert.Equal("PUT", method);
        Assert.Equal(new[] { Role.Admin.ToString() }, roles.AllowedRoles.ToArray());
    }

    [Fact]
    public async Task AdministratorReactivatesAccountWithoutRestoringOldRuntimeResources()
    {
        await using var dataContext = TestDataContext.Create();
        var administrator = Account(1, "admin@example.test", Role.Admin, enabled: true);
        var target = Account(2, "operator@example.test", Role.User, enabled: false);
        target.RefreshTokens.Add(new RefreshToken
        {
            Token = "old-token",
            CreatedByIp = "127.0.0.1",
            Created = DateTime.UtcNow.AddHours(-1),
            Expires = DateTime.UtcNow.AddDays(1)
        });
        var session = new SimulationSession
        {
            Account = target,
            AccountId = target.Id,
            DesiredRobotCount = 3,
            State = SimulationSessionState.Stopped,
            StoppedAt = DateTime.UtcNow.AddMinutes(-10)
        };
        var leaseRevokedAt = DateTime.UtcNow.AddMinutes(-11);
        var lease = new ViewerLease
        {
            SimulationSession = session,
            SimulationSessionId = session.Id,
            Account = target,
            AccountId = target.Id,
            TokenHash = "revoked-viewer",
            CreatedAt = DateTime.UtcNow.AddHours(-1),
            ExpiresAt = DateTime.UtcNow.AddHours(1),
            RevokedAt = leaseRevokedAt
        };
        dataContext.AddRange(administrator, target, session, lease);
        await dataContext.SaveChangesAsync();
        var service = new AccountService(dataContext, Configuration());

        var response = Success(await service.ReactivateAuthorized(
            administrator.Id,
            Principal(administrator),
            target.Id));

        Assert.True(response.Enabled);
        dataContext.ChangeTracker.Clear();
        var reactivated = await dataContext.Accounts
            .Include(account => account.RefreshTokens)
            .SingleAsync(account => account.Id == target.Id);
        var unchangedSession = await dataContext.SimulationSessions
            .SingleAsync(candidate => candidate.Id == session.Id);
        var unchangedLease = await dataContext.ViewerLeases
            .SingleAsync(candidate => candidate.Id == lease.Id);

        Assert.True(reactivated.Enabled);
        Assert.NotNull(reactivated.Updated);
        Assert.NotNull(Assert.Single(reactivated.RefreshTokens).Revoked);
        Assert.Equal(SimulationSessionState.Stopped, unchangedSession.State);
        Assert.Equal(leaseRevokedAt, unchangedLease.RevokedAt);

        var firstUpdate = reactivated.Updated;
        await service.ReactivateAuthorized(
            administrator.Id,
            Principal(administrator),
            target.Id);
        dataContext.ChangeTracker.Clear();
        Assert.Equal(
            firstUpdate,
            (await dataContext.Accounts.SingleAsync(account => account.Id == target.Id)).Updated);
    }

    [Fact]
    public async Task StaleOrNonAdministratorActorCannotReactivateAnotherAccount()
    {
        await using var dataContext = TestDataContext.Create();
        var administrator = Account(1, "admin@example.test", Role.Admin, enabled: true);
        administrator.Updated = DateTime.UtcNow;
        var user = Account(2, "user@example.test", Role.User, enabled: true);
        var target = Account(3, "target@example.test", Role.User, enabled: false);
        dataContext.AddRange(administrator, user, target);
        await dataContext.SaveChangesAsync();
        var service = new AccountService(dataContext, Configuration());

        var stale = Failure(await service.ReactivateAuthorized(
            administrator.Id,
            Principal(administrator, accountVersion: "0"),
            target.Id));
        var nonAdministrator = Failure(await service.ReactivateAuthorized(
            user.Id,
            Principal(user),
            target.Id));

        Assert.IsType<UnauthorizedAccessException>(stale);
        Assert.IsType<UnauthorizedAccessException>(nonAdministrator);
        dataContext.ChangeTracker.Clear();
        Assert.False(
            (await dataContext.Accounts.SingleAsync(account => account.Id == target.Id)).Enabled);
    }

    private static Account Account(
        int id,
        string email,
        Role role,
        bool enabled)
    {
        return new Account
        {
            Id = id,
            FirstName = "RobotSwarm",
            LastName = "Operator",
            Email = email,
            PasswordHash = "not-used",
            Enabled = enabled,
            Role = role
        };
    }

    private static ClaimsPrincipal Principal(
        Account account,
        string? accountVersion = null)
    {
        return new ClaimsPrincipal(new ClaimsIdentity(new[]
        {
            new Claim("id", account.Id.ToString()),
            new Claim(ClaimTypes.Role, account.Role.ToString()),
            new Claim(
                "account_version",
                accountVersion ?? (account.Updated?.Ticks ?? 0L).ToString())
        }, "test"));
    }

    private static AccountResponse Success(
        LanguageExt.Common.Result<AccountResponse> result)
    {
        return result.Match(value => value, error => throw error);
    }

    private static Exception Failure(
        LanguageExt.Common.Result<AccountResponse> result)
    {
        return result.Match<Exception>(
            _ => throw new InvalidOperationException("Se esperaba un error"),
            error => error);
    }

    private static IConfiguration Configuration()
    {
        return new ConfigurationBuilder()
            .AddInMemoryCollection(new Dictionary<string, string?>
            {
                ["AppSettings:Secret"] = Convert.ToBase64String(new byte[32]),
                ["AppSettings:Issuer"] = "robotswarm-tests",
                ["AppSettings:ExpiresIn"] = "15"
            })
            .Build();
    }
}
