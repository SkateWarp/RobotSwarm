using System.Net;
using System.Security.Claims;
using Microsoft.AspNetCore.Http;
using Microsoft.EntityFrameworkCore;
using Microsoft.Extensions.Configuration;
using Microsoft.Extensions.DependencyInjection;
using SwarmBackend.Entities;
using SwarmBackend.Helpers;
using SwarmBackend.Models;
using SwarmBackend.Routes;
using SwarmBackend.Services;

namespace SwarmBackend.Tests;

public sealed class AbuseProtectionTests
{
    [Fact]
    public async Task PublicRegistrationIsClosedWhenTheFlagIsMissing()
    {
        await using var dataContext = TestDataContext.Create();
        var configuration = Configuration();
        var service = new AccountService(dataContext, configuration);

        var result = await AccountRoute.Create(
            new AccountRequest("Public", "User", "public@example.test", "password"),
            service,
            configuration);

        Assert.Equal(StatusCodes.Status403Forbidden, await StatusCode(result));
        Assert.Empty(dataContext.Accounts);
    }

    [Theory]
    [InlineData(null, false)]
    [InlineData("invalid", false)]
    [InlineData("false", false)]
    [InlineData("true", true)]
    public void PublicRegistrationFlagFailsClosed(string? value, bool expected)
    {
        var configuration = Configuration(new Dictionary<string, string?>
        {
            ["Accounts:PublicRegistrationEnabled"] = value
        });

        Assert.Equal(expected, AccountRoute.IsPublicRegistrationEnabled(configuration));
    }

    [Fact]
    public async Task PublicRegistrationKeepsItsExistingContractWhenExplicitlyEnabled()
    {
        await using var dataContext = TestDataContext.Create();
        var configuration = Configuration(new Dictionary<string, string?>
        {
            ["Accounts:PublicRegistrationEnabled"] = "true"
        });
        var service = new AccountService(dataContext, configuration);

        var result = await AccountRoute.Create(
            new AccountRequest("Opted", "In", "opted-in@example.test", "password"),
            service,
            configuration);

        Assert.Equal(StatusCodes.Status200OK, await StatusCode(result));
        var account = Assert.Single(dataContext.Accounts);
        Assert.True(account.Enabled);
        Assert.True(account.IsVerified);
    }

    [Fact]
    public async Task AdminCreatedAccountsRemainEnabledAndVerified()
    {
        await using var dataContext = TestDataContext.Create();
        var service = new AccountService(dataContext, Configuration());

        await service.Create(
            new AccountRequest("Managed", "User", "managed@example.test", "password"),
            Role.User);

        var account = Assert.Single(dataContext.Accounts);
        Assert.True(account.Enabled);
        Assert.True(account.IsVerified);
    }

    [Fact]
    public async Task AccountListDistinguishesDisabledAccounts()
    {
        await using var dataContext = TestDataContext.Create();
        dataContext.Accounts.AddRange(
            new Account
            {
                FirstName = "Active",
                LastName = "User",
                Email = "active@example.test",
                Enabled = true
            },
            new Account
            {
                FirstName = "Disabled",
                LastName = "User",
                Email = "disabled@example.test",
                Enabled = false
            });
        await dataContext.SaveChangesAsync();

        var service = new AccountService(dataContext, Configuration());
        var accounts = (await service.GetAll(null, Role.Admin)).ToArray();

        Assert.True(accounts.Single(account => account.Email == "active@example.test").Enabled);
        Assert.False(accounts.Single(account => account.Email == "disabled@example.test").Enabled);
    }

    [Fact]
    public void RateLimitPartitionsUseTheClientAndAuthenticatedAccount()
    {
        var context = new DefaultHttpContext();
        context.Connection.RemoteIpAddress = IPAddress.Parse("192.0.2.25");

        Assert.Equal(
            "192.0.2.25",
            AbuseProtection.RegistrationPartitionKey(context));
        Assert.Equal(
            "ip:192.0.2.25",
            AbuseProtection.AuthenticationPartitionKey(context));
        Assert.Equal(
            "ip:192.0.2.25",
            AbuseProtection.ViewerHlsPartitionKey(context));
        Assert.Equal(
            "ip:192.0.2.25",
            AbuseProtection.SessionCreationPartitionKey(context));

        context.User = new ClaimsPrincipal(new ClaimsIdentity(
            new[] { new Claim("id", "42") },
            "test"));

        Assert.Equal(
            "account:42",
            AbuseProtection.SessionCreationPartitionKey(context));
    }

    [Fact]
    public void InvalidRateAndSessionLimitsFallBackToBoundedDefaults()
    {
        var configuration = Configuration(new Dictionary<string, string?>
        {
            ["RateLimits:Registration:PermitLimit"] = "0",
            ["RateLimits:Authentication:PermitLimit"] = "0",
            ["Sessions:MaxQueuedSessions"] = "0",
            ["Sessions:QueueTtlMinutes"] = "0",
            ["Sessions:SessionTtlMinutes"] = "1"
        });

        Assert.Equal(
            3,
            AbuseProtection.ReadBoundedInt(
                configuration,
                "RateLimits:Registration:PermitLimit",
                fallback: 3,
                minimum: 1,
                maximum: 100));
        Assert.Equal(
            10,
            AbuseProtection.ReadBoundedInt(
                configuration,
                "RateLimits:Authentication:PermitLimit",
                fallback: 10,
                minimum: 1,
                maximum: 100));
        Assert.Equal(25, SessionLimits.GetMaxQueuedSessions(configuration));
        Assert.Equal(TimeSpan.FromMinutes(30), SessionLimits.GetQueueTtl(configuration));
        Assert.Equal(TimeSpan.FromMinutes(180), SessionLimits.GetSessionTtl(configuration));
    }

    private static IConfiguration Configuration(
        Dictionary<string, string?>? values = null)
    {
        return new ConfigurationBuilder()
            .AddInMemoryCollection(values ?? new Dictionary<string, string?>())
            .Build();
    }

    private static async Task<int> StatusCode(IResult result)
    {
        var context = new DefaultHttpContext
        {
            RequestServices = new ServiceCollection()
                .AddLogging()
                .BuildServiceProvider()
        };
        await result.ExecuteAsync(context);
        return context.Response.StatusCode;
    }
}
