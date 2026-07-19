using System.Globalization;
using System.Threading.RateLimiting;
using Microsoft.AspNetCore.RateLimiting;

namespace SwarmBackend.Helpers;

public static class AbuseProtection
{
    public const string AuthenticationPolicy = "account-authentication";
    public const string RegistrationPolicy = "account-registration";
    public const string SessionCreationPolicy = "session-creation";

    private const int DefaultAuthenticationLimit = 10;
    private const int DefaultAuthenticationWindowSeconds = 60;
    private const int DefaultRegistrationLimit = 3;
    private const int DefaultRegistrationWindowMinutes = 60;
    private const int DefaultSessionCreationLimit = 6;
    private const int DefaultSessionCreationWindowSeconds = 60;

    public static IServiceCollection AddAbuseProtection(
        this IServiceCollection services,
        IConfiguration configuration)
    {
        var authenticationLimit = ReadBoundedInt(
            configuration,
            "RateLimits:Authentication:PermitLimit",
            DefaultAuthenticationLimit,
            1,
            100);
        var authenticationWindow = TimeSpan.FromSeconds(ReadBoundedInt(
            configuration,
            "RateLimits:Authentication:WindowSeconds",
            DefaultAuthenticationWindowSeconds,
            10,
            3_600));
        var registrationLimit = ReadBoundedInt(
            configuration,
            "RateLimits:Registration:PermitLimit",
            DefaultRegistrationLimit,
            1,
            100);
        var registrationWindow = TimeSpan.FromMinutes(ReadBoundedInt(
            configuration,
            "RateLimits:Registration:WindowMinutes",
            DefaultRegistrationWindowMinutes,
            1,
            1_440));
        var sessionCreationLimit = ReadBoundedInt(
            configuration,
            "RateLimits:SessionCreation:PermitLimit",
            DefaultSessionCreationLimit,
            1,
            100);
        var sessionCreationWindow = TimeSpan.FromSeconds(ReadBoundedInt(
            configuration,
            "RateLimits:SessionCreation:WindowSeconds",
            DefaultSessionCreationWindowSeconds,
            10,
            3_600));

        return services.AddRateLimiter(options =>
        {
            options.RejectionStatusCode = StatusCodes.Status429TooManyRequests;
            options.AddPolicy(AuthenticationPolicy, context =>
                FixedWindow(
                    AuthenticationPartitionKey(context),
                    authenticationLimit,
                    authenticationWindow));
            options.AddPolicy(RegistrationPolicy, context =>
                FixedWindow(
                    RegistrationPartitionKey(context),
                    registrationLimit,
                    registrationWindow));
            options.AddPolicy(SessionCreationPolicy, context =>
                FixedWindow(
                    SessionCreationPartitionKey(context),
                    sessionCreationLimit,
                    sessionCreationWindow));
        });
    }

    internal static string RegistrationPartitionKey(HttpContext context)
    {
        return context.Connection.RemoteIpAddress?.ToString() ?? "unknown";
    }

    internal static string AuthenticationPartitionKey(HttpContext context)
    {
        return $"ip:{RegistrationPartitionKey(context)}";
    }

    internal static string SessionCreationPartitionKey(HttpContext context)
    {
        var accountId = context.User.FindFirst("id")?.Value;
        if (int.TryParse(
                accountId,
                NumberStyles.None,
                CultureInfo.InvariantCulture,
                out var parsedAccountId))
        {
            return $"account:{parsedAccountId}";
        }

        return $"ip:{RegistrationPartitionKey(context)}";
    }

    internal static int ReadBoundedInt(
        IConfiguration configuration,
        string key,
        int fallback,
        int minimum,
        int maximum)
    {
        var configured = configuration.GetValue<int?>(key);
        return configured is not null
            && configured.Value >= minimum
            && configured.Value <= maximum
                ? configured.Value
                : fallback;
    }

    private static RateLimitPartition<string> FixedWindow(
        string partitionKey,
        int permitLimit,
        TimeSpan window)
    {
        return RateLimitPartition.GetFixedWindowLimiter(
            partitionKey,
            _ => new FixedWindowRateLimiterOptions
            {
                AutoReplenishment = true,
                PermitLimit = permitLimit,
                QueueLimit = 0,
                Window = window
            });
    }
}
