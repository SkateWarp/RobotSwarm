namespace SwarmBackend.Helpers;

public static class SessionLimits
{
    public const int DefaultMaxRobotsPerSession = 10;
    public const int DefaultMaxQueuedSessions = 25;
    public const int DefaultQueueTtlMinutes = 30;
    public const int DefaultSessionTtlMinutes = 180;

    public static int GetMaxRobotsPerSession(IConfiguration configuration)
    {
        var configured = configuration.GetValue<int?>("Sessions:MaxRobotsPerSession");
        return configured is >= 1 and <= 1000
            ? configured.Value
            : DefaultMaxRobotsPerSession;
    }

    public static int GetMaxQueuedSessions(IConfiguration configuration)
    {
        var configured = configuration.GetValue<int?>("Sessions:MaxQueuedSessions");
        return configured is >= 1 and <= 1000
            ? configured.Value
            : DefaultMaxQueuedSessions;
    }

    public static TimeSpan GetQueueTtl(IConfiguration configuration)
    {
        var configured = configuration.GetValue<int?>("Sessions:QueueTtlMinutes");
        var minutes = configured is >= 1 and <= 1440
            ? configured.Value
            : DefaultQueueTtlMinutes;
        return TimeSpan.FromMinutes(minutes);
    }

    public static TimeSpan GetSessionTtl(IConfiguration configuration)
    {
        var configured = configuration.GetValue<int?>("Sessions:SessionTtlMinutes");
        var minutes = configured is >= 5 and <= 1440
            ? configured.Value
            : DefaultSessionTtlMinutes;
        return TimeSpan.FromMinutes(minutes);
    }
}
