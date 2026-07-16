namespace SwarmBackend.Helpers;

public static class SessionLimits
{
    public const int DefaultMaxRobotsPerSession = 10;

    public static int GetMaxRobotsPerSession(IConfiguration configuration)
    {
        var configured = configuration.GetValue<int?>("Sessions:MaxRobotsPerSession");
        return configured is >= 1 and <= 1000
            ? configured.Value
            : DefaultMaxRobotsPerSession;
    }
}
