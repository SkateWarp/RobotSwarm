namespace SwarmBackend.Helpers;

public static class ControlPlaneGroups
{
    public static string Session(Guid sessionId) => $"session:{sessionId:N}";
    public static string Worker(Guid workerId) => $"worker:{workerId:N}";
}
