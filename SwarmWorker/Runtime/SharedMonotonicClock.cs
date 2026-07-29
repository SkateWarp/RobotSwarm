using System.Globalization;

namespace SwarmWorker.Runtime;

public static class SharedMonotonicClock
{
    private const string LinuxUptimePath = "/proc/uptime";

    public static double GetSeconds()
    {
        var firstField = File.ReadAllText(LinuxUptimePath)
            .Split((char[]?)null, StringSplitOptions.RemoveEmptyEntries)
            .FirstOrDefault();
        if (!double.TryParse(
                firstField,
                NumberStyles.AllowDecimalPoint,
                CultureInfo.InvariantCulture,
                out var seconds)
            || !double.IsFinite(seconds)
            || seconds < 0)
        {
            throw new InvalidOperationException(
                "Linux monotonic uptime is unavailable.");
        }

        return seconds;
    }
}
