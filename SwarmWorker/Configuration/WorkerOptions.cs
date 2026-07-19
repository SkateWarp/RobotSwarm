using System.Text.RegularExpressions;
using Microsoft.Extensions.Options;

namespace SwarmWorker.Configuration;

public sealed class WorkerOptions
{
    public const string SectionName = "Worker";

    public string BackendUrl { get; set; } = string.Empty;
    public Guid WorkerId { get; set; }
    public string WorkerSecret { get; set; } = string.Empty;
    public string Name { get; set; } = string.Empty;
    public int MaxConcurrentSessions { get; set; } = 4;
    public string SessionImage { get; set; } = string.Empty;
    public string? ImageVersion { get; set; }
    public bool AllowMutableSessionImage { get; set; }
    public int MaxRobotsPerSession { get; set; } = 10;

    public int CommandPollIntervalSeconds { get; set; } = 2;
    public int PullBatchSize { get; set; } = 25;
    public int MaxQueuedCommands { get; set; } = 64;
    public int MaxParallelCommands { get; set; } = 2;
    public int DockerCommandTimeoutSeconds { get; set; } = 30;
    public int RosReadyTimeoutSeconds { get; set; } = 180;
    public int FleetUpdateTimeoutSeconds { get; set; } = 120;
    public int EmergencyStopTimeoutSeconds { get; set; } = 10;
    public int BackendDisconnectEmergencyStopSeconds { get; set; } = 30;
    public int ControlHeartbeatIntervalSeconds { get; set; } = 2;
    public int ControlHeartbeatBackendLeaseSeconds { get; set; } = 15;
    public int TaskStatusPollIntervalSeconds { get; set; } = 2;
    public double TaskProgressReportStep { get; set; } = 0.02;
    public int ShutdownDrainSeconds { get; set; } = 30;

    public string DockerExecutable { get; set; } = "docker";
    public double ContainerCpuLimit { get; set; } = 12.0;
    public string ContainerMemory { get; set; } = "3g";
    public string ContainerMemorySwap { get; set; } = "3g";
    public int ContainerPidsLimit { get; set; } = 512;
    public string ContainerShmSize { get; set; } = "512m";
    public string ContainerUser { get; set; } = "1000:1000";
    public bool EnableGpu { get; set; } = true;
    public string GpuRequest { get; set; } = "device=0";
    public bool AllowInsecureTransport { get; set; }
    public ViewerPublisherOptions Viewer { get; set; } = new();

    public Uri GetWorkerHubUri()
    {
        var baseUri = new Uri(BackendUrl, UriKind.Absolute);
        return baseUri.AbsolutePath.TrimEnd('/').EndsWith("/hubs/worker", StringComparison.OrdinalIgnoreCase)
            ? baseUri
            : new Uri(baseUri, "/hubs/worker");
    }

    public string GetAccessToken() => $"{WorkerId:D}.{WorkerSecret}";
}

public sealed class WorkerOptionsValidator : IValidateOptions<WorkerOptions>
{
    private static readonly Regex DockerSizePattern =
        new(@"^[1-9][0-9]*(?:[bkmg])?$", RegexOptions.Compiled | RegexOptions.IgnoreCase);

    private static readonly Regex ContainerUserPattern =
        new(@"^[A-Za-z0-9_.-]+(?::[A-Za-z0-9_.-]+)?$", RegexOptions.Compiled);

    private static readonly Regex WorkerSecretPattern =
        new(@"^[A-Za-z0-9_-]+$", RegexOptions.Compiled);

    public ValidateOptionsResult Validate(string? name, WorkerOptions options)
    {
        var errors = new List<string>();

        if (!Uri.TryCreate(options.BackendUrl, UriKind.Absolute, out var backendUri)
            || (backendUri.Scheme != Uri.UriSchemeHttps && backendUri.Scheme != Uri.UriSchemeHttp))
        {
            errors.Add("Worker:BackendUrl must be an absolute HTTP(S) URL.");
        }
        else if (backendUri.Scheme != Uri.UriSchemeHttps && !options.AllowInsecureTransport)
        {
            errors.Add(
                "Worker:BackendUrl must use HTTPS unless Worker:AllowInsecureTransport is explicitly enabled.");
        }

        if (options.WorkerId == Guid.Empty)
        {
            errors.Add("Worker:WorkerId must be the enrolled worker UUID.");
        }

        if (string.IsNullOrWhiteSpace(options.WorkerSecret)
            || options.WorkerSecret.Length < 24
            || !WorkerSecretPattern.IsMatch(options.WorkerSecret))
        {
            errors.Add("Worker:WorkerSecret must contain the enrolled base64url secret.");
        }

        if (string.IsNullOrWhiteSpace(options.Name) || options.Name.Length > 100)
        {
            errors.Add("Worker:Name is required and must be at most 100 characters.");
        }

        if (options.MaxConcurrentSessions is < 1 or > 32)
        {
            errors.Add("Worker:MaxConcurrentSessions must be between 1 and 32.");
        }

        if (string.IsNullOrWhiteSpace(options.SessionImage)
            || options.SessionImage.Any(char.IsWhiteSpace)
            || options.SessionImage.Contains("://", StringComparison.Ordinal))
        {
            errors.Add("Worker:SessionImage must be a non-empty Docker image reference.");
        }
        else if (!options.AllowMutableSessionImage
                 && !Regex.IsMatch(
                     options.SessionImage,
                     @"^(?:.+@)?sha256:[a-fA-F0-9]{64}$",
                     RegexOptions.CultureInvariant))
        {
            errors.Add(
                "Worker:SessionImage must be pinned to a sha256 digest unless Worker:AllowMutableSessionImage is explicitly enabled.");
        }

        if (string.IsNullOrWhiteSpace(options.ImageVersion)
            || options.ImageVersion.Length > 200)
        {
            errors.Add(
                "Worker:ImageVersion is required and must be at most 200 characters.");
        }

        if (options.MaxRobotsPerSession is < 1 or > 1000)
        {
            errors.Add("Worker:MaxRobotsPerSession must be between 1 and 1000.");
        }

        ValidateRange(
            errors,
            options.CommandPollIntervalSeconds,
            1,
            60,
            "Worker:CommandPollIntervalSeconds");
        ValidateRange(errors, options.PullBatchSize, 1, 100, "Worker:PullBatchSize");
        ValidateRange(errors, options.MaxQueuedCommands, 1, 1000, "Worker:MaxQueuedCommands");
        ValidateRange(errors, options.MaxParallelCommands, 1, 32, "Worker:MaxParallelCommands");
        ValidateRange(
            errors,
            options.DockerCommandTimeoutSeconds,
            5,
            600,
            "Worker:DockerCommandTimeoutSeconds");
        ValidateRange(
            errors,
            options.RosReadyTimeoutSeconds,
            30,
            1800,
            "Worker:RosReadyTimeoutSeconds");
        ValidateRange(
            errors,
            options.FleetUpdateTimeoutSeconds,
            30,
            1800,
            "Worker:FleetUpdateTimeoutSeconds");
        ValidateRange(
            errors,
            options.EmergencyStopTimeoutSeconds,
            2,
            60,
            "Worker:EmergencyStopTimeoutSeconds");
        ValidateRange(
            errors,
            options.BackendDisconnectEmergencyStopSeconds,
            10,
            300,
            "Worker:BackendDisconnectEmergencyStopSeconds");
        ValidateRange(
            errors,
            options.ControlHeartbeatIntervalSeconds,
            1,
            5,
            "Worker:ControlHeartbeatIntervalSeconds");
        ValidateRange(
            errors,
            options.ControlHeartbeatBackendLeaseSeconds,
            12,
            60,
            "Worker:ControlHeartbeatBackendLeaseSeconds");
        if (options.ControlHeartbeatBackendLeaseSeconds
            >= options.BackendDisconnectEmergencyStopSeconds)
        {
            errors.Add(
                "Worker:ControlHeartbeatBackendLeaseSeconds must be shorter than Worker:BackendDisconnectEmergencyStopSeconds.");
        }
        ValidateRange(
            errors,
            options.TaskStatusPollIntervalSeconds,
            1,
            60,
            "Worker:TaskStatusPollIntervalSeconds");
        if (!double.IsFinite(options.TaskProgressReportStep)
            || options.TaskProgressReportStep is <= 0 or > 1)
        {
            errors.Add("Worker:TaskProgressReportStep must be greater than zero and at most 1.");
        }

        ValidateRange(
            errors,
            options.ShutdownDrainSeconds,
            5,
            300,
            "Worker:ShutdownDrainSeconds");

        if (string.IsNullOrWhiteSpace(options.DockerExecutable))
        {
            errors.Add("Worker:DockerExecutable is required.");
        }

        if (options.ContainerCpuLimit is <= 0 or > 128)
        {
            errors.Add("Worker:ContainerCpuLimit must be greater than zero and at most 128.");
        }

        ValidateDockerSize(errors, options.ContainerMemory, "Worker:ContainerMemory");
        ValidateDockerSize(errors, options.ContainerMemorySwap, "Worker:ContainerMemorySwap");
        ValidateDockerSize(errors, options.ContainerShmSize, "Worker:ContainerShmSize");

        if (options.ContainerPidsLimit is < 64 or > 32768)
        {
            errors.Add("Worker:ContainerPidsLimit must be between 64 and 32768.");
        }

        if (!ContainerUserPattern.IsMatch(options.ContainerUser))
        {
            errors.Add("Worker:ContainerUser must be a Docker user or uid[:gid] value.");
        }

        if (options.EnableGpu && string.IsNullOrWhiteSpace(options.GpuRequest))
        {
            errors.Add("Worker:GpuRequest is required when GPU access is enabled.");
        }

        if (options.Viewer.Enabled)
        {
            if (string.IsNullOrWhiteSpace(options.Viewer.PublisherExecutable)
                || !Path.IsPathFullyQualified(options.Viewer.PublisherExecutable))
            {
                errors.Add(
                    "Worker:Viewer:PublisherExecutable must be an absolute path when viewer publishing is enabled.");
            }

            if (!TryValidateViewerPublishBaseUrl(options.Viewer.PublishBaseUrl))
            {
                errors.Add(
                    "Worker:Viewer:PublishBaseUrl must be an RTSP(S) URL without credentials, query, or fragment when viewer publishing is enabled.");
            }
        }

        ValidateRange(
            errors,
            options.Viewer.ProbeTimeoutSeconds,
            1,
            30,
            "Worker:Viewer:ProbeTimeoutSeconds");
        ValidateRange(
            errors,
            options.Viewer.StartupTimeoutSeconds,
            2,
            60,
            "Worker:Viewer:StartupTimeoutSeconds");
        ValidateRange(
            errors,
            options.Viewer.StopTimeoutSeconds,
            1,
            30,
            "Worker:Viewer:StopTimeoutSeconds");
        ValidateRange(
            errors,
            options.Viewer.MaximumLeaseMinutes,
            1,
            30,
            "Worker:Viewer:MaximumLeaseMinutes");

        return errors.Count == 0
            ? ValidateOptionsResult.Success
            : ValidateOptionsResult.Fail(errors);
    }

    private static void ValidateRange(
        ICollection<string> errors,
        int value,
        int minimum,
        int maximum,
        string setting)
    {
        if (value < minimum || value > maximum)
        {
            errors.Add($"{setting} must be between {minimum} and {maximum}.");
        }
    }

    private static void ValidateDockerSize(
        ICollection<string> errors,
        string value,
        string setting)
    {
        if (string.IsNullOrWhiteSpace(value) || !DockerSizePattern.IsMatch(value))
        {
            errors.Add($"{setting} must be a positive Docker size such as 512m or 3g.");
        }
    }

    private static bool TryValidateViewerPublishBaseUrl(string value)
    {
        if (!Uri.TryCreate(value, UriKind.Absolute, out var uri)
            || (uri.Scheme != "rtsp" && uri.Scheme != "rtsps")
            || string.IsNullOrWhiteSpace(uri.Host)
            || !string.IsNullOrEmpty(uri.UserInfo)
            || !string.IsNullOrEmpty(uri.Query)
            || !string.IsNullOrEmpty(uri.Fragment))
        {
            return false;
        }

        try
        {
            _ = uri.Port;
            return true;
        }
        catch (UriFormatException)
        {
            return false;
        }
    }
}

public sealed class ViewerPublisherOptions
{
    public bool Enabled { get; set; }
    public string PublisherExecutable { get; set; } = string.Empty;
    public string PublishBaseUrl { get; set; } = string.Empty;
    public int ProbeTimeoutSeconds { get; set; } = 5;
    public int StartupTimeoutSeconds { get; set; } = 15;
    public int StopTimeoutSeconds { get; set; } = 5;
    public int MaximumLeaseMinutes { get; set; } = 30;
}
