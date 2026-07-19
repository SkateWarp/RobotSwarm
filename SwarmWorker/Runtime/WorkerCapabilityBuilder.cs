using System.Text.Json;
using SwarmWorker.Configuration;

namespace SwarmWorker.Runtime;

public static class WorkerCapabilityBuilder
{
    private static readonly string[] BaseCommandTypes =
    {
        "ProvisionSession",
        "UpdateFleet",
        "StartTask",
        "PauseTask",
        "ResumeTask",
        "CancelTask",
        "EmergencyStop",
        "ResetEmergencyStop",
        "StopSession"
    };

    public static JsonElement Build(
        WorkerOptions options,
        ViewerPublisherAvailability viewer)
    {
        var commandTypes = viewer.IsAvailable
            ? BaseCommandTypes.Append("SetViewerSource").ToArray()
            : BaseCommandTypes;
        var viewerSources = viewer.IsAvailable
            ? viewer.Sources.Select(source => source.ToString()).ToArray()
            : Array.Empty<string>();

        return JsonSerializer.SerializeToElement(new
        {
            workerName = options.Name,
            maxConcurrentSessions = options.MaxConcurrentSessions,
            maxRobotsPerSession = options.MaxRobotsPerSession,
            sessionImage = options.SessionImage,
            gpuEnabled = options.EnableGpu,
            gpuRequest = options.EnableGpu ? options.GpuRequest : null,
            turtleBotModel = "burger",
            rosDistribution = "noetic",
            gazeboGeneration = "classic-11",
            commandTypes,
            viewerSources,
            viewerPublisher = new
            {
                configured = options.Viewer.Enabled,
                available = viewer.IsAvailable,
                videoCodec = viewer.VideoCodec,
                protocolVersion = viewer.IsAvailable ? 2 : (int?)null,
                interactive = viewer.IsAvailable
            },
            taskOutcomes = new
            {
                collaborativeTransportEvidenceVersion = 1
            },
            isolation = new
            {
                perSessionContainer = true,
                internalDockerNetwork = true,
                publishedRosPorts = false,
                privileged = false
            },
            safety = new
            {
                controlHeartbeatSeconds = options.ControlHeartbeatIntervalSeconds,
                backendDisconnectStopSeconds =
                    options.BackendDisconnectEmergencyStopSeconds,
                emergencyStopConfirmationSeconds =
                    options.EmergencyStopTimeoutSeconds,
                taskCancellationConfirmationSeconds =
                    options.TaskCancellationTimeoutSeconds
            }
        });
    }
}
