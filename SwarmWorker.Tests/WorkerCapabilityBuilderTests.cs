using SwarmWorker.Configuration;
using SwarmWorker.Runtime;

namespace SwarmWorker.Tests;

public sealed class WorkerCapabilityBuilderTests
{
    [Fact]
    public void DoesNotAdvertiseViewerCommandsWhenPublisherIsUnavailable()
    {
        var capabilities = WorkerCapabilityBuilder.Build(
            Options(),
            ViewerPublisherAvailability.Unavailable("not installed"));

        var commandTypes = capabilities.GetProperty("commandTypes")
            .EnumerateArray()
            .Select(value => value.GetString())
            .ToArray();

        Assert.DoesNotContain("SetViewerSource", commandTypes);
        Assert.Empty(capabilities.GetProperty("viewerSources").EnumerateArray());
        Assert.False(
            capabilities.GetProperty("viewerPublisher")
                .GetProperty("available")
                .GetBoolean());
    }

    [Fact]
    public void AdvertisesOnlySourcesConfirmedByPublisherProbe()
    {
        var capabilities = WorkerCapabilityBuilder.Build(
            Options(),
            new ViewerPublisherAvailability(
                true,
                new[] { ViewerSourceKind.Scene },
                "H264",
                "ready"));

        var commandTypes = capabilities.GetProperty("commandTypes")
            .EnumerateArray()
            .Select(value => value.GetString())
            .ToArray();
        var viewerSources = capabilities.GetProperty("viewerSources")
            .EnumerateArray()
            .Select(value => value.GetString())
            .ToArray();

        Assert.Contains("SetViewerSource", commandTypes);
        Assert.Equal(new[] { "Scene" }, viewerSources);
        Assert.Equal(
            "H264",
            capabilities.GetProperty("viewerPublisher")
                .GetProperty("videoCodec")
                .GetString());
    }

    private static WorkerOptions Options() =>
        new()
        {
            Name = "test-worker",
            SessionImage = "robotswarm/ros-noetic:test",
            Viewer = new ViewerPublisherOptions { Enabled = true }
        };
}
