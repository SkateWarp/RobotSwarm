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
        Assert.DoesNotContain("StopViewer", commandTypes);
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
        Assert.Contains("StopViewer", commandTypes);
        Assert.Equal(new[] { "Scene" }, viewerSources);
        Assert.Equal(
            "H264",
            capabilities.GetProperty("viewerPublisher")
                .GetProperty("videoCodec")
                .GetString());
        Assert.Equal(
            2,
            capabilities.GetProperty("viewerPublisher")
                .GetProperty("protocolVersion")
                .GetInt32());
        Assert.True(
            capabilities.GetProperty("viewerPublisher")
                .GetProperty("interactive")
                .GetBoolean());
    }

    [Fact]
    public void AdvertisesTheCorrelatedTransportEvidenceContract()
    {
        var capabilities = WorkerCapabilityBuilder.Build(
            Options(),
            ViewerPublisherAvailability.Unavailable("not installed"));

        Assert.Equal(
            1,
            capabilities.GetProperty("taskOutcomes")
                .GetProperty("collaborativeTransportEvidenceVersion")
                .GetInt32());
    }

    private static WorkerOptions Options() =>
        new()
        {
            Name = "test-worker",
            SessionImage = "robotswarm/ros-noetic:test",
            Viewer = new ViewerPublisherOptions { Enabled = true }
        };
}
