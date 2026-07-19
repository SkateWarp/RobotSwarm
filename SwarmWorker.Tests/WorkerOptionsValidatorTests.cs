using SwarmWorker.Configuration;

namespace SwarmWorker.Tests;

public sealed class WorkerOptionsValidatorTests
{
    [Fact]
    public void ViewerDisabledDoesNotRequirePublisherSettings()
    {
        var options = ValidOptions();

        var result = new WorkerOptionsValidator().Validate(null, options);

        Assert.True(result.Succeeded);
    }

    [Fact]
    public void ViewerEnabledRequiresAnAbsoluteHelperAndSafeRtspEndpoint()
    {
        var options = ValidOptions();
        options.Viewer.Enabled = true;
        options.Viewer.PublisherExecutable = "viewer-publisher";
        options.Viewer.PublishBaseUrl = "rtsp://user:secret@media.example.test/live?token=bad";

        var result = new WorkerOptionsValidator().Validate(null, options);

        Assert.False(result.Succeeded);
        var failures = Assert.IsAssignableFrom<IEnumerable<string>>(result.Failures);
        Assert.Contains(
            failures,
            failure => failure.Contains("PublisherExecutable", StringComparison.Ordinal));
        Assert.Contains(
            failures,
            failure => failure.Contains("PublishBaseUrl", StringComparison.Ordinal));
    }

    [Fact]
    public void ViewerEnabledAcceptsVersionedHelperAndCredentialFreeRtspEndpoint()
    {
        var options = ValidOptions();
        options.Viewer.Enabled = true;
        options.Viewer.PublisherExecutable =
            "/home/worker/.local/share/robotswarm/releases/v1/robotswarm-viewer-publisher";
        options.Viewer.PublishBaseUrl = "rtsps://media.example.test:8322/robotswarm";

        var result = new WorkerOptionsValidator().Validate(null, options);

        Assert.True(result.Succeeded);
    }

    private static WorkerOptions ValidOptions() =>
        new()
        {
            BackendUrl = "https://backend.example.test",
            WorkerId = Guid.NewGuid(),
            WorkerSecret = new string('a', 32),
            Name = "test-worker",
            SessionImage = "sha256:" + new string('b', 64),
            ImageVersion = "test-version"
        };
}
