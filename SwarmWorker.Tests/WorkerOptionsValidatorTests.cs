using SwarmWorker.Configuration;

namespace SwarmWorker.Tests;

public sealed class WorkerOptionsValidatorTests
{
    [Fact]
    public void DisconnectFailSafeKeepsEveryWorstCaseTermInsideThirtySeconds()
    {
        var options = new WorkerOptions();

        Assert.Equal(14, options.ControlHeartbeatBackendLeaseSeconds);
        Assert.Equal(1, WorkerOptions.ControlHeartbeatDiscoveryTimeoutSeconds);
        Assert.Equal(5, WorkerOptions.ControlHeartbeatPublishTimeoutSeconds);
        Assert.Equal(7, WorkerOptions.ControlHeartbeatDockerExecTimeoutSeconds);
        Assert.Equal(0.25, WorkerOptions.ControlHeartbeatDeadlineGuardSeconds);
        Assert.Equal(10, WorkerOptions.RosControlHeartbeatTimeoutSeconds);
        Assert.Equal(0.5, WorkerOptions.RosControlWatchdogCheckPeriodSeconds);
        Assert.Equal(15, WorkerOptions.RosControlHeartbeatMaximumFutureSeconds);
        Assert.Equal(8, options.MaximumHealthyControlHeartbeatGapSeconds);
        Assert.Equal(24.25, options.MaximumControlHeartbeatStopSeconds);
        Assert.True(
            options.MaximumControlHeartbeatStopSeconds
            <= options.BackendDisconnectEmergencyStopSeconds);
    }

    [Theory]
    [InlineData(14, 25, true)]
    [InlineData(14, 24, false)]
    public void ValidatorChecksTheCompleteHeartbeatFailSafeInequality(
        int backendLeaseSeconds,
        int declaredStopSeconds,
        bool accepted)
    {
        var options = ValidOptions();
        options.ControlHeartbeatBackendLeaseSeconds = backendLeaseSeconds;
        options.BackendDisconnectEmergencyStopSeconds = declaredStopSeconds;

        var result = new WorkerOptionsValidator().Validate(null, options);

        Assert.Equal(accepted, result.Succeeded);
        if (!accepted)
        {
            Assert.Contains(
                Assert.IsAssignableFrom<IEnumerable<string>>(result.Failures),
                failure => failure.Contains(
                    "worst-case control-heartbeat fail-safe",
                    StringComparison.Ordinal));
        }
    }

    [Fact]
    public void ValidatorKeepsTheBackendLeaseInsideRosMaximumFutureWindow()
    {
        var options = ValidOptions();
        options.ControlHeartbeatBackendLeaseSeconds = 15;

        var result = new WorkerOptionsValidator().Validate(null, options);

        Assert.False(result.Succeeded);
        Assert.Contains(
            Assert.IsAssignableFrom<IEnumerable<string>>(result.Failures),
            failure => failure.Contains(
                "maximum future deadline",
                StringComparison.Ordinal));
    }

    [Theory]
    [InlineData(2, true)]
    [InlineData(4, false)]
    public void ValidatorIncludesBoundedDiscoveryInTheHealthyHeartbeatGap(
        int heartbeatIntervalSeconds,
        bool accepted)
    {
        var options = ValidOptions();
        options.ControlHeartbeatIntervalSeconds = heartbeatIntervalSeconds;

        var result = new WorkerOptionsValidator().Validate(null, options);

        Assert.Equal(accepted, result.Succeeded);
        if (!accepted)
        {
            Assert.Contains(
                Assert.IsAssignableFrom<IEnumerable<string>>(result.Failures),
                failure => failure.Contains(
                    "bounded Docker discovery",
                    StringComparison.Ordinal));
        }
    }

    [Fact]
    public void ViewerPublisherHasEnoughTimeToReportStartup()
    {
        var options = new ViewerPublisherOptions();

        Assert.Equal(50, options.StartupTimeoutSeconds);
    }

    [Theory]
    [InlineData(95, true)]
    [InlineData(121, false)]
    public void ViewerWorkerBudgetCanStayAboveTheHelperBudget(int seconds, bool accepted)
    {
        var options = ValidOptions();
        options.Viewer.StartupTimeoutSeconds = seconds;

        var result = new WorkerOptionsValidator().Validate(null, options);

        Assert.Equal(accepted, result.Succeeded);
    }

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
