using System.Text.Json;
using SwarmWorker.Contracts;
using SwarmWorker.Runtime;

namespace SwarmWorker.Tests;

public sealed class ViewerInputEventTests
{
    [Fact]
    public void WorkerAcceptsTheNormalizedInputContract()
    {
        Assert.True(ViewerInputEventValidator.IsValid(
            new ViewerInputEvent("pointerMove", 0, 1)));
        Assert.True(ViewerInputEventValidator.IsValid(
            new ViewerInputEvent("pointerDown", 0.5, 0.5, Button: 2)));
        Assert.True(ViewerInputEventValidator.IsValid(
            new ViewerInputEvent(
                "wheel",
                0.5,
                0.5,
                DeltaX: -10,
                DeltaY: 20)));
        Assert.True(ViewerInputEventValidator.IsValid(
            new ViewerInputEvent("keyDown", Code: "KeyW")));
        Assert.True(ViewerInputEventValidator.IsReleaseAll(
            new ViewerInputEvent("releaseAll")));
        Assert.Equal(
            "{\"type\":\"releaseAll\"}",
            ExternalViewerPublisher.SerializeInput(new ViewerInputEvent("releaseAll")));
    }

    [Fact]
    public void WorkerRejectsExpandedOrOutOfRangeInput()
    {
        Assert.False(ViewerInputEventValidator.IsValid(null));
        Assert.False(ViewerInputEventValidator.IsValid(
            new ViewerInputEvent("pointerMove", -0.01, 0)));
        Assert.False(ViewerInputEventValidator.IsValid(
            new ViewerInputEvent("pointerUp", 0, 0, Button: 4)));
        Assert.False(ViewerInputEventValidator.IsValid(
            new ViewerInputEvent(
                "wheel",
                0,
                0,
                DeltaX: 0,
                DeltaY: ViewerInputEventValidator.MaximumWheelDelta + 1)));
        Assert.False(ViewerInputEventValidator.IsValid(
            new ViewerInputEvent("keyDown", Code: "arbitrary-command")));
        Assert.False(ViewerInputEventValidator.IsValid(
            new ViewerInputEvent("keyDown", X: 0, Code: "KeyW")));
        Assert.False(ViewerInputEventValidator.IsValid(
            new ViewerInputEvent("releaseAll", X: 0)));

        var expanded = JsonSerializer.Deserialize<ViewerInputEvent>(
            "{\"type\":\"releaseAll\",\"unexpected\":true}");
        Assert.False(ViewerInputEventValidator.IsValid(expanded));
    }

    [Fact]
    public void ReleaseGraceTargetsOnlyTheExactActiveLease()
    {
        var sessionId = Guid.NewGuid();
        var leaseId = Guid.NewGuid();
        var expiresAt = DateTimeOffset.Parse("2026-07-19T12:00:00Z");
        var release = new ViewerInputEnvelope(
            sessionId,
            leaseId,
            new ViewerInputEvent("releaseAll"));
        var ordinaryInput = release with
        {
            Input = new ViewerInputEvent("keyUp", Code: "KeyW")
        };

        Assert.True(ExternalViewerPublisher.CanTargetActiveLease(
            leaseId,
            expiresAt,
            release,
            expiresAt + ViewerInputEventValidator.ReleaseGracePeriod));
        Assert.False(ExternalViewerPublisher.CanTargetActiveLease(
            leaseId,
            expiresAt,
            ordinaryInput,
            expiresAt));
        Assert.False(ExternalViewerPublisher.CanTargetActiveLease(
            Guid.NewGuid(),
            expiresAt,
            release,
            expiresAt));
        Assert.False(ExternalViewerPublisher.CanTargetActiveLease(
            leaseId,
            expiresAt,
            release,
            expiresAt + ViewerInputEventValidator.ReleaseGracePeriod
                + TimeSpan.FromTicks(1)));
        Assert.True(ExternalViewerPublisher.CanTargetActiveLease(
            leaseId,
            expiresAt,
            release,
            expiresAt + TimeSpan.FromHours(1),
            forceRelease: true));
        Assert.False(ExternalViewerPublisher.CanTargetActiveLease(
            Guid.NewGuid(),
            expiresAt,
            release,
            expiresAt + TimeSpan.FromHours(1),
            forceRelease: true));
        Assert.False(ExternalViewerPublisher.CanTargetActiveLease(
            leaseId,
            expiresAt,
            ordinaryInput,
            expiresAt + TimeSpan.FromHours(1),
            forceRelease: true));
    }
}
