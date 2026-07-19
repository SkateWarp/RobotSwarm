using SwarmBackend.Entities;
using SwarmBackend.Helpers;

namespace SwarmBackend.Tests;

public sealed class ViewerStreamAddressTests
{
    [Fact]
    public void SceneSourceAndPathAreUniqueToTheSession()
    {
        var firstSession = Guid.NewGuid();
        var secondSession = Guid.NewGuid();

        Assert.True(ViewerStreamAddress.TryCreate(
            firstSession,
            ViewerSourceType.Scene,
            robotRuntimeId: null,
            out var first));
        Assert.True(ViewerStreamAddress.TryCreate(
            secondSession,
            ViewerSourceType.Scene,
            robotRuntimeId: null,
            out var second));

        Assert.NotEqual(first.SourceId, second.SourceId);
        Assert.NotEqual(first.StreamPath, second.StreamPath);
        Assert.Contains(firstSession.ToString("N"), first.SourceId);
        Assert.Contains(firstSession.ToString("N"), first.StreamPath);
    }

    [Fact]
    public void RobotSourceRoundTripsThroughItsCanonicalPath()
    {
        var sessionId = Guid.NewGuid();
        Assert.True(ViewerStreamAddress.TryCreate(
            sessionId,
            ViewerSourceType.RobotCamera,
            "tb3_7",
            out var created));

        Assert.True(ViewerStreamAddress.TryParse(
            "/" + created.StreamPath + "/",
            out var parsed));
        Assert.Equal(created, parsed);
    }

    [Fact]
    public void RejectsSourceIdCopiedFromAnotherSession()
    {
        var firstSession = Guid.NewGuid();
        var secondSession = Guid.NewGuid();
        Assert.True(ViewerStreamAddress.TryCreate(
            firstSession,
            ViewerSourceType.Scene,
            robotRuntimeId: null,
            out var first));
        var mixedPath = $"session/{secondSession:N}/{first.SourceId}";

        Assert.False(ViewerStreamAddress.TryParse(mixedPath, out _));
    }

    [Fact]
    public void RejectsTrailingPathSegmentsAndUnsafeRobotIds()
    {
        var sessionId = Guid.Parse("aaaaaaaa-bbbb-cccc-dddd-eeeeeeeeeeee");
        Assert.True(ViewerStreamAddress.TryCreate(
            sessionId,
            ViewerSourceType.Scene,
            robotRuntimeId: null,
            out var scene));

        Assert.False(ViewerStreamAddress.TryParse(
            scene.StreamPath + "/another-source",
            out _));
        Assert.False(ViewerStreamAddress.TryParse(
            scene.StreamPath.Replace("session/", "session//"),
            out _));
        Assert.False(ViewerStreamAddress.TryParse(
            scene.StreamPath.Replace(
                sessionId.ToString("N"),
                sessionId.ToString("N").ToUpperInvariant(),
                StringComparison.Ordinal),
            out _));
        Assert.False(ViewerStreamAddress.TryCreate(
            sessionId,
            ViewerSourceType.RobotCamera,
            "tb3/other",
            out _));
    }

    [Theory]
    [InlineData("https://stream.example.test")]
    [InlineData("https://stream.example.test/")]
    public void BuildsWhepUrlFromTheCanonicalPath(string publicBaseUrl)
    {
        var sessionId = Guid.NewGuid();
        Assert.True(ViewerStreamAddress.TryCreate(
            sessionId,
            ViewerSourceType.Scene,
            robotRuntimeId: null,
            out var address));

        var url = ViewerStreamAddress.BuildWhepUrl(publicBaseUrl, address);

        Assert.Equal(
            $"https://stream.example.test/{address.StreamPath}/whep",
            url);
    }
}
