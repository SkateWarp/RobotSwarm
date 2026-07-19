using System.Text.Json;
using SwarmWorker.Runtime;

namespace SwarmWorker.Tests;

public sealed class ViewerSourceCommandParserTests
{
    private const string PublishToken =
        "AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA";
    private static readonly DateTimeOffset Now =
        new(2026, 7, 17, 12, 0, 0, TimeSpan.Zero);

    [Fact]
    public void ParsesCanonicalSceneSource()
    {
        var sessionId = Guid.NewGuid();
        var sourceId = $"scene-{sessionId:N}";
        var payload = JsonSerializer.SerializeToElement(new
        {
            leaseId = Guid.NewGuid(),
            expiresAt = Now.AddMinutes(5),
            publishToken = PublishToken,
            source = "Scene",
            robotRuntimeId = (string?)null,
            sourceId,
            streamPath = $"session/{sessionId:N}/{sourceId}"
        });

        var command = Parse(sessionId, payload);

        Assert.Equal(ViewerSourceKind.Scene, command.Source);
        Assert.Equal(PublishToken, command.PublishToken);
        Assert.Null(command.RobotRuntimeId);
        Assert.Equal(sourceId, command.SourceId);
    }

    [Fact]
    public void ParsesCanonicalRobotSource()
    {
        var sessionId = Guid.NewGuid();
        const string robotRuntimeId = "tb3_12";
        var sourceId = $"robot-{sessionId:N}-{robotRuntimeId}";
        var payload = JsonSerializer.SerializeToElement(new
        {
            leaseId = Guid.NewGuid(),
            expiresAt = Now.AddMinutes(5),
            publishToken = PublishToken,
            source = "RobotCamera",
            robotRuntimeId,
            sourceId,
            streamPath = $"session/{sessionId:N}/{sourceId}"
        });

        var command = Parse(sessionId, payload);

        Assert.Equal(ViewerSourceKind.RobotCamera, command.Source);
        Assert.Equal(robotRuntimeId, command.RobotRuntimeId);
    }

    [Fact]
    public void RejectsSourceIdentifiersFromAnotherSession()
    {
        var assignedSession = Guid.NewGuid();
        var otherSession = Guid.NewGuid();
        var sourceId = $"scene-{otherSession:N}";
        var payload = JsonSerializer.SerializeToElement(new
        {
            leaseId = Guid.NewGuid(),
            expiresAt = Now.AddMinutes(5),
            publishToken = PublishToken,
            source = "Scene",
            sourceId,
            streamPath = $"session/{otherSession:N}/{sourceId}"
        });

        Assert.Throws<InvalidOperationException>(
            () => Parse(assignedSession, payload));
    }

    [Theory]
    [InlineData("tb3/0")]
    [InlineData("tb3_01")]
    [InlineData("robot_0")]
    public void RejectsNonCanonicalRobotRuntimeIds(string robotRuntimeId)
    {
        var sessionId = Guid.NewGuid();
        var sourceId = $"robot-{sessionId:N}-{robotRuntimeId}";
        var payload = JsonSerializer.SerializeToElement(new
        {
            leaseId = Guid.NewGuid(),
            expiresAt = Now.AddMinutes(5),
            publishToken = PublishToken,
            source = "RobotCamera",
            robotRuntimeId,
            sourceId,
            streamPath = $"session/{sessionId:N}/{sourceId}"
        });

        Assert.Throws<InvalidOperationException>(
            () => Parse(sessionId, payload));
    }

    [Fact]
    public void RejectsExpiredOrOverlongLeases()
    {
        var sessionId = Guid.NewGuid();
        var sourceId = $"scene-{sessionId:N}";

        JsonElement Payload(DateTimeOffset expiresAt) =>
            JsonSerializer.SerializeToElement(new
            {
                leaseId = Guid.NewGuid(),
                expiresAt,
                publishToken = PublishToken,
                source = "Scene",
                sourceId,
                streamPath = $"session/{sessionId:N}/{sourceId}"
            });

        Assert.Throws<InvalidOperationException>(
            () => Parse(sessionId, Payload(Now)));
        Assert.Throws<InvalidOperationException>(
            () => Parse(sessionId, Payload(Now.AddMinutes(31))));
    }

    [Fact]
    public void RejectsMissingOrMalformedPublishTokens()
    {
        var sessionId = Guid.NewGuid();
        var sourceId = $"scene-{sessionId:N}";

        JsonElement Payload(string? publishToken, bool includeToken = true)
        {
            var values = new Dictionary<string, object?>
            {
                ["leaseId"] = Guid.NewGuid(),
                ["expiresAt"] = Now.AddMinutes(5),
                ["source"] = "Scene",
                ["sourceId"] = sourceId,
                ["streamPath"] = $"session/{sessionId:N}/{sourceId}"
            };
            if (includeToken)
            {
                values["publishToken"] = publishToken;
            }

            return JsonSerializer.SerializeToElement(values);
        }

        Assert.Throws<InvalidOperationException>(
            () => Parse(sessionId, Payload(null, includeToken: false)));
        Assert.Throws<InvalidOperationException>(
            () => Parse(sessionId, Payload(new string('A', 42))));
        Assert.Throws<InvalidOperationException>(
            () => Parse(sessionId, Payload(new string('B', 43))));
        Assert.Throws<InvalidOperationException>(
            () => Parse(sessionId, Payload(new string('A', 42) + ".")));
    }

    private static ViewerSourceCommand Parse(Guid sessionId, JsonElement payload)
    {
        return ViewerSourceCommandParser.Parse(
            sessionId,
            payload,
            Now,
            TimeSpan.FromMinutes(30));
    }
}
