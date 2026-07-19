using System.Text.Json;
using SwarmBackend.Helpers;

namespace SwarmBackend.Tests;

public sealed class ViewerPublishCompletionValidatorTests
{
    [Fact]
    public void AcceptsReadyH264CompletionForTheIssuedLeaseAndPath()
    {
        var fixture = Fixture();

        var valid = ViewerPublishCompletionValidator.TryValidate(
            fixture.SessionId,
            fixture.Payload,
            fixture.Result,
            out var error);

        Assert.True(valid, error);
    }

    [Fact]
    public void RejectsCompletionBeforePublisherIsReady()
    {
        var fixture = Fixture(ready: false);

        var valid = ViewerPublishCompletionValidator.TryValidate(
            fixture.SessionId,
            fixture.Payload,
            fixture.Result,
            out _);

        Assert.False(valid);
    }

    [Fact]
    public void RejectsCompletionForAnotherSessionPath()
    {
        var fixture = Fixture(resultStreamPath: "session/other/scene-other");

        var valid = ViewerPublishCompletionValidator.TryValidate(
            fixture.SessionId,
            fixture.Payload,
            fixture.Result,
            out _);

        Assert.False(valid);
    }

    [Fact]
    public void RejectsCompletionThatEchoesThePublishToken()
    {
        var fixture = Fixture();
        var result = JsonSerializer.SerializeToElement(new
        {
            sessionId = fixture.SessionId,
            leaseId = fixture.Payload.GetProperty("leaseId").GetGuid(),
            expiresAt = fixture.Payload.GetProperty("expiresAt").GetDateTimeOffset(),
            source = fixture.Payload.GetProperty("source").GetString(),
            robotRuntimeId = (string?)null,
            sourceId = fixture.Payload.GetProperty("sourceId").GetString(),
            streamPath = fixture.Payload.GetProperty("streamPath").GetString(),
            ready = true,
            videoCodec = "H264",
            publishToken = fixture.Payload.GetProperty("publishToken").GetString()
        });

        var valid = ViewerPublishCompletionValidator.TryValidate(
            fixture.SessionId,
            fixture.Payload,
            result,
            out _);

        Assert.False(valid);
    }

    private static CompletionFixture Fixture(
        bool ready = true,
        string? resultStreamPath = null)
    {
        var sessionId = Guid.NewGuid();
        var leaseId = Guid.NewGuid();
        var expiresAt = DateTimeOffset.UtcNow.AddMinutes(5);
        var sourceId = $"scene-{sessionId:N}";
        var streamPath = $"session/{sessionId:N}/{sourceId}";
        var payload = JsonSerializer.SerializeToElement(new
        {
            leaseId,
            expiresAt,
            publishToken = "AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA",
            source = "Scene",
            robotRuntimeId = (string?)null,
            sourceId,
            streamPath
        });
        var result = JsonSerializer.SerializeToElement(new
        {
            sessionId,
            leaseId,
            expiresAt,
            source = "Scene",
            robotRuntimeId = (string?)null,
            sourceId,
            streamPath = resultStreamPath ?? streamPath,
            ready,
            videoCodec = "H264"
        });
        return new CompletionFixture(sessionId, payload, result);
    }

    private sealed record CompletionFixture(
        Guid SessionId,
        JsonElement Payload,
        JsonElement Result);
}
