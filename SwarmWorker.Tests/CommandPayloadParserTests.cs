using System.Text.Json;
using SwarmWorker.Configuration;
using SwarmWorker.Runtime;

namespace SwarmWorker.Tests;

public sealed class CommandPayloadParserTests
{
    [Fact]
    public void GeneratesStableRobotIdsWhenPayloadOmitsThem()
    {
        var payload = JsonSerializer.SerializeToElement(new
        {
            desiredRobotCount = 3,
            arenaVersion = "arena-v1"
        });

        var parsed = CommandPayloadParser.ParseFleet(payload, ValidOptions(), requireArena: true);

        Assert.Equal(3, parsed.DesiredRobotCount);
        Assert.Equal(new[] { "tb3_0", "tb3_1", "tb3_2" }, parsed.RobotIds);
    }

    [Fact]
    public void RejectsRobotCountAboveWorkerLimit()
    {
        var payload = JsonSerializer.SerializeToElement(new { desiredRobotCount = 11 });

        Assert.Throws<InvalidOperationException>(
            () => CommandPayloadParser.ParseFleet(payload, ValidOptions(), requireArena: false));
    }

    [Fact]
    public void RejectsArbitraryRobotIds()
    {
        var payload = JsonSerializer.SerializeToElement(new
        {
            desiredRobotCount = 1,
            robotIds = new[] { "not-a-turtlebot" }
        });

        Assert.Throws<InvalidOperationException>(
            () => CommandPayloadParser.ParseFleet(payload, ValidOptions(), requireArena: false));
    }

    private static WorkerOptions ValidOptions() =>
        new()
        {
            BackendUrl = "https://backend.example.test",
            WorkerId = Guid.NewGuid(),
            WorkerSecret = new string('a', 32),
            Name = "test-worker",
            SessionImage = "robotswarm/ros-noetic:test",
            MaxConcurrentSessions = 4,
            MaxRobotsPerSession = 10
        };
}
