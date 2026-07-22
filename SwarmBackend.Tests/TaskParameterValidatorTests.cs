using System.Text.Json;
using SwarmBackend.Entities;
using SwarmBackend.Helpers;

namespace SwarmBackend.Tests;

public sealed class TaskParameterValidatorTests
{
    public static TheoryData<SwarmTaskRunType, string> FrontendPayloads => new()
    {
        {
            SwarmTaskRunType.FollowLeader,
            """
            {
              "leader_mode": "figure8",
              "config": {
                "leader_mode": "figure8",
                "follow_distance": 0.7,
                "radius": 2
              }
            }
            """
        },
        {
            SwarmTaskRunType.Figure,
            """
            {
              "formation_type": "A",
              "movement_mode": "static",
              "config": {
                "formation_type": "A",
                "movement_mode": "static",
                "spacing": 0.7
              }
            }
            """
        },
        {
            SwarmTaskRunType.CollaborativeTransport,
            """
            {
              "target_x": 3,
              "target_y": -1.5,
              "arrival_tolerance": 0.25,
              "config": {
                "target_x": 3,
                "target_y": -1.5,
                "arrival_tolerance": 0.25,
                "transport_planner": "grf"
              }
            }
            """
        }
    };

    [Theory]
    [MemberData(nameof(FrontendPayloads))]
    public void AcceptsTypedFrontendPayloads(SwarmTaskRunType taskType, string json)
    {
        var parameters = Parse(json);

        var valid = TaskParameterValidator.TryValidate(taskType, parameters, out var error);

        Assert.True(valid, error);
        Assert.Null(error);
    }

    [Theory]
    [InlineData("{ \"leader_mode\": \"teleport\" }")]
    [InlineData("{ \"leader_mode\": \"waypoint\" }")]
    [InlineData("{ \"leader_mode\": \"random\" }")]
    [InlineData("{ \"leader_mode\": \"manual\" }")]
    [InlineData("{ \"config\": { \"radius\": 4.1 } }")]
    [InlineData("{ \"config\": { \"follow_distance\": \"0.7\" } }")]
    public void RejectsMalformedFollowLeaderParameters(string json)
    {
        AssertInvalid(SwarmTaskRunType.FollowLeader, json);
    }

    [Theory]
    [InlineData("{ \"formation_type\": \"hexagon\" }")]
    [InlineData("{ \"movement_mode\": \"teleport\" }")]
    [InlineData("{ \"movement_mode\": \"moving\" }")]
    [InlineData("{ \"movement_mode\": \"adaptive\" }")]
    [InlineData("{ \"config\": { \"spacing\": 0.34 } }")]
    [InlineData("{ \"formation_type\": \"Q\" }")]
    public void RejectsUnsupportedFormationParameters(string json)
    {
        AssertInvalid(SwarmTaskRunType.Figure, json);
    }

    [Theory]
    [InlineData("{ \"target_x\": 4.01 }")]
    [InlineData("{ \"target_y\": false }")]
    [InlineData("{ \"arrival_tolerance\": 0.14 }")]
    [InlineData("{ \"arrival_tolerance\": 0.76 }")]
    [InlineData("{ \"arrival_tolerance\": \"0.25\" }")]
    [InlineData("{ \"config\": { \"transport_planner\": \"legacy\" } }")]
    [InlineData("{ \"config\": { \"transport_planner\": \"unknown\" } }")]
    public void RejectsUnsafeTransportParameters(string json)
    {
        AssertInvalid(SwarmTaskRunType.CollaborativeTransport, json);
    }

    [Fact]
    public void RejectsConflictingTopLevelAndNestedValues()
    {
        var parameters = Parse(
            """
            {
              "target_x": 2,
              "config": { "target_x": -2 }
            }
            """);

        var valid = TaskParameterValidator.TryValidate(
            SwarmTaskRunType.CollaborativeTransport,
            parameters,
            out var error);

        Assert.False(valid);
        Assert.Equal("target_x and config.target_x must match.", error);
    }

    [Fact]
    public void RejectsNonObjectConfigBeforeItReachesTheWorker()
    {
        var parameters = Parse("{ \"config\": [] }");

        var valid = TaskParameterValidator.TryValidate(
            SwarmTaskRunType.FollowLeader,
            parameters,
            out var error);

        Assert.False(valid);
        Assert.Equal("config must be a JSON object.", error);
    }

    [Fact]
    public void AllowsExtraPlannerTuningWithoutWeakeningTheTypedFields()
    {
        var parameters = Parse(
            """
            {
              "target_x": 1,
              "target_y": 2,
              "config": {
                "transport_planner": "grf",
                "experimental_gain": 0.25
              }
            }
            """);

        var valid = TaskParameterValidator.TryValidate(
            SwarmTaskRunType.CollaborativeTransport,
            parameters,
            out var error);

        Assert.True(valid, error);
    }

    private static void AssertInvalid(SwarmTaskRunType taskType, string json)
    {
        var valid = TaskParameterValidator.TryValidate(taskType, Parse(json), out var error);

        Assert.False(valid);
        Assert.False(string.IsNullOrWhiteSpace(error));
    }

    private static JsonElement Parse(string json)
    {
        using var document = JsonDocument.Parse(json);
        return document.RootElement.Clone();
    }
}
