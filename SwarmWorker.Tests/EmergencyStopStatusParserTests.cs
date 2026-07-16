using System.Text.Json;
using SwarmWorker.Runtime;

namespace SwarmWorker.Tests;

public sealed class EmergencyStopStatusParserTests
{
    [Theory]
    [InlineData(true)]
    [InlineData(false)]
    public void ReadsConfirmedState(bool expected)
    {
        var json = JsonSerializer.Serialize(new
        {
            emergency_stop = expected,
            task = new { status = "idle" }
        });
        var output = $"data: {JsonSerializer.Serialize(json)}\n---\n";

        Assert.Equal(expected, EmergencyStopStatusParser.Parse(output));
    }

    [Fact]
    public void RejectsMissingState()
    {
        var json = JsonSerializer.Serialize(new { task = new { status = "idle" } });
        var output = $"data: {JsonSerializer.Serialize(json)}\n---\n";

        Assert.Throws<FormatException>(
            () => EmergencyStopStatusParser.Parse(output));
    }
}
