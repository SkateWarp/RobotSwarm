using SwarmWorker.Runtime;

namespace SwarmWorker.Tests;

public sealed class FleetRosterParserTests
{
    [Fact]
    public void ParsesAndNumericallySortsRoster()
    {
        var roster = FleetRosterParser.Parse(
            "data: \"tb3_10,tb3_2,tb3_1\"\n---\n");

        Assert.Equal(new[] { "tb3_1", "tb3_2", "tb3_10" }, roster);
    }

    [Theory]
    [InlineData("data: ''\n---\n")]
    [InlineData("data: \"\"\n---\n")]
    public void ParsesEmptyRoster(string output)
    {
        Assert.Empty(FleetRosterParser.Parse(output));
    }

    [Fact]
    public void RejectsUnexpectedRobotIds()
    {
        Assert.Throws<FormatException>(
            () => FleetRosterParser.Parse("data: \"tb3_0;rm -rf /\"\n---\n"));
    }
}
