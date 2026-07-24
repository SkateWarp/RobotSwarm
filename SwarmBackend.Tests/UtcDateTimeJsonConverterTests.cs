using System.Text.Json;
using SwarmBackend.Helpers;

namespace SwarmBackend.Tests;

public sealed class UtcDateTimeJsonConverterTests
{
    private static readonly JsonSerializerOptions Options = new()
    {
        Converters = { new UtcDateTimeJsonConverter() }
    };

    [Theory]
    [InlineData(DateTimeKind.Utc)]
    [InlineData(DateTimeKind.Unspecified)]
    public void WritesDatabaseDatesWithAnExplicitUtcZone(DateTimeKind kind)
    {
        var value = new DateTime(2026, 7, 24, 14, 30, 15, 123, kind);

        var json = JsonSerializer.Serialize(value, Options);

        Assert.Equal("\"2026-07-24T14:30:15.123Z\"", json);
    }

    [Fact]
    public void ReadsAZoneLessDatabaseDateAsUtc()
    {
        var value = JsonSerializer.Deserialize<DateTime>(
            "\"2026-07-24T14:30:15.123\"",
            Options);

        Assert.Equal(DateTimeKind.Utc, value.Kind);
        Assert.Equal(new DateTime(2026, 7, 24, 14, 30, 15, 123, DateTimeKind.Utc), value);
    }
}
