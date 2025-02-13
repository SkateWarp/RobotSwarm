using SwarmBackend.Entities;

namespace SwarmBackend.Models;

public record SensorReadingResponse(int Id, double Value, DateTime DateCreated, int SensorId, string? Notes)
{
    public static SensorReadingResponse From(SensorReading reading) => new(reading.Id, reading.Value, reading.DateCreated, reading.SensorId, reading.Notes);
}

public record SensorReadingRequest(double Value, int SensorId, string? Notes);

public record RosSensorReadingRequest(double Value, string SensorName, string Notes);