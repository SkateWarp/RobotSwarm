using SwarmBackend.Entities;

namespace SwarmBackend.Models;

public record SensorReadingResponse(int Id, string Value, DateTime DateCreated, int SensorId, string? Notes)
{
    public static SensorReadingResponse From(SensorReading reading) => new(reading.Id, reading.Value, reading.DateCreated, reading.SensorId, reading.Notes);
}

public record SensorReadingRequest(string Value, int SensorId, string? Notes);

public record RosSensorReadingRequest(string Value, string SensorName, string Notes);

// Batch request models for efficient sensor data submission
public record RosBatchSensorReadingRequest(string SensorName, List<SensorFieldValue> Fields);
public record SensorFieldValue(string FieldName, string Value);

// Frontend sensor reading request (with dictionary of fields)
public record ClientSensorReadingRequest(string SensorName, Dictionary<string, object> SensorFields);