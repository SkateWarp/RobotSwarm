using SwarmBackend.Entities;
using SwarmBackend.Helpers;

namespace SwarmBackend.Models;

public record SensorResponse(int Id, DateTime DateCreated, int RobotId, RobotResponse? Robot, SensorTypeEnum SensorType, string SensorTypeDescription, string? Description, string? Notes)
{
    public static SensorResponse From(Sensor sensor, bool includeRobot = false) => new(sensor.Id, sensor.DateCreated, sensor.RobotId, sensor.Robot != null && includeRobot ? RobotResponse.From(sensor.Robot) : null,
        sensor.SensorType, sensor.SensorType.GetDescriptionAttribute(), sensor.Description, sensor.Notes);
}


public record SensorRequest(int RobotId, SensorTypeEnum SensorType, string? Description, string? Notes);