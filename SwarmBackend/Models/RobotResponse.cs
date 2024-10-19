using SwarmBackend.Entities;

namespace SwarmBackend.Models;

public record RobotResponse(int Id, string Name, string? Description, string? Notes, DateTime DateCreated, IEnumerable<SensorResponse> Sensors, RobotStatus Status)
{
    public static RobotResponse From(Entities.Robot robot) => new(robot.Id, robot.Name, robot.Description, robot.Notes, robot.DateCreated,
        robot.Sensors != null ? robot.Sensors.Select(x => SensorResponse.From(x, false)) : new List<SensorResponse>(), robot.Status);
}

public record RobotRequest(string Name, string? Description, string? Notes);


