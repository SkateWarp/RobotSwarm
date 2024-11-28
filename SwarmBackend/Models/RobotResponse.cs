using SwarmBackend.Entities;
using SwarmBackend.Helpers;

namespace SwarmBackend.Models;

/// <summary>
/// Represents the response model for a Robot entity
/// </summary>
public record RobotResponse(
    int Id,
    string Name,
    string? Description,
    string? Notes,
    DateTime DateCreated,
    IEnumerable<SensorResponse> Sensors,
    RobotStatus Status,
    string StatusDescription,
    bool IsConnected)
{
    /// <summary>
    /// Creates a RobotResponse from a Robot entity
    /// </summary>
    /// <param name="robot">The source Robot entity</param>
    /// <returns>A RobotResponse object</returns>
    public static RobotResponse From(Robot robot) => new(
        robot.Id,
        robot.Name,
        robot.Description,
        robot.Notes,
        robot.DateCreated,
        robot.Sensors?.Select(x => SensorResponse.From(x, false)) ?? new List<SensorResponse>(),
        robot.Status,
        robot.Status.GetDescriptionAttribute(),
        robot.IsConnected);
}

/// <summary>
/// Represents the request model for creating or updating a Robot
/// </summary>
public record RobotRequest(
    string Name,
    string? Description,
    string? Notes);