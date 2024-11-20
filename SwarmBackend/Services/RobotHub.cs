using System.Text.Json;
using Microsoft.AspNetCore.SignalR;
using SwarmBackend.Entities;
using SwarmBackend.Helpers;
using SwarmBackend.Interfaces;
using SwarmBackend.Models;

namespace SwarmBackend.Services;

public class RobotHub(ILogger<RobotHub> logger, DataContext context, ISensorReadingService sensorReadingService, ITaskLogService taskLogService)
    : Hub, IRealtimeService
{
    private static readonly Dictionary<int, string> RobotConnections = [];


    public override async Task OnConnectedAsync()
    {
        await base.OnConnectedAsync();

        try
        {
            // Get robotId from query string
            if (Context.GetHttpContext()?.Request.Query.TryGetValue("robotId", out var robotIdStr) == true
                && int.TryParse(robotIdStr, out var robotId))
            {
                RobotConnections[robotId] = Context.ConnectionId;
                await UpdateRobotConnection(robotId, true);
                await Groups.AddToGroupAsync(Context.ConnectionId, $"robot_{robotId}");
                logger.LogInformation("Robot {RobotId} connected with connection ID: {ConnectionId}", robotId,
                    Context.ConnectionId);
            }
        }
        catch (Exception ex)
        {
            logger.LogError(ex, "Error in OnConnectedAsync");
        }
    }

    public override async Task OnDisconnectedAsync(Exception? exception)
    {
        try
        {
            var robotId = RobotConnections.FirstOrDefault(x => x.Value == Context.ConnectionId).Key;
            if (robotId != 0)
            {
                RobotConnections.Remove(robotId);
                await UpdateRobotConnection(robotId, false);
                logger.LogInformation("Robot {RobotId} disconnected with connection ID: {Context.ConnectionId}",
                    robotId,
                    Context.ConnectionId);
            }
        }
        catch (Exception ex)
        {
            logger.LogError(ex, "Error in OnDisconnectedAsync");
        }

        await base.OnDisconnectedAsync(exception);
    }

    public async Task UpdateRobotConnection(int robotId, bool isConnected)
    {
        var robot = await context.Robots.FindAsync(robotId);
        if (robot != null)
        {
            robot.IsConnected = isConnected;
            if (!isConnected)
            {
                robot.Status = RobotStatus.Idle;
            }

            await context.SaveChangesAsync();

            // Notify all clients about the connection status change
            await Clients.All.SendAsync("RobotConnectionChanged", new
            {
                robotId,
                isConnected,
                status = robot.Status
            });
        }
    }

    public async Task UpdateStatus(int robotId, string status)
    {
        try
        {
            var robot = await context.Robots.FindAsync(robotId);
            if (robot != null)
            {
                robot.Status = Enum.Parse<RobotStatus>(status);
                await context.SaveChangesAsync();

                await Clients.All.SendAsync("RobotStatusChanged", new
                {
                    robotId,
                    status = robot.Status,
                    timestamp = DateTime.UtcNow
                });
            }
        }
        catch (Exception ex)
        {
            logger.LogError(ex, "Error updating status for robot {robotId}", robotId);
            throw;
        }
    }

    public async Task HandleSensorReading(int robotId, RosSensorReadingRequest reading)
    {
        try
        {
            await sensorReadingService.Create(robotId, reading);

            // Broadcast the new reading to interested clients
            await Clients.All.SendAsync("NewSensorReading", reading);
        }
        catch (Exception ex)
        {
            logger.LogError(ex,
                "Error processing sensor reading for robot {RobotId}, sensor {SensorName}: {Message}",
                robotId, reading?.SensorName, ex.Message);

            // Log inner exception if present
            if (ex.InnerException != null)
            {
                logger.LogError(ex.InnerException,
                    "Inner exception details: {Message}",
                    ex.InnerException.Message);
            }

            throw; // Rethrow to ensure the client gets the error
        }
    }


    public async Task SendCommand(int robotId, string command, JsonDocument parameters)
    {
        if (RobotConnections.TryGetValue(robotId, out var connectionId))
        {
            await Clients.Client(connectionId).SendAsync("ExecuteCommand", new
            {
                command,
                parameters,
                timestamp = DateTime.UtcNow
            });
        }
    }

    public async Task HandleTaskLog(int robotId, RosTaskTemplateRequest request)
    {
        try
        {
            await taskLogService.Create(robotId, request);
            await Clients.All.SendAsync("NewTaskLog", request);
            await UpdateRobotConnection(robotId, true);

        }
        catch (System.Exception ex)
        {

            logger.LogError(ex,
                            "Error processing task log for robot {RobotId}, task {TaskType}: {Message}",
                            robotId, request?.TaskType, ex.Message);
            throw;
        }


    }
}