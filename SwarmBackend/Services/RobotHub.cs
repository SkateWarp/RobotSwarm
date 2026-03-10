using System.Text.Json;
using Microsoft.AspNetCore.SignalR;
using Microsoft.EntityFrameworkCore;
using SwarmBackend.Entities;
using SwarmBackend.Helpers;
using SwarmBackend.Interfaces;
using SwarmBackend.Models;

namespace SwarmBackend.Services;

public class RobotHub(ILogger<RobotHub> logger, DataContext context, ISensorReadingService sensorReadingService, ITaskLogService taskLogService)
    : Hub, IRealtimeService
{
    private static readonly Dictionary<int, string> RobotConnections = [];

    /// <summary>Tracks whether the ROS bridge client is connected</summary>
    private static bool _rosBridgeConnected;

    /// <summary>Connection ID of the ROS bridge client (identified by first ForwardSwarmStatus call)</summary>
    private static string? _bridgeConnectionId;


    public override async Task OnConnectedAsync()
    {
        await base.OnConnectedAsync();

        try
        {
            await NotifyRobotsAvailable();
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
            // Check if the disconnecting client is the ROS bridge
            if (_bridgeConnectionId == Context.ConnectionId)
            {
                _rosBridgeConnected = false;
                _bridgeConnectionId = null;
                logger.LogWarning("ROS bridge disconnected");
                await Clients.All.SendAsync("RosConnectionChanged", new { connected = false, timestamp = DateTime.UtcNow });
            }

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
            await Clients.All.SendAsync($"RobotConnectionChanged/{robotId}", new
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

                await Clients.All.SendAsync($"RobotStatusChanged/{robotId}", new
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
            var response = await sensorReadingService.Create(robotId, reading);
            var lastReadings = sensorReadingService.GetLastByRobot(robotId);

            // Broadcast the new reading to interested clients
            await Clients.All.SendAsync("NewSensorReading", response);

            await Clients.All.SendAsync($"AllSensorReadings/{robotId}", lastReadings);
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

    /// <summary>
    /// Handle sensor reading from frontend client
    /// </summary>
    public async Task HandleSensorReadingFromClient(int robotId, string sensorName, Dictionary<string, object> sensorFields)
    {
        try
        {
            var request = new ClientSensorReadingRequest(sensorName, sensorFields);
            var responses = await sensorReadingService.CreateFromClient(robotId, request);
            var lastReadings = sensorReadingService.GetLastByRobot(robotId);

            // Broadcast each new reading
            foreach (var response in responses)
            {
                await Clients.All.SendAsync("NewSensorReading", response);
            }

            await Clients.All.SendAsync($"AllSensorReadings/{robotId}", lastReadings);

            logger.LogInformation("Processed {Count} sensor readings from client for robot {RobotId}",
                responses.Count(), robotId);
        }
        catch (Exception ex)
        {
            logger.LogError(ex,
                "Error processing client sensor reading for robot {RobotId}, sensor {SensorName}: {Message}",
                robotId, sensorName, ex.Message);
            throw;
        }
    }

    /// <summary>
    /// Handle batch sensor readings from ROS Bridge (more efficient - single call for all fields)
    /// </summary>
    public async Task HandleSensorReadingsBatch(int robotId, RosBatchSensorReadingRequest request)
    {
        try
        {
            var responses = await sensorReadingService.CreateBatch(robotId, request);
            var lastReadings = sensorReadingService.GetLastByRobot(robotId);

            // Broadcast each new reading
            foreach (var response in responses)
            {
                await Clients.All.SendAsync("NewSensorReading", response);
            }

            await Clients.All.SendAsync($"AllSensorReadings/{robotId}", lastReadings);

            logger.LogInformation("Processed batch of {Count} sensor readings for robot {RobotId}",
                responses.Count(), robotId);
        }
        catch (Exception ex)
        {
            logger.LogError(ex,
                "Error processing batch sensor reading for robot {RobotId}, sensor {SensorName}: {Message}",
                robotId, request?.SensorName, ex.Message);
            throw;
        }
    }


    public async Task SendCommand(int robotId, string command, string parameters)
    {

        // Send command to all clients in the robot's group
        await Clients.All.SendAsync("ExecuteCommand", new
        {
            command,
            parameters,
            timestamp = DateTime.UtcNow
        });

        logger.LogError("Robot {RobotId}  connected, for the command {Command}", robotId, command);

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

    public async Task HandleFinishTaskLog(int robotId)
    {
        try
        {
            await taskLogService.FinishTask(robotId);
        }
        catch (System.Exception ex)
        {
            logger.LogError(ex,
                            "Error processing finish task log for robot {RobotId}: {Message}",
                            robotId, ex.Message);
            throw;
        }
    }

    public async Task HandleCancelTaskLog(int robotId, int accountId)
    {
        try
        {
            await taskLogService.CancelTasksByRobot(robotId, accountId);
        }
        catch (System.Exception ex)
        {
            logger.LogError(ex,
                "Error processing cancel task log for robot {RobotId}: {Message}",
                robotId, ex.Message);
            throw;
        }
    }

    public async Task<List<int>> NotifyRobotsAvailable()
    {
        var robotIds = await context.Robots
            .Where(r => r.Status != RobotStatus.Disabled)
            .Select(r => r.Id)
            .ToListAsync();

        await Clients.All.SendAsync("RobotsAvailable", robotIds);
        logger.LogInformation("Available robots: {RobotIds}", string.Join(", ", robotIds));

        return robotIds;
    }

    // ==================== Swarm Control Methods ====================
    // These methods are invoked by the frontend and forward commands to the ROS bridge via SwarmCommand event.

    public async Task<bool> SpawnRobots(int count, string pattern)
    {
        logger.LogInformation("SpawnRobots: count={Count}, pattern={Pattern}", count, pattern);
        var command = new { command = "spawn_robots", parameters = new { robot_count = count, spawn_pattern = pattern } };
        await Clients.All.SendAsync("SwarmCommand", JsonSerializer.Serialize(command));
        return true;
    }

    public async Task<bool> SpawnRobotsWithIds(List<RobotDeploymentInfo> robots, string pattern)
    {
        logger.LogInformation("SpawnRobotsWithIds: {Count} robots, pattern={Pattern}", robots.Count, pattern);

        // Assign namespaces to database robots
        for (var i = 0; i < robots.Count; i++)
        {
            var ns = $"tb3_{i}";
            if (int.TryParse(robots[i].Id, out var dbId))
            {
                var robot = await context.Robots.FindAsync(dbId);
                if (robot != null)
                {
                    robot.Namespace = ns;
                    robot.IsConnected = true;
                    robot.Status = RobotStatus.Working;
                }
            }
        }
        await context.SaveChangesAsync();

        var command = new { command = "spawn_robots", parameters = new { robot_count = robots.Count, spawn_pattern = pattern, robot_ids = robots } };
        await Clients.All.SendAsync("SwarmCommand", JsonSerializer.Serialize(command));

        // Notify frontend of deployment
        var deploymentInfo = robots.Select((r, i) => new { id = r.Id, name = r.Name, @namespace = $"tb3_{i}" });
        await Clients.All.SendAsync("RobotDeploymentResponse", JsonSerializer.Serialize(deploymentInfo));

        return true;
    }

    public async Task<bool> DeleteRobots(List<string> robotIds)
    {
        logger.LogInformation("DeleteRobots: {RobotIds}", string.Join(", ", robotIds));

        // Clear namespaces for deleted robots
        var robots = await context.Robots.Where(r => r.Namespace != null).ToListAsync();
        foreach (var robot in robots)
        {
            if (!robotIds.Any() || robotIds.Contains(robot.Namespace!))
            {
                robot.Namespace = null;
                robot.IsConnected = false;
                robot.Status = RobotStatus.Idle;
            }
        }
        await context.SaveChangesAsync();

        var command = new { command = "delete_robots", parameters = new { robot_ids = robotIds } };
        await Clients.All.SendAsync("SwarmCommand", JsonSerializer.Serialize(command));
        return true;
    }

    public async Task<bool> StartFollowLeader(int robotCount, string leaderMode, object config)
    {
        logger.LogInformation("StartFollowLeader: {RobotCount} robots, mode={Mode}", robotCount, leaderMode);
        var command = new
        {
            command = "start_task",
            parameters = new { task_type = "follow_leader", robot_count = robotCount, leader_mode = leaderMode, config }
        };
        await Clients.All.SendAsync("SwarmCommand", JsonSerializer.Serialize(command));
        await Clients.All.SendAsync("TaskEvent", JsonSerializer.Serialize(new { type = "started", task = "follow_leader" }));
        return true;
    }

    public async Task<bool> StartFormation(int robotCount, string formationType, string movementMode, object config)
    {
        logger.LogInformation("StartFormation: {RobotCount} robots, type={Type}, mode={Mode}", robotCount, formationType, movementMode);
        var command = new
        {
            command = "start_task",
            parameters = new { task_type = "formation", robot_count = robotCount, formation_type = formationType, movement_mode = movementMode, config }
        };
        await Clients.All.SendAsync("SwarmCommand", JsonSerializer.Serialize(command));
        await Clients.All.SendAsync("TaskEvent", JsonSerializer.Serialize(new { type = "started", task = "formation" }));
        return true;
    }

    public async Task<bool> StartTransport(int robotCount, double targetX, double targetY, object config)
    {
        logger.LogInformation("StartTransport: {RobotCount} robots, target=({X},{Y})", robotCount, targetX, targetY);
        var command = new
        {
            command = "start_task",
            parameters = new { task_type = "transport", robot_count = robotCount, target_x = targetX, target_y = targetY, config }
        };
        await Clients.All.SendAsync("SwarmCommand", JsonSerializer.Serialize(command));
        await Clients.All.SendAsync("TaskEvent", JsonSerializer.Serialize(new { type = "started", task = "transport" }));
        return true;
    }

    public async Task<bool> StopTask()
    {
        logger.LogInformation("StopTask requested");
        var command = new { command = "stop_task", parameters = new { } };
        await Clients.All.SendAsync("SwarmCommand", JsonSerializer.Serialize(command));
        await Clients.All.SendAsync("TaskEvent", JsonSerializer.Serialize(new { type = "stopped" }));
        return true;
    }

    public async Task<bool> EmergencyStop()
    {
        logger.LogWarning("EMERGENCY STOP requested");
        var command = new { command = "emergency_stop", parameters = new { } };
        await Clients.All.SendAsync("SwarmCommand", JsonSerializer.Serialize(command));
        await Clients.All.SendAsync("EmergencyStop", JsonSerializer.Serialize(new { timestamp = DateTime.UtcNow }));
        return true;
    }

    public async Task<bool> ControlLeader(double linearVelocity, double angularVelocity)
    {
        var command = new { command = "control_leader", parameters = new { linear_velocity = linearVelocity, angular_velocity = angularVelocity } };
        await Clients.All.SendAsync("SwarmCommand", JsonSerializer.Serialize(command));
        return true;
    }

    public async Task<bool> SpawnObstacles(string density)
    {
        logger.LogInformation("SpawnObstacles: density={Density}", density);
        var command = new { command = "spawn_obstacles", parameters = new { density } };
        await Clients.All.SendAsync("SwarmCommand", JsonSerializer.Serialize(command));
        return true;
    }

    public Task<bool> GetRosConnectionStatus()
    {
        return Task.FromResult(_rosBridgeConnected);
    }

    // ==================== Bridge Forwarding Methods ====================
    // Called by the ROS bridge to push data to frontend clients.

    public async Task ForwardSwarmStatus(string statusJson)
    {
        // Detect bridge connection state transition
        if (!_rosBridgeConnected)
        {
            _rosBridgeConnected = true;
            _bridgeConnectionId = Context.ConnectionId;
            logger.LogInformation("ROS bridge connected (ID: {ConnectionId})", Context.ConnectionId);
            await Clients.All.SendAsync("RosConnectionChanged", new { connected = true, timestamp = DateTime.UtcNow });
        }

        // Broadcast full swarm status to frontend
        await Clients.Others.SendAsync("SwarmStatusUpdate", statusJson);

        // Extract per-robot sensor data and send individual RobotSensorUpdate events
        try
        {
            using var doc = JsonDocument.Parse(statusJson);
            if (doc.RootElement.TryGetProperty("robots", out var robots))
            {
                foreach (var robot in robots.EnumerateArray())
                {
                    var id = robot.GetProperty("id").GetString();
                    if (id != null && robot.TryGetProperty("sensors", out var sensors))
                    {
                        await Clients.Others.SendAsync("RobotSensorUpdate", id, sensors.GetRawText());
                    }
                }
            }
        }
        catch (Exception ex)
        {
            logger.LogDebug(ex, "Could not extract per-robot sensor data from status");
        }
    }

    public async Task ForwardFleetEvent(string eventJson)
    {
        _rosBridgeConnected = true;
        await Clients.Others.SendAsync("RobotDeploymentResponse", eventJson);
    }
}
