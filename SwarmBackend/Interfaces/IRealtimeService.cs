using System.Text.Json;
using SwarmBackend.Models;

namespace SwarmBackend.Interfaces;

public interface IRealtimeService
{
    Task OnConnectedAsync();

    Task OnDisconnectedAsync(Exception? exception);

    Task UpdateRobotConnection(int robotId, bool isConnected);

    Task UpdateStatus(int robotId, string status);

    Task HandleSensorReading(int robotId, RosSensorReadingRequest reading);
    Task HandleSensorReadingFromClient(int robotId, string sensorName, Dictionary<string, object> sensorFields);
    Task HandleSensorReadingsBatch(int robotId, RosBatchSensorReadingRequest request);

    Task HandleTaskLog(int robotId, RosTaskTemplateRequest request);
    Task HandleFinishTaskLog(int robotId);
    Task HandleCancelTaskLog(int robotId, int accountId);

    Task SendCommand(int robotId, string command, string parameters);

    Task<List<int>> NotifyRobotsAvailable();

    // Swarm control methods
    Task<bool> SpawnRobots(int count, string pattern);
    Task<bool> SpawnRobotsWithIds(List<RobotDeploymentInfo> robots, string pattern);
    Task<bool> DeleteRobots(List<string> robotIds);
    Task<bool> StartFollowLeader(int robotCount, string leaderMode, object config);
    Task<bool> StartFormation(int robotCount, string formationType, string movementMode, object config);
    Task<bool> StartTransport(int robotCount, double targetX, double targetY, object config);
    Task<bool> StopTask();
    Task<bool> EmergencyStop();
    Task<bool> ControlLeader(double linearVelocity, double angularVelocity);
    Task<bool> SpawnObstacles(string density);
    Task<bool> GetRosConnectionStatus();

    // Bridge forwarding methods
    Task ForwardSwarmStatus(string statusJson);
    Task ForwardFleetEvent(string eventJson);
}
