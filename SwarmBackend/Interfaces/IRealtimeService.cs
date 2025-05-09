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

    Task HandleTaskLog(int robotId, RosTaskTemplateRequest request);
    Task HandleFinishTaskLog(int robotId);
    Task HandleCancelTaskLog(int robotId);

    Task SendCommand(int robotId, string command, string parameters);

    Task<List<int>> NotifyRobotsAvailable();


}