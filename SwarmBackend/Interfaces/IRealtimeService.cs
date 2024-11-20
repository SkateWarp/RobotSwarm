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

    Task SendCommand(int robotId, string command, JsonDocument parameters);


}