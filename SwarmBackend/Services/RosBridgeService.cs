using System.Net.WebSockets;
using System.Text;
using System.Text.Json;
using Microsoft.AspNetCore.SignalR;

namespace SwarmBackend.Services;

/// <summary>
/// Service that bridges ROS WebSocket to SignalR Hub
/// Connects to the ROS Task Orchestrator WebSocket and forwards messages to frontend clients
/// </summary>
public class RosBridgeService : IHostedService, IDisposable
{
    private readonly ILogger<RosBridgeService> _logger;
    private readonly IHubContext<RobotHub> _hubContext;
    private readonly IConfiguration _configuration;
    private ClientWebSocket? _webSocket;
    private CancellationTokenSource? _cts;
    private Task? _receiveTask;
    private readonly string _rosUrl;
    private bool _isConnected;
    private int _reconnectAttempts;
    private const int MaxReconnectAttempts = 10;
    private const int ReconnectDelayMs = 5000;

    public bool IsConnected => _isConnected;

    public RosBridgeService(
        ILogger<RosBridgeService> logger,
        IHubContext<RobotHub> hubContext,
        IConfiguration configuration)
    {
        _logger = logger;
        _hubContext = hubContext;
        _configuration = configuration;
        _rosUrl = configuration.GetValue<string>("RosWebSocket:Url") ?? "ws://localhost:8080";
    }

    public async Task StartAsync(CancellationToken cancellationToken)
    {
        _logger.LogInformation("RosBridgeService starting, will connect to {Url}", _rosUrl);
        _cts = CancellationTokenSource.CreateLinkedTokenSource(cancellationToken);

        // Start connection in background
        _ = ConnectWithRetryAsync(_cts.Token);

        await Task.CompletedTask;
    }

    public async Task StopAsync(CancellationToken cancellationToken)
    {
        _logger.LogInformation("RosBridgeService stopping");
        _cts?.Cancel();

        if (_webSocket?.State == WebSocketState.Open)
        {
            try
            {
                await _webSocket.CloseAsync(
                    WebSocketCloseStatus.NormalClosure,
                    "Service stopping",
                    CancellationToken.None);
            }
            catch (Exception ex)
            {
                _logger.LogWarning(ex, "Error closing WebSocket");
            }
        }

        if (_receiveTask != null)
        {
            try
            {
                await _receiveTask;
            }
            catch (OperationCanceledException)
            {
                // Expected
            }
        }
    }

    private async Task ConnectWithRetryAsync(CancellationToken cancellationToken)
    {
        while (!cancellationToken.IsCancellationRequested && _reconnectAttempts < MaxReconnectAttempts)
        {
            try
            {
                await ConnectAsync(cancellationToken);
                _reconnectAttempts = 0; // Reset on successful connection

                // Start receive loop
                _receiveTask = ReceiveLoopAsync(cancellationToken);
                await _receiveTask;
            }
            catch (OperationCanceledException)
            {
                break;
            }
            catch (Exception ex)
            {
                _isConnected = false;
                _reconnectAttempts++;
                _logger.LogWarning(ex, "ROS WebSocket connection failed (attempt {Attempt}/{Max})",
                    _reconnectAttempts, MaxReconnectAttempts);

                if (_reconnectAttempts < MaxReconnectAttempts)
                {
                    var delay = ReconnectDelayMs * Math.Min(_reconnectAttempts, 5);
                    _logger.LogInformation("Reconnecting in {Delay}ms...", delay);
                    await Task.Delay(delay, cancellationToken);
                }
            }
        }

        if (_reconnectAttempts >= MaxReconnectAttempts)
        {
            _logger.LogError("Max reconnection attempts reached. ROS bridge service stopped.");
        }
    }

    private async Task ConnectAsync(CancellationToken cancellationToken)
    {
        _webSocket?.Dispose();
        _webSocket = new ClientWebSocket();

        _logger.LogInformation("Connecting to ROS at {Url}...", _rosUrl);
        await _webSocket.ConnectAsync(new Uri(_rosUrl), cancellationToken);

        _isConnected = true;
        _logger.LogInformation("Connected to ROS WebSocket");

        // Notify SignalR clients that ROS is connected
        await _hubContext.Clients.All.SendAsync("RosConnectionChanged", new
        {
            connected = true,
            timestamp = DateTime.UtcNow
        }, cancellationToken);
    }

    private async Task ReceiveLoopAsync(CancellationToken cancellationToken)
    {
        var buffer = new byte[8192];
        var messageBuilder = new StringBuilder();

        while (!cancellationToken.IsCancellationRequested &&
               _webSocket?.State == WebSocketState.Open)
        {
            try
            {
                var result = await _webSocket.ReceiveAsync(
                    new ArraySegment<byte>(buffer),
                    cancellationToken);

                if (result.MessageType == WebSocketMessageType.Close)
                {
                    _isConnected = false;
                    _logger.LogInformation("ROS WebSocket closed by server");

                    await _hubContext.Clients.All.SendAsync("RosConnectionChanged", new
                    {
                        connected = false,
                        timestamp = DateTime.UtcNow
                    }, cancellationToken);

                    break;
                }

                messageBuilder.Append(Encoding.UTF8.GetString(buffer, 0, result.Count));

                if (result.EndOfMessage)
                {
                    var json = messageBuilder.ToString();
                    messageBuilder.Clear();

                    await ProcessMessageAsync(json, cancellationToken);
                }
            }
            catch (OperationCanceledException)
            {
                break;
            }
            catch (WebSocketException ex)
            {
                _isConnected = false;
                _logger.LogError(ex, "WebSocket error in receive loop");
                break;
            }
        }

        _isConnected = false;
    }

    private async Task ProcessMessageAsync(string json, CancellationToken cancellationToken)
    {
        try
        {
            using var doc = JsonDocument.Parse(json);
            var root = doc.RootElement;

            if (!root.TryGetProperty("type", out var typeElement))
            {
                return;
            }

            var messageType = typeElement.GetString();

            switch (messageType)
            {
                case "status_update":
                    // Forward swarm status to all SignalR clients
                    await _hubContext.Clients.All.SendAsync(
                        "SwarmStatusUpdate",
                        json,
                        cancellationToken);

                    // Also extract and broadcast individual robot sensor data
                    if (root.TryGetProperty("robots", out var robotsElement))
                    {
                        foreach (var robot in robotsElement.EnumerateArray())
                        {
                            if (robot.TryGetProperty("id", out var idElement) &&
                                robot.TryGetProperty("sensors", out var sensorsElement))
                            {
                                var robotId = idElement.GetString();
                                await _hubContext.Clients.All.SendAsync(
                                    $"RobotSensorUpdate/{robotId}",
                                    sensorsElement.GetRawText(),
                                    cancellationToken);
                            }
                        }
                    }
                    break;

                case "task_started":
                case "task_stopped":
                case "task_complete":
                    await _hubContext.Clients.All.SendAsync(
                        "TaskEvent",
                        json,
                        cancellationToken);
                    break;

                case "emergency_stop_activated":
                    await _hubContext.Clients.All.SendAsync(
                        "EmergencyStop",
                        json,
                        cancellationToken);
                    break;

                case "spawn_response":
                case "delete_response":
                    await _hubContext.Clients.All.SendAsync(
                        "RobotDeploymentResponse",
                        json,
                        cancellationToken);
                    break;

                default:
                    _logger.LogDebug("Unknown message type from ROS: {Type}", messageType);
                    break;
            }
        }
        catch (JsonException ex)
        {
            _logger.LogWarning(ex, "Failed to parse ROS message");
        }
    }

    /// <summary>
    /// Send command to ROS Task Orchestrator
    /// </summary>
    public async Task<bool> SendCommandAsync(string command, object parameters)
    {
        if (_webSocket?.State != WebSocketState.Open)
        {
            _logger.LogWarning("Cannot send command - not connected to ROS");
            return false;
        }

        try
        {
            var message = new
            {
                command = command,
                parameters = parameters,
                timestamp = DateTime.UtcNow.ToString("O")
            };

            var json = JsonSerializer.Serialize(message);
            var bytes = Encoding.UTF8.GetBytes(json);

            await _webSocket.SendAsync(
                new ArraySegment<byte>(bytes),
                WebSocketMessageType.Text,
                true,
                _cts?.Token ?? CancellationToken.None);

            _logger.LogDebug("Sent command to ROS: {Command}", command);
            return true;
        }
        catch (Exception ex)
        {
            _logger.LogError(ex, "Failed to send command to ROS: {Command}", command);
            return false;
        }
    }

    public void Dispose()
    {
        _cts?.Cancel();
        _cts?.Dispose();
        _webSocket?.Dispose();
    }
}
