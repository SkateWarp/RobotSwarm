# Backend Integration Guide
**Connecting .NET Backend to ROS Robot Swarm**

## Architecture Overview

```
┌──────────────────┐    SignalR     ┌──────────────────┐    WebSocket     ┌──────────────────┐
│    Frontend      │ ◄────────────► │   .NET Backend   │ ◄──────────────► │  ROS Orchestrator │
│  (React/Vue/etc) │                │                  │                  │  (Python)         │
└──────────────────┘                └──────────────────┘                  └──────────────────┘
        │                                   │                                     │
        │                                   │                                     │
    User Interface                    Business Logic                      Robot Control
    - Control Panel                   - Authentication                    - Task Execution
    - Robot Monitoring                - Task Validation                   - Status Updates
    - Task Assignment                 - Data Persistence                  - Safety Systems
```

---

## Option 1: Direct WebSocket (Simpler)

Frontend connects directly to ROS Task Orchestrator on `ws://localhost:8080`.

**Pros:**
- Simpler setup
- Lower latency
- No backend modification needed

**Cons:**
- No authentication layer
- No business logic separation
- No data persistence

**Files Created:**
- `SwarmFrontend/src/services/SwarmService.ts` - WebSocket client
- `SwarmFrontend/src/hooks/useSwarmControl.ts` - React hook
- `SwarmFrontend/src/components/SwarmControlPanel.tsx` - UI component

---

## Option 2: Via .NET Backend (Recommended for Production)

### Step 1: Create ROS Bridge Service in .NET

```csharp
// Services/RosBridgeService.cs
using System;
using System.Net.WebSockets;
using System.Text;
using System.Text.Json;
using System.Threading;
using System.Threading.Tasks;
using Microsoft.Extensions.Logging;

namespace YourNamespace.Services
{
    public interface IRosBridgeService
    {
        Task ConnectAsync();
        Task DisconnectAsync();
        Task<bool> SendCommandAsync(string command, object parameters);
        event Action<SwarmStatusUpdate> OnStatusUpdate;
        bool IsConnected { get; }
    }

    public class RosBridgeService : IRosBridgeService, IDisposable
    {
        private readonly ILogger<RosBridgeService> _logger;
        private ClientWebSocket _webSocket;
        private CancellationTokenSource _cts;
        private readonly string _rosUrl;

        public event Action<SwarmStatusUpdate> OnStatusUpdate;
        public bool IsConnected => _webSocket?.State == WebSocketState.Open;

        public RosBridgeService(ILogger<RosBridgeService> logger, IConfiguration config)
        {
            _logger = logger;
            _rosUrl = config.GetValue<string>("RosWebSocket:Url") ?? "ws://localhost:8080";
        }

        public async Task ConnectAsync()
        {
            _webSocket = new ClientWebSocket();
            _cts = new CancellationTokenSource();

            try
            {
                await _webSocket.ConnectAsync(new Uri(_rosUrl), _cts.Token);
                _logger.LogInformation("Connected to ROS at {Url}", _rosUrl);

                // Start receiving messages
                _ = ReceiveLoopAsync();
            }
            catch (Exception ex)
            {
                _logger.LogError(ex, "Failed to connect to ROS");
                throw;
            }
        }

        public async Task DisconnectAsync()
        {
            _cts?.Cancel();
            if (_webSocket?.State == WebSocketState.Open)
            {
                await _webSocket.CloseAsync(WebSocketCloseStatus.NormalClosure,
                    "Closing", CancellationToken.None);
            }
        }

        public async Task<bool> SendCommandAsync(string command, object parameters)
        {
            if (!IsConnected)
            {
                _logger.LogWarning("Cannot send command - not connected to ROS");
                return false;
            }

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
                _cts.Token
            );

            _logger.LogDebug("Sent command: {Command}", command);
            return true;
        }

        private async Task ReceiveLoopAsync()
        {
            var buffer = new byte[4096];

            while (!_cts.Token.IsCancellationRequested && IsConnected)
            {
                try
                {
                    var result = await _webSocket.ReceiveAsync(
                        new ArraySegment<byte>(buffer),
                        _cts.Token
                    );

                    if (result.MessageType == WebSocketMessageType.Close)
                    {
                        break;
                    }

                    var json = Encoding.UTF8.GetString(buffer, 0, result.Count);
                    var message = JsonSerializer.Deserialize<JsonElement>(json);

                    if (message.TryGetProperty("type", out var typeElement) &&
                        typeElement.GetString() == "status_update")
                    {
                        var status = JsonSerializer.Deserialize<SwarmStatusUpdate>(json);
                        OnStatusUpdate?.Invoke(status);
                    }
                }
                catch (OperationCanceledException)
                {
                    break;
                }
                catch (Exception ex)
                {
                    _logger.LogError(ex, "Error receiving from ROS");
                }
            }
        }

        public void Dispose()
        {
            _cts?.Cancel();
            _webSocket?.Dispose();
        }
    }

    // Models
    public class SwarmStatusUpdate
    {
        public string Type { get; set; }
        public string Timestamp { get; set; }
        public TaskStatus Task { get; set; }
        public List<RobotStatus> Robots { get; set; }
        public int RobotCount { get; set; }
        public bool EmergencyStop { get; set; }
    }

    public class TaskStatus
    {
        public string TaskId { get; set; }
        public string TaskType { get; set; }
        public string Status { get; set; }
        public double Progress { get; set; }
    }

    public class RobotStatus
    {
        public string Id { get; set; }
        public string Role { get; set; }
        public Position Position { get; set; }
        public string Status { get; set; }
        public double ThreatLevel { get; set; }
    }

    public class Position
    {
        public double X { get; set; }
        public double Y { get; set; }
        public double Z { get; set; }
    }
}
```

### Step 2: Create SignalR Hub

```csharp
// Hubs/SwarmHub.cs
using Microsoft.AspNetCore.SignalR;
using YourNamespace.Services;

namespace YourNamespace.Hubs
{
    public class SwarmHub : Hub
    {
        private readonly IRosBridgeService _rosBridge;
        private readonly ILogger<SwarmHub> _logger;

        public SwarmHub(IRosBridgeService rosBridge, ILogger<SwarmHub> logger)
        {
            _rosBridge = rosBridge;
            _logger = logger;

            // Forward ROS status updates to SignalR clients
            _rosBridge.OnStatusUpdate += async (status) =>
            {
                await Clients.All.SendAsync("StatusUpdate", status);
            };
        }

        // ==================== Robot Deployment ====================

        public async Task<bool> SpawnRobots(int count, string pattern = "grid")
        {
            _logger.LogInformation("SpawnRobots: count={Count}, pattern={Pattern}", count, pattern);

            return await _rosBridge.SendCommandAsync("spawn_robots", new
            {
                robot_count = Math.Clamp(count, 1, 20),
                spawn_pattern = pattern,
                center = new { x = 0.0, y = 0.0 },
                spacing = 1.0
            });
        }

        public async Task<bool> DeleteRobots(string[] robotIds = null)
        {
            return await _rosBridge.SendCommandAsync("delete_robots", new
            {
                robot_ids = robotIds ?? Array.Empty<string>()
            });
        }

        // ==================== Task Assignment ====================

        public async Task<bool> StartFollowLeader(
            int robotCount,
            string leaderMode,
            List<Waypoint> waypoints = null)
        {
            _logger.LogInformation("Starting follow-leader: mode={Mode}", leaderMode);

            return await _rosBridge.SendCommandAsync("start_task", new
            {
                task_type = "follow_leader",
                parameters = new
                {
                    robot_count = robotCount,
                    leader_mode = leaderMode,
                    waypoints = waypoints ?? new List<Waypoint>(),
                    following_distance = 0.7,
                    max_velocity = 0.22
                }
            });
        }

        public async Task<bool> StartFormation(
            int robotCount,
            string formationType,
            string movementMode = "static")
        {
            _logger.LogInformation("Starting formation: type={Type}", formationType);

            return await _rosBridge.SendCommandAsync("start_task", new
            {
                task_type = "formation",
                parameters = new
                {
                    robot_count = robotCount,
                    formation_type = formationType,
                    formation_spacing = 1.2,
                    movement_mode = movementMode
                }
            });
        }

        public async Task<bool> StartTransport(
            int robotCount,
            double targetX,
            double targetY,
            double objectMass = 2.0)
        {
            _logger.LogInformation("Starting transport to ({X}, {Y})", targetX, targetY);

            return await _rosBridge.SendCommandAsync("start_task", new
            {
                task_type = "transport",
                parameters = new
                {
                    robot_count = robotCount,
                    object_id = "object_1",
                    object_mass = objectMass,
                    object_position = new { x = 2.0, y = 2.0 },
                    target_location = new { x = targetX, y = targetY }
                }
            });
        }

        public async Task<bool> StopTask()
        {
            return await _rosBridge.SendCommandAsync("stop_task", new { });
        }

        public async Task<bool> EmergencyStop()
        {
            _logger.LogWarning("EMERGENCY STOP triggered!");
            return await _rosBridge.SendCommandAsync("emergency_stop", new { });
        }

        // ==================== Leader Control ====================

        public async Task<bool> ControlLeader(double linearVelocity, double angularVelocity)
        {
            return await _rosBridge.SendCommandAsync("control_leader", new
            {
                linear_velocity = Math.Clamp(linearVelocity, -0.26, 0.26),
                angular_velocity = Math.Clamp(angularVelocity, -1.82, 1.82)
            });
        }

        // ==================== Environment ====================

        public async Task<bool> SpawnObstacles(string density)
        {
            return await _rosBridge.SendCommandAsync("configure_environment", new
            {
                obstacle_density = density,
                obstacle_types = new[] { "box", "cylinder" },
                spawn_obstacles = true,
                clear_existing = true
            });
        }

        // ==================== Connection Events ====================

        public override async Task OnConnectedAsync()
        {
            _logger.LogInformation("Client connected: {ConnectionId}", Context.ConnectionId);

            // Ensure ROS connection
            if (!_rosBridge.IsConnected)
            {
                await _rosBridge.ConnectAsync();
            }

            await base.OnConnectedAsync();
        }

        public override Task OnDisconnectedAsync(Exception exception)
        {
            _logger.LogInformation("Client disconnected: {ConnectionId}", Context.ConnectionId);
            return base.OnDisconnectedAsync(exception);
        }
    }

    public class Waypoint
    {
        public double X { get; set; }
        public double Y { get; set; }
        public double? Theta { get; set; }
    }
}
```

### Step 3: Register Services in Program.cs

```csharp
// Program.cs
var builder = WebApplication.CreateBuilder(args);

// Add ROS Bridge service
builder.Services.AddSingleton<IRosBridgeService, RosBridgeService>();

// Add SignalR
builder.Services.AddSignalR();

// Add CORS for frontend
builder.Services.AddCors(options =>
{
    options.AddPolicy("AllowFrontend", policy =>
    {
        policy.WithOrigins("http://localhost:3000") // Your frontend URL
              .AllowAnyHeader()
              .AllowAnyMethod()
              .AllowCredentials();
    });
});

var app = builder.Build();

app.UseCors("AllowFrontend");

// Map SignalR hub
app.MapHub<SwarmHub>("/hubs/swarm");

// Connect to ROS on startup
var rosBridge = app.Services.GetRequiredService<IRosBridgeService>();
await rosBridge.ConnectAsync();

app.Run();
```

### Step 4: appsettings.json

```json
{
  "RosWebSocket": {
    "Url": "ws://localhost:8080"
  },
  "Logging": {
    "LogLevel": {
      "Default": "Information"
    }
  }
}
```

---

## Frontend with SignalR (Option 2)

### Install SignalR Client

```bash
npm install @microsoft/signalr
```

### Create SignalR Service

```typescript
// services/SignalRSwarmService.ts
import * as signalR from '@microsoft/signalr';

class SignalRSwarmService {
  private connection: signalR.HubConnection;
  private onStatusUpdate: ((status: any) => void) | null = null;

  constructor(hubUrl: string = 'http://localhost:5000/hubs/swarm') {
    this.connection = new signalR.HubConnectionBuilder()
      .withUrl(hubUrl)
      .withAutomaticReconnect()
      .build();

    // Handle status updates from server
    this.connection.on('StatusUpdate', (status) => {
      this.onStatusUpdate?.(status);
    });
  }

  async connect(): Promise<void> {
    await this.connection.start();
    console.log('Connected to SignalR hub');
  }

  setStatusCallback(callback: (status: any) => void): void {
    this.onStatusUpdate = callback;
  }

  // Robot Deployment
  async spawnRobots(count: number, pattern: string = 'grid'): Promise<boolean> {
    return await this.connection.invoke('SpawnRobots', count, pattern);
  }

  async deleteRobots(robotIds?: string[]): Promise<boolean> {
    return await this.connection.invoke('DeleteRobots', robotIds || []);
  }

  // Tasks
  async startFollowLeader(
    robotCount: number,
    leaderMode: string,
    waypoints?: any[]
  ): Promise<boolean> {
    return await this.connection.invoke('StartFollowLeader',
      robotCount, leaderMode, waypoints || []);
  }

  async startFormation(
    robotCount: number,
    formationType: string,
    movementMode: string = 'static'
  ): Promise<boolean> {
    return await this.connection.invoke('StartFormation',
      robotCount, formationType, movementMode);
  }

  async startTransport(
    robotCount: number,
    targetX: number,
    targetY: number
  ): Promise<boolean> {
    return await this.connection.invoke('StartTransport',
      robotCount, targetX, targetY);
  }

  async stopTask(): Promise<boolean> {
    return await this.connection.invoke('StopTask');
  }

  async emergencyStop(): Promise<boolean> {
    return await this.connection.invoke('EmergencyStop');
  }

  // Leader Control
  async controlLeader(linear: number, angular: number): Promise<boolean> {
    return await this.connection.invoke('ControlLeader', linear, angular);
  }

  // Environment
  async spawnObstacles(density: string): Promise<boolean> {
    return await this.connection.invoke('SpawnObstacles', density);
  }
}

export default new SignalRSwarmService();
```

---

## Quick Reference: Commands

### From Frontend to ROS

| Action | Direct WebSocket | Via SignalR |
|--------|-----------------|-------------|
| Spawn Robots | `swarmService.spawnRobots(5, 'grid')` | `signalR.invoke('SpawnRobots', 5, 'grid')` |
| Delete Robots | `swarmService.deleteRobots([])` | `signalR.invoke('DeleteRobots', [])` |
| Start Follow-Leader | `swarmService.startFollowLeader({...})` | `signalR.invoke('StartFollowLeader', ...)` |
| Start Formation | `swarmService.startFormation({...})` | `signalR.invoke('StartFormation', ...)` |
| Start Transport | `swarmService.startTransport({...})` | `signalR.invoke('StartTransport', ...)` |
| Stop Task | `swarmService.stopTask()` | `signalR.invoke('StopTask')` |
| Emergency Stop | `swarmService.emergencyStop()` | `signalR.invoke('EmergencyStop')` |
| Control Leader | `swarmService.controlLeader(0.2, 0)` | `signalR.invoke('ControlLeader', 0.2, 0)` |
| Spawn Obstacles | `swarmService.configureEnvironment('medium')` | `signalR.invoke('SpawnObstacles', 'medium')` |

---

## Running the Complete System

### Terminal 1: ROS (Linux/WSL)
```bash
cd ~/RobotSwarm/swarm_ws
source devel/setup.bash
roslaunch robot_swarm_bridge swarm_main.launch robot_count:=5
```

### Terminal 2: .NET Backend (Optional)
```bash
cd ~/RobotSwarm/Backend
dotnet run
```

### Terminal 3: Frontend
```bash
cd ~/RobotSwarm/SwarmFrontend
npm start
```

### Access:
- **Frontend:** http://localhost:3000
- **ROS WebSocket:** ws://localhost:8080
- **SignalR Hub:** http://localhost:5000/hubs/swarm

---

## Troubleshooting

### Issue: WebSocket connection refused
```bash
# Check if ROS Task Orchestrator is running
rostopic list | grep swarm

# Check if port 8080 is open
netstat -an | grep 8080
```

### Issue: SignalR not receiving updates
```bash
# Check backend logs
dotnet run --verbosity detailed

# Verify ROS bridge connection
curl -i http://localhost:5000/health
```

### Issue: Robots not responding
```bash
# Check robot topics
rostopic list | grep robot_1

# Echo robot status
rostopic echo /fleet/robot_list

# Check for errors
rosnode list
rosnode info /task_orchestrator
```
