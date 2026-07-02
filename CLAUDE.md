# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

RobotSwarm is a robotic swarm management system consisting of three main components:
- **SwarmFrontend**: React 17 web app built on the Fuse React admin template (MUI 5, Redux Toolkit, craco)
- **SwarmBackend**: .NET 8 Minimal API backend with PostgreSQL, EF Core, and SignalR for real-time communication
- **swarm_ws**: ROS Noetic workspace with a Python bridge connecting ROS to the backend via SignalR, plus a Gazebo/TurtleBot3 swarm simulation (follow-leader, formations, collaborative transport)

Other root-level directories:
- `firmware/`: Arduino/ESP32 sketches and PID autotune code for physical robots
- `ros_wss/`: experimental standalone .NET rosbridge WebSocket client (not part of the main flow)
- `swarm_ws/src/hero_common/`: third-party HeRo robot platform packages — avoid modifying

Additional design docs at the root: `ARCHITECTURE.md` (simulation system design), `QUICKSTART.md` (ROS/Gazebo setup and component tests), `BACKEND_INTEGRATION.md`, `FRONTEND_API.md`.

## Common Commands

### Full stack
```bash
./start.sh [robot_count]   # DB + backend via docker-compose.yml, then catkin_make + roslaunch swarm_main.launch
```

### Frontend (SwarmFrontend/)
```bash
npm start          # Dev server at localhost:3000 (uses craco)
npm run build      # Production build (no sourcemaps via cross-env)
npm run test       # Run tests (craco test --env=node)
npm run lint       # ESLint
```

### Backend (SwarmBackend/)
```bash
dotnet run                           # Run backend (HTTP 44336, HTTPS 44337)
dotnet ef database update            # Apply EF Core migrations
dotnet ef migrations add <Name>      # Create new migration
```
Swagger UI is always enabled at `/swagger`.

### Database
```bash
# Local development (postgres/postgres/swarm on port 5432)
docker compose -f SwarmBackend/docker-compose.local.yml up -d

# Full stack DB + backend (root; uses .env for DB_USER, DB_PASSWORD, DB_NAME)
docker compose up -d

# Production
docker compose -f docker-compose.prod.yml up -d
```

### ROS Workspace (swarm_ws/) — requires ROS Noetic + TurtleBot3 packages
```bash
catkin_make                   # Build ROS packages (also generates custom msg/srv)
source devel/setup.bash       # Source workspace
export TURTLEBOT3_MODEL=waffle

# Full simulation: Gazebo + arena + N robots + core nodes (bridge, orchestrator, fleet manager)
roslaunch robot_swarm_bridge swarm_main.launch robot_count:=5

# Behavior-specific launches (include swarm_main.launch)
roslaunch robot_swarm_bridge follow_leader.launch
roslaunch robot_swarm_bridge formation.launch
roslaunch robot_swarm_bridge transport.launch

# Bridge only (against real robots, no Gazebo)
rosrun robot_swarm_bridge bridge.py
```
Bridge Python deps: `pip3 install -r swarm_ws/src/robot_swarm_bridge/requirements.txt` (signalrcore, pyyaml, python-dotenv).

## Architecture

### Communication Flow
```
React Frontend <--SignalR--> .NET Backend <--SignalR--> ROS Bridge <--ROS Topics--> Robots (Gazebo or real)
```

1. **Frontend** connects to backend SignalR hub at `/hubs/robot` using `@microsoft/signalr`
2. **Backend** (`RobotHub.cs`) handles real-time robot status, sensor readings, commands, task logs, and swarm control
3. **ROS Bridge** (`bridge.py`) connects to SignalR using `signalrcore` and translates messages to/from ROS topics

There are two message planes:
- **Per-robot data plane**: status/sensor/task-log traffic per robot ID (topics `/robot/{id}/...`)
- **Swarm control plane**: frontend invokes hub swarm methods → backend broadcasts a `SwarmCommand` JSON event → bridge publishes it to `/swarm/commands` → `task_orchestrator.py` dispatches to behavior controllers. Status flows back: orchestrator publishes `/swarm/status` → bridge calls hub `ForwardSwarmStatus` → backend broadcasts `SwarmStatusUpdate` (and per-robot `RobotSensorUpdate`) to the frontend. The backend infers ROS bridge connectivity from these calls and broadcasts `RosConnectionChanged`.

### SignalR Hub Methods (Backend → called by clients)

Per-robot data plane:

| Method | Parameters | Description |
|--------|-----------|-------------|
| `UpdateStatus` | `robotId, status` | Update robot status (Idle/Working/Disabled) |
| `UpdateRobotConnection` | `robotId, isConnected` | Robot connect/disconnect |
| `HandleSensorReading` | `robotId, reading` | Single sensor reading from ROS |
| `HandleSensorReadingsBatch` | `robotId, request` | Batch sensor readings (preferred) |
| `HandleSensorReadingFromClient` | `robotId, sensorName, sensorFields` | Sensor reading from frontend |
| `SendCommand` | `robotId, command, parameters` | Send command to robots |
| `HandleTaskLog` | `robotId, request` | Start a task log |
| `HandleFinishTaskLog` | `robotId` | Finish active task |
| `HandleCancelTaskLog` | `robotId, accountId` | Cancel active tasks |

Swarm control (invoked by frontend; forwarded to bridge as `SwarmCommand`):

| Method | Description |
|--------|-------------|
| `SpawnRobots` / `SpawnRobotsWithIds` | Spawn Gazebo robots; `WithIds` maps DB robots to `tb3_{i}` namespaces |
| `DeleteRobots` | Remove robots, clear namespaces |
| `StartFollowLeader` / `StartFormation` / `StartTransport` | Start a swarm behavior |
| `StopTask` / `EmergencyStop` | Stop current task / e-stop all robots |
| `ControlLeader` | Manual leader teleop (linear/angular velocity) |
| `SpawnObstacles` | Generate arena obstacles by density |
| `GetRosConnectionStatus` | Whether the ROS bridge is connected |

Bridge → backend forwarding: `ForwardSwarmStatus(statusJson)`, `ForwardFleetEvent(eventJson)`.

### SignalR Events (Backend → broadcast to clients)

| Event | Description |
|-------|-------------|
| `RobotConnectionChanged/{robotId}` | Robot connected/disconnected |
| `RobotStatusChanged/{robotId}` | Robot status changed |
| `NewSensorReading` | New individual sensor reading |
| `AllSensorReadings/{robotId}` | All latest readings for a robot |
| `ExecuteCommand` | Command sent to robot(s) |
| `RobotsAvailable` | List of non-disabled robot IDs |
| `NewTaskLog` | New task log created |
| `RobotCreated` | New robot created |
| `SwarmCommand` | Swarm control command JSON (consumed by the ROS bridge) |
| `SwarmStatusUpdate` | Full swarm status JSON from the orchestrator |
| `RobotSensorUpdate` | Per-robot sensor payload extracted from swarm status |
| `RobotDeploymentResponse` | Robot ID ↔ namespace mapping after spawn |
| `TaskEvent` | Task started/stopped notifications |
| `EmergencyStop` | Emergency stop broadcast |
| `RosConnectionChanged` | ROS bridge connected/disconnected |

### ROS Topics

Per robot:

| Topic | Direction | Description |
|-------|-----------|-------------|
| `/robot/{id}/commands` | Bridge → ROS | Commands from frontend |
| `/robot/{id}/status` | ROS → Bridge | Status updates (Working/Idle) |
| `/robot/{id}/sensor_data` | ROS → Bridge | Sensor data (JSON) |
| `/robot/{id}/task/start` | ROS → Bridge | Task started |
| `/robot/{id}/task/finish` | ROS → Bridge | Task finished |
| `/robot/{id}/task/cancel` | ROS → Bridge | Task cancelled |
| `/listOfAvailableRobots` | Bridge → ROS | Available robot IDs |

Swarm-wide:

| Topic | Direction | Description |
|-------|-----------|-------------|
| `/swarm/commands` | Bridge → Orchestrator | Swarm commands (JSON String) |
| `/swarm/status` | Orchestrator → Bridge | Full swarm status (JSON String) |
| `/swarm/emergency_stop` | Orchestrator → all | Latched e-stop flag |

Simulated robots live under `tb3_{n}` namespaces (TurtleBot3, standard `cmd_vel`/`odom`/`scan` topics). Custom message and service definitions are in `robot_swarm_bridge/msg/` and `srv/` (e.g. `TaskRequest`, `SwarmStatus`, `SpawnRobots`, `AssignTask`) — rebuild with `catkin_make` after changing them.

### Frontend Structure (SwarmFrontend/src/)
- `@fuse/`: Fuse React admin template framework (layouts, navigation, themes) — avoid modifying
- `app/fuse-configs/settingsConfig.js`: Project selector — `layout.project` controls which app loads
- `app/fuse-configs/routesConfig.js`: Route configuration based on selected project
- `app/services/SignalRService/signalRConnectionService.js`: SignalR connection singleton
- `app/constants/constants.js`: API URL (`URL`) and branding
- `app/main/apps/GeeTS/`: Main application modules:
  - `Dashboard/` — Robot dashboard with widgets
  - `Realtime/` — Real-time control panel, VNC viewer, command sending
  - `RobotDetail/` — Individual robot view with sensor charts and task logs
  - `TaskLog/`, `Tasks/` — Task management
  - `LeafType/`, `LeafSorting/` — Domain-specific configuration
- Each module follows pattern: `*AppConfig.js` (routes), `*App.js` (main component), `store/` (Redux slice)

### Backend Structure (SwarmBackend/)
- `Program.cs`: Entry point — DI, middleware, Minimal API route groups
- `Services/RobotHub.cs`: SignalR hub (implements `IRealtimeService`)
- `Entities/`: EF Core models — `Robot` (has `Namespace` linking DB robots to simulation namespaces), `Sensor`, `SensorReading`, `TaskLog`, `TaskTemplate`, `Account`, `RobotGroup`
- `Routes/`: Minimal API route extensions (`MapGroup("Robots").MapRobot()`)
- `Services/`: Business logic (each service implements an interface from `Interfaces/`)
- `Models/`: Request/response records
- `Helpers/DataContext.cs`: EF Core DbContext
- Uses `LanguageExt` for functional error handling (`response.Match(Results.Ok, Results.BadRequest)`)

### ROS Bridge & Simulation (swarm_ws/src/robot_swarm_bridge/)
- `src/robot_swarm_bridge/bridge.py`: Main entry point — creates SignalRHandler + ROSHandler per robot, relays `/swarm/commands` and `/swarm/status`
- `src/robot_swarm_bridge/handlers/signalr_handler.py`: SignalR client with auto-reconnect and message queuing
- `src/robot_swarm_bridge/handlers/ros_handler.py`: ROS pub/sub with rate limiting (status: 5s, sensor: 1s, task: 5s)
- `config/config.yaml`: Bridge config — backend SignalR URL, robot IDs, topic templates
- `config/swarm_config.yaml`: Simulation parameters — fleet size, arena, safety distances, behavior tuning
- `scripts/behaviors/`: Swarm behaviors — follow_leader, formation_control, collaborative_transport
- `scripts/core/`: `task_orchestrator.py` (dispatches `/swarm/commands` to behaviors), `fleet_manager.py`, `obstacle_avoidance.py`
- `scripts/environment/obstacle_generator.py`, `scripts/robot_manager.py` (Gazebo spawn/delete services)
- `launch/swarm_main.launch`: Base launch (Gazebo + arena + robots + core nodes), included by behavior launches

### Key Enums
- **RobotStatus**: `Idle`, `Working`, `Disabled`
- **TaskTypeEnum**: `None`, `Transport`, `FollowLeader`, `Formation`
- **SensorTypeEnum**: Defined in `Entities/SensorTypeEnum.cs`, mapped as PostgreSQL enum

## Environment

Backend PostgreSQL connection:
- **Local dev**: `docker-compose.local.yml` uses hardcoded `postgres/postgres/swarm`
- **Root/Production**: `.env` file at project root with `DB_USER`, `DB_PASSWORD`, `DB_NAME`
- Connection string in `appsettings.json` uses `${DB_USER}` env var substitution

Frontend API URL: set in `src/app/constants/constants.js` (production: `https://robot.zerav.la`)

JWT auth configured in `appsettings.json` under `AppSettings` (Secret is base64-encoded).

## CI/CD

GitHub Actions on self-hosted runners, both triggered on push to `main`:
- `frontend_workflow.yml`: on `SwarmFrontend/**` changes — Node.js 18 build, copied to `/var/www/html/`
- `backend_workflow.yml`: on `SwarmBackend/**`, `swarm_ws/**`, or `docker-compose.prod.yml` changes — builds the backend Docker image and redeploys `docker-compose.prod.yml`
