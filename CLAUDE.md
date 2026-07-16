# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

RobotSwarm is a robotic swarm management system consisting of three main components:
- **SwarmFrontend**: React 17 web app built on the Fuse React admin template (MUI 5, Redux Toolkit, craco)
- **SwarmBackend**: .NET 8 Minimal API backend with PostgreSQL, EF Core, and SignalR for real-time communication
- **swarm_ws**: ROS (Robot Operating System) workspace with a Python bridge connecting ROS to the backend via SignalR

## Common Commands

### Frontend (SwarmFrontend/)
```bash
npm start          # Dev server at localhost:3000 (uses craco)
npm run build      # Production build (no sourcemaps via cross-env)
npm run test       # Run tests (craco test --env=node)
npm run lint       # ESLint
```

### Backend (SwarmBackend/)
```bash
dotnet run                           # Run backend (HTTP on port 44336)
dotnet ef database update            # Apply EF Core migrations
dotnet ef migrations add <Name>      # Create new migration
```
Swagger UI is enabled in Development at `/swagger`.

### Database
```bash
# Local development (postgres/postgres/swarm on port 5432)
docker compose -f SwarmBackend/docker-compose.local.yml up -d

# Production (export the values shown in .env.example first)
docker compose -f docker-compose.prod.yml up -d
```

### ROS Workspace (swarm_ws/)
```bash
catkin_make                   # Build ROS packages
source devel/setup.bash       # Source workspace
rosrun robot_swarm_bridge bridge.py  # Run the SignalR-ROS bridge
```

## Architecture

### Communication Flow
```
React Frontend <--SignalR--> .NET Backend <--SignalR--> ROS Bridge <--ROS Topics--> Robots
```

1. **Frontend** connects to backend SignalR hub at `/hubs/robot` using `@microsoft/signalr`
2. **Backend** (`RobotHub.cs`) handles real-time robot status, sensor readings, commands, and task logs
3. **ROS Bridge** (`bridge.py`) connects to SignalR using `signalrcore` Python library and translates messages to/from ROS topics

### SignalR Hub Methods (Backend → called by clients)

| Method | Parameters | Description |
|--------|-----------|-------------|
| `UpdateStatus` | `robotId, status` | Update robot status (Idle/Working/Disabled) |
| `HandleSensorReading` | `robotId, reading` | Single sensor reading from ROS |
| `HandleSensorReadingsBatch` | `robotId, request` | Batch sensor readings (preferred) |
| `HandleSensorReadingFromClient` | `robotId, sensorName, sensorFields` | Sensor reading from frontend |
| `SendCommand` | `robotId, command, parameters` | Send command to robots |
| `HandleTaskLog` | `robotId, request` | Start a task log |
| `HandleFinishTaskLog` | `robotId` | Finish active task |
| `HandleCancelTaskLog` | `robotId, accountId` | Cancel active tasks |

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

### ROS Topics (per robot)

| Topic | Direction | Description |
|-------|-----------|-------------|
| `/robot/{id}/commands` | Bridge → ROS | Commands from frontend |
| `/robot/{id}/status` | ROS → Bridge | Status updates (Working/Idle) |
| `/robot/{id}/sensor_data` | ROS → Bridge | Sensor data (JSON) |
| `/robot/{id}/task/start` | ROS → Bridge | Task started |
| `/robot/{id}/task/finish` | ROS → Bridge | Task finished |
| `/robot/{id}/task/cancel` | ROS → Bridge | Task cancelled |
| `/listOfAvailableRobots` | Bridge → ROS | Available robot IDs |

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
- `Entities/`: EF Core models — `Robot`, `Sensor`, `SensorReading`, `TaskLog`, `TaskTemplate`, `Account`, `RobotGroup`
- `Routes/`: Minimal API route extensions (`MapGroup("Robots").MapRobot()`)
- `Services/`: Business logic (each service implements an interface from `Interfaces/`)
- `Models/`: Request/response records
- `Helpers/DataContext.cs`: EF Core DbContext
- Uses `LanguageExt` for functional error handling (`response.Match(Results.Ok, Results.BadRequest)`)

### ROS Bridge (swarm_ws/src/robot_swarm_bridge/)
- `src/robot_swarm_bridge/bridge.py`: Main entry point — creates SignalRHandler + ROSHandler per robot
- `src/robot_swarm_bridge/handlers/signalr_handler.py`: SignalR client with auto-reconnect and message queuing
- `src/robot_swarm_bridge/handlers/ros_handler.py`: ROS pub/sub with rate limiting (status: 5s, sensor: 1s, task: 5s)
- `scripts/behaviors/`: Swarm behaviors — follow_leader, formation_control, collaborative_transport
- `scripts/core/`: Fleet management, obstacle avoidance, task orchestration

### Key Enums
- **RobotStatus**: `Idle`, `Working`, `Disabled`
- **TaskTypeEnum**: `None`, `Transport`, `FollowLeader`, `Formation`
- **SensorTypeEnum**: Defined in `Entities/SensorTypeEnum.cs`, mapped as PostgreSQL enum

## Environment

Backend PostgreSQL connection:
- **Local dev**: `docker-compose.local.yml` uses hardcoded `postgres/postgres/swarm`
- **Production**: the protected GitHub `production` environment supplies
  `DB_USER`, `DB_PASSWORD`, `DB_NAME`, and `APPSETTINGS_SECRET`
- `.env.example` is a local template; a populated `.env` must remain untracked
- Connection string in `appsettings.json` uses `${DB_USER}` env var substitution

Frontend API URL: set in `src/app/constants/constants.js` (production: `https://robot.zerav.la`)

JWT auth configured in `appsettings.json` under `AppSettings` (Secret is base64-encoded).

## CI/CD

Cloudflare Workers deploys the frontend from the connected GitHub repository
after production-branch commits. GitHub Actions still builds the React app as
a CI check.

The `backend-prod` runner deploys only the backend, database, and media
services, and only when those paths changed. GPU worker deployment is a
separate manually approved maintenance workflow.
