# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

RobotSwarm is a robotic swarm management system consisting of three main components:
- **SwarmFrontend**: React web application for robot control and monitoring
- **SwarmBackend**: .NET 8 API backend with PostgreSQL and SignalR for real-time communication
- **swarm_ws**: ROS (Robot Operating System) workspace with a Python bridge connecting ROS to the backend

## Common Commands

### Frontend (SwarmFrontend/)
```bash
npm start          # Development server at localhost:3000
npm run build      # Production build
npm run test       # Run tests
npm run lint       # ESLint
```

### Backend (SwarmBackend/)
```bash
dotnet run                    # Run backend (listens on port 44336)
dotnet ef database update     # Apply EF Core migrations
dotnet ef migrations add <Name>  # Create new migration
```

### Database (from project root)
```bash
docker compose -f docker-compose.prod.yml up -d  # Production PostgreSQL
docker compose -f SwarmBackend/docker-compose.local.yml up -d  # Local development
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

1. **Frontend** connects to backend via SignalR hub at `/hubs/robot`
2. **Backend** (RobotHub.cs) handles real-time robot status, sensor readings, and task logs
3. **ROS Bridge** (robot_swarm_bridge) connects to SignalR and translates messages to/from ROS topics

### Frontend Structure (SwarmFrontend/src/)
- `app/fuse-configs/settingsConfig.js`: Project selector (GTS, task, baldom, fraga)
- `app/fuse-configs/routesConfig.js`: Route configuration based on project
- `app/services/SignalRService/`: SignalR connection singleton
- `app/main/apps/GeeTS/`: GTS (Gee Tobacco Sorting) application modules
- `app/store/`: Redux store with async reducer injection

### Backend Structure (SwarmBackend/)
- `Program.cs`: Application entry point, middleware config, route groups
- `Services/RobotHub.cs`: SignalR hub for real-time robot communication
- `Entities/`: EF Core entity models (Robot, Sensor, TaskLog, Account, etc.)
- `Routes/`: Minimal API route definitions
- `Services/`: Business logic services

### ROS Bridge (swarm_ws/src/robot_swarm_bridge/)
- `src/robot_swarm_bridge/bridge.py`: Main bridge connecting SignalR to ROS
- `handlers/signalr_handler.py`: SignalR client for backend communication
- `handlers/ros_handler.py`: ROS publisher/subscriber management

### Key Entities
- **Robot**: Managed robots with status (Idle, Working, Disabled) and connection state
- **RobotGroup**: Grouping for robot organization
- **Sensor/SensorReading**: Robot sensor data
- **TaskLog/TaskTemplate**: Task execution tracking

## Project Configuration

The frontend supports multiple projects configured in `settingsConfig.js`:
- `GTS` / `GTS-swedish`: Gee Tobacco Sorting
- `task`: Task dashboard
- `baldom`, `fraga`: Other projects
- `fakeBaldom`, `fakeFraga`: Mockup modes

## Environment

Backend expects PostgreSQL with connection configured via:
- Environment variables: `DB_USER`, `DB_PASSWORD`, `DB_NAME`
- Or connection string in `appsettings.json`

Frontend API URL configured in `src/app/constants/constants.js`.
