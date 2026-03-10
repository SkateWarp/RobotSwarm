# Frontend API Documentation
**WebSocket Control Interface for Multi-Robot Swarm System**

## Connection

### WebSocket Endpoint
```
ws://localhost:8080/swarm
```

### Connection Parameters
- **Protocol**: WebSocket (RFC 6455)
- **Subprotocol**: JSON messages
- **Heartbeat**: 10 seconds
- **Reconnect**: Automatic with exponential backoff

---

## Message Format

All messages use JSON format:

```json
{
  "command": "command_name",
  "parameters": { },
  "timestamp": "2026-01-05T10:30:00Z"
}
```

---

## Commands from Frontend to Backend

### 1. Spawn Robots

```json
{
  "command": "spawn_robots",
  "parameters": {
    "robot_count": 5,
    "spawn_pattern": "grid",
    "center": {"x": 0.0, "y": 0.0},
    "spacing": 1.0
  }
}
```

**Parameters:**
- `robot_count` (int): Number of robots to spawn (1-20)
- `spawn_pattern` (string): "grid" | "circle" | "line"
- `center` (object): Spawn center point {x, y}
- `spacing` (float): Distance between robots in meters

**Response:**
```json
{
  "type": "spawn_response",
  "success": true,
  "message": "Successfully spawned 5/5 robots",
  "robot_ids": ["robot_1", "robot_2", "robot_3", "robot_4", "robot_5"]
}
```

---

### 2. Delete Robots

```json
{
  "command": "delete_robots",
  "parameters": {
    "robot_ids": ["robot_1", "robot_2"]
  }
}
```

**Parameters:**
- `robot_ids` (array): Robot IDs to delete. Empty array = delete all

**Response:**
```json
{
  "type": "delete_response",
  "success": true,
  "message": "Deleted 2/2 robots",
  "deleted_count": 2
}
```

---

### 3. Start Follow-the-Leader Task

```json
{
  "command": "start_task",
  "task_type": "follow_leader",
  "parameters": {
    "robot_count": 5,
    "leader_mode": "waypoint",
    "waypoints": [
      {"x": 2.0, "y": 2.0, "theta": 0.0},
      {"x": 4.0, "y": 2.0, "theta": 1.57},
      {"x": 4.0, "y": 4.0, "theta": 3.14},
      {"x": 2.0, "y": 4.0, "theta": 4.71},
      {"x": 2.0, "y": 2.0, "theta": 0.0}
    ],
    "following_distance": 0.7,
    "time_delay": 0.5,
    "max_velocity": 0.22
  }
}
```

**Leader Modes:**
- `waypoint`: Follow predefined waypoint list
- `manual`: Direct control via frontend
- `circular`: Follow circular path
- `square`: Follow square path
- `figure8`: Follow figure-8 path
- `random`: Random exploration

**Response:**
```json
{
  "type": "task_started",
  "success": true,
  "task_id": "uuid-string",
  "task_type": "follow_leader",
  "message": "Task started successfully"
}
```

---

### 4. Start Formation Task

```json
{
  "command": "start_task",
  "task_type": "formation",
  "parameters": {
    "robot_count": 6,
    "formation_type": "circle",
    "formation_spacing": 1.2,
    "movement_mode": "moving",
    "target_pose": {"x": 5.0, "y": 5.0, "theta": 0.0},
    "formation_velocity": 0.15,
    "obstacle_handling": "adaptive"
  }
}
```

**Formation Types:**
- `line`: Straight line
- `triangle`: Equilateral triangle
- `circle`: Circular arrangement
- `square`: Square grid
- `v`: V-formation
- `diamond`: Diamond shape (4 robots)

**Movement Modes:**
- `static`: Reach formation and hold position
- `moving`: Move while maintaining formation
- `adaptive`: Deform around obstacles, reform after

**Obstacle Handling:**
- `deformation`: Formation stretches around obstacles
- `shift`: Formation moves laterally
- `seek_space`: Find wider paths

**Response:**
```json
{
  "type": "task_started",
  "success": true,
  "task_id": "uuid-string",
  "task_type": "formation",
  "message": "Formation task started"
}
```

---

### 5. Start Collaborative Transport Task

```json
{
  "command": "start_task",
  "task_type": "transport",
  "parameters": {
    "robot_count": 4,
    "object_id": "box_1",
    "object_mass": 2.5,
    "object_position": {"x": 3.0, "y": 3.0},
    "target_location": {"x": 8.0, "y": 8.0},
    "detection_method": "position"
  }
}
```

**Detection Methods:**
- `position`: Use known object position
- `apriltag`: Detect via AprilTag markers
- `visual`: Visual detection (if implemented)

**Response:**
```json
{
  "type": "task_started",
  "success": true,
  "task_id": "uuid-string",
  "task_type": "transport",
  "message": "Transport task started",
  "assigned_robots": ["robot_1", "robot_2", "robot_3", "robot_4"]
}
```

---

### 6. Stop Current Task

```json
{
  "command": "stop_task"
}
```

**Response:**
```json
{
  "type": "task_stopped",
  "success": true,
  "message": "Task stopped successfully"
}
```

---

### 7. Emergency Stop

```json
{
  "command": "emergency_stop"
}
```

**Response:**
```json
{
  "type": "emergency_stop_activated",
  "success": true,
  "message": "All robots stopped"
}
```

---

### 8. Configure Environment

```json
{
  "command": "configure_environment",
  "parameters": {
    "obstacle_density": "medium",
    "obstacle_types": ["box", "cylinder", "wall"],
    "spawn_obstacles": true,
    "clear_existing": true
  }
}
```

**Obstacle Density:**
- `low`: 5 obstacles
- `medium`: 12 obstacles
- `high`: 20 obstacles

**Response:**
```json
{
  "type": "environment_configured",
  "success": true,
  "message": "Spawned 12 obstacles",
  "obstacle_count": 12
}
```

---

### 9. Manual Leader Control (for manual mode)

```json
{
  "command": "control_leader",
  "parameters": {
    "linear_velocity": 0.2,
    "angular_velocity": 0.0
  }
}
```

**Parameters:**
- `linear_velocity` (float): m/s (-0.26 to 0.26)
- `angular_velocity` (float): rad/s (-1.82 to 1.82)

**Response:**
```json
{
  "type": "control_acknowledged",
  "success": true
}
```

---

### 10. Get Status

```json
{
  "command": "get_status",
  "query_type": "all"
}
```

**Query Types:**
- `all`: Complete system status
- `robots`: Robot status only
- `task`: Current task status
- `environment`: Environment/obstacle info

**Response:**
```json
{
  "type": "status_update",
  "timestamp": "2026-01-05T10:30:15Z",
  "task": {
    "task_id": "uuid",
    "task_type": "follow_leader",
    "status": "running",
    "progress": 0.65
  },
  "robots": [
    {
      "id": "robot_1",
      "role": "leader",
      "position": {"x": 3.2, "y": 4.1, "z": 0.0},
      "orientation": {"theta": 1.2},
      "velocity": {"linear": 0.15, "angular": 0.05},
      "status": "active",
      "threat_level": 0.1,
      "battery": 85.3
    },
    {
      "id": "robot_2",
      "role": "follower",
      "position": {"x": 2.5, "y": 4.0, "z": 0.0},
      "orientation": {"theta": 1.15},
      "velocity": {"linear": 0.14, "angular": 0.03},
      "status": "active",
      "threat_level": 0.0,
      "battery": 87.1
    }
  ],
  "obstacles_detected": 12,
  "collisions": 0
}
```

---

## Messages from Backend to Frontend

### 1. Status Update (Periodic)

Sent every 100ms (10 Hz) during active tasks

```json
{
  "type": "status_update",
  "timestamp": "2026-01-05T10:30:00Z",
  "task": {
    "task_id": "uuid",
    "task_type": "follow_leader",
    "status": "running",
    "progress": 0.42
  },
  "robots": [
    {
      "id": "robot_1",
      "role": "leader",
      "position": {"x": 2.5, "y": 3.0, "z": 0.0},
      "velocity": {"linear": 0.2, "angular": 0.0},
      "status": "active",
      "threat_level": 0.0
    }
  ]
}
```

---

### 2. Task Complete

```json
{
  "type": "task_complete",
  "task_id": "uuid",
  "task_type": "follow_leader",
  "success": true,
  "completion_time": 125.3,
  "metrics": {
    "total_distance": 45.2,
    "average_velocity": 0.18,
    "obstacle_avoidances": 8,
    "formation_breaks": 2
  }
}
```

---

### 3. Error Notification

```json
{
  "type": "error",
  "severity": "warning",
  "message": "Robot robot_3 is stuck",
  "error_code": "ROBOT_STUCK",
  "robot_id": "robot_3",
  "timestamp": "2026-01-05T10:30:00Z"
}
```

**Error Severity:**
- `info`: Informational
- `warning`: Warning, system continues
- `error`: Error, task may fail
- `critical`: Critical error, task stopped

---

### 4. Heartbeat

```json
{
  "type": "heartbeat",
  "timestamp": "2026-01-05T10:30:00Z",
  "status": "ok"
}
```

---

## Frontend Implementation Examples

### JavaScript/TypeScript

```javascript
class SwarmController {
  constructor(wsUrl) {
    this.ws = new WebSocket(wsUrl);
    this.setupHandlers();
  }

  setupHandlers() {
    this.ws.onopen = () => {
      console.log('Connected to swarm system');
    };

    this.ws.onmessage = (event) => {
      const message = JSON.parse(event.data);
      this.handleMessage(message);
    };

    this.ws.onerror = (error) => {
      console.error('WebSocket error:', error);
    };

    this.ws.onclose = () => {
      console.log('Disconnected, reconnecting...');
      setTimeout(() => this.reconnect(), 5000);
    };
  }

  handleMessage(message) {
    switch (message.type) {
      case 'status_update':
        this.updateRobotPositions(message.robots);
        this.updateProgress(message.task.progress);
        break;
      case 'task_complete':
        this.onTaskComplete(message);
        break;
      case 'error':
        this.showError(message.message);
        break;
    }
  }

  spawnRobots(count, pattern = 'grid') {
    const command = {
      command: 'spawn_robots',
      parameters: {
        robot_count: count,
        spawn_pattern: pattern,
        center: {x: 0.0, y: 0.0},
        spacing: 1.0
      },
      timestamp: new Date().toISOString()
    };
    this.ws.send(JSON.stringify(command));
  }

  startFollowLeader(waypoints) {
    const command = {
      command: 'start_task',
      task_type: 'follow_leader',
      parameters: {
        robot_count: 5,
        leader_mode: 'waypoint',
        waypoints: waypoints,
        following_distance: 0.7,
        max_velocity: 0.22
      },
      timestamp: new Date().toISOString()
    };
    this.ws.send(JSON.stringify(command));
  }

  startFormation(formationType, robotCount) {
    const command = {
      command: 'start_task',
      task_type: 'formation',
      parameters: {
        robot_count: robotCount,
        formation_type: formationType,
        formation_spacing: 1.2,
        movement_mode: 'static'
      },
      timestamp: new Date().toISOString()
    };
    this.ws.send(JSON.stringify(command));
  }

  startTransport(objectMass, targetLocation) {
    const command = {
      command: 'start_task',
      task_type: 'transport',
      parameters: {
        robot_count: 4,
        object_id: 'box_1',
        object_mass: objectMass,
        target_location: targetLocation,
        detection_method: 'position'
      },
      timestamp: new Date().toISOString()
    };
    this.ws.send(JSON.stringify(command));
  }

  stopTask() {
    const command = {
      command: 'stop_task',
      timestamp: new Date().toISOString()
    };
    this.ws.send(JSON.stringify(command));
  }

  emergencyStop() {
    const command = {
      command: 'emergency_stop',
      timestamp: new Date().toISOString()
    };
    this.ws.send(JSON.stringify(command));
  }
}

// Usage
const swarm = new SwarmController('ws://localhost:8080/swarm');

// Spawn 5 robots in grid pattern
swarm.spawnRobots(5, 'grid');

// Start follow-the-leader with waypoints
const waypoints = [
  {x: 2.0, y: 2.0, theta: 0.0},
  {x: 4.0, y: 2.0, theta: 1.57},
  {x: 4.0, y: 4.0, theta: 3.14},
  {x: 2.0, y: 4.0, theta: 4.71}
];
swarm.startFollowLeader(waypoints);
```

---

### React Component Example

```jsx
import React, { useState, useEffect } from 'react';

function SwarmControl() {
  const [ws, setWs] = useState(null);
  const [robots, setRobots] = useState([]);
  const [taskStatus, setTaskStatus] = useState(null);
  const [robotCount, setRobotCount] = useState(5);
  const [formationType, setFormationType] = useState('triangle');

  useEffect(() => {
    const websocket = new WebSocket('ws://localhost:8080/swarm');

    websocket.onmessage = (event) => {
      const message = JSON.parse(event.data);

      if (message.type === 'status_update') {
        setRobots(message.robots);
        setTaskStatus(message.task);
      }
    };

    setWs(websocket);

    return () => websocket.close();
  }, []);

  const handleSpawnRobots = () => {
    const command = {
      command: 'spawn_robots',
      parameters: {
        robot_count: robotCount,
        spawn_pattern: 'grid',
        center: {x: 0.0, y: 0.0},
        spacing: 1.0
      }
    };
    ws.send(JSON.stringify(command));
  };

  const handleStartFormation = () => {
    const command = {
      command: 'start_task',
      task_type: 'formation',
      parameters: {
        robot_count: robotCount,
        formation_type: formationType,
        formation_spacing: 1.2,
        movement_mode: 'static'
      }
    };
    ws.send(JSON.stringify(command));
  };

  const handleStopTask = () => {
    const command = { command: 'stop_task' };
    ws.send(JSON.stringify(command));
  };

  return (
    <div className="swarm-control">
      <h2>Swarm Control Panel</h2>

      <div className="controls">
        <label>
          Number of Robots:
          <input
            type="number"
            value={robotCount}
            onChange={(e) => setRobotCount(parseInt(e.target.value))}
            min="1"
            max="20"
          />
        </label>

        <button onClick={handleSpawnRobots}>Spawn Robots</button>

        <label>
          Formation Type:
          <select
            value={formationType}
            onChange={(e) => setFormationType(e.target.value)}
          >
            <option value="line">Line</option>
            <option value="triangle">Triangle</option>
            <option value="circle">Circle</option>
            <option value="square">Square</option>
            <option value="v">V-Formation</option>
          </select>
        </label>

        <button onClick={handleStartFormation}>Start Formation</button>
        <button onClick={handleStopTask} className="stop-btn">Stop Task</button>
      </div>

      <div className="status">
        <h3>Task Status</h3>
        {taskStatus && (
          <div>
            <p>Type: {taskStatus.task_type}</p>
            <p>Status: {taskStatus.status}</p>
            <p>Progress: {(taskStatus.progress * 100).toFixed(1)}%</p>
          </div>
        )}
      </div>

      <div className="robots">
        <h3>Active Robots: {robots.length}</h3>
        {robots.map(robot => (
          <div key={robot.id} className="robot-card">
            <h4>{robot.id}</h4>
            <p>Role: {robot.role}</p>
            <p>Position: ({robot.position.x.toFixed(2)}, {robot.position.y.toFixed(2)})</p>
            <p>Status: {robot.status}</p>
            <p>Threat: {(robot.threat_level * 100).toFixed(0)}%</p>
          </div>
        ))}
      </div>
    </div>
  );
}

export default SwarmControl;
```

---

## Error Codes

| Code | Description |
|------|-------------|
| `ROBOT_STUCK` | Robot is stuck and cannot progress |
| `TASK_FAILED` | Task execution failed |
| `SPAWN_FAILED` | Robot spawning failed |
| `COLLISION_DETECTED` | Collision detected (should not happen!) |
| `INVALID_PARAMETERS` | Invalid command parameters |
| `TIMEOUT` | Operation timeout |
| `CONNECTION_LOST` | Connection to robot lost |

---

## Best Practices

1. **Always wait for responses**: Commands are asynchronous
2. **Handle reconnection**: Implement automatic reconnection with backoff
3. **Update UI with status messages**: Use periodic status updates for smooth visualization
4. **Validate inputs**: Check robot count, formation types, etc. before sending
5. **Emergency stop button**: Always visible and accessible
6. **Show threat levels**: Visualize obstacle proximity for each robot
7. **Task progress**: Display progress bars for long-running tasks

---

## Rate Limits

- **Commands**: Max 10 commands/second
- **Status updates**: Automatic at 10 Hz
- **Heartbeat**: Every 10 seconds

---

## Testing

Use `wscat` for command-line testing:

```bash
npm install -g wscat
wscat -c ws://localhost:8080/swarm

# Then send commands:
> {"command": "get_status", "query_type": "all"}
```

---

## Support

For issues or questions, check:
- ROS logs: `~/.ros/log/latest/swarm_log.txt`
- WebSocket connection status
- Robot status in `/fleet/robot_list` topic
