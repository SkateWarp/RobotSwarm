# Multi-Robot Swarm System Architecture
**Target Platform:** ROS Noetic + Gazebo + TurtleBot3
**Language:** Python (with C++ optional modules)
**Communication:** WebSocket (SignalR)
**Date:** 2026-01-05

---

## System Overview

This system enables coordinated multi-robot behaviors with three primary modes:
1. **Follow-the-Leader** (Snake-like formation)
2. **Geometric Formations** (Triangle, Line, Circle, Square)
3. **Collaborative Transportation** (Multi-robot object pushing)

### Key Features
- Scalable robot count (2-20+ robots)
- Strict no-push obstacle avoidance
- Random obstacle generation
- WebSocket-based control interface
- Dynamic task assignment
- Comprehensive parameter configuration

---

## Architecture Diagram

```
┌─────────────────────────────────────────────────────────────┐
│                    WebSocket Controller                      │
│  (Task Assignment, Goal Setting, Configuration, Monitoring)  │
└────────────────────────┬────────────────────────────────────┘
                         │ WebSocket (SignalR)
                         │
┌────────────────────────▼────────────────────────────────────┐
│                 Task Orchestrator Node                       │
│  - Receives commands via WebSocket                          │
│  - Validates parameters                                     │
│  - Dispatches to appropriate behavior controller           │
│  - Monitors execution status                                │
└────────────────────────┬────────────────────────────────────┘
                         │ ROS Services/Topics
         ┌───────────────┼───────────────┐
         │               │               │
┌────────▼──────┐ ┌─────▼──────┐ ┌──────▼────────┐
│ Follow Leader │ │ Formation  │ │ Collaborative │
│  Controller   │ │ Controller │ │  Transport    │
└────────┬──────┘ └─────┬──────┘ └──────┬────────┘
         │               │               │
         └───────────────┼───────────────┘
                         │
         ┌───────────────▼───────────────┐
         │                               │
┌────────▼──────────┐       ┌────────────▼─────────┐
│ Obstacle Avoidance│       │  Robot Fleet Manager  │
│     Module        │       │  - Spawning           │
│ - LiDAR processing│       │  - Namespace mgmt     │
│ - Repulsion field │       │  - Status tracking    │
│ - Path planning   │       │  - Odometry fusion    │
└────────┬──────────┘       └────────────┬─────────┘
         │                               │
         └───────────────┬───────────────┘
                         │
         ┌───────────────▼───────────────┐
         │                               │
┌────────▼──────────┐       ┌────────────▼─────────┐
│ Gazebo Simulation │       │  Random Obstacle     │
│  - TurtleBot3     │       │  Generator           │
│  - Physics engine │       │  - Dynamic spawning  │
│  - Sensor models  │       │  - Variety of shapes │
└───────────────────┘       └──────────────────────┘
```

---

## Module Specifications

### 1. Task Orchestrator Node
**File:** `swarm_ws/src/robot_swarm_bridge/scripts/task_orchestrator.py`

**Responsibilities:**
- WebSocket server endpoint
- Command validation and parsing
- Behavior mode switching
- Status aggregation and reporting
- Emergency stop handling

**Published Topics:**
- `/swarm/current_task` (String) - Active task type
- `/swarm/status` (SwarmStatus) - Overall system status
- `/swarm/robot_count` (Int32) - Number of active robots

**Subscribed Topics:**
- `/swarm/task_complete` (Bool) - Task completion signal

**Services Provided:**
- `/swarm/start_task` (StartTask) - Initiate new task
- `/swarm/stop_task` (Empty) - Halt current task
- `/swarm/configure` (ConfigureSwarm) - Update parameters

**WebSocket Schema:**
```json
{
  "command": "start_task",
  "task_type": "follow_leader | formation | transport",
  "parameters": {
    "robot_count": 5,
    "max_velocity": 0.22,
    "formation_type": "triangle | line | circle | square",
    "leader_waypoints": [[x1, y1], [x2, y2], ...],
    "obstacle_density": "low | medium | high",
    "object_mass": 2.5
  }
}
```

---

### 2. Follow-the-Leader Controller
**File:** `swarm_ws/src/robot_swarm_bridge/scripts/behaviors/follow_leader.py`

**Algorithm:**
1. Leader follows predefined waypoint path or manual control
2. Each follower tracks the robot ahead with time delay
3. Maintains safe following distance (0.5-1.0m configurable)
4. Obstacle avoidance has priority over following

**Key Parameters:**
- `following_distance`: 0.7m (default)
- `time_delay`: 0.5s between robots (creates snake effect)
- `leader_mode`: 'waypoint' | 'manual' | 'exploration' | 'random'
- `path_type`: 'circular' | 'square' | 'figure8' | 'custom'

**Published Topics:**
- `/robot_N/cmd_vel` (Twist) - Velocity commands per robot
- `/swarm/leader/path` (Path) - Current leader trajectory
- `/swarm/formation_state` (FollowState[]) - Each robot's state

**Subscribed Topics:**
- `/robot_N/odom` (Odometry) - Position feedback
- `/robot_N/scan` (LaserScan) - Obstacle detection
- `/swarm/leader/goal` (PoseStamped) - Leader waypoint

**States:**
- INITIALIZING: Robots moving to starting positions
- FOLLOWING: Normal snake-like following
- AVOIDING: Temporary avoidance maneuver
- RECOVERING: Rejoining formation after avoidance
- STOPPED: Task halted

---

### 3. Formation Controller
**File:** `swarm_ws/src/robot_swarm_bridge/scripts/behaviors/formation_control.py`

**Supported Formations:**

| Formation | Description | Robot Positions |
|-----------|-------------|-----------------|
| **Line** | Straight horizontal line | Equal spacing along X-axis |
| **Triangle** | Equilateral triangle | Leader at apex, others at base |
| **Circle** | Robots evenly distributed on circle | Radius auto-calculated from count |
| **Square** | Grid arrangement | Closest square number used |
| **V-Formation** | V-shape for aerodynamics | Configurable angle |
| **Diamond** | 4-robot diamond | Fixed pattern |

**Modes:**
- **Static Formation:** Reach formation and hold position
- **Moving Formation:** Maintain relative positions while moving as a unit
- **Adaptive Formation:** Deform around obstacles, reform after

**Obstacle Handling:**
1. **Deformation Mode:**
   - Robots detect obstacle in path
   - Formation stretches/compresses to navigate
   - Returns to original shape post-obstacle

2. **Formation Shift:**
   - Entire formation moves laterally to avoid
   - Maintains shape throughout

3. **Open Space Seeking:**
   - If path too narrow, formation searches for wider area
   - Uses frontier exploration algorithm

**Key Parameters:**
- `formation_type`: 'line' | 'triangle' | 'circle' | 'square' | 'v' | 'diamond'
- `formation_spacing`: 1.0m (default distance between robots)
- `movement_mode`: 'static' | 'moving' | 'adaptive'
- `formation_velocity`: 0.15 m/s (when moving)
- `deformation_enabled`: true/false

**Published Topics:**
- `/robot_N/cmd_vel` (Twist)
- `/swarm/formation/shape` (PolygonStamped) - Current formation outline
- `/swarm/formation/centroid` (PoseStamped) - Formation center

---

### 4. Collaborative Transportation Controller
**File:** `swarm_ws/src/robot_swarm_bridge/scripts/behaviors/collaborative_transport.py`

**Approach:** Inspired by GRF (Gibbs Random Fields) from reference repo

**Algorithm:**
1. **Object Detection:**
   - Robots locate object via AprilTag or known position
   - Estimate object mass and friction coefficient

2. **Position Assignment:**
   - GRF optimization determines optimal robot positions around object
   - Typically 2-4 robots pushing from different sides
   - Balances forces for straight-line motion

3. **Coordinated Pushing:**
   - Leader robot at front guides direction
   - Side robots apply balanced lateral forces
   - Rear robot (if present) provides main thrust
   - Real-time force feedback adjusts individual efforts

4. **Path Planning:**
   - A* path from object current position to goal
   - Account for object size when checking clearance
   - Re-plan if path blocked

5. **Obstacle Avoidance During Transport:**
   - Formation stops if obstacle detected ahead
   - Re-plan path or navigate around
   - Never push through obstacles

**Object Properties:**
- **Mass Range:** 0.5kg - 5.0kg (configurable)
- **Spawning:**
  - Pre-defined locations
  - Random within arena boundaries
  - Timed appearance (e.g., every 60 seconds)
- **Appearance:**
  - Boxes (with AprilTags for detection)
  - Cylinders
  - Custom meshes
- **Goal Location:**
  - Designated drop-off zone
  - Random goal positions
  - Sequential delivery points

**Key Parameters:**
- `object_mass`: 2.0kg (default)
- `max_robots_per_object`: 4
- `push_force_per_robot`: 0.5N
- `object_spawn_mode`: 'static' | 'random' | 'timed'
- `detection_method`: 'apriltag' | 'position' | 'visual'

**Published Topics:**
- `/robot_N/cmd_vel` (Twist)
- `/swarm/transport/object_pose` (PoseStamped)
- `/swarm/transport/assignments` (RobotAssignment[])

**Subscribed Topics:**
- `/robot_N/odom` (Odometry)
- `/robot_N/scan` (LaserScan)
- `/object/pose` (PoseStamped) - From object tracking

---

### 5. Obstacle Avoidance Module
**File:** `swarm_ws/src/robot_swarm_bridge/scripts/core/obstacle_avoidance.py`

**Critical Requirement:** ZERO PHYSICAL CONTACT

**Implementation:**
1. **Multi-Layer Safety Zones:**
   - **Warning Zone:** 1.0m - Slow down, start planning alternate path
   - **Danger Zone:** 0.5m - Stronger repulsion, significant velocity reduction
   - **Critical Zone:** 0.3m - Emergency stop, reverse if necessary
   - **Absolute Minimum:** 0.2m - Never allow closer approach

2. **Repulsive Potential Field:**
```python
repulsion_force = K_repulsion * (1/distance - 1/safe_distance) * 1/(distance^2)
# K_repulsion = 1.5 (tunable)
# Exponential increase as distance decreases
```

3. **Velocity Scaling:**
```python
velocity_scale = min(1.0, (distance - min_distance) / (safe_distance - min_distance))
# Linear reduction from safe_distance to min_distance
# Zero velocity at min_distance
```

4. **LiDAR Processing:**
- 360° LaserScan divided into 36 sectors (10° each)
- Minimum distance per sector tracked
- Weighted average for directional threat assessment
- 10Hz update rate

5. **Dynamic Window Approach (DWA):**
- Samples velocity space
- Simulates trajectories forward 1.0 second
- Scores based on:
  - Distance to goal
  - Clearance from obstacles
  - Velocity smoothness
  - Formation maintenance (if applicable)
- Selects optimal velocity command

**Parameters:**
- `min_obstacle_distance`: 0.3m (absolute minimum)
- `safe_distance`: 0.8m (comfortable clearance)
- `repulsion_gain`: 1.5
- `velocity_reduction_start`: 1.0m
- `emergency_stop_distance`: 0.25m

**Published Topics:**
- `/robot_N/avoidance/threat_level` (Float32) - 0.0-1.0 danger score
- `/robot_N/avoidance/repulsion_vector` (Vector3) - Avoidance direction

---

### 6. Robot Fleet Manager
**File:** `swarm_ws/src/robot_swarm_bridge/scripts/core/fleet_manager.py`

**Responsibilities:**
- Spawn N robots in Gazebo at non-overlapping positions
- Assign unique namespaces (`/robot_1`, `/robot_2`, ...)
- Track robot status (active, idle, error, stuck)
- Handle robot failures and reassignment
- Coordinate multi-robot TF trees

**Robot Spawning:**
```python
spawn_positions = generate_grid_positions(
    count=robot_count,
    spacing=1.0,  # meters between robots
    arena_center=(0, 0),
    pattern='grid' | 'circle' | 'line'
)
```

**Services Provided:**
- `/fleet/spawn_robots` (SpawnRobots) - Add robots to simulation
- `/fleet/delete_robots` (DeleteRobots) - Remove robots
- `/fleet/get_status` (GetFleetStatus) - Query robot states

**Published Topics:**
- `/fleet/robot_list` (String[]) - Active robot namespaces
- `/fleet/status` (FleetStatus) - Aggregate fleet information

---

### 7. Random Obstacle Generator
**File:** `swarm_ws/src/robot_swarm_bridge/scripts/environment/obstacle_generator.py`

**Features:**
- Dynamically spawns obstacles in Gazebo
- Configurable density levels
- Variety of shapes and sizes
- Ensures obstacles don't block critical areas

**Obstacle Types:**
| Type | Dimensions | Use Case |
|------|------------|----------|
| Box | 0.2-0.5m sides | General obstacles |
| Cylinder | 0.15-0.3m radius, 0.5m height | Pillars, cones |
| Wall | 0.1m x 1.0-3.0m | Path blockers |
| Irregular | Custom meshes | Realistic clutter |

**Density Levels:**
- **Low:** 3-5 obstacles in 10x10m arena
- **Medium:** 8-12 obstacles
- **High:** 15-20 obstacles

**Spawning Strategy:**
1. Divide arena into grid cells
2. Randomly select cells based on density
3. Ensure minimum distance from:
   - Robot start positions (2.0m)
   - Arena boundaries (0.5m)
   - Other obstacles (0.3m minimum spacing)
4. Spawn via Gazebo service

**Parameters:**
- `density`: 'low' | 'medium' | 'high'
- `obstacle_types`: ['box', 'cylinder', 'wall']
- `respawn_interval`: 0 (static) or seconds for dynamic
- `arena_size`: (10.0, 10.0) meters

**Services Provided:**
- `/environment/spawn_obstacles` (SpawnObstacles)
- `/environment/clear_obstacles` (Empty)
- `/environment/randomize` (Empty)

---

## WebSocket Control Schema

### Connection
- **URL:** `ws://localhost:8080/swarm` (configurable)
- **Protocol:** SignalR (for .NET backend compatibility)
- **Authentication:** Token-based (optional)

### Command Messages

#### 1. Start Task
```json
{
  "command": "start_task",
  "task_id": "uuid-string",
  "task_type": "follow_leader",
  "parameters": {
    "robot_count": 5,
    "leader_mode": "waypoint",
    "waypoints": [
      {"x": 2.0, "y": 2.0, "theta": 0.0},
      {"x": 4.0, "y": 2.0, "theta": 1.57},
      {"x": 4.0, "y": 4.0, "theta": 3.14}
    ],
    "following_distance": 0.7,
    "max_velocity": 0.22
  }
}
```

#### 2. Start Formation
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
    "obstacle_handling": "adaptive"
  }
}
```

#### 3. Start Collaborative Transport
```json
{
  "command": "start_task",
  "task_type": "transport",
  "parameters": {
    "robot_count": 4,
    "object_id": "box_1",
    "object_mass": 2.5,
    "target_location": {"x": 8.0, "y": 8.0},
    "coordination_algorithm": "grf"
  }
}
```

#### 4. Configure Environment
```json
{
  "command": "configure_environment",
  "parameters": {
    "obstacle_density": "medium",
    "arena_size": {"x": 10.0, "y": 10.0},
    "spawn_obstacles": true,
    "obstacle_types": ["box", "cylinder"]
  }
}
```

#### 5. Emergency Stop
```json
{
  "command": "emergency_stop"
}
```

#### 6. Query Status
```json
{
  "command": "get_status",
  "query_type": "all | robots | task | environment"
}
```

### Response Messages

#### Status Update (Periodic)
```json
{
  "type": "status_update",
  "timestamp": "2026-01-05T10:30:00Z",
  "task": {
    "task_id": "uuid",
    "task_type": "follow_leader",
    "status": "running | completed | failed",
    "progress": 0.65
  },
  "robots": [
    {
      "id": "robot_1",
      "role": "leader",
      "position": {"x": 3.2, "y": 4.1, "theta": 1.2},
      "velocity": {"linear": 0.15, "angular": 0.05},
      "status": "active | avoiding | stuck | idle",
      "battery": 85.3
    }
  ],
  "obstacles_detected": 12,
  "collisions": 0
}
```

#### Task Complete
```json
{
  "type": "task_complete",
  "task_id": "uuid",
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

## Configuration Files

### Main Config
**File:** `swarm_ws/src/robot_swarm_bridge/config/swarm_config.yaml`

```yaml
# Fleet Configuration
fleet:
  max_robots: 20
  default_robot_count: 5
  robot_model: "turtlebot3_waffle"
  spawn_pattern: "grid"  # grid, circle, line

# Arena Configuration
arena:
  size:
    x: 10.0
    y: 10.0
  boundaries_enabled: true

# WebSocket Configuration
websocket:
  host: "0.0.0.0"
  port: 8080
  signalr_hub: "/hubs/swarm"

# Safety Parameters
safety:
  min_obstacle_distance: 0.3
  safe_obstacle_distance: 0.8
  robot_min_distance: 0.5
  emergency_stop_distance: 0.25
  max_linear_velocity: 0.22
  max_angular_velocity: 2.84

# Obstacle Generation
obstacles:
  default_density: "medium"
  types: ["box", "cylinder", "wall"]
  dynamic_respawn: false
  respawn_interval: 60.0

# Behavior-Specific Configs
behaviors:
  follow_leader:
    following_distance: 0.7
    time_delay: 0.5
    path_type: "waypoint"

  formation:
    default_type: "triangle"
    default_spacing: 1.0
    movement_mode: "static"

  transport:
    default_object_mass: 2.0
    max_robots_per_object: 4
    push_force_per_robot: 0.5

# Logging
logging:
  level: "INFO"  # DEBUG, INFO, WARN, ERROR
  log_file: "/tmp/swarm_log.txt"
  log_ros_topics: true
```

---

## Launch Files

### 1. Main Swarm Launch
**File:** `swarm_ws/src/robot_swarm_bridge/launch/swarm_main.launch`

```xml
<launch>
  <!-- Parameters -->
  <arg name="robot_count" default="5"/>
  <arg name="world_file" default="empty_world"/>
  <arg name="gui" default="true"/>

  <!-- Start Gazebo -->
  <include file="$(find gazebo_ros)/launch/empty_world.launch">
    <arg name="world_name" value="$(find robot_swarm_bridge)/worlds/$(arg world_file).world"/>
    <arg name="paused" value="false"/>
    <arg name="gui" value="$(arg gui)"/>
  </include>

  <!-- Fleet Manager -->
  <node pkg="robot_swarm_bridge" type="fleet_manager.py" name="fleet_manager" output="screen">
    <param name="robot_count" value="$(arg robot_count)"/>
  </node>

  <!-- Task Orchestrator -->
  <node pkg="robot_swarm_bridge" type="task_orchestrator.py" name="task_orchestrator" output="screen"/>

  <!-- Obstacle Avoidance Module -->
  <node pkg="robot_swarm_bridge" type="obstacle_avoidance.py" name="obstacle_avoidance" output="screen"/>

  <!-- Obstacle Generator -->
  <node pkg="robot_swarm_bridge" type="obstacle_generator.py" name="obstacle_generator" output="screen"/>

  <!-- RViz Visualization -->
  <node pkg="rviz" type="rviz" name="rviz" args="-d $(find robot_swarm_bridge)/rviz/swarm.rviz"/>
</launch>
```

### 2. Follow Leader Launch
**File:** `launch/follow_leader.launch`

```xml
<launch>
  <arg name="robot_count" default="5"/>
  <arg name="leader_mode" default="waypoint"/>

  <!-- Include main swarm -->
  <include file="$(find robot_swarm_bridge)/launch/swarm_main.launch">
    <arg name="robot_count" value="$(arg robot_count)"/>
  </include>

  <!-- Follow Leader Controller -->
  <node pkg="robot_swarm_bridge" type="follow_leader.py" name="follow_leader" output="screen">
    <param name="leader_mode" value="$(arg leader_mode)"/>
  </node>
</launch>
```

---

## Message Definitions

### SwarmStatus.msg
```
Header header
string task_type
string status  # initializing, running, paused, completed, failed
uint8 robot_count
uint8 active_robots
float32 progress
RobotStatus[] robot_statuses
```

### RobotStatus.msg
```
string robot_id
string role  # leader, follower, pusher, idle
geometry_msgs/Pose pose
geometry_msgs/Twist velocity
string state  # active, avoiding, stuck, idle
float32 battery_level
```

### FormationShape.msg
```
Header header
string formation_type
geometry_msgs/Point centroid
geometry_msgs/Point[] robot_positions
float32 spacing
```

### TransportTask.msg
```
Header header
string object_id
geometry_msgs/Pose object_pose
geometry_msgs/Pose target_pose
float32 object_mass
string[] assigned_robots
float32 transport_progress
```

---

## Development Phases

### Phase 1: Core Infrastructure (Week 1)
- ✓ Fleet Manager with dynamic spawning
- ✓ Enhanced Obstacle Avoidance Module
- ✓ Task Orchestrator with WebSocket
- ✓ Configuration system
- ✓ Launch files

### Phase 2: Follow-the-Leader (Week 2)
- ✓ Leader waypoint navigation
- ✓ Follower tracking algorithm
- ✓ Snake-like formation maintenance
- ✓ Integration with obstacle avoidance
- ✓ Testing with 3-10 robots

### Phase 3: Geometric Formations (Week 3)
- ✓ Formation shape calculators
- ✓ Static formation controller
- ✓ Moving formation controller
- ✓ Adaptive deformation logic
- ✓ Testing all formation types

### Phase 4: Collaborative Transport (Week 4)
- ✓ GRF-based position optimization
- ✓ Coordinated pushing controller
- ✓ Object detection and tracking
- ✓ Path planning for large objects
- ✓ Testing with various object masses

### Phase 5: Integration & Testing (Week 5)
- ✓ End-to-end scenario testing
- ✓ Performance optimization
- ✓ Documentation completion
- ✓ Demo scenarios
- ✓ Video recordings

---

## Testing Strategy

### Unit Tests
- Obstacle avoidance with static obstacles
- Formation calculation accuracy
- GRF optimization convergence
- WebSocket message parsing

### Integration Tests
- Multi-robot spawning in Gazebo
- Task switching between behaviors
- Obstacle field navigation
- Robot recovery from stuck states

### Scenario Tests
1. **Follow-the-Leader:**
   - 5 robots navigate figure-8 path with 10 random obstacles
   - Success: Complete path without collisions

2. **Formation Control:**
   - 6 robots form hexagon, move 5m forward through obstacle field
   - Success: Maintain formation shape (±10% tolerance)

3. **Collaborative Transport:**
   - 4 robots transport 3kg box 8m to goal
   - Success: Deliver within 5% of target position

### Performance Metrics
- Collision count (target: 0)
- Task completion time
- Formation deviation
- Path efficiency (actual/optimal distance ratio)
- Communication latency

---

## Dependencies

### ROS Packages
```xml
<depend>rospy</depend>
<depend>geometry_msgs</depend>
<depend>nav_msgs</depend>
<depend>sensor_msgs</depend>
<depend>tf2_ros</depend>
<depend>gazebo_ros</depend>
<depend>turtlebot3_description</depend>
<depend>turtlebot3_gazebo</depend>
<depend>move_base</depend>
```

### Python Packages
```
numpy>=1.19.0
scipy>=1.5.0
pyyaml>=5.3.0
signalrcore>=0.9.0
websockets>=10.0
matplotlib>=3.3.0  # for visualization
```

---

## Next Steps

1. Create ROS package structure
2. Implement Fleet Manager
3. Enhance existing obstacle avoidance
4. Develop behavior controllers sequentially
5. Test incrementally with increasing robot counts
6. Document thoroughly throughout

**Ready to begin implementation?**
