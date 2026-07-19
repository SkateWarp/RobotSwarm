# Quick Start Guide

> **Historical local-development guide (2026-01-05).** The commands below
> describe the original standalone ROS prototype and are not production
> deployment or acceptance instructions. They include obsolete model, robot
> count, and direct-WebSocket assumptions. For the current TurtleBot3 Burger
> worker image and protected deployment flow, use
> [docs/gpu-worker-deployment.md](docs/gpu-worker-deployment.md); for current
> architecture and remaining acceptance work, use
> [IMPLEMENTATION_STATUS.md](IMPLEMENTATION_STATUS.md).

**Multi-Robot Swarm System - ROS Noetic + Gazebo + TurtleBot3**

## Installation

### 1. Install Dependencies

```bash
# System dependencies
sudo apt update
sudo apt install -y \
    ros-noetic-turtlebot3-* \
    ros-noetic-gazebo-ros-pkgs \
    ros-noetic-gazebo-ros-control \
    ros-noetic-tf2-tools \
    python3-pip

# Python dependencies
pip3 install -r requirements.txt
```

### 2. Build the Package

```bash
cd ~/RobotSwarm/swarm_ws

# Build ROS messages and services
catkin_make

# Source the workspace
source devel/setup.bash

# Add to .bashrc for persistence
echo "source ~/RobotSwarm/swarm_ws/devel/setup.bash" >> ~/.bashrc
```

### 3. Set TurtleBot3 Model

```bash
# Add to .bashrc
echo "export TURTLEBOT3_MODEL=waffle" >> ~/.bashrc
source ~/.bashrc
```

## Testing Individual Components

### Test 1: Configuration System

```bash
# Test configuration loader
rosrun robot_swarm_bridge config_loader.py
```

**Expected Output:**
```
Configuration loaded from: .../swarm_config.yaml
Fleet max robots: 20
Safety min distance: 0.2
...
```

### Test 2: Fleet Manager

```bash
# Terminal 1: Start Gazebo
roslaunch gazebo_ros empty_world.launch

# Terminal 2: Start Fleet Manager with auto-spawn
rosrun robot_swarm_bridge fleet_manager.py _auto_spawn:=true _robot_count:=3

# Terminal 3: Verify robots spawned
rostopic echo /fleet/robot_list

# Terminal 4: Check robot positions
rostopic echo /gazebo/model_states
```

**Expected Output:**
```
data: "robot_1,robot_2,robot_3"
```

### Test 3: Obstacle Avoidance

```bash
# Spawn a single robot first
rosservice call /fleet/spawn_robots "robot_count: 1
spawn_pattern: 'grid'
center: {x: 0.0, y: 0.0, z: 0.0}
spacing: 1.0"

# Start obstacle avoidance node
rosrun robot_swarm_bridge obstacle_avoidance.py _robot_id:=robot_1

# In another terminal, monitor threat level
rostopic echo /robot_1/avoidance/threat_level

# Test by sending velocity command
rostopic pub -r 10 /robot_1/cmd_vel geometry_msgs/Twist "linear:
  x: 0.2
angular:
  z: 0.0"

# Place obstacles in Gazebo and watch threat level change
```

### Test 4: ROS Services

```bash
# Spawn robots
rosservice call /fleet/spawn_robots "robot_count: 5
spawn_pattern: 'circle'
center: {x: 0.0, y: 0.0, z: 0.0}
spacing: 1.5"

# Get fleet status
rosservice call /fleet/get_status "{}"

# Delete specific robots
rosservice call /fleet/delete_robots "robot_ids: ['robot_3', 'robot_4']"

# Delete all robots
rosservice call /fleet/delete_robots "robot_ids: []"
```

## Configuration

### Edit Configuration File

```bash
nano swarm_ws/src/robot_swarm_bridge/config/swarm_config.yaml
```

**Key Parameters to Adjust:**

```yaml
# Number of robots
fleet:
  default_robot_count: 5  # Change this

# Safety distances (CRITICAL)
safety:
  absolute_min_distance: 0.20  # Minimum safe distance
  max_linear_velocity: 0.26    # Max speed

# Arena size
arena:
  size:
    x: 10.0
    y: 10.0

# WebSocket port
websocket:
  port: 8080
```

## Troubleshooting

### Issue: "Cannot find package robot_swarm_bridge"

**Solution:**
```bash
cd ~/RobotSwarm/swarm_ws
catkin_make
source devel/setup.bash
```

### Issue: "No module named 'robot_swarm_bridge'"

**Solution:**
```bash
# Check if setup.py is configured
cd swarm_ws/src/robot_swarm_bridge
cat setup.py

# Rebuild
cd ~/RobotSwarm/swarm_ws
catkin_make --force-cmake
source devel/setup.bash
```

### Issue: Gazebo doesn't show robots

**Solution:**
```bash
# Set TurtleBot3 model
export TURTLEBOT3_MODEL=waffle

# Check if robot model exists
rospack find turtlebot3_description

# Verify spawn service
rosservice info /gazebo/spawn_sdf_model
```

### Issue: "Service /fleet/spawn_robots not available"

**Solution:**
```bash
# Check if fleet_manager is running
rosnode list | grep fleet

# Start fleet manager if not running
rosrun robot_swarm_bridge fleet_manager.py
```

### Issue: Configuration not loading

**Solution:**
```bash
# Check config file exists
ls swarm_ws/src/robot_swarm_bridge/config/swarm_config.yaml

# Check for YAML syntax errors
python3 -c "import yaml; yaml.safe_load(open('swarm_ws/src/robot_swarm_bridge/config/swarm_config.yaml'))"
```

## Viewing Topics and Services

```bash
# List all robot_swarm_bridge topics
rostopic list | grep robot

# List all fleet services
rosservice list | grep fleet

# View custom messages
rosmsg list | grep robot_swarm_bridge

# View custom services
rossrv list | grep robot_swarm_bridge

# Check message definition
rosmsg show robot_swarm_bridge/SwarmStatus

# Check service definition
rossrv show robot_swarm_bridge/SpawnRobots
```

## Visualization with RViz

```bash
# Start RViz
rosrun rviz rviz

# Add displays:
# 1. RobotModel - Shows robot
# 2. LaserScan - Topic: /robot_1/scan
# 3. Odometry - Topic: /robot_1/odom
# 4. MarkerArray - Topic: /robot_1/avoidance/markers (safety zones)
# 5. TF - Show robot frames
```

## Next Steps

1. **Implement Task Orchestrator** - WebSocket server for frontend control
2. **Create Follow-the-Leader** - First behavior to test
3. **Create Launch Files** - Simplify startup
4. **Build Frontend Interface** - Use FRONTEND_API.md for reference
5. **Test with Multiple Robots** - Verify scaling and safety

## Useful Commands

```bash
# Kill all ROS nodes
rosnode kill -a

# Clean build
cd ~/RobotSwarm/swarm_ws
rm -rf build/ devel/
catkin_make

# View ROS logs
tail -f ~/.ros/log/latest/rosout.log

# Monitor CPU usage
htop

# Check ROS network
rostopic hz /robot_1/odom  # Check publish rate
rostopic bw /robot_1/scan  # Check bandwidth
```

## Performance Tips

1. **Reduce LaserScan rate** if system is slow:
   ```yaml
   sensors:
     lidar:
       update_rate: 5.0  # Default is 10.0
   ```

2. **Reduce visualization** in RViz:
   - Disable safety zone markers
   - Lower marker array update rate

3. **Adjust control loop rate**:
   ```yaml
   control:
     main_loop_rate: 20.0  # Default is 30.0
   ```

## Safety Checklist

Before running with multiple robots:

- ✅ Verify `safety.absolute_min_distance` >= 0.20m
- ✅ Check `safety.max_linear_velocity` <= 0.26 m/s
- ✅ Test obstacle avoidance with single robot first
- ✅ Monitor `/robot_N/avoidance/threat_level` topics
- ✅ Keep emergency stop ready (Ctrl+C in terminal)

## Support

- **Architecture:** See `ARCHITECTURE.md`
- **Frontend API:** See `FRONTEND_API.md`
- **Implementation Status:** See `IMPLEMENTATION_STATUS.md`
- **ROS Logs:** `~/.ros/log/latest/`
- **Config File:** `swarm_ws/src/robot_swarm_bridge/config/swarm_config.yaml`
