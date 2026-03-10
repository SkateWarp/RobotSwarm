#!/bin/bash
# Start entire RobotSwarm stack: DB + Backend + ROS/Gazebo
set -e

ROBOT_COUNT=${1:-5}
SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"

echo "Starting PostgreSQL + Backend..."
docker-compose -f "$SCRIPT_DIR/docker-compose.yml" up --build -d

echo "Waiting for backend to be ready..."
until curl -s http://localhost:44336/hubs/robot > /dev/null 2>&1; do
    sleep 2
done
echo "Backend is up."

echo "Starting ROS + Gazebo with $ROBOT_COUNT robots..."
cd "$SCRIPT_DIR/swarm_ws"
source /opt/ros/noetic/setup.bash
catkin_make
source devel/setup.bash
export TURTLEBOT3_MODEL=waffle
roslaunch robot_swarm_bridge swarm_main.launch robot_count:=$ROBOT_COUNT
