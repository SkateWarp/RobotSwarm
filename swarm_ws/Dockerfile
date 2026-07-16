# ROS Noetic Docker image for robot_swarm_bridge
FROM ros:noetic-ros-base

# Set environment variables
ENV DEBIAN_FRONTEND=noninteractive
ENV ROS_WS=/catkin_ws
ENV ROS_PACKAGE=robot_swarm_bridge

# Install dependencies
RUN apt-get update && apt-get install -y \
    python3-pip \
    python3-catkin-tools \
    ros-noetic-actionlib \
    ros-noetic-actionlib-msgs \
    ros-noetic-tf \
    ros-noetic-gazebo-ros \
    ros-noetic-gazebo-ros-pkgs \
    ros-noetic-robot-state-publisher \
    ros-noetic-turtlebot3 \
    ros-noetic-turtlebot3-simulations \
    ros-noetic-nav-msgs \
    ros-noetic-sensor-msgs \
    ros-noetic-geometry-msgs \
    ros-noetic-visualization-msgs \
    ros-noetic-xacro \
    && rm -rf /var/lib/apt/lists/*

# Install Python dependencies
COPY src/robot_swarm_bridge/requirements.txt /tmp/robot_swarm_requirements.txt
RUN pip3 install --no-cache-dir -r /tmp/robot_swarm_requirements.txt

# Create catkin workspace
RUN mkdir -p ${ROS_WS}/src

# Copy source code
COPY src/ ${ROS_WS}/src/

# Build the workspace
WORKDIR ${ROS_WS}
RUN /bin/bash -c "source /opt/ros/noetic/setup.bash && catkin_make"

# Make Python scripts executable
RUN find ${ROS_WS}/src/${ROS_PACKAGE}/scripts -name "*.py" -exec chmod +x {} \;

# Create entrypoint script
RUN echo '#!/bin/bash\n\
set -e\n\
source /opt/ros/noetic/setup.bash\n\
source ${ROS_WS}/devel/setup.bash\n\
export TURTLEBOT3_MODEL=${TURTLEBOT3_MODEL:-burger}\n\
export ROS_MASTER_URI=${ROS_MASTER_URI:-http://localhost:11311}\n\
exec "$@"' > /ros_entrypoint.sh && chmod +x /ros_entrypoint.sh

# Set entrypoint
ENTRYPOINT ["/ros_entrypoint.sh"]

# Default command
CMD ["roslaunch", "robot_swarm_bridge", "swarm_main.launch", "robot_count:=0", "robot_model:=burger", "gui:=false"]
