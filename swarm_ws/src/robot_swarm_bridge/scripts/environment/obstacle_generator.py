#!/usr/bin/env python3
"""
Random Obstacle Generator for Gazebo
Spawns random obstacles (boxes, cylinders, walls) to create challenging environments
Ensures safe clearance from robot spawn positions and boundaries
"""

import rospy
import numpy as np
import random
from typing import List, Dict, Tuple, Optional
from enum import Enum

# ROS messages and services
from geometry_msgs.msg import Pose, Point, Quaternion
from gazebo_msgs.srv import SpawnModel, DeleteModel, GetWorldProperties
from std_msgs.msg import String
import tf

# Custom services (optional -- only available after catkin_make)
try:
    from robot_swarm_bridge.srv import SpawnObstacles, SpawnObstaclesResponse
    _HAS_CUSTOM_SRV = True
except ImportError:
    _HAS_CUSTOM_SRV = False

# Import configuration
import sys
import os
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from utils.config_loader import config


class ObstacleType(Enum):
    """Available obstacle types"""
    BOX = "box"
    CYLINDER = "cylinder"
    WALL = "wall"


class ObstacleGenerator:
    """
    Random Obstacle Generator

    Features:
    - Spawns obstacles in Gazebo dynamically
    - Multiple obstacle types (boxes, cylinders, walls)
    - Three density levels (low, medium, high)
    - Safe clearance from robots and boundaries
    - Configurable obstacle sizes
    - Clear existing obstacles before spawning new ones

    Density Levels (in 10x10m arena):
    - Low: 5 obstacles
    - Medium: 12 obstacles
    - High: 20 obstacles
    """

    def __init__(self):
        """Initialize obstacle generator"""
        rospy.init_node('obstacle_generator', anonymous=False)

        # Load configuration
        self._load_config()

        # Spawned obstacles tracking
        self.spawned_obstacles: List[str] = []

        # Gazebo services
        rospy.loginfo("Waiting for Gazebo services...")
        rospy.wait_for_service('/gazebo/spawn_sdf_model', timeout=30.0)
        rospy.wait_for_service('/gazebo/delete_model', timeout=30.0)

        self.spawn_service = rospy.ServiceProxy('/gazebo/spawn_sdf_model', SpawnModel)
        self.delete_service = rospy.ServiceProxy('/gazebo/delete_model', DeleteModel)

        # ROS Service (only if custom srv types are available)
        if _HAS_CUSTOM_SRV:
            self.spawn_obstacles_srv = rospy.Service(
                '/environment/spawn_obstacles',
                SpawnObstacles,
                self._handle_spawn_obstacles
            )
        else:
            rospy.logwarn("Custom srv types not built; /environment/spawn_obstacles service unavailable. "
                          "Use topic interface /environment/spawn_obstacles_cmd instead.")

        # Also provide simple topic-based interface
        self.spawn_sub = rospy.Subscriber(
            '/environment/spawn_obstacles_cmd',
            String,
            self._spawn_from_topic,
            queue_size=1
        )
        self.clear_sub = rospy.Subscriber(
            '/environment/clear_obstacles_cmd',
            String,
            self._clear_from_topic,
            queue_size=1
        )

        rospy.loginfo("Obstacle generator initialized")

    def _load_config(self):
        """Load configuration"""
        # Arena
        self.arena_size_x = config.get('arena.size.x', 10.0)
        self.arena_size_y = config.get('arena.size.y', 10.0)
        self.arena_center_x = config.get('arena.center.x', 0.0)
        self.arena_center_y = config.get('arena.center.y', 0.0)

        # Density levels
        self.density_levels = config.get('obstacles.density_levels', {
            'low': 5,
            'medium': 12,
            'high': 20
        })

        # Obstacle sizes
        self.box_size_min = config.get('obstacles.box_size.min', 0.2)
        self.box_size_max = config.get('obstacles.box_size.max', 0.5)
        self.cylinder_radius_min = config.get('obstacles.cylinder_radius.min', 0.15)
        self.cylinder_radius_max = config.get('obstacles.cylinder_radius.max', 0.30)
        self.cylinder_height = config.get('obstacles.cylinder_height', 0.5)
        self.wall_thickness = config.get('obstacles.wall_thickness', 0.1)
        self.wall_length_min = config.get('obstacles.wall_length.min', 1.0)
        self.wall_length_max = config.get('obstacles.wall_length.max', 3.0)

        # Clearances
        self.robot_clearance = config.get('obstacles.min_distance_from_robots', 2.0)
        self.boundary_clearance = config.get('obstacles.min_distance_from_boundaries', 0.5)
        self.obstacle_spacing = config.get('obstacles.min_obstacle_spacing', 0.3)

    def _handle_spawn_obstacles(self, req):
        """Handle spawn obstacles service request"""
        rospy.loginfo(f"Spawning obstacles: density={req.density}, types={req.obstacle_types}")

        # Clear existing if requested
        if req.clear_existing:
            self._clear_all_obstacles()

        # Get count based on density
        density = req.density.lower() if req.density else 'medium'
        count = self.density_levels.get(density, 12)

        # Get obstacle types
        obstacle_types = list(req.obstacle_types) if req.obstacle_types else ['box', 'cylinder']

        # Spawn obstacles
        spawned_count = self._spawn_obstacles(count, obstacle_types)

        return SpawnObstaclesResponse(
            success=spawned_count > 0,
            message=f"Spawned {spawned_count}/{count} obstacles",
            obstacle_count=spawned_count
        )

    def _spawn_from_topic(self, msg: String):
        """Handle spawn request from topic"""
        import json
        try:
            params = json.loads(msg.data)
            density = params.get('density', 'medium')
            types = params.get('types', ['box', 'cylinder'])
            clear = params.get('clear', True)

            if clear:
                self._clear_all_obstacles()

            count = self.density_levels.get(density, 12)
            self._spawn_obstacles(count, types)

        except json.JSONDecodeError:
            # Simple format: just density string
            density = msg.data.lower()
            self._clear_all_obstacles()
            count = self.density_levels.get(density, 12)
            self._spawn_obstacles(count, ['box', 'cylinder'])

    def _clear_from_topic(self, msg: String):
        """Handle clear request from topic"""
        self._clear_all_obstacles()

    def _spawn_obstacles(self, count: int, obstacle_types: List[str]) -> int:
        """
        Spawn random obstacles

        Args:
            count: Number of obstacles to spawn
            obstacle_types: List of types to use

        Returns:
            Number of obstacles successfully spawned
        """
        spawned = 0
        positions_used: List[Tuple[float, float]] = []

        for i in range(count):
            # Choose random type
            obs_type = random.choice(obstacle_types)

            # Generate random position
            position = self._generate_random_position(positions_used)
            if position is None:
                rospy.logwarn(f"Could not find valid position for obstacle {i}")
                continue

            # Generate obstacle
            name = f"obstacle_{len(self.spawned_obstacles)}_{i}"

            success = False
            if obs_type == 'box':
                success = self._spawn_box(name, position)
            elif obs_type == 'cylinder':
                success = self._spawn_cylinder(name, position)
            elif obs_type == 'wall':
                success = self._spawn_wall(name, position)

            if success:
                self.spawned_obstacles.append(name)
                positions_used.append(position)
                spawned += 1

        rospy.loginfo(f"Spawned {spawned} obstacles")
        return spawned

    def _generate_random_position(self, existing_positions: List[Tuple[float, float]]) -> Optional[Tuple[float, float]]:
        """Generate random position avoiding existing obstacles and robots"""
        max_attempts = 100

        # Calculate valid range (inside arena with boundary clearance)
        x_min = self.arena_center_x - self.arena_size_x/2 + self.boundary_clearance
        x_max = self.arena_center_x + self.arena_size_x/2 - self.boundary_clearance
        y_min = self.arena_center_y - self.arena_size_y/2 + self.boundary_clearance
        y_max = self.arena_center_y + self.arena_size_y/2 - self.boundary_clearance

        # Avoid center area where robots spawn
        robot_spawn_radius = self.robot_clearance

        for _ in range(max_attempts):
            x = random.uniform(x_min, x_max)
            y = random.uniform(y_min, y_max)

            # Check distance from center (robot spawn area)
            dist_from_center = np.sqrt(x**2 + y**2)
            if dist_from_center < robot_spawn_radius:
                continue

            # Check distance from existing obstacles
            too_close = False
            for ex, ey in existing_positions:
                dist = np.sqrt((x - ex)**2 + (y - ey)**2)
                if dist < self.obstacle_spacing + 0.5:  # Extra margin
                    too_close = True
                    break

            if not too_close:
                return (x, y)

        return None

    def _spawn_box(self, name: str, position: Tuple[float, float]) -> bool:
        """Spawn a box obstacle"""
        # Random size
        size_x = random.uniform(self.box_size_min, self.box_size_max)
        size_y = random.uniform(self.box_size_min, self.box_size_max)
        size_z = random.uniform(self.box_size_min, self.box_size_max)

        # Random color (gray-ish)
        r = random.uniform(0.3, 0.6)
        g = random.uniform(0.3, 0.6)
        b = random.uniform(0.3, 0.6)

        sdf = f'''<?xml version="1.0"?>
<sdf version="1.6">
  <model name="{name}">
    <static>true</static>
    <link name="link">
      <collision name="collision">
        <geometry>
          <box>
            <size>{size_x} {size_y} {size_z}</size>
          </box>
        </geometry>
      </collision>
      <visual name="visual">
        <geometry>
          <box>
            <size>{size_x} {size_y} {size_z}</size>
          </box>
        </geometry>
        <material>
          <ambient>{r} {g} {b} 1</ambient>
          <diffuse>{r} {g} {b} 1</diffuse>
        </material>
      </visual>
    </link>
  </model>
</sdf>'''

        pose = Pose()
        pose.position.x = position[0]
        pose.position.y = position[1]
        pose.position.z = size_z / 2
        pose.orientation.w = 1.0

        return self._spawn_model(name, sdf, pose)

    def _spawn_cylinder(self, name: str, position: Tuple[float, float]) -> bool:
        """Spawn a cylinder obstacle"""
        radius = random.uniform(self.cylinder_radius_min, self.cylinder_radius_max)
        height = self.cylinder_height

        # Random color
        r = random.uniform(0.3, 0.7)
        g = random.uniform(0.3, 0.7)
        b = random.uniform(0.3, 0.7)

        sdf = f'''<?xml version="1.0"?>
<sdf version="1.6">
  <model name="{name}">
    <static>true</static>
    <link name="link">
      <collision name="collision">
        <geometry>
          <cylinder>
            <radius>{radius}</radius>
            <length>{height}</length>
          </cylinder>
        </geometry>
      </collision>
      <visual name="visual">
        <geometry>
          <cylinder>
            <radius>{radius}</radius>
            <length>{height}</length>
          </cylinder>
        </geometry>
        <material>
          <ambient>{r} {g} {b} 1</ambient>
          <diffuse>{r} {g} {b} 1</diffuse>
        </material>
      </visual>
    </link>
  </model>
</sdf>'''

        pose = Pose()
        pose.position.x = position[0]
        pose.position.y = position[1]
        pose.position.z = height / 2
        pose.orientation.w = 1.0

        return self._spawn_model(name, sdf, pose)

    def _spawn_wall(self, name: str, position: Tuple[float, float]) -> bool:
        """Spawn a wall obstacle"""
        length = random.uniform(self.wall_length_min, self.wall_length_max)
        height = 0.5
        thickness = self.wall_thickness

        # Random rotation
        yaw = random.uniform(0, 2 * np.pi)
        q = tf.transformations.quaternion_from_euler(0, 0, yaw)

        # Random color
        r = random.uniform(0.4, 0.6)
        g = random.uniform(0.4, 0.6)
        b = random.uniform(0.4, 0.6)

        sdf = f'''<?xml version="1.0"?>
<sdf version="1.6">
  <model name="{name}">
    <static>true</static>
    <link name="link">
      <collision name="collision">
        <geometry>
          <box>
            <size>{length} {thickness} {height}</size>
          </box>
        </geometry>
      </collision>
      <visual name="visual">
        <geometry>
          <box>
            <size>{length} {thickness} {height}</size>
          </box>
        </geometry>
        <material>
          <ambient>{r} {g} {b} 1</ambient>
          <diffuse>{r} {g} {b} 1</diffuse>
        </material>
      </visual>
    </link>
  </model>
</sdf>'''

        pose = Pose()
        pose.position.x = position[0]
        pose.position.y = position[1]
        pose.position.z = height / 2
        pose.orientation.x = q[0]
        pose.orientation.y = q[1]
        pose.orientation.z = q[2]
        pose.orientation.w = q[3]

        return self._spawn_model(name, sdf, pose)

    def _spawn_model(self, name: str, sdf: str, pose: Pose) -> bool:
        """Spawn model in Gazebo"""
        try:
            response = self.spawn_service(
                model_name=name,
                model_xml=sdf,
                robot_namespace="",
                initial_pose=pose,
                reference_frame="world"
            )
            return response.success
        except rospy.ServiceException as e:
            rospy.logerr(f"Failed to spawn {name}: {e}")
            return False

    def _clear_all_obstacles(self):
        """Delete all spawned obstacles"""
        rospy.loginfo(f"Clearing {len(self.spawned_obstacles)} obstacles")

        for name in list(self.spawned_obstacles):
            try:
                self.delete_service(model_name=name)
                self.spawned_obstacles.remove(name)
            except rospy.ServiceException as e:
                rospy.logwarn(f"Failed to delete {name}: {e}")

        rospy.loginfo("Obstacles cleared")


if __name__ == '__main__':
    try:
        generator = ObstacleGenerator()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
