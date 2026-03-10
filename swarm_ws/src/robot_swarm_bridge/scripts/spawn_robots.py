#!/usr/bin/env python3
"""
Dynamic robot spawner for counts > 10 or runtime spawning.
Spawns TurtleBot3 robots using xacro URDF processing and Gazebo services.
"""

import rospy
import os
import math
import subprocess
import rospkg

try:
    import xacro
except ImportError:
    rospy.logwarn("xacro module not available, will use rosrun xacro fallback")
    xacro = None

from gazebo_msgs.srv import SpawnModel
from geometry_msgs.msg import Pose
import tf.transformations as tft


def get_robot_urdf(model='waffle'):
    """Process TurtleBot3 xacro into URDF string"""
    rospack = rospkg.RosPack()
    tb3_desc = rospack.get_path('turtlebot3_description')
    urdf_path = os.path.join(tb3_desc, 'urdf', f'turtlebot3_{model}.urdf.xacro')

    if xacro is not None:
        doc = xacro.process_file(urdf_path)
        return doc.toxml()
    else:
        # Fallback: use command line xacro
        result = subprocess.run(
            ['rosrun', 'xacro', 'xacro', urdf_path],
            capture_output=True, text=True
        )
        return result.stdout


def generate_positions(count, start_index, pattern, spacing):
    """Generate spawn positions"""
    positions = []

    if pattern == 'grid':
        total = start_index + count
        grid_size = int(math.ceil(math.sqrt(total)))
        idx = 0
        for row in range(grid_size):
            for col in range(grid_size):
                if idx >= start_index and idx < total:
                    x = (col - grid_size / 2.0 + 0.5) * spacing
                    y = (row - grid_size / 2.0 + 0.5) * spacing
                    positions.append((x, y, 0.0))
                idx += 1
                if len(positions) >= count:
                    break
            if len(positions) >= count:
                break

    elif pattern == 'circle':
        total = start_index + count
        radius = max(spacing, (total * spacing) / (2 * math.pi))
        for i in range(count):
            angle = (2 * math.pi * (start_index + i)) / total
            x = radius * math.cos(angle)
            y = radius * math.sin(angle)
            positions.append((x, y, angle + math.pi))

    elif pattern == 'line':
        for i in range(count):
            x = (start_index + i - (start_index + count) / 2.0 + 0.5) * spacing
            positions.append((x, 0.0, 0.0))

    return positions


def main():
    rospy.init_node('dynamic_robot_spawner', anonymous=True)

    robot_count = rospy.get_param('~robot_count', 5)
    start_index = rospy.get_param('~start_index', 0)
    pattern = rospy.get_param('~spawn_pattern', 'grid')
    spacing = rospy.get_param('~spacing', 1.0)
    model = rospy.get_param('~model', 'waffle')

    rospy.loginfo(f"Spawning {robot_count} robots starting at index {start_index}")

    # Wait for Gazebo
    rospy.loginfo("Waiting for Gazebo spawn service...")
    rospy.wait_for_service('/gazebo/spawn_urdf_model', timeout=60.0)
    spawn_service = rospy.ServiceProxy('/gazebo/spawn_urdf_model', SpawnModel)

    # Get URDF
    rospy.loginfo("Processing robot URDF...")
    robot_urdf = get_robot_urdf(model)

    # Generate positions
    positions = generate_positions(robot_count, start_index, pattern, spacing)

    # Spawn each robot
    rsp_procs = []
    for i, pos in enumerate(positions):
        robot_name = f"tb3_{start_index + i}"
        x, y, yaw = pos[0], pos[1], pos[2] if len(pos) > 2 else 0.0

        # Set robot_description param
        rospy.set_param(f'/{robot_name}/robot_description', robot_urdf)

        # Spawn in Gazebo
        pose = Pose()
        pose.position.x = x
        pose.position.y = y
        pose.position.z = 0.0
        q = tft.quaternion_from_euler(0, 0, yaw)
        pose.orientation.x = q[0]
        pose.orientation.y = q[1]
        pose.orientation.z = q[2]
        pose.orientation.w = q[3]

        try:
            resp = spawn_service(
                model_name=robot_name,
                model_xml=robot_urdf,
                robot_namespace=f'/{robot_name}',
                initial_pose=pose,
                reference_frame='world'
            )
            if resp.success:
                rospy.loginfo(f"Spawned {robot_name} at ({x:.2f}, {y:.2f})")

                # Launch robot_state_publisher
                proc = subprocess.Popen([
                    'rosrun', 'robot_state_publisher', 'robot_state_publisher',
                    f'__ns:={robot_name}',
                    f'_publish_frequency:=50.0',
                    f'_tf_prefix:={robot_name}',
                ], env={**os.environ, 'ROS_NAMESPACE': robot_name})
                rsp_procs.append(proc)
            else:
                rospy.logerr(f"Failed to spawn {robot_name}: {resp.status_message}")
        except rospy.ServiceException as e:
            rospy.logerr(f"Spawn service failed for {robot_name}: {e}")

        rospy.sleep(1.0)  # Stagger spawns to avoid Gazebo overload

    rospy.loginfo(f"Dynamic spawner finished. Spawned {len(rsp_procs)} robots.")

    # Keep alive until shutdown (robot_state_publishers need this)
    rospy.spin()

    # Cleanup
    for proc in rsp_procs:
        proc.terminate()


if __name__ == '__main__':
    main()
