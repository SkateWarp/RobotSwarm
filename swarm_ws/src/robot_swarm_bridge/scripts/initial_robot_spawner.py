#!/usr/bin/env python3
"""
Initial Robot Spawner for TurtleBot3 Swarm System

This script spawns the initial set of robots when the system starts up.
It calls the robot_manager service to spawn the specified number of robots
in appropriate positions within the arena.
"""

import rospy
import time
import math
from std_srvs.srv import Empty
from geometry_msgs.msg import Pose
from tf.transformations import quaternion_from_euler

def main():
    """Main function to spawn initial robots"""
    rospy.init_node('initial_robot_spawner', anonymous=True)
    
    # Get parameters
    num_robots = rospy.get_param("~num_robots", 2)
    
    # Cap at a reasonable number of robots
    num_robots = min(num_robots, 8)
    
    # Wait for robot manager service
    rospy.loginfo("Waiting for robot_manager/spawn_robot service...")
    try:
        rospy.wait_for_service('/robot_manager/spawn_robot', timeout=30)
    except rospy.ROSException:
        rospy.logerr("Service /robot_manager/spawn_robot not available within timeout")
        return
        
    spawn_robot = rospy.ServiceProxy('/robot_manager/spawn_robot', Empty)
    
    # Wait for Gazebo to fully load
    rospy.loginfo(f"Waiting for Gazebo to fully initialize before spawning {num_robots} robots...")
    time.sleep(5.0)
    
    # Spawn the initial robots
    for i in range(num_robots):
        try:
            # Call service to spawn robot
            rospy.loginfo(f"Spawning robot {i+1}/{num_robots}")
            resp = spawn_robot()
            
            # Allow some time between spawns to avoid overwhelming Gazebo
            time.sleep(2.0)
            
        except rospy.ServiceException as e:
            rospy.logerr(f"Service call failed: {e}")
    
    rospy.loginfo(f"Successfully requested spawning of {num_robots} robots")
    
    # Keep the node alive for a while to ensure all robots are properly spawned
    time.sleep(10.0)
    rospy.loginfo("Initial robot spawning completed")

if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass