#!/usr/bin/env python3
"""
Robot Sensor Data Publisher for TurtleBot3 Swarm System

This node collects sensor data from TurtleBot3 robots and publishes it in the required format:
/robot/n/sensor_data with JSON formatted string including wheel encoders, speeds, etc.

The node subscribes to the TurtleBot's native sensor topics and transforms the data into
the required format, publishing at a consistent rate for each robot.
"""

import rospy
import json
import math
from std_msgs.msg import String
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState
from tf.transformations import euler_from_quaternion
from std_msgs.msg import Float32
from geometry_msgs.msg import Twist

class RobotSensorPublisher:
    """Publishes sensor data for a specific robot in the required format"""
    
    def __init__(self, robot_id):
        """
        Initialize the sensor publisher for a specific robot
        
        Args:
            robot_id (int): Robot identifier number
        """
        # Store robot ID and create namespace
        self.robot_id = robot_id
        self.robot_ns = f"/robot/{robot_id}"

        # Get sensor type from ROS parameter (allows multiple publishers per robot)
        # Valid types: Speed, LaserDistance, Encoder, Inercial, MotorSpeedPercentage
        self.sensor_type = rospy.get_param("~sensor_type", "Speed")

        # Initialize sensor data dictionary with default values
        self.sensor_data = {
            "name": f"Robot_{robot_id}",      # Robot identifier for display
            "sensorType": self.sensor_type,   # Configurable via ROS param
            "left_ticks": 0.0,
            "right_ticks": 0.0,
            "left_diff": 0.0,
            "right_diff": 0.0,
            "left_dist": 0.0,
            "right_dist": 0.0,
            "timestep": 0.0,
            "left_speed": 0.0,
            "right_speed": 0.0,
            "left_speed_filtered": 0.0,
            "right_speed_filtered": 0.0
        }
        
        # Store previous values for calculating differences
        self.prev_left_ticks = 0.0
        self.prev_right_ticks = 0.0
        self.prev_time = rospy.Time.now()
        
        # Speed filtering parameters
        self.filter_alpha = 0.3  # Low-pass filter coefficient
        
        # Publisher for sensor data
        self.sensor_pub = rospy.Publisher(
            f"{self.robot_ns}/sensor_data",
            String,
            queue_size=10
        )
        
        # Subscribers to TurtleBot native topics
        self.joint_sub = rospy.Subscriber(
            f"{self.robot_ns}/joint_states",
            JointState,
            self.joint_callback
        )
        
        self.odom_sub = rospy.Subscriber(
            f"{self.robot_ns}/odom",
            Odometry,
            self.odom_callback
        )
        
        # Timer for publishing sensor data at a fixed rate
        self.pub_rate = rospy.get_param("~sensor_publish_rate", 10.0)  # Hz
        self.pub_timer = rospy.Timer(
            rospy.Duration(1.0/self.pub_rate),
            self.publish_sensor_data
        )
        
        rospy.loginfo(f"Sensor publisher initialized for robot {robot_id} with sensor type: {self.sensor_type}")
    
    def joint_callback(self, msg):
        """
        Process joint state messages to extract wheel encoder values
        
        Args:
            msg (JointState): Joint state message from TurtleBot
        """
        try:
            # TurtleBot3 has 'wheel_left_joint' and 'wheel_right_joint'
            if 'wheel_left_joint' in msg.name and 'wheel_right_joint' in msg.name:
                left_idx = msg.name.index('wheel_left_joint')
                right_idx = msg.name.index('wheel_right_joint')
                
                # Get current ticks from joint positions (radians)
                # Convert radians to ticks (assuming 4096 ticks per revolution)
                ticks_per_rev = 4096.0
                left_ticks = (msg.position[left_idx] * ticks_per_rev) / (2 * math.pi)
                right_ticks = (msg.position[right_idx] * ticks_per_rev) / (2 * math.pi)
                
                # Calculate differences from previous readings
                self.sensor_data["left_diff"] = left_ticks - self.prev_left_ticks
                self.sensor_data["right_diff"] = right_ticks - self.prev_right_ticks
                
                # Store current values for next calculation
                self.prev_left_ticks = left_ticks
                self.prev_right_ticks = right_ticks
                
                # Store tick values
                self.sensor_data["left_ticks"] = left_ticks
                self.sensor_data["right_ticks"] = right_ticks
                
                # Calculate wheel speeds from joint velocities
                # TurtleBot3 Waffle wheel radius is approximately 0.033 m
                wheel_radius = 0.033
                if hasattr(msg, 'velocity') and len(msg.velocity) > max(left_idx, right_idx):
                    left_speed = msg.velocity[left_idx] * wheel_radius  # m/s
                    right_speed = msg.velocity[right_idx] * wheel_radius  # m/s
                    
                    # Low-pass filter for speed values
                    self.sensor_data["left_speed"] = left_speed
                    self.sensor_data["right_speed"] = right_speed
                    
                    # Apply low-pass filter to smooth speed values
                    self.sensor_data["left_speed_filtered"] = (
                        self.filter_alpha * left_speed + 
                        (1 - self.filter_alpha) * self.sensor_data["left_speed_filtered"]
                    )
                    self.sensor_data["right_speed_filtered"] = (
                        self.filter_alpha * right_speed + 
                        (1 - self.filter_alpha) * self.sensor_data["right_speed_filtered"]
                    )
                
                # Update distances (integrate from ticks)
                self.sensor_data["left_dist"] += abs(self.sensor_data["left_diff"]) * wheel_radius * 2 * math.pi / ticks_per_rev
                self.sensor_data["right_dist"] += abs(self.sensor_data["right_diff"]) * wheel_radius * 2 * math.pi / ticks_per_rev
                
                # Update timestep
                current_time = rospy.Time.now()
                self.sensor_data["timestep"] = (current_time - self.prev_time).to_sec()
                self.prev_time = current_time
                
        except Exception as e:
            rospy.logwarn(f"Error processing joint states for robot {self.robot_id}: {str(e)}")
    
    def odom_callback(self, msg):
        """
        Process odometry messages for additional data
        
        Args:
            msg (Odometry): Odometry message from TurtleBot
        """
        try:
            # Extract linear and angular velocities
            linear_vel = msg.twist.twist.linear.x
            angular_vel = msg.twist.twist.angular.z
            
            # We could use these to calculate left and right wheel speeds with differential drive kinematics
            # but we already get these more directly from joint states
            pass
            
        except Exception as e:
            rospy.logwarn(f"Error processing odometry for robot {self.robot_id}: {str(e)}")
    
    def publish_sensor_data(self, event=None):
        """
        Publish sensor data at regular intervals
        
        Args:
            event: Timer event (not used)
        """
        try:
            # Convert sensor data to JSON string
            json_str = json.dumps(self.sensor_data)
            
            # Publish the message
            self.sensor_pub.publish(json_str)
            
        except Exception as e:
            rospy.logwarn(f"Error publishing sensor data for robot {self.robot_id}: {str(e)}")
    
    def shutdown(self):
        """Clean shutdown of the sensor publisher"""
        self.pub_timer.shutdown()
        self.joint_sub.unregister()
        self.odom_sub.unregister()
        rospy.loginfo(f"Sensor publisher for robot {self.robot_id} shutdown")

class SensorPublisherManager:
    """Manages sensor publishers for all robots in the swarm"""
    
    def __init__(self):
        """Initialize the sensor publisher manager"""
        rospy.init_node('robot_sensor_publisher_manager', anonymous=False)
        
        # Dictionary to track sensor publishers for each robot
        self.publishers = {}
        
        # Subscribe to robot spawning notifications
        rospy.Subscriber(
            "/robot_manager/robot_spawned",
            String,
            self.handle_robot_spawned
        )
        
        # Subscribe to robot deletion notifications
        rospy.Subscriber(
            "/robot_manager/robot_deleted",
            String,
            self.handle_robot_deleted
        )
        
        rospy.loginfo("Sensor Publisher Manager initialized")
    
    def handle_robot_spawned(self, msg):
        """
        Handle notifications when a new robot is spawned
        
        Args:
            msg (String): Message containing robot ID
        """
        try:
            # Parse the robot ID from the message
            robot_id = int(msg.data)
            
            # Create a new sensor publisher if it doesn't already exist
            if robot_id not in self.publishers:
                self.publishers[robot_id] = RobotSensorPublisher(robot_id)
                rospy.loginfo(f"Created sensor publisher for robot {robot_id}")
                
        except Exception as e:
            rospy.logerr(f"Error handling robot spawned notification: {str(e)}")
    
    def handle_robot_deleted(self, msg):
        """
        Handle notifications when a robot is deleted
        
        Args:
            msg (String): Message containing robot ID
        """
        try:
            # Parse the robot ID from the message
            robot_id = int(msg.data)
            
            # Shutdown and remove the sensor publisher if it exists
            if robot_id in self.publishers:
                self.publishers[robot_id].shutdown()
                del self.publishers[robot_id]
                rospy.loginfo(f"Removed sensor publisher for robot {robot_id}")
                
        except Exception as e:
            rospy.logerr(f"Error handling robot deleted notification: {str(e)}")
    
    def run(self):
        """Main execution loop for the sensor publisher manager"""
        rospy.loginfo("Sensor Publisher Manager is running")
        rospy.spin()
    
    def shutdown(self):
        """Clean shutdown of all sensor publishers"""
        for robot_id, publisher in self.publishers.items():
            publisher.shutdown()
        rospy.loginfo("Sensor Publisher Manager shutdown")

if __name__ == "__main__":
    try:
        manager = SensorPublisherManager()
        rospy.on_shutdown(manager.shutdown)
        manager.run()
    except rospy.ROSInterruptException:
        pass