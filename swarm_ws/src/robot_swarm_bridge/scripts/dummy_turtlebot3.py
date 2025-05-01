#!/usr/bin/env python3

import rospy
import json
import math
import numpy as np
from std_msgs.msg import String
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Twist, TransformStamped
from tf2_ros import TransformBroadcaster

class DummyTurtleBot3:
    def __init__(self):
        rospy.init_node('dummy_turtlebot3', anonymous=True)
        
        # Parameters
        self.robot_id = rospy.get_param('~robot_id', '1')
        self.wheel_radius = rospy.get_param('~wheel_radius', 0.033)  # TurtleBot3 Burger wheel radius
        self.wheel_separation = rospy.get_param('~wheel_separation', 0.160)  # TurtleBot3 Burger
        self.publish_rate = rospy.get_param('~publish_rate', 10.0)  # Hz
        self.noise_amplitude = rospy.get_param('~noise_amplitude', 0.05)  # Noise level (0-1)
        
        # Get simulated position (for tf)
        self.x_pos = rospy.get_param('~x_pos', 0.0)
        self.y_pos = rospy.get_param('~y_pos', 0.0)
        self.yaw = rospy.get_param('~yaw', 0.0)
        
        # Get namespace for ROS topics
        self.namespace = rospy.get_namespace().strip('/')
        if not self.namespace:
            self.namespace = f"tb3_{self.robot_id}"
            rospy.logwarn(f"No namespace specified, using {self.namespace}")
        
        # Initialize simulated values
        self.left_ticks = 0.0
        self.right_ticks = 0.0
        
        self.left_diff = 0.0
        self.right_diff = 0.0
        self.left_dist = 0.0
        self.right_dist = 0.0
        self.timestep = 1.0 / self.publish_rate
        self.left_speed = 0.0
        self.right_speed = 0.0
        self.left_speed_filtered = 0.0
        self.right_speed_filtered = 0.0
        
        # Movement simulation parameters
        self.linear_velocity = 0.0
        self.angular_velocity = 0.0
        self.ticks_per_meter = 4096.0 / (2.0 * math.pi * self.wheel_radius)  # Approximation
        self.alpha = 0.7  # Filter coefficient
        
        # TF broadcaster for simulated position
        self.tf_broadcaster = TransformBroadcaster()
        
        # Publishers
        # Main sensor data topic with robot ID in name
        self.sensor_pub = rospy.Publisher(f'/robot/{self.robot_id}/sensor_data', String, queue_size=10)
        
        # Standard TurtleBot3 topics 
        self.joint_pub = rospy.Publisher('joint_states', JointState, queue_size=10)
        self.odom_pub = rospy.Publisher('odom', Odometry, queue_size=10)
        
        # Subscribe to velocity commands
        rospy.Subscriber('cmd_vel', Twist, self.cmd_vel_callback)
        
        # Timer for publishing at a fixed rate
        self.timer = rospy.Timer(rospy.Duration(1.0/self.publish_rate), self.update_and_publish)
        
        self.last_update_time = rospy.Time.now()
        
        rospy.loginfo(f"Dummy TurtleBot3 (ID: {self.robot_id}, Namespace: {self.namespace}) initialized, publishing at {self.publish_rate} Hz")
    
    def cmd_vel_callback(self, msg):
        """Update simulated robot velocities based on command"""
        self.linear_velocity = msg.linear.x
        self.angular_velocity = msg.angular.z
    
    def add_noise(self, value):
        """Add random noise to simulate sensor readings"""
        noise = (np.random.random() - 0.5) * 2 * self.noise_amplitude * abs(value)
        if abs(value) < 0.001:  # If value is close to zero, reduce noise
            noise *= 0.1
        return value + noise
    
    def update_simulation(self, dt):
        """Update simulation state based on current velocities"""
        if dt <= 0:
            return
            
        # Calculate wheel velocities from robot velocity
        v_right = self.linear_velocity + (self.angular_velocity * self.wheel_separation / 2.0)
        v_left = self.linear_velocity - (self.angular_velocity * self.wheel_separation / 2.0)
        
        # Calculate distance traveled by each wheel
        self.left_dist = v_left * dt
        self.right_dist = v_right * dt
        
        # Calculate encoder ticks difference
        self.left_diff = self.left_dist * self.ticks_per_meter
        self.right_diff = self.right_dist * self.ticks_per_meter
        
        # Update accumulated ticks
        self.left_ticks += self.left_diff
        self.right_ticks += self.right_diff
        
        # Calculate speeds
        self.left_speed = v_left
        self.right_speed = v_right
        
        # Apply low-pass filter
        self.left_speed_filtered = self.alpha * self.left_speed + (1 - self.alpha) * self.left_speed_filtered
        self.right_speed_filtered = self.alpha * self.right_speed + (1 - self.alpha) * self.right_speed_filtered
        
        # Update simulated position
        # This is a simplistic model without proper odometry integration
        self.x_pos += self.linear_velocity * dt * math.cos(self.yaw)
        self.y_pos += self.linear_velocity * dt * math.sin(self.yaw)
        self.yaw += self.angular_velocity * dt
    
    def update_and_publish(self, event):
        """Update simulation and publish sensor data"""
        current_time = rospy.Time.now()
        dt = (current_time - self.last_update_time).to_sec()
        self.last_update_time = current_time
        
        # Update simulation
        self.update_simulation(dt)
        
        # Add noise to all values to simulate real sensors
        noisy_left_ticks = self.add_noise(self.left_ticks)
        noisy_right_ticks = self.add_noise(self.right_ticks)
        noisy_left_diff = self.add_noise(self.left_diff)
        noisy_right_diff = self.add_noise(self.right_diff)
        noisy_left_dist = self.add_noise(self.left_dist)
        noisy_right_dist = self.add_noise(self.right_dist)
        noisy_left_speed = self.add_noise(self.left_speed)
        noisy_right_speed = self.add_noise(self.right_speed)
        noisy_left_speed_filtered = self.add_noise(self.left_speed_filtered)
        noisy_right_speed_filtered = self.add_noise(self.right_speed_filtered)
        
        # Prepare sensor data
        sensor_data = {
            "left_ticks": round(noisy_left_ticks, 1),
            "right_ticks": round(noisy_right_ticks, 1),
            "left_diff": round(noisy_left_diff, 1),
            "right_diff": round(noisy_right_diff, 1),
            "left_dist": round(noisy_left_dist, 1),
            "right_dist": round(noisy_right_dist, 1),
            "timestep": round(dt, 1),
            "left_speed": round(noisy_left_speed, 1),
            "right_speed": round(noisy_right_speed, 1),
            "left_speed_filtered": round(noisy_left_speed_filtered, 1),
            "right_speed_filtered": round(noisy_right_speed_filtered, 1),
            "name": "Speed"
        }
        
        # Convert to JSON and publish
        json_str = json.dumps(sensor_data)
        msg = String()
        msg.data = json_str
        self.sensor_pub.publish(msg)
        
        # Also publish standard TurtleBot3 topics
        self.publish_joint_states(current_time)
        self.publish_odom(current_time)
        self.publish_tf(current_time)
        
        rospy.loginfo_throttle(3, f"[{self.namespace}] Published sensor data")
    
    def publish_joint_states(self, timestamp):
        """Publish simulated joint states"""
        msg = JointState()
        msg.header.stamp = timestamp
        msg.header.frame_id = f"{self.namespace}/base_link"
        
        # TurtleBot3 joint names
        msg.name = [f'{self.namespace}/wheel_left_joint', f'{self.namespace}/wheel_right_joint']
        
        # Convert ticks to radians
        radians_per_tick = (2.0 * math.pi) / (self.ticks_per_meter * 2.0 * math.pi * self.wheel_radius)
        left_pos = self.left_ticks * radians_per_tick
        right_pos = self.right_ticks * radians_per_tick
        
        msg.position = [left_pos, right_pos]
        msg.velocity = [self.left_speed / self.wheel_radius, self.right_speed / self.wheel_radius]
        msg.effort = [0.0, 0.0]
        
        self.joint_pub.publish(msg)
    
    def publish_odom(self, timestamp):
        """Publish simulated odometry"""
        msg = Odometry()
        msg.header.stamp = timestamp
        msg.header.frame_id = "odom"
        msg.child_frame_id = f"{self.namespace}/base_footprint"
        
        # Set the position
        msg.pose.pose.position.x = self.x_pos
        msg.pose.pose.position.y = self.y_pos
        msg.pose.pose.position.z = 0.0
        
        # Set the orientation (simple yaw)
        msg.pose.pose.orientation.x = 0.0
        msg.pose.pose.orientation.y = 0.0
        msg.pose.pose.orientation.z = math.sin(self.yaw / 2.0)
        msg.pose.pose.orientation.w = math.cos(self.yaw / 2.0)
        
        # Set the velocities
        msg.twist.twist.linear.x = self.linear_velocity
        msg.twist.twist.angular.z = self.angular_velocity
        
        self.odom_pub.publish(msg)
    
    def publish_tf(self, timestamp):
        """Publish TF transformation"""
        t = TransformStamped()
        t.header.stamp = timestamp
        t.header.frame_id = "odom"
        t.child_frame_id = f"{self.namespace}/base_footprint"
        
        # Set translation
        t.transform.translation.x = self.x_pos
        t.transform.translation.y = self.y_pos
        t.transform.translation.z = 0.0
        
        # Set rotation (from yaw)
        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = math.sin(self.yaw / 2.0)
        t.transform.rotation.w = math.cos(self.yaw / 2.0)
        
        # Send the transform
        self.tf_broadcaster.sendTransform(t)

if __name__ == '__main__':
    try:
        dummy_robot = DummyTurtleBot3()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass