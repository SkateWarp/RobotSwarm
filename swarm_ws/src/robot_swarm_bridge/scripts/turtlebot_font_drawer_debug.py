#!/usr/bin/env python3
"""
Debug script to check robot positions and movement in ROS.
Run this script to monitor robot odometry and position data.
"""

import rospy
import math
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
from geometry_msgs.msg import Twist
import time

class TurtleBotPositionDebugger:
    """
    Debug tool to monitor TurtleBot3 positioning and test movement
    """
    
    def __init__(self, robot_namespace_template="/robot/{}/"):
        """
        Initialize the TurtleBot position debugger
        
        Args:
            robot_namespace_template (str): Template for robot namespaces
        """
        rospy.init_node('turtlebot_position_debugger', anonymous=True)
        self.robot_namespace_template = robot_namespace_template
        
        # Internal state
        self.positions = {}
        self.orientations = {}
        
        # Subscribe to robot odometry (initially for robots 1 and 2)
        for robot_id in [1, 2]:
            robot_ns = self.robot_namespace_template.format(robot_id)
            robot_name = f"robot_{robot_id}"
            
            # Subscribe to odometry
            rospy.Subscriber(
                f"{robot_ns}odom",
                Odometry,
                self.odom_callback,
                callback_args=robot_name
            )
            
            # Create publishers for movement commands
            self.cmd_vel_pub = rospy.Publisher(
                f"{robot_ns}cmd_vel",
                Twist,
                queue_size=10
            )
            
    def odom_callback(self, msg, robot_name):
        """
        Process odometry data from a robot
        
        Args:
            msg (nav_msgs.msg.Odometry): Odometry message
            robot_name (str): Name of the robot
        """
        try:
            # Extract position
            x = msg.pose.pose.position.x
            y = msg.pose.pose.position.y
            z = msg.pose.pose.position.z
            
            # Extract orientation as Euler angles
            orientation_q = msg.pose.pose.orientation
            roll, pitch, yaw = euler_from_quaternion(
                [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
            )
            
            # Store position and orientation
            self.positions[robot_name] = (x, y, z)
            self.orientations[robot_name] = (roll, pitch, yaw)
            
            # Print position info
            rospy.loginfo(f"{robot_name} Position: x={x:.2f}, y={y:.2f}, z={z:.2f}")
            rospy.loginfo(f"{robot_name} Orientation (rad): roll={roll:.2f}, pitch={pitch:.2f}, yaw={yaw:.2f}")
            rospy.loginfo(f"{robot_name} Orientation (deg): yaw={(yaw * 180.0 / math.pi):.1f}°")
            
        except Exception as e:
            rospy.logerr(f"Error processing odometry for {robot_name}: {str(e)}")

    def test_rotation(self, robot_id=1, angle_degrees=90):
        """
        Test rotating a robot by a specific angle
        
        Args:
            robot_id (int): Robot ID
            angle_degrees (float): Angle to rotate in degrees
        """
        # Convert to radians
        target_angle = math.radians(angle_degrees)
        robot_name = f"robot_{robot_id}"
        
        rospy.loginfo(f"Testing rotation of {robot_name} by {angle_degrees} degrees")
        
        # Wait for position data
        while robot_name not in self.orientations:
            rospy.loginfo(f"Waiting for {robot_name} position data...")
            time.sleep(1.0)
        
        # Get initial orientation
        _, _, initial_yaw = self.orientations[robot_name]
        target_yaw = initial_yaw + target_angle
        
        # Normalize target angle to [-pi, pi]
        while target_yaw > math.pi:
            target_yaw -= 2 * math.pi
        while target_yaw < -math.pi:
            target_yaw += 2 * math.pi
            
        rospy.loginfo(f"Initial yaw: {(initial_yaw * 180.0 / math.pi):.1f}°")
        rospy.loginfo(f"Target yaw: {(target_yaw * 180.0 / math.pi):.1f}°")
        
        # Create twist message
        twist = Twist()
        if target_angle >= 0:
            twist.angular.z = 0.3  # Positive for counter-clockwise
        else:
            twist.angular.z = -0.3  # Negative for clockwise
            
        # Start rotation
        start_time = rospy.Time.now()
        rate = rospy.Rate(10)  # 10 Hz
        
        while not rospy.is_shutdown():
            # Get current orientation
            _, _, current_yaw = self.orientations[robot_name]
            
            # Calculate angle difference
            angle_diff = abs(target_yaw - current_yaw)
            # Handle wrap-around
            if angle_diff > math.pi:
                angle_diff = 2 * math.pi - angle_diff
                
            rospy.loginfo(f"Current yaw: {(current_yaw * 180.0 / math.pi):.1f}°, "
                          f"angle diff: {(angle_diff * 180.0 / math.pi):.1f}°")
                          
            # Check if we're close enough to target angle
            if angle_diff < 0.1:  # ~5.7 degrees
                break
                
            # Check for timeout (10 seconds)
            if (rospy.Time.now() - start_time).to_sec() > 10.0:
                rospy.logwarn("Rotation timeout reached")
                break
                
            # Send rotation command
            self.cmd_vel_pub.publish(twist)
            rate.sleep()
            
        # Stop rotation
        twist = Twist()  # All zeros
        self.cmd_vel_pub.publish(twist)
        rospy.loginfo(f"Rotation complete")
    
    def test_linear_movement(self, robot_id=1, distance=0.5):
        """
        Test moving a robot in a straight line
        
        Args:
            robot_id (int): Robot ID
            distance (float): Distance to move in meters
        """
        robot_name = f"robot_{robot_id}"
        
        rospy.loginfo(f"Testing linear movement of {robot_name} by {distance} meters")
        
        # Wait for position data
        while robot_name not in self.positions:
            rospy.loginfo(f"Waiting for {robot_name} position data...")
            time.sleep(1.0)
        
        # Get initial position
        initial_x, initial_y, _ = self.positions[robot_name]
        _, _, current_yaw = self.orientations[robot_name]
        
        # Calculate target position (in robot's local frame)
        target_x = initial_x + distance * math.cos(current_yaw)
        target_y = initial_y + distance * math.sin(current_yaw)
        
        rospy.loginfo(f"Initial position: ({initial_x:.2f}, {initial_y:.2f})")
        rospy.loginfo(f"Target position: ({target_x:.2f}, {target_y:.2f})")
        
        # Create twist message
        twist = Twist()
        twist.linear.x = 0.1  # Move at 0.1 m/s
        
        # Start movement
        start_time = rospy.Time.now()
        rate = rospy.Rate(10)  # 10 Hz
        
        while not rospy.is_shutdown():
            # Get current position
            current_x, current_y, _ = self.positions[robot_name]
            
            # Calculate distance to target
            dist_to_target = math.sqrt((target_x - current_x)**2 + (target_y - current_y)**2)
            
            rospy.loginfo(f"Current position: ({current_x:.2f}, {current_y:.2f}), "
                          f"Distance to target: {dist_to_target:.2f}m")
                          
            # Check if we're close enough to target
            if dist_to_target < 0.1:  # 10 cm
                break
                
            # Check for timeout (30 seconds or distance / speed * 2)
            timeout = max(30.0, abs(distance) / 0.1 * 2.0)
            if (rospy.Time.now() - start_time).to_sec() > timeout:
                rospy.logwarn("Movement timeout reached")
                break
                
            # Send movement command
            self.cmd_vel_pub.publish(twist)
            rate.sleep()
            
        # Stop movement
        twist = Twist()  # All zeros
        self.cmd_vel_pub.publish(twist)
        rospy.loginfo(f"Movement complete")
        
    def test_circle_drawing(self, robot_id=1, radius=0.5):
        """
        Test drawing a circle with a robot
        
        Args:
            robot_id (int): Robot ID
            radius (float): Circle radius in meters
        """
        robot_name = f"robot_{robot_id}"
        
        rospy.loginfo(f"Testing circle drawing with {robot_name}, radius={radius}m")
        
        # Create twist message for circular motion
        # To move in a circle, we need both linear and angular velocity
        twist = Twist()
        twist.linear.x = 0.1  # Linear velocity (m/s)
        twist.angular.z = twist.linear.x / radius  # Angular velocity (rad/s)
        
        # Calculate the time to complete one circle
        # Time = distance / speed = (2*pi*radius) / linear_speed
        circle_time = 2 * math.pi * radius / twist.linear.x
        
        rospy.loginfo(f"Circle time: {circle_time:.2f} seconds")
        
        # Draw circle
        start_time = rospy.Time.now()
        rate = rospy.Rate(10)  # 10 Hz
        
        while not rospy.is_shutdown():
            # Check if we've completed the circle
            elapsed = (rospy.Time.now() - start_time).to_sec()
            if elapsed > circle_time:
                break
                
            # Send movement command
            self.cmd_vel_pub.publish(twist)
            
            # Log progress
            if int(elapsed) % 2 == 0:  # Log every 2 seconds
                percent_complete = (elapsed / circle_time) * 100
                rospy.loginfo(f"Circle progress: {percent_complete:.1f}%")
                
            rate.sleep()
            
        # Stop movement
        twist = Twist()  # All zeros
        self.cmd_vel_pub.publish(twist)
        rospy.loginfo(f"Circle drawing complete")
        
    def run(self):
        """Run the position debugger"""
        rospy.loginfo("TurtleBot Position Debugger is running")
        rospy.loginfo("Monitoring robot positions...")
        
        # Continue until shutdown
        rospy.spin()

if __name__ == "__main__":
    try:
        debugger = TurtleBotPositionDebugger()
        
        # Wait for initial position data
        rospy.loginfo("Waiting for robot position data...")
        time.sleep(3.0)
        
        # Menu loop for interactive testing
        while not rospy.is_shutdown():
            print("\nTurtleBot Position Debugger Menu:")
            print("1. Monitor positions")
            print("2. Test rotation (90 degrees)")
            print("3. Test linear movement (0.5m)")
            print("4. Test circle drawing (radius 0.5m)")
            print("q. Quit")
            
            choice = input("Enter choice: ")
            
            if choice == '1':
                print("Monitoring positions for 10 seconds...")
                time.sleep(10.0)
            elif choice == '2':
                robot_id = int(input("Enter robot ID (1 or 2): "))
                angle = float(input("Enter angle in degrees: ") or "90")
                debugger.test_rotation(robot_id, angle)
            elif choice == '3':
                robot_id = int(input("Enter robot ID (1 or 2): "))
                distance = float(input("Enter distance in meters: ") or "0.5")
                debugger.test_linear_movement(robot_id, distance)
            elif choice == '4':
                robot_id = int(input("Enter robot ID (1 or 2): "))
                radius = float(input("Enter radius in meters: ") or "0.5")
                debugger.test_circle_drawing(robot_id, radius)
            elif choice.lower() == 'q':
                break
            else:
                print("Invalid choice")
    
    except rospy.ROSInterruptException:
        pass
    except KeyboardInterrupt:
        rospy.loginfo("Debugger terminated by user")
