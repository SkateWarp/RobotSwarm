#!/usr/bin/env python3
"""
Enhanced Robot Manager Node for TurtleBot3 Swarm System

This node provides the following functionality:
- Spawns TurtleBot3 robots in the simulation environment via a ROS service
- Tracks and publishes the status of each robot (idle, working, disconnected)
- Manages robot namespaces (/robot/n/) and coordinates with other components
- Integrates with the enhanced font drawer system for multi-robot coordination
"""

import rospy
import rospkg
import os
import subprocess
import time
import json
import threading
from std_msgs.msg import String, Bool
from geometry_msgs.msg import Pose, Twist
from gazebo_msgs.srv import SpawnModel, DeleteModel
from tf.transformations import quaternion_from_euler, euler_from_quaternion
from std_srvs.srv import Empty
from nav_msgs.msg import Odometry

class RobotManager:
    """Manages TurtleBot3 robots in the swarm system with enhanced coordination"""
    
    def __init__(self):
        """Initialize the robot manager node"""
        rospy.init_node('robot_manager', anonymous=False)
        
        # Robot tracking dictionaries
        self.robots = {}  # Dictionary to track spawned robots
        self.robot_status = {}  # Current status of each robot
        self.robot_positions = {}  # Current positions of robots
        self.robot_orientations = {}  # Current orientations of robots
        
        # Parameters
        self.robot_model = rospy.get_param("~robot_model", "waffle")
        self.status_update_rate = rospy.get_param("~status_update_rate", 1.0)  # Hz
        self.robot_urdf_path = self.get_robot_urdf_path()
        self.arena_width = rospy.get_param("~arena_width", 10.0)
        self.arena_height = rospy.get_param("~arena_height", 10.0)
        
        # Create a lock for thread safety
        self.lock = threading.RLock()
        
        # Services
        self.spawn_service = rospy.Service('/robot_manager/spawn_robot', 
                                         Empty, 
                                         self.handle_spawn_robot)
        self.delete_service = rospy.Service('/robot_manager/delete_robot',
                                         Empty,
                                         self.handle_delete_robot)
        
        # Publishers and subscribers
        self.status_pub = rospy.Publisher('/robot_manager/robot_list', String, queue_size=10)
        self.cmd_vel_pubs = {}  # Publisher for cmd_vel for each robot
        self.sensor_data_pubs = {}  # Publisher for sensor_data for each robot
        self.status_pubs = {}  # Publisher for status for each robot
        
        # Wait for Gazebo services
        rospy.loginfo("Waiting for gazebo services...")
        rospy.wait_for_service('/gazebo/spawn_urdf_model')
        rospy.wait_for_service('/gazebo/delete_model')
        self.gazebo_spawn = rospy.ServiceProxy('/gazebo/spawn_urdf_model', SpawnModel)
        self.gazebo_delete = rospy.ServiceProxy('/gazebo/delete_model', DeleteModel)
        
        # Status update timer
        self.status_timer = rospy.Timer(rospy.Duration(1.0/self.status_update_rate), 
                                      self.update_robot_statuses)
        
        # Subscribe to command topic for drawing tasks
        self.drawer_cmd_pub = rospy.Publisher('/drawer/command', String, queue_size=10)
        
        rospy.loginfo("Robot Manager initialized")
    
    def get_robot_urdf_path(self):
        """Get the path to the TurtleBot3 URDF file"""
        rospack = rospkg.RosPack()
        urdf_path = os.path.join(
            rospack.get_path('turtlebot3_description'),
            'urdf',
            f'turtlebot3_{self.robot_model}.urdf.xacro'
        )
        
        # Check if file exists
        if not os.path.exists(urdf_path):
            rospy.logerr(f"TurtleBot3 URDF file not found at {urdf_path}")
            return None
            
        return urdf_path
    
    def handle_spawn_robot(self, req):
        """
        Handle requests to spawn a new robot
        
        Args:
            req: Service request to spawn a robot
            
        Returns:
            Empty: Empty response
        """
        with self.lock:
            # Find the next available robot ID
            robot_id = 1
            while f"/robot/{robot_id}" in self.robots.values():
                robot_id += 1
                
            robot_name = f"robot_{robot_id}"
            robot_ns = f"/robot/{robot_id}"
            
            # Create the robot model and spawn it in Gazebo
            try:
                # Generate URDF from XACRO
                urdf_content = self.generate_urdf_from_xacro(robot_name)
                if not urdf_content:
                    rospy.logerr(f"Failed to generate URDF for robot {robot_id}")
                    return Empty()
                
                # Calculate position in a grid pattern
                col = ((robot_id - 1) % 3)
                row = ((robot_id - 1) // 3)
                
                # Set initial pose (distributed in the arena)
                initial_pose = Pose()
                initial_pose.position.x = -3.0 + col * 3.0
                initial_pose.position.y = -3.0 + row * 3.0
                initial_pose.position.z = 0.1  # Slightly above ground
                
                # Set orientation
                q = quaternion_from_euler(0, 0, 0)
                initial_pose.orientation.x = q[0]
                initial_pose.orientation.y = q[1]
                initial_pose.orientation.z = q[2]
                initial_pose.orientation.w = q[3]
                
                # Spawn the model in Gazebo
                resp = self.gazebo_spawn(
                    robot_name,
                    urdf_content,
                    robot_ns,
                    initial_pose,
                    "world"
                )
                
                if not resp.success:
                    rospy.logerr(f"Gazebo spawn failed: {resp.status_message}")
                    return Empty()
                
                # Create publishers for this robot
                # Publisher for cmd_vel
                self.cmd_vel_pubs[robot_id] = rospy.Publisher(
                    f"{robot_ns}/cmd_vel", 
                    Twist, 
                    queue_size=10
                )
                
                # Publisher for sensor_data
                self.sensor_data_pubs[robot_id] = rospy.Publisher(
                    f"{robot_ns}/sensor_data", 
                    String, 
                    queue_size=10
                )
                
                # Publisher for status
                self.status_pubs[robot_id] = rospy.Publisher(
                    f"{robot_ns}/status", 
                    String, 
                    queue_size=10
                )
                
                # Subscribe to odom for position tracking
                rospy.Subscriber(
                    f"{robot_ns}/odom",
                    Odometry,
                    self.odom_callback,
                    callback_args=robot_id
                )
                
                # Track the robot
                self.robots[robot_id] = robot_ns
                self.robot_status[robot_id] = 'idle'
                self.robot_positions[robot_id] = (initial_pose.position.x, initial_pose.position.y)
                self.robot_orientations[robot_id] = 0.0
                
                # Publish a notification that a robot was spawned
                rospy.Publisher('/robot_manager/robot_spawned', String, queue_size=10).publish(str(robot_id))
                
                rospy.loginfo(f"Successfully spawned robot {robot_id} at position "
                            f"({initial_pose.position.x}, {initial_pose.position.y})")
                
                # Start the sensor data publisher for this robot
                self._start_sensor_publisher(robot_id)
                
            except Exception as e:
                rospy.logerr(f"Error spawning robot {robot_id}: {str(e)}")
                
        return Empty()
    
    def handle_delete_robot(self, req):
        """
        Handle requests to delete an existing robot
        
        Args:
            req: Service request to delete a robot
            
        Returns:
            Empty: Empty response
        """
        with self.lock:
            if not self.robots:
                rospy.logwarn("No robots to delete")
                return Empty()
            
            # Delete the last robot
            robot_id = max(self.robots.keys())
            robot_name = f"robot_{robot_id}"
            
            try:
                # Delete the model from Gazebo
                resp = self.gazebo_delete(robot_name)
                
                if not resp.success:
                    rospy.logerr(f"Gazebo delete failed: {resp.status_message}")
                    return Empty()
                
                # Remove publishers
                if robot_id in self.cmd_vel_pubs:
                    del self.cmd_vel_pubs[robot_id]
                if robot_id in self.sensor_data_pubs:
                    del self.sensor_data_pubs[robot_id]
                if robot_id in self.status_pubs:
                    del self.status_pubs[robot_id]
                
                # Remove from tracking dictionaries
                del self.robots[robot_id]
                if robot_id in self.robot_status:
                    del self.robot_status[robot_id]
                if robot_id in self.robot_positions:
                    del self.robot_positions[robot_id]
                if robot_id in self.robot_orientations:
                    del self.robot_orientations[robot_id]
                
                # Publish a notification that a robot was deleted
                rospy.Publisher('/robot_manager/robot_deleted', String, queue_size=10).publish(str(robot_id))
                
                rospy.loginfo(f"Successfully deleted robot {robot_id}")
                
            except Exception as e:
                rospy.logerr(f"Error deleting robot {robot_id}: {str(e)}")
                
        return Empty()
    
    def generate_urdf_from_xacro(self, robot_name):
        """
        Generate URDF content from XACRO file
        
        Args:
            robot_name (str): Name for the robot
            
        Returns:
            str: URDF content as string or None if failed
        """
        if not self.robot_urdf_path:
            return None
            
        try:
            # Use subprocess to convert XACRO to URDF
            cmd = ["xacro", self.robot_urdf_path, f"name:={robot_name}"]
            result = subprocess.run(cmd, capture_output=True, text=True, check=True)
            return result.stdout
        except subprocess.CalledProcessError as e:
            rospy.logerr(f"Error generating URDF: {e.stderr}")
            return None
        except Exception as e:
            rospy.logerr(f"Error generating URDF: {str(e)}")
            return None
    
    def odom_callback(self, msg, robot_id):
        """
        Process odometry data to track robot position and orientation
        
        Args:
            msg (nav_msgs.Odometry): Odometry message
            robot_id (int): Robot ID
        """
        with self.lock:
            # Extract position
            x = msg.pose.pose.position.x
            y = msg.pose.pose.position.y
            self.robot_positions[robot_id] = (x, y)
            
            # Extract orientation (yaw)
            orientation_q = msg.pose.pose.orientation
            _, _, yaw = euler_from_quaternion(
                [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
            )
            self.robot_orientations[robot_id] = yaw
    
    def _start_sensor_publisher(self, robot_id):
        """
        Start a thread to publish sensor data for a robot
        
        Args:
            robot_id (int): Robot ID
        """
        def publish_sensor_data():
            """
            Continuously publish sensor data for a robot at 10Hz
            """
            rate = rospy.Rate(10)  # 10Hz
            
            # Simulated encoder ticks
            left_ticks = 0.0
            right_ticks = 0.0
            timestamp = time.time()
            
            while not rospy.is_shutdown() and robot_id in self.robots:
                try:
                    current_time = time.time()
                    dt = current_time - timestamp
                    timestamp = current_time
                    
                    # Get current position and orientation
                    x, y = self.robot_positions.get(robot_id, (0.0, 0.0))
                    theta = self.robot_orientations.get(robot_id, 0.0)
                    
                    # Simulate encoder data based on position change
                    # These are simplistic calculations just for demonstration
                    left_speed = 0.1 * (1 + 0.1 * robot_id)  # Different for each robot
                    right_speed = 0.1 * (1 + 0.1 * robot_id)
                    
                    # Calculate differences
                    left_diff = left_speed * dt * 10.0  # Scale for visibility
                    right_diff = right_speed * dt * 10.0
                    
                    # Update ticks
                    left_ticks += left_diff
                    right_ticks += right_diff
                    
                    # Create sensor data dictionary
                    sensor_data = {
                        "left_ticks": left_ticks,
                        "right_ticks": right_ticks,
                        "left_diff": left_diff,
                        "right_diff": right_diff,
                        "left_dist": left_ticks * 0.01,
                        "right_dist": right_ticks * 0.01,
                        "timestep": dt,
                        "left_speed": left_speed,
                        "right_speed": right_speed,
                        "left_speed_filtered": left_speed * 0.9,
                        "right_speed_filtered": right_speed * 0.9,
                        "name": f"Robot_{robot_id}"
                    }
                    
                    # Convert to JSON string and publish
                    sensor_json = json.dumps(sensor_data)
                    if robot_id in self.sensor_data_pubs:
                        self.sensor_data_pubs[robot_id].publish(sensor_json)
                    
                    rate.sleep()
                except Exception as e:
                    rospy.logerr(f"Error publishing sensor data for robot {robot_id}: {str(e)}")
                    time.sleep(1)  # Prevent tight loop on error
        
        # Start publisher thread
        threading.Thread(target=publish_sensor_data, daemon=True).start()
    
    def update_robot_statuses(self, event=None):
        """
        Periodically update and publish robot statuses
        
        Args:
            event: Timer event (not used)
        """
        with self.lock:
            try:
                # Create status list for all robots
                status_list = []
                
                for robot_id, robot_ns in self.robots.items():
                    status = self.robot_status.get(robot_id, 'disconnected')
                    
                    # Create status entry
                    status_entry = {
                        'robot_id': robot_id,
                        'namespace': robot_ns,
                        'status': status,
                        'timestamp': rospy.Time.now().to_sec()
                    }
                    
                    # Add position if available
                    if robot_id in self.robot_positions:
                        pos = self.robot_positions[robot_id]
                        status_entry['position'] = {'x': pos[0], 'y': pos[1]}
                    
                    status_list.append(status_entry)
                    
                    # Also publish to individual robot status topics
                    if robot_id in self.status_pubs:
                        self.status_pubs[robot_id].publish(status)
                
                # Publish the complete status list
                if status_list:
                    self.status_pub.publish(json.dumps(status_list))
            
            except Exception as e:
                rospy.logerr(f"Error updating robot statuses: {str(e)}")
    
    def assign_task(self, robot_id, task_type, task_params):
        """
        Assign a task to a robot
        
        Args:
            robot_id (int): Robot ID
            task_type (str): Task type (e.g., 'text', 'circle', etc.)
            task_params (dict): Task parameters
            
        Returns:
            bool: True if task was assigned, False otherwise
        """
        with self.lock:
            if robot_id not in self.robots:
                rospy.logwarn(f"Robot {robot_id} not found")
                return False
            
            # Check if robot is idle
            if self.robot_status.get(robot_id) != 'idle':
                rospy.logwarn(f"Robot {robot_id} is not idle (status: {self.robot_status.get(robot_id)})")
                return False
            
            # Update robot status
            self.robot_status[robot_id] = 'working'
            
            # Format command for the font drawer
            robot_name = f"robot_{robot_id}"
            
            # Build command based on task type
            if task_type == 'text':
                text = task_params.get('text', '')
                scale = task_params.get('scale', 1.0)
                command = f"{robot_name}:text:{text}:scale={scale}"
            
            elif task_type in ['circle', 'square', 'rectangle', 'star', 'heart', 'spiral', 'triangle', 'hexagon', 'pentagon', 'octagon']:
                command = f"{robot_name}:{task_type}"
                
                # Add parameters
                params = []
                for key, value in task_params.items():
                    if key != 'task_type':
                        params.append(f"{key}={value}")
                
                if params:
                    command += ":" + ",".join(params)
            
            elif task_type == 'letter':
                letter = task_params.get('letter', 'A')
                scale = task_params.get('scale', 1.0)
                command = f"{robot_name}:{letter}:scale={scale}"
            
            elif task_type == 'combined':
                elements = task_params.get('elements', [])
                if not elements:
                    rospy.logwarn("Combined task requires elements")
                    self.robot_status[robot_id] = 'idle'
                    return False
                
                command = f"{robot_name}:combined:{','.join(elements)}"
                
                # Add scale if provided
                if 'scale' in task_params:
                    command += f":scale={task_params['scale']}"
            
            else:
                rospy.logwarn(f"Unknown task type: {task_type}")
                self.robot_status[robot_id] = 'idle'
                return False
            
            # Send the command to the font drawer
            self.drawer_cmd_pub.publish(command)
            
            # Publish status update to robot's topic
            start_msg = json.dumps({
                'task_type': task_type,
                'parameters': task_params
            })
            if robot_id in self.robots:
                robot_ns = self.robots[robot_id]
                rospy.Publisher(f"{robot_ns}/start", String, queue_size=10).publish(start_msg)
            
            rospy.loginfo(f"Assigned task {task_type} to robot {robot_id}")
            return True
    
    def run(self):
        """Run the robot manager until shutdown"""
        rospy.loginfo("Robot Manager is running")
        rospy.spin()

if __name__ == "__main__":
    try:
        manager = RobotManager()
        manager.run()
    except rospy.ROSInterruptException:
        pass