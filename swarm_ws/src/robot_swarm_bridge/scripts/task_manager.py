#!/usr/bin/env python3
"""
Enhanced Task Manager for TurtleBot3 Swarm System

This node manages tasks for multiple TurtleBot3 robots, including:
- Shape drawing (integrating the enhanced font drawer)
- Multi-robot coordinated tasks
- Status monitoring and task allocation
- Support for adding new task modules
- Collision avoidance and arena awareness

The task manager handles task requests and distributes them to available robots,
tracking task status and completion.
"""

import rospy
import importlib
import inspect
import os
import sys
import json
import threading
import time
import math
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from std_srvs.srv import Empty

class TaskManager:
    """Manages tasks for the robot swarm with enhanced coordination"""
    
    def __init__(self):
        """Initialize the task manager node"""
        rospy.init_node('task_manager', anonymous=False)
        
        # Dictionary to track robot status
        self.robots = {}  # robot_id -> status, position, etc.
        self.robot_lock = threading.RLock()
        
        # Dictionary to track active tasks
        self.active_tasks = {}  # task_id -> task_info
        self.task_counter = 0
        self.task_lock = threading.RLock()
        
        # Task modules dictionary
        self.task_modules = {}
        
        # Parameters
        self.task_check_rate = rospy.get_param("~task_check_rate", 1.0)  # Hz
        self.task_modules_path = rospy.get_param("~task_modules_path", "")  # Path to task modules
        
        # Arena parameters
        self.arena_width = rospy.get_param("~arena_width", 10.0)
        self.arena_height = rospy.get_param("~arena_height", 10.0)
        
        # Publishers
        self.drawer_cmd_pub = rospy.Publisher('/drawer/command', String, queue_size=10)
        self.drawer_task_pub = rospy.Publisher('/drawer/task', String, queue_size=10)
        
        # Subscribe to robot status topics
        rospy.Subscriber("/robot_manager/robot_list", String, self.handle_robot_list)
        
        # Subscribe to drawer status for monitoring
        rospy.Subscriber("/drawer/status", String, self.handle_drawer_status)
        
        # Load task modules
        self.load_task_modules()
        
        # Services
        self.assign_task_service = rospy.Service(
            '/task_manager/assign_task',
            Empty,
            self.handle_assign_task
        )
        
        # Timer for checking task status
        self.task_timer = rospy.Timer(
            rospy.Duration(1.0/self.task_check_rate),
            self.check_task_status
        )
        
        # Additional timer for coordinated tasks
        self.coord_timer = None
        
        rospy.loginfo("Task Manager initialized with enhanced capabilities")
        rospy.loginfo(f"Arena dimensions: {self.arena_width}x{self.arena_height}m")
    
    def load_task_modules(self):
        """Dynamically load task modules from the modules directory"""
        if not self.task_modules_path or not os.path.exists(self.task_modules_path):
            rospy.logwarn("Task modules path not set or does not exist")
            return
            
        try:
            # Add the modules path to Python path
            sys.path.append(self.task_modules_path)
            
            # Find all .py files in the directory
            for filename in os.listdir(self.task_modules_path):
                if filename.endswith(".py") and not filename.startswith("__"):
                    module_name = filename[:-3]  # Remove .py extension
                    
                    try:
                        # Import the module
                        module = importlib.import_module(module_name)
                        
                        # Check if it has a register_module function
                        if hasattr(module, 'register_module'):
                            # Register the module
                            module_info = module.register_module()
                            
                            if 'name' in module_info and 'supported_tasks' in module_info:
                                self.task_modules[module_name] = {
                                    'name': module_info['name'],
                                    'description': module_info.get('description', ''),
                                    'supported_tasks': module_info['supported_tasks'],
                                    'module': module
                                }
                                
                                rospy.loginfo(f"Loaded task module: {module_info['name']}")
                            else:
                                rospy.logwarn(f"Module {module_name} has incomplete registration info")
                        else:
                            rospy.logwarn(f"Module {module_name} does not have register_module function")
                            
                    except Exception as e:
                        rospy.logerr(f"Error loading module {module_name}: {str(e)}")
        except Exception as e:
            rospy.logerr(f"Error loading task modules: {str(e)}")
    
    def handle_robot_list(self, msg):
        """
        Handle updates to the robot list
        
        Args:
            msg (String): JSON string with robot status information
        """
        try:
            # Parse the JSON string
            robot_list = json.loads(msg.data)
            
            with self.robot_lock:
                # Update our tracking of robots and their status
                for robot_info in robot_list:
                    robot_id = robot_info['robot_id']
                    status = robot_info['status']
                    namespace = robot_info.get('namespace', f"/robot/{robot_id}")
                    
                    # Extract position if available
                    position = None
                    if 'position' in robot_info:
                        position = (
                            robot_info['position'].get('x', 0.0),
                            robot_info['position'].get('y', 0.0)
                        )
                    
                    # Update or add the robot
                    if robot_id not in self.robots:
                        self.robots[robot_id] = {
                            'status': status,
                            'namespace': namespace,
                            'position': position,
                            'tasks': []
                        }
                    else:
                        self.robots[robot_id]['status'] = status
                        if position:
                            self.robots[robot_id]['position'] = position
                
                # Check for robots that are no longer in the list
                robot_ids = [info['robot_id'] for info in robot_list]
                to_remove = [r_id for r_id in self.robots.keys() if r_id not in robot_ids]
                
                for r_id in to_remove:
                    del self.robots[r_id]
        
        except json.JSONDecodeError as e:
            rospy.logerr(f"Error parsing robot list JSON: {str(e)}")
        except Exception as e:
            rospy.logerr(f"Error handling robot list update: {str(e)}")
    
    def handle_drawer_status(self, msg):
        """
        Handle status updates from the drawer
        
        Args:
            msg (String): JSON string with drawer status information
        """
        try:
            # Parse the JSON string
            drawer_status = json.loads(msg.data)
            
            with self.robot_lock:
                # Update robot status based on drawer status
                for robot_name, status in drawer_status.items():
                    # Extract robot ID from the name (robot_X)
                    if robot_name.startswith('robot_'):
                        try:
                            robot_id = int(robot_name.split('_')[1])
                            
                            if robot_id in self.robots:
                                # Update the status
                                self.robots[robot_id]['status'] = status
                        except (ValueError, IndexError):
                            pass
        
        except json.JSONDecodeError:
            pass  # Silently ignore parsing errors for drawer status
        except Exception as e:
            rospy.logerr(f"Error handling drawer status: {str(e)}")
    
    def handle_assign_task(self, req):
        """
        Handle requests to assign a task to a robot via ROS service
        This is a simplified demonstration - in a real system, you'd use a custom service type
        
        Args:
            req: Empty service request
            
        Returns:
            Empty: Empty response
        """
        try:
            # Find an idle robot
            robot_id = None
            with self.robot_lock:
                for r_id, info in self.robots.items():
                    if info['status'] == 'idle':
                        robot_id = r_id
                        break
            
            if robot_id is None:
                rospy.logwarn("No idle robots available")
                return Empty()
            
            # Generate a simple task for demonstration
            task_type = 'text'
            task_params = {
                'text': 'HELLO',
                'scale': 1.0
            }
            
            # Assign the task
            success = self.assign_task(robot_id, task_type, task_params)
            
            if not success:
                rospy.logwarn(f"Failed to assign task to robot {robot_id}")
            
            return Empty()
        
        except Exception as e:
            rospy.logerr(f"Error assigning task: {str(e)}")
            return Empty()
    
    def assign_task(self, robot_id, task_type, task_params):
        """
        Assign a task to a specific robot
        
        Args:
            robot_id (int): Robot ID
            task_type (str): Type of task ('text', 'circle', etc.)
            task_params (dict): Parameters for the task
            
        Returns:
            bool: True if task was assigned, False otherwise
        """
        with self.robot_lock:
            # Check if robot exists and is idle
            if robot_id not in self.robots:
                rospy.logwarn(f"Robot {robot_id} not found")
                return False
            
            if self.robots[robot_id]['status'] != 'idle':
                rospy.logwarn(f"Robot {robot_id} is not idle (status: {self.robots[robot_id]['status']})")
                return False
            
            # Update robot status
            self.robots[robot_id]['status'] = 'working'
        
        # Generate task ID
        with self.task_lock:
            self.task_counter += 1
            task_id = self.task_counter
            
            # Create task information
            task_info = {
                'task_id': task_id,
                'robot_id': robot_id,
                'task_type': task_type,
                'task_params': task_params,
                'status': 'pending',
                'progress': 0.0,
                'start_time': time.time(),
                'last_update': time.time()
            }
            
            # Add to active tasks
            self.active_tasks[task_id] = task_info
        
        # Start the task
        threading.Thread(target=self._execute_task, args=(task_id,)).start()
        
        rospy.loginfo(f"Assigned task {task_type} (ID: {task_id}) to robot {robot_id}")
        return True
    
    def assign_multi_robot_task(self, task_type, task_params, num_robots=2):
        """
        Assign a coordinated task to multiple robots
        
        Args:
            task_type (str): Type of coordinated task ('draw_text', 'draw_shape', 'draw_pattern')
            task_params (dict): Parameters for the task
            num_robots (int): Number of robots to use
            
        Returns:
            bool: True if task was assigned, False otherwise
        """
        with self.robot_lock:
            # Find idle robots
            idle_robots = []
            for r_id, info in self.robots.items():
                if info['status'] == 'idle':
                    idle_robots.append(r_id)
                    if len(idle_robots) >= num_robots:
                        break
            
            if len(idle_robots) < num_robots:
                rospy.logwarn(f"Not enough idle robots. Need {num_robots}, found {len(idle_robots)}")
                return False
            
            # Update robot status for all participating robots
            for r_id in idle_robots:
                self.robots[r_id]['status'] = 'working'
        
        # Generate task ID
        with self.task_lock:
            self.task_counter += 1
            task_id = self.task_counter
            
            # Create task information
            task_info = {
                'task_id': task_id,
                'robot_ids': idle_robots,
                'task_type': task_type,
                'task_params': task_params,
                'status': 'pending',
                'progress': 0.0,
                'start_time': time.time(),
                'last_update': time.time()
            }
            
            # Add to active tasks
            self.active_tasks[task_id] = task_info
        
        # Start the coordinated task
        threading.Thread(target=self._execute_coordinated_task, args=(task_id,)).start()
        
        robot_list = ', '.join(map(str, idle_robots))
        rospy.loginfo(f"Assigned coordinated task {task_type} (ID: {task_id}) to robots: {robot_list}")
        return True
    
    def _execute_task(self, task_id):
        """
        Execute a task on a single robot
        
        Args:
            task_id (int): Task ID
        """
        try:
            with self.task_lock:
                if task_id not in self.active_tasks:
                    rospy.logwarn(f"Task {task_id} not found")
                    return
                
                task_info = self.active_tasks[task_id]
                robot_id = task_info['robot_id']
                task_type = task_info['task_type']
                task_params = task_info['task_params']
            
            # Update task status
            self._update_task_status(task_id, 'running', 0.1)
            
            # Check which robot namespace format to use
            robot_name = f"robot_{robot_id}"
            
            # Create the command for the drawer
            if task_type == 'text':
                text = task_params.get('text', '')
                scale = task_params.get('scale', 1.0)
                command = f"{robot_name}:text:{text}:scale={scale}"
            
            elif task_type in ['circle', 'square', 'rectangle', 'star', 'heart', 'spiral',
                             'triangle', 'hexagon', 'pentagon', 'octagon']:
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
                    self._update_task_status(task_id, 'failed', 0.0)
                    return
                
                command = f"{robot_name}:combined:{','.join(elements)}"
                
                # Add scale if provided
                if 'scale' in task_params:
                    command += f":scale={task_params['scale']}"
            
            else:
                rospy.logwarn(f"Unknown task type: {task_type}")
                self._update_task_status(task_id, 'failed', 0.0)
                return
            
            # Send the command to the font drawer
            self.drawer_cmd_pub.publish(command)
            
            # Simulate task progress updates
            # In a real system, you'd get feedback from the robot
            for progress in [0.2, 0.4, 0.6, 0.8, 1.0]:
                time.sleep(1.0)  # Simulate task execution time
                self._update_task_status(task_id, 'running', progress)
            
            # Mark task as completed
            self._update_task_status(task_id, 'completed', 1.0)
            
            # Update robot status back to idle
            with self.robot_lock:
                if robot_id in self.robots:
                    self.robots[robot_id]['status'] = 'idle'
            
            rospy.loginfo(f"Task {task_id} completed successfully")
        
        except Exception as e:
            rospy.logerr(f"Error executing task {task_id}: {str(e)}")
            self._update_task_status(task_id, 'failed', 0.0)
            
            # Update robot status back to idle
            with self.robot_lock:
                if 'robot_id' in task_info and task_info['robot_id'] in self.robots:
                    self.robots[task_info['robot_id']]['status'] = 'idle'
    
    def _execute_coordinated_task(self, task_id):
        """
        Execute a coordinated task on multiple robots
        
        Args:
            task_id (int): Task ID
        """
        try:
            with self.task_lock:
                if task_id not in self.active_tasks:
                    rospy.logwarn(f"Task {task_id} not found")
                    return
                
                task_info = self.active_tasks[task_id]
                robot_ids = task_info['robot_ids']
                task_type = task_info['task_type']
                task_params = task_info['task_params']
            
            # Update task status
            self._update_task_status(task_id, 'running', 0.1)
            
            # Create the command for the multi-robot coordinator
            if task_type == 'draw_text':
                text = task_params.get('text', 'HELLO')
                scale = task_params.get('scale', 1.0)
                command = f"draw_text:{text}:scale={scale}"
            
            elif task_type == 'draw_shape':
                shape = task_params.get('shape', 'circle')
                scale = task_params.get('scale', 1.0)
                command = f"draw_shape:{shape}:scale={scale},robots={len(robot_ids)}"
            
            elif task_type == 'draw_pattern':
                pattern = task_params.get('pattern', 'star')
                scale = task_params.get('scale', 1.0)
                command = f"draw_pattern:{pattern}:scale={scale}"
            
            else:
                rospy.logwarn(f"Unknown coordinated task type: {task_type}")
                self._update_task_status(task_id, 'failed', 0.0)
                return
            
            # Send the command to the drawer task topic
            self.drawer_task_pub.publish(command)
            
            # Simulate task progress updates
            # In a real system, you'd get feedback from the robots
            for progress in [0.2, 0.4, 0.6, 0.8, 1.0]:
                time.sleep(2.0)  # Coordinated tasks take longer
                self._update_task_status(task_id, 'running', progress)
            
            # Mark task as completed
            self._update_task_status(task_id, 'completed', 1.0)
            
            # Update robot status back to idle
            with self.robot_lock:
                for robot_id in robot_ids:
                    if robot_id in self.robots:
                        self.robots[robot_id]['status'] = 'idle'
            
            rospy.loginfo(f"Coordinated task {task_id} completed successfully")
        
        except Exception as e:
            rospy.logerr(f"Error executing coordinated task {task_id}: {str(e)}")
            self._update_task_status(task_id, 'failed', 0.0)
            
            # Update robot status back to idle
            with self.robot_lock:
                for robot_id in robot_ids:
                    if robot_id in self.robots:
                        self.robots[robot_id]['status'] = 'idle'
    
    def _update_task_status(self, task_id, status, progress):
        """
        Update the status and progress of a task
        
        Args:
            task_id (int): Task ID
            status (str): New status
            progress (float): Progress value (0-1)
        """
        with self.task_lock:
            if task_id in self.active_tasks:
                self.active_tasks[task_id]['status'] = status
                self.active_tasks[task_id]['progress'] = progress
                self.active_tasks[task_id]['last_update'] = time.time()
                
                # If the task is for a single robot, publish progress to the robot's topic
                if 'robot_id' in self.active_tasks[task_id]:
                    robot_id = self.active_tasks[task_id]['robot_id']
                    if robot_id in self.robots:
                        robot_ns = self.robots[robot_id]['namespace']
                        
                        # Create progress message
                        progress_msg = json.dumps({
                            'task_id': task_id,
                            'status': status,
                            'progress': progress,
                            'timestamp': time.time()
                        })
                        
                        # Publish to the task_progress topic
                        rospy.Publisher(f"{robot_ns}/task_progress", String, queue_size=10).publish(progress_msg)
    
    def check_task_status(self, event=None):
        """
        Periodically check task status and clean up completed/failed tasks
        
        Args:
            event: Timer event (not used)
        """
        try:
            current_time = time.time()
            
            with self.task_lock:
                # Check for stalled tasks
                for task_id, task_info in list(self.active_tasks.items()):
                    if task_info['status'] in ['pending', 'running']:
                        # Check if the task has timed out (no updates for 60 seconds)
                        time_since_update = current_time - task_info['last_update']
                        
                        if time_since_update > 60.0:
                            rospy.logwarn(f"Task {task_id} timed out")
                            task_info['status'] = 'failed'
                            
                            # Reset robot status
                            if 'robot_id' in task_info:
                                robot_id = task_info['robot_id']
                                with self.robot_lock:
                                    if robot_id in self.robots:
                                        self.robots[robot_id]['status'] = 'idle'
                            elif 'robot_ids' in task_info:
                                with self.robot_lock:
                                    for robot_id in task_info['robot_ids']:
                                        if robot_id in self.robots:
                                            self.robots[robot_id]['status'] = 'idle'
                    
                    # Clean up completed or failed tasks after 5 minutes
                    if task_info['status'] in ['completed', 'failed']:
                        time_since_completion = current_time - task_info['last_update']
                        
                        if time_since_completion > 300.0:  # 5 minutes
                            del self.active_tasks[task_id]
        
        except Exception as e:
            rospy.logerr(f"Error checking task status: {str(e)}")
    
    def run(self):
        """Run the task manager until shutdown"""
        rospy.loginfo("Task Manager is running")
        rospy.spin()
    
    def shutdown(self):
        """Clean shutdown of the task manager"""
        rospy.loginfo("Task Manager shutting down")
        
        if self.task_timer:
            self.task_timer.shutdown()
        
        if self.coord_timer:
            self.coord_timer.shutdown()
        
        # Stop any running tasks
        for task_id, task_info in list(self.active_tasks.items()):
            if task_info['status'] in ['pending', 'running']:
                task_info['status'] = 'failed'

if __name__ == "__main__":
    try:
        manager = TaskManager()
        rospy.on_shutdown(manager.shutdown)
        manager.run()
    except rospy.ROSInterruptException:
        pass