import rospy
from std_msgs.msg import String
import json
import threading
import time

class ROSHandler:
    def __init__(self, robot_id, config, status_callback, sensor_callback, finish_task_callback, cancel_task_callback, start_task_callback):
        self.robot_id = robot_id
        self.config = config
        self.status_callback = status_callback
        self.sensor_callback = sensor_callback
        self.finish_task_callback = finish_task_callback
        self.cancel_task_callback = cancel_task_callback
        self.start_task_callback = start_task_callback
        
        # Buffer for latest messages only - no duplicates
        self.latest_messages = {
            'status': None,
            'sensor': None,
            'task': None
        }
        
        # Locks for thread safety
        self.message_lock = threading.Lock()
        
        # Rate limiting - strict enforcement
        self.next_allowed_time = {
            'status': 0,
            'sensor': 0,
            'task': 0,
            'finish_task': 0,
            'cancel_task': 0
        }
        
        # Rate limiting interval in seconds (changed from 1 to 5)
        self.rate_limit_interval = 5.0
        
        # Setup publishers and subscribers
        self.setup_ros_communication()
        
        # Start the rate-limited sender thread
        self.running = True
        self.sender_thread = threading.Thread(target=self._process_messages)
        self.sender_thread.daemon = True
        self.sender_thread.start()

    def setup_ros_communication(self):
        """Setup ROS publishers and subscribers"""
        # Publishers
        self.command_pub = rospy.Publisher(
            f'/robot/{self.robot_id}/commands',
            String,
            queue_size=10
        )
        
        # Subscribers
        rospy.Subscriber(
            f'/robot/{self.robot_id}/status',
            String,
            self.status_subscriber_callback
        )
        
        rospy.Subscriber(
            f'/robot/{self.robot_id}/sensor_data',
            String,
            self.sensor_subscriber_callback
        )

        rospy.Subscriber(
            f'/robot/{self.robot_id}/task/start',
            String,
            self.start_task_subscriber_callback
        )

        rospy.Subscriber(
            f'/robot/{self.robot_id}/task/finish',
            String,
            self.finish_task_subscriber_callback
        )

        rospy.Subscriber(
            f'/robot/{self.robot_id}/task/cancel',
            String,
            self.cancel_task_subscriber_callback
        )

    def publish_command(self, command_data):
        """Publish command to ROS"""
        try:
            msg = String()
            msg.data = json.dumps(command_data)
            self.command_pub.publish(msg)
        except Exception as e:
            rospy.logerr(f"Error publishing command: {e}")

    def status_subscriber_callback(self, msg):
        """Handle status messages from ROS"""
        try:
            status = msg.data
            # Convert status to match C# enum exactly
            status_mapping = {
                "working": "Working",
                "idle": "Idle",
                "Working": "Working",
                "Idle": "Idle"
            }
            
            normalized_status = status_mapping.get(status.strip('"').strip("'"), status)
            
            # Store in buffer
            with self.message_lock:
                self.latest_messages['status'] = normalized_status
                
        except Exception as e:
            rospy.logerr(f"Error in status callback: {e}")

    def sensor_subscriber_callback(self, msg):
        """Handle sensor data from ROS"""
        try:
            sensor_data = json.loads(msg.data)
            
            # Store in buffer
            with self.message_lock:
                self.latest_messages['sensor'] = sensor_data
                
        except Exception as e:
            rospy.logerr(f"Error in sensor callback: {e}")
    
    def start_task_subscriber_callback(self, msg):
        """Handle task data from ROS"""
        try:
            task_data = json.loads(msg.data)
            
            # Store in buffer
            with self.message_lock:
                self.latest_messages['task'] = task_data
                
        except Exception as e:
            rospy.logerr(f"Error in start task callback: {e}")

    def finish_task_subscriber_callback(self, msg):
        """Handle finish task log from ROS"""
        try:
            # For event-based messages, we need to check rate limiting directly
            current_time = time.time()
            if current_time >= self.next_allowed_time.get('finish_task', 0):
                self.next_allowed_time['finish_task'] = current_time + self.rate_limit_interval
                self.finish_task_callback(self.robot_id)
        except Exception as e:
            rospy.logerr(f"Error in finish task callback: {e}")

    def cancel_task_subscriber_callback(self, msg):
        """Handle cancel task log from ROS"""
        try:
            # For event-based messages, we need to check rate limiting directly
            current_time = time.time()
            if current_time >= self.next_allowed_time.get('cancel_task', 0):
                self.next_allowed_time['cancel_task'] = current_time + self.rate_limit_interval
                self.cancel_task_callback(self.robot_id)
        except Exception as e:
            rospy.logerr(f"Error in cancel task callback: {e}")

    def _process_messages(self):
        """Thread that processes messages at a controlled rate"""
        while self.running:
            try:
                current_time = time.time()
                
                # Process status messages
                if current_time >= self.next_allowed_time.get('status', 0):
                    with self.message_lock:
                        status_data = self.latest_messages.get('status')
                        if status_data is not None:
                            self.next_allowed_time['status'] = current_time + self.rate_limit_interval
                            self.latest_messages['status'] = None
                            rospy.loginfo(f"[RATE_LIMITED] Sending status update for robot {self.robot_id} (every {self.rate_limit_interval}s)")
                            self.status_callback(self.robot_id, status_data)
                
                # Process sensor messages
                if current_time >= self.next_allowed_time.get('sensor', 0):
                    with self.message_lock:
                        sensor_data = self.latest_messages.get('sensor')
                        if sensor_data is not None:
                            self.next_allowed_time['sensor'] = current_time + self.rate_limit_interval
                            self.latest_messages['sensor'] = None
                            rospy.loginfo(f"[RATE_LIMITED] Sending sensor data for robot {self.robot_id} (every {self.rate_limit_interval}s)")
                            self.sensor_callback(self.robot_id, sensor_data)
                
                # Process task messages
                if current_time >= self.next_allowed_time.get('task', 0):
                    with self.message_lock:
                        task_data = self.latest_messages.get('task')
                        if task_data is not None:
                            self.next_allowed_time['task'] = current_time + self.rate_limit_interval
                            self.latest_messages['task'] = None
                            rospy.loginfo(f"[RATE_LIMITED] Sending task data for robot {self.robot_id} (every {self.rate_limit_interval}s)")
                            self.start_task_callback(self.robot_id, task_data)
                
                # Sleep for a short time to avoid high CPU usage
                time.sleep(0.1)
                
            except Exception as e:
                rospy.logerr(f"Error in message processing thread: {e}")

    def cleanup(self):
        """Cleanup ROS connections"""
        self.running = False
        if self.sender_thread.is_alive():
            self.sender_thread.join(timeout=1.0)
        self.command_pub.unregister()