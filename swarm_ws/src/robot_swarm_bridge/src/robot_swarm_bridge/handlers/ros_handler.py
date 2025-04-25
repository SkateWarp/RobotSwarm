import rospy
from std_msgs.msg import String
import json
import threading

class ROSHandler:
    def __init__(self, robot_id, config, status_callback, sensor_callback, finish_task_callback, cancel_task_callback, start_task_callback):
        self.robot_id = robot_id
        self.config = config
        self.status_callback = status_callback
        self.sensor_callback = sensor_callback
        self.finish_task_callback = finish_task_callback
        self.cancel_task_callback = cancel_task_callback
        self.start_task_callback = start_task_callback
        
        # Message buffers for deduplication
        self.latest_status = None
        self.latest_sensor_data = None
        self.latest_task_data = None
        
        # Buffer locks
        self.status_lock = threading.Lock()
        self.sensor_lock = threading.Lock()
        self.task_lock = threading.Lock()
        
        # Setup publishers and subscribers
        self.setup_ros_communication()
        
        # Start timer for processing buffered messages (every 1 second)
        self.timer = rospy.Timer(rospy.Duration(1.0), self.process_buffers)

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
        """Buffer status messages from ROS"""
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
            
            # Store only the latest status
            with self.status_lock:
                self.latest_status = normalized_status
                
        except Exception as e:
            rospy.logerr(f"Error in status callback: {e}")

    def sensor_subscriber_callback(self, msg):
        """Buffer sensor data from ROS"""
        try:
            sensor_data = json.loads(msg.data)
            
            # Store only the latest sensor data
            with self.sensor_lock:
                self.latest_sensor_data = sensor_data
                
        except Exception as e:
            rospy.logerr(f"Error in sensor callback: {e}")
    
    def start_task_subscriber_callback(self, msg):
        """Buffer task data from ROS"""
        try:
            task_data = json.loads(msg.data)
            
            # Store task data
            with self.task_lock:
                self.latest_task_data = task_data
                
        except Exception as e:
            rospy.logerr(f"Error in start task callback: {e}")

    def finish_task_subscriber_callback(self, msg):
        """Handle finish task log from ROS"""
        try:
            # Just trigger this event - these should be infrequent
            self.finish_task_callback(self.robot_id)
        except Exception as e:
            rospy.logerr(f"Error in finish task callback: {e}")

    def cancel_task_subscriber_callback(self, msg):
        """Handle cancel task log from ROS"""
        try:
            # Just trigger this event - these should be infrequent
            self.cancel_task_callback(self.robot_id)
        except Exception as e:
            rospy.logerr(f"Error in cancel task callback: {e}")

    def process_buffers(self, event):
        """Process buffered messages at a fixed rate"""
        # Process status buffer
        with self.status_lock:
            if self.latest_status is not None:
                self.status_callback(self.robot_id, self.latest_status)
                self.latest_status = None
        
        # Process sensor data buffer
        with self.sensor_lock:
            if self.latest_sensor_data is not None:
                self.sensor_callback(self.robot_id, self.latest_sensor_data)
                self.latest_sensor_data = None
        
        # Process task data buffer
        with self.task_lock:
            if self.latest_task_data is not None:
                self.start_task_callback(self.robot_id, self.latest_task_data)
                self.latest_task_data = None

    def cleanup(self):
        """Cleanup ROS connections"""
        self.command_pub.unregister()
        self.timer.shutdown()