import rospy
from std_msgs.msg import String
import json

class ROSHandler:
    def __init__(self, robot_id, config, status_callback, sensor_callback, finish_task_callback, cancel_task_callback, start_task_callback):
        self.robot_id = robot_id
        self.config = config
        self.status_callback = status_callback
        self.sensor_callback = sensor_callback
        self.finish_task_callback = finish_task_callback
        self.cancel_task_callback = cancel_task_callback
        self.start_task_callback = start_task_callback
        
        # Setup publishers and subscribers
        self.setup_ros_communication()

    def setup_ros_communication(self):
        """Setup ROS publishers and subscribers"""
        # Publishers
        self.command_pub = rospy.Publisher(
            f'/robot/{self.robot_id}/commands',  # Removed underscore after 'robot'
            String,
            queue_size=10
        )
        
        # Subscribers
        rospy.Subscriber(
            f'/robot/{self.robot_id}/status',    # Removed underscore after 'robot'
            String,
            self.status_subscriber_callback
        )
        
        rospy.Subscriber(
            f'/robot/{self.robot_id}/sensor_data',  # Removed underscore after 'robot'
            String,
            self.sensor_subscriber_callback
        )

        rospy.Subscriber(
            f'/robot/{self.robot_id}/task/start',  # Removed underscore after 'robot'
            String,
            self.start_task_subscriber_callback
        )

        rospy.Subscriber(
            f'/robot/{self.robot_id}/task/finish',  # Removed underscore after 'robot'
            String,
            self.finish_task_callback
        )

        rospy.Subscriber(
            f'/robot/{self.robot_id}/task/cancel',  # Removed underscore after 'robot'
            String,
            self.cancel_task_callback
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
        
            self.status_callback(self.robot_id, normalized_status)
        except Exception as e:
            rospy.logerr(f"Error in status callback: {e}")

    def sensor_subscriber_callback(self, msg):
        """Handle sensor data from ROS"""
        try:
            sensor_data = json.loads(msg.data)
            self.sensor_callback(self.robot_id, sensor_data)
        except Exception as e:
            rospy.logerr(f"Error in sensor callback: {e}")
    
    def start_task_subscriber_callback(self, msg):
        """Handle sensor data from ROS"""
        try:
            sensor_data = json.loads(msg.data)
            self.start_task_callback(self.robot_id, sensor_data)
        except Exception as e:
            rospy.logerr(f"Error in sensor callback: {e}")

    def finish_task_callback(self, msg):
        """Handle finish task log from ROS"""
        try:
            self.finish_task_callback(self.robot_id)
        except Exception as e:
            rospy.logerr(f"Error in finish task callback: {e}")

    def cancel_task_callback(self, msg):
        """Handle cancel task log from ROS"""
        try:
            self.cancel_task_callback(self.robot_id)
        except Exception as e:
            rospy.logerr(f"Error in cancel task callback: {e}")

    def cleanup(self):
        """Cleanup ROS connections"""
        self.command_pub.unregister()