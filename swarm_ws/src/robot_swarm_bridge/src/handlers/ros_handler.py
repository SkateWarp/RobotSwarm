import rospy
from std_msgs.msg import String
import json

class ROSHandler:
    def __init__(self, robot_id, config, status_callback, sensor_callback):
        self.robot_id = robot_id
        self.config = config
        self.status_callback = status_callback
        self.sensor_callback = sensor_callback
        
        # Setup publishers and subscribers
        self.setup_ros_communication()

    def setup_ros_communication(self):
        """Setup ROS publishers and subscribers"""
        # Publishers
        self.command_pub = rospy.Publisher(
            f'/robot_{self.robot_id}/commands',
            String,
            queue_size=10
        )
        
        # Subscribers
        rospy.Subscriber(
            f'/robot_{self.robot_id}/status',
            String,
            self.status_subscriber_callback
        )
        
        rospy.Subscriber(
            f'/robot_{self.robot_id}/sensor_data',
            String,
            self.sensor_subscriber_callback
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
            status_data = json.loads(msg.data)
            self.status_callback(self.robot_id, status_data)
        except Exception as e:
            rospy.logerr(f"Error in status callback: {e}")

    def sensor_subscriber_callback(self, msg):
        """Handle sensor data from ROS"""
        try:
            sensor_data = json.loads(msg.data)
            self.sensor_callback(self.robot_id, sensor_data)
        except Exception as e:
            rospy.logerr(f"Error in sensor callback: {e}")

    def cleanup(self):
        """Cleanup ROS connections"""
        # Unregister publishers and subscribers if needed
        self.command_pub.unregister()