#!/usr/bin/env python3
import rospy
from handlers.signalr_handler import SignalRHandler
from handlers.ros_handler import ROSHandler
from utils.config import load_config
from utils.logger import setup_logger
from std_msgs.msg import String

class RobotSwarmBridge:
    def __init__(self):
        # Initialize node
        rospy.init_node('robot_swarm_bridge', anonymous=True)
       
        # Setup logger
        self.logger = setup_logger()
       
        # Load configuration
        self.config = load_config()
       
        # Get parameters
        self.robot_ids = rospy.get_param('~robot_ids', [1, 2, 3, 4, 5])
        self.backend_url = rospy.get_param('~backend_url', 'ws://localhost:44337/hubs/robot')

        self.logger.info(f"Robot IDs: {self.robot_ids}")
        self.logger.info(f"Backend URL: {self.backend_url}")
       
        # Initialize handlers
        self.signalr_handler = SignalRHandler(
            self.backend_url,
            self.robot_ids,
            self.on_command_received
        )
       
        if isinstance(self.robot_ids, int):
            self.ros_handlers = {
                self.robot_ids: ROSHandler(self.robot_ids, self.config, self.on_status_changed, self.on_sensor_data, self.on_finish_task, self.on_cancel_task, self.on_start_task)
            }
        else:
            self.ros_handlers = {
                robot_id: ROSHandler(robot_id, self.config, self.on_status_changed, self.on_sensor_data, self.on_finish_task, self.on_cancel_task, self.on_start_task)
                for robot_id in self.robot_ids
            }

    def on_robots_available(self, robot_ids):
        """Handle available robots from SignalR"""
        self.logger.info(f"Available robots: {robot_ids}")
        
        # /listOfAvailableRobots
        command_pub = rospy.Publisher(
            f'/listOfAvailableRobots',
            String,
            queue_size=10
        )

        command_pub.publish(f"{robot_ids}")

       
        # Initialize ROS handlers for each robot
        for robot_id in robot_ids:
            if robot_id not in self.ros_handlers:
                self.ros_handlers[robot_id] = ROSHandler(
                    robot_id,
                    self.config,
                    self.on_status_changed,
                    self.on_sensor_data,
                    self.on_finish_task,
                    self.on_cancel_task,
                    self.on_start_task
                )
                rospy.loginfo(f"Initialized ROS handler for robot {robot_id}")
            else:
                rospy.loginfo(f"ROS handler already exists for robot {robot_id}")
    
    def on_command_received(self, robot_id, command_data):
        """Handle commands received from SignalR"""
        if robot_id in self.ros_handlers:
            self.ros_handlers[robot_id].publish_command(command_data)
        else:
            self.logger.warning(f"Received command for unknown robot {robot_id}")
    
    def on_status_changed(self, robot_id, status):
        """Handle robot status changes from ROS"""
        rospy.loginfo(f"Robot {robot_id} status changed to {status}")
        self.signalr_handler.send_status_update(robot_id, status)
    
    def on_sensor_data(self, robot_id, sensor_data):
        """Handle sensor data from ROS"""
        self.signalr_handler.send_sensor_reading(robot_id, sensor_data)
   
    def on_finish_task(self, robot_id):
        """Handle finish task log from ROS"""
        self.signalr_handler.send_finish_task(robot_id)
   
    def on_cancel_task(self, robot_id):
        """Handle cancel task log from ROS"""
        self.signalr_handler.send_cancel_task(robot_id)
    
    def on_start_task(self, robot_id, task_data):
        """Handle start task log from ROS"""
        self.signalr_handler.send_task(robot_id, task_data)
    
    def run(self):
        """Main run loop"""
        try:
            # Start SignalR connection
            self.signalr_handler.start()
           
            # Keep running until shutdown
            rospy.spin()
           
        except rospy.ROSInterruptException:
            self.logger.info("ROS node interrupted")
        except Exception as e:
            self.logger.error(f"Error in bridge: {e}")
        finally:
            # Cleanup
            self.signalr_handler.stop()
            for handler in self.ros_handlers.values():
                handler.cleanup()

if __name__ == '__main__':
    bridge = RobotSwarmBridge()
    bridge.run()