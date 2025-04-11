#!/usr/bin/env python3
import rospy
import tf2_ros
import geometry_msgs.msg
from std_msgs.msg import String
import json

class RobotTFManager:
    def __init__(self):
        rospy.init_node('robot_tf_manager')
        self.broadcaster = tf2_ros.StaticTransformBroadcaster()
        self.robots = set()
        
        # Create world to map transform
        t = geometry_msgs.msg.TransformStamped()
        t.header.stamp = rospy.Time.now()
        t.header.frame_id = "world"
        t.child_frame_id = "map"
        t.transform.rotation.w = 1.0
        self.broadcaster.sendTransform([t])
        
        # Subscribe to robot list
        rospy.Subscriber("/robot_manager/robot_list", String, self.update_robots)
        rospy.loginfo("TF Manager running")
        rospy.spin()
    
    def update_robots(self, msg):
        try:
            robot_list = json.loads(msg.data)
            transforms = []
            
            for robot_info in robot_list:
                robot_id = robot_info['robot_id']
                
                # Create map to robot transform
                t = geometry_msgs.msg.TransformStamped()
                t.header.stamp = rospy.Time.now()
                t.header.frame_id = "map"
                t.child_frame_id = f"robot{robot_id}/base_footprint"
                
                if 'position' in robot_info:
                    t.transform.translation.x = robot_info['position'].get('x', 0.0)
                    t.transform.translation.y = robot_info['position'].get('y', 0.0)
                t.transform.rotation.w = 1.0
                transforms.append(t)
            
            if transforms:
                self.broadcaster.sendTransform(transforms)
                
        except Exception as e:
            rospy.logerr(f"TF update error: {e}")

if __name__ == '__main__':
    try:
        RobotTFManager()
    except rospy.ROSInterruptException:
        pass