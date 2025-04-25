#!/usr/bin/env python3

import rospy
import json
from std_msgs.msg import String

def sensor_callback(msg):
    """Callback function to process and display received sensor data"""
    try:
        # Parse the JSON data
        sensor_data = json.loads(msg.data)
        
        # Print received data in a formatted way
        rospy.loginfo("\n=== RECEIVED SENSOR DATA ===")
        rospy.loginfo(f"Robot: {sensor_data.get('name', 'Unknown')}")
        rospy.loginfo(f"Ticks: Left={sensor_data.get('left_ticks', 'N/A')}, Right={sensor_data.get('right_ticks', 'N/A')}")
        rospy.loginfo(f"Diff: Left={sensor_data.get('left_diff', 'N/A')}, Right={sensor_data.get('right_diff', 'N/A')}")
        rospy.loginfo(f"Dist: Left={sensor_data.get('left_dist', 'N/A')}, Right={sensor_data.get('right_dist', 'N/A')}")
        rospy.loginfo(f"Speed: Left={sensor_data.get('left_speed', 'N/A')}, Right={sensor_data.get('right_speed', 'N/A')}")
        rospy.loginfo(f"Filtered Speed: Left={sensor_data.get('left_speed_filtered', 'N/A')}, Right={sensor_data.get('right_speed_filtered', 'N/A')}")
        rospy.loginfo(f"Timestep: {sensor_data.get('timestep', 'N/A')}")
        rospy.loginfo("===========================\n")
        
        # Verify the data format matches the expected format
        expected_keys = [
            "left_ticks", "right_ticks", 
            "left_diff", "right_diff", 
            "left_dist", "right_dist", 
            "timestep", 
            "left_speed", "right_speed", 
            "left_speed_filtered", "right_speed_filtered", 
            "name"
        ]
        
        missing_keys = [key for key in expected_keys if key not in sensor_data]
        if missing_keys:
            rospy.logwarn(f"Missing expected keys in sensor data: {missing_keys}")
        else:
            rospy.loginfo("Data format verification: PASSED")
            
    except json.JSONDecodeError:
        rospy.logerr(f"Failed to parse JSON data: {msg.data}")
    except Exception as e:
        rospy.logerr(f"Error processing sensor data: {e}")

def main():
    rospy.init_node('sensor_data_tester', anonymous=True)
    
    # Get robot ID from parameter
    robot_id = rospy.get_param('~robot_id', '1')
    
    # Subscribe to the sensor data topic
    rospy.Subscriber(f'/robot/{robot_id}/sensor_data', String, sensor_callback)
    
    rospy.loginfo(f"Sensor data tester started. Listening to /robot/{robot_id}/sensor_data")
    
    # Keep the program running
    rospy.spin()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass