import os
import yaml
import rospy

def load_config():
    """Load configuration from YAML file"""
    try:
        # Try to get config file path from ROS parameter
        config_file = rospy.get_param('~config_file', None)
        
        if not config_file or not os.path.exists(config_file):
            # Fallback to default config path
            package_path = os.path.dirname(os.path.dirname(os.path.dirname(__file__)))
            config_file = os.path.join(package_path, 'config', 'config.yaml')

        with open(config_file, 'r') as f:
            config = yaml.safe_load(f)
            
        return config
    except Exception as e:
        rospy.logerr(f"Error loading config: {e}")
        # Return default configuration
        return {
            'signalr': {
                'url': 'wss://localhost:44337/hubs/robot',
                'reconnect_delay': 1,
                'max_reconnect_delay': 60
            },
            'ros': {
                'topics': {
                    'status': '/robot_{id}/status',
                    'sensor_data': '/robot_{id}/sensor_data',
                    'commands': '/robot_{id}/commands'
                }
            },
            'logging': {
                'level': 'INFO',
                'file': 'robot_bridge.log'
            }
        }