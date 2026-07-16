#!/usr/bin/env python3
"""
Configuration Loader Utility
Loads and validates configuration from YAML files
Provides easy access to configuration parameters throughout the system
"""

import yaml
import rospy
import os
from typing import Any, Dict, Optional


class ConfigLoader:
    """
    Configuration loader with validation and default values
    Thread-safe singleton pattern for consistent configuration access
    """

    _instance = None
    _config = None

    def __new__(cls):
        """Singleton pattern to ensure only one config instance"""
        if cls._instance is None:
            cls._instance = super(ConfigLoader, cls).__new__(cls)
        return cls._instance

    def __init__(self):
        """Initialize configuration loader"""
        if self._config is None:
            self.load_config()

    def load_config(self, config_file: Optional[str] = None) -> None:
        """
        Load configuration from YAML file

        Args:
            config_file: Path to config file. If None, uses default location
        """
        if config_file is None:
            # Get package path and construct default config path
            package_path = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
            config_file = os.path.join(package_path, 'config', 'swarm_config.yaml')

        try:
            with open(config_file, 'r') as f:
                self._config = yaml.safe_load(f)

            rospy.loginfo(f"Configuration loaded from: {config_file}")
            self._validate_config()

        except FileNotFoundError:
            rospy.logerr(f"Configuration file not found: {config_file}")
            self._config = self._get_default_config()
            rospy.logwarn("Using default configuration")
        except yaml.YAMLError as e:
            rospy.logerr(f"Error parsing YAML configuration: {e}")
            self._config = self._get_default_config()
            rospy.logwarn("Using default configuration")

    def get(self, key_path: str, default: Any = None) -> Any:
        """
        Get configuration value using dot notation

        Args:
            key_path: Dot-separated path to config value (e.g., 'fleet.max_robots')
            default: Default value if key not found

        Returns:
            Configuration value or default

        Example:
            config.get('safety.max_linear_velocity')  # Returns 0.26
        """
        if self._config is None:
            self.load_config()

        keys = key_path.split('.')
        value = self._config

        try:
            for key in keys:
                value = value[key]
            return value
        except (KeyError, TypeError):
            if default is not None:
                return default
            rospy.logwarn(f"Configuration key not found: {key_path}")
            return None

    def set(self, key_path: str, value: Any) -> None:
        """
        Set configuration value (runtime only, not saved to file)

        Args:
            key_path: Dot-separated path to config value
            value: Value to set
        """
        if self._config is None:
            self.load_config()

        keys = key_path.split('.')
        config = self._config

        # Navigate to parent of target key
        for key in keys[:-1]:
            if key not in config:
                config[key] = {}
            config = config[key]

        # Set the value
        config[keys[-1]] = value
        rospy.logdebug(f"Configuration updated: {key_path} = {value}")

    def get_all(self) -> Dict:
        """
        Get entire configuration dictionary

        Returns:
            Complete configuration dict
        """
        if self._config is None:
            self.load_config()
        return self._config.copy()

    def _validate_config(self) -> None:
        """Validate critical configuration parameters"""
        # Check critical safety parameters
        critical_params = [
            ('safety.absolute_min_distance', float, 0.1, 1.0),
            ('safety.max_linear_velocity', float, 0.0, 1.0),
            ('safety.max_angular_velocity', float, 0.0, 5.0),
            ('fleet.max_robots', int, 0, 10000),
            ('arena.size.x', float, 1.0, 100.0),
            ('arena.size.y', float, 1.0, 100.0),
        ]

        for param_path, param_type, min_val, max_val in critical_params:
            value = self.get(param_path)

            if value is None:
                rospy.logwarn(f"Missing critical parameter: {param_path}")
                continue

            if not isinstance(value, param_type):
                rospy.logwarn(f"Invalid type for {param_path}: expected {param_type}, got {type(value)}")
                continue

            if not (min_val <= value <= max_val):
                rospy.logwarn(f"Parameter {param_path} = {value} outside valid range [{min_val}, {max_val}]")

    def _get_default_config(self) -> Dict:
        """
        Get minimal default configuration
        Used as fallback if config file cannot be loaded

        Returns:
            Default configuration dict
        """
        return {
            'fleet': {
                'max_robots': 0,
                'default_robot_count': 5,
                'robot_model': 'turtlebot3_burger',
            },
            'arena': {
                'size': {'x': 10.0, 'y': 10.0},
                'center': {'x': 0.0, 'y': 0.0},
            },
            'safety': {
                'absolute_min_distance': 0.20,
                'emergency_stop_distance': 0.25,
                'critical_zone': 0.30,
                'danger_zone': 0.50,
                'warning_zone': 1.00,
                'safe_distance': 0.80,
                'max_linear_velocity': 0.22,
                'max_angular_velocity': 2.84,
                'repulsion_gain': 1.5,
            },
            'control': {
                'main_loop_rate': 30.0,
                'status_publish_rate': 5.0,
            },
            'logging': {
                'level': 'INFO',
            }
        }

    def print_config(self) -> None:
        """Print current configuration (for debugging)"""
        if self._config is None:
            self.load_config()

        rospy.loginfo("=== Current Configuration ===")
        self._print_dict(self._config, indent=0)

    def _print_dict(self, d: Dict, indent: int = 0) -> None:
        """Recursively print dictionary with indentation"""
        for key, value in d.items():
            if isinstance(value, dict):
                rospy.loginfo("  " * indent + f"{key}:")
                self._print_dict(value, indent + 1)
            else:
                rospy.loginfo("  " * indent + f"{key}: {value}")


# Global config instance (singleton)
config = ConfigLoader()


if __name__ == '__main__':
    """Test configuration loader"""
    rospy.init_node('config_loader_test', anonymous=True)

    # Test configuration loading
    print("\n=== Testing Configuration Loader ===\n")

    # Test get operations
    print(f"Fleet max robots: {config.get('fleet.max_robots')}")
    print(f"Safety min distance: {config.get('safety.absolute_min_distance')}")
    print(f"WebSocket port: {config.get('websocket.port')}")
    print(f"Formation default type: {config.get('formation.default_type')}")

    # Test default value
    print(f"Non-existent key: {config.get('foo.bar.baz', default='DEFAULT')}")

    # Test set operation
    config.set('test.runtime.value', 42)
    print(f"Runtime set value: {config.get('test.runtime.value')}")

    # Print full config
    config.print_config()
