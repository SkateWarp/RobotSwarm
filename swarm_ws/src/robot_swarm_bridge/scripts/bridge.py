#!/usr/bin/env python3
"""ROS executable wrapper for the installed Python bridge package."""

from robot_swarm_bridge.bridge import RobotSwarmBridge


if __name__ == "__main__":
    RobotSwarmBridge().run()
