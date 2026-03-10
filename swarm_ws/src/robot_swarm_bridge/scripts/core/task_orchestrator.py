#!/usr/bin/env python3
"""
Task Orchestrator - Central command router for swarm system.
Receives commands via /swarm/commands (JSON) from the SignalR bridge,
dispatches to fleet manager and behavior nodes, and publishes status.

This version uses topic-based interfaces (no custom service dependencies)
so it works without needing catkin message generation.
"""

import rospy
import json
import math
import uuid
from typing import Dict, Optional, Any, Callable
from enum import Enum
from datetime import datetime

from std_msgs.msg import String, Bool, Float32, Empty
from geometry_msgs.msg import Twist, Pose
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan


class TaskState(Enum):
    IDLE = "idle"
    INITIALIZING = "initializing"
    RUNNING = "running"
    PAUSED = "paused"
    COMPLETED = "completed"
    FAILED = "failed"
    STOPPED = "stopped"


class TaskOrchestrator:
    """
    Central orchestrator for swarm system.

    Communication:
    - Input:  /swarm/commands (JSON String from bridge.py)
    - Output: /swarm/status   (JSON String to bridge.py)
    - Fleet:  /fleet/spawn_command, /fleet/delete_command (JSON topics)
    """

    def __init__(self):
        rospy.init_node('task_orchestrator', anonymous=False)

        # Task state
        self.current_task_id = None
        self.current_task_type = None
        self.task_state = TaskState.IDLE
        self.task_progress = 0.0

        # Robot tracking
        self.robots = {}          # robot_id -> {pose, velocity, status, threat_level, role}
        self.robot_sensor_data = {}
        self.robot_count = 0
        self.emergency_stop_active = False
        self.collision_count = 0

        # Command handlers
        self.command_handlers = {
            'spawn_robots': self._handle_spawn_robots,
            'delete_robots': self._handle_delete_robots,
            'start_task': self._handle_start_task,
            'stop_task': self._handle_stop_task,
            'emergency_stop': self._handle_emergency_stop,
            'control_leader': self._handle_control_leader,
            'get_status': self._handle_get_status,
            'set_waypoints': self._handle_set_waypoints,
            'set_formation': self._handle_set_formation,
            'spawn_obstacles': self._handle_spawn_obstacles,
        }

        # Publishers
        self.status_pub = rospy.Publisher('/swarm/status', String, queue_size=1)
        self.emergency_stop_pub = rospy.Publisher('/swarm/emergency_stop', Bool, queue_size=1, latch=True)
        self.leader_cmd_pub = rospy.Publisher('/leader/cmd_vel', Twist, queue_size=1)

        # Fleet manager interface (topic-based)
        self.fleet_spawn_pub = rospy.Publisher('/fleet/spawn_command', String, queue_size=5)
        self.fleet_delete_pub = rospy.Publisher('/fleet/delete_command', String, queue_size=5)

        # Obstacle generator interface (topic-based)
        self.obstacle_spawn_pub = rospy.Publisher('/environment/spawn_obstacles_cmd', String, queue_size=1)

        # Subscribers
        rospy.Subscriber('/swarm/commands', String, self._command_callback, queue_size=10)
        rospy.Subscriber('/fleet/robot_list', String, self._robot_list_callback, queue_size=1)
        rospy.Subscriber('/swarm/task_complete', Bool, self._task_complete_callback, queue_size=1)

        # Dynamic robot subscribers
        self.odom_subs = {}
        self.threat_subs = {}
        self.scan_subs = {}

        # Status broadcast timer (10 Hz)
        self.status_timer = rospy.Timer(rospy.Duration(0.1), self._broadcast_status)

        rospy.loginfo("Task Orchestrator initialized - listening on /swarm/commands")

    # ==================== Command Callback ====================

    def _command_callback(self, msg):
        try:
            data = json.loads(msg.data)
            command = data.get('command', '')
            parameters = data.get('parameters', {})
            if isinstance(parameters, str):
                try:
                    parameters = json.loads(parameters)
                except (json.JSONDecodeError, TypeError):
                    pass

            rospy.loginfo(f"Received command: {command}")
            handler = self.command_handlers.get(command)
            if handler:
                handler(parameters)
            else:
                rospy.logwarn(f"Unknown command: {command}")
        except json.JSONDecodeError as e:
            rospy.logerr(f"Invalid JSON in command: {e}")
        except Exception as e:
            rospy.logerr(f"Error processing command: {e}")

    # ==================== Command Handlers ====================

    def _handle_spawn_robots(self, params):
        count = params.get('robot_count', 5)
        pattern = params.get('spawn_pattern', 'grid')
        cmd = json.dumps({"count": count, "pattern": pattern})
        self.fleet_spawn_pub.publish(String(data=cmd))
        rospy.loginfo(f"Sent spawn command: {count} robots, pattern={pattern}")

    def _handle_delete_robots(self, params):
        robot_ids = params.get('robot_ids', [])
        if robot_ids:
            cmd = json.dumps({"robot_ids": robot_ids})
        else:
            cmd = json.dumps({"all": True})
        self.fleet_delete_pub.publish(String(data=cmd))
        rospy.loginfo(f"Sent delete command: {robot_ids if robot_ids else 'ALL'}")

    def _handle_start_task(self, params):
        task_type = params.get('task_type', '')
        if not task_type:
            rospy.logwarn("start_task: task_type is required")
            return

        # Stop current task if running
        if self.task_state == TaskState.RUNNING:
            self._handle_stop_task({})

        task_id = str(uuid.uuid4())
        self.current_task_id = task_id
        self.current_task_type = task_type
        self.task_state = TaskState.RUNNING
        self.task_progress = 0.0

        # Assign roles based on task type
        sorted_ids = sorted(self.robots.keys())
        for i, rid in enumerate(sorted_ids):
            if task_type == 'follow_leader' and i == 0:
                self.robots[rid]['role'] = 'leader'
            elif task_type == 'formation':
                self.robots[rid]['role'] = 'formation_member'
            elif task_type == 'transport':
                self.robots[rid]['role'] = 'transporter'
            else:
                self.robots[rid]['role'] = 'follower'

        # Build config payload for the behavior node
        config = params.get('config', {})
        if isinstance(config, str):
            try:
                config = json.loads(config)
            except (json.JSONDecodeError, TypeError):
                config = {}

        if task_type == 'follow_leader':
            start_config = {
                'leader_mode': params.get('leader_mode', 'circular'),
                'waypoints': config.get('waypoints', []),
                'radius': config.get('radius', 2.0),
            }
            start_topic = '/follow_leader/start'
        elif task_type == 'formation':
            start_config = {
                'formation_type': params.get('formation_type', 'triangle'),
                'movement_mode': params.get('movement_mode', 'static'),
            }
            start_topic = '/formation/start'
        elif task_type == 'transport':
            start_config = {
                'target_x': params.get('target_x', 3.0),
                'target_y': params.get('target_y', 3.0),
            }
            start_topic = '/transport/start'
        else:
            rospy.logwarn(f"Unknown task type: {task_type}")
            return

        # Signal start to behavior node with config
        pub = rospy.Publisher(start_topic, String, queue_size=1, latch=True)
        rospy.sleep(0.2)
        pub.publish(String(data=json.dumps(start_config)))
        rospy.loginfo(f"Started task: {task_type} (ID: {task_id}) config={start_config}")

    def _handle_stop_task(self, params):
        if self.task_state not in [TaskState.RUNNING, TaskState.PAUSED]:
            return

        # Signal stop to all behavior nodes
        for topic in ['/follow_leader/stop', '/formation/stop', '/transport/stop']:
            pub = rospy.Publisher(topic, Empty, queue_size=1)
            rospy.sleep(0.05)
            pub.publish(Empty())

        self._stop_all_robots()
        self.task_state = TaskState.STOPPED
        self.task_progress = 0.0
        rospy.loginfo(f"Stopped task: {self.current_task_type}")

    def _handle_emergency_stop(self, params):
        self.emergency_stop_active = True
        self.emergency_stop_pub.publish(Bool(data=True))
        self._stop_all_robots()
        self.task_state = TaskState.STOPPED
        rospy.logwarn("EMERGENCY STOP ACTIVATED")

    def _handle_control_leader(self, params):
        cmd = Twist()
        cmd.linear.x = max(-0.26, min(0.26, params.get('linear_velocity', 0.0)))
        cmd.angular.z = max(-1.82, min(1.82, params.get('angular_velocity', 0.0)))
        self.leader_cmd_pub.publish(cmd)

    def _handle_get_status(self, params):
        return self._build_status()

    def _handle_set_waypoints(self, params):
        waypoints = params.get('waypoints', [])
        pub = rospy.Publisher('/follow_leader/waypoints', String, queue_size=1, latch=True)
        rospy.sleep(0.1)
        pub.publish(String(data=json.dumps(waypoints)))

    def _handle_set_formation(self, params):
        pub = rospy.Publisher('/formation/set_shape', String, queue_size=1, latch=True)
        rospy.sleep(0.1)
        pub.publish(String(data=params.get('formation_type', 'triangle')))

    def _handle_spawn_obstacles(self, params):
        density = params.get('density', 'medium')
        cmd = json.dumps({"density": density, "types": ["box", "cylinder"], "clear": True})
        self.obstacle_spawn_pub.publish(String(data=cmd))
        rospy.loginfo(f"Sent spawn obstacles command: density={density}")

    # ==================== Robot Tracking ====================

    def _robot_list_callback(self, msg):
        robot_ids = [r for r in msg.data.split(',') if r]
        new_ids = set(robot_ids)
        current_ids = set(self.robots.keys())

        # Subscribe to new robots
        for rid in new_ids - current_ids:
            self._subscribe_to_robot(rid)

        # Unsubscribe from removed robots
        for rid in current_ids - new_ids:
            self._unsubscribe_from_robot(rid)

        self.robot_count = len(robot_ids)

    def _task_complete_callback(self, msg):
        if msg.data and self.task_state == TaskState.RUNNING:
            self.task_state = TaskState.COMPLETED
            self.task_progress = 1.0
            rospy.loginfo(f"Task {self.current_task_type} completed")

    def _subscribe_to_robot(self, robot_id):
        if robot_id in self.robots:
            return

        self.robots[robot_id] = {
            'pose': Pose(), 'velocity': Twist(),
            'status': 'active', 'threat_level': 0.0,
            'role': 'follower',
        }
        self.robot_sensor_data[robot_id] = {}

        self.odom_subs[robot_id] = rospy.Subscriber(
            f'/{robot_id}/odom', Odometry,
            lambda msg, rid=robot_id: self._odom_cb(rid, msg), queue_size=1
        )
        self.threat_subs[robot_id] = rospy.Subscriber(
            f'/{robot_id}/threat_level', Float32,
            lambda msg, rid=robot_id: self._threat_cb(rid, msg), queue_size=1
        )
        self.scan_subs[robot_id] = rospy.Subscriber(
            f'/{robot_id}/scan', LaserScan,
            lambda msg, rid=robot_id: self._scan_cb(rid, msg), queue_size=1
        )

    def _unsubscribe_from_robot(self, robot_id):
        for subs_dict in [self.odom_subs, self.threat_subs, self.scan_subs]:
            sub = subs_dict.pop(robot_id, None)
            if sub:
                sub.unregister()
        self.robots.pop(robot_id, None)
        self.robot_sensor_data.pop(robot_id, None)

    def _odom_cb(self, robot_id, msg):
        if robot_id in self.robots:
            self.robots[robot_id]['pose'] = msg.pose.pose
            self.robots[robot_id]['velocity'] = msg.twist.twist

    def _threat_cb(self, robot_id, msg):
        if robot_id in self.robots:
            prev = self.robots[robot_id]['threat_level']
            self.robots[robot_id]['threat_level'] = msg.data
            # Count collision when threat transitions to emergency (1.0)
            if msg.data >= 1.0 and prev < 1.0:
                self.collision_count += 1

    def _scan_cb(self, robot_id, msg):
        if robot_id in self.robot_sensor_data:
            # Downsample scan to reduce bandwidth
            ds = 10
            ranges = list(msg.ranges)[::ds]
            ranges = [r if r < msg.range_max else msg.range_max for r in ranges]
            self.robot_sensor_data[robot_id] = {
                'ranges': ranges,
                'angle_min': msg.angle_min,
                'angle_max': msg.angle_max,
                'angle_increment': msg.angle_increment * ds,
                'range_min': msg.range_min,
                'range_max': msg.range_max,
                'timestamp': datetime.now().isoformat(),
            }

    # ==================== Status Broadcasting ====================

    def _broadcast_status(self, event):
        try:
            status = self._build_status()
            self.status_pub.publish(String(data=json.dumps(status)))
        except Exception as e:
            rospy.logerr_throttle(5.0, f"Error publishing status: {e}")

    def _build_status(self):
        robots_list = []
        for rid, data in self.robots.items():
            pose = data['pose']
            vel = data['velocity']
            sensor = self.robot_sensor_data.get(rid, {})

            siny = 2.0 * (pose.orientation.w * pose.orientation.z + pose.orientation.x * pose.orientation.y)
            cosy = 1.0 - 2.0 * (pose.orientation.y ** 2 + pose.orientation.z ** 2)
            yaw = math.atan2(siny, cosy)

            robots_list.append({
                'id': rid,
                'role': data.get('role', 'follower'),
                'position': {'x': pose.position.x, 'y': pose.position.y, 'z': 0.0},
                'orientation': {'theta': yaw},
                'velocity': {'linear': vel.linear.x, 'angular': vel.angular.z},
                'status': data.get('status', 'unknown'),
                'threat_level': data.get('threat_level', 0.0),
                'sensors': {'lidar': sensor},
            })

        return {
            'type': 'status_update',
            'timestamp': datetime.now().isoformat(),
            'task': {
                'task_id': self.current_task_id,
                'task_type': self.current_task_type,
                'status': self.task_state.value,
                'progress': self.task_progress,
            },
            'robots': robots_list,
            'robot_count': self.robot_count,
            'emergency_stop': self.emergency_stop_active,
            'collisions': self.collision_count,
        }

    def _stop_all_robots(self):
        stop = Twist()
        for rid in self.robots:
            pub = rospy.Publisher(f'/{rid}/cmd_vel', Twist, queue_size=1)
            rospy.sleep(0.01)
            pub.publish(stop)


if __name__ == '__main__':
    try:
        orchestrator = TaskOrchestrator()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
