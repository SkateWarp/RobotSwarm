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
import os
import sys
import threading
import time
import uuid
from enum import Enum
from datetime import datetime

from std_msgs.msg import String, Bool, Float32, Empty
from geometry_msgs.msg import Twist, Pose
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan

sys.path.append(os.path.join(os.path.dirname(__file__), '..'))
from utils.robot_ids import robot_id_sort_key


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
    - Lease:  /swarm/control_heartbeat (std_msgs/Empty, periodic worker pulse)
    - Output: /swarm/status   (JSON String to bridge.py)
    - Fleet:  /fleet/spawn_command, /fleet/delete_command (JSON topics)
    """

    def __init__(self):
        rospy.init_node('task_orchestrator', anonymous=False)

        # Task state
        self.current_task_id = None
        self.current_task_type = None
        self.current_task_config = {}
        self.task_state = TaskState.IDLE
        self.task_progress = 0.0
        self.task_dispatched = False
        self.task_lock = threading.RLock()

        # Robot tracking
        self.robots = {}          # robot_id -> {pose, velocity, status, threat_level, role}
        self.robot_sensor_data = {}
        self.robot_count = 0
        self.emergency_stop_active = False
        self.collision_count = 0

        # The watchdog uses wall-clock monotonic time, not ROS simulation time,
        # so a paused or stalled Gazebo clock cannot suppress the fail-safe.
        self.control_watchdog_enabled = bool(
            rospy.get_param('~control_watchdog_enabled', True)
        )
        self.control_heartbeat_timeout = max(
            1.0,
            float(rospy.get_param('~control_heartbeat_timeout', 10.0)),
        )
        self.control_watchdog_check_period = max(
            0.1,
            min(
                1.0,
                float(rospy.get_param(
                    '~control_watchdog_check_period', 0.5
                )),
            ),
        )
        self._control_clock = time.monotonic
        self.control_heartbeat_seen = False
        self.last_control_heartbeat = None
        self.control_watchdog_tripped = False
        self._control_watchdog_stop = threading.Event()

        # Command handlers
        self.command_handlers = {
            'spawn_robots': self._handle_spawn_robots,
            'delete_robots': self._handle_delete_robots,
            'start_task': self._handle_start_task,
            'pause_task': self._handle_pause_task,
            'resume_task': self._handle_resume_task,
            'stop_task': self._handle_stop_task,
            'emergency_stop': self._handle_emergency_stop,
            'reset_emergency_stop': self._handle_reset_emergency_stop,
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
        self.behavior_start_pubs = {
            'follow_leader': rospy.Publisher(
                '/follow_leader/start', String, queue_size=1
            ),
            'formation': rospy.Publisher(
                '/formation/start', String, queue_size=1
            ),
            'transport': rospy.Publisher(
                '/transport/start', String, queue_size=1
            ),
        }
        self.behavior_stop_pubs = [
            rospy.Publisher('/follow_leader/stop', String, queue_size=1),
            rospy.Publisher('/formation/stop', String, queue_size=1),
            rospy.Publisher('/transport/stop', String, queue_size=1),
        ]
        self.behavior_pause_pubs = {
            'follow_leader': rospy.Publisher(
                '/follow_leader/pause', String, queue_size=1
            ),
            'formation': rospy.Publisher(
                '/formation/pause', String, queue_size=1
            ),
            'transport': rospy.Publisher(
                '/transport/pause', String, queue_size=1
            ),
        }
        self.behavior_resume_pubs = {
            'follow_leader': rospy.Publisher(
                '/follow_leader/resume', String, queue_size=1
            ),
            'formation': rospy.Publisher(
                '/formation/resume', String, queue_size=1
            ),
            'transport': rospy.Publisher(
                '/transport/resume', String, queue_size=1
            ),
        }

        # Fleet manager interface (topic-based)
        self.fleet_spawn_pub = rospy.Publisher('/fleet/spawn_command', String, queue_size=5)
        self.fleet_delete_pub = rospy.Publisher('/fleet/delete_command', String, queue_size=5)

        # Obstacle generator interface (topic-based)
        self.obstacle_spawn_pub = rospy.Publisher('/environment/spawn_obstacles_cmd', String, queue_size=1)

        # Subscribers
        rospy.Subscriber('/swarm/commands', String, self._command_callback, queue_size=10)
        rospy.Subscriber('/fleet/robot_list', String, self._robot_list_callback, queue_size=1)
        rospy.Subscriber('/swarm/task_complete', Bool, self._task_complete_callback, queue_size=1)
        rospy.Subscriber(
            '/swarm/control_heartbeat',
            Empty,
            self._control_heartbeat_callback,
            queue_size=1,
        )
        rospy.Subscriber(
            '/follow_leader/status', String,
            lambda msg: self._behavior_status_callback('follow_leader', msg),
            queue_size=1,
        )
        rospy.Subscriber(
            '/formation/status', String,
            lambda msg: self._behavior_status_callback('formation', msg),
            queue_size=1,
        )
        rospy.Subscriber(
            '/transport/status', String,
            lambda msg: self._behavior_status_callback('transport', msg),
            queue_size=1,
        )

        # Dynamic robot subscribers
        self.odom_subs = {}
        self.threat_subs = {}
        self.scan_subs = {}

        # Status broadcast timer (10 Hz)
        self.status_timer = rospy.Timer(rospy.Duration(0.1), self._broadcast_status)
        self._control_watchdog_thread = threading.Thread(
            target=self._control_watchdog_loop,
            name='control-heartbeat-watchdog',
            daemon=True,
        )
        self._control_watchdog_thread.start()

        rospy.on_shutdown(self._shutdown)
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
        cmd = json.dumps({
            "count": count,
            "pattern": pattern,
            "robot_ids": params.get('robot_ids'),
        })
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
        if task_type not in self.behavior_start_pubs:
            rospy.logwarn("Unknown task type: %s", task_type)
            return

        # Build config payload for the behavior node
        config = params.get('config', {})
        if isinstance(config, str):
            try:
                config = json.loads(config)
            except (json.JSONDecodeError, TypeError):
                config = {}
        if not isinstance(config, dict):
            config = {}

        if task_type == 'follow_leader':
            start_config = {
                'leader_mode': params.get(
                    'leader_mode', config.get('leader_mode', 'circular')
                ),
                'waypoints': config.get('waypoints', []),
                'radius': config.get('radius', 2.0),
                'follow_distance': config.get('follow_distance', 0.7),
            }
        elif task_type == 'formation':
            start_config = {
                'formation_type': params.get(
                    'formation_type', config.get('formation_type', 'triangle')
                ),
                'movement_mode': params.get(
                    'movement_mode', config.get('movement_mode', 'static')
                ),
                'spacing': config.get('spacing', 1.0),
            }
        elif task_type == 'transport':
            start_config = {
                'target_x': params.get('target_x', config.get('target_x', 3.0)),
                'target_y': params.get('target_y', config.get('target_y', 3.0)),
                'transport_planner': config.get('transport_planner', 'grf'),
            }

        task_id = str(params.get('task_id') or uuid.uuid4())
        start_config['task_id'] = task_id

        with self.task_lock:
            if self.emergency_stop_active:
                rospy.logwarn(
                    "Cannot start a task while emergency stop is active"
                )
                return
            if not self.robots:
                rospy.logwarn("Cannot start a task without an active fleet")
                return

            retry_undispatched = False
            if self.current_task_id == task_id:
                if (
                    self.current_task_type != task_type
                    or self.current_task_config != start_config
                ):
                    rospy.logwarn(
                        "Ignoring conflicting redelivery for task %s",
                        task_id,
                    )
                    return
                if (
                    self.task_state == TaskState.INITIALIZING
                    or self.task_dispatched
                ):
                    rospy.loginfo(
                        "Ignoring duplicate start for task %s", task_id
                    )
                    return
                retry_undispatched = True
                rospy.loginfo(
                    "Retrying undispatched task %s after aborted initialization",
                    task_id,
                )

            # Stop the previous controller even if it had reached a terminal
            # state.  The short initialization window lets the stop callbacks
            # reach behavior nodes before the next start message.
            if self.current_task_type is not None and not retry_undispatched:
                self._handle_stop_task({}, force=True)

            self.current_task_id = task_id
            self.current_task_type = task_type
            self.current_task_config = start_config
            self.task_state = TaskState.INITIALIZING
            self.task_progress = 0.0
            self.task_dispatched = False

            sorted_ids = sorted(self.robots.keys(), key=robot_id_sort_key)
            for i, rid in enumerate(sorted_ids):
                if task_type == 'follow_leader' and i == 0:
                    self.robots[rid]['role'] = 'leader'
                elif task_type == 'formation':
                    self.robots[rid]['role'] = 'formation_member'
                elif task_type == 'transport':
                    self.robots[rid]['role'] = 'transporter'
                else:
                    self.robots[rid]['role'] = 'follower'

        rospy.sleep(0.2)
        with self.task_lock:
            if (
                self.current_task_id != task_id
                or self.task_state != TaskState.INITIALIZING
                or self.emergency_stop_active
                or not self.robots
            ):
                return
            self.task_state = TaskState.RUNNING
            try:
                self._publish_current_task()
            except Exception as exc:
                self.task_state = TaskState.FAILED
                rospy.logerr(
                    "Failed to dispatch task %s: %s", task_id, exc
                )
                return
            self.task_dispatched = True
        rospy.loginfo(f"Started task: {task_type} (ID: {task_id}) config={start_config}")

    def _handle_pause_task(self, params):
        with self.task_lock:
            if (
                not self._task_matches(params)
                or self.task_state != TaskState.RUNNING
            ):
                return

            payload = String(data=json.dumps({
                'task_id': self.current_task_id,
            }))
            pub = self.behavior_pause_pubs.get(self.current_task_type)
            if pub is None:
                return
            self.task_state = TaskState.PAUSED
            pub.publish(payload)
            self._stop_all_robots()
        rospy.loginfo("Paused task: %s", self.current_task_id)

    def _handle_resume_task(self, params):
        with self.task_lock:
            if (
                not self._task_matches(params)
                or self.task_state != TaskState.PAUSED
            ):
                return
            if self.emergency_stop_active or not self.robots:
                rospy.logwarn(
                    "Cannot resume task while the fleet is unavailable or stopped"
                )
                return

            payload = String(data=json.dumps({
                'task_id': self.current_task_id,
            }))
            pub = self.behavior_resume_pubs.get(self.current_task_type)
            if pub is None:
                return
            self.task_state = TaskState.RUNNING
            pub.publish(payload)
        rospy.loginfo("Resumed task: %s", self.current_task_id)

    def _handle_stop_task(self, params, force=False):
        with self.task_lock:
            if self.current_task_type is None:
                return
            if not force and not self._task_matches(params):
                rospy.logwarn(
                    "Ignoring uncorrelated or stale stop for task %s",
                    params.get('task_id'),
                )
                return

            self._signal_stop_behaviors()
            self._stop_all_robots()
            self.task_state = TaskState.STOPPED
            self.task_progress = 0.0
        rospy.loginfo(f"Stopped task: {self.current_task_type}")

    def _task_matches(self, params):
        task_id = str(params.get('task_id') or '')
        return bool(task_id) and task_id == self.current_task_id

    def _publish_current_task(self):
        pub = self.behavior_start_pubs.get(self.current_task_type)
        if pub is not None:
            pub.publish(String(data=json.dumps(self.current_task_config)))

    def _activate_emergency_stop_locked(self):
        """Latch the shared emergency-stop path while task_lock is held."""
        self.emergency_stop_active = True
        self.emergency_stop_pub.publish(Bool(data=True))
        self._signal_stop_behaviors()
        self._stop_all_robots()
        self.task_state = TaskState.STOPPED

    def _handle_emergency_stop(self, params):
        with self.task_lock:
            self._activate_emergency_stop_locked()
        rospy.logwarn("EMERGENCY STOP ACTIVATED")

    def _handle_reset_emergency_stop(self, params):
        with self.task_lock:
            now = self._control_clock()
            if (
                self.control_watchdog_enabled
                and self.control_heartbeat_seen
                and self.last_control_heartbeat is not None
                and now - self.last_control_heartbeat
                > self.control_heartbeat_timeout
            ):
                rospy.logwarn(
                    "Emergency-stop reset rejected while the control "
                    "heartbeat is stale"
                )
                return
            self.emergency_stop_active = False
            self.control_watchdog_tripped = False
            self.emergency_stop_pub.publish(Bool(data=False))
            self.task_state = TaskState.STOPPED
            self.task_progress = 0.0
            self._stop_all_robots()
        rospy.logwarn("Emergency stop reset; a new task must be started explicitly")

    # ==================== Control-plane Watchdog ====================

    def _control_heartbeat_callback(self, _msg):
        """Arm/refresh the worker lease using local monotonic arrival time."""
        with self.task_lock:
            first_heartbeat = not self.control_heartbeat_seen
            self.control_heartbeat_seen = True
            self.last_control_heartbeat = self._control_clock()
        if first_heartbeat:
            rospy.loginfo(
                "Control heartbeat watchdog armed (timeout %.1fs)",
                self.control_heartbeat_timeout,
            )

    def _check_control_watchdog(self, now=None):
        """
        Trip the normal latched emergency-stop path after heartbeat expiry.

        The watchdog deliberately remains disarmed until the first heartbeat,
        preventing a startup false-positive while the worker/container link is
        still being established.
        """
        with self.task_lock:
            if (
                not self.control_watchdog_enabled
                or not self.control_heartbeat_seen
                or self.last_control_heartbeat is None
                or self.control_watchdog_tripped
            ):
                return False

            current_time = self._control_clock() if now is None else now
            age = current_time - self.last_control_heartbeat
            if age <= self.control_heartbeat_timeout:
                return False

            self.control_watchdog_tripped = True
            self._activate_emergency_stop_locked()

        rospy.logerr(
            "CONTROL HEARTBEAT LOST (%.1fs > %.1fs); emergency stop latched",
            age,
            self.control_heartbeat_timeout,
        )
        return True

    def _control_watchdog_loop(self):
        """Wall-clock watchdog loop; independent of /clock and Gazebo speed."""
        while not self._control_watchdog_stop.wait(
            self.control_watchdog_check_period
        ):
            try:
                self._check_control_watchdog()
            except Exception as exc:
                rospy.logerr_throttle(
                    5.0,
                    "Control heartbeat watchdog check failed: %s",
                    exc,
                )

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
        with self.task_lock:
            current_ids = set(self.robots.keys())

            # Subscribe to new robots
            for rid in new_ids - current_ids:
                self._subscribe_to_robot(rid)

            # Unsubscribe from removed robots
            for rid in current_ids - new_ids:
                self._unsubscribe_from_robot(rid)

            self.robot_count = len(robot_ids)
            if (
                current_ids
                and not new_ids
                and self.current_task_type is not None
                and self.task_state in (
                    TaskState.INITIALIZING,
                    TaskState.RUNNING,
                    TaskState.PAUSED,
                )
            ):
                self._signal_stop_behaviors()
                self._stop_all_robots()
                self.task_state = TaskState.STOPPED
                self.task_progress = 0.0
                rospy.logwarn(
                    "Cancelled task %s because the fleet became empty",
                    self.current_task_id,
                )

    def _task_complete_callback(self, msg):
        if msg.data:
            rospy.logwarn_throttle(
                10.0,
                "Ignoring /swarm/task_complete without a task_id; "
                "behavior status is the correlated completion source",
            )

    def _behavior_status_callback(self, task_type, msg):
        try:
            status = json.loads(msg.data)
        except (json.JSONDecodeError, TypeError):
            return

        status_task_id = str(status.get('task_id') or '')
        with self.task_lock:
            if (
                task_type != self.current_task_type
                or self.task_state != TaskState.RUNNING
                or not status_task_id
                or status_task_id != self.current_task_id
            ):
                return

            if 'progress' in status:
                try:
                    self.task_progress = max(
                        0.0, min(1.0, float(status['progress']))
                    )
                except (TypeError, ValueError):
                    pass

            completed = (
                task_type == 'transport' and status.get('phase') == 'DONE'
            ) or (
                task_type == 'formation'
                and status.get('movement_mode') == 'static'
                and status.get('state') == 'formed'
            ) or bool(status.get('completed', False))

            if completed:
                self._complete_current_task(status_task_id)

    def _complete_current_task(self, expected_task_id=None):
        with self.task_lock:
            if (
                expected_task_id is not None
                and expected_task_id != self.current_task_id
            ):
                return
            if self.task_state != TaskState.RUNNING:
                return
            self._signal_stop_behaviors()
            self._stop_all_robots()
            self.task_state = TaskState.COMPLETED
            self.task_progress = 1.0
        rospy.loginfo("Task %s completed", self.current_task_type)

    def _signal_stop_behaviors(self):
        if not self.current_task_id:
            return
        payload = String(data=json.dumps({
            'task_id': self.current_task_id,
        }))
        for pub in self.behavior_stop_pubs:
            pub.publish(payload)

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
        with self.task_lock:
            if robot_id in self.robots:
                self.robots[robot_id]['pose'] = msg.pose.pose
                self.robots[robot_id]['velocity'] = msg.twist.twist

    def _threat_cb(self, robot_id, msg):
        with self.task_lock:
            if robot_id in self.robots:
                prev = self.robots[robot_id]['threat_level']
                self.robots[robot_id]['threat_level'] = msg.data
                # Count collision when threat transitions to emergency (1.0)
                if msg.data >= 1.0 and prev < 1.0:
                    self.collision_count += 1

    def _scan_cb(self, robot_id, msg):
        with self.task_lock:
            if robot_id in self.robot_sensor_data:
                # Downsample scan to reduce bandwidth
                ds = 10
                ranges = list(msg.ranges)[::ds]
                ranges = [
                    r if r < msg.range_max else msg.range_max
                    for r in ranges
                ]
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
        with self.task_lock:
            robot_snapshots = [
                (
                    rid,
                    dict(data),
                    self.robot_sensor_data.get(rid, {}),
                )
                for rid, data in self.robots.items()
            ]
            task_snapshot = {
                'task_id': self.current_task_id,
                'task_type': self.current_task_type,
                'status': self.task_state.value,
                'progress': self.task_progress,
            }
            robot_count = self.robot_count
            emergency_stop_active = self.emergency_stop_active
            collision_count = self.collision_count
            heartbeat_seen = self.control_heartbeat_seen
            last_heartbeat = self.last_control_heartbeat
            watchdog_tripped = self.control_watchdog_tripped
            watchdog_enabled = self.control_watchdog_enabled
            heartbeat_timeout = self.control_heartbeat_timeout

        robots_list = []
        for rid, data, sensor in robot_snapshots:
            pose = data['pose']
            vel = data['velocity']

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
            'task': task_snapshot,
            'robots': robots_list,
            'robot_count': robot_count,
            'emergency_stop': emergency_stop_active,
            'collisions': collision_count,
            'control_watchdog': {
                'enabled': watchdog_enabled,
                'armed': heartbeat_seen,
                'tripped': watchdog_tripped,
                'timeout_seconds': heartbeat_timeout,
                'heartbeat_age_seconds': (
                    round(max(0.0, self._control_clock() - last_heartbeat), 3)
                    if last_heartbeat is not None
                    else None
                ),
            },
        }

    def _stop_all_robots(self):
        stop = Twist()
        for rid in self.robots:
            pub = rospy.Publisher(f'/{rid}/cmd_vel', Twist, queue_size=1)
            # Wall sleep is intentional: rospy.sleep() follows /clock and can
            # wedge the emergency-stop path when Gazebo is paused.
            time.sleep(0.01)
            pub.publish(stop)

    def _shutdown(self):
        """Stop the wall-clock watchdog thread during orderly ROS shutdown."""
        self._control_watchdog_stop.set()
        watchdog_thread = getattr(self, '_control_watchdog_thread', None)
        if (
            watchdog_thread is not None
            and watchdog_thread is not threading.current_thread()
        ):
            watchdog_thread.join(timeout=1.0)


if __name__ == '__main__':
    try:
        orchestrator = TaskOrchestrator()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
