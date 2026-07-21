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
from collections import deque
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


FOLLOW_MODES = {'circular', 'square', 'figure8', 'waypoint', 'random', 'manual'}
FORMATION_MODES = {'static', 'moving', 'adaptive'}
FORMATION_SHAPES = {
    'triangle', 'square', 'circle', 'line', 'v', 'v_formation', 'diamond',
}
FORMATION_LETTERS = set('ABCDEFGHIJKLMNOPRSTUVWXYZ')
TRANSPORT_PHASES = {'SEARCH', 'APPROACH', 'PUSH', 'DONE', 'FAILED'}
COLLISION_EVENT_HISTORY_LIMIT = 128
TRANSPORT_COLLISION_SOURCE_VERSION = 2
TRANSPORT_COLLISION_SOURCE_HISTORY_LIMIT = 128
COLLISION_EVENT_TEXT_LIMITS = {
    'robot_id': 64,
    'task_id': 128,
    'task_type': 32,
    'task_phase': 32,
}


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
        self.task_result = None
        self.task_error = None
        self.task_dispatched = False
        self.task_dispatched_at = None
        self.last_behavior_status_at = None
        self.task_lock = threading.RLock()

        # Robot tracking
        self.robots = {}          # robot_id -> {pose, velocity, status, threat_level, role}
        self.robot_sensor_data = {}
        self.robot_count = 0
        self.emergency_stop_active = False
        self.collision_count = 0
        self.collision_event_sequence = 0
        self.collision_events = deque(
            maxlen=COLLISION_EVENT_HISTORY_LIMIT
        )
        self._reset_transport_collision_consumer_locked()
        self.safety_status_timeout = max(
            0.2,
            float(rospy.get_param('~safety_status_timeout', 1.0)),
        )
        self.behavior_status_timeout = max(
            1.0,
            float(rospy.get_param('~behavior_status_timeout', 3.0)),
        )

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
        self.behavior_connection_timeout = max(
            0.1,
            float(rospy.get_param('~behavior_connection_timeout', 10.0)),
        )
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
        self.collision_subs = {}
        self.scan_subs = {}
        self.cmd_vel_pubs = {}

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
            command = {"robot_ids": robot_ids}
        else:
            command = {"all": True}
        request_id = str(params.get('request_id') or '')
        if request_id:
            command['request_id'] = request_id
        self.fleet_delete_pub.publish(String(data=json.dumps(command)))
        rospy.loginfo(f"Sent delete command: {robot_ids if robot_ids else 'ALL'}")

    @staticmethod
    def _number_in_range(value, label, minimum, maximum):
        """Parse one finite task number and keep arena limits explicit."""
        if isinstance(value, bool):
            raise ValueError("{} must be a number".format(label))
        try:
            number = float(value)
        except (TypeError, ValueError, OverflowError):
            raise ValueError("{} must be a number".format(label))
        if not math.isfinite(number):
            raise ValueError("{} must be finite".format(label))
        if not minimum <= number <= maximum:
            raise ValueError(
                "{} must be between {} and {}".format(
                    label, minimum, maximum
                )
            )
        return number

    @classmethod
    def _validated_task_config(cls, task_type, params, config):
        """Build the small, typed configuration accepted by behavior nodes."""
        if task_type == 'follow_leader':
            mode = str(params.get(
                'leader_mode', config.get('leader_mode', 'circular')
            )).strip().lower()
            if mode not in FOLLOW_MODES:
                raise ValueError(
                    "leader_mode must be one of: "
                    + ', '.join(sorted(FOLLOW_MODES))
                )
            radius = cls._number_in_range(
                config.get('radius', 2.0), 'radius', 0.5, 4.0
            )
            follow_distance = cls._number_in_range(
                config.get('follow_distance', 0.7),
                'follow_distance', 0.35, 2.0,
            )
            raw_waypoints = config.get('waypoints', [])
            if not isinstance(raw_waypoints, list) or len(raw_waypoints) > 64:
                raise ValueError("waypoints must be a list with at most 64 points")
            waypoints = []
            for index, waypoint in enumerate(raw_waypoints):
                if isinstance(waypoint, dict):
                    raw_x = waypoint.get('x')
                    raw_y = waypoint.get('y')
                elif isinstance(waypoint, (list, tuple)) and len(waypoint) == 2:
                    raw_x, raw_y = waypoint
                else:
                    raise ValueError(
                        "waypoint {} must contain x and y".format(index)
                    )
                waypoints.append([
                    cls._number_in_range(
                        raw_x, 'waypoint {} x'.format(index), -4.0, 4.0
                    ),
                    cls._number_in_range(
                        raw_y, 'waypoint {} y'.format(index), -4.0, 4.0
                    ),
                ])
            if mode == 'waypoint' and not waypoints:
                raise ValueError("waypoint mode requires at least one waypoint")
            return {
                'leader_mode': mode,
                'waypoints': waypoints,
                'radius': radius,
                'follow_distance': follow_distance,
            }

        if task_type == 'formation':
            shape = str(params.get(
                'formation_type', config.get('formation_type', 'triangle')
            )).strip()
            normalized_shape = shape.lower()
            is_letter = len(shape) == 1 and shape.upper() in FORMATION_LETTERS
            if normalized_shape not in FORMATION_SHAPES and not is_letter:
                raise ValueError("formation_type is not supported")
            mode = str(params.get(
                'movement_mode', config.get('movement_mode', 'static')
            )).strip().lower()
            if mode not in FORMATION_MODES:
                raise ValueError(
                    "movement_mode must be static, moving, or adaptive"
                )
            spacing = cls._number_in_range(
                config.get('spacing', 1.0), 'spacing', 0.35, 2.0
            )
            return {
                'formation_type': shape.upper() if is_letter else normalized_shape,
                'movement_mode': mode,
                'spacing': spacing,
            }

        if task_type == 'transport':
            target_x = cls._number_in_range(
                params.get('target_x', config.get('target_x', 3.0)),
                'target_x', -4.0, 4.0,
            )
            target_y = cls._number_in_range(
                params.get('target_y', config.get('target_y', 3.0)),
                'target_y', -4.0, 4.0,
            )
            planner = str(
                config.get('transport_planner', 'grf')
            ).strip().lower()
            if planner not in {'grf', 'legacy'}:
                raise ValueError("transport_planner must be grf or legacy")
            return {
                'target_x': target_x,
                'target_y': target_y,
                'transport_planner': planner,
            }

        raise ValueError("task_type is not supported")

    def _record_rejected_task(self, task_id, task_type, reason):
        """Expose a validation failure without disturbing another active task."""
        with self.task_lock:
            active = self.task_state in {
                TaskState.INITIALIZING, TaskState.RUNNING, TaskState.PAUSED,
            }
            if active and self.current_task_id != task_id:
                rospy.logwarn("Rejected task %s: %s", task_id, reason)
                return
            self.current_task_id = task_id
            self.current_task_type = task_type
            self.current_task_config = {}
            self.task_state = TaskState.FAILED
            self.task_progress = 0.0
            self.task_result = None
            self.task_error = reason
            self.task_dispatched = False
            self.task_dispatched_at = None
            self.last_behavior_status_at = None
        rospy.logwarn("Rejected task %s: %s", task_id, reason)

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

        task_id = str(params.get('task_id') or uuid.uuid4())
        try:
            start_config = self._validated_task_config(
                task_type, params, config
            )
        except ValueError as exc:
            self._record_rejected_task(task_id, task_type, str(exc))
            return
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

            # A new task starts a fresh delivery window.  Keep the global
            # watermark monotonic, but do not retransmit another task's event
            # payload throughout this task's lifetime.
            if not retry_undispatched:
                self.collision_events.clear()
                self._reset_transport_collision_consumer_locked()

            self.current_task_id = task_id
            self.current_task_type = task_type
            self.current_task_config = start_config
            self.task_state = TaskState.INITIALIZING
            self.task_progress = 0.0
            self.task_result = None
            self.task_error = None
            self.task_dispatched = False
            self.task_dispatched_at = None
            self.last_behavior_status_at = None

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

        # Let the previous stop reach its subscribers before dispatching the
        # replacement.  This must use wall time because the simulation clock
        # may be paused or running faster than real time.
        time.sleep(0.2)
        behavior_pub = self.behavior_start_pubs[task_type]
        if not self._wait_for_behavior_subscriber(behavior_pub):
            with self.task_lock:
                if (
                    self.current_task_id == task_id
                    and self.task_state == TaskState.INITIALIZING
                ):
                    self.task_state = TaskState.FAILED
                    self.task_error = (
                        "Behavior node for {} did not subscribe within "
                        "{:.1f}s".format(
                            task_type, self.behavior_connection_timeout
                        )
                    )
                    self.task_dispatched = False
            rospy.logerr(
                "Behavior node for %s did not subscribe within %.1fs",
                task_type,
                self.behavior_connection_timeout,
            )
            return

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
                self.task_error = str(exc)
                rospy.logerr(
                    "Failed to dispatch task %s: %s", task_id, exc
                )
                return
            self.task_dispatched = True
            self.task_dispatched_at = self._control_clock()
            self.last_behavior_status_at = None
        rospy.loginfo(f"Started task: {task_type} (ID: {task_id}) config={start_config}")

    def _wait_for_behavior_subscriber(self, publisher):
        """Wait for the selected behavior node without relying on /clock."""
        connection_count = getattr(publisher, 'get_num_connections', None)
        if connection_count is None:
            # Lightweight test publishers and older ROS shims do not expose
            # connection counts.  Publishing remains backward compatible.
            return True

        deadline = time.monotonic() + self.behavior_connection_timeout
        is_shutdown = getattr(rospy, 'is_shutdown', lambda: False)
        while time.monotonic() < deadline and not is_shutdown():
            if connection_count() > 0:
                return True
            time.sleep(0.05)
        return connection_count() > 0

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
            self.task_dispatched_at = self._control_clock()
            self.last_behavior_status_at = None
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
            self.task_error = None
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
        self.task_error = None

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
            self.task_error = None
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
                self.task_error = None
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

            self.last_behavior_status_at = self._control_clock()

            if 'progress' in status:
                try:
                    self.task_progress = max(
                        0.0, min(1.0, float(status['progress']))
                    )
                except (TypeError, ValueError):
                    pass

            if task_type == 'transport':
                collision_error = (
                    self._consume_transport_collision_stream_locked(
                        status, status_task_id
                    )
                )
                if collision_error is not None:
                    self._fail_current_task(
                        status_task_id, collision_error
                    )
                    return
                self.task_result = self._transport_result(
                    status, status_task_id
                )

            completed = (
                task_type == 'transport' and status.get('phase') == 'DONE'
            ) or (
                task_type == 'formation'
                and status.get('movement_mode') == 'static'
                and status.get('state') == 'formed'
            ) or bool(status.get('completed', False))

            failed = (
                task_type in ('formation', 'follow_leader')
                and status.get('state') == 'failed'
            ) or (
                task_type == 'transport'
                and status.get('phase') == 'FAILED'
            )
            if failed:
                self._fail_current_task(
                    status_task_id,
                    str(status.get('error') or '{} failed'.format(task_type)),
                )
            elif completed:
                self._complete_current_task(status_task_id)

    def _transport_result(self, status, expected_task_id):
        """Keep the useful transport summary without forwarding diagnostics."""
        phase = status.get('phase')
        if not isinstance(phase, str):
            phase = None

        searching_count = status.get('searching_robot_count')
        if (
            isinstance(searching_count, bool)
            or not isinstance(searching_count, int)
            or searching_count < 0
        ):
            searching_count = None

        all_pushers_confirmed = status.get('all_pushers_confirmed')
        if not isinstance(all_pushers_confirmed, bool):
            all_pushers_confirmed = None

        contributor_count = status.get('useful_contributor_count')
        if (
            isinstance(contributor_count, bool)
            or not isinstance(contributor_count, int)
            or contributor_count < 0
        ):
            contributor_count = None

        contributor_ids = status.get('useful_contributor_ids')
        if (
            not isinstance(contributor_ids, list)
            or not all(
                isinstance(robot_id, str) and robot_id
                for robot_id in contributor_ids
            )
            or len(set(contributor_ids)) != len(contributor_ids)
            or contributor_count != len(contributor_ids)
        ):
            contributor_ids = None
        else:
            contributor_ids = sorted(
                contributor_ids,
                key=robot_id_sort_key,
            )

        result = {
            'phase': phase,
            'searching_robot_count': searching_count,
            'all_pushers_confirmed': all_pushers_confirmed,
            'useful_contributor_count': contributor_count,
            'useful_contributor_ids': contributor_ids,
            'discovery': self._transport_discovery(
                status.get('discovery'), expected_task_id
            ),
        }
        return {'transport': result}

    @staticmethod
    def _transport_discovery(discovery, expected_task_id):
        """Validate the one-shot payload-found event before exposing it."""
        if not isinstance(discovery, dict):
            return None
        if discovery.get('event') != 'payload_found':
            return None
        if str(discovery.get('task_id') or '') != expected_task_id:
            return None
        if discovery.get('announced') is not True:
            return None

        event_id = discovery.get('event_id')
        finder = discovery.get('finder')
        position = discovery.get('object_position')
        if not isinstance(event_id, str) or not event_id:
            return None
        if not isinstance(finder, str) or not finder:
            return None
        if not isinstance(position, dict):
            return None

        try:
            distance = float(discovery.get('distance'))
            position_x = float(position.get('x'))
            position_y = float(position.get('y'))
            sim_time = float(discovery.get('sim_time'))
        except (TypeError, ValueError):
            return None
        if not all(math.isfinite(value) for value in (
            distance, position_x, position_y, sim_time
        )):
            return None
        if distance < 0.0 or sim_time < 0.0:
            return None

        notified = discovery.get('notified_robots')
        if not isinstance(notified, list) or not all(
            isinstance(robot_id, str) and robot_id
            for robot_id in notified
        ):
            return None

        return {
            'event': 'payload_found',
            'event_id': event_id,
            'task_id': expected_task_id,
            'announced': True,
            'finder': finder,
            'distance': distance,
            'object_position': {
                'x': position_x,
                'y': position_y,
            },
            'sim_time': sim_time,
            'notified_robots': list(notified),
        }

    def _fail_current_task(self, expected_task_id, reason):
        """Stop a behavior that rejected an unsafe runtime configuration."""
        with self.task_lock:
            if (
                expected_task_id != self.current_task_id
                or self.task_state != TaskState.RUNNING
            ):
                return
            self._signal_stop_behaviors()
            self._stop_all_robots()
            self.task_state = TaskState.FAILED
            self.task_progress = 0.0
            self.task_error = reason
        rospy.logerr("Task %s failed: %s", self.current_task_type, reason)

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
            self.task_error = None
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
            'collision_active': False,
            'last_threat_at': None,
            'last_collision_at': None,
            'last_odom_at': None,
            'last_scan_at': None,
            'subscribed_at': self._control_clock(),
            'role': 'follower',
        }
        self.robot_sensor_data[robot_id] = {}
        self.cmd_vel_pubs[robot_id] = rospy.Publisher(
            f'/{robot_id}/cmd_vel', Twist, queue_size=1
        )

        self.odom_subs[robot_id] = rospy.Subscriber(
            f'/{robot_id}/odom', Odometry,
            lambda msg, rid=robot_id: self._odom_cb(rid, msg), queue_size=1
        )
        self.threat_subs[robot_id] = rospy.Subscriber(
            f'/{robot_id}/threat_level', Float32,
            lambda msg, rid=robot_id: self._threat_cb(rid, msg), queue_size=1
        )
        self.collision_subs[robot_id] = rospy.Subscriber(
            f'/{robot_id}/collision_state', Bool,
            lambda msg, rid=robot_id: self._collision_cb(rid, msg),
            queue_size=1,
        )
        self.scan_subs[robot_id] = rospy.Subscriber(
            f'/{robot_id}/scan', LaserScan,
            lambda msg, rid=robot_id: self._scan_cb(rid, msg), queue_size=1
        )

    def _unsubscribe_from_robot(self, robot_id):
        for subs_dict in [
            self.odom_subs,
            self.threat_subs,
            self.collision_subs,
            self.scan_subs,
        ]:
            sub = subs_dict.pop(robot_id, None)
            if sub:
                sub.unregister()
        command_pub = self.cmd_vel_pubs.pop(robot_id, None)
        if command_pub is not None:
            command_pub.publish(Twist())
            command_pub.unregister()
        self.robots.pop(robot_id, None)
        self.robot_sensor_data.pop(robot_id, None)

    def _odom_cb(self, robot_id, msg):
        with self.task_lock:
            if robot_id in self.robots:
                self.robots[robot_id]['pose'] = msg.pose.pose
                self.robots[robot_id]['velocity'] = msg.twist.twist
                self.robots[robot_id]['last_odom_at'] = self._control_clock()

    def _threat_cb(self, robot_id, msg):
        with self.task_lock:
            if robot_id in self.robots:
                try:
                    threat = float(msg.data)
                except (TypeError, ValueError):
                    return
                if not math.isfinite(threat):
                    return
                self.robots[robot_id]['threat_level'] = max(
                    0.0, min(1.0, threat)
                )
                self.robots[robot_id]['last_threat_at'] = (
                    self._control_clock()
                )

    def _collision_cb(self, robot_id, msg):
        """Aggregate rising physical-contact states into collision episodes."""
        with self.task_lock:
            if robot_id in self.robots:
                active = bool(msg.data)
                previous = bool(
                    self.robots[robot_id].get('collision_active', False)
                )
                self.robots[robot_id]['collision_active'] = active
                self.robots[robot_id]['last_collision_at'] = (
                    self._control_clock()
                )
                if (
                    active
                    and not previous
                    and self.current_task_type != 'transport'
                ):
                    self.collision_count += 1
                    self.collision_event_sequence += 1
                    self.collision_events.append(
                        self._sealed_collision_event_locked(robot_id)
                    )

    def _reset_transport_collision_consumer_locked(self):
        """Forget delivery state while retaining the global event watermark."""
        self.transport_collision_source_id = None
        self.transport_collision_source_task_id = None
        self.transport_collision_source_task_start = None
        self.transport_collision_source_watermark = None
        self.transport_collision_source_next_sequence = None
        self.transport_collision_source_last_control_sequence = 0
        self.transport_collision_source_fingerprints = {}

    def _ensure_transport_collision_consumer_locked(self):
        """Support focused fixtures constructed without ``__init__``."""
        if not hasattr(self, 'transport_collision_source_fingerprints'):
            self._reset_transport_collision_consumer_locked()

    @staticmethod
    def _transport_source_integer(value, minimum=0):
        if isinstance(value, bool) or not isinstance(value, int):
            return None
        return value if value >= minimum else None

    @staticmethod
    def _transport_source_number(value):
        if (
            isinstance(value, bool)
            or not isinstance(value, (int, float))
        ):
            return None
        value = float(value)
        return value if math.isfinite(value) and value >= 0.0 else None

    def _consume_transport_collision_stream_locked(
        self, status, expected_task_id
    ):
        """Validate and copy the transport-owned causal collision stream.

        ``/collision_state`` remains the live compatibility state, but its
        asynchronous callback cannot authoritatively label a transport phase.
        Only this source stream advances the aggregate counter for transport.
        Every validation error fails the task closed before a terminal status
        can be accepted.
        """
        self._ensure_transport_collision_consumer_locked()
        stream = status.get('collision_events')
        if not isinstance(stream, dict):
            return 'Transport collision event stream is missing'

        version = self._transport_source_integer(stream.get('version'), 1)
        source_id = stream.get('source_id')
        stream_task_id = stream.get('task_id')
        task_start = self._transport_source_integer(
            stream.get('task_start_sequence')
        )
        history_limit = self._transport_source_integer(
            stream.get('history_limit'), 1
        )
        first_sequence = self._transport_source_integer(
            stream.get('first_sequence'), 1
        )
        last_sequence = self._transport_source_integer(
            stream.get('last_sequence')
        )
        watermark = self._transport_source_integer(
            stream.get('watermark')
        )
        events = stream.get('events')
        errors = stream.get('protocol_errors')
        terminal = stream.get('terminal')
        valid = stream.get('valid')

        if version != TRANSPORT_COLLISION_SOURCE_VERSION:
            return 'Transport collision event stream version is unsupported'
        if (
            not isinstance(source_id, str)
            or not source_id
            or len(source_id) > 128
            or not source_id.isprintable()
        ):
            return 'Transport collision source identity is invalid'
        if stream_task_id != expected_task_id:
            return 'Transport collision stream belongs to another task'
        if (
            task_start is None
            or history_limit is None
            or history_limit > TRANSPORT_COLLISION_SOURCE_HISTORY_LIMIT
            or first_sequence is None
            or last_sequence is None
            or watermark is None
            or last_sequence != watermark
            or watermark < task_start
        ):
            return 'Transport collision event stream metadata is invalid'
        if (
            not isinstance(events, list)
            or len(events) > history_limit
            or not isinstance(errors, list)
            or any(not isinstance(error, str) for error in errors)
            or errors
            or valid is not True
            or not isinstance(terminal, bool)
        ):
            return 'Transport collision event stream reported a source error'

        phase = status.get('phase')
        phase_is_terminal = phase in ('DONE', 'FAILED')
        if terminal != phase_is_terminal:
            return 'Transport collision terminal marker is inconsistent'
        if phase_is_terminal:
            terminal_watermark = self._transport_source_integer(
                stream.get('terminal_watermark')
            )
            if terminal_watermark != watermark:
                return 'Transport terminal collision watermark is incomplete'

        parsed = []
        previous_source_sequence = None
        previous_control_sequence = None
        for event in events:
            if not isinstance(event, dict) or event.get('valid') is not True:
                return 'Transport collision event is malformed'
            sequence = self._transport_source_integer(
                event.get('sequence'), 1
            )
            source_sequence = self._transport_source_integer(
                event.get('source_sequence'), 1
            )
            control_sequence = self._transport_source_integer(
                event.get('control_sequence'), 1
            )
            sim_time = self._transport_source_number(
                event.get('sim_time')
            )
            wall_time = self._transport_source_number(
                event.get('wall_time')
            )
            robot_id = self._collision_event_text(
                event.get('robot_id'), 'robot_id'
            )
            task_id = self._collision_event_text(
                event.get('task_id'), 'task_id'
            )
            task_type = self._collision_event_text(
                event.get('task_type'), 'task_type'
            )
            task_phase = self._collision_event_text(
                event.get('task_phase'), 'task_phase'
            )
            if (
                sequence is None
                or source_sequence != sequence
                or control_sequence is None
                or sim_time is None
                or wall_time is None
                or event.get('source_id') != source_id
                or task_id != expected_task_id
                or task_type != 'transport'
                or task_phase not in TRANSPORT_PHASES
                or robot_id not in self.robots
            ):
                return 'Transport collision event has invalid causal data'
            if (
                previous_source_sequence is not None
                and sequence != previous_source_sequence + 1
            ):
                return 'Transport collision source history is discontinuous'
            if (
                previous_control_sequence is not None
                and control_sequence < previous_control_sequence
            ):
                return 'Transport collision control sequence regressed'
            previous_source_sequence = sequence
            previous_control_sequence = control_sequence
            fingerprint = (
                source_id, robot_id, task_id, task_type, task_phase,
                control_sequence, sim_time, wall_time,
            )
            parsed.append((sequence, fingerprint, {
                'robot_id': robot_id,
                'task_id': task_id,
                'task_type': task_type,
                'task_phase': task_phase,
                'source_id': source_id,
                'source_sequence': sequence,
                'source_control_sequence': control_sequence,
                'source_sim_time': sim_time,
                'source_wall_time': wall_time,
            }))

        expected_first = parsed[0][0] if parsed else watermark + 1
        if (
            first_sequence != expected_first
            or (parsed and parsed[-1][0] != watermark)
        ):
            return 'Transport collision source watermark is discontinuous'

        if self.transport_collision_source_id is None:
            current_source_id = source_id
            current_task_id = expected_task_id
            current_task_start = task_start
            previous_watermark = task_start
            next_sequence = task_start + 1
            last_control_sequence = 0
            fingerprints = {}
        else:
            if source_id != self.transport_collision_source_id:
                return 'Transport collision source restarted during the task'
            if (
                expected_task_id != self.transport_collision_source_task_id
                or task_start != self.transport_collision_source_task_start
            ):
                return 'Transport collision source task window changed'
            current_source_id = self.transport_collision_source_id
            current_task_id = self.transport_collision_source_task_id
            current_task_start = self.transport_collision_source_task_start
            previous_watermark = self.transport_collision_source_watermark
            next_sequence = self.transport_collision_source_next_sequence
            last_control_sequence = (
                self.transport_collision_source_last_control_sequence
            )
            fingerprints = dict(
                self.transport_collision_source_fingerprints
            )

        if watermark < previous_watermark:
            return 'Transport collision source watermark regressed'
        if first_sequence > next_sequence and watermark >= next_sequence:
            return 'Transport collision source dropped an event'

        new_events = []
        for sequence, fingerprint, copied_event in parsed:
            if sequence < next_sequence:
                previous = fingerprints.get(sequence)
                if previous is not None and previous != fingerprint:
                    return 'Transport collision source reused a sequence'
                continue
            if sequence > next_sequence:
                return 'Transport collision source skipped an event'
            control_sequence = copied_event['source_control_sequence']
            if control_sequence < last_control_sequence:
                return 'Transport collision control sequence regressed'
            fingerprints[sequence] = fingerprint
            next_sequence = sequence + 1
            last_control_sequence = control_sequence
            new_events.append(copied_event)

        if next_sequence != watermark + 1:
            return 'Transport collision source watermark has an event gap'

        for copied_event in new_events:
            self.collision_count += 1
            self.collision_event_sequence += 1
            copied_event['sequence'] = self.collision_event_sequence
            self.collision_events.append(copied_event)

        if len(fingerprints) > TRANSPORT_COLLISION_SOURCE_HISTORY_LIMIT:
            retained = sorted(fingerprints)[
                -TRANSPORT_COLLISION_SOURCE_HISTORY_LIMIT:
            ]
            fingerprints = {
                sequence: fingerprints[sequence]
                for sequence in retained
            }

        self.transport_collision_source_id = current_source_id
        self.transport_collision_source_task_id = current_task_id
        self.transport_collision_source_task_start = current_task_start
        self.transport_collision_source_watermark = watermark
        self.transport_collision_source_next_sequence = next_sequence
        self.transport_collision_source_last_control_sequence = (
            last_control_sequence
        )
        self.transport_collision_source_fingerprints = fingerprints
        return None

    @staticmethod
    def _collision_event_text(value, field):
        """Return a compact printable event value, or ``None`` if unsafe."""
        if not isinstance(value, str):
            return None
        value = value.strip()
        limit = COLLISION_EVENT_TEXT_LIMITS[field]
        if not value or len(value) > limit or not value.isprintable():
            return None
        return value

    def _sealed_collision_event_locked(self, robot_id):
        """Capture task identity and authoritative phase at the contact edge."""
        task_type = self._collision_event_text(
            self.current_task_type, 'task_type'
        )
        task_phase = None
        if task_type == 'transport':
            task_result = self.task_result
            transport_result = (
                task_result.get('transport')
                if isinstance(task_result, dict) else None
            )
            candidate = (
                transport_result.get('phase')
                if isinstance(transport_result, dict) else None
            )
            if isinstance(candidate, str):
                candidate = candidate.strip().upper()
                if candidate in TRANSPORT_PHASES:
                    task_phase = candidate

        return {
            'sequence': self.collision_event_sequence,
            'robot_id': self._collision_event_text(robot_id, 'robot_id'),
            'task_id': self._collision_event_text(
                self.current_task_id, 'task_id'
            ),
            'task_type': task_type,
            'task_phase': task_phase,
        }

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
                if robot_id in self.robots:
                    self.robots[robot_id]['last_scan_at'] = (
                        self._control_clock()
                    )

    # ==================== Status Broadcasting ====================

    def _broadcast_status(self, event):
        try:
            task_id, health_error = self._runtime_health_error()
            if health_error:
                self._fail_current_task(task_id, health_error)
            status = self._build_status()
            self.status_pub.publish(String(data=json.dumps(status)))
        except Exception as e:
            rospy.logerr_throttle(5.0, f"Error publishing status: {e}")

    def _runtime_health_error(self):
        """Return a correlated fail-closed error for a running task."""
        with self.task_lock:
            if self.task_state != TaskState.RUNNING:
                return self.current_task_id, None

            now = self._control_clock()
            dispatched_at = getattr(self, 'task_dispatched_at', None)
            last_status_at = getattr(self, 'last_behavior_status_at', None)
            behavior_timeout = getattr(self, 'behavior_status_timeout', 3.0)
            status_reference = (
                last_status_at if last_status_at is not None else dispatched_at
            )
            if (
                status_reference is not None
                and now - status_reference > behavior_timeout
            ):
                return self.current_task_id, (
                    "Behavior status heartbeat became stale for {}".format(
                        self.current_task_type
                    )
                )

            safety_timeout = self.safety_status_timeout
            if (
                dispatched_at is None
                or now - dispatched_at <= safety_timeout
            ):
                return self.current_task_id, None

            stale = {}
            for robot_id, data in self.robots.items():
                missing = []
                for label, key in (
                    ('odometry', 'last_odom_at'),
                    ('lidar', 'last_scan_at'),
                ):
                    stamp = data.get(key)
                    if (
                        stamp is None
                        or now < stamp
                        or now - stamp > safety_timeout
                    ):
                        missing.append(label)
                if missing:
                    stale[robot_id] = missing

            if not stale:
                return self.current_task_id, None
            summary = '; '.join(
                '{}: {}'.format(robot_id, ', '.join(inputs))
                for robot_id, inputs in sorted(stale.items())
            )
            return self.current_task_id, (
                "Safety telemetry became stale ({})".format(summary)
            )

    def _safety_freshness(self, data, now):
        """Describe the age of every safety input without calling unknown safe."""
        ages = {}
        stale = []
        for label, key in (
            ('odometry', 'last_odom_at'),
            ('lidar', 'last_scan_at'),
            ('threat', 'last_threat_at'),
            ('collision', 'last_collision_at'),
        ):
            stamp = data.get(key)
            age = None if stamp is None else max(0.0, now - stamp)
            ages[label] = round(age, 3) if age is not None else None
            if (
                stamp is None
                or now < stamp
                or now - stamp > self.safety_status_timeout
            ):
                stale.append(label)
        return stale, ages

    def _build_status(self):
        with self.task_lock:
            safety_now = self._control_clock()
            robot_snapshots = [
                (
                    rid,
                    dict(data),
                    self.robot_sensor_data.get(rid, {}),
                    self._safety_freshness(data, safety_now),
                )
                for rid, data in self.robots.items()
            ]
            task_snapshot = {
                'task_id': self.current_task_id,
                'task_type': self.current_task_type,
                'status': self.task_state.value,
                'progress': self.task_progress,
                'result': self.task_result,
                'error': self.task_error,
            }
            robot_count = self.robot_count
            emergency_stop_active = self.emergency_stop_active
            collision_count = self.collision_count
            collision_event_sequence = self.collision_event_sequence
            collision_events = [
                dict(event) for event in self.collision_events
            ]
            heartbeat_seen = self.control_heartbeat_seen
            last_heartbeat = self.last_control_heartbeat
            watchdog_tripped = self.control_watchdog_tripped
            watchdog_enabled = self.control_watchdog_enabled
            heartbeat_timeout = self.control_heartbeat_timeout

        robots_list = []
        stale_robot_ids = []
        for rid, data, sensor, freshness in robot_snapshots:
            pose = data['pose']
            vel = data['velocity']
            stale_inputs, safety_ages = freshness
            safety_stale = bool(stale_inputs)
            if safety_stale:
                stale_robot_ids.append(rid)

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
                'threat_level': (
                    1.0 if safety_stale
                    else data.get('threat_level', 0.0)
                ),
                'collision': data.get('collision_active', False),
                'safety_stale': safety_stale,
                'safety_stale_inputs': stale_inputs,
                'safety_input_age_seconds': safety_ages,
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
            'collision_metric': 'per_robot_geometric_contact_episodes',
            'collision_events': {
                'version': 1,
                'first_sequence': (
                    collision_events[0]['sequence']
                    if collision_events else collision_event_sequence + 1
                ),
                'last_sequence': collision_event_sequence,
                'events': collision_events,
            },
            'safety': {
                'stale': bool(stale_robot_ids),
                'stale_robot_ids': stale_robot_ids,
                'timeout_seconds': self.safety_status_timeout,
            },
            'control_watchdog': {
                'enabled': watchdog_enabled,
                'armed': heartbeat_seen,
                'tripped': watchdog_tripped,
                'timeout_seconds': heartbeat_timeout,
                'heartbeat_age_seconds': (
                    round(max(0.0, safety_now - last_heartbeat), 3)
                    if last_heartbeat is not None
                    else None
                ),
            },
        }

    def _stop_all_robots(self):
        stop = Twist()
        for pub in self.cmd_vel_pubs.values():
            pub.publish(stop)

    def _shutdown(self):
        """Stop active behaviors and the wall-clock watchdog on shutdown."""
        try:
            self._signal_stop_behaviors()
        finally:
            try:
                self._stop_all_robots()
            finally:
                self._control_watchdog_stop.set()
                watchdog_thread = getattr(
                    self, '_control_watchdog_thread', None
                )
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
