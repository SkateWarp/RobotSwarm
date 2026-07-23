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
from robot_swarm_bridge.utils.safety_publish import SafetyPublishLane


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

    SHUTDOWN_TIMEOUT = 1.0
    _EMERGENCY_ORDER_INIT_LOCK = threading.Lock()

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
        self.task_ever_dispatched = False
        self.task_dispatched_at = None
        self.last_behavior_status_at = None
        self.task_lock = threading.RLock()

        # Robot tracking
        self.robots = {}          # robot_id -> {pose, velocity, status, threat_level, role}
        self.robot_sensor_data = {}
        self.robot_count = 0
        self.emergency_stop_active = False
        # A behavior shutdown can be blocked in its own rospy publisher. Keep
        # an independent, process-local command channel latched until this
        # session ends; a user reset is not allowed to clear this condition.
        self.supervised_stop_active = False
        self.supervised_stop_context = None
        self._supervised_stop_state_started = False
        self._pending_safety_stop_acks = {}
        self._safety_ack_publish_inflight = set()
        self._safety_ack_retry_requested = set()
        self._shutdown_started = False
        self._safety_zero_lock = threading.Lock()
        self._emergency_order_condition = threading.Condition()
        self._emergency_order_generation = 0
        self._pending_emergency_true_generations = set()
        self._safety_zero_inflight = set()
        self._safety_zero_followups = set()
        self._safety_zero_receipts = {}
        self._retired_safety_publishers = {}
        self._safety_publisher_snapshot = ()
        self.ordinary_stop_active = False
        self._ordinary_stop_sequence = 0
        self._ordinary_stop_operations = {}
        self._emergency_publication_debts = {}
        self._emergency_publication_inflight = set()
        self._safety_fallback_lanes = {}
        self._safety_fallback_lane_factory = SafetyPublishLane
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
        self.formation_planning_status_timeout = max(
            self.behavior_status_timeout,
            float(rospy.get_param(
                '~formation_planning_status_timeout', 30.0
            )),
        )
        self.last_behavior_status = None

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
        # Safety publications are synchronous on purpose. A returned publish
        # must represent a socket write attempt, not only a local queue insert.
        self.emergency_stop_pub = rospy.Publisher(
            '/swarm/emergency_stop', Bool, latch=True
        )
        self.leader_cmd_pub = rospy.Publisher('/leader/cmd_vel', Twist)
        self.safety_stop_ack_pub = rospy.Publisher(
            '/swarm/safety_stop_ack', String, latch=True
        )
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
        fixed_safety_publishers = [
            ('emergency-stop', self.emergency_stop_pub),
            ('leader', self.leader_cmd_pub),
            ('safety-stop-ack', self.safety_stop_ack_pub),
        ]
        fixed_safety_publishers.extend(
            ('behavior-stop-{}'.format(index), publisher)
            for index, publisher in enumerate(self.behavior_stop_pubs)
        )
        fixed_safety_publishers.extend(
            ('behavior-pause-{}'.format(task_type), publisher)
            for task_type, publisher in self.behavior_pause_pubs.items()
        )
        for label, publisher in fixed_safety_publishers:
            if not self._register_safety_fallback_lane(label, publisher):
                raise RuntimeError(
                    "Could not reserve the {} safety publication lane".format(
                        label
                    )
                )

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
            '/swarm/safety_stop_request',
            String,
            self._safety_stop_request_callback,
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
        self._refresh_safety_publisher_snapshot()

        # Status broadcast timer (10 Hz)
        self.status_timer = rospy.Timer(rospy.Duration(0.1), self._broadcast_status)
        self.safety_stop_timer = rospy.Timer(
            rospy.Duration(0.05), self._safety_stop_zero_timer
        )
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
            arrival_tolerance = cls._number_in_range(
                params.get(
                    'arrival_tolerance',
                    config.get('arrival_tolerance', 0.5),
                ),
                'arrival_tolerance', 0.15, 0.75,
            )
            planner = str(
                config.get('transport_planner', 'grf')
            ).strip().lower()
            if planner not in {'grf', 'legacy'}:
                raise ValueError("transport_planner must be grf or legacy")
            return {
                'target_x': target_x,
                'target_y': target_y,
                'arrival_tolerance': arrival_tolerance,
                'transport_planner': planner,
            }

        raise ValueError("task_type is not supported")

    def _record_rejected_task(self, task_id, task_type, reason):
        """Expose a validation failure without disturbing another active task."""
        with self.task_lock:
            active = self.task_state in {
                TaskState.INITIALIZING, TaskState.RUNNING, TaskState.PAUSED,
            }
            if active:
                rospy.logwarn(
                    "Rejected task %s without changing the active task: %s",
                    task_id,
                    reason,
                )
                return
            with self._safety_zero_lock:
                if (
                    self._shutdown_started
                    or self.supervised_stop_active
                    or self.emergency_stop_active
                    or self.ordinary_stop_active
                ):
                    rospy.logwarn(
                        "Rejected task %s while the safety latch was closed",
                        task_id,
                    )
                    return
                self.current_task_id = task_id
                self.current_task_type = task_type
                self.current_task_config = {}
                self.task_state = TaskState.FAILED
                self.task_progress = 0.0
                self.task_result = None
                self.task_error = reason
                self.task_dispatched = False
                self.task_ever_dispatched = False
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
            if (
                self.supervised_stop_active
                or self.emergency_stop_active
                or self.ordinary_stop_active
            ):
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
                    or self.task_ever_dispatched
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
                if not self._wait_for_ordinary_stop(0.2):
                    rospy.logwarn(
                        "New task %s is waiting for the previous stop to "
                        "finish",
                        task_id,
                    )
                    return

            # Pair the final initialization commit with the shutdown gate. A
            # previous behavior stop may have blocked after the first check;
            # it must not be able to overwrite a shutdown state on return.
            with self._safety_zero_lock:
                if (
                    self._shutdown_started
                    or self.supervised_stop_active
                    or self.emergency_stop_active
                    or self.ordinary_stop_active
                ):
                    return

                # A new task starts a fresh delivery window. Keep the global
                # watermark monotonic, but do not retransmit another task's
                # event payload throughout this task's lifetime.
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
                if not retry_undispatched:
                    self.task_ever_dispatched = False
                self.task_dispatched_at = None
                self.last_behavior_status_at = None

                sorted_ids = sorted(
                    self.robots.keys(), key=robot_id_sort_key
                )
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

        dispatch_error = None
        late_safety_latch = False
        with self.task_lock:
            if (
                self.current_task_id != task_id
                or self.task_state != TaskState.INITIALIZING
                or self.supervised_stop_active
                or self.emergency_stop_active
                or self.ordinary_stop_active
                or not self.robots
            ):
                return
            self.task_state = TaskState.RUNNING
            # Once a start publish is attempted, the same identity must never
            # be treated as "never dispatched" again. ROS can raise after a
            # subscriber has already received the message.
            self.task_ever_dispatched = True
            try:
                self._publish_current_task()
            except Exception as exc:
                dispatch_error = exc
                self.task_state = TaskState.FAILED
                self.task_error = str(exc)
                rospy.logerr(
                    "Failed to dispatch task %s: %s", task_id, exc
                )
            with self._safety_zero_lock:
                late_safety_latch = (
                    self._shutdown_started
                    or self.supervised_stop_active
                    or self.emergency_stop_active
                    or self.ordinary_stop_active
                )
                if late_safety_latch:
                    self.task_state = TaskState.FAILED
                    self.task_error = (
                        "Task start returned after the safety latch closed"
                    )
                    self.task_dispatched = False
                    self.task_dispatched_at = None
                    self.last_behavior_status_at = None
                elif dispatch_error is None:
                    self.task_dispatched = True
                    self.task_dispatched_at = self._control_clock()
                    self.last_behavior_status_at = None

        if late_safety_latch:
            self._compensate_late_lifecycle_message(
                task_id, 'late-start'
            )
            return
        if dispatch_error is not None:
            return
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

    def _wait_for_ordinary_stop(self, timeout):
        """Wait briefly without ROS time for a prior lifecycle stop debt."""
        deadline = time.monotonic() + max(0.0, float(timeout))
        while time.monotonic() < deadline:
            with self._safety_zero_lock:
                if not self.ordinary_stop_active:
                    return True
            time.sleep(0.005)
        with self._safety_zero_lock:
            return not self.ordinary_stop_active

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
            stop_delivery = self._start_ordinary_stop_publications(
                'pause',
                ((self.current_task_type, pub, payload),),
            )
            if not stop_delivery['scheduling_confirmed']:
                self.task_state = TaskState.FAILED
                self.task_error = (
                    "Task pause could not schedule every safety publication"
                )
        rospy.loginfo("Paused task: %s", self.current_task_id)

    def _handle_resume_task(self, params):
        task_id = None
        publish_error = None
        late_safety_latch = False
        with self.task_lock:
            if (
                not self._task_matches(params)
                or self.task_state != TaskState.PAUSED
            ):
                return
            if (
                self.supervised_stop_active
                or self.emergency_stop_active
                or self.ordinary_stop_active
                or not self.robots
            ):
                rospy.logwarn(
                    "Cannot resume task while the fleet is unavailable or stopped"
                )
                return

            payload = String(data=json.dumps({
                'task_id': self.current_task_id,
            }))
            task_id = self.current_task_id
            pub = self.behavior_resume_pubs.get(self.current_task_type)
            if pub is None:
                return
            self.task_state = TaskState.RUNNING
            try:
                pub.publish(payload)
            except Exception as exc:
                publish_error = exc
            with self._safety_zero_lock:
                late_safety_latch = (
                    self._shutdown_started
                    or self.supervised_stop_active
                    or self.emergency_stop_active
                )
                if late_safety_latch:
                    self.task_state = TaskState.FAILED
                    self.task_error = (
                        "Task resume returned after the safety latch closed"
                    )
                    self.task_dispatched = False
                    self.task_dispatched_at = None
                    self.last_behavior_status_at = None
                elif publish_error is not None:
                    self.task_state = TaskState.FAILED
                    self.task_error = str(publish_error)
                else:
                    self.task_dispatched_at = self._control_clock()
                    self.last_behavior_status_at = None

        if late_safety_latch:
            self._compensate_late_lifecycle_message(
                task_id, 'late-resume'
            )
            return
        if publish_error is not None:
            rospy.logerr(
                "Failed to resume task %s: %s", task_id, publish_error
            )
            return
        rospy.loginfo("Resumed task: %s", task_id)

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

            stop_delivery = self._signal_stop_behaviors()
            with self._safety_zero_lock:
                if self._shutdown_started or self.supervised_stop_active:
                    return
                if not stop_delivery['scheduling_confirmed']:
                    self.task_state = TaskState.FAILED
                    self.task_error = (
                        "Task stop could not schedule every safety publication"
                    )
                else:
                    self.task_state = TaskState.STOPPED
                    self.task_error = None
                self.task_progress = 0.0
                self.task_dispatched = False
                self.task_dispatched_at = None
                self.last_behavior_status_at = None
        rospy.loginfo(f"Stopped task: {self.current_task_type}")

    def _task_matches(self, params):
        task_id = str(params.get('task_id') or '')
        return bool(task_id) and task_id == self.current_task_id

    def _publish_current_task(self):
        pub = self.behavior_start_pubs.get(self.current_task_type)
        if pub is not None:
            pub.publish(String(data=json.dumps(self.current_task_config)))

    @staticmethod
    def _report_safety_lane_error(label, action, error):
        try:
            rospy.logerr(
                "Safety publication lane %s %s: %s", label, action, error
            )
        except Exception:
            pass

    def _register_safety_fallback_lane(self, label, publisher):
        """Reserve a per-publisher worker before shutdown can exhaust threads."""
        lanes = getattr(self, '_safety_fallback_lanes', None)
        if lanes is None:
            self._safety_fallback_lanes = {}
            lanes = self._safety_fallback_lanes
        key = id(publisher)
        current = lanes.get(key)
        if current is not None and current.available:
            return True
        factory = getattr(
            self, '_safety_fallback_lane_factory', SafetyPublishLane
        )
        lane = factory(label, self._report_safety_lane_error)
        if not lane.available:
            lane.close()
            return False
        lanes[key] = lane
        return True

    def _discard_safety_fallback_lane(self, publisher):
        lane = getattr(self, '_safety_fallback_lanes', {}).pop(
            id(publisher), None
        )
        if lane is not None:
            lane.close()

    def _submit_safety_fallback(self, publisher, callback):
        lane = getattr(self, '_safety_fallback_lanes', {}).get(id(publisher))
        return lane is not None and lane.submit(callback)

    @staticmethod
    def _detached_safety_publish(
        label, publisher, message, on_return=None
    ):
        accepted = False
        try:
            publisher.publish(message)
            accepted = True
        except Exception as exc:
            try:
                rospy.logerr(
                    "Detached safety publication failed for %s: %s",
                    label,
                    exc,
                )
            except Exception:
                pass
        finally:
            if on_return is not None:
                try:
                    on_return(accepted)
                except Exception as exc:
                    try:
                        rospy.logerr(
                            "Safety publication completion failed for %s: %s",
                            label,
                            exc,
                        )
                    except Exception:
                        pass

    def _start_detached_safety_publish(
        self, label, publisher, message, on_return=None
    ):
        last_error = None
        for attempt in range(2):
            worker = threading.Thread(
                target=self._detached_safety_publish,
                args=(label, publisher, message, on_return),
                name='orchestrator-safety-{}-{}'.format(label, attempt + 1),
                daemon=True,
            )
            try:
                worker.start()
                return True
            except Exception as exc:
                last_error = exc
                try:
                    rospy.logerr(
                        "Could not start detached safety publication for %s "
                        "(attempt %d): %s",
                        label,
                        attempt + 1,
                        exc,
                    )
                except Exception:
                    pass
        if self._submit_safety_fallback(
            publisher,
            lambda: self._detached_safety_publish(
                label, publisher, message, on_return
            ),
        ):
            return True
        if last_error is not None:
            try:
                rospy.logerr(
                    "Detached safety publication was not started for %s: %s",
                    label,
                    last_error,
                )
            except Exception:
                pass
        return False

    def _start_ordinary_stop_publications(self, context, publications):
        """Keep reasserting zeros until lifecycle stops and a final zero return."""
        publications = tuple(publications)
        with self._safety_zero_lock:
            if self._shutdown_started:
                return {
                    'operation': None,
                    'scheduling_confirmed': False,
                    'failed_publications': ['shutdown'],
                    'failed_robots': [],
                }
            self._ordinary_stop_sequence += 1
            operation_id = self._ordinary_stop_sequence
            self._ordinary_stop_operations[operation_id] = {
                'task_id': self.current_task_id,
                'context': context,
                'pending_publications': set(range(len(publications))),
                'publications': {
                    index: publication
                    for index, publication in enumerate(publications)
                },
                'inflight_publications': set(),
                'failure_reporting': set(),
                'required_zero_receipts': None,
            }
            self.ordinary_stop_active = True

        zero_schedule = self._schedule_safety_zero_batch()
        failed_publications = []
        for index, (label, _publisher, _message) in enumerate(publications):
            if not self._schedule_ordinary_stop_publication(
                operation_id, index, context
            ):
                failed_publications.append(label)

        if not publications:
            self._arm_ordinary_stop_final_zero(operation_id)

        return {
            'operation': operation_id,
            'scheduling_confirmed': (
                not failed_publications
                and zero_schedule['scheduling_confirmed']
            ),
            'failed_publications': failed_publications,
            'failed_robots': list(zero_schedule['failed_robots']),
        }

    def _schedule_ordinary_stop_publication(
        self, operation_id, publication_index, context='retry'
    ):
        """Start or retry one lifecycle stop without allowing duplicates."""
        with self._safety_zero_lock:
            operation = self._ordinary_stop_operations.get(operation_id)
            if operation is None:
                return True
            if publication_index not in operation['pending_publications']:
                return True
            if publication_index in operation['inflight_publications']:
                return True
            publication = operation['publications'][publication_index]
            operation['inflight_publications'].add(publication_index)

        label, publisher, message = publication
        completion = (
            lambda accepted, current_operation=operation_id,
            current_index=publication_index: (
                self._ordinary_stop_publish_returned(
                    current_operation, current_index, accepted
                )
            )
        )
        started = self._start_detached_safety_publish(
            '{}-{}'.format(context, label),
            publisher,
            message,
            on_return=completion,
        )
        if not started:
            with self._safety_zero_lock:
                operation = self._ordinary_stop_operations.get(operation_id)
                if operation is not None:
                    operation['inflight_publications'].discard(
                        publication_index
                    )
        return started

    def _ordinary_stop_publish_returned(
        self, operation_id, publication_index, accepted
    ):
        """Advance one ordinary stop only after its ROS publish returned."""
        should_arm_final_zero = False
        failure = None
        with self._safety_zero_lock:
            operation = self._ordinary_stop_operations.get(operation_id)
            if operation is None:
                return
            operation['inflight_publications'].discard(publication_index)
            if accepted:
                operation['pending_publications'].discard(publication_index)
            else:
                publication = operation['publications'].get(publication_index)
                if publication is not None:
                    operation['failure_reporting'].add(publication_index)
                    failure = (
                        operation.get('task_id'),
                        operation.get('context', 'stop'),
                        publication[0],
                    )
            should_arm_final_zero = (
                not operation['pending_publications']
                and operation['required_zero_receipts'] is None
            )
        if failure is not None:
            self._record_ordinary_stop_delivery_failure(
                operation_id,
                publication_index,
                *failure
            )
        if should_arm_final_zero:
            self._arm_ordinary_stop_final_zero(operation_id)

    def _record_ordinary_stop_delivery_failure(
        self, operation_id, publication_index, task_id, context, label
    ):
        """Expose a failed delivery before allowing that stop to retry."""
        try:
            with self.task_lock:
                with self._safety_zero_lock:
                    operation = self._ordinary_stop_operations.get(
                        operation_id
                    )
                    if (
                        operation is None
                        or publication_index
                        not in operation['failure_reporting']
                    ):
                        return
                    safety_override = (
                        self._shutdown_started
                        or self.supervised_stop_active
                        or self.emergency_stop_active
                    )
                    if not safety_override and self.current_task_id == task_id:
                        self.task_state = TaskState.FAILED
                        self.task_progress = 0.0
                        self.task_error = (
                            "Task {} publication failed for {}; the stop "
                            "remains latched for retry"
                        ).format(context.replace('-', ' '), label)
                        self.task_dispatched = False
                        self.task_dispatched_at = None
                        self.last_behavior_status_at = None
        finally:
            with self._safety_zero_lock:
                operation = self._ordinary_stop_operations.get(operation_id)
                if operation is not None:
                    operation['failure_reporting'].discard(publication_index)

    def _retry_ordinary_stop_publications(self):
        """Retry stop messages which never obtained a worker or returned."""
        with self._safety_zero_lock:
            pending = tuple(
                (operation_id, publication_index)
                for operation_id, operation
                in self._ordinary_stop_operations.items()
                for publication_index in operation['pending_publications']
                if publication_index
                not in operation['inflight_publications']
                and publication_index not in operation['failure_reporting']
            )
        for operation_id, publication_index in pending:
            self._schedule_ordinary_stop_publication(
                operation_id, publication_index
            )

    def _arm_ordinary_stop_final_zero(self, operation_id):
        """Require one zero return after every lifecycle stop returned."""
        with self._safety_zero_lock:
            operation = self._ordinary_stop_operations.get(operation_id)
            if (
                operation is None
                or operation['pending_publications']
                or operation['required_zero_receipts'] is not None
            ):
                return
            publishers = tuple(self._safety_publisher_snapshot)
            required_zero_receipts = {}
            for robot_id, publisher in publishers:
                receipt_key = (robot_id, id(publisher))
                lane_key = receipt_key + ('supervisor',)
                receipt_count = self._safety_zero_receipts.get(
                    receipt_key, 0
                )
                if lane_key in self._safety_zero_inflight:
                    # That zero was physically started before every lifecycle
                    # stop returned. Count it, then require a fresh reassertion
                    # on the exact same publisher lane.
                    required_zero_receipts[receipt_key] = receipt_count + 2
                    self._safety_zero_followups.add(lane_key)
                else:
                    required_zero_receipts[receipt_key] = receipt_count + 1
            operation['required_zero_receipts'] = required_zero_receipts

        self._schedule_safety_zero_batch(publishers)
        self._complete_ordinary_stop_operations()

    def _complete_ordinary_stop_operations(self):
        """Release ordinary stop gates whose final zero really returned."""
        with self._safety_zero_lock:
            completed = []
            for operation_id, operation in (
                self._ordinary_stop_operations.items()
            ):
                required = operation['required_zero_receipts']
                if operation['pending_publications'] or required is None:
                    continue
                if all(
                    self._safety_zero_receipts.get(key, 0) >= count
                    for key, count in required.items()
                ):
                    completed.append(operation_id)
            for operation_id in completed:
                self._ordinary_stop_operations.pop(operation_id, None)
            self.ordinary_stop_active = bool(
                self._ordinary_stop_operations
            )

    def _compensate_late_lifecycle_message(self, task_id, context):
        """Put stops after a start, resume or reset which returned too late."""
        self._install_emergency_publication_debts(task_id, context)
        self._schedule_safety_zero_batch()

    def _install_emergency_publication_debts(self, task_id, context):
        """Keep failed e-stop and behavior-stop messages eligible for retry."""
        publications = [
            (
                ('emergency', id(self.emergency_stop_pub)),
                '{}-emergency'.format(context),
                self.emergency_stop_pub,
                Bool(data=True),
            ),
        ]
        if task_id:
            stop_message = String(data=json.dumps({'task_id': task_id}))
            publications.extend(
                (
                    ('stop', index, id(publisher)),
                    '{}-stop-{}'.format(context, index),
                    publisher,
                    stop_message,
                )
                for index, publisher
                in enumerate(tuple(self.behavior_stop_pubs))
            )

        with self._safety_zero_lock:
            for key, label, publisher, message in publications:
                self._emergency_publication_debts.setdefault(
                    key, (label, publisher, message)
                )
        self._retry_emergency_publications()

    def _retry_emergency_publications(self):
        """Retry latched lifecycle messages after worker or publish failure."""
        with self._safety_zero_lock:
            pending = tuple(
                key for key in self._emergency_publication_debts
                if key not in self._emergency_publication_inflight
            )
        for key in pending:
            self._schedule_emergency_publication(key)

    def _schedule_emergency_publication(self, key):
        with self._safety_zero_lock:
            publication = self._emergency_publication_debts.get(key)
            if publication is None:
                return True
            if key in self._emergency_publication_inflight:
                return True
            self._emergency_publication_inflight.add(key)

        label, publisher, message = publication
        completion = lambda accepted: self._emergency_publish_returned(
            key, accepted
        )
        started = self._start_detached_safety_publish(
            label, publisher, message, on_return=completion
        )
        if not started:
            with self._safety_zero_lock:
                self._emergency_publication_inflight.discard(key)
        return started

    def _emergency_publish_returned(self, key, accepted):
        with self._safety_zero_lock:
            self._emergency_publication_inflight.discard(key)
            if accepted:
                self._emergency_publication_debts.pop(key, None)

    def _ensure_emergency_ordering(self):
        """Return the ordering condition, including for ``__new__`` tests."""
        with self._EMERGENCY_ORDER_INIT_LOCK:
            condition = getattr(
                self, '_emergency_order_condition', None
            )
            if condition is None:
                condition = threading.Condition()
                self._emergency_order_condition = condition
            if not hasattr(self, '_emergency_order_generation'):
                self._emergency_order_generation = 0
            if not hasattr(
                self, '_pending_emergency_true_generations'
            ):
                self._pending_emergency_true_generations = set()
        return condition

    def _watchdog_can_latch_locked(self, now):
        """Check the watchdog trip predicate under ``_safety_zero_lock``."""
        if (
            not self.control_watchdog_enabled
            or not self.control_heartbeat_seen
            or self.last_control_heartbeat is None
            or self.control_watchdog_tripped
            or self._shutdown_started
        ):
            return False
        return (
            now - self.last_control_heartbeat
            > self.control_heartbeat_timeout
        )

    def _begin_emergency_true(self, watchdog=False, now=None):
        """Register and latch one True before it can wait for task state."""
        condition = self._ensure_emergency_ordering()
        with condition:
            with self._safety_zero_lock:
                if watchdog and not self._watchdog_can_latch_locked(now):
                    return False, None, None
                if self._shutdown_started:
                    return False, None, None
                self._emergency_order_generation += 1
                generation = self._emergency_order_generation
                self._pending_emergency_true_generations.add(generation)
                latched, task_id = self._latch_emergency_stop_locked(
                    watchdog
                )
        return latched, task_id, generation

    def _finish_emergency_true(self, generation):
        """Release resets that entered after this True was registered."""
        condition = self._ensure_emergency_ordering()
        with condition:
            self._pending_emergency_true_generations.discard(generation)
            condition.notify_all()

    def _capture_emergency_reset_generation(self):
        """Wait for True operations that preceded this reset invocation."""
        condition = self._ensure_emergency_ordering()
        with condition:
            generation = self._emergency_order_generation
            while any(
                pending <= generation
                for pending in self._pending_emergency_true_generations
            ):
                condition.wait()
        return generation, condition

    def _latch_emergency_stop_locked(self, watchdog=False):
        """Close the motion gate while ``_safety_zero_lock`` is held."""
        if self._shutdown_started:
            return False, None
        self.emergency_stop_active = True
        if watchdog:
            self.control_watchdog_tripped = True
        return True, self.current_task_id

    def _commit_emergency_stop_state(self):
        """Reconcile task state after the lock-free safety fan-out began."""
        with self.task_lock:
            with self._safety_zero_lock:
                if (
                    self._shutdown_started
                    or self.supervised_stop_active
                    or not self.emergency_stop_active
                ):
                    return False
                self.task_state = TaskState.STOPPED
                self.task_error = None
                self.task_progress = 0.0
                self.task_dispatched = False
                self.task_dispatched_at = None
                self.last_behavior_status_at = None
        return True

    def _latch_and_publish_emergency_stop(
        self, context, watchdog=False, now=None
    ):
        """Order, latch and commit a True through one shared path."""
        latched, task_id, generation = self._begin_emergency_true(
            watchdog=watchdog, now=now
        )
        if not latched:
            return False, False
        try:
            self._compensate_late_lifecycle_message(
                task_id, context
            )
            committed = self._commit_emergency_stop_state()
        finally:
            self._finish_emergency_true(generation)
        return True, committed

    def _handle_emergency_stop(self, params):
        latched, committed = self._latch_and_publish_emergency_stop(
            'emergency-stop'
        )
        if latched and committed:
            rospy.logwarn("EMERGENCY STOP ACTIVATED")

    def _safety_stop_request_callback(self, msg):
        """Latch a behavior shutdown request before any task lock is taken."""
        validation_errors = []
        try:
            payload = json.loads(msg.data)
        except (AttributeError, TypeError, ValueError):
            payload = None
        if not isinstance(payload, dict):
            payload = {}
            validation_errors.append('payload is not a JSON object')

        source = payload.get('source')
        reason = payload.get('reason')
        requested_task_id = payload.get('task_id')
        request_id = payload.get('request_id')
        active_task_id = self.current_task_id
        active_task_type = self.current_task_type
        if source != 'formation':
            validation_errors.append('source is not formation')
        if reason != 'shutdown':
            validation_errors.append('reason is not shutdown')
        if requested_task_id is not None and (
            not isinstance(requested_task_id, str)
            or not requested_task_id
        ):
            validation_errors.append('task_id is malformed')
            requested_task_id = None
        if requested_task_id != active_task_id:
            validation_errors.append('task_id does not match the active task')
        if not isinstance(request_id, str) or not request_id:
            validation_errors.append('request_id is malformed')
            request_id = None
        if active_task_type not in (None, 'formation'):
            validation_errors.append('active task is not a formation')

        context = {
            'source': source if isinstance(source, str) else 'unknown',
            'reason': reason if isinstance(reason, str) else 'invalid_request',
            'task_id': requested_task_id,
            'request_id': request_id,
            'active_task_id': active_task_id,
            'active_task_type': active_task_type,
            'valid': not validation_errors,
        }
        if validation_errors:
            context['validation_error'] = '; '.join(validation_errors)[:256]

        with self._safety_zero_lock:
            if self.supervised_stop_context is None:
                self.supervised_stop_context = context
            self.supervised_stop_active = True
            # Set this atomic gate before task_lock. Start/resume paths consult
            # both bits before they can dispatch behavior work.
            self.emergency_stop_active = True

        with self._safety_zero_lock:
            safety_publishers = tuple(self._safety_publisher_snapshot)
            required_zero_receipts = {
                (robot_id, id(publisher)): (
                    self._safety_zero_receipts.get(
                        (robot_id, id(publisher)), 0
                    ) + 1
                )
                for robot_id, publisher in safety_publishers
            }
        zero_schedule = self._schedule_safety_zero_batch(safety_publishers)
        state_worker_started = self._start_supervised_state_worker()
        if request_id is not None:
            pending = {
                'request_id': request_id,
                'valid_request': not validation_errors,
                'required_zero_receipts': required_zero_receipts,
                'scheduled_publisher_ids': set(
                    zero_schedule['scheduled_publisher_ids']
                ),
            }
            if pending['valid_request']:
                with self._safety_zero_lock:
                    if len(self._pending_safety_stop_acks) >= 32:
                        oldest = next(iter(self._pending_safety_stop_acks))
                        self._pending_safety_stop_acks.pop(oldest, None)
                    self._pending_safety_stop_acks[request_id] = pending
            self._schedule_safety_stop_ack(pending)

    def _publish_safety_stop_ack(self, pending):
        """Publish one correlated ACK and report a live reverse connection."""
        with self._safety_zero_lock:
            supervisor_latched = self.supervised_stop_active
            state_worker_started = (
                self._supervised_stop_state_started
                and not self._shutdown_started
            )
            supervisor_will_remain_active = (
                supervisor_latched
                and state_worker_started
            )
            required_receipts = dict(
                pending.get('required_zero_receipts', {})
            )
            scheduled_publishers = set(
                pending.get('scheduled_publisher_ids', ())
            )
            receipt_snapshot = dict(self._safety_zero_receipts)
        zero_publish_return_count = sum(
            receipt_snapshot.get(publisher_id, 0) >= required_count
            for publisher_id, required_count
            in required_receipts.items()
        )
        zero_publications_confirmed = (
            zero_publish_return_count == len(required_receipts)
        )
        scheduling_confirmed = all(
            publisher_id in scheduled_publishers
            or receipt_snapshot.get(publisher_id, 0) >= required_count
            for publisher_id, required_count
            in required_receipts.items()
        )
        accepted = (
            pending['valid_request']
            and scheduling_confirmed
            and zero_publications_confirmed
            and state_worker_started
            and supervisor_will_remain_active
        )
        message = String(data=json.dumps({
            'request_id': pending['request_id'],
            'accepted': accepted,
            'supervisor_latched': supervisor_latched,
            'supervisor_will_remain_active': (
                supervisor_will_remain_active
            ),
            'valid_request': pending['valid_request'],
            'zero_worker_count': len(scheduled_publishers),
            'zero_target_count': len(required_receipts),
            'zero_publish_return_count': zero_publish_return_count,
            'zero_publications_confirmed': zero_publications_confirmed,
            'state_worker_started': state_worker_started,
        }))
        connection_count = getattr(
            self.safety_stop_ack_pub, 'get_num_connections', None
        )
        subscriber_connected = None
        if callable(connection_count):
            try:
                subscriber_connected = connection_count() > 0
            except Exception:
                subscriber_connected = None
        try:
            self.safety_stop_ack_pub.publish(message)
        except Exception as exc:
            try:
                rospy.logerr(
                    "Safety-stop acknowledgement failed: %s", exc
                )
            except Exception:
                pass
            return {'accepted': accepted, 'delivered': False}
        return {
            'accepted': accepted,
            'delivered': subscriber_connected is not False,
        }

    def _schedule_safety_stop_ack(self, pending):
        """Coalesce one ACK on its prestarted, publisher-specific lane."""
        request_id = pending['request_id']
        with self._safety_zero_lock:
            if request_id in self._safety_ack_publish_inflight:
                self._safety_ack_retry_requested.add(request_id)
                return True
            self._safety_ack_publish_inflight.add(request_id)

        def publish_ack():
            acknowledgement = self._publish_safety_stop_ack(pending)
            reschedule = False
            with self._safety_zero_lock:
                current = self._pending_safety_stop_acks.get(request_id)
                if (
                    acknowledgement['accepted']
                    and acknowledgement['delivered']
                    and current is pending
                ):
                    self._pending_safety_stop_acks.pop(request_id, None)
                    current = None
                reschedule = (
                    current is pending
                    and request_id in self._safety_ack_retry_requested
                )
                self._safety_ack_retry_requested.discard(request_id)
                self._safety_ack_publish_inflight.discard(request_id)
            if reschedule:
                self._schedule_safety_stop_ack(pending)

        if self._submit_safety_fallback(
            self.safety_stop_ack_pub, publish_ack
        ):
            return True
        with self._safety_zero_lock:
            self._safety_ack_publish_inflight.discard(request_id)
        try:
            rospy.logerr(
                "Safety-stop acknowledgement could not be scheduled for %s",
                request_id,
            )
        except Exception:
            pass
        return False

    def _retry_pending_safety_stop_acks(self, zero_schedule=None):
        """Republish an ACK after scheduling, delivery, or link recovery."""
        with self._safety_zero_lock:
            if zero_schedule is not None:
                recovered_publishers = set(
                    zero_schedule['scheduled_publisher_ids']
                )
                for pending in self._pending_safety_stop_acks.values():
                    pending['scheduled_publisher_ids'].update(
                        recovered_publishers
                    )
            pending_acks = tuple(self._pending_safety_stop_acks.values())
        for pending in pending_acks:
            self._schedule_safety_stop_ack(pending)

    def _start_supervised_state_worker(self):
        """Reserve one state worker and leave a failed start retryable."""
        with self._safety_zero_lock:
            if self._shutdown_started or not self.supervised_stop_active:
                return False
            if self._supervised_stop_state_started:
                return True
            self._supervised_stop_state_started = True

        for attempt in range(2):
            worker = threading.Thread(
                target=self._apply_supervised_stop_state,
                name='supervised-safety-stop-state',
                daemon=True,
            )
            try:
                worker.start()
                return True
            except Exception as exc:
                try:
                    rospy.logerr(
                        "Could not start supervised-stop state worker "
                        "(attempt %d): %s",
                        attempt + 1,
                        exc,
                    )
                except Exception:
                    pass

        with self._safety_zero_lock:
            self._supervised_stop_state_started = False
        return False

    def _apply_supervised_stop_state(self):
        """Correlate the already-latched safety stop with task lifecycle."""
        with self.task_lock:
            context = dict(self.supervised_stop_context or {})
            self.emergency_stop_active = True
            try:
                self.emergency_stop_pub.publish(Bool(data=True))
            except Exception as exc:
                rospy.logerr(
                    "Supervised emergency-stop signal failed: %s", exc
                )
            try:
                self._signal_stop_behaviors()
            except Exception as exc:
                rospy.logerr(
                    "Supervised behavior-stop signal failed: %s", exc
                )
            if self.current_task_id is None:
                self.task_state = TaskState.STOPPED
            else:
                self.task_state = TaskState.FAILED
                self.task_error = (
                    "Independent safety supervisor latched a stop from "
                    "{} ({})".format(
                        context.get('source', 'unknown'),
                        context.get('reason', 'unknown'),
                    )
                )
            self.task_progress = 0.0
            self.task_dispatched = False
            self.task_dispatched_at = None
            self.last_behavior_status_at = None

    def _schedule_safety_zero_batch(self, publishers=None):
        """Give each independent command socket at most one in-flight zero."""
        if publishers is None:
            publishers = tuple(self._safety_publisher_snapshot)
        else:
            publishers = tuple(publishers)
        scheduled_count = 0
        scheduled_publisher_ids = []
        failed_robots = []
        for robot_id, publisher in publishers:
            if self._schedule_safety_zero(
                robot_id, publisher, lane='supervisor'
            ):
                scheduled_count += 1
                scheduled_publisher_ids.append(
                    (robot_id, id(publisher))
                )
            else:
                failed_robots.append(robot_id)
        return {
            'requested_count': len(publishers),
            'scheduled_count': scheduled_count,
            'scheduled_publisher_ids': scheduled_publisher_ids,
            'failed_robots': failed_robots,
            'scheduling_confirmed': not failed_robots,
        }

    def _schedule_safety_zero(self, robot_id, publisher, lane):
        """Start one zero attempt unless that exact socket is still busy."""
        key = (robot_id, id(publisher), lane)
        with self._safety_zero_lock:
            if key in self._safety_zero_inflight:
                return True
            self._safety_zero_inflight.add(key)
        last_error = None
        for attempt in range(2):
            worker = threading.Thread(
                target=self._publish_supervised_zero,
                args=(key, robot_id, publisher),
                name='supervised-zero-{}-{}'.format(
                    robot_id, attempt + 1
                ),
                daemon=True,
            )
            try:
                worker.start()
                return True
            except Exception as exc:
                last_error = exc
                try:
                    rospy.logerr(
                        "Could not start safety-zero worker for %s "
                        "(attempt %d): %s",
                        robot_id,
                        attempt + 1,
                        exc,
                    )
                except Exception:
                    pass
        if self._submit_safety_fallback(
            publisher,
            lambda: self._publish_supervised_zero(
                key, robot_id, publisher
            ),
        ):
            return True
        with self._safety_zero_lock:
            self._safety_zero_inflight.discard(key)
        if last_error is not None:
            try:
                rospy.logerr(
                    "Safety-zero publication was not scheduled for %s: %s",
                    robot_id,
                    last_error,
                )
            except Exception:
                pass
        return False

    def _schedule_retired_safety_zeros(self):
        """Retry only publishers which still owe a final accepted zero."""
        with self._safety_zero_lock:
            retired = tuple(self._retired_safety_publishers.values())
        for robot_id, publisher in retired:
            self._schedule_safety_zero(
                robot_id, publisher, lane='retirement'
            )

    def _publish_supervised_zero(self, key, robot_id, publisher):
        accepted = False
        retired = None
        schedule_followup = False
        try:
            publisher.publish(Twist())
            accepted = True
        except Exception as exc:
            try:
                rospy.logerr(
                    "Independent safety zero failed for %s: %s",
                    robot_id,
                    exc,
                )
            except Exception:
                pass
        finally:
            retired_key = (robot_id, id(publisher))
            with self._safety_zero_lock:
                self._safety_zero_inflight.discard(key)
                if accepted:
                    self._safety_zero_receipts[retired_key] = (
                        self._safety_zero_receipts.get(retired_key, 0) + 1
                    )
                    retired = self._retired_safety_publishers.pop(
                        retired_key, None
                    )
                    if retired is not None:
                        self._safety_publisher_snapshot = tuple(
                            (name, candidate)
                            for name, candidate
                            in self._safety_publisher_snapshot
                            if not (
                                name == robot_id
                                and candidate is publisher
                            )
                        )
                if key in self._safety_zero_followups:
                    self._safety_zero_followups.discard(key)
                    schedule_followup = True
        if schedule_followup:
            self._schedule_safety_zero(
                robot_id, publisher, lane=key[2]
            )
        if retired is not None:
            try:
                publisher.unregister()
            except Exception as exc:
                try:
                    rospy.logwarn(
                        "Retired command publisher for %s could not be "
                        "unregistered: %s",
                        robot_id,
                        exc,
                    )
                except Exception:
                    pass
            self._discard_safety_fallback_lane(publisher)
        if accepted:
            self._retry_pending_safety_stop_acks()
            self._complete_ordinary_stop_operations()

    def _safety_stop_zero_timer(self, _event):
        """Reassert zeros while any emergency stop remains latched."""
        if (
            self.supervised_stop_active
            or self.emergency_stop_active
            or self.ordinary_stop_active
        ):
            if self.ordinary_stop_active:
                self._retry_ordinary_stop_publications()
            if self.emergency_stop_active:
                self._retry_emergency_publications()
            zero_schedule = self._schedule_safety_zero_batch()
            if self.supervised_stop_active:
                self._start_supervised_state_worker()
                self._retry_pending_safety_stop_acks(zero_schedule)
            if self.ordinary_stop_active:
                self._complete_ordinary_stop_operations()
        else:
            self._schedule_retired_safety_zeros()

    def _handle_reset_emergency_stop(self, params):
        reset_generation, order_condition = (
            self._capture_emergency_reset_generation()
        )
        task_id = None
        publish_error = None
        late_safety_latch = False
        with self.task_lock:
            # Keep the supervised-latch check and the local reset commit in
            # one safety critical section. The request callback never holds
            # this lock while waiting for task_lock, so this order is acyclic.
            with order_condition:
                with self._safety_zero_lock:
                    if (
                        self._emergency_order_generation
                        != reset_generation
                    ):
                        rospy.logwarn(
                            "Emergency-stop reset rejected because a newer "
                            "stop was latched"
                        )
                        return
                    if self.supervised_stop_active:
                        rospy.logwarn(
                            "Emergency-stop reset rejected after a "
                            "supervised behavior shutdown"
                        )
                        return
                    if not self.emergency_stop_active:
                        rospy.logwarn(
                            "Emergency-stop reset ignored because no stop "
                            "is latched"
                        )
                        return
                    if self._emergency_publication_debts:
                        rospy.logwarn(
                            "Emergency-stop reset rejected while stop "
                            "messages are still pending delivery"
                        )
                        return
                    now = self._control_clock()
                    if (
                        self.control_watchdog_enabled
                        and self.control_heartbeat_seen
                        and self.last_control_heartbeat is not None
                        and now - self.last_control_heartbeat
                        > self.control_heartbeat_timeout
                    ):
                        rospy.logwarn(
                            "Emergency-stop reset rejected while the "
                            "control heartbeat is stale"
                        )
                        return
                    self.emergency_stop_active = False
                    self.control_watchdog_tripped = False
            task_id = self.current_task_id
            try:
                self.emergency_stop_pub.publish(Bool(data=False))
            except Exception as exc:
                publish_error = exc
            with order_condition:
                with self._safety_zero_lock:
                    late_safety_latch = (
                        self._shutdown_started
                        or self.supervised_stop_active
                        or self.emergency_stop_active
                        or self._emergency_order_generation
                        != reset_generation
                    )
                    if late_safety_latch or publish_error is not None:
                        self.emergency_stop_active = True
                        self.control_watchdog_tripped = True
                    if late_safety_latch or publish_error is not None:
                        self.task_state = TaskState.FAILED
                        self.task_error = (
                            "Emergency-stop reset returned after the safety "
                            "latch closed"
                            if late_safety_latch
                            else str(publish_error)
                        )
                    else:
                        self.task_state = TaskState.STOPPED
                        self.task_error = None
                    self.task_progress = 0.0
                    self.task_dispatched = False
                    self.task_dispatched_at = None
                    self.last_behavior_status_at = None
            if late_safety_latch or publish_error is not None:
                self._compensate_late_lifecycle_message(
                    task_id,
                    (
                        'late-reset'
                        if late_safety_latch
                        else 'failed-reset'
                    ),
                )
            zero_schedule = self._stop_all_robots()
            if not zero_schedule['scheduling_confirmed']:
                with self._safety_zero_lock:
                    self.emergency_stop_active = True
                    self.control_watchdog_tripped = True
                    self.task_state = TaskState.FAILED
                    self.task_error = (
                        "Emergency-stop reset could not schedule every final "
                        "zero publication"
                    )
                late_safety_latch = True

        if late_safety_latch or publish_error is not None:
            if publish_error is not None:
                rospy.logerr(
                    "Emergency-stop reset publication failed: %s",
                    publish_error,
                )
            return
        rospy.logwarn("Emergency stop reset; a new task must be started explicitly")

    # ==================== Control-plane Watchdog ====================

    def _control_heartbeat_callback(self, _msg):
        """Arm/refresh the worker lease using local monotonic arrival time."""
        with self._safety_zero_lock:
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
        current_time = self._control_clock() if now is None else now
        with self._safety_zero_lock:
            if (
                not self.control_watchdog_enabled
                or not self.control_heartbeat_seen
                or self.last_control_heartbeat is None
                or self.control_watchdog_tripped
                or self._shutdown_started
            ):
                return False

            age = current_time - self.last_control_heartbeat
            if age <= self.control_heartbeat_timeout:
                return False

        latched, _committed = self._latch_and_publish_emergency_stop(
            'control-watchdog', watchdog=True, now=current_time
        )
        if not latched:
            return False

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
        with self._safety_zero_lock:
            stopped = (
                self.supervised_stop_active
                or self.emergency_stop_active
            )
        if not stopped:
            cmd.linear.x = max(
                -0.26, min(0.26, params.get('linear_velocity', 0.0))
            )
            cmd.angular.z = max(
                -1.82, min(1.82, params.get('angular_velocity', 0.0))
            )
        try:
            self.leader_cmd_pub.publish(cmd)
        finally:
            # A request can latch while a previously admitted ROS publish is
            # blocked. Once that call returns, put a zero after it instead of
            # letting its late velocity become the final message.
            if not stopped:
                with self._safety_zero_lock:
                    stopped_after_publish = (
                        self.supervised_stop_active
                        or self.emergency_stop_active
                    )
                if stopped_after_publish:
                    self._schedule_safety_zero(
                        'leader',
                        self.leader_cmd_pub,
                        lane='manual-compensation',
                    )

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

            self.robot_count = len(self.robots)
            if (
                not self.robots
                and self.current_task_type is not None
                and self.task_state in (
                    TaskState.INITIALIZING,
                    TaskState.RUNNING,
                    TaskState.PAUSED,
                )
            ):
                stop_delivery = self._signal_stop_behaviors()
                with self._safety_zero_lock:
                    if (
                        self._shutdown_started
                        or self.supervised_stop_active
                    ):
                        return
                    if stop_delivery['scheduling_confirmed']:
                        self.task_state = TaskState.STOPPED
                        self.task_error = None
                    else:
                        self.task_state = TaskState.FAILED
                        self.task_error = (
                            "Fleet loss could not schedule every safety "
                            "publication"
                        )
                    self.task_progress = 0.0
                    self.task_dispatched = False
                    self.task_dispatched_at = None
                    self.last_behavior_status_at = None
                    cancelled_task_id = self.current_task_id
                rospy.logwarn(
                    "Cancelled task %s because no announced robot could "
                    "remain active",
                    cancelled_task_id,
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
            self.last_behavior_status = status

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

        arrival_tolerance = self._transport_source_number(
            status.get('arrival_tolerance')
        )
        if (
            arrival_tolerance is None
            or not 0.15 <= arrival_tolerance <= 0.75
        ):
            arrival_tolerance = None

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
            'arrival_tolerance': arrival_tolerance,
            'target_marker': self._transport_target_marker(
                status.get('target_marker')
            ),
            'discovery': self._transport_discovery(
                status.get('discovery'), expected_task_id
            ),
        }
        return {'transport': result}

    @staticmethod
    def _transport_target_marker(marker):
        """Copy only bounded facts about the optional Gazebo target ghost."""
        if not isinstance(marker, dict):
            return None

        model_name = marker.get('model_name')
        command_published = marker.get('command_published')
        synchronized = marker.get('synchronized')
        if (
            model_name != 'target_marker'
            or not isinstance(command_published, bool)
            or not isinstance(synchronized, bool)
        ):
            return None

        position = marker.get('position')
        clean_position = None
        if position is not None:
            if not isinstance(position, dict):
                return None
            x = position.get('x')
            y = position.get('y')
            if (
                isinstance(x, bool)
                or isinstance(y, bool)
                or not isinstance(x, (int, float))
                or not isinstance(y, (int, float))
            ):
                return None
            x = float(x)
            y = float(y)
            if (
                not math.isfinite(x)
                or not math.isfinite(y)
                or not -4.0 <= x <= 4.0
                or not -4.0 <= y <= 4.0
            ):
                return None
            clean_position = {'x': x, 'y': y}

        if synchronized and clean_position is None:
            return None

        return {
            'model_name': 'target_marker',
            'published': command_published,
            'synchronized': synchronized,
            'position': clean_position,
        }

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
            stop_delivery = self._signal_stop_behaviors()
            with self._safety_zero_lock:
                if self._shutdown_started or self.supervised_stop_active:
                    return
                self.task_state = TaskState.FAILED
                self.task_progress = 0.0
                self.task_error = reason
                if not stop_delivery['scheduling_confirmed']:
                    self.task_error += (
                        "; every safety publication could not be scheduled"
                    )
                self.task_dispatched = False
                self.task_dispatched_at = None
                self.last_behavior_status_at = None
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
            stop_delivery = self._signal_stop_behaviors()
            with self._safety_zero_lock:
                if self._shutdown_started or self.supervised_stop_active:
                    return
                if stop_delivery['scheduling_confirmed']:
                    self.task_state = TaskState.COMPLETED
                    self.task_progress = 1.0
                    self.task_error = None
                else:
                    self.task_state = TaskState.FAILED
                    self.task_progress = 0.0
                    self.task_error = (
                        "Task completion could not schedule every safety "
                        "publication"
                    )
                self.task_dispatched = False
                self.task_dispatched_at = None
                self.last_behavior_status_at = None
        rospy.loginfo("Task %s completed", self.current_task_type)

    def _signal_stop_behaviors(self):
        if not self.current_task_id:
            return self._start_ordinary_stop_publications(
                'task-stop', ()
            )
        payload = String(data=json.dumps({
            'task_id': self.current_task_id,
        }))
        publications = tuple(
            ('behavior-{}'.format(index), publisher, payload)
            for index, publisher in enumerate(tuple(self.behavior_stop_pubs))
        )
        return self._start_ordinary_stop_publications(
            'task-stop', publications
        )

    def _subscribe_to_robot(self, robot_id):
        with self._safety_zero_lock:
            if self._shutdown_started or self.supervised_stop_active:
                rospy.logwarn(
                    "Ignoring robot %s after the safety latch closed",
                    robot_id,
                )
                return
        if robot_id in self.robots:
            return

        robot_state = {
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
        command_pub = None
        subscribers = {}
        try:
            # This publisher only carries safety zeros. Synchronous delivery
            # keeps shutdown from mistaking a local queue insert for a socket
            # write.
            command_pub = rospy.Publisher(f'/{robot_id}/cmd_vel', Twist)
            if not self._register_safety_fallback_lane(
                robot_id, command_pub
            ):
                raise RuntimeError(
                    "the safety publication lane could not be reserved"
                )
            subscribers['odom'] = rospy.Subscriber(
                f'/{robot_id}/odom', Odometry,
                lambda msg, rid=robot_id: self._odom_cb(rid, msg), queue_size=1
            )
            subscribers['threat'] = rospy.Subscriber(
                f'/{robot_id}/threat_level', Float32,
                lambda msg, rid=robot_id: self._threat_cb(rid, msg),
                queue_size=1,
            )
            subscribers['collision'] = rospy.Subscriber(
                f'/{robot_id}/collision_state', Bool,
                lambda msg, rid=robot_id: self._collision_cb(rid, msg),
                queue_size=1,
            )
            subscribers['scan'] = rospy.Subscriber(
                f'/{robot_id}/scan', LaserScan,
                lambda msg, rid=robot_id: self._scan_cb(rid, msg), queue_size=1
            )
        except Exception as exc:
            for subscriber in subscribers.values():
                try:
                    subscriber.unregister()
                except Exception:
                    pass
            if command_pub is not None:
                self._discard_safety_fallback_lane(command_pub)
                try:
                    command_pub.unregister()
                except Exception:
                    pass
            rospy.logerr("Could not subscribe to robot %s: %s", robot_id, exc)
            return

        installed = False
        with self._safety_zero_lock:
            if (
                not self._shutdown_started
                and not self.supervised_stop_active
                and robot_id not in self.robots
            ):
                self.robots[robot_id] = robot_state
                self.robot_sensor_data[robot_id] = {}
                self.cmd_vel_pubs[robot_id] = command_pub
                self.odom_subs[robot_id] = subscribers['odom']
                self.threat_subs[robot_id] = subscribers['threat']
                self.collision_subs[robot_id] = subscribers['collision']
                self.scan_subs[robot_id] = subscribers['scan']
                self._refresh_safety_publisher_snapshot_locked()
                installed = True

        if installed:
            return

        # Resources created during a closing safety gate were never exposed as
        # command sources, so they carry no motion debt. Dispose of them rather
        # than letting a late robot escape the immutable shutdown snapshot.
        for subscriber in subscribers.values():
            try:
                subscriber.unregister()
            except Exception:
                pass
        try:
            command_pub.unregister()
        except Exception:
            pass
        self._discard_safety_fallback_lane(command_pub)
        rospy.logwarn(
            "Discarded robot %s because the safety latch closed during setup",
            robot_id,
        )

    def _unsubscribe_from_robot(self, robot_id):
        # Keep the command socket reachable until one zero has been accepted.
        # Removing it from the active fleet first is safe because the previous
        # immutable snapshot still contains it until the retired entry is
        # installed below.
        command_pub = self.cmd_vel_pubs.pop(robot_id, None)
        if command_pub is not None:
            key = (robot_id, id(command_pub))
            with self._safety_zero_lock:
                self._retired_safety_publishers[key] = (
                    robot_id, command_pub
                )
            self._refresh_safety_publisher_snapshot()
            self._schedule_safety_zero(
                robot_id, command_pub, lane='retirement'
            )

        for subs_dict in [
            self.odom_subs,
            self.threat_subs,
            self.collision_subs,
            self.scan_subs,
        ]:
            sub = subs_dict.pop(robot_id, None)
            if sub:
                sub.unregister()
        self.robots.pop(robot_id, None)
        self.robot_sensor_data.pop(robot_id, None)

    def _refresh_safety_publisher_snapshot(self):
        """Replace the lock-free roster without dropping stop debts."""
        with self._safety_zero_lock:
            self._refresh_safety_publisher_snapshot_locked()

    def _refresh_safety_publisher_snapshot_locked(self):
        """Refresh the immutable safety roster while its lock is held."""
        active = list(self.cmd_vel_pubs.items())
        leader_publisher = getattr(self, 'leader_cmd_pub', None)
        if leader_publisher is not None:
            active.append(('leader', leader_publisher))
        candidates = active + list(
            self._retired_safety_publishers.values()
        )
        unique = {}
        for name, publisher in candidates:
            unique[(name, id(publisher))] = (name, publisher)
        self._safety_publisher_snapshot = tuple(sorted(
            unique.values(),
            key=lambda item: (item[0], id(item[1])),
        ))

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
            last_behavior_status = getattr(
                self, 'last_behavior_status', None
            )
            # Formation planning deliberately keeps every cmd_vel at zero.
            # Routed placement grows with the fleet and can exceed the short
            # moving-behavior heartbeat window. A correlated FORMING status
            # therefore gets one bounded planning window; as soon as any
            # assignment exists, the normal fail-fast timeout applies again.
            if (
                self.current_task_type == 'formation'
                and isinstance(last_behavior_status, dict)
                and str(last_behavior_status.get('task_id') or '')
                == str(self.current_task_id or '')
                and last_behavior_status.get('state') == 'forming'
                and not last_behavior_status.get('robot_assignments')
            ):
                robot_count = max(0, int(getattr(
                    self, 'robot_count', 0
                )))
                scaled_planning_timeout = 5.0 + 1.5 * robot_count
                behavior_timeout = min(
                    getattr(
                        self,
                        'formation_planning_status_timeout',
                        30.0,
                    ),
                    max(behavior_timeout, scaled_planning_timeout),
                )
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
            supervised_stop_active = self.supervised_stop_active
            supervised_stop_context = (
                dict(self.supervised_stop_context)
                if isinstance(self.supervised_stop_context, dict)
                else None
            )
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
            'supervised_stop': {
                'active': supervised_stop_active,
                'context': supervised_stop_context,
            },
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
        """Fan out zeros so a blocked socket cannot hide a healthy robot."""
        return self._schedule_safety_zero_batch()

    def _shutdown(self):
        """Fan out final zeros without trusting any one ROS socket."""
        timeout = max(0.0, float(self.SHUTDOWN_TIMEOUT))
        deadline = time.monotonic() + timeout

        # This gate is deliberately independent of task_lock. A behavior-state
        # callback may be blocked there while ROS is already shutting down.
        with self._safety_zero_lock:
            self._shutdown_started = True
            self.supervised_stop_active = True
            self.emergency_stop_active = True
            self._supervised_stop_state_started = True
            shutdown_task_id = self.current_task_id
            shutdown_task_type = self.current_task_type
            self.task_state = (
                TaskState.STOPPED
                if shutdown_task_id is None
                else TaskState.FAILED
            )
            self.task_progress = 0.0
            self.task_dispatched = False
            self.task_dispatched_at = None
            self.last_behavior_status_at = None
            if shutdown_task_id is not None:
                self.task_error = (
                    "Task orchestrator shut down under a latched safety stop"
                )
            if self.supervised_stop_context is None:
                self.supervised_stop_context = {
                    'source': 'task_orchestrator',
                    'reason': 'shutdown',
                    'task_id': shutdown_task_id,
                    'active_task_id': shutdown_task_id,
                    'active_task_type': shutdown_task_type,
                    'valid': True,
                }
            command_publishers = tuple(self._safety_publisher_snapshot)

        watchdog_stop = getattr(self, '_control_watchdog_stop', None)
        if watchdog_stop is None:
            watchdog_stop = threading.Event()
            self._control_watchdog_stop = watchdog_stop
        watchdog_stop.set()

        attempts = []
        failed_to_start = []

        def start_publish(label, publisher, message):
            finished = threading.Event()

            def publish():
                try:
                    publisher.publish(message)
                except Exception as exc:
                    try:
                        rospy.logerr(
                            "Shutdown publication failed for %s: %s",
                            label,
                            exc,
                        )
                    except Exception:
                        pass
                finally:
                    finished.set()

            started = False
            last_error = None
            for attempt in range(2):
                worker = threading.Thread(
                    target=publish,
                    name='orchestrator-shutdown-{}-{}'.format(
                        label, attempt + 1
                    ),
                    daemon=True,
                )
                try:
                    worker.start()
                    started = True
                    break
                except Exception as exc:
                    last_error = exc
                    try:
                        rospy.logerr(
                            "Could not start shutdown publication for %s "
                            "(attempt %d): %s",
                            label,
                            attempt + 1,
                            exc,
                        )
                    except Exception:
                        pass
            if not started and self._submit_safety_fallback(
                publisher, publish
            ):
                started = True
            if not started:
                failed_to_start.append(label)
                finished.set()
                try:
                    rospy.logerr(
                        "Shutdown publication was not started for %s: %s",
                        label,
                        last_error,
                    )
                except Exception:
                    pass
            attempts.append((label, finished))

        # Start every command attempt before touching behavior publishers.
        # A blocked first robot therefore cannot prevent healthy robots from
        # receiving their independent zero during the same shutdown window.
        for robot_id, publisher in command_publishers:
            start_publish(robot_id, publisher, Twist())

        start_publish(
            'emergency-stop',
            self.emergency_stop_pub,
            Bool(data=True),
        )

        if shutdown_task_id:
            stop_message = String(data=json.dumps({
                'task_id': shutdown_task_id,
            }))
            for index, publisher in enumerate(tuple(self.behavior_stop_pubs)):
                start_publish(
                    'behavior-stop-{}'.format(index),
                    publisher,
                    stop_message,
                )

        for _label, finished in attempts:
            finished.wait(max(0.0, deadline - time.monotonic()))

        unfinished = list(failed_to_start)
        unfinished.extend(
            label for label, finished in attempts if not finished.is_set()
        )
        if unfinished:
            try:
                rospy.logerr(
                    "Shutdown deadline expired before publication returned "
                    "for: %s",
                    ', '.join(unfinished),
                )
            except Exception:
                pass

        watchdog_thread = getattr(self, '_control_watchdog_thread', None)
        if (
            watchdog_thread is not None
            and watchdog_thread is not threading.current_thread()
        ):
            watchdog_thread.join(timeout=max(
                0.0, deadline - time.monotonic()
            ))


if __name__ == '__main__':
    try:
        orchestrator = TaskOrchestrator()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
