#!/usr/bin/env python3
"""
Formation Control Swarm Behavior for TurtleBot3 Burger robots in Gazebo

Implements geometric and letter-based formation shapes with static, moving, and
adaptive movement modes. Robots maintain formation while the centroid follows
configurable paths (circular, linear, waypoints). Includes global minimum-cost
assignment, PID-based differential drive control, obstacle avoidance integration,
and RViz marker visualization.

Supported shapes: triangle, square, circle, line, v_formation, diamond
Supported letters: A, B, C, D, E, F, G, H, I, J, K, L, M, N, O, P, R, S, T, U, V, W, X, Y, Z
"""

import rospy
import json
import math
import threading
from typing import List, Dict, Optional, Tuple
from enum import Enum

# ROS messages
from geometry_msgs.msg import Twist, Pose, Point, Quaternion
from nav_msgs.msg import Odometry
from std_msgs.msg import String, Bool
from visualization_msgs.msg import Marker, MarkerArray

# Import obstacle avoidance from core
import sys
import os
sys.path.append(os.path.join(os.path.dirname(__file__), '..'))
from core.obstacle_avoidance import ObstacleAvoidance
from utils.robot_ids import sort_robot_ids
from robot_swarm_bridge.algorithms.formation import (
    minimum_distance_assignment,
    sample_letter_formation,
)


# ---------------------------------------------------------------------------
# Letter definitions on a 5-column x 7-row grid (col, row).
# Row 0 is bottom, row 6 is top. Col 0 is left, col 4 is right.
# ---------------------------------------------------------------------------
LETTERS = {
    'A': [
        (0, 0), (0, 1), (0, 2), (0, 3), (0, 4), (0, 5),
        (1, 6), (2, 6),
        (3, 6), (4, 5), (4, 4), (4, 3), (4, 2), (4, 1), (4, 0),
        (1, 3), (2, 3), (3, 3),
    ],
    'B': [
        (0, 0), (0, 1), (0, 2), (0, 3), (0, 4), (0, 5), (0, 6),
        (1, 6), (2, 6), (3, 6),
        (4, 5), (4, 4),
        (1, 3), (2, 3), (3, 3),
        (4, 2), (4, 1),
        (1, 0), (2, 0), (3, 0),
    ],
    'C': [
        (1, 0), (2, 0), (3, 0), (4, 0),
        (0, 1), (0, 2), (0, 3), (0, 4), (0, 5),
        (1, 6), (2, 6), (3, 6), (4, 6),
    ],
    'D': [
        (0, 0), (0, 1), (0, 2), (0, 3), (0, 4), (0, 5), (0, 6),
        (1, 6), (2, 6), (3, 5),
        (4, 4), (4, 3), (4, 2),
        (3, 1), (2, 0), (1, 0),
    ],
    'E': [
        (0, 0), (0, 1), (0, 2), (0, 3), (0, 4), (0, 5), (0, 6),
        (1, 6), (2, 6), (3, 6), (4, 6),
        (1, 3), (2, 3),
        (1, 0), (2, 0), (3, 0), (4, 0),
    ],
    'F': [
        (0, 0), (0, 1), (0, 2), (0, 3), (0, 4), (0, 5), (0, 6),
        (1, 6), (2, 6), (3, 6), (4, 6),
        (1, 3), (2, 3),
    ],
    'G': [
        (1, 6), (2, 6), (3, 6),
        (0, 5), (0, 4), (0, 3), (0, 2), (0, 1),
        (1, 0), (2, 0), (3, 0),
        (4, 1), (4, 2), (4, 3),
        (3, 3), (2, 3),
    ],
    'H': [
        (0, 0), (0, 1), (0, 2), (0, 3), (0, 4), (0, 5), (0, 6),
        (4, 0), (4, 1), (4, 2), (4, 3), (4, 4), (4, 5), (4, 6),
        (1, 3), (2, 3), (3, 3),
    ],
    'I': [
        (1, 6), (2, 6), (3, 6),
        (2, 5), (2, 4), (2, 3), (2, 2), (2, 1),
        (1, 0), (2, 0), (3, 0),
    ],
    'J': [
        (1, 6), (2, 6), (3, 6), (4, 6),
        (3, 5), (3, 4), (3, 3), (3, 2), (3, 1),
        (2, 0), (1, 0), (0, 1),
    ],
    'K': [
        (0, 0), (0, 1), (0, 2), (0, 3), (0, 4), (0, 5), (0, 6),
        (1, 3),
        (2, 4), (2, 2),
        (3, 5), (3, 1),
        (4, 6), (4, 0),
    ],
    'L': [
        (0, 0), (0, 1), (0, 2), (0, 3), (0, 4), (0, 5), (0, 6),
        (1, 0), (2, 0), (3, 0), (4, 0),
    ],
    'M': [
        (0, 0), (0, 1), (0, 2), (0, 3), (0, 4), (0, 5), (0, 6),
        (1, 5),
        (2, 4),
        (3, 5),
        (4, 0), (4, 1), (4, 2), (4, 3), (4, 4), (4, 5), (4, 6),
    ],
    'N': [
        (0, 0), (0, 1), (0, 2), (0, 3), (0, 4), (0, 5), (0, 6),
        (1, 5),
        (2, 4), (2, 3),
        (3, 2),
        (4, 0), (4, 1), (4, 2), (4, 3), (4, 4), (4, 5), (4, 6),
    ],
    'O': [
        (1, 6), (2, 6), (3, 6),
        (0, 5), (0, 4), (0, 3), (0, 2), (0, 1),
        (1, 0), (2, 0), (3, 0),
        (4, 1), (4, 2), (4, 3), (4, 4), (4, 5),
    ],
    'P': [
        (0, 0), (0, 1), (0, 2), (0, 3), (0, 4), (0, 5), (0, 6),
        (1, 6), (2, 6), (3, 6),
        (4, 5), (4, 4),
        (1, 3), (2, 3), (3, 3),
    ],
    'R': [
        (0, 0), (0, 1), (0, 2), (0, 3), (0, 4), (0, 5), (0, 6),
        (1, 6), (2, 6), (3, 6),
        (4, 5), (4, 4),
        (1, 3), (2, 3), (3, 3),
        (2, 2), (3, 1), (4, 0),
    ],
    'S': [
        (1, 0), (2, 0), (3, 0),
        (4, 1), (4, 2),
        (3, 3), (2, 3), (1, 3),
        (0, 4), (0, 5),
        (1, 6), (2, 6), (3, 6),
    ],
    'T': [
        (0, 6), (1, 6), (2, 6), (3, 6), (4, 6),
        (2, 5), (2, 4), (2, 3), (2, 2), (2, 1), (2, 0),
    ],
    'U': [
        (0, 6), (0, 5), (0, 4), (0, 3), (0, 2), (0, 1),
        (1, 0), (2, 0), (3, 0),
        (4, 1), (4, 2), (4, 3), (4, 4), (4, 5), (4, 6),
    ],
    'V': [
        (0, 6), (0, 5),
        (1, 4), (1, 3),
        (2, 2), (2, 1), (2, 0),
        (3, 3), (3, 4),
        (4, 5), (4, 6),
    ],
    'W': [
        (0, 6), (0, 5), (0, 4), (0, 3), (0, 2), (0, 1), (0, 0),
        (1, 1),
        (2, 2),
        (3, 1),
        (4, 6), (4, 5), (4, 4), (4, 3), (4, 2), (4, 1), (4, 0),
    ],
    'X': [
        (0, 6), (0, 0),
        (1, 5), (1, 1),
        (2, 4), (2, 3), (2, 2),
        (3, 5), (3, 1),
        (4, 6), (4, 0),
    ],
    'Y': [
        (0, 6), (0, 5),
        (1, 4),
        (2, 3), (2, 2), (2, 1), (2, 0),
        (3, 4),
        (4, 6), (4, 5),
    ],
    'Z': [
        (0, 6), (1, 6), (2, 6), (3, 6), (4, 6),
        (4, 5), (3, 4), (2, 3), (1, 2), (0, 1),
        (0, 0), (1, 0), (2, 0), (3, 0), (4, 0),
    ],
}


class MovementMode(Enum):
    """Formation movement modes"""
    STATIC = "static"
    MOVING = "moving"
    ADAPTIVE = "adaptive"


class CentroidPath(Enum):
    """Centroid path types for moving mode"""
    CIRCULAR = "circular"
    LINEAR = "linear"
    WAYPOINTS = "waypoints"


class FormationState(Enum):
    """Overall formation lifecycle states"""
    IDLE = "idle"
    FORMING = "forming"
    FORMED = "formed"
    MOVING = "moving"
    DEFORMING = "deforming"
    REFORMING = "reforming"
    STOPPED = "stopped"


class PIDController:
    """PID controller with anti-windup clamping"""

    def __init__(self, kp: float, ki: float, kd: float, max_output: float):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.max_output = max_output
        self.integral = 0.0
        self.previous_error = 0.0

    def compute(self, error: float, dt: float) -> float:
        if dt <= 0:
            return 0.0
        self.integral += error * dt
        # Anti-windup
        self.integral = max(-self.max_output, min(self.max_output, self.integral))
        derivative = (error - self.previous_error) / dt
        self.previous_error = error
        output = self.kp * error + self.ki * self.integral + self.kd * derivative
        return max(-self.max_output, min(self.max_output, output))

    def reset(self):
        self.integral = 0.0
        self.previous_error = 0.0


def normalize_angle(angle: float) -> float:
    """Normalize angle to [-pi, pi]"""
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle


def quaternion_to_yaw(q: Quaternion) -> float:
    """Extract yaw from quaternion"""
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


class FormationController:
    """
    Formation Control node for TurtleBot3 Burger swarms.

    Computes geometric or letter-shaped target positions relative to a moving
    centroid, assigns robots to positions via global minimum-cost matching, and drives
    each robot with PID-based differential-drive control. Obstacle avoidance
    is applied per-robot before publishing cmd_vel.

    Parameters (rospy.get_param):
        ~robot_count        : int   (default 6)
        ~formation_type     : str   (default 'triangle')
        ~movement_mode      : str   (default 'moving')
        ~spacing            : float (default 1.0 m)
        ~max_linear_vel     : float (default 0.2 m/s)
        ~max_angular_vel    : float (default 1.5 rad/s)
        ~centroid_speed     : float (default 0.1 m/s)
        ~path_radius        : float (default 2.5 m)
        ~centroid_path      : str   (default 'circular')

    Published topics:
        /{ns}/cmd_vel               geometry_msgs/Twist
        /formation/status           std_msgs/String   (JSON)
        /formation/markers          visualization_msgs/MarkerArray

    Subscribed topics:
        /{ns}/odom                  nav_msgs/Odometry
        /fleet/robot_list           std_msgs/String
        /formation/set_shape        std_msgs/String
        /formation/start            std_msgs/String  (JSON config)
        /formation/stop             std_msgs/String  (JSON task_id)
    """

    # ------------------------------------------------------------------
    # Construction
    # ------------------------------------------------------------------
    def __init__(self):
        rospy.init_node('formation_control', anonymous=False)

        # ---- Parameters ----
        self.robot_count = rospy.get_param('~robot_count', 6)
        formation_type_str = rospy.get_param('~formation_type', 'triangle')
        movement_mode_str = rospy.get_param('~movement_mode', 'moving')
        self.spacing = rospy.get_param('~spacing', 1.0)
        self.max_linear_vel = rospy.get_param('~max_linear_vel', 0.2)
        self.max_angular_vel = rospy.get_param('~max_angular_vel', 1.5)
        self.centroid_speed = rospy.get_param('~centroid_speed', 0.1)
        self.path_radius = rospy.get_param('~path_radius', 2.5)
        centroid_path_str = rospy.get_param('~centroid_path', 'circular')

        # Waypoints for the waypoints centroid path mode
        # Loaded as JSON list of {"x": ..., "y": ...} dicts
        waypoints_param = rospy.get_param('~waypoints', '[]')
        try:
            self.centroid_waypoints: List[Dict] = json.loads(waypoints_param) \
                if isinstance(waypoints_param, str) else waypoints_param
        except (json.JSONDecodeError, TypeError):
            self.centroid_waypoints = []

        # ---- Resolve enums ----
        self.formation_type: str = formation_type_str  # may be shape name or letter
        try:
            self.movement_mode = MovementMode(movement_mode_str)
        except ValueError:
            self.movement_mode = MovementMode.MOVING
        try:
            self.centroid_path = CentroidPath(centroid_path_str)
        except ValueError:
            self.centroid_path = CentroidPath.CIRCULAR

        # ---- State ----
        self.is_running = False
        self.is_paused = False
        self.emergency_stop_active = False
        self.current_task_id = None
        self.formation_state = FormationState.IDLE
        self.lock = threading.RLock()
        self.command_lock = threading.RLock()

        # Centroid state
        self.centroid_x = 0.0
        self.centroid_y = 0.0
        self.centroid_heading = 0.0  # heading angle of centroid path
        self.centroid_time = 0.0     # elapsed time for path parametrization
        self.current_waypoint_idx = 0

        # Robots (populated from /fleet/robot_list or default namespace list)
        self.robot_ids: List[str] = []
        self.robot_poses: Dict[str, Optional[Pose]] = {}
        self.robot_yaws: Dict[str, float] = {}

        # Formation positions: list of (x, y) offsets from centroid (unscaled)
        self.formation_offsets: List[Tuple[float, float]] = []
        # Mapping: robot_id -> index in formation_offsets
        self.assignments: Dict[str, int] = {}
        self.assignment_pending = False
        self._assignment_generation = 0

        # PID controllers per robot (linear, angular)
        self.pid_linear: Dict[str, PIDController] = {}
        self.pid_angular: Dict[str, PIDController] = {}

        # Obstacle avoidance per robot
        self.avoidance: Dict[str, ObstacleAvoidance] = {}

        # Publishers per robot
        self.cmd_vel_pubs: Dict[str, rospy.Publisher] = {}
        # Odom subscribers stored to prevent garbage collection
        self._odom_subs: Dict[str, rospy.Subscriber] = {}

        # Global publishers
        self.status_pub = rospy.Publisher(
            '/formation/status', String, queue_size=1
        )
        self.marker_pub = rospy.Publisher(
            '/formation/markers', MarkerArray, queue_size=1
        )

        # Global subscribers
        rospy.Subscriber(
            '/fleet/robot_list', String, self._fleet_list_cb, queue_size=1
        )
        rospy.Subscriber(
            '/formation/set_shape', String, self._set_shape_cb, queue_size=1
        )
        rospy.Subscriber(
            '/formation/start', String, self._start_cb, queue_size=1
        )
        rospy.Subscriber(
            '/formation/stop', String, self._stop_cb, queue_size=1
        )
        rospy.Subscriber(
            '/formation/pause', String, self._pause_cb, queue_size=1
        )
        rospy.Subscriber(
            '/formation/resume', String, self._resume_cb, queue_size=1
        )
        rospy.Subscriber(
            '/swarm/emergency_stop', Bool,
            self._emergency_stop_cb, queue_size=1
        )

        # ---- Bootstrap robots from parameter if fleet topic not yet available ----
        if self.robot_count > 0 and len(self.robot_ids) == 0:
            default_ids = [f'tb3_{i}' for i in range(self.robot_count)]
            self._update_robot_list(default_ids)

        # Compute the initial formation against the exact t=0 path pose.
        with self.command_lock:
            self._reset_centroid_path_pose_locked()
            assignment_snapshot = self._recompute_formation_locked()
        self._compute_and_commit_assignment(assignment_snapshot)

        # ---- Control timer at 20 Hz ----
        self.control_rate = 20.0
        self.control_timer = rospy.Timer(
            rospy.Duration(1.0 / self.control_rate), self._control_loop
        )

        rospy.loginfo(
            "FormationController ready: type=%s, mode=%s, robots=%d, spacing=%.2f, "
            "centroid_path=%s, path_radius=%.2f, centroid_speed=%.2f",
            self.formation_type, self.movement_mode.value, self.robot_count,
            self.spacing, self.centroid_path.value, self.path_radius,
            self.centroid_speed,
        )

    # ------------------------------------------------------------------
    # Subscriber callbacks
    # ------------------------------------------------------------------
    def _fleet_list_cb(self, msg: String):
        """Handle /fleet/robot_list (comma-separated namespace list)."""
        ids = sort_robot_ids(
            s.strip() for s in msg.data.split(',') if s.strip()
        )
        assignment_snapshot = None
        with self.command_lock:
            if ids != self.robot_ids:
                if not ids:
                    self.is_running = False
                    self.is_paused = False
                    self.formation_state = FormationState.STOPPED
                    self.current_task_id = None
                    self._cancel_pending_assignment_locked(
                        clear_assignments=True
                    )
                    self._stop_all_robots()

                self._update_robot_list(ids)
                if ids:
                    assignment_snapshot = self._recompute_formation_locked()
                else:
                    self.formation_offsets = []
                    self.assignments = {}
                    self.assignment_pending = False
        self._compute_and_commit_assignment(assignment_snapshot)

    def _set_shape_cb(self, msg: String):
        """Dynamically change formation type at runtime."""
        new_type = msg.data.strip()
        assignment_snapshot = None
        with self.command_lock:
            if new_type and new_type != self.formation_type:
                rospy.loginfo(
                    "Formation shape changed: %s -> %s",
                    self.formation_type, new_type,
                )
                self.formation_type = new_type
                assignment_snapshot = self._recompute_formation_locked()
        self._compute_and_commit_assignment(assignment_snapshot)

    def _start_cb(self, msg):
        """Start the formation behavior with optional runtime config."""
        # Parse config from task orchestrator (JSON String)
        try:
            config = json.loads(msg.data) if msg.data else {}
        except (json.JSONDecodeError, AttributeError):
            config = {}

        assignment_snapshot = None
        with self.command_lock:
            if self.emergency_stop_active:
                rospy.logwarn(
                    "Formation start rejected while emergency stop is active"
                )
                return
            if not self.robot_ids:
                rospy.logwarn(
                    "Formation start rejected because the fleet is empty"
                )
                return

            recompute = False
            if 'formation_type' in config:
                new_type = config['formation_type']
                recompute = recompute or new_type != self.formation_type
                self.formation_type = new_type
            if 'movement_mode' in config:
                try:
                    self.movement_mode = MovementMode(config['movement_mode'])
                except ValueError:
                    pass
            if 'spacing' in config:
                try:
                    new_spacing = max(0.35, float(config['spacing']))
                    recompute = recompute or new_spacing != self.spacing
                    self.spacing = new_spacing
                except (TypeError, ValueError):
                    pass
            self.current_task_id = config.get('task_id')
            self.is_running = True
            self.is_paused = False
            self.formation_state = FormationState.FORMING
            self._reset_centroid_path_pose_locked()

            if recompute:
                assignment_snapshot = self._recompute_formation_locked()
            else:
                assignment_snapshot = self._prepare_assignment_locked()

            for pid in self.pid_linear.values():
                pid.reset()
            for pid in self.pid_angular.values():
                pid.reset()
        self._compute_and_commit_assignment(assignment_snapshot)
        rospy.loginfo(
            "Formation control STARTED: type=%s, mode=%s",
            self.formation_type, self.movement_mode.value,
        )

    def _stop_cb(self, msg: String):
        """Stop the formation behavior and halt all robots."""
        with self.command_lock:
            if not self._task_command_matches(msg):
                return
            self.is_running = False
            self.is_paused = False
            self.formation_state = FormationState.STOPPED
            self._cancel_pending_assignment_locked(clear_assignments=True)
            self._stop_all_robots()
        rospy.loginfo("Formation control STOPPED")

    def _pause_cb(self, msg: String):
        with self.command_lock:
            if not self._task_command_matches(msg):
                return
            if not self.is_running or self.is_paused:
                return
            self.is_paused = True
            self._stop_all_robots()
        rospy.loginfo("Formation control PAUSED")

    def _resume_cb(self, msg: String):
        with self.command_lock:
            if not self._task_command_matches(msg):
                return
            if (
                not self.is_running
                or not self.is_paused
                or self.emergency_stop_active
            ):
                return
            self.is_paused = False
        rospy.loginfo("Formation control RESUMED")

    def _task_command_matches(self, msg: String) -> bool:
        try:
            payload = json.loads(msg.data) if msg.data else {}
        except (json.JSONDecodeError, AttributeError):
            payload = {}
        requested_id = payload.get('task_id')
        return (
            isinstance(requested_id, str)
            and bool(requested_id)
            and requested_id == self.current_task_id
        )

    def _emergency_stop_cb(self, msg: Bool):
        with self.command_lock:
            self.emergency_stop_active = bool(msg.data)
            if self.emergency_stop_active:
                self.is_running = False
                self.is_paused = False
                self.formation_state = FormationState.STOPPED
                self._cancel_pending_assignment_locked(
                    clear_assignments=True
                )
                self._stop_all_robots()
                rospy.logwarn("Formation emergency stop latched")

    def _odom_cb(self, msg: Odometry, robot_id: str):
        """Per-robot odometry callback."""
        pose = msg.pose.pose
        yaw = quaternion_to_yaw(pose.orientation)
        should_assign = False
        with self.lock:
            first_update = self.robot_poses.get(robot_id) is None
            if robot_id not in self.robot_poses:
                return
            self.robot_poses[robot_id] = pose
            self.robot_yaws[robot_id] = yaw
            should_assign = (
                first_update
                and self.assignment_pending
                and all(
                    self.robot_poses.get(rid) is not None
                    for rid in self.robot_ids
                )
            )

        # Keep obstacle avoidance module in sync
        av = self.avoidance.get(robot_id)
        if av is not None:
            av.set_position(pose.position.x, pose.position.y, yaw)
        if should_assign:
            with self.command_lock:
                if self.assignment_pending:
                    assignment_snapshot = self._prepare_assignment_locked()
                else:
                    assignment_snapshot = None
            self._compute_and_commit_assignment(assignment_snapshot)

    # ------------------------------------------------------------------
    # Robot list management
    # ------------------------------------------------------------------
    def _update_robot_list(self, new_ids: List[str]):
        """Add/remove robots to match the given list."""
        with self.lock:
            old_set = set(self.robot_ids)
            new_set = set(new_ids)

            # Remove departed robots
            for rid in old_set - new_set:
                self._remove_robot(rid)

            # Add new robots
            for rid in new_set - old_set:
                self._add_robot(rid)

            # Maintain ordered list
            self.robot_ids = sort_robot_ids(
                rid for rid in new_ids if rid in new_set
            )
            self.robot_count = len(self.robot_ids)

    def _add_robot(self, robot_id: str):
        """Register publishers, subscribers, PID, and avoidance for a robot."""
        self.robot_poses[robot_id] = None
        self.robot_yaws[robot_id] = 0.0

        self.cmd_vel_pubs[robot_id] = rospy.Publisher(
            f'/{robot_id}/cmd_vel', Twist, queue_size=1
        )
        self._odom_subs[robot_id] = rospy.Subscriber(
            f'/{robot_id}/odom', Odometry,
            callback=self._odom_cb,
            callback_args=robot_id,
            queue_size=1,
        )
        self.pid_linear[robot_id] = PIDController(
            kp=0.6, ki=0.01, kd=0.15, max_output=self.max_linear_vel
        )
        self.pid_angular[robot_id] = PIDController(
            kp=1.2, ki=0.0, kd=0.1, max_output=self.max_angular_vel
        )
        self.avoidance[robot_id] = ObstacleAvoidance(robot_id)

        rospy.loginfo("Added robot: %s", robot_id)

    def _remove_robot(self, robot_id: str):
        """Unregister a robot."""
        pub = self.cmd_vel_pubs.pop(robot_id, None)
        if pub is not None:
            pub.publish(Twist())
            pub.unregister()

        sub = self._odom_subs.pop(robot_id, None)
        if sub is not None:
            sub.unregister()

        avoidance = self.avoidance.pop(robot_id, None)
        if avoidance is not None:
            avoidance.shutdown()

        for store in (
            self.robot_poses, self.robot_yaws,
            self.pid_linear, self.pid_angular,
            self.assignments,
        ):
            store.pop(robot_id, None)

        rospy.loginfo("Removed robot: %s", robot_id)

    # ------------------------------------------------------------------
    # Formation position computation
    # ------------------------------------------------------------------
    def _recompute_formation(self):
        """Recompute formation offsets and reassign robots."""
        with self.command_lock:
            assignment_snapshot = self._recompute_formation_locked()
        self._compute_and_commit_assignment(assignment_snapshot)

    def _recompute_formation_locked(self) -> Optional[Dict]:
        """Recompute offsets and snapshot an assignment under command_lock."""
        n = len(self.robot_ids)
        if n == 0:
            self.formation_offsets = []
            self._cancel_pending_assignment_locked(clear_assignments=True)
            return None

        self.formation_offsets = self._compute_formation_positions(
            self.formation_type, n, self.spacing
        )
        return self._prepare_assignment_locked()

    def _compute_formation_positions(
        self, formation_type: str, n_robots: int, spacing: float
    ) -> List[Tuple[float, float]]:
        """
        Return a list of (x, y) world-coordinate offsets from the centroid
        for *n_robots* in the requested formation, scaled by *spacing*.
        """
        ft = formation_type.lower()

        # Check if it is a single uppercase letter
        if len(formation_type) == 1 and formation_type.upper() in LETTERS:
            return self._compute_letter_positions(
                formation_type.upper(), n_robots, spacing
            )

        if ft == 'triangle':
            return self._triangle_positions(n_robots, spacing)
        elif ft == 'square':
            return self._square_positions(n_robots, spacing)
        elif ft == 'circle':
            return self._circle_positions(n_robots, spacing)
        elif ft == 'line':
            return self._line_positions(n_robots, spacing)
        elif ft in ('v_formation', 'v'):
            return self._v_formation_positions(n_robots, spacing)
        elif ft == 'diamond':
            return self._diamond_positions(n_robots, spacing)
        else:
            # Try as a letter if it is a known key
            key = formation_type.upper()
            if key in LETTERS:
                return self._compute_letter_positions(key, n_robots, spacing)
            rospy.logwarn(
                "Unknown formation type '%s', falling back to circle",
                formation_type,
            )
            return self._circle_positions(n_robots, spacing)

    # ---- Geometric shapes ----

    def _triangle_positions(
        self, n: int, spacing: float
    ) -> List[Tuple[float, float]]:
        """Equilateral triangle: vertices + edge interpolation."""
        positions: List[Tuple[float, float]] = []
        # Compute the number of rows needed so that total slots >= n
        # Row k has (k+1) robots. Total for rows 0..R = (R+1)(R+2)/2.
        rows = 0
        while (rows + 1) * (rows + 2) // 2 < n:
            rows += 1
        # Height of triangle
        height = rows * spacing * 0.866  # sqrt(3)/2
        idx = 0
        for row in range(rows + 1):
            count_in_row = row + 1
            y_offset = (height / 2.0) - row * spacing * 0.866
            row_width = row * spacing
            for j in range(count_in_row):
                if idx >= n:
                    break
                if count_in_row == 1:
                    x_offset = 0.0
                else:
                    x_offset = -row_width / 2.0 + j * (row_width / (count_in_row - 1))
                positions.append((x_offset, y_offset))
                idx += 1
            if idx >= n:
                break
        return positions

    def _square_positions(
        self, n: int, spacing: float
    ) -> List[Tuple[float, float]]:
        """Robots distributed along the perimeter of a square."""
        if n < 4:
            return self._line_positions(n, spacing)

        # Side length chosen so that perimeter accommodates n robots
        # Place robots along 4 edges
        per_side = max(1, n // 4)
        remainder = n - per_side * 4
        side_len = per_side * spacing

        positions: List[Tuple[float, float]] = []
        half = side_len / 2.0

        # Distribute remainder across the first sides
        sides_count = [per_side] * 4
        for i in range(remainder):
            sides_count[i % 4] += 1

        # Top edge: left to right
        for i in range(sides_count[0]):
            t = i / max(1, sides_count[0])
            positions.append((-half + t * side_len, half))
        # Right edge: top to bottom
        for i in range(sides_count[1]):
            t = i / max(1, sides_count[1])
            positions.append((half, half - t * side_len))
        # Bottom edge: right to left
        for i in range(sides_count[2]):
            t = i / max(1, sides_count[2])
            positions.append((half - t * side_len, -half))
        # Left edge: bottom to top
        for i in range(sides_count[3]):
            t = i / max(1, sides_count[3])
            positions.append((-half, -half + t * side_len))

        return positions[:n]

    def _circle_positions(
        self, n: int, spacing: float
    ) -> List[Tuple[float, float]]:
        """Evenly spaced on a circle whose circumference ~ n * spacing."""
        radius = max(spacing, (n * spacing) / (2.0 * math.pi))
        positions: List[Tuple[float, float]] = []
        for i in range(n):
            angle = 2.0 * math.pi * i / n
            positions.append((radius * math.cos(angle), radius * math.sin(angle)))
        return positions

    def _line_positions(
        self, n: int, spacing: float
    ) -> List[Tuple[float, float]]:
        """Straight line centered at centroid, along the x-axis."""
        positions: List[Tuple[float, float]] = []
        total_width = (n - 1) * spacing
        for i in range(n):
            positions.append((-total_width / 2.0 + i * spacing, 0.0))
        return positions

    def _v_formation_positions(
        self, n: int, spacing: float
    ) -> List[Tuple[float, float]]:
        """V shape: leader at tip, followers alternate left/right."""
        positions: List[Tuple[float, float]] = []
        half_angle = math.radians(30)  # 60-degree V opening

        # Leader at the front
        positions.append((0.0, 0.0))

        for i in range(1, n):
            side = 1 if (i % 2 == 1) else -1  # alternate right / left
            rank = (i + 1) // 2  # distance rank from leader
            x = side * rank * spacing * math.sin(half_angle)
            y = -rank * spacing * math.cos(half_angle)
            positions.append((x, y))

        return positions

    def _diamond_positions(
        self, n: int, spacing: float
    ) -> List[Tuple[float, float]]:
        """Diamond / rhombus shape. 4 vertices + interpolation."""
        if n < 4:
            return self._line_positions(n, spacing)

        # Base diamond vertices
        half_diag = spacing * max(1, (n - 1) / 4.0)

        vertices = [
            (0.0, half_diag),    # top
            (half_diag, 0.0),    # right
            (0.0, -half_diag),   # bottom
            (-half_diag, 0.0),   # left
        ]

        if n == 4:
            return vertices

        # Interpolate additional robots along the 4 edges
        positions: List[Tuple[float, float]] = []
        per_edge = max(1, n // 4)
        remainder = n - per_edge * 4
        edge_counts = [per_edge] * 4
        for i in range(remainder):
            edge_counts[i % 4] += 1

        for edge_idx in range(4):
            start = vertices[edge_idx]
            end = vertices[(edge_idx + 1) % 4]
            count = edge_counts[edge_idx]
            for k in range(count):
                t = k / max(1, count)
                px = start[0] + t * (end[0] - start[0])
                py = start[1] + t * (end[1] - start[1])
                positions.append((px, py))

        return positions[:n]

    # ---- Letter shapes ----

    def _compute_letter_positions(
        self, letter: str, n_robots: int, spacing: float
    ) -> List[Tuple[float, float]]:
        """
        Convert a grid-based letter definition into world-coordinate offsets
        sampled at a safe density for any fleet size.
        """
        grid = LETTERS.get(letter)
        if grid is None:
            rospy.logwarn("Letter '%s' not defined, falling back to circle", letter)
            return self._circle_positions(n_robots, spacing)

        return sample_letter_formation(grid, n_robots, spacing)

    # ------------------------------------------------------------------
    # Robot-to-position assignment (global minimum cost)
    # ------------------------------------------------------------------
    def _assign_robots_to_positions(self):
        """Snapshot state briefly, then solve without holding command_lock."""
        with self.command_lock:
            assignment_snapshot = self._prepare_assignment_locked()
        return self._compute_and_commit_assignment(assignment_snapshot)

    def _prepare_assignment_locked(self) -> Optional[Dict]:
        """
        Capture an immutable assignment request while holding command_lock.

        The Hungarian solver is deliberately invoked later, after command_lock
        has been released, so emergency-stop and lifecycle callbacks are never
        queued behind its O(n^3) computation.
        """
        self._assignment_generation += 1
        generation = self._assignment_generation

        n = min(len(self.robot_ids), len(self.formation_offsets))
        if n == 0:
            self.assignments = {}
            self.assignment_pending = False
            return None

        target_world = tuple(self._get_world_targets()[:n])
        robot_ids = tuple(self.robot_ids[:n])
        robot_positions: List[Tuple[float, float]] = []
        previous_slots: List[Optional[int]] = []

        with self.lock:
            poses = [self.robot_poses.get(robot_id) for robot_id in robot_ids]
            if any(pose is None for pose in poses):
                self.assignments = {}
                self.assignment_pending = True
                return None

            for robot_id, pose in zip(robot_ids, poses):
                robot_positions.append((pose.position.x, pose.position.y))
                previous_slots.append(self.assignments.get(robot_id))

        # Stop the control loop from using an assignment that belongs to the
        # previous centroid/shape while the new solution is in flight.
        self.assignments = {}
        self.assignment_pending = True
        return {
            'generation': generation,
            'robot_ids': robot_ids,
            'robot_positions': tuple(robot_positions),
            'target_world': target_world,
            'previous_slots': tuple(previous_slots),
            'switch_penalty': (self.spacing * 0.35) ** 2,
        }

    def _compute_and_commit_assignment(
        self, assignment_snapshot: Optional[Dict]
    ) -> bool:
        """
        Solve an assignment without command_lock and commit only if current.

        A newer shape/fleet/start request, stop, or emergency stop advances the
        generation and makes this result stale before it can affect control.
        """
        if assignment_snapshot is None:
            return False

        slots = minimum_distance_assignment(
            assignment_snapshot['robot_positions'],
            assignment_snapshot['target_world'],
            previous_slots=assignment_snapshot['previous_slots'],
            switch_penalty=assignment_snapshot['switch_penalty'],
        )

        with self.command_lock:
            if (
                assignment_snapshot['generation']
                != self._assignment_generation
            ):
                return False
            self.assignments = {
                robot_id: slot_index
                for robot_id, slot_index in zip(
                    assignment_snapshot['robot_ids'], slots
                )
            }
            self.assignment_pending = False
        return True

    def _cancel_pending_assignment_locked(
        self, clear_assignments: bool = False
    ):
        """Invalidate any solver result that was computed from older state."""
        self._assignment_generation += 1
        self.assignment_pending = False
        if clear_assignments:
            self.assignments = {}

    def _get_world_targets(self) -> List[Tuple[float, float]]:
        """
        Return world positions of every formation slot, accounting for
        centroid position and heading rotation.
        """
        cos_h = math.cos(self.centroid_heading)
        sin_h = math.sin(self.centroid_heading)
        targets: List[Tuple[float, float]] = []
        for (ox, oy) in self.formation_offsets:
            # Rotate offset by centroid heading
            rx = ox * cos_h - oy * sin_h
            ry = ox * sin_h + oy * cos_h
            targets.append((self.centroid_x + rx, self.centroid_y + ry))
        return targets

    # ------------------------------------------------------------------
    # Centroid update
    # ------------------------------------------------------------------
    def _reset_centroid_path_pose_locked(self):
        """Reset to the exact pose used by the configured path at t=0."""
        self.centroid_time = 0.0
        self.current_waypoint_idx = 0
        self.centroid_x = 0.0
        self.centroid_y = 0.0
        self.centroid_heading = 0.0

        if self.movement_mode == MovementMode.STATIC:
            return

        if self.centroid_path == CentroidPath.CIRCULAR:
            self._set_circular_centroid_pose()
            return

        if (
            self.centroid_path == CentroidPath.WAYPOINTS
            and self.centroid_waypoints
        ):
            # Skip waypoints already at the deterministic path origin and set
            # the heading that the first moving tick will use.
            for _ in range(len(self.centroid_waypoints)):
                wp = self.centroid_waypoints[
                    self.current_waypoint_idx % len(self.centroid_waypoints)
                ]
                dx = wp.get('x', 0.0) - self.centroid_x
                dy = wp.get('y', 0.0) - self.centroid_y
                if math.hypot(dx, dy) >= 0.15:
                    self.centroid_heading = math.atan2(dy, dx)
                    break
                self.current_waypoint_idx = (
                    self.current_waypoint_idx + 1
                ) % len(self.centroid_waypoints)

    def _set_circular_centroid_pose(self):
        """Set the circular path pose for the current centroid_time."""
        angular_speed = self.centroid_speed / max(0.1, self.path_radius)
        t = self.centroid_time * angular_speed
        self.centroid_x = self.path_radius * math.cos(t)
        self.centroid_y = self.path_radius * math.sin(t)
        self.centroid_heading = t + math.pi / 2.0

    def _update_centroid(self, dt: float):
        """Advance the centroid along the configured path."""
        if self.movement_mode == MovementMode.STATIC:
            # Centroid stays at the average of initial robot positions
            return

        self.centroid_time += dt

        if self.centroid_path == CentroidPath.CIRCULAR:
            self._set_circular_centroid_pose()

        elif self.centroid_path == CentroidPath.LINEAR:
            # Move in a straight line along the current heading
            self.centroid_x += self.centroid_speed * math.cos(self.centroid_heading) * dt
            self.centroid_y += self.centroid_speed * math.sin(self.centroid_heading) * dt

        elif self.centroid_path == CentroidPath.WAYPOINTS:
            if len(self.centroid_waypoints) == 0:
                return
            wp = self.centroid_waypoints[self.current_waypoint_idx % len(self.centroid_waypoints)]
            wx = wp.get('x', 0.0)
            wy = wp.get('y', 0.0)
            dx = wx - self.centroid_x
            dy = wy - self.centroid_y
            dist = math.sqrt(dx * dx + dy * dy)
            if dist < 0.15:
                # Advance to next waypoint
                self.current_waypoint_idx = (self.current_waypoint_idx + 1) % len(self.centroid_waypoints)
            else:
                # Move toward waypoint
                heading = math.atan2(dy, dx)
                self.centroid_heading = heading
                step = min(self.centroid_speed * dt, dist)
                self.centroid_x += step * math.cos(heading)
                self.centroid_y += step * math.sin(heading)

    # ------------------------------------------------------------------
    # 20 Hz control loop
    # ------------------------------------------------------------------
    def _control_loop(self, event):
        with self.command_lock:
            if (
                not self.is_running
                or self.is_paused
                or self.emergency_stop_active
            ):
                return
            self._control_step(event)

    def _control_step(self, event):
        """Main 20 Hz timer callback."""
        if not self.is_running:
            return

        n = len(self.robot_ids)
        if n == 0:
            return

        dt = 1.0 / self.control_rate

        # ---- Update centroid ----
        self._update_centroid(dt)

        # ---- Compute world target positions ----
        world_targets = self._get_world_targets()

        # ---- Share robot positions with obstacle avoidance modules ----
        robot_position_list: List[Tuple[str, Point]] = []
        with self.lock:
            for rid in self.robot_ids:
                pose = self.robot_poses.get(rid)
                if pose is not None:
                    robot_position_list.append((
                        rid,
                        Point(x=pose.position.x, y=pose.position.y, z=0.0),
                    ))
        for av in self.avoidance.values():
            av.update_robot_positions(robot_position_list)

        # ---- Adaptive mode: detect obstacle proximity ----
        deforming = False
        obstacle_center_x = 0.0
        obstacle_center_y = 0.0
        if self.movement_mode == MovementMode.ADAPTIVE:
            # Check threat level across all robots; if any is significant, deform
            for rid in self.robot_ids:
                av = self.avoidance.get(rid)
                if av is not None and av.compute_threat_level() > 0.3:
                    deforming = True
                    # Rough estimate of obstacle position (use robot with highest threat)
                    pose = self.robot_poses.get(rid)
                    if pose is not None:
                        rep = av.compute_repulsion_force()
                        obstacle_center_x = pose.position.x - rep.x * 0.5
                        obstacle_center_y = pose.position.y - rep.y * 0.5
                    break

        # ---- Per-robot control ----
        all_in_position = bool(self.robot_ids)

        for rid in self.robot_ids:
            slot_idx = self.assignments.get(rid)
            if slot_idx is None or slot_idx >= len(world_targets):
                all_in_position = False
                pub = self.cmd_vel_pubs.get(rid)
                if pub is not None:
                    pub.publish(Twist())
                continue

            target_x, target_y = world_targets[slot_idx]

            # Adaptive deformation: push target away from detected obstacle
            if deforming and self.movement_mode == MovementMode.ADAPTIVE:
                ox = target_x - obstacle_center_x
                oy = target_y - obstacle_center_y
                obs_dist = math.sqrt(ox * ox + oy * oy)
                if obs_dist < self.spacing * 2.0 and obs_dist > 0.01:
                    push = (self.spacing * 2.0 - obs_dist) * 0.5
                    target_x += push * (ox / obs_dist)
                    target_y += push * (oy / obs_dist)

            with self.lock:
                pose = self.robot_poses.get(rid)
                yaw = self.robot_yaws.get(rid, 0.0)

            if pose is None:
                all_in_position = False
                pub = self.cmd_vel_pubs.get(rid)
                if pub is not None:
                    pub.publish(Twist())
                continue

            robot_x = pose.position.x
            robot_y = pose.position.y

            dx = target_x - robot_x
            dy = target_y - robot_y
            distance = math.sqrt(dx * dx + dy * dy)
            angle_to_target = math.atan2(dy, dx)
            angle_error = normalize_angle(angle_to_target - yaw)

            if distance > 0.1:
                all_in_position = False

            # ---- PID control ----
            linear_vel = self.pid_linear[rid].compute(distance, dt)
            # Reduce forward speed when facing away from target
            cos_factor = max(0.0, math.cos(angle_error))
            linear_vel *= cos_factor

            angular_vel = self.pid_angular[rid].compute(angle_error, dt)

            # If very close, damp velocities to avoid oscillation
            if distance < 0.05:
                linear_vel *= 0.3
                angular_vel *= 0.3

            # Clamp
            linear_vel = max(-self.max_linear_vel, min(self.max_linear_vel, linear_vel))
            angular_vel = max(-self.max_angular_vel, min(self.max_angular_vel, angular_vel))

            desired_cmd = Twist()
            desired_cmd.linear.x = linear_vel
            desired_cmd.angular.z = angular_vel

            # ---- Obstacle avoidance ----
            av = self.avoidance.get(rid)
            if av is not None:
                safe_cmd = av.apply_avoidance(desired_cmd)
            else:
                safe_cmd = desired_cmd

            # ---- Publish ----
            pub = self.cmd_vel_pubs.get(rid)
            if pub is not None:
                pub.publish(safe_cmd)

        # ---- Update formation state ----
        if deforming:
            self.formation_state = FormationState.DEFORMING
        elif all_in_position:
            if self.movement_mode == MovementMode.STATIC:
                self.formation_state = FormationState.FORMED
            else:
                self.formation_state = FormationState.MOVING
        else:
            self.formation_state = FormationState.FORMING

        # ---- Publish status and markers ----
        self._publish_status(world_targets)
        self._publish_markers(world_targets)

    # ------------------------------------------------------------------
    # Stop helpers
    # ------------------------------------------------------------------
    def _stop_all_robots(self):
        """Publish zero velocity to every robot."""
        stop = Twist()
        with self.lock:
            pubs = list(self.cmd_vel_pubs.values())
        for pub in pubs:
            pub.publish(stop)

    # ------------------------------------------------------------------
    # Status publishing
    # ------------------------------------------------------------------
    def _publish_status(self, world_targets: List[Tuple[float, float]]):
        """Publish JSON status to /formation/status."""
        assignments_dict: Dict[str, Dict] = {}
        for rid, slot_idx in self.assignments.items():
            if slot_idx < len(world_targets):
                tx, ty = world_targets[slot_idx]
                assignments_dict[rid] = {'slot': slot_idx, 'target_x': tx, 'target_y': ty}

        status = {
            'task_id': self.current_task_id,
            'paused': self.is_paused,
            'formation_type': self.formation_type,
            'movement_mode': self.movement_mode.value,
            'state': self.formation_state.value,
            'centroid': {'x': round(self.centroid_x, 4), 'y': round(self.centroid_y, 4)},
            'centroid_heading': round(self.centroid_heading, 4),
            'robot_count': len(self.robot_ids),
            'robot_assignments': assignments_dict,
        }
        self.status_pub.publish(String(data=json.dumps(status)))

    # ------------------------------------------------------------------
    # RViz marker publishing
    # ------------------------------------------------------------------
    def _publish_markers(self, world_targets: List[Tuple[float, float]]):
        """Publish MarkerArray for RViz: target positions + centroid."""
        marker_array = MarkerArray()
        stamp = rospy.Time.now()

        # ---- Target position markers (green cylinders) ----
        for idx, (tx, ty) in enumerate(world_targets):
            m = Marker()
            m.header.frame_id = "map"
            m.header.stamp = stamp
            m.ns = "formation_targets"
            m.id = idx
            m.type = Marker.CYLINDER
            m.action = Marker.ADD
            m.pose.position.x = tx
            m.pose.position.y = ty
            m.pose.position.z = 0.01
            m.pose.orientation.w = 1.0
            m.scale.x = 0.18
            m.scale.y = 0.18
            m.scale.z = 0.02
            m.color.r = 0.0
            m.color.g = 1.0
            m.color.b = 0.0
            m.color.a = 0.6
            m.lifetime = rospy.Duration(0.2)
            marker_array.markers.append(m)

        # ---- Lines from each robot to its target (thin blue) ----
        line_id = len(world_targets) + 1
        for rid, slot_idx in self.assignments.items():
            if slot_idx >= len(world_targets):
                continue
            pose = self.robot_poses.get(rid)
            if pose is None:
                continue
            tx, ty = world_targets[slot_idx]
            m = Marker()
            m.header.frame_id = "map"
            m.header.stamp = stamp
            m.ns = "formation_links"
            m.id = line_id
            line_id += 1
            m.type = Marker.LINE_STRIP
            m.action = Marker.ADD
            m.scale.x = 0.03
            m.color.r = 0.3
            m.color.g = 0.3
            m.color.b = 1.0
            m.color.a = 0.5
            m.lifetime = rospy.Duration(0.2)
            p1 = Point(x=pose.position.x, y=pose.position.y, z=0.05)
            p2 = Point(x=tx, y=ty, z=0.05)
            m.points.append(p1)
            m.points.append(p2)
            marker_array.markers.append(m)

        # ---- Centroid marker (yellow sphere) ----
        cm = Marker()
        cm.header.frame_id = "map"
        cm.header.stamp = stamp
        cm.ns = "formation_centroid"
        cm.id = 0
        cm.type = Marker.SPHERE
        cm.action = Marker.ADD
        cm.pose.position.x = self.centroid_x
        cm.pose.position.y = self.centroid_y
        cm.pose.position.z = 0.15
        cm.pose.orientation.w = 1.0
        cm.scale.x = 0.2
        cm.scale.y = 0.2
        cm.scale.z = 0.2
        cm.color.r = 1.0
        cm.color.g = 1.0
        cm.color.b = 0.0
        cm.color.a = 0.9
        cm.lifetime = rospy.Duration(0.2)
        marker_array.markers.append(cm)

        # ---- Centroid heading arrow (red) ----
        ha = Marker()
        ha.header.frame_id = "map"
        ha.header.stamp = stamp
        ha.ns = "formation_heading"
        ha.id = 0
        ha.type = Marker.ARROW
        ha.action = Marker.ADD
        ha.scale.x = 0.06
        ha.scale.y = 0.12
        ha.scale.z = 0.0
        ha.color.r = 1.0
        ha.color.g = 0.2
        ha.color.b = 0.2
        ha.color.a = 0.8
        ha.lifetime = rospy.Duration(0.2)
        arrow_len = 0.5
        p_start = Point(
            x=self.centroid_x, y=self.centroid_y, z=0.15
        )
        p_end = Point(
            x=self.centroid_x + arrow_len * math.cos(self.centroid_heading),
            y=self.centroid_y + arrow_len * math.sin(self.centroid_heading),
            z=0.15,
        )
        ha.points.append(p_start)
        ha.points.append(p_end)
        marker_array.markers.append(ha)

        # ---- Formation outline (connect target positions in order) ----
        if len(world_targets) >= 3:
            outline = Marker()
            outline.header.frame_id = "map"
            outline.header.stamp = stamp
            outline.ns = "formation_outline"
            outline.id = 0
            outline.type = Marker.LINE_STRIP
            outline.action = Marker.ADD
            outline.scale.x = 0.04
            outline.color.r = 0.0
            outline.color.g = 0.6
            outline.color.b = 1.0
            outline.color.a = 0.7
            outline.lifetime = rospy.Duration(0.2)
            for (tx, ty) in world_targets:
                outline.points.append(Point(x=tx, y=ty, z=0.05))
            # Close loop
            outline.points.append(Point(x=world_targets[0][0], y=world_targets[0][1], z=0.05))
            marker_array.markers.append(outline)

        # ---- Text label showing formation type ----
        label = Marker()
        label.header.frame_id = "map"
        label.header.stamp = stamp
        label.ns = "formation_label"
        label.id = 0
        label.type = Marker.TEXT_VIEW_FACING
        label.action = Marker.ADD
        label.pose.position.x = self.centroid_x
        label.pose.position.y = self.centroid_y + 0.4
        label.pose.position.z = 0.5
        label.pose.orientation.w = 1.0
        label.scale.z = 0.25
        label.color.r = 1.0
        label.color.g = 1.0
        label.color.b = 1.0
        label.color.a = 1.0
        label.text = f"{self.formation_type} ({self.formation_state.value})"
        label.lifetime = rospy.Duration(0.2)
        marker_array.markers.append(label)

        self.marker_pub.publish(marker_array)


# ======================================================================
# Entry point
# ======================================================================
if __name__ == '__main__':
    try:
        node = FormationController()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
