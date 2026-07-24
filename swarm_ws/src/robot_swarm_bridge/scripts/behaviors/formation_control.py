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
import time
import uuid
from typing import List, Dict, Optional, Tuple
from enum import Enum

# ROS messages
from geometry_msgs.msg import Twist, Pose, Point, Quaternion
from gazebo_msgs.msg import ModelStates
from nav_msgs.msg import Odometry
from std_msgs.msg import String, Bool
from visualization_msgs.msg import Marker, MarkerArray

# Import obstacle avoidance from core
import sys
import os
sys.path.append(os.path.join(os.path.dirname(__file__), '..'))
from core.obstacle_avoidance import ObstacleAvoidance
from utils.robot_ids import sort_robot_ids
from robot_swarm_bridge.utils.safety_publish import SafetyPublishLane
from robot_swarm_bridge.algorithms.formation import (
    ensure_minimum_spacing,
    find_safe_formation_center,
    formation_targets_are_safe,
    hungarian_assignment,
    minimum_distance_assignment,
    plan_obstacle_aware_route,
    point_to_route_distance,
    routes_conflict,
    sample_letter_formation,
    signed_distance_to_zone,
    straight_route_is_safe,
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
    FAILED = "failed"
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
        ~max_linear_vel     : float (default 0.22 m/s)
        ~max_angular_vel    : float (default 1.5 rad/s)
        ~centroid_speed     : float (default 0.1 m/s)
        ~path_radius        : float (default 2.5 m)
        ~centroid_path      : str   (default 'circular')
        ~odom_timeout_wall_s: float (default 0.75 s)
        ~odom_initialization_timeout_wall_s: float (default 10.0 s)

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

    BURGER_MAX_LINEAR_VELOCITY = 0.22
    BURGER_MAX_ANGULAR_VELOCITY = 2.84
    BURGER_RADIUS = 0.11
    ASSIGNMENT_WORKER_SHUTDOWN_TIMEOUT = 30.0
    EMERGENCY_STOP_TIMEOUT = 0.25

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
        configured_linear_limit = rospy.get_param('~max_linear_vel', 0.22)
        configured_angular_limit = rospy.get_param('~max_angular_vel', 1.5)
        self.max_linear_vel, linear_limit_valid = self._bounded_motion_limit(
            configured_linear_limit,
            self.BURGER_MAX_LINEAR_VELOCITY,
        )
        self.max_angular_vel, angular_limit_valid = self._bounded_motion_limit(
            configured_angular_limit,
            self.BURGER_MAX_ANGULAR_VELOCITY,
        )
        # Keep safe numeric limits in every downstream controller. The flag
        # still remembers a malformed parameter so the publication gate can
        # reject the task instead of silently accepting a fallback value.
        self._motion_limits_valid = (
            linear_limit_valid and angular_limit_valid
        )
        self.centroid_speed = rospy.get_param('~centroid_speed', 0.1)
        self.path_radius = max(
            0.10, float(rospy.get_param('~path_radius', 2.5))
        )
        self.minimum_safe_path_radius = max(
            0.10,
            float(rospy.get_param('~minimum_safe_path_radius', 0.25)),
        )
        self.orbit_sample_step = min(
            0.08,
            max(0.02, float(rospy.get_param('~orbit_sample_step', 0.04))),
        )
        self.route_tracking_margin = min(
            0.12,
            max(0.0, float(rospy.get_param(
                '~formation_route_tracking_margin', 0.02
            ))),
        )
        self.route_waypoint_tolerance = min(
            0.08,
            max(0.02, float(rospy.get_param(
                '~formation_route_waypoint_tolerance', 0.08
            ))),
        )
        self.route_stall_timeout = min(
            60.0,
            max(
                5.0,
                float(rospy.get_param(
                    '~formation_route_stall_timeout', 20.0
                )),
            ),
        )
        self.route_progress_epsilon = min(
            0.25,
            max(0.02, float(rospy.get_param(
                '~formation_route_progress_epsilon', 0.10
            ))),
        )
        centroid_path_str = rospy.get_param('~centroid_path', 'circular')
        self.position_tolerance = max(
            0.03, float(rospy.get_param('~position_tolerance', 0.09))
        )
        self.position_release_tolerance = max(
            self.position_tolerance + 0.02,
            float(rospy.get_param('~position_release_tolerance', 0.14)),
        )
        self.settle_time = max(
            0.0, float(rospy.get_param('~settle_time', 0.5))
        )
        self.arena_size = float(rospy.get_param('~arena_size', 10.0))
        self.arena_margin = max(
            0.0, float(rospy.get_param('~arena_margin', 0.35))
        )
        self.arena_profile = str(
            rospy.get_param('~arena_profile', 'swarm_arena')
        )
        self.formation_obstacle_clearance = max(
            0.0,
            float(rospy.get_param('~spawn_obstacle_clearance', 0.30)),
        )
        self.formation_search_step = max(
            0.05, float(rospy.get_param('~spawn_search_step', 0.10))
        )
        self.formation_search_limit = max(
            1, int(rospy.get_param('~formation_search_limit', 512))
        )
        self.spawn_exclusion_zones = rospy.get_param(
            '~spawn_exclusion_zones', []
        )
        if not isinstance(self.spawn_exclusion_zones, list):
            self.spawn_exclusion_zones = []

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
        self._emergency_stop_generation = 0
        self._latest_emergency_true_generation = 0
        self._emergency_true_pending = set()
        self.current_task_id = None
        # Start and stop travel on separate ROS topics. Remember a bounded set
        # of stops which arrived first, so transport reordering cannot revive
        # a task the orchestrator already cancelled.
        self._cancelled_task_ids = {}
        self.formation_state = FormationState.IDLE
        self.lock = threading.RLock()
        self.command_lock = threading.RLock()

        # Centroid state
        self.centroid_x = 0.0
        self.centroid_y = 0.0
        self.centroid_heading = 0.0  # heading angle of centroid path
        # The path heading is useful for visualization, but it must not rotate
        # a letter (or any other rigid formation) as the centroid travels.
        self.formation_heading = 0.0
        self.centroid_time = 0.0     # elapsed time for path parametrization
        self.current_waypoint_idx = 0
        self.path_center_x = 0.0
        self.path_center_y = 0.0
        self.effective_path_radius = self.path_radius
        self.orbit_path_validated = False
        self.orbit_validation_samples = 0
        self.orbit_validation_live_models = 0
        self.orbit_radius_adapted = False
        self._live_orbit_validation_key = None
        self._live_orbit_validation_safe = False
        # A moving target is held still while the robots assemble. Starting the
        # path before that point turns formation into a chase, especially for a
        # large fleet whose slots take longer to fill.
        self._initial_formation_acquired = False
        self._maximum_position_error = 0.0

        # Robots (populated from /fleet/robot_list or default namespace list)
        self.robot_ids: List[str] = []
        self.robot_poses: Dict[str, Optional[Pose]] = {}
        self.robot_yaws: Dict[str, float] = {}
        self.invalid_robot_poses: Tuple[str, ...] = ()
        self.odom_received_at: Dict[str, Optional[float]] = {}
        # A pose received before a task starts is useful for planning, but it
        # is not proof that the new task still has a live odometry stream.
        # Each start waits for one new sample per robot before applying the
        # normal, deliberately short stale-data timeout.
        self.odom_confirmed_for_task: Dict[str, bool] = {}
        self.odom_timeout_wall_s = max(
            0.2, float(rospy.get_param('~odom_timeout_wall_s', 0.75))
        )
        initialization_timeout = float(rospy.get_param(
            '~odom_initialization_timeout_wall_s', 10.0
        ))
        if not math.isfinite(initialization_timeout):
            initialization_timeout = 10.0
        self.odom_initialization_timeout_wall_s = max(
            self.odom_timeout_wall_s, initialization_timeout
        )
        self.task_started_at: Optional[float] = None
        self.stale_odometry: List[str] = []
        self.waiting_for_odometry: List[str] = []
        self.model_poses: Dict[str, Tuple[float, float, float]] = {}
        self.invalid_model_poses: Tuple[str, ...] = ()
        self.invalid_avoidance_limits: Tuple[str, ...] = ()
        self.placement_error: Optional[str] = None

        # Formation positions: list of (x, y) offsets from centroid (unscaled)
        self.formation_offsets: List[Tuple[float, float]] = []
        # Mapping: robot_id -> index in formation_offsets
        self.assignments: Dict[str, int] = {}
        self.route_waypoints: Dict[str, List[Tuple[float, float]]] = {}
        self.route_waypoint_indices: Dict[str, int] = {}
        self.route_batches: List[List[str]] = []
        self.route_batch_index = 0
        self.active_placement_plan: Optional[Dict] = None
        self.assignment_pending = False
        self._assignment_generation = 0
        # A robot can settle after a long placement solve. Retry from the live
        # poses a small, fixed number of times instead of releasing a stale
        # route or failing immediately on harmless spawn jitter.
        self.live_replan_limit = 2
        self._live_replan_attempts = 0
        self._route_progress_token = None
        self._route_progress_best: Dict[str, float] = {}
        self._route_stall_duration: Dict[str, float] = {}
        self._active_route_batch_ids: Tuple[str, ...] = ()
        self._last_route_stall_detail: Optional[Dict] = None
        self._stop_publication_attempt = 0
        self._last_stop_publication = None
        # A failed zero command belongs to one publisher instance, not merely
        # to a robot name.  Keep that instance reachable until it accepts a
        # zero; otherwise a fleet resize could hide the failed stop by
        # removing the robot and successfully stopping only the reduced
        # roster.  A returning robot reuses its retained publisher, which
        # bounds the registry to the publishers that are active or still owe
        # a stop acknowledgement at this local boundary.
        self._stop_publication_lock = threading.RLock()
        # This lock only protects the hand-off from regular stop accounting to
        # the final shutdown result. It is never held while talking to ROS.
        self._stop_result_lock = threading.RLock()
        self._emergency_stop_condition = threading.Condition(
            self._stop_result_lock
        )
        self._command_publisher_sequence = 0
        self._command_publisher_generations = {}
        self._stop_publication_debts = {}
        self._safety_fallback_lanes = {}
        self._safety_fallback_lane_factory = SafetyPublishLane
        self._shutdown_publisher_snapshot = ()
        self._shutdown_started = False
        self._shutdown_publication = None
        self._shutdown_status_delivery = None
        self._shutdown_supervisor_delivery = None
        self._safety_stop_ack_lock = threading.Lock()
        self._safety_stop_ack_waiters = {}
        self._motion_publish_inflight = set()
        self._shutdown_motion_inflight = frozenset()
        # Gazebo's differential-drive plugins can finish settling a freshly
        # spawned Burger by a few centimetres while a ten-robot route is being
        # solved. The complete live-geometry gate still validates every route
        # before release; these tolerances only avoid throwing away a safe
        # plan for normal spawn settling.
        self.assignment_pose_drift_tolerance = min(
            0.15,
            max(0.02, float(rospy.get_param(
                '~assignment_pose_drift_tolerance', 0.08
            ))),
        )
        self.assignment_yaw_drift_tolerance = min(
            0.20,
            max(0.05, float(rospy.get_param(
                '~assignment_yaw_drift_tolerance', 0.15
            ))),
        )
        # Route and orbit planning can take several seconds for ten robots.
        # Never perform that work on an odometry callback thread: blocking the
        # callback would make the very sample which triggered planning appear
        # stale. A single worker coalesces repeated requests, while generation
        # checks reject any result made obsolete by a newer fleet or task.
        self._assignment_worker_stop = threading.Event()
        self._assignment_worker_wakeup = threading.Event()
        self._assignment_worker = threading.Thread(
            target=self._assignment_worker_loop,
            name='formation-assignment-worker',
            daemon=True,
        )
        self._assignment_worker.start()
        self._slot_reached: Dict[str, bool] = {}
        self._settled_duration = 0.0

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
        # Shutdown waits for a correlated ACK. Keep this one publisher
        # synchronous so a return cannot mean only "placed in a local queue".
        self.safety_stop_request_pub = rospy.Publisher(
            '/swarm/safety_stop_request', String
        )
        if not self._register_safety_fallback_lane(
            'supervisor-request', self.safety_stop_request_pub
        ):
            raise RuntimeError(
                "Could not reserve the supervised-stop request lane"
            )
        self.marker_pub = rospy.Publisher(
            '/formation/markers', MarkerArray, queue_size=1
        )

        # Global subscribers
        rospy.Subscriber(
            '/fleet/robot_list', String, self._fleet_list_cb, queue_size=1
        )
        rospy.Subscriber(
            '/gazebo/model_states', ModelStates,
            self._model_states_cb, queue_size=1,
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
        rospy.Subscriber(
            '/swarm/safety_stop_ack', String,
            self._safety_stop_ack_cb, queue_size=1,
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
        rospy.on_shutdown(self._shutdown)

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
        fleet_stop_publication = None
        with self.command_lock:
            if self._shutdown_started:
                return
            if ids != self.robot_ids:
                self._initial_formation_acquired = False
                if self.robot_ids:
                    if not ids:
                        self.is_running = False
                        self.is_paused = False
                        self.formation_state = FormationState.STOPPED
                        self._cancel_pending_assignment_locked(
                            clear_assignments=True
                        )
                    fleet_stop_publication = self._stop_all_robots(
                        'fleet_empty' if not ids else 'fleet_change'
                    )
                    if self._shutdown_started:
                        return
                    if (
                        not ids
                        and fleet_stop_publication[
                            'publication_confirmed'
                        ]
                    ):
                        self.current_task_id = None

                self._update_robot_list(ids)
                if self._shutdown_started:
                    return
                if (
                    ids
                    and (
                        fleet_stop_publication is None
                        or fleet_stop_publication[
                            'publication_confirmed'
                        ]
                    )
                ):
                    assignment_snapshot = self._recompute_formation_locked()
                else:
                    if not ids:
                        self.formation_offsets = []
                    self.assignments = {}
                    self.assignment_pending = False
        if fleet_stop_publication is not None:
            self._publish_stop_status_safely('fleet-change stop')
        self._compute_and_commit_assignment(assignment_snapshot)

    def _model_states_cb(self, msg: ModelStates):
        """Track configured obstacle poses, including the pushable object."""
        arena_profile = getattr(self, 'arena_profile', 'swarm_arena')
        configured_models = set()
        for zone in self.spawn_exclusion_zones:
            if not isinstance(zone, dict) or not zone.get('model'):
                continue
            worlds = zone.get('worlds')
            if isinstance(worlds, str):
                worlds = [worlds]
            if worlds and arena_profile not in worlds:
                continue
            configured_models.add(zone['model'])

        try:
            names = list(getattr(msg, 'name', ()))
            poses = list(getattr(msg, 'pose', ()))
        except (TypeError, ValueError):
            names = poses = None

        with self.lock:
            invalid_models = set(getattr(
                self, 'invalid_model_poses', ()
            )) & configured_models

        names_are_valid = (
            names is not None
            and all(isinstance(name, str) for name in names)
        )
        snapshot_is_ambiguous = (
            names_are_valid and len(set(names)) != len(names)
        )
        if (
            not names_are_valid
            or len(names) != len(poses)
            or snapshot_is_ambiguous
        ):
            # ModelStates is one scene snapshot. If its parallel arrays do not
            # line up or one name occurs twice, none of the active obstacle
            # poses can be trusted.
            invalid_models.update(configured_models)
            with self.lock:
                self.model_poses = {}
                self.invalid_model_poses = tuple(sorted(invalid_models))
            return

        live_poses = {}
        for model_name, pose in zip(names, poses):
            if model_name not in configured_models:
                continue
            try:
                orientation = pose.orientation
                quaternion = tuple(
                    float(getattr(orientation, axis))
                    for axis in ('x', 'y', 'z', 'w')
                )
                if not all(math.isfinite(value) for value in quaternion):
                    raise ValueError("model quaternion is not finite")
                live_pose = (
                    float(pose.position.x),
                    float(pose.position.y),
                    float(quaternion_to_yaw(orientation)),
                )
                pose_is_valid = all(
                    math.isfinite(value) for value in live_pose
                )
            except (
                AttributeError, TypeError, ValueError, OverflowError
            ):
                pose_is_valid = False

            if pose_is_valid:
                live_poses[model_name] = live_pose
                invalid_models.discard(model_name)
            else:
                # Do not drop back to the configured pose silently. Remember
                # the rejected sample so the next command batch fails closed.
                invalid_models.add(model_name)
        with self.lock:
            self.model_poses = live_poses
            self.invalid_model_poses = tuple(sorted(invalid_models))

    def _set_shape_cb(self, msg: String):
        """Dynamically change formation type at runtime."""
        new_type = msg.data.strip()
        assignment_snapshot = None
        with self.command_lock:
            if self._shutdown_started:
                return
            if new_type and new_type != self.formation_type:
                rospy.loginfo(
                    "Formation shape changed: %s -> %s",
                    self.formation_type, new_type,
                )
                self.formation_type = new_type
                self._initial_formation_acquired = False
                assignment_snapshot = self._recompute_formation_locked()
                if self._shutdown_started:
                    self._cancel_pending_assignment_locked(
                        clear_assignments=True
                    )
                    return
        self._compute_and_commit_assignment(assignment_snapshot)

    def _start_cb(self, msg):
        """Start the formation behavior with optional runtime config."""
        # Parse config from task orchestrator (JSON String)
        try:
            config = json.loads(msg.data) if msg.data else {}
        except (json.JSONDecodeError, AttributeError):
            config = {}
        if not isinstance(config, dict):
            config = {}
        requested_task_id = config.get('task_id')
        if (
            not isinstance(requested_task_id, str)
            or not requested_task_id.strip()
        ):
            rospy.logwarn(
                "Formation start rejected because task_id is missing or "
                "invalid"
            )
            return

        assignment_snapshot = None
        with self.command_lock:
            if getattr(self, '_shutdown_started', False):
                rospy.logwarn(
                    "Formation start rejected because shutdown has begun"
                )
                return
            if requested_task_id in getattr(
                self, '_cancelled_task_ids', {}
            ):
                rospy.logwarn(
                    "Formation start rejected because task %s was already "
                    "cancelled",
                    requested_task_id,
                )
                return
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
            last_stop = getattr(self, '_last_stop_publication', None)
            with self.lock:
                if self._shutdown_started:
                    return
                pending_stop_debt = bool(getattr(
                    self, '_stop_publication_debts', {}
                ))
            if (
                pending_stop_debt
                or (
                    last_stop is not None
                    and not last_stop.get('publication_confirmed', False)
                )
            ):
                rospy.logwarn(
                    "Formation start rejected because the previous "
                    "zero-velocity publication was not accepted by every "
                    "robot publisher"
                )
                self._publish_stop_status_safely('start rejection')
                return

            requested_formation = self.formation_type
            requested_mode = self.movement_mode
            requested_spacing = self.spacing
            requested_radius = self.path_radius
            recompute = False
            if 'formation_type' in config:
                new_type = config['formation_type']
                recompute = recompute or new_type != self.formation_type
                requested_formation = new_type
            if 'movement_mode' in config:
                try:
                    requested_mode = MovementMode(config['movement_mode'])
                except ValueError:
                    pass
            if 'spacing' in config:
                try:
                    new_spacing = max(0.35, float(config['spacing']))
                    recompute = recompute or new_spacing != self.spacing
                    requested_spacing = new_spacing
                except (TypeError, ValueError):
                    pass
            if 'path_radius' in config:
                try:
                    new_radius = max(0.10, float(config['path_radius']))
                    recompute = recompute or new_radius != self.path_radius
                    requested_radius = new_radius
                except (TypeError, ValueError):
                    pass

            task_started_at = time.monotonic()
            with self.lock:
                # Pair the final start commit with the shutdown epoch. If
                # shutdown wins this short hand-off, no field can reactivate
                # the controller after its final state has been published.
                with self._stop_result_lock:
                    if (
                        self._shutdown_started
                        or self.emergency_stop_active
                    ):
                        return
                    self.formation_type = requested_formation
                    self.movement_mode = requested_mode
                    self.spacing = requested_spacing
                    self.path_radius = requested_radius
                    self.current_task_id = requested_task_id
                    self._last_stop_publication = None
                    self.task_started_at = task_started_at
                    self.odom_confirmed_for_task = {
                        robot_id: False for robot_id in self.robot_ids
                    }
                    self.stale_odometry = []
                    self.waiting_for_odometry = []
                    self.is_running = True
                    self.is_paused = False
                    self.formation_state = FormationState.FORMING
                    self.placement_error = None
                    self._slot_reached = {}
                    self._settled_duration = 0.0
                    self._initial_formation_acquired = False
                    self._live_replan_attempts = 0
                    self._active_route_batch_ids = ()
                    self._last_route_stall_detail = None
                    self._reset_centroid_path_pose_locked()

                    for pid in self.pid_linear.values():
                        pid.reset()
                    for pid in self.pid_angular.values():
                        pid.reset()

            if recompute:
                assignment_snapshot = self._recompute_formation_locked()
            else:
                assignment_snapshot = self._prepare_assignment_locked()

            if self._shutdown_started:
                self.is_running = False
                self.is_paused = False
                self._cancel_pending_assignment_locked(
                    clear_assignments=True
                )
                return
        # Route planning for a large fleet can take longer than the normal
        # behavior heartbeat window. Publish the correlated, stationary
        # FORMING state before entering the solver so the orchestrator can
        # distinguish bounded planning from a node that never accepted the
        # task.
        try:
            self._publish_status([], 0.0)
        except Exception as exc:
            rospy.logwarn(
                "Formation planning status could not be published: %s", exc
            )
        self._compute_and_commit_assignment(assignment_snapshot)
        if self._shutdown_started:
            return
        if self.placement_error:
            rospy.logerr(
                "Formation control could not start: %s",
                self.placement_error,
            )
            return
        rospy.loginfo(
            "Formation control STARTED: type=%s, mode=%s",
            self.formation_type, self.movement_mode.value,
        )

    def _stop_cb(self, msg: String):
        """Stop the formation behavior and halt all robots."""
        requested_task_id = self._task_id_from_message(msg)
        if requested_task_id is None:
            return
        with self.command_lock:
            self._remember_cancelled_task_locked(requested_task_id)
            if requested_task_id != self.current_task_id:
                return
            with self._stop_result_lock:
                if self._shutdown_started:
                    return
                self.is_running = False
                self.is_paused = False
                self.formation_state = FormationState.STOPPED
                self.placement_error = None
                self._initial_formation_acquired = False
            self._cancel_pending_assignment_locked(clear_assignments=True)
            publication = self._stop_all_robots('task_stop')
            if self._shutdown_started:
                return
            self._publish_stop_status_safely('stop')
        if publication['publication_confirmed']:
            rospy.loginfo("Formation control STOPPED")
        else:
            rospy.logerr(
                "Formation control stop remains unconfirmed at the local "
                "publication boundary"
            )

    def _pause_cb(self, msg: String):
        with self.command_lock:
            if not self._task_command_matches(msg):
                return
            with self._stop_result_lock:
                if (
                    self._shutdown_started
                    or not self.is_running
                    or self.is_paused
                ):
                    return
                self.is_paused = True
            publication = self._stop_all_robots('task_pause')
            if self._shutdown_started:
                return
            self._publish_stop_status_safely('pause')
        if publication['publication_confirmed']:
            rospy.loginfo("Formation control PAUSED")
        else:
            rospy.logerr(
                "Formation pause failed because zero velocity was not "
                "accepted by every local publisher"
            )

    def _resume_cb(self, msg: String):
        with self.command_lock:
            if not self._task_command_matches(msg):
                return
            with self._stop_result_lock:
                if (
                    self._shutdown_started
                    or not self.is_running
                    or not self.is_paused
                    or self.emergency_stop_active
                ):
                    return
                self.is_paused = False
        rospy.loginfo("Formation control RESUMED")

    @staticmethod
    def _task_id_from_message(msg: String) -> Optional[str]:
        try:
            payload = json.loads(msg.data) if msg.data else {}
        except (json.JSONDecodeError, AttributeError):
            payload = {}
        requested_id = payload.get('task_id')
        if not isinstance(requested_id, str) or not requested_id.strip():
            return None
        return requested_id

    def _remember_cancelled_task_locked(self, task_id: str):
        cancelled = getattr(self, '_cancelled_task_ids', None)
        if cancelled is None:
            self._cancelled_task_ids = {}
            cancelled = self._cancelled_task_ids
        if task_id in cancelled:
            return
        if len(cancelled) >= 64:
            cancelled.pop(next(iter(cancelled)))
        cancelled[task_id] = None

    def _task_command_matches(self, msg: String) -> bool:
        requested_id = self._task_id_from_message(msg)
        return requested_id is not None and requested_id == self.current_task_id

    def _emergency_stop_cb(self, msg: Bool):
        requested_stop = bool(msg.data)
        with self._emergency_stop_condition:
            if self._shutdown_started:
                return
            self._emergency_stop_generation = getattr(
                self, '_emergency_stop_generation', 0
            ) + 1
            generation = self._emergency_stop_generation
            if not requested_stop:
                required_true_generation = getattr(
                    self, '_latest_emergency_true_generation', 0
                )
            else:
                self._latest_emergency_true_generation = generation
                self._emergency_true_pending.add(generation)
                # Close the motion gate before command_lock. A normal stop or
                # motion publisher may be blocked while holding that lock.
                self.emergency_stop_active = True
                self.is_running = False
                self.is_paused = False
                self.formation_state = FormationState.STOPPED
                self.placement_error = None
                self._initial_formation_acquired = False
                task_id = self.current_task_id

        if not requested_stop:
            with self._emergency_stop_condition:
                while any(
                    pending_generation <= required_true_generation
                    for pending_generation in self._emergency_true_pending
                ):
                    self._emergency_stop_condition.wait()
            with self.command_lock:
                with self._emergency_stop_condition:
                    if (
                        self._shutdown_started
                        or generation != self._emergency_stop_generation
                    ):
                        return
                    self.emergency_stop_active = False
            return

        # The latch above is visible before command_lock, so a blocked normal
        # publication cannot delay the independent emergency fan-out.
        publication = None
        latest_true = False
        try:
            targets = self._cached_command_publisher_snapshot()
            self._stop_from_shutdown_snapshot(
                targets,
                'emergency_stop',
                task_id,
                time.monotonic() + max(
                    0.0, float(self.EMERGENCY_STOP_TIMEOUT)
                ),
            )
            with self.command_lock:
                with self._emergency_stop_condition:
                    if self._shutdown_started:
                        return
                    current_edge = (
                        generation == self._emergency_stop_generation
                    )
                    latest_true = (
                        generation
                        == self._latest_emergency_true_generation
                    )
                    if current_edge:
                        self.emergency_stop_active = True
                    if latest_true:
                        # A start already waiting on the pose lock cannot
                        # restore these fields after this True edge.
                        self.is_running = False
                        self.is_paused = False
                        self.formation_state = FormationState.STOPPED
                        self._initial_formation_acquired = False
                if latest_true:
                    self._cancel_pending_assignment_locked(
                        clear_assignments=True
                    )
                    publication = self._stop_all_robots_bounded(
                        'emergency_stop', self.EMERGENCY_STOP_TIMEOUT
                    )
                    if self._shutdown_started:
                        return
                    if not publication['publication_confirmed']:
                        self._fail_for_unconfirmed_stop_locked(
                            "Zero-velocity publication was not accepted for: "
                            + ", ".join(publication['failed_robots'])
                            + "."
                        )
                    self._publish_stop_status_safely('emergency stop')
        finally:
            with self._emergency_stop_condition:
                self._emergency_true_pending.discard(generation)
                self._emergency_stop_condition.notify_all()
        if publication is None:
            return
        if publication['publication_confirmed']:
            rospy.logwarn("Formation emergency stop latched")
        else:
            rospy.logerr(
                "Formation emergency stop latched, but zero-velocity "
                "publication was not accepted by every publisher"
            )

    def _shutdown(self):
        """Stop planning and publish a final zero command before leaving."""
        # This atomic flag also fences a lifecycle callback which began before
        # shutdown but finishes after the bounded emergency publication.
        with self._stop_result_lock:
            self._shutdown_started = True
            self._shutdown_motion_inflight = frozenset(
                self._motion_publish_inflight
            )
        shutdown_timeout = max(
            0.0, float(self.ASSIGNMENT_WORKER_SHUTDOWN_TIMEOUT)
        )
        shutdown_deadline = time.monotonic() + shutdown_timeout
        status_reserve = min(0.02, shutdown_timeout / 4.0)
        work_deadline = shutdown_deadline - status_reserve
        request_budget = min(0.05, shutdown_timeout / 4.0)
        request_deadline = min(
            work_deadline,
            time.monotonic() + request_budget,
        )
        supervisor_delivery = self._request_supervised_shutdown_stop(
            request_deadline
        )
        supervisor_error = None
        if not supervisor_delivery['publication_confirmed']:
            supervisor_error = (
                "The independent safety-stop request was not confirmed "
                "before the bounded shutdown deadline."
            )
        worker_stop = getattr(self, '_assignment_worker_stop', None)
        worker_wakeup = getattr(self, '_assignment_worker_wakeup', None)
        worker = getattr(self, '_assignment_worker', None)
        # Close the worker's input first. A plan which is already being solved
        # is invalidated after acquiring the lifecycle lock. Keep one deadline
        # for lock acquisition, zero publication, and the worker join: a
        # planner which accidentally retains command_lock must not deadlock ROS
        # shutdown before the old join timeout even starts.
        if worker_stop is not None:
            worker_stop.set()
        if worker_wakeup is not None:
            worker_wakeup.set()

        publisher_snapshot = self._cached_command_publisher_snapshot()
        status_context = self._shutdown_status_context()
        status_context['supervisor_stop_request'] = supervisor_delivery
        remaining = max(0.0, work_deadline - time.monotonic())
        publication_reserve = min(0.10, remaining / 2.0)
        lifecycle_wait = max(0.0, remaining - publication_reserve)
        lifecycle_lock_acquired = self.command_lock.acquire(
            timeout=lifecycle_wait
        )
        if not lifecycle_lock_acquired:
            publication = self._stop_from_shutdown_snapshot(
                publisher_snapshot,
                'shutdown',
                status_context['task_id'],
                work_deadline,
                self._shutdown_motion_inflight,
            )
            self._seal_shutdown_publication(publication)
            self._publish_bounded_shutdown_failure(
                "Formation shutdown could not acquire the lifecycle lock "
                "before its bounded deadline. The assignment worker exit "
                "and lifecycle cleanup were not confirmed."
                + (
                    " " + supervisor_error
                    if supervisor_error is not None else ""
                ),
                publication,
                status_context,
                shutdown_deadline,
            )
            return

        try:
            self.is_running = False
            self.is_paused = False
            self.formation_state = FormationState.STOPPED
            self._initial_formation_acquired = False
            self._cancel_pending_assignment_locked(clear_assignments=True)
            publication = self._stop_from_shutdown_snapshot(
                publisher_snapshot,
                'shutdown',
                status_context['task_id'],
                work_deadline,
                self._shutdown_motion_inflight,
            )
            self._stop_publication_attempt = publication['attempt']
            self._seal_shutdown_publication(publication)
            if not publication['publication_confirmed']:
                self._fail_for_unconfirmed_stop_locked(
                    "Zero-velocity publication was not accepted for: "
                    + ", ".join(publication['failed_robots'])
                    + "."
                )
            if supervisor_error is not None:
                self._fail_for_unconfirmed_stop_locked(supervisor_error)
        finally:
            self.command_lock.release()

        if worker is not None and worker is not threading.current_thread():
            worker.join(timeout=max(
                0.0, work_deadline - time.monotonic()
            ))
            if worker.is_alive():
                self._publish_bounded_shutdown_failure(
                    "Formation assignment worker did not stop before the "
                    "bounded shutdown deadline. Its exit and lifecycle "
                    "cleanup were not confirmed.",
                    publication,
                    status_context,
                    shutdown_deadline,
                )
                return

        self._publish_shutdown_status(
            status_context,
            publication,
            self.formation_state,
            self.placement_error,
            shutdown_deadline,
        )

    def _seal_shutdown_publication(self, publication):
        """Make the bounded shutdown result final for late callbacks."""
        with self._stop_result_lock:
            self._shutdown_publication = publication
            self._last_stop_publication = publication

    def _safety_stop_ack_cb(self, msg):
        """Release only the waiter named by a positive supervisor ACK."""
        try:
            payload = json.loads(msg.data)
        except (AttributeError, TypeError, ValueError):
            return
        if not isinstance(payload, dict):
            return
        request_id = payload.get('request_id')
        if (
            not isinstance(request_id, str)
            or not request_id
            or payload.get('accepted') is not True
            or payload.get('supervisor_latched') is not True
            or payload.get('supervisor_will_remain_active') is not True
            or payload.get('valid_request') is not True
            or payload.get('zero_publications_confirmed') is not True
        ):
            return
        with self._safety_stop_ack_lock:
            waiter = self._safety_stop_ack_waiters.get(request_id)
            if waiter is None:
                return
            waiter['ack'] = dict(payload)
            waiter['event'].set()

    def _request_supervised_shutdown_stop(self, deadline):
        """Request and observe an ACK from the independent orchestrator."""
        publisher = getattr(self, 'safety_stop_request_pub', None)
        if publisher is None:
            delivery = {
                'attempted': False,
                'subscriber_connected': False,
                'publication_returned': False,
                'acknowledgement_received': False,
                'publication_confirmed': False,
                'error': 'safety-stop request publisher is unavailable',
            }
            with self._stop_result_lock:
                self._shutdown_supervisor_delivery = delivery
            return delivery
        request_id = uuid.uuid4().hex
        waiter = {'event': threading.Event(), 'ack': None}
        with self._safety_stop_ack_lock:
            self._safety_stop_ack_waiters[request_id] = waiter

        request = String(data=json.dumps({
            'source': 'formation',
            'reason': 'shutdown',
            'request_id': request_id,
            'task_id': (
                self.current_task_id
                if isinstance(self.current_task_id, str)
                else None
            ),
        }))

        connection_count = getattr(publisher, 'get_num_connections', None)
        subscriber_connected = None
        if callable(connection_count):
            try:
                subscriber_connected = connection_count() > 0
            except Exception:
                subscriber_connected = None

        result = {'error': None}
        publication_finished = threading.Event()

        def publish_request():
            try:
                publisher.publish(request)
            except Exception as exc:
                result['error'] = str(exc)[:256]
                try:
                    rospy.logerr(
                        "Formation could not request the supervised stop: %s",
                        exc,
                    )
                except Exception:
                    pass
            finally:
                publication_finished.set()

        request_started = False
        last_start_error = None
        for attempt in (1, 2):
            request_thread = threading.Thread(
                target=publish_request,
                name='formation-supervised-stop-request-{}'.format(attempt),
                daemon=True,
            )
            try:
                request_thread.start()
                request_started = True
                break
            except Exception as exc:
                last_start_error = exc
                try:
                    rospy.logerr(
                        "Supervised-stop request worker could not start "
                        "(attempt %d): %s",
                        attempt,
                        exc,
                    )
                except Exception:
                    pass
        if not request_started and self._submit_safety_fallback(
            publisher, publish_request
        ):
            request_started = True
        if not request_started:
            with self._safety_stop_ack_lock:
                self._safety_stop_ack_waiters.pop(request_id, None)
            delivery = {
                'attempted': False,
                'subscriber_connected': subscriber_connected,
                'publication_returned': False,
                'acknowledgement_received': False,
                'publication_confirmed': False,
                'error': (
                    'safety-stop request worker could not start: '
                    + str(last_start_error)[:192]
                ),
            }
            with self._stop_result_lock:
                self._shutdown_supervisor_delivery = delivery
            return delivery
        publication_finished.wait(max(
            0.0, deadline - time.monotonic()
        ))
        publication_returned = publication_finished.is_set()
        if publication_returned and result['error'] is None:
            waiter['event'].wait(max(
                0.0, deadline - time.monotonic()
            ))
        with self._safety_stop_ack_lock:
            self._safety_stop_ack_waiters.pop(request_id, None)
            acknowledgement_received = waiter['ack'] is not None
        confirmed = (
            publication_returned
            and result['error'] is None
            and subscriber_connected is not False
            and acknowledgement_received
        )
        error = result['error']
        if error is None and not publication_returned:
            error = 'safety-stop request exceeded shutdown deadline'
        elif error is None and subscriber_connected is False:
            error = 'safety-stop supervisor had no ROS subscriber'
        elif error is None and not acknowledgement_received:
            error = 'safety-stop acknowledgement exceeded shutdown deadline'
        delivery = {
            'attempted': True,
            'subscriber_connected': subscriber_connected,
            'publication_returned': publication_returned,
            'acknowledgement_received': acknowledgement_received,
            'publication_confirmed': confirmed,
            'error': None if confirmed else error,
        }
        with self._stop_result_lock:
            self._shutdown_supervisor_delivery = delivery
        return delivery

    def _shutdown_status_context(self):
        """Capture only atomic shutdown fields; never traverse live mappings."""
        movement_mode = getattr(self, 'movement_mode', MovementMode.STATIC)
        return {
            'task_id': (
                self.current_task_id
                if isinstance(self.current_task_id, str)
                else None
            ),
            'formation_type': getattr(self, 'formation_type', 'unknown'),
            'movement_mode': movement_mode.value,
            'centroid_x': float(getattr(self, 'centroid_x', 0.0)),
            'centroid_y': float(getattr(self, 'centroid_y', 0.0)),
            'centroid_heading': float(getattr(
                self, 'centroid_heading', 0.0
            )),
            'formation_heading': float(getattr(
                self, 'formation_heading', 0.0
            )),
            'robot_count': int(getattr(self, 'robot_count', 0)),
            'settled_for': float(getattr(self, '_settled_duration', 0.0)),
            'status_publisher': self.status_pub,
        }

    def _publish_shutdown_status(
        self, context, publication, state, error, deadline
    ):
        """Publish shutdown state from immutable scalar and stop snapshots."""
        status = {
            'task_id': context['task_id'] or publication.get('task_id'),
            'paused': False,
            'formation_type': context['formation_type'],
            'movement_mode': context['movement_mode'],
            'state': state.value,
            'centroid': {
                'x': round(context['centroid_x'], 4),
                'y': round(context['centroid_y'], 4),
            },
            'centroid_heading': round(context['centroid_heading'], 4),
            'formation_heading': round(context['formation_heading'], 4),
            'robot_count': context['robot_count'],
            'maximum_position_error': 0.0,
            'settled_for': round(context['settled_for'], 3),
            'robot_assignments': {},
            'stale_odometry': [],
            'waiting_for_odometry': [],
            'stop_publication': self._stop_publication_status_payload(
                publication
            ),
            'supervisor_stop_request': dict(
                context['supervisor_stop_request']
            ),
        }
        if error:
            status['error'] = error
        status_message = String(data=json.dumps(status))
        with self._stop_result_lock:
            self._shutdown_status_delivery = (
                context['status_publisher'], status_message
            )

        def publish_status():
            try:
                context['status_publisher'].publish(status_message)
            except Exception as exc:
                try:
                    rospy.logerr(
                        "Formation shutdown status could not be published: "
                        "%s",
                        exc,
                    )
                except Exception:
                    pass

        status_thread = threading.Thread(
            target=publish_status,
            name='formation-shutdown-status',
            daemon=True,
        )
        try:
            status_thread.start()
        except Exception as exc:
            try:
                rospy.logerr(
                    "Formation shutdown status worker could not start: %s",
                    exc,
                )
            except Exception:
                pass
            return
        status_thread.join(timeout=max(
            0.0, deadline - time.monotonic()
        ))

    def _reassert_shutdown_status(self):
        """Put the final snapshot after a status publish that returned late."""
        with self._stop_result_lock:
            delivery = self._shutdown_status_delivery
        if delivery is None:
            return
        publisher, message = delivery
        for attempt in (1, 2):
            worker = threading.Thread(
                target=self._shutdown_status_reassertion_attempt,
                args=(publisher, message, attempt),
                name='formation-shutdown-status-reassert-{}'.format(attempt),
                daemon=True,
            )
            try:
                worker.start()
            except Exception as exc:
                try:
                    rospy.logerr(
                        "Formation shutdown status reassertion worker %d "
                        "could not start: %s",
                        attempt,
                        exc,
                    )
                except Exception:
                    pass

    @staticmethod
    def _shutdown_status_reassertion_attempt(publisher, message, attempt):
        """Run one detached attempt to restore the terminal status order."""
        try:
            publisher.publish(message)
        except Exception as exc:
            try:
                rospy.logerr(
                    "Formation shutdown status reassertion %d failed: %s",
                    attempt,
                    exc,
                )
            except Exception:
                pass

    def _publish_bounded_shutdown_failure(
        self, error, publication, status_context, deadline
    ):
        """Publish a minimal immutable failure status without command_lock."""
        self.is_running = False
        self.is_paused = False
        self._initial_formation_acquired = False
        self.formation_state = FormationState.FAILED
        self.orbit_path_validated = False
        if self.placement_error:
            if error not in self.placement_error:
                self.placement_error = (
                    self.placement_error.rstrip() + " " + error
                )
        else:
            self.placement_error = error
        self._publish_shutdown_status(
            status_context,
            publication,
            FormationState.FAILED,
            self.placement_error,
            deadline,
        )

    def _odom_cb(self, msg: Odometry, robot_id: str):
        """Per-robot odometry callback."""
        pose = None
        yaw = None
        try:
            pose = msg.pose.pose
            orientation = pose.orientation
            quaternion = tuple(
                float(getattr(orientation, axis))
                for axis in ('x', 'y', 'z', 'w')
            )
            if not all(math.isfinite(value) for value in quaternion):
                raise ValueError("odometry quaternion is not finite")
            yaw = quaternion_to_yaw(orientation)
            pose_is_valid = self._planar_robot_pose_is_finite(pose, yaw)
        except (AttributeError, TypeError, ValueError, OverflowError):
            pose_is_valid = False
        should_assign = False
        with self.lock:
            if robot_id not in self.robot_poses:
                return
            invalid_robots = set(getattr(
                self, 'invalid_robot_poses', ()
            ))
            if not pose_is_valid:
                # Keep the last valid pose, but make the rejected sample visible
                # to the same lock-protected gate that publishes cmd_vel.
                invalid_robots.add(robot_id)
                self.invalid_robot_poses = tuple(sorted(invalid_robots))
                return

            first_update = self.robot_poses.get(robot_id) is None
            invalid_robots.discard(robot_id)
            self.invalid_robot_poses = tuple(sorted(invalid_robots))
            observed_at = time.monotonic()
            self.robot_poses[robot_id] = pose
            self.robot_yaws[robot_id] = yaw
            self.odom_received_at[robot_id] = observed_at
            if (
                self.task_started_at is not None
                and observed_at >= self.task_started_at
            ):
                self.odom_confirmed_for_task[robot_id] = True
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
            self._request_pending_assignment()

    def _request_pending_assignment(self):
        """Wake the planner without holding up a sensor callback."""
        wakeup = getattr(self, '_assignment_worker_wakeup', None)
        if wakeup is not None:
            wakeup.set()

    def _assignment_worker_loop(self):
        """Coalesce pending plans and solve them away from ROS callbacks."""
        while not self._assignment_worker_stop.is_set():
            self._assignment_worker_wakeup.wait(0.25)
            if self._assignment_worker_stop.is_set():
                break
            if not self._assignment_worker_wakeup.is_set():
                continue
            self._assignment_worker_wakeup.clear()
            prepared_generation = None
            with self.command_lock:
                if not self.assignment_pending:
                    continue
                try:
                    assignment_snapshot = self._prepare_assignment_locked()
                    prepared_generation = self._assignment_generation
                except Exception as exc:
                    failed_generation = self._assignment_generation
                    self._fail_assignment_worker(
                        exc,
                        'snapshot preparation',
                        failed_generation,
                    )
                    continue
            try:
                self._compute_and_commit_assignment(assignment_snapshot)
            except Exception as exc:
                self._fail_assignment_worker(
                    exc,
                    'plan computation',
                    prepared_generation,
                )

    def _fail_assignment_worker(
        self,
        exc: Exception,
        stage: str,
        expected_generation: Optional[int],
    ) -> bool:
        """Fail the current task closed without terminating the worker."""
        if self._shutdown_started:
            return False
        error = (
            "Formation assignment planning failed unexpectedly during "
            f"{stage}; a zero-velocity stop was requested for the fleet."
        )
        status_error = None
        handler_error = None
        applied = False
        try:
            with self.command_lock:
                if self._shutdown_started:
                    return False
                if (
                    expected_generation is not None
                    and expected_generation != self._assignment_generation
                ):
                    return False
                self.is_running = False
                self.is_paused = False
                self._initial_formation_acquired = False
                self._cancel_pending_assignment_locked(
                    clear_assignments=True
                )
                self.orbit_path_validated = False
                self.placement_error = error
                self.formation_state = FormationState.FAILED
                # Each command publisher is attempted independently. Keep the
                # status channel independent too: neither a broken socket nor
                # a test double is allowed to terminate this worker.
                self._stop_all_robots('assignment_worker_failure')
                try:
                    self._publish_status([], 0.0)
                except Exception as status_exc:
                    status_error = status_exc
                applied = True
        except Exception as failure_exc:
            handler_error = failure_exc

        for message, detail in (
            (
                "Formation assignment worker failed closed during %s: %s",
                (stage, exc),
            ),
            (
                "Formation assignment failure status could not be "
                "published: %s",
                (status_error,),
            ),
            (
                "Formation assignment failure handler encountered an "
                "additional error: %s",
                (handler_error,),
            ),
        ):
            if detail[0] is None:
                continue
            try:
                rospy.logerr(message, *detail)
            except Exception:
                pass
        return applied

    # ------------------------------------------------------------------
    # Robot list management
    # ------------------------------------------------------------------
    @staticmethod
    def _report_safety_lane_error(label, action, error):
        try:
            rospy.logerr(
                "Safety publication lane %s %s: %s", label, action, error
            )
        except Exception:
            pass

    def _register_safety_fallback_lane(self, label, publisher):
        """Reserve a worker while the process still has normal resources."""
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

    def _ensure_command_publisher_registry_locked(self):
        """Create the small publisher registry for old test configurations."""
        if not hasattr(self, '_command_publisher_sequence'):
            self._command_publisher_sequence = 0
        if not hasattr(self, '_command_publisher_generations'):
            self._command_publisher_generations = {}
        if not hasattr(self, '_stop_publication_debts'):
            self._stop_publication_debts = {}

    def _command_publisher_generation_locked(self, robot_id, publisher):
        """Return a stable, non-address identity for one publisher object."""
        self._ensure_command_publisher_registry_locked()
        object_key = id(publisher)
        generation = self._command_publisher_generations.get(object_key)
        if generation is None:
            self._command_publisher_sequence += 1
            generation = self._command_publisher_sequence
            self._command_publisher_generations[object_key] = generation
        return generation

    def _refresh_shutdown_publisher_snapshot_locked(self):
        """Replace the lock-free shutdown snapshot with immutable entries."""
        self._ensure_command_publisher_registry_locked()
        retained_object_keys = {
            id(publisher) for publisher in self.cmd_vel_pubs.values()
        }
        retained_object_keys.update(
            id(debt['publisher'])
            for debt in self._stop_publication_debts.values()
        )
        for object_key in tuple(self._command_publisher_generations):
            if object_key not in retained_object_keys:
                self._command_publisher_generations.pop(object_key, None)
        publishers = {}
        for robot_id, publisher in self.cmd_vel_pubs.items():
            generation = self._command_publisher_generation_locked(
                robot_id, publisher
            )
            debt = self._stop_publication_debts.get(generation)
            publishers[generation] = (
                robot_id,
                publisher,
                generation,
                self._public_stop_debt(debt) if debt is not None else None,
            )
        for generation, debt in self._stop_publication_debts.items():
            if generation in publishers:
                continue
            publishers[generation] = (
                debt['robot_id'],
                debt['publisher'],
                generation,
                self._public_stop_debt(debt),
            )
        self._shutdown_publisher_snapshot = tuple(sorted(
            publishers.values(),
            key=lambda item: (item[0], item[2]),
        ))

    def _cached_command_publisher_snapshot(self):
        """Read the immutable emergency snapshot without acquiring a lock."""
        return tuple(
            {
                'robot_id': robot_id,
                'publisher': publisher,
                'publisher_generation': generation,
                'debt': dict(debt) if debt is not None else None,
            }
            for robot_id, publisher, generation, debt
            in getattr(self, '_shutdown_publisher_snapshot', ())
        )

    def _publisher_is_active_locked(self, publisher):
        return any(
            active_publisher is publisher
            for active_publisher in self.cmd_vel_pubs.values()
        )

    def _forget_publisher_generation_locked(self, publisher, generation):
        """Drop registry metadata once no command or debt retains the object."""
        if self._publisher_is_active_locked(publisher):
            return
        if generation in self._stop_publication_debts:
            return
        object_key = id(publisher)
        if self._command_publisher_generations.get(object_key) == generation:
            self._command_publisher_generations.pop(object_key, None)

    def _close_retired_command_publisher_locked(
        self, publisher, generation
    ):
        """Unregister a stopped publisher which no longer belongs to a robot."""
        try:
            publisher.unregister()
        except Exception as exc:
            try:
                rospy.logerr(
                    "Command publisher unregister failed for generation %d: "
                    "%s",
                    generation,
                    exc,
                )
            except Exception:
                pass
        self._discard_safety_fallback_lane(publisher)
        self._forget_publisher_generation_locked(publisher, generation)

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
                rid for rid in new_ids if rid in self.cmd_vel_pubs
            )
            self.robot_count = len(self.robot_ids)

    def _add_robot(self, robot_id: str):
        """Build one robot provisionally, then commit it behind shutdown."""
        with self._stop_result_lock:
            if self._shutdown_started:
                return False

        retained_publishers = [
            debt['publisher']
            for debt in getattr(self, '_stop_publication_debts', {}).values()
            if debt['robot_id'] == robot_id
        ]
        owns_command_publisher = not retained_publishers
        command_publisher = None
        odom_subscriber = None
        avoidance = None
        lane_registered = False

        def discard_provisional_resources():
            if odom_subscriber is not None:
                try:
                    odom_subscriber.unregister()
                except Exception:
                    pass
            if avoidance is not None:
                try:
                    avoidance.shutdown()
                except Exception:
                    pass
            if command_publisher is not None and owns_command_publisher:
                if lane_registered:
                    self._discard_safety_fallback_lane(command_publisher)
                try:
                    command_publisher.unregister()
                except Exception:
                    pass

        try:
            if retained_publishers:
                # Do not let a fresh publisher with the same robot ID make an
                # old failed socket look healthy. The exact instance which
                # owes the zero remains the boundary when the robot returns.
                command_publisher = retained_publishers[0]
            else:
                command_publisher = rospy.Publisher(
                    f'/{robot_id}/cmd_vel', Twist, queue_size=1
                )
            lane_registered = self._register_safety_fallback_lane(
                robot_id, command_publisher
            )
            if not lane_registered:
                raise RuntimeError(
                    "the safety publication lane could not be reserved"
                )
            odom_subscriber = rospy.Subscriber(
                f'/{robot_id}/odom', Odometry,
                callback=self._odom_cb,
                callback_args=robot_id,
                queue_size=1,
            )
            linear_pid = PIDController(
                kp=0.6,
                ki=0.01,
                kd=0.15,
                max_output=self.max_linear_vel,
            )
            angular_pid = PIDController(
                kp=1.2,
                ki=0.0,
                kd=0.1,
                max_output=self.max_angular_vel,
            )
            avoidance = ObstacleAvoidance(robot_id)
            avoidance_limits_valid = self._align_avoidance_motion_limits(
                avoidance
            )
        except Exception:
            discard_provisional_resources()
            raise

        with self._stop_result_lock:
            if self._shutdown_started:
                installed = False
            else:
                if not hasattr(self, 'odom_received_at'):
                    self.odom_received_at = {}
                self.robot_poses[robot_id] = None
                self.robot_yaws[robot_id] = 0.0
                self.odom_received_at[robot_id] = None
                self.odom_confirmed_for_task[robot_id] = False
                self.cmd_vel_pubs[robot_id] = command_publisher
                self._command_publisher_generation_locked(
                    robot_id, command_publisher
                )
                self._odom_subs[robot_id] = odom_subscriber
                self.pid_linear[robot_id] = linear_pid
                self.pid_angular[robot_id] = angular_pid
                self.avoidance[robot_id] = avoidance
                invalid_limits = set(getattr(
                    self, 'invalid_avoidance_limits', ()
                ))
                if avoidance_limits_valid:
                    invalid_limits.discard(robot_id)
                else:
                    invalid_limits.add(robot_id)
                self.invalid_avoidance_limits = tuple(sorted(invalid_limits))
                self._refresh_shutdown_publisher_snapshot_locked()
                installed = True

        if not installed:
            discard_provisional_resources()
            rospy.logwarn(
                "Discarded robot %s because formation shutdown began "
                "during setup",
                robot_id,
            )
            return False

        if not avoidance_limits_valid:
            rospy.logerr(
                "Robot %s has invalid velocity limits; obstacle avoidance "
                "was locked to zero velocity.",
                robot_id,
            )
        rospy.loginfo("Added robot: %s", robot_id)
        return True

    @staticmethod
    def _bounded_motion_limit(value, hardware_limit) -> Tuple[float, bool]:
        """Return one finite, non-negative limit inside the hardware cap."""
        try:
            configured = float(value)
            hardware = float(hardware_limit)
        except (TypeError, ValueError, OverflowError):
            return 0.0, False
        if (
            not math.isfinite(configured)
            or not math.isfinite(hardware)
            or configured < 0.0
            or hardware < 0.0
        ):
            return 0.0, False
        return min(configured, hardware), True

    def _align_avoidance_motion_limits(self, avoidance) -> bool:
        """Intersect obstacle steering limits with the owner's safe envelope."""
        owner_linear, owner_linear_valid = self._bounded_motion_limit(
            getattr(self, 'max_linear_vel', None),
            self.BURGER_MAX_LINEAR_VELOCITY,
        )
        owner_angular, owner_angular_valid = self._bounded_motion_limit(
            getattr(self, 'max_angular_vel', None),
            self.BURGER_MAX_ANGULAR_VELOCITY,
        )
        avoidance_linear, avoidance_linear_valid = self._bounded_motion_limit(
            getattr(avoidance, 'max_linear_velocity', None),
            self.BURGER_MAX_LINEAR_VELOCITY,
        )
        avoidance_angular, avoidance_angular_valid = self._bounded_motion_limit(
            getattr(avoidance, 'max_angular_velocity', None),
            self.BURGER_MAX_ANGULAR_VELOCITY,
        )
        limits_valid = (
            getattr(self, '_motion_limits_valid', True)
            and owner_linear_valid
            and owner_angular_valid
            and avoidance_linear_valid
            and avoidance_angular_valid
        )
        if not limits_valid:
            # A bad value on either axis makes the whole differential-drive
            # command unsafe. Lock both axes instead of guessing a fallback.
            avoidance.max_linear_velocity = 0.0
            avoidance.max_angular_velocity = 0.0
            return False

        avoidance.max_linear_velocity = min(
            owner_linear, avoidance_linear
        )
        avoidance.max_angular_velocity = min(
            owner_angular, avoidance_angular
        )
        return True

    def _remove_robot(self, robot_id: str):
        """Unregister a robot."""
        pub = self.cmd_vel_pubs.pop(robot_id, None)
        if pub is not None:
            generation = self._command_publisher_generation_locked(
                robot_id, pub
            )
            debt_is_pending = generation in getattr(
                self, '_stop_publication_debts', {}
            )
            if not debt_is_pending:
                self._close_retired_command_publisher_locked(pub, generation)
            else:
                # The fleet-wide stop immediately preceding a resize already
                # attempted this publisher.  Retain a failed instance for the
                # next retry instead of unregistering it and losing the only
                # object that can settle its debt.
                rospy.logwarn(
                    "Retaining command publisher generation %d for %s until "
                    "its zero-velocity publication is accepted.",
                    generation,
                    robot_id,
                )

        sub = self._odom_subs.pop(robot_id, None)
        if sub is not None:
            sub.unregister()

        avoidance = self.avoidance.pop(robot_id, None)
        if avoidance is not None:
            avoidance.shutdown()

        for store in (
            self.robot_poses, self.robot_yaws,
            getattr(self, 'odom_received_at', {}),
            getattr(self, 'odom_confirmed_for_task', {}),
            self.pid_linear, self.pid_angular,
            self.assignments, self._slot_reached,
            self.route_waypoints, self.route_waypoint_indices,
        ):
            store.pop(robot_id, None)
        invalid_robots = set(getattr(self, 'invalid_robot_poses', ()))
        invalid_robots.discard(robot_id)
        self.invalid_robot_poses = tuple(sorted(invalid_robots))
        invalid_limits = set(getattr(
            self, 'invalid_avoidance_limits', ()
        ))
        invalid_limits.discard(robot_id)
        self.invalid_avoidance_limits = tuple(sorted(invalid_limits))
        self._refresh_shutdown_publisher_snapshot_locked()

        rospy.loginfo("Removed robot: %s", robot_id)

    # ------------------------------------------------------------------
    # Formation position computation
    # ------------------------------------------------------------------
    def _recompute_formation(self):
        """Recompute formation offsets and reassign robots."""
        with self.command_lock:
            if self._shutdown_started:
                return False
            assignment_snapshot = self._recompute_formation_locked()
        self._compute_and_commit_assignment(assignment_snapshot)

    def _recompute_formation_locked(self) -> Optional[Dict]:
        """Recompute offsets and snapshot an assignment under command_lock."""
        if self._shutdown_started:
            return None
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
        positions: List[Tuple[float, float]]

        # Check if it is a single uppercase letter
        if len(formation_type) == 1 and formation_type.upper() in LETTERS:
            positions = self._compute_letter_positions(
                formation_type.upper(), n_robots, spacing
            )
        elif ft == 'triangle':
            positions = self._triangle_positions(n_robots, spacing)
        elif ft == 'square':
            positions = self._square_positions(n_robots, spacing)
        elif ft == 'circle':
            positions = self._circle_positions(n_robots, spacing)
        elif ft == 'line':
            positions = self._line_positions(n_robots, spacing)
        elif ft in ('v_formation', 'v'):
            positions = self._v_formation_positions(n_robots, spacing)
        elif ft == 'diamond':
            positions = self._diamond_positions(n_robots, spacing)
        else:
            # Try as a letter if it is a known key
            key = formation_type.upper()
            if key in LETTERS:
                positions = self._compute_letter_positions(
                    key, n_robots, spacing
                )
            else:
                rospy.logwarn(
                    "Unknown formation type '%s', falling back to circle",
                    formation_type,
                )
                positions = self._circle_positions(n_robots, spacing)

        return ensure_minimum_spacing(positions, spacing)

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
            if self._shutdown_started:
                return False
            assignment_snapshot = self._prepare_assignment_locked()
        return self._compute_and_commit_assignment(assignment_snapshot)

    def _prepare_assignment_locked(self) -> Optional[Dict]:
        """
        Capture an immutable assignment request while holding command_lock.

        The Hungarian solver is deliberately invoked later, after command_lock
        has been released, so emergency-stop and lifecycle callbacks are never
        queued behind its O(n^3) computation.
        """
        if self._shutdown_started:
            return None
        self._assignment_generation += 1
        generation = self._assignment_generation

        n = min(len(self.robot_ids), len(self.formation_offsets))
        if n == 0:
            self.assignments = {}
            self.route_waypoints = {}
            self.route_waypoint_indices = {}
            self.route_batches = []
            self.route_batch_index = 0
            self.active_placement_plan = None
            self._live_orbit_validation_key = None
            self._live_orbit_validation_safe = False
            self.assignment_pending = False
            return None

        robot_ids = tuple(self.robot_ids[:n])
        robot_positions: List[Tuple[float, float]] = []
        robot_yaws: List[float] = []
        previous_slots: List[Optional[int]] = []

        with self.lock:
            poses = [self.robot_poses.get(robot_id) for robot_id in robot_ids]
            if any(pose is None for pose in poses):
                self.assignments = {}
                self.route_waypoints = {}
                self.route_waypoint_indices = {}
                self.route_batches = []
                self.route_batch_index = 0
                self.active_placement_plan = None
                self._live_orbit_validation_key = None
                self._live_orbit_validation_safe = False
                self.assignment_pending = True
                return None

            for robot_id, pose in zip(robot_ids, poses):
                robot_positions.append((pose.position.x, pose.position.y))
                robot_yaws.append(float(self.robot_yaws.get(robot_id, 0.0)))
                previous_slots.append(self.assignments.get(robot_id))
            model_poses = dict(self.model_poses)
            invalid_model_poses = tuple(getattr(
                self, 'invalid_model_poses', ()
            ))

        if (
            self.movement_mode != MovementMode.STATIC
            and not self._initial_formation_acquired
        ):
            # This also covers a fleet resize: the last first-odometry callback
            # rebuilds the assignment from the complete, current fleet.
            self._reset_centroid_path_pose_locked(robot_positions)

        placement_plan = None
        preferred_center = (
            sum(x for x, _ in robot_positions) / n,
            sum(y for _, y in robot_positions) / n,
        )
        if (
            self.movement_mode == MovementMode.STATIC
            or self.centroid_path != CentroidPath.CIRCULAR
        ):
            # Static shapes and open moving paths first need a safe place to
            # assemble. Route-aware placement is solved after releasing
            # command_lock because it may test several whole-pattern
            # translations and assignments. Linear and waypoint paths keep this
            # plan active so every later target is checked against the live
            # arena immediately before its command batch is published.
            self.centroid_heading = 0.0
            slot_clearance = min(0.34, self.spacing - 0.01)
            placement_plan = {
                'kind': (
                    'static'
                    if self.movement_mode == MovementMode.STATIC
                    else self.centroid_path.value
                ),
                'offsets': tuple(self.formation_offsets[:n]),
                'preferred_center': preferred_center,
                'arena_size': self.arena_size,
                'arena_margin': self.arena_margin,
                'obstacle_clearance': self.formation_obstacle_clearance,
                'route_clearance': (
                    self.formation_obstacle_clearance
                    + self.route_tracking_margin
                ),
                'search_step': self.formation_search_step,
                'search_limit': self.formation_search_limit,
                'exclusion_zones': tuple(self.spawn_exclusion_zones),
                'arena_profile': self.arena_profile,
                'model_poses': model_poses,
                'invalid_model_poses': invalid_model_poses,
                'slot_clearance': slot_clearance,
                'planned_slot_clearance': min(
                    self.spacing - 0.01,
                    slot_clearance + self.route_tracking_margin,
                ),
            }
        elif self.centroid_path == CentroidPath.CIRCULAR:
            # A circular task needs more than a safe first frame. The solver
            # below checks the complete rigid footprint around one full lap,
            # then plans every robot's route to that validated entry pose.
            slot_clearance = min(0.34, self.spacing - 0.01)
            placement_plan = {
                'kind': 'circular',
                'offsets': tuple(self.formation_offsets[:n]),
                'preferred_center': preferred_center,
                'requested_radius': self.path_radius,
                'minimum_radius': self.minimum_safe_path_radius,
                'sample_step': self.orbit_sample_step,
                'arena_size': self.arena_size,
                'arena_margin': self.arena_margin,
                'obstacle_clearance': self.formation_obstacle_clearance,
                'route_clearance': (
                    self.formation_obstacle_clearance
                    + self.route_tracking_margin
                ),
                'search_step': self.formation_search_step,
                'search_limit': self.formation_search_limit,
                'exclusion_zones': tuple(self.spawn_exclusion_zones),
                'arena_profile': self.arena_profile,
                'model_poses': model_poses,
                'invalid_model_poses': invalid_model_poses,
                'slot_clearance': slot_clearance,
                'planned_slot_clearance': min(
                    self.spacing - 0.01,
                    slot_clearance + self.route_tracking_margin,
                ),
            }

        self.placement_error = None
        if placement_plan is None:
            target_world = tuple(self._get_world_targets()[:n])
        else:
            preferred_x, preferred_y = placement_plan['preferred_center']
            target_world = tuple(
                (preferred_x + offset_x, preferred_y + offset_y)
                for offset_x, offset_y in placement_plan['offsets']
            )

        # Stop the control loop from using an assignment that belongs to the
        # previous centroid/shape while the new solution is in flight.
        self.assignments = {}
        self.route_waypoints = {}
        self.route_waypoint_indices = {}
        self.route_batches = []
        self.route_batch_index = 0
        # There is deliberately no active geometry while the replacement
        # assignment is being solved. The control loop has no assignments in
        # this state, so it can only publish zero commands until commit.
        self.active_placement_plan = None
        self._live_orbit_validation_key = None
        self._live_orbit_validation_safe = False
        self.assignment_pending = True
        return {
            'generation': generation,
            'robot_ids': robot_ids,
            'robot_positions': tuple(robot_positions),
            'robot_yaws': tuple(robot_yaws),
            'target_world': target_world,
            'previous_slots': tuple(previous_slots),
            'switch_penalty': (self.spacing * 0.35) ** 2,
            'placement_plan': placement_plan,
        }

    @staticmethod
    def _route_length(route: List[Tuple[float, float]]) -> float:
        return sum(
            math.hypot(end[0] - start[0], end[1] - start[1])
            for start, end in zip(route, route[1:])
        )

    @staticmethod
    def _orbit_radius_candidates(
        requested_radius: float, minimum_radius: float
    ) -> List[float]:
        """Return a short, deterministic set of conservative orbit radii."""

        requested = max(0.10, float(requested_radius))
        floor = min(requested, max(0.10, float(minimum_radius)))
        candidates = [requested]
        candidates.extend(
            radius for radius in (1.0, 0.75, 0.50, 0.40, 0.30, floor)
            if floor - 1e-9 <= radius < requested - 1e-9
        )
        unique = []
        for radius in candidates:
            if not any(abs(radius - item) <= 1e-9 for item in unique):
                unique.append(radius)
        return unique

    @staticmethod
    def _rigid_orbit_offsets(
        offsets: Tuple[Tuple[float, float], ...],
        radius: float,
        maximum_sample_step: float,
    ) -> Tuple[List[Tuple[float, float]], int]:
        """Sample one full lap while keeping every formation offset fixed."""

        samples = max(
            8,
            int(math.ceil(
                2.0 * math.pi * radius / maximum_sample_step
            )),
        )
        swept_offsets = []
        for sample_index in range(samples):
            phase = 2.0 * math.pi * sample_index / samples
            centroid_x = radius * math.cos(phase)
            centroid_y = radius * math.sin(phase)
            swept_offsets.extend(
                (centroid_x + offset_x, centroid_y + offset_y)
                for offset_x, offset_y in offsets
            )
        return swept_offsets, samples

    def _rigid_orbit_is_safe(
        self,
        offsets: Tuple[Tuple[float, float], ...],
        path_center: Tuple[float, float],
        radius: float,
        placement_plan: Dict,
        model_poses: Dict[str, Tuple[float, float, float]],
    ) -> bool:
        """Validate the continuous full-lap footprint against live geometry."""

        if not self._model_poses_are_finite(model_poses):
            return False
        sample_step = float(placement_plan['sample_step'])
        swept_offsets, _sample_count = self._rigid_orbit_offsets(
            offsets, radius, sample_step
        )
        swept_targets = [
            (path_center[0] + offset_x, path_center[1] + offset_y)
            for offset_x, offset_y in swept_offsets
        ]
        padding = sample_step / 2.0
        return formation_targets_are_safe(
            swept_targets,
            placement_plan['arena_size'],
            placement_plan['arena_margin'] + padding,
            placement_plan['obstacle_clearance'] + padding,
            placement_plan['exclusion_zones'],
            placement_plan['arena_profile'],
            model_poses,
        )

    def _routes_are_safe(
        self,
        robot_ids: Tuple[str, ...],
        robot_positions: Dict[str, Tuple[float, float]],
        targets: List[Tuple[float, float]],
        slots: List[int],
        routes: Dict[str, List[Tuple[float, float]]],
        placement_plan: Dict,
        model_poses: Dict[str, Tuple[float, float, float]],
        route_indices: Optional[Dict[str, int]] = None,
    ) -> bool:
        """Check every remaining entry segment and its reserved final slot."""

        if not self._model_poses_are_finite(model_poses):
            return False
        if len(robot_ids) != len(slots):
            return False
        if not formation_targets_are_safe(
            targets,
            placement_plan['arena_size'],
            placement_plan['arena_margin'],
            placement_plan['obstacle_clearance'],
            placement_plan['exclusion_zones'],
            placement_plan['arena_profile'],
            model_poses,
        ):
            return False

        indices = route_indices or {}
        for robot_id, slot_index in zip(robot_ids, slots):
            start = robot_positions.get(robot_id)
            if start is None or not (0 <= slot_index < len(targets)):
                return False
            route = list(routes.get(robot_id, []))
            route_index = max(0, int(indices.get(robot_id, 0)))
            remaining = route[route_index:]
            final_target = targets[slot_index]
            if (
                not remaining
                or math.hypot(
                    remaining[-1][0] - final_target[0],
                    remaining[-1][1] - final_target[1],
                ) > 1e-9
            ):
                remaining.append(final_target)

            # Slot reservations are a planning/staging constraint, not a live
            # obstacle. During execution, occupied slots are protected by the
            # staged batches and the normal inter-robot avoidance layer.
            route_zones = list(placement_plan['exclusion_zones'])
            if not formation_targets_are_safe(
                remaining,
                placement_plan['arena_size'],
                placement_plan['arena_margin'],
                placement_plan['obstacle_clearance'],
                route_zones,
                placement_plan['arena_profile'],
                model_poses,
            ):
                return False
            points = [start] + remaining
            if any(
                not straight_route_is_safe(
                    segment_start,
                    segment_end,
                    placement_plan['obstacle_clearance'],
                    route_zones,
                    placement_plan['arena_profile'],
                    model_poses,
                )
                for segment_start, segment_end in zip(points, points[1:])
            ):
                return False
        return True

    def _placement_plan_is_live_safe(
        self,
        placement_plan: Dict,
        robot_ids: Tuple[str, ...],
        robot_positions: Dict[str, Tuple[float, float]],
        targets: List[Tuple[float, float]],
        slots: List[int],
        routes: Dict[str, List[Tuple[float, float]]],
        model_poses: Dict[str, Tuple[float, float, float]],
        path_center: Optional[Tuple[float, float]] = None,
        path_radius: Optional[float] = None,
        route_indices: Optional[Dict[str, int]] = None,
    ) -> bool:
        """Revalidate a placement without trusting its planning snapshot."""

        if not self._model_poses_are_finite(model_poses):
            return False
        if placement_plan['kind'] == 'circular':
            if path_center is None or path_radius is None:
                return False
            if not self._rigid_orbit_is_safe(
                placement_plan['offsets'],
                path_center,
                path_radius,
                placement_plan,
                model_poses,
            ):
                return False
        return self._routes_are_safe(
            robot_ids,
            robot_positions,
            targets,
            slots,
            routes,
            placement_plan,
            model_poses,
            route_indices,
        )

    def _plan_safe_circular_placement(self, plan: Dict) -> Optional[Dict]:
        """Find a full rigid orbit using the current live obstacle snapshot."""

        requested_radius = float(plan['requested_radius'])
        sample_step = float(plan['sample_step'])
        # Signed distance is 1-Lipschitz. Reserving half a sample step at each
        # sampled pose covers the unsampled arc between neighbouring poses.
        validation_padding = sample_step / 2.0
        preferred_entry_x, preferred_entry_y = plan['preferred_center']

        for radius in self._orbit_radius_candidates(
            requested_radius, plan['minimum_radius']
        ):
            swept_offsets, sample_count = self._rigid_orbit_offsets(
                plan['offsets'], radius, sample_step
            )
            preferred_path_center = (
                preferred_entry_x - radius,
                preferred_entry_y,
            )
            path_center = find_safe_formation_center(
                swept_offsets,
                preferred_path_center,
                plan['arena_size'],
                plan['arena_margin'] + validation_padding,
                plan['obstacle_clearance'] + validation_padding,
                plan['search_step'],
                plan['exclusion_zones'],
                plan['arena_profile'],
                plan['model_poses'],
            )
            if path_center is None:
                continue

            if not self._rigid_orbit_is_safe(
                plan['offsets'],
                path_center,
                radius,
                plan,
                plan['model_poses'],
            ):
                continue

            entry_center = (
                path_center[0] + radius,
                path_center[1],
            )
            targets = [
                (entry_center[0] + offset_x,
                 entry_center[1] + offset_y)
                for offset_x, offset_y in plan['offsets']
            ]
            if not formation_targets_are_safe(
                targets,
                plan['arena_size'],
                plan['arena_margin'],
                plan['obstacle_clearance'],
                plan['exclusion_zones'],
                plan['arena_profile'],
                plan['model_poses'],
            ):
                continue
            return {
                'path_center': path_center,
                'entry_center': entry_center,
                'radius': radius,
                'targets': targets,
                'sample_count': sample_count,
                'live_model_count': len(plan['model_poses']),
            }
        return None

    def _plan_routes_to_targets(
        self,
        assignment_snapshot: Dict,
        placement_plan: Dict,
        targets: List[Tuple[float, float]],
    ) -> Optional[Tuple[
        List[int], Dict[str, List[Tuple[float, float]]]
    ]]:
        """Choose slots by actual route length, then reserve parked slots."""

        route_options = []
        route_costs = []
        largest_cost = 0.0
        for robot_index, start in enumerate(
            assignment_snapshot['robot_positions']
        ):
            previous_slot = assignment_snapshot['previous_slots'][robot_index]
            robot_routes = []
            robot_costs = []
            for slot_index, target in enumerate(targets):
                route = plan_obstacle_aware_route(
                    start,
                    target,
                    placement_plan['arena_size'],
                    placement_plan['arena_margin'],
                    placement_plan['route_clearance'],
                    placement_plan['exclusion_zones'],
                    placement_plan['arena_profile'],
                    placement_plan['model_poses'],
                    circle_samples=8,
                )
                robot_routes.append(route)
                if route is None:
                    robot_costs.append(float('inf'))
                    continue
                cost = self._route_length(route) ** 2
                if previous_slot is not None and previous_slot != slot_index:
                    cost += assignment_snapshot['switch_penalty']
                robot_costs.append(cost)
                largest_cost = max(largest_cost, cost)
            route_options.append(robot_routes)
            route_costs.append(robot_costs)

        if any(not any(math.isfinite(cost) for cost in row)
               for row in route_costs):
            return None
        if any(not any(math.isfinite(row[column]) for row in route_costs)
               for column in range(len(targets))):
            return None

        blocked_cost = (largest_cost + 1.0) * (len(targets) + 1)
        slots = hungarian_assignment([
            [cost if math.isfinite(cost) else blocked_cost for cost in row]
            for row in route_costs
        ])
        if any(
            slot_index < 0
            or route_options[robot_index][slot_index] is None
            for robot_index, slot_index in enumerate(slots)
        ):
            return None

        slot_zones = [
            {
                'name': 'formation_slot_{}'.format(index),
                'shape': 'circle',
                'x': target[0],
                'y': target[1],
                'radius': 0.0,
                'clearance': placement_plan['planned_slot_clearance'],
            }
            for index, target in enumerate(targets)
        ]
        routes = {}
        for robot_index, (robot_id, slot_index) in enumerate(zip(
            assignment_snapshot['robot_ids'], slots
        )):
            route_zones = list(placement_plan['exclusion_zones'])
            route_zones.extend(
                zone for index, zone in enumerate(slot_zones)
                if index != slot_index
            )
            route = plan_obstacle_aware_route(
                assignment_snapshot['robot_positions'][robot_index],
                targets[slot_index],
                placement_plan['arena_size'],
                placement_plan['arena_margin'],
                placement_plan['route_clearance'],
                route_zones,
                placement_plan['arena_profile'],
                placement_plan['model_poses'],
                circle_samples=8,
            )
            if route is None:
                return None
            routes[robot_id] = route
        return slots, routes

    def _plan_routed_static_assignment(
        self, assignment_snapshot: Dict, static_plan: Dict
    ) -> Optional[Tuple[
        Tuple[float, float],
        List[Tuple[float, float]],
        List[int],
        Dict[str, List[Tuple[float, float]]],
    ]]:
        """Choose slots by actual route length, then reserve parked slots."""

        center = find_safe_formation_center(
            static_plan['offsets'],
            static_plan['preferred_center'],
            static_plan['arena_size'],
            static_plan['arena_margin'],
            static_plan['obstacle_clearance'],
            static_plan['search_step'],
            static_plan['exclusion_zones'],
            static_plan['arena_profile'],
            static_plan['model_poses'],
        )
        if center is None:
            return None

        targets = [
            (center[0] + offset_x, center[1] + offset_y)
            for offset_x, offset_y in static_plan['offsets']
        ]
        routed = self._plan_routes_to_targets(
            assignment_snapshot, static_plan, targets
        )
        if routed is None:
            return None
        slots, routes = routed
        return center, targets, slots, routes

    def _build_route_batches(
        self,
        robot_ids: Tuple[str, ...],
        robot_positions: Tuple[Tuple[float, float], ...],
        routes: Dict[str, List[Tuple[float, float]]],
        clearance: float,
    ) -> Optional[List[List[str]]]:
        """Stage mutually clear routes and move start-position blockers first."""

        starts = dict(zip(robot_ids, robot_positions))
        dependencies = {robot_id: set() for robot_id in robot_ids}
        for robot_id in robot_ids:
            route = routes[robot_id]
            for other_id in robot_ids:
                if other_id == robot_id:
                    continue
                if point_to_route_distance(starts[other_id], route) < clearance:
                    dependencies[robot_id].add(other_id)

        remaining = set(robot_ids)
        batches = []
        while remaining:
            ready = [
                robot_id for robot_id in sort_robot_ids(remaining)
                if not (dependencies[robot_id] & remaining)
            ]
            if not ready:
                return None

            batch = []
            for robot_id in ready:
                if any(
                    routes_conflict(
                        routes[robot_id], routes[other_id], clearance
                    )
                    for other_id in batch
                ):
                    continue
                batch.append(robot_id)

            batches.append(batch)
            remaining.difference_update(batch)
        return batches

    def _compute_and_commit_assignment(
        self, assignment_snapshot: Optional[Dict]
    ) -> bool:
        """
        Solve an assignment without command_lock and commit only if current.

        A newer shape/fleet/start request, stop, or emergency stop advances the
        generation and makes this result stale before it can affect control.
        """
        if self._shutdown_started or assignment_snapshot is None:
            return False

        placement_plan = assignment_snapshot.get('placement_plan')
        if (
            placement_plan is not None
            and (
                placement_plan.get('invalid_model_poses')
                or not self._model_poses_are_finite(
                    placement_plan.get('model_poses', {})
                )
            )
        ):
            return self._reject_assignment_plan(
                assignment_snapshot,
                "The live obstacle state contained a non-finite pose; no "
                "robot was released.",
                'live geometry',
            )
        planned_center = None
        planned_orbit = None
        planned_routes = {}
        if placement_plan is None:
            slots = minimum_distance_assignment(
                assignment_snapshot['robot_positions'],
                assignment_snapshot['target_world'],
                previous_slots=assignment_snapshot['previous_slots'],
                switch_penalty=assignment_snapshot['switch_penalty'],
            )
            route_batches = []
        else:
            if placement_plan['kind'] != 'circular':
                plan = self._plan_routed_static_assignment(
                    assignment_snapshot, placement_plan
                )
                if plan is not None:
                    planned_center, planned_targets, slots, full_routes = plan
            else:
                planned_orbit = self._plan_safe_circular_placement(
                    placement_plan
                )
                plan = None
                if planned_orbit is not None:
                    planned_center = planned_orbit['entry_center']
                    planned_targets = planned_orbit['targets']
                    routed = self._plan_routes_to_targets(
                        assignment_snapshot,
                        placement_plan,
                        planned_targets,
                    )
                    if routed is not None:
                        slots, full_routes = routed
                        plan = True

            if plan is None:
                if placement_plan['kind'] == 'circular':
                    error = (
                        "No collision-free full orbit and entry-route "
                        "assignment fits this formation and spacing inside "
                        "the live arena."
                    )
                else:
                    error = (
                        "No collision-free placement and route assignment "
                        "fits this formation and spacing inside the arena."
                    )
                return self._reject_assignment_plan(
                    assignment_snapshot, error, 'placement'
                )
            assignment_snapshot['target_world'] = tuple(planned_targets)
            route_batches = self._build_route_batches(
                assignment_snapshot['robot_ids'],
                assignment_snapshot['robot_positions'],
                full_routes,
                placement_plan['slot_clearance'],
            )
            if route_batches is None:
                return self._reject_assignment_plan(
                    assignment_snapshot,
                    "No collision-free staged route fits this formation "
                    "and spacing inside the arena.",
                    'route staging',
                )
            planned_routes = {
                robot_id: route[1:]
                for robot_id, route in full_routes.items()
            }

        with self.command_lock:
            if self._shutdown_started:
                return False
            if (
                getattr(self, '_assignment_worker_stop', None) is not None
                and self._assignment_worker_stop.is_set()
            ):
                return False
            if (
                assignment_snapshot['generation']
                != self._assignment_generation
            ):
                return False
            if placement_plan is not None:
                with self.lock:
                    live_model_poses = dict(self.model_poses)
                    invalid_live_model_poses = tuple(getattr(
                        self, 'invalid_model_poses', ()
                    ))
                    live_robot_positions = {
                        robot_id: (
                            self.robot_poses[robot_id].position.x,
                            self.robot_poses[robot_id].position.y,
                        )
                        for robot_id in assignment_snapshot['robot_ids']
                        if self.robot_poses.get(robot_id) is not None
                    }
                    live_robot_yaws = {
                        robot_id: self.robot_yaws.get(robot_id)
                        for robot_id in assignment_snapshot['robot_ids']
                    }
                if self._shutdown_started:
                    return False
                path_center = None
                path_radius = None
                if planned_orbit is not None:
                    path_center = planned_orbit['path_center']
                    path_radius = planned_orbit['radius']
                pose_drift = self._assignment_pose_drift(
                    assignment_snapshot,
                    live_robot_positions,
                    live_robot_yaws,
                )
                live_plan_is_safe = (
                    pose_drift is None
                    and not invalid_live_model_poses
                    and self._placement_plan_is_live_safe(
                        placement_plan,
                        assignment_snapshot['robot_ids'],
                        live_robot_positions,
                        list(assignment_snapshot['target_world']),
                        list(slots),
                        full_routes,
                        live_model_poses,
                        path_center,
                        path_radius,
                        {
                            robot_id: 1
                            for robot_id in assignment_snapshot['robot_ids']
                        },
                    )
                )
                if not live_plan_is_safe:
                    detail = pose_drift or {
                        'gate': 'assignment_live_revalidation'
                    }
                    detail['stage'] = 'assignment_commit'
                    self._record_live_safety_failure(detail)
                    if (
                        not invalid_live_model_poses
                        and self._schedule_live_replan_locked(
                            placement_plan,
                            live_robot_positions,
                            live_model_poses,
                            detail,
                        )
                    ):
                        return False
                    return self._reject_assignment_plan(
                        assignment_snapshot,
                        "The live arena changed while the formation plan "
                        "was being computed; no robot was released.",
                        'live revalidation',
                    )
            if planned_center is not None:
                self.centroid_x, self.centroid_y = planned_center
                self.centroid_heading = 0.0
            if planned_orbit is not None:
                self.path_center_x, self.path_center_y = (
                    planned_orbit['path_center']
                )
                self.effective_path_radius = planned_orbit['radius']
                self.orbit_path_validated = True
                self.orbit_validation_samples = planned_orbit[
                    'sample_count'
                ]
                self.orbit_validation_live_models = planned_orbit[
                    'live_model_count'
                ]
                self.orbit_radius_adapted = (
                    abs(self.effective_path_radius - self.path_radius) > 1e-9
                )
                self.centroid_time = 0.0
                self._set_circular_centroid_pose()
            self.placement_error = None
            self.assignments = {
                robot_id: slot_index
                for robot_id, slot_index in zip(
                    assignment_snapshot['robot_ids'], slots
                )
            }
            self.route_waypoints = planned_routes
            self.route_waypoint_indices = {
                robot_id: 0 for robot_id in planned_routes
            }
            self.route_batches = route_batches
            self.route_batch_index = 0
            self.active_placement_plan = (
                None if placement_plan is None else dict(placement_plan)
            )
            self._live_orbit_validation_key = None
            self._live_orbit_validation_safe = False
            self.assignment_pending = False
            self._slot_reached = {
                robot_id: False
                for robot_id in assignment_snapshot['robot_ids']
            }
            self._settled_duration = 0.0
        return True

    def _reject_assignment_plan(
        self, assignment_snapshot: Dict, error: str, stage: str
    ) -> bool:
        """Discard one current unsafe plan without reviving stale work."""

        with self.command_lock:
            if self._shutdown_started:
                return False
            if (
                assignment_snapshot['generation']
                != self._assignment_generation
            ):
                return False
            self.assignments = {}
            self.route_waypoints = {}
            self.route_waypoint_indices = {}
            self.route_batches = []
            self.route_batch_index = 0
            self.active_placement_plan = None
            self._live_orbit_validation_key = None
            self._live_orbit_validation_safe = False
            self.assignment_pending = False
            self.orbit_path_validated = False
            self.placement_error = error
            self.formation_state = FormationState.FAILED
        rospy.logerr(
            "Formation %s failed: type=%s robots=%d spacing=%.2f",
            stage,
            self.formation_type,
            len(assignment_snapshot['robot_ids']),
            self.spacing,
        )
        return False

    def _cancel_pending_assignment_locked(
        self, clear_assignments: bool = False
    ):
        """Invalidate any solver result that was computed from older state."""
        self._assignment_generation += 1
        self.assignment_pending = False
        if clear_assignments:
            self.assignments = {}
            self.route_waypoints = {}
            self.route_waypoint_indices = {}
            self.route_batches = []
            self.route_batch_index = 0
            self.active_placement_plan = None
            self._live_orbit_validation_key = None
            self._live_orbit_validation_safe = False
            self._slot_reached = {}
        self._settled_duration = 0.0

    def _model_geometry_matches_plan(self, plan, live_model_poses) -> bool:
        """Distinguish robot settling from a genuinely moving obstacle."""
        planned_model_poses = plan.get('model_poses', {})
        if set(planned_model_poses) != set(live_model_poses):
            return False
        try:
            for model_name, planned_pose in planned_model_poses.items():
                live_pose = live_model_poses[model_name]
                if (
                    abs(float(live_pose[0]) - float(planned_pose[0])) > 0.005
                    or abs(float(live_pose[1]) - float(planned_pose[1])) > 0.005
                    or abs(normalize_angle(
                        float(live_pose[2]) - float(planned_pose[2])
                    )) > 0.005
                ):
                    return False
        except (IndexError, TypeError, ValueError, OverflowError):
            return False
        return True

    def _assignment_pose_drift(
        self, assignment_snapshot, live_robot_positions, live_robot_yaws
    ):
        """Describe material odometry movement since a solve was started."""
        robot_ids = assignment_snapshot.get('robot_ids', ())
        positions = assignment_snapshot.get('robot_positions', ())
        yaws = assignment_snapshot.get('robot_yaws', ())
        if (
            len(robot_ids) != len(positions)
            or len(robot_ids) != len(yaws)
            or set(live_robot_positions) != set(robot_ids)
            or set(live_robot_yaws) != set(robot_ids)
        ):
            return {'gate': 'assignment_pose_snapshot', 'kind': 'incomplete'}
        try:
            for robot_id, old_position, old_yaw in zip(
                robot_ids, positions, yaws
            ):
                live_position = live_robot_positions[robot_id]
                position_drift = math.hypot(
                    float(live_position[0]) - float(old_position[0]),
                    float(live_position[1]) - float(old_position[1]),
                )
                yaw_drift = abs(normalize_angle(
                    float(live_robot_yaws[robot_id]) - float(old_yaw)
                ))
                if not math.isfinite(position_drift + yaw_drift):
                    raise ValueError('non-finite pose drift')
                if (
                    position_drift > self.assignment_pose_drift_tolerance
                    or yaw_drift > self.assignment_yaw_drift_tolerance
                ):
                    return {
                        'gate': 'assignment_pose_snapshot',
                        'kind': 'drift',
                        'robot': robot_id,
                        'position_drift': round(position_drift, 6),
                        'yaw_drift': round(yaw_drift, 6),
                    }
        except (IndexError, TypeError, ValueError, OverflowError):
            return {'gate': 'assignment_pose_snapshot', 'kind': 'invalid'}
        return None

    def _live_poses_can_be_replanned(
        self, plan, robot_positions, live_model_poses
    ) -> bool:
        """Require finite, contact-free starts in unchanged live geometry."""
        if (
            not self.is_running
            or self._live_replan_attempts >= self.live_replan_limit
            or not self._model_geometry_matches_plan(plan, live_model_poses)
            or set(robot_positions) != set(self.robot_ids)
        ):
            return False
        usable_half = plan['arena_size'] / 2.0 - self.BURGER_RADIUS
        for robot_id in self.robot_ids:
            try:
                position = tuple(float(value) for value in robot_positions[
                    robot_id
                ])
            except (TypeError, ValueError, OverflowError):
                return False
            if (
                len(position) != 2
                or not all(math.isfinite(value) for value in position)
                or abs(position[0]) > usable_half
                or abs(position[1]) > usable_half
            ):
                return False
            for zone in plan['exclusion_zones']:
                if not self._zone_is_active_for_plan(
                    zone, plan['arena_profile']
                ):
                    continue
                try:
                    clearance = signed_distance_to_zone(
                        position, zone, live_model_poses
                    )
                except (TypeError, ValueError, OverflowError):
                    return False
                if (
                    not math.isfinite(clearance)
                    or clearance <= self.BURGER_RADIUS + 1e-6
                ):
                    return False
        return True

    def _schedule_live_replan_locked(
        self, plan, robot_positions, live_model_poses, detail
    ) -> bool:
        """Queue a bounded replacement plan from settled live robot poses."""
        if not self._live_poses_can_be_replanned(
            plan, robot_positions, live_model_poses
        ):
            return False
        publication = self._stop_all_robots('live_replan')
        if not publication['publication_confirmed']:
            self._publish_stop_status_safely('live replan')
            return False
        self._live_replan_attempts += 1
        self._cancel_pending_assignment_locked(clear_assignments=True)
        self.assignment_pending = True
        self.orbit_path_validated = False
        self.placement_error = None
        self.formation_state = FormationState.FORMING
        self._initial_formation_acquired = False
        self._request_pending_assignment()
        try:
            rospy.logwarn(
                "Formation route changed while planning; retrying from live "
                "poses (%d/%d, gate=%s, robot=%s).",
                self._live_replan_attempts,
                self.live_replan_limit,
                detail.get('gate', 'unknown'),
                detail.get('robot', 'unknown'),
            )
        except Exception:
            pass
        return True

    @staticmethod
    def _model_poses_are_finite(
        model_poses: Dict[str, Tuple[float, float, float]]
    ) -> bool:
        """Reject corrupt Gazebo geometry instead of using a fallback pose."""

        try:
            return all(
                len(pose) >= 3
                and all(
                    math.isfinite(float(value)) for value in pose[:3]
                )
                for pose in model_poses.values()
            )
        except (TypeError, ValueError, OverflowError):
            return False

    @staticmethod
    def _planar_robot_pose_is_finite(
        pose: Optional[Pose], yaw: Optional[float]
    ) -> bool:
        """Check the planar odometry values used by the controller."""

        if pose is None:
            return False
        try:
            values = (pose.position.x, pose.position.y, yaw)
            return all(math.isfinite(float(value)) for value in values)
        except (AttributeError, TypeError, ValueError, OverflowError):
            return False

    def _live_robot_positions_locked(
        self,
    ) -> Optional[Dict[str, Tuple[float, float]]]:
        """Return a finite fleet snapshot, or None for rejected odometry."""

        invalid_robots = set(getattr(self, 'invalid_robot_poses', ()))
        robot_positions = {}
        for robot_id in self.robot_ids:
            pose = self.robot_poses.get(robot_id)
            yaw = self.robot_yaws.get(robot_id)
            if (
                robot_id in invalid_robots
                or not self._planar_robot_pose_is_finite(pose, yaw)
            ):
                return None
            robot_positions[robot_id] = (
                pose.position.x,
                pose.position.y,
            )
        return robot_positions

    def _live_motion_is_safe_locked(
        self,
        world_targets: List[Tuple[float, float]],
        assembling_on_routes: bool,
    ) -> bool:
        """Check the active plan immediately before publishing its batch."""

        robot_ids = tuple(self.robot_ids)
        robot_positions = self._live_robot_positions_locked()
        if robot_positions is None:
            self._record_live_safety_failure({'gate': 'robot_pose'})
            return False

        model_poses = dict(self.model_poses)
        if (
            getattr(self, 'invalid_model_poses', ())
            or not self._model_poses_are_finite(model_poses)
        ):
            self._record_live_safety_failure({
                'gate': 'model_pose',
                'invalid_models': list(getattr(
                    self, 'invalid_model_poses', ()
                )),
            })
            return False

        plan = getattr(self, 'active_placement_plan', None)
        if plan is None:
            # The brief assignment-planning window has neither assignments nor
            # routes and may safely emit its all-zero hold batch. Any assigned
            # fleet without a plan has lost its geometric safety contract.
            safe_without_plan = (
                not self.assignments and not self.route_batches
            )
            if not safe_without_plan:
                self._record_live_safety_failure({'gate': 'missing_plan'})
            return safe_without_plan

        if (
            plan['kind'] == 'circular'
            and not self._cached_live_orbit_is_safe_locked(
                plan, model_poses
            )
        ):
            self._record_live_safety_failure({'gate': 'orbit'})
            return False
        if not formation_targets_are_safe(
            world_targets,
            plan['arena_size'],
            plan['arena_margin'],
            plan['obstacle_clearance'],
            plan['exclusion_zones'],
            plan['arena_profile'],
            model_poses,
        ):
            self._record_live_safety_failure({'gate': 'targets'})
            return False
        if not assembling_on_routes:
            return True

        slots = [self.assignments.get(robot_id) for robot_id in robot_ids]
        if any(slot is None for slot in slots):
            self._record_live_safety_failure({
                'gate': 'assignments',
                'missing': [
                    robot_id for robot_id, slot in zip(robot_ids, slots)
                    if slot is None
                ],
            })
            return False
        routes_safe = self._routes_are_safe(
            robot_ids,
            robot_positions,
            world_targets,
            slots,
            self.route_waypoints,
            plan,
            model_poses,
            self.route_waypoint_indices,
        )
        if not routes_safe:
            self._record_live_safety_failure({'gate': 'route'})
        return routes_safe

    def _record_live_safety_failure(self, detail: Dict):
        """Keep one structured explanation for a fail-closed geometry gate."""
        self._last_live_safety_failure = dict(detail)
        try:
            rospy.logerr(
                "Formation live safety gate rejected: %s",
                json.dumps(detail, sort_keys=True),
            )
        except Exception:
            pass

    @staticmethod
    def _zone_is_active_for_plan(zone: Dict, arena_profile: str) -> bool:
        if not isinstance(zone, dict):
            return False
        worlds = zone.get('worlds')
        if isinstance(worlds, str):
            worlds = [worlds]
        return (
            (not worlds or arena_profile in worlds)
            and zone.get('shape') in ('box', 'circle')
        )

    def _validated_motion_command(self, command: Twist) -> Optional[Twist]:
        """Return a detached safe copy, or None for an invalid Twist."""

        if not getattr(self, '_motion_limits_valid', True):
            return None
        try:
            configured_linear_limit = float(self.max_linear_vel)
            configured_angular_limit = float(self.max_angular_vel)
            linear = (
                float(command.linear.x),
                float(command.linear.y),
                float(command.linear.z),
            )
            angular = (
                float(command.angular.x),
                float(command.angular.y),
                float(command.angular.z),
            )
        except (AttributeError, TypeError, ValueError, OverflowError):
            return None

        if (
            not math.isfinite(configured_linear_limit)
            or not math.isfinite(configured_angular_limit)
            or configured_linear_limit < 0.0
            or configured_angular_limit < 0.0
            or not all(math.isfinite(value) for value in linear + angular)
        ):
            return None

        # A bad ROS parameter must not enlarge the TurtleBot3 Burger envelope.
        linear_limit = min(
            configured_linear_limit, self.BURGER_MAX_LINEAR_VELOCITY
        )
        angular_limit = min(
            configured_angular_limit, self.BURGER_MAX_ANGULAR_VELOCITY
        )
        epsilon = 1e-9
        if (
            any(abs(value) > epsilon for value in linear[1:] + angular[:2])
            or any(abs(value) > linear_limit + epsilon for value in linear)
            or any(abs(value) > angular_limit + epsilon for value in angular)
        ):
            return None

        validated = Twist()
        (
            validated.linear.x,
            validated.linear.y,
            validated.linear.z,
        ) = linear
        (
            validated.angular.x,
            validated.angular.y,
            validated.angular.z,
        ) = angular
        return validated

    def _publish_motion_batch_locked(self, commands) -> bool:
        """Publish one safe batch and compensate a late safety-stop edge."""
        for robot_id, command in commands.items():
            publisher = self.cmd_vel_pubs.get(robot_id)
            if publisher is None:
                continue
            requests_motion = (
                float(command.linear.x) != 0.0
                or float(command.angular.z) != 0.0
            )
            generation = None
            with self._stop_result_lock:
                if self._shutdown_started or self.emergency_stop_active:
                    return False
                if requests_motion:
                    generation = self._command_publisher_generation_locked(
                        robot_id, publisher
                    )
                    self._motion_publish_inflight.add(generation)

            try:
                publisher.publish(command)
            except Exception:
                if generation is not None:
                    with self._stop_result_lock:
                        safety_stop_started = (
                            self._shutdown_started
                            or self.emergency_stop_active
                        )
                        if not safety_stop_started:
                            self._motion_publish_inflight.discard(generation)
                    if safety_stop_started:
                        self._compensate_late_motion(
                            robot_id, publisher, generation
                        )
                raise

            with self._stop_result_lock:
                safety_stop_started = (
                    self._shutdown_started or self.emergency_stop_active
                )
                if generation is not None and not safety_stop_started:
                    self._motion_publish_inflight.discard(generation)
            if safety_stop_started:
                if generation is not None:
                    self._compensate_late_motion(
                        robot_id, publisher, generation
                    )
                return False
        return True

    def _compensate_late_motion(
        self, robot_id, publisher, generation
    ) -> bool:
        """Start redundant re-zero attempts without delaying shutdown."""
        for attempt in (1, 2):
            worker = threading.Thread(
                target=self._late_motion_zero_attempt,
                args=(robot_id, publisher, generation, attempt),
                name='formation-late-zero-{}-{}'.format(
                    generation, attempt
                ),
                daemon=True,
            )
            try:
                worker.start()
            except Exception as exc:
                try:
                    rospy.logerr(
                        "Late-motion zero worker %d could not start for %s: "
                        "%s",
                        attempt,
                        robot_id,
                        exc,
                    )
                except Exception:
                    pass

        # A worker can start successfully and still fail inside publish().
        # Always queue independent attempts on the lane reserved when this
        # publisher was created; thread-start success alone is not a receipt.
        for attempt in (3, 4):
            self._submit_safety_fallback(
                publisher,
                lambda current_attempt=attempt: (
                    self._late_motion_zero_attempt(
                        robot_id,
                        publisher,
                        generation,
                        current_attempt,
                    )
                ),
            )
        return False

    def _late_motion_zero_attempt(
        self, robot_id, publisher, generation, attempt
    ):
        """Run one detached zero attempt for a late motion publication."""
        try:
            publisher.publish(Twist())
        except Exception as exc:
            try:
                rospy.logerr(
                    "Detached late-motion zero attempt %d failed for %s: %s",
                    attempt,
                    robot_id,
                    exc,
                )
            except Exception:
                pass
            return
        with self._stop_result_lock:
            self._motion_publish_inflight.discard(generation)

    def _cached_live_orbit_is_safe_locked(
        self,
        placement_plan: Dict,
        model_poses: Dict[str, Tuple[float, float, float]],
    ) -> bool:
        """Reuse full-lap validation until obstacle motion is material."""

        if not self._model_poses_are_finite(model_poses):
            return False
        pose_key = tuple(sorted(
            (
                model_name,
                int(round(float(pose[0]) / 0.005)),
                int(round(float(pose[1]) / 0.005)),
                int(round(float(pose[2]) / 0.005)),
            )
            for model_name, pose in model_poses.items()
        ))
        key = (
            round(float(self.path_center_x), 6),
            round(float(self.path_center_y), 6),
            round(float(self.effective_path_radius), 6),
            tuple(self.formation_offsets),
            pose_key,
        )
        if key != getattr(self, '_live_orbit_validation_key', None):
            self._live_orbit_validation_safe = self._rigid_orbit_is_safe(
                tuple(self.formation_offsets),
                (self.path_center_x, self.path_center_y),
                self.effective_path_radius,
                placement_plan,
                model_poses,
            )
            self._live_orbit_validation_key = key
        return bool(self._live_orbit_validation_safe)

    def _can_advance_route_waypoint(
        self,
        robot_id: str,
        robot_position: Tuple[float, float],
        slot_index: int,
        world_targets: List[Tuple[float, float]],
        waypoints: List[Tuple[float, float]],
        next_index: int,
    ) -> bool:
        """Only cut a planned corner when the remaining live route is safe."""

        plan = getattr(self, 'active_placement_plan', None)
        if plan is None:
            return True
        with self.lock:
            return self._routes_are_safe(
                (robot_id,),
                {robot_id: robot_position},
                world_targets,
                [slot_index],
                {robot_id: waypoints},
                plan,
                dict(self.model_poses),
                {robot_id: next_index},
            )

    def _get_world_targets(self) -> List[Tuple[float, float]]:
        """
        Return world positions of every rigid formation slot.

        ``centroid_heading`` follows the path tangent for telemetry and RViz.
        Slot offsets use their own fixed heading so letters remain readable
        instead of spinning once per lap.
        """
        return self._world_targets_at(self.centroid_x, self.centroid_y)

    def _world_targets_at(
        self, centroid_x: float, centroid_y: float
    ) -> List[Tuple[float, float]]:
        """Translate the fixed formation footprint to one centroid pose."""

        formation_heading = getattr(self, 'formation_heading', 0.0)
        cos_h = math.cos(formation_heading)
        sin_h = math.sin(formation_heading)
        targets: List[Tuple[float, float]] = []
        for (ox, oy) in self.formation_offsets:
            rx = ox * cos_h - oy * sin_h
            ry = ox * sin_h + oy * cos_h
            targets.append((centroid_x + rx, centroid_y + ry))
        return targets

    # ------------------------------------------------------------------
    # Centroid update
    # ------------------------------------------------------------------
    def _reset_centroid_path_pose_locked(self, robot_positions=None):
        """Anchor the configured path at the fleet's current centroid."""
        if robot_positions is None:
            with self.lock:
                robot_positions = [
                    (pose.position.x, pose.position.y)
                    for robot_id in self.robot_ids
                    for pose in (self.robot_poses.get(robot_id),)
                    if pose is not None
                ]

        if robot_positions:
            start_x = sum(x for x, _ in robot_positions) / len(robot_positions)
            start_y = sum(y for _, y in robot_positions) / len(robot_positions)
        else:
            start_x = self.centroid_x
            start_y = self.centroid_y

        self.centroid_time = 0.0
        self.current_waypoint_idx = 0
        self.centroid_x = start_x
        self.centroid_y = start_y
        self.centroid_heading = 0.0
        self.path_center_x = start_x
        self.path_center_y = start_y
        self.effective_path_radius = self.path_radius
        self.orbit_path_validated = False
        self.orbit_validation_samples = 0
        self.orbit_validation_live_models = 0
        self.orbit_radius_adapted = False
        self._live_orbit_validation_key = None
        self._live_orbit_validation_safe = False
        self._maximum_position_error = 0.0

        if self.movement_mode == MovementMode.STATIC:
            return

        if (
            self.movement_mode != MovementMode.STATIC
            and self.centroid_path == CentroidPath.CIRCULAR
        ):
            # This provisional pose only exists until the full-lap solver has
            # selected a safe path center and effective radius.
            self.path_center_x = start_x - self.effective_path_radius
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
        radius = max(
            0.1, getattr(self, 'effective_path_radius', self.path_radius)
        )
        angular_speed = self.centroid_speed / radius
        t = self.centroid_time * angular_speed
        self.centroid_x = self.path_center_x + radius * math.cos(t)
        self.centroid_y = self.path_center_y + radius * math.sin(t)
        self.centroid_heading = t + math.pi / 2.0

    def _update_centroid(self, dt: float):
        """Advance the centroid along the configured path."""
        if (
            self.movement_mode == MovementMode.STATIC
            or not self._initial_formation_acquired
        ):
            # Centroid stays at the average of initial robot positions
            return

        # Slow the path before tracking error grows large enough to break the
        # formation. This lets small swarms use the requested pace while larger
        # letters automatically move at the pace their outer slots can hold.
        slowdown_error = max(0.02, self.position_tolerance - 0.04)
        stop_error = max(
            slowdown_error + 0.01,
            min(
                self.position_release_tolerance - 0.02,
                self.position_tolerance + 0.01,
            ),
        )
        tracking_error = self._maximum_position_error
        if not math.isfinite(tracking_error) or tracking_error >= stop_error:
            return
        if tracking_error > slowdown_error:
            dt *= (stop_error - tracking_error) / (
                stop_error - slowdown_error
            )

        self.centroid_time += dt

        if self.centroid_path == CentroidPath.CIRCULAR:
            if not getattr(self, 'orbit_path_validated', False):
                self.centroid_time -= dt
                self.placement_error = (
                    "The circular formation path was not validated before "
                    "movement."
                )
                return
            previous_pose = (
                self.centroid_x,
                self.centroid_y,
                self.centroid_heading,
            )
            self._set_circular_centroid_pose()
            with self.lock:
                model_poses = dict(self.model_poses)
            if not formation_targets_are_safe(
                self._get_world_targets(),
                self.arena_size,
                self.arena_margin,
                self.formation_obstacle_clearance,
                self.spawn_exclusion_zones,
                self.arena_profile,
                model_poses,
            ):
                self.centroid_time -= dt
                (
                    self.centroid_x,
                    self.centroid_y,
                    self.centroid_heading,
                ) = previous_pose
                self.placement_error = (
                    "A live obstacle entered the validated formation orbit."
                )

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
    def _route_batch_member_is_clear(self, robot_id, world_targets):
        """Return whether a staged robot is close enough to clear its route.

        Slot convergence still uses ``position_tolerance`` below.  Batch
        release is a different question: once a robot is inside the wider
        hysteresis band, keeping every later route stopped can deadlock a
        healthy formation when avoidance settles a Burger a few centimetres
        outside the strict target radius.
        """
        if self._slot_reached.get(robot_id, False):
            return True
        slot_index = self.assignments.get(robot_id)
        if slot_index is None or slot_index >= len(world_targets):
            return False
        with self.lock:
            pose = self.robot_poses.get(robot_id)
        if pose is None:
            return False
        target_x, target_y = world_targets[slot_index]
        return math.hypot(
            target_x - pose.position.x,
            target_y - pose.position.y,
        ) <= self.position_release_tolerance

    def _route_stall_detail(
        self,
        active_route_ids,
        route_distances: Dict[str, float],
        dt: float,
    ) -> Optional[Dict]:
        """Detect a staged robot that stopped making useful route progress.

        A later batch cannot be released while an earlier corridor remains
        occupied.  If avoidance leaves one member stationary well outside the
        safe release band, stop the fleet and let the bounded live replanner
        build a fresh assignment from the poses that Gazebo actually reached.
        """
        if not active_route_ids:
            self._route_progress_token = None
            self._route_progress_best = {}
            self._route_stall_duration = {}
            self._active_route_batch_ids = ()
            return None

        ordered_ids = tuple(sort_robot_ids(active_route_ids))
        self._active_route_batch_ids = ordered_ids
        token = (
            self._assignment_generation,
            self.route_batch_index,
            tuple(
                (
                    robot_id,
                    self.route_waypoint_indices.get(robot_id, 0),
                )
                for robot_id in ordered_ids
            ),
        )
        tracked = {
            robot_id: route_distances[robot_id]
            for robot_id in ordered_ids
            if (
                robot_id in route_distances
                and route_distances[robot_id]
                > self.position_release_tolerance
            )
        }
        if token != self._route_progress_token:
            self._route_progress_token = token
            self._route_progress_best = dict(tracked)
            self._route_stall_duration = {
                robot_id: 0.0 for robot_id in tracked
            }
            return None

        for robot_id in list(self._route_progress_best):
            if robot_id not in tracked:
                self._route_progress_best.pop(robot_id, None)
                self._route_stall_duration.pop(robot_id, None)

        for robot_id, distance in tracked.items():
            best = self._route_progress_best.get(robot_id)
            if best is None or distance <= best - self.route_progress_epsilon:
                self._route_progress_best[robot_id] = distance
                self._route_stall_duration[robot_id] = 0.0
                continue

            stalled_for = self._route_stall_duration.get(robot_id, 0.0) + dt
            self._route_stall_duration[robot_id] = stalled_for
            if stalled_for >= self.route_stall_timeout:
                detail = {
                    'gate': 'route_stall',
                    'robot': robot_id,
                    'batch': self.route_batch_index,
                    'distance': round(distance, 4),
                    'best_distance': round(
                        self._route_progress_best[robot_id], 4
                    ),
                    'stalled_for': round(stalled_for, 3),
                }
                self._last_route_stall_detail = detail
                return detail
        return None

    def _control_loop(self, event):
        with self.command_lock:
            if (
                self._shutdown_started
                or
                not self.is_running
                or self.is_paused
                or self.emergency_stop_active
            ):
                return
            try:
                self._control_step(event)
            except Exception as exc:
                # A malformed scan or another unexpected live-data value must
                # not kill rospy's timer thread while its last Twist is active.
                self.placement_error = (
                    "Formation control failed on invalid live data; motion "
                    "was disabled and a fleet stop was requested."
                )
                self.formation_state = FormationState.FAILED
                self._initial_formation_acquired = False
                self.is_running = False
                self._cancel_pending_assignment_locked(
                    clear_assignments=True
                )
                self._stop_all_robots('control_exception')
                try:
                    self._publish_status([], 0.0)
                except Exception as status_exc:
                    rospy.logerr(
                        "Formation failure status could not be published: %s",
                        status_exc,
                    )
                rospy.logerr(
                    "Formation control exception failed closed: %s", exc
                )

    def _control_step(self, event):
        """Main 20 Hz timer callback."""
        if self._shutdown_started or not self.is_running:
            return

        waiting, stale = self._odometry_readiness(self.robot_ids)
        self.waiting_for_odometry = waiting
        self.stale_odometry = stale
        if stale:
            self.placement_error = (
                "Odometry became stale or unavailable for: "
                + ", ".join(stale)
            )

        if not waiting and not stale:
            with self.lock:
                live_robot_positions = self._live_robot_positions_locked()
            if live_robot_positions is None:
                self.placement_error = (
                    "Robot odometry contained a non-finite planar pose; a "
                    "fleet stop was requested."
                )

        invalid_avoidance_limits = sorted(set(getattr(
            self, 'invalid_avoidance_limits', ()
        )) & set(self.robot_ids))
        if invalid_avoidance_limits:
            self.placement_error = (
                "Obstacle-avoidance velocity limits were invalid for: "
                + ", ".join(invalid_avoidance_limits)
            )

        if self.placement_error:
            self.formation_state = FormationState.FAILED
            self._initial_formation_acquired = False
            self._cancel_pending_assignment_locked(clear_assignments=True)
            self._stop_all_robots('control_failure')
            self._publish_status([], 0.0)
            self.is_running = False
            return

        if waiting:
            # A newly spawned robot can take a few wall-clock seconds to
            # publish its first odometry message. Nobody moves until the
            # complete fleet is observable, so assignment and avoidance never
            # operate on a partial swarm.
            self.formation_state = FormationState.FORMING
            self._stop_all_robots('odometry_wait')
            self._publish_status([], 0.0)
            return

        n = len(self.robot_ids)
        if n == 0:
            return

        dt = 1.0 / self.control_rate

        # ---- Update centroid ----
        self._update_centroid(dt)
        if self.placement_error:
            self.formation_state = FormationState.FAILED
            self._initial_formation_acquired = False
            self._cancel_pending_assignment_locked(clear_assignments=True)
            self._stop_all_robots('centroid_failure')
            self._publish_status([], 0.0)
            self.is_running = False
            return

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

        assembling_on_routes = (
            self.movement_mode == MovementMode.STATIC
            or not self._initial_formation_acquired
        )

        # ---- Adaptive mode: detect obstacle proximity ----
        deforming = False
        obstacle_center_x = 0.0
        obstacle_center_y = 0.0
        if (
            self.movement_mode == MovementMode.ADAPTIVE
            and not assembling_on_routes
        ):
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

        control_targets = list(world_targets)
        adaptive_targets_valid = True
        if deforming:
            adaptive_targets_valid = all(math.isfinite(value) for value in (
                obstacle_center_x, obstacle_center_y, self.spacing,
            ))
            if adaptive_targets_valid:
                deformed_targets = []
                for target_x, target_y in control_targets:
                    offset_x = target_x - obstacle_center_x
                    offset_y = target_y - obstacle_center_y
                    obstacle_distance = math.hypot(offset_x, offset_y)
                    if 0.01 < obstacle_distance < self.spacing * 2.0:
                        push = (
                            self.spacing * 2.0 - obstacle_distance
                        ) * 0.5
                        target_x += push * offset_x / obstacle_distance
                        target_y += push * offset_y / obstacle_distance
                    deformed_targets.append((target_x, target_y))
                adaptive_targets_valid = all(
                    math.isfinite(value)
                    for target in deformed_targets
                    for value in target
                )
                if adaptive_targets_valid:
                    control_targets = deformed_targets

        # ---- Per-robot control ----
        all_in_position = bool(self.robot_ids)
        maximum_position_error = 0.0
        active_route_ids = None
        commands_to_publish: Dict[str, Twist] = {}
        route_distances: Dict[str, float] = {}
        if assembling_on_routes and self.route_batches:
            while (
                self.route_batch_index < len(self.route_batches)
                and all(
                    self._route_batch_member_is_clear(
                        robot_id, world_targets
                    )
                    for robot_id in self.route_batches[
                        self.route_batch_index
                    ]
                )
            ):
                self.route_batch_index += 1
            if self.route_batch_index < len(self.route_batches):
                active_route_ids = set(
                    self.route_batches[self.route_batch_index]
                )

        for rid in self.robot_ids:
            slot_idx = self.assignments.get(rid)
            if slot_idx is None or slot_idx >= len(world_targets):
                all_in_position = False
                commands_to_publish[rid] = Twist()
                continue

            target_x, target_y = control_targets[slot_idx]

            final_target_x = target_x
            final_target_y = target_y

            with self.lock:
                pose = self.robot_poses.get(rid)
                yaw = self.robot_yaws.get(rid, 0.0)

            if pose is None:
                all_in_position = False
                commands_to_publish[rid] = Twist()
                continue

            robot_x = pose.position.x
            robot_y = pose.position.y

            final_dx = final_target_x - robot_x
            final_dy = final_target_y - robot_y
            final_distance = math.hypot(final_dx, final_dy)
            maximum_position_error = max(
                maximum_position_error, final_distance
            )

            if assembling_on_routes:
                waypoints = self.route_waypoints.get(rid, [])
                waypoint_index = self.route_waypoint_indices.get(rid, 0)
                while waypoint_index < len(waypoints) - 1:
                    waypoint_x, waypoint_y = waypoints[waypoint_index]
                    if math.hypot(
                        waypoint_x - robot_x, waypoint_y - robot_y
                    ) > self.route_waypoint_tolerance:
                        break
                    if not self._can_advance_route_waypoint(
                        rid,
                        (robot_x, robot_y),
                        slot_idx,
                        control_targets,
                        waypoints,
                        waypoint_index + 1,
                    ):
                        break
                    waypoint_index += 1
                self.route_waypoint_indices[rid] = waypoint_index
                if waypoint_index < len(waypoints):
                    target_x, target_y = waypoints[waypoint_index]

            dx = target_x - robot_x
            dy = target_y - robot_y
            distance = math.hypot(dx, dy)
            route_distances[rid] = distance
            angle_to_target = math.atan2(dy, dx)
            angle_error = normalize_angle(angle_to_target - yaw)

            was_reached = self._slot_reached.get(rid, False)
            tolerance = (
                self.position_release_tolerance
                if was_reached
                else self.position_tolerance
            )
            at_target = final_distance <= tolerance
            self._slot_reached[rid] = at_target
            if not at_target:
                all_in_position = False

            if (
                active_route_ids is not None
                and rid not in active_route_ids
                and not at_target
            ):
                commands_to_publish[rid] = Twist()
                continue

            # ---- PID control ----
            hold_assembly_slot = assembling_on_routes and at_target
            if hold_assembly_slot:
                # A reached robot should wait for its neighbours instead of
                # orbiting its slot under residual PID and repulsion forces.
                linear_vel = 0.0
                angular_vel = 0.0
                if not was_reached:
                    self.pid_linear[rid].reset()
                    self.pid_angular[rid].reset()
            else:
                linear_vel = self.pid_linear[rid].compute(distance, dt)
                # Reduce forward speed when facing away from target
                cos_factor = max(0.0, math.cos(angle_error))
                linear_vel *= cos_factor

                angular_vel = self.pid_angular[rid].compute(angle_error, dt)

                # If very close, damp velocities to avoid oscillation
                if distance < self.position_release_tolerance:
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

            commands_to_publish[rid] = safe_cmd

        route_stall_detail = self._route_stall_detail(
            active_route_ids, route_distances, dt
        )

        # ModelStates can change while PID commands are being calculated. Hold
        # the same lock used by its callback for one final geometric check and
        # the complete publication batch. No positive command from a stale
        # entry plan is allowed to leak out before the next timer cycle.
        live_safety_exception = None
        invalid_command_ids = []
        late_waiting = []
        late_stale = []
        live_replan_detail = None
        motion_batch_complete = True
        with self.lock:
            validated_commands = {}
            requests_motion = False
            for rid, command in commands_to_publish.items():
                validated = self._validated_motion_command(command)
                if validated is None:
                    invalid_command_ids.append(rid)
                else:
                    validated_commands[rid] = validated
                    requests_motion = requests_motion or (
                        float(validated.linear.x) != 0.0
                        or float(validated.angular.z) != 0.0
                    )

            if invalid_command_ids or not adaptive_targets_valid:
                live_motion_is_safe = False
            else:
                try:
                    live_motion_is_safe = self._live_motion_is_safe_locked(
                        control_targets, assembling_on_routes
                    )
                except Exception as exc:  # fail closed on corrupt live telemetry
                    live_motion_is_safe = False
                    live_safety_exception = exc

            # The geometric check above can scan a complete orbit. Recheck the
            # odometry clock after that work, at the last gate before the first
            # Twist, while the pose callback is still excluded by this lock.
            if live_motion_is_safe and requests_motion:
                late_waiting, late_stale = self._odometry_readiness(
                    self.robot_ids
                )
                if late_waiting or late_stale:
                    self.waiting_for_odometry = late_waiting
                    self.stale_odometry = sorted(set(
                        late_waiting + late_stale
                    ))
                    live_motion_is_safe = False

            if not live_motion_is_safe:
                if invalid_command_ids:
                    self.placement_error = (
                        "A non-finite or out-of-bounds velocity command was "
                        "rejected for: " + ", ".join(invalid_command_ids)
                    )
                elif not adaptive_targets_valid:
                    self.placement_error = (
                        "Adaptive formation produced invalid target geometry; "
                        "a fleet stop was requested."
                    )
                elif late_waiting or late_stale:
                    self.placement_error = (
                        "Odometry became stale or unavailable for: "
                        + ", ".join(self.stale_odometry)
                    )
                elif live_safety_exception is None:
                    detail = dict(getattr(
                        self, '_last_live_safety_failure', {}
                    ))
                    if (
                        assembling_on_routes
                        and detail.get('gate') == 'route'
                    ):
                        live_replan_detail = detail
                    else:
                        self.placement_error = (
                            "A live obstacle or invalid robot odometry "
                            "invalidated the formation orbit, slots, or "
                            "remaining entry route."
                        )
                else:
                    self.placement_error = (
                        "Formation live safety validation failed "
                        "unexpectedly; a fleet stop was requested."
                    )
            elif route_stall_detail is not None:
                # Do not publish another positive batch after liveness has
                # selected a replan. _schedule_live_replan_locked performs the
                # correlated zero-velocity fan-out before replacing routes.
                live_replan_detail = route_stall_detail
            else:
                motion_batch_complete = self._publish_motion_batch_locked(
                    validated_commands
                )

        if not motion_batch_complete or self._shutdown_started:
            return

        if live_replan_detail is not None:
            with self.lock:
                live_robot_positions = self._live_robot_positions_locked()
                live_model_poses = dict(self.model_poses)
                invalid_live_models = tuple(getattr(
                    self, 'invalid_model_poses', ()
                ))
            plan = getattr(self, 'active_placement_plan', None)
            if (
                plan is not None
                and live_robot_positions is not None
                and not invalid_live_models
                and self._schedule_live_replan_locked(
                    plan,
                    live_robot_positions,
                    live_model_poses,
                    live_replan_detail,
                )
            ):
                self._publish_status([], 0.0)
                return
            if not self.placement_error:
                self.placement_error = (
                    "A live obstacle or invalid robot odometry invalidated "
                    "the formation orbit, slots, or remaining entry route."
                )

        if live_safety_exception is not None:
            rospy.logerr(
                "Formation live safety validation failed closed: %s",
                live_safety_exception,
            )
        if self.placement_error:
            self.formation_state = FormationState.FAILED
            self._initial_formation_acquired = False
            self._cancel_pending_assignment_locked(clear_assignments=True)
            self._stop_all_robots('live_safety_failure')
            self._publish_status([], 0.0)
            self.is_running = False
            return

        if all_in_position:
            self._settled_duration += dt
        else:
            self._settled_duration = 0.0
        formation_is_settled = (
            all_in_position and self._settled_duration >= self.settle_time
        )
        self._maximum_position_error = maximum_position_error

        # ---- Update formation state ----
        if deforming:
            self.formation_state = FormationState.DEFORMING
        elif formation_is_settled:
            self._live_replan_attempts = 0
            if self.movement_mode == MovementMode.STATIC:
                self.formation_state = FormationState.FORMED
            else:
                self._initial_formation_acquired = True
                self.formation_state = FormationState.MOVING
        else:
            self.formation_state = FormationState.FORMING

        # ---- Publish status and markers ----
        self._publish_status(control_targets, maximum_position_error)
        self._publish_markers(control_targets)

    def _odometry_readiness(self, robot_ids, now=None):
        """Split robots into first-message waiters and stale data sources."""
        started_at = getattr(self, 'task_started_at', None)
        invalid_robots = set(getattr(self, 'invalid_robot_poses', ()))
        if started_at is None:
            return [], [
                robot_id for robot_id in robot_ids
                if robot_id in invalid_robots
            ]
        if now is None:
            now = time.monotonic()
        timeout = getattr(self, 'odom_timeout_wall_s', 0.75)
        initialization_timeout = getattr(
            self, 'odom_initialization_timeout_wall_s', 10.0
        )
        received = getattr(self, 'odom_received_at', {})
        confirmed = getattr(self, 'odom_confirmed_for_task', {})
        waiting = []
        stale = []
        for robot_id in robot_ids:
            if robot_id in invalid_robots:
                stale.append(robot_id)
                continue
            stamp = received.get(robot_id)
            if not confirmed.get(robot_id, False):
                within_initialization_window = (
                    now >= started_at
                    and now - started_at <= initialization_timeout
                )
                if within_initialization_window:
                    waiting.append(robot_id)
                else:
                    stale.append(robot_id)
            elif stamp is None:
                stale.append(robot_id)
            elif now < stamp or now - stamp > timeout:
                stale.append(robot_id)
        return waiting, stale

    def _stale_odometry(self, robot_ids, now=None):
        """List robots that exceeded their applicable odometry timeout."""
        _, stale = self._odometry_readiness(robot_ids, now=now)
        return stale

    # ------------------------------------------------------------------
    # Stop helpers
    # ------------------------------------------------------------------
    def _command_publisher_snapshot(self):
        """Copy active publishers and unresolved stop debts exactly once."""
        with self.lock:
            self._refresh_shutdown_publisher_snapshot_locked()
            return self._cached_command_publisher_snapshot()

    @staticmethod
    def _public_stop_debt(debt):
        """Remove the retained ROS object from one status-safe debt copy."""
        return {
            'robot_id': debt['robot_id'],
            'publisher_generation': int(debt['publisher_generation']),
            'task_id': debt.get('task_id'),
            'reason': str(debt.get('reason', ''))[:64],
            'first_attempt': int(debt.get('first_attempt', 0)),
            'last_attempt': int(debt.get('last_attempt', 0)),
        }

    def _record_stop_attempt(self, targets, accepted_generations, reason):
        """Merge one attempt into the per-publisher stop ledger."""
        retired_publishers = []
        with self.lock:
            # The short result lock is nested only under the state lock. The
            # shutdown path takes it on its own, so there is no lock cycle and
            # no late attempt can reopen or clear the final ledger state.
            with self._stop_result_lock:
                if self._shutdown_started:
                    publication = self._shutdown_publication
                    if publication is None:
                        publication = self._unrecorded_stop_attempt(
                            targets, accepted_generations, reason
                        )
                    return publication

                self._ensure_command_publisher_registry_locked()
                self._stop_publication_attempt = (
                    getattr(self, '_stop_publication_attempt', 0) + 1
                )
                attempt = self._stop_publication_attempt
                current_task_id = (
                    self.current_task_id
                    if isinstance(self.current_task_id, str)
                    else None
                )
                for target in targets:
                    generation = target['publisher_generation']
                    publisher = target['publisher']
                    if generation in accepted_generations:
                        self._stop_publication_debts.pop(generation, None)
                        if not self._publisher_is_active_locked(publisher):
                            retired_publishers.append((publisher, generation))
                        continue

                    previous = self._stop_publication_debts.get(generation)
                    self._stop_publication_debts[generation] = {
                        'robot_id': target['robot_id'],
                        'publisher': publisher,
                        'publisher_generation': generation,
                        'task_id': (
                            previous.get('task_id')
                            if previous is not None
                            else current_task_id
                        ),
                        'reason': (
                            previous.get('reason')
                            if previous is not None
                            else str(reason)[:64]
                        ),
                        'first_attempt': (
                            previous.get('first_attempt', attempt)
                            if previous is not None
                            else attempt
                        ),
                        'last_attempt': attempt,
                    }

                pending_debts = sorted(
                    self._stop_publication_debts.values(),
                    key=lambda debt: (
                        debt['robot_id'], debt['publisher_generation']
                    ),
                )
                correlated_task_id = current_task_id
                if correlated_task_id is None and pending_debts:
                    correlated_task_id = pending_debts[0].get('task_id')
                pending_robots = sort_robot_ids(set(
                    debt['robot_id'] for debt in pending_debts
                ))
                publication = {
                    'task_id': correlated_task_id,
                    'attempt': attempt,
                    'reason': str(reason)[:64],
                    'requested_count': len(targets),
                    'accepted_count': len(accepted_generations),
                    'failed_robots': pending_robots,
                    'unconfirmed_publishers': [
                        self._public_stop_debt(debt)
                        for debt in pending_debts
                    ],
                    # rospy accepted publish() for every active or retained
                    # local publisher. This is not a physical acknowledgement
                    # from Gazebo or a TurtleBot.
                    'publication_confirmed': not pending_debts,
                }
                self._last_stop_publication = publication
                self._refresh_shutdown_publisher_snapshot_locked()

            # unregister() belongs to ROS and can block. Leave the result lock
            # first so it can never delay the bounded shutdown fan-out.
            for publisher, generation in retired_publishers:
                self._close_retired_command_publisher_locked(
                    publisher, generation
                )
        return publication

    def _unrecorded_stop_attempt(
        self, targets, accepted_generations, reason
    ):
        """Describe a regular stop superseded by an in-flight shutdown."""
        attempt = getattr(self, '_stop_publication_attempt', 0) + 1
        current_task_id = (
            self.current_task_id
            if isinstance(self.current_task_id, str)
            else None
        )
        pending = []
        for target in targets:
            generation = target['publisher_generation']
            if generation in accepted_generations:
                continue
            previous = target.get('debt')
            pending.append({
                'robot_id': target['robot_id'],
                'publisher_generation': generation,
                'task_id': (
                    previous.get('task_id')
                    if previous is not None
                    else current_task_id
                ),
                'reason': (
                    previous.get('reason')
                    if previous is not None
                    else str(reason)[:64]
                ),
                'first_attempt': (
                    previous.get('first_attempt', attempt)
                    if previous is not None
                    else attempt
                ),
                'last_attempt': attempt,
            })
        pending.sort(key=lambda debt: (
            debt['robot_id'], debt['publisher_generation']
        ))
        return {
            'task_id': current_task_id,
            'attempt': attempt,
            'reason': str(reason)[:64],
            'requested_count': len(targets),
            'accepted_count': len(accepted_generations),
            'failed_robots': sort_robot_ids(set(
                debt['robot_id'] for debt in pending
            )),
            'unconfirmed_publishers': pending,
            'publication_confirmed': not pending,
        }

    def _stop_all_robots(
        self,
        reason='unspecified',
        publisher_snapshot=None,
        publication_lock_timeout=None,
        lifecycle_locked=True,
    ):
        """Request zero velocity from active and previously failed sockets."""
        targets = (
            tuple(publisher_snapshot)
            if publisher_snapshot is not None
            else self._command_publisher_snapshot()
        )
        if not hasattr(self, '_stop_publication_lock'):
            self._stop_publication_lock = threading.RLock()
        if publication_lock_timeout is None:
            publication_lock_acquired = self._stop_publication_lock.acquire()
        else:
            publication_lock_acquired = self._stop_publication_lock.acquire(
                timeout=max(0.0, publication_lock_timeout)
            )

        if not publication_lock_acquired:
            publication = self._record_stop_attempt(targets, set(), reason)
            stop_error = (
                "Zero-velocity publication could not begin before the "
                "bounded shutdown deadline."
            )
            if lifecycle_locked:
                self._fail_for_unconfirmed_stop_locked(stop_error)
            return publication

        try:
            report = self._stop_from_shutdown_snapshot(
                targets,
                reason,
                self.current_task_id,
                time.monotonic() + max(
                    0.0, float(self.EMERGENCY_STOP_TIMEOUT)
                ),
            )
            failed_generations = {
                item['publisher_generation']
                for item in report['unconfirmed_publishers']
            }
            accepted_generations = {
                target['publisher_generation']
                for target in targets
                if target['publisher_generation'] not in failed_generations
            }
            publication = self._record_stop_attempt(
                targets, accepted_generations, reason
            )
        finally:
            self._stop_publication_lock.release()

        if not publication['publication_confirmed'] and lifecycle_locked:
            publication_error = (
                "Zero-velocity publication was not accepted for: "
                + ", ".join(publication['failed_robots'])
                + "."
            )
            self._fail_for_unconfirmed_stop_locked(publication_error)
        return publication

    def _stop_all_robots_bounded(self, reason, timeout):
        """Fan out a stop so one blocked socket cannot hide healthy robots."""
        targets = self._command_publisher_snapshot()
        report = self._stop_from_shutdown_snapshot(
            targets,
            reason,
            self.current_task_id,
            time.monotonic() + max(0.0, float(timeout)),
        )
        failed_generations = {
            item['publisher_generation']
            for item in report['unconfirmed_publishers']
        }
        accepted_generations = {
            target['publisher_generation']
            for target in targets
            if target['publisher_generation'] not in failed_generations
        }
        publication = self._record_stop_attempt(
            targets, accepted_generations, reason
        )
        if not publication['publication_confirmed']:
            self._fail_for_unconfirmed_stop_locked(
                "Zero-velocity publication was not accepted for: "
                + ", ".join(publication['failed_robots'])
                + "."
            )
        return publication

    def _stop_from_shutdown_snapshot(
        self, targets, reason, task_id, deadline,
        motion_inflight=frozenset(),
    ):
        """Attempt the cached publisher set without taking a state lock."""
        accepted_generations = set()
        failed_targets = []
        stop = Twist()
        results = {}
        results_lock = threading.Lock()
        completed = {}

        def publish_zero(target):
            error = None
            try:
                target['publisher'].publish(stop)
            except Exception as exc:
                error = exc
            with results_lock:
                results[target['publisher_generation']] = error
            completed[target['publisher_generation']].set()

        for target in targets:
            generation = target['publisher_generation']
            completed[generation] = threading.Event()
            started = False
            last_error = None
            for attempt in (1, 2):
                worker = threading.Thread(
                    target=publish_zero,
                    args=(target,),
                    name='formation-shutdown-stop-{}-{}'.format(
                        generation, attempt
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
                            "Zero-velocity worker could not start for %s "
                            "(attempt %d): %s",
                            target['robot_id'],
                            attempt,
                            exc,
                        )
                    except Exception:
                        pass
            if not started and self._submit_safety_fallback(
                target['publisher'],
                lambda current=target: publish_zero(current),
            ):
                started = True
            if not started:
                with results_lock:
                    results[generation] = last_error
                completed[generation].set()

        # Shutdown deliberately bypasses the regular stop serialization.
        # Every publisher gets an independent attempt before this wait, so a
        # blocked callback or ROS socket cannot suppress a healthy zero.
        for target in targets:
            generation = target['publisher_generation']
            completed[generation].wait(max(
                0.0, deadline - time.monotonic()
            ))

        with results_lock:
            final_results = dict(results)
        for target in targets:
            generation = target['publisher_generation']
            if generation not in final_results:
                failed_targets.append((
                    target,
                    RuntimeError(
                        'zero publication exceeded shutdown deadline'
                    ),
                ))
            elif final_results[generation] is None:
                if generation in motion_inflight:
                    failed_targets.append((
                        target,
                        RuntimeError(
                            'motion publication was still in flight at '
                            'shutdown'
                        ),
                    ))
                else:
                    accepted_generations.add(generation)
            else:
                failed_targets.append((target, final_results[generation]))

        attempt = getattr(self, '_stop_publication_attempt', 0) + 1
        pending = []
        for target, exc in failed_targets:
            previous = target.get('debt')
            debt = {
                'robot_id': target['robot_id'],
                'publisher_generation': target['publisher_generation'],
                'task_id': (
                    previous.get('task_id')
                    if previous is not None
                    else task_id
                ),
                'reason': (
                    previous.get('reason')
                    if previous is not None
                    else str(reason)[:64]
                ),
                'first_attempt': (
                    previous.get('first_attempt', attempt)
                    if previous is not None
                    else attempt
                ),
                'last_attempt': attempt,
            }
            pending.append(debt)
            try:
                rospy.logerr(
                    "Zero-velocity publication was not accepted for %s: %s",
                    target['robot_id'],
                    exc,
                )
            except Exception:
                pass
        pending.sort(key=lambda debt: (
            debt['robot_id'], debt['publisher_generation']
        ))
        return {
            'task_id': task_id,
            'attempt': attempt,
            'reason': str(reason)[:64],
            'requested_count': len(targets),
            'accepted_count': len(accepted_generations),
            'failed_robots': sort_robot_ids(set(
                debt['robot_id'] for debt in pending
            )),
            'unconfirmed_publishers': pending,
            'publication_confirmed': not pending,
        }

    def _fail_for_unconfirmed_stop_locked(self, publication_error):
        """Apply the normal fail-closed lifecycle while command_lock is held."""
        self.is_running = False
        self.is_paused = False
        self._initial_formation_acquired = False
        self.formation_state = FormationState.FAILED
        self.orbit_path_validated = False
        self._cancel_pending_assignment_locked(clear_assignments=True)
        if self.placement_error:
            if publication_error not in self.placement_error:
                self.placement_error = (
                    self.placement_error.rstrip() + " "
                    + publication_error
                )
        else:
            self.placement_error = publication_error

    @staticmethod
    def _stop_publication_status_payload(stop_publication):
        """Return the stable public part of one local stop record."""
        return {
            'task_id': stop_publication.get('task_id'),
            'attempt': int(stop_publication.get('attempt', 0)),
            'reason': str(stop_publication.get('reason', ''))[:64],
            'requested_count': int(stop_publication.get(
                'requested_count', 0
            )),
            'accepted_count': int(stop_publication.get(
                'accepted_count', 0
            )),
            'failed_robots': list(stop_publication.get(
                'failed_robots', ()
            )),
            'unconfirmed_publishers': [
                {
                    'robot_id': debt.get('robot_id'),
                    'publisher_generation': int(debt.get(
                        'publisher_generation', 0
                    )),
                    'task_id': debt.get('task_id'),
                    'reason': str(debt.get('reason', ''))[:64],
                    'first_attempt': int(debt.get('first_attempt', 0)),
                    'last_attempt': int(debt.get('last_attempt', 0)),
                }
                for debt in stop_publication.get(
                    'unconfirmed_publishers', ()
                )
            ],
            'publication_confirmed': bool(stop_publication.get(
                'publication_confirmed', False
            )),
        }

    def _publish_stop_status_safely(self, context):
        """Keep a failed command socket from hiding the stop result."""
        if getattr(self, '_shutdown_started', False):
            return False
        try:
            self._publish_status([], 0.0)
            return True
        except Exception as exc:
            try:
                rospy.logerr(
                    "Formation %s status could not be published: %s",
                    context,
                    exc,
                )
            except Exception:
                pass
            return False

    # ------------------------------------------------------------------
    # Status publishing
    # ------------------------------------------------------------------
    def _publish_status(
        self,
        world_targets: List[Tuple[float, float]],
        maximum_position_error: float = 0.0,
    ):
        """Publish JSON status to /formation/status."""
        # Shutdown publishes its own bounded snapshot. A lifecycle callback
        # that was already in progress must not overwrite that final record.
        if getattr(self, '_shutdown_started', False):
            return
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
            'formation_heading': round(
                getattr(self, 'formation_heading', 0.0), 4
            ),
            'robot_count': len(self.robot_ids),
            'maximum_position_error': round(maximum_position_error, 4),
            'settled_for': round(self._settled_duration, 3),
            'robot_assignments': assignments_dict,
            'stale_odometry': list(getattr(self, 'stale_odometry', [])),
            'waiting_for_odometry': list(getattr(
                self, 'waiting_for_odometry', []
            )),
        }
        if (
            self.route_batches
            or self._live_replan_attempts
            or self._last_route_stall_detail is not None
        ):
            status['routing'] = {
                'batch_index': min(
                    self.route_batch_index, len(self.route_batches)
                ),
                'batch_count': len(self.route_batches),
                'active_batch': list(self._active_route_batch_ids),
                'live_replan_attempts': self._live_replan_attempts,
                'stall_timeout': round(self.route_stall_timeout, 3),
                'progress_threshold': round(
                    self.route_progress_epsilon, 4
                ),
                'last_stall': (
                    None
                    if self._last_route_stall_detail is None
                    else dict(self._last_route_stall_detail)
                ),
            }
        if (
            self.movement_mode != MovementMode.STATIC
            and self.centroid_path == CentroidPath.CIRCULAR
        ):
            effective_radius = float(getattr(
                self, 'effective_path_radius', self.path_radius
            ))
            status['orbit'] = {
                'validated': bool(getattr(
                    self, 'orbit_path_validated', False
                )),
                'requested_radius': round(float(self.path_radius), 4),
                'effective_radius': round(effective_radius, 4),
                'radius_adapted': bool(getattr(
                    self, 'orbit_radius_adapted', False
                )),
                'path_center': {
                    'x': round(float(self.path_center_x), 4),
                    'y': round(float(self.path_center_y), 4),
                },
                'validation_samples': int(getattr(
                    self, 'orbit_validation_samples', 0
                )),
                'live_obstacle_models': int(getattr(
                    self, 'orbit_validation_live_models', 0
                )),
                'completed_laps': round(
                    self.centroid_time * self.centroid_speed
                    / max(0.1, effective_radius)
                    / (2.0 * math.pi),
                    4,
                ),
            }
        if self.placement_error:
            status['error'] = self.placement_error
        stop_publication = getattr(self, '_last_stop_publication', None)
        if stop_publication is not None:
            status['stop_publication'] = (
                self._stop_publication_status_payload(stop_publication)
            )
        status_message = String(data=json.dumps(status))
        if self._shutdown_started:
            return
        try:
            self.status_pub.publish(status_message)
        finally:
            # A regular publish can have entered rospy before the epoch changed
            # and return after the bounded final status. Reassert the immutable
            # shutdown snapshot after that stale delivery completes.
            if self._shutdown_started:
                self._reassert_shutdown_status()

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
