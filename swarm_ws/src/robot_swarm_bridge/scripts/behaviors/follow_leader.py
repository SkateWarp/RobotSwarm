#!/usr/bin/env python3
"""
Follow-the-Leader Swarm Behavior Controller for TurtleBot3 Burger in Gazebo

Creates a chain: Leader -> Follower 1 -> Follower 2 -> ... -> Follower N
The leader follows predefined parametric paths (circular, square, figure8, waypoint)
or accepts manual velocity commands. Each follower tracks its own distance-based
target along the leader's travelled path using PID control.

Integrates with the ObstacleAvoidance module for safe navigation.
Dynamically adapts to robots being added or removed via /fleet/robot_list.
"""

import rospy
import math
import json
import os
import sys
import threading
import time
from typing import Dict, List, Optional, Tuple

import numpy as np

# ROS messages
from geometry_msgs.msg import Twist, Point, Pose
from gazebo_msgs.msg import ModelStates
from nav_msgs.msg import Odometry
from std_msgs.msg import String, Bool
import random as rng
from visualization_msgs.msg import Marker, MarkerArray

# Import ObstacleAvoidance from core
sys.path.append(os.path.join(os.path.dirname(__file__), '..'))
from core.obstacle_avoidance import ObstacleAvoidance
from utils.robot_ids import robot_id_sort_key, sort_robot_ids
from robot_swarm_bridge.algorithms.formation import (
    plan_obstacle_aware_route,
    signed_distance_to_zone,
)
from robot_swarm_bridge.algorithms.path_trace import ArcLengthTrace


PARAMETRIC_MODES = ('circular', 'square', 'figure8')

# One lobe of x=sin(u), y=sin(u)cos(u), measured for a unit radius.
# Keeping the complete chain shorter than this prevents two robots from being
# assigned to the figure-eight crossing from opposite lobes at the same time.
FIGURE8_LOBE_LENGTH = 3.048611735

# Keep the sampled path fine enough that a thin wall cannot sit unnoticed
# between two checks. The configured clearance is much larger than this step.
PATH_PLACEMENT_SAMPLE_STEP = 0.025
PATH_PLACEMENT_COARSE_SAMPLES = 72


# ---------------------------------------------------------------------------
# PID Controller
# ---------------------------------------------------------------------------

class PIDController:
    """
    Discrete PID controller with anti-windup clamping.

    The integral term is clamped to [-max_output, max_output] to prevent
    integral windup when the actuator saturates.
    """

    def __init__(self, kp, ki, kd, max_output, min_output=None):
        """
        Args:
            kp: Proportional gain.
            ki: Integral gain.
            kd: Derivative gain.
            max_output: Upper saturation limit.
            min_output: Lower saturation limit (defaults to -max_output).
        """
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.max_output = max_output
        self.min_output = min_output if min_output is not None else -max_output

        self._integral = 0.0
        self._prev_error = 0.0

    def compute(self, error, dt):
        """
        Compute one PID step.

        Args:
            error: Current error (setpoint - measurement).
            dt: Time elapsed since last call (seconds).

        Returns:
            Clamped control output.
        """
        if dt <= 0.0:
            return 0.0

        # Proportional
        p_term = self.kp * error

        # Integral with anti-windup clamp
        self._integral += error * dt
        integral_limit = self.max_output / max(self.ki, 1e-6)
        self._integral = max(-integral_limit, min(integral_limit, self._integral))
        i_term = self.ki * self._integral

        # Derivative (on error)
        d_term = self.kd * (error - self._prev_error) / dt
        self._prev_error = error

        output = p_term + i_term + d_term
        return max(self.min_output, min(self.max_output, output))

    def reset(self):
        """Reset internal state."""
        self._integral = 0.0
        self._prev_error = 0.0


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def normalize_angle(angle):
    """Wrap angle to [-pi, pi]."""
    return math.atan2(math.sin(angle), math.cos(angle))


def yaw_from_quaternion(q):
    """Extract yaw from a geometry_msgs/Quaternion."""
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


# ---------------------------------------------------------------------------
# Follow-the-Leader Node
# ---------------------------------------------------------------------------

class FollowTheLeader:
    """
    ROS node implementing Follow-the-Leader swarm behaviour.

    Fleet membership comes from ``/fleet/robot_list``. At task start the node
    chooses the front robot as leader, then orders the remaining robots by
    spatial proximity so every follower tracks the robot immediately ahead.

    Leader motion is generated internally according to the chosen *mode*
    (circular, square, figure8, waypoint, manual).  Follower motion uses a
    distance-based playback of the leader's path, combined with a PID tracking
    loop. Every follower receives a different point on the same path.

    All velocity commands pass through an :class:`ObstacleAvoidance` filter
    before being published. Parametric paths are anchored at the leader pose
    captured for each task and grow when necessary to fit the whole chain.
    """

    # -- initialisation -----------------------------------------------------

    def __init__(self):
        rospy.init_node('follow_leader', anonymous=False)

        # ---- ROS parameters ------------------------------------------------
        self.robot_count = rospy.get_param('~robot_count', 5)
        self.leader_mode = rospy.get_param('~leader_mode', 'circular')
        self.follow_distance = rospy.get_param('~follow_distance', 0.7)
        self.max_linear_vel = rospy.get_param('~max_linear_vel', 0.2)
        self.max_angular_vel = rospy.get_param('~max_angular_vel', 1.5)
        self.follower_speed_reserve = max(0.0, min(
            self.max_linear_vel * 0.5,
            float(rospy.get_param('~follower_speed_reserve', 0.05)),
        ))
        self.path_radius = max(
            0.5, float(rospy.get_param('~path_radius', 2.0))
        )
        self.requested_path_radius = self.path_radius
        self.arena_size = float(rospy.get_param(
            '/fleet_manager/arena_size',
            rospy.get_param('~arena_size', 10.0),
        ))
        self.arena_margin = max(0.0, float(rospy.get_param(
            '/fleet_manager/arena_margin',
            rospy.get_param('~arena_margin', 0.35),
        )))
        self.arena_profile = rospy.get_param(
            '~arena_profile', 'swarm_arena'
        )
        self.spawn_obstacle_clearance = max(0.0, float(rospy.get_param(
            '~spawn_obstacle_clearance', 0.30,
        )))
        configured_zones = rospy.get_param('~spawn_exclusion_zones', [])
        self.spawn_exclusion_zones = (
            configured_zones if isinstance(configured_zones, list) else []
        )
        self.model_poses = {}
        self.path_relocation_step = max(0.25, float(rospy.get_param(
            '~path_relocation_step', 0.50,
        )))
        self.path_relocation_limit = max(1, int(rospy.get_param(
            '~path_relocation_limit', 128,
        )))
        self.path_planner_async = bool(rospy.get_param(
            '~path_planner_async', True,
        ))
        self.waypoints = rospy.get_param(
            '~waypoints', [[2, 0], [2, 2], [0, 2], [0, 0]]
        )

        # ---- Derived / internal constants ----------------------------------
        self.control_rate = 20.0  # Hz
        self.dt = 1.0 / self.control_rate
        self.waypoint_tolerance = 0.15  # metres

        # ---- Leader path state ---------------------------------------------
        # Parametric motion clock. It advances only after the chain is ready.
        self.path_t = 0.0
        self.current_waypoint_idx = 0
        self.path_anchor_x = 0.0
        self.path_anchor_y = 0.0
        self.path_anchor_yaw = 0.0
        self.path_rotation = 0.0
        self.path_phase = 0.0
        self.path_direction = 1.0
        self.path_initial_heading_error = 0.0
        self.path_anchor_ready = False
        self.path_radius_was_adapted = False
        self.path_error = None
        self.path_planning = False
        self.path_plan_generation = 0
        self.path_plan_thread = None
        self.path_planning_started_at = None
        self.path_planning_wall_s = 0.0
        self.path_relocated = False
        self.path_staging = False
        self.staging_targets = {}
        self.staging_routes = {}
        self.staging_route_indices = {}
        self.staging_batches = []
        self.staging_batch_index = 0
        self.staging_reached = {}
        self.staging_phase = None
        self.staging_settle_ticks = 0
        self.chain_assembled = False
        self.chain_settle_ticks = 0
        settle_time = max(
            0.0, float(rospy.get_param('~chain_settle_time', 0.35))
        )
        self.chain_settle_required = max(
            1, int(math.ceil(settle_time / self.dt))
        )
        self.chain_settle_tolerance = max(
            0.06, float(rospy.get_param('~chain_settle_tolerance', 0.14))
        )
        self.leader_speed_scale = 0.0
        leader_ramp_time = max(
            0.1, float(rospy.get_param('~leader_ramp_time', 1.2))
        )
        self.leader_ramp_step = self.dt / leader_ramp_time

        # ---- Per-robot data (keyed by namespace string) --------------------
        self.robot_names = []  # sorted list, index 0 = leader
        self.poses = {}        # str -> Pose
        self.yaws = {}         # str -> float  (cache to avoid repeated quat conversion)
        self.odom_received_at = {}
        self.odom_timeout_wall_s = max(
            0.2, float(rospy.get_param('~odom_timeout_wall_s', 0.75))
        )
        self.task_started_at = None
        self.stale_odometry = []
        self.linear_pids = {}  # str -> PIDController
        self.angular_pids = {} # str -> PIDController
        self.avoidance = {}    # str -> ObstacleAvoidance
        self.cmd_pubs = {}     # str -> rospy.Publisher
        self.odom_subs = {}    # str -> rospy.Subscriber

        # ---- Behaviour state -----------------------------------------------
        self.is_active = False
        self.is_paused = False
        self.emergency_stop_active = False
        self.current_task_id = None
        self.manual_twist = Twist()
        self.leader_trace = ArcLengthTrace(minimum_step=0.015)
        self.lock = threading.RLock()  # guards robot_names / per-robot dicts
        self.command_lock = threading.RLock()

        # ---- Global publishers / subscribers --------------------------------
        self.status_pub = rospy.Publisher(
            '/follow_leader/status', String, queue_size=1
        )
        self.marker_pub = rospy.Publisher(
            '/follow_leader/markers', MarkerArray, queue_size=1
        )

        rospy.Subscriber(
            '/fleet/robot_list', String, self._fleet_list_cb, queue_size=1
        )
        rospy.Subscriber(
            '/gazebo/model_states', ModelStates,
            self._model_states_cb, queue_size=1,
        )
        rospy.Subscriber(
            '/follow_leader/start', String, self._start_cb, queue_size=1
        )
        rospy.Subscriber(
            '/follow_leader/stop', String, self._stop_cb, queue_size=1
        )
        rospy.Subscriber(
            '/follow_leader/pause', String, self._pause_cb, queue_size=1
        )
        rospy.Subscriber(
            '/follow_leader/resume', String, self._resume_cb, queue_size=1
        )
        rospy.Subscriber(
            '/leader/cmd_vel', Twist, self._manual_cmd_cb, queue_size=1
        )
        rospy.Subscriber(
            '/swarm/emergency_stop', Bool,
            self._emergency_stop_cb, queue_size=1
        )

        # ---- Bootstrap default fleet if no /fleet/robot_list arrives -------
        self._bootstrap_default_fleet()

        # ---- Control timer -------------------------------------------------
        self.timer = rospy.Timer(
            rospy.Duration(self.dt), self._control_loop
        )
        rospy.on_shutdown(self._shutdown)

        rospy.loginfo(
            "[follow_leader] Initialised: mode=%s, robots=%d, follow_dist=%.2f, "
            "radius=%.2f",
            self.leader_mode, self.robot_count, self.follow_distance,
            self.path_radius,
        )

    # -- fleet management ---------------------------------------------------

    def _bootstrap_default_fleet(self):
        """
        Create the initial robot list from ``~robot_count`` using the tb3_N
        naming convention.  This is used if ``/fleet/robot_list`` has not
        published yet.
        """
        names = ['tb3_{}'.format(i) for i in range(self.robot_count)]
        self._sync_fleet(names)

    def _fleet_list_cb(self, msg):
        """
        Handle ``/fleet/robot_list`` (comma-separated namespace string).
        Adds/removes robots dynamically.
        """
        names = [n.strip() for n in msg.data.split(',') if n.strip()]
        self._sync_fleet(names)

    def _sync_fleet(self, new_names):
        """
        Synchronise internal data structures with *new_names*.

        * Robots present in *new_names* but not yet tracked are added.
        * Robots no longer in *new_names* are removed.
        * The list is sorted so that the chain order is deterministic.
        """
        new_names_sorted = sort_robot_ids(new_names)

        with self.command_lock:
            with self.lock:
                old_set = set(self.robot_names)
                new_set = set(new_names_sorted)
                # Fleet messages describe membership, not follow order. Once
                # a spatial chain is chosen, a periodically sorted roster must
                # not quietly replace it and restart path planning.
                if new_set == old_set:
                    return

                old_leader = self.robot_names[0] if self.robot_names else None

                if not new_set and (self.is_active or self.is_paused):
                    self._cancel_path_plan_locked(clear_staging=True)
                    self._stop_all()
                    self.is_active = False
                    self.is_paused = False
                    self.leader_trace.clear()

                # Add new robots
                for name in new_set - old_set:
                    self._add_robot(name)

                # Remove departed robots
                for name in old_set - new_set:
                    self._remove_robot(name)

                self.robot_names = new_names_sorted
                new_leader = self.robot_names[0] if self.robot_names else None
                if old_leader != new_leader:
                    self.leader_trace.clear()

                # A changed chain can require a larger closed path. Re-anchor
                # at the leader's current pose on the next control tick so the
                # desired position does not jump across the arena.
                if (
                    new_names_sorted
                    and self.is_active
                    and self.leader_mode in PARAMETRIC_MODES
                ):
                    self._cancel_path_plan_locked(clear_staging=True)
                    self.path_t = 0.0
                    self.path_anchor_ready = False
                    self.path_error = None
                    self.chain_assembled = False
                    self.chain_settle_ticks = 0
                    self.leader_speed_scale = 0.0
                    self.leader_trace.clear()
                    for pid in self.linear_pids.values():
                        pid.reset()
                    for pid in self.angular_pids.values():
                        pid.reset()

        rospy.loginfo(
            "[follow_leader] Fleet synced: %s", ', '.join(new_names_sorted)
        )

    def _chain_poses_ready(self, names):
        """Return True once every robot in the chain has an odometry pose."""
        return all(self.poses.get(name) is not None for name in names)

    def _add_robot(self, name):
        """Register publishers, subscribers, PID controllers, etc. for *name*."""
        if not hasattr(self, 'odom_received_at'):
            self.odom_received_at = {}
        self.poses[name] = None
        self.yaws[name] = 0.0
        self.odom_received_at[name] = None

        self.linear_pids[name] = PIDController(
            kp=1.0, ki=0.0, kd=0.3,
            max_output=self.max_linear_vel,
        )
        self.angular_pids[name] = PIDController(
            kp=2.0, ki=0.0, kd=0.4,
            max_output=self.max_angular_vel,
        )

        self.cmd_pubs[name] = rospy.Publisher(
            '/{}/cmd_vel'.format(name), Twist, queue_size=1
        )
        self.odom_subs[name] = rospy.Subscriber(
            '/{}/odom'.format(name), Odometry,
            callback=self._odom_cb, callback_args=name,
            queue_size=1,
        )
        self.avoidance[name] = ObstacleAvoidance(name)

        rospy.loginfo("[follow_leader] Added robot: %s", name)

    def _remove_robot(self, name):
        """Unregister a robot and clean up resources."""
        pub = self.cmd_pubs.pop(name, None)
        if pub is not None:
            pub.publish(Twist())
            pub.unregister()

        odom_sub = self.odom_subs.pop(name, None)
        if odom_sub is not None:
            odom_sub.unregister()

        avoidance = self.avoidance.pop(name, None)
        if avoidance is not None:
            avoidance.shutdown()

        for d in (
            self.poses, self.yaws,
            getattr(self, 'odom_received_at', {}),
            self.linear_pids, self.angular_pids,
        ):
            d.pop(name, None)

        rospy.loginfo("[follow_leader] Removed robot: %s", name)

    # -- callbacks ----------------------------------------------------------

    def _odom_cb(self, msg, robot_name):
        """
        Store the latest pose.
        """
        pose = msg.pose.pose
        yaw = yaw_from_quaternion(pose.orientation)

        with self.lock:
            if robot_name not in self.poses:
                return
            self.poses[robot_name] = pose
            self.yaws[robot_name] = yaw
            self.odom_received_at[robot_name] = time.monotonic()
            avoidance = self.avoidance.get(robot_name)

        # Keep obstacle avoidance module in sync
        if avoidance is not None:
            avoidance.set_position(pose.position.x, pose.position.y, yaw)

    def _model_states_cb(self, msg):
        """Track live poses for configured obstacles and the pushable payload."""
        configured_models = {
            zone.get('model')
            for zone in self.spawn_exclusion_zones
            if isinstance(zone, dict) and zone.get('model')
        }
        live_poses = {}
        for model_name, pose in zip(msg.name, msg.pose):
            if model_name not in configured_models:
                continue
            live_poses[model_name] = (
                pose.position.x,
                pose.position.y,
                yaw_from_quaternion(pose.orientation),
            )
        with self.lock:
            self.model_poses = live_poses

    def _start_cb(self, msg):
        # Parse config from task orchestrator (JSON String)
        try:
            config = json.loads(msg.data) if msg.data else {}
        except (json.JSONDecodeError, AttributeError):
            config = {}

        with self.command_lock:
            if self.emergency_stop_active:
                rospy.logwarn(
                    "[follow_leader] Start rejected while emergency stop is active"
                )
                return
            with self.lock:
                if not self.robot_names:
                    rospy.logwarn(
                        "[follow_leader] Start rejected without an active fleet"
                    )
                    return

            # Apply runtime config while the control loop is quiescent.
            if 'leader_mode' in config:
                self.leader_mode = config['leader_mode']
            if 'waypoints' in config and config['waypoints']:
                parsed_waypoints = []
                for waypoint in config['waypoints']:
                    if isinstance(waypoint, dict):
                        if 'x' not in waypoint or 'y' not in waypoint:
                            continue
                        parsed_waypoints.append([
                            float(waypoint['x']), float(waypoint['y'])
                        ])
                    elif (
                        isinstance(waypoint, (list, tuple))
                        and len(waypoint) >= 2
                    ):
                        parsed_waypoints.append([
                            float(waypoint[0]), float(waypoint[1])
                        ])
                if parsed_waypoints:
                    self.waypoints = parsed_waypoints
            if 'radius' in config:
                try:
                    self.requested_path_radius = max(
                        0.5, float(config['radius'])
                    )
                    self.path_radius = self.requested_path_radius
                except (TypeError, ValueError):
                    pass
            if 'follow_distance' in config:
                try:
                    self.follow_distance = max(
                        0.35, float(config['follow_distance'])
                    )
                except (TypeError, ValueError):
                    pass

            if self.leader_mode == 'random':
                self.waypoints = [
                    [rng.uniform(-3.5, 3.5), rng.uniform(-3.5, 3.5)]
                    for _ in range(8)
                ]
                self.leader_mode = 'waypoint'

            self.current_task_id = config.get('task_id')
            self.task_started_at = time.monotonic()
            self.stale_odometry = []
            self.is_active = True
            self.is_paused = False
            self._cancel_path_plan_locked(clear_staging=True)
            self.path_t = 0.0
            self.current_waypoint_idx = 0
            self.path_anchor_ready = False
            self.path_radius_was_adapted = False
            self.path_error = None
            self.chain_assembled = False
            self.chain_settle_ticks = 0
            self.leader_speed_scale = 0.0
            self.leader_trace.clear()
            with self.lock:
                self.robot_names = self._spatial_chain_order(self.robot_names)
                names = list(self.robot_names)
                if names and self._chain_poses_ready(names):
                    pose = self.poses.get(names[0])
                    if pose is not None:
                        leader_yaw = self.yaws.get(names[0], 0.0)
                        self._seed_trace_from_chain(names, leader_yaw)
                        if self.leader_mode in PARAMETRIC_MODES:
                            self._begin_path_planning_locked(names)
                for pid in self.linear_pids.values():
                    pid.reset()
                for pid in self.angular_pids.values():
                    pid.reset()
            start_error = self.path_error

        self._publish_status([])
        if start_error:
            self._stop_all()
            rospy.logerr("[follow_leader] %s", start_error)
        else:
            rospy.loginfo(
                "[follow_leader] Behaviour STARTED: mode=%s, waypoints=%d, "
                "radius=%.2f",
                self.leader_mode, len(self.waypoints), self.path_radius,
            )

    def _spatial_chain_order(self, names):
        """Pick a front robot as leader and order followers by proximity."""
        ordered = [
            name for name in names
            if name in self.poses and self.poses.get(name) is not None
        ]
        missing = [name for name in names if name not in ordered]
        if len(ordered) <= 1:
            return names

        nominal_yaw = self.yaws.get(ordered[0], 0.0)
        forward = np.array([math.cos(nominal_yaw), math.sin(nominal_yaw)])
        leader = max(
            ordered,
            key=lambda name: (
                float(np.dot(
                    np.array([
                        self.poses[name].position.x,
                        self.poses[name].position.y,
                    ]),
                    forward,
                )),
                -robot_id_sort_key(name)[1],
                name,
            ),
        )
        result = [leader]
        remaining = set(name for name in ordered if name != leader)
        tail = leader
        while remaining:
            tail_pose = self.poses.get(tail)
            if tail_pose is None:
                break
            tail_xy = np.array([
                tail_pose.position.x,
                tail_pose.position.y,
            ])
            next_name = min(
                remaining,
                key=lambda name: (
                    float(np.linalg.norm(
                        np.array([
                            self.poses[name].position.x,
                            self.poses[name].position.y,
                        ]) - tail_xy
                    )),
                    robot_id_sort_key(name),
                ),
            )
            result.append(next_name)
            remaining.remove(next_name)
            tail = next_name

        result.extend(sort_robot_ids(remaining))
        result.extend(missing)
        return result

    def _stop_cb(self, msg):
        with self.command_lock:
            if not self._task_command_matches(msg):
                return
            self._cancel_path_plan_locked(clear_staging=True)
            self.is_active = False
            self.is_paused = False
            self._stop_all()
        rospy.loginfo("[follow_leader] Behaviour STOPPED")

    def _pause_cb(self, msg):
        with self.command_lock:
            if not self._task_command_matches(msg):
                return
            if not self.is_active or self.is_paused:
                return
            self.is_paused = True
            self._stop_all()
        rospy.loginfo("[follow_leader] Behaviour PAUSED")

    def _resume_cb(self, msg):
        with self.command_lock:
            if not self._task_command_matches(msg):
                return
            if (
                not self.is_active
                or not self.is_paused
                or self.emergency_stop_active
            ):
                return
            self.is_paused = False
        rospy.loginfo("[follow_leader] Behaviour RESUMED")

    def _task_command_matches(self, msg):
        try:
            payload = json.loads(msg.data) if msg.data else {}
        except (json.JSONDecodeError, AttributeError):
            payload = {}
        requested_id = payload.get('task_id')
        return bool(requested_id) and requested_id == self.current_task_id

    def _manual_cmd_cb(self, msg):
        self.manual_twist = msg

    def _emergency_stop_cb(self, msg):
        with self.command_lock:
            self.emergency_stop_active = bool(msg.data)
            if self.emergency_stop_active:
                self._cancel_path_plan_locked(clear_staging=True)
                self.is_active = False
                self.is_paused = False
                self._stop_all()
                rospy.logwarn("[follow_leader] Emergency stop latched")

    def _shutdown(self):
        """Publish a final zero command before leaving the ROS graph."""
        with self.command_lock:
            self.is_active = False
            self.is_paused = False
            self._stop_all()

    # -- leader path generation ---------------------------------------------

    def _cancel_path_plan_locked(self, clear_staging=False):
        """Invalidate a planner result that belongs to older task state."""
        self.path_plan_generation = getattr(
            self, 'path_plan_generation', 0
        ) + 1
        self.path_planning = False
        self.path_planning_started_at = None
        if clear_staging:
            self.path_staging = False
            self.path_relocated = False
            self.staging_routes = {}
            self.staging_route_indices = {}
            self.staging_batches = []
            self.staging_batch_index = 0
            self.staging_reached = {}
            self.staging_phase = None
            self.staging_settle_ticks = 0

    def _begin_path_planning_locked(self, names):
        """Start a non-blocking closed-path search from an immutable scene."""
        if self.path_planning or not names:
            return

        with self.lock:
            if not self._chain_poses_ready(names):
                return
            positions = {
                name: (
                    self.poses[name].position.x,
                    self.poses[name].position.y,
                )
                for name in names
            }
            yaws = {
                name: self.yaws.get(name, 0.0) for name in names
            }
            model_poses = dict(self.model_poses)

        self.path_plan_generation += 1
        generation = self.path_plan_generation
        self.path_planning = True
        self.path_planning_started_at = time.monotonic()
        self.path_planning_wall_s = 0.0
        self.path_staging = False
        self.path_relocated = False
        self.path_error = None
        snapshot = {
            'generation': generation,
            'task_id': self.current_task_id,
            'names': tuple(names),
            'positions': positions,
            'yaws': yaws,
            'mode': self.leader_mode,
            'requested_radius': self.requested_path_radius,
            'follow_distance': self.follow_distance,
            'model_poses': model_poses,
            'started_at': self.path_planning_started_at,
        }

        if not getattr(self, 'path_planner_async', True):
            self._path_plan_worker(snapshot)
            return

        planner = threading.Thread(
            target=self._path_plan_worker,
            args=(snapshot,),
            name='follow-path-planner-{}'.format(generation),
            daemon=True,
        )
        self.path_plan_thread = planner
        planner.start()

    def _path_plan_worker(self, snapshot):
        """Solve one path snapshot and commit it only if it is still current."""
        try:
            result = self._build_path_plan(snapshot)
        except Exception as exc:  # keep the timer alive after planner failures
            rospy.logerr("[follow_leader] Path planner failed: %s", exc)
            result = {'error': 'Path planning failed: {}'.format(exc)}

        with self.command_lock:
            if (
                snapshot['generation'] != self.path_plan_generation
                or snapshot['task_id'] != self.current_task_id
                or tuple(self.robot_names) != snapshot['names']
                or not self.is_active
            ):
                return

            self.path_planning = False
            self.path_planning_wall_s = max(
                0.0, time.monotonic() - snapshot['started_at']
            )
            self.path_planning_started_at = None
            if result.get('error'):
                self.path_error = result['error']
                self.path_anchor_ready = False
                return

            self.path_radius = result['radius']
            self.path_radius_was_adapted = result['radius_adapted']
            self.path_anchor_x = result['anchor'][0]
            self.path_anchor_y = result['anchor'][1]
            self.path_anchor_yaw = snapshot['yaws'][snapshot['names'][0]]
            self.path_phase = result['phase']
            self.path_direction = result['direction']
            self.path_rotation = result['rotation']
            self.path_initial_heading_error = result['heading_error']
            self.path_relocated = result['relocated']
            self.path_staging = bool(result['route'])
            self.staging_routes = {
                snapshot['names'][0]: list(result['route'][1:])
            } if result['route'] else {}
            self.staging_route_indices = {
                snapshot['names'][0]: 0
            } if result['route'] else {}
            self.staging_reached = {
                name: False for name in snapshot['names']
            }
            self.staging_phase = (
                'assembling' if self.path_staging else None
            )
            self.staging_settle_ticks = 0
            self.path_anchor_ready = not self.path_staging
            self.path_t = 0.0
            self.chain_assembled = False
            self.chain_settle_ticks = 0
            self.leader_speed_scale = 0.0
            self.path_error = None

    def _build_path_plan(self, snapshot):
        """Find an anchored lap, or the nearest safely reachable start."""
        mode = snapshot['mode']
        count = len(snapshot['names'])
        requested = max(0.5, float(snapshot['requested_radius']))
        required_length = count * snapshot['follow_distance']
        if mode == 'circular':
            minimum = required_length / (2.0 * math.pi)
        elif mode == 'square':
            minimum = required_length / 8.0
        else:
            minimum = max(0.8, required_length / FIGURE8_LOBE_LENGTH)
        radius = max(requested, minimum)

        leader = snapshot['names'][0]
        start = snapshot['positions'][leader]
        leader_yaw = snapshot['yaws'][leader]
        placement = self._find_path_placement(
            mode, radius, start[0], start[1], leader_yaw,
            snapshot['model_poses'],
        )
        if placement is not None:
            return self._path_plan_result(
                start, radius, requested, placement, False, []
            )

        for delta_x, delta_y in self._path_relocation_offsets(start):
            anchor = (start[0] + delta_x, start[1] + delta_y)
            placement = self._find_path_placement(
                mode, radius, anchor[0], anchor[1], leader_yaw,
                snapshot['model_poses'],
            )
            if placement is None:
                continue
            route = plan_obstacle_aware_route(
                start,
                anchor,
                self.arena_size,
                self.arena_margin,
                self.spawn_obstacle_clearance,
                self.spawn_exclusion_zones,
                self.arena_profile,
                snapshot['model_poses'],
                circle_samples=8,
            )
            if route is None:
                continue
            return self._path_plan_result(
                anchor, radius, requested, placement, True, route
            )

        return {'error': self._path_fit_error(
            mode, count, radius, requested, minimum
        )}

    @staticmethod
    def _path_plan_result(
        anchor, radius, requested, placement, relocated, route
    ):
        phase, direction, rotation, heading_error = placement
        return {
            'anchor': anchor,
            'radius': radius,
            'radius_adapted': radius > requested + 1e-9,
            'phase': phase,
            'direction': direction,
            'rotation': rotation,
            'heading_error': heading_error,
            'relocated': relocated,
            'route': route,
        }

    def _path_relocation_offsets(self, start):
        """Return a bounded nearest-first grid of alternate path starts."""
        usable_half = self.arena_size / 2.0 - self.arena_margin
        step = getattr(self, 'path_relocation_step', 0.50)
        limit = getattr(self, 'path_relocation_limit', 128)
        min_x = int(math.ceil((-usable_half - start[0]) / step))
        max_x = int(math.floor((usable_half - start[0]) / step))
        min_y = int(math.ceil((-usable_half - start[1]) / step))
        max_y = int(math.floor((usable_half - start[1]) / step))
        offsets = [
            (index_x * step, index_y * step)
            for index_x in range(min_x, max_x + 1)
            for index_y in range(min_y, max_y + 1)
            if index_x != 0 or index_y != 0
        ]
        offsets.sort(key=lambda offset: (
            round(offset[0] * offset[0] + offset[1] * offset[1], 12),
            abs(offset[0]) + abs(offset[1]),
            abs(offset[0]),
            0 if offset[1] >= 0.0 else 1,
            0 if offset[0] >= 0.0 else 1,
        ))
        return offsets[:limit]

    def _path_fit_error(self, mode, count, radius, requested, minimum):
        return (
            "No reachable {mode} path for {count} robots fits inside the "
            "{arena:.1f} m arena while clearing its configured obstacles. "
            "Its effective radius is {radius:.2f} m (requested "
            "{requested:.2f} m; minimum {minimum:.2f} m for the "
            "{distance:.2f} m chain). Reduce the radius/robot count/follow "
            "distance, or use a larger arena."
        ).format(
            mode=mode,
            count=count,
            arena=self.arena_size,
            radius=radius,
            requested=requested,
            minimum=minimum,
            distance=max(0, count - 1) * self.follow_distance,
        )

    def _minimum_path_radius(self, mode, robot_count=None):
        """Return a conservative lower bound for closed leader paths.

        A closed path must have enough arc length for the complete train.  For
        a figure eight the tighter limit is one lobe, otherwise robots on
        opposite halves of the chain can be sent through the crossing together.
        """
        count = len(self.robot_names) if robot_count is None else robot_count
        count = max(1, int(count))
        required_length = count * self.follow_distance

        if mode == 'circular':
            return required_length / (2.0 * math.pi)
        if mode == 'square':
            return required_length / 8.0
        if mode == 'figure8':
            return max(
                0.8,
                required_length / FIGURE8_LOBE_LENGTH,
            )
        return 0.0

    def _path_period(self, mode, radius):
        """Return the time needed for one complete parametric lap."""
        speed = max(1e-6, self._parametric_path_speed())
        if mode == 'circular':
            return 2.0 * math.pi * radius / speed
        if mode == 'square':
            return 8.0 * radius / (speed * 0.8)
        if mode == 'figure8':
            return 4.0 * math.pi * radius / speed
        return 0.0

    def _path_lap_progress(self):
        """Describe progress around the current closed leader path.

        ``path_t`` starts only after the chain is assembled, and it resets
        whenever a task or fleet layout is reconfigured.  That makes this a
        useful acceptance clock: setup time cannot be mistaken for movement
        around the requested path.
        """
        if self.leader_mode not in PARAMETRIC_MODES:
            return {
                'path_period_s': None,
                'path_progress_laps': None,
                'current_lap_progress': None,
                'completed_laps': 0,
            }

        period = self._path_period(self.leader_mode, self.path_radius)
        if period <= 0.0 or not math.isfinite(period):
            return {
                'path_period_s': None,
                'path_progress_laps': None,
                'current_lap_progress': None,
                'completed_laps': 0,
            }

        progress = max(0.0, self.path_t) / period
        completed = int(math.floor(progress))
        return {
            'path_period_s': round(period, 4),
            'path_progress_laps': round(progress, 4),
            'current_lap_progress': round(progress - completed, 4),
            'completed_laps': completed,
        }

    def _parametric_path_speed(self):
        """Slow long leader chains while keeping followers able to catch up."""
        count = max(1, len(self.robot_names))
        cruise_limit = max(
            0.06,
            self.max_linear_vel - self.follower_speed_reserve,
        )
        if count <= 3:
            return cruise_limit
        scale = min(1.0, 2.0 / float(count))
        return min(
            cruise_limit,
            max(0.06, self.max_linear_vel * scale),
        )

    def _leader_speed_limit(self):
        """Leave a little velocity headroom for a follower that falls behind."""
        return max(
            0.06,
            self.max_linear_vel - self.follower_speed_reserve,
        )

    def _local_path_point(self, mode, t, radius):
        """Generate a path that starts at the origin, heading along +x."""
        path_speed = self._parametric_path_speed()
        if mode == 'circular':
            angle = path_speed * t / radius
            return (
                radius * math.sin(angle),
                radius * (1.0 - math.cos(angle)),
            )

        if mode == 'square':
            side = radius * 2.0
            perimeter = side * 4.0
            distance = (path_speed * 0.8 * t) % perimeter

            if distance < side:
                return distance, 0.0
            if distance < 2.0 * side:
                return side, distance - side
            if distance < 3.0 * side:
                return side - (distance - 2.0 * side), side
            return 0.0, side - (distance - 3.0 * side)

        if mode == 'figure8':
            angle = path_speed * t / radius * 0.5
            raw_x = radius * math.sin(angle)
            raw_y = raw_x * math.cos(angle)

            # The raw lemniscate leaves the origin at 45 degrees. Rotating it
            # here gives every parametric mode the same start convention.
            inverse_root_two = 1.0 / math.sqrt(2.0)
            return (
                (raw_x + raw_y) * inverse_root_two,
                (-raw_x + raw_y) * inverse_root_two,
            )

        return 0.0, 0.0

    def _anchored_path_point(
        self, mode, t, radius, anchor_x, anchor_y, rotation,
        phase=0.0, direction=1.0,
    ):
        start_x, start_y = self._local_path_point(mode, phase, radius)
        local_x, local_y = self._local_path_point(
            mode, phase + direction * t, radius
        )
        local_x -= start_x
        local_y -= start_y
        cosine = math.cos(rotation)
        sine = math.sin(rotation)
        return (
            anchor_x + cosine * local_x - sine * local_y,
            anchor_y + sine * local_x + cosine * local_y,
        )

    def _path_fits_arena(
        self, mode, radius, anchor_x, anchor_y, rotation,
        phase=0.0, direction=1.0, model_poses=None,
    ):
        """Verify a complete lap at the final 2.5 cm resolution."""
        period = self._path_period(mode, radius)
        path_upper_length = self._parametric_path_speed() * period
        sample_count = max(
            360,
            int(math.ceil(
                path_upper_length / PATH_PLACEMENT_SAMPLE_STEP
            )),
        )
        sample_count += (-sample_count) % 4
        active_zones = list(self._active_exclusion_zones())
        if model_poses is None:
            model_poses = self._model_pose_snapshot()
        return self._path_fits_arena_samples(
            mode,
            radius,
            anchor_x,
            anchor_y,
            rotation,
            phase,
            direction,
            period,
            sample_count,
            active_zones,
            model_poses,
        )

    def _path_fits_arena_samples(
        self, mode, radius, anchor_x, anchor_y, rotation,
        phase, direction, period, sample_count, active_zones, model_poses,
    ):
        """Check one sampled lap using already resolved planning context."""
        usable_half = self.arena_size / 2.0 - self.arena_margin
        if usable_half <= 0.0 or period <= 0.0 or sample_count < 4:
            return False
        for index in range(sample_count + 1):
            x, y = self._anchored_path_point(
                mode,
                period * index / sample_count,
                radius,
                anchor_x,
                anchor_y,
                rotation,
                phase,
                direction,
            )
            if abs(x) > usable_half + 1e-9 or abs(y) > usable_half + 1e-9:
                return False
            if not self._path_point_is_clear(
                x, y, active_zones, model_poses
            ):
                return False
        return True

    def _model_pose_snapshot(self):
        """Copy live model poses so one placement uses one consistent scene."""
        with self.lock:
            return dict(getattr(self, 'model_poses', {}))

    def _active_exclusion_zones(self):
        """Yield configured collision zones for the selected arena world."""
        arena_profile = getattr(self, 'arena_profile', 'swarm_arena')
        for zone in getattr(self, 'spawn_exclusion_zones', []):
            if not isinstance(zone, dict):
                continue
            worlds = zone.get('worlds')
            if isinstance(worlds, str):
                worlds = [worlds]
            if worlds and arena_profile not in worlds:
                continue
            if zone.get('shape') in ('box', 'circle'):
                yield zone

    def _path_point_is_clear(self, x, y, active_zones, model_poses):
        """Return whether one robot-centre point clears every active zone."""
        default_clearance = max(
            0.0, float(getattr(self, 'spawn_obstacle_clearance', 0.30))
        )
        for zone in active_zones:
            try:
                clearance = max(
                    0.0, float(zone.get('clearance', default_clearance))
                )
                clearance += max(0.0, float(zone.get('padding', 0.0)))
                surface_distance = signed_distance_to_zone(
                    (x, y), zone, model_poses
                )
            except (TypeError, ValueError):
                # A malformed active footprint must not silently permit a path.
                return False
            if (
                not math.isfinite(surface_distance)
                or surface_distance < clearance - 1e-9
            ):
                return False
        return True

    @staticmethod
    def _heading_offsets():
        """Try a straight departure first, then progressively wider turns."""
        offsets = [0.0]
        step = math.pi / 12.0
        for index in range(1, 12):
            offsets.extend((index * step, -index * step))
        offsets.append(math.pi)
        return offsets

    def _find_path_placement(
        self, mode, radius, anchor_x, anchor_y, leader_yaw,
        model_poses=None,
    ):
        """Place a complete lap at the anchor with the gentlest feasible turn."""
        if model_poses is None:
            model_poses = self._model_pose_snapshot()
        period = self._path_period(mode, radius)
        tangent_step = min(0.001, period / 10000.0)
        active_zones = list(self._active_exclusion_zones())

        # Different points on a closed curve have different footprints around
        # the anchor. This matters when a fleet starts close to an arena wall.
        # Circle phase only changes where its parameter clock starts; after
        # tangent alignment its anchored footprint is identical.
        phase_indices = (0,) if mode == 'circular' else range(24)
        for heading_offset in self._heading_offsets():
            for phase_index in phase_indices:
                phase = period * phase_index / 24.0
                start_x, start_y = self._local_path_point(
                    mode, phase, radius
                )
                for direction in (1.0, -1.0):
                    next_x, next_y = self._local_path_point(
                        mode, phase + direction * tangent_step, radius
                    )
                    tangent = math.atan2(
                        next_y - start_y, next_x - start_x
                    )
                    rotation = leader_yaw + heading_offset - tangent
                    if not self._path_fits_arena_samples(
                        mode,
                        radius,
                        anchor_x,
                        anchor_y,
                        rotation,
                        phase,
                        direction,
                        period,
                        PATH_PLACEMENT_COARSE_SAMPLES,
                        active_zones,
                        model_poses,
                    ):
                        continue
                    # Coarse screening may miss a thin obstacle between two
                    # samples. The normal verifier remains the final authority.
                    if self._path_fits_arena(
                        mode,
                        radius,
                        anchor_x,
                        anchor_y,
                        rotation,
                        phase,
                        direction,
                        model_poses,
                    ):
                        return (
                            phase,
                            direction,
                            rotation,
                            normalize_angle(heading_offset),
                        )
        return None

    def _configure_parametric_path(self, leader_pose, leader_yaw, robot_count):
        """Anchor a safe closed path at the current leader pose."""
        requested = max(0.5, float(self.requested_path_radius))
        minimum = self._minimum_path_radius(self.leader_mode, robot_count)
        effective = max(requested, minimum)

        self.path_radius = effective
        self.path_radius_was_adapted = effective > requested + 1e-9
        self.path_anchor_x = leader_pose.position.x
        self.path_anchor_y = leader_pose.position.y
        self.path_anchor_yaw = leader_yaw
        self.path_anchor_ready = False

        model_poses = self._model_pose_snapshot()
        placement = self._find_path_placement(
            self.leader_mode,
            effective,
            self.path_anchor_x,
            self.path_anchor_y,
            self.path_anchor_yaw,
            model_poses,
        )
        if placement is None:
            self.path_error = (
                "The anchored {mode} path for {count} robots does not fit "
                "inside the {arena:.1f} m arena while clearing its configured "
                "obstacles. Its effective radius is "
                "{radius:.2f} m (requested {requested:.2f} m; minimum "
                "{minimum:.2f} m for the {distance:.2f} m chain). Move the "
                "fleet closer to the centre, reduce the radius/robot count/"
                "follow distance, or use a larger arena."
            ).format(
                mode=self.leader_mode,
                count=robot_count,
                arena=self.arena_size,
                radius=effective,
                requested=requested,
                minimum=minimum,
                distance=max(0, robot_count - 1) * self.follow_distance,
            )
            return False

        (
            self.path_phase,
            self.path_direction,
            self.path_rotation,
            self.path_initial_heading_error,
        ) = placement
        self.path_error = None
        self.path_anchor_ready = True
        self.path_t = 0.0
        if self.path_radius_was_adapted:
            rospy.loginfo(
                "[follow_leader] Increased %s radius from %.2f m to %.2f m "
                "for %d robots",
                self.leader_mode, requested, effective, robot_count,
            )
        return True

    def _generate_leader_path(self, mode, t):
        """
        Return the desired (x, y) position of the leader at parametric time *t*
        for the given *mode*.

        For *manual* and *waypoint* modes this is not used; the leader velocity
        is computed differently (see ``_update_leader``).

        Args:
            mode: One of 'circular', 'square', 'figure8'.
            t: Parametric time (seconds elapsed since start).

        Returns:
            (x, y) tuple -- desired world-frame position.
        """
        return self._anchored_path_point(
            mode,
            t,
            self.path_radius,
            self.path_anchor_x,
            self.path_anchor_y,
            self.path_rotation,
            self.path_phase,
            self.path_direction,
        )

    def _leader_path_velocity(self, mode, t, dt_sample=0.05):
        """
        Numerically differentiate the parametric path to obtain the desired
        velocity vector at time *t*.

        Returns:
            (vx, vy) -- desired world-frame velocity components.
        """
        x0, y0 = self._generate_leader_path(mode, t)
        x1, y1 = self._generate_leader_path(mode, t + dt_sample)
        vx = (x1 - x0) / dt_sample
        vy = (y1 - y0) / dt_sample
        return vx, vy

    # -- leader update ------------------------------------------------------

    def _update_leader(self, dt):
        """
        Compute and return a ``Twist`` command for the leader robot.

        The leader is always ``self.robot_names[0]``.
        """
        cmd = Twist()
        leader = self.robot_names[0]
        pose = self.poses.get(leader)
        if pose is None:
            return cmd

        cur_x = pose.position.x
        cur_y = pose.position.y
        cur_yaw = self.yaws.get(leader, 0.0)

        # ---- Manual mode: just relay the externally published twist --------
        if self.leader_mode == 'manual':
            return self.manual_twist

        # ---- Waypoint mode: go-to-goal with waypoint cycling ---------------
        if self.leader_mode == 'waypoint':
            if not self.waypoints:
                return cmd
            wp = self.waypoints[self.current_waypoint_idx % len(self.waypoints)]
            tx, ty = float(wp[0]), float(wp[1])

            dx = tx - cur_x
            dy = ty - cur_y
            dist = math.hypot(dx, dy)

            if dist < self.waypoint_tolerance:
                self.current_waypoint_idx = (
                    (self.current_waypoint_idx + 1) % len(self.waypoints)
                )
                rospy.loginfo(
                    "[follow_leader] Leader reached waypoint %d, next -> %d",
                    self.current_waypoint_idx - 1,
                    self.current_waypoint_idx % len(self.waypoints),
                )
                return cmd  # zero-velocity for one tick to settle

            angle_to_target = math.atan2(dy, dx)
            angle_error = normalize_angle(angle_to_target - cur_yaw)

            linear = min(dist * 0.8, self._leader_speed_limit()) * math.cos(
                min(abs(angle_error), math.pi / 2)
            )
            angular = self.angular_pids[leader].compute(angle_error, dt)

            cmd.linear.x = max(0.0, linear)  # leader should not reverse
            cmd.angular.z = angular
            return cmd

        # ---- Parametric-path modes (circular, square, figure8) -------------
        if self.leader_mode in ('circular', 'square', 'figure8'):
            tx, ty = self._generate_leader_path(self.leader_mode, self.path_t)
            vx, vy = self._leader_path_velocity(self.leader_mode, self.path_t)

            dx = tx - cur_x
            dy = ty - cur_y
            dist = math.hypot(dx, dy)

            # Blend: feed-forward path velocity + proportional correction
            desired_vx = vx + 0.8 * dx
            desired_vy = vy + 0.8 * dy

            desired_heading = math.atan2(desired_vy, desired_vx)
            angle_error = normalize_angle(desired_heading - cur_yaw)

            speed = math.hypot(desired_vx, desired_vy)
            linear = min(speed, self._leader_speed_limit()) * math.cos(
                min(abs(angle_error), math.pi / 2)
            )
            angular = self.angular_pids[leader].compute(angle_error, dt)

            cmd.linear.x = max(0.0, linear)
            cmd.angular.z = angular
            return cmd

        return cmd

    # -- follower update ----------------------------------------------------

    def _update_follower(self, robot_idx, dt):
        """
        Compute and return a ``Twist`` for the follower at *robot_idx*.

        Each follower tracks its own arc-length point on the leader's travelled
        path.  Sampling by distance, rather than predecessor heading, keeps the
        spacing stable through corners and tight curves.
        """
        cmd = Twist()
        name = self.robot_names[robot_idx]

        pose = self.poses.get(name)
        if pose is None:
            return cmd

        cur_x = pose.position.x
        cur_y = pose.position.y
        cur_yaw = self.yaws.get(name, 0.0)

        try:
            target = self.leader_trace.points_behind([
                robot_idx * self.follow_distance
            ])[0]
        except ValueError:
            return cmd

        target_x = target.x
        target_y = target.y

        predecessor = self.robot_names[robot_idx - 1]
        predecessor_pose = self.poses.get(predecessor)
        if predecessor_pose is None:
            return cmd
        separation_x = cur_x - predecessor_pose.position.x
        separation_y = cur_y - predecessor_pose.position.y
        separation = math.hypot(separation_x, separation_y)
        if 1e-6 < separation < self.follow_distance * 0.85:
            target_x = (
                predecessor_pose.position.x
                + self.follow_distance * separation_x / separation
            )
            target_y = (
                predecessor_pose.position.y
                + self.follow_distance * separation_y / separation
            )
        target_x, target_y = self._clamp_follow_target(target_x, target_y)

        dx = target_x - cur_x
        dy = target_y - cur_y
        dist = math.hypot(dx, dy)

        # Angle from follower to target
        angle_to_target = math.atan2(dy, dx)
        angle_error = normalize_angle(angle_to_target - cur_yaw)

        # Linear velocity: PID on distance error, modulated by heading alignment
        if dist > 0.03:
            raw_linear = self.linear_pids[name].compute(dist, dt)
            alignment_factor = math.cos(min(abs(angle_error), math.pi / 2))
            linear = raw_linear * alignment_factor
            if dist > self.follow_distance * 1.25 and alignment_factor < 0.25:
                linear = max(linear, min(0.045, raw_linear * 0.35))
            linear = max(0.0, min(linear, self.max_linear_vel))
        else:
            linear = 0.0

        # Angular velocity: PID on heading error
        angular = self.angular_pids[name].compute(angle_error, dt)

        # If very close and nearly aligned, damp oscillations
        if dist < 0.08 and abs(angle_error) < 0.1:
            angular *= 0.3

        cmd.linear.x = linear
        cmd.angular.z = angular
        return cmd

    def _clamp_follow_target(self, x, y):
        """Keep follower targets inside the part of the arena robots can use."""
        usable_half = self.arena_size / 2.0 - self.arena_margin - 0.18
        if usable_half <= 0.0:
            return x, y
        return (
            min(usable_half, max(-usable_half, x)),
            min(usable_half, max(-usable_half, y)),
        )

    def _seed_trace_from_chain(self, names, leader_yaw=0.0):
        """Start the trace on the fleet's current shape.

        The first few seconds should straighten the existing chain, not send
        every robot to an invented line behind the leader.  Adding poses from
        tail to head gives the arc-length sampler a useful path immediately;
        any small missing length is extended behind the tail.
        """
        self.leader_trace.clear()
        for name in reversed(names):
            pose = self.poses.get(name)
            if pose is None:
                continue
            self.leader_trace.append(
                pose.position.x,
                pose.position.y,
                self.yaws.get(name, leader_yaw),
            )

        leader_pose = self.poses.get(names[0]) if names else None
        if not self.leader_trace.points and leader_pose is not None:
            self.leader_trace.seed_line(
                leader_pose.position.x,
                leader_pose.position.y,
                leader_yaw,
                0.0,
            )

        needed = max(0, len(names) - 1) * self.follow_distance
        self.leader_trace.ensure_distance(needed)

    def _chain_target_errors(self, names):
        """Return each follower's distance from its initial trace slot."""
        if len(names) <= 1:
            return []
        try:
            targets = self.leader_trace.points_behind(
                index * self.follow_distance
                for index in range(1, len(names))
            )
        except ValueError:
            return []

        errors = []
        for name, target in zip(names[1:], targets):
            pose = self.poses.get(name)
            if pose is None:
                return []
            errors.append(math.hypot(
                target.x - pose.position.x,
                target.y - pose.position.y,
            ))
        return errors

    def _update_chain_assembly(self, names):
        """Hold the leader briefly while all followers take their trace slots."""
        if self.chain_assembled:
            return True

        errors = self._chain_target_errors(names)
        ready = len(names) <= 1 or (
            len(errors) == len(names) - 1
            and max(errors, default=0.0) <= self.chain_settle_tolerance
        )
        if ready:
            self.chain_settle_ticks += 1
        else:
            self.chain_settle_ticks = 0

        if self.chain_settle_ticks >= self.chain_settle_required:
            self.chain_assembled = True
            rospy.loginfo(
                "[follow_leader] Chain assembled; leader is starting"
            )
        return self.chain_assembled

    def _staging_leader_command(self, leader, dt):
        """Drive the leader along its cached route to the safe lap start."""
        route = self.staging_routes.get(leader, [])
        index = self.staging_route_indices.get(leader, 0)
        pose = self.poses.get(leader)
        if pose is None or not route:
            return Twist(), True

        while index < len(route) - 1:
            waypoint = route[index]
            if math.hypot(
                waypoint[0] - pose.position.x,
                waypoint[1] - pose.position.y,
            ) > 0.10:
                break
            index += 1
        self.staging_route_indices[leader] = index

        target_x, target_y = route[index]
        dx = target_x - pose.position.x
        dy = target_y - pose.position.y
        distance = math.hypot(dx, dy)
        if index == len(route) - 1 and distance <= 0.10:
            return Twist(), True

        heading = math.atan2(dy, dx)
        heading_error = normalize_angle(
            heading - self.yaws.get(leader, 0.0)
        )
        command = Twist()
        command.linear.x = min(
            self._leader_speed_limit(), 0.8 * distance
        ) * max(0.0, math.cos(heading_error))
        command.angular.z = self.angular_pids[leader].compute(
            heading_error, dt
        )
        return command, False

    def _staging_alignment_command(self, leader, dt):
        """Rotate smoothly onto the first lap tangent before time advances."""
        velocity_x, velocity_y = self._leader_path_velocity(
            self.leader_mode, 0.0, dt_sample=0.01
        )
        desired_yaw = math.atan2(velocity_y, velocity_x)
        error = normalize_angle(desired_yaw - self.yaws.get(leader, 0.0))
        command = Twist()
        if abs(error) > 0.06:
            command.angular.z = self.angular_pids[leader].compute(error, dt)
        return command, abs(error) <= 0.06

    def _complete_staging_if_ready(self, names, aligned):
        """Revalidate the lap and hand control to normal following."""
        if not aligned:
            self.staging_settle_ticks = 0
            return False
        self.staging_settle_ticks += 1
        if self.staging_settle_ticks < self.chain_settle_required:
            return False

        model_poses = self._model_pose_snapshot()
        if not self._path_fits_arena(
            self.leader_mode,
            self.path_radius,
            self.path_anchor_x,
            self.path_anchor_y,
            self.path_rotation,
            self.path_phase,
            self.path_direction,
            model_poses,
        ):
            rospy.logwarn(
                "[follow_leader] Scene changed during staging; replanning"
            )
            self._cancel_path_plan_locked(clear_staging=True)
            self.path_anchor_ready = False
            self.chain_assembled = False
            self.chain_settle_ticks = 0
            self.leader_speed_scale = 0.0
            self._begin_path_planning_locked(names)
            return False

        self.path_staging = False
        self.staging_phase = None
        self.path_anchor_ready = True
        self.path_t = 0.0
        self.leader_speed_scale = 0.0
        for pid in self.linear_pids.values():
            pid.reset()
        for pid in self.angular_pids.values():
            pid.reset()
        rospy.loginfo(
            "[follow_leader] Chain staged at safe %s path start",
            self.leader_mode,
        )
        return True

    def _control_path_staging(self, names, dt):
        """Lead the assembled chain to a relocated closed-path start."""
        leader = names[0]
        leader_pose = self.poses.get(leader)
        if leader_pose is None:
            self._stop_all()
            self._publish_status([])
            return

        needed_trace = max(0, len(names) - 1) * self.follow_distance
        self.leader_trace.append(
            leader_pose.position.x,
            leader_pose.position.y,
            self.yaws.get(leader, 0.0),
        )
        self.leader_trace.ensure_distance(needed_trace)
        self.leader_trace.trim(
            needed_trace + max(1.0, self.follow_distance * 2.0)
        )
        chain_ready = self._update_chain_assembly(names)

        route_complete = False
        leader_command = Twist()
        if not chain_ready:
            self.staging_phase = 'assembling'
        elif self.staging_phase in ('assembling', 'relocating'):
            self.staging_phase = 'relocating'
            leader_command, route_complete = self._staging_leader_command(
                leader, dt
            )
            if route_complete:
                self.staging_phase = 'settling'

        if self.staging_phase == 'settling':
            spacing_errors = []
            for predecessor, follower in zip(names, names[1:]):
                predecessor_pose = self.poses.get(predecessor)
                follower_pose = self.poses.get(follower)
                if predecessor_pose is None or follower_pose is None:
                    spacing_errors = []
                    break
                gap = math.hypot(
                    predecessor_pose.position.x - follower_pose.position.x,
                    predecessor_pose.position.y - follower_pose.position.y,
                )
                spacing_errors.append(abs(gap - self.follow_distance))
            settled = len(names) <= 1 or (
                len(spacing_errors) == len(names) - 1
                and max(spacing_errors, default=0.0)
                <= self.chain_settle_tolerance
            )
            if settled:
                self.staging_settle_ticks += 1
            else:
                self.staging_settle_ticks = 0
            if self.staging_settle_ticks >= self.chain_settle_required:
                self.staging_phase = 'aligning'
                self.staging_settle_ticks = 0

        aligned = False
        if self.staging_phase == 'aligning':
            leader_command, aligned = self._staging_alignment_command(
                leader, dt
            )

        robot_positions = [
            (
                name,
                Point(
                    x=self.poses[name].position.x,
                    y=self.poses[name].position.y,
                    z=0.0,
                ),
            )
            for name in names if self.poses.get(name) is not None
        ]
        for name in names:
            avoidance = self.avoidance.get(name)
            if avoidance is not None:
                avoidance.update_robot_positions(robot_positions)

        entries = []
        for index, name in enumerate(names):
            desired = leader_command if index == 0 else self._update_follower(
                index, dt
            )
            avoidance = self.avoidance.get(name)
            safe = (
                avoidance.apply_avoidance(desired)
                if avoidance is not None else desired
            )
            publisher = self.cmd_pubs.get(name)
            if publisher is not None:
                publisher.publish(safe)
            pose = self.poses.get(name)
            if pose is not None:
                entries.append({
                    'name': name,
                    'role': 'leader' if index == 0 else 'follower',
                    'chain_index': index,
                    'x': round(pose.position.x, 4),
                    'y': round(pose.position.y, 4),
                    'yaw': round(self.yaws.get(name, 0.0), 4),
                    'linear_vel': round(safe.linear.x, 4),
                    'angular_vel': round(safe.angular.z, 4),
                })

        if self.staging_phase == 'aligning':
            self._complete_staging_if_ready(names, aligned)
        self._publish_status(entries)
        self._publish_markers(names)

    @staticmethod
    def _scale_command(command, scale):
        """Apply the leader's smooth start ramp to a velocity command."""
        scaled = Twist()
        scaled.linear.x = command.linear.x * scale
        scaled.linear.y = command.linear.y * scale
        scaled.linear.z = command.linear.z * scale
        scaled.angular.x = command.angular.x * scale
        scaled.angular.y = command.angular.y * scale
        scaled.angular.z = command.angular.z * scale
        return scaled

    # -- main control loop --------------------------------------------------

    def _control_loop(self, event):
        with self.command_lock:
            if (
                not self.is_active
                or self.is_paused
                or self.emergency_stop_active
            ):
                return
            stale = self._stale_odometry(self.robot_names)
            if stale:
                self.stale_odometry = stale
                self.path_error = (
                    "Odometry became stale or unavailable for: "
                    + ", ".join(stale)
                )
            if self.path_error:
                self._stop_all()
                self._publish_status([])
                return
            self._control_step(event)

    def _stale_odometry(self, names, now=None):
        """List robots whose pose has not refreshed within the safety window."""
        started_at = getattr(self, 'task_started_at', None)
        if started_at is None:
            return []
        if now is None:
            now = time.monotonic()
        timeout = getattr(self, 'odom_timeout_wall_s', 0.75)
        received = getattr(self, 'odom_received_at', {})
        stale = []
        for name in names:
            stamp = received.get(name)
            reference = started_at if stamp is None else stamp
            if now < reference or now - reference > timeout:
                stale.append(name)
        return stale

    def _control_step(self, event):
        """
        Timer callback executing at 20 Hz.

        1. Update inter-robot positions for obstacle avoidance.
        2. Compute leader velocity.
        3. Compute each follower velocity.
        4. Pass all commands through obstacle avoidance.
        5. Publish safe commands.
        6. Publish status and visualisation.
        """
        with self.lock:
            names = list(self.robot_names)

        if not self.is_active or len(names) == 0:
            return

        dt = self.dt

        leader_pose = self.poses.get(names[0])
        if (
            self.leader_mode in PARAMETRIC_MODES
            and not self.path_anchor_ready
        ):
            if not self._chain_poses_ready(names):
                self._stop_all()
                self._publish_status([])
                return
            if self.path_staging:
                self._control_path_staging(names, dt)
                return
            if not self.path_planning:
                with self.lock:
                    self.robot_names = self._spatial_chain_order(
                        self.robot_names
                    )
                    names = list(self.robot_names)
                self._begin_path_planning_locked(names)
            if not self.path_anchor_ready:
                self._stop_all()
                self._publish_status([])
                return
            leader_pose = self.poses.get(names[0])

        if leader_pose is not None:
            leader_yaw = self.yaws.get(names[0], 0.0)
            needed_trace = max(0, len(names) - 1) * self.follow_distance
            if not self.leader_trace.points:
                self._seed_trace_from_chain(names, leader_yaw)
            else:
                self.leader_trace.append(
                    leader_pose.position.x,
                    leader_pose.position.y,
                    leader_yaw,
                )
                self.leader_trace.ensure_distance(needed_trace)
            self.leader_trace.trim(needed_trace + max(1.0, self.follow_distance * 2.0))

        chain_ready = self._update_chain_assembly(names)
        if chain_ready:
            self.leader_speed_scale = min(
                1.0,
                self.leader_speed_scale + self.leader_ramp_step,
            )
        else:
            self.leader_speed_scale = 0.0

        # -- 1. Collect current positions for inter-robot avoidance ----------
        robot_positions = []
        for name in names:
            pose = self.poses.get(name)
            if pose is not None:
                robot_positions.append((
                    name,
                    Point(x=pose.position.x, y=pose.position.y, z=0.0),
                ))

        for name in names:
            if name in self.avoidance:
                self.avoidance[name].update_robot_positions(robot_positions)

        # -- 2-4. Compute & publish commands for every robot -----------------
        status_entries = []

        for idx, name in enumerate(names):
            if idx == 0:
                if chain_ready:
                    desired_cmd = self._scale_command(
                        self._update_leader(dt), self.leader_speed_scale
                    )
                else:
                    desired_cmd = Twist()
                    avoidance = self.avoidance.get(name)
                    if avoidance is not None:
                        avoidance.reset_motion()
            else:
                desired_cmd = self._update_follower(idx, dt)

            # Obstacle avoidance filter
            if name in self.avoidance:
                safe_cmd = self.avoidance[name].apply_avoidance(desired_cmd)
            else:
                safe_cmd = desired_cmd

            # Publish
            pub = self.cmd_pubs.get(name)
            if pub is not None:
                pub.publish(safe_cmd)

            # Gather status info
            pose = self.poses.get(name)
            if pose is not None:
                status_entries.append({
                    'name': name,
                    'role': 'leader' if idx == 0 else 'follower',
                    'chain_index': idx,
                    'x': round(pose.position.x, 4),
                    'y': round(pose.position.y, 4),
                    'yaw': round(self.yaws.get(name, 0.0), 4),
                    'linear_vel': round(safe_cmd.linear.x, 4),
                    'angular_vel': round(safe_cmd.angular.z, 4),
                })

        # Advance parametric path time
        if chain_ready:
            self.path_t += dt * self.leader_speed_scale

        # -- 5. Publish status -----------------------------------------------
        self._publish_status(status_entries)

        # -- 6. Publish visualisation markers --------------------------------
        self._publish_markers(names)

    # -- status publishing --------------------------------------------------

    def _publish_status(self, entries):
        """Publish a JSON status message on ``/follow_leader/status``."""
        planning_wall_s = getattr(self, 'path_planning_wall_s', 0.0)
        planning_started_at = getattr(self, 'path_planning_started_at', None)
        if self.path_planning and planning_started_at is not None:
            planning_wall_s = max(
                0.0, time.monotonic() - planning_started_at
            )
        if self.path_error:
            state = 'failed'
        elif self.is_paused:
            state = 'paused'
        elif self.is_active and self.path_planning:
            state = 'planning_path'
        elif self.is_active and self.path_staging:
            state = 'staging_path'
        elif (
            self.is_active
            and self.leader_mode in PARAMETRIC_MODES
            and not self.path_anchor_ready
        ):
            state = 'waiting_for_odometry'
        elif self.is_active:
            state = 'running'
        else:
            state = 'stopped'

        status = {
            'task_id': self.current_task_id,
            'active': self.is_active and not self.path_error,
            'paused': self.is_paused,
            'state': state,
            'leader_mode': self.leader_mode,
            'follow_distance': self.follow_distance,
            'requested_radius': round(self.requested_path_radius, 4),
            'effective_radius': round(self.path_radius, 4),
            'radius_adapted': self.path_radius_was_adapted,
            'path_relocated': self.path_relocated,
            'setup_phase': (
                self.staging_phase
                if self.path_staging else (
                    'planning' if self.path_planning else None
                )
            ),
            'planning_wall_s': round(planning_wall_s, 4),
            'initial_heading_error': round(
                self.path_initial_heading_error, 4
            ),
            'chain_ready': self.chain_assembled,
            'leader_speed_scale': round(self.leader_speed_scale, 3),
            'trace_length': round(self.leader_trace.span, 3),
            'path_t': round(self.path_t, 3),
            'robots': entries,
            'stale_odometry': list(getattr(self, 'stale_odometry', [])),
        }
        status.update(self._path_lap_progress())
        if self.path_anchor_ready:
            status['path_anchor'] = {
                'x': round(self.path_anchor_x, 4),
                'y': round(self.path_anchor_y, 4),
                'yaw': round(self.path_anchor_yaw, 4),
            }
        if self.path_error:
            status['error'] = self.path_error
        self.status_pub.publish(String(data=json.dumps(status)))

    # -- RViz visualisation -------------------------------------------------

    def _publish_markers(self, names):
        """
        Publish an RViz MarkerArray:
        - A LINE_STRIP connecting all robots in chain order.
        - A SPHERE for the leader.
        - Small SPHEREs for each follower.
        """
        marker_array = MarkerArray()

        # -- Chain line -----------------------------------------------------
        if len(names) >= 2:
            line = Marker()
            line.header.frame_id = 'map'
            line.header.stamp = rospy.Time.now()
            line.ns = 'follow_leader_chain'
            line.id = 0
            line.type = Marker.LINE_STRIP
            line.action = Marker.ADD
            line.scale.x = 0.04
            line.color.r = 0.2
            line.color.g = 1.0
            line.color.b = 0.2
            line.color.a = 0.9
            line.lifetime = rospy.Duration(0.15)

            for name in names:
                pose = self.poses.get(name)
                if pose is not None:
                    p = Point()
                    p.x = pose.position.x
                    p.y = pose.position.y
                    p.z = 0.08
                    line.points.append(p)

            if len(line.points) >= 2:
                marker_array.markers.append(line)

        # -- Leader sphere --------------------------------------------------
        if names:
            leader_pose = self.poses.get(names[0])
            if leader_pose is not None:
                sphere = Marker()
                sphere.header.frame_id = 'map'
                sphere.header.stamp = rospy.Time.now()
                sphere.ns = 'follow_leader_head'
                sphere.id = 1
                sphere.type = Marker.SPHERE
                sphere.action = Marker.ADD
                sphere.pose.position.x = leader_pose.position.x
                sphere.pose.position.y = leader_pose.position.y
                sphere.pose.position.z = 0.35
                sphere.pose.orientation.w = 1.0
                sphere.scale.x = 0.25
                sphere.scale.y = 0.25
                sphere.scale.z = 0.25
                sphere.color.r = 1.0
                sphere.color.g = 0.3
                sphere.color.b = 0.0
                sphere.color.a = 0.9
                sphere.lifetime = rospy.Duration(0.15)
                marker_array.markers.append(sphere)

        # -- Follower spheres -----------------------------------------------
        for idx in range(1, len(names)):
            pose = self.poses.get(names[idx])
            if pose is None:
                continue
            sphere = Marker()
            sphere.header.frame_id = 'map'
            sphere.header.stamp = rospy.Time.now()
            sphere.ns = 'follow_leader_followers'
            sphere.id = 100 + idx
            sphere.type = Marker.SPHERE
            sphere.action = Marker.ADD
            sphere.pose.position.x = pose.position.x
            sphere.pose.position.y = pose.position.y
            sphere.pose.position.z = 0.30
            sphere.pose.orientation.w = 1.0
            sphere.scale.x = 0.15
            sphere.scale.y = 0.15
            sphere.scale.z = 0.15
            # Gradient colour along the chain
            frac = idx / max(len(names) - 1, 1)
            sphere.color.r = 0.0
            sphere.color.g = 0.5 + 0.5 * (1.0 - frac)
            sphere.color.b = 0.3 + 0.7 * frac
            sphere.color.a = 0.85
            sphere.lifetime = rospy.Duration(0.15)
            marker_array.markers.append(sphere)

        if marker_array.markers:
            self.marker_pub.publish(marker_array)

    # -- utilities ----------------------------------------------------------

    def _stop_all(self):
        """Send zero-velocity to every robot."""
        stop = Twist()
        with self.lock:
            pubs = list(self.cmd_pubs.values())
        for pub in pubs:
            pub.publish(stop)


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------

if __name__ == '__main__':
    try:
        node = FollowTheLeader()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
