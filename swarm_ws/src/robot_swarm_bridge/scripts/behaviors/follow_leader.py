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
from typing import Dict, List, Optional, Tuple

# ROS messages
from geometry_msgs.msg import Twist, Point, Pose
from nav_msgs.msg import Odometry
from std_msgs.msg import String, Bool
import random as rng
from visualization_msgs.msg import Marker, MarkerArray

# Import ObstacleAvoidance from core
sys.path.append(os.path.join(os.path.dirname(__file__), '..'))
from core.obstacle_avoidance import ObstacleAvoidance
from utils.robot_ids import sort_robot_ids
from robot_swarm_bridge.algorithms.path_trace import ArcLengthTrace


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

    Chain order is determined by the sorted robot namespace list received on
    ``/fleet/robot_list``.  The first robot in the sorted list is the leader;
    every subsequent robot follows the one immediately before it.

    Leader motion is generated internally according to the chosen *mode*
    (circular, square, figure8, waypoint, manual).  Follower motion uses a
    distance-based playback of the leader's path, combined with a PID tracking
    loop. Every follower receives a different point on the same path.

    All velocity commands pass through an :class:`ObstacleAvoidance` filter
    before being published.
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
        self.path_radius = rospy.get_param('~path_radius', 2.0)
        self.waypoints = rospy.get_param(
            '~waypoints', [[2, 0], [2, 2], [0, 2], [0, 0]]
        )

        # ---- Derived / internal constants ----------------------------------
        self.control_rate = 20.0  # Hz
        self.dt = 1.0 / self.control_rate
        self.waypoint_tolerance = 0.15  # metres

        # ---- Leader path state ---------------------------------------------
        self.path_t = 0.0  # parametric time, always increasing while active
        self.current_waypoint_idx = 0

        # ---- Per-robot data (keyed by namespace string) --------------------
        self.robot_names = []  # sorted list, index 0 = leader
        self.poses = {}        # str -> Pose
        self.yaws = {}         # str -> float  (cache to avoid repeated quat conversion)
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
                old_leader = self.robot_names[0] if self.robot_names else None
                old_set = set(self.robot_names)
                new_set = set(new_names_sorted)

                if not new_set and (self.is_active or self.is_paused):
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

        rospy.loginfo(
            "[follow_leader] Fleet synced: %s", ', '.join(new_names_sorted)
        )

    def _add_robot(self, name):
        """Register publishers, subscribers, PID controllers, etc. for *name*."""
        self.poses[name] = None
        self.yaws[name] = 0.0

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
            avoidance = self.avoidance.get(robot_name)

        # Keep obstacle avoidance module in sync
        if avoidance is not None:
            avoidance.set_position(pose.position.x, pose.position.y, yaw)

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
                    self.path_radius = max(0.5, float(config['radius']))
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
            self.is_active = True
            self.is_paused = False
            self.path_t = 0.0
            self.current_waypoint_idx = 0
            self.leader_trace.clear()
            with self.lock:
                names = list(self.robot_names)
                if names:
                    pose = self.poses.get(names[0])
                    if pose is not None:
                        self.leader_trace.seed_line(
                            pose.position.x,
                            pose.position.y,
                            self.yaws.get(names[0], 0.0),
                            max(0, len(names) - 1) * self.follow_distance,
                        )
                for pid in self.linear_pids.values():
                    pid.reset()
                for pid in self.angular_pids.values():
                    pid.reset()
        rospy.loginfo(
            "[follow_leader] Behaviour STARTED: mode=%s, waypoints=%d, radius=%.2f",
            self.leader_mode, len(self.waypoints), self.path_radius,
        )

    def _stop_cb(self, msg):
        with self.command_lock:
            if not self._task_command_matches(msg):
                return
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
                self.is_active = False
                self.is_paused = False
                self._stop_all()
                rospy.logwarn("[follow_leader] Emergency stop latched")

    # -- leader path generation ---------------------------------------------

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
        r = self.path_radius

        if mode == 'circular':
            omega = self.max_linear_vel / r  # constant arc-length speed
            x = r * math.cos(omega * t)
            y = r * math.sin(omega * t)
            return x, y

        elif mode == 'square':
            # Perimeter traversal at constant speed
            side = r * 2.0
            perimeter = 4.0 * side
            speed = self.max_linear_vel * 0.8
            dist = (speed * t) % perimeter  # distance along perimeter

            if dist < side:
                x = dist
                y = 0.0
            elif dist < 2 * side:
                x = side
                y = dist - side
            elif dist < 3 * side:
                x = side - (dist - 2 * side)
                y = side
            else:
                x = 0.0
                y = side - (dist - 3 * side)

            # Centre the square at origin
            x -= side / 2.0
            y -= side / 2.0
            return x, y

        elif mode == 'figure8':
            # Lemniscate of Bernoulli (parametric):
            #   x = a * sin(t)
            #   y = a * sin(t) * cos(t) = (a/2) * sin(2t)
            omega = self.max_linear_vel / r * 0.5
            x = r * math.sin(omega * t)
            y = r * math.sin(omega * t) * math.cos(omega * t)
            return x, y

        # Fallback
        return 0.0, 0.0

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

            linear = min(dist * 0.8, self.max_linear_vel) * math.cos(
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
            linear = min(speed, self.max_linear_vel) * math.cos(
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

        Each follower tracks a point farther back on the leader's path.
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

    # -- main control loop --------------------------------------------------

    def _control_loop(self, event):
        with self.command_lock:
            if (
                not self.is_active
                or self.is_paused
                or self.emergency_stop_active
            ):
                return
            self._control_step(event)

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
        if leader_pose is not None:
            leader_yaw = self.yaws.get(names[0], 0.0)
            needed_trace = max(0, len(names) - 1) * self.follow_distance
            if not self.leader_trace.points:
                self.leader_trace.seed_line(
                    leader_pose.position.x,
                    leader_pose.position.y,
                    leader_yaw,
                    needed_trace,
                )
            else:
                self.leader_trace.append(
                    leader_pose.position.x,
                    leader_pose.position.y,
                    leader_yaw,
                )
                self.leader_trace.ensure_distance(needed_trace)
            self.leader_trace.trim(needed_trace + max(1.0, self.follow_distance * 2.0))

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
                desired_cmd = self._update_leader(dt)
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
        self.path_t += dt

        # -- 5. Publish status -----------------------------------------------
        self._publish_status(status_entries)

        # -- 6. Publish visualisation markers --------------------------------
        self._publish_markers(names)

    # -- status publishing --------------------------------------------------

    def _publish_status(self, entries):
        """Publish a JSON status message on ``/follow_leader/status``."""
        status = {
            'task_id': self.current_task_id,
            'active': self.is_active,
            'paused': self.is_paused,
            'leader_mode': self.leader_mode,
            'follow_distance': self.follow_distance,
            'trace_length': round(self.leader_trace.span, 3),
            'path_t': round(self.path_t, 3),
            'robots': entries,
        }
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
