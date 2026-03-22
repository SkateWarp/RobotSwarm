#!/usr/bin/env python3
"""
Moving Target Tracking Swarm Behavior Controller for TurtleBot3 Waffle in Gazebo

Implements multi-robot pursuit and surrounding of a moving target using three
configurable strategies:

  - **pursuit**: All robots converge on the target's predicted position using
    proportional navigation (PN). Each robot steers toward where the target
    will be, not where it currently is.

  - **surround**: Robots spread evenly around the target at a configurable
    standoff radius. Uses a distributed slot-assignment scheme so each robot
    converges on a unique angular position, creating an encirclement.

  - **intercept**: Robots predict the target's future trajectory via linear
    extrapolation and navigate to cut-off points ahead of the target, forming
    a net that tightens as the target approaches.

The target position is read from ``/gazebo/model_states`` (simulation) or from
an external topic ``/target/pose`` (real-world / custom tracker). The node
automatically falls back to ``/target/pose`` if the configured model name is
not found in Gazebo.

Integrates with ObstacleAvoidance for safe navigation and publishes RViz
markers and JSON status for the frontend.
"""

import rospy
import math
import json
import os
import sys
import threading
from collections import deque
from typing import Dict, List, Optional, Tuple

# ROS messages
from geometry_msgs.msg import Twist, Point, Pose, PoseStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import String, Empty
from gazebo_msgs.msg import ModelStates
from visualization_msgs.msg import Marker, MarkerArray

# Import ObstacleAvoidance from core
sys.path.append(os.path.join(os.path.dirname(__file__), '..'))
from core.obstacle_avoidance import ObstacleAvoidance


# ---------------------------------------------------------------------------
# PID Controller
# ---------------------------------------------------------------------------

class PIDController:
    """Discrete PID controller with anti-windup clamping."""

    def __init__(self, kp, ki, kd, max_output, min_output=None):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.max_output = max_output
        self.min_output = min_output if min_output is not None else -max_output
        self._integral = 0.0
        self._prev_error = 0.0

    def compute(self, error, dt):
        if dt <= 0.0:
            return 0.0
        p_term = self.kp * error
        self._integral += error * dt
        integral_limit = self.max_output / max(self.ki, 1e-6)
        self._integral = max(-integral_limit, min(integral_limit, self._integral))
        i_term = self.ki * self._integral
        d_term = self.kd * (error - self._prev_error) / dt
        self._prev_error = error
        output = p_term + i_term + d_term
        return max(self.min_output, min(self.max_output, output))

    def reset(self):
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
# Target Tracking Node
# ---------------------------------------------------------------------------

class TargetTracking:
    """
    ROS node implementing multi-robot moving target tracking.

    Strategies:
      - pursuit:   Proportional navigation toward predicted target position.
      - surround:  Distributed encirclement at a configurable standoff radius.
      - intercept: Robots navigate to predicted future positions of the target.
    """

    def __init__(self):
        rospy.init_node('target_tracking', anonymous=False)

        # ---- ROS parameters ------------------------------------------------
        self.strategy = rospy.get_param('~strategy', 'surround')
        self.target_model_name = rospy.get_param('~target_model', 'target_box')
        self.standoff_radius = rospy.get_param('~standoff_radius', 1.5)
        self.max_linear_vel = rospy.get_param('~max_linear_vel', 0.22)
        self.max_angular_vel = rospy.get_param('~max_angular_vel', 1.5)
        self.prediction_horizon = rospy.get_param('~prediction_horizon', 1.0)
        self.robot_count = rospy.get_param('~robot_count', 5)

        # ---- Derived / internal constants ----------------------------------
        self.control_rate = 20.0  # Hz
        self.dt = 1.0 / self.control_rate
        self.velocity_history_seconds = 5.0
        self.history_maxlen = int(self.control_rate * self.velocity_history_seconds)

        # ---- Target state --------------------------------------------------
        self.target_pose = None       # (x, y)
        self.target_yaw = 0.0
        self.target_velocity = (0.0, 0.0)  # estimated (vx, vy)
        self.target_history = deque(maxlen=self.history_maxlen)
        self.target_lock = threading.Lock()

        # ---- Per-robot data ------------------------------------------------
        self.robot_names = []
        self.poses = {}
        self.yaws = {}
        self.linear_pids = {}
        self.angular_pids = {}
        self.avoidance = {}
        self.cmd_pubs = {}
        self.odom_subs = {}

        # ---- Behaviour state -----------------------------------------------
        self.is_active = False
        self.lock = threading.Lock()

        # ---- Publishers / subscribers --------------------------------------
        self.status_pub = rospy.Publisher(
            '/target_tracking/status', String, queue_size=1
        )
        self.marker_pub = rospy.Publisher(
            '/target_tracking/markers', MarkerArray, queue_size=1
        )

        rospy.Subscriber(
            '/fleet/robot_list', String, self._fleet_list_cb, queue_size=1
        )
        rospy.Subscriber(
            '/target_tracking/start', String, self._start_cb, queue_size=1
        )
        rospy.Subscriber(
            '/target_tracking/stop', Empty, self._stop_cb, queue_size=1
        )

        # Target position sources
        rospy.Subscriber(
            '/gazebo/model_states', ModelStates,
            self._model_states_cb, queue_size=1
        )
        rospy.Subscriber(
            '/target/pose', PoseStamped,
            self._target_pose_cb, queue_size=1
        )

        # ---- Bootstrap default fleet ---------------------------------------
        self._bootstrap_default_fleet()

        # ---- Control timer -------------------------------------------------
        self.timer = rospy.Timer(
            rospy.Duration(self.dt), self._control_loop
        )

        rospy.loginfo(
            "[target_tracking] Initialised: strategy=%s, standoff=%.2f, "
            "prediction=%.2fs, target_model=%s",
            self.strategy, self.standoff_radius,
            self.prediction_horizon, self.target_model_name,
        )

    # -- fleet management ---------------------------------------------------

    def _bootstrap_default_fleet(self):
        names = ['tb3_{}'.format(i) for i in range(self.robot_count)]
        self._sync_fleet(names)

    def _fleet_list_cb(self, msg):
        if not msg.data.strip():
            return
        names = [n.strip() for n in msg.data.split(',') if n.strip()]
        self._sync_fleet(names)

    def _sync_fleet(self, new_names):
        new_names_sorted = sorted(set(new_names))
        with self.lock:
            old_set = set(self.robot_names)
            new_set = set(new_names_sorted)
            for name in new_set - old_set:
                self._add_robot(name)
            for name in old_set - new_set:
                self._remove_robot(name)
            self.robot_names = new_names_sorted
        rospy.loginfo(
            "[target_tracking] Fleet synced: %s", ', '.join(new_names_sorted)
        )

    def _add_robot(self, name):
        self.poses[name] = None
        self.yaws[name] = 0.0

        self.linear_pids[name] = PIDController(
            kp=1.2, ki=0.0, kd=0.3,
            max_output=self.max_linear_vel,
        )
        self.angular_pids[name] = PIDController(
            kp=2.5, ki=0.0, kd=0.5,
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
        rospy.loginfo("[target_tracking] Added robot: %s", name)

    def _remove_robot(self, name):
        if name in self.cmd_pubs:
            self.cmd_pubs[name].publish(Twist())
        if name in self.odom_subs:
            self.odom_subs[name].unregister()
        for d in (self.poses, self.yaws, self.linear_pids, self.angular_pids,
                  self.cmd_pubs, self.odom_subs, self.avoidance):
            d.pop(name, None)
        rospy.loginfo("[target_tracking] Removed robot: %s", name)

    # -- callbacks ----------------------------------------------------------

    def _odom_cb(self, msg, robot_name):
        pose = msg.pose.pose
        yaw = yaw_from_quaternion(pose.orientation)
        self.poses[robot_name] = pose
        self.yaws[robot_name] = yaw
        if robot_name in self.avoidance:
            self.avoidance[robot_name].set_position(
                pose.position.x, pose.position.y, yaw
            )

    def _model_states_cb(self, msg):
        """Extract target position from Gazebo model states."""
        try:
            idx = msg.name.index(self.target_model_name)
        except ValueError:
            return  # target model not found, rely on /target/pose

        pose = msg.pose[idx]
        now = rospy.Time.now()
        x, y = pose.position.x, pose.position.y
        yaw = yaw_from_quaternion(pose.orientation)

        with self.target_lock:
            self._update_target(x, y, yaw, now)

    def _target_pose_cb(self, msg):
        """Accept target position from an external tracker."""
        x = msg.pose.position.x
        y = msg.pose.position.y
        yaw = yaw_from_quaternion(msg.pose.orientation)
        now = msg.header.stamp if msg.header.stamp.to_sec() > 0 else rospy.Time.now()

        with self.target_lock:
            self._update_target(x, y, yaw, now)

    def _update_target(self, x, y, yaw, stamp):
        """Update target state and estimate velocity from position history."""
        prev_pose = self.target_pose
        prev_stamp = self.target_history[-1][0] if self.target_history else None

        self.target_pose = (x, y)
        self.target_yaw = yaw
        self.target_history.append((stamp, x, y))

        # Estimate velocity using recent history (sliding window average)
        if len(self.target_history) >= 2:
            # Use samples spanning ~0.5s for smoother estimates
            window_duration = 0.5
            recent = self.target_history[-1]
            older = None
            for entry in reversed(self.target_history):
                dt = (recent[0] - entry[0]).to_sec()
                if dt >= window_duration:
                    older = entry
                    break
            if older is None:
                older = self.target_history[0]

            dt = (recent[0] - older[0]).to_sec()
            if dt > 0.01:
                vx = (recent[1] - older[1]) / dt
                vy = (recent[2] - older[2]) / dt
                # Exponential smoothing on velocity estimate
                alpha = 0.3
                self.target_velocity = (
                    alpha * vx + (1 - alpha) * self.target_velocity[0],
                    alpha * vy + (1 - alpha) * self.target_velocity[1],
                )

    def _start_cb(self, msg):
        try:
            config = json.loads(msg.data) if msg.data else {}
        except (json.JSONDecodeError, AttributeError):
            config = {}

        if 'strategy' in config:
            self.strategy = config['strategy']
        if 'standoff_radius' in config:
            self.standoff_radius = float(config['standoff_radius'])
        if 'prediction_horizon' in config:
            self.prediction_horizon = float(config['prediction_horizon'])
        if 'target_model' in config:
            self.target_model_name = config['target_model']

        self.is_active = True
        with self.lock:
            for pid in self.linear_pids.values():
                pid.reset()
            for pid in self.angular_pids.values():
                pid.reset()
        rospy.loginfo(
            "[target_tracking] STARTED: strategy=%s, standoff=%.2f, "
            "prediction=%.2fs",
            self.strategy, self.standoff_radius, self.prediction_horizon,
        )

    def _stop_cb(self, _msg):
        self.is_active = False
        self._stop_all()
        rospy.loginfo("[target_tracking] STOPPED")

    # -- target prediction --------------------------------------------------

    def _predict_target_position(self, horizon):
        """
        Predict the target's position *horizon* seconds into the future
        using linear extrapolation of the estimated velocity.

        Returns:
            (x, y) predicted position, or current position if unknown.
        """
        if self.target_pose is None:
            return None

        tx, ty = self.target_pose
        vx, vy = self.target_velocity
        return (tx + vx * horizon, ty + vy * horizon)

    # -- strategy: pursuit --------------------------------------------------

    def _compute_pursuit(self, robot_name, dt):
        """
        Proportional navigation: steer toward the predicted future position
        of the target. The prediction horizon scales with distance so that
        far-away robots lead the target more aggressively.
        """
        cmd = Twist()
        pose = self.poses.get(robot_name)
        if pose is None or self.target_pose is None:
            return cmd

        cur_x = pose.position.x
        cur_y = pose.position.y
        cur_yaw = self.yaws.get(robot_name, 0.0)

        # Scale prediction horizon with distance
        tx, ty = self.target_pose
        dist_to_target = math.hypot(tx - cur_x, ty - cur_y)
        adaptive_horizon = min(
            self.prediction_horizon * (dist_to_target / 3.0),
            self.prediction_horizon * 2.0,
        )

        predicted = self._predict_target_position(adaptive_horizon)
        if predicted is None:
            return cmd
        px, py = predicted

        dx = px - cur_x
        dy = py - cur_y
        dist = math.hypot(dx, dy)

        angle_to_target = math.atan2(dy, dx)
        angle_error = normalize_angle(angle_to_target - cur_yaw)

        # Slow down as we approach the target
        if dist > 0.3:
            raw_linear = self.linear_pids[robot_name].compute(dist, dt)
            alignment = math.cos(min(abs(angle_error), math.pi / 2))
            linear = max(0.0, min(raw_linear * alignment, self.max_linear_vel))
        else:
            # Close enough — creep forward to maintain contact
            linear = min(0.05, dist * 0.3)

        angular = self.angular_pids[robot_name].compute(angle_error, dt)

        cmd.linear.x = linear
        cmd.angular.z = angular
        return cmd

    # -- strategy: surround -------------------------------------------------

    def _compute_surround(self, robot_name, robot_idx, total_robots, dt):
        """
        Distributed encirclement: each robot is assigned an angular slot
        around the target (evenly spaced). The goal point for each robot is
        on a circle of radius ``standoff_radius`` centred at the predicted
        target position. Slot assignment rotates with the target's heading
        so the formation follows naturally.
        """
        cmd = Twist()
        pose = self.poses.get(robot_name)
        if pose is None or self.target_pose is None:
            return cmd

        cur_x = pose.position.x
        cur_y = pose.position.y
        cur_yaw = self.yaws.get(robot_name, 0.0)

        # Predict target position slightly into the future
        predicted = self._predict_target_position(self.prediction_horizon * 0.5)
        if predicted is None:
            return cmd
        px, py = predicted

        # Compute the assigned angular slot
        slot_angle = (2.0 * math.pi * robot_idx / max(total_robots, 1)
                      + self.target_yaw)

        # Goal point on the standoff circle
        goal_x = px + self.standoff_radius * math.cos(slot_angle)
        goal_y = py + self.standoff_radius * math.sin(slot_angle)

        dx = goal_x - cur_x
        dy = goal_y - cur_y
        dist = math.hypot(dx, dy)

        angle_to_goal = math.atan2(dy, dx)
        angle_error = normalize_angle(angle_to_goal - cur_yaw)

        # PID on distance to assigned slot
        if dist > 0.05:
            raw_linear = self.linear_pids[robot_name].compute(dist, dt)
            alignment = math.cos(min(abs(angle_error), math.pi / 2))
            linear = max(0.0, min(raw_linear * alignment, self.max_linear_vel))
        else:
            linear = 0.0
            # When at the slot, face the target
            face_angle = math.atan2(py - cur_y, px - cur_x)
            angle_error = normalize_angle(face_angle - cur_yaw)

        angular = self.angular_pids[robot_name].compute(angle_error, dt)

        cmd.linear.x = linear
        cmd.angular.z = angular
        return cmd

    # -- strategy: intercept ------------------------------------------------

    def _compute_intercept(self, robot_name, robot_idx, total_robots, dt):
        """
        Predictive interception: robots navigate to points along the target's
        predicted future trajectory, forming a net ahead of the target.

        Each robot is assigned a different time horizon along the predicted
        path, creating a staggered formation that covers multiple possible
        future positions.
        """
        cmd = Twist()
        pose = self.poses.get(robot_name)
        if pose is None or self.target_pose is None:
            return cmd

        cur_x = pose.position.x
        cur_y = pose.position.y
        cur_yaw = self.yaws.get(robot_name, 0.0)

        # Stagger prediction horizons across robots
        if total_robots > 1:
            # Spread robots from 0.5x to 2.0x of the prediction horizon
            t_frac = 0.5 + 1.5 * (robot_idx / (total_robots - 1))
        else:
            t_frac = 1.0

        horizon = self.prediction_horizon * t_frac
        predicted = self._predict_target_position(horizon)
        if predicted is None:
            return cmd
        px, py = predicted

        # Offset each robot slightly to the side to form a net
        if total_robots > 1:
            spread = self.standoff_radius * 0.5
            lateral_offset = spread * (robot_idx / (total_robots - 1) - 0.5) * 2.0
        else:
            lateral_offset = 0.0

        # Perpendicular to target velocity
        vx, vy = self.target_velocity
        speed = math.hypot(vx, vy)
        if speed > 0.01:
            perp_x = -vy / speed
            perp_y = vx / speed
        else:
            perp_x, perp_y = 0.0, 0.0

        goal_x = px + lateral_offset * perp_x
        goal_y = py + lateral_offset * perp_y

        dx = goal_x - cur_x
        dy = goal_y - cur_y
        dist = math.hypot(dx, dy)

        angle_to_goal = math.atan2(dy, dx)
        angle_error = normalize_angle(angle_to_goal - cur_yaw)

        if dist > 0.1:
            raw_linear = self.linear_pids[robot_name].compute(dist, dt)
            alignment = math.cos(min(abs(angle_error), math.pi / 2))
            linear = max(0.0, min(raw_linear * alignment, self.max_linear_vel))
        else:
            linear = 0.0

        angular = self.angular_pids[robot_name].compute(angle_error, dt)

        cmd.linear.x = linear
        cmd.angular.z = angular
        return cmd

    # -- main control loop --------------------------------------------------

    def _control_loop(self, event):
        with self.lock:
            names = list(self.robot_names)

        if not self.is_active or len(names) == 0:
            return

        dt = self.dt
        total = len(names)

        # Collect robot positions for inter-robot avoidance
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

        # Compute and publish commands
        status_entries = []

        with self.target_lock:
            target_pos = self.target_pose
            target_vel = self.target_velocity

        for idx, name in enumerate(names):
            if self.strategy == 'pursuit':
                desired_cmd = self._compute_pursuit(name, dt)
            elif self.strategy == 'surround':
                desired_cmd = self._compute_surround(name, idx, total, dt)
            elif self.strategy == 'intercept':
                desired_cmd = self._compute_intercept(name, idx, total, dt)
            else:
                desired_cmd = self._compute_pursuit(name, dt)

            # Obstacle avoidance filter
            if name in self.avoidance:
                safe_cmd = self.avoidance[name].apply_avoidance(desired_cmd)
            else:
                safe_cmd = desired_cmd

            pub = self.cmd_pubs.get(name)
            if pub is not None:
                pub.publish(safe_cmd)

            pose = self.poses.get(name)
            if pose is not None:
                entry = {
                    'name': name,
                    'role': 'tracker',
                    'index': idx,
                    'x': round(pose.position.x, 4),
                    'y': round(pose.position.y, 4),
                    'yaw': round(self.yaws.get(name, 0.0), 4),
                    'linear_vel': round(safe_cmd.linear.x, 4),
                    'angular_vel': round(safe_cmd.angular.z, 4),
                }
                if target_pos is not None:
                    entry['dist_to_target'] = round(
                        math.hypot(target_pos[0] - pose.position.x,
                                   target_pos[1] - pose.position.y), 4
                    )
                status_entries.append(entry)

        # Publish status
        self._publish_status(status_entries, target_pos, target_vel)

        # Publish visualisation markers
        self._publish_markers(names, target_pos, target_vel)

    # -- status publishing --------------------------------------------------

    def _publish_status(self, entries, target_pos, target_vel):
        status = {
            'active': self.is_active,
            'strategy': self.strategy,
            'standoff_radius': self.standoff_radius,
            'prediction_horizon': self.prediction_horizon,
            'target': {
                'x': round(target_pos[0], 4) if target_pos else None,
                'y': round(target_pos[1], 4) if target_pos else None,
                'vx': round(target_vel[0], 4),
                'vy': round(target_vel[1], 4),
                'speed': round(math.hypot(target_vel[0], target_vel[1]), 4),
            },
            'robots': entries,
        }
        self.status_pub.publish(String(data=json.dumps(status)))

    # -- RViz visualisation -------------------------------------------------

    def _publish_markers(self, names, target_pos, target_vel):
        marker_array = MarkerArray()

        # -- Target sphere --------------------------------------------------
        if target_pos is not None:
            target_marker = Marker()
            target_marker.header.frame_id = 'map'
            target_marker.header.stamp = rospy.Time.now()
            target_marker.ns = 'target_tracking_target'
            target_marker.id = 0
            target_marker.type = Marker.SPHERE
            target_marker.action = Marker.ADD
            target_marker.pose.position.x = target_pos[0]
            target_marker.pose.position.y = target_pos[1]
            target_marker.pose.position.z = 0.4
            target_marker.pose.orientation.w = 1.0
            target_marker.scale.x = 0.3
            target_marker.scale.y = 0.3
            target_marker.scale.z = 0.3
            target_marker.color.r = 1.0
            target_marker.color.g = 0.0
            target_marker.color.b = 0.0
            target_marker.color.a = 0.9
            target_marker.lifetime = rospy.Duration(0.15)
            marker_array.markers.append(target_marker)

            # -- Target velocity arrow ------------------------------------------
            speed = math.hypot(target_vel[0], target_vel[1])
            if speed > 0.02:
                arrow = Marker()
                arrow.header.frame_id = 'map'
                arrow.header.stamp = rospy.Time.now()
                arrow.ns = 'target_tracking_velocity'
                arrow.id = 1
                arrow.type = Marker.ARROW
                arrow.action = Marker.ADD
                start = Point()
                start.x = target_pos[0]
                start.y = target_pos[1]
                start.z = 0.3
                end = Point()
                scale = 2.0  # exaggerate for visibility
                end.x = target_pos[0] + target_vel[0] * scale
                end.y = target_pos[1] + target_vel[1] * scale
                end.z = 0.3
                arrow.points = [start, end]
                arrow.scale.x = 0.04  # shaft diameter
                arrow.scale.y = 0.08  # head diameter
                arrow.scale.z = 0.0
                arrow.color.r = 1.0
                arrow.color.g = 0.5
                arrow.color.b = 0.0
                arrow.color.a = 0.8
                arrow.lifetime = rospy.Duration(0.15)
                marker_array.markers.append(arrow)

            # -- Predicted position marker --------------------------------------
            predicted = self._predict_target_position(self.prediction_horizon)
            if predicted is not None:
                pred_marker = Marker()
                pred_marker.header.frame_id = 'map'
                pred_marker.header.stamp = rospy.Time.now()
                pred_marker.ns = 'target_tracking_predicted'
                pred_marker.id = 2
                pred_marker.type = Marker.SPHERE
                pred_marker.action = Marker.ADD
                pred_marker.pose.position.x = predicted[0]
                pred_marker.pose.position.y = predicted[1]
                pred_marker.pose.position.z = 0.35
                pred_marker.pose.orientation.w = 1.0
                pred_marker.scale.x = 0.2
                pred_marker.scale.y = 0.2
                pred_marker.scale.z = 0.2
                pred_marker.color.r = 1.0
                pred_marker.color.g = 0.8
                pred_marker.color.b = 0.0
                pred_marker.color.a = 0.5
                pred_marker.lifetime = rospy.Duration(0.15)
                marker_array.markers.append(pred_marker)

            # -- Standoff circle (for surround strategy) ----------------------
            if self.strategy == 'surround':
                circle = Marker()
                circle.header.frame_id = 'map'
                circle.header.stamp = rospy.Time.now()
                circle.ns = 'target_tracking_standoff'
                circle.id = 3
                circle.type = Marker.LINE_STRIP
                circle.action = Marker.ADD
                circle.scale.x = 0.02
                circle.color.r = 0.3
                circle.color.g = 0.8
                circle.color.b = 1.0
                circle.color.a = 0.6
                circle.lifetime = rospy.Duration(0.15)
                num_pts = 36
                for i in range(num_pts + 1):
                    angle = 2.0 * math.pi * i / num_pts
                    p = Point()
                    p.x = target_pos[0] + self.standoff_radius * math.cos(angle)
                    p.y = target_pos[1] + self.standoff_radius * math.sin(angle)
                    p.z = 0.05
                    circle.points.append(p)
                marker_array.markers.append(circle)

        # -- Lines from each robot to target ---------------------------------
        if target_pos is not None:
            for idx, name in enumerate(names):
                pose = self.poses.get(name)
                if pose is None:
                    continue
                line = Marker()
                line.header.frame_id = 'map'
                line.header.stamp = rospy.Time.now()
                line.ns = 'target_tracking_lines'
                line.id = 100 + idx
                line.type = Marker.LINE_STRIP
                line.action = Marker.ADD
                line.scale.x = 0.02
                line.lifetime = rospy.Duration(0.15)

                # Colour based on distance
                dist = math.hypot(
                    target_pos[0] - pose.position.x,
                    target_pos[1] - pose.position.y,
                )
                # Green when close, yellow when far
                frac = min(dist / 4.0, 1.0)
                line.color.r = frac
                line.color.g = 1.0 - 0.3 * frac
                line.color.b = 0.0
                line.color.a = 0.6

                start = Point()
                start.x = pose.position.x
                start.y = pose.position.y
                start.z = 0.1
                end = Point()
                end.x = target_pos[0]
                end.y = target_pos[1]
                end.z = 0.1
                line.points = [start, end]
                marker_array.markers.append(line)

        # -- Robot spheres ---------------------------------------------------
        for idx, name in enumerate(names):
            pose = self.poses.get(name)
            if pose is None:
                continue
            sphere = Marker()
            sphere.header.frame_id = 'map'
            sphere.header.stamp = rospy.Time.now()
            sphere.ns = 'target_tracking_robots'
            sphere.id = 200 + idx
            sphere.type = Marker.SPHERE
            sphere.action = Marker.ADD
            sphere.pose.position.x = pose.position.x
            sphere.pose.position.y = pose.position.y
            sphere.pose.position.z = 0.3
            sphere.pose.orientation.w = 1.0
            sphere.scale.x = 0.15
            sphere.scale.y = 0.15
            sphere.scale.z = 0.15
            # Gradient: blue to cyan across robots
            frac = idx / max(len(names) - 1, 1)
            sphere.color.r = 0.0
            sphere.color.g = 0.3 + 0.7 * frac
            sphere.color.b = 1.0 - 0.3 * frac
            sphere.color.a = 0.85
            sphere.lifetime = rospy.Duration(0.15)
            marker_array.markers.append(sphere)

        if marker_array.markers:
            self.marker_pub.publish(marker_array)

    # -- utilities ----------------------------------------------------------

    def _stop_all(self):
        stop = Twist()
        for pub in self.cmd_pubs.values():
            pub.publish(stop)


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------

if __name__ == '__main__':
    try:
        node = TargetTracking()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
