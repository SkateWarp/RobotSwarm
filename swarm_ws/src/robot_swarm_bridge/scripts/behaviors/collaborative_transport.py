#!/usr/bin/env python3
"""
Collaborative Transport using Gibbs Random Fields (GRF)

Based on: "Cooperative Object Transportation using Gibbs Random Fields"
          (rezeck et al., IROS 2021)

Uses Coulomb-Buckingham potential functions and Metropolis-Hastings MCMC
sampling for decentralized multi-robot coordination. Robots autonomously
search for, approach, and push a transport object to a target location.

Phases:
  SEARCH   - Robots spread out via random walk to locate the object
  APPROACH - All robots converge on the discovered object
  PUSH     - GRF-based coordinated pushing with MCMC velocity sampling
  DONE     - Object delivered to target; all robots stop

Works with dynamically changing TurtleBot3 Burger fleets in Gazebo.
"""

import rospy
import numpy as np
import json
import math
import threading
from typing import List, Dict, Optional, Tuple
from enum import Enum

# ROS messages
from geometry_msgs.msg import Twist, Point, Quaternion
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String, Bool
from gazebo_msgs.msg import ModelStates
from visualization_msgs.msg import Marker, MarkerArray

# Import obstacle avoidance from core
import sys
import os
sys.path.append(os.path.join(os.path.dirname(__file__), '..'))
package_source = os.path.abspath(
    os.path.join(os.path.dirname(__file__), '..', '..', 'src')
)
if package_source not in sys.path:
    sys.path.insert(0, package_source)

from core.obstacle_avoidance import ObstacleAvoidance
from utils.robot_ids import sort_robot_ids
from robot_swarm_bridge.algorithms.grf_adapter import (
    build_transport_snapshot,
    mcmc_iterations_for_fleet,
)
from robot_swarm_bridge.algorithms.grf_transport import (
    GRFConfig,
    GibbsRandomFieldTransport,
    InteractionMode,
    RobotSnapshot,
    Vec2,
)


class TransportPhase(Enum):
    """Transport task phases."""
    IDLE = "IDLE"
    SEARCH = "SEARCH"
    APPROACH = "APPROACH"
    PUSH = "PUSH"
    DONE = "DONE"


# ---------------------------------------------------------------------------
# Coulomb-Buckingham potential
# ---------------------------------------------------------------------------

def coulomb_buckingham(r, epsilon, r0, alpha, q1, q2, epsilon0):
    """
    Compute the Coulomb-Buckingham potential energy.

    U(r) = epsilon * [ 6/(a-6) * exp(a*(1 - r/r0))
                      - a/(a-6) * (r0/r)^6 ]
          + q1*q2 / (4*pi*epsilon0*r)

    Parameters
    ----------
    r : float
        Distance between the two interacting entities (must be > 0).
    epsilon : float
        Depth of the potential well.
    r0 : float
        Equilibrium distance.
    alpha : float
        Steepness parameter (referred to as *a* in the formula).
    q1, q2 : float
        Charges.  Same sign -> repulsion, opposite sign -> attraction.
    epsilon0 : float
        Permittivity constant (scaling for the Coulomb term).

    Returns
    -------
    float
        Potential energy U(r).
    """
    # Prevent numerical blow-up
    r = max(r, 1e-4)
    a = alpha
    denom = a - 6.0
    if abs(denom) < 1e-8:
        denom = 1e-8  # avoid division by zero when alpha == 6

    exp_term = math.exp(a * (1.0 - r / r0))
    r0_over_r_6 = (r0 / r) ** 6

    u_buck = epsilon * ((6.0 / denom) * exp_term - (a / denom) * r0_over_r_6)
    u_coul = (q1 * q2) / (4.0 * math.pi * epsilon0 * r)
    return u_buck + u_coul


# ---------------------------------------------------------------------------
# Main node
# ---------------------------------------------------------------------------

class CollaborativeTransport:
    """
    GRF-based collaborative transport controller.

    Each robot independently computes its velocity using Metropolis-Hastings
    MCMC sampling over three energy terms:

        U_s  - obstacle / wall repulsion
        U_t  - transport-object interaction (push / reposition via occlusion)
        U_st - inter-robot cohesion + velocity consensus
    """

    def __init__(self):
        rospy.init_node('collaborative_transport', anonymous=False)

        # ---- ROS parameters ------------------------------------------------
        self.robot_count = rospy.get_param('~robot_count', 4)
        self.object_name = rospy.get_param('~object_name', 'transport_object')
        self.target_x = rospy.get_param('~target_x', 3.0)
        self.target_y = rospy.get_param('~target_y', 3.0)
        self.vmax = rospy.get_param('~vmax', 0.15)
        self.sensing_range = rospy.get_param('~sensing_range', 2.0)
        self.safezone = rospy.get_param('~safezone', 0.5)
        self.mcmc_iterations = rospy.get_param('~mcmc_iterations', 60)
        self.mcmc_sigma = rospy.get_param('~mcmc_sigma', 0.05)
        self.mcmc_burnin = rospy.get_param('~mcmc_burnin', 0.6)
        self.arrival_tolerance = rospy.get_param('~arrival_tolerance', 0.5)
        self.min_ready_robots = max(
            1, int(rospy.get_param('~min_ready_robots', 1))
        )
        planner = str(
            rospy.get_param('~transport_planner', 'grf')
        ).strip().lower()
        if planner not in ('grf', 'legacy'):
            rospy.logwarn(
                "[transport] unknown planner '%s'; using grf", planner
            )
            planner = 'grf'
        self.transport_planner = planner

        self.grf_mcmc_iterations = max(
            1,
            int(rospy.get_param(
                '~grf_mcmc_iterations', self.mcmc_iterations
            )),
        )
        self.grf_large_fleet_threshold = max(
            1,
            int(rospy.get_param('~grf_large_fleet_threshold', 20)),
        )
        requested_large_iterations = max(
            1,
            int(rospy.get_param(
                '~grf_large_fleet_iterations',
                min(24, self.grf_mcmc_iterations),
            )),
        )
        self.grf_large_fleet_iterations = min(
            self.grf_mcmc_iterations,
            requested_large_iterations,
        )
        self.grf_temperature = float(
            rospy.get_param('~grf_temperature', 1.0)
        )
        self.grf_random_seed = int(
            rospy.get_param('~grf_random_seed', 0)
        )
        self.grf_object_radius = float(
            rospy.get_param('~grf_object_radius', 0.2)
        )
        self.grf_contour_samples = max(
            3,
            int(rospy.get_param('~grf_contour_samples', 16)),
        )

        # ---- GRF energy parameters -----------------------------------------
        # U_s  (obstacle avoidance) -- same-sign charges -> repulsion
        self.us_epsilon = 0.04
        self.us_epsilon0 = 0.04
        self.us_r0 = 0.8
        self.us_alpha = 0.05
        self.us_q1 = 1.0
        self.us_q2 = 1.0

        # U_t  (object interaction) -- opposite-sign charges -> attraction
        self.ut_epsilon = 0.04
        self.ut_epsilon0 = 0.04
        self.ut_r0 = 0.8
        self.ut_alpha = 0.2
        self.ut_q1 = 100.0
        self.ut_q2 = -1.0

        # U_st (inter-robot cohesion) -- mild attraction + consensus
        self.ust_epsilon = 0.04
        self.ust_epsilon0 = 0.04
        self.ust_r0 = 0.8
        self.ust_alpha = 0.65
        self.ust_q1 = 50.0
        self.ust_q2 = -1.0

        # Velocity consensus weight inside U_st
        self.velocity_consensus_weight = 0.5
        self._grf_kernels: Dict[int, GibbsRandomFieldTransport] = {}
        self._grf_step_index = 0
        self._active_planner = self.transport_planner
        self._active_grf_iterations = 0

        # ---- Phase / state --------------------------------------------------
        self.phase = TransportPhase.IDLE
        self.is_running = False
        self.is_paused = False
        self.emergency_stop_active = False
        self.current_task_id = None
        self.command_epoch = 0
        self.phase_lock = threading.Lock()
        self.command_lock = threading.RLock()

        # ---- Robot bookkeeping ----------------------------------------------
        # Populated dynamically from /fleet/robot_list
        self.robot_namespaces: List[str] = []

        # Per-robot state  (keyed by namespace string)
        self.robot_positions: Dict[str, np.ndarray] = {}    # [x, y]
        self.robot_yaws: Dict[str, float] = {}
        self.robot_velocities: Dict[str, np.ndarray] = {}   # [vx, vy] world
        self.robot_scans: Dict[str, LaserScan] = {}
        self.avoidance_modules: Dict[str, ObstacleAvoidance] = {}

        self.data_lock = threading.Lock()

        # ---- Object / obstacle tracking from Gazebo -------------------------
        self.object_position: Optional[np.ndarray] = None  # [x, y]
        self.object_found = False
        self.obstacle_positions: List[np.ndarray] = []     # list of [x,y]
        self.model_lock = threading.Lock()

        # ---- Publishers (per-robot cmd_vel created in _setup_robot) ----------
        self.cmd_vel_pubs: Dict[str, rospy.Publisher] = {}
        self.odom_subs: Dict[str, rospy.Subscriber] = {}
        self.scan_subs: Dict[str, rospy.Subscriber] = {}

        self.status_pub = rospy.Publisher(
            '/transport/status', String, queue_size=1
        )
        self.marker_pub = rospy.Publisher(
            '/transport/markers', MarkerArray, queue_size=1
        )

        # ---- Subscribers (global) -------------------------------------------
        rospy.Subscriber(
            '/fleet/robot_list', String,
            self._fleet_callback, queue_size=1
        )
        rospy.Subscriber(
            '/gazebo/model_states', ModelStates,
            self._model_states_callback, queue_size=1
        )
        rospy.Subscriber(
            '/transport/start', String,
            self._start_callback, queue_size=1
        )
        rospy.Subscriber(
            '/transport/stop', String,
            self._stop_callback, queue_size=1
        )
        rospy.Subscriber(
            '/transport/pause', String,
            self._pause_callback, queue_size=1
        )
        rospy.Subscriber(
            '/transport/resume', String,
            self._resume_callback, queue_size=1
        )
        rospy.Subscriber(
            '/swarm/emergency_stop', Bool,
            self._emergency_stop_callback, queue_size=1
        )

        # ---- Seed default fleet if no /fleet/robot_list is published --------
        # Build initial namespace list from robot_count param
        self._build_default_fleet()

        # ---- 10 Hz control timer -------------------------------------------
        self.control_timer = rospy.Timer(
            rospy.Duration(0.1), self._control_loop
        )

        rospy.loginfo(
            "[transport] GRF collaborative transport initialised  "
            "robots=%d  object='%s'  target=(%.1f, %.1f)  planner=%s",
            self.robot_count, self.object_name,
            self.target_x, self.target_y,
            self.transport_planner,
        )

    # ======================================================================
    # Fleet management
    # ======================================================================

    def _build_default_fleet(self):
        """Create the namespace list from ~robot_count and set up each robot."""
        ns_list = [f'tb3_{i}' for i in range(self.robot_count)]
        self._update_fleet(ns_list)

    def _fleet_callback(self, msg: String):
        """Handle /fleet/robot_list (comma-separated namespace strings)."""
        ns_list = sort_robot_ids(
            s.strip() for s in msg.data.split(',') if s.strip()
        )
        self._update_fleet(ns_list)

    def _update_fleet(self, ns_list: List[str]):
        """Synchronise internal bookkeeping with the given namespace list."""
        with self.command_lock:
            if not ns_list and (self.is_running or self.is_paused):
                self.is_running = False
                self.is_paused = False
                self.command_epoch += 1
                with self.phase_lock:
                    self.phase = TransportPhase.IDLE

            with self.data_lock:
                # Add new robots
                for ns in ns_list:
                    if ns not in self.robot_namespaces:
                        self._setup_robot(ns)
                # Remove departed robots
                for ns in list(self.robot_namespaces):
                    if ns not in ns_list:
                        self._teardown_robot(ns)
                self.robot_namespaces = list(ns_list)
                self.robot_count = len(self.robot_namespaces)

    def _setup_robot(self, ns: str):
        """Create subscribers, publishers, and avoidance module for one robot."""
        # Publishers
        self.cmd_vel_pubs[ns] = rospy.Publisher(
            f'/{ns}/cmd_vel', Twist, queue_size=1
        )

        # Subscribers
        self.odom_subs[ns] = rospy.Subscriber(
            f'/{ns}/odom', Odometry,
            lambda msg, _ns=ns: self._odom_callback(_ns, msg),
            queue_size=1,
        )
        self.scan_subs[ns] = rospy.Subscriber(
            f'/{ns}/scan', LaserScan,
            lambda msg, _ns=ns: self._scan_callback(_ns, msg),
            queue_size=1,
        )

        # Initial state
        self.robot_positions[ns] = np.zeros(2)
        self.robot_yaws[ns] = 0.0
        self.robot_velocities[ns] = np.zeros(2)

        # Obstacle avoidance (used only in SEARCH / APPROACH)
        self.avoidance_modules[ns] = ObstacleAvoidance(ns)

        rospy.loginfo("[transport] registered robot %s", ns)

    def _teardown_robot(self, ns: str):
        """Remove bookkeeping for a robot that left the fleet."""
        pub = self.cmd_vel_pubs.pop(ns, None)
        if pub is not None:
            pub.publish(Twist())
            pub.unregister()

        odom_sub = self.odom_subs.pop(ns, None)
        if odom_sub is not None:
            odom_sub.unregister()

        scan_sub = self.scan_subs.pop(ns, None)
        if scan_sub is not None:
            scan_sub.unregister()

        avoidance = self.avoidance_modules.pop(ns, None)
        if avoidance is not None:
            avoidance.shutdown()

        for d in (self.robot_positions, self.robot_yaws,
                  self.robot_velocities, self.robot_scans):
            d.pop(ns, None)
        if ns in self.robot_namespaces:
            self.robot_namespaces.remove(ns)
        rospy.loginfo("[transport] unregistered robot %s", ns)

    # ======================================================================
    # Subscriber callbacks
    # ======================================================================

    def _odom_callback(self, ns: str, msg: Odometry):
        pos = msg.pose.pose.position
        yaw = self._quat_to_yaw(msg.pose.pose.orientation)
        with self.data_lock:
            if ns not in self.cmd_vel_pubs:
                return
            self.robot_positions[ns] = np.array([pos.x, pos.y])
            self.robot_yaws[ns] = yaw
            v = msg.twist.twist
            # Odometry twist is expressed in the robot body frame. Convert it
            # before comparing with GRF samples, which are world-frame vectors.
            cos_yaw = math.cos(yaw)
            sin_yaw = math.sin(yaw)
            self.robot_velocities[ns] = np.array([
                v.linear.x * cos_yaw - v.linear.y * sin_yaw,
                v.linear.x * sin_yaw + v.linear.y * cos_yaw,
            ])

        # Keep obstacle avoidance module in sync
        av = self.avoidance_modules.get(ns)
        if av is not None:
            av.set_position(pos.x, pos.y, yaw)

    def _scan_callback(self, ns: str, msg: LaserScan):
        with self.data_lock:
            if ns not in self.cmd_vel_pubs:
                return
            self.robot_scans[ns] = msg

    def _model_states_callback(self, msg: ModelStates):
        """Track the transport object and other non-robot models (obstacles)."""
        with self.model_lock:
            self.obstacle_positions = []
            found = False
            for i, name in enumerate(msg.name):
                px = msg.pose[i].position.x
                py = msg.pose[i].position.y
                if name == self.object_name:
                    self.object_position = np.array([px, py])
                    found = True
                elif name.startswith('obstacle_') or name.startswith('wall_'):
                    # Only explicitly named arena obstacles are included.
                    self.obstacle_positions.append(np.array([px, py]))
            if found and not self.object_found:
                rospy.loginfo(
                    "[transport] object '%s' detected at (%.2f, %.2f)",
                    self.object_name,
                    self.object_position[0],
                    self.object_position[1],
                )
            self.object_found = found

    def _start_callback(self, msg):
        # Parse config from task orchestrator (JSON String)
        try:
            config = json.loads(msg.data) if msg.data else {}
        except (json.JSONDecodeError, AttributeError):
            config = {}

        with self.command_lock:
            if self.emergency_stop_active:
                rospy.logwarn(
                    "[transport] Start rejected while emergency stop is active"
                )
                return
            with self.data_lock:
                if not self.robot_namespaces:
                    rospy.logwarn(
                        "[transport] Start rejected without an active fleet"
                    )
                    return

            # Apply runtime config
            if 'target_x' in config:
                self.target_x = float(config['target_x'])
            if 'target_y' in config:
                self.target_y = float(config['target_y'])

            requested_planner = config.get(
                'transport_planner', config.get('planner')
            )
            if requested_planner is not None:
                requested_planner = str(requested_planner).strip().lower()
                if requested_planner in ('grf', 'legacy'):
                    self.transport_planner = requested_planner
                else:
                    rospy.logwarn(
                        "[transport] ignoring unknown planner '%s'",
                        requested_planner,
                    )

            if 'grf_mcmc_iterations' in config:
                self.grf_mcmc_iterations = max(
                    1, int(config['grf_mcmc_iterations'])
                )
            if 'grf_large_fleet_threshold' in config:
                self.grf_large_fleet_threshold = max(
                    1, int(config['grf_large_fleet_threshold'])
                )
            if 'grf_large_fleet_iterations' in config:
                self.grf_large_fleet_iterations = max(
                    1, int(config['grf_large_fleet_iterations'])
                )
            self.grf_large_fleet_iterations = min(
                self.grf_mcmc_iterations,
                self.grf_large_fleet_iterations,
            )
            self._grf_kernels.clear()
            self._grf_step_index = 0
            self._active_planner = self.transport_planner
            self._active_grf_iterations = 0
            self.current_task_id = config.get('task_id')
            self.is_running = True
            self.is_paused = False
            self.command_epoch += 1

            with self.phase_lock:
                self.phase = TransportPhase.SEARCH
            rospy.loginfo(
                "[transport] >>> phase SEARCH  target=(%.1f, %.1f) "
                "planner=%s",
                self.target_x, self.target_y, self.transport_planner,
            )

    def _stop_callback(self, msg: String):
        with self.command_lock:
            if not self._task_command_matches(msg):
                return
            self.is_running = False
            self.is_paused = False
            self.command_epoch += 1
            with self.phase_lock:
                self.phase = TransportPhase.IDLE
            self._stop_all_robots()
        rospy.loginfo("[transport] stopped (IDLE)")

    def _pause_callback(self, msg: String):
        with self.command_lock:
            if not self._task_command_matches(msg):
                return
            if not self.is_running or self.is_paused:
                return
            self.is_paused = True
            self.command_epoch += 1
            self._stop_all_robots()
        rospy.loginfo("[transport] paused")

    def _resume_callback(self, msg: String):
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
            self.command_epoch += 1
        rospy.loginfo("[transport] resumed")

    def _task_command_matches(self, msg: String) -> bool:
        try:
            payload = json.loads(msg.data) if msg.data else {}
        except (json.JSONDecodeError, AttributeError):
            payload = {}
        requested_id = payload.get('task_id')
        return bool(requested_id) and requested_id == self.current_task_id

    def _emergency_stop_callback(self, msg: Bool):
        with self.command_lock:
            self.emergency_stop_active = bool(msg.data)
            self.command_epoch += 1
            if self.emergency_stop_active:
                self.is_running = False
                self.is_paused = False
                with self.phase_lock:
                    self.phase = TransportPhase.IDLE
                self._stop_all_robots()
                rospy.logwarn("[transport] Emergency stop latched")

    # ======================================================================
    # Helpers
    # ======================================================================

    @staticmethod
    def _quat_to_yaw(q: Quaternion) -> float:
        siny = 2.0 * (q.w * q.z + q.x * q.y)
        cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny, cosy)

    @staticmethod
    def _normalize_angle(a: float) -> float:
        while a > math.pi:
            a -= 2.0 * math.pi
        while a < -math.pi:
            a += 2.0 * math.pi
        return a

    def _stop_all_robots(self):
        stop = Twist()
        with self.data_lock:
            pubs = list(self.cmd_vel_pubs.values())
        for pub in pubs:
            pub.publish(stop)

    def _command_allowed(self, expected_epoch: int) -> bool:
        return (
            self.command_epoch == expected_epoch
            and self.is_running
            and not self.is_paused
            and not self.emergency_stop_active
        )

    def _publish_command(
        self, namespace: str, command: Twist, expected_epoch: int
    ) -> bool:
        """Publish only while the control cycle still owns the active epoch."""
        with self.command_lock:
            if not self._command_allowed(expected_epoch):
                return False
            with self.data_lock:
                pub = self.cmd_vel_pubs.get(namespace)
                if pub is None:
                    return False
                pub.publish(command)
            return True

    def _set_phase(
        self, phase: TransportPhase, expected_epoch: int
    ) -> bool:
        with self.command_lock:
            if not self._command_allowed(expected_epoch):
                return False
            with self.phase_lock:
                self.phase = phase
            return True

    def _holonomic_to_diff_drive(self, vx: float, vy: float,
                                  yaw: float) -> Twist:
        """
        Convert a desired world-frame (vx, vy) velocity into a
        differential-drive Twist (linear.x, angular.z).
        """
        cmd = Twist()
        speed = math.sqrt(vx * vx + vy * vy)
        if speed < 1e-6:
            return cmd

        theta_desired = math.atan2(vy, vx)
        angle_error = self._normalize_angle(theta_desired - yaw)

        linear = min(speed, self.vmax)
        angular = 2.0 * angle_error

        # Reduce linear velocity when the heading error is large
        if abs(angle_error) > math.pi / 3.0:
            linear *= 0.3

        cmd.linear.x = linear
        cmd.angular.z = max(-2.84, min(2.84, angular))
        return cmd

    def _grf_kernel_for_fleet(self, fleet_size: int):
        iterations = mcmc_iterations_for_fleet(
            fleet_size,
            self.grf_mcmc_iterations,
            self.grf_large_fleet_threshold,
            self.grf_large_fleet_iterations,
        )
        kernel = self._grf_kernels.get(iterations)
        if kernel is None:
            kernel = GibbsRandomFieldTransport(
                GRFConfig(
                    time_step=0.1,
                    max_speed=self.vmax,
                    sensing_radius=self.sensing_range,
                    obstacle_influence_radius=self.safezone,
                    mcmc_iterations=iterations,
                    proposal_sigma=self.mcmc_sigma,
                    burn_in_fraction=self.mcmc_burnin,
                    temperature=self.grf_temperature,
                    random_seed=self.grf_random_seed,
                    velocity_consensus_weight=(
                        self.velocity_consensus_weight
                    ),
                )
            )
            self._grf_kernels[iterations] = kernel
        return kernel, iterations

    def _build_grf_snapshot(self, namespaces, object_pos, obstacles):
        robots = []
        positions = {}
        yaws = {}

        with self.data_lock:
            for ns in namespaces:
                position = self.robot_positions.get(ns)
                if position is None:
                    continue
                velocity = self.robot_velocities.get(ns, np.zeros(2))
                yaw = float(self.robot_yaws.get(ns, 0.0))
                positions[ns] = position.copy()
                yaws[ns] = yaw
                robots.append(
                    RobotSnapshot(
                        robot_id=ns,
                        position=Vec2(
                            float(position[0]), float(position[1])
                        ),
                        velocity=Vec2(
                            float(velocity[0]), float(velocity[1])
                        ),
                        heading=yaw,
                    )
                )

        snapshot = build_transport_snapshot(
            robots=robots,
            object_center=Vec2(
                float(object_pos[0]), float(object_pos[1])
            ),
            target=Vec2(self.target_x, self.target_y),
            obstacle_points=tuple(
                Vec2(float(point[0]), float(point[1]))
                for point in obstacles
            ),
            object_radius=self.grf_object_radius,
            contour_samples=self.grf_contour_samples,
        )
        return snapshot, positions, yaws

    def _grf_push_commands(
        self, namespaces, object_pos, obstacles, expected_epoch
    ):
        snapshot, positions, yaws = self._build_grf_snapshot(
            namespaces, object_pos, obstacles
        )
        kernel, iterations = self._grf_kernel_for_fleet(
            len(snapshot.robots)
        )
        with self.command_lock:
            if not self._command_allowed(expected_epoch):
                return {}, yaws
            step_index = self._grf_step_index
        step = kernel.compute(snapshot, step_index=step_index)

        commands = {}
        for result in step.robots:
            if result.interaction_mode == InteractionMode.INVALID:
                commands[result.robot_id] = None
                continue

            if result.interaction_mode == InteractionMode.NO_OBJECT:
                position = positions[result.robot_id]
                difference = object_pos - position
                distance = float(np.linalg.norm(difference))
                if distance > 0.01:
                    speed = min(self.vmax, 0.5 * distance)
                    direction = difference / distance
                    commands[result.robot_id] = Vec2(
                        float(direction[0] * speed),
                        float(direction[1] * speed),
                    )
                else:
                    commands[result.robot_id] = Vec2()
                continue

            commands[result.robot_id] = result.velocity

        with self.command_lock:
            if not self._command_allowed(expected_epoch):
                return {}, yaws
            self._grf_step_index = max(
                self._grf_step_index, step_index + 1
            )
            self._active_planner = 'grf'
            self._active_grf_iterations = iterations
        return commands, yaws

    def _publish_grf_commands(
        self, namespaces, commands, yaws, expected_epoch
    ):
        for ns in namespaces:
            velocity = commands.get(ns)
            yaw = yaws.get(ns)
            if velocity is None or yaw is None or not math.isfinite(yaw):
                self._publish_command(ns, Twist(), expected_epoch)
                continue
            if not velocity.is_finite():
                self._publish_command(ns, Twist(), expected_epoch)
                continue

            cmd = self._holonomic_to_diff_drive(
                velocity.x, velocity.y, yaw
            )
            with self.data_lock:
                avoidance = self.avoidance_modules.get(ns)
            if avoidance is not None:
                cmd = avoidance.apply_avoidance(cmd)
            self._publish_command(ns, cmd, expected_epoch)

    def _legacy_push_step(
        self, namespaces, yaws, expected_epoch, fallback=False
    ):
        with self.command_lock:
            if not self._command_allowed(expected_epoch):
                return
            self._active_planner = (
                'legacy_fallback' if fallback else 'legacy'
            )
            self._active_grf_iterations = 0
        for ns in namespaces:
            if ns not in yaws:
                self._publish_command(ns, Twist(), expected_epoch)
                continue

            try:
                sampled_vel = self._mcmc_sample_velocity(ns)
            except Exception as exc:
                rospy.logwarn_throttle(
                    5.0,
                    "[transport] legacy planner failed for {}: {}".format(
                        ns, exc
                    ),
                )
                self._publish_command(ns, Twist(), expected_epoch)
                continue
            if (
                not np.all(np.isfinite(sampled_vel))
                or not math.isfinite(yaws[ns])
            ):
                self._publish_command(ns, Twist(), expected_epoch)
                continue

            cmd = self._holonomic_to_diff_drive(
                float(sampled_vel[0]),
                float(sampled_vel[1]),
                yaws[ns],
            )
            with self.data_lock:
                avoidance = self.avoidance_modules.get(ns)
            if avoidance is not None:
                cmd = avoidance.apply_avoidance(cmd)
            self._publish_command(ns, cmd, expected_epoch)

    # ======================================================================
    # GRF energy functions
    # ======================================================================

    def _compute_target_occlusion(self, robot_pos: np.ndarray,
                                   object_pos: np.ndarray,
                                   target_pos: np.ndarray) -> float:
        """
        Compute how much the transport object occludes the robot's view of
        the target.

        Returns a value in [0, 1].
          - 0 means the robot is *behind* the object relative to target
            (good pushing position, low occlusion).
          - 1 means the robot is *in front* of the object
            (should reposition, high occlusion).

        Logic:
            d_rt = unit vector from object toward target
            d_ro = unit vector from object toward robot
            dot  = d_rt . d_ro          in [-1, 1]
            occlusion = (dot + 1) / 2   in [ 0, 1]

        When the robot is behind the object (opposite side from target)
        the dot product is negative -> low occlusion.
        When the robot is on the same side as the target the dot product
        is positive -> high occlusion.
        """
        to_target = target_pos - object_pos
        to_robot = robot_pos - object_pos

        norm_t = np.linalg.norm(to_target)
        norm_r = np.linalg.norm(to_robot)
        if norm_t < 1e-6 or norm_r < 1e-6:
            return 0.5

        d_rt = to_target / norm_t
        d_ro = to_robot / norm_r
        dot = float(np.dot(d_rt, d_ro))
        occlusion = (dot + 1.0) / 2.0
        return occlusion

    # ---- U_s: obstacle/wall repulsion energy ----

    def _compute_obstacle_energy(self, robot_pos: np.ndarray,
                                  proposed_vel: np.ndarray,
                                  scan: Optional[LaserScan],
                                  robot_yaw: float) -> float:
        """
        Repulsive energy from obstacles detected via LaserScan and from
        known obstacle model positions.
        Only activated when distance < safezone.
        """
        energy = 0.0

        # --- From LaserScan ---
        if scan is not None:
            n_rays = len(scan.ranges)
            for i in range(0, n_rays, max(1, n_rays // 36)):
                r = scan.ranges[i]
                if math.isinf(r) or math.isnan(r):
                    continue
                if r < scan.range_min or r > scan.range_max:
                    continue
                if r < self.safezone:
                    # Effective distance after applying proposed step
                    angle = (
                        robot_yaw + scan.angle_min + i * scan.angle_increment
                    )
                    obs_dir = np.array([math.cos(angle), math.sin(angle)])
                    step = proposed_vel * 0.1  # dt = 0.1 s
                    effective_r = max(r - float(np.dot(step, obs_dir)), 1e-4)
                    energy += coulomb_buckingham(
                        effective_r,
                        self.us_epsilon, self.us_r0, self.us_alpha,
                        self.us_q1, self.us_q2, self.us_epsilon0,
                    )

        # --- From Gazebo obstacle models ---
        with self.model_lock:
            for obs in self.obstacle_positions:
                d = float(np.linalg.norm(robot_pos - obs))
                if d < self.safezone:
                    direction = (robot_pos - obs)
                    if d > 1e-4:
                        direction /= d
                    step = proposed_vel * 0.1
                    effective_d = max(d - float(np.dot(step, -direction)), 1e-4)
                    energy += coulomb_buckingham(
                        effective_d,
                        self.us_epsilon, self.us_r0, self.us_alpha,
                        self.us_q1, self.us_q2, self.us_epsilon0,
                    )

        return energy

    # ---- U_t: transport-object interaction energy ----

    def _compute_object_energy(self, robot_pos: np.ndarray,
                                proposed_vel: np.ndarray,
                                object_pos: np.ndarray,
                                target_pos: np.ndarray) -> float:
        """
        Object interaction energy with occlusion-based sign flip.

        When occlusion is LOW  (robot behind object): attractive potential
        encourages the robot to approach and push.
        When occlusion is HIGH (robot in front):      repulsive potential
        forces the robot to reposition out of the way.
        """
        d = float(np.linalg.norm(robot_pos - object_pos))
        if d < 1e-4:
            d = 1e-4

        occlusion = self._compute_target_occlusion(robot_pos, object_pos,
                                                     target_pos)

        # Modulate the charge sign with occlusion
        # Low occlusion  -> q2 stays negative (attraction)
        # High occlusion -> q2 flipped to positive (repulsion)
        effective_q2 = self.ut_q2 * (1.0 - 2.0 * occlusion)

        # Effective distance after proposed step toward / away from object
        direction = (object_pos - robot_pos)
        if np.linalg.norm(direction) > 1e-6:
            direction = direction / np.linalg.norm(direction)
        step = proposed_vel * 0.1
        effective_d = max(d - float(np.dot(step, direction)), 1e-4)

        energy = coulomb_buckingham(
            effective_d,
            self.ut_epsilon, self.ut_r0, self.ut_alpha,
            self.ut_q1, effective_q2, self.ut_epsilon0,
        )
        return energy

    # ---- U_st: inter-robot interaction energy ----

    def _compute_robot_energy(self, robot_pos: np.ndarray,
                               proposed_vel: np.ndarray,
                               other_positions: List[np.ndarray],
                               other_velocities: List[np.ndarray]) -> float:
        """
        Inter-robot energy: moderate attraction to maintain group cohesion
        plus a velocity-consensus term that rewards aligning velocities
        with neighbours.
        """
        energy = 0.0

        for opos, ovel in zip(other_positions, other_velocities):
            d = float(np.linalg.norm(robot_pos - opos))
            if d < 1e-4:
                d = 1e-4

            # Coulomb-Buckingham attraction / spacing
            direction = (opos - robot_pos)
            if np.linalg.norm(direction) > 1e-6:
                direction = direction / np.linalg.norm(direction)
            step = proposed_vel * 0.1
            effective_d = max(d - float(np.dot(step, direction)), 1e-4)

            energy += coulomb_buckingham(
                effective_d,
                self.ust_epsilon, self.ust_r0, self.ust_alpha,
                self.ust_q1, self.ust_q2, self.ust_epsilon0,
            )

            # Velocity consensus: penalise difference from neighbour velocity
            vel_diff = proposed_vel - ovel
            energy += self.velocity_consensus_weight * float(
                np.dot(vel_diff, vel_diff)
            )

        return energy

    # ======================================================================
    # MCMC velocity sampling
    # ======================================================================

    def _mcmc_sample_velocity(self, ns: str) -> np.ndarray:
        """
        Metropolis-Hastings MCMC sampling for one robot.

        1. Start from the robot's current velocity.
        2. For *mcmc_iterations* iterations propose a new velocity drawn
           from a Gaussian centred on the current sample.
        3. Compute total energy E = U_s + U_t + U_st for the proposal.
        4. Accept with probability min(1, exp(-(E_new - E_old))).
        5. Discard the first *mcmc_burnin* fraction of samples (burn-in).
        6. Return the mean of the remaining accepted samples.
        """
        with self.data_lock:
            if ns not in self.robot_positions:
                return np.zeros(2)
            robot_pos = self.robot_positions[ns].copy()
            robot_yaw = self.robot_yaws[ns]
            current_vel = self.robot_velocities[ns].copy()
            scan = self.robot_scans.get(ns)

            # Build lists of other robots' positions and velocities
            other_positions = []
            other_velocities = []
            for other_ns in self.robot_namespaces:
                if other_ns == ns:
                    continue
                other_positions.append(self.robot_positions[other_ns].copy())
                other_velocities.append(self.robot_velocities[other_ns].copy())

        with self.model_lock:
            object_pos = self.object_position.copy() if self.object_position is not None else np.zeros(2)

        target_pos = np.array([self.target_x, self.target_y])

        # --- Energy helper ---
        def total_energy(vel: np.ndarray) -> float:
            e_s = self._compute_obstacle_energy(
                robot_pos, vel, scan, robot_yaw
            )
            e_t = self._compute_object_energy(robot_pos, vel,
                                               object_pos, target_pos)
            e_st = self._compute_robot_energy(robot_pos, vel,
                                               other_positions,
                                               other_velocities)
            return e_s + e_t + e_st

        # --- MCMC ---
        v_current = current_vel.copy()
        e_current = total_energy(v_current)

        samples: List[np.ndarray] = []

        for _ in range(self.mcmc_iterations):
            # Propose
            v_proposed = v_current + np.random.normal(0.0, self.mcmc_sigma, 2)
            # Clamp to vmax
            speed = np.linalg.norm(v_proposed)
            if speed > self.vmax:
                v_proposed = v_proposed / speed * self.vmax

            e_proposed = total_energy(v_proposed)

            # Metropolis acceptance
            delta_e = e_proposed - e_current
            if delta_e < 0.0 or np.random.random() < math.exp(-delta_e):
                v_current = v_proposed
                e_current = e_proposed

            samples.append(v_current.copy())

        # Discard burn-in
        burn = int(self.mcmc_burnin * len(samples))
        kept = samples[burn:] if burn < len(samples) else samples
        if len(kept) == 0:
            kept = samples

        # Average the remaining samples
        avg_vel = np.mean(kept, axis=0)

        # Final clamp
        speed = np.linalg.norm(avg_vel)
        if speed > self.vmax:
            avg_vel = avg_vel / speed * self.vmax

        return avg_vel

    # ======================================================================
    # Phase behaviours
    # ======================================================================

    def _search_phase(self, expected_epoch):
        """
        SEARCH: random walk with obstacle avoidance until the transport
        object is within sensing_range of *any* robot.
        """
        with self.model_lock:
            obj_pos = self.object_position.copy() if self.object_found else None

        with self.data_lock:
            namespaces = list(self.robot_namespaces)
            positions = {ns: self.robot_positions[ns].copy()
                         for ns in namespaces if ns in self.robot_positions}
            yaws = {ns: self.robot_yaws[ns]
                    for ns in namespaces if ns in self.robot_yaws}

        # Check if any robot can see the object
        if obj_pos is not None:
            for ns in namespaces:
                if ns not in positions:
                    continue
                d = float(np.linalg.norm(positions[ns] - obj_pos))
                if d < self.sensing_range:
                    rospy.loginfo(
                        "[transport] %s found object at distance %.2f  "
                        ">>> phase APPROACH", ns, d,
                    )
                    self._set_phase(
                        TransportPhase.APPROACH, expected_epoch
                    )
                    return

        # Random walk for every robot
        for ns in namespaces:
            if ns not in positions or ns not in yaws:
                continue

            # Slight random heading perturbation
            rand_ang = np.random.uniform(-0.5, 0.5)
            cmd = Twist()
            cmd.linear.x = self.vmax * 0.6
            cmd.angular.z = rand_ang

            # Let the ObstacleAvoidance module keep us safe
            with self.data_lock:
                avoidance = self.avoidance_modules.get(ns)
            if avoidance is not None:
                cmd = avoidance.apply_avoidance(cmd)

            self._publish_command(ns, cmd, expected_epoch)

    def _approach_phase(self, expected_epoch):
        """
        APPROACH: robots navigate toward the transport object using an
        attractive potential. Transition once the configured useful subset is
        ready; late joiners continue converging during PUSH.
        """
        with self.model_lock:
            if self.object_position is None:
                # Object lost -- fall back to SEARCH
                self._set_phase(TransportPhase.SEARCH, expected_epoch)
                rospy.logwarn("[transport] object lost  >>> phase SEARCH")
                return
            obj_pos = self.object_position.copy()

        with self.data_lock:
            namespaces = list(self.robot_namespaces)
            positions = {ns: self.robot_positions[ns].copy()
                         for ns in namespaces if ns in self.robot_positions}
            yaws = {ns: self.robot_yaws[ns]
                    for ns in namespaces if ns in self.robot_yaws}

        ready_count = 0

        for ns in namespaces:
            if ns not in positions or ns not in yaws:
                continue

            diff = obj_pos - positions[ns]
            d = float(np.linalg.norm(diff))

            if d <= self.sensing_range:
                ready_count += 1

            # Simple attractive velocity toward the object
            if d > 0.01:
                direction = diff / d
                speed = min(self.vmax, 0.5 * d)
                vx, vy = direction * speed
            else:
                vx, vy = 0.0, 0.0

            cmd = self._holonomic_to_diff_drive(vx, vy, yaws[ns])

            # Safety layer
            with self.data_lock:
                avoidance = self.avoidance_modules.get(ns)
            if avoidance is not None:
                cmd = avoidance.apply_avoidance(cmd)

            self._publish_command(ns, cmd, expected_epoch)

        required_count = min(len(namespaces), self.min_ready_robots)
        if required_count > 0 and ready_count >= required_count:
            rospy.loginfo(
                "[transport] %d/%d ready near object  >>> phase PUSH",
                ready_count, len(namespaces),
            )
            self._set_phase(TransportPhase.PUSH, expected_epoch)

    def _push_phase(self, expected_epoch):
        """
        PUSH: GRF-based coordinated pushing using MCMC velocity sampling.
        Transition to DONE when the object is within arrival_tolerance
        of the target.
        """
        with self.model_lock:
            if self.object_position is None:
                self._set_phase(TransportPhase.SEARCH, expected_epoch)
                rospy.logwarn("[transport] object lost during push  >>> SEARCH")
                return
            obj_pos = self.object_position.copy()
            obstacles = [
                point.copy() for point in self.obstacle_positions
            ]

        target_pos = np.array([self.target_x, self.target_y])
        dist_to_target = float(np.linalg.norm(obj_pos - target_pos))

        # Check arrival
        if dist_to_target < self.arrival_tolerance:
            rospy.loginfo(
                "[transport] object delivered (dist=%.2f)  >>> phase DONE",
                dist_to_target,
            )
            with self.command_lock:
                if not self._command_allowed(expected_epoch):
                    return
                self.is_running = False
                self.command_epoch += 1
                with self.phase_lock:
                    self.phase = TransportPhase.DONE
                self._stop_all_robots()
            return

        with self.data_lock:
            namespaces = list(self.robot_namespaces)
            yaws = {ns: self.robot_yaws[ns]
                    for ns in namespaces if ns in self.robot_yaws}

        if self.transport_planner == 'legacy':
            self._legacy_push_step(
                namespaces, yaws, expected_epoch
            )
            return

        try:
            commands, grf_yaws = self._grf_push_commands(
                namespaces, obj_pos, obstacles, expected_epoch
            )
        except Exception as exc:
            rospy.logwarn_throttle(
                5.0,
                "[transport] GRF planner failed ({}); "
                "using legacy planner for this step".format(exc),
            )
            self._legacy_push_step(
                namespaces, yaws, expected_epoch, fallback=True
            )
            return

        self._publish_grf_commands(
            namespaces, commands, grf_yaws, expected_epoch
        )

    # ======================================================================
    # 10 Hz control loop
    # ======================================================================

    def _control_loop(self, event):
        with self.command_lock:
            if (
                not self.is_running
                or self.is_paused
                or self.emergency_stop_active
            ):
                return
            expected_epoch = self.command_epoch
            expected_task_id = self.current_task_id

        with self.data_lock:
            positions = [
                (
                    namespace,
                    Point(
                        x=float(self.robot_positions[namespace][0]),
                        y=float(self.robot_positions[namespace][1]),
                        z=0.0,
                    ),
                )
                for namespace in self.robot_namespaces
                if namespace in self.robot_positions
            ]
            avoidance_modules = list(self.avoidance_modules.items())
        for namespace, avoidance in avoidance_modules:
            avoidance.update_robot_positions(positions)

        with self.phase_lock:
            phase = self.phase

        if phase == TransportPhase.SEARCH:
            self._search_phase(expected_epoch)
        elif phase == TransportPhase.APPROACH:
            self._approach_phase(expected_epoch)
        elif phase == TransportPhase.PUSH:
            self._push_phase(expected_epoch)
        elif phase == TransportPhase.DONE:
            pass  # remain idle
        elif phase == TransportPhase.IDLE:
            pass

        with self.phase_lock:
            phase = self.phase

        with self.command_lock:
            if self.current_task_id != expected_task_id:
                return
            if phase == TransportPhase.DONE:
                with self.phase_lock:
                    live_phase = self.phase
                if (
                    live_phase != TransportPhase.DONE
                    or self.is_running
                    or self.command_epoch != expected_epoch + 1
                ):
                    return
            elif self.command_epoch != expected_epoch:
                return

            # Keep task identity and lifecycle state stable through the status
            # publish. A new start cannot pair this cycle's terminal phase
            # with a later task ID.
            self._publish_status(
                phase,
                task_id=expected_task_id,
                paused=self.is_paused,
            )

        # Status is correlated above; visualisation may safely follow unlocked.
        self._publish_markers(phase)

    # ======================================================================
    # Status publishing
    # ======================================================================

    def _publish_status(
        self,
        phase: TransportPhase,
        task_id=None,
        paused=None,
    ):
        with self.model_lock:
            obj = self.object_position.tolist() if self.object_position is not None else [0, 0]

        target = [self.target_x, self.target_y]
        dist = math.sqrt((obj[0] - target[0]) ** 2 + (obj[1] - target[1]) ** 2)

        # Build per-robot assignment info
        assignments = {}
        with self.data_lock:
            for ns in self.robot_namespaces:
                pos = self.robot_positions.get(ns, np.zeros(2))
                assignments[ns] = {
                    'x': round(float(pos[0]), 3),
                    'y': round(float(pos[1]), 3),
                }

        status = {
            'task_id': self.current_task_id if task_id is None else task_id,
            'paused': self.is_paused if paused is None else paused,
            'phase': phase.value,
            'object_pos': {'x': round(obj[0], 3), 'y': round(obj[1], 3)},
            'target_pos': {'x': target[0], 'y': target[1]},
            'distance_to_target': round(dist, 3),
            'progress': round(max(0.0, 1.0 - dist / max(dist + 0.01, 1.0)), 3),
            'planner': self._active_planner,
            'grf_mcmc_iterations': self._active_grf_iterations,
            'robot_assignments': assignments,
        }
        self.status_pub.publish(String(data=json.dumps(status)))

    # ======================================================================
    # RViz visualisation
    # ======================================================================

    def _publish_markers(self, phase: TransportPhase):
        ma = MarkerArray()
        stamp = rospy.Time.now()

        # --- Object sphere ---
        with self.model_lock:
            if self.object_position is not None:
                m = Marker()
                m.header.frame_id = 'map'
                m.header.stamp = stamp
                m.ns = 'transport_object'
                m.id = 0
                m.type = Marker.SPHERE
                m.action = Marker.ADD
                m.pose.position.x = self.object_position[0]
                m.pose.position.y = self.object_position[1]
                m.pose.position.z = 0.15
                m.pose.orientation.w = 1.0
                m.scale.x = m.scale.y = m.scale.z = 0.3
                m.color.r = 0.9
                m.color.g = 0.5
                m.color.b = 0.0
                m.color.a = 0.9
                m.lifetime = rospy.Duration(0.2)
                ma.markers.append(m)

        # --- Target cylinder ---
        m = Marker()
        m.header.frame_id = 'map'
        m.header.stamp = stamp
        m.ns = 'transport_target'
        m.id = 1
        m.type = Marker.CYLINDER
        m.action = Marker.ADD
        m.pose.position.x = self.target_x
        m.pose.position.y = self.target_y
        m.pose.position.z = 0.01
        m.pose.orientation.w = 1.0
        m.scale.x = m.scale.y = 0.8
        m.scale.z = 0.02
        m.color.r = 0.0
        m.color.g = 1.0
        m.color.b = 0.0
        m.color.a = 0.5
        m.lifetime = rospy.Duration(0.2)
        ma.markers.append(m)

        # --- Arrow from object to target ---
        with self.model_lock:
            if self.object_position is not None:
                m = Marker()
                m.header.frame_id = 'map'
                m.header.stamp = stamp
                m.ns = 'transport_direction'
                m.id = 2
                m.type = Marker.ARROW
                m.action = Marker.ADD
                start = Point(x=self.object_position[0],
                              y=self.object_position[1], z=0.25)
                end = Point(x=self.target_x, y=self.target_y, z=0.25)
                m.points = [start, end]
                m.scale.x = 0.06
                m.scale.y = 0.12
                m.scale.z = 0.0
                m.color.r = 0.0
                m.color.g = 0.4
                m.color.b = 1.0
                m.color.a = 0.8
                m.lifetime = rospy.Duration(0.2)
                ma.markers.append(m)

        # --- Per-robot force arrows (only during PUSH) ---
        if phase == TransportPhase.PUSH:
            with self.data_lock:
                for idx, ns in enumerate(self.robot_namespaces):
                    if ns not in self.robot_positions:
                        continue
                    pos = self.robot_positions[ns]
                    vel = self.robot_velocities[ns]
                    m = Marker()
                    m.header.frame_id = 'map'
                    m.header.stamp = stamp
                    m.ns = 'robot_forces'
                    m.id = 100 + idx
                    m.type = Marker.ARROW
                    m.action = Marker.ADD
                    start = Point(x=pos[0], y=pos[1], z=0.2)
                    scale = 2.0  # exaggerate for visibility
                    end = Point(x=pos[0] + vel[0] * scale,
                                y=pos[1] + vel[1] * scale, z=0.2)
                    m.points = [start, end]
                    m.scale.x = 0.04
                    m.scale.y = 0.08
                    m.scale.z = 0.0
                    m.color.r = 1.0
                    m.color.g = 0.2
                    m.color.b = 0.2
                    m.color.a = 0.9
                    m.lifetime = rospy.Duration(0.2)
                    ma.markers.append(m)

        self.marker_pub.publish(ma)


# ======================================================================

if __name__ == '__main__':
    try:
        node = CollaborativeTransport()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
