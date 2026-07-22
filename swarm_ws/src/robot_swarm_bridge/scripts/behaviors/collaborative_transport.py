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
  APPROACH - Robots form payload-contact and companion-pushing chains
  PUSH     - GRF-based coordinated pushing with MCMC velocity sampling
  DONE     - Object delivered to target; all robots stop
  FAILED   - Payload state became unsafe or invalid; all robots stop

Works with dynamically changing TurtleBot3 Burger fleets in Gazebo.
"""

import rospy
import numpy as np
import json
import math
import threading
import time
import uuid
from collections import deque
from typing import List, Dict, Optional, Tuple
from enum import Enum

# ROS messages
from geometry_msgs.msg import Twist, Point, Quaternion
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String, Bool
from gazebo_msgs.msg import ModelState, ModelStates
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

from core.obstacle_avoidance import LidarRangeMask, ObstacleAvoidance
from utils.robot_ids import sort_robot_ids
from robot_swarm_bridge.algorithms.formation import (
    formation_targets_are_safe,
    hungarian_assignment,
    plan_obstacle_aware_route,
    routes_conflict,
    straight_route_is_safe,
)
from robot_swarm_bridge.algorithms.grf_adapter import (
    build_transport_snapshot,
    contact_center_distance,
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
    FAILED = "FAILED"


DEFAULT_ENGAGEMENT_HOLD_TIME = 0.25
DEFAULT_COMPRESSION_TRACKING_TOLERANCE = 0.025
DEFAULT_MODEL_STATES_TIMEOUT_WALL_S = 0.75
TRANSPORT_COLLISION_EVENT_HISTORY_LIMIT = 128
TRANSPORT_COLLISION_PROTOCOL_ERROR_LIMIT = 16


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
        self.vmax = rospy.get_param('~vmax', 0.16)
        self.sensing_range = rospy.get_param('~sensing_range', 2.0)
        self.search_speed = min(
            self.vmax,
            max(
                0.03,
                float(rospy.get_param(
                    '~search_speed', self.vmax * 0.75
                )),
            ),
        )
        self.search_angular_gain = max(
            0.1,
            float(rospy.get_param('~search_angular_gain', 1.4)),
        )
        self.search_max_angular_speed = min(
            1.2,
            max(
                0.2,
                float(rospy.get_param(
                    '~search_max_angular_speed', 0.8
                )),
            ),
        )
        self.search_waypoint_tolerance = max(
            0.08,
            float(rospy.get_param(
                '~search_waypoint_tolerance', 0.20
            )),
        )
        self.search_lane_overlap = min(
            0.95,
            max(
                0.25,
                float(rospy.get_param('~search_lane_overlap', 0.75)),
            ),
        )
        self.safezone = rospy.get_param('~safezone', 0.5)
        self.mcmc_iterations = rospy.get_param('~mcmc_iterations', 60)
        self.mcmc_sigma = rospy.get_param('~mcmc_sigma', 0.05)
        self.mcmc_burnin = rospy.get_param('~mcmc_burnin', 0.6)
        self.default_arrival_tolerance = min(
            0.75,
            max(
                0.15,
                float(rospy.get_param('~arrival_tolerance', 0.5)),
            ),
        )
        self.arrival_tolerance = self.default_arrival_tolerance
        self.transport_arrival_release_margin = min(
            0.25,
            max(
                0.02,
                float(rospy.get_param(
                    '~transport_arrival_release_margin', 0.10
                )),
            ),
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
            int(rospy.get_param('~grf_large_fleet_threshold', 10)),
        )
        requested_large_iterations = max(
            1,
            int(rospy.get_param(
                '~grf_large_fleet_iterations',
                min(12, self.grf_mcmc_iterations),
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
        self.robot_radius = float(
            rospy.get_param('~robot_radius', 0.11)
        )
        self.object_half_width = max(
            0.01,
            float(rospy.get_param('~object_half_width', 0.20)),
        )
        self.object_half_height = max(
            0.01,
            float(rospy.get_param('~object_half_height', 0.20)),
        )
        self.object_height = max(
            0.02,
            float(rospy.get_param('~object_height', 0.20)),
        )
        self.object_rest_z = float(rospy.get_param(
            '~object_rest_z', self.object_height / 2.0
        ))
        self.object_z_tolerance = max(
            0.01,
            float(rospy.get_param('~object_z_tolerance', 0.05)),
        )
        self.robot_forward_contact_extent = max(
            0.001,
            float(rospy.get_param(
                '~robot_forward_contact_extent', 0.038
            )),
        )
        self.robot_rear_contact_extent = max(
            0.001,
            float(rospy.get_param(
                '~robot_rear_contact_extent', 0.102
            )),
        )
        self.robot_lidar_collision_radius = max(
            0.001,
            float(rospy.get_param(
                '~robot_lidar_collision_radius', 0.055
            )),
        )
        self.robot_lidar_center_offset = float(rospy.get_param(
            '~robot_lidar_center_offset', -0.017
        ))
        self.robot_lidar_mask_tolerance = max(
            0.0,
            float(rospy.get_param(
                '~robot_lidar_mask_tolerance', 0.05
            )),
        )
        self.robot_lidar_mask_closer_tolerance = max(
            0.0,
            float(rospy.get_param(
                '~robot_lidar_mask_closer_tolerance', 0.04
            )),
        )
        requested_contact_slop = max(
            0.0,
            float(rospy.get_param('~transport_contact_slop', 0.005)),
        )
        self.transport_contact_slop = min(0.005, requested_contact_slop)
        self.transport_contact_heading_tolerance = min(
            math.pi / 2.0,
            max(
                0.05,
                float(rospy.get_param(
                    '~transport_contact_heading_tolerance', 0.45
                )),
            ),
        )
        # Two Burgers fit comfortably across the rear half of the 40 cm
        # payload. Extra robots line up behind those two and push the robot in
        # front of them, so every selected robot contributes forward force.
        requested_pushers = max(
            1, int(rospy.get_param('~max_pushing_robots', 2))
        )
        self.max_pushing_robots = min(2, requested_pushers)
        self.transport_push_slot_angle = min(
            0.80,
            max(
                0.35,
                float(rospy.get_param(
                    '~transport_push_slot_angle', 0.65
                )),
            ),
        )
        maximum_lane_offset = max(
            0.05,
            min(self.object_half_width, self.object_half_height) - 0.02,
        )
        self.transport_payload_lane_offset = min(
            maximum_lane_offset,
            max(
                0.05,
                float(rospy.get_param(
                    '~transport_payload_lane_offset', 0.18
                )),
            ),
        )
        self.transport_orbit_radius = max(
            0.50,
            float(rospy.get_param('~transport_orbit_radius', 0.60)),
        )
        self.transport_staging_clearance = min(
            0.50,
            max(
                0.05,
                float(rospy.get_param(
                    '~transport_staging_clearance', 0.25
                )),
            ),
        )
        self.transport_engagement_speed = min(
            self.vmax,
            max(
                0.04,
                float(rospy.get_param(
                    '~transport_engagement_speed', 0.12
                )),
            ),
        )
        self.transport_engagement_hold_time = max(
            0.0,
            float(rospy.get_param(
                '~transport_engagement_hold_time',
                DEFAULT_ENGAGEMENT_HOLD_TIME,
            )),
        )
        self.transport_engagement_release_hold_time = min(
            0.75,
            max(
                0.0,
                float(rospy.get_param(
                    '~transport_engagement_release_hold_time', 0.35
                )),
            ),
        )
        self.transport_launch_heading_tolerance = min(
            0.20,
            max(
                0.04,
                float(rospy.get_param(
                    '~transport_launch_heading_tolerance', 0.08
                )),
            ),
        )
        self.transport_launch_settle_time = min(
            1.0,
            max(
                0.10,
                float(rospy.get_param(
                    '~transport_launch_settle_time', 0.35
                )),
            ),
        )
        self.transport_launch_settle_timeout = max(
            self.transport_launch_settle_time,
            min(
                3.0,
                float(rospy.get_param(
                    '~transport_launch_settle_timeout', 1.25
                )),
            ),
        )
        self.transport_launch_settle_speed = min(
            0.01,
            max(
                0.001,
                float(rospy.get_param(
                    '~transport_launch_settle_speed', 0.004
                )),
            ),
        )
        self.transport_companion_engagement_angle = min(
            0.50,
            max(
                0.15,
                float(rospy.get_param(
                    '~transport_companion_engagement_angle', 0.20
                )),
            ),
        )
        self.transport_engagement_release_angle_margin = min(
            0.30,
            max(
                0.05,
                float(rospy.get_param(
                    '~transport_engagement_release_angle_margin', 0.15
                )),
            ),
        )
        self.transport_engagement_minimum_closing_error = min(
            0.04,
            max(
                0.01,
                float(rospy.get_param(
                    '~transport_engagement_minimum_closing_error', 0.02
                )),
            ),
        )
        self.transport_alignment_retreat_clearance = min(
            0.08,
            max(
                0.02,
                float(rospy.get_param(
                    '~transport_alignment_retreat_clearance', 0.055
                )),
            ),
        )
        self.transport_push_heading_cone = min(
            0.35,
            max(
                0.08,
                float(rospy.get_param(
                    '~transport_push_heading_cone', 0.18
                )),
            ),
        )
        self.transport_chain_heading_cone = min(
            0.12,
            max(
                0.03,
                float(rospy.get_param(
                    '~transport_chain_heading_cone', 0.06
                )),
            ),
        )
        self.transport_push_angular_limit = min(
            0.60,
            max(
                0.15,
                float(rospy.get_param(
                    '~transport_push_angular_limit', 0.25
                )),
            ),
        )
        self.transport_companion_preload = min(
            0.05,
            max(
                0.01,
                float(rospy.get_param(
                    '~transport_companion_preload', 0.018
                )),
            ),
        )
        self.transport_single_companion_preload = min(
            0.05,
            max(
                self.transport_companion_preload,
                float(rospy.get_param(
                    '~transport_single_companion_preload', 0.025
                )),
            ),
        )
        self.transport_push_spacing_deadband = min(
            0.01,
            max(
                0.001,
                float(rospy.get_param(
                    '~transport_push_spacing_deadband', 0.003
                )),
            ),
        )
        self.transport_push_yield_gain = min(
            3.0,
            max(
                0.25,
                float(rospy.get_param(
                    '~transport_push_yield_gain', 1.5
                )),
            ),
        )
        self.transport_push_max_yield_speed = min(
            0.03,
            max(
                0.004,
                float(rospy.get_param(
                    '~transport_push_max_yield_speed', 0.012
                )),
            ),
        )
        self.transport_min_useful_push_speed = min(
            self.vmax,
            max(
                0.016,
                float(rospy.get_param(
                    '~transport_min_useful_push_speed', 0.035
                )),
            ),
        )
        self.transport_push_ramp_initial_speed = min(
            self.transport_min_useful_push_speed,
            max(
                0.0,
                float(rospy.get_param(
                    '~transport_push_ramp_initial_speed', 0.018
                )),
            ),
        )
        self.transport_push_ramp_rate = min(
            0.10,
            max(
                0.0,
                float(rospy.get_param(
                    '~transport_push_ramp_rate', 0.08
                )),
            ),
        )
        # A wall-time-heavy N-robot control pass may span several simulated
        # timer periods at 3x.  Advance the loaded-chain ramp once per complete
        # fleet batch so a missed deadline cannot make the front row jump
        # ahead while the rear row still has its previous command.
        self.transport_push_ramp_step = min(
            0.02,
            max(
                0.001,
                float(rospy.get_param(
                    '~transport_push_ramp_step', 0.006
                )),
            ),
        )
        self.transport_push_ramp_hold_batches = max(
            0,
            min(
                5,
                int(rospy.get_param(
                    '~transport_push_ramp_hold_batches', 2
                )),
            ),
        )
        self.transport_all_push_hold_time = max(
            0.0,
            float(rospy.get_param(
                '~transport_all_push_hold_time', 0.75
            )),
        )
        self.transport_contact_closing_speed = min(
            0.03,
            max(
                0.012,
                float(rospy.get_param(
                    '~transport_contact_closing_speed', 0.018
                )),
            ),
        )
        self.transport_assembly_closing_speed = min(
            self.transport_contact_closing_speed,
            max(
                0.006,
                float(rospy.get_param(
                    '~transport_assembly_closing_speed', 0.012
                )),
            ),
        )
        self.transport_terminal_closing_speed = min(
            0.015,
            max(
                0.006,
                float(rospy.get_param(
                    '~transport_terminal_closing_speed', 0.010
                )),
            ),
        )
        self.transport_push_recovery_closing_speed = min(
            0.03,
            max(
                0.006,
                float(rospy.get_param(
                    '~transport_push_recovery_closing_speed', 0.025
                )),
            ),
        )
        self.transport_single_companion_recovery_speed = min(
            0.025,
            max(
                self.transport_push_recovery_closing_speed,
                float(rospy.get_param(
                    '~transport_single_companion_recovery_speed', 0.025
                )),
            ),
        )
        companion_contact_distance = (
            self.robot_forward_contact_extent
            + self.robot_rear_contact_extent
        )
        self.transport_companion_contact_distance = max(
            companion_contact_distance,
            float(rospy.get_param(
                '~transport_companion_contact_distance',
                companion_contact_distance + self.transport_contact_slop,
            )),
        )
        self.transport_chain_corridor_width = min(
            0.18,
            max(
                0.08,
                float(rospy.get_param(
                    '~transport_chain_corridor_width', 0.12
                )),
            ),
        )
        self.transport_chain_alignment_tolerance = min(
            math.pi / 2.0,
            max(
                0.10,
                float(rospy.get_param(
                    '~transport_chain_alignment_tolerance', 0.55
                )),
            ),
        )
        self.transport_chain_staging_spacing = min(
            0.50,
            max(
                self.transport_companion_contact_distance + 0.15,
                float(rospy.get_param(
                    '~transport_chain_staging_spacing', 0.31
                )),
            ),
        )
        self.transport_large_fleet_staging_spacing = min(
            0.60,
            max(
                self.transport_chain_staging_spacing,
                float(rospy.get_param(
                    '~transport_large_fleet_staging_spacing', 0.38
                )),
            ),
        )
        self.transport_large_fleet_staging_lateral_offset = min(
            0.50,
            max(
                0.0,
                float(rospy.get_param(
                    '~transport_large_fleet_staging_lateral_offset', 0.34
                )),
            ),
        )
        self.transport_chain_staging_speed = min(
            self.vmax,
            max(
                0.06,
                float(rospy.get_param(
                    '~transport_chain_staging_speed', 0.16
                )),
            ),
        )
        self.transport_compression_speed = min(
            self.transport_chain_staging_speed,
            max(
                0.04,
                float(rospy.get_param(
                    '~transport_compression_speed', 0.12
                )),
            ),
        )
        self.transport_compression_tracking_tolerance = min(
            0.05,
            max(
                0.015,
                float(rospy.get_param(
                    '~transport_compression_tracking_tolerance',
                    DEFAULT_COMPRESSION_TRACKING_TOLERANCE,
                )),
            ),
        )
        self.transport_chain_staging_tolerance = min(
            0.03,
            max(
                0.005,
                float(rospy.get_param(
                    '~transport_chain_staging_tolerance', 0.025
                )),
            ),
        )
        self.transport_chain_staging_release_tolerance = min(
            0.10,
            max(
                self.transport_chain_staging_tolerance,
                float(rospy.get_param(
                    '~transport_chain_staging_release_tolerance', 0.06
                )),
            ),
        )
        self.transport_chain_staging_contact_tolerance = min(
            0.003,
            max(
                0.0,
                float(rospy.get_param(
                    '~transport_chain_staging_contact_tolerance', 0.003
                )),
            ),
        )
        self.transport_chain_assembly_gap = min(
            0.10,
            max(
                0.015,
                float(rospy.get_param(
                    '~transport_chain_assembly_gap', 0.10
                )),
            ),
        )
        self.arena_size = float(rospy.get_param('~arena_size', 10.0))
        self.arena_margin = max(
            0.0, float(rospy.get_param('~arena_margin', 0.35))
        )
        self.arena_profile = str(
            rospy.get_param('~arena_profile', 'swarm_arena')
        )
        self.transport_odom_timeout = max(
            0.2,
            float(rospy.get_param('~transport_odom_timeout', 2.0)),
        )
        self.model_states_timeout_wall_s = max(
            0.2,
            float(rospy.get_param(
                '~model_states_timeout_wall_s',
                DEFAULT_MODEL_STATES_TIMEOUT_WALL_S,
            )),
        )
        self.transport_payload_target_margin = max(
            0.0,
            float(rospy.get_param(
                '~transport_payload_target_margin', 0.05
            )),
        )
        self.transport_route_obstacle_clearance = max(
            0.0,
            float(rospy.get_param('~spawn_obstacle_clearance', 0.30)),
        )
        self.transport_route_robot_clearance = max(
            self.robot_radius * 2.0 + 0.03,
            float(rospy.get_param(
                '~transport_route_robot_clearance', 0.32
            )),
        )
        self.transport_rendezvous_clearance_hysteresis = min(
            0.12,
            max(
                0.02,
                float(rospy.get_param(
                    '~transport_rendezvous_clearance_hysteresis', 0.04
                )),
            ),
        )
        self.transport_rendezvous_release_hold_time = max(
            0.0,
            float(rospy.get_param(
                '~transport_rendezvous_release_hold_time', 0.40
            )),
        )
        self.transport_rendezvous_yield_distance = min(
            0.40,
            max(
                0.18,
                float(rospy.get_param(
                    '~transport_rendezvous_yield_distance', 0.28
                )),
            ),
        )
        self.transport_parallel_row_minimum_clearance = max(
            self.robot_radius * 2.0 + 0.03,
            float(rospy.get_param(
                '~transport_parallel_row_minimum_clearance', 0.25
            )),
        )
        self.transport_route_parent_clearance = max(
            self.transport_companion_contact_distance + 0.015,
            float(rospy.get_param(
                '~transport_route_parent_clearance', 0.20
            )),
        )
        self.transport_route_waypoint_tolerance = max(
            0.04,
            float(rospy.get_param(
                '~transport_route_waypoint_tolerance', 0.09
            )),
        )
        self.transport_parking_route_waypoint_tolerance = min(
            self.transport_route_waypoint_tolerance,
            max(
                0.02,
                float(rospy.get_param(
                    '~transport_parking_route_waypoint_tolerance', 0.04
                )),
            ),
        )
        self.transport_route_final_handoff_tolerance = max(
            self.transport_route_waypoint_tolerance,
            float(rospy.get_param(
                '~transport_route_final_handoff_tolerance', 0.25
            )),
        )
        self.transport_assembly_handoff_tolerance = min(
            0.05,
            max(
                0.015,
                float(rospy.get_param(
                    '~transport_assembly_handoff_tolerance', 0.015
                )),
            ),
        )
        self.transport_assembly_heading_tolerance = min(
            0.15,
            max(
                0.05,
                float(rospy.get_param(
                    '~transport_assembly_heading_tolerance', 0.10
                )),
            ),
        )
        self.transport_assembly_contact_lateral_tolerance = min(
            0.025,
            max(
                0.010,
                float(rospy.get_param(
                    '~transport_assembly_contact_lateral_tolerance',
                    0.022,
                )),
            ),
        )
        self.transport_payload_route_awareness = max(
            0.20,
            float(rospy.get_param('~awareness_radius', 0.80)),
        )
        self.transport_payload_route_lidar_offset = abs(float(
            rospy.get_param('~lidar_offset_x', -0.032)
        ))
        self.transport_payload_route_margin = max(
            0.03,
            float(rospy.get_param(
                '~transport_payload_route_margin', 0.05
            )),
        )
        self.transport_payload_gate_half_width = max(
            0.08,
            float(rospy.get_param(
                '~transport_payload_gate_half_width', 0.12
            )),
        )
        self.transport_payload_gate_plane_tolerance = max(
            0.01,
            float(rospy.get_param(
                '~transport_payload_gate_plane_tolerance', 0.02
            )),
        )
        self.transport_payload_approach_min_clearance = max(
            0.04,
            float(rospy.get_param(
                '~transport_payload_approach_min_clearance', 0.08
            )),
        )
        self.spawn_exclusion_zones = rospy.get_param(
            '~spawn_exclusion_zones', []
        )
        if not isinstance(self.spawn_exclusion_zones, list):
            self.spawn_exclusion_zones = []
        self.transport_progress_timeout = max(
            3.0,
            float(rospy.get_param('~transport_progress_timeout', 20.0)),
        )
        self.transport_progress_delta = max(
            0.005,
            float(rospy.get_param('~transport_progress_delta', 0.005)),
        )
        self.transport_ready_hold_time = max(
            0.0,
            float(rospy.get_param('~transport_ready_hold_time', 0.30)),
        )
        self.transport_payload_recovery_margin = min(
            0.10,
            max(
                0.015,
                float(rospy.get_param(
                    '~transport_payload_recovery_margin', 0.05
                )),
            ),
        )
        self.transport_payload_contact_release_margin = min(
            0.08,
            max(
                0.015,
                float(rospy.get_param(
                    '~transport_payload_contact_release_margin', 0.04
                )),
            ),
        )
        # Kept as the nominal face-on distance for configuration bounds.  The
        # phase transition below uses the rotated box surface in each robot's
        # direction, so corner contacts are handled correctly too.
        self.transport_contact_distance = contact_center_distance(
            min(self.object_half_width, self.object_half_height),
            self.robot_forward_contact_extent,
            self.transport_contact_slop,
        )
        requested_avoidance_range = float(
            rospy.get_param('~object_avoidance_range', 1.10)
        )
        self.object_avoidance_range = max(
            self.transport_contact_distance,
            requested_avoidance_range,
        )
        self.object_lidar_tolerance = max(
            0.0,
            float(rospy.get_param('~object_lidar_tolerance', 0.06)),
        )
        self.object_lidar_closer_tolerance = max(
            0.0,
            min(
                0.03,
                float(rospy.get_param(
                    '~object_lidar_closer_tolerance', 0.025
                )),
            ),
        )
        self.object_lidar_docking_closer_tolerance = max(
            self.object_lidar_closer_tolerance,
            min(
                0.04,
                float(rospy.get_param(
                    '~object_lidar_docking_closer_tolerance', 0.035
                )),
            ),
        )
        self.object_lidar_contact_closer_tolerance = max(
            self.object_lidar_closer_tolerance,
            min(
                0.08,
                float(rospy.get_param(
                    '~object_lidar_contact_closer_tolerance', 0.07
                )),
            ),
        )
        self.object_lidar_envelope_tolerance = max(
            0.0,
            float(rospy.get_param(
                '~object_lidar_envelope_tolerance', 0.10
            )),
        )
        self.object_lidar_envelope_closer_tolerance = max(
            0.0,
            float(rospy.get_param(
                '~object_lidar_envelope_closer_tolerance', 0.02
            )),
        )
        self.object_lidar_corner_radius = max(
            0.01,
            float(rospy.get_param(
                '~object_lidar_corner_radius', 0.08
            )),
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
        self.object_potential_weight = float(
            rospy.get_param('~grf_object_potential_weight', 0.02)
        )
        self.neighbor_potential_weight = float(
            rospy.get_param('~grf_neighbor_potential_weight', 0.02)
        )
        self.orbit_alignment_weight = float(
            rospy.get_param('~grf_orbit_alignment_weight', 60.0)
        )
        self.push_alignment_weight = float(
            rospy.get_param('~grf_push_alignment_weight', 80.0)
        )
        self.velocity_consensus_weight = float(
            rospy.get_param('~grf_velocity_consensus_weight', 12.0)
        )
        self._grf_kernels: Dict[int, GibbsRandomFieldTransport] = {}
        self._grf_step_index = 0
        self._active_planner = self.transport_planner
        self._active_grf_iterations = 0
        self.transport_roles = {}
        self.transport_staged = set()
        self.transport_pre_staged = set()
        self.transport_chain_released = set()
        self.transport_compression_progress = 0.0
        self.transport_compression_updated_at = None
        self.transport_last_rendezvous_log_count = None
        self.transport_rendezvous_recoveries = {}
        self.transport_rendezvous_recovery_cooldowns = {}
        self.transport_rendezvous_pair_decisions = {}
        self.transport_all_ready_since = None
        self.transport_best_distance = None
        self.transport_last_progress_time = None
        self.transport_initial_target_distance = None
        self.transport_reported_progress = 0.0
        self.transport_near_target_recovery_started_at = None
        self.transport_arrival_latched = False
        self.transport_arrival_direction = None
        self.transport_route_namespace = None
        self.transport_route_kind = None
        self.transport_route_target = None
        self.transport_route_waypoints = []
        self.transport_route_waypoint_index = 0
        self.transport_route_last_plan_time = None
        self.transport_route_complete = False
        self.transport_route_reverse_active = False
        self.transport_route_reverse_finished = False
        self.transport_route_turn_direction = 0.0
        self.transport_assembly_route_states = {}
        self.transport_last_commands = {}
        self.transport_engaged = set()
        self.transport_physical_engaged = set()
        self.transport_aligned_engaged = set()
        self.transport_engagement_complete = False
        self.transport_engagement_ready_since = None
        self.transport_engagement_last_ready = {}
        self.transport_launch_settle_started_at = None
        self.transport_queue_docking_started = False
        self.transport_queue_settle_started_at = None
        self.transport_synchronized_push_started = False
        self.transport_push_ramp_started_at = None
        self.transport_push_reference_speed = None
        self.transport_push_ramp_batches = 0
        self.transport_control_sequence = 0
        self.transport_last_batch_publish_span = 0.0
        self.transport_last_control_sim_time = None
        self.transport_last_control_commands = {}
        self.transport_push_raw_speeds = {}
        self.transport_push_coordinated_speeds = {}
        self.transport_push_link_states = {}
        self.transport_push_hard_stop_sources = ()
        self.transport_useful_contributors = set()
        self.transport_current_useful_pushers = set()
        self.transport_all_pushers_confirmed = False
        self.transport_all_pushers_since = None
        self.transport_engagement_parent_distances = {}
        self.transport_near_target_recovery_started_at = None
        self.transport_discovery = None
        self.search_routes = {}
        self.search_route_indices = {}
        self.search_route_directions = {}
        self.search_route_signature = ()
        self.search_navigation_routes = {}
        self.search_navigation_indices = {}
        self.search_navigation_destinations = {}
        self.search_navigation_targets = {}

        # ---- Phase / state --------------------------------------------------
        self.phase = TransportPhase.IDLE
        self.is_running = False
        self.is_paused = False
        self.emergency_stop_active = False
        self.emergency_reset_pending = False
        self.current_task_id = None
        self.command_epoch = 0
        self.phase_lock = threading.Lock()
        self.command_lock = threading.RLock()
        # Phase methods update a related set of task-state fields.  Keep one
        # complete cycle from overlapping a lifecycle reset; command_epoch
        # remains the fast cancellation gate for individual Twist publishes.
        self.control_cycle_lock = threading.Lock()
        self.transport_collision_lock = threading.Lock()
        self.transport_collision_source_id = uuid.uuid4().hex
        self.transport_collision_source_sequence = 0
        self.transport_collision_task_id = None
        self.transport_collision_task_start_sequence = 0
        self.transport_collision_events = deque(
            maxlen=TRANSPORT_COLLISION_EVENT_HISTORY_LIMIT
        )
        self.transport_collision_protocol_errors = deque(
            maxlen=TRANSPORT_COLLISION_PROTOCOL_ERROR_LIMIT
        )
        self.transport_safety_control_sequence = 0
        self.transport_active_safety_context = None

        # ---- Robot bookkeeping ----------------------------------------------
        # Populated dynamically from /fleet/robot_list
        self.robot_namespaces: List[str] = []

        # Per-robot state  (keyed by namespace string)
        self.robot_positions: Dict[str, np.ndarray] = {}    # [x, y]
        self.robot_yaws: Dict[str, float] = {}
        self.robot_velocities: Dict[str, np.ndarray] = {}   # [vx, vy] world
        self.robot_odom_received_at: Dict[str, Optional[float]] = {}
        self.robot_scans: Dict[str, LaserScan] = {}
        self.avoidance_modules: Dict[str, ObstacleAvoidance] = {}

        self.data_lock = threading.Lock()

        # ---- Object / obstacle tracking from Gazebo -------------------------
        self.object_position: Optional[np.ndarray] = None  # [x, y]
        self.object_velocity = np.zeros(2)                 # [vx, vy] world
        self.object_yaw: float = 0.0
        self.object_z: Optional[float] = None
        self.object_error: Optional[str] = None
        self.failure_reason: Optional[str] = None
        self.model_states_received_at: Optional[float] = None
        self.target_marker_name = 'target_marker'
        self.target_marker_position: Optional[np.ndarray] = None
        self.target_marker_synced = False
        self.target_marker_command_published = False
        self.target_marker_sync_tolerance = 0.02
        self.object_found = False
        self.obstacle_positions: List[np.ndarray] = []     # list of [x,y]
        self.model_poses: Dict[str, Tuple[float, float, float]] = {}
        self.model_lock = threading.Lock()

        # ---- Publishers (per-robot cmd_vel created in _setup_robot) ----------
        self.cmd_vel_pubs: Dict[str, rospy.Publisher] = {}
        self.odom_subs: Dict[str, rospy.Subscriber] = {}
        self.scan_subs: Dict[str, rospy.Subscriber] = {}

        self.status_pub = rospy.Publisher(
            '/transport/status', String, queue_size=1
        )
        self.discovery_pub = rospy.Publisher(
            '/transport/discovery', String, queue_size=1
        )
        self.marker_pub = rospy.Publisher(
            '/transport/markers', MarkerArray, queue_size=1
        )
        # Queue the visual command without waiting on Gazebo while the task
        # lifecycle locks are held. ModelStates provides the real confirmation.
        self.target_marker_pub = rospy.Publisher(
            '/gazebo/set_model_state', ModelState,
            queue_size=1, latch=True,
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
        rospy.on_shutdown(self._shutdown)

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
        with self._control_cycle_mutex():
            self._update_fleet_serialized(ns_list)

    def _update_fleet_serialized(self, ns_list: List[str]):
        """Synchronise internal bookkeeping with the given namespace list."""
        with self.command_lock:
            roster_changed = list(ns_list) != self.robot_namespaces
            if roster_changed and (self.is_running or self.is_paused):
                # A control pass may still be working from the previous pose
                # and publisher roster.  Give the changed fleet a new command
                # epoch so that old pass cannot publish or commit its result.
                self.command_epoch += 1
                self._stop_all_robots()
            if not ns_list and (self.is_running or self.is_paused):
                self.is_running = False
                self.is_paused = False
                self._clear_transport_progress()
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
            if roster_changed:
                self.transport_roles = {}
                self.transport_staged = set()
                self.transport_pre_staged = set()
                self.transport_chain_released = set()
                self.transport_all_ready_since = None
                self._reset_transport_route()
                self._reset_search_routes()

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
        if not hasattr(self, 'robot_odom_received_at'):
            self.robot_odom_received_at = {}
        self.robot_odom_received_at[ns] = None

        # Obstacle avoidance owns the filtered physical-contact edge.  Its
        # callback stays synchronous with this controller's safety pass so the
        # event cannot inherit a later transport phase from ROS callback
        # scheduling.
        self.avoidance_modules[ns] = ObstacleAvoidance(
            ns,
            collision_edge_callback=self._record_transport_collision_edge,
        )

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
                  self.robot_velocities, self.robot_scans,
                  getattr(self, 'robot_odom_received_at', {})):
            d.pop(ns, None)
        getattr(self, 'transport_last_commands', {}).pop(ns, None)
        if ns in self.robot_namespaces:
            self.robot_namespaces.remove(ns)
        rospy.loginfo("[transport] unregistered robot %s", ns)

    # ======================================================================
    # Subscriber callbacks
    # ======================================================================

    def _odom_callback(self, ns: str, msg: Odometry):
        pos = msg.pose.pose.position
        yaw = self._quat_to_yaw(msg.pose.pose.orientation)
        pose_values = (float(pos.x), float(pos.y), float(yaw))
        if not all(math.isfinite(value) for value in pose_values):
            rospy.logwarn_throttle(
                2.0,
                "[transport] ignoring invalid odometry from {}".format(ns),
            )
            return

        get_time = getattr(rospy, 'get_time', lambda: 0.0)
        received_at = float(get_time())
        if not math.isfinite(received_at):
            rospy.logwarn_throttle(
                2.0,
                "[transport] ignoring {} odometry without a valid clock".format(
                    ns
                ),
            )
            return

        with self.data_lock:
            if ns not in self.cmd_vel_pubs:
                return
            self.robot_positions[ns] = np.array(pose_values[:2])
            self.robot_yaws[ns] = yaw
            v = msg.twist.twist
            # Odometry twist is expressed in the robot body frame. Convert it
            # before comparing with GRF samples, which are world-frame vectors.
            cos_yaw = math.cos(yaw)
            sin_yaw = math.sin(yaw)
            velocity = np.array([
                v.linear.x * cos_yaw - v.linear.y * sin_yaw,
                v.linear.x * sin_yaw + v.linear.y * cos_yaw,
            ])
            if not np.all(np.isfinite(velocity)):
                velocity = np.zeros(2)
            self.robot_velocities[ns] = velocity
            if not hasattr(self, 'robot_odom_received_at'):
                self.robot_odom_received_at = {}
            self.robot_odom_received_at[ns] = received_at

    def _scan_callback(self, ns: str, msg: LaserScan):
        with self.data_lock:
            if ns not in self.cmd_vel_pubs:
                return
            self.robot_scans[ns] = msg

    def _model_states_callback(self, msg: ModelStates):
        """Track the transport object and other non-robot models (obstacles)."""
        received_at = float(time.monotonic())
        with self.model_lock:
            self.model_states_received_at = (
                received_at if math.isfinite(received_at) else None
            )
            self.obstacle_positions = []
            self.model_poses = {}
            object_pose = None
            object_twist = None
            target_marker_position = None
            for i, name in enumerate(msg.name):
                px = msg.pose[i].position.x
                py = msg.pose[i].position.y
                if math.isfinite(px) and math.isfinite(py):
                    self.model_poses[name] = (
                        px,
                        py,
                        self._quat_to_yaw(msg.pose[i].orientation),
                    )
                    if name == getattr(
                        self, 'target_marker_name', 'target_marker'
                    ):
                        target_marker_position = np.array([px, py])
                if name == self.object_name:
                    object_pose = msg.pose[i]
                    twists = getattr(msg, 'twist', ())
                    if i < len(twists):
                        object_twist = twists[i]
                elif name.startswith('obstacle_') or name.startswith('wall_'):
                    # Only explicitly named arena obstacles are included.
                    if math.isfinite(px) and math.isfinite(py):
                        self.obstacle_positions.append(np.array([px, py]))

            self.target_marker_position = target_marker_position
            marker_target = np.array(
                [
                    getattr(self, 'target_x', float('nan')),
                    getattr(self, 'target_y', float('nan')),
                ],
                dtype=float,
            )
            self.target_marker_synced = bool(
                target_marker_position is not None
                and np.all(np.isfinite(marker_target))
                and float(np.linalg.norm(
                    target_marker_position - marker_target
                )) <= getattr(self, 'target_marker_sync_tolerance', 0.02)
            )

            was_found = self.object_found
            self.object_found = False
            self.object_position = None
            self.object_velocity = np.zeros(2)
            self.object_error = None

            if object_pose is None:
                self.object_z = None
                self.object_error = "Transport payload is missing from Gazebo"
                return

            position = object_pose.position
            orientation = object_pose.orientation
            self.object_z = float(position.z)
            pose_values = (
                position.x, position.y, position.z,
                orientation.x, orientation.y,
                orientation.z, orientation.w,
            )
            quaternion_norm = math.sqrt(
                orientation.x * orientation.x
                + orientation.y * orientation.y
                + orientation.z * orientation.z
                + orientation.w * orientation.w
            )
            minimum_z = self.object_rest_z - self.object_z_tolerance
            maximum_z = self.object_rest_z + self.object_z_tolerance

            if not all(math.isfinite(value) for value in pose_values):
                self.object_error = "Transport payload pose is not finite"
                return
            if quaternion_norm < 0.5:
                self.object_error = "Transport payload orientation is invalid"
                return
            if not minimum_z <= position.z <= maximum_z:
                self.object_error = (
                    "Transport payload left the supported floor plane "
                    "(z={:.3f}m, expected {:.3f}..{:.3f}m)".format(
                        position.z, minimum_z, maximum_z
                    )
                )
                return

            self.object_position = np.array([position.x, position.y])
            if object_twist is not None:
                velocity = object_twist.linear
                if math.isfinite(velocity.x) and math.isfinite(velocity.y):
                    self.object_velocity = np.array([
                        velocity.x, velocity.y
                    ])
            self.object_yaw = self._quat_to_yaw(orientation)
            self.object_found = True
            if not was_found:
                rospy.loginfo(
                    "[transport] object '%s' detected at (%.2f, %.2f)",
                    self.object_name,
                    self.object_position[0],
                    self.object_position[1],
                )

    def _place_target_marker(self):
        """Move the collision-free Gazebo ghost to this task's destination."""
        publisher = getattr(self, 'target_marker_pub', None)
        self.target_marker_command_published = False
        self.target_marker_synced = False
        if publisher is None:
            return

        state = ModelState()
        state.model_name = getattr(
            self, 'target_marker_name', 'target_marker'
        )
        state.reference_frame = 'world'
        state.pose.position.x = self.target_x
        state.pose.position.y = self.target_y
        state.pose.position.z = 0.0
        state.pose.orientation.w = 1.0
        try:
            publisher.publish(state)
        except Exception as exc:
            rospy.logwarn(
                "[transport] could not publish the Gazebo target marker: %s",
                exc,
            )
            return
        self.target_marker_command_published = True

    def _start_callback(self, msg):
        with self._control_cycle_mutex():
            self._start_callback_serialized(msg)

    def _start_callback_serialized(self, msg):
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

            default_tolerance = float(getattr(
                self,
                'default_arrival_tolerance',
                getattr(self, 'arrival_tolerance', 0.5),
            ))
            requested_tolerance = config.get(
                'arrival_tolerance', default_tolerance
            )
            try:
                requested_tolerance = float(requested_tolerance)
            except (TypeError, ValueError, OverflowError):
                requested_tolerance = default_tolerance
            if (
                not math.isfinite(requested_tolerance)
                or not 0.15 <= requested_tolerance <= 0.75
            ):
                rospy.logwarn(
                    "[transport] arrival_tolerance must be between "
                    "0.15 and 0.75 m; using %.2f m",
                    default_tolerance,
                )
                requested_tolerance = default_tolerance
            self.arrival_tolerance = requested_tolerance

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
            self.transport_roles = {}
            self.transport_staged = set()
            self.transport_pre_staged = set()
            self.transport_chain_released = set()
            self.transport_all_ready_since = None
            self.transport_best_distance = None
            self.transport_last_progress_time = None
            self._reset_transport_route(reset_control_sequence=True)
            self._reset_search_state()
            self._begin_transport_progress()
            self.object_error = None
            self.failure_reason = None
            self.current_task_id = config.get('task_id')
            self._place_target_marker()
            self._begin_transport_collision_stream(self.current_task_id)
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
        with self._control_cycle_mutex():
            self._stop_callback_serialized(msg)

    def _stop_callback_serialized(self, msg: String):
        with self.command_lock:
            if not self._task_command_matches(msg):
                return
            self.is_running = False
            self.is_paused = False
            self.object_error = None
            self.failure_reason = None
            self.transport_roles = {}
            self.transport_staged = set()
            self.transport_pre_staged = set()
            self.transport_chain_released = set()
            self.transport_all_ready_since = None
            self.transport_best_distance = None
            self.transport_last_progress_time = None
            self._reset_transport_route(reset_control_sequence=True)
            self._reset_search_state()
            self._clear_transport_progress()
            self.command_epoch += 1
            with self.phase_lock:
                self.phase = TransportPhase.IDLE
            self._stop_all_robots()
        rospy.loginfo("[transport] stopped (IDLE)")

    def _pause_callback(self, msg: String):
        with self._control_cycle_mutex():
            self._pause_callback_serialized(msg)

    def _pause_callback_serialized(self, msg: String):
        with self.command_lock:
            if not self._task_command_matches(msg):
                return
            if not self.is_running or self.is_paused:
                return
            self.is_paused = True
            self._reset_transport_route(preserve_approach=True)
            self.command_epoch += 1
            self._stop_all_robots()
        rospy.loginfo("[transport] paused")

    def _resume_callback(self, msg: String):
        with self._control_cycle_mutex():
            self._resume_callback_serialized(msg)

    def _resume_callback_serialized(self, msg: String):
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
            self._reset_transport_route(preserve_approach=True)
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
        active = bool(msg.data)
        with self.command_lock:
            self.emergency_stop_active = active
            self.command_epoch += 1
            if active:
                # Cancel publications and send zeros without waiting for a
                # possibly expensive GRF cycle to leave its state critical
                # section.
                self.emergency_reset_pending = True
                self.is_running = False
                self.is_paused = False
                with self.phase_lock:
                    self.phase = TransportPhase.IDLE
                self._stop_all_robots()
            else:
                self.emergency_reset_pending = False
        if not active:
            return

        # Clean immediately when the cycle mutex is free. Otherwise the active
        # cycle performs the same cleanup in its finally block. Never block
        # the emergency-stop callback on planner work.
        cycle_lock = self._control_cycle_mutex()
        if cycle_lock.acquire(False):
            try:
                self._finalize_emergency_reset()
            finally:
                cycle_lock.release()
        rospy.logwarn("[transport] Emergency stop latched")

    def _finalize_emergency_reset(self):
        """Scrub phase-owned state after an emergency cancellation."""
        with self.command_lock:
            if (
                not self.emergency_stop_active
                or not getattr(self, 'emergency_reset_pending', False)
            ):
                return
            self._reset_transport_route(reset_control_sequence=True)
            self._clear_transport_progress()
            self.failure_reason = None
            self.emergency_reset_pending = False
            with self.phase_lock:
                self.phase = TransportPhase.IDLE

    def _shutdown(self):
        """Fail closed when this controller leaves the ROS graph."""
        with self.command_lock:
            self.is_running = False
            self.is_paused = False
            self.command_epoch += 1
            self._stop_all_robots()

    # ======================================================================
    # Helpers
    # ======================================================================

    def _ensure_transport_collision_stream(self):
        """Create causal-stream state for ROS-free ``__new__`` fixtures."""
        if not hasattr(self, 'transport_collision_lock'):
            self.transport_collision_lock = threading.Lock()
        if not hasattr(self, 'transport_collision_source_id'):
            self.transport_collision_source_id = uuid.uuid4().hex
        if not hasattr(self, 'transport_collision_source_sequence'):
            self.transport_collision_source_sequence = 0
        if not hasattr(self, 'transport_collision_task_id'):
            self.transport_collision_task_id = None
        if not hasattr(self, 'transport_collision_task_start_sequence'):
            self.transport_collision_task_start_sequence = int(
                self.transport_collision_source_sequence
            )
        if not hasattr(self, 'transport_collision_events'):
            self.transport_collision_events = deque(
                maxlen=TRANSPORT_COLLISION_EVENT_HISTORY_LIMIT
            )
        if not hasattr(self, 'transport_collision_protocol_errors'):
            self.transport_collision_protocol_errors = deque(
                maxlen=TRANSPORT_COLLISION_PROTOCOL_ERROR_LIMIT
            )
        if not hasattr(self, 'transport_safety_control_sequence'):
            self.transport_safety_control_sequence = 0
        if not hasattr(self, 'transport_active_safety_context'):
            self.transport_active_safety_context = None

    def _begin_transport_collision_stream(self, task_id):
        """Open one bounded event window without resetting its watermark."""
        self._ensure_transport_collision_stream()
        with self.transport_collision_lock:
            self.transport_collision_task_id = (
                str(task_id) if task_id is not None else None
            )
            self.transport_collision_task_start_sequence = int(
                self.transport_collision_source_sequence
            )
            self.transport_collision_events.clear()
            self.transport_collision_protocol_errors.clear()
            self.transport_active_safety_context = None

    def _collision_protocol_error_locked(self, reason):
        reason = str(reason).strip()
        if (
            reason
            and reason not in self.transport_collision_protocol_errors
        ):
            self.transport_collision_protocol_errors.append(reason)

    def _transport_collision_context(self):
        """Return the phase snapshot owned by the current control cycle."""
        context = getattr(
            self, 'transport_active_safety_context', None
        )
        return dict(context) if isinstance(context, dict) else None

    def _record_transport_collision_edge(self, robot_id, context):
        """Seal a filtered rising edge at its safety evaluation source."""
        self._ensure_transport_collision_stream()
        with self.transport_collision_lock:
            expected_task_id = self.transport_collision_task_id
            valid = isinstance(context, dict)
            if valid:
                task_id = context.get('task_id')
                phase = context.get('task_phase')
                control_sequence = context.get('control_sequence')
                sim_time = context.get('sim_time')
                wall_time = context.get('wall_time')
            else:
                task_id = None
                phase = None
                control_sequence = None
                sim_time = None
                wall_time = None

            if not isinstance(robot_id, str) or not robot_id:
                valid = False
                self._collision_protocol_error_locked(
                    'collision edge has no robot identity'
                )
            if (
                expected_task_id is None
                or task_id != expected_task_id
            ):
                valid = False
                self._collision_protocol_error_locked(
                    'collision edge is outside the active transport task'
                )
            if phase not in {
                TransportPhase.SEARCH.value,
                TransportPhase.APPROACH.value,
                TransportPhase.PUSH.value,
                TransportPhase.DONE.value,
                TransportPhase.FAILED.value,
            }:
                valid = False
                self._collision_protocol_error_locked(
                    'collision edge has no authoritative transport phase'
                )
            if (
                isinstance(control_sequence, bool)
                or not isinstance(control_sequence, int)
                or control_sequence <= 0
            ):
                valid = False
                self._collision_protocol_error_locked(
                    'collision edge has no authoritative control sequence'
                )
            for label, value in (
                ('simulation', sim_time), ('wall', wall_time)
            ):
                if (
                    isinstance(value, bool)
                    or not isinstance(value, (int, float))
                    or not math.isfinite(float(value))
                    or float(value) < 0.0
                ):
                    valid = False
                    self._collision_protocol_error_locked(
                        'collision edge has no valid {} time'.format(label)
                    )

            active_robots = set(getattr(self, 'robot_namespaces', ()))
            if active_robots and robot_id not in active_robots:
                valid = False
                self._collision_protocol_error_locked(
                    'collision edge names a robot outside the active fleet'
                )

            self.transport_collision_source_sequence += 1
            source_sequence = self.transport_collision_source_sequence
            self.transport_collision_events.append({
                'sequence': source_sequence,
                'source_sequence': source_sequence,
                'source_id': self.transport_collision_source_id,
                'robot_id': robot_id if isinstance(robot_id, str) else None,
                'task_id': task_id,
                'task_type': 'transport',
                'task_phase': phase,
                'control_sequence': control_sequence,
                'sim_time': sim_time,
                'wall_time': wall_time,
                'valid': valid,
            })

    def _transport_collision_stream_snapshot(self, task_id, terminal=False):
        """Copy a bounded, self-validating source stream into one status."""
        self._ensure_transport_collision_stream()
        task_id = str(task_id) if task_id is not None else None
        with self.transport_collision_lock:
            if self.transport_collision_task_id is None:
                self.transport_collision_task_id = task_id
                self.transport_collision_task_start_sequence = int(
                    self.transport_collision_source_sequence
                )
            elif task_id != self.transport_collision_task_id:
                self._collision_protocol_error_locked(
                    'collision stream task identity changed before publication'
                )

            events = [
                dict(event) for event in self.transport_collision_events
            ]
            watermark = int(self.transport_collision_source_sequence)
            first_sequence = (
                int(events[0]['sequence'])
                if events else watermark + 1
            )
            errors = list(self.transport_collision_protocol_errors)
            stream = {
                'version': 2,
                'source_id': self.transport_collision_source_id,
                'task_id': self.transport_collision_task_id,
                'task_start_sequence': int(
                    self.transport_collision_task_start_sequence
                ),
                'history_limit': TRANSPORT_COLLISION_EVENT_HISTORY_LIMIT,
                'first_sequence': first_sequence,
                'last_sequence': watermark,
                'watermark': watermark,
                'terminal': bool(terminal),
                'valid': not errors and all(
                    event.get('valid') is True for event in events
                ),
                'protocol_errors': errors,
                'events': events,
            }
            if terminal:
                stream['terminal_watermark'] = watermark
            return stream

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

    def _arrival_progress_tolerance(self) -> float:
        """Return a usable delivery tolerance for progress calculations."""
        try:
            tolerance = float(getattr(self, 'arrival_tolerance', 0.5))
        except (TypeError, ValueError, OverflowError):
            return 0.0
        if not math.isfinite(tolerance):
            return 0.0
        return max(0.0, tolerance)

    def _clear_transport_progress(self):
        """Forget the distance baseline when a transport task ends."""
        self.transport_initial_target_distance = None
        self.transport_reported_progress = 0.0

    def _begin_transport_progress(self):
        """Snapshot this task's payload-to-target distance."""
        self._clear_transport_progress()
        with self.model_lock:
            if self.object_position is None:
                return
            object_position = np.asarray(
                self.object_position, dtype=float
            ).copy()

        target = np.array([self.target_x, self.target_y], dtype=float)
        if (
            object_position.shape != (2,)
            or not np.all(np.isfinite(object_position))
            or not np.all(np.isfinite(target))
        ):
            return

        initial_distance = float(np.linalg.norm(object_position - target))
        self.transport_initial_target_distance = initial_distance
        if initial_distance <= self._arrival_progress_tolerance():
            self.transport_reported_progress = 1.0

    def _transport_progress(self, distance) -> float:
        """Report normalized delivery progress without moving backwards."""
        try:
            distance = float(distance)
        except (TypeError, ValueError, OverflowError):
            return float(getattr(self, 'transport_reported_progress', 0.0))
        if not math.isfinite(distance) or distance < 0.0:
            return float(getattr(self, 'transport_reported_progress', 0.0))

        previous = float(getattr(
            self, 'transport_reported_progress', 0.0
        ))
        if not math.isfinite(previous):
            previous = 0.0
        previous = max(0.0, min(1.0, previous))

        initial = getattr(self, 'transport_initial_target_distance', None)
        try:
            initial = float(initial)
        except (TypeError, ValueError, OverflowError):
            initial = None
        if initial is None or not math.isfinite(initial) or initial < 0.0:
            initial = distance
            self.transport_initial_target_distance = distance

        tolerance = self._arrival_progress_tolerance()
        if initial <= tolerance:
            current = 1.0
        else:
            current = (initial - distance) / (initial - tolerance)
            current = max(0.0, min(1.0, current))

        self.transport_reported_progress = max(previous, current)
        return self.transport_reported_progress

    def _stop_all_robots(self):
        stop = Twist()
        with self.data_lock:
            publishers = list(self.cmd_vel_pubs.items())
            avoidance_modules = list(getattr(
                self, 'avoidance_modules', {}
            ).values())
            if not hasattr(self, 'transport_last_commands'):
                self.transport_last_commands = {}
            for namespace, _publisher in publishers:
                self.transport_last_commands[namespace] = (0.0, 0.0)
        for _namespace, pub in publishers:
            pub.publish(stop)
        for avoidance in avoidance_modules:
            reset_motion = getattr(avoidance, 'reset_motion', None)
            if callable(reset_motion):
                reset_motion()

    def _sync_avoidance_snapshot(self, positions, yaws):
        """Install one coherent fleet pose in every avoidance helper."""
        points = [
            (
                namespace,
                Point(
                    x=float(position[0]),
                    y=float(position[1]),
                    z=0.0,
                ),
            )
            for namespace, position in positions.items()
            if (
                np.asarray(position).shape == (2,)
                and np.all(np.isfinite(position))
            )
        ]
        with self.data_lock:
            modules = list(getattr(
                self, 'avoidance_modules', {}
            ).items())
        point_by_name = dict(points)
        for namespace, avoidance in modules:
            point = point_by_name.get(namespace)
            yaw = yaws.get(namespace)
            if (
                point is None
                or yaw is None
                or not math.isfinite(float(yaw))
            ):
                continue
            avoidance.update_robot_positions(points)
            avoidance.set_position(
                float(point.x), float(point.y), float(yaw)
            )

    def _reset_transport_route(
        self, preserve_approach=False, reset_control_sequence=False
    ):
        """Discard the cached route when the active approach move changes."""
        compression_progress = float(getattr(
            self, 'transport_compression_progress', 0.0
        ))
        self.transport_route_namespace = None
        self.transport_route_kind = None
        self.transport_route_target = None
        self.transport_route_waypoints = []
        self.transport_route_waypoint_index = 0
        self.transport_route_last_plan_time = None
        self.transport_route_complete = False
        self.transport_route_reverse_active = False
        self.transport_route_reverse_finished = False
        self.transport_route_turn_direction = 0.0
        self.transport_assembly_route_states = {}
        self.transport_compression_progress = 0.0
        self.transport_compression_updated_at = None
        self.transport_last_rendezvous_log_count = None
        self.transport_rendezvous_recoveries = {}
        self.transport_rendezvous_recovery_cooldowns = {}
        self.transport_rendezvous_pair_decisions = {}
        if preserve_approach:
            self.transport_compression_progress = max(
                0.0, min(1.0, compression_progress)
            )
        self.transport_engaged = set()
        self.transport_physical_engaged = set()
        self.transport_aligned_engaged = set()
        self.transport_engagement_complete = False
        self.transport_engagement_ready_since = None
        self.transport_engagement_last_ready = {}
        self.transport_launch_settle_started_at = None
        self.transport_queue_docking_started = False
        self.transport_queue_settle_started_at = None
        self.transport_synchronized_push_started = False
        self.transport_push_ramp_started_at = None
        self.transport_push_reference_speed = None
        self.transport_push_ramp_batches = 0
        if reset_control_sequence:
            self.transport_control_sequence = 0
        self.transport_last_batch_publish_span = 0.0
        self.transport_last_control_sim_time = None
        self.transport_last_control_commands = {}
        self.transport_push_raw_speeds = {}
        self.transport_push_coordinated_speeds = {}
        self.transport_push_link_states = {}
        self.transport_push_hard_stop_sources = ()
        self.transport_useful_contributors = set()
        self.transport_current_useful_pushers = set()
        self.transport_all_pushers_confirmed = False
        self.transport_all_pushers_since = None
        self.transport_engagement_parent_distances = {}
        self.transport_near_target_recovery_started_at = None
        self.transport_arrival_latched = False
        self.transport_arrival_direction = None

    def _reset_search_routes(self):
        """Forget coverage paths so the active fleet gets fresh lanes."""
        self.search_routes = {}
        self.search_route_indices = {}
        self.search_route_directions = {}
        self.search_route_signature = ()
        self.search_navigation_routes = {}
        self.search_navigation_indices = {}
        self.search_navigation_destinations = {}
        self.search_navigation_targets = {}

    def _reset_search_state(self):
        """Clear discovery and coverage state at a task boundary."""
        self.transport_discovery = None
        self._reset_search_routes()

    def _control_cycle_mutex(self):
        """Return the mutex that serializes phase work and lifecycle resets."""
        lock = getattr(self, 'control_cycle_lock', None)
        if lock is None:
            # A few ROS-free unit fixtures construct the controller via
            # __new__. Production instances always create this in __init__.
            lock = threading.Lock()
            self.control_cycle_lock = lock
        return lock

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
                if not hasattr(self, 'transport_last_commands'):
                    self.transport_last_commands = {}
                self.transport_last_commands[namespace] = (
                    float(command.linear.x), float(command.angular.z)
                )
                pub.publish(command)
            return True

    def _set_phase(
        self, phase: TransportPhase, expected_epoch: int
    ) -> bool:
        with self.command_lock:
            if not self._command_allowed(expected_epoch):
                return False
            if phase == TransportPhase.PUSH:
                # A robot may spend longer than the progress timeout getting
                # back into contact. Start a fresh watchdog window once it is
                # ready to push again.
                self.transport_best_distance = None
                self.transport_last_progress_time = None
                self.transport_near_target_recovery_started_at = None
                self.transport_arrival_latched = False
                self.transport_arrival_direction = None
                self.transport_engaged = set()
                self.transport_physical_engaged = set()
                self.transport_aligned_engaged = set()
                self.transport_engagement_complete = False
                self.transport_engagement_ready_since = None
                self.transport_engagement_last_ready = {}
                self.transport_launch_settle_started_at = None
                self.transport_queue_docking_started = False
                self.transport_queue_settle_started_at = None
                self.transport_synchronized_push_started = False
                self.transport_push_ramp_started_at = None
                self.transport_push_reference_speed = None
                self.transport_push_ramp_batches = 0
                self.transport_last_batch_publish_span = 0.0
                self.transport_last_control_sim_time = None
                self.transport_last_control_commands = {}
                self.transport_push_raw_speeds = {}
                self.transport_push_coordinated_speeds = {}
                self.transport_push_link_states = {}
                self.transport_push_hard_stop_sources = ()
                self.transport_useful_contributors = set()
                self.transport_current_useful_pushers = set()
                self.transport_all_pushers_confirmed = False
                self.transport_all_pushers_since = None
                self.transport_engagement_parent_distances = {}
            with self.phase_lock:
                self.phase = phase
            return True

    def _fail_transport(self, reason: str, expected_epoch: int) -> bool:
        """Stop an active task and preserve a correlated failure status."""
        with self.command_lock:
            if not self._command_allowed(expected_epoch):
                return False
            self.object_error = reason
            self.failure_reason = reason
            self.is_running = False
            self.is_paused = False
            self.command_epoch += 1
            with self.phase_lock:
                self.phase = TransportPhase.FAILED
            self._stop_all_robots()
        rospy.logerr("[transport] %s  >>> phase FAILED", reason)
        return True

    def _complete_transport(self, distance: float, expected_epoch: int) -> bool:
        """Finish from either positioning or pushing once the payload arrives."""
        with self.command_lock:
            if not self._command_allowed(expected_epoch):
                return False
            self.is_running = False
            self.command_epoch += 1
            with self.phase_lock:
                self.phase = TransportPhase.DONE
            self._stop_all_robots()
        rospy.loginfo(
            "[transport] object delivered (dist=%.2f)  >>> phase DONE",
            distance,
        )
        return True

    def _reposition_if_stalled(
        self, distance: float, expected_epoch: int
    ) -> bool:
        """Rotate blocked robots into fresh roles when progress stops."""
        get_time = getattr(rospy, 'get_time', lambda: 0.0)
        now = float(get_time())
        if not math.isfinite(now):
            return False
        if (
            self.transport_best_distance is None
            or distance
            <= self.transport_best_distance - self.transport_progress_delta
        ):
            self.transport_best_distance = distance
            self.transport_last_progress_time = now
            return False
        if self.transport_last_progress_time is None:
            self.transport_last_progress_time = now
            return False
        if (
            now - self.transport_last_progress_time
            < self.transport_progress_timeout
        ):
            return False

        self.transport_roles = {}
        self.transport_staged = set()
        self.transport_pre_staged = set()
        self.transport_chain_released = set()
        self.transport_all_ready_since = None
        self._reset_transport_route()
        self.transport_best_distance = distance
        self.transport_last_progress_time = now
        if not self._set_phase(TransportPhase.APPROACH, expected_epoch):
            return False
        self._stop_all_robots()
        rospy.logwarn(
            "[transport] payload progress stalled at %.2fm; "
            "reassigning contact robots",
            distance,
        )
        return True

    @staticmethod
    def _short_robot_list(namespaces):
        ordered = sort_robot_ids(namespaces)
        if len(ordered) <= 6:
            return ', '.join(ordered)
        return '{}, and {} more'.format(
            ', '.join(ordered[:6]), len(ordered) - 6
        )

    def _transport_model_state_error(self, now=None):
        """Explain why Gazebo geometry is not fresh enough to command."""
        if now is None:
            now = time.monotonic()
        try:
            now = float(now)
        except (TypeError, ValueError, OverflowError):
            return "Transport requires a valid wall clock"
        if not math.isfinite(now):
            return "Transport requires a valid wall clock"

        with self.model_lock:
            received_at = getattr(self, 'model_states_received_at', None)
        try:
            received_at = float(received_at)
        except (TypeError, ValueError, OverflowError):
            received_at = None
        if received_at is None or not math.isfinite(received_at):
            return "Transport requires fresh Gazebo model states"

        timeout = getattr(
            self,
            'model_states_timeout_wall_s',
            DEFAULT_MODEL_STATES_TIMEOUT_WALL_S,
        )
        age = now - received_at
        if age < 0.0 or age > timeout:
            return (
                "Gazebo model states became stale "
                "({:.3f}s old; limit {:.3f}s)".format(age, timeout)
            )
        return None

    def _transport_odometry_error(
        self, namespaces, positions, yaws, received_at, now=None,
    ):
        """Explain why this roster is not safe to command yet."""
        if now is None:
            get_time = getattr(rospy, 'get_time', lambda: 0.0)
            now = float(get_time())
        if not math.isfinite(now):
            return "Transport requires a valid simulation clock"

        timeout = getattr(self, 'transport_odom_timeout', 2.0)
        missing = []
        invalid = []
        stale = []
        for namespace in namespaces:
            position = positions.get(namespace)
            yaw = yaws.get(namespace)
            stamp = received_at.get(namespace)
            if stamp is None:
                missing.append(namespace)
                continue
            try:
                stamp = float(stamp)
            except (TypeError, ValueError):
                invalid.append(namespace)
                continue
            if (
                position is None
                or np.asarray(position).shape != (2,)
                or not np.all(np.isfinite(position))
                or yaw is None
                or not math.isfinite(float(yaw))
                or not math.isfinite(stamp)
            ):
                invalid.append(namespace)
                continue
            age = now - stamp
            if age < 0.0 or age > timeout:
                stale.append(namespace)

        problems = []
        if missing:
            problems.append(
                'missing: {}'.format(self._short_robot_list(missing))
            )
        if invalid:
            problems.append(
                'invalid: {}'.format(self._short_robot_list(invalid))
            )
        if stale:
            problems.append(
                'stale: {}'.format(self._short_robot_list(stale))
            )
        if not problems:
            return None
        return (
            "Transport requires fresh odometry ({})."
            .format('; '.join(problems))
        )

    def _transport_static_zones(self):
        """Return configured obstacles, excluding the payload we must touch."""
        object_name = getattr(self, 'object_name', 'transport_object')
        zones = []
        for zone in getattr(self, 'spawn_exclusion_zones', ()):
            if not isinstance(zone, dict):
                continue
            if (
                zone.get('model') == object_name
                or zone.get('name') == object_name
            ):
                continue
            zones.append(zone)
        return zones

    def _transport_points_are_safe(self, points, obstacle_clearance):
        """Check finite points against the shared arena and obstacle model."""
        arena_size = float(getattr(self, 'arena_size', 10.0))
        arena_margin = float(getattr(self, 'arena_margin', 0.35))
        obstacle_clearance = float(obstacle_clearance)
        if (
            not math.isfinite(arena_size)
            or not math.isfinite(arena_margin)
            or not math.isfinite(obstacle_clearance)
            or arena_size <= 0.0
            or arena_margin < 0.0
            or obstacle_clearance < 0.0
        ):
            return False

        model_lock = getattr(self, 'model_lock', None)
        if model_lock is None:
            model_poses = dict(getattr(self, 'model_poses', {}))
        else:
            with model_lock:
                model_poses = dict(getattr(self, 'model_poses', {}))
        try:
            return formation_targets_are_safe(
                [
                    (float(point[0]), float(point[1]))
                    for point in points
                ],
                arena_size,
                arena_margin,
                obstacle_clearance,
                self._transport_static_zones(),
                getattr(self, 'arena_profile', 'swarm_arena'),
                model_poses,
            )
        except (IndexError, TypeError, ValueError, OverflowError):
            return False

    def _transport_target_error(self):
        target = (self.target_x, self.target_y)
        payload_clearance = (
            max(self.object_half_width, self.object_half_height)
            + getattr(self, 'transport_payload_target_margin', 0.05)
        )
        if self._transport_points_are_safe(
            [target], payload_clearance
        ):
            return None
        usable_size = max(
            0.0,
            float(getattr(self, 'arena_size', 10.0))
            - 2.0 * float(getattr(self, 'arena_margin', 0.35)),
        )
        return (
            "Transport target is infeasible: it must be finite, stay inside "
            "the {:.2f}m usable arena, and clear every configured static "
            "obstacle.".format(usable_size)
        )

    def _transport_layout_error(
        self, targets, object_pos, object_yaw, fleet_size=None,
    ):
        """Reject layouts whose rendezvous or completed chains cannot fit."""
        layout_points = []
        rendezvous_points = []
        for target in targets.values():
            for key in (
                'staging_position', 'assembly_position', 'final_position'
            ):
                point = target.get(key)
                if point is not None:
                    layout_points.append(np.asarray(point, dtype=float))

            staging_position = target.get('staging_position')
            if staging_position is not None:
                rendezvous_points.append(
                    np.asarray(staging_position, dtype=float)
                )

            # The accordion merge follows this straight morph. Checking a few
            # interior points keeps the whole concurrent corridor out of
            # static obstacles, not just its two endpoints.
            assembly_position = target.get('assembly_position')
            if staging_position is not None and assembly_position is not None:
                for progress in (
                    0.125, 0.25, 0.375, 0.50,
                    0.625, 0.75, 0.875,
                ):
                    layout_points.append(self._compression_target(
                        target, progress
                    ))

            # Payload routing deliberately hands off outside its normal LiDAR
            # awareness zone. That handoff is part of the staging plan too.
            if (
                target.get('role') == 'payload_push'
                and target.get('staging_position') is not None
            ):
                layout_points.append(
                    self._payload_staging_route_destination(target)
                )

        # Validate the completed chain at the destination as well as at its
        # current assembly pose. The rear direction remains the delivery
        # direction; using the zero-distance fallback at the goal would rotate
        # the capacity check by ninety degrees.
        payload_count = sum(
            target.get('role') == 'payload_push'
            for target in targets.values()
        )
        if payload_count:
            _, rear, lateral = self._transport_frame(object_pos)
            target_center = np.array(
                [self.target_x, self.target_y], dtype=float
            )
            contact_slots = self._payload_contact_slots(
                payload_count,
                target_center,
                object_yaw,
                rear,
                lateral,
            )
            for target in targets.values():
                try:
                    chain_index = int(target['chain_index'])
                    depth = int(target['chain_depth'])
                    contact_position = contact_slots[chain_index][0]
                except (IndexError, KeyError, TypeError, ValueError):
                    return "Transport layout contains an invalid push-chain role."
                layout_points.append(
                    contact_position
                    + rear * self.transport_companion_contact_distance * depth
                )

        minimum_rendezvous_clearance = max(
            2.0 * getattr(self, 'robot_radius', 0.11) + 0.03,
            getattr(
                self, 'transport_parallel_row_minimum_clearance', 0.25
            ),
        )
        for index, point in enumerate(rendezvous_points):
            if any(
                float(np.linalg.norm(point - other))
                < minimum_rendezvous_clearance - 1e-6
                for other in rendezvous_points[index + 1:]
            ):
                return (
                    "Transport rendezvous gates overlap for {} robots; "
                    "increase their spacing or reduce the fleet size."
                    .format(len(targets))
                )

        clearance = getattr(
            self, 'transport_route_obstacle_clearance', 0.30
        )
        if self._transport_points_are_safe(layout_points, clearance):
            return None
        if fleet_size is None:
            fleet_size = len(targets)
        usable_size = max(
            0.0,
            float(getattr(self, 'arena_size', 10.0))
            - 2.0 * float(getattr(self, 'arena_margin', 0.35)),
        )
        return (
            "Transport layout is infeasible for {} robots: a staging, "
            "rendezvous, route-handoff, or final chain point leaves the {:.2f}m "
            "usable arena or overlaps a configured static obstacle clearance."
            .format(fleet_size, usable_size)
        )

    def _transport_frame(self, object_pos):
        """Return goal, rear, and lateral unit vectors for the payload."""
        goal = None
        if getattr(self, 'transport_arrival_latched', False):
            arrival_direction = getattr(
                self, 'transport_arrival_direction', None
            )
            if arrival_direction is not None:
                arrival_direction = np.asarray(
                    arrival_direction, dtype=float
                )
                direction_norm = float(np.linalg.norm(arrival_direction))
                if (
                    arrival_direction.shape == (2,)
                    and np.all(np.isfinite(arrival_direction))
                    and direction_norm > 1e-9
                ):
                    goal = arrival_direction / direction_norm

        if goal is None:
            difference = np.array([
                self.target_x - float(object_pos[0]),
                self.target_y - float(object_pos[1]),
            ])
            distance = float(np.linalg.norm(difference))
            if distance <= 1e-9:
                goal = np.array([1.0, 0.0])
            else:
                goal = difference / distance
        rear = -goal
        lateral = np.array([-rear[1], rear[0]])
        return goal, rear, lateral

    def _loaded_companion_distance(self, measured_distance=None):
        """Return a settled link distance with room inside the contact gate."""
        contact_distance = float(getattr(
            self, 'transport_companion_contact_distance', 0.145
        ))
        contact_slop = max(
            0.0,
            float(getattr(self, 'transport_contact_slop', 0.005)),
        )
        loaded_distance = max(0.001, contact_distance - contact_slop)
        if measured_distance is None:
            return loaded_distance

        try:
            measured_distance = float(measured_distance)
        except (TypeError, ValueError):
            return loaded_distance
        if not math.isfinite(measured_distance):
            return loaded_distance

        # Preserve a link that settled a little deeper in Gazebo, but never
        # aim at the outer contact boundary. A target on that boundary turns
        # a sub-millimetre odometry wobble into a broken five-robot chain.
        return min(
            loaded_distance,
            max(0.001, loaded_distance - contact_slop, measured_distance),
        )

    def _payload_contact_slots(
        self, active_count, object_pos, object_yaw, rear, lateral
    ):
        """Return stable, symmetric contact slots on the rear support face."""
        if active_count <= 1:
            lateral_offsets = (0.0,)
        else:
            lane_offset = self.transport_payload_lane_offset
            lateral_offsets = (-lane_offset, lane_offset)

        cosine = math.cos(object_yaw)
        sine = math.sin(object_yaw)
        rear_local = np.array([
            rear[0] * cosine + rear[1] * sine,
            -rear[0] * sine + rear[1] * cosine,
        ])

        slots = []
        for lateral_offset in lateral_offsets:
            offset = lateral * lateral_offset
            offset_local = np.array([
                offset[0] * cosine + offset[1] * sine,
                -offset[0] * sine + offset[1] * cosine,
            ])
            exit_distance = float('inf')
            for origin, direction, half_size in (
                (offset_local[0], rear_local[0], self.object_half_width),
                (offset_local[1], rear_local[1], self.object_half_height),
            ):
                if abs(direction) < 1e-9:
                    continue
                near = (-half_size - origin) / direction
                far = (half_size - origin) / direction
                if near > far:
                    near, far = far, near
                exit_distance = min(exit_distance, far)

            if not math.isfinite(exit_distance) or exit_distance <= 0.0:
                exit_distance = min(
                    self.object_half_width, self.object_half_height
                )
            rear_distance = (
                exit_distance
                + self.robot_forward_contact_extent
                + self.transport_contact_slop
            )
            position = (
                object_pos + rear * rear_distance + offset
            )
            radial_offset = position - object_pos
            radial = radial_offset / max(
                float(np.linalg.norm(radial_offset)), 1e-9
            )
            slots.append((position, radial))
        return slots

    def _transport_targets(
        self, namespaces, positions, object_pos, object_yaw,
        velocities=None,
    ):
        """Put every robot in a useful position in one of the push chains."""
        available = [ns for ns in sort_robot_ids(namespaces) if ns in positions]
        if not available:
            self.transport_roles = {}
            self.transport_staged = set()
            self.transport_pre_staged = set()
            self.transport_chain_released = set()
            self._reset_transport_route()
            return {}

        velocities = velocities or {}
        object_velocity = np.asarray(
            getattr(self, 'object_velocity', np.zeros(2)), dtype=float
        )
        if object_velocity.shape != (2,) or not np.all(
            np.isfinite(object_velocity)
        ):
            object_velocity = np.zeros(2)

        goal, rear, lateral = self._transport_frame(object_pos)
        payload_pusher_count = min(
            len(available), self.max_pushing_robots
        )
        contact_slots = self._payload_contact_slots(
            payload_pusher_count, object_pos, object_yaw, rear, lateral
        )
        contact_positions = [slot[0] for slot in contact_slots]
        radials = [slot[1] for slot in contact_slots]

        # Fill both chains one row at a time. This keeps their lengths within
        # one robot of each other for any fleet size. The expanded positions
        # are rendezvous gates: every robot can travel there at the same time
        # without entering another Burger's final contact envelope.
        companion_slots = []
        companion_count = len(available) - payload_pusher_count
        large_fleet_staging = companion_count > 2
        configured_spacing = (
            getattr(
                self, 'transport_large_fleet_staging_spacing', 0.38
            )
            if large_fleet_staging
            else self.transport_chain_staging_spacing
        )
        route_clearance = getattr(
            self, 'transport_route_robot_clearance', 0.32
        )
        staging_spacing = max(
            configured_spacing, route_clearance + 0.04
        )
        staging_lateral_offset = (
            max(
                getattr(
                    self,
                    'transport_large_fleet_staging_lateral_offset',
                    0.34,
                ),
                route_clearance + 0.02,
            )
            if large_fleet_staging
            else 0.0
        )
        for index in range(companion_count):
            chain_index = index % payload_pusher_count
            depth = index // payload_pusher_count + 1
            contact_position = contact_positions[chain_index]
            lateral_projection = float(np.dot(
                contact_position - object_pos, lateral
            ))
            staging_side = 1.0 if lateral_projection >= 0.0 else -1.0
            companion_slots.append({
                'chain_index': chain_index,
                'depth': depth,
                'position': (
                    contact_position
                    + rear
                    * self.transport_companion_contact_distance
                    * depth
                ),
                'staging_position': (
                    contact_position
                    + rear * (
                        self.transport_staging_clearance
                        + depth * staging_spacing
                    )
                    + lateral * staging_side * staging_lateral_offset
                ),
            })

        expected_slots = {
            ('payload_push', chain_index, 0)
            for chain_index in range(payload_pusher_count)
        }
        expected_slots.update(
            (
                'companion_push',
                slot['chain_index'],
                slot['depth'],
            )
            for slot in companion_slots
        )

        # Work from one role snapshot. Fleet callbacks and stop commands run
        # on other rospy threads and may clear self.transport_roles while a
        # control cycle is finishing; local roles keep that cycle harmless.
        current_roles = {
            namespace: dict(role) if isinstance(role, dict) else role
            for namespace, role in self.transport_roles.items()
        }
        current_slots = set()
        roles_have_expected_shape = True
        for role in current_roles.values():
            if not isinstance(role, dict):
                roles_have_expected_shape = False
                break
            try:
                current_slots.add((
                    role['role'],
                    int(role['chain_index']),
                    int(role['depth']),
                ))
            except (KeyError, TypeError, ValueError):
                roles_have_expected_shape = False
                break

        roles_are_current = (
            set(current_roles) == set(available)
            and roles_have_expected_shape
            and current_slots == expected_slots
        )
        if not roles_are_current:
            roles = {}
            unused_robots = set(available)
            unused_slots = set(range(payload_pusher_count))
            namespace_order = {
                namespace: index
                for index, namespace in enumerate(available)
            }
            candidates = sorted(
                (
                    float(np.linalg.norm(
                        positions[namespace] - contact_positions[index]
                    )),
                    namespace_order[namespace],
                    index,
                    namespace,
                )
                for namespace in available
                for index in range(payload_pusher_count)
            )
            for _, _, slot_index, namespace in candidates:
                if namespace not in unused_robots or slot_index not in unused_slots:
                    continue
                roles[namespace] = {
                    'role': 'payload_push',
                    'chain_index': slot_index,
                    'depth': 0,
                    'staging_position': (
                        contact_positions[slot_index]
                        + rear * self.transport_staging_clearance
                    ),
                }
                unused_robots.remove(namespace)
                unused_slots.remove(slot_index)
                if not unused_slots:
                    break

            remaining_robots = sort_robot_ids(unused_robots)
            assembly_positions = [
                contact_positions[slot['chain_index']]
                + rear * (
                    self.transport_staging_clearance
                    + slot['depth']
                    * self._loaded_companion_distance()
                )
                for slot in companion_slots
            ]
            rendezvous_positions = [
                np.asarray(slot['staging_position'], dtype=float)
                for slot in companion_slots
            ]
            slot_assignment = hungarian_assignment([
                [
                    float(np.dot(
                        positions[namespace] - rendezvous_position,
                        positions[namespace] - rendezvous_position,
                    ))
                    for rendezvous_position in rendezvous_positions
                ]
                for namespace in remaining_robots
            ]) if remaining_robots else []

            # A global distance match keeps the concurrent paths short and
            # avoids sending a nearby robot across the other rendezvous lane.
            for namespace, slot_index in zip(
                remaining_robots, slot_assignment
            ):
                slot = companion_slots[slot_index]
                roles[namespace] = {
                    'role': 'companion_push',
                    'chain_index': slot['chain_index'],
                    'depth': slot['depth'],
                    'staging_position': slot['staging_position'].copy(),
                    'assembly_position': (
                        assembly_positions[slot_index].copy()
                    ),
                }
                unused_robots.remove(namespace)
            self.transport_roles = roles
            self.transport_staged = set()
            self.transport_pre_staged = set()
            self.transport_chain_released = set()
            self._reset_transport_route()
            rospy.loginfo(
                "[transport] assigned %d payload pushers and "
                "%d companion pushers",
                payload_pusher_count, companion_count,
            )
        else:
            roles = current_roles

        owner_by_slot = {
            (role['chain_index'], role['depth']): namespace
            for namespace, role in roles.items()
        }
        maximum_chain_depth = max(
            (int(role['depth']) for role in roles.values()),
            default=0,
        )
        targets = {}
        for namespace in available:
            role = roles[namespace]
            chain_index = role['chain_index']
            depth = role['depth']
            if role['role'] == 'payload_push':
                staging_position = np.asarray(
                    role.get(
                        'staging_position',
                        contact_positions[chain_index]
                        + rear * self.transport_staging_clearance,
                    ),
                    dtype=float,
                )
                staging_offset = staging_position - object_pos
                staging_radial = staging_offset / max(
                    float(np.linalg.norm(staging_offset)), 1e-9
                )
                targets[namespace] = {
                    'role': role['role'],
                    'fleet_size': len(available),
                    'companion_count': companion_count,
                    'chain_index': chain_index,
                    'chain_depth': depth,
                    'max_chain_depth': maximum_chain_depth,
                    'position': contact_positions[chain_index],
                    'final_position': contact_positions[chain_index],
                    'radial': radials[chain_index],
                    'staging_position': staging_position,
                    'staging_radial': staging_radial,
                    'push_direction': goal,
                    'parent_namespace': None,
                    'parent_position': object_pos,
                    'parent_velocity': object_velocity.copy(),
                    'robot_velocity': np.asarray(
                        velocities.get(namespace, np.zeros(2)), dtype=float
                    ).copy(),
                }
                continue

            parent_namespace = owner_by_slot[(chain_index, depth - 1)]
            parent_position = positions[parent_namespace].copy()
            # Keep any small amount of measured settling, but express the
            # link in the live transport frame and leave it inside the contact
            # gate. A world-fixed offset shears long columns when the payload
            # turns; a rear-facing offset lets every row rotate together.
            captured_distance = getattr(
                self, 'transport_engagement_parent_distances', {}
            ).get(namespace)
            if not getattr(
                self, 'transport_synchronized_push_started', False
            ):
                # Assembly is a zero-load operation. Keep partial columns at
                # the neutral contact surface so a closing tail cannot drive
                # the crate before the complete-fleet launch barrier.
                captured_distance = self.transport_companion_contact_distance
            else:
                captured_distance = self._loaded_companion_distance(
                    captured_distance
                )
            desired = parent_position + rear * captured_distance
            final_position = (
                contact_positions[chain_index]
                + rear * self.transport_companion_contact_distance * depth
            )
            assembly_position = np.asarray(
                role.get(
                    'assembly_position',
                    parent_position
                    + rear * self._loaded_companion_distance(),
                ),
                dtype=float,
            )
            staging_position = np.asarray(
                role.get(
                    'staging_position',
                    contact_positions[chain_index]
                    + rear * (
                        self.transport_staging_clearance
                        + depth * getattr(
                            self, 'transport_chain_staging_spacing', 0.31
                        )
                    ),
                ),
                dtype=float,
            )
            desired_offset = desired - object_pos
            desired_radial = desired_offset / max(
                float(np.linalg.norm(desired_offset)), 1e-9
            )
            targets[namespace] = {
                'role': role['role'],
                'fleet_size': len(available),
                'companion_count': companion_count,
                'chain_index': chain_index,
                'chain_depth': depth,
                'max_chain_depth': maximum_chain_depth,
                'position': desired,
                'final_position': final_position,
                'assembly_position': assembly_position,
                'staging_position': staging_position,
                'radial': desired_radial,
                'push_direction': goal,
                'parent_namespace': parent_namespace,
                'parent_position': parent_position,
                'parent_velocity': np.asarray(
                    velocities.get(parent_namespace, np.zeros(2)),
                    dtype=float,
                ).copy(),
                'robot_velocity': np.asarray(
                    velocities.get(namespace, np.zeros(2)), dtype=float
                ).copy(),
            }
        return targets

    @staticmethod
    def _transport_neighbours(targets):
        """Split physical contacts, shielded corridors, and parallel rows."""
        contact_neighbours = {
            namespace: set() for namespace in targets
        }
        shielded_neighbours = {
            namespace: set() for namespace in targets
        }
        row_neighbours = {
            namespace: set() for namespace in targets
        }
        rows = {}
        chains = {}

        for namespace, target in targets.items():
            parent = target.get('parent_namespace')
            if parent in contact_neighbours:
                contact_neighbours[namespace].add(parent)
                contact_neighbours[parent].add(namespace)

            depth = target.get('chain_depth')
            if depth is not None:
                rows.setdefault(depth, set()).add(namespace)
            chain_index = target.get('chain_index')
            if chain_index is not None:
                chains.setdefault(chain_index, set()).add(namespace)

        # A rear link naturally sits within the predictive braking distance
        # of its grandparent. Its parent shields that corridor, but only the
        # adjacent pair may make physical contact. Nonadjacent links retain
        # the hard centre-distance stop if a chain folds or compresses.
        for chain in chains.values():
            for namespace in chain:
                shielded_neighbours[namespace].update(
                    chain
                    - {namespace}
                    - contact_neighbours[namespace]
                )

        for row in rows.values():
            for namespace in row:
                row_neighbours[namespace].update(row - {namespace})

        return contact_neighbours, shielded_neighbours, row_neighbours

    @staticmethod
    def _ready_chain_prefix(targets, locally_ready):
        """Count a link only when it and every parent are ready now."""
        ready = set()
        ordered = sorted(
            targets,
            key=lambda namespace: targets[namespace].get(
                'chain_depth', 0
            ),
        )
        for namespace in ordered:
            if namespace not in locally_ready:
                continue
            parent = targets[namespace].get('parent_namespace')
            if parent is None or parent in ready:
                ready.add(namespace)
        return ready

    def _robot_lidar_masks(self, namespaces, positions, yaws):
        """Predict the small LDS body seen by another Burger's scanner."""
        center_offset = getattr(
            self, 'robot_lidar_center_offset', -0.017
        )
        radius = getattr(self, 'robot_lidar_collision_radius', 0.055)
        maximum_distance = getattr(self, 'sensing_range', 2.0)
        tolerance = getattr(self, 'robot_lidar_mask_tolerance', 0.03)
        closer_tolerance = getattr(
            self, 'robot_lidar_mask_closer_tolerance', 0.015
        )
        masks = []
        for namespace in namespaces:
            position = positions.get(namespace)
            yaw = yaws.get(namespace)
            if position is None or yaw is None or not math.isfinite(yaw):
                continue
            center_x = (
                float(position[0])
                + center_offset * math.cos(yaw)
            )
            center_y = (
                float(position[1])
                + center_offset * math.sin(yaw)
            )
            masks.append(LidarRangeMask(
                center_x=center_x,
                center_y=center_y,
                radius=radius,
                maximum_center_distance=maximum_distance,
                tolerance=tolerance,
                closer_tolerance=closer_tolerance,
            ))
        return masks

    def _chain_contact_body_lidar_masks(
        self, namespaces, positions, yaws
    ):
        """Mask the Burger chassis of deliberate adjacent chain links.

        The LDS cap is narrower than the 14 cm square base.  At normal chain
        spacing an oblique rear beam can therefore see a base corner without
        crossing the round cap mask.  Use the full footprint only for the
        adjacent links already admitted by the chain-contact checks; wider
        peer categories keep the smaller cap mask above.
        """
        front_extent = max(0.0, getattr(
            self, 'robot_forward_contact_extent', 0.038
        ))
        rear_extent = max(0.0, getattr(
            self, 'robot_rear_contact_extent', 0.102
        ))
        half_length = 0.5 * (front_extent + rear_extent)
        center_offset = 0.5 * (front_extent - rear_extent)
        half_width = max(0.001, getattr(
            self, 'robot_body_half_width', 0.070
        ))
        maximum_distance = max(
            2.0 * getattr(self, 'robot_radius', 0.11),
            getattr(
                self, 'transport_companion_contact_distance', 0.145
            ) + 0.075,
            getattr(
                self, 'transport_route_parent_clearance', 0.20
            ) + 0.05,
        )
        tolerance = getattr(self, 'robot_lidar_mask_tolerance', 0.05)
        closer_tolerance = getattr(
            self, 'robot_lidar_mask_closer_tolerance', 0.04
        )

        masks = []
        for namespace in namespaces:
            position = positions.get(namespace)
            yaw = yaws.get(namespace)
            if position is None or yaw is None or not math.isfinite(yaw):
                continue
            center_x = (
                float(position[0])
                + center_offset * math.cos(yaw)
            )
            center_y = (
                float(position[1])
                + center_offset * math.sin(yaw)
            )
            masks.append(LidarRangeMask(
                center_x=center_x,
                center_y=center_y,
                radius=0.0,
                maximum_center_distance=maximum_distance,
                tolerance=tolerance,
                closer_tolerance=closer_tolerance,
                half_width=half_length,
                half_height=half_width,
                yaw=float(yaw),
                occludes_behind=True,
                accept_sensor_floor=True,
            ))
        return masks

    def _transport_robot_lidar_masks(
        self, adjacent_contacts, other_peers, positions, yaws
    ):
        """Build full masks for adjacent links and cap masks for other peers."""
        adjacent = set(adjacent_contacts)
        masks = self._chain_contact_body_lidar_masks(
            adjacent, positions, yaws
        )
        masks.extend(self._robot_lidar_masks(
            set(other_peers) - adjacent, positions, yaws
        ))
        return masks

    def _chain_link_is_ordered(
        self, parent_namespace, child_namespace, positions, targets
    ):
        """Check that one child is approaching the rear of its parent."""
        parent_position = positions.get(parent_namespace)
        child_position = positions.get(child_namespace)
        child_target = targets.get(child_namespace)
        if (
            parent_position is None
            or child_position is None
            or child_target is None
        ):
            return False

        push_direction = np.asarray(
            child_target.get('push_direction'), dtype=float
        )
        if (
            push_direction.shape != (2,)
            or not np.all(np.isfinite(push_direction))
        ):
            return False
        push_norm = float(np.linalg.norm(push_direction))
        if push_norm <= 1e-9:
            return False
        push_direction /= push_norm

        direction_to_parent = parent_position - child_position
        distance = float(np.linalg.norm(direction_to_parent))
        corridor_distance = max(
            getattr(self, 'transport_orbit_radius', 0.60),
            getattr(
                self, 'transport_companion_contact_distance', 0.145
            ) + 0.15,
        )
        if distance <= 1e-9 or distance > corridor_distance:
            return False

        unit_to_parent = direction_to_parent / distance
        alignment = float(np.dot(unit_to_parent, push_direction))
        alignment_tolerance = getattr(
            self, 'transport_chain_alignment_tolerance', 0.55
        )
        if alignment < math.cos(alignment_tolerance):
            return False

        lateral_error = abs(float(
            push_direction[0] * direction_to_parent[1]
            - push_direction[1] * direction_to_parent[0]
        ))
        return lateral_error <= getattr(
            self, 'transport_chain_corridor_width', 0.12
        )

    def _chain_path_is_ordered(
        self, namespace, other_namespace, positions, targets
    ):
        """Require every link between two chain robots to be in order."""
        target = targets.get(namespace)
        other_target = targets.get(other_namespace)
        if target is None or other_target is None:
            return False
        if target.get('chain_index') != other_target.get('chain_index'):
            return False

        try:
            first_depth = int(target['chain_depth'])
            second_depth = int(other_target['chain_depth'])
        except (KeyError, TypeError, ValueError):
            return False
        if first_depth == second_depth:
            return False

        chain_index = target.get('chain_index')
        owners = {
            int(candidate['chain_depth']): candidate_namespace
            for candidate_namespace, candidate in targets.items()
            if (
                candidate.get('chain_index') == chain_index
                and candidate.get('chain_depth') is not None
            )
        }
        shallow_depth, deep_depth = sorted((first_depth, second_depth))
        for child_depth in range(shallow_depth + 1, deep_depth + 1):
            parent_namespace = owners.get(child_depth - 1)
            child_namespace = owners.get(child_depth)
            if (
                parent_namespace is None
                or child_namespace is None
                or not self._chain_link_is_ordered(
                    parent_namespace,
                    child_namespace,
                    positions,
                    targets,
                )
            ):
                return False
        return True

    def _nearby_chain_contacts(
        self, namespace, neighbours, positions, targets
    ):
        """Keep only correctly ordered links inside the push corridor."""
        position = positions.get(namespace)
        if position is None:
            return ()

        corridor_distance = max(
            getattr(self, 'transport_orbit_radius', 0.60),
            getattr(
                self, 'transport_companion_contact_distance', 0.145
            ) + 0.15,
        )
        allowed = []
        for other_namespace in neighbours.get(namespace, ()):
            other_position = positions.get(other_namespace)
            if other_position is None:
                continue
            offset = other_position - position
            distance = float(np.linalg.norm(offset))
            if distance <= 1e-9 or distance > corridor_distance:
                continue
            if not self._chain_path_is_ordered(
                namespace, other_namespace, positions, targets
            ):
                continue
            allowed.append(other_namespace)
        return tuple(allowed)

    def _parallel_lane_contacts(self, namespace, positions, targets):
        """Return robots that are safely separated in the other push lane."""
        target = targets.get(namespace)
        position = positions.get(namespace)
        if target is None or position is None:
            return ()

        push_direction = np.asarray(
            target.get('push_direction'), dtype=float
        )
        if (
            push_direction.shape != (2,)
            or not np.all(np.isfinite(push_direction))
        ):
            return ()
        norm = float(np.linalg.norm(push_direction))
        if norm <= 1e-9:
            return ()
        push_direction /= norm
        lateral = np.array([-push_direction[1], push_direction[0]])
        minimum_clearance = max(
            0.29,
            2.0 * getattr(self, 'robot_radius', 0.11) + 0.05,
        )

        neighbours = []
        for other_namespace, other_target in targets.items():
            if (
                other_namespace == namespace
                or other_target.get('chain_index')
                == target.get('chain_index')
            ):
                continue
            other_position = positions.get(other_namespace)
            if other_position is None:
                continue
            offset = np.asarray(other_position) - position
            lateral_clearance = abs(float(np.dot(offset, lateral)))
            if lateral_clearance >= minimum_clearance:
                neighbours.append(other_namespace)
        return tuple(neighbours)

    def _parallel_push_rows_ready(self, positions, targets):
        """Require a physical gap between same-depth opposite-lane robots."""
        minimum_clearance = getattr(
            self,
            'transport_parallel_row_minimum_clearance',
            max(0.25, 2.0 * getattr(self, 'robot_radius', 0.11) + 0.03),
        )
        ordered_names = sorted(targets)
        for index, namespace in enumerate(ordered_names):
            target = targets[namespace]
            position = positions.get(namespace)
            if position is None:
                return False
            push_direction = np.asarray(
                target.get('push_direction'), dtype=float
            )
            if (
                push_direction.shape != (2,)
                or not np.all(np.isfinite(push_direction))
            ):
                return False
            direction_norm = float(np.linalg.norm(push_direction))
            if direction_norm <= 1e-9:
                return False
            push_direction /= direction_norm
            lateral = np.array([-push_direction[1], push_direction[0]])

            for other_namespace in ordered_names[index + 1:]:
                other_target = targets[other_namespace]
                if (
                    other_target.get('chain_index')
                    == target.get('chain_index')
                    or other_target.get('chain_depth')
                    != target.get('chain_depth')
                ):
                    continue
                other_position = positions.get(other_namespace)
                if other_position is None:
                    return False
                lateral_clearance = abs(float(np.dot(
                    np.asarray(other_position, dtype=float)
                    - np.asarray(position, dtype=float),
                    lateral,
                )))
                if lateral_clearance < minimum_clearance:
                    return False
        return True

    def _slot_velocity(self, position, object_pos, target, pushing=False):
        """Orbit payload slots safely; close companion links directly."""
        if target['role'] == 'companion_push':
            desired_position = target['position']
            if not pushing:
                # APPROACH builds each queue at bumper distance while its
                # payload lead is still parked clear of the box. PUSH can
                # then translate the complete queue instead of closing eight
                # gaps through two robots that are already touching it.
                desired_position = target.get(
                    'assembly_position', desired_position
                )
            difference = desired_position - position
            distance = float(np.linalg.norm(difference))
            parent_velocity = np.asarray(
                target.get('parent_velocity', np.zeros(2)), dtype=float
            )
            if parent_velocity.shape != (2,) or not np.all(
                np.isfinite(parent_velocity)
            ):
                parent_velocity = np.zeros(2)

            velocity = difference * 1.4
            if pushing:
                # Once the payload is moving, match the live parent before
                # adding the small contact pressure. APPROACH uses fixed
                # world slots so a wobbling front robot cannot drag its whole
                # lane sideways.
                velocity += (
                    parent_velocity
                    + target['push_direction'] * 0.04
                )
            elif float(np.linalg.norm(velocity)) < 0.01:
                # Keep turning toward the parent until APPROACH records this
                # link as staged. A zero vector would leave a correctly placed
                # differential-drive robot facing the wrong way forever.
                velocity = target['push_direction'] * 0.01
            if distance > 0.015 and float(np.linalg.norm(velocity)) < 0.03:
                velocity += difference / distance * 0.03
            if pushing and not self._target_has_nominal_contact(
                position, object_pos, target
            ):
                velocity = self._cap_open_contact_velocity(velocity, target)
            speed = float(np.linalg.norm(velocity))
            if speed > self.vmax:
                velocity *= self.vmax / speed
            return (
                Vec2(float(velocity[0]), float(velocity[1])),
                True,
            )

        if pushing:
            # PUSH always begins from the verified staging row behind the
            # payload. Close that clear corridor directly; orbiting here sends
            # the two lead robots sideways just when the chain should engage.
            difference = target['position'] - position
            payload_velocity = np.asarray(
                target.get('parent_velocity', np.zeros(2)), dtype=float
            )
            if payload_velocity.shape != (2,) or not np.all(
                np.isfinite(payload_velocity)
            ):
                payload_velocity = np.zeros(2)
            velocity = (
                payload_velocity
                + difference * 1.4
                + target['push_direction'] * 0.03
            )
            if not self._target_has_nominal_contact(
                position, object_pos, target
            ):
                velocity = self._cap_open_contact_velocity(velocity, target)
            speed = float(np.linalg.norm(velocity))
            engagement_speed = getattr(
                self, 'transport_engagement_speed', self.vmax
            )
            if speed > engagement_speed:
                velocity *= engagement_speed / speed
            return (
                Vec2(float(velocity[0]), float(velocity[1])),
                True,
            )

        desired_position = target['position']
        desired_radial = target['radial']
        desired_position = target.get(
            'staging_position', desired_position
        )
        desired_radial = target.get(
            'staging_radial', desired_radial
        )

        offset = position - object_pos
        radius = float(np.linalg.norm(offset))
        if radius <= 1e-9:
            current_radial = -target['radial']
        else:
            current_radial = offset / radius

        angle_error = math.atan2(
            current_radial[0] * desired_radial[1]
            - current_radial[1] * desired_radial[0],
            float(np.dot(current_radial, desired_radial)),
        )
        aligned = abs(angle_error) <= 0.22

        if not aligned:
            desired_radius = float(np.linalg.norm(
                desired_position - object_pos
            ))
            orbit_radius = max(self.transport_orbit_radius, desired_radius)
            radial_speed = max(
                -0.08,
                min(0.08, 0.7 * (orbit_radius - radius)),
            )
            turn_sign = 1.0 if angle_error > 0.0 else -1.0
            tangent = np.array([
                -current_radial[1] * turn_sign,
                current_radial[0] * turn_sign,
            ])
            tangent_speed = min(
                self.vmax,
                max(0.06, 0.08 + 0.05 * abs(angle_error)),
            )
            velocity = tangent * tangent_speed + current_radial * radial_speed
        else:
            difference = desired_position - position
            distance = float(np.linalg.norm(difference))
            if distance <= 0.015:
                if pushing:
                    velocity = target['push_direction'] * 0.03
                else:
                    # Rear-face pushers should face the shared translation
                    # direction before the force-controlled phase begins.
                    velocity = target['push_direction'] * 0.01
            else:
                speed = min(self.vmax, max(0.03, 0.7 * distance))
                velocity = difference / distance * speed

        payload_velocity = np.asarray(
            target.get('parent_velocity', np.zeros(2)), dtype=float
        )
        if payload_velocity.shape == (2,) and np.all(
            np.isfinite(payload_velocity)
        ):
            # Direct slots move with the payload. Feed-forward prevents a late
            # lane from settling behind a box that another lane already nudged.
            velocity += payload_velocity

        speed = float(np.linalg.norm(velocity))
        if speed > self.vmax:
            velocity *= self.vmax / speed
        allow_designated_contact = aligned
        return (
            Vec2(float(velocity[0]), float(velocity[1])),
            allow_designated_contact,
        )

    def _cap_open_contact_velocity(self, velocity, target):
        """Let an open link catch its parent without striking it at speed."""
        velocity = np.asarray(velocity, dtype=float).copy()
        push_direction = np.asarray(
            target.get('push_direction', np.zeros(2)), dtype=float
        )
        direction_norm = float(np.linalg.norm(push_direction))
        if direction_norm <= 1e-9 or not np.all(np.isfinite(velocity)):
            return velocity
        push_direction /= direction_norm

        parent_velocity = np.asarray(
            target.get('parent_velocity', np.zeros(2)), dtype=float
        )
        if parent_velocity.shape != (2,) or not np.all(
            np.isfinite(parent_velocity)
        ):
            parent_velocity = np.zeros(2)
        parent_forward = max(
            0.0, float(np.dot(parent_velocity, push_direction))
        )
        closing_speed = getattr(
            self, 'transport_push_recovery_closing_speed', 0.025
        )
        if (
            target.get('role') == 'companion_push'
            and target.get('companion_count') == 1
        ):
            closing_speed = getattr(
                self,
                'transport_single_companion_recovery_speed',
                0.025,
            )
        forward_limit = parent_forward + max(0.0, closing_speed)
        requested_forward = float(np.dot(velocity, push_direction))
        if requested_forward > forward_limit:
            velocity -= push_direction * (
                requested_forward - forward_limit
            )
        return velocity

    def _target_has_nominal_contact(self, position, object_pos, target):
        """Return true only while a pusher is at its physical force surface."""
        role = target.get('role')
        parent_position = target.get('parent_position')
        if parent_position is None:
            # Some callers use a lightweight synthetic target. The live target
            # builder always supplies a parent, so preserve the old behaviour
            # for those callers rather than guessing at incomplete geometry.
            return True

        if role == 'companion_push':
            distance = float(np.linalg.norm(
                np.asarray(parent_position, dtype=float)
                - np.asarray(position, dtype=float)
            ))
            return distance <= getattr(
                self, 'transport_companion_contact_distance', 0.145
            ) + 1e-4
        if role == 'payload_push':
            return self._payload_contact_near(
                position,
                object_pos,
                getattr(self, 'object_yaw', 0.0),
                margin=(
                    getattr(self, 'transport_contact_slop', 0.005)
                    + 1e-4
                ),
            )
        return True

    def _slot_contact_ready(
        self, position, yaw, object_pos, object_yaw, target,
        require_inward_heading=True, contact_margin=0.015,
        staging=False,
    ):
        if target['role'] == 'companion_push':
            parent_position = target.get('parent_position')
            if parent_position is None:
                return False
            toward_parent = np.asarray(parent_position) - position
            distance = float(np.linalg.norm(toward_parent))
            if distance <= 1e-9:
                return False
            direction = toward_parent / distance
            push_direction = np.asarray(
                target['push_direction'], dtype=float
            )
            push_norm = float(np.linalg.norm(push_direction))
            if (
                push_direction.shape != (2,)
                or not np.all(np.isfinite(push_direction))
                or push_norm <= 1e-9
            ):
                return False
            push_direction /= push_norm
            angular_error = math.acos(max(
                -1.0,
                min(1.0, float(np.dot(direction, push_direction))),
            ))
            if angular_error > 0.25:
                return False
            if staging:
                desired_position = target.get(
                    'assembly_position', target['position']
                )
                if float(np.linalg.norm(
                    np.asarray(desired_position) - position
                )) > max(0.0, contact_margin):
                    return False
                # Assembly is sampled from two independently moving bodies.
                # Give the staging latch a tightly bounded millimetre-scale
                # capture band so a settled link does not stall forever just
                # outside the nominal distance. This is deliberately not used
                # by _companion_engagement_geometry: launch still waits for a
                # currently connected physical chain.
                staging_contact_limit = (
                    self.transport_companion_contact_distance
                    + max(
                        0.0,
                        min(
                            0.003,
                            getattr(
                                self,
                                'transport_chain_staging_contact_tolerance',
                                0.003,
                            ),
                        ),
                    )
                )
                if distance > staging_contact_limit:
                    return False
                lateral_error = abs(float(
                    push_direction[0] * toward_parent[1]
                    - push_direction[1] * toward_parent[0]
                ))
                if lateral_error > getattr(
                    self,
                    'transport_assembly_contact_lateral_tolerance',
                    0.022,
                ):
                    return False
                minimum_contact_distance = (
                    self._loaded_companion_distance()
                    - max(
                        0.0,
                        getattr(self, 'transport_contact_slop', 0.005),
                    )
                )
                if distance < minimum_contact_distance:
                    return False
            elif distance > (
                self.transport_companion_contact_distance
                + max(0.0, contact_margin)
            ):
                return False
            if not require_inward_heading:
                return True
            heading_direction = (
                push_direction if staging else direction
            )
            desired_yaw = math.atan2(
                heading_direction[1], heading_direction[0]
            )
            heading_tolerance = (
                getattr(
                    self, 'transport_assembly_heading_tolerance', 0.10
                )
                if staging
                else self.transport_contact_heading_tolerance
            )
            return abs(self._normalize_angle(
                desired_yaw - yaw
            )) <= heading_tolerance

        if target['role'] != 'payload_push':
            return False
        desired_position = target['position']
        desired_radial = target['radial']
        if staging:
            desired_position = target.get(
                'staging_position', desired_position
            )
            desired_radial = target.get(
                'staging_radial', desired_radial
            )

        offset = position - object_pos
        distance = float(np.linalg.norm(offset))
        if distance <= 1e-9:
            return False
        radial = offset / distance
        angular_error = math.acos(max(
            -1.0, min(1.0, float(np.dot(radial, desired_radial)))
        ))
        if angular_error > 0.25:
            return False

        target_error = float(np.linalg.norm(
            np.asarray(desired_position) - position
        ))
        if target_error > max(0.0, contact_margin):
            return False
        if not require_inward_heading:
            return True
        push_direction = target['push_direction']
        desired_yaw = math.atan2(
            float(push_direction[1]), float(push_direction[0])
        )
        heading_error = abs(self._normalize_angle(
            desired_yaw - float(yaw)
        ))
        heading_tolerance = (
            getattr(
                self, 'transport_assembly_heading_tolerance', 0.10
            )
            if staging
            else self.transport_contact_heading_tolerance
        )
        return heading_error <= heading_tolerance

    def _companion_engagement_geometry(
        self, position, parent_position, push_direction,
        release=False, allow_alignment_retreat=False,
    ):
        """Measure and close a companion link against its live parent."""
        child = np.asarray(position, dtype=float)
        parent = np.asarray(parent_position, dtype=float)
        direction = np.asarray(push_direction, dtype=float)
        direction_norm = float(np.linalg.norm(direction))
        to_parent = parent - child
        distance = float(np.linalg.norm(to_parent))
        if direction_norm <= 1e-9 or distance <= 1e-9:
            return False, child.copy()

        direction /= direction_norm
        unit_to_parent = to_parent / distance
        longitudinal = float(np.dot(to_parent, direction))
        angular_error = math.acos(max(
            -1.0,
            min(1.0, float(np.dot(unit_to_parent, direction))),
        ))
        engagement_angle = getattr(
            self, 'transport_companion_engagement_angle', 0.35
        )
        release_margin = 0.015 if release else 0.0
        release_angle_margin = (
            getattr(
                self,
                'transport_engagement_release_angle_margin',
                0.15,
            )
            if release else 0.0
        )
        ready = (
            longitudinal > 0.05
            and angular_error
            <= engagement_angle + release_angle_margin
            and distance <= (
                self.transport_companion_contact_distance
                + release_margin
            )
        )

        # Close to the physical bumper surface, not merely to the outer edge
        # of the contact tolerance. Holding at the same distance used by the
        # ready check leaves no settling margin and makes a one-millimetre
        # odometry wobble reset the synchronized-launch timer forever.
        engagement_distance = self._loaded_companion_distance()
        aligned_contact = (
            parent
            - direction * engagement_distance
        )
        destination = aligned_contact.copy()
        gap = distance - engagement_distance
        if (
            not ready
            and allow_alignment_retreat
            and angular_error > engagement_angle
            and distance <= (
                self.transport_companion_contact_distance + 0.03
            )
        ):
            # A tail robot that reaches its parent at an angle cannot slide
            # sideways while its bumper is loaded. Give only the unblocked
            # end of the chain enough room to back out, straighten, and make
            # a clean second approach. Middle links never retreat into the
            # companion waiting behind them.
            retreat_clearance = getattr(
                self, 'transport_alignment_retreat_clearance', 0.055
            )
            destination = (
                parent
                - direction * (
                    self.transport_companion_contact_distance
                    + retreat_clearance
                )
            )
            return False, destination
        if (
            not ready
            and gap > 0.0
            and angular_error <= engagement_angle
        ):
            # A millimetre-scale point error produces less wheel torque than
            # a Burger can use in contact. Keep a modest virtual target past
            # the surface; the next 10 Hz cycle stops at measured contact.
            requested_closing = max(
                float(np.dot(aligned_contact - child, direction)),
                getattr(
                    self,
                    'transport_engagement_minimum_closing_error',
                    0.02,
                ),
            )
            current_closing = float(np.dot(
                destination - child, direction
            ))
            destination += direction * max(
                0.0, requested_closing - current_closing
            )
        return ready, destination

    def _engagement_contact_with_grace(
        self, namespace, ready, previously_engaged, now
    ):
        """Ignore a brief noisy contact sample without latching an open link."""
        last_ready = getattr(
            self, 'transport_engagement_last_ready', {}
        )
        if ready:
            last_ready[namespace] = now
            self.transport_engagement_last_ready = last_ready
            return True

        if namespace not in previously_engaged:
            return False
        ready_at = last_ready.get(namespace)
        hold_time = getattr(
            self, 'transport_engagement_release_hold_time', 0.35
        )
        if (
            hold_time <= 0.0
            or
            ready_at is None
            or not math.isfinite(now)
            or not math.isfinite(ready_at)
            or now < ready_at
        ):
            return False
        return now - ready_at <= hold_time

    def _payload_contact_near(
        self, position, object_pos, object_yaw, margin=None
    ):
        """Keep pressure while a lead robot still touches the rear face."""
        if margin is None:
            margin = getattr(
                self, 'transport_payload_contact_release_margin', 0.04
            )
        offset = np.asarray(position) - np.asarray(object_pos)
        if float(np.linalg.norm(offset)) <= 1e-9:
            return False
        _, rear, _ = self._transport_frame(object_pos)
        if float(np.dot(offset, rear)) <= 0.05:
            return False

        cosine = math.cos(object_yaw)
        sine = math.sin(object_yaw)
        local_x = offset[0] * cosine + offset[1] * sine
        local_y = -offset[0] * sine + offset[1] * cosine
        outside_x = max(abs(float(local_x)) - self.object_half_width, 0.0)
        outside_y = max(abs(float(local_y)) - self.object_half_height, 0.0)
        surface_clearance = math.hypot(outside_x, outside_y)
        return surface_clearance <= (
            self.robot_forward_contact_extent
            + max(0.0, float(margin))
        )

    def _ramped_push_speed(self, requested_speed, now=None):
        """Load a newly assembled chain without hitting the payload."""
        reference_speed = getattr(
            self, 'transport_push_reference_speed', None
        )
        if reference_speed is not None:
            return min(
                requested_speed,
                max(0.0, float(reference_speed)),
            )

        started_at = getattr(
            self, 'transport_push_ramp_started_at', None
        )
        ramp_rate = getattr(self, 'transport_push_ramp_rate', 0.0)
        if started_at is None or ramp_rate <= 0.0:
            return requested_speed

        if now is None:
            get_time = getattr(rospy, 'get_time', lambda: started_at)
            now = float(get_time())
        if not math.isfinite(now) or now < started_at:
            return requested_speed

        initial_speed = getattr(
            self, 'transport_push_ramp_initial_speed', 0.018
        )
        ramped_speed = initial_speed + ramp_rate * (now - started_at)
        return min(requested_speed, max(initial_speed, ramped_speed))

    def _coordinated_push_speed(self, maximum_depth):
        """Return a pace that a complete physical chain can sustain."""
        try:
            maximum_depth = max(0, int(maximum_depth))
        except (TypeError, ValueError):
            maximum_depth = 0

        if maximum_depth == 0:
            return 0.88 * self.vmax

        # Each extra bumper link adds compliance and one more drivetrain that
        # has to settle before the payload row accelerates.  In live Gazebo a
        # depth-four column stayed safe but lost simultaneous pressure at the
        # former 0.038 m/s ceiling.  Ease the pace down to 0.024 m/s there;
        # larger fleets retain that useful floor instead of slowing forever.
        push_scale = max(0.15, 0.35 - 0.05 * maximum_depth)
        return push_scale * self.vmax

    def _advance_push_reference(
        self, targets, full_chain_connected, tracking_ready=True
    ):
        """Move the shared launch pace after one complete fleet command."""
        reference_speed = getattr(
            self, 'transport_push_reference_speed', None
        )
        if reference_speed is None:
            return

        if not full_chain_connected:
            self.transport_push_reference_speed = min(
                reference_speed,
                getattr(self, 'transport_contact_closing_speed', 0.018),
            )
            self.transport_push_ramp_batches = 0
            return

        if not tracking_ready:
            self.transport_push_ramp_batches = 0
            initial_speed = getattr(
                self, 'transport_push_ramp_initial_speed', 0.018
            )
            ramp_step = getattr(
                self, 'transport_push_ramp_step', 0.006
            )
            self.transport_push_reference_speed = max(
                min(reference_speed, initial_speed),
                reference_speed - ramp_step,
            )
            return

        batches = getattr(self, 'transport_push_ramp_batches', 0) + 1
        self.transport_push_ramp_batches = batches
        hold_batches = getattr(
            self, 'transport_push_ramp_hold_batches', 0
        )
        if batches <= hold_batches:
            return

        maximum_depth = max(
            (
                int(target.get('max_chain_depth', 0))
                for target in targets.values()
            ),
            default=0,
        )
        maximum_reference = self._coordinated_push_speed(maximum_depth)
        self.transport_push_reference_speed = min(
            maximum_reference,
            reference_speed
            + getattr(self, 'transport_push_ramp_step', 0.006),
        )
        # Every increment gets its own settling window.  Without this reset,
        # the hold delayed only the first step and a large fleet climbed the
        # rest of the ramp one expensive control cycle at a time.
        self.transport_push_ramp_batches = 0

    def _goal_push_velocity(
        self, sampled_velocity, object_pos, chain_depth=0,
        position=None, target=None,
    ):
        """Maintain forward pressure without collapsing a push chain."""
        goal, _, _ = self._transport_frame(object_pos)
        maximum_depth = 0
        if target is not None:
            try:
                maximum_depth = max(
                    0, int(target.get('max_chain_depth', 0))
                )
            except (TypeError, ValueError):
                maximum_depth = 0
        coordinated_speed = self._coordinated_push_speed(maximum_depth)
        forward_speed = coordinated_speed
        companion_parent_forward = None
        batch_reference = getattr(
            self, 'transport_push_reference_speed', None
        )
        lateral_feedforward = np.zeros(2)
        correction = np.zeros(2)
        nominal_contact = (
            target is None
            or position is None
            or self._target_has_nominal_contact(
                position, object_pos, target
            )
        )

        if target is not None and target.get('role') == 'companion_push':
            parent_velocity = np.asarray(
                target.get('parent_velocity', np.zeros(2)), dtype=float
            )
            robot_velocity = np.asarray(
                target.get('robot_velocity', np.zeros(2)), dtype=float
            )
            if parent_velocity.shape != (2,) or not np.all(
                np.isfinite(parent_velocity)
            ):
                parent_velocity = np.zeros(2)
            if robot_velocity.shape != (2,) or not np.all(
                np.isfinite(robot_velocity)
            ):
                robot_velocity = np.zeros(2)

            parent_forward = max(
                0.0,
                min(self.vmax, float(np.dot(parent_velocity, goal))),
            )
            companion_parent_forward = parent_forward
            robot_forward = float(np.dot(robot_velocity, goal))
            preload = 0.0
            if nominal_contact:
                preload = getattr(
                    self, 'transport_companion_preload', 0.015
                )
                if target.get('companion_count') == 1:
                    preload = getattr(
                        self,
                        'transport_single_companion_preload',
                        0.025,
                    )
            if batch_reference is None:
                # The pre-batch fallback follows the live parent. Runtime
                # tests and legacy callers can still exercise this controller
                # without constructing synchronized-ramp state.
                forward_speed = parent_forward + preload
                forward_speed += max(
                    -0.01,
                    min(0.025, 0.45 * (
                        parent_forward - robot_forward
                    )),
                )
            else:
                # Every loaded row receives the same batch reference. Using
                # each predecessor's measured speed as the whole command base
                # creates a low-pass cascade: by depth four, the rear robot is
                # still starting while the payload row is already moving.
                # Local distance and velocity feedback now add only the force
                # needed at this one bumper link.
                loaded_distance = self._loaded_companion_distance()
                to_parent = (
                    np.asarray(target['parent_position'], dtype=float)
                    - np.asarray(position, dtype=float)
                )
                distance_error = (
                    float(np.dot(to_parent, goal)) - loaded_distance
                )
                # A loaded bumper normally transmits the shared command.  An
                # opening link gets a small catch-up boost; a compressed link
                # yields briefly so its parent can reopen the nominal gap.
                # Without the signed half, the contact guard slows one rear
                # robot and a fleet-wide pace clamp can freeze that short gap
                # forever.
                spacing_deadband = getattr(
                    self, 'transport_push_spacing_deadband', 0.003
                )
                if distance_error < -spacing_deadband:
                    yield_speed = min(
                        getattr(
                            self, 'transport_push_max_yield_speed', 0.012
                        ),
                        getattr(
                            self, 'transport_push_yield_gain', 1.5
                        ) * (-distance_error - spacing_deadband),
                    )
                    minimum_speed = 0.25 * float(batch_reference)
                    forward_speed = max(
                        minimum_speed,
                        float(batch_reference) - yield_speed,
                    )
                else:
                    gap_excess = max(
                        0.0, distance_error - spacing_deadband
                    )
                    speed_lag = max(
                        0.0,
                        parent_forward - robot_forward - 0.002,
                    )
                    catch_up = min(
                        0.008,
                        1.5 * gap_excess + 0.20 * speed_lag,
                    )
                    forward_speed = min(
                        self.vmax,
                        float(batch_reference) + catch_up,
                    )
            lateral_feedforward = (
                parent_velocity - goal * parent_forward
            )

        spacing_correction = 0.0
        if position is not None and target is not None:
            correction = np.asarray(target['position']) - position
            longitudinal_error = float(np.dot(correction, goal))
            longitudinal_gain = (
                3.2
                if target.get('role') == 'companion_push'
                else 2.4
            )
            spacing_correction = min(
                0.060, longitudinal_gain * longitudinal_error
            )
            if target.get('role') != 'companion_push':
                spacing_correction = max(0.0, spacing_correction)
            correction = (
                correction - goal * longitudinal_error
            ) * 0.7

        if (
            companion_parent_forward is not None
            and batch_reference is None
        ):
            # Bound the child-to-parent speed, rather than the child's whole
            # command. This keeps a loaded link pressing gently, gives an
            # open link a finite catch-up speed, and lets a compressed link
            # fall behind its parent until the safe spacing is restored.
            relative_speed = (
                forward_speed
                - companion_parent_forward
                + spacing_correction
            )
            relative_speed = max(
                -0.040, min(0.025, relative_speed)
            )
            forward_speed = max(
                0.0, companion_parent_forward + relative_speed
            )
        elif companion_parent_forward is not None:
            # The synchronized branch already folded longitudinal spacing
            # into its bounded local adjustment above.
            forward_speed = max(0.0, forward_speed)
        else:
            forward_speed = max(
                0.0, forward_speed + spacing_correction
            )

        if target is not None and not nominal_contact:
            limited_velocity = self._cap_open_contact_velocity(
                goal * max(0.0, forward_speed), target
            )
            forward_speed = max(
                0.0, float(np.dot(limited_velocity, goal))
            )
        if not (
            target is not None
            and target.get('role') == 'companion_push'
            and batch_reference is not None
        ):
            forward_speed = self._ramped_push_speed(
                max(0.0, forward_speed)
            )
        lateral_velocity = lateral_feedforward + correction
        lateral_velocity -= goal * float(np.dot(lateral_velocity, goal))
        lateral_speed = float(np.linalg.norm(lateral_velocity))
        heading_cone = getattr(
            self,
            'transport_chain_heading_cone'
            if maximum_depth > 0
            else 'transport_push_heading_cone',
            0.06 if maximum_depth > 0 else 0.18,
        )
        lateral_limit = forward_speed * math.tan(heading_cone)
        if lateral_speed > lateral_limit and lateral_speed > 1e-9:
            lateral_velocity *= lateral_limit / lateral_speed

        velocity = goal * forward_speed + lateral_velocity
        speed = float(np.linalg.norm(velocity))
        if speed > self.vmax:
            velocity *= self.vmax / speed
        return Vec2(float(velocity[0]), float(velocity[1]))

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

        # Gazebo's Burger needs time to build wheel torque while turning.
        # Blend translation in only after it is facing the route closely;
        # otherwise a nominal point command becomes a metre-wide arc.
        linear *= (
            max(0.0, math.cos(angle_error))
            * self._aligned_motion_scale(angle_error)
        )

        cmd.linear.x = linear
        cmd.angular.z = max(-2.84, min(2.84, angular))
        return cmd

    def _loaded_chain_command(self, velocity, yaw, target):
        """Convert a loaded-chain velocity without letting the row shear."""
        command = self._holonomic_to_diff_drive(
            velocity.x, velocity.y, yaw
        )
        try:
            maximum_depth = int(target.get('max_chain_depth', 0))
        except (TypeError, ValueError):
            maximum_depth = 0
        if maximum_depth <= 0:
            return command

        push_direction = np.asarray(
            target.get('push_direction', np.zeros(2)), dtype=float
        )
        direction_norm = float(np.linalg.norm(push_direction))
        if direction_norm <= 1e-9 or not np.all(np.isfinite(push_direction)):
            return Twist()
        push_direction /= direction_norm
        desired_yaw = math.atan2(
            float(push_direction[1]), float(push_direction[0])
        )
        heading_error = self._normalize_angle(desired_yaw - yaw)

        # Differential wheel acceleration is shared between turning and
        # translation. Once a loaded Burger points more than a few degrees
        # away from the common force direction, slow its forward wheel pair
        # and straighten first. Otherwise every depth draws a different arc
        # and the rear link opens before its requested catch-up reaches Gazebo.
        full_motion_angle = getattr(
            self, 'transport_launch_heading_tolerance', 0.08
        )
        stop_motion_angle = max(
            full_motion_angle + 0.04,
            min(0.24, full_motion_angle + 0.16),
        )
        command.linear.x *= self._aligned_motion_scale(
            heading_error,
            full_motion_angle=full_motion_angle,
            stop_motion_angle=stop_motion_angle,
        )
        command.angular.z = max(
            -2.84, min(2.84, 2.0 * heading_error)
        )
        return command

    @staticmethod
    def _aligned_motion_scale(
        heading_error, full_motion_angle=0.20, stop_motion_angle=0.65
    ):
        """Smoothly hold translation until a differential drive is aligned."""
        error = abs(float(heading_error))
        if error <= full_motion_angle:
            return 1.0
        if error >= stop_motion_angle:
            return 0.0
        blend = (
            (stop_motion_angle - error)
            / (stop_motion_angle - full_motion_angle)
        )
        return blend * blend * (3.0 - 2.0 * blend)

    def _chain_pose_hold_command(
        self, position, yaw, destination, push_direction,
        forward_limit=0.06, reverse_limit=0.04,
    ):
        """Hold a chain pose without turning around to correct overshoot."""
        goal = np.asarray(push_direction, dtype=float)
        goal_norm = float(np.linalg.norm(goal))
        if goal_norm <= 1e-9:
            return Twist()
        goal /= goal_norm
        lateral = np.array([-goal[1], goal[0]])
        difference = np.asarray(destination, dtype=float) - position
        longitudinal_error = float(np.dot(difference, goal))
        lateral_error = float(np.dot(difference, lateral))

        linear = max(
            -abs(reverse_limit),
            min(abs(forward_limit), 1.2 * longitudinal_error),
        )
        if abs(linear) < 0.015 and abs(lateral_error) > 0.02:
            linear = min(abs(forward_limit), 0.02)

        desired_yaw = math.atan2(float(goal[1]), float(goal[0]))
        heading_error = self._normalize_angle(desired_yaw - yaw)
        linear *= (
            max(0.0, math.cos(heading_error))
            * self._aligned_motion_scale(heading_error)
        )
        steering_direction = 1.0 if linear >= 0.0 else -1.0
        angular = (
            2.0 * heading_error
            + steering_direction * 2.5 * lateral_error
        )

        command = Twist()
        command.linear.x = linear
        command.angular.z = max(-1.5, min(1.5, angular))
        return command

    def _companion_assembly_command(
        self, position, yaw, target, allow_closing=True
    ):
        """Straighten a rear pusher before closing its final queue gap."""
        push_direction = np.asarray(
            target.get('push_direction', np.zeros(2)), dtype=float
        )
        direction_norm = float(np.linalg.norm(push_direction))
        if (
            push_direction.shape != (2,)
            or not np.all(np.isfinite(push_direction))
            or direction_norm <= 1e-9
        ):
            return Twist()
        push_direction /= direction_norm

        fixed_destination = np.asarray(
            target.get('assembly_position', target['position']), dtype=float
        )
        lateral = np.array([-push_direction[1], push_direction[0]])
        handoff = self._companion_assembly_route_destination(target)
        lateral_error = float(np.dot(
            fixed_destination - position, lateral
        ))
        handoff_tolerance = getattr(
            self, 'transport_assembly_handoff_tolerance', 0.015
        )
        closing_span = max(
            0.0,
            float(np.dot(fixed_destination - handoff, push_direction)),
        )
        handoff_progress = float(np.dot(
            position - handoff, push_direction
        ))
        handoff_is_acquired = (
            abs(lateral_error) <= handoff_tolerance
            and handoff_progress >= -handoff_tolerance
            and handoff_progress <= closing_span + handoff_tolerance
        )
        if not handoff_is_acquired or not allow_closing:
            # Finish acquiring the lane while there is still an open gap to
            # the parent. Contact is allowed only after this side error is
            # gone, otherwise a small forward command applies a large yawing
            # moment to the robot already holding the front of the queue.
            correction_speed = min(
                0.06,
                getattr(self, 'transport_chain_staging_speed', 0.16),
            )
            return self._payload_staging_command(
                position,
                yaw,
                handoff,
                push_direction,
                forward_limit=correction_speed,
                reverse_limit=correction_speed,
            )

        desired_yaw = math.atan2(
            float(push_direction[1]), float(push_direction[0])
        )
        heading_error = self._normalize_angle(desired_yaw - yaw)
        closing_angle = min(
            getattr(
                self, 'transport_companion_engagement_angle', 0.20
            ),
            getattr(
                self, 'transport_launch_heading_tolerance', 0.08
            ),
            getattr(
                self, 'transport_assembly_heading_tolerance', 0.10
            ),
        )
        if abs(heading_error) > closing_angle:
            # The route controller may hand a robot over while it is still
            # facing its last waypoint. Driving during this turn makes a
            # differential-drive Burger orbit the contact point indefinitely.
            command = Twist()
            command.angular.z = max(
                -2.84, min(2.84, 2.0 * heading_error)
            )
            return command

        closing_speed = getattr(
            self, 'transport_assembly_closing_speed', 0.012
        )
        destination = fixed_destination
        parent_position = np.asarray(
            target.get('parent_position', np.empty(0)), dtype=float
        )
        if (
            parent_position.shape == (2,)
            and np.all(np.isfinite(parent_position))
        ):
            destination = (
                parent_position
                - push_direction * self._loaded_companion_distance()
            )
        return self._chain_pose_hold_command(
            position,
            yaw,
            destination,
            push_direction,
            forward_limit=closing_speed,
            reverse_limit=closing_speed,
        )

    def _payload_staging_command(
        self, position, yaw, destination, push_direction,
        forward_limit=0.08, reverse_limit=0.06,
    ):
        """Drive a payload pusher to one point, then face the push direction."""
        difference = np.asarray(destination, dtype=float) - position
        distance = float(np.linalg.norm(difference))
        settled_distance = min(
            0.012,
            getattr(self, 'transport_chain_staging_tolerance', 0.025),
        )

        if distance <= settled_distance:
            desired_yaw = math.atan2(
                float(push_direction[1]), float(push_direction[0])
            )
            command = Twist()
            command.angular.z = max(
                -1.5,
                min(
                    1.5,
                    2.0 * self._normalize_angle(desired_yaw - yaw),
                ),
            )
            return command

        bearing = math.atan2(float(difference[1]), float(difference[0]))
        forward_error = self._normalize_angle(bearing - yaw)
        reverse_error = self._normalize_angle(bearing + math.pi - yaw)
        if abs(reverse_error) < abs(forward_error):
            direction = -1.0
            heading_error = reverse_error
            speed_limit = abs(reverse_limit)
        else:
            direction = 1.0
            heading_error = forward_error
            speed_limit = abs(forward_limit)

        # Point-bearing steering converges from diagonal arrivals. Choosing
        # reverse when it is the shorter turn also backs a pusher away after
        # an overshoot instead of making it turn beside the payload.
        speed = min(speed_limit, 1.2 * distance)
        speed *= (
            max(0.0, math.cos(heading_error))
            * self._aligned_motion_scale(heading_error)
        )
        command = Twist()
        command.linear.x = direction * speed
        command.angular.z = max(
            -1.5, min(1.5, 2.0 * heading_error)
        )
        return command

    def _payload_staging_route_destination(self, target):
        """Place the route gate outside the payload avoidance envelope."""
        staging_position = np.asarray(
            target.get('staging_position', target['position']), dtype=float
        )
        push_direction = np.asarray(target['push_direction'], dtype=float)
        staging_surface_gap = (
            getattr(self, 'robot_forward_contact_extent', 0.038)
            + getattr(self, 'transport_contact_slop', 0.005)
            + getattr(self, 'transport_staging_clearance', 0.25)
        )
        required_surface_gap = (
            getattr(self, 'transport_payload_route_awareness', 0.80)
            + getattr(self, 'transport_payload_route_lidar_offset', 0.032)
            + getattr(self, 'transport_payload_route_margin', 0.05)
        )
        final_approach = max(
            0.10,
            required_surface_gap - staging_surface_gap,
        )
        return staging_position - push_direction * final_approach

    def _companion_assembly_route_destination(self, target):
        """Route to a clear handoff, then let guarded contact close the link."""
        assembly_position = np.asarray(
            target.get('assembly_position', target['position']),
            dtype=float,
        )
        push_direction = np.asarray(
            target.get('push_direction', np.zeros(2)), dtype=float
        )
        direction_norm = float(np.linalg.norm(push_direction))
        if direction_norm <= 1e-9:
            return assembly_position

        contact_distance = self._loaded_companion_distance()
        route_clearance = max(
            contact_distance,
            getattr(self, 'transport_route_parent_clearance', 0.20),
        )
        # The endpoint stays just outside its clearance circle. The normal
        # final-handoff controller owns the deliberate bumper contact.
        handoff_distance = route_clearance + 0.01
        return (
            assembly_position
            - push_direction / direction_norm
            * max(0.0, handoff_distance - contact_distance)
        )

    def _guard_payload_approach(
        self, position, yaw, command, object_pos, object_yaw
    ):
        """Stop a pre-push command before it can make payload contact."""
        linear = float(command.linear.x)
        if abs(linear) <= 1e-9:
            return command, False

        to_object = np.asarray(object_pos, dtype=float) - position
        center_distance = float(np.linalg.norm(to_object))
        if center_distance <= 1e-9:
            return Twist(), True
        heading = np.array([math.cos(yaw), math.sin(yaw)])
        closing_speed = float(np.dot(
            heading * linear, to_object / center_distance
        ))
        if closing_speed <= 0.0:
            return command, False

        clearance = center_distance - self._contact_distance_at(
            position, object_pos, object_yaw
        )
        if clearance > getattr(
            self,
            'transport_payload_approach_min_clearance', 0.08
        ):
            return command, False
        return Twist(), True

    def _staging_alignment_command(self, position, yaw, target):
        """Rotate a positioned link toward the force shared by its chain."""
        if target['role'] == 'companion_push':
            direction = np.asarray(target['parent_position']) - position
        else:
            direction = np.asarray(target['push_direction'])
        return self._push_alignment_command(yaw, direction)

    def _push_alignment_command(self, yaw, push_direction):
        """Turn in place until a robot faces one shared force direction."""
        direction = np.asarray(push_direction, dtype=float)
        if (
            direction.shape != (2,)
            or not np.all(np.isfinite(direction))
            or not math.isfinite(float(yaw))
        ):
            return Twist()
        if float(np.linalg.norm(direction)) <= 1e-9:
            return Twist()

        desired_yaw = math.atan2(float(direction[1]), float(direction[0]))
        command = Twist()
        command.angular.z = max(
            -2.84,
            min(
                2.84,
                2.0 * self._normalize_angle(desired_yaw - yaw),
            ),
        )
        return command

    def _chain_staging_command(self, position, yaw, target):
        """Pre-position a rear link without touching an unfinished chain."""
        staging_position = target.get('staging_position')
        if staging_position is None:
            return Twist()

        difference = np.asarray(staging_position) - position
        distance = float(np.linalg.norm(difference))
        settled_distance = min(
            0.01,
            getattr(self, 'transport_chain_staging_tolerance', 0.025),
        )
        if distance > settled_distance:
            staging_speed = getattr(
                self, 'transport_chain_staging_speed', 0.16
            )
            speed = min(
                self.vmax,
                staging_speed,
                max(0.03, 0.8 * distance),
            )
            velocity = difference / distance * speed
            return self._holonomic_to_diff_drive(
                float(velocity[0]), float(velocity[1]), yaw
            )

        push_direction = np.asarray(target['push_direction'])
        desired_yaw = math.atan2(
            float(push_direction[1]), float(push_direction[0])
        )
        command = Twist()
        command.angular.z = max(
            -2.84,
            min(
                2.84,
                2.0 * self._normalize_angle(desired_yaw - yaw),
            ),
        )
        return command

    def _plan_transport_route(
        self, namespace, start, end, positions, ignored_namespaces=(),
        close_approach_namespaces=(),
    ):
        """Plan around arena obstacles and robots that are currently held."""
        route_zones = [
            dict(zone)
            for zone in getattr(self, 'spawn_exclusion_zones', ())
            if isinstance(zone, dict)
        ]
        ignored = set(ignored_namespaces)
        ignored.add(namespace)
        close_approach = set(close_approach_namespaces)
        robot_clearance = getattr(
            self, 'transport_route_robot_clearance', 0.32
        )
        parent_clearance = getattr(
            self, 'transport_route_parent_clearance', 0.20
        )
        for other_namespace, other_position in positions.items():
            if other_namespace in ignored:
                continue
            point = np.asarray(other_position, dtype=float)
            if point.shape != (2,) or not np.all(np.isfinite(point)):
                continue
            route_zones.append({
                'name': 'held_{}'.format(other_namespace),
                'shape': 'circle',
                'x': float(point[0]),
                'y': float(point[1]),
                'radius': 0.0,
                'clearance': (
                    parent_clearance
                    if other_namespace in close_approach
                    else robot_clearance
                ),
            })

        with self.model_lock:
            model_poses = dict(getattr(self, 'model_poses', {}))
        return plan_obstacle_aware_route(
            (float(start[0]), float(start[1])),
            (float(end[0]), float(end[1])),
            getattr(self, 'arena_size', 10.0),
            getattr(self, 'arena_margin', 0.35),
            getattr(self, 'transport_route_obstacle_clearance', 0.30),
            route_zones,
            getattr(self, 'arena_profile', 'swarm_arena'),
            model_poses,
            circle_samples=8,
        )

    def _load_assembly_route_state(self, namespace):
        """Load one robot's approach route into the working fields."""
        state = getattr(
            self, 'transport_assembly_route_states', {}
        ).get(namespace)
        if state is None:
            self.transport_route_namespace = None
            self.transport_route_kind = None
            self.transport_route_target = None
            self.transport_route_waypoints = []
            self.transport_route_waypoint_index = 0
            self.transport_route_last_plan_time = None
            self.transport_route_complete = False
            self.transport_route_reverse_active = False
            self.transport_route_reverse_finished = False
            self.transport_route_turn_direction = 0.0
            return

        self.transport_route_namespace = namespace
        self.transport_route_kind = state.get('kind')
        self.transport_route_target = state.get('target')
        self.transport_route_waypoints = list(
            state.get('waypoints', ())
        )
        self.transport_route_waypoint_index = int(
            state.get('waypoint_index', 0)
        )
        self.transport_route_last_plan_time = state.get(
            'last_plan_time'
        )
        self.transport_route_complete = bool(state.get('complete', False))
        self.transport_route_reverse_active = bool(
            state.get('reverse_active', False)
        )
        self.transport_route_reverse_finished = bool(
            state.get('reverse_finished', False)
        )
        self.transport_route_turn_direction = float(
            state.get('turn_direction', 0.0)
        )

    def _save_assembly_route_state(self, namespace):
        """Keep independent route progress for every approaching robot."""
        states = getattr(self, 'transport_assembly_route_states', None)
        if not isinstance(states, dict):
            states = {}
            self.transport_assembly_route_states = states
        target = getattr(self, 'transport_route_target', None)
        states[namespace] = {
            'kind': getattr(self, 'transport_route_kind', None),
            'target': (
                None
                if target is None
                else np.asarray(target, dtype=float).copy()
            ),
            'waypoints': [
                np.asarray(point, dtype=float).copy()
                for point in getattr(
                    self, 'transport_route_waypoints', ()
                )
            ],
            'waypoint_index': getattr(
                self, 'transport_route_waypoint_index', 0
            ),
            'last_plan_time': getattr(
                self, 'transport_route_last_plan_time', None
            ),
            'complete': getattr(
                self, 'transport_route_complete', False
            ),
            'reverse_active': getattr(
                self, 'transport_route_reverse_active', False
            ),
            'reverse_finished': getattr(
                self, 'transport_route_reverse_finished', False
            ),
            'turn_direction': getattr(
                self, 'transport_route_turn_direction', 0.0
            ),
        }

    def _transport_route_command(
        self, namespace, position, yaw, destination, positions,
        route_kind, ignored_namespaces=(), close_approach_namespaces=(),
        allow_final_handoff=True, final_handoff_tolerance=None,
    ):
        """Run one cached route without sharing state between movers."""
        arguments = (
            namespace,
            position,
            yaw,
            destination,
            positions,
            route_kind,
        )
        keyword_arguments = {
            'ignored_namespaces': ignored_namespaces,
            'close_approach_namespaces': close_approach_namespaces,
            'allow_final_handoff': allow_final_handoff,
            'final_handoff_tolerance': final_handoff_tolerance,
        }
        self._load_assembly_route_state(namespace)
        try:
            return self._transport_route_command_loaded(
                *arguments, **keyword_arguments
            )
        finally:
            self._save_assembly_route_state(namespace)

    def _transport_route_command_loaded(
        self, namespace, position, yaw, destination, positions,
        route_kind, ignored_namespaces=(), close_approach_namespaces=(),
        allow_final_handoff=True, final_handoff_tolerance=None,
    ):
        """Follow a cached route, returning None for the final precise move."""
        destination = np.asarray(destination, dtype=float)
        if destination.shape != (2,) or not np.all(np.isfinite(destination)):
            return Twist()

        cached_target = getattr(self, 'transport_route_target', None)
        route_matches = (
            getattr(self, 'transport_route_namespace', None) == namespace
            and getattr(self, 'transport_route_kind', None) == route_kind
            and cached_target is not None
            and float(np.linalg.norm(
                destination - np.asarray(cached_target, dtype=float)
            )) <= 0.02
        )
        get_time = getattr(rospy, 'get_time', lambda: 0.0)
        now = float(get_time())
        last_plan_time = getattr(
            self, 'transport_route_last_plan_time', None
        )
        retry_failed_route = (
            route_matches
            and not getattr(self, 'transport_route_waypoints', [])
            and (
                last_plan_time is None
                or not math.isfinite(now)
                or now < last_plan_time
                or now - last_plan_time >= 2.0
            )
        )
        if (
            route_matches
            and getattr(self, 'transport_route_complete', False)
        ):
            return None
        if not route_matches or retry_failed_route:
            route = self._plan_transport_route(
                namespace,
                position,
                destination,
                positions,
                ignored_namespaces,
                close_approach_namespaces,
            )
            self.transport_route_namespace = namespace
            self.transport_route_kind = route_kind
            self.transport_route_target = destination.copy()
            self.transport_route_waypoint_index = 0
            self.transport_route_complete = False
            self.transport_route_reverse_active = False
            self.transport_route_reverse_finished = False
            self.transport_route_turn_direction = 0.0
            self.transport_route_last_plan_time = (
                now if math.isfinite(now) else None
            )
            if route is None:
                self.transport_route_waypoints = []
                rospy.logwarn_throttle(
                    2.0,
                    "[transport] no safe %s route for %s; holding",
                    route_kind,
                    namespace,
                )
                return Twist()
            self.transport_route_waypoints = [
                np.asarray(point, dtype=float)
                for point in (route[1:] or route[-1:])
            ]
            rospy.loginfo(
                "[transport] routed %s %s through %d waypoint(s)",
                namespace,
                route_kind,
                len(self.transport_route_waypoints),
            )

        waypoints = getattr(self, 'transport_route_waypoints', [])
        if not waypoints:
            return Twist()
        waypoint_index = min(
            getattr(self, 'transport_route_waypoint_index', 0),
            len(waypoints) - 1,
        )
        tolerance = getattr(
            self, 'transport_route_waypoint_tolerance', 0.09
        )
        if route_kind == 'pre_staging':
            # Parking routes weave around robots that are deliberately held
            # still. A broad corner tolerance can skip the outside of that
            # detour and aim straight through the held robot's brake zone.
            tolerance = min(
                tolerance,
                getattr(
                    self,
                    'transport_parking_route_waypoint_tolerance',
                    0.04,
                ),
            )
        previous_waypoint_index = waypoint_index
        while waypoint_index < len(waypoints) - 1:
            if float(np.linalg.norm(
                waypoints[waypoint_index] - position
            )) > tolerance:
                break
            waypoint_index += 1
        self.transport_route_waypoint_index = waypoint_index
        if waypoint_index != previous_waypoint_index:
            # Start the new leg from rest. Carrying the smoothed velocity from
            # the old bearing turns a safe polyline corner into a wide arc.
            avoidance = getattr(
                self, 'avoidance_modules', {}
            ).get(namespace)
            if avoidance is not None:
                avoidance.reset_motion()

        difference = waypoints[waypoint_index] - position
        distance = float(np.linalg.norm(difference))
        if final_handoff_tolerance is None:
            final_tolerance = getattr(
                self,
                'transport_route_final_handoff_tolerance',
                max(tolerance, 0.25),
            )
        else:
            try:
                final_tolerance = max(
                    0.0, float(final_handoff_tolerance)
                )
            except (TypeError, ValueError, OverflowError):
                final_tolerance = 0.0
        if (
            allow_final_handoff
            and
            waypoint_index == len(waypoints) - 1
            and distance <= final_tolerance
        ):
            self.transport_route_complete = True
            avoidance = getattr(
                self, 'avoidance_modules', {}
            ).get(namespace)
            if avoidance is not None:
                avoidance.reset_motion()
            return None
        if distance <= 1e-9:
            return Twist()

        speed = min(
            self.vmax,
            getattr(self, 'transport_chain_staging_speed', 0.16),
            max(0.04, 0.8 * distance),
        )
        velocity = difference / distance * speed
        desired_yaw = math.atan2(
            float(velocity[1]), float(velocity[0])
        )
        forward_error = self._normalize_angle(desired_yaw - yaw)
        reverse_error = self._normalize_angle(
            desired_yaw + math.pi - yaw
        )
        final_assembly_leg = (
            route_kind == 'assembly'
            and waypoint_index == len(waypoints) - 1
        )
        reverse_active = bool(getattr(
            self, 'transport_route_reverse_active', False
        ))
        reverse_finished = bool(getattr(
            self, 'transport_route_reverse_finished', False
        ))
        turn_clearance = max(0.08, getattr(
            self, 'transport_route_reverse_turn_clearance', 0.12
        ))
        if final_assembly_leg and not reverse_active and not reverse_finished:
            if (
                distance > turn_clearance
                and abs(reverse_error) < abs(forward_error)
            ):
                reverse_active = True
                self.transport_route_reverse_active = True
            else:
                self.transport_route_reverse_finished = True

        if reverse_active and distance <= turn_clearance:
            # Stop before the rear shell reaches the waiting parent. Clearing
            # the smoother here gives the Burger one deliberate turn, with a
            # full body-length of room, before the precise forward handoff.
            self.transport_route_reverse_active = False
            self.transport_route_reverse_finished = True
            self.transport_route_turn_direction = (
                1.0 if forward_error >= 0.0 else -1.0
            )
            reverse_active = False
            avoidance = getattr(
                self, 'avoidance_modules', {}
            ).get(namespace)
            if avoidance is not None:
                avoidance.reset_motion()

        turn_direction = float(getattr(
            self, 'transport_route_turn_direction', 0.0
        ))
        turn_tolerance = max(0.05, getattr(
            self, 'transport_route_turn_tolerance', 0.12
        ))
        if turn_direction != 0.0:
            if abs(forward_error) <= turn_tolerance:
                self.transport_route_turn_direction = 0.0
                avoidance = getattr(
                    self, 'avoidance_modules', {}
                ).get(namespace)
                if avoidance is not None:
                    avoidance.reset_motion()
                return Twist()

            command = Twist()
            turn_speed = min(
                0.90,
                max(0.18, 1.2 * abs(forward_error)),
            )
            command.angular.z = math.copysign(
                turn_speed, turn_direction
            )
            return command

        if reverse_active:
            # Assembly owns one obstacle-cleared mover. Commit to the shorter
            # reverse heading for that final planned leg instead of changing
            # direction whenever yaw crosses the forward/reverse boundary.
            # Parking routes remain forward-only.
            command = Twist()
            command.linear.x = -speed * (
                max(0.0, math.cos(reverse_error))
                * self._aligned_motion_scale(reverse_error)
            )
            command.angular.z = max(
                -2.84, min(2.84, 2.0 * reverse_error)
            )
            return command
        return self._holonomic_to_diff_drive(
            float(velocity[0]), float(velocity[1]), yaw
        )

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
                    object_potential_weight=(
                        self.object_potential_weight
                    ),
                    neighbor_potential_weight=(
                        self.neighbor_potential_weight
                    ),
                    orbit_alignment_weight=(
                        self.orbit_alignment_weight
                    ),
                    push_alignment_weight=self.push_alignment_weight,
                    velocity_consensus_weight=(
                        self.velocity_consensus_weight
                    ),
                )
            )
            self._grf_kernels[iterations] = kernel
        return kernel, iterations

    def _build_grf_snapshot(
        self, namespaces, object_pos, object_yaw, obstacles
    ):
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
            object_half_width=self.object_half_width,
            object_half_height=self.object_half_height,
            object_yaw=object_yaw,
        )
        return snapshot, positions, yaws

    def _grf_push_commands(
        self, namespaces, object_pos, object_yaw, obstacles, expected_epoch
    ):
        snapshot, positions, yaws = self._build_grf_snapshot(
            namespaces, object_pos, object_yaw, obstacles
        )
        kernel, iterations = self._grf_kernel_for_fleet(
            len(snapshot.robots)
        )
        with self.command_lock:
            if not self._command_allowed(expected_epoch):
                return {}, positions, yaws
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
                return {}, positions, yaws
            self._grf_step_index = max(
                self._grf_step_index, step_index + 1
            )
            self._active_planner = 'grf'
            self._active_grf_iterations = iterations
        return commands, positions, yaws

    def _loaded_link_gap_state(self, namespace, targets, positions):
        """Describe one companion's measured gap along the push lane."""
        target = targets.get(namespace)
        child_position = positions.get(namespace)
        if target is None or child_position is None:
            return None
        parent_namespace = target.get('parent_namespace')
        parent_position = positions.get(parent_namespace)
        push_direction = np.asarray(
            target.get('push_direction', np.empty(0)), dtype=float
        )
        if (
            parent_position is None
            or push_direction.shape != (2,)
            or not np.all(np.isfinite(push_direction))
        ):
            return None
        direction_norm = float(np.linalg.norm(push_direction))
        if direction_norm <= 1e-9:
            return None
        push_direction /= direction_norm

        gap = float(np.dot(
            np.asarray(parent_position, dtype=float)
            - np.asarray(child_position, dtype=float),
            push_direction,
        ))
        error = gap - self._loaded_companion_distance()
        deadband = getattr(
            self, 'transport_push_spacing_deadband', 0.003
        )
        if error < -deadband:
            state = 'compressed'
        elif error > deadband:
            state = 'open'
        else:
            state = 'nominal'
        return state, gap, error

    @staticmethod
    def _empty_push_arbitration():
        return {
            'raw_speeds': {},
            'coordinated_speeds': {},
            'link_states': {},
            'hard_stop_sources': (),
        }

    def _store_push_arbitration(self, arbitration):
        """Keep the arbitration record that belongs to a published batch."""
        self.transport_push_raw_speeds = dict(
            arbitration.get('raw_speeds', {})
        )
        self.transport_push_coordinated_speeds = dict(
            arbitration.get('coordinated_speeds', {})
        )
        self.transport_push_link_states = dict(
            arbitration.get('link_states', {})
        )
        self.transport_push_hard_stop_sources = tuple(
            arbitration.get('hard_stop_sources', ())
        )

    def _coordinate_loaded_chain_commands(
        self, commands, targets, positions, record_telemetry=True
    ):
        """Coordinate both loaded lanes without freezing a short link.

        Arbitration only lowers post-avoidance commands.  A compressed child
        may yield while its parent moves ahead, nominal rows inherit their
        parent's pace, and an open row gets only a small catch-up allowance.
        Commands are changed in place and the matching telemetry is returned.
        """
        if not commands or not targets:
            arbitration = self._empty_push_arbitration()
            if record_telemetry:
                self._store_push_arbitration(arbitration)
            return arbitration

        raw_speeds = {
            namespace: max(
                0.0,
                float(commands.get(namespace, Twist()).linear.x),
            )
            for namespace in targets
        }
        link_states = {
            namespace: self._loaded_link_gap_state(
                namespace, targets, positions
            )
            for namespace, target in targets.items()
            if target.get('role') == 'companion_push'
        }

        # A zero post-avoidance command is a translation stop, except when a
        # child is yielding at an already short designated contact.  In that
        # one case moving the parent away makes the geometry safer.  Any other
        # stop holds the complete mechanically coupled fleet while retaining
        # each robot's in-place steering command.
        motion_epsilon = 1e-3
        hard_stops = []
        for namespace, speed in raw_speeds.items():
            if speed > motion_epsilon:
                continue
            target = targets[namespace]
            state = link_states.get(namespace)
            if (
                target.get('role') != 'companion_push'
                or state is None
                or state[0] != 'compressed'
            ):
                hard_stops.append(namespace)
        if hard_stops:
            for namespace in targets:
                if namespace in commands:
                    commands[namespace].linear.x = 0.0
            arbitration = {
                'raw_speeds': raw_speeds,
                'coordinated_speeds': {
                    namespace: 0.0 for namespace in targets
                },
                'link_states': link_states,
                'hard_stop_sources': tuple(sorted(hard_stops)),
            }
            if record_telemetry:
                self._store_push_arbitration(arbitration)
            return arbitration

        chains = {}
        roots = []
        for namespace, target in targets.items():
            chain_index = int(target.get('chain_index', 0))
            chains.setdefault(chain_index, []).append(namespace)
            if target.get('role') == 'payload_push':
                roots.append(namespace)
        for chain in chains.values():
            chain.sort(key=lambda item: int(
                targets[item].get('chain_depth', 0)
            ))

        # Work backwards first.  A nominal or open child that cannot sustain
        # its parent's pace limits that lane.  A compressed edge is excluded:
        # its parent must keep moving briefly or the gap can never reopen.
        lane_caps = dict(raw_speeds)
        for chain in chains.values():
            for namespace in reversed(chain[1:]):
                parent_namespace = targets[namespace].get(
                    'parent_namespace'
                )
                state = link_states.get(namespace)
                if parent_namespace not in lane_caps or state is None:
                    continue
                if state[0] != 'compressed':
                    lane_caps[parent_namespace] = min(
                        lane_caps[parent_namespace], lane_caps[namespace]
                    )

        root_pace = min(
            (lane_caps[namespace] for namespace in roots),
            default=0.0,
        )
        coordinated = {namespace: 0.0 for namespace in targets}
        for namespace in roots:
            coordinated[namespace] = min(
                raw_speeds[namespace], root_pace
            )

        reference_speed = getattr(
            self, 'transport_push_reference_speed', None
        )
        try:
            reference_speed = max(0.0, float(reference_speed))
        except (TypeError, ValueError):
            reference_speed = root_pace
        minimum_yield_speed = 0.25 * reference_speed
        maximum_yield = getattr(
            self, 'transport_push_max_yield_speed', 0.012
        )
        yield_gain = getattr(
            self, 'transport_push_yield_gain', 1.5
        )

        for chain in chains.values():
            for namespace in chain[1:]:
                parent_namespace = targets[namespace].get(
                    'parent_namespace'
                )
                parent_speed = coordinated.get(parent_namespace, 0.0)
                state = link_states.get(namespace)
                if state is None:
                    speed_limit = parent_speed
                elif state[0] == 'open':
                    speed_limit = parent_speed + 0.008
                elif state[0] == 'compressed':
                    compression = max(
                        0.0,
                        -state[2] - getattr(
                            self, 'transport_push_spacing_deadband', 0.003
                        ),
                    )
                    yield_speed = min(
                        maximum_yield, yield_gain * compression
                    )
                    speed_limit = max(
                        0.0, parent_speed - yield_speed
                    )
                    if parent_speed > minimum_yield_speed:
                        speed_limit = max(
                            minimum_yield_speed, speed_limit
                        )
                else:
                    speed_limit = parent_speed
                coordinated[namespace] = min(
                    raw_speeds[namespace], max(0.0, speed_limit)
                )

        for namespace, speed in coordinated.items():
            if namespace in commands:
                commands[namespace].linear.x = speed
        arbitration = {
            'raw_speeds': raw_speeds,
            'coordinated_speeds': coordinated,
            'link_states': link_states,
            'hard_stop_sources': (),
        }
        if record_telemetry:
            self._store_push_arbitration(arbitration)
        return arbitration

    def _transport_all_pusher_proof_speed(self):
        """Return the forward speed that proves a useful fleet command."""
        reference_speed = getattr(
            self, 'transport_push_reference_speed', None
        )
        try:
            reference_speed = float(reference_speed)
        except (TypeError, ValueError, OverflowError):
            reference_speed = None

        if (
            reference_speed is None
            or not math.isfinite(reference_speed)
            or reference_speed <= 0.0
        ):
            return 0.015

        # A loaded chain deliberately slows near the target.  Keep the
        # complete-fleet proof meaningful at that calm pace instead of
        # demanding nearly the whole reference from every wheel.
        return max(0.003, min(0.015, 0.50 * reference_speed))

    def _publish_grf_commands(
        self, namespaces, commands, positions, yaws,
        object_pos, object_yaw, expected_epoch
    ):
        # GRF planning is the expensive part of a large-fleet cycle.  Keep
        # every avoidance helper on the same pose snapshot used by the plan;
        # mixing a newer self pose with older neighbour poses can turn a real
        # 0.14 m loaded link into a false 0.13 m emergency at 3x simulation.
        self._sync_avoidance_snapshot(positions, yaws)
        with self.data_lock:
            robot_velocities = getattr(self, 'robot_velocities', {})
            velocities = {
                namespace: robot_velocities[namespace].copy()
                for namespace in namespaces
                if namespace in robot_velocities
            }
        targets = self._transport_targets(
            namespaces, positions, object_pos, object_yaw, velocities
        )
        (
            contact_neighbours,
            shielded_neighbours,
            _row_neighbours,
        ) = self._transport_neighbours(targets)
        payload_contacts = {
            namespace: self._payload_contact_near(
                positions[namespace], object_pos, object_yaw
            )
            for namespace, target in targets.items()
            if (
                target['role'] == 'payload_push'
                and namespace in positions
            )
        }
        payload_row_ready = (
            len(payload_contacts) <= 1
            or all(payload_contacts.values())
        )
        engaged = set(getattr(self, 'transport_engaged', set()))
        physical_engaged = set(getattr(
            self, 'transport_physical_engaged', engaged
        ))
        full_chain_connected = (
            bool(targets) and set(targets) <= physical_engaged
        )
        near_target_recovery = (
            getattr(self, 'transport_synchronized_push_started', False)
            and not full_chain_connected
            and getattr(self, 'transport_arrival_latched', False)
        )
        recovery_speed = getattr(
            self, 'transport_contact_closing_speed', 0.018
        )
        planned_commands = {}
        for ns in namespaces:
            target = targets.get(ns)
            position = positions.get(ns)
            yaw = yaws.get(ns)
            if (
                target is None
                or position is None
                or yaw is None
                or not math.isfinite(yaw)
            ):
                planned_commands[ns] = Twist()
                continue

            recovery_margin = (
                self.transport_payload_recovery_margin
                if target['role'] == 'payload_push'
                else 0.015
            )
            payload_contact_confirmed = payload_contacts.get(ns, False)
            contact_ready = payload_contact_confirmed or self._slot_contact_ready(
                position, yaw, object_pos, object_yaw, target,
                require_inward_heading=False,
                contact_margin=recovery_margin,
            )
            if contact_ready:
                velocity = self._goal_push_velocity(
                    commands.get(ns), object_pos,
                    target.get('chain_depth', 0), position, target,
                )
                allow_designated_contact = True
            else:
                velocity, allow_designated_contact = self._slot_velocity(
                    position, object_pos, target, pushing=True
                )
            if not velocity.is_finite():
                planned_commands[ns] = Twist()
                continue

            cmd = self._loaded_chain_command(
                velocity, yaw, target
            )
            cmd = self._keep_chain_push_pressure(
                ns, cmd, object_pos, yaw, target, contact_ready,
                allow_designated_contact
            )
            recovery_pace = (
                not full_chain_connected and ns in physical_engaged
            )
            hold_connected_prefix = (
                getattr(
                    self, 'transport_synchronized_push_started', False
                )
                and not full_chain_connected
                and ns in physical_engaged
            )
            recovery_limit = (
                0.0 if hold_connected_prefix
                else recovery_speed
            )
            terminal_speed_limit = None
            if near_target_recovery and ns not in physical_engaged:
                closing_speed = getattr(
                    self, 'transport_terminal_closing_speed', 0.010
                )
                if target['role'] == 'companion_push':
                    push_direction = np.asarray(
                        target['push_direction'], dtype=float
                    )
                    direction_norm = float(np.linalg.norm(push_direction))
                    if direction_norm > 1e-9:
                        push_direction /= direction_norm
                    parent_velocity = np.asarray(
                        target.get('parent_velocity', np.zeros(2)),
                        dtype=float,
                    )
                    parent_forward = 0.0
                    if (
                        push_direction.shape == (2,)
                        and parent_velocity.shape == (2,)
                        and np.all(np.isfinite(push_direction))
                        and np.all(np.isfinite(parent_velocity))
                    ):
                        parent_forward = max(0.0, float(np.dot(
                            parent_velocity, push_direction
                        )))
                    parent_position = np.asarray(
                        target.get('parent_position', np.zeros(2)),
                        dtype=float,
                    )
                    to_parent = parent_position - np.asarray(
                        position, dtype=float
                    )
                    parent_distance = float(np.linalg.norm(to_parent))
                    local_contact = False
                    if (
                        parent_position.shape == (2,)
                        and to_parent.shape == (2,)
                        and np.all(np.isfinite(to_parent))
                        and parent_distance > 1e-9
                        and direction_norm > 1e-9
                    ):
                        local_contact = (
                            parent_distance <= getattr(
                                self,
                                'transport_companion_contact_distance',
                                0.145,
                            )
                            and float(np.dot(
                                to_parent / parent_distance,
                                push_direction,
                            )) >= math.cos(getattr(
                                self,
                                'transport_companion_engagement_angle',
                                0.35,
                            ))
                        )
                    # A locally loaded descendant should translate with its
                    # open parent, not add another closing speed and compress
                    # the whole disconnected subtree like an accordion.
                    relative_closing = (
                        0.0
                        if local_contact
                        else closing_speed
                    )
                    terminal_speed_limit = (
                        parent_forward + relative_closing
                    )
                else:
                    terminal_speed_limit = closing_speed
                if cmd.linear.x > 0.0:
                    cmd.linear.x = min(
                        cmd.linear.x, terminal_speed_limit
                    )
            if (
                (not payload_row_ready or recovery_pace)
                and cmd.linear.x > 0.0
            ):
                # One lead must not drag the payload away from the other
                # column. The same calm pace lets an open rear subtree catch
                # the connected prefix instead of chasing a moving payload.
                cmd.linear.x = min(
                    cmd.linear.x, recovery_limit,
                )
            if hold_connected_prefix:
                cmd = Twist()
            allowed_chain_contacts = self._nearby_chain_contacts(
                ns, contact_neighbours, positions, targets
            )
            shielded_chain_robots = self._nearby_chain_contacts(
                ns, shielded_neighbours, positions, targets
            )
            parallel_lane_robots = set(self._parallel_lane_contacts(
                ns, positions, targets
            ))
            other_lidar_neighbours = set(parallel_lane_robots)
            other_lidar_neighbours.update(shielded_chain_robots)
            repulsion_exempt = set(parallel_lane_robots)
            repulsion_exempt.update(shielded_chain_robots)
            parent_contact_allowed = (
                target['role'] == 'companion_push'
                and target['parent_namespace'] in allowed_chain_contacts
                and allow_designated_contact
            )
            cmd = self._apply_transport_avoidance(
                ns, cmd, object_pos,
                # The payload is the deliberate contact object throughout
                # PUSH. Rear links can also reach it if a front lane opens,
                # which matches the preference for payload over companions.
                allow_payload_contact=True,
                # A designated lead may briefly lose the surface while the
                # other lane moves the box. Keep the modeled payload masked
                # during that recovery so its own corner return cannot stop
                # the robot that is trying to restore contact.
                payload_contact_confirmed=(
                    target['role'] == 'payload_push'
                ),
                allowed_contact_position=(
                    target['parent_position']
                    if parent_contact_allowed
                    else None
                ),
                allowed_contact_namespace=(
                    target['parent_namespace']
                    if parent_contact_allowed
                    else None
                ),
                allowed_contact_namespaces=allowed_chain_contacts,
                repulsion_exempt_namespaces=repulsion_exempt,
                parallel_motion_exempt_namespaces=(
                    parallel_lane_robots
                ),
                shielded_motion_exempt_namespaces=(
                    shielded_chain_robots
                ),
                soft_steering=(target['role'] == 'payload_push'),
                additional_lidar_masks=self._transport_robot_lidar_masks(
                    allowed_chain_contacts,
                    other_lidar_neighbours,
                    positions,
                    yaws,
                ),
                object_yaw=object_yaw,
                minimum_linear_speed=self._transport_motion_floor(
                    cmd, position, yaw, target, contact_ready
                ),
            )
            if (
                (not payload_row_ready or recovery_pace)
                and cmd.linear.x > 0.0
            ):
                # Avoidance smoothing remembers the previous command, so
                # enforce the fleet recovery cap on its final output too.
                cmd.linear.x = min(
                    cmd.linear.x, recovery_limit,
                )
            if terminal_speed_limit is not None and cmd.linear.x > 0.0:
                # Avoidance remains free to slow or stop. This repeats only the
                # upper bound so its smoother cannot restore excess closing.
                cmd.linear.x = min(cmd.linear.x, terminal_speed_limit)
            if hold_connected_prefix:
                # A true pose hold also clears remembered steering. The open
                # subtree keeps its closing command, while its connected
                # prefix cannot pull the gap wider or carry the payload on.
                with self.data_lock:
                    avoidance = getattr(
                        self, 'avoidance_modules', {}
                    ).get(ns)
                if avoidance is not None:
                    avoidance.reset_motion()
                cmd = Twist()
            cmd = self._stabilize_push_steering(cmd)
            planned_commands[ns] = cmd

        push_arbitration = self._empty_push_arbitration()
        if full_chain_connected and planned_commands:
            push_arbitration = self._coordinate_loaded_chain_commands(
                planned_commands,
                targets,
                positions,
                record_telemetry=False,
            )
            for namespace, command in planned_commands.items():
                if namespace not in targets:
                    continue
                angular_limit = getattr(
                    self, 'transport_push_angular_limit', 0.25
                )
                command.angular.z = max(
                    -angular_limit,
                    min(angular_limit, float(command.angular.z)),
                )

        # Computing avoidance for ten robots is considerably more expensive
        # than publishing ten Twists.  Do all safety work against the same
        # pose snapshot, then release the rear rows first in one tight batch.
        # No payload lead can begin a new ramp step while a tail still has its
        # command from the previous cycle.
        publish_order = sorted(
            namespaces,
            key=lambda namespace: (
                -int(targets.get(namespace, {}).get('chain_depth', -1)),
                int(targets.get(namespace, {}).get('chain_index', 0)),
                namespace,
            ),
        )
        get_time = getattr(rospy, 'get_time', lambda: 0.0)
        batch_started_at = float(get_time())
        published_all = True
        for namespace in publish_order:
            command = planned_commands.get(namespace, Twist())
            # Keep the safety smoother commit in the same epoch as its Twist.
            # Otherwise a stop can clear the smoother between these operations
            # and an old control pass can immediately restore stale motion.
            with self.command_lock:
                if not self._publish_command(
                    namespace, command, expected_epoch
                ):
                    published_all = False
                    break
                if not self._command_allowed(expected_epoch):
                    published_all = False
                    break
                with self.data_lock:
                    avoidance = getattr(
                        self, 'avoidance_modules', {}
                    ).get(namespace)
                commit_command = getattr(
                    avoidance, 'commit_published_command', None
                )
                if callable(commit_command):
                    commit_command(command)
        batch_finished_at = float(get_time())
        if not published_all:
            return False

        # A pause, stop, new task, or fleet change may land after the final
        # Twist.  Commit batch bookkeeping only if this cycle still owns the
        # command epoch; otherwise an old pass could restart a cleared ramp.
        with self.command_lock:
            if not self._command_allowed(expected_epoch):
                return False
            self._store_push_arbitration(push_arbitration)
            self.transport_control_sequence = getattr(
                self, 'transport_control_sequence', 0
            ) + 1
            self.transport_last_batch_publish_span = max(
                0.0, batch_finished_at - batch_started_at
            )
            self.transport_last_control_sim_time = batch_finished_at
            self.transport_last_control_commands = {
                namespace: (
                    float(command.linear.x),
                    float(command.angular.z),
                )
                for namespace, command in planned_commands.items()
                if namespace in targets
            }
            reference_speed = getattr(
                self, 'transport_push_reference_speed', None
            )
            try:
                reference_speed = float(reference_speed)
            except (TypeError, ValueError, OverflowError):
                reference_speed = None
            minimum_speed = self._transport_all_pusher_proof_speed()
            current_pushers = {
                namespace
                for namespace, command in planned_commands.items()
                if (
                    namespace in physical_engaged
                    and float(command.linear.x) >= minimum_speed
                )
            }
            self.transport_current_useful_pushers = current_pushers
            all_useful_now = (
                full_chain_connected
                and set(targets) <= current_pushers
            )
            if full_chain_connected:
                contributors = set(getattr(
                    self, 'transport_useful_contributors', set()
                ))
                contributors.update(current_pushers)
                self.transport_useful_contributors = contributors
            useful_since = getattr(
                self, 'transport_all_pushers_since', None
            )
            hold_time = getattr(
                self, 'transport_all_push_hold_time', 0.75
            )
            all_pushers_confirmed = bool(getattr(
                self, 'transport_all_pushers_confirmed', False
            ))
            if (
                full_chain_connected
                and useful_since is not None
                and batch_started_at >= useful_since
                and batch_started_at - useful_since >= hold_time
            ):
                # Until this batch began, every robot still held the previous
                # coherent command. Credit that interval even if this newly
                # planned batch contains one weak command.
                all_pushers_confirmed = True
            if all_useful_now:
                if (
                    useful_since is None
                    or batch_finished_at < useful_since
                ):
                    useful_since = batch_finished_at
                    self.transport_all_pushers_since = useful_since
                all_pushers_confirmed = (
                    all_pushers_confirmed
                    or batch_finished_at - useful_since >= hold_time
                )
            else:
                self.transport_all_pushers_since = None
            self.transport_all_pushers_confirmed = all_pushers_confirmed

            tracking_ready = full_chain_connected
            if reference_speed is not None and tracking_ready:
                tracking_speed = max(
                    min(0.015, 0.75 * float(reference_speed)),
                    0.60 * float(reference_speed),
                )
                tracking_ready = all(
                    float(planned_commands[namespace].linear.x)
                    >= tracking_speed
                    for namespace in targets
                )
                if tracking_ready and set(targets) <= set(velocities):
                    goal_direction, _, _ = self._transport_frame(
                        object_pos
                    )
                    minimum_motion = 0.70 * float(reference_speed)
                    tracking_ready = all(
                        float(np.dot(
                            velocities[namespace], goal_direction
                        )) >= minimum_motion
                        for namespace in targets
                    )
            self._advance_push_reference(
                targets, full_chain_connected, tracking_ready
            )
            self.transport_synchronized_push_started = True
        return True

    def _stabilize_push_steering(self, command):
        """Keep a loaded push row from weaving itself out of contact."""
        if abs(float(command.linear.x)) < getattr(
            self, 'transport_min_useful_push_speed', 0.035
        ):
            # A stopped or misaligned robot must be able to turn back into
            # the column. Capping that recovery turn makes it trace a wide
            # arc and pull the opposite lane inward.
            return command

        angular_limit = getattr(
            self, 'transport_push_angular_limit', 0.25
        )
        command.angular.z = max(
            -angular_limit,
            min(angular_limit, float(command.angular.z)),
        )
        return command

    def _transport_motion_floor(
        self, command, position, yaw, target, contact_ready
    ):
        """Keep one safe, aligned link from falling behind its parent."""
        requested_speed = max(0.0, float(command.linear.x))
        useful_speed = 0.015
        if (
            not getattr(
                self, 'transport_synchronized_push_started', False
            )
            or requested_speed < useful_speed
        ):
            return 0.0

        push_direction = np.asarray(
            target.get('push_direction', np.zeros(2)), dtype=float
        )
        direction_norm = float(np.linalg.norm(push_direction))
        if direction_norm <= 1e-9 or not np.all(np.isfinite(push_direction)):
            return 0.0
        push_direction /= direction_norm
        desired_yaw = math.atan2(
            float(push_direction[1]), float(push_direction[0])
        )
        if abs(self._normalize_angle(desired_yaw - float(yaw))) > getattr(
            self, 'transport_launch_heading_tolerance', 0.08
        ):
            return 0.0

        if target.get('role') == 'payload_push':
            locally_safe = bool(contact_ready)
        else:
            parent_position = np.asarray(
                target.get('parent_position', np.zeros(2)), dtype=float
            )
            child_position = np.asarray(position, dtype=float)
            to_parent = parent_position - child_position
            parent_distance = float(np.linalg.norm(to_parent))
            locally_safe = False
            if (
                parent_position.shape == (2,)
                and child_position.shape == (2,)
                and np.all(np.isfinite(parent_position))
                and np.all(np.isfinite(child_position))
                and parent_distance > 1e-9
            ):
                parent_direction = to_parent / parent_distance
                locally_safe = (
                    float(np.dot(to_parent, push_direction)) > 0.05
                    and float(np.dot(parent_direction, push_direction))
                    >= math.cos(
                        getattr(
                            self,
                            'transport_companion_engagement_angle',
                            0.20,
                        )
                        + getattr(
                            self,
                            'transport_engagement_release_angle_margin',
                            0.15,
                        )
                    )
                    and parent_distance <= (
                        getattr(
                            self,
                            'transport_companion_contact_distance',
                            0.145,
                        )
                        + 0.03
                    )
                )
        if not locally_safe:
            return 0.0

        reference_speed = getattr(
            self, 'transport_push_reference_speed', None
        )
        try:
            reference_speed = float(reference_speed)
        except (TypeError, ValueError):
            reference_speed = useful_speed
        if not math.isfinite(reference_speed):
            reference_speed = useful_speed
        return min(
            requested_speed,
            max(useful_speed, reference_speed),
        )

    def _keep_chain_push_pressure(
        self, namespace, command, object_pos, yaw, target,
        contact_ready, contact_allowed
    ):
        """Keep a direct payload lead loaded while it faces the goal."""
        if not contact_ready or not contact_allowed:
            return command
        if target.get('role') != 'payload_push':
            # Companion pressure is already part of the parent-following
            # velocity controller. Reapplying a minimum here would override
            # its command to slow down and decompress a short link.
            return command
        if not math.isfinite(float(yaw)):
            return command

        goal, _, _ = self._transport_frame(object_pos)
        heading = np.array([math.cos(yaw), math.sin(yaw)])
        minimum_alignment = math.cos(getattr(
            self, 'transport_push_heading_cone', 0.18
        ))
        if float(np.dot(heading, goal)) < minimum_alignment:
            return command

        floor = getattr(self, 'transport_min_useful_push_speed', 0.035)
        if target.get('role') == 'payload_push':
            floor = max(floor, min(self.vmax, floor + 0.010))
        floor = self._ramped_push_speed(floor)
        if float(command.linear.x) >= floor:
            return command

        held = Twist()
        held.linear.x = min(self.vmax, floor)
        held.angular.z = float(command.angular.z)
        return held

    def _object_lidar_mask(
        self, object_pos, contact_confirmed=False, object_yaw=None,
        closer_tolerance=None,
    ):
        """Describe the payload surface that may be ignored beam by beam."""
        if object_yaw is None:
            object_yaw = getattr(self, 'object_yaw', 0.0)
        if closer_tolerance is None:
            closer_tolerance = (
                self.object_lidar_contact_closer_tolerance
                if contact_confirmed
                else self.object_lidar_closer_tolerance
            )
        return LidarRangeMask(
            center_x=float(object_pos[0]),
            center_y=float(object_pos[1]),
            radius=self.grf_object_radius,
            maximum_center_distance=self.object_avoidance_range,
            tolerance=self.object_lidar_tolerance,
            closer_tolerance=max(0.0, float(closer_tolerance)),
            half_width=self.object_half_width,
            half_height=self.object_half_height,
            yaw=float(object_yaw),
            occludes_behind=True,
            accept_sensor_floor=True,
        )

    def _object_lidar_corner_masks(
        self, object_pos, contact_confirmed=False, object_yaw=None
    ):
        """Cover corner returns while exact pose updates catch up at 3x."""
        if object_yaw is None:
            object_yaw = getattr(self, 'object_yaw', 0.0)
        yaw = float(object_yaw)
        cosine = math.cos(yaw)
        sine = math.sin(yaw)
        radius = getattr(self, 'object_lidar_corner_radius', 0.08)
        tolerance = getattr(
            self, 'object_lidar_envelope_tolerance', 0.10
        )
        closer_tolerance = getattr(
            self, 'object_lidar_envelope_closer_tolerance', 0.02
        )
        if contact_confirmed:
            closer_tolerance = max(
                closer_tolerance,
                getattr(
                    self,
                    'object_lidar_contact_closer_tolerance',
                    0.05,
                ),
            )
        masks = []
        for local_x in (-self.object_half_width, self.object_half_width):
            for local_y in (
                -self.object_half_height, self.object_half_height
            ):
                masks.append(LidarRangeMask(
                    center_x=(
                        float(object_pos[0])
                        + local_x * cosine - local_y * sine
                    ),
                    center_y=(
                        float(object_pos[1])
                        + local_x * sine + local_y * cosine
                    ),
                    radius=radius,
                    maximum_center_distance=self.object_avoidance_range,
                    tolerance=tolerance,
                    closer_tolerance=closer_tolerance,
                    occludes_behind=True,
                    accept_sensor_floor=True,
                ))
        return masks

    def _contact_distance_at(self, robot_pos, object_pos, object_yaw):
        """Return contact-ready centre distance toward a rotated box face."""
        offset_x = float(robot_pos[0] - object_pos[0])
        offset_y = float(robot_pos[1] - object_pos[1])
        distance = math.hypot(offset_x, offset_y)
        if distance <= 1e-9:
            return (
                self.robot_forward_contact_extent
                + self.transport_contact_slop
            )

        cos_yaw = math.cos(object_yaw)
        sin_yaw = math.sin(object_yaw)
        unit_x = offset_x / distance
        unit_y = offset_y / distance
        local_x = unit_x * cos_yaw + unit_y * sin_yaw
        local_y = -unit_x * sin_yaw + unit_y * cos_yaw

        candidates = []
        if abs(local_x) > 1e-9:
            candidates.append(self.object_half_width / abs(local_x))
        if abs(local_y) > 1e-9:
            candidates.append(self.object_half_height / abs(local_y))
        surface_distance = min(candidates) if candidates else 0.0
        return (
            surface_distance
            + self.robot_forward_contact_extent
            + self.transport_contact_slop
        )

    def _apply_transport_avoidance(
        self,
        namespace,
        cmd,
        object_pos,
        allow_payload_contact=True,
        payload_contact_confirmed=False,
        allowed_contact_position=None,
        allowed_contact_namespace=None,
        allowed_contact_namespaces=(),
        repulsion_exempt_namespaces=(),
        parallel_motion_exempt_namespaces=(),
        shielded_motion_exempt_namespaces=(),
        additional_lidar_masks=(),
        soft_steering=True,
        object_yaw=None,
        minimum_linear_speed=0.0,
        payload_docking=False,
    ):
        """Keep normal safety active while allowing deliberate contact."""

        with self.data_lock:
            avoidance = self.avoidance_modules.get(namespace)
        if avoidance is None:
            return cmd
        lidar_masks = list(additional_lidar_masks)
        if allow_payload_contact:
            docking_mask_active = (
                bool(payload_docking)
                and not payload_contact_confirmed
                and float(cmd.linear.x) > 0.0
            )
            closer_tolerance = None
            if docking_mask_active:
                closer_tolerance = min(
                    0.04,
                    max(
                        self.object_lidar_closer_tolerance,
                        getattr(
                            self,
                            'object_lidar_docking_closer_tolerance',
                            0.035,
                        ),
                    ),
                )
            lidar_masks.append(self._object_lidar_mask(
                object_pos,
                payload_contact_confirmed,
                object_yaw,
                closer_tolerance=closer_tolerance,
            ))
            lidar_masks.extend(
                self._object_lidar_corner_masks(
                    object_pos, payload_contact_confirmed, object_yaw
                )
            )
        return avoidance.apply_avoidance(
            cmd,
            lidar_range_mask=tuple(lidar_masks),
            allowed_contact_position=allowed_contact_position,
            allowed_contact_namespace=allowed_contact_namespace,
            allowed_contact_namespaces=allowed_contact_namespaces,
            repulsion_exempt_namespaces=repulsion_exempt_namespaces,
            parallel_motion_exempt_namespaces=(
                parallel_motion_exempt_namespaces
            ),
            shielded_motion_exempt_namespaces=(
                shielded_motion_exempt_namespaces
            ),
            soft_steering=soft_steering,
            minimum_linear_speed=minimum_linear_speed,
            collision_context=self._transport_collision_context(),
        )

    def _legacy_push_step(
        self,
        namespaces,
        positions,
        yaws,
        object_pos,
        object_yaw,
        expected_epoch,
        fallback=False,
    ):
        with self.command_lock:
            if not self._command_allowed(expected_epoch):
                return
            self._active_planner = (
                'legacy_fallback' if fallback else 'legacy'
            )
            self._active_grf_iterations = 0
        commands = {}
        for ns in namespaces:
            if ns not in yaws:
                commands[ns] = None
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
                commands[ns] = None
                continue
            if (
                not np.all(np.isfinite(sampled_vel))
                or not math.isfinite(yaws[ns])
            ):
                commands[ns] = None
                continue

            commands[ns] = Vec2(
                float(sampled_vel[0]),
                float(sampled_vel[1]),
            )

        self._publish_grf_commands(
            namespaces,
            commands,
            positions,
            yaws,
            object_pos,
            object_yaw,
            expected_epoch,
        )

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

    def _build_search_routes(self, namespaces, positions):
        """Give each robot a share of a smooth lawnmower coverage path."""
        ordered = sort_robot_ids(namespaces)
        if not ordered:
            self._reset_search_routes()
            return

        half_extent = max(
            0.50,
            self.arena_size / 2.0
            - self.arena_margin
            - self.robot_radius,
        )
        usable_width = half_extent * 2.0
        maximum_lane_spacing = max(
            0.50,
            2.0 * max(0.10, self.sensing_range)
            * self.search_lane_overlap,
        )
        coverage_lanes = max(
            1,
            int(math.ceil(usable_width / maximum_lane_spacing)) + 1,
        )
        lane_count = max(len(ordered), coverage_lanes)

        if lane_count == 1:
            lane_y_values = [0.0]
        else:
            lane_y_values = [
                -half_extent
                + usable_width * lane_index / float(lane_count - 1)
                for lane_index in range(lane_count)
            ]

        routes = {}
        route_indices = {}
        route_directions = {}
        for robot_index, namespace in enumerate(ordered):
            assigned_lanes = range(robot_index, lane_count, len(ordered))
            route = []
            for route_lane_index, lane_index in enumerate(assigned_lanes):
                y = lane_y_values[lane_index]
                left = np.array([-half_extent, y], dtype=float)
                right = np.array([half_extent, y], dtype=float)
                if route_lane_index % 2 == 0:
                    route.extend((left, right))
                else:
                    route.extend((right, left))

            if not route:
                route = [np.array([0.0, 0.0], dtype=float)]

            position = positions.get(namespace)
            if position is None:
                initial_index = 0
            else:
                initial_index = min(
                    range(len(route)),
                    key=lambda index: float(
                        np.linalg.norm(position - route[index])
                    ),
                )

            routes[namespace] = route
            route_indices[namespace] = initial_index
            route_directions[namespace] = (
                -1 if initial_index == len(route) - 1 else 1
            )

        self.search_routes = routes
        self.search_route_indices = route_indices
        self.search_route_directions = route_directions
        self.search_route_signature = tuple(ordered)
        self.search_navigation_routes = {}
        self.search_navigation_indices = {}
        self.search_navigation_destinations = {}
        self.search_navigation_targets = {}

    def _search_target(self, namespace, position):
        """Return the current coverage target and advance it when reached."""
        route = self.search_routes.get(namespace, ())
        if not route:
            return None

        index = self.search_route_indices.get(namespace, 0)
        index = max(0, min(index, len(route) - 1))
        target = route[index]
        if (
            len(route) > 1
            and float(np.linalg.norm(position - target))
            <= self.search_waypoint_tolerance
        ):
            direction = self.search_route_directions.get(namespace, 1)
            next_index = index + direction
            if next_index < 0 or next_index >= len(route):
                direction *= -1
                next_index = index + direction
            index = max(0, min(next_index, len(route) - 1))
            self.search_route_indices[namespace] = index
            self.search_route_directions[namespace] = direction
            target = route[index]

        return target.copy()

    def _plan_search_segment(self, start, destination):
        """Plan one coverage leg around the arena's known static obstacles."""
        route_zones = [
            dict(zone)
            for zone in getattr(self, 'spawn_exclusion_zones', ())
            if isinstance(zone, dict)
            and zone.get('model') != getattr(
                self, 'object_name', 'transport_object'
            )
        ]
        with self.model_lock:
            model_poses = dict(getattr(self, 'model_poses', {}))

        try:
            route = plan_obstacle_aware_route(
                (float(start[0]), float(start[1])),
                (float(destination[0]), float(destination[1])),
                getattr(self, 'arena_size', 10.0),
                getattr(self, 'arena_margin', 0.35),
                getattr(self, 'transport_route_obstacle_clearance', 0.30),
                route_zones,
                getattr(self, 'arena_profile', 'swarm_arena'),
                model_poses,
                circle_samples=8,
            )
        except (TypeError, ValueError) as exc:
            rospy.logwarn_throttle(
                5.0,
                '[transport] search route planner rejected a segment: {}'.format(
                    exc
                ),
            )
            route = None
        if route is None:
            rospy.logwarn_throttle(
                5.0,
                '[transport] no safe search route is available; holding',
            )
            return []
        waypoints = [
            np.asarray(point, dtype=float) for point in route[1:]
        ]
        return waypoints or [destination.copy()]

    def _search_navigation_target(self, namespace, position, destination):
        """Return the next obstacle-aware waypoint on a coverage leg."""
        cached_destination = self.search_navigation_destinations.get(namespace)
        destination_changed = (
            cached_destination is None
            or float(np.linalg.norm(cached_destination - destination)) > 1e-6
        )
        cached_route = self.search_navigation_routes.get(namespace)
        if destination_changed or not cached_route:
            self.search_navigation_routes[namespace] = (
                self._plan_search_segment(position, destination)
            )
            self.search_navigation_indices[namespace] = 0
            self.search_navigation_destinations[namespace] = destination.copy()

        route = self.search_navigation_routes.get(namespace, ())
        if not route:
            self.search_navigation_targets.pop(namespace, None)
            return None

        index = self.search_navigation_indices.get(namespace, 0)
        index = max(0, min(index, len(route) - 1))
        intermediate_tolerance = min(
            0.12, self.search_waypoint_tolerance
        )
        while (
            index < len(route) - 1
            and float(np.linalg.norm(position - route[index]))
            <= intermediate_tolerance
        ):
            index += 1
        self.search_navigation_indices[namespace] = index
        target = route[index].copy()
        self.search_navigation_targets[namespace] = target.copy()
        return target

    def _payload_is_visible_from(self, robot_position, object_position):
        """Check whether a known static obstacle blocks the range sighting."""
        try:
            start = (
                float(robot_position[0]), float(robot_position[1])
            )
            end = (
                float(object_position[0]), float(object_position[1])
            )
        except (IndexError, TypeError, ValueError, OverflowError):
            return False

        if not all(math.isfinite(value) for value in start + end):
            return False

        # Gazebo supplies the object's global pose, so this is an
        # omnidirectional range detector rather than a camera model.  Use the
        # physical obstacle outlines for occlusion; navigation clearances and
        # spawn padding should not make a wall look wider than it is.
        zones = []
        for configured in self._transport_static_zones():
            zone = dict(configured)
            zone.pop('clearance', None)
            zone.pop('padding', None)
            zones.append(zone)

        model_lock = getattr(self, 'model_lock', None)
        if model_lock is None:
            model_poses = dict(getattr(self, 'model_poses', {}))
        else:
            with model_lock:
                model_poses = dict(getattr(self, 'model_poses', {}))

        try:
            # A tiny positive clearance distinguishes a segment crossing an
            # obstacle from one that merely has zero configured padding.
            return straight_route_is_safe(
                start,
                end,
                1e-4,
                zones,
                getattr(self, 'arena_profile', 'swarm_arena'),
                model_poses,
            )
        except (TypeError, ValueError, OverflowError) as exc:
            rospy.logwarn_throttle(
                5.0,
                '[transport] payload visibility check failed: {}'.format(exc),
            )
            return False

    def _announce_payload_found(
        self, finder, distance, object_position, expected_epoch
    ):
        """Broadcast the first valid sighting and wake the whole fleet."""
        with self.command_lock:
            if not self._command_allowed(expected_epoch):
                return False
            with self.phase_lock:
                if self.phase != TransportPhase.SEARCH:
                    return False
            if getattr(self, 'transport_discovery', None) is not None:
                return False

            get_time = getattr(rospy, 'get_time', lambda: 0.0)
            sim_time = float(get_time())
            if not math.isfinite(sim_time):
                sim_time = 0.0

            with self.data_lock:
                roster = sort_robot_ids(self.robot_namespaces)
            task_id = self.current_task_id
            notice = {
                'event': 'payload_found',
                'event_id': '{}:payload-found'.format(task_id or 'unknown'),
                'task_id': task_id,
                'announced': True,
                'finder': finder,
                'distance': round(float(distance), 3),
                'object_position': {
                    'x': round(float(object_position[0]), 3),
                    'y': round(float(object_position[1]), 3),
                },
                'sim_time': round(sim_time, 3),
                'notified_robots': [
                    namespace for namespace in roster
                    if namespace != finder
                ],
            }
            self.transport_discovery = notice
            with self.phase_lock:
                self.phase = TransportPhase.APPROACH

            publisher = getattr(self, 'discovery_pub', None)
            if publisher is not None:
                publisher.publish(String(data=json.dumps(notice)))

        rospy.loginfo(
            "[transport] %s found the payload at %.2fm; "
            "notified %d teammates  >>> phase APPROACH",
            finder,
            distance,
            len(notice['notified_robots']),
        )
        return True

    def _search_phase(self, expected_epoch):
        """
        SEARCH: every robot covers a separate lane until one sees the payload.

        The first sighting is published once on /transport/discovery.  The
        controller then changes the whole fleet to APPROACH on the same
        control cycle, which is the simulated equivalent of a finder telling
        its teammates where to rendezvous.
        """
        with self.model_lock:
            obj_pos = self.object_position.copy() if self.object_found else None
            object_error = self.object_error

        if obj_pos is None and object_error:
            self._fail_transport(object_error, expected_epoch)
            return

        with self.data_lock:
            namespaces = list(self.robot_namespaces)
            positions = {ns: self.robot_positions[ns].copy()
                         for ns in namespaces if ns in self.robot_positions}
            yaws = {ns: self.robot_yaws[ns]
                    for ns in namespaces if ns in self.robot_yaws}
            odom_received_at = dict(getattr(
                self, 'robot_odom_received_at', {}
            ))
            avoidance_modules = {
                ns: self.avoidance_modules.get(ns) for ns in namespaces
            }

        odometry_error = self._transport_odometry_error(
            namespaces,
            positions,
            yaws,
            odom_received_at,
        )
        if odometry_error is not None:
            self._fail_transport(odometry_error, expected_epoch)
            return

        # If several robots see it on the same tick, use the closest robot and
        # the natural namespace order as a stable tie-breaker.
        if obj_pos is not None:
            robot_order = {
                namespace: index for index, namespace
                in enumerate(sort_robot_ids(namespaces))
            }
            sightings = []
            for ns in namespaces:
                if ns not in positions:
                    continue
                d = float(np.linalg.norm(positions[ns] - obj_pos))
                if (
                    d <= self.sensing_range
                    and self._payload_is_visible_from(positions[ns], obj_pos)
                ):
                    sightings.append((d, robot_order.get(ns, 0), ns))
            if sightings:
                distance, _, finder = min(sightings)
                self._announce_payload_found(
                    finder, distance, obj_pos, expected_epoch
                )
                return

        signature = tuple(sort_robot_ids(namespaces))
        if (
            signature != getattr(self, 'search_route_signature', ())
            or any(ns not in self.search_routes for ns in namespaces)
        ):
            self._build_search_routes(namespaces, positions)

        # Persistent targets make broad, natural arcs.  The shared avoidance
        # layer still owns the final collision and acceleration constraints.
        for ns in namespaces:
            if ns not in positions or ns not in yaws:
                continue

            coverage_target = self._search_target(ns, positions[ns])
            if coverage_target is None:
                continue
            target = self._search_navigation_target(
                ns, positions[ns], coverage_target
            )
            if target is None:
                # Never leave the previous velocity active after planning
                # fails.  The empty cached route is retried on the next tick.
                self._publish_command(ns, Twist(), expected_epoch)
                continue
            offset = target - positions[ns]
            distance = float(np.linalg.norm(offset))
            desired_heading = math.atan2(offset[1], offset[0])
            heading_error = self._normalize_angle(
                desired_heading - yaws[ns]
            )

            alignment = max(0.0, math.cos(heading_error))
            distance_scale = min(
                1.0,
                max(
                    0.25,
                    distance / max(0.01, self.search_waypoint_tolerance * 2.0),
                ),
            )
            cmd = Twist()
            cmd.linear.x = (
                self.search_speed
                * (0.10 + 0.90 * alignment)
                * distance_scale
            )
            cmd.angular.z = max(
                -self.search_max_angular_speed,
                min(
                    self.search_max_angular_speed,
                    self.search_angular_gain * heading_error,
                ),
            )

            avoidance = avoidance_modules.get(ns)
            if avoidance is not None:
                requested_angular = cmd.angular.z
                cmd = avoidance.apply_avoidance(
                    cmd,
                    collision_context=self._transport_collision_context(),
                )
                if (
                    abs(cmd.linear.x) < 1e-4
                    and abs(cmd.angular.z) < 1e-4
                ):
                    # A robot stopped nose-first at a wall still has a safe
                    # escape: Burger's circular footprint can turn in place.
                    # Retry without forward motion so it does not sit there
                    # for the rest of the coverage pass.
                    recovery = Twist()
                    if abs(requested_angular) >= 0.15:
                        recovery.angular.z = requested_angular
                    else:
                        recovery.angular.z = (
                            0.45
                            if sum(ord(char) for char in ns) % 2 == 0
                            else -0.45
                        )
                    cmd = avoidance.apply_avoidance(
                        recovery,
                        collision_context=(
                            self._transport_collision_context()
                        ),
                    )

            self._publish_command(ns, cmd, expected_epoch)

    def _rendezvous_pose_ready(self, position, yaw, target):
        """Check one robot's expanded, off-payload rendezvous gate."""
        destination = np.asarray(
            target.get('staging_position', target.get('position')),
            dtype=float,
        )
        push_direction = np.asarray(
            target.get('push_direction', np.empty(0)), dtype=float
        )
        if (
            destination.shape != (2,)
            or push_direction.shape != (2,)
            or not np.all(np.isfinite(destination))
            or not np.all(np.isfinite(push_direction))
            or not math.isfinite(float(yaw))
        ):
            return False
        direction_norm = float(np.linalg.norm(push_direction))
        if direction_norm <= 1e-9:
            return False
        tolerance = getattr(
            self, 'transport_chain_staging_release_tolerance', 0.06
        )
        if float(np.linalg.norm(
            destination - np.asarray(position, dtype=float)
        )) > tolerance:
            return False
        desired_yaw = math.atan2(
            float(push_direction[1]), float(push_direction[0])
        )
        return abs(self._normalize_angle(
            desired_yaw - float(yaw)
        )) <= getattr(
            self, 'transport_assembly_heading_tolerance', 0.10
        )

    @staticmethod
    def _compression_target(target, progress):
        """Move into the lane first, then shorten every row together."""
        start = np.asarray(
            target.get('staging_position', target.get('position')),
            dtype=float,
        )
        finish = start
        if target.get('role') == 'companion_push':
            finish = np.asarray(
                target.get('assembly_position', target.get('position')),
                dtype=float,
            )
        progress = max(0.0, min(1.0, float(progress)))
        push_direction = np.asarray(
            target.get('push_direction', np.empty(0)), dtype=float
        )
        direction_norm = float(np.linalg.norm(push_direction))
        if (
            push_direction.shape != (2,)
            or not np.all(np.isfinite(push_direction))
            or direction_norm <= 1e-9
        ):
            return start + (finish - start) * progress
        push_direction /= direction_norm
        difference = finish - start
        longitudinal = push_direction * float(np.dot(
            difference, push_direction
        ))
        lateral = difference - longitudinal
        if progress < 0.5:
            return start + lateral * (progress * 2.0)
        return (
            start
            + lateral
            + longitudinal * ((progress - 0.5) * 2.0)
        )

    def _advance_transport_compression(
        self, companions, positions, targets, yaws=None
    ):
        """Move both push lanes through one synchronized accordion motion."""
        if not companions:
            self.transport_compression_progress = 1.0
            return 1.0

        progress = float(getattr(
            self, 'transport_compression_progress', 0.0
        ))
        if not math.isfinite(progress):
            progress = 0.0
        progress = max(0.0, min(1.0, progress))

        paths = []
        tracking_errors = []
        for namespace in companions:
            target = targets.get(namespace)
            position = positions.get(namespace)
            if target is None or position is None:
                continue
            start = np.asarray(
                target.get('staging_position', target.get('position')),
                dtype=float,
            )
            finish = start
            if target.get('role') == 'companion_push':
                finish = np.asarray(
                    target.get('assembly_position', target.get('position')),
                    dtype=float,
                )
            path = finish - start
            push_direction = np.asarray(
                target.get('push_direction', np.empty(0)), dtype=float
            )
            direction_norm = float(np.linalg.norm(push_direction))
            if (
                push_direction.shape == (2,)
                and np.all(np.isfinite(push_direction))
                and direction_norm > 1e-9
            ):
                push_direction /= direction_norm
                longitudinal_length = abs(float(np.dot(
                    path, push_direction
                )))
                lateral_length = float(np.linalg.norm(
                    path - push_direction * float(np.dot(
                        path, push_direction
                    ))
                ))
                paths.append(
                    2.0 * max(longitudinal_length, lateral_length)
                )
            else:
                paths.append(float(np.linalg.norm(path)))
            tracking_errors.append(float(np.linalg.norm(
                self._compression_target(target, progress) - position
            )))

        if len(paths) != len(companions):
            return progress
        maximum_path = max(paths, default=0.0)
        if maximum_path <= 1e-9:
            self.transport_compression_progress = 1.0
            return 1.0

        get_time = getattr(rospy, 'get_time', lambda: 0.0)
        now = float(get_time())
        previous = getattr(
            self, 'transport_compression_updated_at', None
        )
        self.transport_compression_updated_at = now
        if (
            previous is None
            or not math.isfinite(now)
            or not math.isfinite(float(previous))
            or now < float(previous)
        ):
            return progress

        tracking_tolerance = getattr(
            self,
            'transport_compression_tracking_tolerance',
            DEFAULT_COMPRESSION_TRACKING_TOLERANCE,
        )
        heading_ready = True
        if progress >= 0.5 and yaws is not None:
            heading_tolerance = getattr(
                self, 'transport_assembly_heading_tolerance', 0.10
            )
            for namespace in companions:
                target = targets.get(namespace)
                yaw = yaws.get(namespace)
                if target is None or yaw is None or not math.isfinite(yaw):
                    heading_ready = False
                    break
                push_direction = np.asarray(
                    target.get('push_direction', np.empty(0)), dtype=float
                )
                if (
                    push_direction.shape != (2,)
                    or not np.all(np.isfinite(push_direction))
                    or float(np.linalg.norm(push_direction)) <= 1e-9
                ):
                    heading_ready = False
                    break
                desired_yaw = math.atan2(
                    float(push_direction[1]), float(push_direction[0])
                )
                if abs(self._normalize_angle(
                    desired_yaw - float(yaw)
                )) > heading_tolerance:
                    heading_ready = False
                    break

        if (
            max(tracking_errors, default=0.0) <= tracking_tolerance
            and heading_ready
        ):
            elapsed = min(0.25, max(0.0, now - float(previous)))
            speed = getattr(
                self, 'transport_compression_speed', 0.12
            )
            progress += elapsed * max(0.0, speed) / maximum_path
            # Stop exactly at the end of the lateral move.  The next tick
            # turns every Burger back toward the common push direction before
            # any row starts closing; otherwise a reversed tail can meet its
            # parent rear-to-rear and physically lock the final approach.
            if self.transport_compression_progress < 0.5 < progress:
                progress = 0.5
            progress = min(1.0, progress)
            self.transport_compression_progress = progress
        return progress

    def _rendezvous_pair_key(self, first_namespace, second_namespace):
        ordered = sort_robot_ids((first_namespace, second_namespace))
        return tuple(ordered) if len(ordered) == 2 else ()

    def _rendezvous_recovery_destination(self, namespace, target):
        """Use the same final point as the cached rendezvous route."""
        state = getattr(
            self, 'transport_assembly_route_states', {}
        ).get(namespace, {})
        cached_target = state.get('target') if isinstance(state, dict) else None
        if (
            isinstance(state, dict)
            and state.get('kind') == 'rendezvous'
            and cached_target is not None
        ):
            destination = np.asarray(cached_target, dtype=float)
            if destination.shape == (2,) and np.all(np.isfinite(destination)):
                return destination.copy()
        return np.asarray(
            target.get('staging_position', target.get('position')),
            dtype=float,
        )

    def _remaining_rendezvous_recovery_route(
        self, namespace, position, recovery_state=None
    ):
        """Return the untravelled rendezvous polyline without advancing it."""
        position = np.asarray(position, dtype=float)
        if recovery_state is None:
            recovery_state = getattr(
                self, 'transport_rendezvous_recoveries', {}
            ).get(namespace, {})
        destination = np.asarray(
            recovery_state.get('destination', np.empty(0)), dtype=float
        )
        fallback = [position]
        if destination.shape == (2,) and np.all(np.isfinite(destination)):
            fallback.append(destination)

        route_state = getattr(
            self, 'transport_assembly_route_states', {}
        ).get(namespace, {})
        if not isinstance(route_state, dict):
            return fallback
        cached_target = route_state.get('target')
        if (
            route_state.get('kind') != 'rendezvous'
            or cached_target is None
            or destination.shape != (2,)
            or float(np.linalg.norm(
                destination - np.asarray(cached_target, dtype=float)
            )) > 0.02
        ):
            return fallback

        waypoints = [
            np.asarray(point, dtype=float)
            for point in route_state.get('waypoints', ())
            if np.asarray(point, dtype=float).shape == (2,)
        ]
        if not waypoints:
            return fallback
        waypoint_index = min(
            max(0, int(route_state.get('waypoint_index', 0))),
            len(waypoints) - 1,
        )
        return [position] + waypoints[waypoint_index:]

    @staticmethod
    def _rendezvous_route_length(route):
        return sum(
            float(np.linalg.norm(
                np.asarray(end, dtype=float)
                - np.asarray(start, dtype=float)
            ))
            for start, end in zip(route, route[1:])
        )

    @staticmethod
    def _rendezvous_pair_passing_sign(pair):
        """Give an unordered pair one stable passing side for the task."""
        signature = '|'.join(pair)
        checksum = sum(
            (index + 1) * ord(character)
            for index, character in enumerate(signature)
        )
        return 1.0 if checksum % 2 == 0 else -1.0

    def _coordinate_mutual_rendezvous_recovery(self, pair, positions):
        """Elect one short-route winner and give its peer a yield pocket."""
        if len(pair) != 2 or any(name not in positions for name in pair):
            return False
        recoveries = getattr(
            self, 'transport_rendezvous_recoveries', {}
        )
        decisions = getattr(
            self, 'transport_rendezvous_pair_decisions', None
        )
        if not isinstance(decisions, dict):
            decisions = {}
            self.transport_rendezvous_pair_decisions = decisions

        decision = decisions.get(pair)
        if not isinstance(decision, dict):
            ranks = {namespace: index for index, namespace in enumerate(pair)}
            route_lengths = {}
            for namespace in pair:
                route = self._remaining_rendezvous_recovery_route(
                    namespace,
                    positions[namespace],
                    recoveries.get(namespace, {}),
                )
                route_lengths[namespace] = self._rendezvous_route_length(route)
            priority = min(
                pair,
                key=lambda namespace: (
                    route_lengths[namespace], ranks[namespace]
                ),
            )
            yielder = pair[1] if priority == pair[0] else pair[0]
            get_time = getattr(rospy, 'get_time', lambda: 0.0)
            decision = {
                'priority_namespace': priority,
                'yielding_namespace': yielder,
                'passing_sign': self._rendezvous_pair_passing_sign(pair),
                'route_lengths': route_lengths,
                'started_at': float(get_time()),
                'clear_since': None,
            }
            decisions[pair] = decision

            # Replan the winner on the next tick with the yielding robot as a
            # held obstacle.  The pair decision deliberately survives that
            # cache reset, so the two robots cannot exchange roles mid-pass.
            route_states = getattr(
                self, 'transport_assembly_route_states', {}
            )
            if isinstance(route_states, dict):
                route_states.pop(priority, None)
            rospy.loginfo(
                "[transport] rendezvous right-of-way %s over %s "
                "(remaining route %.3fm vs %.3fm)",
                priority,
                yielder,
                route_lengths[priority],
                route_lengths[yielder],
            )

        priority = decision['priority_namespace']
        yielder = decision['yielding_namespace']
        for namespace, obstacle_namespace in (
            (priority, yielder), (yielder, priority)
        ):
            state = recoveries.get(namespace)
            if (
                not isinstance(state, dict)
                or state.get('obstacle_namespace') != obstacle_namespace
            ):
                continue
            state['pair'] = pair
            state['priority_namespace'] = priority
            state['yielding_namespace'] = yielder
            state['right_of_way'] = namespace == priority
            state['passing_sign'] = decision['passing_sign']

            if namespace != yielder or state.get('yield_position') is not None:
                continue
            position = np.asarray(positions[namespace], dtype=float)
            obstacle_position = np.asarray(
                positions[obstacle_namespace], dtype=float
            )
            separation = position - obstacle_position
            distance = float(np.linalg.norm(separation))
            if distance <= 1e-9:
                continue
            radial = separation / distance
            tangent = np.array([-radial[1], radial[0]])
            direction = (
                tangent * float(decision['passing_sign'])
                + radial * 0.65
            )
            direction /= max(1e-9, float(np.linalg.norm(direction)))
            yield_distance = max(
                getattr(self, 'transport_rendezvous_yield_distance', 0.28),
                0.75 * (
                    getattr(self, 'transport_route_robot_clearance', 0.32)
                    + getattr(
                        self,
                        'transport_rendezvous_clearance_hysteresis',
                        0.04,
                    )
                ),
            )
            state['direction'] = direction
            state['yield_position'] = position + direction * yield_distance
        return True

    def _rendezvous_pair_is_clear(self, pair, positions, now):
        decision = getattr(
            self, 'transport_rendezvous_pair_decisions', {}
        ).get(pair)
        recoveries = getattr(
            self, 'transport_rendezvous_recoveries', {}
        )
        if not isinstance(decision, dict):
            return False
        priority = decision.get('priority_namespace')
        if priority in set(getattr(self, 'transport_pre_staged', set())):
            return True
        if any(
            namespace not in positions or namespace not in recoveries
            for namespace in pair
        ):
            decision['clear_since'] = None
            return False

        clearance = (
            getattr(self, 'transport_route_robot_clearance', 0.32)
            + getattr(
                self, 'transport_rendezvous_clearance_hysteresis', 0.04
            )
        )
        center_distance = float(np.linalg.norm(
            np.asarray(positions[pair[0]], dtype=float)
            - np.asarray(positions[pair[1]], dtype=float)
        ))
        routes = {
            namespace: self._remaining_rendezvous_recovery_route(
                namespace, positions[namespace], recoveries[namespace]
            )
            for namespace in pair
        }
        route_points = {
            namespace: [
                (float(point[0]), float(point[1]))
                for point in route
            ]
            for namespace, route in routes.items()
        }
        clear_now = (
            center_distance >= clearance
            and not routes_conflict(
                route_points[pair[0]],
                route_points[pair[1]],
                clearance,
            )
        )
        if not clear_now or not math.isfinite(now):
            decision['clear_since'] = None
            return False

        clear_since = decision.get('clear_since')
        if (
            clear_since is None
            or not math.isfinite(float(clear_since))
            or now < float(clear_since)
        ):
            decision['clear_since'] = now
            return getattr(
                self, 'transport_rendezvous_release_hold_time', 0.40
            ) <= 0.0
        return now - float(clear_since) >= getattr(
            self, 'transport_rendezvous_release_hold_time', 0.40
        )

    def _clear_rendezvous_pair(
        self, pair, cooldown=False, resolved=False
    ):
        recoveries = getattr(
            self, 'transport_rendezvous_recoveries', {}
        )
        route_states = getattr(
            self, 'transport_assembly_route_states', {}
        )
        for namespace in pair:
            recoveries.pop(namespace, None)
            if isinstance(route_states, dict):
                route_states.pop(namespace, None)

        decisions = getattr(
            self, 'transport_rendezvous_pair_decisions', {}
        )
        if resolved:
            decisions.pop(pair, None)
        elif pair in decisions:
            decisions[pair]['clear_since'] = None

        if cooldown:
            get_time = getattr(rospy, 'get_time', lambda: 0.0)
            now = float(get_time())
            cooldowns = getattr(
                self, 'transport_rendezvous_recovery_cooldowns', None
            )
            if not isinstance(cooldowns, dict):
                cooldowns = {}
                self.transport_rendezvous_recovery_cooldowns = cooldowns
            for namespace in pair:
                cooldowns[namespace] = now + 1.0

    def _clear_rendezvous_recovery(self, namespace, cooldown=False):
        recoveries = getattr(
            self, 'transport_rendezvous_recoveries', {}
        )
        recoveries.pop(namespace, None)
        if cooldown:
            get_time = getattr(rospy, 'get_time', lambda: 0.0)
            now = float(get_time())
            cooldowns = getattr(
                self, 'transport_rendezvous_recovery_cooldowns', None
            )
            if not isinstance(cooldowns, dict):
                cooldowns = {}
                self.transport_rendezvous_recovery_cooldowns = cooldowns
            cooldowns[namespace] = now + 1.0
        states = getattr(self, 'transport_assembly_route_states', {})
        if isinstance(states, dict):
            states.pop(namespace, None)

    def _rendezvous_route_ignored_namespaces(
        self, namespace, active_movers
    ):
        """Let a right-of-way winner plan around its yielding peer."""
        ignored = set(active_movers)
        state = getattr(
            self, 'transport_rendezvous_recoveries', {}
        ).get(namespace, {})
        if isinstance(state, dict) and state.get('right_of_way'):
            ignored.discard(state.get('yielding_namespace'))
        return ignored

    def _rendezvous_recovery_command(
        self, namespace, position, yaw, target, positions
    ):
        """Follow a bounded yield manoeuvre when a moving peer blocks us."""
        recoveries = getattr(
            self, 'transport_rendezvous_recoveries', None
        )
        if not isinstance(recoveries, dict):
            recoveries = {}
            self.transport_rendezvous_recoveries = recoveries
        state = recoveries.get(namespace)
        if not isinstance(state, dict):
            return None

        obstacle_namespace = state.get('obstacle_namespace')
        obstacle_position = positions.get(obstacle_namespace)
        origin = np.asarray(state.get('origin', np.empty(0)), dtype=float)
        direction = np.asarray(
            state.get('direction', np.empty(0)), dtype=float
        )
        pair = tuple(state.get('pair', ()))
        if (
            obstacle_position is None
            or origin.shape != (2,)
            or direction.shape != (2,)
            or not np.all(np.isfinite(origin))
            or not np.all(np.isfinite(direction))
        ):
            if pair:
                self._clear_rendezvous_pair(pair, resolved=True)
            else:
                self._clear_rendezvous_recovery(namespace)
            return None

        get_time = getattr(rospy, 'get_time', lambda: 0.0)
        now = float(get_time())
        if pair and self._rendezvous_pair_is_clear(pair, positions, now):
            self._clear_rendezvous_pair(pair, resolved=True)
            return None

        last_position = np.asarray(
            state.get('last_position', origin), dtype=float
        )
        last_progress_at = float(state.get('last_progress_at', now))
        started_at = float(state.get('started_at', now))
        if (
            last_position.shape == (2,)
            and np.all(np.isfinite(last_position))
            and float(np.linalg.norm(position - last_position)) >= 0.01
        ):
            state['last_position'] = np.asarray(position, dtype=float).copy()
            state['last_progress_at'] = now
            last_progress_at = now

        # The elected winner keeps its normal route.  Only its peer leaves
        # the corridor, so reciprocal blockers cannot mirror one another's
        # tangent forever.
        if pair and bool(state.get('right_of_way')):
            return None

        yield_position = np.asarray(
            state.get('yield_position', np.empty(0)), dtype=float
        )
        yield_remaining = float('inf')
        if yield_position.shape == (2,) and np.all(np.isfinite(yield_position)):
            yield_remaining = float(np.linalg.norm(
                yield_position - np.asarray(position, dtype=float)
            ))

        obstacle_distance = float(np.linalg.norm(
            position - obstacle_position
        ))
        if pair and yield_remaining <= 0.025:
            desired_clearance = (
                getattr(self, 'transport_route_robot_clearance', 0.32)
                + getattr(
                    self, 'transport_rendezvous_clearance_hysteresis', 0.04
                )
            )
            if obstacle_distance < desired_clearance:
                radial = np.asarray(position, dtype=float) - np.asarray(
                    obstacle_position, dtype=float
                )
                radial_norm = float(np.linalg.norm(radial))
                if radial_norm > 1e-9:
                    radial /= radial_norm
                    extension = max(
                        0.05,
                        desired_clearance - obstacle_distance + 0.03,
                    )
                    yield_position = (
                        np.asarray(position, dtype=float)
                        + radial * extension
                    )
                    state['yield_position'] = yield_position
                    yield_remaining = extension
            if yield_remaining <= 0.025:
                # This is a local traffic yield, not a fleet parking gate:
                # the winner and every unrelated robot remain active.
                state['last_progress_at'] = now
                return Twist()

        stalled = (
            math.isfinite(now)
            and math.isfinite(last_progress_at)
            and now >= last_progress_at
            and now - last_progress_at >= 2.5
        )
        expired = (
            math.isfinite(now)
            and math.isfinite(started_at)
            and now >= started_at
            and now - started_at >= 8.0
        )
        if stalled or expired:
            if pair:
                self._clear_rendezvous_pair(
                    pair, cooldown=True, resolved=False
                )
            else:
                self._clear_rendezvous_recovery(namespace, cooldown=True)
            rospy.logwarn(
                "[transport] %s rendezvous detour made no progress; "
                "replanning its normal route",
                namespace,
            )
            return None

        travelled = float(np.linalg.norm(position - origin))
        release_distance = max(
            0.40,
            getattr(self, 'transport_route_robot_clearance', 0.32) + 0.08,
        )
        if (
            not pair
            and (travelled >= 0.16 or obstacle_distance >= release_distance)
        ):
            self._clear_rendezvous_recovery(namespace)
            return None

        if pair and yield_position.shape == (2,):
            direction = yield_position - np.asarray(position, dtype=float)
        direction_norm = float(np.linalg.norm(direction))
        if direction_norm <= 1e-9:
            return Twist() if pair else None
        direction /= direction_norm
        speed = min(
            self.vmax,
            max(0.04, min(0.08, self.transport_chain_staging_speed)),
        )
        return self._holonomic_to_diff_drive(
            float(direction[0] * speed),
            float(direction[1] * speed),
            yaw,
        )

    def _start_rendezvous_recovery(
        self, namespace, position, target, positions,
        travel_direction=None,
    ):
        """Start or join a stable right-of-way decision for one blocker."""
        get_time = getattr(rospy, 'get_time', lambda: 0.0)
        now = float(get_time())
        cooldowns = getattr(
            self, 'transport_rendezvous_recovery_cooldowns', {}
        )
        cooldown_until = cooldowns.get(namespace)
        if (
            cooldown_until is not None
            and math.isfinite(now)
            and now < float(cooldown_until)
        ):
            return False
        cooldowns.pop(namespace, None)
        route_direction = None
        if travel_direction is not None:
            route_direction = np.asarray(travel_direction, dtype=float)
            if (
                route_direction.shape != (2,)
                or not np.all(np.isfinite(route_direction))
            ):
                return False
            direction_norm = float(np.linalg.norm(route_direction))
            if direction_norm <= 1e-9:
                return False
            route_direction = route_direction / direction_norm

        corridor_width = getattr(
            self, 'transport_route_robot_clearance', 0.32
        )
        candidates = []
        for other_namespace, other_position in positions.items():
            if other_namespace == namespace:
                continue
            offset = np.asarray(other_position, dtype=float) - position
            if offset.shape != (2,) or not np.all(np.isfinite(offset)):
                continue
            if route_direction is not None:
                forward_offset = float(np.dot(offset, route_direction))
                lateral_offset = abs(float(
                    route_direction[0] * offset[1]
                    - route_direction[1] * offset[0]
                ))
                if forward_offset <= 1e-6 or lateral_offset > corridor_width:
                    continue
            candidates.append((float(np.linalg.norm(offset)), other_namespace))
        if not candidates:
            return False
        distance, obstacle_namespace = min(candidates)
        maximum_distance = max(
            0.40,
            getattr(self, 'transport_route_robot_clearance', 0.32) + 0.10,
        )
        if distance > maximum_distance or distance <= 1e-9:
            return False

        obstacle_position = positions[obstacle_namespace]
        radial = (position - obstacle_position) / distance
        tangent = np.array([-radial[1], radial[0]])
        destination = self._rendezvous_recovery_destination(
            namespace, target
        )
        toward_destination = destination - position
        if float(np.dot(-tangent, toward_destination)) > float(np.dot(
            tangent, toward_destination
        )):
            tangent *= -1.0
        direction = tangent + radial * 0.45
        direction_norm = float(np.linalg.norm(direction))
        if direction_norm <= 1e-9:
            return False

        recoveries = getattr(
            self, 'transport_rendezvous_recoveries', None
        )
        if not isinstance(recoveries, dict):
            recoveries = {}
            self.transport_rendezvous_recoveries = recoveries
        if namespace in recoveries:
            return False
        recoveries[namespace] = {
            'obstacle_namespace': obstacle_namespace,
            'origin': np.asarray(position, dtype=float).copy(),
            'direction': direction / direction_norm,
            'destination': destination.copy(),
            'started_at': now,
            'last_progress_at': now,
            'last_position': np.asarray(position, dtype=float).copy(),
        }
        pair = self._rendezvous_pair_key(namespace, obstacle_namespace)
        obstacle_state = recoveries.get(obstacle_namespace)
        decisions = getattr(
            self, 'transport_rendezvous_pair_decisions', {}
        )
        reciprocal = (
            isinstance(obstacle_state, dict)
            and obstacle_state.get('obstacle_namespace') == namespace
        )
        if pair and (reciprocal or pair in decisions):
            self._coordinate_mutual_rendezvous_recovery(pair, positions)
        rospy.loginfo(
            "[transport] %s taking a short rendezvous detour around %s",
            namespace,
            obstacle_namespace,
        )
        return True

    def _publish_concurrent_approach_command(
        self,
        namespace,
        command,
        target,
        positions,
        yaws,
        targets,
        object_pos,
        object_yaw,
        neighbours,
        expected_epoch,
        payload_terminal=False,
        chain_motion=False,
        alignment_only=False,
    ):
        """Apply the normal safety layers to one concurrent approach move."""
        if alignment_only:
            with self.data_lock:
                avoidance = getattr(
                    self, 'avoidance_modules', {}
                ).get(namespace)
            reset_motion = getattr(avoidance, 'reset_motion', None)
            if callable(reset_motion):
                reset_motion()
            self._publish_command(namespace, command, expected_epoch)
            return

        if not chain_motion:
            recovery_command = self._rendezvous_recovery_command(
                namespace,
                positions[namespace],
                yaws[namespace],
                target,
                positions,
            )
            if recovery_command is not None:
                command = recovery_command

        stopped = (
            abs(float(command.linear.x)) < 1e-6
            and abs(float(command.angular.z)) < 1e-6
        )
        if stopped:
            with self.data_lock:
                avoidance = getattr(
                    self, 'avoidance_modules', {}
                ).get(namespace)
            if avoidance is not None:
                avoidance.reset_motion()
            self._publish_command(namespace, Twist(), expected_epoch)
            return

        contact_neighbours, shielded_neighbours, row_neighbours = neighbours
        allowed_contacts = set()
        shielded_contacts = set()
        parallel_contacts = set()
        if chain_motion:
            allowed_contacts.update(self._nearby_chain_contacts(
                namespace, contact_neighbours, positions, targets
            ))
            shielded_contacts.update(self._nearby_chain_contacts(
                namespace, shielded_neighbours, positions, targets
            ))
            parallel_contacts.update(
                set(row_neighbours.get(namespace, ()))
                & set(self._parallel_lane_contacts(
                    namespace, positions, targets
                ))
            )

        parent_namespace = target.get('parent_namespace')
        parent_contact_allowed = (
            target.get('role') == 'companion_push'
            and parent_namespace in allowed_contacts
        )
        if target.get('role') == 'payload_push':
            command, payload_guarded = self._guard_payload_approach(
                positions[namespace], yaws[namespace], command,
                object_pos, object_yaw,
            )
            if payload_guarded:
                with self.data_lock:
                    avoidance = getattr(
                        self, 'avoidance_modules', {}
                    ).get(namespace)
                if avoidance is not None:
                    avoidance.reset_motion()

        other_lidar_neighbours = (
            shielded_contacts | parallel_contacts
        )
        avoidance_options = {
            'allow_payload_contact': (
                target.get('role') == 'payload_push'
                and payload_terminal
            ),
            'payload_contact_confirmed': payload_terminal,
            'allowed_contact_position': (
                target.get('parent_position')
                if parent_contact_allowed else None
            ),
            'allowed_contact_namespace': (
                parent_namespace if parent_contact_allowed else None
            ),
            'allowed_contact_namespaces': tuple(sorted(allowed_contacts)),
            'repulsion_exempt_namespaces': tuple(sorted(
                shielded_contacts | parallel_contacts
            )),
            'parallel_motion_exempt_namespaces': tuple(sorted(
                parallel_contacts
            )),
            'shielded_motion_exempt_namespaces': tuple(sorted(
                shielded_contacts
            )),
            'additional_lidar_masks': self._transport_robot_lidar_masks(
                allowed_contacts, other_lidar_neighbours, positions, yaws
            ),
            'soft_steering': not chain_motion,
            'object_yaw': object_yaw,
        }
        requested_signed_linear = float(command.linear.x)
        requested_linear = abs(requested_signed_linear)
        requested_direction = np.array([
            math.cos(yaws[namespace]), math.sin(yaws[namespace])
        ])
        if requested_signed_linear < 0.0:
            requested_direction *= -1.0
        command = self._apply_transport_avoidance(
            namespace,
            command,
            object_pos,
            **avoidance_options
        )
        # Avoidance may keep a Burger turning while it refuses the requested
        # translation.  That is still a blocked rendezvous route; waiting for
        # angular.z to become zero leaves two facing robots spinning forever.
        translation_stopped = abs(float(command.linear.x)) < 1e-4
        if (
            not chain_motion
            and requested_linear >= 0.01
            and translation_stopped
            and self._start_rendezvous_recovery(
                namespace,
                positions[namespace],
                target,
                positions,
                travel_direction=requested_direction,
            )
        ):
            recovery_command = self._rendezvous_recovery_command(
                namespace,
                positions[namespace],
                yaws[namespace],
                target,
                positions,
            )
            if recovery_command is not None:
                command = self._apply_transport_avoidance(
                    namespace,
                    recovery_command,
                    object_pos,
                    **avoidance_options
                )
        self._publish_command(namespace, command, expected_epoch)

    def _approach_phase(self, expected_epoch):
        """Rendezvous the whole fleet, then form both push lanes together."""
        with self.model_lock:
            object_pos = (
                self.object_position.copy()
                if self.object_position is not None else None
            )
            object_yaw = self.object_yaw
            object_error = self.object_error

        if object_pos is None:
            self._fail_transport(
                object_error or "Transport payload was lost",
                expected_epoch,
            )
            return

        with self.data_lock:
            namespaces = list(self.robot_namespaces)
            robot_velocities = getattr(self, 'robot_velocities', {})
            positions = {
                namespace: self.robot_positions[namespace].copy()
                for namespace in namespaces
                if namespace in self.robot_positions
            }
            yaws = {
                namespace: self.robot_yaws[namespace]
                for namespace in namespaces
                if namespace in self.robot_yaws
            }
            velocities = {
                namespace: robot_velocities[namespace].copy()
                for namespace in namespaces
                if namespace in robot_velocities
            }
            odom_received_at = dict(getattr(
                self, 'robot_odom_received_at', {}
            ))

        odometry_error = self._transport_odometry_error(
            namespaces, positions, yaws, odom_received_at
        )
        if odometry_error is not None:
            self._fail_transport(odometry_error, expected_epoch)
            return
        target_error = self._transport_target_error()
        if target_error is not None:
            self._fail_transport(target_error, expected_epoch)
            return

        targets = self._transport_targets(
            namespaces, positions, object_pos, object_yaw, velocities
        )
        layout_error = self._transport_layout_error(
            targets,
            object_pos,
            object_yaw,
            fleet_size=len(namespaces),
        )
        if layout_error is not None:
            self._fail_transport(layout_error, expected_epoch)
            return
        neighbours = self._transport_neighbours(targets)

        position_ready = set()
        locally_ready = set()
        previous_staged = set(getattr(
            self, 'transport_staged', set()
        ))
        for namespace in namespaces:
            target = targets.get(namespace)
            if target is None:
                continue
            margin = (
                0.025
                if target['role'] == 'payload_push'
                else getattr(
                    self, 'transport_chain_staging_tolerance', 0.025
                )
            )
            if namespace in previous_staged:
                margin = max(
                    margin,
                    getattr(
                        self,
                        'transport_chain_staging_release_tolerance',
                        0.06,
                    ),
                )
            if not self._slot_contact_ready(
                positions[namespace], yaws[namespace],
                object_pos, object_yaw, target,
                require_inward_heading=False,
                contact_margin=margin,
                staging=True,
            ):
                continue
            position_ready.add(namespace)
            if self._slot_contact_ready(
                positions[namespace], yaws[namespace],
                object_pos, object_yaw, target,
                require_inward_heading=True,
                contact_margin=margin,
                staging=True,
            ):
                locally_ready.add(namespace)

        staged = self._ready_chain_prefix(targets, locally_ready)
        self.transport_staged = staged
        payload_names = {
            namespace for namespace, target in targets.items()
            if target['role'] == 'payload_push'
        }
        companion_names = set(targets) - payload_names

        rendezvoused = set(getattr(
            self, 'transport_pre_staged', set()
        )) & set(targets)
        for namespace in namespaces:
            if (
                namespace not in rendezvoused
                and self._rendezvous_pose_ready(
                    positions[namespace], yaws[namespace], targets[namespace]
                )
            ):
                rendezvoused.add(namespace)
        self.transport_pre_staged = rendezvoused
        all_rendezvoused = set(targets) <= rendezvoused

        if not all_rendezvoused:
            self.transport_chain_released = set()
            self.transport_compression_progress = 0.0
            self.transport_compression_updated_at = None
            active_movers = set(targets) - rendezvoused
            for namespace in namespaces:
                target = targets[namespace]
                ignored_movers = self._rendezvous_route_ignored_namespaces(
                    namespace, active_movers
                )
                staging_position = np.asarray(
                    target.get('staging_position', target['position']),
                    dtype=float,
                )
                payload_terminal = False
                if namespace in rendezvoused:
                    command = self._payload_staging_command(
                        positions[namespace],
                        yaws[namespace],
                        staging_position,
                        target['push_direction'],
                        forward_limit=0.04,
                        reverse_limit=0.04,
                    )
                elif target['role'] == 'payload_push':
                    route_destination = (
                        self._payload_staging_route_destination(target)
                    )
                    command = self._transport_route_command(
                        namespace,
                        positions[namespace],
                        yaws[namespace],
                        route_destination,
                        positions,
                        'rendezvous',
                        ignored_namespaces=ignored_movers,
                        final_handoff_tolerance=0.04,
                    )
                    if command is None:
                        command = self._payload_staging_command(
                            positions[namespace],
                            yaws[namespace],
                            staging_position,
                            target['push_direction'],
                            forward_limit=0.08,
                            reverse_limit=0.06,
                        )
                        payload_terminal = True
                else:
                    command = self._transport_route_command(
                        namespace,
                        positions[namespace],
                        yaws[namespace],
                        staging_position,
                        positions,
                        'rendezvous',
                        ignored_namespaces=ignored_movers,
                        final_handoff_tolerance=getattr(
                            self, 'transport_chain_staging_tolerance', 0.025
                        ),
                    )
                    if command is None:
                        command = self._chain_staging_command(
                            positions[namespace], yaws[namespace], target
                        )
                self._publish_concurrent_approach_command(
                    namespace,
                    command,
                    target,
                    positions,
                    yaws,
                    targets,
                    object_pos,
                    object_yaw,
                    neighbours,
                    expected_epoch,
                    payload_terminal=payload_terminal,
                )
            rendezvous_count = len(rendezvoused)
            if getattr(
                self, 'transport_last_rendezvous_log_count', None
            ) != rendezvous_count:
                self.transport_last_rendezvous_log_count = rendezvous_count
                rospy.loginfo(
                    "[transport] concurrent rendezvous %d/%d near the "
                    "reported payload",
                    rendezvous_count,
                    len(targets),
                )
            return

        if set(getattr(
            self, 'transport_chain_released', set()
        )) != companion_names:
            self.transport_chain_released = set(companion_names)
            self.transport_compression_progress = 0.0
            self.transport_compression_updated_at = None
            self.transport_assembly_route_states = {}
            rospy.loginfo(
                "[transport] all %d robots rendezvoused; "
                "compressing both push lanes together",
                len(targets),
            )

        progress = self._advance_transport_compression(
            set(targets), positions, targets, yaws
        )
        for namespace in namespaces:
            target = targets[namespace]
            alignment_only = False
            if target['role'] == 'payload_push':
                destination = np.asarray(
                    target.get('staging_position', target['position']),
                    dtype=float,
                )
                command = self._payload_staging_command(
                    positions[namespace],
                    yaws[namespace],
                    destination,
                    target['push_direction'],
                    forward_limit=0.04,
                    reverse_limit=0.04,
                )
            else:
                destination = self._compression_target(target, progress)
                final_error = float(np.linalg.norm(
                    np.asarray(target.get(
                        'assembly_position', target['position']
                    )) - positions[namespace]
                ))
                if namespace in staged:
                    command = self._chain_pose_hold_command(
                        positions[namespace],
                        yaws[namespace],
                        np.asarray(target.get(
                            'assembly_position', target['position']
                        )),
                        target['push_direction'],
                        forward_limit=0.04,
                        reverse_limit=0.04,
                    )
                elif progress >= 1.0 and namespace in position_ready:
                    command = self._staging_alignment_command(
                        positions[namespace], yaws[namespace], target
                    )
                    alignment_only = True
                elif (
                    progress >= 1.0
                    and final_error <= getattr(
                        self, 'transport_route_parent_clearance', 0.20
                    ) + 0.03
                ):
                    command = self._companion_assembly_command(
                        positions[namespace],
                        yaws[namespace],
                        target,
                        allow_closing=True,
                    )
                elif progress > 0.5:
                    # After the midpoint handoff, keep every chassis facing
                    # the force direction.  At exactly 0.5 the point-bearing
                    # controller still settles the last lateral centimetres
                    # and then turns in place; only then may the heading gate
                    # release longitudinal motion.  Continuing point-bearing
                    # beyond that gate could choose a shorter reverse approach
                    # whose rear footprint locks against its parent's rear.
                    command = self._chain_pose_hold_command(
                        positions[namespace],
                        yaws[namespace],
                        destination,
                        target['push_direction'],
                        forward_limit=getattr(
                            self, 'transport_compression_speed', 0.12
                        ),
                        reverse_limit=0.06,
                    )
                else:
                    command = self._payload_staging_command(
                        positions[namespace],
                        yaws[namespace],
                        destination,
                        target['push_direction'],
                        forward_limit=getattr(
                            self, 'transport_compression_speed', 0.12
                        ),
                        reverse_limit=0.06,
                    )

            midpoint_error = float(np.linalg.norm(
                destination - positions[namespace]
            ))
            midpoint_settled_distance = min(
                0.012,
                getattr(
                    self, 'transport_chain_staging_tolerance', 0.025
                ),
            )
            if (
                abs(progress - 0.5) <= 1e-9
                and midpoint_error <= midpoint_settled_distance
                and abs(float(command.linear.x)) < 1e-6
            ):
                # Midpoint rows are still separated by the wide rendezvous
                # gap, so a Burger can safely turn in place.  The normal
                # obstacle smoother may pick the opposite turn when it sees
                # both adjacent rows at the edge of its awareness radius;
                # bypass it only for this zero-translation heading barrier.
                alignment_only = True

            self._publish_concurrent_approach_command(
                namespace,
                command,
                target,
                positions,
                yaws,
                targets,
                object_pos,
                object_yaw,
                neighbours,
                expected_epoch,
                chain_motion=True,
                alignment_only=alignment_only,
            )

        ready_count = len(staged & set(namespaces))
        required_count = len(namespaces)
        if (
            required_count > 0
            and len(targets) == required_count
            and ready_count == required_count
            and self._parallel_push_rows_ready(positions, targets)
        ):
            ready_hold_time = getattr(
                self, 'transport_ready_hold_time', 0.0
            )
            if ready_hold_time > 0.0:
                now = float(rospy.get_time())
                if self.transport_all_ready_since is None:
                    self.transport_all_ready_since = now
                if now - self.transport_all_ready_since < ready_hold_time:
                    return
            self.transport_all_ready_since = None
            rospy.loginfo(
                "[transport] all %d robots connected in both push lanes "
                ">>> phase PUSH",
                ready_count,
            )
            self._set_phase(TransportPhase.PUSH, expected_epoch)
        else:
            self.transport_all_ready_since = None

    def _publish_transport_launch_hold(
        self, namespaces, targets, expected_epoch
    ):
        """Clear approach momentum and hold one coherent zero-command batch."""
        with self.data_lock:
            avoidance_modules = {
                namespace: getattr(self, 'avoidance_modules', {}).get(
                    namespace
                )
                for namespace in namespaces
            }
        for avoidance in avoidance_modules.values():
            if avoidance is not None:
                avoidance.reset_motion()

        publish_order = sorted(
            namespaces,
            key=lambda namespace: (
                -int(targets.get(namespace, {}).get('chain_depth', -1)),
                int(targets.get(namespace, {}).get('chain_index', 0)),
                namespace,
            ),
        )
        for namespace in publish_order:
            if not self._publish_command(
                namespace, Twist(), expected_epoch
            ):
                return False
        return True

    def _transport_fleet_is_settled(
        self, namespaces, started_at, now
    ):
        """Return true after one bounded zero-command settling window."""
        if (
            started_at is None
            or not math.isfinite(float(started_at))
            or not math.isfinite(float(now))
            or now < started_at
        ):
            return False

        elapsed = now - started_at
        settle_time = getattr(
            self, 'transport_launch_settle_time', 0.35
        )
        if elapsed < settle_time:
            return False

        with self.data_lock:
            robot_velocities = getattr(self, 'robot_velocities', {})
            velocities = {
                namespace: np.asarray(
                    robot_velocities[namespace], dtype=float
                ).copy()
                for namespace in namespaces
                if namespace in robot_velocities
            }
        speed_limit = getattr(
            self, 'transport_launch_settle_speed', 0.004
        )
        if len(velocities) == len(namespaces) and all(
            velocity.shape == (2,)
            and np.all(np.isfinite(velocity))
            and float(np.linalg.norm(velocity)) <= speed_limit
            for velocity in velocities.values()
        ):
            return True

        # A stale or noisy odometry sample must not deadlock a complete fleet.
        # Zero commands remain published throughout this bounded fallback.
        return elapsed >= getattr(
            self, 'transport_launch_settle_timeout', 1.25
        )

    def _transport_launch_is_settled(self, namespaces, now):
        """Wait for stopped docking wheels before applying payload load."""
        return self._transport_fleet_is_settled(
            namespaces,
            getattr(
                self, 'transport_launch_settle_started_at', None
            ),
            now,
        )

    def _transport_queue_is_settled(self, namespaces, now):
        """Wait for APPROACH motion to stop before the queues dock."""
        return self._transport_fleet_is_settled(
            namespaces,
            getattr(
                self, 'transport_queue_settle_started_at', None
            ),
            now,
        )

    def _return_transport_queues_to_staging(
        self, namespaces, targets, expected_epoch, reason
    ):
        """Back out of docking instead of pressing through an open queue."""
        self._publish_transport_launch_hold(
            namespaces, targets, expected_epoch
        )
        self.transport_staged = set()
        self.transport_pre_staged = set()
        self.transport_chain_released = set()
        self.transport_all_ready_since = None
        self._reset_transport_route()
        if self._set_phase(TransportPhase.APPROACH, expected_epoch):
            rospy.logwarn(
                "[transport] %s; rebuilding both queues clear of the payload",
                reason,
            )
        return True

    def _engage_push_chains(
        self, namespaces, positions, yaws, obj_pos, obj_yaw,
        expected_epoch,
    ):
        """Close and keep every payload and companion contact connected."""
        targets = self._transport_targets(
            namespaces, positions, obj_pos, obj_yaw
        )
        push_names = set(targets)
        if not push_names:
            self.transport_engaged = set()
            self.transport_physical_engaged = set()
            self.transport_aligned_engaged = set()
            self.transport_engagement_complete = True
            return False

        previous_engaged = set(
            getattr(self, 'transport_engaged', set())
        )
        get_time = getattr(rospy, 'get_time', lambda: 0.0)
        now = float(get_time())
        self.transport_engagement_last_ready = {
            namespace: ready_at
            for namespace, ready_at in getattr(
                self, 'transport_engagement_last_ready', {}
            ).items()
            if namespace in push_names
        }
        raw_contact_ready = set()
        contact_ready = set()
        physical_contact_ready = set()
        heading_ready_names = set()
        companion_close_targets = {}
        for namespace in push_names:
            if namespace not in positions or namespace not in yaws:
                continue
            target = targets[namespace]
            ready = False
            raw_ready = False
            physical_ready = False
            push_direction = np.asarray(
                target.get('push_direction', np.zeros(2)), dtype=float
            )
            push_norm = float(np.linalg.norm(push_direction))
            heading_ready = False
            if push_norm > 1e-9 and np.all(np.isfinite(push_direction)):
                desired_yaw = math.atan2(
                    float(push_direction[1]), float(push_direction[0])
                )
                heading_ready = abs(self._normalize_angle(
                    desired_yaw - float(yaws[namespace])
                )) <= getattr(
                    self, 'transport_launch_heading_tolerance', 0.08
                )
            if heading_ready:
                heading_ready_names.add(namespace)
            if target['role'] == 'payload_push':
                lateral = np.array([
                    -push_direction[1], push_direction[0]
                ])
                lateral_error = abs(float(np.dot(
                    positions[namespace]
                    - np.asarray(target['position']),
                    lateral,
                )))
                ready = (
                    self._payload_contact_near(
                        positions[namespace], obj_pos, obj_yaw,
                        # The first synchronized push begins only after real
                        # contact. The wider margin is release hysteresis for
                        # an already moving chain, not a launch condition.
                        margin=(
                            0.03
                            if getattr(
                                self,
                                'transport_synchronized_push_started',
                                False,
                            )
                            else self.transport_contact_slop
                        ),
                    )
                    and lateral_error <= 0.08
                )
                physical_ready = ready
                raw_ready = ready and heading_ready
            else:
                parent_namespace = target['parent_namespace']
                physical_ready, close_target = (
                    self._companion_engagement_geometry(
                        positions[namespace],
                        positions[parent_namespace],
                        target['push_direction'],
                        release=False,
                        allow_alignment_retreat=(
                            int(target.get('chain_depth', 0))
                            >= int(target.get('max_chain_depth', 0))
                        ),
                    )
                )
                ready = physical_ready
                raw_ready = physical_ready and heading_ready
                if namespace in previous_engaged and not physical_ready:
                    ready, close_target = (
                        self._companion_engagement_geometry(
                            positions[namespace],
                            positions[parent_namespace],
                            target['push_direction'],
                            release=True,
                            allow_alignment_retreat=(
                                int(target.get('chain_depth', 0))
                                >= int(target.get('max_chain_depth', 0))
                            ),
                        )
                    )
                companion_close_targets[namespace] = close_target
            if physical_ready:
                physical_contact_ready.add(namespace)
            if raw_ready:
                raw_contact_ready.add(namespace)
            ready = self._engagement_contact_with_grace(
                namespace, ready, previous_engaged, now
            )
            if ready:
                contact_ready.add(namespace)

        # Contact only counts when it is connected all the way to the
        # payload. A rear robot touching an unconnected parent cannot yet
        # transfer useful force to the crate.
        connected = {
            namespace
            for namespace in contact_ready
            if targets[namespace]['role'] == 'payload_push'
        }
        for namespace in sorted(
            push_names,
            key=lambda name: (
                int(targets[name].get('chain_depth', 0)),
                int(targets[name].get('chain_index', 0)),
                name,
            ),
        ):
            target = targets[namespace]
            if target['role'] != 'companion_push':
                continue
            if (
                namespace in contact_ready
                and target['parent_namespace'] in connected
            ):
                connected.add(namespace)

        # Release hysteresis keeps a noisy link from dropping out of the
        # connected prefix, but it must not satisfy the launch barrier. Every
        # link has to remain physically ready for the whole launch hold.
        raw_connected = {
            namespace
            for namespace in raw_contact_ready
            if targets[namespace]['role'] == 'payload_push'
        }
        for namespace in sorted(
            push_names,
            key=lambda name: (
                int(targets[name].get('chain_depth', 0)),
                int(targets[name].get('chain_index', 0)),
                name,
            ),
        ):
            target = targets[namespace]
            if target['role'] != 'companion_push':
                continue
            if (
                namespace in raw_contact_ready
                and target['parent_namespace'] in raw_connected
            ):
                raw_connected.add(namespace)
        contact_connected = {
            namespace
            for namespace in physical_contact_ready
            if targets[namespace]['role'] == 'payload_push'
        }
        for namespace in sorted(
            push_names,
            key=lambda name: (
                int(targets[name].get('chain_depth', 0)),
                int(targets[name].get('chain_index', 0)),
                name,
            ),
        ):
            target = targets[namespace]
            if target['role'] != 'companion_push':
                continue
            if (
                namespace in physical_contact_ready
                and target['parent_namespace'] in contact_connected
            ):
                contact_connected.add(namespace)
        self.transport_physical_engaged = contact_connected
        self.transport_aligned_engaged = raw_connected

        # APPROACH assembles the two queues while their lead robots are still
        # parked clear of the payload.  Track that local connectivity without
        # requiring a payload root, otherwise PUSH would move each lead away
        # from the companions it had just assembled.
        queue_connected = {
            namespace
            for namespace, target in targets.items()
            if (
                target['role'] == 'payload_push'
                and namespace in heading_ready_names
            )
        }
        for namespace in sorted(
            push_names,
            key=lambda name: (
                int(targets[name].get('chain_depth', 0)),
                int(targets[name].get('chain_index', 0)),
                name,
            ),
        ):
            target = targets[namespace]
            if target['role'] != 'companion_push':
                continue
            if (
                namespace in raw_contact_ready
                and target['parent_namespace'] in queue_connected
            ):
                queue_connected.add(namespace)
        parallel_rows_ready = self._parallel_push_rows_ready(
            positions, targets
        )
        local_queues_ready = (
            push_names <= queue_connected and parallel_rows_ready
        )
        root_stage_margin = getattr(
            self, 'transport_chain_staging_release_tolerance', 0.06
        )
        payload_names = {
            namespace
            for namespace, target in targets.items()
            if target['role'] == 'payload_push'
        }
        roots_staged = (
            payload_names <= (set(positions) & set(yaws))
            and payload_names <= heading_ready_names
            and all(
                targets[namespace].get('staging_position') is None
                or float(np.linalg.norm(
                    np.asarray(
                        targets[namespace]['staging_position'],
                        dtype=float,
                    )
                    - np.asarray(positions[namespace], dtype=float)
                )) <= root_stage_margin
                for namespace in payload_names
            )
        )
        docking_started = bool(getattr(
            self, 'transport_queue_docking_started', False
        ))
        complete_queues_have_unsafe_rows = (
            push_names <= queue_connected and not parallel_rows_ready
        )
        if complete_queues_have_unsafe_rows:
            payload_contacted = bool(
                payload_names & physical_contact_ready
            )
            if not docking_started or not payload_contacted:
                return self._return_transport_queues_to_staging(
                    namespaces,
                    targets,
                    expected_epoch,
                    "parallel queue rows lost their safe clearance",
                )

            self._fail_transport(
                "Parallel queue rows lost their safe clearance while a "
                "lead robot was loaded against the payload",
                expected_epoch,
            )
            return True
        queues_ready = local_queues_ready and (
            docking_started or roots_staged
        )

        # The complete physical chain is a launch barrier, not a global
        # emergency brake. Once synchronized pushing has begun, each command
        # follows its live parent and can close a local gap while the other
        # links keep adding force. Rebuilding the whole chain for one noisy
        # contact sample makes long fleets pulse between push and pose-hold.
        if getattr(
            self, 'transport_synchronized_push_started', False
        ):
            self.transport_engaged = connected
            self.transport_engagement_complete = True
            self.transport_engagement_ready_since = None
            return False

        parent_distances = {
            namespace: float(distance)
            for namespace, distance in getattr(
                self, 'transport_engagement_parent_distances', {}
            ).items()
            if (
                namespace in connected
                and targets[namespace]['role'] == 'companion_push'
                and math.isfinite(float(distance))
            )
        }
        for namespace in connected:
            target = targets[namespace]
            if (
                target['role'] == 'companion_push'
                and namespace not in parent_distances
            ):
                measured_distance = float(np.linalg.norm(
                    positions[namespace]
                    - positions[target['parent_namespace']]
                ))
                parent_distances[namespace] = (
                    self._loaded_companion_distance(measured_distance)
                )
        self.transport_engagement_parent_distances = parent_distances

        was_complete = getattr(
            self, 'transport_engagement_complete', False
        )
        self.transport_engaged = connected
        all_connected = (
            push_names <= raw_connected
            and self._parallel_push_rows_ready(positions, targets)
        )
        if all_connected and was_complete:
            if self._transport_launch_is_settled(namespaces, now):
                self.transport_launch_settle_started_at = None
                rospy.loginfo(
                    "[transport] all %d push-chain links settled; "
                    "starting synchronized payload push",
                    len(push_names),
                )
                return False
            self._publish_transport_launch_hold(
                namespaces, targets, expected_epoch
            )
            return True

        if all_connected:
            ready_since = getattr(
                self, 'transport_engagement_ready_since', None
            )
            if ready_since is None or now < ready_since:
                self.transport_engagement_ready_since = now
                ready_since = now
            hold_time = getattr(
                self,
                'transport_engagement_hold_time',
                DEFAULT_ENGAGEMENT_HOLD_TIME,
            )
            if now - ready_since >= hold_time:
                self.transport_engagement_complete = True
                self.transport_engagement_ready_since = None
                self.transport_push_ramp_started_at = now
                self.transport_push_reference_speed = getattr(
                    self, 'transport_push_ramp_initial_speed', 0.018
                )
                self.transport_push_ramp_batches = 0
                self.transport_useful_contributors = set()
                self.transport_current_useful_pushers = set()
                self.transport_all_pushers_confirmed = False
                self.transport_all_pushers_since = None
                self.transport_launch_settle_started_at = now
                self._publish_transport_launch_hold(
                    namespaces, targets, expected_epoch
                )
                rospy.loginfo(
                    "[transport] all %d push-chain links engaged; "
                    "settling approach momentum before synchronized push",
                    len(push_names),
                )
                return True
        else:
            self.transport_engagement_ready_since = None
            self.transport_launch_settle_started_at = None

        if docking_started and not local_queues_ready:
            # Stop both lane prefixes and close only the first open link in
            # each queue. This remains safe after one lead touches the box:
            # the lead and every verified prefix are passive brakes, while a
            # single frontier closes at the low assembly speed. Re-parking a
            # queue that is already touching its companions makes the lead
            # reverse into its child and deadlocks the robot-distance guard.
            # The frontier controller below resumes synchronized docking only
            # after both chains are connected again.
            open_contacts = sorted(
                namespace
                for namespace, target in targets.items()
                if (
                    target['role'] == 'companion_push'
                    and namespace not in physical_contact_ready
                )
            )
            open_headings = sorted(
                namespace
                for namespace in push_names
                if namespace not in heading_ready_names
            )
            rospy.logwarn_throttle(
                2.0,
                "[transport] an internal queue link opened during docking; "
                "holding both lanes while it closes "
                "(contacts=%s headings=%s parallel_rows=%s)",
                open_contacts,
                open_headings,
                parallel_rows_ready,
            )
        if not docking_started and local_queues_ready and not roots_staged:
            return self._return_transport_queues_to_staging(
                namespaces,
                targets,
                expected_epoch,
                "a queue lead left its staging anchor before docking",
            )
        if queues_ready and not docking_started:
            settled_at = getattr(
                self, 'transport_queue_settle_started_at', None
            )
            if settled_at is None or now < settled_at:
                self.transport_queue_settle_started_at = now
            if not self._transport_queue_is_settled(namespaces, now):
                self._publish_transport_launch_hold(
                    namespaces, targets, expected_epoch
                )
                return True
            self.transport_queue_settle_started_at = None
            self.transport_queue_docking_started = True
            docking_started = True
            rospy.loginfo(
                "[transport] both off-payload queues settled; "
                "docking the complete lanes"
            )
        elif not local_queues_ready:
            self.transport_queue_settle_started_at = None

        self.transport_engagement_complete = False
        if was_complete:
            rospy.logwarn_throttle(
                2.0,
                "[transport] a push-chain contact opened; "
                "restoring the connected formation",
            )

        (
            contact_neighbours,
            shielded_neighbours,
            _row_neighbours,
        ) = self._transport_neighbours(targets)

        # Repair one local queue link per lane while every verified prefix is
        # held. Once every internal link is ready, translate each complete
        # queue as a unit. A lane that reaches the payload first stops as a
        # unit, so it cannot walk the box away from the other lane.
        frontier_names = set()
        advancing_chain_indices = set()
        chain_indices = {
            target.get('chain_index') for target in targets.values()
        }
        for chain_index in sorted(chain_indices):
            chain = sorted(
                (
                    namespace for namespace, target in targets.items()
                    if target.get('chain_index') == chain_index
                ),
                key=lambda namespace: int(
                    targets[namespace].get('chain_depth', 0)
                ),
            )
            if queues_ready:
                payload_lead = next((
                    namespace for namespace in chain
                    if targets[namespace]['role'] == 'payload_push'
                ), None)
                if payload_lead not in physical_contact_ready:
                    advancing_chain_indices.add(chain_index)
                continue
            for namespace in chain:
                if namespace in queue_connected:
                    continue
                parent_namespace = targets[namespace].get(
                    'parent_namespace'
                )
                if (
                    parent_namespace is None
                    or parent_namespace in queue_connected
                ):
                    frontier_names.add(namespace)
                break

        planned_commands = {}
        for namespace in namespaces:
            target = targets.get(namespace)
            if target is None:
                with self.data_lock:
                    avoidance = getattr(
                        self, 'avoidance_modules', {}
                    ).get(namespace)
                if avoidance is not None:
                    avoidance.reset_motion()
                planned_commands[namespace] = Twist()
                continue

            if namespace not in heading_ready_names and (
                namespace in physical_contact_ready
                or target['role'] == 'payload_push'
            ):
                # A Burger is circular in plan view. Once its bumper is on the
                # intended surface, clear any remembered linear command and
                # turn at the same centre until it faces the common force.
                # Pose correction while turning was slowly walking an
                # otherwise complete N=10 chain—and the payload—around an arc.
                with self.data_lock:
                    avoidance = getattr(
                        self, 'avoidance_modules', {}
                    ).get(namespace)
                if avoidance is not None:
                    avoidance.reset_motion()
                planned_commands[namespace] = (
                    self._push_alignment_command(
                        yaws[namespace], target['push_direction']
                    )
                )
                continue

            queue_advancing = (
                queues_ready
                and target.get('chain_index') in advancing_chain_indices
            )
            if not queue_advancing and (
                namespace in queue_connected
                or namespace not in frontier_names
            ):
                # Verified local prefixes and deeper waiting rows are passive
                # brakes. Only a repair frontier or a complete moving queue
                # may add assembly force.
                with self.data_lock:
                    avoidance = getattr(
                        self, 'avoidance_modules', {}
                    ).get(namespace)
                if avoidance is not None:
                    avoidance.reset_motion()
                planned_commands[namespace] = Twist()
                continue

            if target['role'] == 'payload_push':
                destination = target['position']
                contact_speed = getattr(
                    self, 'transport_assembly_closing_speed', 0.012
                )
                forward_limit = contact_speed
                reverse_limit = contact_speed
                command = self._chain_pose_hold_command(
                    positions[namespace],
                    yaws[namespace],
                    destination,
                    target['push_direction'],
                    forward_limit=forward_limit,
                    reverse_limit=reverse_limit,
                )
                allowed_contacts = tuple(sorted(
                    contact_neighbours.get(namespace, ())
                ))
                body_contacts = set(self._nearby_chain_contacts(
                    namespace,
                    contact_neighbours,
                    positions,
                    targets,
                ))
                parallel_contacts = tuple(sorted(
                    self._parallel_lane_contacts(
                        namespace, positions, targets
                    )
                ))
                other_lidar_contacts = set(parallel_contacts)
                other_lidar_contacts.update(
                    set(allowed_contacts) - body_contacts
                )
                command = self._apply_transport_avoidance(
                    namespace,
                    command,
                    obj_pos,
                    allow_payload_contact=True,
                    payload_contact_confirmed=self._payload_contact_near(
                        positions[namespace], obj_pos, obj_yaw,
                        margin=0.03,
                    ),
                    allowed_contact_namespaces=allowed_contacts,
                    repulsion_exempt_namespaces=parallel_contacts,
                    parallel_motion_exempt_namespaces=parallel_contacts,
                    soft_steering=False,
                    # ModelStates and LaserScan are separate streams. At 3x
                    # real time their payload surfaces can differ by a few
                    # centimetres, but only a complete, aligned queue that is
                    # actively advancing may use the narrow docking margin.
                    payload_docking=(
                        docking_started
                        and queue_advancing
                        and float(command.linear.x) > 0.0
                    ),
                    additional_lidar_masks=(
                        self._transport_robot_lidar_masks(
                            body_contacts,
                            other_lidar_contacts,
                            positions,
                            yaws,
                        )
                    ),
                    object_yaw=obj_yaw,
                )
                planned_commands[namespace] = command
                continue

            parent_namespace = target['parent_namespace']
            destination = companion_close_targets[namespace]
            if queue_advancing:
                push_direction = np.asarray(
                    target['push_direction'], dtype=float
                )
                direction_norm = float(np.linalg.norm(push_direction))
                if direction_norm > 1e-9:
                    destination = (
                        np.asarray(positions[namespace], dtype=float)
                        + push_direction / direction_norm * 0.10
                    )
            contact_speed = getattr(
                self, 'transport_assembly_closing_speed', 0.012
            )
            forward_limit = contact_speed
            reverse_limit = contact_speed
            command = self._chain_pose_hold_command(
                positions[namespace],
                yaws[namespace],
                destination,
                target['push_direction'],
                forward_limit=forward_limit,
                reverse_limit=reverse_limit,
            )
            allowed_contacts = tuple(sorted(
                set(contact_neighbours.get(namespace, ()))
                | {parent_namespace}
            ))
            body_contacts = set(self._nearby_chain_contacts(
                namespace,
                contact_neighbours,
                positions,
                targets,
            ))
            shielded_contacts = tuple(sorted(
                self._nearby_chain_contacts(
                    namespace,
                    shielded_neighbours,
                    positions,
                    targets,
                )
            ))
            parallel_contacts = tuple(sorted(
                self._parallel_lane_contacts(
                    namespace, positions, targets
                )
            ))
            # Once the columns are laterally separated, robots in the other
            # column share the same motion corridor even when their depths
            # differ. The explicit centre-distance guard still takes over if
            # the measured lane clearance collapses.
            repulsion_exempt = tuple(sorted(
                set(shielded_contacts) | set(parallel_contacts)
            ))
            other_lidar_contacts = tuple(sorted(
                set(shielded_contacts)
                | set(parallel_contacts)
                | (set(allowed_contacts) - body_contacts)
            ))
            command = self._apply_transport_avoidance(
                namespace,
                command,
                obj_pos,
                # The live parent is the stopping surface for this move.
                # Payload returns visible around it must not taper the final
                # few millimetres below the Burger's useful closing speed.
                allow_payload_contact=True,
                allowed_contact_position=target['parent_position'],
                allowed_contact_namespace=parent_namespace,
                allowed_contact_namespaces=allowed_contacts,
                repulsion_exempt_namespaces=repulsion_exempt,
                parallel_motion_exempt_namespaces=parallel_contacts,
                shielded_motion_exempt_namespaces=shielded_contacts,
                soft_steering=False,
                additional_lidar_masks=self._transport_robot_lidar_masks(
                    body_contacts,
                    other_lidar_contacts,
                    positions,
                    yaws,
                ),
                object_yaw=obj_yaw,
            )
            planned_commands[namespace] = command

        for chain_index in advancing_chain_indices:
            lane_names = [
                namespace
                for namespace, target in targets.items()
                if target.get('chain_index') == chain_index
            ]
            lane_speed = min(
                max(
                    0.0,
                    float(planned_commands.get(
                        namespace, Twist()
                    ).linear.x),
                )
                for namespace in lane_names
            )
            for namespace in lane_names:
                command = planned_commands.get(namespace)
                if command is not None:
                    # Dock as one nonnegative train. If any member has to
                    # stop or retreat, the whole lane stops; leaving that one
                    # negative while only zeroing its peers would reverse it
                    # into the child behind it.
                    command.linear.x = lane_speed

        publish_order = sorted(
            namespaces,
            key=lambda namespace: (
                -int(targets.get(namespace, {}).get('chain_depth', -1)),
                int(targets.get(namespace, {}).get('chain_index', 0)),
                namespace,
            ),
        )
        for namespace in publish_order:
            command = planned_commands.get(namespace, Twist())
            if not self._publish_command(
                namespace,
                command,
                expected_epoch,
            ):
                break
            with self.data_lock:
                avoidance = getattr(
                    self, 'avoidance_modules', {}
                ).get(namespace)
            commit_command = getattr(
                avoidance, 'commit_published_command', None
            )
            if callable(commit_command):
                commit_command(command)
        return True

    def _push_phase(self, expected_epoch):
        """
        PUSH: GRF-based coordinated pushing using MCMC velocity sampling.
        Transition to DONE when the object is within arrival_tolerance
        of the target.
        """
        with self.model_lock:
            obj_pos = (
                self.object_position.copy()
                if self.object_position is not None else None
            )
            obj_yaw = self.object_yaw
            object_error = self.object_error
            obstacles = [
                point.copy() for point in self.obstacle_positions
            ]

        if obj_pos is None:
            reason = object_error or "Transport payload was lost"
            self._fail_transport(reason, expected_epoch)
            return

        target_pos = np.array([self.target_x, self.target_y])
        dist_to_target = float(np.linalg.norm(obj_pos - target_pos))
        if (
            self.transport_synchronized_push_started
            and dist_to_target < self.arrival_tolerance
            and not getattr(self, 'transport_arrival_latched', False)
        ):
            difference = target_pos - obj_pos
            if dist_to_target > 1e-9:
                arrival_direction = difference / dist_to_target
            else:
                arrival_direction = self._transport_frame(obj_pos)[0]
            with self.command_lock:
                if not self._command_allowed(expected_epoch):
                    return
                if not getattr(
                    self, 'transport_arrival_latched', False
                ):
                    self.transport_arrival_direction = (
                        np.asarray(arrival_direction, dtype=float).copy()
                    )
                    self.transport_arrival_latched = True
                    rospy.loginfo(
                        "[transport] payload entered the target envelope; "
                        "holding the delivery direction for final contact"
                    )
        with self.data_lock:
            namespaces = list(self.robot_namespaces)
            positions = {ns: self.robot_positions[ns].copy()
                         for ns in namespaces if ns in self.robot_positions}
            yaws = {ns: self.robot_yaws[ns]
                    for ns in namespaces if ns in self.robot_yaws}

        if self._engage_push_chains(
            namespaces, positions, yaws, obj_pos, obj_yaw, expected_epoch
        ):
            # A large fleet can spend well over the normal progress timeout
            # closing its chain from front to back. The delivery watchdog
            # starts when the final physical contact is confirmed, not when
            # APPROACH first hands control to PUSH.
            if getattr(
                self, 'transport_engagement_complete', False
            ):
                self.transport_best_distance = dist_to_target
                get_time = getattr(rospy, 'get_time', lambda: 0.0)
                self.transport_last_progress_time = float(get_time())
            return

        # Delivery belongs to the complete fleet, not merely to whichever
        # payload leads happened to remain attached.  A link that opens near
        # the goal first gets the same low-speed recovery used mid-route; the
        # task finishes only after every robot is connected and has issued a
        # useful synchronized push command at least once.
        full_chain_connected = bool(namespaces) and set(namespaces) <= set(
            getattr(
                self,
                'transport_physical_engaged',
                getattr(self, 'transport_engaged', set()),
            )
        )
        full_fleet_contributed = bool(getattr(
            self, 'transport_all_pushers_confirmed', False
        ))
        current_full_fleet_push = bool(namespaces) and set(namespaces) <= set(
            getattr(self, 'transport_current_useful_pushers', set())
        )
        arrival_distance_safe = dist_to_target <= (
            self.arrival_tolerance
            + getattr(self, 'transport_arrival_release_margin', 0.10)
        )
        if (
            self.transport_synchronized_push_started
            and full_chain_connected
            and full_fleet_contributed
            and current_full_fleet_push
            and getattr(self, 'transport_arrival_latched', False)
            and arrival_distance_safe
        ):
            self._complete_transport(dist_to_target, expected_epoch)
            return

        recovering_at_target = (
            self.transport_synchronized_push_started
            and not full_chain_connected
            and getattr(self, 'transport_arrival_latched', False)
        )
        terminal_contact_pending = (
            self.transport_synchronized_push_started
            and getattr(self, 'transport_arrival_latched', False)
        )
        if terminal_contact_pending:
            if not arrival_distance_safe:
                self._fail_transport(
                    "The payload left the target envelope while the fleet "
                    "was restoring final contact.",
                    expected_epoch,
                )
                return
            get_time = getattr(rospy, 'get_time', lambda: 0.0)
            now = float(get_time())
            recovery_started = getattr(
                self, 'transport_near_target_recovery_started_at', None
            )
            if (
                recovery_started is None
                or not math.isfinite(recovery_started)
                or now < recovery_started
            ):
                self.transport_near_target_recovery_started_at = now
            elif (
                math.isfinite(now)
                and now - recovery_started
                >= self.transport_progress_timeout
            ):
                self._fail_transport(
                    "The payload reached its target, but the complete fleet "
                    "could not finish a safe synchronized contact.",
                    expected_epoch,
                )
                return
        else:
            self.transport_near_target_recovery_started_at = None
        if (
            not getattr(self, 'transport_arrival_latched', False)
            and self._reposition_if_stalled(
                dist_to_target, expected_epoch
            )
        ):
            return

        if self.transport_planner == 'legacy':
            self._legacy_push_step(
                namespaces,
                positions,
                yaws,
                obj_pos,
                obj_yaw,
                expected_epoch,
            )
            return

        try:
            commands, grf_positions, grf_yaws = self._grf_push_commands(
                namespaces, obj_pos, obj_yaw, obstacles, expected_epoch
            )
        except Exception as exc:
            rospy.logwarn_throttle(
                5.0,
                "[transport] GRF planner failed ({}); "
                "using coordinated goal commands for this step".format(exc),
            )
            commands = {}
            grf_positions = positions
            grf_yaws = yaws
            with self.command_lock:
                if not self._command_allowed(expected_epoch):
                    return
                self._active_planner = 'coordinated_fallback'
                self._active_grf_iterations = 0

        self._publish_grf_commands(
            namespaces, commands, grf_positions, grf_yaws,
            obj_pos, obj_yaw, expected_epoch
        )

    # ======================================================================
    # 10 Hz control loop
    # ======================================================================

    def _control_loop(self, event):
        with self._control_cycle_mutex():
            try:
                self._control_loop_serialized(event)
            finally:
                # An emergency stop deliberately does not wait for this mutex.
                # Finish its phase-state cleanup once any in-flight pass exits.
                self._finalize_emergency_reset()

    def _control_loop_serialized(self, event):
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
            namespaces = list(self.robot_namespaces)
            positions = {
                namespace: self.robot_positions[namespace].copy()
                for namespace in namespaces
                if namespace in self.robot_positions
            }
            yaws = {
                namespace: self.robot_yaws[namespace]
                for namespace in namespaces
                if namespace in self.robot_yaws
            }
            odom_received_at = dict(getattr(
                self, 'robot_odom_received_at', {}
            ))

        input_error = self._transport_model_state_error()
        if input_error is None:
            input_error = self._transport_odometry_error(
                namespaces, positions, yaws, odom_received_at
            )

        with self.phase_lock:
            phase = self.phase

        self._ensure_transport_collision_stream()
        get_sim_time = getattr(rospy, 'get_time', lambda: 0.0)
        try:
            sim_time = float(get_sim_time())
        except (TypeError, ValueError, OverflowError):
            sim_time = -1.0
        self.transport_safety_control_sequence += 1
        self.transport_active_safety_context = {
            'task_id': expected_task_id,
            'task_phase': phase.value,
            'control_sequence': self.transport_safety_control_sequence,
            'sim_time': sim_time,
            'wall_time': time.monotonic(),
        }

        try:
            if input_error is not None:
                self._fail_transport(input_error, expected_epoch)
            else:
                self._sync_avoidance_snapshot(positions, yaws)
                if phase == TransportPhase.SEARCH:
                    self._search_phase(expected_epoch)
                elif phase == TransportPhase.APPROACH:
                    self._approach_phase(expected_epoch)
                elif phase == TransportPhase.PUSH:
                    self._push_phase(expected_epoch)
                elif phase in (TransportPhase.DONE, TransportPhase.FAILED):
                    pass  # remain idle
                elif phase == TransportPhase.IDLE:
                    pass
        finally:
            self.transport_active_safety_context = None

        with self.phase_lock:
            phase = self.phase

        with self.command_lock:
            if self.current_task_id != expected_task_id:
                return
            if phase in (TransportPhase.DONE, TransportPhase.FAILED):
                with self.phase_lock:
                    live_phase = self.phase
                if (
                    live_phase != phase
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
            object_position = (
                self.object_position.copy()
                if self.object_position is not None else None
            )
            marker_position = getattr(
                self, 'target_marker_position', None
            )
            if marker_position is not None:
                marker_position = marker_position.copy()
            marker_synced = bool(getattr(
                self, 'target_marker_synced', False
            ))
        obj = object_position.tolist() if object_position is not None else [0, 0]

        target = [self.target_x, self.target_y]
        dist = math.sqrt((obj[0] - target[0]) ** 2 + (obj[1] - target[1]) ** 2)
        progress = self._transport_progress(
            dist if object_position is not None else None
        )

        # Build per-robot assignment info
        assignments = {}
        transport_roles = getattr(self, 'transport_roles', {})
        role_owner = {
            (role['chain_index'], role['depth']): namespace
            for namespace, role in transport_roles.items()
            if isinstance(role, dict)
            and 'chain_index' in role
            and 'depth' in role
        }
        rendezvoused = set(getattr(
            self, 'transport_pre_staged', set()
        ))
        compression_progress = max(0.0, min(
            1.0,
            float(getattr(
                self, 'transport_compression_progress', 0.0
            )),
        ))
        if phase != TransportPhase.APPROACH:
            approach_stage = None
        elif len(rendezvoused) < len(self.robot_namespaces):
            approach_stage = 'rendezvous'
        elif compression_progress < 0.5:
            approach_stage = 'lane_alignment'
        elif compression_progress < 1.0:
            approach_stage = 'lane_compression'
        else:
            approach_stage = 'contact_alignment'
        with self.data_lock:
            for ns in self.robot_namespaces:
                pos = self.robot_positions.get(ns, np.zeros(2))
                assignment = {
                    'x': round(float(pos[0]), 3),
                    'y': round(float(pos[1]), 3),
                    'yaw': round(float(
                        self.robot_yaws.get(ns, 0.0)
                    ), 3),
                }
                last_command = getattr(
                    self, 'transport_last_commands', {}
                ).get(ns)
                if last_command is not None:
                    assignment['command'] = {
                        'linear': round(float(last_command[0]), 4),
                        'angular': round(float(last_command[1]), 4),
                    }
                discovery = getattr(self, 'transport_discovery', None)
                if phase == TransportPhase.SEARCH:
                    assignment['activity'] = 'searching'
                    route = getattr(self, 'search_routes', {}).get(ns, ())
                    if route:
                        route_index = getattr(
                            self, 'search_route_indices', {}
                        ).get(ns, 0)
                        route_index = max(
                            0, min(route_index, len(route) - 1)
                        )
                        coverage_target = route[route_index]
                        search_target = getattr(
                            self, 'search_navigation_targets', {}
                        ).get(ns, coverage_target)
                        assignment['search_target'] = {
                            'x': round(float(search_target[0]), 3),
                            'y': round(float(search_target[1]), 3),
                        }
                        if float(np.linalg.norm(
                            np.asarray(search_target)
                            - np.asarray(coverage_target)
                        )) > 1e-6:
                            assignment['search_coverage_target'] = {
                                'x': round(float(coverage_target[0]), 3),
                                'y': round(float(coverage_target[1]), 3),
                            }
                elif discovery is not None:
                    if phase == TransportPhase.APPROACH:
                        if ns not in rendezvoused:
                            assignment['activity'] = 'rendezvousing'
                        elif ns in getattr(
                            self, 'transport_staged', set()
                        ):
                            assignment['activity'] = 'ready_to_push'
                        else:
                            assignment['activity'] = 'forming_push_lane'
                    elif phase == TransportPhase.PUSH:
                        assignment['activity'] = 'pushing_payload'
                    elif phase == TransportPhase.DONE:
                        assignment['activity'] = 'delivery_complete'
                    else:
                        assignment['activity'] = 'discovery_acknowledged'
                    assignment['notice_received'] = (
                        discovery.get('finder') == ns
                        or ns in discovery.get('notified_robots', ())
                    )
                    assignment['finder'] = (
                        discovery.get('finder') == ns
                    )
                role = transport_roles.get(ns)
                if isinstance(role, dict):
                    chain_index = role.get('chain_index')
                    depth = role.get('depth')
                    parent_namespace = None
                    if isinstance(depth, int) and depth > 0:
                        parent_namespace = role_owner.get((
                            chain_index, depth - 1
                        ))
                    assignment.update({
                        'role': role.get('role'),
                        'chain_index': chain_index,
                        'chain_depth': depth,
                        'parent_namespace': parent_namespace,
                        'staged': ns in getattr(
                            self, 'transport_staged', set()
                        ),
                        'pre_staged': ns in getattr(
                            self, 'transport_pre_staged', set()
                        ),
                        'rendezvous_ready': ns in rendezvoused,
                        'assembly_released': ns in getattr(
                            self, 'transport_chain_released', set()
                        ),
                        'engaged': ns in getattr(
                            self, 'transport_engaged', set()
                        ),
                        'physical_engaged': ns in getattr(
                            self, 'transport_physical_engaged', set()
                        ),
                        'aligned_engaged': ns in getattr(
                            self, 'transport_aligned_engaged', set()
                        ),
                    })
                    staging_position = role.get('staging_position')
                    if staging_position is not None:
                        staging_position = np.asarray(
                            staging_position, dtype=float
                        )
                        if staging_position.shape == (2,):
                            assignment['staging_target'] = {
                                'x': round(float(staging_position[0]), 3),
                                'y': round(float(staging_position[1]), 3),
                            }
                assignments[ns] = assignment

        assembly_routes = {}
        route_states = getattr(
            self, 'transport_assembly_route_states', {}
        )
        if isinstance(route_states, dict):
            for namespace in sort_robot_ids(route_states):
                state = route_states.get(namespace)
                if not isinstance(state, dict):
                    continue
                route_status = {
                    'waypoint_index': int(state.get(
                        'waypoint_index', 0
                    )),
                    'waypoint_count': len(state.get('waypoints', ())),
                    'complete': bool(state.get('complete', False)),
                    'reversing': bool(state.get(
                        'reverse_active', False
                    )),
                    'turning_for_handoff': bool(state.get(
                        'turn_direction', 0.0
                    )),
                }
                route_target = state.get('target')
                if route_target is not None:
                    route_target = np.asarray(
                        route_target, dtype=float
                    )
                    if route_target.shape == (2,):
                        route_status['target'] = {
                            'x': round(float(route_target[0]), 3),
                            'y': round(float(route_target[1]), 3),
                        }
                assembly_routes[namespace] = route_status

        control_commands = {
            namespace: {
                'linear': float(command[0]),
                'angular': float(command[1]),
            }
            for namespace, command in getattr(
                self, 'transport_last_control_commands', {}
            ).items()
        }
        push_link_states = {}
        for namespace, state in getattr(
            self, 'transport_push_link_states', {}
        ).items():
            if state is None:
                continue
            push_link_states[namespace] = {
                'state': state[0],
                'gap': float(state[1]),
                'error': float(state[2]),
            }
        push_arbitration = {
            'raw_linear': {
                namespace: float(speed)
                for namespace, speed in getattr(
                    self, 'transport_push_raw_speeds', {}
                ).items()
            },
            'coordinated_linear': {
                namespace: float(speed)
                for namespace, speed in getattr(
                    self, 'transport_push_coordinated_speeds', {}
                ).items()
            },
            'links': push_link_states,
            'hard_stop_sources': list(getattr(
                self, 'transport_push_hard_stop_sources', ()
            )),
        }
        queue_docking_started = bool(getattr(
            self, 'transport_queue_docking_started', False
        ))
        queue_settling = (
            not queue_docking_started
            and getattr(
                self, 'transport_queue_settle_started_at', None
            ) is not None
        )

        status_task_id = (
            self.current_task_id if task_id is None else task_id
        )
        status = {
            'task_id': status_task_id,
            'paused': self.is_paused if paused is None else paused,
            'phase': phase.value,
            'collision_events': self._transport_collision_stream_snapshot(
                status_task_id,
                terminal=phase in (
                    TransportPhase.DONE, TransportPhase.FAILED
                ),
            ),
            'object_pos': {'x': round(obj[0], 3), 'y': round(obj[1], 3)},
            'target_pos': {'x': target[0], 'y': target[1]},
            'arrival_tolerance': round(float(getattr(
                self, 'arrival_tolerance', 0.5
            )), 3),
            'target_marker': {
                'model_name': getattr(
                    self, 'target_marker_name', 'target_marker'
                ),
                'command_published': bool(getattr(
                    self, 'target_marker_command_published', False
                )),
                'synchronized': marker_synced,
                'position': (
                    None if marker_position is None else {
                        'x': round(float(marker_position[0]), 3),
                        'y': round(float(marker_position[1]), 3),
                    }
                ),
            },
            'distance_to_target': round(dist, 3),
            'progress': round(progress, 3),
            'planner': self._active_planner,
            'grf_mcmc_iterations': self._active_grf_iterations,
            'engagement_complete': getattr(
                self, 'transport_engagement_complete', False
            ),
            'synchronized_push_started': getattr(
                self, 'transport_synchronized_push_started', False
            ),
            'control_sequence': getattr(
                self, 'transport_control_sequence', 0
            ),
            'control_sim_time': getattr(
                self, 'transport_last_control_sim_time', None
            ),
            'control_commands': control_commands,
            'batch_publish_span_s': round(float(getattr(
                self, 'transport_last_batch_publish_span', 0.0
            )), 4),
            'push_arbitration': push_arbitration,
            'push_reference_speed': (
                None
                if getattr(
                    self, 'transport_push_reference_speed', None
                ) is None
                else round(float(
                    self.transport_push_reference_speed
                ), 4)
            ),
            'useful_contributor_count': len(getattr(
                self, 'transport_useful_contributors', set()
            )),
            'useful_contributor_ids': sort_robot_ids(getattr(
                self, 'transport_useful_contributors', set()
            )),
            'current_useful_pusher_count': len(getattr(
                self, 'transport_current_useful_pushers', set()
            )),
            'current_useful_pusher_ids': sort_robot_ids(getattr(
                self, 'transport_current_useful_pushers', set()
            )),
            'all_pusher_proof_minimum_speed': round(
                self._transport_all_pusher_proof_speed(), 4
            ),
            'all_pushers_confirmed': bool(getattr(
                self, 'transport_all_pushers_confirmed', False
            )),
            'arrival_latched': bool(getattr(
                self, 'transport_arrival_latched', False
            )),
            'route_robot': getattr(
                self, 'transport_route_namespace', None
            ),
            'route_kind': getattr(
                self, 'transport_route_kind', None
            ),
            'route_complete': getattr(
                self, 'transport_route_complete', False
            ),
            'queue_docking_started': queue_docking_started,
            'queue_settling': queue_settling,
            'approach_stage': approach_stage,
            'rendezvous_ready_count': len(
                rendezvoused & set(self.robot_namespaces)
            ),
            'compression_progress': round(compression_progress, 4),
            'approach_routes': assembly_routes,
            'assembly_routes': assembly_routes,
            'searching_robot_count': (
                len(self.robot_namespaces)
                if phase == TransportPhase.SEARCH else 0
            ),
            'discovery': getattr(self, 'transport_discovery', None),
            'robot_assignments': assignments,
        }
        route_target = getattr(self, 'transport_route_target', None)
        if route_target is not None:
            route_target = np.asarray(route_target, dtype=float)
            if route_target.shape == (2,):
                status['route_target'] = {
                    'x': round(float(route_target[0]), 3),
                    'y': round(float(route_target[1]), 3),
                }
        if phase == TransportPhase.FAILED:
            status['error'] = (
                getattr(self, 'failure_reason', None)
                or
                getattr(self, 'object_error', None)
                or 'Transport task failed'
            )
        object_z = getattr(self, 'object_z', None)
        if object_z is not None:
            status['object_z'] = round(object_z, 3)
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
