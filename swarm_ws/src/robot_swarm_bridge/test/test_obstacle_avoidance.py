#!/usr/bin/env python3

import math
import threading
import unittest

import numpy as np

from test_formation_control import formation, make_closed_loop_controller
from test_ros_lifecycle import LaserScan, Point, ROS, Twist


BaseObstacleAvoidance = ROS['obstacle'].ObstacleAvoidance
LidarRangeMask = ROS['obstacle'].LidarRangeMask


class ObstacleAvoidance(BaseObstacleAvoidance):
    """Give ordinary motion tests the fresh clear scan their fixture implies."""

    def __init__(self, robot_name):
        super().__init__(robot_name)
        self._scan_received_at = self._clock()


def forward_command(speed=0.15):
    command = Twist()
    command.linear.x = speed
    return command


class ObstacleAvoidanceMotionTests(unittest.TestCase):
    def make_avoidance(self):
        avoidance = ObstacleAvoidance('tb3_0')
        # Most unit cases use a sensor at the base origin.  Transport-specific
        # coverage below exercises Burger's real rearward LDS offset.
        avoidance.lidar_offset_x = 0.0
        avoidance.lidar_offset_y = 0.0
        avoidance.sector_min = [avoidance.max_valid_range] * 8
        return avoidance

    def test_repulsion_does_not_move_a_robot_with_no_command(self):
        avoidance = self.make_avoidance()
        avoidance.sector_min[0] = 0.3

        safe = avoidance.apply_avoidance(Twist())

        self.assertEqual(safe.linear.x, 0.0)
        self.assertEqual(safe.angular.z, 0.0)

    def test_missing_or_stale_scan_fails_closed(self):
        avoidance = BaseObstacleAvoidance('tb3_0')

        missing = avoidance.apply_avoidance(forward_command())
        self.assertEqual(0.0, missing.linear.x)
        self.assertEqual(1.0, avoidance.get_threat_level())

        avoidance._scan_received_at = (
            avoidance._clock() - avoidance.scan_timeout_wall_s - 0.1
        )
        stale = avoidance.apply_avoidance(forward_command())
        self.assertEqual(0.0, stale.linear.x)
        self.assertEqual(0.0, stale.angular.z)

    def test_centered_obstacle_chooses_one_stable_escape_side(self):
        avoidance = self.make_avoidance()
        avoidance.sector_min[0] = 0.45

        first = avoidance.apply_avoidance(forward_command())
        first_direction = 1 if first.angular.z > 0.0 else -1
        second = avoidance.apply_avoidance(forward_command())
        second_direction = 1 if second.angular.z > 0.0 else -1

        self.assertNotEqual(first.angular.z, 0.0)
        self.assertEqual(first_direction, second_direction)

    def test_soft_steering_can_be_suppressed_without_disabling_stops(self):
        avoidance = self.make_avoidance()
        avoidance.smoothing_alpha = 0.0
        avoidance.max_linear_acceleration = 100.0
        avoidance.max_angular_acceleration = 100.0
        avoidance.sector_min[1] = 0.45
        requested = forward_command(0.10)
        requested.angular.z = 0.40

        precise = avoidance.apply_avoidance(
            requested, soft_steering=False
        )

        self.assertGreater(precise.linear.x, 0.0)
        self.assertAlmostEqual(0.40, precise.angular.z)

        avoidance.reset_motion()
        avoidance.sector_min[0] = 0.10
        stopped = avoidance.apply_avoidance(
            requested, soft_steering=False
        )
        self.assertEqual(0.0, stopped.linear.x)
        self.assertEqual(0.0, stopped.angular.z)

    def test_opt_in_pressure_floor_never_exceeds_raw_safety(self):
        def smoothed_avoidance():
            avoidance = self.make_avoidance()
            avoidance.smoothing_alpha = 0.8
            avoidance.max_linear_acceleration = 100.0
            return avoidance

        loaded = smoothed_avoidance()
        useful = loaded.apply_avoidance(
            forward_command(0.018), minimum_linear_speed=0.015
        )
        self.assertAlmostEqual(0.015, useful.linear.x)

        safety_limited = smoothed_avoidance()
        slower = safety_limited.apply_avoidance(
            forward_command(0.012), minimum_linear_speed=0.015
        )
        self.assertGreaterEqual(slower.linear.x, 0.0)
        self.assertLessEqual(slower.linear.x, 0.012)

        emergency = smoothed_avoidance()
        emergency.sector_min[0] = 0.10
        stopped = emergency.apply_avoidance(
            forward_command(0.018), minimum_linear_speed=0.015
        )
        self.assertEqual(0.0, stopped.linear.x)
        self.assertEqual(0.0, stopped.angular.z)

        ordinary = smoothed_avoidance()
        ema_only = ordinary.apply_avoidance(forward_command(0.018))
        self.assertAlmostEqual(
            (1.0 - ordinary.smoothing_alpha) * 0.018,
            ema_only.linear.x,
        )
        self.assertLess(ema_only.linear.x, 0.015)

    def test_published_fleet_cap_updates_smoothing_state(self):
        avoidance = self.make_avoidance()
        avoidance.smoothing_alpha = 0.0
        avoidance.max_linear_acceleration = 100.0
        avoidance.max_angular_acceleration = 100.0
        safe = avoidance.apply_avoidance(forward_command(0.018))
        self.assertAlmostEqual(0.018, safe.linear.x)

        published = forward_command(0.009)
        published.angular.z = 0.12
        avoidance.commit_published_command(published)

        self.assertAlmostEqual(0.009, avoidance._prev_linear)
        self.assertAlmostEqual(0.12, avoidance._prev_angular)

    def test_contact_mask_ignores_only_requested_lidar_sector(self):
        avoidance = self.make_avoidance()
        avoidance.max_linear_acceleration = 100.0
        avoidance.sector_min[0] = 0.1

        blocked = avoidance.apply_avoidance(forward_command())
        avoidance._prev_linear = 0.0
        avoidance._prev_angular = 0.0
        allowed = avoidance.apply_avoidance(
            forward_command(),
            ignored_sector_indices=(
                value for value in [0, 0, -1, 8, 'bad']
            ),
        )

        self.assertEqual(blocked.linear.x, 0.0)
        self.assertEqual(blocked.angular.z, 0.0)
        self.assertGreater(allowed.linear.x, 0.0)

    def test_payload_mask_preserves_a_nearer_return_in_the_same_beam(self):
        avoidance = self.make_avoidance()
        avoidance.max_linear_acceleration = 100.0
        avoidance.set_position(0.0, 0.0, 0.0)
        mask = LidarRangeMask(
            center_x=0.42,
            center_y=0.0,
            radius=0.20,
            maximum_center_distance=1.0,
            tolerance=0.08,
        )

        payload_scan = LaserScan()
        payload_scan.ranges = [0.22]
        payload_scan.angle_min = 0.0
        payload_scan.angle_increment = 0.0
        avoidance._scan_callback(payload_scan)
        allowed = avoidance.apply_avoidance(
            forward_command(), lidar_range_mask=mask
        )
        self.assertGreater(allowed.linear.x, 0.0)
        self.assertEqual(0.0, avoidance._threat_pub.messages[-1].data)

        # Four centimetres closer used to be swallowed by the sector-wide
        # payload tolerance.  It is an emergency return and must remain.
        wall_scan = LaserScan()
        wall_scan.ranges = [0.18]
        wall_scan.angle_min = 0.0
        wall_scan.angle_increment = 0.0
        avoidance._scan_callback(wall_scan)
        blocked = avoidance.apply_avoidance(
            forward_command(), lidar_range_mask=mask
        )
        self.assertEqual(0.0, blocked.linear.x)
        self.assertEqual(1.0, avoidance._threat_pub.messages[-1].data)

    def test_payload_surface_and_nearer_wall_share_a_sector_safely(self):
        avoidance = self.make_avoidance()
        avoidance.set_position(0.0, 0.0, 0.0)
        scan = LaserScan()
        scan.ranges = [0.22, 0.18]
        scan.angle_min = 0.0
        scan.angle_increment = 0.05
        avoidance._scan_callback(scan)

        distances = avoidance._sector_distances(lidar_range_mask=LidarRangeMask(
            center_x=0.42,
            center_y=0.0,
            radius=0.20,
            maximum_center_distance=1.0,
        ))

        self.assertAlmostEqual(0.18, distances[0])

    def test_embedded_helper_publishes_threat_and_contact_state(self):
        avoidance = self.make_avoidance()
        scan = LaserScan()
        scan.ranges = [0.10]
        scan.angle_min = 0.0
        scan.angle_increment = 0.0
        avoidance._scan_callback(scan)

        avoidance.apply_avoidance(forward_command())

        self.assertEqual(1.0, avoidance._threat_pub.messages[-1].data)
        self.assertTrue(avoidance._collision_pub.messages[-1].data)

    def test_sector_mask_never_bypasses_robot_clearance(self):
        avoidance = self.make_avoidance()
        avoidance.set_position(0.0, 0.0, 0.0)
        avoidance.update_robot_positions([
            ('tb3_0', Point(x=0.0, y=0.0)),
            ('tb3_1', Point(x=0.25, y=0.0)),
        ])

        safe = avoidance.apply_avoidance(
            forward_command(), ignored_sector_indices=range(8)
        )

        self.assertEqual(safe.linear.x, 0.0)
        self.assertEqual(safe.angular.z, 0.0)

    def test_close_lidar_returns_are_kept_for_emergency_stop(self):
        avoidance = self.make_avoidance()
        scan = LaserScan()
        scan.ranges = [0.08]
        scan.angle_min = 0.0
        scan.angle_increment = 0.0

        avoidance._scan_callback(scan)
        safe = avoidance.apply_avoidance(forward_command())

        self.assertAlmostEqual(avoidance.sector_min[0], 0.08)
        self.assertEqual(safe.linear.x, 0.0)
        self.assertEqual(safe.angular.z, 0.0)

    def test_head_on_neighbour_reduces_speed_before_emergency_distance(self):
        clear = self.make_avoidance()
        clear.max_linear_acceleration = 100.0
        clear_speed = clear.apply_avoidance(forward_command(0.2)).linear.x

        yielding = self.make_avoidance()
        yielding.max_linear_acceleration = 100.0
        yielding.set_position(0.0, 0.0, 0.0)
        yielding.update_robot_positions([
            ('tb3_0', Point(x=0.0, y=0.0)),
            ('tb3_1', Point(x=0.4, y=0.0)),
        ])
        yielding_speed = yielding.apply_avoidance(
            forward_command(0.2), ignored_sector_indices=range(8)
        ).linear.x

        self.assertGreater(clear_speed, yielding_speed)
        self.assertGreater(yielding_speed, 0.0)

    def test_predictive_braking_stops_before_robot_emergency_distance(self):
        avoidance = self.make_avoidance()
        avoidance.max_linear_acceleration = 0.5
        avoidance.set_position(0.0, 0.0, 0.0)
        avoidance.update_robot_positions([
            ('tb3_0', Point(x=0.0, y=0.0)),
            ('tb3_1', Point(x=0.30, y=0.0)),
        ])

        safe = avoidance.apply_avoidance(forward_command(0.2))

        self.assertEqual(0.0, safe.linear.x)

    def test_robot_behind_does_not_block_motion_that_increases_clearance(self):
        avoidance = self.make_avoidance()
        avoidance.max_linear_acceleration = 100.0
        avoidance.set_position(0.0, 0.0, 0.0)
        avoidance.update_robot_positions([
            ('tb3_0', Point(x=0.0, y=0.0)),
            ('tb3_1', Point(x=-0.25, y=0.0)),
        ])

        safe = avoidance.apply_avoidance(forward_command(0.15))

        self.assertGreater(safe.linear.x, 0.0)
        self.assertEqual(1.0, avoidance._threat_pub.messages[-1].data)
        self.assertFalse(avoidance._collision_pub.messages[-1].data)

    def test_one_designated_robot_can_be_pushed_without_false_safety_events(self):
        avoidance = self.make_avoidance()
        avoidance.max_linear_acceleration = 100.0
        avoidance.set_position(0.0, 0.0, 0.0)
        avoidance.update_robot_positions([
            ('tb3_0', Point(x=0.0, y=0.0)),
            ('tb3_1', Point(x=0.22, y=0.0)),
        ])
        scan = LaserScan()
        scan.ranges = [0.11]
        scan.angle_min = 0.0
        scan.angle_increment = 0.0
        avoidance._scan_callback(scan)

        blocked = avoidance.apply_avoidance(forward_command())
        self.assertEqual(0.0, blocked.linear.x)
        self.assertEqual(1.0, avoidance._threat_pub.messages[-1].data)
        self.assertTrue(avoidance._collision_pub.messages[-1].data)

        allowed = avoidance.apply_avoidance(
            forward_command(), allowed_contact_position=(0.22, 0.0)
        )

        self.assertGreater(allowed.linear.x, 0.0)
        self.assertEqual(0.0, allowed.angular.z)
        self.assertEqual(0.0, avoidance._threat_pub.messages[-1].data)
        self.assertFalse(avoidance._collision_pub.messages[-1].data)

    def test_allowed_contact_excludes_only_one_matching_robot(self):
        avoidance = self.make_avoidance()
        avoidance.max_linear_acceleration = 100.0
        avoidance.set_position(0.0, 0.0, 0.0)
        avoidance.update_robot_positions([
            ('tb3_0', Point(x=0.0, y=0.0)),
            ('tb3_1', Point(x=0.22, y=0.0)),
            ('tb3_2', Point(x=0.22, y=0.0)),
        ])

        safe = avoidance.apply_avoidance(
            forward_command(), allowed_contact_position=(0.22, 0.0)
        )

        self.assertEqual(0.0, safe.linear.x)
        self.assertEqual(1.0, avoidance._threat_pub.messages[-1].data)
        self.assertTrue(avoidance._collision_pub.messages[-1].data)

    def test_named_contact_uses_the_latest_fleet_position(self):
        avoidance = self.make_avoidance()
        avoidance.max_linear_acceleration = 100.0
        avoidance.set_position(0.0, 0.0, 0.0)
        avoidance.update_robot_positions([
            ('tb3_0', Point(x=0.0, y=0.0)),
            ('tb3_1', Point(x=0.22, y=0.0)),
        ])

        safe = avoidance.apply_avoidance(
            forward_command(),
            # This pose is deliberately stale. Identity must resolve it to
            # the current fleet snapshot before the safety exception applies.
            allowed_contact_position=(0.24, 0.0),
            allowed_contact_namespace='tb3_1',
        )

        self.assertGreater(safe.linear.x, 0.0)
        self.assertEqual(0.0, avoidance._threat_pub.messages[-1].data)
        self.assertFalse(avoidance._collision_pub.messages[-1].data)

    def test_allowed_robot_contact_does_not_hide_a_lidar_obstacle(self):
        avoidance = self.make_avoidance()
        avoidance.max_linear_acceleration = 100.0
        avoidance.set_position(0.0, 0.0, 0.0)
        avoidance.update_robot_positions([
            ('tb3_0', Point(x=0.0, y=0.0)),
            ('tb3_1', Point(x=0.22, y=0.0)),
        ])
        scan = LaserScan()
        # The permitted robot surface is expected at 0.11 m.  This closer
        # return cannot belong to it and must remain visible to safety.
        scan.ranges = [0.08]
        scan.angle_min = 0.0
        scan.angle_increment = 0.0
        avoidance._scan_callback(scan)

        safe = avoidance.apply_avoidance(
            forward_command(), allowed_contact_position=(0.22, 0.0)
        )

        self.assertEqual(0.0, safe.linear.x)
        self.assertEqual(1.0, avoidance._threat_pub.messages[-1].data)
        self.assertTrue(avoidance._collision_pub.messages[-1].data)

    def test_declared_contact_accepts_a_realistic_body_return(self):
        avoidance = self.make_avoidance()
        avoidance.max_linear_acceleration = 100.0
        avoidance.set_position(0.0, 0.0, 0.0)
        contact_position = (0.234, 0.0)
        avoidance.update_robot_positions([
            ('tb3_0', Point(x=0.0, y=0.0)),
            ('tb3_parent', Point(x=contact_position[0], y=0.0)),
        ])

        scan = LaserScan()
        scan.angle_min = 0.0
        scan.angle_increment = 0.0
        # The round-body estimate is 0.124 m. Gazebo's mesh can return a
        # point several centimetres behind it, especially on an oblique ray.
        scan.ranges = [0.195]
        avoidance._scan_callback(scan)
        allowed = avoidance.apply_avoidance(
            forward_command(),
            allowed_contact_namespace='tb3_parent',
        )
        self.assertGreater(allowed.linear.x, 0.0)

        avoidance.reset_motion()
        scan.ranges = [0.10]
        avoidance._scan_callback(scan)
        closer_obstacle = avoidance.apply_avoidance(
            forward_command(),
            allowed_contact_namespace='tb3_parent',
        )
        self.assertEqual(0.0, closer_obstacle.linear.x)

    def test_repulsion_exemption_keeps_hard_robot_braking(self):
        avoidance = self.make_avoidance()
        avoidance.max_linear_acceleration = 100.0
        avoidance.set_position(0.0, 0.0, 0.0)
        avoidance.update_robot_positions([
            ('tb3_0', Point(x=0.0, y=0.0)),
            ('tb3_1', Point(x=0.25, y=0.0)),
        ])

        safe = avoidance.apply_avoidance(
            forward_command(),
            repulsion_exempt_namespaces=('tb3_1',),
        )

        self.assertEqual(0.0, safe.linear.x)
        self.assertEqual(1.0, avoidance._threat_pub.messages[-1].data)

    def test_parallel_lane_exemption_still_stops_a_collapsed_lane(self):
        avoidance = self.make_avoidance()
        avoidance.max_linear_acceleration = 100.0
        avoidance.set_position(0.0, 0.0, 0.0)
        avoidance.update_robot_positions([
            ('tb3_0', Point(x=0.0, y=0.0)),
            ('tb3_row_peer', Point(x=0.05, y=0.34)),
        ])

        clear_lane = avoidance.apply_avoidance(
            forward_command(),
            repulsion_exempt_namespaces=('tb3_row_peer',),
            parallel_motion_exempt_namespaces=('tb3_row_peer',),
        )

        # The first command is still softened by the normal acceleration
        # filter, but the row peer must not add any predictive braking.
        self.assertGreater(clear_lane.linear.x, 0.08)

        avoidance.reset_motion()
        avoidance.update_robot_positions([
            ('tb3_0', Point(x=0.0, y=0.0)),
            ('tb3_row_peer', Point(x=0.05, y=0.25)),
        ])
        collapsed_lane = avoidance.apply_avoidance(
            forward_command(),
            repulsion_exempt_namespaces=('tb3_row_peer',),
            parallel_motion_exempt_namespaces=('tb3_row_peer',),
        )

        self.assertEqual(0.0, collapsed_lane.linear.x)
        self.assertEqual(1.0, avoidance._threat_pub.messages[-1].data)

    def test_shielded_chain_skips_prediction_but_keeps_the_hard_stop(self):
        avoidance = self.make_avoidance()
        avoidance.max_linear_acceleration = 100.0
        avoidance.set_position(0.0, 0.0, 0.0)
        avoidance.update_robot_positions([
            ('tb3_0', Point(x=0.0, y=0.0)),
            ('tb3_grandparent', Point(x=0.28, y=0.0)),
        ])

        shielded = avoidance.apply_avoidance(
            forward_command(),
            repulsion_exempt_namespaces=('tb3_grandparent',),
            shielded_motion_exempt_namespaces=('tb3_grandparent',),
        )
        self.assertGreater(shielded.linear.x, 0.08)

        avoidance.reset_motion()
        avoidance.update_robot_positions([
            ('tb3_0', Point(x=0.0, y=0.0)),
            ('tb3_grandparent', Point(x=0.23, y=0.0)),
        ])
        collapsed = avoidance.apply_avoidance(
            forward_command(),
            repulsion_exempt_namespaces=('tb3_grandparent',),
            shielded_motion_exempt_namespaces=('tb3_grandparent',),
        )
        self.assertEqual(0.0, collapsed.linear.x)

    def test_allowed_contact_cannot_compress_below_guard_distance(self):
        avoidance = self.make_avoidance()
        avoidance.max_linear_acceleration = 100.0
        avoidance.set_position(0.0, 0.0, 0.0)
        avoidance.update_robot_positions([
            ('tb3_0', Point(x=0.0, y=0.0)),
            ('tb3_parent', Point(x=0.145, y=0.0)),
        ])
        closing = avoidance.apply_avoidance(
            forward_command(),
            allowed_contact_namespace='tb3_parent',
        )
        self.assertGreater(closing.linear.x, 0.0)

        avoidance.reset_motion()
        avoidance.update_robot_positions([
            ('tb3_0', Point(x=0.0, y=0.0)),
            ('tb3_parent', Point(x=0.11, y=0.0)),
        ])
        compressed = avoidance.apply_avoidance(
            forward_command(),
            allowed_contact_namespace='tb3_parent',
        )
        self.assertEqual(0.0, compressed.linear.x)

    def test_contact_guard_brakes_before_crossing_its_minimum(self):
        avoidance = self.make_avoidance()
        avoidance.max_linear_acceleration = 0.5
        avoidance.control_dt = 0.05
        avoidance._prev_linear = 0.0

        scale = avoidance._minimum_distance_motion_scale(
            distance=0.121,
            requested_velocity=0.20,
            minimum_distance=0.12,
        )
        next_step = 0.20 * scale * avoidance.control_dt
        self.assertGreater(scale, 0.0)
        self.assertLessEqual(next_step, 0.001 + 1e-9)

        avoidance._prev_linear = 0.20
        self.assertEqual(
            0.0,
            avoidance._minimum_distance_motion_scale(
                distance=0.121,
                requested_velocity=0.20,
                minimum_distance=0.12,
            ),
        )
        self.assertGreater(
            avoidance._minimum_distance_motion_scale(
                distance=0.121,
                requested_velocity=0.20,
                minimum_distance=0.12,
                stop_if_overspeed=False,
            ),
            0.0,
        )

    def test_contact_guard_clamps_smoothed_output_to_kinematic_limit(self):
        for direction in (1.0, -1.0):
            with self.subTest(direction=direction):
                avoidance = self.make_avoidance()
                avoidance.max_linear_acceleration = 0.5
                avoidance.control_dt = 0.05
                avoidance.smoothing_alpha = 0.8
                avoidance.set_position(0.0, 0.0, 0.0)

                # Reproduce a robot arriving at the contact guard with a
                # high command already held in the smoother.  Apply the
                # command repeatedly and move the fixed parent closer by the
                # published velocity on every controller tick.
                requested_speed = 0.20
                avoidance._prev_linear = direction * requested_speed
                distance = 0.14
                for _ in range(40):
                    avoidance.update_robot_positions([
                        ('tb3_0', Point(x=0.0, y=0.0)),
                        (
                            'tb3_parent',
                            Point(x=direction * distance, y=0.0),
                        ),
                    ])
                    scale = avoidance._minimum_distance_motion_scale(
                        distance,
                        requested_speed,
                        avoidance.allowed_contact_min_dist,
                        stop_if_overspeed=False,
                    )
                    raw_speed = requested_speed * scale
                    safe = avoidance.apply_avoidance(
                        forward_command(direction * requested_speed),
                        allowed_contact_namespace='tb3_parent',
                    )
                    closing_speed = max(0.0, direction * safe.linear.x)

                    self.assertLessEqual(
                        closing_speed, raw_speed + 1e-9
                    )
                    distance -= closing_speed * avoidance.control_dt
                    self.assertGreaterEqual(
                        distance + 1e-9,
                        avoidance.allowed_contact_min_dist,
                    )

    def test_contact_guard_does_not_clamp_motion_away_from_contact(self):
        avoidance = self.make_avoidance()
        avoidance.max_linear_acceleration = 0.5
        avoidance.control_dt = 0.05
        avoidance.smoothing_alpha = 0.8
        avoidance._prev_linear = 0.20
        avoidance.set_position(0.0, 0.0, 0.0)
        avoidance.update_robot_positions([
            ('tb3_0', Point(x=0.0, y=0.0)),
            ('tb3_parent', Point(x=-0.14, y=0.0)),
        ])

        safe = avoidance.apply_avoidance(
            forward_command(0.05),
            allowed_contact_namespace='tb3_parent',
        )

        # The parent is behind us, so ordinary comfortable deceleration is
        # retained instead of being clamped to the new requested speed.
        self.assertGreater(safe.linear.x, 0.05)

    def test_declared_chain_contacts_are_allowed_in_both_directions(self):
        avoidance = self.make_avoidance()
        avoidance.max_linear_acceleration = 100.0
        avoidance.set_position(0.0, 0.0, 0.0)
        avoidance.update_robot_positions([
            ('tb3_0', Point(x=0.0, y=0.0)),
            ('tb3_parent', Point(x=0.145, y=0.0)),
            ('tb3_child', Point(x=-0.145, y=0.0)),
        ])

        safe = avoidance.apply_avoidance(
            forward_command(),
            allowed_contact_namespaces=('tb3_parent', 'tb3_child'),
        )

        self.assertGreater(safe.linear.x, 0.0)
        self.assertFalse(avoidance._collision_pub.messages[-1].data)

    def test_chain_exemption_does_not_hide_an_unrelated_robot(self):
        avoidance = self.make_avoidance()
        avoidance.max_linear_acceleration = 100.0
        avoidance.set_position(0.0, 0.0, 0.0)
        avoidance.update_robot_positions([
            ('tb3_0', Point(x=0.0, y=0.0)),
            ('tb3_parent', Point(x=0.145, y=0.0)),
            ('tb3_child', Point(x=-0.145, y=0.0)),
            ('tb3_unexpected', Point(x=0.22, y=0.0)),
        ])

        safe = avoidance.apply_avoidance(
            forward_command(),
            allowed_contact_namespaces=('tb3_parent', 'tb3_child'),
        )

        self.assertEqual(0.0, safe.linear.x)
        self.assertTrue(avoidance._collision_pub.messages[-1].data)

    def test_multiple_lidar_masks_keep_a_closer_return_visible(self):
        avoidance = self.make_avoidance()
        avoidance.max_linear_acceleration = 100.0
        avoidance.set_position(0.0, 0.0, 0.0)
        scan = LaserScan()
        scan.angle_min = 0.0
        scan.angle_increment = math.pi / 2.0
        scan.ranges = [0.18, 0.18]
        avoidance._scan_callback(scan)
        masks = (
            LidarRangeMask(
                center_x=0.235, center_y=0.0, radius=0.055,
                maximum_center_distance=0.8, tolerance=0.03,
                closer_tolerance=0.015,
            ),
            LidarRangeMask(
                center_x=0.0, center_y=0.235, radius=0.055,
                maximum_center_distance=0.8, tolerance=0.03,
                closer_tolerance=0.015,
            ),
        )

        masked = avoidance._sector_distances(lidar_range_mask=masks)
        self.assertEqual(avoidance.max_valid_range, masked[0])
        self.assertEqual(avoidance.max_valid_range, masked[2])

        scan.ranges = [0.14, 0.18]
        avoidance._scan_callback(scan)
        closer = avoidance._sector_distances(lidar_range_mask=masks)
        self.assertAlmostEqual(0.14, closer[0])
        self.assertEqual(avoidance.max_valid_range, closer[2])


class TransportPayloadMaskIntegrationTests(unittest.TestCase):
    @staticmethod
    def make_transport(avoidance):
        controller = ROS['transport'].CollaborativeTransport.__new__(
            ROS['transport'].CollaborativeTransport
        )
        controller.data_lock = threading.Lock()
        controller.avoidance_modules = {'tb3_0': avoidance}
        controller.grf_object_radius = 0.20
        controller.object_half_width = 0.20
        controller.object_half_height = 0.20
        controller.object_avoidance_range = 1.0
        controller.object_lidar_tolerance = 0.04
        controller.object_lidar_closer_tolerance = 0.025
        controller.object_lidar_contact_closer_tolerance = 0.05
        controller.object_yaw = 0.0
        controller.robot_radius = 0.11
        controller.robot_forward_contact_extent = 0.038
        controller.robot_rear_contact_extent = 0.102
        controller.robot_lidar_collision_radius = 0.055
        controller.robot_lidar_center_offset = -0.017
        controller.robot_lidar_mask_tolerance = 0.05
        controller.robot_lidar_mask_closer_tolerance = 0.04
        controller.transport_companion_contact_distance = 0.145
        controller.transport_contact_slop = 0.005
        controller.transport_contact_heading_tolerance = 0.45
        return controller

    @staticmethod
    def reverse_command(speed=0.03):
        command = Twist()
        command.linear.x = -speed
        return command

    def test_adjacent_chain_body_masks_oblique_blind_zone_return(self):
        avoidance = ObstacleAvoidance('tb3_0')
        avoidance.max_linear_acceleration = 100.0
        avoidance.set_position(0.0, 0.0, 0.0)
        avoidance.update_robot_positions([
            ('tb3_0', Point(x=0.0, y=0.0)),
            ('tb3_child', Point(x=-0.141, y=0.0)),
        ])
        controller = self.make_transport(avoidance)
        positions = {'tb3_child': np.array([-0.141, 0.0])}
        yaws = {'tb3_child': 0.0}

        scan = LaserScan()
        scan.range_min = 0.12
        scan.angle_min = math.radians(150.0)
        scan.angle_increment = 0.0
        scan.ranges = [0.12]
        avoidance._scan_callback(scan)

        # The narrow LDS cap misses this rear-corner ray.  The declared
        # adjacent link gets the real 14 cm Burger footprint instead.
        cap_masks = controller._robot_lidar_masks(
            ('tb3_child',), positions, yaws
        )
        cap_distances = avoidance._sector_distances(
            lidar_range_mask=cap_masks
        )
        self.assertAlmostEqual(0.12, cap_distances[3])

        body_masks = controller._transport_robot_lidar_masks(
            ('tb3_child',), (), positions, yaws
        )
        body_distances = avoidance._sector_distances(
            lidar_range_mask=body_masks
        )
        self.assertEqual(avoidance.max_valid_range, body_distances[3])
        self.assertAlmostEqual(0.070, body_masks[0].half_width)
        self.assertAlmostEqual(0.070, body_masks[0].half_height)

        safe = controller._apply_transport_avoidance(
            'tb3_0', self.reverse_command(), (2.0, 2.0),
            allow_payload_contact=False,
            allowed_contact_namespaces=('tb3_child',),
            additional_lidar_masks=body_masks,
        )

        self.assertLess(safe.linear.x, 0.0)
        self.assertFalse(avoidance._collision_pub.messages[-1].data)

    def test_assembly_parent_body_mask_covers_the_complete_handoff_turn(self):
        avoidance = ObstacleAvoidance('tb3_7')
        controller = self.make_transport(avoidance)
        child_position = np.array([-0.625, -0.603])
        parent_position = np.array([-0.616, -0.827])
        parent_yaw = -math.pi / 2.0
        mask = controller._chain_contact_body_lidar_masks(
            ('tb3_3',),
            {'tb3_3': parent_position},
            {'tb3_3': parent_yaw},
        )[0]

        for child_yaw in np.linspace(-math.pi, math.pi, 361):
            lidar_x = (
                float(child_position[0])
                + avoidance.lidar_offset_x * math.cos(child_yaw)
            )
            lidar_y = (
                float(child_position[1])
                + avoidance.lidar_offset_x * math.sin(child_yaw)
            )
            beam_angle = controller._normalize_angle(
                math.atan2(
                    mask.center_y - lidar_y,
                    mask.center_x - lidar_x,
                ) - child_yaw
            )
            self.assertIsNotNone(avoidance._surface_range_for_mask(
                beam_angle,
                mask,
                (lidar_x, lidar_y, child_yaw),
            ))

    def test_adjacent_body_mask_keeps_too_close_return_visible(self):
        avoidance = ObstacleAvoidance('tb3_0')
        avoidance.max_linear_acceleration = 100.0
        avoidance.set_position(0.0, 0.0, 0.0)
        avoidance.update_robot_positions([
            ('tb3_0', Point(x=0.0, y=0.0)),
            ('tb3_child', Point(x=-0.141, y=0.0)),
        ])
        controller = self.make_transport(avoidance)
        positions = {'tb3_child': np.array([-0.141, 0.0])}
        yaws = {'tb3_child': 0.0}
        body_masks = controller._transport_robot_lidar_masks(
            ('tb3_child',), (), positions, yaws
        )

        scan = LaserScan()
        scan.range_min = 0.12
        scan.angle_min = math.radians(150.0)
        scan.angle_increment = 0.0
        scan.ranges = [0.08]
        avoidance._scan_callback(scan)
        blocked = controller._apply_transport_avoidance(
            'tb3_0', self.reverse_command(), (2.0, 2.0),
            allow_payload_contact=False,
            allowed_contact_namespaces=('tb3_child',),
            additional_lidar_masks=body_masks,
        )

        self.assertEqual(0.0, blocked.linear.x)
        self.assertTrue(avoidance._collision_pub.messages[-1].data)

    def test_adjacent_body_mask_does_not_hide_undeclared_robot(self):
        avoidance = ObstacleAvoidance('tb3_0')
        avoidance.max_linear_acceleration = 100.0
        avoidance.set_position(0.0, 0.0, 0.0)
        avoidance.update_robot_positions([
            ('tb3_0', Point(x=0.0, y=0.0)),
            ('tb3_child', Point(x=-0.141, y=0.0)),
            ('tb3_unexpected', Point(x=-0.225, y=0.0)),
        ])
        controller = self.make_transport(avoidance)
        positions = {
            'tb3_child': np.array([-0.141, 0.0]),
            'tb3_unexpected': np.array([-0.225, 0.0]),
        }
        yaws = {'tb3_child': 0.0, 'tb3_unexpected': 0.0}
        body_masks = controller._transport_robot_lidar_masks(
            ('tb3_child',), (), positions, yaws
        )

        scan = LaserScan()
        scan.range_min = 0.12
        scan.angle_min = math.radians(150.0)
        scan.angle_increment = 0.0
        scan.ranges = [0.12]
        avoidance._scan_callback(scan)
        blocked = controller._apply_transport_avoidance(
            'tb3_0', self.reverse_command(), (2.0, 2.0),
            allow_payload_contact=False,
            allowed_contact_namespaces=('tb3_child',),
            additional_lidar_masks=body_masks,
        )

        self.assertEqual(0.0, blocked.linear.x)
        self.assertTrue(avoidance._collision_pub.messages[-1].data)

    def test_only_adjacent_contacts_receive_full_body_masks(self):
        controller = self.make_transport(ObstacleAvoidance('tb3_0'))
        positions = {
            'tb3_child': np.array([-0.141, 0.0]),
            'tb3_grandparent': np.array([0.29, 0.0]),
            'tb3_parallel': np.array([0.0, 0.32]),
        }
        yaws = {namespace: 0.0 for namespace in positions}

        masks = controller._transport_robot_lidar_masks(
            ('tb3_child',),
            ('tb3_child', 'tb3_grandparent', 'tb3_parallel'),
            positions,
            yaws,
        )
        body_masks = [mask for mask in masks if mask.half_width is not None]
        cap_masks = [mask for mask in masks if mask.half_width is None]

        self.assertEqual(1, len(body_masks))
        self.assertEqual(2, len(cap_masks))
        self.assertAlmostEqual(-0.173, body_masks[0].center_x)
        self.assertCountEqual(
            [0.273, -0.017],
            [round(mask.center_x, 3) for mask in cap_masks],
        )

    def test_transport_masks_payload_surface_but_not_closer_wall(self):
        avoidance = ObstacleAvoidance('tb3_0')
        avoidance.max_linear_acceleration = 100.0
        avoidance.set_position(0.0, 0.0, 0.0)
        controller = self.make_transport(avoidance)

        scan = LaserScan()
        # Base centre is 0.389 m from the box.  Burger's laser sits 0.032 m
        # behind it, so the exact range to the 0.2 m face is 0.221 m.
        scan.ranges = [0.221]
        scan.angle_min = 0.0
        scan.angle_increment = 0.0
        avoidance._scan_callback(scan)
        allowed = controller._apply_transport_avoidance(
            'tb3_0', forward_command(), (0.389, 0.0)
        )

        scan.ranges = [0.18]
        avoidance._scan_callback(scan)
        blocked = controller._apply_transport_avoidance(
            'tb3_0', forward_command(), (0.389, 0.0)
        )

        self.assertGreater(allowed.linear.x, 0.0)
        self.assertEqual(0.0, blocked.linear.x)

        scan.ranges = [0.18]
        avoidance._scan_callback(scan)
        confirmed_contact = controller._apply_transport_avoidance(
            'tb3_0', forward_command(), (0.389, 0.0),
            payload_contact_confirmed=True,
        )
        self.assertGreater(confirmed_contact.linear.x, 0.0)

        scan.ranges = [0.15]
        avoidance._scan_callback(scan)
        still_closer = controller._apply_transport_avoidance(
            'tb3_0', forward_command(), (0.389, 0.0),
            payload_contact_confirmed=True,
        )
        self.assertEqual(0.0, still_closer.linear.x)

    def test_rotated_square_surface_is_masked_at_its_true_range(self):
        avoidance = ObstacleAvoidance('tb3_0')
        avoidance.max_linear_acceleration = 100.0
        avoidance.set_position(0.0, 0.0, 0.0)
        controller = self.make_transport(avoidance)
        controller.object_yaw = math.pi / 4.0

        scan = LaserScan()
        # This is the live failure geometry: 0.389 m between model centres.
        # A rotated corner is only 0.138 m from Burger's rear-mounted LDS, so
        # an unmasked scan is an emergency stop even though it is the payload.
        scan.ranges = [0.138]
        scan.angle_min = 0.0
        scan.angle_increment = 0.0
        avoidance._scan_callback(scan)

        safe = controller._apply_transport_avoidance(
            'tb3_0', forward_command(), (0.389, 0.0)
        )

        self.assertGreater(safe.linear.x, 0.0)
        self.assertEqual(0.0, avoidance._threat_pub.messages[-1].data)

    def test_blind_zone_payload_masks_floor_and_background_returns(self):
        avoidance = ObstacleAvoidance('tb3_0')
        avoidance.max_linear_acceleration = 100.0
        avoidance.set_position(0.0, 0.0, 0.0)
        controller = self.make_transport(avoidance)

        scan = LaserScan()
        scan.range_min = 0.12
        scan.angle_min = 0.0
        scan.angle_increment = 0.0

        # At physical face contact the box surface is 0.075 m from Burger's
        # rear-mounted LDS.  Gazebo may clamp to 0.120 m or pass through the
        # blind payload and report a farther background surface instead.
        for reported_range in (0.12, 0.15, 0.50):
            with self.subTest(reported_range=reported_range):
                scan.ranges = [reported_range]
                avoidance._scan_callback(scan)
                allowed = controller._apply_transport_avoidance(
                    'tb3_0', forward_command(), (0.243, 0.0)
                )
                self.assertGreater(allowed.linear.x, 0.0)
                self.assertEqual(
                    0.0, avoidance._threat_pub.messages[-1].data
                )

        # At 3x real time the scan can arrive after the pose used to predict
        # the box surface. A sensor-floor return on an intersecting payload
        # ray is still the known payload, even if that pose predicts 0.221 m.
        scan.ranges = [0.12]
        avoidance._scan_callback(scan)
        delayed_pose = controller._apply_transport_avoidance(
            'tb3_0', forward_command(), (0.389, 0.0)
        )
        self.assertGreater(delayed_pose.linear.x, 0.0)

        # If a driver can distinguish a return 4 cm below the sensor floor, it
        # is not payload evidence and must still trigger the safety stop.
        scan.ranges = [0.08]
        avoidance._scan_callback(scan)
        blocked = controller._apply_transport_avoidance(
            'tb3_0', forward_command(), (0.243, 0.0)
        )
        self.assertEqual(0.0, blocked.linear.x)
        self.assertEqual(1.0, avoidance._threat_pub.messages[-1].data)


class FormationAvoidanceIntegrationTests(unittest.TestCase):
    def test_tight_formations_converge_without_robot_repulsion_deadlock(self):
        cases = (
            (5, 'square', 0.7),
            (7, 'A', 0.7),
            (8, 'V', 0.55),
            (9, 'diamond', 0.55),
            (10, 'S', 0.55),
        )

        for robot_count, shape, spacing in cases:
            with self.subTest(robot_count=robot_count, shape=shape):
                controller, targets = make_closed_loop_controller(
                    robot_count, shape, spacing
                )
                controller.avoidance = {
                    robot_id: ObstacleAvoidance(robot_id)
                    for robot_id in controller.robot_ids
                }
                closest_seen = float('inf')

                for _ in range(1500):
                    controller._control_step(None)
                    dt = 1.0 / controller.control_rate
                    for robot_id in controller.robot_ids:
                        command = controller.cmd_vel_pubs[
                            robot_id
                        ].messages[-1]
                        yaw = formation.normalize_angle(
                            controller.robot_yaws[robot_id]
                            + command.angular.z * dt
                        )
                        pose = controller.robot_poses[robot_id]
                        pose.position.x += (
                            command.linear.x * math.cos(yaw) * dt
                        )
                        pose.position.y += (
                            command.linear.x * math.sin(yaw) * dt
                        )
                        controller.robot_yaws[robot_id] = yaw
                        controller.avoidance[robot_id].set_position(
                            pose.position.x, pose.position.y, yaw
                        )

                    positions = [
                        (
                            controller.robot_poses[rid].position.x,
                            controller.robot_poses[rid].position.y,
                        )
                        for rid in controller.robot_ids
                    ]
                    closest_seen = min(
                        closest_seen,
                        min(
                            math.hypot(a[0] - b[0], a[1] - b[1])
                            for index, a in enumerate(positions)
                            for b in positions[index + 1:]
                        ),
                    )
                    if (
                        controller.formation_state
                        == formation.FormationState.FORMED
                    ):
                        break

                self.assertEqual(
                    controller.formation_state,
                    formation.FormationState.FORMED,
                )
                errors = [
                    math.hypot(
                        controller.robot_poses[rid].position.x
                        - targets[controller.assignments[rid]][0],
                        controller.robot_poses[rid].position.y
                        - targets[controller.assignments[rid]][1],
                    )
                    for rid in controller.robot_ids
                ]
                self.assertLessEqual(max(errors), 0.1)
                self.assertGreater(closest_seen, 0.3)


if __name__ == '__main__':
    unittest.main()
