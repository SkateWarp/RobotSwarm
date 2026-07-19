#!/usr/bin/env python3

import math
import sys
import unittest
from pathlib import Path


PACKAGE_DIR = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PACKAGE_DIR / 'src'))

from robot_swarm_bridge.algorithms.formation import (  # noqa: E402
    plan_obstacle_aware_route,
    point_to_route_distance,
    routes_conflict,
    straight_route_is_safe,
)


class ObstacleAwareRouteTests(unittest.TestCase):
    def assert_route_is_clear(self, route, clearance, zones):
        self.assertIsNotNone(route)
        for start, end in zip(route, route[1:]):
            self.assertTrue(straight_route_is_safe(
                start,
                end,
                clearance,
                zones,
                'swarm_arena',
            ))

    def test_direct_route_has_no_extra_waypoints(self):
        route = plan_obstacle_aware_route(
            (-1.0, -0.5),
            (1.0, 0.5),
            10.0,
            0.35,
            0.2,
            [],
            'swarm_arena',
        )

        self.assertEqual(route, [(-1.0, -0.5), (1.0, 0.5)])

    def test_box_detour_uses_clear_inflated_corners(self):
        zones = [{
            'shape': 'box',
            'x': 0.0,
            'y': 0.0,
            'width': 0.8,
            'height': 0.8,
        }]
        route = plan_obstacle_aware_route(
            (-2.0, 0.0),
            (2.0, 0.0),
            10.0,
            0.35,
            0.2,
            zones,
            'swarm_arena',
        )

        self.assert_route_is_clear(route, 0.2, zones)
        self.assertEqual(route[0], (-2.0, 0.0))
        self.assertEqual(route[-1], (2.0, 0.0))
        self.assertGreaterEqual(len(route), 4)
        self.assertTrue(any(abs(y) > 0.6 for _, y in route[1:-1]))

    def test_circle_detour_follows_a_clear_sampled_ring(self):
        zones = [{
            'shape': 'circle',
            'x': 0.0,
            'y': 0.0,
            'radius': 0.5,
        }]
        route = plan_obstacle_aware_route(
            (-2.0, 0.0),
            (2.0, 0.0),
            10.0,
            0.35,
            0.2,
            zones,
            'swarm_arena',
        )

        self.assert_route_is_clear(route, 0.2, zones)
        self.assertEqual(route[0], (-2.0, 0.0))
        self.assertEqual(route[-1], (2.0, 0.0))
        self.assertGreaterEqual(len(route), 4)
        self.assertTrue(all(
            math.hypot(x, y) >= 0.7 - 1e-9
            for x, y in route[1:-1]
        ))

    def test_zone_can_use_a_smaller_clearance_for_robot_slots(self):
        zones = [{
            'shape': 'circle',
            'x': 0.0,
            'y': 0.0,
            'radius': 0.0,
            'clearance': 0.2,
        }]

        route = plan_obstacle_aware_route(
            (-1.0, 0.0),
            (1.0, 0.0),
            10.0,
            0.35,
            0.4,
            zones,
            'swarm_arena',
        )

        self.assertIsNotNone(route)
        self.assertTrue(any(abs(y) >= 0.2 for _, y in route[1:-1]))
        self.assertTrue(all(
            math.hypot(x, y) >= 0.2 - 1e-9
            for x, y in route[1:-1]
        ))

    def test_route_fails_when_obstacle_closes_the_arena(self):
        zones = [{
            'shape': 'box',
            'x': 0.0,
            'y': 0.0,
            'width': 0.8,
            'height': 3.8,
        }]

        route = plan_obstacle_aware_route(
            (-1.0, 0.0),
            (1.0, 0.0),
            4.0,
            0.1,
            0.1,
            zones,
            'swarm_arena',
        )

        self.assertIsNone(route)

    def test_route_conflicts_cover_crossings_and_waiting_robots(self):
        horizontal = [(-1.0, 0.0), (1.0, 0.0)]
        crossing = [(0.0, -1.0), (0.0, 1.0)]
        separate = [(-1.0, 0.5), (1.0, 0.5)]

        self.assertAlmostEqual(
            0.1, point_to_route_distance((0.0, 0.1), horizontal)
        )
        self.assertTrue(routes_conflict(horizontal, crossing, 0.3))
        self.assertFalse(routes_conflict(horizontal, separate, 0.3))


if __name__ == '__main__':
    unittest.main()
