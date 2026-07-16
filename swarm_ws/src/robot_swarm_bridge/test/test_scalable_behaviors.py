#!/usr/bin/env python3

import math
import unittest

from robot_swarm_bridge.algorithms.formation import (
    hungarian_assignment,
    minimum_distance_assignment,
    sample_letter_formation,
)
from robot_swarm_bridge.algorithms.path_trace import ArcLengthTrace


class FormationAlgorithmTests(unittest.TestCase):
    def test_hungarian_finds_global_optimum(self):
        self.assertEqual(
            hungarian_assignment([
                [10.0, 1.0, 1.0],
                [1.0, 10.0, 1.0],
                [1.0, 1.0, 10.0],
            ]),
            [1, 2, 0],
        )

    def test_previous_slot_penalty_avoids_equivalent_crossing(self):
        assignment = minimum_distance_assignment(
            robot_positions=[(-1.0, 0.0), (1.0, 0.0)],
            target_positions=[(0.0, 0.0), (0.0, 0.0)],
            previous_slots=[0, 1],
            switch_penalty=0.5,
        )
        self.assertEqual(assignment, [0, 1])

    def test_letter_sampler_returns_unique_slots_for_large_fleet(self):
        letter_l = [
            (0, 0), (0, 1), (0, 2), (0, 3), (0, 4), (0, 5), (0, 6),
            (1, 0), (2, 0), (3, 0), (4, 0),
        ]
        slots = sample_letter_formation(letter_l, robot_count=40, spacing=0.6)
        self.assertEqual(len(slots), 40)
        self.assertEqual(len(set(slots)), 40)
        self.assertGreater(max(x for x, _ in slots) - min(x for x, _ in slots), 1.0)
        self.assertGreater(max(y for _, y in slots) - min(y for _, y in slots), 1.0)


class ArcLengthTraceTests(unittest.TestCase):
    def test_interpolates_by_distance_not_time(self):
        trace = ArcLengthTrace()
        trace.append(0.0, 0.0, 0.0)
        trace.append(1.0, 0.0, 0.0)
        trace.append(3.0, 0.0, 0.0)

        point = trace.points_behind([1.5])[0]
        self.assertAlmostEqual(point.x, 1.5)
        self.assertAlmostEqual(point.y, 0.0)

    def test_seed_line_provides_distinct_initial_targets(self):
        trace = ArcLengthTrace()
        trace.seed_line(2.0, 1.0, math.pi / 2.0, distance_behind=2.0)
        targets = trace.points_behind([0.0, 1.0, 2.0])
        self.assertAlmostEqual(targets[0].y, 1.0)
        self.assertAlmostEqual(targets[1].y, 0.0)
        self.assertAlmostEqual(targets[2].y, -1.0)

    def test_trim_bounds_history_by_fleet_span(self):
        trace = ArcLengthTrace()
        for index in range(21):
            trace.append(index * 0.1, 0.0, 0.0)
        trace.trim(0.5)
        self.assertLessEqual(
            trace.total_distance - trace.points[0].distance,
            0.6,
        )

    def test_small_odometry_steps_accumulate_into_trace_distance(self):
        trace = ArcLengthTrace(minimum_step=0.015)
        trace.append(0.0, 0.0, 0.0)

        for index in range(1, 101):
            trace.append(index * 0.01, 0.0, 0.0)

        self.assertAlmostEqual(trace.total_distance, 1.0, places=6)
        self.assertAlmostEqual(trace.points_behind([0.7])[0].x, 0.3, places=6)


if __name__ == "__main__":
    unittest.main()
