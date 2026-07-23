#!/usr/bin/env python3
"""ROS-free checks for the moving formation acceptance window."""

import types
import unittest

from test_transport_acceptance_metrics import LIVE


class FormationLiveAcceptanceTest(unittest.TestCase):
    FORMATION_TIMEOUT_RTF_FLOOR = 2.7
    FORMATION_TIMEOUT_MARGIN_SIM_S = 15.0

    @staticmethod
    def formation_case():
        return next(
            case for case in LIVE.SCENARIOS
            if case["name"] == "formation_A_n7"
        )

    @staticmethod
    def valid_status(case, task_id):
        return {
            "task_id": task_id,
            "state": "moving",
            "movement_mode": "moving",
            "paused": False,
            "formation_type": case["shape"],
            "robot_count": case["count"],
            "maximum_position_error": 0.08,
            "robot_assignments": {
                name: {} for name in LIVE.acceptance_robot_ids(case["count"])
            },
            "stale_odometry": [],
            "waiting_for_odometry": [],
        }

    def test_normal_runner_remains_static_outside_the_explicit_option(self):
        case = self.formation_case()
        harness = LIVE.AcceptanceHarness.__new__(LIVE.AcceptanceHarness)
        harness.args = types.SimpleNamespace(formation_active_seconds=0.0)

        parameters = harness.task_parameters(case, "formation-static")

        self.assertEqual("static", parameters["movement_mode"])
        self.assertEqual(case["shape"], parameters["formation_type"])

    def test_matrix_option_dispatches_moving_formation(self):
        case = self.formation_case()
        harness = LIVE.AcceptanceHarness.__new__(LIVE.AcceptanceHarness)
        harness.args = types.SimpleNamespace(formation_active_seconds=15.0)

        parameters = harness.task_parameters(case, "formation-moving")

        self.assertEqual("moving", parameters["movement_mode"])
        self.assertEqual({"spacing": case["spacing"]}, parameters["config"])

    def test_active_status_requires_shape_fleet_error_and_odometry(self):
        case = self.formation_case()
        task_id = "formation-window"
        robots = LIVE.acceptance_robot_ids(case["count"])
        valid = self.valid_status(case, task_id)

        self.assertTrue({
            "stale_odometry", "waiting_for_odometry",
        }.issubset(LIVE.RESULT_BEHAVIOR_STATUS_KEYS))

        self.assertEqual(
            [],
            LIVE.formation_active_status_failures(
                valid, task_id, case, robots, 0.12
            ),
        )

        invalid = (
            {**valid, "task_id": "older-task"},
            {**valid, "state": "forming"},
            {**valid, "movement_mode": "static"},
            {**valid, "paused": True},
            {**valid, "formation_type": "V"},
            {**valid, "robot_count": case["count"] - 1},
            {**valid, "maximum_position_error": 0.121},
            {**valid, "error": "unsafe placement"},
            {**valid, "stale_odometry": ["tb3_0"]},
            {
                **valid,
                "robot_assignments": {
                    name: {} for name in robots[:-1]
                },
            },
        )
        for status in invalid:
            with self.subTest(status=status):
                self.assertTrue(
                    LIVE.formation_active_status_failures(
                        status, task_id, case, robots, 0.12
                    )
                )

    def test_active_window_is_bounded_and_only_for_one_formation(self):
        formation = self.formation_case()
        transport = next(
            case for case in LIVE.SCENARIOS
            if case["behavior"] == "transport"
        )

        self.assertEqual(
            0.0, LIVE.validate_formation_active_selection(0, [transport])
        )
        self.assertEqual(
            15.0,
            LIVE.validate_formation_active_selection(15, [formation]),
        )
        for seconds, cases in (
            (14.9, [formation]),
            (121, [formation]),
            (15, [transport]),
            (15, [formation, formation]),
        ):
            with self.subTest(seconds=seconds, cases=cases):
                with self.assertRaises(ValueError):
                    LIVE.validate_formation_active_selection(seconds, cases)

    def test_measured_formation_matrix_keeps_a_simulation_time_margin(self):
        measured_cases = {
            "formation_triangle_n3": (15.15, 35),
            "formation_square_n5": (142.61, 90),
            "formation_A_n7": (156.65, 65),
            "formation_V_n8": (190.05, 90),
            "formation_diamond_n9": (213.65, 90),
            "formation_S_n10": (208.75, 85),
        }
        cases = {case["name"]: case for case in LIVE.SCENARIOS}

        for name, (measured_sim_s, timeout_wall_s) in measured_cases.items():
            with self.subTest(scenario=name):
                self.assertEqual(timeout_wall_s, cases[name]["timeout"])
                available_sim_s = (
                    cases[name]["timeout"] * self.FORMATION_TIMEOUT_RTF_FLOOR
                )
                self.assertGreaterEqual(
                    available_sim_s - measured_sim_s,
                    self.FORMATION_TIMEOUT_MARGIN_SIM_S,
                )


if __name__ == "__main__":
    unittest.main()
