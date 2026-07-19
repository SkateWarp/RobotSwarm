#!/usr/bin/env python3
"""ROS-free checks for the visible follow-the-leader acceptance gate."""

import unittest

from test_transport_acceptance_metrics import LIVE


class FollowLiveAcceptanceTest(unittest.TestCase):
    def _status(self, **changes):
        status = {
            "task_id": "follow-lap-test",
            "state": "running",
            "active": True,
            "leader_mode": "figure8",
            "chain_ready": True,
            "path_period_s": 446.5,
            "path_progress_laps": 1.0,
            "current_lap_progress": 0.0,
            "completed_laps": 1,
        }
        status.update(changes)
        return status

    def test_correlated_completed_lap_passes(self):
        self.assertTrue(LIVE.follow_lap_requirement_met(
            self._status(), "follow-lap-test", "figure8", 1
        ))

    def test_rounded_progress_cannot_hide_an_incomplete_lap(self):
        status = self._status(
            path_progress_laps=1.0,
            current_lap_progress=1.0,
            completed_laps=0,
        )

        self.assertFalse(LIVE.follow_lap_requirement_met(
            status, "follow-lap-test", "figure8", 1
        ))

    def test_setup_or_stale_status_cannot_pass(self):
        invalid_cases = (
            (self._status(task_id="older-task"), "follow-lap-test"),
            (self._status(), ""),
            (self._status(state="waiting_for_odometry"), "follow-lap-test"),
            (self._status(active=False), "follow-lap-test"),
            (self._status(chain_ready=False), "follow-lap-test"),
            (self._status(leader_mode="circular"), "follow-lap-test"),
            (self._status(path_period_s=None), "follow-lap-test"),
        )

        for status, task_id in invalid_cases:
            with self.subTest(status=status):
                self.assertFalse(LIVE.follow_lap_requirement_met(
                    status, task_id, "figure8", 1
                ))

    def test_every_visible_follow_case_requires_a_full_lap(self):
        expected_caps = {
            "follow_circular_n3": 120,
            "follow_square_n6": 150,
            "follow_figure8_n10": 240,
        }
        follow_cases = {
            case["name"]: case
            for case in LIVE.SCENARIOS
            if case["behavior"] == "follow"
        }

        self.assertEqual(set(expected_caps), set(follow_cases))
        for name, duration in expected_caps.items():
            with self.subTest(scenario=name):
                self.assertEqual(1, follow_cases[name]["required_laps"])
                self.assertEqual(duration, follow_cases[name]["duration"])

    def test_follow_setup_diagnostics_are_kept_in_results(self):
        expected = {"setup_phase", "path_relocated", "planning_wall_s"}

        self.assertTrue(
            expected.issubset(LIVE.RESULT_BEHAVIOR_STATUS_KEYS)
        )


if __name__ == "__main__":
    unittest.main()
