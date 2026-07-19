#!/usr/bin/env python3
"""ROS-free checks for live collaborative-transport acceptance accounting."""

import importlib.util
from pathlib import Path
import sys
import types
import unittest


def _empty_message_module(name, **classes):
    module = types.ModuleType(name)
    for class_name, value in classes.items():
        setattr(module, class_name, value)
    return module


class _Message:
    def __init__(self, *args, **kwargs):
        for key, value in kwargs.items():
            setattr(self, key, value)


def _load_acceptance_module():
    """Load the runner without requiring a ROS installation on the test host."""
    stubs = {
        "rospy": types.ModuleType("rospy"),
        "gazebo_msgs": types.ModuleType("gazebo_msgs"),
        "gazebo_msgs.msg": _empty_message_module(
            "gazebo_msgs.msg", ModelState=_Message, ModelStates=_Message
        ),
        "gazebo_msgs.srv": _empty_message_module(
            "gazebo_msgs.srv", SetModelState=_Message
        ),
        "geometry_msgs": types.ModuleType("geometry_msgs"),
        "geometry_msgs.msg": _empty_message_module(
            "geometry_msgs.msg", Twist=_Message
        ),
        "rosgraph_msgs": types.ModuleType("rosgraph_msgs"),
        "rosgraph_msgs.msg": _empty_message_module(
            "rosgraph_msgs.msg", Clock=_Message
        ),
        "std_msgs": types.ModuleType("std_msgs"),
        "std_msgs.msg": _empty_message_module(
            "std_msgs.msg", String=_Message
        ),
    }
    original_modules = {
        name: sys.modules.get(name) for name in stubs
    }
    try:
        sys.modules.update(stubs)
        path = Path(__file__).with_name("robotswarm_live_acceptance.py")
        spec = importlib.util.spec_from_file_location(
            "robotswarm_live_acceptance_metrics_under_test", path
        )
        module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)
        return module
    finally:
        for name, original in original_modules.items():
            if original is None:
                sys.modules.pop(name, None)
            else:
                sys.modules[name] = original


LIVE = _load_acceptance_module()


def _vector(x=0.0, y=0.0, z=0.0):
    return types.SimpleNamespace(x=x, y=y, z=z)


def _twist(x=0.0, y=0.0):
    return types.SimpleNamespace(
        linear=_vector(x, y), angular=_vector()
    )


class TransportAcceptanceMetricsTest(unittest.TestCase):
    robots = ("tb3_0", "tb3_1")

    def setUp(self):
        self.args = types.SimpleNamespace(
            transport_direct_contact_clearance=0.075,
            transport_companion_contact_distance=0.16,
            transport_cmd_max_age=0.75,
            transport_cmd_min_speed=0.015,
            transport_cmd_min_goal_cosine=0.50,
            transport_contribution_noise_floor=0.003,
            transport_contribution_speed_tolerance=0.003,
            transport_contribution_tracking_fraction=0.75,
            transport_control_gap_tolerance=1.0,
            min_transport_push_duration=0.75,
            min_transport_push_samples=5,
            min_transport_useful_fraction=0.50,
            min_transport_rendezvous_travel=0.10,
            min_transport_root_rendezvous_travel=0.03,
            transport_motion_detection_distance=0.01,
            transport_rendezvous_moving_speed=0.02,
            min_center_distance=0.24,
            min_obstacle_clearance=0.10,
            min_boundary_clearance=0.10,
            min_transport_chain_center_distance=0.12,
        )
        self.case = {
            "name": "unit_transport",
            "behavior": "transport",
            "target": (1.0, 0.0),
        }
        self.metrics = LIVE.CaseMetrics(
            self.case, self.robots, 0.0, 0.0, self.args
        )

    def _status(self, sequence, control_time, commands, phase="PUSH"):
        return {
            "phase": phase,
            "engagement_complete": True,
            "synchronized_push_started": True,
            "grf_mcmc_iterations": 4,
            "control_sequence": sequence,
            "control_sim_time": control_time,
            "control_commands": {
                robot: {"linear": speed, "angular": 0.0}
                for robot, speed in commands.items()
            },
            "push_reference_speed": 0.015,
            "robot_assignments": {
                "tb3_0": {
                    "role": "payload_push",
                    "chain_index": 0,
                    "chain_depth": 0,
                },
                "tb3_1": {
                    "role": "companion_push",
                    "chain_index": 0,
                    "chain_depth": 1,
                    "parent_namespace": "tb3_0",
                },
            },
        }

    def _observe(
        self, sequence, control_time, commands, object_x=0.0,
        root_yaw=0.0, companion_yaw=0.0, companion_gap=0.145,
        phase="PUSH", payload_velocity=0.006, parent_velocity=0.006,
    ):
        root_x = object_x - 0.25
        model_pose = {
            "transport_object": (object_x, 0.0, 0.0),
            "tb3_0": (root_x, 0.0, root_yaw),
            "tb3_1": (root_x - companion_gap, 0.0, companion_yaw),
        }
        positions = {
            name: model_pose[name][:2] for name in self.robots
        }
        twists = {
            "transport_object": _twist(payload_velocity),
            "tb3_0": _twist(parent_velocity),
            "tb3_1": _twist(0.006),
        }
        status = self._status(sequence, control_time, commands, phase)
        self.metrics._transport_participation(
            positions, model_pose, twists, status, {}, control_time,
            control_time, [],
        )

    def _coordinated_window(self, commands=None, **kwargs):
        commands = commands or {name: 0.0061 for name in self.robots}
        for index in range(6):
            self._observe(
                index + 1, index * 0.2, commands,
                object_x=index * 0.001, **kwargs
            )

    def test_slow_coordinated_push_contributes_but_warns_on_nominal_pace(self):
        self._coordinated_window()

        report = self.metrics.report()
        self.assertEqual(6, report["transport_distinct_control_batches"])
        self.assertGreaterEqual(
            report["transport_maximum_continuous_all_useful_s"], 0.75
        )
        self.assertGreaterEqual(
            report["transport_maximum_continuous_all_useful_samples"], 5
        )
        self.assertEqual(1.0, report["transport_all_useful_fraction"])
        self.assertEqual(0.0, report["transport_all_nominal_pace_fraction"])
        self.assertEqual(
            list(self.robots),
            report["transport_nominal_pace_warning_robots"],
        )
        companion = report["transport_participation"]["tb3_1"]
        self.assertAlmostEqual(
            0.0061, companion["command_goal_speed_mps"]["mean"], places=4
        )
        self.assertAlmostEqual(
            0.003,
            companion["adaptive_contribution_threshold_mps"]["max"],
            places=4,
        )

    def test_zero_command_carried_companion_does_not_contribute(self):
        self._coordinated_window(
            commands={"tb3_0": 0.0061, "tb3_1": 0.0}
        )

        report = self.metrics.report()
        companion = report["transport_participation"]["tb3_1"]
        self.assertEqual(0, companion["push_intent_samples"])
        self.assertEqual(0.0, companion["useful_pushing_fraction"])
        self.assertEqual(0.0, report["transport_all_useful_fraction"])

    def test_companion_tracks_parent_velocity_not_payload_velocity(self):
        for index in range(6):
            self._observe(
                index + 1, index * 0.2,
                {"tb3_0": 0.0061, "tb3_1": 0.0061},
                object_x=index * 0.001,
                payload_velocity=0.006,
                parent_velocity=0.010,
            )

        report = self.metrics.report()["transport_participation"]
        self.assertEqual(6, report["tb3_0"]["push_intent_samples"])
        self.assertEqual(0, report["tb3_1"]["push_intent_samples"])
        self.assertAlmostEqual(
            0.007,
            report["tb3_1"]["adaptive_contribution_threshold_mps"]["max"],
            places=4,
        )

    def test_loaded_robot_can_contribute_below_a_faster_parent_velocity(self):
        for index in range(6):
            self._observe(
                index + 1, index * 0.2,
                {"tb3_0": 0.0151, "tb3_1": 0.0151},
                object_x=index * 0.001,
                payload_velocity=0.020,
                parent_velocity=0.020,
            )

        report = self.metrics.report()
        self.assertEqual(1.0, report["transport_all_useful_fraction"])
        self.assertAlmostEqual(
            0.015,
            report["transport_participation"]["tb3_1"]
            ["adaptive_contribution_threshold_mps"]["max"],
            places=4,
        )

    def test_large_fleet_control_cadence_keeps_one_continuous_window(self):
        commands = {name: 0.0061 for name in self.robots}
        for index in range(6):
            self._observe(
                index + 1, index * 0.55, commands,
                object_x=index * 0.001,
            )

        report = self.metrics.report()
        self.assertEqual(
            6, report["transport_maximum_continuous_all_useful_samples"]
        )
        self.assertAlmostEqual(
            2.75,
            report["transport_maximum_continuous_all_useful_s"],
            places=4,
        )

    def test_backward_and_sideways_commands_do_not_contribute(self):
        for yaw in (3.141592653589793, 1.5707963267948966):
            with self.subTest(yaw=yaw):
                self.metrics = LIVE.CaseMetrics(
                    self.case, self.robots, 0.0, 0.0, self.args
                )
                self._coordinated_window(companion_yaw=yaw)
                companion = self.metrics.report()[
                    "transport_participation"
                ]["tb3_1"]
                self.assertEqual(0, companion["push_intent_samples"])
                self.assertEqual(0.0, companion["useful_pushing_fraction"])

    def test_broken_chain_cannot_claim_companion_contribution(self):
        self._observe(
            1, 0.0, {name: 0.0061 for name in self.robots}
        )
        for index in range(1, 6):
            self._observe(
                index + 1, index * 0.2,
                {name: 0.0061 for name in self.robots},
                object_x=index * 0.001, companion_gap=0.30,
            )

        report = self.metrics.report()
        companion = report["transport_participation"]["tb3_1"]
        self.assertEqual(1, companion["connected_samples"])
        self.assertEqual(0.0, companion["useful_pushing_fraction"])
        self.assertIsNotNone(report["transport_first_connection_loss"])

    def test_duplicate_control_sequence_is_ignored(self):
        commands = {name: 0.0061 for name in self.robots}
        self._observe(1, 0.0, commands)
        self._observe(1, 0.0, commands, object_x=0.001)
        self._observe(2, 0.2, commands, object_x=0.002)

        report = self.metrics.report()
        self.assertEqual(2, report["transport_distinct_control_batches"])
        self.assertEqual(
            1, report["transport_duplicate_control_status_samples"]
        )
        for item in report["transport_participation"].values():
            self.assertEqual(2, item["push_phase_samples"])

    def test_terminal_zero_batch_does_not_penalize_completed_proof(self):
        self._coordinated_window()
        before = self.metrics.report()
        self._observe(
            7, 1.2, {name: 0.0 for name in self.robots},
            object_x=0.006, phase="DONE",
        )
        after = self.metrics.report()

        self.assertEqual(
            before["transport_distinct_control_batches"],
            after["transport_distinct_control_batches"],
        )
        self.assertEqual(
            before["transport_all_useful_fraction"],
            after["transport_all_useful_fraction"],
        )
        self.assertEqual(
            before["transport_maximum_continuous_all_useful_samples"],
            after["transport_maximum_continuous_all_useful_samples"],
        )

    def test_search_motion_window_observes_the_complete_fleet(self):
        positions = {"tb3_0": (0.0, 0.0), "tb3_1": (0.5, 0.0)}
        twists = {name: _twist(0.03) for name in self.robots}
        for index in range(3):
            self.metrics._transport_discovery_motion(
                positions, twists, {"phase": "SEARCH"}, index * 0.1,
                index * 0.1,
            )

        response = self.metrics.report()["transport_discovery_response"]
        self.assertTrue(response["simultaneous_motion_window_supported"])
        self.assertTrue(response["search_motion_window_supported"])
        self.assertEqual(2, response["peak_simultaneous_movers"])
        self.assertEqual(
            [], response["robots_not_observed_moving_during_search"]
        )
        self.assertEqual(
            [], LIVE.transport_search_motion_failures(response, 2)
        )

    def test_search_gate_is_strict_only_with_a_supported_window(self):
        incomplete = {
            "simultaneous_motion_window_supported": True,
            "search_motion_window_supported": True,
            "peak_simultaneous_movers": 1,
            "robots_not_observed_moving_during_search": ["tb3_1"],
        }
        self.assertEqual(
            2, len(LIVE.transport_search_motion_failures(incomplete, 2))
        )

        incomplete["simultaneous_motion_window_supported"] = False
        incomplete["search_motion_window_supported"] = False
        self.assertEqual(
            [], LIVE.transport_search_motion_failures(incomplete, 2)
        )

    def _contact_report(self):
        return {
            "model_samples": 100,
            "minimum_unexpected_robot_center_distance_m": 0.3486,
            "minimum_static_obstacle_clearance_m": 0.7179,
            "minimum_boundary_clearance_m": 2.735,
            "minimum_declared_chain_center_distance_m": 0.1399,
            "transport_participation": {
                "tb3_0": {
                    "role": "payload_push",
                    "direct_contact_samples": 40,
                    "companion_contact_samples": 0,
                    "declared_parent_namespaces": [],
                },
                "tb3_1": {
                    "role": "payload_push",
                    "direct_contact_samples": 38,
                    "companion_contact_samples": 0,
                    "declared_parent_namespaces": [],
                },
                "tb3_2": {
                    "role": "companion_push",
                    "direct_contact_samples": 0,
                    "companion_contact_samples": 35,
                    "declared_parent_namespaces": ["tb3_0"],
                },
            },
        }

    def test_transport_docking_does_not_turn_raw_counter_into_failure(self):
        classification = LIVE.classify_contact_episodes(
            "transport", 1, self._contact_report(), self.args
        )

        self.assertEqual(
            "expected_transport_docking",
            classification["classification"],
        )
        self.assertEqual(1, classification["raw_collision_count_delta"])
        self.assertEqual(
            1,
            classification["classified_expected_contact_count_delta"],
        )
        self.assertEqual(
            0,
            classification["classified_unexpected_contact_count_delta"],
        )
        self.assertFalse(classification["hard_failure"])

    def test_non_chain_robot_contact_remains_a_hard_failure(self):
        report = self._contact_report()
        report["minimum_unexpected_robot_center_distance_m"] = 0.20

        classification = LIVE.classify_contact_episodes(
            "transport", 1, report, self.args
        )

        self.assertEqual(
            "unexpected_contact", classification["classification"]
        )
        self.assertEqual(
            ["minimum_unexpected_robot_center_distance_m"],
            classification["crossed_clearance_limits"],
        )
        self.assertTrue(classification["hard_failure"])

    def test_static_contact_remains_a_hard_failure_during_transport(self):
        report = self._contact_report()
        report["minimum_static_obstacle_clearance_m"] = 0.05

        classification = LIVE.classify_contact_episodes(
            "transport", 1, report, self.args
        )

        self.assertEqual(
            "unexpected_contact", classification["classification"]
        )
        self.assertIn(
            "minimum_static_obstacle_clearance_m",
            classification["crossed_clearance_limits"],
        )
        self.assertTrue(classification["hard_failure"])

    def test_missing_declared_docking_contact_fails_closed(self):
        report = self._contact_report()
        report["transport_participation"]["tb3_2"][
            "companion_contact_samples"
        ] = 0

        classification = LIVE.classify_contact_episodes(
            "transport", 1, report, self.args
        )

        self.assertEqual(
            "unexpected_contact", classification["classification"]
        )
        self.assertEqual(
            ["tb3_2"], classification["missing_expected_contact_robots"]
        )
        self.assertTrue(classification["hard_failure"])

    def test_non_transport_contact_is_never_relabelled_as_docking(self):
        classification = LIVE.classify_contact_episodes(
            "formation", 1, self._contact_report(), self.args
        )

        self.assertEqual(
            "unexpected_contact", classification["classification"]
        )
        self.assertTrue(classification["hard_failure"])


if __name__ == "__main__":
    unittest.main()
