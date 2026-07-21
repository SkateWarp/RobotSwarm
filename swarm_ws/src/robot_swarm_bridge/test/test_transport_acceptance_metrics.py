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
            min_transport_search_travel=0.05,
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

    def test_production_rtf_gate_is_the_default(self):
        args = LIVE.build_parser().parse_args([])

        self.assertEqual(2.90, args.min_rtf)
        self.assertEqual(0.05, args.min_transport_search_travel)

    def test_n1_timeout_has_explicit_phase_liveness_budgets(self):
        case = next(
            item for item in LIVE.SCENARIOS
            if item["name"] == "transport_grf_n1"
        )

        self.assertEqual(325, case["timeout"])
        self.assertEqual(
            {"SEARCH": 245.0, "APPROACH": 20.0, "PUSH": 55.0},
            case["phase_timeouts"],
        )
        self.assertEqual(
            5.0,
            case["timeout"] - sum(case["phase_timeouts"].values()),
        )

    def test_n3_timeout_has_measured_phase_liveness_budgets(self):
        case = next(
            item for item in LIVE.SCENARIOS
            if item["name"] == "transport_grf_n3"
        )

        self.assertEqual(290, case["timeout"])
        self.assertEqual(
            {"SEARCH": 115.0, "APPROACH": 125.0, "PUSH": 45.0},
            case["phase_timeouts"],
        )
        self.assertEqual(
            5.0,
            case["timeout"] - sum(case["phase_timeouts"].values()),
        )

    def test_n4_timeout_covers_the_loaded_payload_without_weakening_liveness(self):
        case = next(
            item for item in LIVE.SCENARIOS
            if item["name"] == "transport_grf_n4"
        )

        self.assertEqual(355, case["timeout"])
        self.assertEqual(
            {"SEARCH": 60.0, "APPROACH": 100.0, "PUSH": 190.0},
            case["phase_timeouts"],
        )
        self.assertEqual(
            5.0,
            case["timeout"] - sum(case["phase_timeouts"].values()),
        )

    def test_transport_result_allowlist_keeps_final_diagnostic_geometry(self):
        required = {
            "object_pos", "target_pos", "progress", "object_z",
            "route_target", "queue_docking_started", "queue_settling",
        }

        self.assertTrue(required <= set(LIVE.RESULT_BEHAVIOR_STATUS_KEYS))

    def _status(
        self, sequence, control_time, commands, phase="PUSH",
        reference_speed=0.015,
    ):
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
            "push_reference_speed": reference_speed,
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
        reference_speed=0.015,
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
        status = self._status(
            sequence, control_time, commands, phase, reference_speed
        )
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

    def test_first_complete_fleet_batch_latency_is_reported(self):
        self._observe(
            1, 0.2, {"tb3_0": 0.0061, "tb3_1": 0.0},
            object_x=0.0,
        )
        self._observe(
            2, 0.8, {name: 0.0061 for name in self.robots},
            object_x=0.001,
        )
        self._observe(
            3, 1.0, {name: 0.0061 for name in self.robots},
            object_x=0.002,
        )

        report = self.metrics.report()
        self.assertEqual(0.8, report[
            "transport_first_all_useful_sim_time_s"
        ])
        self.assertEqual(0.8, report[
            "transport_time_to_first_all_useful_batch_sim_s"
        ])
        self.assertEqual(0.8, report[
            "transport_time_to_first_all_useful_batch_wall_s"
        ])

    def test_short_transport_progress_gate_follows_arrival_contract(self):
        self.assertEqual(
            0.5,
            LIVE.required_transport_goal_progress(0.55, 1.0, 0.5),
        )

    def test_long_transport_keeps_configured_progress_gate(self):
        self.assertEqual(
            0.55,
            LIVE.required_transport_goal_progress(0.55, 1.4, 0.5),
        )

    def test_degenerate_transport_keeps_explicit_progress_epsilon(self):
        self.assertEqual(
            LIVE.TRANSPORT_PROGRESS_EPSILON_M,
            LIVE.required_transport_goal_progress(0.55, 0.4, 0.5),
        )

    def test_push_progress_gate_uses_distance_when_push_becomes_active(self):
        self.assertAlmostEqual(
            0.4973,
            LIVE.required_transport_goal_progress(0.55, 0.9973, 0.5),
            places=4,
        )

        self._coordinated_window()
        self.assertEqual(
            1.0,
            self.metrics.report()[
                "transport_push_initial_goal_distance_m"
            ],
        )

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
            0.0112,
            report["transport_participation"]["tb3_1"]
            ["adaptive_contribution_threshold_mps"]["max"],
            places=4,
        )

    def test_measured_payload_momentum_cannot_outrun_requested_pace(self):
        for index in range(6):
            self._observe(
                index + 1, index * 0.2,
                {"tb3_0": 0.018, "tb3_1": 0.018},
                object_x=index * 0.001,
                payload_velocity=0.0248,
                parent_velocity=0.0248,
                reference_speed=0.018,
            )

        report = self.metrics.report()
        self.assertEqual(1.0, report["transport_all_useful_fraction"])
        for robot in self.robots:
            item = report["transport_participation"][robot]
            self.assertEqual(6, item["useful_pushing_samples"])
            self.assertAlmostEqual(
                0.0135,
                item["adaptive_contribution_threshold_mps"]["max"],
                places=4,
            )
            self.assertAlmostEqual(
                0.0248,
                item["role_reference_goal_speed_mps"]["max"],
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
        twists = {name: _twist(0.03) for name in self.robots}
        for index in range(3):
            positions = {
                "tb3_0": (index * 0.03, 0.0),
                "tb3_1": (0.5 + index * 0.03, 0.0),
            }
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
            [], response["robots_below_required_search_travel"]
        )
        self.assertAlmostEqual(
            0.06, response["search_path_length_m"]["tb3_0"]
        )
        self.assertEqual(
            [], LIVE.transport_search_motion_failures(
                response, 2, require_search=True
            )
        )

    def test_required_search_rejects_a_token_motion_sample(self):
        twists = {name: _twist(0.03) for name in self.robots}
        for index in range(3):
            positions = {
                "tb3_0": (index * 0.005, 0.0),
                "tb3_1": (0.5 + index * 0.005, 0.0),
            }
            self.metrics._transport_discovery_motion(
                positions, twists, {"phase": "SEARCH"}, index * 0.1,
                index * 0.1,
            )

        response = self.metrics.report()["transport_discovery_response"]
        failures = LIVE.transport_search_motion_failures(
            response, 2, require_search=True
        )

        self.assertEqual(list(self.robots), response[
            "robots_below_required_search_travel"
        ])
        self.assertEqual(1, len(failures))
        self.assertIn("sustain enough", failures[0])

    def test_approach_peak_cannot_satisfy_search_simultaneity(self):
        search_frames = [
            ((0.00, 0.50), (0.03, 0.00)),
            ((0.03, 0.50), (0.03, 0.00)),
            ((0.06, 0.53), (0.00, 0.03)),
            ((0.06, 0.56), (0.00, 0.03)),
        ]
        for index, (x_positions, speeds) in enumerate(search_frames):
            positions = {
                "tb3_0": (x_positions[0], 0.0),
                "tb3_1": (x_positions[1], 0.0),
            }
            twists = {
                robot: _twist(speed)
                for robot, speed in zip(self.robots, speeds)
            }
            self.metrics._transport_discovery_motion(
                positions, twists, {"phase": "SEARCH"}, index * 0.1,
                index * 0.1,
            )

        self.metrics.transport_discovery_notice = {'announced': True}
        approach_positions = {
            "tb3_0": (0.07, 0.0), "tb3_1": (0.57, 0.0)
        }
        self.metrics._transport_discovery_motion(
            approach_positions,
            {name: _twist(0.03) for name in self.robots},
            {"phase": "APPROACH"}, 0.5, 0.5,
        )

        response = self.metrics.report()["transport_discovery_response"]
        failures = LIVE.transport_search_motion_failures(
            response, 2, require_search=True
        )

        self.assertEqual(1, response["peak_simultaneous_search_movers"])
        self.assertEqual(2, response["peak_simultaneous_rendezvous_movers"])
        self.assertEqual(1, len(failures))
        self.assertIn("simultaneously", failures[0])

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

    def test_required_search_rejects_an_unsupported_window(self):
        response = {
            "simultaneous_motion_window_supported": False,
            "search_motion_window_supported": False,
            "peak_simultaneous_movers": 0,
            "robots_not_observed_moving_during_search": list(self.robots),
        }

        failures = LIVE.transport_search_motion_failures(
            response, len(self.robots), require_search=True
        )

        self.assertEqual(1, len(failures))
        self.assertIn("active search", failures[0])

    def test_every_transport_scenario_starts_outside_initial_sensor_range(self):
        cases = [
            case for case in LIVE.SCENARIOS
            if case["behavior"] == "transport"
        ]

        self.assertEqual([1, 2, 3, 4, 10], [case["count"] for case in cases])
        self.assertTrue(all(case.get("require_search") is True for case in cases))
        self.assertTrue(all(case.get("object_start") == (-3.5, -3.0) for case in cases))

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

    def test_declared_docking_cannot_relabel_a_filtered_safety_contact(self):
        attribution = LIVE.collision_attribution_report(
            1, {"tb3_0": 1}, 8, set(), [{
                "robot_id": "tb3_0",
                "phase": "PUSH",
                "docking_window_open": True,
            }]
        )
        classification = LIVE.classify_contact_episodes(
            "transport", 1, self._contact_report(), self.args, attribution
        )

        self.assertEqual("unexpected_contact", classification["classification"])
        self.assertEqual(1, classification["raw_collision_count_delta"])
        self.assertEqual(
            0,
            classification["classified_expected_contact_count_delta"],
        )
        self.assertEqual(
            1,
            classification["classified_unexpected_contact_count_delta"],
        )
        self.assertTrue(classification["hard_failure"])

    def test_early_collision_is_not_absolved_by_later_valid_docking(self):
        attribution = LIVE.collision_attribution_report(
            2, {"tb3_0": 1, "tb3_2": 1}, 10, set(), [
                {
                    "robot_id": "tb3_2",
                    "phase": "APPROACH",
                    "docking_window_open": False,
                },
                {
                    "robot_id": "tb3_0",
                    "phase": "PUSH",
                    "docking_window_open": True,
                },
            ]
        )

        classification = LIVE.classify_contact_episodes(
            "transport", 2, self._contact_report(), self.args, attribution
        )

        self.assertEqual(
            "unexpected_contact", classification["classification"]
        )
        self.assertEqual(2, classification[
            "classified_unexpected_contact_count_delta"
        ])
        self.assertEqual(0, classification[
            "classified_expected_contact_count_delta"
        ])
        self.assertTrue(classification["hard_failure"])

    def test_incomplete_temporal_attribution_cannot_be_called_docking(self):
        attribution = LIVE.collision_attribution_report(
            2, {"tb3_0": 1}, 10, set(), [{
                "robot_id": "tb3_0",
                "phase": "PUSH",
                "docking_window_open": True,
            }]
        )

        classification = LIVE.classify_contact_episodes(
            "transport", 2, self._contact_report(), self.args, attribution
        )

        self.assertFalse(classification["temporal_attribution_complete"])
        self.assertEqual(
            "unexpected_contact", classification["classification"]
        )
        self.assertEqual(
            2,
            classification["classified_unexpected_contact_count_delta"],
        )
        self.assertEqual(
            0,
            classification["classified_expected_contact_count_delta"],
        )
        self.assertTrue(classification["hard_failure"])

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
