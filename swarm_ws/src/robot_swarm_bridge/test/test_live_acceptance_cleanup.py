#!/usr/bin/env python3
"""ROS-free checks for sequential live-acceptance cleanup."""

import contextlib
import io
import json
import threading
import types
import unittest
from unittest import mock

from test_transport_acceptance_metrics import LIVE


def message(data):
    return types.SimpleNamespace(data=data)


def twist(linear=0.0, angular=0.0):
    return types.SimpleNamespace(
        linear=types.SimpleNamespace(x=linear),
        angular=types.SimpleNamespace(z=angular),
    )


def collision_event(
    sequence,
    robot="tb3_0",
    phase="PUSH",
    task_id="case-1",
    source_sim_time=None,
    source_wall_time=None,
):
    return {
        "sequence": sequence,
        "robot_id": robot,
        "task_id": task_id,
        "task_type": "transport",
        "task_phase": phase,
        "source_id": "transport-source-a",
        "source_sequence": sequence,
        "source_control_sequence": sequence,
        "source_sim_time": (
            float(sequence)
            if source_sim_time is None else source_sim_time
        ),
        "source_wall_time": (
            float(sequence)
            if source_wall_time is None else source_wall_time
        ),
    }


class FakeSubscriber:
    def __init__(self, topic="", callback=None):
        self.topic = topic
        self.callback = callback
        self.unregistered = False

    def unregister(self):
        self.unregistered = True


class LiveAcceptanceCleanupTest(unittest.TestCase):
    def make_harness(self):
        harness = LIVE.AcceptanceHarness.__new__(LIVE.AcceptanceHarness)
        harness.lock = threading.RLock()
        harness.roster = ["tb3_0", "tb3_1", "tb3_2"]
        harness.cmd_vel_subscribers = {
            name: FakeSubscriber("/{}/cmd_vel".format(name))
            for name in harness.roster
        }
        harness.cmd_vel_subscriber_tokens = {
            name: index + 1 for index, name in enumerate(harness.roster)
        }
        harness.next_cmd_vel_subscriber_token = 3
        harness.cmd_velocities = {
            name: {"linear_x": 0.1, "angular_z": 0.0, "wall_time": 1.0}
            for name in harness.roster
        }
        harness.robot_collision_active = {}
        harness.collision_event_stream_seen = False
        harness.collision_event_last_sequence = 0
        harness.active_collision_robot_ids = set()
        harness.active_collision_episode_counts = {}
        harness.active_collision_episode_records = []
        harness.active_collision_status_samples = 0
        harness.active_collision_missing_robots = set()
        harness.active_collision_protocol_errors = []
        harness.active_collision_event_fingerprints = {}
        harness.active_collision_duplicate_count = 0
        harness.active_collision_baseline_sequence = 0
        harness.active_collision_next_sequence = 1
        harness.active_collision_expected_task_type = None
        harness.active_collision_source_id = None
        harness.active_collision_source_last_sequence = None
        harness.active_collision_source_last_control_sequence = 0
        harness.active_task_id = None
        harness.swarm_task = {}
        harness.behavior_status = {
            "formation": {}, "follow": {}, "transport": {}
        }
        harness.behavior_status_sequences = {
            "formation": 0, "follow": 0, "transport": 0
        }
        harness.behavior_status_wall_times = {
            "formation": None, "follow": None, "transport": None
        }
        harness.sim_time = 0.0
        harness.active_case_wall_start = None
        harness.active_case_sim_start = None
        harness.active_transport_phase = None
        harness.active_transport_phase_started_wall = None
        harness.active_transport_phase_error = None
        harness.active_transport_phase_timeline = []
        harness.active_transport_phase_timeline_dropped = 0
        harness.model_names = {"world", *harness.roster}
        harness.emergency_stop = False
        harness.collision_count = 0
        harness.stop_requested = False
        harness.case_cleanup_failures = []
        harness.last_cleanup_failures = []
        harness.args = types.SimpleNamespace(
            delete_after=False,
            task_id=None,
        )
        return harness

    def test_transport_phase_timeline_is_correlated_relative_and_compact(self):
        harness = self.make_harness()
        harness.active_task_id = "case-1"
        harness.active_case_wall_start = 10.0
        harness.active_case_sim_start = 100.0
        harness.sim_time = 106.0

        harness._record_transport_phase(
            {"task_id": "other", "phase": "SEARCH"}, 11.0
        )
        harness.behavior_status_sequences["transport"] = 4
        harness._record_transport_phase(
            {
                "task_id": "case-1",
                "phase": "SEARCH",
                "robot_assignments": {"unbounded": "not retained"},
            },
            12.5,
        )
        harness._record_transport_phase(
            {"task_id": "case-1", "phase": "SEARCH"}, 13.0
        )
        harness.sim_time = 115.0
        harness.behavior_status_sequences["transport"] = 8
        harness._record_transport_phase(
            {"task_id": "case-1", "phase": "APPROACH"}, 15.0
        )

        report = harness._transport_timeline_report()
        self.assertEqual(0, report["dropped_transitions"])
        self.assertEqual(
            [
                {
                    "phase": "SEARCH",
                    "wall_elapsed_s": 2.5,
                    "sim_elapsed_s": 6.0,
                    "status_sequence": 4,
                },
                {
                    "phase": "APPROACH",
                    "wall_elapsed_s": 5.0,
                    "sim_elapsed_s": 15.0,
                    "status_sequence": 8,
                },
            ],
            report["transitions"],
        )
        self.assertNotIn(
            "robot_assignments", report["transitions"][0]
        )

    def test_n1_transport_wait_fails_on_its_search_phase_budget(self):
        harness = self.make_harness()
        harness.active_task_id = "case-1"
        harness.swarm_task = {"task_id": "case-1", "status": "running"}
        harness.active_transport_phase = "SEARCH"
        harness.active_transport_phase_started_wall = 0.0
        clock = [0.0]

        original_monotonic = LIVE.time.monotonic
        original_sleep = LIVE.time.sleep
        original_shutdown = getattr(LIVE.rospy, "is_shutdown", None)
        LIVE.time.monotonic = lambda: clock[0]
        LIVE.time.sleep = lambda _seconds: clock.__setitem__(0, clock[0] + 50.0)
        LIVE.rospy.is_shutdown = lambda: False
        try:
            with contextlib.redirect_stderr(io.StringIO()):
                response = harness._wait_for_transport_completion(
                    {
                        "timeout": 325.0,
                        "phase_timeouts": {
                            "SEARCH": 245.0,
                            "APPROACH": 20.0,
                            "PUSH": 55.0,
                        },
                    },
                    "case-1",
                )
        finally:
            LIVE.time.monotonic = original_monotonic
            LIVE.time.sleep = original_sleep
            if original_shutdown is None:
                delattr(LIVE.rospy, "is_shutdown")
            else:
                LIVE.rospy.is_shutdown = original_shutdown

        self.assertFalse(response["completion_wait_satisfied"])
        self.assertEqual("phase_timeout", response["termination_reason"])
        self.assertEqual("SEARCH", response["timeout_phase"])
        self.assertEqual(250.0, response["completion_wait_elapsed_wall_s"])
        self.assertEqual(250.0, response["timeout_phase_elapsed_wall_s"])
        self.assertEqual(
            LIVE.TRANSPORT_PHASE_TIMEOUT_TOLERANCE_WALL_S,
            response["transport_phase_timeout_tolerance_wall_s"],
        )

    def test_phase_transition_near_budget_tick_uses_the_small_tolerance(self):
        harness = self.make_harness()
        harness.active_task_id = "case-1"
        harness.swarm_task = {"task_id": "case-1", "status": "running"}
        harness.active_transport_phase = "SEARCH"
        harness.active_transport_phase_started_wall = 0.0
        clock = [245.1]

        def complete_after_tick(_seconds):
            clock[0] += 0.05
            harness.swarm_task["status"] = "completed"
            harness.active_transport_phase = "DONE"
            harness.active_transport_phase_started_wall = clock[0]

        original_monotonic = LIVE.time.monotonic
        original_sleep = LIVE.time.sleep
        original_shutdown = getattr(LIVE.rospy, "is_shutdown", None)
        LIVE.time.monotonic = lambda: clock[0]
        LIVE.time.sleep = complete_after_tick
        LIVE.rospy.is_shutdown = lambda: False
        try:
            response = harness._wait_for_transport_completion(
                {
                    "timeout": 325.0,
                    "phase_timeouts": {"SEARCH": 245.0},
                },
                "case-1",
            )
        finally:
            LIVE.time.monotonic = original_monotonic
            LIVE.time.sleep = original_sleep
            if original_shutdown is None:
                delattr(LIVE.rospy, "is_shutdown")
            else:
                LIVE.rospy.is_shutdown = original_shutdown

        self.assertTrue(response["completion_wait_satisfied"])
        self.assertEqual("task_completed", response["termination_reason"])

    def test_transport_phase_regression_does_not_reset_its_budget(self):
        harness = self.make_harness()
        harness.active_task_id = "case-1"
        harness.active_case_wall_start = 0.0
        harness.active_case_sim_start = 0.0
        harness.swarm_task = {"task_id": "case-1", "status": "running"}

        harness._record_transport_phase(
            {"task_id": "case-1", "phase": "SEARCH"}, 1.0
        )
        harness._record_transport_phase(
            {"task_id": "case-1", "phase": "APPROACH"}, 2.0
        )
        harness._record_transport_phase(
            {"task_id": "case-1", "phase": "SEARCH"}, 3.0
        )

        report = harness._transport_timeline_report()
        self.assertEqual("APPROACH", harness.active_transport_phase)
        self.assertEqual(2.0, harness.active_transport_phase_started_wall)
        self.assertEqual(
            "transport phase regressed from APPROACH to SEARCH",
            report["protocol_error"],
        )
        response = harness._wait_for_transport_completion(
            {"timeout": 325.0, "phase_timeouts": {"APPROACH": 100.0}},
            "case-1",
        )
        self.assertFalse(response["completion_wait_satisfied"])
        self.assertEqual("phase_protocol_error", response["termination_reason"])

    def test_unknown_transport_phase_is_a_protocol_error(self):
        harness = self.make_harness()
        harness.active_task_id = "case-1"
        harness._record_transport_phase(
            {"task_id": "case-1", "phase": "TELEPORT"}, 1.0
        )

        report = harness._transport_timeline_report()
        self.assertEqual("unknown transport phase: TELEPORT", report["protocol_error"])
        self.assertEqual([], report["transitions"])

    def test_transport_wait_reports_a_correlated_completion(self):
        harness = self.make_harness()
        harness.active_task_id = "case-1"
        harness.swarm_task = {"task_id": "case-1", "status": "completed"}
        harness.active_transport_phase = "DONE"
        harness.active_transport_phase_started_wall = 4.0

        original_shutdown = getattr(LIVE.rospy, "is_shutdown", None)
        LIVE.rospy.is_shutdown = lambda: False
        try:
            response = harness._wait_for_transport_completion(
                {"timeout": 325.0, "phase_timeouts": {}}, "case-1"
            )
        finally:
            if original_shutdown is None:
                delattr(LIVE.rospy, "is_shutdown")
            else:
                LIVE.rospy.is_shutdown = original_shutdown

        self.assertTrue(response["completion_wait_satisfied"])
        self.assertEqual("task_completed", response["termination_reason"])
        self.assertIsNone(response["timeout_phase"])

    def test_departed_cmd_vel_monitors_are_unregistered_and_forgotten(self):
        harness = self.make_harness()
        old_zero = harness.cmd_vel_subscribers["tb3_0"]
        old_two = harness.cmd_vel_subscribers["tb3_2"]
        created = []

        def subscribe(topic, _message_type, callback, queue_size):
            self.assertEqual(1, queue_size)
            subscriber = FakeSubscriber(topic, callback)
            created.append(subscriber)
            return subscriber

        original_subscriber = getattr(LIVE.rospy, "Subscriber", None)
        LIVE.rospy.Subscriber = subscribe
        try:
            harness._roster_cb(message("tb3_1,tb3_3"))
        finally:
            if original_subscriber is None:
                delattr(LIVE.rospy, "Subscriber")
            else:
                LIVE.rospy.Subscriber = original_subscriber

        self.assertTrue(old_zero.unregistered)
        self.assertTrue(old_two.unregistered)
        self.assertEqual({"tb3_1", "tb3_3"}, set(harness.cmd_vel_subscribers))
        self.assertEqual({"tb3_1"}, set(harness.cmd_velocities))
        self.assertEqual(["/tb3_3/cmd_vel"], [item.topic for item in created])

        harness._cmd_vel_cb("tb3_0", twist(0.2), token=1)
        self.assertNotIn("tb3_0", harness.cmd_velocities)

        new_token = harness.cmd_vel_subscriber_tokens["tb3_3"]
        harness._cmd_vel_cb("tb3_3", twist(0.2), token=new_token - 1)
        self.assertNotIn("tb3_3", harness.cmd_velocities)
        harness._cmd_vel_cb("tb3_3", twist(0.2), token=new_token)
        self.assertEqual(0.2, harness.cmd_velocities["tb3_3"]["linear_x"])

    def test_matrix_reuses_a_bounded_robot_namespace_set(self):
        self.assertEqual(
            ["tb3_0", "tb3_1", "tb3_2"],
            LIVE.acceptance_robot_ids(3),
        )
        self.assertEqual([], LIVE.acceptance_robot_ids(0))

    def test_collision_attribution_is_corroborated_when_counts_match(self):
        report = LIVE.collision_attribution_report(
            3, {"tb3_0": 1, "tb3_2": 2}, 12, set()
        )

        self.assertTrue(report["attribution_corroborated"])
        self.assertEqual(0, report["unattributed_episode_count"])
        self.assertEqual(["tb3_0", "tb3_2"], report[
            "robots_with_observed_episodes"
        ])

    def test_collision_attribution_stays_explicitly_partial_on_mismatch(self):
        report = LIVE.collision_attribution_report(
            2, {"tb3_0": 1}, 8, {"tb3_1"}
        )

        self.assertFalse(report["attribution_corroborated"])
        self.assertEqual(1, report["unattributed_episode_count"])
        self.assertEqual(["tb3_1"], report["missing_robot_ids"])

    def test_swarm_status_tracks_sealed_rising_edges_once(self):
        harness = self.make_harness()
        harness.active_task_id = "case-1"
        harness.active_transport_phase = "SEARCH"
        harness._begin_collision_attribution(
            ["tb3_0", "tb3_1"], "transport"
        )

        def event(sequence, robot, phase):
            return collision_event(sequence, robot=robot, phase=phase)

        def send(count, events, phase):
            harness._swarm_cb(message(json.dumps({
                "task": {
                    "task_id": "case-1",
                    "status": "running",
                    "result": {"transport": {"phase": phase}},
                },
                "collisions": count,
                "collision_events": {
                    "version": 1,
                    "first_sequence": 1,
                    "last_sequence": count,
                    "events": events,
                },
                "robots": [],
            })))

        first = event(1, "tb3_1", "SEARCH")
        send(1, [first], "SEARCH")
        send(1, [first], "SEARCH")
        harness.active_transport_phase = "PUSH"
        send(2, [first, event(2, "tb3_1", "PUSH")], "PUSH")
        report = harness._finish_collision_attribution(2)

        self.assertTrue(report["attribution_corroborated"])
        self.assertEqual({"tb3_1": 2}, report["episode_count_by_robot"])
        self.assertEqual(
            {"PUSH": 1, "SEARCH": 1}, report["episode_count_by_phase"]
        )
        self.assertEqual(1, report["pre_docking_episode_count"])
        self.assertEqual(1, report["docking_window_episode_count"])
        self.assertTrue(report["temporal_attribution_complete"])
        self.assertEqual(2, report["duplicate_event_count"])

    def test_collision_edge_uses_sealed_phase_not_current_swarm_phase(self):
        harness = self.make_harness()
        harness.active_task_id = "case-1"
        harness._begin_collision_attribution(["tb3_0"], "transport")
        harness.active_case_wall_start = 100.0
        harness.active_case_sim_start = 50.0
        harness.sim_time = 80.0

        # Both current status sources have advanced to PUSH.  The event remains
        # APPROACH because the producer sealed it at the rising edge.
        harness.active_transport_phase = "PUSH"
        with mock.patch.object(LIVE.time, "monotonic", return_value=150.0):
            harness._swarm_cb(message(json.dumps({
                "task": {
                    "task_id": "case-1",
                    "status": "running",
                    "result": {"transport": {"phase": "PUSH"}},
                },
                "collisions": 1,
                "collision_events": {
                    "version": 1,
                    "first_sequence": 1,
                    "last_sequence": 1,
                    "events": [collision_event(
                        1,
                        phase="APPROACH",
                        source_sim_time=53.0,
                        source_wall_time=105.0,
                    )],
                },
                "robots": [{"id": "tb3_0", "collision": True}],
            })))

        report = harness._finish_collision_attribution(1)

        self.assertEqual(
            {"APPROACH": 1}, report["episode_count_by_phase"]
        )
        self.assertEqual(1, report["pre_docking_episode_count"])
        record = report["episode_records"][0]
        self.assertEqual(5.0, record["wall_elapsed_s"])
        self.assertEqual(3.0, record["sim_elapsed_s"])
        self.assertEqual(50.0, record["observed_wall_elapsed_s"])
        self.assertEqual(30.0, record["observed_sim_elapsed_s"])
        self.assertEqual(105.0, record["source_wall_time"])
        self.assertEqual(53.0, record["source_sim_time"])
        self.assertEqual("transport-source-a", record["source_id"])
        classification = LIVE.classify_contact_episodes(
            "transport", 1, {
                "transport_participation": {
                    "tb3_0": {
                        "role": "payload_push",
                        "direct_contact_samples": 1,
                        "companion_contact_samples": 0,
                        "declared_parent_namespaces": [],
                    },
                },
                "model_samples": 1,
                "minimum_static_obstacle_clearance_m": 1.0,
                "minimum_boundary_clearance_m": 1.0,
            },
            types.SimpleNamespace(
                min_center_distance=0.28,
                min_obstacle_clearance=0.13,
                min_boundary_clearance=0.05,
                min_transport_chain_center_distance=0.1,
            ),
            report,
        )
        self.assertEqual(
            "unexpected_contact", classification["classification"]
        )
        self.assertEqual(1, classification[
            "classified_unexpected_contact_count_delta"
        ])
        self.assertTrue(classification["hard_failure"])

    def test_collision_stream_gap_is_fail_closed(self):
        harness = self.make_harness()
        harness.active_task_id = "case-1"
        harness._begin_collision_attribution(["tb3_0"], "transport")

        harness._swarm_cb(message(json.dumps({
            "task": {"task_id": "case-1", "status": "running"},
            "collisions": 2,
            "collision_events": {
                "version": 1,
                "first_sequence": 2,
                "last_sequence": 2,
                "events": [collision_event(2)],
            },
            "robots": [],
        })))

        report = harness._finish_collision_attribution(2)
        self.assertFalse(report["temporal_attribution_complete"])
        self.assertFalse(report["attribution_corroborated"])
        self.assertEqual(1, report["unattributed_episode_count"])
        self.assertTrue(any(
            "dropped sequence" in error
            for error in report["protocol_errors"]
        ))

    def test_transport_collision_event_requires_causal_source_fields(self):
        harness = self.make_harness()
        harness.active_task_id = "case-1"
        harness._begin_collision_attribution(["tb3_0"], "transport")
        event = collision_event(1)
        event.pop("source_wall_time")

        harness._swarm_cb(message(json.dumps({
            "task": {"task_id": "case-1", "status": "running"},
            "collisions": 1,
            "collision_events": {
                "version": 1,
                "first_sequence": 1,
                "last_sequence": 1,
                "events": [event],
            },
            "robots": [],
        })))

        report = harness._finish_collision_attribution(1)
        self.assertFalse(report["temporal_attribution_complete"])
        self.assertEqual([], report["episode_records"])
        self.assertTrue(any(
            "causal source data" in error
            for error in report["protocol_errors"]
        ))

    def test_collision_attribution_memory_is_bounded_and_fails_closed(self):
        harness = self.make_harness()
        harness.active_task_id = "case-1"
        harness._begin_collision_attribution(["tb3_0"], "transport")

        def event(sequence):
            return collision_event(sequence)

        def send(count, first, events):
            harness._swarm_cb(message(json.dumps({
                "task": {"task_id": "case-1", "status": "running"},
                "collisions": count,
                "collision_events": {
                    "version": 1,
                    "first_sequence": first,
                    "last_sequence": count,
                    "events": events,
                },
                "robots": [],
            })))

        send(1, 1, [event(1)])
        send(129, 2, [event(sequence) for sequence in range(2, 130)])

        self.assertLessEqual(
            len(harness.active_collision_event_fingerprints), 128
        )
        self.assertLessEqual(
            len(harness.active_collision_episode_records), 128
        )
        report = harness._finish_collision_attribution(129)
        self.assertFalse(report["temporal_attribution_complete"])
        self.assertFalse(report["attribution_corroborated"])
        self.assertTrue(any(
            "capacity exceeded" in error
            for error in report["protocol_errors"]
        ))

    def test_collision_event_from_another_task_is_fail_closed(self):
        harness = self.make_harness()
        harness.active_task_id = "case-1"
        harness._begin_collision_attribution(["tb3_0"], "transport")

        harness._swarm_cb(message(json.dumps({
            "task": {"task_id": "case-1", "status": "running"},
            "collisions": 1,
            "collision_events": {
                "version": 1,
                "first_sequence": 1,
                "last_sequence": 1,
                "events": [collision_event(
                    1, task_id="previous-case"
                )],
            },
            "robots": [],
        })))

        report = harness._finish_collision_attribution(1)
        self.assertFalse(report["temporal_attribution_complete"])
        self.assertEqual({}, report["episode_count_by_robot"])
        self.assertTrue(any(
            "another task" in error for error in report["protocol_errors"]
        ))

    def test_static_obstacle_gate_defaults_outside_burger_contact(self):
        parser = LIVE.build_parser()

        self.assertEqual(0.13, parser.parse_args([]).min_obstacle_clearance)
        self.assertEqual(
            0.2,
            parser.parse_args([
                "--min-obstacle-clearance", "0.2"
            ]).min_obstacle_clearance,
        )

    def test_unconfirmed_stop_keeps_the_correlated_task_identity(self):
        harness = self.make_harness()
        harness.active_task_id = "case-1"
        harness.swarm_task = {"task_id": "case-1", "status": "running"}
        commands = []
        waits = []
        harness.send = lambda command, parameters=None: commands.append(
            (command, parameters)
        )

        def wait_for(
            _predicate, _timeout, description, continue_after_stop=False,
        ):
            waits.append((description, continue_after_stop))
            return False

        harness.wait_for = wait_for

        with self.assertRaisesRegex(RuntimeError, "correlated stop"):
            harness.stop_task(continue_after_stop=True)

        self.assertEqual("case-1", harness.active_task_id)
        self.assertEqual(
            [("stop_task", {"task_id": "case-1"})], commands
        )
        self.assertEqual([("task stop", True)], waits)

    def test_cleanup_can_confirm_a_stop_after_a_signal(self):
        harness = self.make_harness()
        harness.stop_requested = True
        harness.active_task_id = "case-2"
        harness.swarm_task = {"task_id": "case-2", "status": "running"}
        commands = []

        def send(command, parameters=None):
            commands.append((command, parameters))
            harness.swarm_task["status"] = "STOPPED"

        def wait_for(
            predicate, _timeout, description, continue_after_stop=False,
        ):
            self.assertEqual("task stop", description)
            self.assertTrue(continue_after_stop)
            return predicate()

        harness.send = send
        harness.wait_for = wait_for

        harness.stop_task(continue_after_stop=True)

        self.assertIsNone(harness.active_task_id)
        self.assertEqual(
            [("stop_task", {"task_id": "case-2"})], commands
        )

    def test_case_cleanup_does_not_reset_payload_after_failed_stop(self):
        harness = self.make_harness()
        resets = []

        def stop_task(continue_after_stop=False):
            self.assertTrue(continue_after_stop)
            raise RuntimeError("task stop was not confirmed")

        harness.stop_task = stop_task
        harness.reset_object = lambda: resets.append("payload")

        with self.assertRaisesRegex(RuntimeError, "not confirmed"):
            harness._cleanup_case({"behavior": "transport"})

        self.assertEqual([], resets)

    def test_fleet_delete_confirms_empty_roster_models_and_monitors(self):
        harness = self.make_harness()
        commands = []
        descriptions = []

        def send(command, parameters=None):
            commands.append((command, parameters))
            harness.roster = []
            harness.model_names = {"world", "transport_object"}
            harness.cmd_vel_subscribers = {}
            harness.cmd_velocities = {}

        def wait_for(
            predicate, _timeout, description, continue_after_stop=False,
        ):
            self.assertTrue(continue_after_stop)
            descriptions.append(description)
            return predicate()

        harness.send = send
        harness.wait_for = wait_for

        departed = harness._delete_fleet(continue_after_stop=True)

        self.assertEqual({"tb3_0", "tb3_1", "tb3_2"}, departed)
        self.assertEqual([("delete_robots", {})], commands)
        self.assertEqual(
            ["empty fleet", "Gazebo model deletion", "cmd_vel monitor cleanup"],
            descriptions,
        )

    def test_fleet_delete_rejects_a_residual_gazebo_robot(self):
        harness = self.make_harness()

        def leave_stale_model(_command, _parameters=None):
            harness.roster.clear()
            harness.cmd_vel_subscribers.clear()
            harness.cmd_velocities.clear()

        harness.send = leave_stale_model
        harness.wait_for = (
            lambda predicate, _timeout, _description,
            continue_after_stop=False: predicate()
        )

        with self.assertRaisesRegex(RuntimeError, "remained in Gazebo"):
            harness._delete_fleet(continue_after_stop=True)

    def test_final_cleanup_never_deletes_after_unconfirmed_stop(self):
        harness = self.make_harness()
        harness.args.delete_after = True
        deletions = []

        def fail_stop(continue_after_stop=False):
            self.assertTrue(continue_after_stop)
            raise RuntimeError("unconfirmed task stop")

        harness.stop_task = fail_stop
        harness._delete_fleet = lambda continue_after_stop=False: (
            deletions.append(continue_after_stop)
        )

        failures = harness._final_cleanup()

        self.assertEqual([], deletions)
        self.assertEqual(
            ["task stop failed: unconfirmed task stop"], failures
        )

    def test_cleanup_failure_is_reported_and_returns_nonzero(self):
        harness = self.make_harness()
        harness.roster = []
        harness.model_names = {"world"}
        harness.cmd_vel_subscribers = {}
        harness.cmd_velocities = {}
        harness.sim_time = 0.0
        harness.command_pub = types.SimpleNamespace(
            get_num_connections=lambda: 1
        )
        harness.wait_for = (
            lambda predicate, _timeout, _description,
            continue_after_stop=False: predicate()
        )
        harness._final_cleanup = lambda: [
            "fleet deletion failed: residual tb3_0"
        ]
        output = io.StringIO()

        with contextlib.redirect_stdout(output):
            exit_code = harness.run([])

        summary_line = next(
            line for line in output.getvalue().splitlines()
            if line.startswith("SUMMARY_JSON ")
        )
        summary = json.loads(summary_line.split(" ", 1)[1])
        self.assertEqual(1, exit_code)
        self.assertFalse(summary["all_passed"])
        self.assertFalse(summary["cleanup_passed"])
        self.assertEqual(
            ["fleet deletion failed: residual tb3_0"],
            summary["cleanup_failures"],
        )

    def test_case_cleanup_failure_is_kept_in_the_global_summary(self):
        harness = self.make_harness()
        harness.roster = []
        harness.model_names = {"world"}
        harness.cmd_vel_subscribers = {}
        harness.cmd_velocities = {}
        harness.sim_time = 0.0
        harness.command_pub = types.SimpleNamespace(
            get_num_connections=lambda: 1
        )
        harness.wait_for = (
            lambda predicate, _timeout, _description,
            continue_after_stop=False: predicate()
        )
        def run_case(_case):
            harness.case_cleanup_failures.append(
                "transport_case: payload reset failed"
            )
            return {
                "scenario": "transport_case",
                "passed": False,
                "cleanup_failures": ["payload reset failed"],
            }

        harness.run_case = run_case
        harness._final_cleanup = lambda: []
        output = io.StringIO()

        with contextlib.redirect_stdout(output):
            exit_code = harness.run([{"name": "transport_case"}])

        summary_line = next(
            line for line in output.getvalue().splitlines()
            if line.startswith("SUMMARY_JSON ")
        )
        summary = json.loads(summary_line.split(" ", 1)[1])
        self.assertEqual(1, exit_code)
        self.assertFalse(summary["cleanup_passed"])
        self.assertEqual(
            ["transport_case: payload reset failed"],
            summary["cleanup_failures"],
        )

    def test_case_cleanup_failure_survives_a_later_case_exception(self):
        harness = self.make_harness()
        harness.roster = []
        harness.model_names = {"world"}
        harness.cmd_vel_subscribers = {}
        harness.cmd_velocities = {}
        harness.sim_time = 0.0
        harness.command_pub = types.SimpleNamespace(
            get_num_connections=lambda: 1
        )
        harness.wait_for = (
            lambda predicate, _timeout, _description,
            continue_after_stop=False: predicate()
        )

        def fail_after_cleanup(_case):
            harness.case_cleanup_failures.append(
                "transport_case: payload reset failed"
            )
            raise RuntimeError("result serialization failed")

        harness.run_case = fail_after_cleanup
        harness._final_cleanup = lambda: []

        with self.assertRaisesRegex(RuntimeError, "serialization"):
            harness.run([{"name": "transport_case"}])

        self.assertEqual(
            ["transport_case: payload reset failed"],
            harness.last_cleanup_failures,
        )


if __name__ == "__main__":
    unittest.main()
