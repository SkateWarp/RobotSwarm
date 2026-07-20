#!/usr/bin/env python3
"""ROS-free checks for sequential live-acceptance cleanup."""

import contextlib
import io
import json
import threading
import types
import unittest

from test_transport_acceptance_metrics import LIVE


def message(data):
    return types.SimpleNamespace(data=data)


def twist(linear=0.0, angular=0.0):
    return types.SimpleNamespace(
        linear=types.SimpleNamespace(x=linear),
        angular=types.SimpleNamespace(z=angular),
    )


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
        harness.active_collision_robot_ids = set()
        harness.active_collision_previous = {}
        harness.active_collision_episode_counts = {}
        harness.active_collision_status_samples = 0
        harness.active_collision_missing_robots = set()
        harness.active_task_id = None
        harness.swarm_task = {}
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

    def test_swarm_status_tracks_per_robot_rising_edges(self):
        harness = self.make_harness()
        harness.active_task_id = "case-1"
        harness._begin_collision_attribution(["tb3_0", "tb3_1"])

        def send(count, first, second):
            harness._swarm_cb(message(json.dumps({
                "task": {"task_id": "case-1", "status": "running"},
                "collisions": count,
                "robots": [
                    {"id": "tb3_0", "collision": first},
                    {"id": "tb3_1", "collision": second},
                ],
            })))

        send(1, False, True)
        send(1, False, False)
        send(2, False, True)
        report = harness._finish_collision_attribution(2)

        self.assertTrue(report["attribution_corroborated"])
        self.assertEqual({"tb3_1": 2}, report["episode_count_by_robot"])

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
