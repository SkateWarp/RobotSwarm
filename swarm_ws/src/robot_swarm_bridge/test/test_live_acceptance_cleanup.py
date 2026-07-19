#!/usr/bin/env python3
"""ROS-free checks for sequential live-acceptance cleanup."""

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
        harness.emergency_stop = False
        harness.collision_count = 0
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


if __name__ == "__main__":
    unittest.main()
