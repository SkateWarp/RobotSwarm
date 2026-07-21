#!/usr/bin/env python3
"""Contract checks for the API production acceptance driver."""

from __future__ import annotations

import datetime as dt
import importlib.util
import json
import sys
import unittest
from pathlib import Path
from unittest import mock


SCRIPT = Path(__file__).with_name("robotswarm-prod-e2e.py")


def load_driver():
    spec = importlib.util.spec_from_file_location("robotswarm_prod_e2e", SCRIPT)
    if spec is None or spec.loader is None:
        raise RuntimeError("could not load the API acceptance driver")
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


DRIVER = load_driver()


class ReportDouble:
    def __init__(self):
        self.http = []
        self.data = {}

    def add_http(self, *args):
        self.http.append(args)

    def set_path(self, *path, value):
        target = self.data
        for part in path[:-1]:
            target = target.setdefault(part, {})
        target[path[-1]] = value


class TransportDouble:
    def __init__(self, playlist_status=200):
        self.report = ReportDouble()
        self.playlist_status = playlist_status
        self.raw_calls = 0

    def raw(self, *_args, **_kwargs):
        self.raw_calls += 1
        body = b"#EXTM3U\n" if self.playlist_status == 200 else b""
        return DRIVER.HttpResult(self.playlist_status, {}, body, 4)


class UserDouble:
    alias = "user_a"

    def __init__(self, command_state, transport, is_ready=True):
        self.command_state = command_state
        self.transport = transport
        self.is_ready = is_ready

    def call(self, *_args, **_kwargs):
        return None, {
            "command": {"state": self.command_state},
            "isReady": self.is_ready,
        }


def lease():
    expires = dt.datetime.now(dt.timezone.utc) + dt.timedelta(minutes=5)
    return DRIVER.ViewerLease(
        "4cdf7896-aa27-4ba3-9317-8624af4965e9",
        "a" * 43,
        "https://robot.zerav.la/api/viewer/hls/example/index.m3u8",
        expires.isoformat(),
        expires,
    )


class ProductionAcceptanceContractTests(unittest.TestCase):
    @staticmethod
    def race_user(start_status, start_body):
        user = mock.Mock()

        def call(method, _path, **_kwargs):
            if method == "POST":
                return (
                    DRIVER.HttpResult(start_status, {}, b"", 4),
                    start_body,
                )
            if method == "DELETE":
                return DRIVER.HttpResult(200, {}, b"", 5), None
            raise AssertionError(f"unexpected method: {method}")

        user.call.side_effect = call
        return user

    def test_same_session_race_accepts_a_task_that_commits_before_stop(self):
        user = self.race_user(
            202,
            {
                "task": {
                    "id": "6bfaf287-c9e1-4f03-9b41-fbda57c28bb6",
                }
            },
        )

        start_status, stop_status, winner = (
            DRIVER.race_task_start_with_session_stop(user, "session")
        )

        self.assertEqual((202, 200, "task_committed_first"), (
            start_status,
            stop_status,
            winner,
        ))
        post_call = next(
            call for call in user.call.call_args_list if call.args[0] == "POST"
        )
        self.assertTrue(post_call.kwargs["retry_transient_conflict"])
        self.assertTrue(post_call.kwargs["idempotent"])

    def test_same_session_race_accepts_stop_winning_with_known_conflict(self):
        user = self.race_user(
            409,
            {"message": "The session cannot start a task in the current state."},
        )

        result = DRIVER.race_task_start_with_session_stop(user, "session")

        self.assertEqual((409, 200, "stop_committed_first"), result)

    def test_same_session_race_rejects_an_unexplained_conflict(self):
        user = self.race_user(
            409,
            {"code": "serialization_conflict", "retryable": True},
        )

        with self.assertRaisesRegex(
            DRIVER.HarnessFailure,
            "race_task_conflict_unexpected",
        ):
            DRIVER.race_task_start_with_session_stop(user, "session")

    def test_retry_conflict_reuses_the_same_idempotency_key(self):
        report = DRIVER.Report()
        transport = DRIVER.Transport(report, timeout=1.0)
        conflict = DRIVER.HttpResult(
            409,
            {},
            json.dumps(
                {
                    "code": "serialization_conflict",
                    "retryable": True,
                    "message": "Retry the command.",
                }
            ).encode(),
            5,
        )
        accepted = DRIVER.HttpResult(202, {}, b"{}", 7)

        with mock.patch.object(
            transport,
            "_raw_with_idempotency",
            side_effect=[conflict, accepted],
        ) as request, mock.patch.object(DRIVER.time, "sleep") as sleep:
            result, _ = transport.json_call(
                "POST",
                "/api/sessions/example/tasks",
                actor="user_a",
                target="tasks_self",
                name="start_parallel_task",
                expected={202},
                payload={"type": "Figure", "parameters": {}},
                idempotent=True,
                retry_transient_conflict=True,
            )

        self.assertEqual(202, result.status)
        self.assertEqual(2, request.call_count)
        first_key = request.call_args_list[0].kwargs["idempotency_key"]
        second_key = request.call_args_list[1].kwargs["idempotency_key"]
        self.assertEqual(first_key, second_key)
        self.assertTrue(first_key)
        sleep.assert_called_once_with(DRIVER.COMMAND_RETRY_DELAYS_SECONDS[0])
        self.assertEqual(1, len(report.data["idempotency_retries"]))

    def test_retry_conflict_accepts_the_rolling_deploy_message(self):
        legacy = DRIVER.HttpResult(
            409,
            {},
            json.dumps({"message": DRIVER.RETRY_CONFLICT_MESSAGE}).encode(),
            5,
        )

        self.assertTrue(DRIVER.Transport._is_retry_conflict(legacy))

    def test_unrelated_conflict_is_not_retried(self):
        report = DRIVER.Report()
        transport = DRIVER.Transport(report, timeout=1.0)
        conflict = DRIVER.HttpResult(
            409,
            {},
            b'{"message":"The session already has an active task."}',
            5,
        )

        with mock.patch.object(
            transport, "_raw_with_idempotency", return_value=conflict
        ) as request, mock.patch.object(DRIVER.time, "sleep") as sleep:
            with self.assertRaises(DRIVER.HarnessFailure):
                transport.json_call(
                    "POST",
                    "/api/sessions/example/tasks",
                    actor="user_a",
                    target="tasks_self",
                    name="start_parallel_task",
                    expected={202},
                    payload={"type": "Figure", "parameters": {}},
                    idempotent=True,
                    retry_transient_conflict=True,
                )

        self.assertEqual(1, request.call_count)
        sleep.assert_not_called()

    def test_other_idempotent_commands_do_not_auto_retry_transient_conflicts(self):
        report = DRIVER.Report()
        transport = DRIVER.Transport(report, timeout=1.0)
        conflict = DRIVER.HttpResult(
            409,
            {},
            b'{"code":"serialization_conflict","retryable":true}',
            5,
        )

        with mock.patch.object(
            transport, "_raw_with_idempotency", return_value=conflict
        ) as request, mock.patch.object(DRIVER.time, "sleep") as sleep:
            with self.assertRaises(DRIVER.HarnessFailure):
                transport.json_call(
                    "PATCH",
                    "/api/sessions/example/fleet",
                    actor="user_a",
                    target="session_self",
                    name="update_fleet",
                    expected={202},
                    payload={"robotCount": 5},
                    idempotent=True,
                )

        self.assertEqual(1, request.call_count)
        sleep.assert_not_called()

    def test_uncertain_creation_is_reconciled_after_a_late_commit(self):
        session_id = "6bfaf287-c9e1-4f03-9b41-fbda57c28bb6"
        listings = [[], [{
            "id": session_id,
            "state": "Ready",
            "desiredRobotCount": 3,
        }]]
        user = mock.Mock()
        user.call.side_effect = [
            (None, listing) for listing in listings
        ]

        with mock.patch.object(DRIVER.time, "sleep"):
            recovered = DRIVER.wait_for_uncertain_creation(
                user, 5.0, 0.01
            )

        self.assertEqual([session_id], recovered)
        self.assertEqual(2, user.call.call_count)

    def test_cleanup_stops_a_session_found_after_an_uncertain_create(self):
        session_id = "6bfaf287-c9e1-4f03-9b41-fbda57c28bb6"
        report = ReportDouble()
        user = mock.Mock()
        users = {"user_a": user}
        sessions = {}
        uncertain = {"user_a"}

        with mock.patch.object(
            DRIVER,
            "wait_for_uncertain_creation",
            return_value=[session_id],
        ), mock.patch.object(
            DRIVER,
            "wait_session_stopped",
            return_value="Stopped",
        ):
            complete = DRIVER.cleanup_sessions(
                report,
                users,
                sessions,
                uncertain,
                1.0,
                0.01,
            )

        self.assertTrue(complete)
        self.assertEqual(session_id, sessions["user_a"])
        self.assertNotIn("user_a", uncertain)
        user.call.assert_called_once_with(
            "DELETE",
            f"/api/sessions/{session_id}",
            target="session_self",
            name="cleanup_stop_session",
            expected={200, 404},
        )
        self.assertEqual("Stopped", report.data["cleanup"]["user_a"])

    def test_cleanup_fails_closed_when_reconciliation_is_unavailable(self):
        report = ReportDouble()
        users = {"user_a": mock.Mock()}

        with mock.patch.object(
            DRIVER,
            "wait_for_uncertain_creation",
            side_effect=DRIVER.HarnessFailure(
                "cleanup_session_reconciliation_unavailable",
                "cleanup",
            ),
        ):
            complete = DRIVER.cleanup_sessions(
                report,
                users,
                {},
                {"user_a"},
                1.0,
                0.01,
            )

        self.assertFalse(complete)
        self.assertEqual(
            "reconciliation_failed",
            report.data["cleanup"]["user_a"],
        )

    def test_uncertain_creation_requires_a_fresh_final_listing(self):
        session_id = "6bfaf287-c9e1-4f03-9b41-fbda57c28bb6"
        user = mock.Mock()
        user.call.side_effect = [
            (None, []),
            (None, [{
                "id": session_id,
                "state": "Ready",
                "desiredRobotCount": 3,
            }]),
        ]
        clock = [0.0, 0.0, 1.1]

        with mock.patch.object(
            DRIVER.time, "monotonic", side_effect=clock
        ), mock.patch.object(DRIVER.time, "sleep"):
            recovered = DRIVER.wait_for_uncertain_creation(
                user, 1.0, 0.1
            )

        self.assertEqual([session_id], recovered)
        self.assertEqual(
            "cleanup_final_session_reconciliation",
            user.call.call_args_list[-1].kwargs["name"],
        )

    def test_uncertain_creation_fails_closed_when_final_listing_fails(self):
        user = mock.Mock()
        user.call.side_effect = [
            (None, []),
            DRIVER.HarnessFailure("network_error", "network"),
        ]
        clock = [0.0, 0.0, 1.1]

        with mock.patch.object(
            DRIVER.time, "monotonic", side_effect=clock
        ), mock.patch.object(DRIVER.time, "sleep"):
            with self.assertRaisesRegex(
                DRIVER.HarnessFailure,
                "network_error",
            ):
                DRIVER.wait_for_uncertain_creation(user, 1.0, 0.1)

    def test_failed_viewer_command_stops_playlist_polling_immediately(self):
        transport = TransportDouble(playlist_status=404)
        user = UserDouble("Failed", transport)

        with self.assertRaises(DRIVER.HarnessFailure) as raised:
            DRIVER.wait_playlist(user, "session", lease(), 30.0, 0.01)

        self.assertEqual("viewer_command_failed", raised.exception.code)
        self.assertEqual(0, transport.raw_calls)
        self.assertEqual(1, len(transport.report.http))

    def test_completed_viewer_command_accepts_an_hls_playlist(self):
        transport = TransportDouble(playlist_status=200)
        user = UserDouble("Completed", transport)

        result = DRIVER.wait_playlist(user, "session", lease(), 1.0, 0.01)

        self.assertEqual(200, result.status)
        self.assertEqual(1, transport.raw_calls)

    def test_pending_command_cannot_reuse_an_old_playlist(self):
        transport = TransportDouble(playlist_status=200)
        user = UserDouble("Running", transport, is_ready=False)

        with self.assertRaises(DRIVER.HarnessFailure) as raised:
            DRIVER.wait_playlist(user, "session", lease(), 0.02, 0.005)

        self.assertEqual("viewer_playlist_timeout", raised.exception.code)
        self.assertEqual(0, transport.raw_calls)

    def test_fast_figure_uses_recorded_intervals_when_first_poll_misses_running(self):
        tasks = {
            "figure": {"id": "figure", "state": "Completed"},
            "follow": {"id": "follow", "state": "Running"},
        }

        def read_sample(_barrier, _user, _session_id, task_id):
            return tasks[task_id]

        with mock.patch.object(DRIVER, "read_task_synchronized", side_effect=read_sample):
            figure, follow, sampled_together = DRIVER.wait_figure_while_following(
                {"user_a": mock.Mock(), "user_b": mock.Mock()},
                {"user_a": "session-a", "user_b": "session-b"},
                {"user_a": "figure", "user_b": "follow"},
                1.0,
                0.01,
            )

        self.assertEqual("Completed", figure["state"])
        self.assertEqual("Running", follow["state"])
        self.assertFalse(sampled_together)


if __name__ == "__main__":
    unittest.main()
