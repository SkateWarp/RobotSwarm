#!/usr/bin/env python3
"""Contract checks for the API production acceptance driver."""

from __future__ import annotations

import datetime as dt
import importlib.util
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


if __name__ == "__main__":
    unittest.main()
