#!/usr/bin/env python3
"""Small contract tests for the visible production acceptance driver."""

from __future__ import annotations

import importlib.util
import sys
import threading
import types
import unittest
from pathlib import Path


SCRIPT = Path(__file__).with_name("robotswarm-visible-e2e.py")


def load_driver():
    """Load the hyphenated script without requiring browser packages in CI."""
    if "websocket" not in sys.modules:
        websocket = types.ModuleType("websocket")
        websocket.create_connection = None
        sys.modules["websocket"] = websocket
    if "PIL" not in sys.modules:
        pil = types.ModuleType("PIL")
        pil.Image = types.SimpleNamespace(Image=object)
        sys.modules["PIL"] = pil

    spec = importlib.util.spec_from_file_location("robotswarm_visible_e2e", SCRIPT)
    if spec is None or spec.loader is None:
        raise RuntimeError("could not load the visible acceptance driver")
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


DRIVER = load_driver()


class TaskSelectionDouble:
    def __init__(self):
        self.clicks = []
        self.waits = []

    def click_selector(self, selector, description):
        self.clicks.append((selector, description))

    def wait_js(self, condition, timeout, description):
        self.waits.append((condition, timeout, description))


class StopSessionDouble:
    def __init__(self):
        self.created_session = True
        self.create_requested = True
        self.buttons = []
        self.waits = []
        self.confirmed = False
        self.inventory_reads = 0
        self.direct_stops = []

    def has_button(self, text):
        if text == DRIVER.STOP_BUTTON:
            return True
        if text == DRIVER.CREATE_BUTTON:
            return self.confirmed
        return False

    def click_button(self, text):
        self.buttons.append(text)
        if text == "Detener y liberar":
            self.confirmed = True

    def wait_js(self, condition, timeout, description):
        self.waits.append((condition, timeout, description))

    def _occupying_sessions(self):
        self.inventory_reads += 1
        return []

    def _stop_session_via_api(self, session_id):
        self.direct_stops.append(session_id)


class UncertainCreationDouble:
    def __init__(self):
        self.created_session = False
        self.create_requested = True
        self.inventory_reads = 0
        self.direct_stops = []
        self.inventory = [
            [
                {"id": "11111111-1111-4111-8111-111111111111", "state": "Ready"},
                {"id": "22222222-2222-4222-8222-222222222222", "state": "Stopping"},
            ],
            [],
        ]

    def has_button(self, text):
        return text == DRIVER.CREATE_BUTTON

    def _occupying_sessions(self):
        self.inventory_reads += 1
        return self.inventory.pop(0)

    def _stop_session_via_api(self, session_id):
        self.direct_stops.append(session_id)


class SessionApiCdp:
    def __init__(self):
        self.expression = ""
        self.await_promise = False

    def evaluate(self, expression, *, await_promise=False, timeout=30):
        self.expression = expression
        self.await_promise = await_promise
        return {"authenticated": True, "status": 200, "body": []}


class ViewerLifecycleDouble:
    def __init__(self):
        self.buttons = []
        self.waits = []
        self.status_samples = 0

    def task_status(self):
        self.status_samples += 1
        return {"state": "Running", "outcome": None, "progressPercent": 12}

    def click_button(self, text):
        self.buttons.append(text)

    def wait_js(self, condition, timeout, description):
        self.waits.append((condition, timeout, description))

    def has_button(self, text):
        return text == DRIVER.STOP_BUTTON


class PrivacyCdp:
    def __init__(self):
        self.expression = ""

    def evaluate(self, expression):
        self.expression = expression
        return True


class PrivacyDouble:
    expected_origin = "https://rs.zerav.la"

    def __init__(self):
        self.cdp = PrivacyCdp()

    def current_page(self):
        return {
            "origin": self.expected_origin,
            "path": "/apps/accounts/all",
            "url": self.expected_origin + "/apps/accounts/all",
            "title": "RobotSwarm",
        }


class VisibleAcceptanceContractTests(unittest.TestCase):
    def test_task_cards_are_selected_as_accessible_radios(self):
        ui = TaskSelectionDouble()

        DRIVER.RobotSwarmUi.select_task_type(ui, "Figure")

        self.assertEqual(
            ui.clicks,
            [('[data-testid="task-option-Figure"]', "task type Figure")],
        )
        self.assertIn("aria-checked", ui.waits[0][0])

    def test_login_uses_the_stable_session_panel_marker(self):
        source = SCRIPT.read_text(encoding="utf-8")

        self.assertEqual(DRIVER.WORKSPACE_SELECTOR, '[data-testid="session-panel"]')
        self.assertIn("document.querySelector({json.dumps(WORKSPACE_SELECTOR)})", source)

    def test_session_cleanup_confirms_the_destructive_dialog(self):
        ui = StopSessionDouble()

        result = DRIVER.RobotSwarmUi.stop_created_session(ui, timeout=0.1)

        self.assertEqual(ui.buttons, [DRIVER.STOP_BUTTON, "Detener y liberar"])
        self.assertIn("role", ui.waits[0][0])
        self.assertTrue(result["released"])
        self.assertFalse(ui.created_session)
        self.assertFalse(ui.create_requested)
        self.assertEqual(result["verifiedBy"], "authenticated-session-list")
        self.assertEqual(ui.inventory_reads, 1)

    def test_uncertain_creation_is_reconciled_and_all_occupants_are_stopped(self):
        ui = UncertainCreationDouble()

        result = DRIVER.RobotSwarmUi.stop_created_session(ui, timeout=0.1)

        self.assertTrue(result["released"])
        self.assertTrue(result["reconciledUncertainCreation"])
        self.assertEqual(ui.inventory_reads, 2)
        self.assertEqual(
            ui.direct_stops,
            [
                "11111111-1111-4111-8111-111111111111",
                "22222222-2222-4222-8222-222222222222",
            ],
        )

    def test_session_inventory_uses_a_fresh_authenticated_backend_read(self):
        cdp = SessionApiCdp()
        ui = types.SimpleNamespace(cdp=cdp)

        result = DRIVER.RobotSwarmUi._session_api_request(ui, "GET")

        self.assertEqual(result, [])
        self.assertTrue(cdp.await_promise)
        self.assertIn("https://robot.zerav.la/api/sessions", cdp.expression)
        self.assertIn("jwt_access_token", cdp.expression)
        self.assertIn("Authorization", cdp.expression)
        self.assertIn("cache: 'no-store'", cdp.expression)
        self.assertIn("credentials: 'omit'", cdp.expression)

    def test_parallel_waits_for_running_siblings_after_a_failure(self):
        stop_event = threading.Event()
        sibling_started = threading.Event()
        allow_sibling_finish = threading.Event()
        sibling_finished = threading.Event()
        returned = threading.Event()
        observed = {}
        users = [
            types.SimpleNamespace(label="A", ui=types.SimpleNamespace(stop_event=stop_event)),
            types.SimpleNamespace(label="B", ui=types.SimpleNamespace(stop_event=stop_event)),
        ]

        def operation(user):
            if user.label == "A":
                self.assertTrue(sibling_started.wait(1))
                raise RuntimeError("primary failure")
            sibling_started.set()
            while not stop_event.wait(0.01):
                pass
            allow_sibling_finish.wait(1)
            sibling_finished.set()

        def run_parallel():
            try:
                DRIVER.parallel(users, operation)
            except BaseException as exc:  # The test records the original worker failure.
                observed["error"] = exc
            finally:
                returned.set()

        runner = threading.Thread(target=run_parallel)
        runner.start()
        try:
            self.assertTrue(sibling_started.wait(1))
            self.assertTrue(stop_event.wait(1))
            self.assertFalse(returned.wait(0.05))
        finally:
            allow_sibling_finish.set()
            runner.join(2)

        self.assertFalse(runner.is_alive())
        self.assertTrue(sibling_finished.is_set())
        self.assertIsInstance(observed.get("error"), RuntimeError)

    def test_closing_only_the_viewer_keeps_the_task_and_session(self):
        ui = ViewerLifecycleDouble()

        result = DRIVER.RobotSwarmUi.close_viewer_while_task_runs(
            ui, continuity_seconds=0.01
        )

        self.assertEqual(ui.buttons, [DRIVER.CLOSE_VIEWER_BUTTON])
        self.assertTrue(result["viewerClosed"])
        self.assertTrue(result["sessionStillAllocated"])
        self.assertEqual(result["taskAfter"]["state"], "Running")
        self.assertGreater(ui.status_samples, 1)

    def test_capture_redacts_any_email_and_short_session_marker(self):
        ui = PrivacyDouble()

        DRIVER.RobotSwarmUi.prepare_safe_capture(
            ui, "tester@example.invalid", "not-a-real-password"
        )

        expression = ui.cdp.expression
        self.assertIn("hasEmail", expression)
        self.assertIn("hasSessionMarker", expression)
        self.assertIn('input[aria-label="Search"]', expression)
        self.assertIn('button[aria-label="Abrir menú de usuario"]', expression)
        self.assertIn(".MuiAvatar-root", expression)

    def test_production_credentials_are_pinned_to_the_public_frontend(self):
        self.assertEqual(
            DRIVER.PRODUCTION_ORIGIN,
            DRIVER.validate_site("https://rs.zerav.la/apps/GTS/realtime"),
        )
        with self.assertRaisesRegex(DRIVER.DriverError, "only be sent"):
            DRIVER.validate_site("https://attacker.invalid/apps/GTS/realtime")

    def test_signal_flag_is_cleared_before_owned_cleanup(self):
        source = SCRIPT.read_text(encoding="utf-8")
        main_finally = source[source.rindex("    finally:\n") :]

        self.assertLess(
            main_finally.index("stop_event.clear()"),
            main_finally.index("normalize_viewer()"),
        )


if __name__ == "__main__":
    unittest.main()
