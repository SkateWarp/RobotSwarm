#!/usr/bin/env python3
"""Small contract tests for the visible production acceptance driver."""

from __future__ import annotations

import importlib.util
import inspect
import sys
import threading
import types
import unittest
from pathlib import Path
from unittest import mock


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


class TaskApiCdp:
    def __init__(self):
        self.expression = ""
        self.await_promise = False

    def evaluate(self, expression, *, await_promise=False, timeout=30):
        self.expression = expression
        self.await_promise = await_promise
        return {
            "authenticated": True,
            "status": 200,
            "body": [
                {
                    "id": "11111111-1111-4111-8111-111111111111",
                    "type": "Figure",
                    "state": "Queued",
                    "createdAt": "2026-07-20T11:00:00",
                }
            ],
        }


class TaskAcceptanceDouble:
    def __init__(self, inventories):
        self.stop_event = threading.Event()
        self.inventories = list(inventories)

    def raise_if_interrupted(self):
        if self.stop_event.is_set():
            raise KeyboardInterrupt

    def _task_api_request(self, session_id):
        self.session_id = session_id
        return self.inventories.pop(0)


class StartOrderDouble:
    def __init__(self):
        self.events = []

    def _task_start_baseline(self):
        self.events.append("baseline")
        return "22222222-2222-4222-8222-222222222222", set()

    def wait_clickable_button(self, text):
        self.events.append(("hit-test", text))

    def click_button(self, text):
        self.events.append(("click", text))

    def _wait_for_new_task(self, session_id, previous_ids, expected_type):
        self.events.append(("get", expected_type))
        return {"type": expected_type, "state": "Queued"}


class BarrierDouble:
    def __init__(self, events, broken=False):
        self.events = events
        self.broken = broken

    def wait(self, timeout):
        self.events.append("barrier")
        if self.broken:
            raise threading.BrokenBarrierError


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

    def test_task_inventory_keeps_the_browser_token_inside_chrome(self):
        cdp = TaskApiCdp()
        ui = types.SimpleNamespace(cdp=cdp)

        result = DRIVER.RobotSwarmUi._task_api_request(
            ui, "22222222-2222-4222-8222-222222222222"
        )

        self.assertEqual(result[0]["type"], "Figure")
        self.assertTrue(cdp.await_promise)
        self.assertIn("/api/sessions/22222222-2222-4222-8222-222222222222/tasks", cdp.expression)
        self.assertIn("jwt_access_token", cdp.expression)
        self.assertIn("credentials: 'omit'", cdp.expression)

    def test_task_start_is_reconciled_as_one_new_backend_task(self):
        task = {
            "id": "33333333-3333-4333-8333-333333333333",
            "type": "FollowLeader",
            "state": "Queued",
        }
        ui = TaskAcceptanceDouble([[task]])

        accepted = DRIVER.RobotSwarmUi._wait_for_new_task(
            ui,
            "22222222-2222-4222-8222-222222222222",
            set(),
            "FollowLeader",
            timeout=0.1,
        )

        self.assertEqual(accepted["type"], "FollowLeader")
        self.assertEqual(accepted["verifiedBy"], "authenticated-task-list")

    def test_task_start_refuses_multiple_new_backend_tasks(self):
        tasks = [
            {
                "id": "33333333-3333-4333-8333-333333333333",
                "type": "Figure",
                "state": "Queued",
            },
            {
                "id": "44444444-4444-4444-8444-444444444444",
                "type": "Figure",
                "state": "Queued",
            },
        ]
        ui = TaskAcceptanceDouble([tasks])

        with self.assertRaisesRegex(DRIVER.DriverError, "more than one task"):
            DRIVER.RobotSwarmUi._wait_for_new_task(
                ui,
                "22222222-2222-4222-8222-222222222222",
                set(),
                "Figure",
                timeout=0.1,
            )

    def test_task_start_makes_one_final_read_at_the_timeout_boundary(self):
        task = {
            "id": "33333333-3333-4333-8333-333333333333",
            "type": "Figure",
            "state": "Queued",
        }
        ui = TaskAcceptanceDouble([[task]])

        accepted = DRIVER.RobotSwarmUi._wait_for_new_task(
            ui,
            "22222222-2222-4222-8222-222222222222",
            set(),
            "Figure",
            timeout=0,
        )

        self.assertEqual("Figure", accepted["type"])
        self.assertEqual([], ui.inventories)

    def test_task_start_refuses_a_different_new_task_type(self):
        task = {
            "id": "33333333-3333-4333-8333-333333333333",
            "type": "CollaborativeTransport",
            "state": "Queued",
        }
        ui = TaskAcceptanceDouble([[task]])

        with self.assertRaisesRegex(DRIVER.DriverError, "different task"):
            DRIVER.RobotSwarmUi._wait_for_new_task(
                ui,
                "22222222-2222-4222-8222-222222222222",
                set(),
                "Figure",
                timeout=0,
            )

    def test_persisted_intervals_prove_a_fast_figure_overlapped_follow(self):
        proof = DRIVER.recorded_task_overlap(
            {
                "A": {
                    "state": "Completed",
                    "startedAt": "2026-07-20T11:00:00",
                    "completedAt": "2026-07-20T11:00:10.0000000",
                },
                "B": {
                    "state": "Running",
                    "startedAt": "2026-07-20T11:00:02",
                    "completedAt": None,
                },
            }
        )

        self.assertIsNotNone(proof)
        self.assertEqual(8000, proof["overlapMilliseconds"])
        self.assertTrue(proof["intervalsOverlap"])

    def test_persisted_intervals_reject_non_overlapping_tasks(self):
        proof = DRIVER.recorded_task_overlap(
            {
                "A": {
                    "state": "Completed",
                    "startedAt": "2026-07-20T11:00:00Z",
                    "completedAt": "2026-07-20T11:00:02Z",
                },
                "B": {
                    "state": "Running",
                    "startedAt": "2026-07-20T11:00:03Z",
                    "completedAt": None,
                },
            }
        )

        self.assertIsNone(proof)

    def test_persisted_intervals_support_the_legacy_missing_start_boundary(self):
        proof = DRIVER.recorded_task_overlap(
            {
                "A": {
                    "state": "Completed",
                    "createdAt": "2026-07-20T11:00:00",
                    "startedAt": None,
                    "completedAt": "2026-07-20T11:00:10",
                },
                "B": {
                    "state": "Running",
                    "startedAt": "2026-07-20T11:00:09",
                    "completedAt": None,
                },
            }
        )

        self.assertEqual("createdAt-legacy-fallback", proof["figureStartBoundary"])
        self.assertEqual(1000, proof["overlapMilliseconds"])

    def test_task_start_orders_baseline_hit_test_barrier_click_and_get(self):
        ui = StartOrderDouble()
        barrier = BarrierDouble(ui.events)

        DRIVER.RobotSwarmUi._start_selected_task(ui, "Figure", barrier)

        self.assertEqual(
            [
                "baseline",
                ("hit-test", DRIVER.START_TASK_BUTTON),
                "barrier",
                ("click", DRIVER.START_TASK_BUTTON),
                ("get", "Figure"),
            ],
            ui.events,
        )

    def test_broken_task_barrier_never_clicks(self):
        ui = StartOrderDouble()
        barrier = BarrierDouble(ui.events, broken=True)

        with self.assertRaisesRegex(DRIVER.DriverError, "barrier was broken"):
            DRIVER.RobotSwarmUi._start_selected_task(ui, "Figure", barrier)

        self.assertNotIn(("click", DRIVER.START_TASK_BUTTON), ui.events)

    def test_covered_button_does_not_dispatch_a_mouse_click(self):
        ui = object.__new__(DRIVER.RobotSwarmUi)
        ui.cdp = mock.Mock()
        ui.cdp.evaluate.return_value = False
        ui._mouse_click = mock.Mock()

        with self.assertRaisesRegex(DRIVER.DriverError, "covered"):
            ui.click_button(DRIVER.START_TASK_BUTTON)

        ui._mouse_click.assert_not_called()

    def test_select_closes_its_overlay_and_button_clicks_are_hit_tested(self):
        source = SCRIPT.read_text(encoding="utf-8")

        self.assertIn("closed option menu", source)
        self.assertIn("aria-expanded", source)
        self.assertIn(".MuiPopover-root", source)
        self.assertIn("blocksPointer", source)
        self.assertIn("document.elementFromPoint(x, y)", source)
        self.assertIn("!element.contains(hit)", source)

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

    def test_interaction_waits_for_control_authorization_before_clicking(self):
        ui = TaskSelectionDouble()

        DRIVER.RobotSwarmUi.enable_viewer_control(ui)

        self.assertEqual(
            ui.clicks,
            [('[aria-label="Activar control interactivo"]', "interactive viewer control")],
        )
        self.assertEqual(30, ui.waits[0][1])
        self.assertIn("!button.disabled", ui.waits[0][0])
        self.assertIn("Desactivar control interactivo", ui.waits[1][0])

    def test_viewer_fps_uses_the_accessible_label(self):
        source = SCRIPT.read_text(encoding="utf-8")

        self.assertIn(
            "fps?.getAttribute('aria-label') || fps?.textContent",
            source,
        )
        self.assertIn(
            "fpsChip?.getAttribute('aria-label') || fpsChip?.textContent",
            source,
        )

    def test_private_scenes_are_compared_before_camera_interaction(self):
        source = SCRIPT.read_text(encoding="utf-8")
        main_flow = source[source.index("        parallel(users, lambda user: user.ui.open_viewer") :]

        self.assertLess(
            main_flow.index("        private_scenes = parallel("),
            main_flow.index("        interaction = parallel("),
        )

    def test_scene_metric_keeps_small_structural_changes(self):
        left = [(120, 120, 120)] * 10_000
        encoder_noise = [(125, 125, 125)] * 10_000
        extra_robots = left.copy()
        extra_robots[:10] = [(20, 20, 20)] * 10

        self.assertEqual(DRIVER.pixel_difference_ratio(left, encoder_noise), 0)
        self.assertEqual(DRIVER.pixel_difference_ratio(left, extra_robots), 0.001)
        self.assertGreater(
            DRIVER.pixel_difference_ratio(left, extra_robots),
            DRIVER.MIN_SCENE_DIFFERENCE_RATIO,
        )

    def test_windows_capture_output_is_locale_tolerant(self):
        source = SCRIPT.read_text(encoding="utf-8")
        helper = source[
            source.index("def run_interruptible_process(") :
            source.index("\n\nclass CdpClient")
        ]

        self.assertIn('encoding="utf-8"', helper)
        self.assertIn('errors="replace"', helper)

    def test_windows_capture_failure_keeps_a_bounded_diagnostic(self):
        result = DRIVER.subprocess.CompletedProcess(
            ["powershell.exe"],
            1,
            stdout="",
            stderr="detalle " * 100,
        )

        error = DRIVER.windows_capture_failure(result, "capture failed")

        self.assertIn("capture failed (exit 1:", str(error))
        self.assertLessEqual(len(str(error)), 430)

    def test_windows_capture_requires_default_desktop_evidence_before_and_after(self):
        payload = {
            "x": 0,
            "y": 0,
            "width": 1920,
            "height": 1040,
            "interactiveDesktopBefore": DRIVER.INTERACTIVE_DESKTOP_PATH,
            "interactiveDesktopAfter": DRIVER.INTERACTIVE_DESKTOP_PATH,
        }
        result = DRIVER.subprocess.CompletedProcess(
            ["powershell.exe"],
            0,
            stdout="unrelated warning\n" + DRIVER.json.dumps(payload) + "\n",
            stderr="",
        )

        evidence = DRIVER.windows_capture_evidence(result)

        self.assertEqual(evidence["bounds"]["width"], 1920)
        self.assertEqual(
            evidence["interactiveDesktop"],
            {"before": r"WinSta0\Default", "after": r"WinSta0\Default"},
        )

        payload["interactiveDesktopAfter"] = r"WinSta0\Winlogon"
        result.stdout = DRIVER.json.dumps(payload)
        with self.assertRaisesRegex(DRIVER.DriverError, "unlocked Default"):
            DRIVER.windows_capture_evidence(result)

    def test_copy_from_screen_is_guarded_by_read_only_win32_checks(self):
        guard = DRIVER.WINDOWS_INTERACTIVE_DESKTOP_GUARD
        self.assertIn("GetProcessWindowStation", guard)
        self.assertIn("GetThreadDesktop", guard)
        self.assertIn("OpenInputDesktop", guard)
        self.assertIn("GetUserObjectInformationW", guard)
        self.assertIn("UOI_NAME", guard)
        self.assertIn("UOI_IO", guard)
        self.assertIn("ProcessIdToSessionId", guard)
        self.assertIn("WTSGetActiveConsoleSessionId", guard)
        self.assertIn("WTSQuerySessionInformationW", guard)
        self.assertIn("WTS_CONNECT_STATE", guard)
        self.assertIn("WTS_ACTIVE", guard)
        self.assertIn("WTSFreeMemory", guard)
        self.assertIn("CloseDesktop", guard)
        self.assertIn('"WinSta0"', guard)
        self.assertIn('"Default"', guard)
        self.assertNotIn("SwitchDesktop", guard)
        self.assertNotIn("LockWorkStation", guard)

        for capture in (DRIVER.desktop_screenshot, DRIVER.owned_window_screenshot):
            with self.subTest(capture=capture.__name__):
                source = inspect.getsource(capture)
                before = source.index('"before CopyFromScreen"')
                copy = source.index("$graphics.CopyFromScreen")
                after = source.index('"after CopyFromScreen"')
                self.assertLess(before, copy)
                self.assertLess(copy, after)
                self.assertIn("WINDOWS_INTERACTIVE_DESKTOP_GUARD", source)
                self.assertIn("windows_capture_evidence(result)", source)
                self.assertIn("windows_capture_failure(", source)
                self.assertLess(
                    source.index("evidence = windows_capture_evidence(result)"),
                    source.index("write_bytes_secure(destination, raw)"),
                )
                self.assertNotIn("tesseract", source.lower())
                self.assertNotIn("windows.media.ocr", source.lower())

    def test_capture_redacts_any_email_and_short_session_marker(self):
        ui = PrivacyDouble()

        DRIVER.RobotSwarmUi.prepare_safe_capture(
            ui, "tester@example.invalid", "not-a-real-password"
        )

        expression = ui.cdp.expression
        self.assertIn("hasEmail", expression)
        self.assertIn("hasSessionMarker", expression)
        self.assertIn("[data-sensitive]", expression)
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
