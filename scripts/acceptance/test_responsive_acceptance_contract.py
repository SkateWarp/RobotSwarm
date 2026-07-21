#!/usr/bin/env python3
"""Contract tests for the read-only responsive production harness."""

from __future__ import annotations

import argparse
import importlib.util
import inspect
import struct
import sys
import tempfile
import types
import unittest
from pathlib import Path
from unittest import mock


SCRIPT = Path(__file__).with_name("robotswarm-responsive-e2e.py")


def load_harness():
    """Load the browser-backed script without browser packages or network access."""
    if "websocket" not in sys.modules:
        websocket = types.ModuleType("websocket")
        websocket.create_connection = None
        sys.modules["websocket"] = websocket
    if "PIL" not in sys.modules:
        pil = types.ModuleType("PIL")
        pil.Image = types.SimpleNamespace(Image=object)
        sys.modules["PIL"] = pil

    specification = importlib.util.spec_from_file_location(
        "robotswarm_responsive_e2e",
        SCRIPT,
    )
    if specification is None or specification.loader is None:
        raise RuntimeError("could not load the responsive acceptance harness")
    module = importlib.util.module_from_spec(specification)
    sys.modules[specification.name] = module
    specification.loader.exec_module(module)
    return module


HARNESS = load_harness()


class MetricsCdp:
    def __init__(self):
        self.calls = []
        self.evaluations = []

    def call(self, method, parameters=None):
        self.calls.append((method, parameters))
        return {}

    def evaluate(self, expression, *, await_promise=False, timeout=30):
        self.evaluations.append((expression, await_promise, timeout))
        return True


class InspectionCdp:
    def __init__(self, observation):
        self.observation = observation
        self.expression = ""
        self.await_promise = False

    def evaluate(self, expression, *, await_promise=False, timeout=30):
        self.expression = expression
        self.await_promise = await_promise
        return self.observation


class CaptureUi:
    def __init__(self):
        self.cdp = object()
        self.waits = []
        self.screenshots = []

    def wait_js(self, condition, timeout, description):
        self.waits.append((condition, timeout, description))

    def screenshot(self, destination, email, password):
        self.screenshots.append((destination.name, email, password))
        raw = b"\x89PNG\r\n\x1a\n" + b"\x00" * 8 + struct.pack(">II", 768, 900)
        destination.write_bytes(raw)
        return {
            "file": destination.name,
            "sha256": "0" * 64,
            "bytes": len(raw),
            "page": {
                "origin": "https://rs.zerav.la",
                "path": "/apps/GTS/realtime",
                "title": "RobotSwarm",
            },
        }


def valid_observation(width=768, *, scrollbar_width=0):
    visible_width = width - scrollbar_width
    return {
        "layoutViewportWidth": width,
        "innerHeight": 900,
        "visualViewportWidth": float(visible_width),
        "visibleViewportWidth": float(visible_width),
        "devicePixelRatio": 1,
        "clientWidth": visible_width,
        "scrollWidth": visible_width,
        "horizontalOverflowPx": 0,
        "panelVisible": True,
        "panelInsideViewport": True,
        "panelVisibleWidth": visible_width,
        "panelVisibleHeight": 500,
    }


class ResponsiveAcceptanceContractTests(unittest.TestCase):
    def test_width_matrix_is_versioned_and_complete(self):
        self.assertEqual(HARNESS.VIEWPORT_WIDTHS, (360, 768, 1366, 1920))
        self.assertEqual(HARNESS.VIEWPORT_HEIGHT, 900)

    def test_production_flag_and_full_sha_are_mandatory(self):
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            chrome = root / "chrome.exe"
            chrome.touch()
            base = dict(
                execute_production=False,
                deployment_commit="a" * 40,
                port=9350,
                page_timeout=90,
                chrome=chrome,
                profile_root=root,
                url=HARNESS.DEFAULT_URL,
            )

            with self.assertRaisesRegex(HARNESS.DriverError, "--execute-production"):
                HARNESS.validate_options(argparse.Namespace(**base))

            base["execute_production"] = True
            base["deployment_commit"] = "abc123"
            with self.assertRaisesRegex(HARNESS.DriverError, "full lowercase Git SHA"):
                HARNESS.validate_options(argparse.Namespace(**base))

    def test_cli_requires_chrome_profile_and_output(self):
        required = {
            action.dest
            for action in HARNESS.build_parser()._actions
            if getattr(action, "required", False)
        }

        self.assertTrue(
            {"deployment_commit", "chrome", "profile_root", "output_dir"}.issubset(required)
        )

    def test_credentials_are_owner_only_and_account_a_is_reused(self):
        with tempfile.TemporaryDirectory() as temporary:
            credentials = Path(temporary) / "credentials.env"
            credentials.write_text(
                "TEST_A_ID=101\n"
                "TEST_A_EMAIL=account-a@example.invalid\n"
                "TEST_A_PASSWORD=not-a-real-password-a\n"
                "TEST_B_ID=202\n"
                "TEST_B_EMAIL=account-b@example.invalid\n"
                "TEST_B_PASSWORD=not-a-real-password-b\n",
                encoding="utf-8",
            )
            credentials.chmod(0o600)

            with mock.patch.object(HARNESS.VISIBLE.stat, "S_IMODE", return_value=0o600):
                account = HARNESS.load_test_account(credentials)
            self.assertEqual(account["email"], "account-a@example.invalid")

            credentials.chmod(0o644)
            with mock.patch.object(HARNESS.VISIBLE.stat, "S_IMODE", return_value=0o644):
                with self.assertRaisesRegex(HARNESS.DriverError, "0600"):
                    HARNESS.load_test_account(credentials)

    def test_device_metrics_keep_real_chrome_and_exact_css_pixels(self):
        cdp = MetricsCdp()

        HARNESS.apply_device_metrics(cdp, 360)

        method, parameters = cdp.calls[0]
        self.assertEqual(method, "Emulation.setDeviceMetricsOverride")
        self.assertEqual(parameters["width"], 360)
        self.assertEqual(parameters["height"], 900)
        self.assertEqual(parameters["deviceScaleFactor"], 1)
        self.assertFalse(parameters["mobile"])
        self.assertTrue(cdp.evaluations[0][1])

    def test_layout_inspection_returns_only_bounded_geometry(self):
        cdp = InspectionCdp(valid_observation())

        observation = HARNESS.inspect_control_page(cdp)

        self.assertEqual(observation["horizontalOverflowPx"], 0)
        self.assertIn("session-panel", cdp.expression)
        self.assertIn("panelBox.right <= visibleViewportWidth", cdp.expression)
        self.assertNotIn("innerText", cdp.expression)
        self.assertTrue(cdp.await_promise)

    def test_reserved_vertical_scrollbar_does_not_change_requested_layout_width(self):
        observation = valid_observation(360, scrollbar_width=15)

        HARNESS.validate_viewport_observation(360, observation)

        self.assertEqual(observation["layoutViewportWidth"], 360)
        self.assertEqual(observation["visibleViewportWidth"], 345)

    def test_viewport_gate_rejects_wrong_width_overflow_and_hidden_panel(self):
        HARNESS.validate_viewport_observation(768, valid_observation())

        cases = (
            ({**valid_observation(), "layoutViewportWidth": 767}, "different layout"),
            ({**valid_observation(), "horizontalOverflowPx": 1}, "overflows"),
            ({**valid_observation(), "panelVisible": False}, "not visible"),
            ({**valid_observation(), "panelInsideViewport": False}, "clipped"),
        )
        for observation, message in cases:
            with self.subTest(message=message):
                with self.assertRaisesRegex(HARNESS.DriverError, message):
                    HARNESS.validate_viewport_observation(768, observation)

    def test_width_error_reports_requested_and_real_browser_values(self):
        observation = valid_observation(767, scrollbar_width=15)

        with self.assertRaises(HARNESS.DriverError) as raised:
            HARNESS.validate_viewport_observation(768, observation)

        message = str(raised.exception)
        self.assertIn("requested=768px", message)
        self.assertIn("layout=767px", message)
        self.assertIn("visual=752px", message)
        self.assertIn("client=752px", message)
        self.assertIn("visible=752px", message)

    def test_capture_writes_one_sanitized_png_for_the_requested_width(self):
        ui = CaptureUi()
        account = {"email": "account-a@example.invalid", "password": "secret"}
        with tempfile.TemporaryDirectory() as temporary:
            output = Path(temporary)
            with mock.patch.object(HARNESS, "apply_device_metrics") as apply_metrics:
                with mock.patch.object(
                    HARNESS,
                    "inspect_control_page",
                    return_value=valid_observation(),
                ):
                    result = HARNESS.capture_viewport(
                        ui,
                        output,
                        "run-1",
                        account,
                        768,
                    )

            apply_metrics.assert_called_once_with(ui.cdp, 768)
            self.assertEqual(result["requestedCssWidth"], 768)
            self.assertEqual(result["screenshot"]["width"], 768)
            self.assertEqual(ui.screenshots[0][0], "run-1-control-768px.png")

    def test_cleanup_requires_port_process_and_profile_release(self):
        complete = {"portFree": True, "processExited": True, "profileRemoved": True}
        self.assertTrue(HARNESS.browser_cleanup_passed(complete))

        for field in complete:
            with self.subTest(field=field):
                incomplete = {**complete, field: False}
                self.assertFalse(HARNESS.browser_cleanup_passed(incomplete))

    def test_main_is_read_only_and_closes_the_owned_browser_in_finally(self):
        source = inspect.getsource(HARNESS.main)
        finally_block = source[source.index("    finally:") :]

        self.assertNotIn(".create_session(", source)
        self.assertNotIn(".start_", source)
        self.assertNotIn(".click_button(", source)
        self.assertNotIn("_session_api_request", source)
        self.assertIn("chrome.close_owned()", finally_block)
        self.assertIn("Emulation.clearDeviceMetricsOverride", finally_block)

    def test_no_headless_or_gpu_disabling_argument_is_present(self):
        source = SCRIPT.read_text(encoding="utf-8")

        self.assertNotIn('"--headless', source)
        self.assertNotIn('"--disable-gpu', source)
        self.assertEqual(source.count('"schemaVersion":'), 1)


if __name__ == "__main__":
    unittest.main()
