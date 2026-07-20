#!/usr/bin/env python3
"""Contract tests for the visible sections acceptance harness."""

from __future__ import annotations

import argparse
import importlib.util
import sys
import tempfile
import types
import unittest
from pathlib import Path
from unittest import mock


SCRIPT = Path(__file__).with_name("robotswarm-sections-e2e.py")


def load_harness():
    """Load both hyphenated scripts without installing browser packages in CI."""
    if "websocket" not in sys.modules:
        websocket = types.ModuleType("websocket")
        websocket.create_connection = None
        sys.modules["websocket"] = websocket
    if "PIL" not in sys.modules:
        pil = types.ModuleType("PIL")
        pil.Image = types.SimpleNamespace(Image=object)
        sys.modules["PIL"] = pil

    spec = importlib.util.spec_from_file_location("robotswarm_sections_e2e", SCRIPT)
    if spec is None or spec.loader is None:
        raise RuntimeError("could not load the sections acceptance harness")
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


HARNESS = load_harness()


class CaptureCdp:
    def __init__(self):
        self.expressions = []

    def evaluate(self, expression):
        self.expressions.append(expression)
        return True


class RoleCdp:
    def __init__(self, profile_role, token_role):
        self.value = {"profileRole": profile_role, "tokenRole": token_role}

    def evaluate(self, _expression):
        return self.value


class CaptureUi(HARNESS.SectionsUi):
    def __init__(self):
        self.cdp = CaptureCdp()
        self.expected_origin = "https://rs.zerav.la"

    def current_page(self):
        return {
            "origin": self.expected_origin,
            "path": "/apps/accounts/all",
            "url": self.expected_origin + "/apps/accounts/all",
            "title": "RobotSwarm",
        }


class CleanupCdp:
    def __init__(self):
        self.expressions = []

    def evaluate(self, expression):
        self.expressions.append(expression)
        return {"x": 25, "y": 30}


class CleanupUi(HARNESS.SectionsUi):
    def __init__(self):
        self.cdp = CleanupCdp()
        self.mouse = []
        self.dialogs = []
        self.buttons = []
        self.waits = []

    def visit_section(self, section, timeout):
        return {"path": section.path}

    def _mouse_click(self, x, y):
        self.mouse.append((x, y))

    def wait_dialog(self, title, timeout=12, required_text=None):
        self.dialogs.append((title, required_text))

    def click_button(self, text):
        self.buttons.append(text)

    def wait_js(self, condition, timeout, description):
        self.waits.append((condition, timeout, description))


class ExistingGroupCdp:
    def evaluate(self, expression):
        return 1


class ExistingGroupUi(HARNESS.SectionsUi):
    def __init__(self):
        self.cdp = ExistingGroupCdp()


class ScreenshotUi(HARNESS.SectionsUi):
    def screenshot(self, destination, email, password):
        destination.write_bytes(b"\x89PNG\r\n\x1a\nunit-test")
        return {"file": destination.name}


class SectionsAcceptanceContractTests(unittest.TestCase):
    def test_production_flag_is_mandatory(self):
        options = argparse.Namespace(
            execute_production=False,
            expected_role="User",
            port=9340,
            page_timeout=90,
            deployment_commit="a" * 40,
            chrome=Path("/does/not/matter"),
            url="https://rs.zerav.la/apps/GTS/realtime",
        )

        with self.assertRaisesRegex(HARNESS.DriverError, "--execute-production"):
            HARNESS.validate_options(options)

    def test_single_account_credentials_require_owner_only_file(self):
        with tempfile.TemporaryDirectory() as temporary:
            path = Path(temporary) / "credentials.env"
            path.write_text(
                "TEST_EMAIL=tester@example.invalid\n"
                "TEST_PASSWORD=a-not-real-password\n"
                "TEST_ROLE=Admin\n",
                encoding="utf-8",
            )
            with mock.patch.object(HARNESS.stat, "S_IMODE", return_value=0o600):
                credentials = HARNESS.read_section_credentials(path)

            self.assertEqual(credentials["declaredRole"], "Admin")
            self.assertEqual(credentials["email"], "tester@example.invalid")
            with mock.patch.object(HARNESS.stat, "S_IMODE", return_value=0o644):
                with self.assertRaisesRegex(HARNESS.DriverError, "0600"):
                    HARNESS.read_section_credentials(path)

    def test_existing_two_user_credentials_reuse_account_a(self):
        with tempfile.TemporaryDirectory() as temporary:
            path = Path(temporary) / "credentials.env"
            path.write_text(
                "TEST_A_ID=101\n"
                "TEST_A_EMAIL=account-a@example.invalid\n"
                "TEST_A_PASSWORD=not-a-real-password-a\n"
                "TEST_B_ID=202\n"
                "TEST_B_EMAIL=account-b@example.invalid\n"
                "TEST_B_PASSWORD=not-a-real-password-b\n",
                encoding="utf-8",
            )

            with mock.patch.object(HARNESS.stat, "S_IMODE", return_value=0o600):
                credentials = HARNESS.read_section_credentials(path)

            self.assertEqual(credentials["email"], "account-a@example.invalid")
            self.assertEqual(credentials["declaredRole"], "")

    def test_user_menu_is_reduced_and_admin_menu_is_complete(self):
        user = HARNESS.expected_menu_presence("User")
        admin = HARNESS.expected_menu_presence("Admin")

        self.assertTrue(user["Historial de tareas"])
        self.assertTrue(user["Control de simulación"])
        self.assertFalse(user["Usuarios"])
        self.assertTrue(all(admin.values()))
        self.assertEqual(HARNESS.validate_menu_presence("User", user), user)

        wrong = dict(user)
        wrong["Robots"] = True
        with self.assertRaises(HARNESS.DriverError):
            HARNESS.validate_menu_presence("User", wrong)

    def test_backend_admin_endpoints_follow_the_real_role_policy(self):
        user = {key: 403 for key in HARNESS.ADMIN_ENDPOINTS}
        admin = {key: 200 for key in HARNESS.ADMIN_ENDPOINTS}

        self.assertEqual(HARNESS.validate_admin_endpoint_statuses("User", user), user)
        self.assertEqual(HARNESS.validate_admin_endpoint_statuses("Admin", admin), admin)
        with self.assertRaises(HARNESS.DriverError):
            HARNESS.validate_admin_endpoint_statuses("User", admin)

    def test_profile_and_jwt_claim_must_agree_on_role(self):
        ui = HARNESS.SectionsUi.__new__(HARNESS.SectionsUi)
        ui.cdp = RoleCdp("User", "User")
        self.assertEqual(ui.current_role(), "User")

        ui.cdp = RoleCdp("User", "Admin")
        self.assertEqual(ui.current_role(), "")

    def test_all_five_sections_have_stable_markers(self):
        self.assertEqual(
            [section.key for section in HARNESS.SECTIONS],
            ["history", "templates", "robots", "groups", "users"],
        )
        self.assertTrue(all(section.marker.startswith('[data-testid="') for section in HARNESS.SECTIONS))
        self.assertFalse(HARNESS.SECTIONS[0].admin_only)
        self.assertTrue(all(section.admin_only for section in HARNESS.SECTIONS[1:]))

    def test_report_sanitizer_covers_operational_identifiers(self):
        sanitizer = HARNESS.SectionsSanitizer(["exact-secret"])
        raw = (
            "tester@example.invalid exact-secret "
            "550e8400-e29b-41d4-a716-446655440000 "
            "10.0.0.126 fd00::126 worker:gpu-lan-01"
        )

        cleaned = sanitizer.text(raw)

        self.assertNotIn("tester@example.invalid", cleaned)
        self.assertNotIn("exact-secret", cleaned)
        self.assertNotIn("550e8400", cleaned)
        self.assertNotIn("10.0.0.126", cleaned)
        self.assertNotIn("fd00::126", cleaned)
        self.assertNotIn("gpu-lan-01", cleaned)
        self.assertEqual(
            sanitizer.text("2026-07-19T23:18:00Z"),
            "2026-07-19T23:18:00Z",
        )

    def test_capture_adds_generic_personal_and_machine_redaction(self):
        ui = CaptureUi()

        ui.prepare_safe_capture("tester@example.invalid", "not-a-real-password")

        expression = "\n".join(ui.cdp.expressions)
        self.assertIn("tbody td", expression)
        self.assertIn('[role="dialog"] input', expression)
        self.assertIn("hasUuid", expression)
        self.assertIn("hasIpv4", expression)
        self.assertIn("hasIpv6", expression)
        self.assertIn("hasWorker", expression)
        self.assertIn("previousRestore", expression)

    def test_safe_screenshot_verifies_mode_and_records_redaction(self):
        ui = ScreenshotUi.__new__(ScreenshotUi)
        with tempfile.TemporaryDirectory() as temporary:
            directory = Path(temporary)
            destination = directory / "safe.png"

            with mock.patch.object(HARNESS.stat, "S_IMODE", return_value=0o600):
                result = ui.safe_screenshot(destination, "email", "password")

            self.assertTrue(all(result["redaction"].values()))

            with mock.patch.object(HARNESS.stat, "S_IMODE", return_value=0o644):
                with self.assertRaisesRegex(HARNESS.DriverError, "0600"):
                    ui.safe_screenshot(destination, "email", "password")

    def test_temporary_group_cleanup_scopes_delete_to_exact_card(self):
        ui = CleanupUi()

        result = ui.delete_temp_group("rs-sections-unit", timeout=15)

        self.assertTrue(result["released"])
        self.assertEqual(ui.mouse, [(25.0, 30.0)])
        self.assertEqual(
            ui.dialogs,
            [("Eliminar grupo", "rs-sections-unit")],
        )
        self.assertEqual(ui.buttons, ["Eliminar grupo"])
        selection = ui.cdp.expressions[0]
        self.assertIn("cards.length !== 1", selection)
        self.assertIn('aria-label=\\"Eliminar rs-sections-unit\\"', selection)
        self.assertIn("deleted temporary group", ui.waits[0][2])

    def test_existing_group_name_is_never_claimed_for_cleanup(self):
        ui = ExistingGroupUi()

        with self.assertRaisesRegex(HARNESS.DriverError, "already exists"):
            ui.create_temp_group("rs-sections-collision", timeout=15)

        self.assertIsNone(getattr(ui, "_owned_temp_group_name", None))

    def test_parser_exposes_role_profile_and_explicit_production_switch(self):
        parser = HARNESS.build_parser()
        actions = {action.dest: action for action in parser._actions}

        self.assertIn("execute_production", actions)
        self.assertIn("expected_role", actions)
        self.assertIn("profile_root", actions)
        self.assertTrue(actions["expected_role"].required)
        self.assertTrue(actions["profile_root"].required)
        self.assertEqual(set(actions["expected_role"].choices), {"User", "Admin"})

    def test_failure_path_keeps_a_sanitized_visible_capture(self):
        source = SCRIPT.read_text(encoding="utf-8")

        self.assertIn('"failureScreenshot"', source)
        self.assertIn('"failure"', source)
        self.assertIn("ui.safe_screenshot(", source)


if __name__ == "__main__":
    unittest.main()
