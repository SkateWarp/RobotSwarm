#!/usr/bin/env python3
"""Aceptación visible de las secciones administrativas de RobotSwarm.

El recorrido usa una ventana normal de Chrome y el mismo controlador CDP del
arnés visual principal. No crea sesiones ROS. La única mutación intencional es
un grupo temporal, creado por la prueba Admin y eliminado siempre en ``finally``.
"""

from __future__ import annotations

import argparse
import datetime as dt
import importlib.util
import json
import os
import re
import secrets
import signal
import stat
import sys
import threading
from dataclasses import dataclass
from pathlib import Path
from typing import Any


HERE = Path(__file__).resolve().parent
VISIBLE_DRIVER = HERE / "robotswarm-visible-e2e.py"


def load_visible_driver():
    """Load the existing hyphenated script as a shared implementation module."""
    module_name = "robotswarm_visible_e2e_shared"
    if module_name in sys.modules:
        return sys.modules[module_name]

    specification = importlib.util.spec_from_file_location(module_name, VISIBLE_DRIVER)
    if specification is None or specification.loader is None:
        raise RuntimeError("The visible RobotSwarm driver could not be loaded")
    module = importlib.util.module_from_spec(specification)
    sys.modules[module_name] = module
    specification.loader.exec_module(module)
    return module


VISIBLE = load_visible_driver()
DriverError = VISIBLE.DriverError

DEFAULT_URL = "https://rs.zerav.la/apps/GTS/realtime"
DEFAULT_CREDENTIALS = VISIBLE.DEFAULT_CREDENTIALS
DEFAULT_OUTPUT = Path("/tmp/robotswarm-sections-e2e-output")
DEFAULT_PORT = 9340


@dataclass(frozen=True)
class Section:
    key: str
    label: str
    path: str
    marker: str
    heading: str
    admin_only: bool = False


SECTIONS = (
    Section(
        "history",
        "Historial",
        "/apps/GTS/taskLogs",
        '[data-testid="task-history-page"]',
        "Historial de tareas",
    ),
    Section(
        "templates",
        "Plantillas",
        "/apps/GTS/task-templates",
        '[data-testid="task-templates-page"]',
        "Plantillas de tareas",
        True,
    ),
    Section(
        "robots",
        "Robots",
        "/apps/GTS/leafSorting",
        '[data-testid="robot-registry-page"]',
        "Robots",
        True,
    ),
    Section(
        "groups",
        "Grupos",
        "/apps/GTS/robot-groups",
        '[data-testid="robot-groups-page"]',
        "Grupos de robots",
        True,
    ),
    Section(
        "users",
        "Usuarios",
        "/apps/accounts/all",
        '[data-testid="accounts-page"]',
        "Usuarios",
        True,
    ),
)

MENU_LABELS = (
    "Plantillas de tareas",
    "Historial de tareas",
    "Control de simulación",
    "Robots",
    "Grupos de robots",
    "Usuarios",
)
USER_MENU = {"Historial de tareas", "Control de simulación"}
ADMIN_MENU = set(MENU_LABELS)
ADMIN_ENDPOINTS = {
    "taskTemplates": "https://robot.zerav.la/TaskTemplate",
    "robotGroups": "https://robot.zerav.la/RobotGroups",
}


class SectionsSanitizer(VISIBLE.Sanitizer):
    """Cover identifiers that are useful operationally but unsafe in evidence."""

    def text(self, value: Any) -> str:
        result = super().text(value)
        result = re.sub(
            r"(?i)(?:^|(?<=[^0-9a-f]))[0-9a-f]{8}-[0-9a-f]{4}-[1-5][0-9a-f]{3}-"
            r"[89ab][0-9a-f]{3}-[0-9a-f]{12}(?=$|[^0-9a-f])",
            "[UUID REDACTED]",
            result,
        )
        result = re.sub(
            r"(?<![0-9])(?:[0-9]{1,3}\.){3}[0-9]{1,3}(?![0-9])",
            "[IP REDACTED]",
            result,
        )
        result = re.sub(
            r"(?i)(?<![0-9a-f:])(?:(?:[0-9a-f]{1,4}:){3,7}[0-9a-f]{1,4}|"
            r"[0-9a-f:]*::[0-9a-f:]*)(?![0-9a-f:])",
            "[IP REDACTED]",
            result,
        )
        result = re.sub(
            r"(?i)\b(?:worker|trabajador|host)\s*(?:[:#=-]\s*)?[A-Za-z0-9][A-Za-z0-9._-]+",
            "[WORKER REDACTED]",
            result,
        )
        return result[:1200]


def read_section_credentials(path: Path) -> dict[str, str]:
    """Read account A, or a dedicated single account, without sourcing the file."""
    try:
        details = path.lstat()
    except FileNotFoundError as exc:
        raise DriverError("The sections credential file is missing") from exc
    if stat.S_ISLNK(details.st_mode) or not stat.S_ISREG(details.st_mode):
        raise DriverError("The sections credential path must be a regular file")
    if details.st_uid != os.getuid() or stat.S_IMODE(details.st_mode) != 0o600:
        raise DriverError("The sections credential file must be owner-only (0600)")
    if details.st_size <= 0 or details.st_size > 4096:
        raise DriverError("The sections credential file has an invalid size")

    values: dict[str, str] = {}
    for raw_line in path.read_text(encoding="utf-8").splitlines():
        line = raw_line.strip()
        if not line or line.startswith("#"):
            continue
        key, separator, raw_value = line.partition("=")
        if not separator:
            raise DriverError("The sections credential file contains an invalid line")
        value = raw_value.strip()
        if len(value) >= 2 and value[0] == value[-1] and value[0] in "'\"":
            value = value[1:-1]
        values[key.strip()] = value

    single_account = {"TEST_EMAIL", "TEST_PASSWORD"}
    paired_account = {"TEST_A_EMAIL", "TEST_A_PASSWORD"}
    if single_account.issubset(values):
        email_key = "TEST_EMAIL"
        password_key = "TEST_PASSWORD"
        role_key = "TEST_ROLE"
    elif paired_account.issubset(values):
        email_key = "TEST_A_EMAIL"
        password_key = "TEST_A_PASSWORD"
        role_key = "TEST_A_ROLE"
    else:
        raise DriverError("The sections credential file is incomplete")
    if not values[email_key] or not values[password_key]:
        raise DriverError("The sections credential file is incomplete")
    declared_role = values.get(role_key, "")
    if declared_role and declared_role not in {"User", "Admin"}:
        raise DriverError(f"{role_key} must be User or Admin")
    return {
        "email": values[email_key],
        "password": values[password_key],
        "declaredRole": declared_role,
    }


def expected_menu_presence(role: str) -> dict[str, bool]:
    allowed = ADMIN_MENU if role == "Admin" else USER_MENU
    return {label: label in allowed for label in MENU_LABELS}


def validate_menu_presence(role: str, observed: dict[str, bool]) -> dict[str, bool]:
    expected = expected_menu_presence(role)
    mismatches = [
        label
        for label in MENU_LABELS
        if bool(observed.get(label)) is not expected[label]
    ]
    if mismatches:
        raise DriverError("The navigation menu does not match the expected account role")
    return expected


def validate_admin_endpoint_statuses(role: str, observed: dict[str, int]) -> dict[str, int]:
    expected_status = 200 if role == "Admin" else 403
    if set(observed) != set(ADMIN_ENDPOINTS) or any(
        status != expected_status for status in observed.values()
    ):
        raise DriverError("The backend authorization policy does not match the expected role")
    return observed


def validate_options(args: argparse.Namespace) -> str:
    if not args.execute_production:
        raise DriverError("Refusing to open the site without --execute-production")
    if args.expected_role not in {"User", "Admin"}:
        raise DriverError("The expected role must be User or Admin")
    if not 1024 <= args.port <= 65535:
        raise DriverError("The CDP port must be non-privileged")
    if not 10 <= args.page_timeout <= 300:
        raise DriverError("The page timeout must be between 10 and 300 seconds")
    if not re.fullmatch(r"[0-9a-f]{40}", args.deployment_commit):
        raise DriverError("The deployment commit must be a full lowercase Git SHA")
    if not args.chrome.is_file():
        raise DriverError("The Chrome executable was not found")
    return VISIBLE.validate_site(args.url)


def write_report_secure(path: Path, value: Any) -> None:
    VISIBLE.write_json_secure(path, value)
    details = path.lstat()
    if (
        stat.S_ISLNK(details.st_mode)
        or not stat.S_ISREG(details.st_mode)
        or details.st_uid != os.getuid()
        or stat.S_IMODE(details.st_mode) != 0o600
    ):
        raise DriverError("The sanitized report is not owner-only (0600)")


class SectionsUi(VISIBLE.RobotSwarmUi):
    """Small page-object layer for the five management sections."""

    def role_evidence(self) -> dict[str, str]:
        evidence = self.cdp.evaluate(
            """
            (() => {
                try {
                    const stored = JSON.parse(localStorage.getItem('jwt_user') || 'null');
                    const token = localStorage.getItem('jwt_access_token') || '';
                    const segment = token.split('.')[1] || '';
                    const padded = segment.replace(/-/g, '+').replace(/_/g, '/')
                        + '='.repeat((4 - segment.length % 4) % 4);
                    const payload = JSON.parse(atob(padded));
                    const claim = payload.role
                        || payload['http://schemas.microsoft.com/ws/2008/06/identity/claims/role'];
                    const normalize = value => {
                        const roles = Array.isArray(value) ? value : [value];
                        if (roles.includes('Admin')) return 'Admin';
                        if (roles.includes('User')) return 'User';
                        return '';
                    };
                    return {
                        profileRole: normalize(stored?.role),
                        tokenRole: normalize(claim),
                    };
                } catch (_) {
                    return {profileRole: '', tokenRole: ''};
                }
            })()
            """
        )
        if not isinstance(evidence, dict):
            return {"profileRole": "", "tokenRole": ""}
        return {
            "profileRole": str(evidence.get("profileRole") or ""),
            "tokenRole": str(evidence.get("tokenRole") or ""),
        }

    def current_role(self) -> str:
        evidence = self.role_evidence()
        if evidence["profileRole"] != evidence["tokenRole"]:
            return ""
        return evidence["tokenRole"]

    def require_role(self, expected_role: str) -> str:
        self.wait_js(
            "Boolean(localStorage.getItem('jwt_user'))",
            15,
            "stored authenticated account",
        )
        observed = self.current_role()
        if observed != expected_role:
            raise DriverError("The authenticated account does not have the expected role")
        return observed

    def require_admin_endpoint_policy(self, role: str) -> dict[str, int]:
        endpoints = json.dumps(ADMIN_ENDPOINTS)
        observed = self.cdp.evaluate(
            f"""
            (async () => {{
                const token = localStorage.getItem('jwt_access_token');
                if (!token) return null;
                const endpoints = {endpoints};
                const entries = await Promise.all(Object.entries(endpoints).map(
                    async ([key, url]) => {{
                        try {{
                            const response = await fetch(url, {{
                                method: 'GET',
                                cache: 'no-store',
                                credentials: 'omit',
                                headers: {{Authorization: `Bearer ${{token}}`}},
                            }});
                            return [key, response.status];
                        }} catch (_) {{
                            return [key, 0];
                        }}
                    }}
                ));
                return Object.fromEntries(entries);
            }})()
            """,
            await_promise=True,
            timeout=30,
        )
        if not isinstance(observed, dict):
            raise DriverError("The backend authorization policy could not be inspected")
        statuses = {key: int(value) for key, value in observed.items()}
        return validate_admin_endpoint_statuses(role, statuses)

    def menu_presence(self) -> dict[str, bool]:
        expression = f"""
            (() => {{
                const wanted = {json.dumps(MENU_LABELS)};
                const normalize = value => (value || '').replace(/\s+/g, ' ').trim();
                const labels = [...document.querySelectorAll(
                    '.fuse-list-item-text-primary, nav a, #fuse-navbar a, #fuse-navbar-side-panel a'
                )].map(item => normalize(item.textContent));
                return Object.fromEntries(wanted.map(label => [label, labels.includes(label)]));
            }})()
        """
        result = self.cdp.evaluate(expression)
        if not isinstance(result, dict):
            raise DriverError("The navigation menu could not be inspected")
        return {label: bool(result.get(label)) for label in MENU_LABELS}

    def require_menu(self, role: str) -> dict[str, bool]:
        self.wait_js(
            "document.querySelectorAll('.fuse-list-item-text-primary, #fuse-navbar a').length > 0",
            30,
            "role-filtered navigation",
        )
        observed = self.menu_presence()
        validate_menu_presence(role, observed)
        return observed

    def visit_section(self, section: Section, timeout: float) -> dict[str, Any]:
        self.navigate(self.expected_origin + section.path)
        marker = json.dumps(section.marker)
        heading = json.dumps(section.heading.casefold())
        self.wait_js(
            f"""
            (() => {{
                const root = document.querySelector({marker});
                if (!root || !root.innerText.toLocaleLowerCase().includes({heading})) return false;
                return !root.querySelector('.MuiCircularProgress-root, .MuiLinearProgress-root, [aria-busy="true"]')
                    && !root.innerText.includes('Cargando cuentas…');
            }})()
            """,
            timeout,
            f"settled {section.label} page",
        )
        state = self.cdp.evaluate(
            f"""
            (() => {{
                const root = document.querySelector({marker});
                if (!root) return null;
                return {{
                    heading: root.innerText.toLocaleLowerCase().includes({heading}),
                    error: Boolean(root.querySelector(
                        '.MuiAlert-standardError, .MuiAlert-filledError, .MuiAlert-outlinedError'
                    )),
                    tableRows: root.querySelectorAll('tbody tr').length,
                    cards: root.querySelectorAll('.MuiCard-root').length,
                    empty: /No hay|Todavía no hay|catálogo está vacío/.test(root.innerText),
                }};
            }})()
            """
        )
        if not isinstance(state, dict) or not state.get("heading"):
            raise DriverError(f"The {section.label} marker was not found")
        if state.get("error"):
            raise DriverError(f"The {section.label} page reported a backend error")
        page = self.current_page()
        if page["path"] != section.path:
            raise DriverError(f"The {section.label} page did not keep its canonical route")
        return {
            "path": page["path"],
            "marker": section.marker,
            "heading": True,
            "tableRows": int(state.get("tableRows") or 0),
            "cards": int(state.get("cards") or 0),
            "emptyState": bool(state.get("empty")),
        }

    def require_admin_redirect(self, section: Section, timeout: float) -> dict[str, Any]:
        if not section.admin_only:
            raise DriverError("Redirect checks are only valid for admin routes")
        self.navigate(self.expected_origin + section.path)
        restricted_path = json.dumps(section.path)
        marker = json.dumps(section.marker)
        self.wait_js(
            f"""
            location.pathname === '/apps/GTS/realtime'
                && location.pathname !== {restricted_path}
                && !document.querySelector({marker})
                && document.querySelector('[data-testid="session-panel"]')
            """,
            timeout,
            f"User redirect away from {section.label}",
        )
        page = self.current_page()
        if page["path"] == section.path or page["path"].startswith("/login"):
            raise DriverError("An admin-only route was not safely rejected")
        return {
            "requestedPath": section.path,
            "redirected": True,
            "destinationPath": page["path"],
            "restrictedMarkerAbsent": True,
        }

    def prepare_safe_capture(
        self,
        email: str,
        password: str,
        title_marker: str | None = None,
    ) -> None:
        super().prepare_safe_capture(email, password, title_marker)
        extended = self.cdp.evaluate(
            """
            (() => {
                const previousRestore = window.__robotswarmRestorePrivate;
                const privateNodes = [];
                const hide = element => {
                    if (!element || privateNodes.some(entry => entry.element === element)) return;
                    privateNodes.push({element, visibility: element.style.visibility});
                    element.style.visibility = 'hidden';
                };

                document.querySelectorAll(
                    'tbody td, [role="dialog"] input, [role="dialog"] textarea, '
                    + 'button[aria-label="Abrir menú de usuario"] .MuiTypography-root, '
                    + 'button[aria-label="Abrir menú de usuario"] .MuiAvatar-root, '
                    + '.user .avatar, '
                    + '[data-testid="robot-registry-page"] .MuiCardContent-root .MuiTypography-root, '
                    + '[data-testid="robot-groups-page"] .MuiCardHeader-content, '
                    + '[data-testid="robot-groups-page"] .MuiCardContent-root .MuiTypography-root'
                ).forEach(hide);
                document.querySelectorAll('pre').forEach(hide);

                const walker = document.createTreeWalker(document.body, NodeFilter.SHOW_TEXT);
                while (walker.nextNode()) {
                    const value = walker.currentNode.nodeValue || '';
                    const hasEmail = /[A-Za-z0-9._%+-]+@[A-Za-z0-9.-]+[.][A-Za-z]{2,}/.test(value);
                    const hasUuid = /(?:^|[^0-9a-f])[0-9a-f]{8}-[0-9a-f]{4}-[1-5][0-9a-f]{3}-[89ab][0-9a-f]{3}-[0-9a-f]{12}(?=$|[^0-9a-f])/i.test(value);
                    const hasIpv4 = /(?:^|[^0-9])(?:[0-9]{1,3}[.]){3}[0-9]{1,3}(?=$|[^0-9])/.test(value);
                    const hasIpv6 = /(?:^|[^0-9a-f:])(?:(?:[0-9a-f]{1,4}:){3,7}[0-9a-f]{1,4}|[0-9a-f:]*::[0-9a-f:]*)(?=$|[^0-9a-f:])/i.test(value);
                    const hasWorker = /(?:worker|trabajador|host)\s*(?:[:#=-]\s*)?[A-Za-z0-9][A-Za-z0-9._-]+/i.test(value);
                    if (hasEmail || hasUuid || hasIpv4 || hasIpv6 || hasWorker) hide(walker.currentNode.parentElement);
                }

                window.__robotswarmRestorePrivate = () => {
                    privateNodes.forEach(entry => {
                        entry.element.style.visibility = entry.visibility;
                    });
                    previousRestore?.();
                    delete window.__robotswarmRestorePrivate;
                };
                return true;
            })()
            """
        )
        if extended is not True:
            self.restore_after_capture()
            raise DriverError("The section screenshot could not be redacted")

    def safe_screenshot(
        self,
        destination: Path,
        email: str,
        password: str,
    ) -> dict[str, Any]:
        result = self.screenshot(destination, email, password)
        if stat.S_IMODE(destination.stat().st_mode) != 0o600:
            raise DriverError("A section screenshot is not owner-only (0600)")
        if isinstance(result.get("page"), dict):
            result["page"] = {
                "origin": str(result["page"].get("origin", "")),
                "path": str(result["page"].get("path", "")),
                "title": "RobotSwarm",
            }
        result["redaction"] = {
            "emails": True,
            "personalCells": True,
            "uuids": True,
            "ipAddresses": True,
            "workerNames": True,
        }
        return result

    def _visible_selector_exists(self, selector: str) -> bool:
        return bool(
            self.cdp.evaluate(
                f"""
                [...document.querySelectorAll({json.dumps(selector)})]
                    .some(item => item.offsetParent !== null && !item.disabled)
                """
            )
        )

    def wait_dialog(
        self,
        title: str,
        timeout: float = 12,
        required_text: str | None = None,
    ) -> None:
        title_check = "true" if title == "*" else f"dialog.innerText.includes({json.dumps(title)})"
        content_check = (
            "true"
            if required_text is None
            else f"dialog.innerText.includes({json.dumps(required_text)})"
        )
        self.wait_js(
            f"""
            (() => {{
                const dialog = [...document.querySelectorAll('[role="dialog"]')]
                    .find(item => item.offsetParent !== null);
                return Boolean(dialog && {title_check} && {content_check});
            }})()
            """,
            timeout,
            f"{title} dialog",
        )

    def cancel_dialog(self, button: str = "Cancelar") -> None:
        self.click_button(button)
        self.wait_js(
            "![...document.querySelectorAll('[role=\"dialog\"]')].some(item => item.offsetParent !== null)",
            12,
            "closed dialog",
        )

    def open_row_dialog(
        self,
        selector: str,
        title: str,
        destination: Path,
        email: str,
        password: str,
        *,
        optional: bool = False,
        close_button: str = "Cancelar",
    ) -> dict[str, Any]:
        if not self._visible_selector_exists(selector):
            if optional:
                return {"available": False, "reason": "no-row"}
            raise DriverError(f"The row action for {title} was not available")
        self.click_selector(selector, title)
        self.wait_dialog(title)
        capture = self.safe_screenshot(destination, email, password)
        self.cancel_dialog(close_button)
        return {"available": True, "opened": True, "cancelled": True, "screenshot": capture}

    def open_button_dialog(
        self,
        button: str,
        title: str,
        destination: Path,
        email: str,
        password: str,
    ) -> dict[str, Any]:
        self.click_button(button)
        self.wait_dialog(title)
        capture = self.safe_screenshot(destination, email, password)
        self.cancel_dialog()
        return {"available": True, "opened": True, "cancelled": True, "screenshot": capture}

    def create_temp_group(self, name: str, timeout: float) -> dict[str, Any]:
        count = self.cdp.evaluate(
            f"""
            [...document.querySelectorAll('.MuiCardHeader-title')]
                .filter(item => item.textContent.trim() === {json.dumps(name)}).length
            """
        )
        if count != 0:
            raise DriverError("The temporary group marker unexpectedly already exists")

        self.click_button("Crear grupo")
        self.wait_dialog("Crear grupo")
        self.fill('[role="dialog"] input', name)
        # From this point the save may reach the backend even if CDP loses the reply.
        self._owned_temp_group_name = name
        self.click_button("Guardar")
        self.wait_js(
            f"""
            ![...document.querySelectorAll('[role="dialog"]')].some(item => item.offsetParent !== null)
                && [...document.querySelectorAll('.MuiCardHeader-title')]
                    .some(item => item.textContent.trim() === {json.dumps(name)})
            """,
            timeout,
            "persisted temporary group",
        )
        return {"created": True, "membershipChanged": False}

    def delete_temp_group(self, name: str, timeout: float) -> dict[str, Any]:
        groups = next(section for section in SECTIONS if section.key == "groups")
        self.visit_section(groups, timeout)
        point = self.cdp.evaluate(
            f"""
            (() => {{
                const cards = [...document.querySelectorAll('.MuiCard-root')].filter(card =>
                    card.querySelector('.MuiCardHeader-title')?.textContent.trim() === {json.dumps(name)}
                );
                if (cards.length === 0) return {{absent: true}};
                if (cards.length !== 1) return false;
                const button = cards[0].querySelector({json.dumps(f'[aria-label="Eliminar {name}"]')});
                if (!button || button.disabled) return false;
                button.scrollIntoView({{block: 'center'}});
                const box = button.getBoundingClientRect();
                return {{x: box.left + box.width / 2, y: box.top + box.height / 2}};
            }})()
            """
        )
        if isinstance(point, dict) and point.get("absent"):
            if getattr(self, "_owned_temp_group_name", None) == name:
                self._owned_temp_group_name = None
            return {"requested": True, "released": True, "alreadyAbsent": True}
        if not isinstance(point, dict) or "x" not in point:
            raise DriverError("The temporary group could not be selected safely for cleanup")

        self._mouse_click(float(point["x"]), float(point["y"]))
        self.wait_dialog("Eliminar grupo", required_text=name)
        self.click_button("Eliminar grupo")
        self.wait_js(
            f"""
            ![...document.querySelectorAll('.MuiCardHeader-title')]
                .some(item => item.textContent.trim() === {json.dumps(name)})
            """,
            timeout,
            "deleted temporary group",
        )
        if getattr(self, "_owned_temp_group_name", None) == name:
            self._owned_temp_group_name = None
        return {"requested": True, "released": True, "alreadyAbsent": False}


def screenshot_name(run_id: str, role: str, suffix: str) -> str:
    return f"{run_id}-{role.lower()}-{suffix}.png"


def run_user_acceptance(
    ui: SectionsUi,
    args: argparse.Namespace,
    credentials: dict[str, str],
    report: dict[str, Any],
    run_id: str,
) -> None:
    report["authorization"]["backendAdminEndpoints"] = ui.require_admin_endpoint_policy(
        "User"
    )
    report["navigation"] = {
        "observed": ui.require_menu("User"),
        "reducedForRole": True,
    }

    history = next(section for section in SECTIONS if section.key == "history")
    history_result = ui.visit_section(history, args.page_timeout)
    history_result["screenshot"] = ui.safe_screenshot(
        args.output_dir / screenshot_name(run_id, "User", "history"),
        credentials["email"],
        credentials["password"],
    )
    history_result["dialog"] = ui.open_row_dialog(
        '[data-testid="task-history-page"] [aria-label^="Ver detalle de "]',
        "*",
        args.output_dir / screenshot_name(run_id, "User", "history-dialog"),
        credentials["email"],
        credentials["password"],
        optional=True,
        close_button="Cerrar",
    )
    report["sections"][history.key] = history_result

    for section in (item for item in SECTIONS if item.admin_only):
        report["authorization"][section.key] = ui.require_admin_redirect(
            section,
            args.page_timeout,
        )


def run_admin_acceptance(
    ui: SectionsUi,
    args: argparse.Namespace,
    credentials: dict[str, str],
    report: dict[str, Any],
    run_id: str,
    temp_group: str,
) -> None:
    report["authorization"]["backendAdminEndpoints"] = ui.require_admin_endpoint_policy(
        "Admin"
    )
    report["navigation"] = {
        "observed": ui.require_menu("Admin"),
        "completeForRole": True,
    }

    for section in SECTIONS:
        result = ui.visit_section(section, args.page_timeout)
        result["screenshot"] = ui.safe_screenshot(
            args.output_dir / screenshot_name(run_id, "Admin", section.key),
            credentials["email"],
            credentials["password"],
        )

        if section.key == "history":
            result["dialog"] = ui.open_row_dialog(
                '[data-testid="task-history-page"] [aria-label^="Ver detalle de "]',
                "*",
                args.output_dir / screenshot_name(run_id, "Admin", "history-dialog"),
                credentials["email"],
                credentials["password"],
                optional=True,
                close_button="Cerrar",
            )
        elif section.key == "templates":
            result["dialog"] = ui.open_row_dialog(
                '[data-testid="task-templates-page"] [aria-label^="Editar la plantilla "]',
                "Editar plantilla de tarea",
                args.output_dir / screenshot_name(run_id, "Admin", "templates-dialog"),
                credentials["email"],
                credentials["password"],
            )
        elif section.key == "robots":
            result["dialog"] = ui.open_button_dialog(
                "Registrar robot",
                "Registrar robot",
                args.output_dir / screenshot_name(run_id, "Admin", "robots-dialog"),
                credentials["email"],
                credentials["password"],
            )
        elif section.key == "groups":
            result["dialog"] = ui.open_button_dialog(
                "Crear grupo",
                "Crear grupo",
                args.output_dir / screenshot_name(run_id, "Admin", "groups-dialog"),
                credentials["email"],
                credentials["password"],
            )
            result["temporaryGroup"] = ui.create_temp_group(temp_group, args.page_timeout)
            result["temporaryGroup"]["screenshot"] = ui.safe_screenshot(
                args.output_dir / screenshot_name(run_id, "Admin", "groups-temporary"),
                credentials["email"],
                credentials["password"],
            )
        elif section.key == "users":
            result["dialog"] = ui.open_row_dialog(
                '[data-testid="accounts-page"] tbody tr td:first-child',
                "Editar cuenta",
                args.output_dir / screenshot_name(run_id, "Admin", "users-dialog"),
                credentials["email"],
                credentials["password"],
            )
        report["sections"][section.key] = result


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Run visible User/Admin acceptance for the RobotSwarm management sections"
    )
    parser.add_argument(
        "--execute-production",
        action="store_true",
        help="confirmación obligatoria antes de abrir el sitio real",
    )
    parser.add_argument("--expected-role", choices=("User", "Admin"), required=True)
    parser.add_argument("--deployment-commit", required=True)
    parser.add_argument("--url", default=DEFAULT_URL)
    parser.add_argument("--credentials", type=Path, default=DEFAULT_CREDENTIALS)
    parser.add_argument("--chrome", type=Path, default=VISIBLE.DEFAULT_CHROME)
    parser.add_argument(
        "--profile-root",
        type=Path,
        required=True,
        help="directorio temporal de Windows para el perfil efímero",
    )
    parser.add_argument("--port", type=int, default=DEFAULT_PORT)
    parser.add_argument("--page-timeout", type=float, default=90)
    parser.add_argument("--output-dir", type=Path, default=DEFAULT_OUTPUT)
    parser.add_argument("--output-json", type=Path)
    return parser


def main() -> int:
    args = build_parser().parse_args()
    expected_origin = validate_options(args)
    credentials = read_section_credentials(args.credentials)
    if credentials["declaredRole"] and credentials["declaredRole"] != args.expected_role:
        raise DriverError("TEST_ROLE does not match --expected-role")
    if not VISIBLE.port_is_free(args.port):
        raise DriverError("The requested CDP port is occupied; no browser was launched")

    VISIBLE.validate_secure_directory(args.output_dir)
    run_id = dt.datetime.now(dt.timezone.utc).strftime("%Y%m%dT%H%M%SZ") + f"-{os.getpid()}"
    output_json = args.output_json or args.output_dir / f"sections-e2e-{run_id}.json"
    VISIBLE.validate_secure_directory(output_json.parent)

    temp_group = f"rs-sections-{secrets.token_hex(6)}"
    sanitizer = SectionsSanitizer(
        [credentials["email"], credentials["password"], temp_group]
    )
    profile = args.profile_root / f"robotswarm-sections-e2e-{run_id}-{args.expected_role.lower()}"
    chrome = VISIBLE.OwnedChrome(
        args.expected_role,
        args.port,
        profile,
        run_id,
        args.chrome,
        args.url,
    )
    stop_event = threading.Event()
    ui: SectionsUi | None = None
    report: dict[str, Any] = {
        "schemaVersion": 1,
        "runId": run_id,
        "startedAt": VISIBLE.utc_now(),
        "site": VISIBLE.clean_url(args.url),
        "deploymentCommit": args.deployment_commit,
        "productionExecution": True,
        "expectedRole": args.expected_role,
        "browser": {
            "visible": True,
            "headless": False,
            "gpuDisabled": False,
            "cdpPort": args.port,
            "profileOwned": True,
        },
        "sections": {},
        "authorization": {},
        "temporaryGroup": {"cleanupRequired": False},
        "cleanup": {},
        "success": False,
    }

    interrupted = False

    def note_signal(_number: int, _frame: Any) -> None:
        nonlocal interrupted
        interrupted = True
        stop_event.set()

    previous_int = signal.signal(signal.SIGINT, note_signal)
    previous_term = signal.signal(signal.SIGTERM, note_signal)
    failure: BaseException | None = None
    cleanup_passed = False
    try:
        print(
            f"Launching one owned, visible Chrome window for the {args.expected_role} sections run…",
            flush=True,
        )
        chrome.launch()
        ui = SectionsUi(chrome, expected_origin, stop_event)
        report["browser"].update(
            {
                "product": chrome.product,
                "profile": chrome.profile.name,
                "ownedPid": chrome.process.pid if chrome.process else None,
            }
        )
        ui.navigate(expected_origin + "/apps/GTS/realtime")
        ui.login(credentials["email"], credentials["password"])
        report["observedRole"] = ui.require_role(args.expected_role)

        if args.expected_role == "User":
            run_user_acceptance(ui, args, credentials, report, run_id)
        else:
            run_admin_acceptance(ui, args, credentials, report, run_id, temp_group)
        if interrupted:
            raise KeyboardInterrupt
        report["success"] = True
    except BaseException as exc:
        failure = exc
        report["error"] = {
            "type": type(exc).__name__,
            "message": sanitizer.text(exc),
        }
        if ui is not None:
            try:
                report["failureScreenshot"] = ui.safe_screenshot(
                    args.output_dir / screenshot_name(
                        run_id,
                        args.expected_role,
                        "failure",
                    ),
                    credentials["email"],
                    credentials["password"],
                )
            except Exception as capture_error:
                report["failureScreenshot"] = {
                    "captured": False,
                    "error": sanitizer.text(capture_error),
                }
    finally:
        # A signal stops the normal traversal, but cleanup must still be allowed to poll the page.
        stop_event.clear()
        cleanup_group_ok = True
        owned_temp_group = getattr(ui, "_owned_temp_group_name", None) if ui else None
        report["temporaryGroup"]["cleanupRequired"] = owned_temp_group == temp_group
        if owned_temp_group == temp_group:
            try:
                if not ui:
                    raise DriverError("The browser was unavailable for temporary-group cleanup")
                group_cleanup = ui.delete_temp_group(temp_group, args.page_timeout)
                report["cleanup"]["temporaryGroup"] = group_cleanup
                cleanup_group_ok = bool(group_cleanup.get("released"))
            except Exception as exc:
                cleanup_group_ok = False
                report["cleanup"]["temporaryGroup"] = {
                    "requested": True,
                    "released": False,
                    "error": sanitizer.text(exc),
                }
        elif owned_temp_group:
            cleanup_group_ok = False
            report["cleanup"]["temporaryGroup"] = {
                "requested": False,
                "released": False,
                "error": "Temporary-group ownership marker did not match",
            }
        else:
            report["cleanup"]["temporaryGroup"] = {
                "requested": False,
                "reason": "not-created",
            }

        try:
            browser_cleanup = chrome.close_owned()
        except Exception as exc:
            browser_cleanup = {
                "requested": True,
                "portFree": VISIBLE.port_is_free(args.port),
                "error": sanitizer.text(exc),
            }
        report["cleanup"]["browser"] = browser_cleanup
        cleanup_browser_ok = bool(browser_cleanup.get("portFree")) and bool(
            browser_cleanup.get("processExited")
        ) and bool(browser_cleanup.get("profileRemoved"))
        cleanup_passed = cleanup_group_ok and cleanup_browser_ok
        report["cleanup"]["passed"] = cleanup_passed
        if not cleanup_passed:
            report["success"] = False
            report.setdefault(
                "error",
                {
                    "type": "CleanupError",
                    "message": "Owned browser or temporary-group cleanup did not finish",
                },
            )
        report["completedAt"] = VISIBLE.utc_now()
        report = sanitizer.value(report)
        write_report_secure(output_json, report)
        signal.signal(signal.SIGINT, previous_int)
        signal.signal(signal.SIGTERM, previous_term)

    print(f"Sanitized report: {output_json}", flush=True)
    exit_code = VISIBLE.result_exit_code(failure, cleanup_passed)
    if failure is not None:
        print(f"Sections acceptance failed: {sanitizer.text(failure)}", file=sys.stderr, flush=True)
    elif exit_code != 0:
        print("Sections acceptance passed, but cleanup did not finish.", file=sys.stderr, flush=True)
    return exit_code


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except DriverError as exc:
        print(f"Sections acceptance could not start: {exc}", file=sys.stderr)
        raise SystemExit(2)
