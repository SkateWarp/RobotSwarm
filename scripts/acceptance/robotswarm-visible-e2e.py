#!/usr/bin/env python3
"""Prueba visual de dos usuarios de RobotSwarm mediante Chrome DevTools Protocol.

El script lanza dos ventanas normales de Chrome. No usa modo headless ni desactiva
la GPU. Las credenciales nunca se imprimen y solamente se escriben en los campos
reales del formulario de inicio de sesión.
"""

from __future__ import annotations

import argparse
import base64
import concurrent.futures
import contextlib
import datetime as dt
import hashlib
import hmac
import io
import json
import os
import re
import secrets
import shutil
import signal
import socket
import stat
import struct
import subprocess
import sys
import tempfile
import threading
import time
import urllib.error
import urllib.parse
import urllib.request
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Callable


PYDEPS = Path("/tmp/robotswarm-browser-pydeps")
if str(PYDEPS) not in sys.path:
    sys.path.insert(0, str(PYDEPS))

try:
    import websocket
except ImportError as exc:  # pragma: no cover - checked before a production run
    raise SystemExit(f"websocket-client is not available in {PYDEPS}") from exc

try:
    from PIL import Image
except ImportError as exc:  # pragma: no cover - checked before a production run
    raise SystemExit("Pillow is required for the visible video comparison") from exc


DEFAULT_URL = "https://rs.zerav.la/apps/GTS/realtime"
PRODUCTION_ORIGIN = "https://rs.zerav.la"
DEFAULT_CHROME = Path("/mnt/c/Program Files/Google/Chrome/Application/chrome.exe")
DEFAULT_CREDENTIALS = Path("/tmp/robotswarm-e2e-credentials.env")
DEFAULT_BINDING_KEY = Path("/tmp/robotswarm-e2e-binding.key")
DEFAULT_API_HOST = "robot.zerav.la"
POWERSHELL = Path("/mnt/c/Windows/System32/WindowsPowerShell/v1.0/powershell.exe")
DEFAULT_PORTS = {"A": 9332, "B": 9333}
SCENE_PIXEL_THRESHOLD = 32
MIN_SCENE_DIFFERENCE_RATIO = 0.00075
FAST_FIGURE_RECORD_SETTLE_SECONDS = 5.0
MAX_VIEWER_LEASE_SECONDS = 30 * 60
VIEWER_VISIBILITY_RETRY_SECONDS = 1.0
VIEWER_LEASE_COUNTDOWN = re.compile(r"Vence en (0|[1-9]\d*):([0-5]\d)")

LOGIN_EMAIL = 'input[name="email"]'
LOGIN_PASSWORD = 'input[name="password"]'
LOGIN_SUBMIT = 'button[type="submit"]'
WORKSPACE_HEADING = "Sesión de simulación"
WORKSPACE_SELECTOR = '[data-testid="session-panel"]'
CREATE_BUTTON = "Crear simulación"
OPEN_VIEWER_BUTTON = "Abrir visor"
CLOSE_VIEWER_BUTTON = "Cerrar visor"
STOP_BUTTON = "Detener sesión"
START_TASK_BUTTON = "Iniciar tarea"
TASK_HEADING = "Tarea del enjambre"

UI_TASK_STATES = {
    "En cola": "Queued",
    "En ejecución": "Running",
    "En pausa": "Paused",
    "Cancelando": "Cancelling",
    "Completada": "Completed",
    "Cancelada": "Cancelled",
    "Fallida": "Failed",
}

UI_OUTCOMES = {
    "Pendiente de verificación": "Pending",
    "Correcto": "Succeeded",
    "Fallido": "Failed",
    "Cancelado": "Cancelled",
}


def pixel_difference_ratio(
    left_pixels: Any,
    right_pixels: Any,
    threshold: int = SCENE_PIXEL_THRESHOLD,
) -> float:
    """Measure structural changes while ignoring small encoder fluctuations."""
    if len(left_pixels) != len(right_pixels) or len(left_pixels) == 0:
        raise ValueError("The image samples must have the same non-zero size")

    changed = 0
    for left, right in zip(left_pixels, right_pixels):
        if max(abs(int(a) - int(b)) for a, b in zip(left, right)) > threshold:
            changed += 1
    return changed / len(left_pixels)

SESSION_SLOT_STATES = {"Queued", "Provisioning", "Ready", "Active", "Paused", "Stopping"}


class DriverError(RuntimeError):
    """Expected acceptance-driver failure with a message safe for the report."""


def utc_now() -> str:
    return dt.datetime.now(dt.timezone.utc).isoformat(timespec="seconds").replace("+00:00", "Z")


def parse_viewer_lease_countdown(value: Any) -> int:
    """Return the remaining lease seconds exposed by the browser viewer."""
    if not isinstance(value, str):
        raise DriverError("The viewer lease countdown was not published")

    match = VIEWER_LEASE_COUNTDOWN.fullmatch(value.strip())
    if match is None:
        raise DriverError("The viewer lease countdown did not use the Vence en M:SS format")

    remaining = int(match.group(1)) * 60 + int(match.group(2))
    if remaining <= 0:
        raise DriverError("The interactive viewer published an expired lease countdown")
    if remaining > MAX_VIEWER_LEASE_SECONDS:
        raise DriverError("The viewer lease countdown exceeded the 30-minute backend limit")
    return remaining


def parse_backend_time(value: Any, field: str) -> dt.datetime | None:
    if value is None:
        return None
    if not isinstance(value, str):
        raise DriverError(f"The backend task {field} timestamp is invalid")
    match = re.fullmatch(
        r"(\d{4}-\d{2}-\d{2}T\d{2}:\d{2}:\d{2})(?:\.(\d{1,7}))?(Z|[+-]\d{2}:\d{2})?",
        value,
    )
    if match is None:
        raise DriverError(f"The backend task {field} timestamp is invalid")
    base, fraction, zone = match.groups()
    microseconds = (fraction or "").ljust(7, "0")[:6]
    # Npgsql stores these UTC values as timestamp-without-time-zone, so
    # System.Text.Json legitimately omits the suffix on this response.
    normalized_zone = "+00:00" if zone in (None, "Z") else zone
    try:
        parsed = dt.datetime.fromisoformat(f"{base}.{microseconds}{normalized_zone}")
    except ValueError as exc:
        raise DriverError(f"The backend task {field} timestamp is invalid") from exc
    return parsed.astimezone(dt.timezone.utc)


def clean_url(value: str) -> str:
    parsed = urllib.parse.urlsplit(value)
    return urllib.parse.urlunsplit((parsed.scheme, parsed.netloc, parsed.path, "", ""))


class Sanitizer:
    def __init__(self, secrets: list[str]) -> None:
        self._secrets = sorted((item for item in secrets if item), key=len, reverse=True)

    def text(self, value: Any) -> str:
        result = str(value)
        for secret in self._secrets:
            result = result.replace(secret, "[REDACTED]")
        result = re.sub(r"(?i)Bearer\s+[A-Za-z0-9._~+/=-]+", "Bearer [REDACTED]", result)
        result = re.sub(
            r"\beyJ[A-Za-z0-9_-]{8,}\.[A-Za-z0-9_-]{8,}\.[A-Za-z0-9_-]{8,}\b",
            "[JWT REDACTED]",
            result,
        )
        result = re.sub(r"[A-Za-z0-9._%+-]+@[A-Za-z0-9.-]+\.[A-Za-z]{2,}", "[EMAIL REDACTED]", result)
        result = re.sub(r"([?&](?:token|key|secret|auth|jwt)=[^&#\s]+)", "?[REDACTED]", result, flags=re.I)
        return result[:1200]

    def value(self, value: Any) -> Any:
        if isinstance(value, dict):
            return {str(key): self.value(item) for key, item in value.items()}
        if isinstance(value, (list, tuple)):
            return [self.value(item) for item in value]
        if isinstance(value, str):
            return self.text(value)
        return value


def read_credentials(path: Path) -> dict[str, dict[str, str]]:
    """Read the small env file without sourcing it or exposing it in a child process."""
    try:
        info = path.lstat()
    except FileNotFoundError as exc:
        raise DriverError(f"Credential file is missing: {path}") from exc
    if stat.S_ISLNK(info.st_mode) or not stat.S_ISREG(info.st_mode):
        raise DriverError("Credential path must be a regular file, not a link")
    if stat.S_IMODE(info.st_mode) != 0o600:
        raise DriverError("Credential file must have mode 0600")
    if info.st_uid != os.getuid():
        raise DriverError("Credential file must be owned by the current user")
    if info.st_size > 4096:
        raise DriverError("Credential file is unexpectedly large")

    values: dict[str, str] = {}
    for line in path.read_text(encoding="utf-8").splitlines():
        line = line.strip()
        if not line or line.startswith("#"):
            continue
        key, separator, raw_value = line.partition("=")
        if not separator:
            raise DriverError("Credential file contains an invalid line")
        key = key.strip()
        raw_value = raw_value.strip()
        if len(raw_value) >= 2 and raw_value[0] == raw_value[-1] and raw_value[0] in "'\"":
            raw_value = raw_value[1:-1]
        values[key] = raw_value

    required = {
        "TEST_A_ID",
        "TEST_A_EMAIL",
        "TEST_A_PASSWORD",
        "TEST_B_ID",
        "TEST_B_EMAIL",
        "TEST_B_PASSWORD",
    }
    if not required.issubset(values) or any(not values[key] for key in required):
        raise DriverError("Credential file does not contain both complete test accounts")
    try:
        marker_a = int(values["TEST_A_ID"])
        marker_b = int(values["TEST_B_ID"])
    except ValueError as exc:
        raise DriverError("Credential file contains an invalid account marker") from exc
    if marker_a <= 0 or marker_b <= 0 or marker_a == marker_b:
        raise DriverError("Credential file does not contain two distinct account markers")
    return {
        "A": {
            "id": str(marker_a),
            "email": values["TEST_A_EMAIL"],
            "password": values["TEST_A_PASSWORD"],
        },
        "B": {
            "id": str(marker_b),
            "email": values["TEST_B_EMAIL"],
            "password": values["TEST_B_PASSWORD"],
        },
    }


def load_binding_key(path: Path) -> bytes:
    flags = os.O_RDONLY
    if hasattr(os, "O_NOFOLLOW"):
        flags |= os.O_NOFOLLOW
    try:
        descriptor = os.open(path, flags)
    except OSError as exc:
        raise DriverError("The account-binding key is unavailable") from exc
    try:
        details = os.fstat(descriptor)
        if not stat.S_ISREG(details.st_mode):
            raise DriverError("The account-binding key must be a regular file")
        if details.st_uid != os.getuid() or stat.S_IMODE(details.st_mode) != 0o600:
            raise DriverError("The account-binding key must be owner-only (0600)")
        value = os.read(descriptor, 33)
    finally:
        os.close(descriptor)
    if len(value) != 32:
        raise DriverError("The account-binding key has an invalid size")
    return value


def account_fingerprint(credential: dict[str, str], binding_key: bytes) -> str:
    """Match the API run to this credential without writing the credential itself."""
    message = b"\0".join(
        (
            b"robotswarm-e2e-account-binding-v2",
            credential["id"].encode("ascii"),
            credential["email"].strip().lower().encode("utf-8"),
            credential["password"].encode("utf-8"),
        )
    )
    return hmac.new(
        binding_key,
        message,
        hashlib.sha256,
    ).hexdigest()


def load_api_acceptance(
    path: Path,
    expected_commit: str,
    expected_fingerprints: dict[str, str],
    maximum_age_minutes: float = 30,
) -> dict[str, Any]:
    try:
        details = path.lstat()
    except OSError as exc:
        raise DriverError("The API acceptance report is unavailable") from exc
    if stat.S_ISLNK(details.st_mode) or not stat.S_ISREG(details.st_mode):
        raise DriverError("The API acceptance report must be a regular file")
    if details.st_uid != os.getuid() or stat.S_IMODE(details.st_mode) != 0o600:
        raise DriverError("The API acceptance report must be owner-only (0600)")
    if details.st_size <= 0 or details.st_size > 1024 * 1024:
        raise DriverError("The API acceptance report has an invalid size")
    try:
        report = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, UnicodeError, json.JSONDecodeError) as exc:
        raise DriverError("The API acceptance report is not valid JSON") from exc
    if not isinstance(report, dict):
        raise DriverError("The API acceptance report root must be an object")
    if report.get("schema_version") != 1 or report.get("result") != "passed":
        raise DriverError("The API acceptance report did not pass")
    configuration = report.get("configuration")
    if not isinstance(configuration, dict):
        raise DriverError("The API acceptance configuration is invalid")
    if configuration.get("production_execution") is not True or configuration.get("users") != 2:
        raise DriverError("The API report does not describe a real two-user production run")
    if configuration.get("api_host") != DEFAULT_API_HOST:
        raise DriverError("The API report belongs to another backend origin")
    if configuration.get("deployment_commit") != expected_commit:
        raise DriverError("The API and visual runs target different backend revisions")
    reported_fingerprints = configuration.get("account_fingerprints")
    if not isinstance(reported_fingerprints, dict):
        raise DriverError("The API report has no account binding")
    if not hmac.compare_digest(
        str(reported_fingerprints.get("user_a", "")), expected_fingerprints["A"]
    ) or not hmac.compare_digest(
        str(reported_fingerprints.get("user_b", "")), expected_fingerprints["B"]
    ):
        raise DriverError("The API and visual runs use different test accounts")
    robot_counts = configuration.get("robot_counts")
    if not isinstance(robot_counts, dict):
        raise DriverError("The API acceptance report has no robot-count object")
    if not all(isinstance(robot_counts.get(name), int) for name in ("user_a", "user_b")):
        raise DriverError("The API acceptance report has no verified robot counts")
    cleanup = report.get("cleanup")
    if not isinstance(cleanup, dict):
        raise DriverError("The API acceptance cleanup object is invalid")
    if cleanup.get("attempted") is not True or cleanup.get("complete") is not True:
        raise DriverError("The API acceptance cleanup was incomplete")
    required_isolation = (
        "api_cross_session",
        "api_cross_stop",
        "api_cross_tasks",
        "api_unauthenticated",
        "hls_cross_user_a",
        "hls_cross_user_b",
        "hls_unauthenticated",
        "stop_completed_before_lease_expiry",
        "stop_kept_user_b_active",
        "stop_kept_user_b_viewer",
        "stopped_user_a_hls_rejected",
    )
    isolation = report.get("isolation")
    if not isinstance(isolation, dict):
        raise DriverError("The API acceptance isolation object is invalid")
    missing = [name for name in required_isolation if isolation.get(name) is not True]
    if missing:
        raise DriverError("The API acceptance report is missing a required isolation gate")
    checks = report.get("checks")
    if (
        not isinstance(checks, list)
        or not checks
        or any(not isinstance(item, dict) or item.get("status") != "passed" for item in checks)
    ):
        raise DriverError("The API acceptance report contains a failed check")
    finished_text = report.get("finished_at")
    if not isinstance(finished_text, str):
        raise DriverError("The API acceptance report has no completion time")
    try:
        finished = dt.datetime.fromisoformat(finished_text.replace("Z", "+00:00"))
    except ValueError as exc:
        raise DriverError("The API acceptance completion time is invalid") from exc
    if finished.tzinfo is None:
        raise DriverError("The API acceptance completion time has no timezone")
    age = dt.datetime.now(dt.timezone.utc) - finished.astimezone(dt.timezone.utc)
    if age.total_seconds() < -60 or age > dt.timedelta(minutes=maximum_age_minutes):
        raise DriverError("The API acceptance report is too old for this visual run")
    return {
        "result": "passed",
        "finishedAt": finished_text,
        "cleanupComplete": True,
        "requiredIsolationGates": len(required_isolation),
        "robotCounts": {"A": robot_counts["user_a"], "B": robot_counts["user_b"]},
        "apiHost": DEFAULT_API_HOST,
        "deploymentCommit": expected_commit,
        "accountsBound": True,
    }


def port_is_free(port: int) -> bool:
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as probe:
        probe.settimeout(0.25)
        return probe.connect_ex(("127.0.0.1", port)) != 0


def local_json(url: str, timeout: float = 2.0) -> Any:
    opener = urllib.request.build_opener(urllib.request.ProxyHandler({}))
    request = urllib.request.Request(url, headers={"Accept": "application/json"})
    with opener.open(request, timeout=timeout) as response:
        return json.loads(response.read().decode("utf-8"))


def windows_path(path: Path) -> str:
    result = subprocess.run(
        ["wslpath", "-w", str(path)],
        check=True,
        capture_output=True,
        text=True,
        timeout=10,
    )
    return result.stdout.strip()


def run_interruptible_process(
    command: list[str],
    *,
    timeout: float,
    stop_event: threading.Event,
) -> subprocess.CompletedProcess[str]:
    """Run a short helper while keeping Ctrl-C responsive."""
    process = subprocess.Popen(
        command,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        text=True,
        encoding="utf-8",
        # Windows PowerShell can still write localized text in its OEM code page.
        # The helpers return ASCII JSON, so replacing unrelated bytes keeps that
        # contract readable without making the capture depend on the host locale.
        errors="replace",
        start_new_session=True,
    )

    def stop_process() -> None:
        if process.poll() is not None:
            return
        with contextlib.suppress(ProcessLookupError):
            os.killpg(process.pid, signal.SIGTERM)
        with contextlib.suppress(subprocess.TimeoutExpired):
            process.wait(timeout=2)
        if process.poll() is None:
            with contextlib.suppress(ProcessLookupError):
                os.killpg(process.pid, signal.SIGKILL)

    deadline = time.monotonic() + timeout
    while process.poll() is None:
        if stop_event.is_set():
            stop_process()
            process.communicate()
            raise KeyboardInterrupt
        if time.monotonic() >= deadline:
            stop_process()
            process.communicate()
            raise DriverError("A Windows capture helper timed out")
        time.sleep(0.2)
    stdout, stderr = process.communicate()
    return subprocess.CompletedProcess(command, process.returncode, stdout, stderr)


class CdpClient:
    def __init__(self, websocket_url: str, port: int, timeout: float = 15.0) -> None:
        self._port = port
        self._sequence = 0
        self._lock = threading.Lock()
        self._socket = websocket.create_connection(
            websocket_url,
            timeout=timeout,
            origin=f"http://127.0.0.1:{port}",
            http_no_proxy=["127.0.0.1", "localhost"],
            enable_multithread=True,
        )

    def close(self) -> None:
        with contextlib.suppress(Exception):
            self._socket.close()

    def call(self, method: str, params: dict[str, Any] | None = None, timeout: float = 30.0) -> dict[str, Any]:
        with self._lock:
            self._sequence += 1
            message_id = self._sequence
            payload = {"id": message_id, "method": method, "params": params or {}}
            self._socket.settimeout(timeout)
            self._socket.send(json.dumps(payload, separators=(",", ":")))
            while True:
                reply = json.loads(self._socket.recv())
                if reply.get("id") != message_id:
                    continue
                if "error" in reply:
                    error = reply["error"]
                    raise DriverError(f"CDP {method} failed ({error.get('code', 'unknown')})")
                return reply.get("result", {})

    def evaluate(
        self,
        expression: str,
        *,
        await_promise: bool = False,
        context_id: int | None = None,
        timeout: float = 30.0,
    ) -> Any:
        parameters: dict[str, Any] = {
            "expression": expression,
            "awaitPromise": await_promise,
            "returnByValue": True,
            "userGesture": True,
        }
        if context_id is not None:
            parameters["contextId"] = context_id
        result = self.call(
            "Runtime.evaluate",
            parameters,
            timeout=timeout,
        )
        remote = result.get("result", {})
        if remote.get("subtype") == "error" or result.get("exceptionDetails"):
            raise DriverError("JavaScript evaluation failed")
        return remote.get("value")

    def create_isolated_world(self) -> int:
        """Create a DOM-visible world whose globals the application cannot edit."""
        frame_tree = self.call("Page.getFrameTree")
        frame_id = (
            (frame_tree.get("frameTree") or {}).get("frame") or {}
        ).get("id")
        if not isinstance(frame_id, str) or not frame_id:
            raise DriverError("Could not identify the main frame for trusted input evidence")
        result = self.call(
            "Page.createIsolatedWorld",
            {
                "frameId": frame_id,
                "worldName": "robotswarm-acceptance-input",
                "grantUniveralAccess": False,
            },
        )
        context_id = result.get("executionContextId")
        if not isinstance(context_id, int) or context_id <= 0:
            raise DriverError("Chrome did not create the isolated input evidence world")
        return context_id


@dataclass
class OwnedChrome:
    label: str
    port: int
    profile: Path
    run_id: str
    chrome_path: Path
    site_url: str
    target_nonce: str = field(default_factory=lambda: secrets.token_urlsafe(24))
    process: subprocess.Popen[bytes] | None = None
    browser_id: str | None = None
    browser_ws: str | None = None
    page_id: str | None = None
    page: CdpClient | None = None
    product: str | None = None
    closed: bool = False

    @property
    def marker(self) -> Path:
        return self.profile / ".robotswarm-visible-owner.json"

    @property
    def launch_target(self) -> str:
        return (
            "data:text/html,"
            f"<title>RobotSwarm</title>robotswarm-owned-{self.target_nonce}"
        )

    def owns_launch_target(self, url: Any) -> bool:
        value = str(url or "")
        return (
            value.startswith("data:text/html,")
            and f"robotswarm-owned-{self.target_nonce}" in value
        )

    def is_local_cdp_endpoint(
        self,
        websocket_url: Any,
        endpoint_type: str,
        endpoint_id: str | None = None,
    ) -> bool:
        try:
            endpoint = urllib.parse.urlsplit(str(websocket_url or ""))
            expected_path = (
                f"/devtools/page/{endpoint_id}"
                if endpoint_type == "page" and endpoint_id
                else None
            )
            browser_prefix = "/devtools/browser/"
            browser_id = (
                endpoint.path[len(browser_prefix) :]
                if endpoint.path.startswith(browser_prefix)
                else ""
            )
            path_matches = (
                endpoint.path == expected_path
                if expected_path
                else endpoint_type == "browser"
                and bool(browser_id)
                and "/" not in browser_id
            )
            return (
                endpoint.scheme == "ws"
                and endpoint.hostname in {"127.0.0.1", "localhost"}
                and endpoint.port == self.port
                and endpoint.username is None
                and endpoint.password is None
                and not endpoint.query
                and not endpoint.fragment
                and path_matches
            )
        except ValueError:
            return False

    def launch(self) -> None:
        if not port_is_free(self.port):
            raise DriverError(f"CDP port {self.port} is already in use; refusing to attach")
        if self.profile.exists():
            raise DriverError(f"Owned Chrome profile already exists: {self.profile.name}")
        self.profile.mkdir(parents=True, mode=0o700)
        marker = {"runId": self.run_id, "label": self.label, "port": self.port, "pid": None}
        self.marker.write_text(json.dumps(marker), encoding="utf-8")

        x_position = 0 if self.label == "A" else 960
        arguments = [
            str(self.chrome_path),
            f"--remote-debugging-port={self.port}",
            "--remote-debugging-address=127.0.0.1",
            "--remote-allow-origins=*",
            f"--user-data-dir={windows_path(self.profile)}",
            "--no-first-run",
            "--no-default-browser-check",
            "--autoplay-policy=no-user-gesture-required",
            "--new-window",
            "--window-size=960,1000",
            f"--window-position={x_position},0",
            self.launch_target,
        ]
        forbidden = ("--headless", "--disable-gpu")
        if any(any(argument.startswith(item) for item in forbidden) for argument in arguments):
            raise DriverError("Visible Chrome arguments unexpectedly disable rendering")

        self.process = subprocess.Popen(
            arguments,
            stdin=subprocess.DEVNULL,
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
            start_new_session=True,
        )
        marker["pid"] = self.process.pid
        self.marker.write_text(json.dumps(marker), encoding="utf-8")
        self._connect()

    def _connect(self) -> None:
        deadline = time.monotonic() + 35
        version: dict[str, Any] | None = None
        page_target: dict[str, Any] | None = None
        while time.monotonic() < deadline:
            try:
                candidate_version = local_json(
                    f"http://127.0.0.1:{self.port}/json/version"
                )
                browser_ws = str(
                    candidate_version.get("webSocketDebuggerUrl") or ""
                )
                targets = local_json(f"http://127.0.0.1:{self.port}/json/list")
                candidate_page = next(
                    (
                        item
                        for item in targets
                        if item.get("type") == "page"
                        and self.owns_launch_target(item.get("url"))
                        and item.get("id")
                        and self.is_local_cdp_endpoint(
                            item.get("webSocketDebuggerUrl"),
                            "page",
                            str(item["id"]),
                        )
                    ),
                    None,
                )
                if (
                    self.is_local_cdp_endpoint(browser_ws, "browser")
                    and candidate_page is not None
                ):
                    version = candidate_version
                    page_target = candidate_page
                    break
            except (OSError, urllib.error.URLError, json.JSONDecodeError):
                pass
            # Windows may hand the launch to a new browser process and let the
            # WSL interop process exit first. The unique launch target binds the
            # CDP endpoint to this invocation even after that handoff.
            time.sleep(0.25)
        if (
            not version
            or not version.get("webSocketDebuggerUrl")
            or page_target is None
        ):
            raise DriverError(
                f"Chrome {self.label} did not expose its owned CDP target on {self.port}"
            )

        self.browser_ws = str(version["webSocketDebuggerUrl"])
        self.browser_id = self.browser_ws.rstrip("/").rsplit("/", 1)[-1]
        self.product = str(version.get("Browser", "Chrome"))
        self.page_id = str(page_target.get("id") or "")
        if not self.page_id:
            raise DriverError(f"Chrome {self.label} has no owned page target")
        self.page = CdpClient(str(page_target["webSocketDebuggerUrl"]), self.port)
        self.page.call("Page.enable")
        self.page.call("Runtime.enable")

    def close_owned(self) -> dict[str, Any]:
        outcome: dict[str, Any] = {"requested": True, "method": None, "portFree": False}
        if self.closed:
            outcome.update({"method": "already-closed", "portFree": port_is_free(self.port)})
            return outcome

        marker_ok = False
        try:
            marker = json.loads(self.marker.read_text(encoding="utf-8"))
            marker_ok = (
                marker.get("runId") == self.run_id
                and marker.get("label") == self.label
                and marker.get("port") == self.port
                and self.process is not None
                and marker.get("pid") == self.process.pid
            )
        except (OSError, ValueError):
            marker_ok = False
        if not marker_ok:
            outcome.update({"method": "refused-owner-marker", "error": "Ownership marker did not match"})
            return outcome

        endpoint_matches = False
        if self.browser_id and self.page_id:
            try:
                current = local_json(f"http://127.0.0.1:{self.port}/json/version")
                current_id = str(current.get("webSocketDebuggerUrl", "")).rstrip("/").rsplit("/", 1)[-1]
                targets = local_json(f"http://127.0.0.1:{self.port}/json/list")
                target_still_owned = any(
                    item.get("type") == "page" and item.get("id") == self.page_id
                    for item in targets
                )
                endpoint_matches = (
                    current_id == self.browser_id and target_still_owned
                )
            except Exception:
                endpoint_matches = False

        if endpoint_matches and self.browser_ws:
            try:
                browser = CdpClient(self.browser_ws, self.port)
                try:
                    browser.call("Browser.close", timeout=10)
                finally:
                    browser.close()
                outcome["method"] = "Browser.close"
            except Exception:
                outcome["method"] = "Browser.close-failed"
            time.sleep(0.5)
            if not port_is_free(self.port) and self.process and self.process.poll() is None:
                self.process.terminate()
                outcome["method"] += "+owned-pid-terminate"
        elif self.process and self.process.poll() is None:
            # This exact Popen object was created above. No process-name matching is used.
            self.process.terminate()
            outcome["method"] = "owned-pid-terminate"
        else:
            outcome["method"] = "process-already-exited"

        if self.page:
            self.page.close()
        if self.process:
            with contextlib.suppress(subprocess.TimeoutExpired):
                self.process.wait(timeout=12)

        deadline = time.monotonic() + 15
        while time.monotonic() < deadline and not port_is_free(self.port):
            time.sleep(0.25)
        outcome["portFree"] = port_is_free(self.port)
        outcome["processExited"] = self.process is None or self.process.poll() is not None
        self.closed = bool(outcome["portFree"] and outcome["processExited"])
        if self.closed and self.profile.exists():
            shutil.rmtree(self.profile)
            outcome["profileRemoved"] = True
        else:
            outcome["profileRemoved"] = False
        return outcome


class RobotSwarmUi:
    def __init__(
        self,
        chrome: OwnedChrome,
        expected_origin: str,
        stop_event: threading.Event,
    ) -> None:
        if not chrome.page:
            raise DriverError("CDP page is not connected")
        self.chrome = chrome
        self.cdp = chrome.page
        self.expected_origin = expected_origin
        self.stop_event = stop_event
        self.create_requested = False
        self.created_session = False
        self.started_task_session_id: str | None = None
        self.started_task_id: str | None = None
        self._click_evidence: list[dict[str, Any]] = []

    def raise_if_interrupted(self) -> None:
        if self.stop_event.is_set():
            raise KeyboardInterrupt

    def navigate(self, url: str) -> None:
        self.cdp.call("Page.navigate", {"url": url})
        self.wait_js("document.readyState === 'complete'", 60, "page load")

    def wait_js(self, condition: str, timeout: float, description: str) -> Any:
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            self.raise_if_interrupted()
            value = self.cdp.evaluate(f"Boolean({condition})")
            if value:
                return value
            time.sleep(0.35)
        raise DriverError(f"Timed out waiting for {description}")

    def current_page(self) -> dict[str, str]:
        result = self.cdp.evaluate(
            "({url: location.href, origin: location.origin, path: location.pathname, title: document.title})"
        )
        if not isinstance(result, dict):
            raise DriverError("Could not inspect the current page")
        return {key: str(result.get(key, "")) for key in ("url", "origin", "path", "title")}

    def browser_media_capabilities(self) -> dict[str, bool]:
        """Mirror the hls.js platform gate without creating a viewer lease."""
        result = self.cdp.evaluate("""
            (() => {
                const mediaSource = window.MediaSource || window.WebKitMediaSource;
                const sourceBuffer = window.SourceBuffer || window.WebKitSourceBuffer;
                const sourceBufferReady = !sourceBuffer || Boolean(
                    sourceBuffer.prototype &&
                    typeof sourceBuffer.prototype.appendBuffer === 'function' &&
                    typeof sourceBuffer.prototype.remove === 'function'
                );
                const supports = mime => Boolean(
                    mediaSource &&
                    typeof mediaSource.isTypeSupported === 'function' &&
                    mediaSource.isTypeSupported(mime)
                );
                const baselineAvc = supports(
                    'video/mp4;codecs="avc1.42E01E,mp4a.40.2"'
                );
                const baselineAudio = supports('audio/mp4;codecs="mp4a.40.2"');
                return {
                    mediaSource: Boolean(mediaSource),
                    sourceBufferReady,
                    baselineAvc,
                    baselineAudio,
                    hlsJsEquivalent: Boolean(
                        mediaSource && sourceBufferReady && (baselineAvc || baselineAudio)
                    ),
                };
            })()
        """)
        if not isinstance(result, dict) or not all(
            isinstance(result.get(key), bool)
            for key in (
                "mediaSource",
                "sourceBufferReady",
                "baselineAvc",
                "baselineAudio",
                "hlsJsEquivalent",
            )
        ):
            raise DriverError("Could not inspect the browser HLS capabilities")
        return {key: bool(value) for key, value in result.items()}

    def require_hls_media_capabilities(self) -> dict[str, bool]:
        capabilities = self.browser_media_capabilities()
        if not capabilities["hlsJsEquivalent"] or not capabilities["baselineAvc"]:
            raise DriverError("Visible Chrome does not currently expose H.264 MediaSource support")
        return capabilities

    def assert_origin(self) -> None:
        if self.current_page()["origin"] != self.expected_origin:
            raise DriverError("Login was redirected outside the expected RobotSwarm origin")

    def fill(self, selector: str, value: str) -> None:
        expression = f"""
            (() => {{
                const element = document.querySelector({json.dumps(selector)});
                if (!element) return false;
                const setter = Object.getOwnPropertyDescriptor(HTMLInputElement.prototype, 'value').set;
                setter.call(element, {json.dumps(value)});
                element.dispatchEvent(new Event('input', {{bubbles: true}}));
                element.dispatchEvent(new Event('change', {{bubbles: true}}));
                return true;
            }})()
        """
        if self.cdp.evaluate(expression) is not True:
            raise DriverError("A required form input was not found")

    def click_selector(
        self,
        selector: str,
        description: str,
        *,
        require_trusted: bool = False,
    ) -> dict[str, Any]:
        click_context = self.cdp.create_isolated_world()
        point = self.cdp.evaluate(f"""
            (() => {{
                const element = document.querySelector({json.dumps(selector)});
                if (!element || element.disabled) return false;
                element.scrollIntoView({{block: 'center'}});
                const box = element.getBoundingClientRect();
                const x = box.left + box.width / 2;
                const y = box.top + box.height / 2;
                const hit = document.elementFromPoint(x, y);
                if (!hit || !element.contains(hit)) return false;
                globalThis.__robotswarmAcceptanceClick = {{received: false, trusted: false}};
                element.addEventListener('click', event => {{
                    globalThis.__robotswarmAcceptanceClick = {{
                        received: true,
                        trusted: Boolean(event.isTrusted),
                    }};
                }}, {{capture: true, once: true}});
                return {{x, y}};
            }})()
        """, context_id=click_context)
        if not isinstance(point, dict):
            raise DriverError(f"Could not click {description}")
        return self._activate_prepared_click(
            float(point["x"]),
            float(point["y"]),
            f"""
                (() => {{
                    const element = document.querySelector({json.dumps(selector)});
                    if (!element || element.disabled) return false;
                    element.click();
                    return true;
                }})()
            """,
            description,
            require_trusted=require_trusted,
            click_context=click_context,
        )

    def has_button(self, text: str) -> bool:
        return bool(self.cdp.evaluate(self._text_element_expression("button", text, click=False)))

    def click_button(self, text: str, *, require_trusted: bool = False) -> dict[str, Any]:
        click_context = self.cdp.create_isolated_world()
        point = self.cdp.evaluate(
            self._text_element_point_expression("button", text, arm_click_probe=True),
            context_id=click_context,
        )
        if not isinstance(point, dict):
            raise DriverError(f"Button is missing, disabled, or covered: {text}")
        return self._activate_prepared_click(
            float(point["x"]),
            float(point["y"]),
            self._text_element_expression("button", text, click=True),
            f"button {text}",
            require_trusted=require_trusted,
            click_context=click_context,
        )

    def _activate_prepared_click(
        self,
        x: float,
        y: float,
        fallback_expression: str,
        description: str,
        *,
        require_trusted: bool,
        click_context: int,
    ) -> dict[str, Any]:
        """Prefer one trusted CDP click, then use one verified browser fallback."""
        self.cdp.call("Page.bringToFront")
        self.cdp.evaluate("window.focus(); true", context_id=click_context)
        self._mouse_click(x, y)
        try:
            click = self.cdp.evaluate(
                "globalThis.__robotswarmAcceptanceClick || null",
                context_id=click_context,
            )
        except DriverError:
            # A navigation can replace the JavaScript context immediately after
            # a successful click.  The caller's state wait remains authoritative.
            evidence = {
                "description": description,
                "received": True,
                "trusted": None,
                "fallbackUsed": False,
                "contextReplaced": True,
            }
            if require_trusted:
                raise DriverError(f"Could not prove a trusted click for {description}")
            self._click_evidence.append(evidence)
            return evidence
        if isinstance(click, dict) and click.get("received") is True:
            evidence = {
                "description": description,
                "received": True,
                "trusted": click.get("trusted") is True,
                "fallbackUsed": False,
                "contextReplaced": False,
            }
            if require_trusted and evidence["trusted"] is not True:
                raise DriverError(f"The click for {description} was not a trusted browser event")
            self._click_evidence.append(evidence)
            return evidence
        if require_trusted:
            raise DriverError(f"Chrome did not dispatch a trusted click for {description}")
        if self.cdp.evaluate(fallback_expression, context_id=click_context) is not True:
            raise DriverError(f"Could not activate {description}")
        evidence = {
            "description": description,
            "received": True,
            "trusted": False,
            "fallbackUsed": True,
            "contextReplaced": False,
        }
        self._click_evidence.append(evidence)
        return evidence

    def click_evidence(self) -> list[dict[str, Any]]:
        return [dict(item) for item in self._click_evidence]

    def _mouse_click(self, x: float, y: float) -> None:
        self.cdp.call(
            "Input.dispatchMouseEvent",
            {
                "type": "mouseMoved",
                "x": x,
                "y": y,
                "buttons": 0,
                "pointerType": "mouse",
            },
        )
        self.cdp.call(
            "Input.dispatchMouseEvent",
            {
                "type": "mousePressed",
                "x": x,
                "y": y,
                "button": "left",
                "buttons": 1,
                "clickCount": 1,
                "pointerType": "mouse",
            },
        )
        self.cdp.call(
            "Input.dispatchMouseEvent",
            {
                "type": "mouseReleased",
                "x": x,
                "y": y,
                "button": "left",
                "buttons": 0,
                "clickCount": 1,
                "pointerType": "mouse",
            },
        )

    def select_option(self, label_id: str, option_text: str) -> None:
        """Open a Material UI Select and choose its visible option with mouse events."""
        click_context = self.cdp.create_isolated_world()
        control_point = self.cdp.evaluate(f"""
            (() => {{
                const labelId = {json.dumps(label_id)};
                const label = document.getElementById(labelId);
                const labelled = [...document.querySelectorAll('[aria-labelledby]')]
                    .find(item => (item.getAttribute('aria-labelledby') || '').split(/\\s+/).includes(labelId));
                const control = labelled || label?.parentElement?.querySelector(
                    '[role="combobox"], [role="button"], .MuiSelect-select'
                );
                if (!control || control.getAttribute('aria-disabled') === 'true') return false;
                control.scrollIntoView({{block: 'center'}});
                const box = control.getBoundingClientRect();
                const x = box.left + box.width / 2;
                const y = box.top + box.height / 2;
                const hit = document.elementFromPoint(x, y);
                if (!hit || !control.contains(hit)) return false;
                globalThis.__robotswarmAcceptanceClick = {{received: false, trusted: false}};
                control.addEventListener('click', event => {{
                    globalThis.__robotswarmAcceptanceClick = {{
                        received: true,
                        trusted: Boolean(event.isTrusted),
                    }};
                }}, {{capture: true, once: true}});
                return {{x, y}};
            }})()
        """, context_id=click_context)
        if not isinstance(control_point, dict):
            raise DriverError(f"Material UI select was not found: {label_id}")
        self._activate_prepared_click(
            float(control_point["x"]),
            float(control_point["y"]),
            f"""
                (() => {{
                    const label = document.getElementById({json.dumps(label_id)});
                    const control = label
                        ? document.querySelector(`[aria-labelledby~="${{label.id}}"]`)
                        : null;
                    if (!control || control.getAttribute('aria-disabled') === 'true') return false;
                    control.click();
                    return true;
                }})()
            """,
            f"Material UI select {label_id}",
            require_trusted=False,
            click_context=click_context,
        )

        option_expression = self._text_element_point_expression('[role="option"]', option_text)
        deadline = time.monotonic() + 12
        while time.monotonic() < deadline:
            self.raise_if_interrupted()
            point = self.cdp.evaluate(option_expression)
            if isinstance(point, dict):
                click_context = self.cdp.create_isolated_world()
                armed_point = self.cdp.evaluate(
                    self._text_element_point_expression(
                        '[role="option"]', option_text, arm_click_probe=True
                    ),
                    context_id=click_context,
                )
                if not isinstance(armed_point, dict):
                    raise DriverError(f"Material UI option disappeared: {option_text}")
                self._activate_prepared_click(
                    float(armed_point["x"]),
                    float(armed_point["y"]),
                    self._text_element_expression(
                        '[role="option"]', option_text, click=True
                    ),
                    f"Material UI option {option_text}",
                    require_trusted=False,
                    click_context=click_context,
                )
                selected_expression = f"""
                    (() => {{
                        const labelId = {json.dumps(label_id)};
                        const normalize = value => (value || '').replace(/\\s+/g, ' ').trim();
                        const control = [...document.querySelectorAll('[aria-labelledby]')]
                            .find(item => (item.getAttribute('aria-labelledby') || '').split(/\\s+/).includes(labelId));
                        return normalize(control?.textContent) === {json.dumps(option_text)};
                    }})()
                """
                self.wait_js(selected_expression, 8, f"selected option {option_text}")
                closed_expression = f"""
                    (() => {{
                        const labelId = {json.dumps(label_id)};
                        const control = [...document.querySelectorAll('[aria-labelledby]')]
                            .find(item => (item.getAttribute('aria-labelledby') || '')
                                .split(/\\s+/).includes(labelId));
                        if (!control || control.getAttribute('aria-expanded') === 'true') return false;
                        const blocksPointer = element => {{
                            const style = getComputedStyle(element);
                            const box = element.getBoundingClientRect();
                            return style.display !== 'none'
                                && style.visibility !== 'hidden'
                                && style.pointerEvents !== 'none'
                                && box.width > 0
                                && box.height > 0;
                        }};
                        return ![...document.querySelectorAll(
                            '[role="listbox"], .MuiMenu-root, .MuiPopover-root, '
                            + '.MuiModal-root, .MuiBackdrop-root'
                        )].some(blocksPointer);
                    }})()
                """
                self.wait_js(
                    closed_expression,
                    8,
                    f"closed option menu {option_text}",
                )
                return
            time.sleep(0.2)
        raise DriverError(f"Material UI option did not open: {option_text}")

    def wait_clickable_button(self, text: str, timeout: float = 8) -> None:
        expression = self._text_element_point_expression("button", text)
        self.wait_js(
            f"Boolean({expression})",
            timeout,
            f"uncovered button {text}",
        )

    def select_task_type(self, task_type: str) -> None:
        """Choose one of the task cards exposed as an accessible radio."""
        selector = f'[data-testid="task-option-{task_type}"]'
        self.click_selector(selector, f"task type {task_type}")
        self.wait_js(
            f"document.querySelector({json.dumps(selector)})?.getAttribute('aria-checked') === 'true'",
            8,
            f"selected task type {task_type}",
        )

    @staticmethod
    def _text_element_expression(selector: str, text: str, *, click: bool) -> str:
        action = "element.click(); return true;" if click else "return true;"
        return f"""
            (() => {{
                const wanted = {json.dumps(text)};
                const normalize = value => (value || '').replace(/\\s+/g, ' ').trim();
                const element = [...document.querySelectorAll({json.dumps(selector)})]
                    .find(item => normalize(item.textContent) === wanted && item.offsetParent !== null);
                if (!element || element.disabled) return false;
                element.scrollIntoView({{block: 'center'}});
                {action}
            }})()
        """

    @staticmethod
    def _text_element_point_expression(
        selector: str,
        text: str,
        *,
        arm_click_probe: bool = False,
    ) -> str:
        arm = """
                globalThis.__robotswarmAcceptanceClick = {received: false, trusted: false};
                element.addEventListener('click', event => {
                    globalThis.__robotswarmAcceptanceClick = {
                        received: true,
                        trusted: Boolean(event.isTrusted),
                    };
                }, {capture: true, once: true});
        """ if arm_click_probe else ""
        return f"""
            (() => {{
                const wanted = {json.dumps(text)};
                const normalize = value => (value || '').replace(/\\s+/g, ' ').trim();
                const element = [...document.querySelectorAll({json.dumps(selector)})]
                    .find(item => normalize(item.textContent) === wanted && item.offsetParent !== null);
                if (!element || element.disabled || element.getAttribute('aria-disabled') === 'true') return false;
                element.scrollIntoView({{block: 'center'}});
                const box = element.getBoundingClientRect();
                const x = box.left + box.width / 2;
                const y = box.top + box.height / 2;
                const hit = document.elementFromPoint(x, y);
                if (!hit || !element.contains(hit)) return false;
                {arm}
                return {{x, y}};
            }})()
        """

    def login(self, email: str, password: str) -> None:
        self.assert_origin()
        self.wait_js(
            f"document.querySelector({json.dumps(LOGIN_EMAIL)}) || document.querySelector({json.dumps(WORKSPACE_SELECTOR)})",
            45,
            "login or workspace",
        )
        if self.cdp.evaluate(f"Boolean(document.querySelector({json.dumps(LOGIN_EMAIL)}))"):
            self.fill(LOGIN_EMAIL, email)
            self.fill(LOGIN_PASSWORD, password)
            self.wait_js(
                f"!document.querySelector({json.dumps(LOGIN_SUBMIT)})?.disabled",
                15,
                "enabled login button",
            )
            self.click_selector(LOGIN_SUBMIT, "login")
        self.wait_js(
            f"Boolean(document.querySelector({json.dumps(WORKSPACE_SELECTOR)}))",
            75,
            "simulation workspace",
        )
        self.assert_origin()

    def create_session(self, robot_count: int) -> None:
        if not self.has_button(CREATE_BUTTON):
            raise DriverError("Account already has a session; refusing to claim or stop it")
        expression = f"""
            (() => {{
                const heading = [...document.querySelectorAll('h1,h2,h3,h4,h5,h6')]
                    .find(item => item.textContent.trim() === {json.dumps(WORKSPACE_HEADING)});
                const panel = heading?.closest('.MuiPaper-root');
                const slider = panel?.querySelector('input[type="range"]');
                if (!slider) return false;
                const setter = Object.getOwnPropertyDescriptor(HTMLInputElement.prototype, 'value').set;
                setter.call(slider, {json.dumps(str(robot_count))});
                slider.dispatchEvent(new Event('input', {{bubbles: true}}));
                slider.dispatchEvent(new Event('change', {{bubbles: true}}));
                return true;
            }})()
        """
        if self.cdp.evaluate(expression) is not True:
            raise DriverError("Robot-count slider was not found")
        self.wait_js(
            f"document.body.innerText.includes({json.dumps(f'Robots: {robot_count}')})",
            10,
            "selected robot count",
        )
        self.create_requested = True
        self.click_button(CREATE_BUTTON)
        self.wait_js(
            "document.body.innerText.includes('Arena:') && !document.body.innerText.includes('Crear simulación')",
            75,
            "created simulation",
        )
        self.created_session = True

    def wait_ready(self, robot_count: int, timeout: float) -> dict[str, Any]:
        condition = f"""
            (() => {{
                const text = document.body.innerText;
                return text.includes('Sesión de simulación') && text.includes('Lista para operar') &&
                    text.includes({json.dumps(f'Robots activos: {robot_count}')});
            }})()
        """
        self.wait_js(condition, timeout, f"Ready state for {robot_count} robots")
        return {"state": "Ready", "activeRobots": robot_count}

    def request_viewer(self) -> None:
        self.click_button(OPEN_VIEWER_BUTTON)

    def _viewer_frame_state(self) -> dict[str, Any]:
        state = self.cdp.evaluate("""
            (() => {
                const videos = [...document.querySelectorAll('video')]
                    .filter(item => item.offsetParent !== null);
                return {
                    visibilityState: document.visibilityState,
                    focused: document.hasFocus(),
                    decoded: videos.some(video =>
                        video.readyState >= 2
                        && video.videoWidth > 0
                        && video.videoHeight > 0
                    ),
                };
            })()
        """)
        if not isinstance(state, dict):
            raise DriverError("Could not inspect the private viewer frame state")
        return state

    def _restore_viewer_visibility(self) -> None:
        """Restore the real owned window and activate its page without emulation."""
        try:
            window = self.cdp.call(
                "Browser.getWindowForTarget",
                timeout=3.0,
            )
            window_id = window.get("windowId")
            bounds = window.get("bounds") or {}
            if isinstance(window_id, int) and bounds.get("windowState") == "minimized":
                self.cdp.call(
                    "Browser.setWindowBounds",
                    {
                        "windowId": window_id,
                        "bounds": {"windowState": "normal"},
                    },
                    timeout=3.0,
                )
        except DriverError:
            # Page activation still recovers a background tab when the Browser
            # window domain is unavailable on a particular Chrome build.
            pass
        self.cdp.call("Page.bringToFront")
        self.cdp.evaluate("window.focus(); true")

    def wait_viewer_frame(self, timeout: float) -> None:
        deadline = time.monotonic() + timeout
        next_visibility_recovery = 0.0
        while time.monotonic() < deadline:
            self.raise_if_interrupted()
            state = self._viewer_frame_state()
            if (
                state.get("visibilityState") == "visible"
                and state.get("decoded") is True
            ):
                return

            now = time.monotonic()
            if (
                state.get("visibilityState") != "visible"
                and now >= next_visibility_recovery
            ):
                self._restore_viewer_visibility()
                next_visibility_recovery = now + VIEWER_VISIBILITY_RETRY_SECONDS

            remaining = deadline - time.monotonic()
            if remaining > 0:
                time.sleep(min(0.35, remaining))
        raise DriverError("Timed out waiting for a decoded private viewer frame")

    def open_viewer(self, timeout: float) -> None:
        self.request_viewer()
        self.wait_viewer_frame(timeout)

    def viewer_startup_state(self) -> dict[str, Any]:
        """Return a bounded, token-free snapshot when the first frame is late."""
        result = self.cdp.evaluate(r"""
            (() => {
                const clean = value => (value || '').replace(/\s+/g, ' ').trim().slice(0, 240);
                const visible = element => Boolean(element && element.offsetParent !== null);
                return {
                    path: location.pathname,
                    visibilityState: document.visibilityState,
                    focused: document.hasFocus(),
                    userActivationActive: Boolean(navigator.userActivation?.isActive),
                    userActivationSeen: Boolean(navigator.userActivation?.hasBeenActive),
                    mediaSource: Boolean(window.MediaSource || window.WebKitMediaSource),
                    sourceBuffer: Boolean(window.SourceBuffer || window.WebKitSourceBuffer),
                    baselineAvc: Boolean(
                        (window.MediaSource || window.WebKitMediaSource)?.isTypeSupported?.(
                            'video/mp4;codecs="avc1.42E01E,mp4a.40.2"'
                        )
                    ),
                    hlsResourceCount: performance.getEntriesByType('resource').filter(entry =>
                        /\/hls\/|\.m3u8(?:$|\?)/i.test(entry.name)
                    ).length,
                    preparing: visible(document.querySelector('[data-testid="viewer-preparing"]')),
                    commandError: visible(document.querySelector('[data-testid="viewer-command-error"]')),
                    privateViewerMounted: visible(document.querySelector('[data-testid="private-viewer"]')),
                    closing: visible(document.querySelector('[data-testid="viewer-closing"]')),
                    statuses: [...document.querySelectorAll(
                        '[data-testid="viewer-status"], [role="alert"], [data-testid="viewer-preparing"]'
                    )].filter(visible).map(element => clean(element.textContent)).filter(Boolean).slice(0, 8),
                    videos: [...document.querySelectorAll('video')].map(video => ({
                        visible: visible(video),
                        readyState: video.readyState,
                        width: video.videoWidth,
                        height: video.videoHeight,
                        paused: video.paused,
                        mediaErrorCode: video.error?.code || null,
                    })).slice(0, 4),
                    openButton: [...document.querySelectorAll('button')].some(button =>
                        clean(button.textContent) === 'Abrir visor' && !button.disabled),
                    closeButton: [...document.querySelectorAll('button')].some(button =>
                        clean(button.textContent).startsWith('Cerrar visor') && !button.disabled),
                };
            })()
        """)
        if not isinstance(result, dict):
            raise DriverError("Could not inspect the private viewer startup state")
        return result

    def close_viewer_while_task_runs(self, continuity_seconds: float = 10.0) -> dict[str, Any]:
        before = self.task_status()
        if before.get("state") != "Running":
            raise DriverError("The viewer-close check requires a running task")

        self.click_button(CLOSE_VIEWER_BUTTON)
        self.wait_js(
            """
                !document.querySelector('[data-testid="private-viewer"]') &&
                !document.querySelector('[data-testid="viewer-closing"]') &&
                [...document.querySelectorAll('button')].some(button =>
                    button.textContent.trim() === 'Abrir visor' && !button.disabled)
            """,
            90,
            "completed viewer close",
        )

        deadline = time.monotonic() + continuity_seconds
        samples = 0
        while time.monotonic() < deadline:
            state = self.task_status()
            if state.get("state") != "Running":
                raise DriverError(
                    f"The ROS task stopped after closing only the viewer ({state.get('state')})"
                )
            if self.has_button(CREATE_BUTTON) or not self.has_button(STOP_BUTTON):
                raise DriverError("The simulation session was released while closing only its viewer")
            samples += 1
            time.sleep(min(0.5, max(0.0, deadline - time.monotonic())))

        return {
            "viewerClosed": True,
            "continuitySeconds": continuity_seconds,
            "runningSamples": samples,
            "taskBefore": before,
            "taskAfter": self.task_status(),
            "sessionStillAllocated": True,
        }

    def viewer_state(self) -> dict[str, Any]:
        result = self.cdp.evaluate("""
            (() => {
                const viewer = document.querySelector('[data-testid="private-viewer"]');
                const video = document.querySelector('[data-testid="viewer-video"]');
                const status = document.querySelector('[data-testid="viewer-status"]');
                const fps = document.querySelector('[data-testid="viewer-fps"]');
                const lease = document.querySelector('[data-testid="viewer-lease-countdown"]');
                const control = document.querySelector('[aria-label="Activar control interactivo"], '
                    + '[aria-label="Desactivar control interactivo"]');
                const fullscreen = document.querySelector('[aria-label="Abrir visor en pantalla completa"], '
                    + '[aria-label="Salir de pantalla completa"]');
                const fullscreenElement = document.fullscreenElement
                    || document.webkitFullscreenElement
                    || document.webkitCurrentFullScreenElement
                    || null;
                const visibleAlerts = [...document.querySelectorAll('[role="alert"]')]
                    .filter(item => item.offsetParent !== null)
                    .map(item => (item.textContent || '').replace(/\\s+/g, ' ').trim());
                if (!viewer || !video) return null;
                const box = viewer.getBoundingClientRect();
                return {
                    status: (status?.textContent || '').trim(),
                    fps: (fps?.getAttribute('aria-label') || fps?.textContent || '').trim(),
                    leaseCountdown: (lease?.getAttribute('aria-label') || lease?.textContent || '').trim(),
                    hlsInteractive: !document.querySelector('[data-testid="whep-fallback-note"]'),
                    controlText: (control?.textContent || '').trim(),
                    controlDisabled: Boolean(control?.disabled),
                    fullscreenText: (fullscreen?.textContent || '').trim(),
                    isFullscreen: fullscreenElement === viewer,
                    viewerRect: {x: box.x, y: box.y, width: box.width, height: box.height},
                    viewport: {width: innerWidth, height: innerHeight},
                    alerts: visibleAlerts,
                    video: {
                        readyState: video.readyState,
                        width: video.videoWidth,
                        height: video.videoHeight,
                        paused: video.paused,
                    },
                };
            })()
        """)
        if not isinstance(result, dict):
            raise DriverError("The private viewer state could not be inspected")
        return result

    def require_interactive_hls(self) -> dict[str, Any]:
        self.wait_js(
            """
                (() => {
                    const status = document.querySelector('[data-testid="viewer-status"]')?.textContent.trim();
                    const fpsChip = document.querySelector('[data-testid="viewer-fps"]');
                    const fps = (fpsChip?.getAttribute('aria-label') || fpsChip?.textContent || '').trim();
                    const leaseChip = document.querySelector('[data-testid="viewer-lease-countdown"]');
                    const lease = (leaseChip?.getAttribute('aria-label') || leaseChip?.textContent || '').trim();
                    const leaseParts = /^Vence en (0|[1-9]\\d*):([0-5]\\d)$/.exec(lease);
                    const leaseSeconds = leaseParts
                        ? Number(leaseParts[1]) * 60 + Number(leaseParts[2])
                        : 0;
                    const control = document.querySelector('[aria-label="Activar control interactivo"]');
                    return status === 'En vivo' && /^Video \\d+(?:\\.\\d+)? FPS$/.test(fps)
                        && leaseSeconds > 0 && leaseSeconds <= 30 * 60
                        && control && !control.disabled
                        && !document.querySelector('[data-testid="whep-fallback-note"]');
                })()
            """,
            30,
            "an interactive HLS viewer with decoded FPS",
        )
        state = self.viewer_state()
        if not state.get("hlsInteractive"):
            raise DriverError("The viewer fell back to WHEP, which is video-only")
        if state.get("status") != "En vivo":
            raise DriverError(f"The private viewer was not live: {state.get('status')}")
        if state.get("controlDisabled"):
            raise DriverError("The interactive-control button was disabled")
        if not re.fullmatch(r"Video \d+(?:\.\d+)? FPS", str(state.get("fps", ""))):
            raise DriverError("The viewer did not expose a numeric decoded FPS value")
        state["leaseSecondsRemaining"] = parse_viewer_lease_countdown(
            state.get("leaseCountdown")
        )
        video = state.get("video") or {}
        if int(video.get("readyState") or 0) < 2 or int(video.get("width") or 0) <= 0:
            raise DriverError("The interactive viewer had no decoded video frame")
        return state

    def video_content_rect(self) -> dict[str, float]:
        if self.cdp.evaluate("""
            (() => {
                const video = document.querySelector('[data-testid="viewer-video"]');
                if (!video) return false;
                video.scrollIntoView({block: 'center', inline: 'center'});
                return true;
            })()
        """) is not True:
            raise DriverError("The viewer video was not found")
        time.sleep(0.2)
        result = self.cdp.evaluate("""
            (() => {
                const video = document.querySelector('[data-testid="viewer-video"]');
                if (!video || video.videoWidth <= 0 || video.videoHeight <= 0) return null;
                const box = video.getBoundingClientRect();
                const scale = Math.min(box.width / video.videoWidth, box.height / video.videoHeight);
                const width = video.videoWidth * scale;
                const height = video.videoHeight * scale;
                const x = box.left + (box.width - width) / 2;
                const y = box.top + (box.height - height) / 2;
                const visibleWidth = Math.max(0, Math.min(x + width, innerWidth) - Math.max(x, 0));
                const visibleHeight = Math.max(0, Math.min(y + height, innerHeight) - Math.max(y, 0));
                return {
                    x,
                    y,
                    width,
                    height,
                    visibleFraction: (visibleWidth * visibleHeight) / (width * height),
                    viewportWidth: innerWidth,
                    viewportHeight: innerHeight,
                };
            })()
        """)
        if not isinstance(result, dict) or min(float(result.get(key, 0)) for key in ("width", "height")) < 100:
            raise DriverError("The useful video rectangle could not be measured")
        if float(result.get("visibleFraction", 0)) < 0.95:
            raise DriverError("Less than 95% of the useful video region was visible")
        return {key: float(value) for key, value in result.items()}

    @staticmethod
    def _average_hash(image: Image.Image) -> str:
        resampling = getattr(Image, "Resampling", Image).LANCZOS
        sample = image.convert("L").resize((16, 16), resampling)
        values = list(sample.getdata())
        threshold = sum(values) / len(values)
        bits = "".join("1" if value >= threshold else "0" for value in values)
        return f"{int(bits, 2):064x}"

    def capture_video_clip(self, destination: Path) -> dict[str, Any]:
        rect = self.video_content_rect()
        prepared = self.cdp.evaluate("""
            (() => {
                const viewer = document.querySelector('[data-testid="private-viewer"]');
                const video = document.querySelector('[data-testid="viewer-video"]');
                if (!viewer || !video || window.__robotswarmClipRestore) return false;
                const hidden = [...viewer.children]
                    .filter(item => item !== video)
                    .map(item => ({item, visibility: item.style.visibility}));
                hidden.forEach(entry => { entry.item.style.visibility = 'hidden'; });
                const outline = video.style.outline;
                const cursor = video.style.cursor;
                video.style.outline = 'none';
                video.style.cursor = 'default';
                window.__robotswarmClipRestore = () => {
                    hidden.forEach(entry => { entry.item.style.visibility = entry.visibility; });
                    video.style.outline = outline;
                    video.style.cursor = cursor;
                    delete window.__robotswarmClipRestore;
                };
                return true;
            })()
        """)
        if prepared is not True:
            raise DriverError("The video could not be prepared for an overlay-free capture")
        try:
            captured = self.cdp.call(
                "Page.captureScreenshot",
                {"format": "png", "fromSurface": True, "captureBeyondViewport": False},
                timeout=30,
            )
            raw = base64.b64decode(captured.get("data", ""), validate=True)
            if not raw.startswith(b"\x89PNG\r\n\x1a\n") or len(raw) > 30 * 1024 * 1024:
                raise DriverError("Chrome returned an invalid video screenshot")
            with Image.open(io.BytesIO(raw)) as screenshot:
                scale_x = screenshot.width / rect["viewportWidth"]
                scale_y = screenshot.height / rect["viewportHeight"]
                inset_x = rect["width"] * 0.04
                inset_y = rect["height"] * 0.08
                crop_box = (
                    max(0, round((rect["x"] + inset_x) * scale_x)),
                    max(0, round((rect["y"] + inset_y) * scale_y)),
                    min(screenshot.width, round((rect["x"] + rect["width"] - inset_x) * scale_x)),
                    min(screenshot.height, round((rect["y"] + rect["height"] - inset_y) * scale_y)),
                )
                clip = screenshot.crop(crop_box)
                average_hash = self._average_hash(clip)
                buffer = io.BytesIO()
                clip.save(buffer, format="PNG")
                clip_raw = buffer.getvalue()
                clip_size = {"width": clip.width, "height": clip.height}
        finally:
            self.cdp.evaluate("window.__robotswarmClipRestore?.()")
        write_bytes_secure(destination, clip_raw)
        return {
            "file": destination.name,
            "sha256": hashlib.sha256(clip_raw).hexdigest(),
            "averageHash": average_hash,
            "bytes": len(clip_raw),
            **clip_size,
        }

    @staticmethod
    def _hash_distance(left: str, right: str) -> int:
        return bin(int(left, 16) ^ int(right, 16)).count("1")

    @staticmethod
    def _scene_difference(left: Path, right: Path) -> float:
        with Image.open(left) as left_image, Image.open(right) as right_image:
            left_rgb = left_image.convert("RGB")
            right_rgb = right_image.convert("RGB")
            if left_rgb.size != right_rgb.size:
                raise DriverError("The synchronized private scenes have different dimensions")
            return pixel_difference_ratio(list(left_rgb.getdata()), list(right_rgb.getdata()))

    def _require_control_healthy(self, gesture: str) -> dict[str, Any]:
        time.sleep(0.25)
        state = self.viewer_state()
        if state.get("controlText") != "Control activo" or state.get("alerts"):
            raise DriverError(f"Interactive control became unhealthy after {gesture}")
        return {"dispatched": True, "controlHealthyAfter": True}

    def enable_viewer_control(self) -> None:
        selector = '[aria-label="Activar control interactivo"]'
        self.wait_js(
            f"""
                (() => {{
                    const button = document.querySelector({json.dumps(selector)});
                    return button && !button.disabled
                        && button.getAttribute('aria-disabled') !== 'true';
                }})()
            """,
            30,
            "interactive viewer control authorization",
        )
        self.click_selector(selector, "interactive viewer control")
        self.wait_js(
            "Boolean(document.querySelector('[aria-label=\"Desactivar control interactivo\"]'))",
            20,
            "interactive viewer grant",
        )

    def exercise_interaction(self, output_dir: Path, run_id: str, label: str) -> dict[str, Any]:
        before_state = self.require_interactive_hls()
        baseline_samples: list[dict[str, Any]] = []
        for index in range(6):
            self.raise_if_interrupted()
            baseline_samples.append(self.capture_video_clip(
                output_dir
                / f"{run_id}-{label.lower()}-interaction-baseline-{index + 1}.png"
            ))
            if index < 5:
                time.sleep(2)
        before = baseline_samples[-1]
        baseline_distance = max(
            self._hash_distance(left["averageHash"], right["averageHash"])
            for offset, left in enumerate(baseline_samples)
            for right in baseline_samples[offset + 1:]
        )
        control_enabled = False
        mouse_down = False
        key_down = False
        last_x = 0.0
        last_y = 0.0
        gesture_checks: dict[str, dict[str, Any]] = {}
        try:
            self.enable_viewer_control()
            control_enabled = True
            rect = self.video_content_rect()
            start_x = rect["x"] + rect["width"] * 0.42
            start_y = rect["y"] + rect["height"] * 0.48
            end_x = rect["x"] + rect["width"] * 0.63
            end_y = rect["y"] + rect["height"] * 0.38
            last_x, last_y = start_x, start_y
            self._mouse_click(start_x, start_y)
            gesture_checks["click"] = self._require_control_healthy("click")

            self.cdp.call("Input.dispatchMouseEvent", {
                "type": "mousePressed", "x": start_x, "y": start_y, "button": "left", "buttons": 1,
                "clickCount": 1,
            })
            mouse_down = True
            for step in range(1, 11):
                ratio = step / 10
                last_x = start_x + (end_x - start_x) * ratio
                last_y = start_y + (end_y - start_y) * ratio
                self.cdp.call("Input.dispatchMouseEvent", {
                    "type": "mouseMoved", "x": last_x, "y": last_y,
                    "button": "left", "buttons": 1,
                })
                time.sleep(0.05)
            self.cdp.call("Input.dispatchMouseEvent", {
                "type": "mouseReleased", "x": end_x, "y": end_y, "button": "left", "buttons": 0,
                "clickCount": 1,
            })
            mouse_down = False
            gesture_checks["drag"] = self._require_control_healthy("drag")

            self.cdp.call("Input.dispatchMouseEvent", {
                "type": "mouseWheel", "x": end_x, "y": end_y, "deltaX": 0, "deltaY": -180,
            })
            gesture_checks["wheel"] = self._require_control_healthy("wheel")

            self.cdp.call("Input.dispatchKeyEvent", {
                "type": "rawKeyDown", "code": "KeyW", "key": "w", "windowsVirtualKeyCode": 87,
                "nativeVirtualKeyCode": 87,
            })
            key_down = True
            time.sleep(0.12)
            self.cdp.call("Input.dispatchKeyEvent", {
                "type": "keyUp", "code": "KeyW", "key": "w", "windowsVirtualKeyCode": 87,
                "nativeVirtualKeyCode": 87,
            })
            key_down = False
            gesture_checks["KeyW"] = self._require_control_healthy("KeyW")
            time.sleep(2)

            self.click_button("Control activo")
            self.wait_js(
                "document.querySelector('[aria-label=\"Activar control interactivo\"]')?.textContent.includes('Interactuar')",
                15,
                "interactive control release",
            )
            control_enabled = False
            after_path = output_dir / f"{run_id}-{label.lower()}-interaction-after.png"
            after = self.capture_video_clip(after_path)
            distance = self._hash_distance(before["averageHash"], after["averageHash"])
            required_distance = max(8, baseline_distance + 4)
            deadline = time.monotonic() + 10
            while distance < required_distance and time.monotonic() < deadline:
                time.sleep(2)
                after = self.capture_video_clip(after_path)
                distance = self._hash_distance(before["averageHash"], after["averageHash"])
            if distance < required_distance:
                raise DriverError(
                    "The Gazebo view did not respond above baseline noise "
                    f"(distance {distance}, baseline {baseline_distance})"
                )
            return {
                "hlsInteractive": True,
                "gestures": gesture_checks,
                "combinedVisualResponse": True,
                "before": before,
                "after": after,
                "baselineSamples": baseline_samples,
                "maximumIdleDistance": baseline_distance,
                "averageHashDistance": distance,
                "requiredAverageHashDistance": required_distance,
                "controlReleased": True,
                "decodedFpsLabel": before_state.get("fps"),
            }
        finally:
            if key_down:
                with contextlib.suppress(Exception):
                    self.cdp.call("Input.dispatchKeyEvent", {
                        "type": "keyUp", "code": "KeyW", "key": "w", "windowsVirtualKeyCode": 87,
                        "nativeVirtualKeyCode": 87,
                    })
            if mouse_down or control_enabled:
                with contextlib.suppress(Exception):
                    self.cdp.call("Input.dispatchMouseEvent", {
                        "type": "mouseReleased", "x": last_x, "y": last_y, "button": "left",
                        "buttons": 0, "clickCount": 1,
                    })
            with contextlib.suppress(Exception):
                if self.has_button("Control activo"):
                    self.click_button("Control activo")

    def exercise_fullscreen(
        self,
        destination: Path,
        email: str,
        password: str,
    ) -> dict[str, Any]:
        entered = False
        left_with_escape = False
        try:
            self.click_button("Pantalla completa", require_trusted=True)
            self.wait_js(
                "document.fullscreenElement === document.querySelector('[data-testid=\"private-viewer\"]')",
                15,
                "viewer fullscreen",
            )
            entered = True
            state = self.viewer_state()
            viewer = state["viewerRect"]
            viewport = state["viewport"]
            covers_viewport = (
                abs(float(viewer["x"])) <= 2
                and abs(float(viewer["y"])) <= 2
                and abs(float(viewer["width"]) - float(viewport["width"])) <= 2
                and abs(float(viewer["height"]) - float(viewport["height"])) <= 2
            )
            if not covers_viewport:
                raise DriverError("The fullscreen viewer did not cover the browser viewport")
            screenshot = self.screenshot(destination, email, password)
            windows_screenshot = owned_window_screenshot(
                self,
                destination.with_name(destination.stem + "-windows.png"),
                email,
                password,
                f"RobotSwarm-Fullscreen-{self.chrome.run_id}-{self.chrome.label}",
            )
            self.cdp.call("Input.dispatchKeyEvent", {
                "type": "rawKeyDown", "code": "Escape", "key": "Escape", "windowsVirtualKeyCode": 27,
                "nativeVirtualKeyCode": 27,
            })
            self.cdp.call("Input.dispatchKeyEvent", {
                "type": "keyUp", "code": "Escape", "key": "Escape", "windowsVirtualKeyCode": 27,
                "nativeVirtualKeyCode": 27,
            })
            self.wait_js(
                "!document.fullscreenElement && !document.webkitFullscreenElement",
                15,
                "Escape leaving fullscreen",
            )
            left_with_escape = True
            # Chrome clears fullscreenElement before React has necessarily
            # rendered the ordinary toolbar again. Wait for the public UI
            # contract instead of sampling that short transition as a failure.
            self.wait_js(
                """
                    (() => {
                        const status = document.querySelector(
                            '[data-testid="viewer-status"]'
                        )?.textContent.trim();
                        const button = document.querySelector(
                            '[aria-label="Abrir visor en pantalla completa"]'
                        );
                        return status === 'En vivo'
                            && (button?.textContent || '').trim()
                                === 'Pantalla completa';
                    })()
                """,
                5,
                "live controls after leaving fullscreen",
            )
            restored = self.viewer_state()
            if restored.get("status") != "En vivo" or restored.get("fullscreenText") != "Pantalla completa":
                raise DriverError("The viewer did not restore its live non-fullscreen controls")
            return {
                "entered": True,
                "coveredViewport": True,
                "leftWithEscape": True,
                "pageScreenshot": screenshot,
                "windowsScreenshot": windows_screenshot,
            }
        finally:
            is_fullscreen = False
            with contextlib.suppress(Exception):
                is_fullscreen = bool(self.cdp.evaluate(
                    "Boolean(document.fullscreenElement || document.webkitFullscreenElement)"
                ))
            if entered and not left_with_escape and is_fullscreen:
                with contextlib.suppress(Exception):
                    self.cdp.call("Input.dispatchKeyEvent", {
                        "type": "rawKeyDown", "code": "Escape", "key": "Escape",
                        "windowsVirtualKeyCode": 27, "nativeVirtualKeyCode": 27,
                    })
                    self.cdp.call("Input.dispatchKeyEvent", {
                        "type": "keyUp", "code": "Escape", "key": "Escape",
                        "windowsVirtualKeyCode": 27, "nativeVirtualKeyCode": 27,
                    })
                    time.sleep(1)
            with contextlib.suppress(Exception):
                if self.has_button("Salir"):
                    self.click_button("Salir")
                    time.sleep(1)
            with contextlib.suppress(Exception):
                if self.cdp.evaluate("Boolean(document.fullscreenElement || document.webkitFullscreenElement)"):
                    self.cdp.evaluate(
                        "document.exitFullscreen ? document.exitFullscreen() : document.webkitExitFullscreen?.()",
                        await_promise=True,
                    )

    def normalize_viewer(self) -> dict[str, Any]:
        result: dict[str, Any] = {"controlReleased": False, "fullscreenExited": False}
        with contextlib.suppress(Exception):
            if self.has_button("Control activo"):
                self.click_button("Control activo")
            result["controlReleased"] = not self.has_button("Control activo")
        with contextlib.suppress(Exception):
            if self.cdp.evaluate("Boolean(document.fullscreenElement || document.webkitFullscreenElement)"):
                self.cdp.call("Input.dispatchKeyEvent", {
                    "type": "rawKeyDown", "code": "Escape", "key": "Escape",
                    "windowsVirtualKeyCode": 27, "nativeVirtualKeyCode": 27,
                })
                self.cdp.call("Input.dispatchKeyEvent", {
                    "type": "keyUp", "code": "Escape", "key": "Escape",
                    "windowsVirtualKeyCode": 27, "nativeVirtualKeyCode": 27,
                })
                time.sleep(1)
            if self.cdp.evaluate("Boolean(document.fullscreenElement || document.webkitFullscreenElement)"):
                self.cdp.evaluate(
                    "document.exitFullscreen ? document.exitFullscreen() : document.webkitExitFullscreen?.()",
                    await_promise=True,
                )
            result["fullscreenExited"] = not bool(self.cdp.evaluate(
                "Boolean(document.fullscreenElement || document.webkitFullscreenElement)"
            ))
        return result

    def _task_start_baseline(self) -> tuple[str, set[str]]:
        sessions = self._occupying_sessions()
        if len(sessions) != 1:
            raise DriverError("Task start requires exactly one owned simulation session")
        session_id = sessions[0]["id"]
        tasks = self._task_api_request(session_id)
        return session_id, {task["id"] for task in tasks}

    def _wait_for_new_task(
        self,
        session_id: str,
        previous_ids: set[str],
        expected_type: str,
        timeout: float = 30,
    ) -> dict[str, Any]:
        def reconcile(tasks: list[dict[str, Any]]) -> dict[str, Any] | None:
            created = [task for task in tasks if task["id"] not in previous_ids]
            if len(created) > 1:
                raise DriverError("The UI created more than one task from a single start click")
            if not created:
                return None
            task = created[0]
            if task.get("type") != expected_type:
                raise DriverError("The UI created a different task than the one selected")
            self.started_task_id = task["id"]
            return {
                "type": task["type"],
                "state": task["state"],
                "observedAt": utc_now(),
                "verifiedBy": "authenticated-task-list",
            }

        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            self.raise_if_interrupted()
            accepted = reconcile(self._task_api_request(session_id))
            if accepted is not None:
                return accepted
            time.sleep(0.2)
        self.raise_if_interrupted()
        accepted = reconcile(self._task_api_request(session_id))
        if accepted is not None:
            return accepted
        raise DriverError("The start click did not create a task in the backend")

    def _start_selected_task(
        self,
        expected_type: str,
        start_barrier: threading.Barrier,
    ) -> tuple[float, dict[str, Any]]:
        session_id, previous_ids = self._task_start_baseline()
        self.wait_clickable_button(START_TASK_BUTTON)
        try:
            start_barrier.wait(timeout=30)
        except threading.BrokenBarrierError as exc:
            raise DriverError(
                f"Parallel task-start barrier was broken before {expected_type} click"
            ) from exc
        click_started = time.monotonic()
        self.click_button(START_TASK_BUTTON, require_trusted=True)
        accepted = self._wait_for_new_task(session_id, previous_ids, expected_type)
        self.started_task_session_id = session_id
        return click_started, accepted

    def start_figure_letter_a(self, start_barrier: threading.Barrier) -> dict[str, Any]:
        self.select_task_type("Figure")
        self.wait_js("Boolean(document.getElementById('formation-type-label'))", 10, "figure controls")
        self.select_option("formation-type-label", "Letra")
        self.wait_js(
            "Boolean(document.getElementById('formation-letter-label'))",
            10,
            "letter controls",
        )
        self.select_option("formation-letter-label", "A")
        click_started, accepted = self._start_selected_task("Figure", start_barrier)
        return {
            "requested": "Figure",
            "parameters": {"formation": "A"},
            "clickedAt": utc_now(),
            "backendAcceptance": accepted,
            "_clickMonotonic": click_started,
        }

    def start_follow_figure8(self, start_barrier: threading.Barrier) -> dict[str, Any]:
        self.select_task_type("FollowLeader")
        self.wait_js("Boolean(document.getElementById('leader-mode-label'))", 10, "leader controls")
        self.select_option("leader-mode-label", "Ocho")
        click_started, accepted = self._start_selected_task("FollowLeader", start_barrier)
        return {
            "requested": "FollowLeader",
            "parameters": {"leaderMode": "figure8"},
            "clickedAt": utc_now(),
            "backendAcceptance": accepted,
            "_clickMonotonic": click_started,
        }

    def task_status(self) -> dict[str, Any]:
        result = self.cdp.evaluate(f"""
            (() => {{
                const normalize = value => (value || '').replace(/\\s+/g, ' ').trim();
                const heading = [...document.querySelectorAll('h1,h2,h3,h4,h5,h6')]
                    .find(item => normalize(item.textContent) === {json.dumps(TASK_HEADING)});
                const panel = heading?.closest('.MuiPaper-root');
                if (!panel) return null;
                const text = normalize(panel.innerText);
                const stateLabels = {json.dumps(UI_TASK_STATES, ensure_ascii=False)};
                const outcomeLabels = {json.dumps(UI_OUTCOMES, ensure_ascii=False)};
                const stateChip = panel.querySelector('[data-testid="task-state"] .MuiChip-label');
                const state = stateLabels[normalize(stateChip?.textContent)] || null;
                const outcomeNode = [...panel.querySelectorAll('[data-testid="task-result"] p')]
                    .find(item => normalize(item.textContent).startsWith('Resultado verificado:'));
                const outcomeText = normalize(outcomeNode?.textContent).replace('Resultado verificado:', '').trim();
                const outcome = outcomeLabels[outcomeText] || null;
                const progress = text.match(/\\b(\\d{{1,3}})\\s*%/)?.[1] || null;
                return {{state, outcome, progressPercent: progress === null ? null : Number(progress)}};
            }})()
        """)
        if not isinstance(result, dict):
            return {"state": None, "outcome": None, "progressPercent": None}
        return result

    def show_task_panel(self) -> dict[str, Any]:
        result = self.cdp.evaluate("""
            (() => {
                const panel = document.querySelector('[data-testid="task-panel"]');
                if (!panel) return null;
                panel.scrollIntoView({block: 'center', inline: 'center'});
                const box = panel.getBoundingClientRect();
                const visibleWidth = Math.max(0, Math.min(box.right, innerWidth) - Math.max(box.left, 0));
                const visibleHeight = Math.max(0, Math.min(box.bottom, innerHeight) - Math.max(box.top, 0));
                return {
                    visibleFraction: (visibleWidth * visibleHeight) / Math.max(1, box.width * box.height),
                };
            })()
        """)
        if not isinstance(result, dict) or float(result.get("visibleFraction", 0)) < 0.85:
            raise DriverError("The task result panel could not be made visible")
        time.sleep(0.3)
        return result

    def show_released_session(self) -> dict[str, Any]:
        result = self.cdp.evaluate(f"""
            (() => {{
                const normalize = value => (value || '').replace(/\\s+/g, ' ').trim();
                const button = [...document.querySelectorAll('button')]
                    .find(item => normalize(item.textContent) === {json.dumps(CREATE_BUTTON)}
                        && item.offsetParent !== null);
                if (!button) return null;
                button.scrollIntoView({{block: 'center', inline: 'center'}});
                const box = button.getBoundingClientRect();
                const visibleWidth = Math.max(0, Math.min(box.right, innerWidth) - Math.max(box.left, 0));
                const visibleHeight = Math.max(0, Math.min(box.bottom, innerHeight) - Math.max(box.top, 0));
                const viewer = document.querySelector('[data-testid="private-viewer"]');
                return {{
                    createButtonVisibleFraction:
                        (visibleWidth * visibleHeight) / Math.max(1, box.width * box.height),
                    viewerVisible: Boolean(viewer && viewer.offsetParent !== null),
                }};
            }})()
        """)
        if (
            not isinstance(result, dict)
            or float(result.get("createButtonVisibleFraction", 0)) < 0.95
            or result.get("viewerVisible") is True
        ):
            raise DriverError("The released-session state could not be made visible")
        time.sleep(0.3)
        return result

    def wait_task_state(self, accepted: set[str], timeout: float) -> dict[str, Any]:
        deadline = time.monotonic() + timeout
        last: dict[str, Any] = {"state": None, "outcome": None, "progressPercent": None}
        while time.monotonic() < deadline:
            self.raise_if_interrupted()
            last = self.task_status()
            if last.get("state") in accepted:
                return last
            if last.get("state") in {"Failed", "Cancelled"}:
                raise DriverError(f"Task entered unexpected terminal state {last['state']}")
            time.sleep(0.5)
        raise DriverError(f"Timed out waiting for task state {sorted(accepted)}; last state was {last.get('state')}")

    def wait_verified_figure(self, timeout: float) -> dict[str, Any]:
        deadline = time.monotonic() + timeout
        last: dict[str, Any] = {}
        while time.monotonic() < deadline:
            self.raise_if_interrupted()
            last = self.task_status()
            if last.get("state") == "Completed" and last.get("outcome") == "Succeeded":
                return last
            if last.get("state") in {"Failed", "Cancelled"}:
                raise DriverError(f"Figure task ended as {last.get('state')}")
            if last.get("state") == "Completed" and last.get("outcome") not in {None, "Pending"}:
                raise DriverError(f"Figure task outcome was {last.get('outcome')}, not Succeeded")
            time.sleep(1)
        raise DriverError(
            f"Figure task did not reach Completed/Succeeded; state={last.get('state')}, outcome={last.get('outcome')}"
        )

    def _video_metrics_chunk(self, seconds: float) -> dict[str, Any]:
        self.raise_if_interrupted()
        expression = f"""
            (() => new Promise((resolve, reject) => {{
                const video = document.querySelector('[data-testid="viewer-video"]');
                if (!video) {{ reject(new Error('visible video not found')); return; }}
                video.scrollIntoView({{block: 'center', inline: 'center'}});
                const box = video.getBoundingClientRect();
                const visibleWidth = Math.max(0, Math.min(box.right, innerWidth) - Math.max(box.left, 0));
                const visibleHeight = Math.max(0, Math.min(box.bottom, innerHeight) - Math.max(box.top, 0));
                const visibleFraction = (visibleWidth * visibleHeight) / Math.max(1, box.width * box.height);
                if (video.offsetParent === null || visibleFraction < 0.95) {{
                    reject(new Error('viewer video is not sufficiently visible'));
                    return;
                }}
                const durationMs = {seconds * 1000.0};
                const quality = () => video.getVideoPlaybackQuality ? video.getVideoPlaybackQuality() : null;
                const before = quality();
                const started = performance.now();
                const mediaStarted = video.currentTime;
                let callbackFrames = 0;
                let finished = false;
                let lastPresentedFrames = null;
                let callbackId = null;

                const finish = () => {{
                    if (finished) return;
                    finished = true;
                    if (callbackId !== null && video.cancelVideoFrameCallback) video.cancelVideoFrameCallback(callbackId);
                    const ended = performance.now();
                    const after = quality();
                    const elapsedSeconds = (ended - started) / 1000;
                    const decoded = before && after ? after.totalVideoFrames - before.totalVideoFrames : callbackFrames;
                    const dropped = before && after ? after.droppedVideoFrames - before.droppedVideoFrames : null;
                    resolve({{
                        requestVideoFrameCallbackSupported: typeof video.requestVideoFrameCallback === 'function',
                        getVideoPlaybackQualitySupported: Boolean(before && after),
                        elapsedSeconds,
                        callbackFrames,
                        callbackFps: callbackFrames / elapsedSeconds,
                        decodedFrames: decoded,
                        decodedFps: decoded / elapsedSeconds,
                        droppedFrames: dropped,
                        droppedRatio: dropped === null ? null : dropped / Math.max(1, decoded),
                        // A live HLS player can seek backwards to a new live
                        // edge while it keeps decoding at full rate. Treat
                        // that correction as a regression diagnostic, not as
                        // negative presented time that cancels earlier chunks.
                        mediaTimeAdvancedSeconds: Math.max(
                            0, video.currentTime - mediaStarted
                        ),
                        mediaTimeRegressedSeconds: Math.max(
                            0, mediaStarted - video.currentTime
                        ),
                        readyState: video.readyState,
                        paused: video.paused,
                        playbackRate: video.playbackRate,
                        width: video.videoWidth,
                        height: video.videoHeight,
                        visibleFraction,
                        lastPresentedFrames,
                    }});
                }};
                const onFrame = (_now, metadata) => {{
                    callbackFrames += 1;
                    lastPresentedFrames = metadata.presentedFrames;
                    if (performance.now() - started >= durationMs) finish();
                    else callbackId = video.requestVideoFrameCallback(onFrame);
                }};
                if (typeof video.requestVideoFrameCallback === 'function') {{
                    callbackId = video.requestVideoFrameCallback(onFrame);
                }}
                setTimeout(finish, durationMs + 750);
            }}))()
        """
        result = self.cdp.evaluate(expression, await_promise=True, timeout=seconds + 5)
        self.raise_if_interrupted()
        if not isinstance(result, dict):
            raise DriverError("Video measurement returned an invalid result")
        return result

    def video_metrics(self, seconds: float) -> dict[str, Any]:
        """Measure a long interval in short chunks so a signal can stop it."""
        chunks: list[dict[str, Any]] = []
        remaining = seconds
        while remaining > 0.001:
            self.raise_if_interrupted()
            interval = min(2.0, remaining)
            chunks.append(self._video_metrics_chunk(interval))
            remaining -= interval

        elapsed = sum(float(item.get("elapsedSeconds") or 0) for item in chunks)
        callback_frames = sum(int(item.get("callbackFrames") or 0) for item in chunks)
        decoded_frames = sum(int(item.get("decodedFrames") or 0) for item in chunks)
        dropped_values = [item.get("droppedFrames") for item in chunks]
        dropped_frames = (
            None
            if any(value is None for value in dropped_values)
            else sum(int(value) for value in dropped_values)
        )
        last = chunks[-1]
        return {
            "requestVideoFrameCallbackSupported": all(
                item.get("requestVideoFrameCallbackSupported") is True for item in chunks
            ),
            "getVideoPlaybackQualitySupported": all(
                item.get("getVideoPlaybackQualitySupported") is True for item in chunks
            ),
            "elapsedSeconds": elapsed,
            "callbackFrames": callback_frames,
            "callbackFps": callback_frames / max(elapsed, 0.001),
            "decodedFrames": decoded_frames,
            "decodedFps": decoded_frames / max(elapsed, 0.001),
            "droppedFrames": dropped_frames,
            "droppedRatio": (
                None
                if dropped_frames is None
                else dropped_frames / max(1, decoded_frames)
            ),
            "mediaTimeAdvancedSeconds": sum(
                float(item.get("mediaTimeAdvancedSeconds") or 0) for item in chunks
            ),
            "mediaTimeRegressedSeconds": sum(
                float(item.get("mediaTimeRegressedSeconds") or 0) for item in chunks
            ),
            "readyState": last.get("readyState"),
            "paused": last.get("paused"),
            "playbackRate": last.get("playbackRate"),
            "width": last.get("width"),
            "height": last.get("height"),
            "visibleFraction": min(
                float(item.get("visibleFraction") or 0) for item in chunks
            ),
            "lastPresentedFrames": last.get("lastPresentedFrames"),
            "sampleChunks": len(chunks),
        }

    def prepare_safe_capture(self, email: str, password: str, title_marker: str | None = None) -> None:
        page = self.current_page()
        if page["origin"] != self.expected_origin or page["path"].startswith("/login"):
            raise DriverError("Refusing to capture a login or foreign-origin page")
        safety = self.cdp.evaluate(
            f"""
                (() => {{
                    const privateNodes = [];
                    const hide = element => {{
                        if (!element || privateNodes.some(entry => entry.element === element)) return;
                        privateNodes.push({{element, visibility: element.style.visibility}});
                        element.style.visibility = 'hidden';
                    }};
                    const passwordValue = {json.dumps(password)};
                    const containsBoundedPassword = value => {{
                        let offset = 0;
                        while (offset <= value.length - passwordValue.length) {{
                            const index = value.indexOf(passwordValue, offset);
                            if (index < 0) return false;
                            const before = index === 0 ? '' : value[index - 1];
                            const end = index + passwordValue.length;
                            const after = end >= value.length ? '' : value[end];
                            const startsAtBoundary = !before || !/[A-Za-z0-9]/.test(before);
                            const endsAtBoundary = !after || !/[A-Za-z0-9]/.test(after);
                            if (startsAtBoundary && endsAtBoundary) return true;
                            offset = index + 1;
                        }}
                        return false;
                    }};
                    document.querySelectorAll(
                        'input[type="password"], input[type="email"], input[type="search"], '
                        + 'input[name="email"], input[aria-label="Search"], .email, .username, '
                        + '[data-sensitive], '
                        + 'button[aria-label="Abrir menú de usuario"] .MuiTypography-root, '
                        + 'button[aria-label="Abrir menú de usuario"] .MuiAvatar-root, '
                        + '.user .avatar'
                    )
                        .forEach(hide);
                    const walker = document.createTreeWalker(document.body, NodeFilter.SHOW_TEXT);
                    while (walker.nextNode()) {{
                        const value = walker.currentNode.nodeValue || '';
                        const hasEmail = /[A-Za-z0-9._%+-]+@[A-Za-z0-9.-]+[.][A-Za-z]{{2,}}/.test(value);
                        const hasUuid = /(?:^|[^0-9a-f])[0-9a-f]{{8}}-[0-9a-f]{{4}}-[1-5][0-9a-f]{{3}}-[89ab][0-9a-f]{{3}}-[0-9a-f]{{12}}(?=$|[^0-9a-f])/i.test(value);
                        const hasSessionMarker = /(?:sesi[oó]n)[ ]+[0-9a-f]{{8}}(?:$|[^0-9a-f])/i.test(value);
                        const hasPrivateIp = /(?:^|[^0-9])(?:10(?:[.][0-9]{{1,3}}){{3}}|192[.]168(?:[.][0-9]{{1,3}}){{2}}|172[.](?:1[6-9]|2[0-9]|3[01])(?:[.][0-9]{{1,3}}){{2}})(?=$|[^0-9])/.test(value);
                        const hasWorkerName = /(?:worker|trabajador)[ ]*[:#-][ ]*[A-Za-z0-9._-]+/i.test(value);
                        const hasShortPassword = passwordValue.length < 8
                            && containsBoundedPassword(value);
                        if (value.includes({json.dumps(email)}) || hasEmail || hasUuid || hasSessionMarker || hasPrivateIp || hasWorkerName || hasShortPassword) {{
                            hide(walker.currentNode.parentElement);
                        }}
                    }}
                    const passwordIsVisible = [...document.body.querySelectorAll('*')]
                        .filter(element => {{
                            const style = getComputedStyle(element);
                            return element.offsetParent !== null
                                && style.visibility !== 'hidden'
                                && style.display !== 'none';
                        }})
                        .flatMap(element => [...element.childNodes])
                        .filter(node => node.nodeType === Node.TEXT_NODE)
                        .some(node => {{
                            const value = (node.nodeValue || '').trim();
                            return passwordValue.length >= 8
                                ? value.includes(passwordValue)
                                : containsBoundedPassword(value);
                        }});
                    if (passwordIsVisible) {{
                        privateNodes.forEach(entry => {{
                            entry.element.style.visibility = entry.visibility;
                        }});
                        return false;
                    }}
                    const previousTitle = document.title;
                    if ({json.dumps(title_marker)} !== null) document.title = {json.dumps(title_marker)};
                    window.__robotswarmRestorePrivate = () => {{
                        privateNodes.forEach(entry => {{
                            entry.element.style.visibility = entry.visibility;
                        }});
                        document.title = previousTitle;
                        delete window.__robotswarmRestorePrivate;
                    }};
                    return true;
                }})()
            """
        )
        if safety is not True:
            raise DriverError("A credential appeared in visible text; screenshot was refused")

    def restore_after_capture(self) -> None:
        with contextlib.suppress(Exception):
            self.cdp.evaluate("window.__robotswarmRestorePrivate?.()")

    def screenshot(self, destination: Path, email: str, password: str) -> dict[str, Any]:
        page = self.current_page()
        self.prepare_safe_capture(email, password)
        try:
            captured = self.cdp.call(
                "Page.captureScreenshot",
                {"format": "png", "fromSurface": True, "captureBeyondViewport": False},
                timeout=30,
            )
            raw = base64.b64decode(captured.get("data", ""), validate=True)
            if not raw.startswith(b"\x89PNG\r\n\x1a\n") or len(raw) > 30 * 1024 * 1024:
                raise DriverError("Chrome returned an invalid screenshot")
            write_bytes_secure(destination, raw)
            return {
                "file": destination.name,
                "sha256": hashlib.sha256(raw).hexdigest(),
                "bytes": len(raw),
                "page": {"origin": page["origin"], "path": page["path"], "title": page["title"]},
            }
        finally:
            self.restore_after_capture()

    def _session_api_request(self, method: str, session_id: str | None = None) -> Any:
        """Use the browser's current login without bringing a token back into Python."""
        if method not in {"GET", "DELETE"}:
            raise DriverError("The session cleanup requested an unsupported HTTP method")
        if session_id is not None and not re.fullmatch(
            r"[0-9a-f]{8}-[0-9a-f]{4}-[1-5][0-9a-f]{3}-[89ab][0-9a-f]{3}-[0-9a-f]{12}",
            session_id,
            flags=re.I,
        ):
            raise DriverError("The session cleanup received an invalid session identifier")

        endpoint = f"https://{DEFAULT_API_HOST}/api/sessions"
        if session_id is not None:
            endpoint += f"/{session_id}"
        response = self.cdp.evaluate(
            f"""
                (async () => {{
                    const token = localStorage.getItem('jwt_access_token');
                    if (!token) return {{authenticated: false, status: 0, body: null}};
                    try {{
                        const response = await fetch({json.dumps(endpoint)}, {{
                            method: {json.dumps(method)},
                            cache: 'no-store',
                            credentials: 'omit',
                            headers: {{
                                Accept: 'application/json',
                                Authorization: `Bearer ${{token}}`,
                            }},
                        }});
                        let body = null;
                        try {{
                            body = await response.json();
                        }} catch (_) {{
                            // A 404 or an empty response body is still represented by its status.
                        }}
                        return {{authenticated: true, status: response.status, body}};
                    }} catch (_) {{
                        return {{authenticated: true, status: 0, body: null}};
                    }}
                }})()
            """,
            await_promise=True,
            timeout=30,
        )
        if not isinstance(response, dict) or response.get("authenticated") is not True:
            raise DriverError("The authenticated session inventory is unavailable")
        status = response.get("status")
        accepted = {200} if method == "GET" else {200, 404}
        if not isinstance(status, int) or status not in accepted:
            raise DriverError("The authenticated session cleanup request failed")
        return response.get("body")

    def _task_api_request(self, session_id: str) -> list[dict[str, Any]]:
        """Read task state with the browser token while keeping it inside Chrome."""
        if not re.fullmatch(
            r"[0-9a-f]{8}-[0-9a-f]{4}-[1-5][0-9a-f]{3}-[89ab][0-9a-f]{3}-[0-9a-f]{12}",
            session_id,
            flags=re.I,
        ):
            raise DriverError("The task inventory received an invalid session identifier")

        endpoint = f"https://{DEFAULT_API_HOST}/api/sessions/{session_id}/tasks"
        response = self.cdp.evaluate(
            f"""
                (async () => {{
                    const token = localStorage.getItem('jwt_access_token');
                    if (!token) return {{authenticated: false, status: 0, body: null}};
                    try {{
                        const response = await fetch({json.dumps(endpoint)}, {{
                            method: 'GET',
                            cache: 'no-store',
                            credentials: 'omit',
                            headers: {{
                                Accept: 'application/json',
                                Authorization: `Bearer ${{token}}`,
                            }},
                        }});
                        let body = null;
                        try {{
                            body = await response.json();
                        }} catch (_) {{
                            // The status below remains the authoritative failure signal.
                        }}
                        return {{authenticated: true, status: response.status, body}};
                    }} catch (_) {{
                        return {{authenticated: true, status: 0, body: null}};
                    }}
                }})()
            """,
            await_promise=True,
            timeout=30,
        )
        if not isinstance(response, dict) or response.get("authenticated") is not True:
            raise DriverError("The authenticated task inventory is unavailable")
        if response.get("status") != 200 or not isinstance(response.get("body"), list):
            raise DriverError("The authenticated task inventory request failed")

        tasks: list[dict[str, Any]] = []
        for task in response["body"]:
            if not isinstance(task, dict):
                raise DriverError("The authenticated task inventory is invalid")
            task_id = task.get("id")
            task_type = task.get("type")
            state = task.get("state")
            created_at = task.get("createdAt")
            started_at = task.get("startedAt")
            completed_at = task.get("completedAt")
            if (
                not isinstance(task_id, str)
                or not re.fullmatch(
                    r"[0-9a-f]{8}-[0-9a-f]{4}-[1-5][0-9a-f]{3}-[89ab][0-9a-f]{3}-[0-9a-f]{12}",
                    task_id,
                    flags=re.I,
                )
                or not isinstance(task_type, str)
                or not isinstance(state, str)
                or not isinstance(created_at, str)
                or (started_at is not None and not isinstance(started_at, str))
                or (completed_at is not None and not isinstance(completed_at, str))
            ):
                raise DriverError("The authenticated task inventory is invalid")
            tasks.append(
                {
                    "id": task_id,
                    "type": task_type,
                    "state": state,
                    "createdAt": created_at,
                    "startedAt": started_at,
                    "completedAt": completed_at,
                }
            )
        return tasks

    def started_task_record(self) -> dict[str, Any]:
        if not self.started_task_session_id or not self.started_task_id:
            raise DriverError("The visible run has no reconciled task to inspect")
        task = next(
            (
                item
                for item in self._task_api_request(self.started_task_session_id)
                if item["id"] == self.started_task_id
            ),
            None,
        )
        if task is None:
            raise DriverError("The reconciled task disappeared from the backend inventory")
        return {
            "type": task["type"],
            "state": task["state"],
            "createdAt": task["createdAt"],
            "startedAt": task["startedAt"],
            "completedAt": task["completedAt"],
        }

    @staticmethod
    def _occupies_account_slot(session: Any) -> bool:
        if not isinstance(session, dict):
            raise DriverError("The authenticated session inventory is invalid")
        session_id = session.get("id")
        state = session.get("state")
        if not isinstance(session_id, str) or not re.fullmatch(
            r"[0-9a-f]{8}-[0-9a-f]{4}-[1-5][0-9a-f]{3}-[89ab][0-9a-f]{3}-[0-9a-f]{12}",
            session_id,
            flags=re.I,
        ):
            raise DriverError("The authenticated session inventory has an invalid identifier")
        if state in SESSION_SLOT_STATES:
            return True
        if state in {"Failed", "Expired"}:
            return session.get("computeWorkerId") is not None
        if state != "Stopped":
            raise DriverError("The authenticated session inventory has an unknown state")
        return False

    def _occupying_sessions(self) -> list[dict[str, Any]]:
        sessions = self._session_api_request("GET")
        if not isinstance(sessions, list):
            raise DriverError("The authenticated session inventory is invalid")
        return [session for session in sessions if self._occupies_account_slot(session)]

    def _stop_session_via_api(self, session_id: str) -> None:
        self._session_api_request("DELETE", session_id)

    def stop_created_session(self, timeout: float = 180) -> dict[str, Any]:
        if not (self.created_session or self.create_requested):
            return {"requested": False, "reason": "not-created-by-driver"}
        uncertain_creation = self.create_requested and not self.created_session
        if self.has_button(STOP_BUTTON):
            self.click_button(STOP_BUTTON)
            self.wait_js(
                "Boolean(document.querySelector('[role=\"dialog\"]'))",
                10,
                "stop-session confirmation",
            )
            self.click_button("Detener y liberar")

        deadline = time.monotonic() + timeout
        stop_attempts: dict[str, float] = {}
        stop_requests = 0
        last_error: str | None = None
        while time.monotonic() < deadline:
            try:
                occupants = self._occupying_sessions()
                last_error = None
            except DriverError as exc:
                last_error = str(exc)
                time.sleep(min(1, max(0, deadline - time.monotonic())))
                continue

            if not occupants:
                self.created_session = False
                self.create_requested = False
                return {
                    "requested": True,
                    "released": True,
                    "verifiedBy": "authenticated-session-list",
                    "reconciledUncertainCreation": uncertain_creation,
                    "stopRequests": stop_requests,
                }

            now = time.monotonic()
            made_request = False
            for session in occupants:
                session_id = session["id"]
                if now - stop_attempts.get(session_id, -10) < 5:
                    continue
                stop_attempts[session_id] = now
                try:
                    self._stop_session_via_api(session_id)
                    stop_requests += 1
                    made_request = True
                except DriverError as exc:
                    last_error = str(exc)
            if not made_request:
                time.sleep(min(1, max(0, deadline - time.monotonic())))

        result = {"requested": True, "released": False, "error": "cleanup timeout"}
        if last_error:
            result["lastError"] = last_error
        return result


@dataclass
class UserRun:
    label: str
    robot_count: int
    credentials: dict[str, str]
    chrome: OwnedChrome
    ui: RobotSwarmUi | None = None
    report: dict[str, Any] = field(default_factory=dict)


def parallel(users: list[UserRun], operation: Callable[[UserRun], Any]) -> dict[str, Any]:
    results: dict[str, Any] = {}
    pool = concurrent.futures.ThreadPoolExecutor(max_workers=len(users))
    jobs = {pool.submit(operation, user): user.label for user in users}
    try:
        for job in concurrent.futures.as_completed(jobs):
            label = jobs[job]
            try:
                results[label] = job.result()
            except BaseException:
                for user in users:
                    if user.ui:
                        user.ui.stop_event.set()
                for pending in jobs:
                    pending.cancel()
                # A cancelled Future may already be running. Join every submitted
                # operation before the caller clears stop_event and starts cleanup.
                for started in jobs:
                    with contextlib.suppress(BaseException):
                        started.result()
                raise
    finally:
        pool.shutdown(wait=True)
    return results


def recorded_task_overlap(records: dict[str, dict[str, Any]]) -> dict[str, Any] | None:
    figure = records.get("A", {})
    follow = records.get("B", {})
    if figure.get("state") != "Completed" or follow.get("state") != "Running":
        return None
    figure_started_value = figure.get("startedAt") or figure.get("createdAt")
    figure_started = parse_backend_time(figure_started_value, "startedAt")
    figure_completed = parse_backend_time(figure.get("completedAt"), "completedAt")
    follow_started = parse_backend_time(follow.get("startedAt"), "startedAt")
    if figure_started is None or figure_completed is None or follow_started is None:
        return None
    overlap_started = max(figure_started, follow_started)
    if figure_completed <= overlap_started:
        return None
    return {
        "intervalsOverlap": True,
        "overlapMilliseconds": round((figure_completed - overlap_started).total_seconds() * 1000),
        "figureStartedAt": figure_started_value,
        "figureStartBoundary": "startedAt" if figure.get("startedAt") else "createdAt-legacy-fallback",
        "figureCompletedAt": figure.get("completedAt"),
        "followStartedAt": follow.get("startedAt"),
    }


def observe_same_round_running(users: list[UserRun], timeout: float = 60) -> dict[str, Any]:
    """Prove overlap by reading both task panels concurrently in one polling round."""
    deadline = time.monotonic() + timeout
    record_settle_deadline: float | None = None
    round_number = 0
    last: dict[str, Any] = {}
    while time.monotonic() < deadline:
        round_number += 1
        for user in users:
            assert user.ui
            user.ui.raise_if_interrupted()

        def sample(user: UserRun) -> dict[str, Any]:
            if not user.ui:
                raise DriverError(f"User {user.label} has no UI for concurrency sampling")
            started = time.monotonic()
            status = user.ui.task_status()
            ended = time.monotonic()
            return {
                "status": status,
                "sampledAt": utc_now(),
                "_sampleMidpoint": (started + ended) / 2,
            }

        last = parallel(users, sample)
        states = {label: result["status"].get("state") for label, result in last.items()}
        if states == {"A": "Running", "B": "Running"}:
            midpoints = [float(result.pop("_sampleMidpoint")) for result in last.values()]
            return {
                "provedBy": "both task panels sampled Running in the same polling round",
                "sameRound": True,
                "round": round_number,
                "sampleSkewMs": round((max(midpoints) - min(midpoints)) * 1000, 3),
                "users": last,
            }
        if states.get("A") in {"Completed", "Failed", "Cancelled"}:
            if states.get("A") == "Completed":
                if record_settle_deadline is None:
                    record_settle_deadline = min(
                        deadline,
                        time.monotonic() + FAST_FIGURE_RECORD_SETTLE_SECONDS,
                    )
                records = parallel(
                    users,
                    lambda user: user.ui.started_task_record(),  # type: ignore[union-attr]
                )
                interval = recorded_task_overlap(records)
                if interval is not None:
                    for result in last.values():
                        result.pop("_sampleMidpoint", None)
                    return {
                        "provedBy": "persisted backend task intervals after the fast Figure completed",
                        "sameRound": False,
                        "intervalOverlap": interval,
                        "users": last,
                        "backendTasks": records,
                    }
                # SignalR can paint the terminal Figure just before the next
                # authenticated task-list read reflects both timestamps. Give
                # that durable evidence a short settling window, but never infer
                # overlap from the UI state alone.
                if (
                    states.get("B") == "Running"
                    and time.monotonic() < record_settle_deadline
                ):
                    time.sleep(0.2)
                    continue
            raise DriverError(
                f"Could not prove concurrent Running states: A reached {states.get('A')} before a shared Running round"
            )
        if states.get("B") in {"Completed", "Failed", "Cancelled"}:
            raise DriverError(
                f"Could not prove concurrent Running states: B reached {states.get('B')} before a shared Running round"
            )
        time.sleep(0.2)
    states = {label: result.get("status", {}).get("state") for label, result in last.items()}
    raise DriverError(f"Timed out proving a same-round Running overlap; last states were {states}")


def result_exit_code(failure: BaseException | None, cleanup_passed: bool) -> int:
    if isinstance(failure, KeyboardInterrupt):
        return 130
    if failure is not None:
        return 1
    return 0 if cleanup_passed else 3


def powershell_quote(value: str) -> str:
    return "'" + value.replace("'", "''") + "'"


INTERACTIVE_DESKTOP_PATH = r"WinSta0\Default"
WINDOWS_INTERACTIVE_DESKTOP_GUARD = r"""
public static class RobotSwarmInteractiveDesktop {
    private const uint DESKTOP_READOBJECTS = 0x0001;
    private const int UOI_NAME = 2;
    private const int UOI_IO = 6;
    private const int WTS_CONNECT_STATE = 8;
    private const int WTS_ACTIVE = 0;
    private const uint INVALID_SESSION_ID = 0xffffffff;

    [DllImport("kernel32.dll")]
    private static extern uint GetCurrentProcessId();
    [DllImport("kernel32.dll")]
    private static extern uint GetCurrentThreadId();
    [DllImport("kernel32.dll", SetLastError=true)]
    [return: MarshalAs(UnmanagedType.Bool)]
    private static extern bool ProcessIdToSessionId(uint processId, out uint sessionId);
    [DllImport("kernel32.dll")]
    private static extern uint WTSGetActiveConsoleSessionId();
    [DllImport("user32.dll", SetLastError=true)]
    private static extern IntPtr GetProcessWindowStation();
    [DllImport("user32.dll", SetLastError=true)]
    private static extern IntPtr GetThreadDesktop(uint threadId);
    [DllImport("user32.dll", SetLastError=true)]
    private static extern IntPtr OpenInputDesktop(
        uint flags, [MarshalAs(UnmanagedType.Bool)] bool inherit, uint desiredAccess);
    [DllImport("user32.dll", SetLastError=true)]
    [return: MarshalAs(UnmanagedType.Bool)]
    private static extern bool CloseDesktop(IntPtr desktop);
    [DllImport("user32.dll", EntryPoint="GetUserObjectInformationW",
        CharSet=CharSet.Unicode, SetLastError=true)]
    [return: MarshalAs(UnmanagedType.Bool)]
    private static extern bool GetUserObjectName(
        IntPtr handle, int index, StringBuilder value, uint length, out uint needed);
    [DllImport("user32.dll", EntryPoint="GetUserObjectInformationW", SetLastError=true)]
    [return: MarshalAs(UnmanagedType.Bool)]
    private static extern bool GetUserObjectIo(
        IntPtr handle, int index, out int value, uint length, out uint needed);
    [DllImport("wtsapi32.dll", EntryPoint="WTSQuerySessionInformationW",
        CharSet=CharSet.Unicode, SetLastError=true)]
    [return: MarshalAs(UnmanagedType.Bool)]
    private static extern bool WTSQuerySessionInformation(
        IntPtr server, uint sessionId, int informationClass,
        out IntPtr buffer, out uint bytesReturned);
    [DllImport("wtsapi32.dll")]
    private static extern void WTSFreeMemory(IntPtr memory);

    private static InvalidOperationException Failure(string checkpoint, string reason) {
        return new InvalidOperationException(
            "Interactive desktop check failed " + checkpoint + ": " + reason);
    }

    private static string ObjectName(IntPtr handle, string checkpoint, string kind) {
        StringBuilder value = new StringBuilder(128);
        uint needed;
        if (!GetUserObjectName(
            handle, UOI_NAME, value, (uint)(value.Capacity * sizeof(char)), out needed)) {
            int error = Marshal.GetLastWin32Error();
            throw Failure(checkpoint, "could not read " + kind + " name (Win32 " + error + ")");
        }
        return value.ToString();
    }

    private static void RequireInputObject(
        IntPtr handle, string checkpoint, string kind) {
        int receivesInput;
        uint needed;
        if (!GetUserObjectIo(handle, UOI_IO, out receivesInput, sizeof(int), out needed)) {
            int error = Marshal.GetLastWin32Error();
            throw Failure(checkpoint, kind + " input state is unavailable (Win32 " + error + ")");
        }
        if (receivesInput == 0) {
            throw Failure(checkpoint, kind + " is not receiving user input");
        }
    }

    private static void RequireActiveConsoleSession(string checkpoint) {
        uint sessionId;
        if (!ProcessIdToSessionId(GetCurrentProcessId(), out sessionId)) {
            int error = Marshal.GetLastWin32Error();
            throw Failure(checkpoint, "process session is unavailable (Win32 " + error + ")");
        }
        uint consoleSessionId = WTSGetActiveConsoleSessionId();
        if (consoleSessionId == INVALID_SESSION_ID || consoleSessionId != sessionId) {
            throw Failure(checkpoint, "process session is not the active console session");
        }

        IntPtr buffer;
        uint bytesReturned;
        if (!WTSQuerySessionInformation(
            IntPtr.Zero, sessionId, WTS_CONNECT_STATE, out buffer, out bytesReturned)) {
            int error = Marshal.GetLastWin32Error();
            throw Failure(checkpoint, "session state is unavailable (Win32 " + error + ")");
        }
        try {
            if (buffer == IntPtr.Zero || bytesReturned < sizeof(int)) {
                throw Failure(checkpoint, "session state response is invalid");
            }
            if (Marshal.ReadInt32(buffer) != WTS_ACTIVE) {
                throw Failure(checkpoint, "console session is not active");
            }
        } finally {
            if (buffer != IntPtr.Zero) {
                WTSFreeMemory(buffer);
            }
        }
    }

    public static string RequireDefaultInputDesktop(string checkpoint) {
        RequireActiveConsoleSession(checkpoint);
        IntPtr station = GetProcessWindowStation();
        if (station == IntPtr.Zero) {
            int error = Marshal.GetLastWin32Error();
            throw Failure(checkpoint, "window station is unavailable (Win32 " + error + ")");
        }
        if (!String.Equals(
            ObjectName(station, checkpoint, "window station"),
            "WinSta0",
            StringComparison.OrdinalIgnoreCase)) {
            throw Failure(checkpoint, "window station is not interactive");
        }

        IntPtr threadDesktop = GetThreadDesktop(GetCurrentThreadId());
        if (threadDesktop == IntPtr.Zero) {
            int error = Marshal.GetLastWin32Error();
            throw Failure(checkpoint, "thread desktop is unavailable (Win32 " + error + ")");
        }
        if (!String.Equals(
            ObjectName(threadDesktop, checkpoint, "thread desktop"),
            "Default",
            StringComparison.OrdinalIgnoreCase)) {
            throw Failure(checkpoint, "thread desktop is not Default");
        }
        RequireInputObject(threadDesktop, checkpoint, "thread desktop");

        IntPtr desktop = OpenInputDesktop(0, false, DESKTOP_READOBJECTS);
        if (desktop == IntPtr.Zero) {
            int error = Marshal.GetLastWin32Error();
            throw Failure(checkpoint, "input desktop is unavailable (Win32 " + error + ")");
        }
        try {
            string name = ObjectName(desktop, checkpoint, "input desktop");
            if (!String.Equals(name, "Default", StringComparison.OrdinalIgnoreCase)) {
                throw Failure(checkpoint, "input desktop is not Default");
            }
            RequireInputObject(desktop, checkpoint, "input desktop");
        } finally {
            if (!CloseDesktop(desktop)) {
                int error = Marshal.GetLastWin32Error();
                throw Failure(checkpoint, "input desktop handle did not close (Win32 " + error + ")");
            }
        }
        return "WinSta0\\Default";
    }
}
"""


def windows_capture_failure(
    result: subprocess.CompletedProcess[str],
    fallback: str,
) -> DriverError:
    diagnostic = re.sub(r"\s+", " ", (result.stderr or result.stdout or "")).strip()
    if diagnostic:
        diagnostic = diagnostic[-400:]
        return DriverError(f"{fallback} (exit {result.returncode}: {diagnostic})")
    return DriverError(f"{fallback} (exit {result.returncode})")


def windows_capture_evidence(result: subprocess.CompletedProcess[str]) -> dict[str, Any]:
    """Accept only bounded geometry proven on Default before and after capture."""
    try:
        line = next(item for item in reversed(result.stdout.splitlines()) if item.strip())
        payload = json.loads(line)
        if not isinstance(payload, dict):
            raise ValueError
        before = payload["interactiveDesktopBefore"]
        after = payload["interactiveDesktopAfter"]
        bounds = {name: int(payload[name]) for name in ("x", "y", "width", "height")}
    except (KeyError, StopIteration, TypeError, ValueError, json.JSONDecodeError) as exc:
        raise DriverError("Windows capture returned invalid interactive-desktop evidence") from exc
    if before != INTERACTIVE_DESKTOP_PATH or after != INTERACTIVE_DESKTOP_PATH:
        raise DriverError("Windows capture did not remain on the unlocked Default desktop")
    if bounds["width"] <= 0 or bounds["height"] <= 0:
        raise DriverError("Windows capture returned invalid screen geometry")
    return {
        "bounds": bounds,
        "interactiveDesktop": {"before": before, "after": after},
    }


def png_details(raw: bytes, filename: str) -> dict[str, Any]:
    if not raw.startswith(b"\x89PNG\r\n\x1a\n") or len(raw) > 60 * 1024 * 1024:
        raise DriverError("Desktop capture returned an invalid PNG")
    width, height = struct.unpack(">II", raw[16:24])
    return {
        "file": filename,
        "sha256": hashlib.sha256(raw).hexdigest(),
        "bytes": len(raw),
        "width": width,
        "height": height,
    }


def require_live_video(
    metrics: dict[str, Any],
    requested_seconds: float,
    label: str,
    minimum_fps: float,
    maximum_dropped_ratio: float,
) -> None:
    if not metrics.get("requestVideoFrameCallbackSupported"):
        raise DriverError(f"{label} does not support requestVideoFrameCallback")
    if int(metrics.get("callbackFrames") or 0) <= 0 or float(metrics.get("decodedFps") or 0) <= 0:
        raise DriverError(f"{label} did not decode any video frames")
    if float(metrics.get("mediaTimeAdvancedSeconds") or 0) < requested_seconds * 0.7:
        raise DriverError(f"{label} video did not keep advancing during the measurement")
    if float(metrics.get("decodedFps") or 0) < minimum_fps:
        raise DriverError(f"{label} decoded below {minimum_fps:.1f} FPS")
    if float(metrics.get("callbackFps") or 0) < minimum_fps:
        raise DriverError(f"{label} presented below {minimum_fps:.1f} FPS")
    dropped_ratio = metrics.get("droppedRatio")
    if dropped_ratio is not None and float(dropped_ratio) > maximum_dropped_ratio:
        raise DriverError(f"{label} dropped too many frames ({float(dropped_ratio):.1%})")


def desktop_screenshot(
    users: list[UserRun],
    destination: Path,
    run_id: str,
    _profile_root: Path,
    *,
    mode: str,
) -> dict[str, Any]:
    """Tile the owned windows and validate the state that the image must prove."""
    if len(users) != 2 or any(not user.ui for user in users):
        raise DriverError("Two connected owned browsers are required for a desktop capture")
    if mode not in {"viewers", "tasks", "post_stop"}:
        raise DriverError("The desktop capture mode is invalid")
    markers = {user.label: f"RobotSwarm-E2E-{run_id}-{user.label}" for user in users}
    prepared: list[UserRun] = []
    validate_secure_directory(destination.parent)
    descriptor, temporary_name = tempfile.mkstemp(
        prefix=f".{destination.name}.", suffix=".windows.png", dir=str(destination.parent)
    )
    os.fchmod(descriptor, 0o600)
    os.close(descriptor)
    temporary = Path(temporary_name)
    try:
        for user in users:
            assert user.ui
            user.ui.prepare_safe_capture(
                user.credentials["email"],
                user.credentials["password"],
                markers[user.label],
            )
            prepared.append(user)
        time.sleep(0.5)

        position_script = r"""
$ErrorActionPreference = 'Stop'
$markerA = @@MARKER_A@@
$markerB = @@MARKER_B@@
Add-Type -AssemblyName System.Windows.Forms
Add-Type @'
using System;
using System.Runtime.InteropServices;
public static class RobotSwarmDesktopPosition {
    [DllImport("user32.dll")]
    public static extern bool SetProcessDPIAware();
    [DllImport("user32.dll", SetLastError=true)]
    public static extern bool SetWindowPos(
        IntPtr hWnd, IntPtr hWndInsertAfter, int x, int y, int width, int height, uint flags);
}
'@
[RobotSwarmDesktopPosition]::SetProcessDPIAware() | Out-Null
$windows = @(Get-Process chrome -ErrorAction SilentlyContinue | Where-Object {
    $_.MainWindowHandle -ne 0 -and ($_.MainWindowTitle.Contains($markerA) -or $_.MainWindowTitle.Contains($markerB))
})
$windowA = @($windows | Where-Object { $_.MainWindowTitle.Contains($markerA) })
$windowB = @($windows | Where-Object { $_.MainWindowTitle.Contains($markerB) })
if ($windowA.Count -ne 1 -or $windowB.Count -ne 1) {
    throw "Could not identify exactly two owned Chrome windows"
}
$area = [System.Windows.Forms.Screen]::PrimaryScreen.WorkingArea
$leftWidth = [Math]::Floor($area.Width / 2)
$rightWidth = $area.Width - $leftWidth
$notTopMost = [IntPtr](-2)
$show = [uint32]0x0040
if (-not [RobotSwarmDesktopPosition]::SetWindowPos(
    $windowA[0].MainWindowHandle, $notTopMost, $area.X, $area.Y, $leftWidth, $area.Height, $show)) {
    throw "Could not position Chrome A"
}
if (-not [RobotSwarmDesktopPosition]::SetWindowPos(
    $windowB[0].MainWindowHandle, $notTopMost, $area.X + $leftWidth, $area.Y, $rightWidth, $area.Height, $show)) {
    throw "Could not position Chrome B"
}
"positioned"
"""
        position_script = (
            position_script.replace("@@MARKER_A@@", powershell_quote(markers["A"]))
            .replace("@@MARKER_B@@", powershell_quote(markers["B"]))
        )
        position_result = run_interruptible_process(
            [str(POWERSHELL), "-NoLogo", "-NoProfile", "-NonInteractive", "-Command", position_script],
            timeout=30,
            stop_event=users[0].ui.stop_event,
        )
        if position_result.returncode != 0:
            raise DriverError("The owned Chrome windows could not be tiled")
        time.sleep(0.9)
        capture_validation: dict[str, Any] = {}
        if mode == "viewers":
            for user in users:
                assert user.ui
                state = user.ui.require_interactive_hls()
                rect = user.ui.video_content_rect()
                capture_validation[user.label] = {
                    "live": state.get("status") == "En vivo",
                    "visibleFraction": rect["visibleFraction"],
                    "decodedFpsLabel": state.get("fps"),
                }
        elif mode == "tasks":
            for user in users:
                assert user.ui
                panel = user.ui.show_task_panel()
                task = user.ui.task_status()
                if task.get("state") not in {"Running", "Completed"}:
                    raise DriverError(f"User {user.label} had no capturable task state")
                capture_validation[user.label] = {"panel": panel, "task": task}
        else:
            by_label = {user.label: user for user in users}
            user_a = by_label.get("A")
            user_b = by_label.get("B")
            if not user_a or not user_b or not user_a.ui or not user_b.ui:
                raise DriverError("Post-stop capture requires users A and B")
            released = user_a.ui.show_released_session()
            live = user_b.ui.require_interactive_hls()
            video = user_b.ui.video_content_rect()
            panel = user_b.ui.show_task_panel()
            task = user_b.ui.task_status()
            if task.get("state") != "Running":
                raise DriverError("User B was not Running during the post-stop capture")
            capture_validation = {
                "A": {"releasedSession": released},
                "B": {
                    "live": live.get("status") == "En vivo",
                    "decodedFpsLabel": live.get("fps"),
                    "videoVisibleBeforeTaskScroll": video["visibleFraction"],
                    "panel": panel,
                    "task": task,
                },
            }

        script = r"""
$ErrorActionPreference = 'Stop'
$markerA = @@MARKER_A@@
$markerB = @@MARKER_B@@
$destination = @@DESTINATION@@
Add-Type -AssemblyName System.Windows.Forms
Add-Type -AssemblyName System.Drawing
Add-Type @'
using System;
using System.Runtime.InteropServices;
using System.Text;
@@INTERACTIVE_DESKTOP_GUARD@@
public static class RobotSwarmDesktopCapture {
    [DllImport("user32.dll")]
    public static extern bool SetProcessDPIAware();
    [DllImport("user32.dll", SetLastError=true)]
    public static extern bool SetWindowPos(
        IntPtr hWnd, IntPtr hWndInsertAfter, int x, int y, int width, int height, uint flags);
}
'@
[RobotSwarmDesktopCapture]::SetProcessDPIAware() | Out-Null
$windows = @(Get-Process chrome -ErrorAction SilentlyContinue | Where-Object {
    $_.MainWindowHandle -ne 0 -and ($_.MainWindowTitle.Contains($markerA) -or $_.MainWindowTitle.Contains($markerB))
})
$windowA = @($windows | Where-Object { $_.MainWindowTitle.Contains($markerA) })
$windowB = @($windows | Where-Object { $_.MainWindowTitle.Contains($markerB) })
if ($windowA.Count -ne 1 -or $windowB.Count -ne 1) {
    throw "Could not identify exactly two owned Chrome windows"
}
$area = [System.Windows.Forms.Screen]::PrimaryScreen.WorkingArea
$leftWidth = [Math]::Floor($area.Width / 2)
$rightWidth = $area.Width - $leftWidth
$topMost = [IntPtr](-1)
$notTopMost = [IntPtr](-2)
$show = [uint32]0x0040
try {
    if (-not [RobotSwarmDesktopCapture]::SetWindowPos(
        $windowA[0].MainWindowHandle, $topMost, $area.X, $area.Y, $leftWidth, $area.Height, $show)) {
        throw "Could not position Chrome A"
    }
    if (-not [RobotSwarmDesktopCapture]::SetWindowPos(
        $windowB[0].MainWindowHandle, $topMost, $area.X + $leftWidth, $area.Y, $rightWidth, $area.Height, $show)) {
        throw "Could not position Chrome B"
    }
    Start-Sleep -Milliseconds 900
    $desktopBefore = [RobotSwarmInteractiveDesktop]::RequireDefaultInputDesktop(
        "before CopyFromScreen")
    $bitmap = New-Object System.Drawing.Bitmap($area.Width, $area.Height)
    $graphics = [System.Drawing.Graphics]::FromImage($bitmap)
    try {
        $graphics.CopyFromScreen($area.X, $area.Y, 0, 0, $bitmap.Size)
        $desktopAfter = [RobotSwarmInteractiveDesktop]::RequireDefaultInputDesktop(
            "after CopyFromScreen")
        $stream = [System.IO.File]::Open(
            $destination,
            [System.IO.FileMode]::Create,
            [System.IO.FileAccess]::Write,
            [System.IO.FileShare]::None)
        try {
            $bitmap.Save($stream, [System.Drawing.Imaging.ImageFormat]::Png)
        } finally {
            $stream.Dispose()
        }
    } finally {
        $graphics.Dispose()
        $bitmap.Dispose()
    }
} finally {
    [RobotSwarmDesktopCapture]::SetWindowPos(
        $windowA[0].MainWindowHandle, $notTopMost, $area.X, $area.Y, $leftWidth, $area.Height, $show) | Out-Null
    [RobotSwarmDesktopCapture]::SetWindowPos(
        $windowB[0].MainWindowHandle, $notTopMost, $area.X + $leftWidth, $area.Y, $rightWidth, $area.Height, $show) | Out-Null
}
@{
    x=$area.X
    y=$area.Y
    width=$area.Width
    height=$area.Height
    interactiveDesktopBefore=$desktopBefore
    interactiveDesktopAfter=$desktopAfter
} | ConvertTo-Json -Compress
"""
        script = (
            script.replace("@@INTERACTIVE_DESKTOP_GUARD@@", WINDOWS_INTERACTIVE_DESKTOP_GUARD)
            .replace("@@MARKER_A@@", powershell_quote(markers["A"]))
            .replace("@@MARKER_B@@", powershell_quote(markers["B"]))
            .replace("@@DESTINATION@@", powershell_quote(windows_path(temporary)))
        )
        result = run_interruptible_process(
            [str(POWERSHELL), "-NoLogo", "-NoProfile", "-NonInteractive", "-Command", script],
            timeout=45,
            stop_event=users[0].ui.stop_event,
        )
        if result.returncode != 0:
            raise windows_capture_failure(result, "The owned-window desktop capture failed")
        if not temporary.is_file():
            raise DriverError("PowerShell did not create the desktop screenshot")
        evidence = windows_capture_evidence(result)
        raw = temporary.read_bytes()
        write_bytes_secure(destination, raw)
        details = png_details(raw, destination.name)
        details["desktopBounds"] = evidence["bounds"]
        details["interactiveDesktop"] = evidence["interactiveDesktop"]
        details["ownedWindowsSelected"] = ["A", "B"]
        details["captureMode"] = mode
        details["validationAfterResize"] = capture_validation
        details["humanReviewRequired"] = True
        return details
    finally:
        with contextlib.suppress(OSError):
            temporary.unlink()
        for user in reversed(prepared):
            assert user.ui
            user.ui.restore_after_capture()


def owned_window_screenshot(
    ui: RobotSwarmUi,
    destination: Path,
    email: str,
    password: str,
    title_marker: str,
) -> dict[str, Any]:
    """Capture the exact owned Chrome window while the viewer is fullscreen."""
    validate_secure_directory(destination.parent)
    descriptor, temporary_name = tempfile.mkstemp(
        prefix=f".{destination.name}.", suffix=".windows.png", dir=str(destination.parent)
    )
    os.fchmod(descriptor, 0o600)
    os.close(descriptor)
    temporary = Path(temporary_name)
    ui.prepare_safe_capture(email, password, title_marker)
    try:
        script = r"""
$ErrorActionPreference = 'Stop'
$marker = @@MARKER@@
$destination = @@DESTINATION@@
Add-Type -AssemblyName System.Drawing
Add-Type @'
using System;
using System.Runtime.InteropServices;
using System.Text;
@@INTERACTIVE_DESKTOP_GUARD@@
public static class RobotSwarmOwnedWindowCapture {
    [StructLayout(LayoutKind.Sequential)]
    public struct RECT { public int Left; public int Top; public int Right; public int Bottom; }
    [DllImport("user32.dll")]
    public static extern bool SetProcessDPIAware();
    [DllImport("user32.dll")]
    public static extern bool GetWindowRect(IntPtr hWnd, out RECT rect);
    [DllImport("user32.dll", SetLastError=true)]
    public static extern bool SetWindowPos(
        IntPtr hWnd, IntPtr hWndInsertAfter, int x, int y, int width, int height, uint flags);
}
'@
[RobotSwarmOwnedWindowCapture]::SetProcessDPIAware() | Out-Null
$deadline = [DateTime]::UtcNow.AddSeconds(5)
do {
    $windows = @(Get-Process chrome -ErrorAction SilentlyContinue | Where-Object {
        $_.MainWindowHandle -ne 0 -and $_.MainWindowTitle.Contains($marker)
    })
    if ($windows.Count -eq 1) { break }
    Start-Sleep -Milliseconds 100
} while ([DateTime]::UtcNow -lt $deadline)
if ($windows.Count -ne 1) { throw "Could not identify the owned fullscreen Chrome window" }
$window = $windows[0]
$topMost = [IntPtr](-1)
$notTopMost = [IntPtr](-2)
$flags = [uint32]0x0043
try {
    if (-not [RobotSwarmOwnedWindowCapture]::SetWindowPos(
        $window.MainWindowHandle, $topMost, 0, 0, 0, 0, $flags)) {
        throw "Could not foreground the owned fullscreen window"
    }
    Start-Sleep -Milliseconds 500
    $rect = New-Object RobotSwarmOwnedWindowCapture+RECT
    if (-not [RobotSwarmOwnedWindowCapture]::GetWindowRect($window.MainWindowHandle, [ref]$rect)) {
        throw "Could not read the owned window bounds"
    }
    $width = $rect.Right - $rect.Left
    $height = $rect.Bottom - $rect.Top
    if ($width -lt 320 -or $height -lt 240) { throw "Owned window bounds are invalid" }
    $desktopBefore = [RobotSwarmInteractiveDesktop]::RequireDefaultInputDesktop(
        "before CopyFromScreen")
    $bitmap = New-Object System.Drawing.Bitmap($width, $height)
    $graphics = [System.Drawing.Graphics]::FromImage($bitmap)
    try {
        $graphics.CopyFromScreen($rect.Left, $rect.Top, 0, 0, $bitmap.Size)
        $desktopAfter = [RobotSwarmInteractiveDesktop]::RequireDefaultInputDesktop(
            "after CopyFromScreen")
        $stream = [System.IO.File]::Open(
            $destination,
            [System.IO.FileMode]::Create,
            [System.IO.FileAccess]::Write,
            [System.IO.FileShare]::None)
        try {
            $bitmap.Save($stream, [System.Drawing.Imaging.ImageFormat]::Png)
        } finally {
            $stream.Dispose()
        }
    } finally {
        $graphics.Dispose()
        $bitmap.Dispose()
    }
    @{
        x=$rect.Left
        y=$rect.Top
        width=$width
        height=$height
        interactiveDesktopBefore=$desktopBefore
        interactiveDesktopAfter=$desktopAfter
    } | ConvertTo-Json -Compress
} finally {
    [RobotSwarmOwnedWindowCapture]::SetWindowPos(
        $window.MainWindowHandle, $notTopMost, 0, 0, 0, 0, $flags) | Out-Null
}
"""
        script = (
            script.replace("@@INTERACTIVE_DESKTOP_GUARD@@", WINDOWS_INTERACTIVE_DESKTOP_GUARD)
            .replace("@@MARKER@@", powershell_quote(title_marker))
            .replace("@@DESTINATION@@", powershell_quote(windows_path(temporary)))
        )
        result = run_interruptible_process(
            [str(POWERSHELL), "-NoLogo", "-NoProfile", "-NonInteractive", "-Command", script],
            timeout=45,
            stop_event=ui.stop_event,
        )
        if result.returncode != 0:
            raise windows_capture_failure(
                result,
                "The owned fullscreen Windows capture failed",
            )
        if not temporary.is_file():
            raise DriverError("The owned fullscreen Windows capture produced no PNG")
        evidence = windows_capture_evidence(result)
        raw = temporary.read_bytes()
        write_bytes_secure(destination, raw)
        details = png_details(raw, destination.name)
        details["windowBounds"] = evidence["bounds"]
        details["interactiveDesktop"] = evidence["interactiveDesktop"]
        details["ownedWindowSelected"] = ui.chrome.label
        details["humanReviewRequired"] = True
        return details
    finally:
        with contextlib.suppress(OSError):
            temporary.unlink()
        ui.restore_after_capture()


def validate_secure_directory(path: Path) -> None:
    if path.exists() or path.is_symlink():
        details = path.lstat()
        if stat.S_ISLNK(details.st_mode) or not stat.S_ISDIR(details.st_mode):
            raise DriverError("The output path must be a real directory")
        if details.st_uid != os.getuid():
            raise DriverError("The output directory has a different owner")
        if stat.S_IMODE(details.st_mode) != 0o700:
            raise DriverError("The output directory must have mode 0700")
        return
    path.mkdir(parents=True, mode=0o700)
    details = path.lstat()
    if details.st_uid != os.getuid() or stat.S_IMODE(details.st_mode) != 0o700:
        raise DriverError("The output directory could not be created with mode 0700")


def write_bytes_secure(path: Path, value: bytes) -> None:
    validate_secure_directory(path.parent)
    descriptor, temporary_name = tempfile.mkstemp(prefix=f".{path.name}.", dir=str(path.parent))
    try:
        os.fchmod(descriptor, 0o600)
        with os.fdopen(descriptor, "wb") as stream:
            stream.write(value)
        os.replace(temporary_name, path)
    except Exception:
        with contextlib.suppress(OSError):
            os.close(descriptor)
        with contextlib.suppress(OSError):
            os.unlink(temporary_name)
        raise


def write_json_secure(path: Path, value: Any) -> None:
    validate_secure_directory(path.parent)
    descriptor, temporary_name = tempfile.mkstemp(prefix=f".{path.name}.", dir=str(path.parent))
    try:
        os.fchmod(descriptor, 0o600)
        with os.fdopen(descriptor, "w", encoding="utf-8") as stream:
            json.dump(value, stream, ensure_ascii=False, indent=2, sort_keys=True)
            stream.write("\n")
        os.replace(temporary_name, path)
    except Exception:
        with contextlib.suppress(OSError):
            os.close(descriptor)
        with contextlib.suppress(OSError):
            os.unlink(temporary_name)
        raise


def validate_site(url: str) -> str:
    parsed = urllib.parse.urlsplit(url)
    if parsed.scheme != "https" or not parsed.hostname or parsed.username or parsed.password:
        raise DriverError("The test URL must be an HTTPS origin without embedded credentials")
    if parsed.query or parsed.fragment:
        raise DriverError("The test URL must not contain a query string or fragment")
    origin = f"{parsed.scheme}://{parsed.netloc}"
    if origin != PRODUCTION_ORIGIN:
        raise DriverError("The production credentials may only be sent to https://rs.zerav.la")
    return origin


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Run the visible two-user RobotSwarm acceptance check")
    parser.add_argument("--url", default=DEFAULT_URL)
    parser.add_argument("--credentials", type=Path, default=DEFAULT_CREDENTIALS)
    parser.add_argument("--binding-key", type=Path, default=DEFAULT_BINDING_KEY)
    parser.add_argument("--api-report", type=Path, required=True)
    parser.add_argument("--deployment-commit", required=True)
    parser.add_argument("--chrome", type=Path, default=DEFAULT_CHROME)
    parser.add_argument(
        "--profile-root",
        type=Path,
        required=True,
        help="directorio temporal de Windows para los dos perfiles efimeros",
    )
    parser.add_argument("--port-a", type=int, default=DEFAULT_PORTS["A"])
    parser.add_argument("--port-b", type=int, default=DEFAULT_PORTS["B"])
    parser.add_argument("--robots-a", type=int, default=3)
    parser.add_argument("--robots-b", type=int, default=7)
    parser.add_argument("--ready-timeout", type=float, default=300)
    parser.add_argument("--viewer-timeout", type=float, default=240)
    parser.add_argument("--task-timeout", type=float, default=600)
    parser.add_argument("--video-seconds", type=float, default=10)
    parser.add_argument("--min-video-fps", type=float, default=24)
    parser.add_argument("--max-dropped-ratio", type=float, default=0.10)
    parser.add_argument("--max-capture-skew-ms", type=float, default=500)
    parser.add_argument("--hold-seconds", type=float, default=0)
    parser.add_argument("--leave-sessions", action="store_true")
    parser.add_argument("--output-dir", type=Path, default=Path("/tmp/robotswarm-visible-e2e-output"))
    parser.add_argument("--output-json", type=Path)
    return parser


def main() -> int:
    args = build_parser().parse_args()
    if args.port_a == args.port_b or not all(1024 <= port <= 65535 for port in (args.port_a, args.port_b)):
        raise DriverError("Two distinct non-privileged CDP ports are required")
    if not all(1 <= count <= 10 for count in (args.robots_a, args.robots_b)):
        raise DriverError("Robot counts must be between 1 and 10")
    if args.robots_a == args.robots_b:
        raise DriverError("The two visual-isolation rosters must have different robot counts")
    if not 2 <= args.video_seconds <= 120 or not 0 <= args.hold_seconds <= 600:
        raise DriverError("Video/hold durations are outside the safe range")
    if not 1 <= args.min_video_fps <= 60 or not 0 <= args.max_dropped_ratio <= 1:
        raise DriverError("Video quality thresholds are outside the safe range")
    if not 10 <= args.max_capture_skew_ms <= 5000:
        raise DriverError("The scene-capture skew threshold is outside the safe range")
    if not 30 <= args.task_timeout <= 1800:
        raise DriverError("Task timeout must be between 30 and 1800 seconds")
    if not re.fullmatch(r"[0-9a-f]{40}", args.deployment_commit):
        raise DriverError("The deployment commit must be a full lowercase Git SHA")
    if not args.chrome.is_file():
        raise DriverError(f"Chrome executable was not found: {args.chrome}")
    if not POWERSHELL.is_file():
        raise DriverError(f"Windows PowerShell was not found: {POWERSHELL}")
    expected_origin = validate_site(args.url)

    credentials = read_credentials(args.credentials)
    binding_key = load_binding_key(args.binding_key)
    fingerprints = {
        label: account_fingerprint(credential, binding_key)
        for label, credential in credentials.items()
    }
    del binding_key
    api_acceptance = load_api_acceptance(
        args.api_report,
        args.deployment_commit,
        fingerprints,
    )
    if api_acceptance["robotCounts"] != {"A": args.robots_a, "B": args.robots_b}:
        raise DriverError("The API and visual acceptance runs use different robot counts")
    sanitizer = Sanitizer(
        [
            credentials["A"]["email"],
            credentials["A"]["password"],
            credentials["B"]["email"],
            credentials["B"]["password"],
        ]
    )
    run_id = dt.datetime.now(dt.timezone.utc).strftime("%Y%m%dT%H%M%SZ") + f"-{os.getpid()}"
    output_json = args.output_json or args.output_dir / f"visible-e2e-{run_id}.json"
    validate_secure_directory(args.output_dir)
    validate_secure_directory(output_json.parent)
    report: dict[str, Any] = {
        "schemaVersion": 1,
        "runId": run_id,
        "startedAt": utc_now(),
        "site": clean_url(args.url),
        "rendering": {
            "visible": True,
            "headless": False,
            "gpuDisabled": False,
            "minimumDecodedFps": args.min_video_fps,
            "maximumDroppedRatio": args.max_dropped_ratio,
        },
        "users": {},
        "apiAcceptance": api_acceptance,
        "cleanup": {},
        "success": False,
    }

    users: list[UserRun] = []
    for label, port, robot_count in (
        ("A", args.port_a, args.robots_a),
        ("B", args.port_b, args.robots_b),
    ):
        profile = args.profile_root / f"robotswarm-visible-e2e-{run_id}-{label.lower()}"
        chrome = OwnedChrome(label, port, profile, run_id, args.chrome, args.url)
        users.append(UserRun(label, robot_count, credentials[label], chrome))
        report["users"][label] = {"requestedRobots": robot_count, "cdpPort": port}

    interrupted = False
    stop_event = threading.Event()

    def note_signal(_number: int, _frame: Any) -> None:
        nonlocal interrupted
        interrupted = True
        stop_event.set()

    previous_int = signal.signal(signal.SIGINT, note_signal)
    previous_term = signal.signal(signal.SIGTERM, note_signal)

    failure: BaseException | None = None
    cleanup_passed = False
    try:
        if not all(port_is_free(user.chrome.port) for user in users):
            raise DriverError("A requested CDP port is occupied; no browser was launched")

        print(
            f"Launching two owned, visible Chrome windows on CDP ports {args.port_a}/{args.port_b}…",
            flush=True,
        )
        for user in users:
            if interrupted:
                raise KeyboardInterrupt
            user.chrome.launch()
            user.ui = RobotSwarmUi(user.chrome, expected_origin, stop_event)
            report["users"][user.label]["browser"] = {
                "product": user.chrome.product,
                "profile": user.chrome.profile.name,
                "ownedPid": user.chrome.process.pid if user.chrome.process else None,
            }

        def login_user(user: UserRun) -> dict[str, bool]:
            assert user.ui
            user.ui.navigate(args.url)
            user.ui.login(user.credentials["email"], user.credentials["password"])
            return user.ui.require_hls_media_capabilities()

        media_capabilities = parallel(users, login_user)
        for user in users:
            report["users"][user.label]["login"] = "passed"
            report["users"][user.label]["browser"]["mediaCapabilities"] = (
                media_capabilities[user.label]
            )
        print("Both accounts reached the real simulation workspace.", flush=True)

        parallel(users, lambda user: user.ui.create_session(user.robot_count))  # type: ignore[union-attr]
        ready = parallel(
            users,
            lambda user: user.ui.wait_ready(user.robot_count, args.ready_timeout),  # type: ignore[union-attr]
        )
        for label, result in ready.items():
            report["users"][label]["session"] = result
        print("Both isolated sessions are Ready.", flush=True)

        parallel(users, lambda user: user.ui.open_viewer(args.viewer_timeout))  # type: ignore[union-attr]
        scene_barrier = threading.Barrier(2)

        def capture_private_scene(user: UserRun) -> dict[str, Any]:
            assert user.ui
            state = user.ui.require_interactive_hls()
            if state.get("controlText") != "Interactuar":
                raise DriverError(f"User {user.label} still had interactive control enabled")
            try:
                scene_barrier.wait(timeout=20)
            except threading.BrokenBarrierError as exc:
                raise DriverError("The synchronized scene-capture barrier was broken") from exc
            started = time.monotonic()
            captured_at = utc_now()
            clip = user.ui.capture_video_clip(
                args.output_dir / f"{run_id}-{user.label.lower()}-private-scene.png"
            )
            ended = time.monotonic()
            return {
                "clip": clip,
                "capturedAt": captured_at,
                "_midpoint": (started + ended) / 2,
            }

        private_scenes = parallel(users, capture_private_scene)
        capture_midpoints = [float(item.pop("_midpoint")) for item in private_scenes.values()]
        capture_skew_ms = (max(capture_midpoints) - min(capture_midpoints)) * 1000
        if capture_skew_ms > args.max_capture_skew_ms:
            raise DriverError(
                f"The private scene captures exceeded the skew limit ({capture_skew_ms:.1f} ms)"
            )
        scene_distance = RobotSwarmUi._hash_distance(
            private_scenes["A"]["clip"]["averageHash"],
            private_scenes["B"]["clip"]["averageHash"],
        )
        scene_difference_ratio = RobotSwarmUi._scene_difference(
            args.output_dir / private_scenes["A"]["clip"]["file"],
            args.output_dir / private_scenes["B"]["clip"]["file"],
        )
        if scene_difference_ratio < MIN_SCENE_DIFFERENCE_RATIO:
            raise DriverError(
                "The two private viewer regions were unexpectedly similar "
                f"(changed-pixel ratio {scene_difference_ratio:.6f})"
            )
        report["sceneDistinctness"] = {
            "synchronized": True,
            "captureSkewMs": round(capture_skew_ms, 3),
            "maximumCaptureSkewMs": args.max_capture_skew_ms,
            "requestedRosters": {"A": args.robots_a, "B": args.robots_b},
            "differentRequestedRosters": True,
            "averageHashDistance": scene_distance,
            "changedPixelRatio": round(scene_difference_ratio, 7),
            "changedPixelThreshold": SCENE_PIXEL_THRESHOLD,
            "minimumChangedPixelRatio": MIN_SCENE_DIFFERENCE_RATIO,
            "clips": {
                "A": private_scenes["A"],
                "B": private_scenes["B"],
            },
            "humanReviewRequired": True,
            "requiresApiCrossDenial": True,
        }
        interaction = parallel(
            users,
            lambda user: user.ui.exercise_interaction(  # type: ignore[union-attr]
                args.output_dir,
                run_id,
                user.label,
            ),
        )
        for user in users:
            assert user.ui
            fullscreen = user.ui.exercise_fullscreen(
                args.output_dir / f"{run_id}-{user.label.lower()}-fullscreen.png",
                user.credentials["email"],
                user.credentials["password"],
            )
            report["users"][user.label]["interaction"] = interaction[user.label]
            report["users"][user.label]["fullscreen"] = fullscreen
        report["desktopPrivateViewers"] = desktop_screenshot(
            users,
            args.output_dir / f"{run_id}-desktop-private-viewers.png",
            run_id + "-private-viewers",
            args.profile_root,
            mode="viewers",
        )
        print("Both private viewers accepted real input and entered fullscreen.", flush=True)

        viewer_ready = parallel(
            users,
            lambda user: user.ui.screenshot(  # type: ignore[union-attr]
                args.output_dir / f"{run_id}-{user.label.lower()}-viewer.png",
                user.credentials["email"],
                user.credentials["password"],
            ),
        )

        start_barrier = threading.Barrier(2)

        def start_required_task(user: UserRun) -> dict[str, Any]:
            assert user.ui
            return (
                user.ui.start_figure_letter_a(start_barrier)
                if user.label == "A"
                else user.ui.start_follow_figure8(start_barrier)
            )

        started = parallel(users, start_required_task)
        click_points = [float(result.pop("_clickMonotonic")) for result in started.values()]
        overlap = observe_same_round_running(users)
        for label, result in started.items():
            report["users"][label]["task"] = result
        report["taskConcurrency"] = {
            "barrierImmediatelyBeforeStartClick": True,
            "clickSkewMs": round((max(click_points) - min(click_points)) * 1000, 3),
            "overlap": overlap,
        }
        print("UI started Figure/letter A for A and FollowLeader/figure-eight for B.", flush=True)

        user_a, user_b = users
        assert user_a.label == "A" and user_b.label == "B" and user_a.ui and user_b.ui
        verified_a = user_a.ui.wait_verified_figure(args.task_timeout)
        running_b = user_b.ui.task_status()
        if running_b.get("state") != "Running":
            raise DriverError(f"User B was expected to remain Running, but was {running_b.get('state')}")
        report["users"]["A"]["task"]["accepted"] = verified_a
        report["users"]["B"]["task"]["accepted"] = running_b

        viewer_close_b = user_b.ui.close_viewer_while_task_runs(10.0)
        viewer_closed_screenshot = user_b.ui.screenshot(
            args.output_dir / f"{run_id}-b-viewer-closed-task-running.png",
            user_b.credentials["email"],
            user_b.credentials["password"],
        )
        user_b.ui.open_viewer(args.viewer_timeout)
        reopened_state = user_b.ui.require_interactive_hls()
        if user_b.ui.task_status().get("state") != "Running":
            raise DriverError("User B task did not remain Running after reopening its viewer")
        report["users"]["B"]["viewerLifecycle"] = {
            **viewer_close_b,
            "closedScreenshot": viewer_closed_screenshot,
            "reopened": True,
            "reopenedState": reopened_state,
            "taskStillRunning": True,
        }
        print("B closed and reopened only its viewer while ROS kept running.", flush=True)

        metrics = parallel(users, lambda user: user.ui.video_metrics(args.video_seconds))  # type: ignore[union-attr]
        for label, result in metrics.items():
            require_live_video(
                result,
                args.video_seconds,
                f"User {label}",
                args.min_video_fps,
                args.max_dropped_ratio,
            )
        task_scene_screenshots = parallel(
            users,
            lambda user: user.ui.screenshot(  # type: ignore[union-attr]
                args.output_dir / f"{run_id}-{user.label.lower()}-task-scene.png",
                user.credentials["email"],
                user.credentials["password"],
            ),
        )
        parallel(users, lambda user: user.ui.show_task_panel())  # type: ignore[union-attr]
        task_screenshots = parallel(
            users,
            lambda user: user.ui.screenshot(  # type: ignore[union-attr]
                args.output_dir / f"{run_id}-{user.label.lower()}-task-state.png",
                user.credentials["email"],
                user.credentials["password"],
            ),
        )
        desktop_before_stop = desktop_screenshot(
            users,
            args.output_dir / f"{run_id}-desktop-both-task-state.png",
            run_id,
            args.profile_root,
            mode="tasks",
        )
        for label in ("A", "B"):
            report["users"][label]["viewer"] = {
                "readyScreenshot": viewer_ready[label],
                "taskSceneScreenshot": task_scene_screenshots[label],
                "taskScreenshot": task_screenshots[label],
                "video": metrics[label],
            }
        report["desktopBeforeStopA"] = desktop_before_stop
        print("A completed with a verified outcome while B remained Running; both windows were captured.", flush=True)

        stopped_a = user_a.ui.stop_created_session()
        if not stopped_a.get("released"):
            raise DriverError("User A session did not release before the continuity measurement")
        report["users"]["A"]["stoppedBeforePeerMeasurement"] = stopped_a
        running_b_after_stop = user_b.ui.task_status()
        if running_b_after_stop.get("state") != "Running":
            raise DriverError(
                f"User B stopped running when A ended; state was {running_b_after_stop.get('state')}"
            )
        post_stop_video = user_b.ui.video_metrics(10.0)
        require_live_video(
            post_stop_video,
            10.0,
            "User B after stopping A",
            args.min_video_fps,
            args.max_dropped_ratio,
        )
        if user_b.ui.task_status().get("state") != "Running":
            raise DriverError("User B did not remain Running throughout the 10-second peer-stop measurement")
        post_stop_viewer_screenshot = user_b.ui.screenshot(
            args.output_dir / f"{run_id}-b-after-a-stopped-viewer.png",
            user_b.credentials["email"],
            user_b.credentials["password"],
        )
        user_b.ui.show_task_panel()
        post_stop_task_screenshot = user_b.ui.screenshot(
            args.output_dir / f"{run_id}-b-after-a-stopped-task.png",
            user_b.credentials["email"],
            user_b.credentials["password"],
        )
        desktop_after_stop = desktop_screenshot(
            users,
            args.output_dir / f"{run_id}-desktop-a-stopped-b-running.png",
            run_id + "-after-stop",
            args.profile_root,
            mode="post_stop",
        )
        report["users"]["B"]["afterPeerStopped"] = {
            "task": user_b.ui.task_status(),
            "video10Seconds": post_stop_video,
            "viewerScreenshot": post_stop_viewer_screenshot,
            "taskScreenshot": post_stop_task_screenshot,
        }
        report["desktopAfterStopA"] = desktop_after_stop
        print("After stopping A, B stayed Running and decoded video for another 10 seconds.", flush=True)

        if args.hold_seconds:
            deadline = time.monotonic() + args.hold_seconds
            while time.monotonic() < deadline and not stop_event.is_set():
                time.sleep(min(0.5, deadline - time.monotonic()))
        if stop_event.is_set():
            raise KeyboardInterrupt
        report["success"] = True
    except BaseException as exc:  # cleanup still runs for Ctrl-C and partial launches
        failure = exc
        report["error"] = {
            "type": type(exc).__name__,
            "message": sanitizer.text(exc),
        }
    finally:
        # A signal interrupts the normal traversal, but it must not interrupt
        # the release of the viewers, sessions, Chrome profiles, or report.
        stop_event.clear()
        for user in users:
            if user.ui:
                report["users"][user.label]["clickAudit"] = user.ui.click_evidence()
                report["cleanup"][f"viewer{user.label}"] = user.ui.normalize_viewer()
        if not args.leave_sessions:
            for user in reversed(users):
                if user.ui:
                    try:
                        report["cleanup"][f"session{user.label}"] = user.ui.stop_created_session()
                    except Exception as exc:
                        report["cleanup"][f"session{user.label}"] = {
                            "requested": True,
                            "released": False,
                            "error": sanitizer.text(exc),
                        }
        else:
            for user in users:
                report["cleanup"][f"session{user.label}"] = {"requested": False, "reason": "--leave-sessions"}

        for user in reversed(users):
            try:
                report["cleanup"][f"browser{user.label}"] = user.chrome.close_owned()
            except Exception as exc:
                report["cleanup"][f"browser{user.label}"] = {
                    "requested": True,
                    "portFree": port_is_free(user.chrome.port),
                    "error": sanitizer.text(exc),
                }
        browser_cleanup_ok = all(
            bool(report["cleanup"].get(f"browser{label}", {}).get("portFree"))
            and bool(report["cleanup"].get(f"browser{label}", {}).get("processExited"))
            and bool(report["cleanup"].get(f"browser{label}", {}).get("profileRemoved"))
            for label in ("A", "B")
        )
        session_cleanup_ok = not args.leave_sessions and all(
            not report["cleanup"].get(f"session{label}", {}).get("requested")
            or bool(report["cleanup"].get(f"session{label}", {}).get("released"))
            for label in ("A", "B")
        )
        viewer_cleanup_ok = all(
            bool(report["cleanup"].get(f"viewer{label}", {}).get("controlReleased"))
            and bool(report["cleanup"].get(f"viewer{label}", {}).get("fullscreenExited"))
            for label in ("A", "B")
        )
        cleanup_passed = browser_cleanup_ok and session_cleanup_ok and viewer_cleanup_ok
        report["cleanup"]["passed"] = cleanup_passed
        if not cleanup_passed:
            report["success"] = False
            report.setdefault(
                "error",
                {"type": "CleanupError", "message": "Owned browser or session cleanup did not finish"},
            )
        report["completedAt"] = utc_now()
        report = sanitizer.value(report)
        write_json_secure(output_json, report)
        signal.signal(signal.SIGINT, previous_int)
        signal.signal(signal.SIGTERM, previous_term)

    print(f"Sanitized report: {output_json}", flush=True)
    exit_code = result_exit_code(failure, cleanup_passed)
    if failure is not None:
        print(f"Visible acceptance failed: {sanitizer.text(failure)}", file=sys.stderr, flush=True)
    elif exit_code != 0:
        print("Visible acceptance passed, but owned-resource cleanup did not finish.", file=sys.stderr, flush=True)
    return exit_code


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except DriverError as exc:
        print(f"Visible acceptance could not start: {exc}", file=sys.stderr)
        raise SystemExit(2)
