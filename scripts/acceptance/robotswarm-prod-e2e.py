#!/usr/bin/env python3
"""Prueba de aceptacion multiusuario para RobotSwarm.

El arnes es deliberadamente conservador: no abre conexiones sin la bandera
``--execute-production``, no muestra credenciales ni identificadores reales y
siempre intenta detener las sesiones que haya creado.
"""

from __future__ import annotations

import argparse
import concurrent.futures
import dataclasses
import datetime as dt
import hashlib
import hmac
import json
import os
import random
import re
import ssl
import stat
import sys
import tempfile
import threading
import time
import urllib.error
import urllib.parse
import urllib.request
import uuid
from pathlib import Path
from typing import Any, Iterable


API_ORIGIN = "https://robot.zerav.la"
CREDENTIALS_PATH = Path("/tmp/robotswarm-e2e-credentials.env")
BINDING_KEY_PATH = Path("/tmp/robotswarm-e2e-binding.key")
MAX_RESPONSE_BYTES = 2 * 1024 * 1024
POST_STOP_HLS_RESERVE_SECONDS = 10.0
LIVE_STATES = {"Queued", "Provisioning", "Ready", "Active", "Paused", "Stopping"}
CONTROL_STATES = {"Ready", "Active", "Paused"}
TERMINAL_TASK_STATES = {"Completed", "Cancelled", "Failed"}
UUID_PATTERN = re.compile(
    r"(?i)\b[0-9a-f]{8}-[0-9a-f]{4}-[1-5][0-9a-f]{3}-"
    r"[89ab][0-9a-f]{3}-[0-9a-f]{12}\b"
)
DOTNET_TIMESTAMP_PATTERN = re.compile(
    r"^(\d{4}-\d{2}-\d{2}T\d{2}:\d{2}:\d{2})"
    r"(?:\.(\d{1,7}))?(Z|[+-]\d{2}:\d{2})?$"
)


class HarnessFailure(Exception):
    """Falla prevista cuyo codigo es seguro para el informe saneado."""

    def __init__(self, code: str, phase: str = "runtime") -> None:
        super().__init__(code)
        self.code = code
        self.phase = phase


@dataclasses.dataclass(frozen=True)
class Credential:
    account_marker: int
    email: str
    password: str


@dataclasses.dataclass
class HttpResult:
    status: int
    headers: dict[str, str]
    body: bytes
    elapsed_ms: int


@dataclasses.dataclass(frozen=True)
class ViewerLease:
    lease_id: str
    token: str
    hls_url: str
    expires_at_raw: str
    expires_at: dt.datetime


class NoRedirect(urllib.request.HTTPRedirectHandler):
    """Impide que un bearer se reenvie a un destino inesperado."""

    def redirect_request(self, req, fp, code, msg, headers, newurl):  # noqa: ANN001
        return None


class Report:
    def __init__(self) -> None:
        now = utc_now()
        self._lock = threading.Lock()
        self.data: dict[str, Any] = {
            "schema_version": 1,
            "result": "running",
            "started_at": now,
            "finished_at": None,
            "configuration": {
                "users": 2,
                "robot_counts": {"user_a": 3, "user_b": 7},
                "task": {
                    "parallel": True,
                    "user_a": {"type": "Figure", "formation": "triangle"},
                    "user_b": {"type": "FollowLeader", "mode": "figure8"},
                },
                "production_execution": True,
                "sensitive_values_redacted": True,
            },
            "checks": [],
            "sessions": {
                "user_a": {"robot_count": 3, "ready": False, "final_state": "unknown"},
                "user_b": {"robot_count": 7, "ready": False, "final_state": "unknown"},
            },
            "session_creation": {
                "user_a": {
                    "attempts": 0,
                    "conflict_responses": 0,
                    "retries_performed": 0,
                    "first_conflict_observed": False,
                    "retry_delays_ms": [],
                    "created": False,
                },
                "user_b": {
                    "attempts": 0,
                    "conflict_responses": 0,
                    "retries_performed": 0,
                    "first_conflict_observed": False,
                    "retry_delays_ms": [],
                    "created": False,
                },
            },
            "task": {
                "user_a": {
                    "type": "Figure",
                    "accepted": False,
                    "terminal_state": "unknown",
                    "outcome_state": "unknown",
                    "progress": 0.0,
                    "result_contract": "not_checked",
                    "running_overlap_observed": False,
                    "intervals_overlap": False,
                    "overlap_milliseconds": 0,
                    "timing_captured": False,
                },
                "user_b": {
                    "type": "FollowLeader",
                    "accepted": False,
                    "running_while_user_a_completed": False,
                    "cancel_requested": False,
                    "terminal_state": "unknown",
                    "outcome_state": "unknown",
                    "timing_captured": False,
                },
            },
            "viewer": {
                "user_a": {
                    "lease_issued": False,
                    "lease_rotated": False,
                    "previous_lease_revoked": False,
                    "lease_refreshed_before_stop": False,
                    "remaining_seconds_before_stop": 0,
                    "remaining_seconds_after_stop": 0,
                    "remaining_seconds_before_post_stop_hls": 0,
                    "playlist_ready": False,
                },
                "user_b": {
                    "lease_issued": False,
                    "lease_refreshed_before_stop": False,
                    "remaining_seconds_before_stop": 0,
                    "remaining_seconds_after_stop": 0,
                    "remaining_seconds_before_post_stop_hls": 0,
                    "playlist_ready": False,
                },
            },
            "isolation": {
                "api_unauthenticated": False,
                "api_cross_session": False,
                "api_cross_tasks": False,
                "api_cross_stop": False,
                "hls_cross_user_a": False,
                "hls_cross_user_b": False,
                "hls_unauthenticated": False,
                "stop_kept_user_b_active": False,
                "stop_kept_user_b_viewer": False,
                "stopped_user_a_hls_rejected": False,
                "stop_completed_before_lease_expiry": False,
                "stop_duration_ms": 0,
                "post_stop_hls_required_margin_seconds": 0,
            },
            "cleanup": {
                "attempted": False,
                "user_a": "not_created",
                "user_b": "not_created",
                "complete": False,
            },
            "http_audit": [],
            "error": None,
        }

    def add_check(self, name: str, status: str, detail: str | None = None) -> None:
        entry: dict[str, str] = {"name": name, "status": status}
        if detail is not None:
            entry["detail"] = detail
        with self._lock:
            self.data["checks"].append(entry)

    def add_http(
        self,
        name: str,
        actor: str,
        target: str,
        method: str,
        expected: Iterable[int],
        observed: int,
        elapsed_ms: int,
        passed: bool,
    ) -> None:
        entry = {
            "name": name,
            "actor": actor,
            "target": target,
            "method": method,
            "expected_status": sorted(set(expected)),
            "observed_status": observed,
            "elapsed_ms": elapsed_ms,
            "passed": passed,
        }
        with self._lock:
            self.data["http_audit"].append(entry)

    def set_path(self, *path: str, value: Any) -> None:
        with self._lock:
            cursor = self.data
            for part in path[:-1]:
                cursor = cursor[part]
            cursor[path[-1]] = value

    def record_creation_attempt(
        self,
        alias: str,
        attempt: int,
        status: int,
        retry_delay_ms: int | None = None,
    ) -> None:
        with self._lock:
            entry = self.data["session_creation"][alias]
            entry["attempts"] = attempt
            if status == 409:
                entry["first_conflict_observed"] = True
                entry["conflict_responses"] += 1
            elif status == 202:
                entry["created"] = True
            if retry_delay_ms is not None:
                entry["retries_performed"] += 1
                entry["retry_delays_ms"].append(retry_delay_ms)

    def record_creation_started(self, alias: str, attempt: int) -> None:
        with self._lock:
            self.data["session_creation"][alias]["attempts"] = attempt

    def finish(self, result: str, error: HarnessFailure | None = None) -> None:
        with self._lock:
            self.data["result"] = result
            self.data["finished_at"] = utc_now()
            self.data["error"] = (
                None
                if error is None
                else {"code": error.code, "phase": error.phase}
            )


class Transport:
    def __init__(self, report: Report, timeout: float) -> None:
        self.report = report
        self.timeout = timeout
        self.context = ssl.create_default_context()

    @staticmethod
    def _validate_url(url: str, *, hls_only: bool = False) -> str:
        parsed = urllib.parse.urlsplit(url)
        expected = urllib.parse.urlsplit(API_ORIGIN)
        if (
            parsed.scheme != expected.scheme
            or parsed.hostname != expected.hostname
            or parsed.port not in (None, 443)
            or parsed.username is not None
            or parsed.password is not None
            or parsed.fragment
        ):
            raise HarnessFailure("unexpected_response_origin", "viewer")
        if hls_only and not parsed.path.startswith("/api/viewer/hls/"):
            raise HarnessFailure("unexpected_hls_path", "viewer")
        return urllib.parse.urlunsplit(parsed)

    def raw(
        self,
        method: str,
        path_or_url: str,
        *,
        bearer: str | None = None,
        payload: dict[str, Any] | None = None,
        hls_only: bool = False,
    ) -> HttpResult:
        if path_or_url.startswith("/"):
            url = f"{API_ORIGIN}{path_or_url}"
        else:
            url = self._validate_url(path_or_url, hls_only=hls_only)

        headers = {"Accept": "application/json"}
        body = None
        if payload is not None:
            body = json.dumps(payload, separators=(",", ":")).encode("utf-8")
            headers["Content-Type"] = "application/json"
        if bearer:
            headers["Authorization"] = f"Bearer {bearer}"

        request = urllib.request.Request(url, data=body, headers=headers, method=method)
        opener = urllib.request.build_opener(
            urllib.request.HTTPSHandler(context=self.context),
            NoRedirect(),
        )
        started = time.monotonic()
        try:
            with opener.open(request, timeout=self.timeout) as response:
                response_body = response.read(MAX_RESPONSE_BYTES + 1)
                if len(response_body) > MAX_RESPONSE_BYTES:
                    raise HarnessFailure("response_too_large", "network")
                return HttpResult(
                    status=response.status,
                    headers={key.lower(): value for key, value in response.headers.items()},
                    body=response_body,
                    elapsed_ms=round((time.monotonic() - started) * 1000),
                )
        except urllib.error.HTTPError as error:
            response_body = error.read(MAX_RESPONSE_BYTES + 1)
            if len(response_body) > MAX_RESPONSE_BYTES:
                raise HarnessFailure("response_too_large", "network") from None
            return HttpResult(
                status=error.code,
                headers={key.lower(): value for key, value in error.headers.items()},
                body=response_body,
                elapsed_ms=round((time.monotonic() - started) * 1000),
            )
        except (urllib.error.URLError, TimeoutError, ssl.SSLError, OSError):
            raise HarnessFailure("network_error", "network") from None

    def json_call(
        self,
        method: str,
        path: str,
        *,
        actor: str,
        target: str,
        name: str,
        expected: set[int],
        bearer: str | None = None,
        payload: dict[str, Any] | None = None,
        idempotent: bool = False,
        record: bool = True,
    ) -> tuple[HttpResult, Any]:
        if idempotent:
            # La clave vive solo en memoria y nunca se incluye en el informe.
            idempotency_key = str(uuid.uuid4())
        else:
            idempotency_key = None

        headers_payload = payload
        result = self._raw_with_idempotency(
            method,
            path,
            bearer=bearer,
            payload=headers_payload,
            idempotency_key=idempotency_key,
        )
        passed = result.status in expected
        if record:
            self.report.add_http(
                name,
                actor,
                target,
                method,
                expected,
                result.status,
                result.elapsed_ms,
                passed,
            )
        if not passed:
            raise HarnessFailure(f"unexpected_http_status_{name}", name)
        if not result.body:
            return result, None
        try:
            decoded = json.loads(result.body)
        except (UnicodeDecodeError, json.JSONDecodeError):
            raise HarnessFailure(f"invalid_json_{name}", name) from None
        return result, decoded

    def _raw_with_idempotency(
        self,
        method: str,
        path: str,
        *,
        bearer: str | None,
        payload: dict[str, Any] | None,
        idempotency_key: str | None,
    ) -> HttpResult:
        url = f"{API_ORIGIN}{path}"
        headers = {"Accept": "application/json"}
        body = None
        if payload is not None:
            body = json.dumps(payload, separators=(",", ":")).encode("utf-8")
            headers["Content-Type"] = "application/json"
        if bearer:
            headers["Authorization"] = f"Bearer {bearer}"
        if idempotency_key:
            headers["Idempotency-Key"] = idempotency_key

        request = urllib.request.Request(url, data=body, headers=headers, method=method)
        opener = urllib.request.build_opener(
            urllib.request.HTTPSHandler(context=self.context),
            NoRedirect(),
        )
        started = time.monotonic()
        try:
            with opener.open(request, timeout=self.timeout) as response:
                response_body = response.read(MAX_RESPONSE_BYTES + 1)
                if len(response_body) > MAX_RESPONSE_BYTES:
                    raise HarnessFailure("response_too_large", "network")
                return HttpResult(
                    response.status,
                    {key.lower(): value for key, value in response.headers.items()},
                    response_body,
                    round((time.monotonic() - started) * 1000),
                )
        except urllib.error.HTTPError as error:
            response_body = error.read(MAX_RESPONSE_BYTES + 1)
            if len(response_body) > MAX_RESPONSE_BYTES:
                raise HarnessFailure("response_too_large", "network") from None
            return HttpResult(
                error.code,
                {key.lower(): value for key, value in error.headers.items()},
                response_body,
                round((time.monotonic() - started) * 1000),
            )
        except (urllib.error.URLError, TimeoutError, ssl.SSLError, OSError):
            raise HarnessFailure("network_error", "network") from None


class AuthenticatedUser:
    def __init__(
        self,
        alias: str,
        account_marker: int,
        transport: Transport,
        jwt_token: str,
        refresh_token: str,
    ) -> None:
        self.alias = alias
        self.account_marker = account_marker
        self.transport = transport
        self.jwt_token = jwt_token
        self.refresh_token = refresh_token
        self.authenticated_at = time.monotonic()
        self._refresh_lock = threading.Lock()

    def ensure_fresh(self) -> None:
        if time.monotonic() - self.authenticated_at < 13 * 60:
            return
        with self._refresh_lock:
            if time.monotonic() - self.authenticated_at < 13 * 60:
                return
            _, data = self.transport.json_call(
                "POST",
                "/Accounts/refreshToken",
                actor=self.alias,
                target="authentication",
                name="auth_refresh",
                expected={200},
                payload={"refreshToken": self.refresh_token},
            )
            _, self.jwt_token, self.refresh_token = validate_auth_response(
                data, self.account_marker
            )
            self.authenticated_at = time.monotonic()

    def call(
        self,
        method: str,
        path: str,
        *,
        target: str,
        name: str,
        expected: set[int],
        payload: dict[str, Any] | None = None,
        idempotent: bool = False,
        record: bool = True,
    ) -> tuple[HttpResult, Any]:
        self.ensure_fresh()
        return self.transport.json_call(
            method,
            path,
            actor=self.alias,
            target=target,
            name=name,
            expected=expected,
            bearer=self.jwt_token,
            payload=payload,
            idempotent=idempotent,
            record=record,
        )


def utc_now() -> str:
    return dt.datetime.now(dt.timezone.utc).isoformat(timespec="seconds").replace("+00:00", "Z")


def safe_progress(message: str) -> None:
    print(f"[robotswarm-e2e] {message}", file=sys.stderr, flush=True)


def account_fingerprint(credential: Credential, binding_key: bytes) -> str:
    """Create a report-safe binding to the exact local test credential."""
    message = b"\0".join(
        (
            b"robotswarm-e2e-account-binding-v2",
            str(credential.account_marker).encode("ascii"),
            credential.email.strip().lower().encode("utf-8"),
            credential.password.encode("utf-8"),
        )
    )
    return hmac.new(
        binding_key,
        message,
        hashlib.sha256,
    ).hexdigest()


def load_binding_key(path: Path, report: Report) -> bytes:
    flags = os.O_RDONLY
    if hasattr(os, "O_NOFOLLOW"):
        flags |= os.O_NOFOLLOW
    try:
        fd = os.open(path, flags)
    except OSError:
        raise HarnessFailure("binding_key_unavailable", "preflight") from None
    try:
        details = os.fstat(fd)
        if not stat.S_ISREG(details.st_mode):
            raise HarnessFailure("binding_key_not_regular", "preflight")
        if stat.S_IMODE(details.st_mode) != 0o600 or details.st_uid != os.geteuid():
            raise HarnessFailure("binding_key_permissions_invalid", "preflight")
        value = os.read(fd, 33)
    finally:
        os.close(fd)
    if len(value) != 32:
        raise HarnessFailure("binding_key_size_invalid", "preflight")
    report.add_check("binding_key_owner_only", "passed", "0600")
    return value


def load_credentials(report: Report) -> dict[str, Credential]:
    flags = os.O_RDONLY
    if hasattr(os, "O_NOFOLLOW"):
        flags |= os.O_NOFOLLOW
    try:
        fd = os.open(CREDENTIALS_PATH, flags)
    except OSError:
        raise HarnessFailure("credentials_unavailable", "preflight") from None

    try:
        metadata = os.fstat(fd)
        if not stat.S_ISREG(metadata.st_mode):
            raise HarnessFailure("credentials_not_regular_file", "preflight")
        if stat.S_IMODE(metadata.st_mode) != 0o600:
            raise HarnessFailure("credentials_permissions_not_0600", "preflight")
        if metadata.st_uid != os.geteuid():
            raise HarnessFailure("credentials_owner_mismatch", "preflight")
        if metadata.st_size > 16 * 1024:
            raise HarnessFailure("credentials_file_too_large", "preflight")
        with os.fdopen(fd, "r", encoding="utf-8", errors="strict", closefd=False) as stream:
            content = stream.read(16 * 1024 + 1)
    except UnicodeError:
        raise HarnessFailure("credentials_encoding_invalid", "preflight") from None
    finally:
        os.close(fd)

    expected_keys = {
        "TEST_A_ID",
        "TEST_A_EMAIL",
        "TEST_A_PASSWORD",
        "TEST_B_ID",
        "TEST_B_EMAIL",
        "TEST_B_PASSWORD",
    }
    values: dict[str, str] = {}
    for raw_line in content.splitlines():
        line = raw_line.strip()
        if not line or line.startswith("#"):
            continue
        if "=" not in line:
            raise HarnessFailure("credentials_format_invalid", "preflight")
        key, value = line.split("=", 1)
        key = key.strip()
        if key not in expected_keys or key in values:
            raise HarnessFailure("credentials_schema_invalid", "preflight")
        if not value:
            raise HarnessFailure("credentials_value_missing", "preflight")
        values[key] = value
    if set(values) != expected_keys:
        raise HarnessFailure("credentials_schema_invalid", "preflight")

    try:
        marker_a = int(values["TEST_A_ID"])
        marker_b = int(values["TEST_B_ID"])
    except ValueError:
        raise HarnessFailure("credentials_account_marker_invalid", "preflight") from None
    if marker_a <= 0 or marker_b <= 0 or marker_a == marker_b:
        raise HarnessFailure("credentials_accounts_not_distinct", "preflight")
    if values["TEST_A_EMAIL"] == values["TEST_B_EMAIL"]:
        raise HarnessFailure("credentials_accounts_not_distinct", "preflight")

    report.add_check("credentials_file_permissions", "passed", "0600")
    report.add_check("credentials_schema", "passed")
    return {
        "user_a": Credential(marker_a, values["TEST_A_EMAIL"], values["TEST_A_PASSWORD"]),
        "user_b": Credential(marker_b, values["TEST_B_EMAIL"], values["TEST_B_PASSWORD"]),
    }


def validate_auth_response(
    data: Any,
    expected_account_marker: int,
) -> tuple[int, str, str]:
    if not isinstance(data, dict):
        raise HarnessFailure("authentication_response_invalid", "authentication")
    jwt_token = data.get("jwtToken")
    refresh_token = data.get("refreshToken")
    account_marker = data.get("id")
    if account_marker != expected_account_marker:
        raise HarnessFailure("authenticated_account_mismatch", "authentication")
    if not isinstance(jwt_token, str) or len(jwt_token) < 80:
        raise HarnessFailure("authentication_response_invalid", "authentication")
    if not isinstance(refresh_token, str) or len(refresh_token) < 32:
        raise HarnessFailure("authentication_response_invalid", "authentication")
    return account_marker, jwt_token, refresh_token


def authenticate(
    alias: str,
    credential: Credential,
    transport: Transport,
) -> AuthenticatedUser:
    _, data = transport.json_call(
        "POST",
        "/Accounts/authenticate",
        actor=alias,
        target="authentication",
        name="authenticate",
        expected={200},
        payload={"email": credential.email, "password": credential.password},
    )
    _, jwt_token, refresh_token = validate_auth_response(data, credential.account_marker)
    return AuthenticatedUser(
        alias,
        credential.account_marker,
        transport,
        jwt_token,
        refresh_token,
    )


def require_uuid(data: Any, key: str, phase: str) -> str:
    if not isinstance(data, dict) or not isinstance(data.get(key), str):
        raise HarnessFailure("resource_identifier_missing", phase)
    try:
        return str(uuid.UUID(data[key]))
    except (ValueError, AttributeError):
        raise HarnessFailure("resource_identifier_invalid", phase) from None


def occupying_session(item: Any) -> bool:
    if not isinstance(item, dict):
        raise HarnessFailure("session_response_invalid", "preflight")
    state = item.get("state")
    if state in LIVE_STATES:
        return True
    return state in {"Failed", "Expired"} and item.get("computeWorkerId") is not None


def wait_session_ready(
    user: AuthenticatedUser,
    session_id: str,
    expected_count: int,
    timeout: float,
    poll_interval: float,
) -> str:
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        _, data = user.call(
            "GET",
            f"/api/sessions/{session_id}",
            target="session_self",
            name="poll_session_ready",
            expected={200},
            record=False,
        )
        if not isinstance(data, dict):
            raise HarnessFailure("session_response_invalid", "session_ready")
        state = data.get("state")
        if data.get("desiredRobotCount") != expected_count:
            raise HarnessFailure("session_robot_count_mismatch", "session_ready")
        if state in CONTROL_STATES:
            return state
        if state in {"Stopped", "Failed", "Expired"}:
            raise HarnessFailure("session_became_terminal", "session_ready")
        time.sleep(poll_interval)
    raise HarnessFailure("session_ready_timeout", "session_ready")


def wait_session_stopped(
    user: AuthenticatedUser,
    session_id: str,
    timeout: float,
    poll_interval: float,
) -> str:
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        result, data = user.call(
            "GET",
            f"/api/sessions/{session_id}",
            target="session_self",
            name="poll_session_stopped",
            expected={200, 404},
            record=False,
        )
        if result.status == 404:
            return "released"
        if not isinstance(data, dict):
            raise HarnessFailure("session_response_invalid", "cleanup")
        state = data.get("state")
        if state == "Stopped":
            return "Stopped"
        if state in {"Failed", "Expired"} and data.get("computeWorkerId") is None:
            return "released_terminal"
        time.sleep(poll_interval)
    return "timeout"


def parse_dotnet_timestamp(value: Any, phase: str) -> dt.datetime:
    if not isinstance(value, str):
        raise HarnessFailure("timestamp_invalid", phase)
    match = DOTNET_TIMESTAMP_PATTERN.fullmatch(value)
    if match is None:
        raise HarnessFailure("timestamp_invalid", phase)
    base, fraction, zone = match.groups()
    microseconds = (fraction or "").ljust(7, "0")[:6]
    # Estas columnas son UTC aunque PostgreSQL las conserve como
    # ``timestamp without time zone``; System.Text.Json puede omitir el sufijo.
    normalized_zone = "+00:00" if zone in (None, "Z") else zone
    normalized = f"{base}.{microseconds}{normalized_zone}"
    try:
        parsed = dt.datetime.fromisoformat(normalized)
    except ValueError:
        raise HarnessFailure("timestamp_invalid", phase) from None
    return parsed.astimezone(dt.timezone.utc)


def validate_lease(data: Any, session_id: str) -> ViewerLease:
    if not isinstance(data, dict):
        raise HarnessFailure("viewer_lease_response_invalid", "viewer")
    if data.get("sessionId") != session_id or data.get("source") != "Scene":
        raise HarnessFailure("viewer_lease_scope_invalid", "viewer")
    lease_id = require_uuid(data, "leaseId", "viewer")
    token = data.get("token")
    hls_url = data.get("hlsUrl")
    expires_at_value = data.get("expiresAt")
    if not isinstance(token, str) or not re.fullmatch(r"[A-Za-z0-9_-]{43,128}", token):
        raise HarnessFailure("viewer_token_invalid", "viewer")
    if not isinstance(hls_url, str) or not hls_url:
        raise HarnessFailure("viewer_hls_unavailable", "viewer")
    Transport._validate_url(hls_url, hls_only=True)
    try:
        expires_at = parse_dotnet_timestamp(expires_at_value, "viewer")
    except HarnessFailure:
        raise HarnessFailure("viewer_expiry_invalid", "viewer") from None
    if expires_at <= dt.datetime.now(dt.timezone.utc):
        raise HarnessFailure("viewer_lease_already_expired", "viewer")
    return ViewerLease(lease_id, token, hls_url, expires_at_value, expires_at)


def lease_remaining_seconds(lease: ViewerLease) -> int:
    return max(
        0,
        int((lease.expires_at - dt.datetime.now(dt.timezone.utc)).total_seconds()),
    )


def get_post_stop_playlist(
    transport: Transport,
    report: Report,
    alias: str,
    lease: ViewerLease,
    request_timeout: float,
) -> HttpResult:
    """Comprueba vigencia y, sin trabajo intermedio, ejecuta el GET HLS."""

    required_margin = request_timeout + POST_STOP_HLS_RESERVE_SECONDS
    remaining = lease_remaining_seconds(lease)
    report.set_path(
        "viewer",
        alias,
        "remaining_seconds_before_post_stop_hls",
        value=remaining,
    )
    report.set_path(
        "isolation",
        "post_stop_hls_required_margin_seconds",
        value=required_margin,
    )
    if remaining <= required_margin:
        raise HarnessFailure("post_stop_hls_lease_margin_too_short", "stop_isolation")
    return transport.raw(
        "GET",
        lease.hls_url,
        bearer=lease.token,
        hls_only=True,
    )


def wait_playlist(
    user: AuthenticatedUser,
    session_id: str,
    lease: ViewerLease,
    timeout: float,
    poll_interval: float,
) -> HttpResult:
    transport = user.transport
    actor = user.alias
    deadline = time.monotonic() + timeout
    last_status = 0
    while time.monotonic() < deadline:
        _, status = user.call(
            "GET",
            f"/api/sessions/{session_id}/viewer-lease/{lease.lease_id}",
            target="viewer_self",
            name="poll_viewer_lease_status",
            expected={200},
            record=False,
        )
        if not isinstance(status, dict):
            raise HarnessFailure("viewer_status_response_invalid", "viewer")
        command = status.get("command")
        command_state = command.get("state") if isinstance(command, dict) else None
        if command_state in {"Failed", "Cancelled"}:
            transport.report.add_http(
                "viewer_playlist_self",
                actor,
                "viewer_self",
                "GET",
                {200},
                last_status,
                0,
                False,
            )
            raise HarnessFailure("viewer_command_failed", "viewer")

        if command_state != "Completed" or status.get("isReady") is not True:
            time.sleep(poll_interval)
            continue

        result = transport.raw(
            "GET", lease.hls_url, bearer=lease.token, hls_only=True
        )
        last_status = result.status
        if result.status == 200 and result.body.startswith(b"#EXTM3U"):
            transport.report.add_http(
                "viewer_playlist_self",
                actor,
                "viewer_self",
                "GET",
                {200},
                200,
                result.elapsed_ms,
                True,
            )
            return result
        if result.status not in {404, 425, 429, 502, 503, 504}:
            transport.report.add_http(
                "viewer_playlist_self",
                actor,
                "viewer_self",
                "GET",
                {200},
                last_status,
                result.elapsed_ms,
                False,
            )
            raise HarnessFailure("viewer_playlist_unexpected_status", "viewer")
        time.sleep(poll_interval)
    transport.report.add_http(
        "viewer_playlist_self",
        actor,
        "viewer_self",
        "GET",
        {200},
        last_status,
        0,
        False,
    )
    raise HarnessFailure("viewer_playlist_timeout", "viewer")


def wait_task(
    user: AuthenticatedUser,
    session_id: str,
    task_id: str,
    timeout: float,
    poll_interval: float,
) -> dict[str, Any]:
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        _, data = user.call(
            "GET",
            f"/api/sessions/{session_id}/tasks",
            target="tasks_self",
            name="poll_task",
            expected={200},
            record=False,
        )
        if not isinstance(data, list):
            raise HarnessFailure("task_list_response_invalid", "task")
        task = next(
            (candidate for candidate in data if isinstance(candidate, dict) and candidate.get("id") == task_id),
            None,
        )
        if task is None:
            raise HarnessFailure("task_missing_after_acceptance", "task")
        if task.get("state") in TERMINAL_TASK_STATES:
            return task
        time.sleep(poll_interval)
    raise HarnessFailure("task_completion_timeout", "task")


def read_task(
    user: AuthenticatedUser,
    session_id: str,
    task_id: str,
) -> dict[str, Any]:
    _, data = user.call(
        "GET",
        f"/api/sessions/{session_id}/tasks",
        target="tasks_self",
        name="poll_task",
        expected={200},
        record=False,
    )
    if not isinstance(data, list):
        raise HarnessFailure("task_list_response_invalid", "task")
    task = next(
        (
            candidate
            for candidate in data
            if isinstance(candidate, dict) and candidate.get("id") == task_id
        ),
        None,
    )
    if task is None:
        raise HarnessFailure("task_missing_after_acceptance", "task")
    return task


def start_task(
    user: AuthenticatedUser,
    session_id: str,
    task_type: str,
    parameters: dict[str, Any],
    barrier: threading.Barrier,
) -> tuple[str, dict[str, Any]]:
    barrier.wait(timeout=10)
    _, command = user.call(
        "POST",
        f"/api/sessions/{session_id}/tasks",
        target="tasks_self",
        name="start_parallel_task",
        expected={202},
        payload={"type": task_type, "parameters": parameters},
        idempotent=True,
    )
    if not isinstance(command, dict) or not isinstance(command.get("task"), dict):
        raise HarnessFailure("task_command_response_invalid", "task")
    return require_uuid(command["task"], "id", "task"), command["task"]


def read_task_synchronized(
    barrier: threading.Barrier,
    user: AuthenticatedUser,
    session_id: str,
    task_id: str,
) -> dict[str, Any]:
    barrier.wait(timeout=10)
    return read_task(user, session_id, task_id)


def wait_figure_while_following(
    users: dict[str, AuthenticatedUser],
    sessions: dict[str, str],
    task_ids: dict[str, str],
    timeout: float,
    poll_interval: float,
) -> tuple[dict[str, Any], dict[str, Any], bool]:
    deadline = time.monotonic() + timeout
    both_running_observed = False
    with concurrent.futures.ThreadPoolExecutor(max_workers=2) as executor:
        while time.monotonic() < deadline:
            barrier = threading.Barrier(2)
            figure_future = executor.submit(
                read_task_synchronized,
                barrier,
                users["user_a"],
                sessions["user_a"],
                task_ids["user_a"],
            )
            follow_future = executor.submit(
                read_task_synchronized,
                barrier,
                users["user_b"],
                sessions["user_b"],
                task_ids["user_b"],
            )
            figure = figure_future.result()
            follow = follow_future.result()
            figure_state = figure.get("state")
            follow_state = follow.get("state")
            if figure_state == "Running" and follow_state == "Running":
                both_running_observed = True
            if follow_state in TERMINAL_TASK_STATES:
                raise HarnessFailure("follow_task_ended_before_figure", "task")
            if figure_state in TERMINAL_TASK_STATES:
                if follow_state != "Running":
                    raise HarnessFailure(
                        "follow_task_not_running_at_figure_completion", "task"
                    )
                if not both_running_observed:
                    raise HarnessFailure("parallel_running_overlap_not_observed", "task")
                return figure, follow, both_running_observed
            time.sleep(min(poll_interval, 0.5))
    raise HarnessFailure("parallel_task_timeout", "task")


def validate_roster(data: Any, expected_count: int) -> None:
    if not isinstance(data, list) or len(data) != expected_count:
        raise HarnessFailure("robot_roster_size_mismatch", "roster")
    ordinals: set[int] = set()
    runtime_markers: set[str] = set()
    for robot in data:
        if not isinstance(robot, dict):
            raise HarnessFailure("robot_roster_response_invalid", "roster")
        ordinal = robot.get("ordinal")
        runtime_marker = robot.get("runtimeId")
        if not isinstance(ordinal, int) or not isinstance(runtime_marker, str):
            raise HarnessFailure("robot_roster_response_invalid", "roster")
        if robot.get("state") not in {"Ready", "Active"}:
            raise HarnessFailure("robot_roster_not_ready", "roster")
        ordinals.add(ordinal)
        runtime_markers.add(runtime_marker)
    if ordinals != set(range(expected_count)) or len(runtime_markers) != expected_count:
        raise HarnessFailure("robot_roster_membership_mismatch", "roster")


def create_session(
    user: AuthenticatedUser,
    robot_count: int,
    barrier: threading.Barrier,
) -> tuple[str, dict[str, Any]]:
    barrier.wait(timeout=10)
    maximum_conflict_retries = 3
    random_source = random.SystemRandom()
    for attempt in range(1, maximum_conflict_retries + 2):
        user.transport.report.record_creation_started(user.alias, attempt)
        result, data = user.call(
            "POST",
            "/api/sessions",
            target="session_self",
            name="create_session_concurrent",
            expected={202, 409},
            payload={"robotCount": robot_count},
        )
        if result.status == 202:
            user.transport.report.record_creation_attempt(user.alias, attempt, 202)
            session_id = require_uuid(data, "id", "session_create")
            if data.get("desiredRobotCount") != robot_count:
                raise HarnessFailure("session_robot_count_mismatch", "session_create")
            return session_id, data

        if attempt > maximum_conflict_retries:
            user.transport.report.record_creation_attempt(user.alias, attempt, 409)
            break

        # Un 409 durante dos altas simultaneas puede ser el conflicto de
        # serializacion previsto. El jitter evita que ambos reintentos vuelvan
        # a competir en el mismo instante.
        delay = min(
            3.0,
            0.35 * (2 ** (attempt - 1)) + random_source.uniform(0.10, 0.35),
        )
        user.transport.report.record_creation_attempt(
            user.alias,
            attempt,
            409,
            retry_delay_ms=round(delay * 1000),
        )
        time.sleep(delay)

    raise HarnessFailure("session_creation_conflict_exhausted", "session_create")


def reconcile_created_session(
    user: AuthenticatedUser,
    robot_count: int,
) -> str | None:
    """Recupera una creacion cuyo resultado de red pudo quedar indeterminado."""

    try:
        matches = list_created_sessions(
            user,
            robot_count,
            "reconcile_session_creation",
        )
    except HarnessFailure:
        return None
    if len(matches) != 1:
        return None
    return matches[0]


def list_created_sessions(
    user: AuthenticatedUser,
    robot_count: int | None,
    request_name: str,
) -> list[str]:
    """Lista sesiones activas del ensayo sin conservar datos de la cuenta."""

    _, listed = user.call(
        "GET",
        "/api/sessions",
        target="session_list_self",
        name=request_name,
        expected={200},
    )
    if not isinstance(listed, list):
        raise HarnessFailure("session_list_response_invalid", "cleanup")

    matches = []
    for item in listed:
        if not isinstance(item, dict):
            raise HarnessFailure("session_response_invalid", "cleanup")
        if (
            (robot_count is None or item.get("desiredRobotCount") == robot_count)
            and occupying_session(item)
        ):
            matches.append(
                require_uuid(item, "id", "session_reconciliation")
            )
    return matches


def wait_for_uncertain_creation(
    user: AuthenticatedUser,
    timeout: float,
    poll_interval: float,
) -> list[str]:
    """Da tiempo a que aparezca un POST cuyo resultado de red fue incierto."""

    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        try:
            matches = list_created_sessions(
                user,
                None,
                "cleanup_reconcile_session_creation",
            )
        except HarnessFailure:
            matches = []
        else:
            if matches:
                return matches
        time.sleep(poll_interval)

    # Close the gap between the last periodic observation and the deadline.
    # A failed final read is not evidence that the uncertain POST rolled back.
    return list_created_sessions(
        user,
        None,
        "cleanup_final_session_reconciliation",
    )


def issue_lease(
    user: AuthenticatedUser,
    session_id: str,
    barrier: threading.Barrier | None,
) -> ViewerLease:
    if barrier is not None:
        barrier.wait(timeout=10)
    _, data = user.call(
        "POST",
        f"/api/sessions/{session_id}/viewer-lease",
        target="viewer_self",
        name="create_viewer_lease_concurrent",
        expected={201},
        payload={"source": "Scene", "robotRuntimeId": None},
        idempotent=True,
    )
    return validate_lease(data, session_id)


def assert_status(
    report: Report,
    result: HttpResult,
    *,
    name: str,
    actor: str,
    target: str,
    expected: int,
) -> None:
    passed = result.status == expected
    report.add_http(
        name,
        actor,
        target,
        "GET",
        {expected},
        result.status,
        result.elapsed_ms,
        passed,
    )
    if not passed:
        raise HarnessFailure(f"unexpected_http_status_{name}", name)


def cleanup_sessions(
    report: Report,
    users: dict[str, AuthenticatedUser],
    sessions: dict[str, str],
    uncertain_creations: set[str],
    timeout: float,
    poll_interval: float,
) -> bool:
    report.set_path("cleanup", "attempted", value=True)
    complete = True
    for alias in ("user_a", "user_b"):
        session_id = sessions.get(alias)
        user = users.get(alias)
        if session_id is None:
            if alias in uncertain_creations:
                if user is None:
                    report.set_path(
                        "cleanup", alias, value="authentication_unavailable"
                    )
                    complete = False
                    continue
                try:
                    recovered_ids = wait_for_uncertain_creation(
                        user,
                        timeout,
                        poll_interval,
                    )
                except HarnessFailure:
                    report.set_path(
                        "cleanup", alias, value="reconciliation_failed"
                    )
                    complete = False
                    continue
                if not recovered_ids:
                    report.set_path(
                        "cleanup",
                        alias,
                        value="not_created_after_reconciliation",
                    )
                    uncertain_creations.discard(alias)
                    continue
                session_ids = recovered_ids
                sessions[alias] = recovered_ids[0]
                uncertain_creations.discard(alias)
            else:
                report.set_path("cleanup", alias, value="not_created")
                continue
        else:
            session_ids = [session_id]

        if user is None:
            report.set_path("cleanup", alias, value="authentication_unavailable")
            complete = False
            continue

        try:
            states = []
            for recovered_id in dict.fromkeys(session_ids):
                user.call(
                    "DELETE",
                    f"/api/sessions/{recovered_id}",
                    target="session_self",
                    name="cleanup_stop_session",
                    expected={200, 404},
                )
                states.append(
                    wait_session_stopped(
                        user,
                        recovered_id,
                        timeout,
                        poll_interval,
                    )
                )
            state = states[0] if len(states) == 1 else "released_all"
            report.set_path("cleanup", alias, value=state)
            report.set_path("sessions", alias, "final_state", value=state)
            if any(
                item not in {"Stopped", "released", "released_terminal"}
                for item in states
            ):
                complete = False
        except HarnessFailure:
            report.set_path("cleanup", alias, value="failed")
            complete = False
    report.set_path("cleanup", "complete", value=complete)
    return complete


def run_acceptance(args: argparse.Namespace, report: Report) -> int:
    credentials = load_credentials(report)
    binding_key = load_binding_key(args.binding_key, report)
    report.data["configuration"]["api_host"] = urllib.parse.urlsplit(API_ORIGIN).hostname
    report.data["configuration"]["deployment_commit"] = args.deployment_commit
    report.data["configuration"]["account_fingerprints"] = {
        alias: account_fingerprint(credential, binding_key)
        for alias, credential in credentials.items()
    }
    transport = Transport(report, args.request_timeout)
    users: dict[str, AuthenticatedUser] = {}
    sessions: dict[str, str] = {}
    uncertain_creations: set[str] = set()
    failure: HarnessFailure | None = None

    try:
        safe_progress("autenticando los dos usuarios de prueba")
        for alias in ("user_a", "user_b"):
            users[alias] = authenticate(alias, credentials[alias], transport)
        report.add_check("two_distinct_accounts_authenticated", "passed")

        safe_progress("verificando que las cuentas no tengan sesiones previas activas")
        for alias, user in users.items():
            _, listed = user.call(
                "GET",
                "/api/sessions",
                target="session_list_self",
                name="preflight_session_list",
                expected={200},
            )
            if not isinstance(listed, list):
                raise HarnessFailure("session_list_response_invalid", "preflight")
            if any(occupying_session(item) for item in listed):
                raise HarnessFailure(f"preexisting_live_session_{alias}", "preflight")
        report.add_check("accounts_without_preexisting_live_sessions", "passed")

        safe_progress("creando sesiones concurrentes con flotas de 3 y 7 robots")
        create_barrier = threading.Barrier(2)
        with concurrent.futures.ThreadPoolExecutor(max_workers=2) as executor:
            futures = {
                alias: executor.submit(create_session, users[alias], count, create_barrier)
                for alias, count in (("user_a", 3), ("user_b", 7))
            }
            create_errors: list[BaseException] = []
            for alias, future in futures.items():
                try:
                    session_id, _ = future.result()
                    sessions[alias] = session_id
                except BaseException as error:
                    if isinstance(error, HarnessFailure) and error.code == "network_error":
                        uncertain_creations.add(alias)
                    create_errors.append(error)
            if create_errors:
                for alias, count in (("user_a", 3), ("user_b", 7)):
                    if alias in sessions or alias not in uncertain_creations:
                        continue
                    recovered = reconcile_created_session(users[alias], count)
                    if recovered is not None:
                        sessions[alias] = recovered
                        uncertain_creations.discard(alias)
                first_error = create_errors[0]
                if isinstance(first_error, HarnessFailure):
                    raise first_error
                raise HarnessFailure("concurrent_session_creation_failed", "session_create")
        report.add_check("sessions_created_concurrently", "passed")

        safe_progress("esperando que ambas sesiones queden listas")
        with concurrent.futures.ThreadPoolExecutor(max_workers=2) as executor:
            futures = {
                "user_a": executor.submit(
                    wait_session_ready,
                    users["user_a"],
                    sessions["user_a"],
                    3,
                    args.ready_timeout,
                    args.poll_interval,
                ),
                "user_b": executor.submit(
                    wait_session_ready,
                    users["user_b"],
                    sessions["user_b"],
                    7,
                    args.ready_timeout,
                    args.poll_interval,
                ),
            }
            for alias, future in futures.items():
                state = future.result()
                report.set_path("sessions", alias, "ready", value=True)
                report.set_path("sessions", alias, "final_state", value=state)
        report.add_check("both_sessions_ready", "passed")

        safe_progress("comprobando los rosters exactos de 3 y 7 robots")
        for alias, count in (("user_a", 3), ("user_b", 7)):
            _, roster = users[alias].call(
                "GET",
                f"/api/sessions/{sessions[alias]}/robots",
                target="robots_self",
                name="read_exact_robot_roster",
                expected={200},
            )
            validate_roster(roster, count)
        report.add_check("exact_robot_rosters_3_and_7", "passed")

        safe_progress("probando aislamiento de recursos HTTP")
        anonymous_session = transport.raw("GET", f"/api/sessions/{sessions['user_a']}")
        assert_status(
            report,
            anonymous_session,
            name="unauthenticated_session_read",
            actor="anonymous",
            target="session_user_a",
            expected=401,
        )
        report.set_path("isolation", "api_unauthenticated", value=True)
        users["user_b"].call(
            "GET",
            f"/api/sessions/{sessions['user_a']}",
            target="session_user_a",
            name="cross_session_read",
            expected={404},
        )
        report.set_path("isolation", "api_cross_session", value=True)
        users["user_b"].call(
            "GET",
            f"/api/sessions/{sessions['user_a']}/tasks",
            target="tasks_user_a",
            name="cross_task_read",
            expected={404},
        )
        report.set_path("isolation", "api_cross_tasks", value=True)
        users["user_b"].call(
            "DELETE",
            f"/api/sessions/{sessions['user_a']}",
            target="session_user_a",
            name="cross_session_stop",
            expected={404},
        )
        report.set_path("isolation", "api_cross_stop", value=True)
        users["user_a"].call(
            "GET",
            f"/api/sessions/{sessions['user_a']}",
            target="session_self",
            name="session_survived_cross_stop",
            expected={200},
        )

        safe_progress("emitiendo dos leases privados y esperando sus playlists")
        lease_barrier = threading.Barrier(2)
        with concurrent.futures.ThreadPoolExecutor(max_workers=2) as executor:
            futures = {
                alias: executor.submit(issue_lease, users[alias], sessions[alias], lease_barrier)
                for alias in ("user_a", "user_b")
            }
            leases = {alias: future.result() for alias, future in futures.items()}
        for alias in leases:
            report.set_path("viewer", alias, "lease_issued", value=True)

        with concurrent.futures.ThreadPoolExecutor(max_workers=2) as executor:
            futures = {
                alias: executor.submit(
                    wait_playlist,
                    users[alias],
                    sessions[alias],
                    leases[alias],
                    args.viewer_timeout,
                    args.poll_interval,
                )
                for alias in ("user_a", "user_b")
            }
            for alias, future in futures.items():
                future.result()
                report.set_path("viewer", alias, "playlist_ready", value=True)

        previous_a_token = leases["user_a"].token
        previous_a_hls = leases["user_a"].hls_url
        leases["user_a"] = issue_lease(
            users["user_a"], sessions["user_a"], barrier=None
        )
        report.set_path("viewer", "user_a", "lease_rotated", value=True)
        revoked_a = transport.raw(
            "GET", previous_a_hls, bearer=previous_a_token, hls_only=True
        )
        assert_status(
            report,
            revoked_a,
            name="rotated_viewer_lease_revoked",
            actor="user_a",
            target="viewer_self_previous_lease",
            expected=401,
        )
        report.set_path("viewer", "user_a", "previous_lease_revoked", value=True)
        wait_playlist(
            users["user_a"],
            sessions["user_a"],
            leases["user_a"],
            args.viewer_timeout,
            args.poll_interval,
        )

        safe_progress("comprobando que los tokens HLS no cruzan sesiones")
        cross_a = transport.raw(
            "GET",
            leases["user_a"].hls_url,
            bearer=leases["user_b"].token,
            hls_only=True,
        )
        assert_status(
            report,
            cross_a,
            name="cross_hls_user_a",
            actor="user_b",
            target="viewer_user_a",
            expected=401,
        )
        report.set_path("isolation", "hls_cross_user_a", value=True)
        cross_b = transport.raw(
            "GET",
            leases["user_b"].hls_url,
            bearer=leases["user_a"].token,
            hls_only=True,
        )
        assert_status(
            report,
            cross_b,
            name="cross_hls_user_b",
            actor="user_a",
            target="viewer_user_b",
            expected=401,
        )
        report.set_path("isolation", "hls_cross_user_b", value=True)
        unauthenticated = transport.raw(
            "GET", leases["user_a"].hls_url, hls_only=True
        )
        assert_status(
            report,
            unauthenticated,
            name="unauthenticated_hls",
            actor="anonymous",
            target="viewer_user_a",
            expected=401,
        )
        report.set_path("isolation", "hls_unauthenticated", value=True)

        safe_progress("iniciando Figure y FollowLeader en paralelo")
        figure_parameters = {
            "formation_type": "triangle",
            "movement_mode": "static",
            "config": {
                "formation_type": "triangle",
                "movement_mode": "static",
                "spacing": 0.7,
            },
        }
        follow_parameters = {
            "leader_mode": "figure8",
            "config": {
                "leader_mode": "figure8",
                "follow_distance": 0.7,
                "radius": 2,
            },
        }
        task_barrier = threading.Barrier(2)
        with concurrent.futures.ThreadPoolExecutor(max_workers=2) as executor:
            futures = {
                "user_a": executor.submit(
                    start_task,
                    users["user_a"],
                    sessions["user_a"],
                    "Figure",
                    figure_parameters,
                    task_barrier,
                ),
                "user_b": executor.submit(
                    start_task,
                    users["user_b"],
                    sessions["user_b"],
                    "FollowLeader",
                    follow_parameters,
                    task_barrier,
                ),
            }
            task_ids = {alias: future.result()[0] for alias, future in futures.items()}
        report.set_path("task", "user_a", "accepted", value=True)
        report.set_path("task", "user_b", "accepted", value=True)

        terminal_task, running_follow, both_running_observed = wait_figure_while_following(
            users,
            sessions,
            task_ids,
            args.task_timeout,
            args.poll_interval,
        )
        state = terminal_task.get("state")
        outcome = terminal_task.get("outcomeState")
        progress = terminal_task.get("progress")
        report.set_path("task", "user_a", "terminal_state", value=state)
        safe_outcome = (
            outcome
            if outcome in {"Pending", "Succeeded", "Failed", "Cancelled"}
            else "invalid"
        )
        report.set_path("task", "user_a", "outcome_state", value=safe_outcome)
        if isinstance(progress, (int, float)) and not isinstance(progress, bool):
            report.set_path("task", "user_a", "progress", value=round(float(progress), 4))
        if (
            state != "Completed"
            or outcome != "Succeeded"
            or not isinstance(progress, (int, float))
            or isinstance(progress, bool)
            or float(progress) < 0.999
        ):
            raise HarnessFailure("initial_task_did_not_succeed", "task")
        figure_result = terminal_task.get("result")
        if figure_result is None:
            result_contract = "nullable_not_required"
        elif isinstance(figure_result, dict):
            result_contract = "object_present"
        else:
            raise HarnessFailure("figure_result_contract_invalid", "task")
        report.set_path("task", "user_a", "result_contract", value=result_contract)
        if running_follow.get("state") != "Running":
            raise HarnessFailure("follow_task_not_running_at_figure_completion", "task")
        figure_started_at = parse_dotnet_timestamp(
            terminal_task.get("startedAt"), "task_timing"
        )
        figure_completed_at = parse_dotnet_timestamp(
            terminal_task.get("completedAt"), "task_timing"
        )
        follow_started_at = parse_dotnet_timestamp(
            running_follow.get("startedAt"), "task_timing"
        )
        if running_follow.get("completedAt") is not None:
            raise HarnessFailure("running_follow_has_completion_timestamp", "task_timing")
        overlap_started_at = max(figure_started_at, follow_started_at)
        overlap_milliseconds = int(
            (figure_completed_at - overlap_started_at).total_seconds() * 1000
        )
        if overlap_milliseconds <= 0:
            raise HarnessFailure("parallel_task_intervals_do_not_overlap", "task_timing")
        report.set_path("task", "user_a", "timing_captured", value=True)
        report.set_path(
            "task", "user_a", "running_overlap_observed", value=both_running_observed
        )
        report.set_path("task", "user_a", "intervals_overlap", value=True)
        report.set_path(
            "task",
            "user_a",
            "overlap_milliseconds",
            value=overlap_milliseconds,
        )
        report.set_path(
            "task", "user_b", "running_while_user_a_completed", value=True
        )
        _, user_b_during_figure = users["user_b"].call(
            "GET",
            f"/api/sessions/{sessions['user_b']}",
            target="session_self",
            name="follow_session_active_at_figure_completion",
            expected={200},
        )
        if (
            not isinstance(user_b_during_figure, dict)
            or user_b_during_figure.get("state") != "Active"
        ):
            raise HarnessFailure("follow_session_not_active", "task")
        report.add_check("parallel_tasks_figure_completed_while_follow_running", "passed")

        # Se renuevan ambos leases justo antes del stop. El margen minimo evita
        # atribuir a expiracion natural el 401 que debe producir la sesion A.
        renewal_barrier = threading.Barrier(2)
        with concurrent.futures.ThreadPoolExecutor(max_workers=2) as executor:
            renewal_futures = {
                alias: executor.submit(
                    issue_lease, users[alias], sessions[alias], renewal_barrier
                )
                for alias in ("user_a", "user_b")
            }
            leases = {
                alias: future.result() for alias, future in renewal_futures.items()
            }
        with concurrent.futures.ThreadPoolExecutor(max_workers=2) as executor:
            playlist_futures = {
                alias: executor.submit(
                    wait_playlist,
                    users[alias],
                    sessions[alias],
                    leases[alias],
                    args.viewer_timeout,
                    args.poll_interval,
                )
                for alias in ("user_a", "user_b")
            }
            for future in playlist_futures.values():
                future.result()
        pre_stop_remaining: dict[str, int] = {}
        for alias in ("user_a", "user_b"):
            remaining_seconds = lease_remaining_seconds(leases[alias])
            pre_stop_remaining[alias] = remaining_seconds
            report.set_path(
                "viewer", alias, "lease_refreshed_before_stop", value=True
            )
            report.set_path(
                "viewer", alias, "remaining_seconds_before_stop", value=remaining_seconds
            )
            if remaining_seconds < 90:
                raise HarnessFailure("viewer_lease_margin_too_short", "stop_isolation")
        report.add_check("fresh_viewer_leases_before_stop", "passed", "margin_at_least_90s")

        post_stop_required_margin = (
            args.request_timeout + POST_STOP_HLS_RESERVE_SECONDS
        )
        stop_wait_budget = (
            min(pre_stop_remaining.values())
            - post_stop_required_margin
            # Reserva una solicitud para refresh eventual, otra para DELETE y
            # una ultima para el sondeo que puede cruzar su deadline local.
            - (3 * args.request_timeout)
            - 1.0
        )
        if stop_wait_budget <= 0:
            raise HarnessFailure("viewer_lease_cannot_cover_stop_budget", "stop_isolation")

        safe_progress("deteniendo la primera sesion y conservando activa la segunda")
        stop_started = time.monotonic()
        users["user_a"].call(
            "DELETE",
            f"/api/sessions/{sessions['user_a']}",
            target="session_self",
            name="stop_user_a_session",
            expected={200},
        )
        stopped_state = wait_session_stopped(
            users["user_a"],
            sessions["user_a"],
            min(
                args.cleanup_timeout,
                stop_wait_budget,
            ),
            args.poll_interval,
        )
        stop_duration_ms = round((time.monotonic() - stop_started) * 1000)
        report.set_path(
            "isolation", "stop_duration_ms", value=stop_duration_ms
        )
        if stopped_state != "Stopped":
            raise HarnessFailure("user_a_stop_timeout", "stop_isolation")
        report.set_path("sessions", "user_a", "final_state", value=stopped_state)

        for alias in ("user_a", "user_b"):
            remaining_after_stop = lease_remaining_seconds(leases[alias])
            report.set_path(
                "viewer",
                alias,
                "remaining_seconds_after_stop",
                value=remaining_after_stop,
            )
            if remaining_after_stop <= post_stop_required_margin:
                raise HarnessFailure("viewer_lease_expired_during_stop", "stop_isolation")
        report.set_path(
            "isolation", "stop_completed_before_lease_expiry", value=True
        )

        stopped_viewer = get_post_stop_playlist(
            transport,
            report,
            "user_a",
            leases["user_a"],
            args.request_timeout,
        )
        assert_status(
            report,
            stopped_viewer,
            name="stopped_session_hls_rejected",
            actor="user_a",
            target="viewer_user_a_stopped",
            expected=401,
        )
        report.set_path("isolation", "stopped_user_a_hls_rejected", value=True)

        _, user_b_session = users["user_b"].call(
            "GET",
            f"/api/sessions/{sessions['user_b']}",
            target="session_self",
            name="user_b_survived_user_a_stop",
            expected={200},
        )
        if not isinstance(user_b_session, dict) or user_b_session.get("state") != "Active":
            raise HarnessFailure("user_b_not_active_after_user_a_stop", "stop_isolation")
        follow_after_stop = read_task(
            users["user_b"], sessions["user_b"], task_ids["user_b"]
        )
        if follow_after_stop.get("state") != "Running":
            raise HarnessFailure("follow_task_not_running_after_user_a_stop", "stop_isolation")
        report.set_path("isolation", "stop_kept_user_b_active", value=True)
        remaining_viewer = get_post_stop_playlist(
            transport,
            report,
            "user_b",
            leases["user_b"],
            args.request_timeout,
        )
        assert_status(
            report,
            remaining_viewer,
            name="user_b_viewer_survived_user_a_stop",
            actor="user_b",
            target="viewer_self",
            expected=200,
        )
        if not remaining_viewer.body.startswith(b"#EXTM3U"):
            raise HarnessFailure("user_b_playlist_invalid_after_user_a_stop", "stop_isolation")
        report.set_path("isolation", "stop_kept_user_b_viewer", value=True)
        report.add_check("stop_isolation", "passed")

        users["user_b"].call(
            "POST",
            f"/api/sessions/{sessions['user_b']}/tasks/{task_ids['user_b']}/cancel",
            target="tasks_self",
            name="cancel_follow_task",
            expected={202},
            payload={},
            idempotent=True,
        )
        report.set_path("task", "user_b", "cancel_requested", value=True)
        cancelled_follow = wait_task(
            users["user_b"],
            sessions["user_b"],
            task_ids["user_b"],
            args.task_timeout,
            args.poll_interval,
        )
        cancelled_state = cancelled_follow.get("state")
        cancelled_outcome = cancelled_follow.get("outcomeState")
        report.set_path("task", "user_b", "terminal_state", value=cancelled_state)
        report.set_path(
            "task",
            "user_b",
            "outcome_state",
            value=(
                cancelled_outcome
                if cancelled_outcome in {"Pending", "Succeeded", "Failed", "Cancelled"}
                else "invalid"
            ),
        )
        if cancelled_state != "Cancelled" or cancelled_outcome != "Cancelled":
            raise HarnessFailure("follow_task_cancellation_not_confirmed", "task")
        follow_completed_at = parse_dotnet_timestamp(
            cancelled_follow.get("completedAt"), "task_timing"
        )
        if follow_completed_at <= follow_started_at:
            raise HarnessFailure("follow_task_interval_invalid", "task_timing")
        final_overlap_start = max(figure_started_at, follow_started_at)
        final_overlap_end = min(figure_completed_at, follow_completed_at)
        if final_overlap_end <= final_overlap_start:
            raise HarnessFailure("parallel_task_intervals_do_not_overlap", "task_timing")
        report.set_path("task", "user_b", "timing_captured", value=True)
        report.add_check("follow_task_cancelled", "passed")
    except HarnessFailure as error:
        failure = error
    except (KeyboardInterrupt, concurrent.futures.CancelledError):
        failure = HarnessFailure("execution_interrupted", "runtime")
    except Exception:
        # No se imprime la excepcion: podria contener una URL o un valor sensible.
        failure = HarnessFailure("unexpected_internal_error", "runtime")
    finally:
        safe_progress("ejecutando limpieza de las sesiones creadas")
        cleanup_complete = cleanup_sessions(
            report,
            users,
            sessions,
            uncertain_creations,
            args.cleanup_timeout,
            args.poll_interval,
        )
        for user in users.values():
            user.account_marker = 0
            user.jwt_token = ""
            user.refresh_token = ""
        credentials.clear()

    if failure is None and cleanup_complete:
        report.finish("passed")
        return 0
    if failure is None:
        failure = HarnessFailure("cleanup_incomplete", "cleanup")
    elif not cleanup_complete:
        failure = HarnessFailure("test_failed_and_cleanup_incomplete", "cleanup")
    report.finish("failed", failure)
    return 1 if cleanup_complete else 3


def plan_report(args: argparse.Namespace, report: Report) -> int:
    credentials = load_credentials(report)
    binding_key = load_binding_key(args.binding_key, report)
    credentials.clear()
    del binding_key
    report.data["configuration"]["production_execution"] = False
    report.add_check("network_guard", "passed", "no_network_requests")
    report.add_check("planned_two_user_acceptance", "passed")
    report.set_path("cleanup", "attempted", value=False)
    report.set_path("cleanup", "complete", value=True)
    report.finish("planned")
    return 0


def assert_report_is_sanitized(value: Any) -> None:
    forbidden_keys = {
        "id",
        "session_id",
        "task_id",
        "lease_id",
        "account_id",
        "url",
        "token",
        "email",
        "password",
    }

    def visit(item: Any) -> None:
        if isinstance(item, dict):
            for key, child in item.items():
                if key.lower() in forbidden_keys:
                    raise HarnessFailure("report_redaction_guard", "report")
                visit(child)
        elif isinstance(item, list):
            for child in item:
                visit(child)
        elif isinstance(item, str):
            lowered = item.lower()
            if "http://" in lowered or "https://" in lowered or "@" in item:
                raise HarnessFailure("report_redaction_guard", "report")
            if UUID_PATTERN.search(item):
                raise HarnessFailure("report_redaction_guard", "report")

    visit(value)


def write_report(path: Path, report: dict[str, Any]) -> None:
    target = path.expanduser().resolve()
    target.parent.mkdir(parents=True, exist_ok=True)
    serialized = json.dumps(report, indent=2, sort_keys=True) + "\n"
    fd, temporary_name = tempfile.mkstemp(prefix=".robotswarm-e2e-", dir=target.parent)
    try:
        os.fchmod(fd, 0o600)
        with os.fdopen(fd, "w", encoding="utf-8") as stream:
            stream.write(serialized)
            stream.flush()
            os.fsync(stream.fileno())
        os.replace(temporary_name, target)
    finally:
        try:
            os.unlink(temporary_name)
        except FileNotFoundError:
            pass


def emit(report: Report, output: Path | None) -> None:
    try:
        assert_report_is_sanitized(report.data)
        payload = report.data
    except HarnessFailure:
        payload = {
            "schema_version": 1,
            "result": "failed",
            "finished_at": utc_now(),
            "error": {"code": "report_redaction_guard", "phase": "report"},
        }
    if output is not None:
        write_report(output, payload)
    print(json.dumps(payload, indent=2, sort_keys=True), flush=True)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Arnes multiusuario de RobotSwarm con salida saneada.",
    )
    mode = parser.add_mutually_exclusive_group(required=True)
    mode.add_argument(
        "--plan",
        action="store_true",
        help="valida el archivo local y muestra el plan sin usar la red",
    )
    mode.add_argument(
        "--execute-production",
        action="store_true",
        help="autoriza expresamente las solicitudes y mutaciones de produccion",
    )
    parser.add_argument("--output", type=Path, help="archivo JSON saneado (modo 0600)")
    parser.add_argument("--binding-key", type=Path, default=BINDING_KEY_PATH)
    parser.add_argument(
        "--deployment-commit",
        help="SHA completo del backend verificado en el host antes de la prueba",
    )
    parser.add_argument("--request-timeout", type=float, default=20.0)
    parser.add_argument("--ready-timeout", type=float, default=480.0)
    parser.add_argument("--task-timeout", type=float, default=360.0)
    parser.add_argument("--viewer-timeout", type=float, default=240.0)
    parser.add_argument("--cleanup-timeout", type=float, default=300.0)
    parser.add_argument("--poll-interval", type=float, default=2.0)
    args = parser.parse_args()
    if args.execute_production and not re.fullmatch(r"[0-9a-f]{40}", args.deployment_commit or ""):
        parser.error("--deployment-commit debe ser un SHA Git completo en produccion")
    for name in (
        "request_timeout",
        "ready_timeout",
        "task_timeout",
        "viewer_timeout",
        "cleanup_timeout",
        "poll_interval",
    ):
        if getattr(args, name) <= 0:
            parser.error(f"--{name.replace('_', '-')} debe ser mayor que cero")
    return args


def main() -> int:
    args = parse_args()
    report = Report()
    try:
        exit_code = plan_report(args, report) if args.plan else run_acceptance(args, report)
    except HarnessFailure as error:
        report.finish("failed", error)
        exit_code = 2
    except Exception:
        report.finish("failed", HarnessFailure("unexpected_internal_error", "runtime"))
        exit_code = 2
    emit(report, args.output)
    return exit_code


if __name__ == "__main__":
    raise SystemExit(main())
