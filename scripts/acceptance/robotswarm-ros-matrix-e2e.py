#!/usr/bin/env python3
"""Run the complete ROS/Gazebo matrix through one visible production browser.

This is deliberately an operator-run acceptance harness.  It uses account A
only to allocate a fresh worker session and a real HLS viewer, then executes a
single ROS scenario inside that session's immutable container.  Every case is
cleaned before the next one is allowed to start.

The script never prints or records session, lease, container, network, token,
password, or email identifiers.  Those values exist only long enough to bind
the browser, Docker resources, and viewer evidence to the same live session.
"""

from __future__ import annotations

import argparse
import contextlib
import ctypes
import dataclasses
import datetime as dt
import errno
import fcntl
import hashlib
import importlib.util
import ipaddress
import json
import math
import os
import re
import selectors
import shutil
import signal
import stat
import subprocess
import sys
import tempfile
import threading
import time
import uuid
from pathlib import Path
from types import ModuleType
from typing import Any, Callable, Iterable, Sequence


SCRIPT_DIR = Path(__file__).resolve().parent
VISIBLE_DRIVER_PATH = SCRIPT_DIR / "robotswarm-visible-e2e.py"
DEFAULT_URL = "https://rs.zerav.la/apps/GTS/realtime"
DEFAULT_VIEWER_RUNTIME = Path(
    os.environ.get(
        "ROBOTSWARM_VIEWER_RUNTIME_DIR",
        str(
            Path(os.environ["XDG_RUNTIME_DIR"]) / "robotswarm-viewer"
            if os.environ.get("XDG_RUNTIME_DIR")
            else Path(f"/tmp/robotswarm-viewer-{os.getuid()}")
        ),
    )
)
LIVE_ACCEPTANCE_PATH = (
    "/catkin_ws/src/robot_swarm_bridge/test/robotswarm_live_acceptance.py"
)
GUI_PREFLIGHT_PATH = (
    "/catkin_ws/src/robot_swarm_bridge/scripts/gazebo_gui_preflight.py"
)
GUI_PROBE_PLUGIN_PATH = (
    "/catkin_ws/devel/lib/librobotswarm_gazebo_gui_probe.so"
)
MINIMUM_STARTUP_SCENE_FPS = 45.0
MINIMUM_REAL_TIME_FACTOR = 2.90
ACTIVE_PROBE_WARMUP_SECONDS = 2.0
ACTIVE_PROBE_SAMPLE_SECONDS = 5.0
# The previous active probe exhausted its 25 s budget and took about 28 s
# including teardown.  Match the publisher's bounded 45 s startup allowance;
# the 2 s warm-up and 5 s measurement remain unchanged.
ACTIVE_PROBE_TIMEOUT_SECONDS = 45.0
ACTIVE_SCENARIO_VIDEO_SECONDS = 5.0
MATRIX_FORMATION_ACTIVE_SECONDS = 75.0
MATRIX_FOLLOW_ACTIVE_SECONDS = 75.0
ACTIVE_PROBE_TOKEN_ENV = "ROBOTSWARM_ACTIVE_PROBE_TOKEN"
ACTIVE_PROBE_ATTESTATION_PREFIX = "ROBOTSWARM_GUI_REPORT_ATTESTATION "
PIDFD_OPEN_SYSCALL_X86_64 = 434
PIDFD_SEND_SIGNAL_SYSCALL_X86_64 = 424
TARGET_BROWSER_VIDEO_FPS = 30.0
MINIMUM_BROWSER_VIDEO_FPS = 27.0
MAXIMUM_BROWSER_DROPPED_RATIO = 0.10
MAXIMUM_CHILD_OUTPUT_BYTES = 16 * 1024 * 1024
MAXIMUM_RENDER_REPORT_BYTES = 64 * 1024
MAXIMUM_PREFLIGHT_SCRIPT_BYTES = 512 * 1024
MAXIMUM_PROBE_PLUGIN_BYTES = 32 * 1024 * 1024
CHILD_READ_CHUNK_BYTES = 64 * 1024
CHILD_GRACEFUL_STOP = (
    (signal.SIGINT, 25.0),
    (signal.SIGTERM, 10.0),
    (signal.SIGKILL, 10.0),
)

MANAGED_LABEL = "io.robotswarm.managed"
SESSION_LABEL = "io.robotswarm.session-id"
WORKER_LABEL = "io.robotswarm.worker-id"
IMAGE_VERSION_LABEL = "io.robotswarm.image-version"
IMAGE_REVISION_LABEL = "org.opencontainers.image.revision"

UUID_PATTERN = re.compile(
    r"\b[0-9a-f]{8}-[0-9a-f]{4}-[1-5][0-9a-f]{3}-"
    r"[89ab][0-9a-f]{3}-[0-9a-f]{12}\b",
    re.IGNORECASE,
)
COMPACT_UUID_PATTERN = re.compile(r"(?<![0-9a-f])[0-9a-f]{32}(?![0-9a-f])", re.IGNORECASE)
EMAIL_PATTERN = re.compile(r"[A-Za-z0-9._%+-]+@[A-Za-z0-9.-]+\.[A-Za-z]{2,}")
JWT_PATTERN = re.compile(r"\beyJ[A-Za-z0-9_-]{8,}\.[A-Za-z0-9_-]{8,}\.[A-Za-z0-9_-]{8,}\b")
SECRET_QUERY_PATTERN = re.compile(
    r"([?&](?:token|key|secret|auth|jwt)=[^&#\s]+)", re.IGNORECASE
)
SHA_PATTERN = re.compile(r"[0-9a-f]{40}")
IMAGE_ID_PATTERN = re.compile(r"sha256:([0-9a-f]{64})")

SENSITIVE_REPORT_KEYS = {
    "authorization",
    "containerid",
    "email",
    "leaseid",
    "networkid",
    "password",
    "sessionid",
    "token",
}


@dataclasses.dataclass(frozen=True)
class ScenarioSpec:
    name: str
    behavior: str
    robot_count: int


# Keep this in the same order as robotswarm_live_acceptance.py.  The contract
# test compares both catalogs, so additions cannot silently skip this matrix.
SCENARIOS = (
    ScenarioSpec("formation_triangle_n3", "formation", 3),
    ScenarioSpec("formation_square_n5", "formation", 5),
    ScenarioSpec("formation_A_n7", "formation", 7),
    ScenarioSpec("formation_V_n8", "formation", 8),
    ScenarioSpec("formation_diamond_n9", "formation", 9),
    ScenarioSpec("formation_S_n10", "formation", 10),
    ScenarioSpec("follow_circular_n3", "follow", 3),
    ScenarioSpec("follow_square_n6", "follow", 6),
    ScenarioSpec("follow_figure8_n10", "follow", 10),
    ScenarioSpec("transport_grf_n1", "transport", 1),
    ScenarioSpec("transport_grf_n2", "transport", 2),
    ScenarioSpec("transport_grf_n3", "transport", 3),
    ScenarioSpec("transport_grf_n4", "transport", 4),
    ScenarioSpec("transport_grf_n10", "transport", 10),
)
SCENARIOS_BY_NAME = {item.name: item for item in SCENARIOS}


class MatrixError(RuntimeError):
    """Expected, report-safe acceptance failure."""


class CleanupError(MatrixError):
    """A case could not prove that all of its resources were removed."""


@dataclasses.dataclass(frozen=True)
class ProcessOutput:
    returncode: int
    stdout: str
    stderr: str


@dataclasses.dataclass(frozen=True)
class ContainerHandle:
    identifier: str
    image_version: str
    private_ip: str = ""
    network_gateway: str = ""


@dataclasses.dataclass(frozen=True)
class RosEvidence:
    result: dict[str, Any]
    summary: dict[str, Any]
    result_sha256: str
    summary_sha256: str
    returncode: int


@dataclasses.dataclass(frozen=True)
class GazeboGuiEvidence:
    document: dict[str, Any]
    raw: bytes
    adapter: str
    average_fps: float
    post_render_fps: float
    real_time_factor: float


@dataclasses.dataclass(frozen=True)
class ActiveProbeAttestation:
    sha256: str
    process_id: int
    process_start_ticks: int


@dataclasses.dataclass(frozen=True)
class BoundViewerPublisher:
    process_path: Path
    executable: Path
    environment: dict[str, str]


@dataclasses.dataclass(frozen=True)
class BoundViewerGzclient:
    process_path: Path
    executable: Path
    environment: dict[str, str]


@dataclasses.dataclass(frozen=True)
class ActiveProbeRuntime:
    command_prefix: tuple[str, ...]
    environment: dict[str, str]
    gzclient: str
    primary_environment: dict[str, str] | None = None


@dataclasses.dataclass
class TimedCommand:
    started: float | None = None
    finished: float | None = None
    output: ProcessOutput | None = None
    error: BaseException | None = None


def utc_now() -> str:
    return dt.datetime.now(dt.timezone.utc).isoformat(timespec="seconds").replace(
        "+00:00", "Z"
    )


def load_visible_driver() -> ModuleType:
    """Load the established CDP driver without importing it during unit tests."""
    module_name = "robotswarm_visible_e2e_for_ros_matrix"
    existing = sys.modules.get(module_name)
    if existing is not None:
        return existing
    spec = importlib.util.spec_from_file_location(module_name, VISIBLE_DRIVER_PATH)
    if spec is None or spec.loader is None:
        raise MatrixError("The visible Chrome driver could not be loaded")
    module = importlib.util.module_from_spec(spec)
    sys.modules[module_name] = module
    try:
        spec.loader.exec_module(module)
    except SystemExit as exc:
        sys.modules.pop(module_name, None)
        raise MatrixError("The visible Chrome driver dependencies are unavailable") from exc
    return module


def clean_site_url(value: str) -> str:
    from urllib.parse import urlsplit, urlunsplit

    parsed = urlsplit(value)
    return urlunsplit((parsed.scheme, parsed.netloc, parsed.path, "", ""))


def normalize_report_key(value: Any) -> str:
    return re.sub(r"[^a-z0-9]", "", str(value).lower())


def is_sensitive_report_key(value: Any) -> bool:
    normalized = normalize_report_key(value)
    if normalized in SENSITIVE_REPORT_KEYS:
        return True
    if any(
        marker in normalized
        for marker in ("authorization", "credential", "email", "password", "secret", "token")
    ):
        return True
    return any(
        marker in normalized
        for marker in (
            "apikey",
            "bindingkey",
            "containerid",
            "idempotencykey",
            "leaseid",
            "networkid",
            "sessionid",
        )
    )


def redact_text(value: str, secrets: Iterable[str] = ()) -> str:
    result = value
    for secret in sorted((item for item in secrets if item), key=len, reverse=True):
        result = result.replace(secret, "[REDACTED]")
    result = re.sub(r"(?i)Bearer\s+[A-Za-z0-9._~+/=-]+", "Bearer [REDACTED]", result)
    result = JWT_PATTERN.sub("[JWT REDACTED]", result)
    result = EMAIL_PATTERN.sub("[EMAIL REDACTED]", result)
    result = UUID_PATTERN.sub("[UUID REDACTED]", result)
    result = COMPACT_UUID_PATTERN.sub("[ID REDACTED]", result)
    result = SECRET_QUERY_PATTERN.sub("?[REDACTED]", result)
    return result[:4000]


def sanitize_report_value(value: Any, secrets: Iterable[str] = ()) -> Any:
    """Remove identifiers from child JSON without changing numeric evidence."""
    if isinstance(value, dict):
        clean: dict[str, Any] = {}
        for key, item in value.items():
            if is_sensitive_report_key(key):
                continue
            safe_key = redact_text(str(key), secrets)
            clean[safe_key] = sanitize_report_value(item, secrets)
        return clean
    if isinstance(value, (list, tuple)):
        return [sanitize_report_value(item, secrets) for item in value]
    if isinstance(value, str):
        return redact_text(value, secrets)
    return value


def assert_report_safe(value: Any, secrets: Iterable[str] = ()) -> None:
    """Fail closed if a future schema tries to persist a private identifier."""
    secret_values = tuple(item for item in secrets if item)

    def visit(item: Any) -> None:
        if isinstance(item, dict):
            for key, child in item.items():
                if is_sensitive_report_key(key):
                    raise MatrixError("The report contains a sensitive field")
                visit(str(key))
                visit(child)
            return
        if isinstance(item, (list, tuple)):
            for child in item:
                visit(child)
            return
        if not isinstance(item, str):
            return
        if any(secret in item for secret in secret_values):
            raise MatrixError("The report contains a credential value")
        if (
            UUID_PATTERN.search(item)
            or COMPACT_UUID_PATTERN.search(item)
            or EMAIL_PATTERN.search(item)
            or JWT_PATTERN.search(item)
            or re.search(r"(?i)Bearer\s+[A-Za-z0-9._~+/=-]+", item)
            or SECRET_QUERY_PATTERN.search(item)
        ):
            raise MatrixError("The report contains a private identifier")

    visit(value)


def validate_secure_directory(path: Path) -> None:
    if path.exists() or path.is_symlink():
        metadata = path.lstat()
        if stat.S_ISLNK(metadata.st_mode) or not stat.S_ISDIR(metadata.st_mode):
            raise MatrixError("The output directory must be a real directory")
        if metadata.st_uid != os.getuid():
            raise MatrixError("The output directory has a different owner")
        if stat.S_IMODE(metadata.st_mode) != 0o700:
            raise MatrixError("The output directory must have mode 0700")
        return
    path.mkdir(parents=True, mode=0o700)
    metadata = path.lstat()
    if metadata.st_uid != os.getuid() or stat.S_IMODE(metadata.st_mode) != 0o700:
        raise MatrixError("The output directory could not be created securely")


def validate_output_target(path: Path) -> None:
    validate_secure_directory(path.parent)
    if not (path.exists() or path.is_symlink()):
        return
    metadata = path.lstat()
    if (
        stat.S_ISLNK(metadata.st_mode)
        or not stat.S_ISREG(metadata.st_mode)
        or metadata.st_uid != os.getuid()
        or stat.S_IMODE(metadata.st_mode) != 0o600
    ):
        raise MatrixError("The existing output report is not a private owned file")


def write_bytes_secure(path: Path, payload: bytes) -> None:
    validate_secure_directory(path.parent)
    descriptor, temporary_name = tempfile.mkstemp(prefix=f".{path.name}.", dir=str(path.parent))
    try:
        os.fchmod(descriptor, 0o600)
        with os.fdopen(descriptor, "wb") as stream:
            stream.write(payload)
            stream.flush()
            os.fsync(stream.fileno())
        os.replace(temporary_name, path)
    except Exception:
        with contextlib.suppress(OSError):
            os.close(descriptor)
        with contextlib.suppress(OSError):
            os.unlink(temporary_name)
        raise


def write_json_secure(path: Path, document: Any, secrets: Iterable[str] = ()) -> None:
    safe_document = sanitize_report_value(document, secrets)
    assert_report_safe(safe_document, secrets)
    payload = (
        json.dumps(
            safe_document,
            ensure_ascii=False,
            indent=2,
            sort_keys=True,
            allow_nan=False,
        )
        + "\n"
    ).encode("utf-8")
    write_bytes_secure(path, payload)


def strict_json_loads(value: str) -> Any:
    def reject_constant(name: str) -> Any:
        raise ValueError(f"non-finite JSON constant: {name}")

    return json.loads(value, parse_constant=reject_constant)


class HostRunLock:
    """Prevent two account-A matrix runs from racing on the same host."""

    def __init__(self, path: Path = Path("/tmp/robotswarm-ros-matrix-e2e.lock")) -> None:
        self.path = path
        self.stream: Any = None

    def __enter__(self) -> "HostRunLock":
        flags = os.O_RDWR | os.O_CREAT | os.O_CLOEXEC
        if hasattr(os, "O_NOFOLLOW"):
            flags |= os.O_NOFOLLOW
        descriptor = os.open(self.path, flags, 0o600)
        metadata = os.fstat(descriptor)
        if (
            not stat.S_ISREG(metadata.st_mode)
            or metadata.st_uid != os.getuid()
            or stat.S_IMODE(metadata.st_mode) != 0o600
        ):
            os.close(descriptor)
            raise MatrixError("The matrix host lock is not a private owned file")
        self.stream = os.fdopen(descriptor, "a+b", buffering=0)
        try:
            fcntl.flock(self.stream.fileno(), fcntl.LOCK_EX | fcntl.LOCK_NB)
        except BlockingIOError as exc:
            self.stream.close()
            self.stream = None
            raise MatrixError("Another ROS matrix run already owns account A") from exc
        return self

    def __exit__(self, _kind: Any, _value: Any, _traceback: Any) -> None:
        if self.stream is None:
            return
        fcntl.flock(self.stream.fileno(), fcntl.LOCK_UN)
        self.stream.close()
        self.stream = None


def _signal_process_group(process: subprocess.Popen[str], number: int) -> None:
    if process.poll() is not None:
        return
    with contextlib.suppress(ProcessLookupError):
        os.killpg(process.pid, number)


def run_command(
    arguments: Sequence[str],
    *,
    timeout: float,
    stop_event: threading.Event | None = None,
    cancel_event: threading.Event | None = None,
    environment: dict[str, str] | None = None,
    cwd: Path | None = None,
    on_started: Callable[[float], None] | None = None,
    on_exited: Callable[[float], None] | None = None,
) -> ProcessOutput:
    """Capture both child streams without ever buffering beyond the hard cap."""
    process = subprocess.Popen(
        list(arguments),
        stdin=subprocess.DEVNULL,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        start_new_session=True,
        env=environment,
        cwd=str(cwd) if cwd is not None else None,
    )
    if on_started is not None:
        on_started(time.monotonic())
    if process.stdout is None or process.stderr is None:
        _signal_process_group(process, signal.SIGKILL)
        raise MatrixError("A child process did not expose bounded output pipes")

    selector = selectors.DefaultSelector()
    streams = {
        "stdout": (process.stdout, bytearray()),
        "stderr": (process.stderr, bytearray()),
    }
    for name, (stream, _buffer) in streams.items():
        os.set_blocking(stream.fileno(), False)
        selector.register(stream, selectors.EVENT_READ, data=name)

    command_deadline = time.monotonic() + timeout
    stop_reason: str | None = None
    stop_plan: tuple[tuple[int, float], ...] = ()
    stop_stage = -1
    stop_deadline: float | None = None
    retained_bytes = 0
    exit_notified = False

    def begin_stop(reason: str) -> None:
        nonlocal stop_reason, stop_plan, stop_stage, stop_deadline
        if stop_reason is not None:
            return
        stop_reason = reason
        stop_plan = (
            ((signal.SIGKILL, 10.0),)
            if reason == "overflow"
            else CHILD_GRACEFUL_STOP
        )
        stop_stage = 0
        number, grace = stop_plan[stop_stage]
        _signal_process_group(process, number)
        stop_deadline = time.monotonic() + grace

    try:
        while True:
            now = time.monotonic()
            if stop_reason is None:
                if stop_event is not None and stop_event.is_set():
                    begin_stop("interrupted")
                elif cancel_event is not None and cancel_event.is_set():
                    begin_stop("cancelled")
                elif now >= command_deadline:
                    begin_stop("timeout")
            elif (
                process.poll() is None
                and stop_deadline is not None
                and now >= stop_deadline
            ):
                stop_stage += 1
                if stop_stage >= len(stop_plan):
                    raise MatrixError("A bounded child process could not be stopped")
                number, grace = stop_plan[stop_stage]
                _signal_process_group(process, number)
                stop_deadline = now + grace

            events = selector.select(0.25)
            for key, _mask in events:
                stream, buffer = streams[str(key.data)]
                try:
                    chunk = os.read(stream.fileno(), CHILD_READ_CHUNK_BYTES)
                except BlockingIOError:
                    continue
                if not chunk:
                    with contextlib.suppress(Exception):
                        selector.unregister(stream)
                    continue
                room = max(0, MAXIMUM_CHILD_OUTPUT_BYTES - retained_bytes)
                if len(chunk) > room:
                    if room:
                        buffer.extend(chunk[:room])
                        retained_bytes += room
                    begin_stop("overflow")
                    continue
                buffer.extend(chunk)
                retained_bytes += len(chunk)

            if process.poll() is not None:
                if on_exited is not None and not exit_notified:
                    on_exited(time.monotonic())
                    exit_notified = True
                if not selector.get_map():
                    break
                if not events:
                    # Once the process has exited, an unready descriptor cannot
                    # produce more bytes.  Closing it also prevents inherited
                    # descriptors in an escaped descendant from hanging us.
                    break
    finally:
        if process.poll() is None:
            _signal_process_group(process, signal.SIGKILL)
            with contextlib.suppress(subprocess.TimeoutExpired):
                process.wait(timeout=10)
        selector.close()
        process.stdout.close()
        process.stderr.close()

    if stop_reason == "interrupted":
        raise KeyboardInterrupt
    if stop_reason == "timeout":
        raise MatrixError("A bounded child process timed out")
    if stop_reason == "cancelled":
        raise MatrixError("A bounded child process was cancelled for peer failure")
    if stop_reason == "overflow":
        raise MatrixError("A child process exceeded the bounded output limit")
    stdout = bytes(streams["stdout"][1]).decode("utf-8", errors="replace")
    stderr = bytes(streams["stderr"][1]).decode("utf-8", errors="replace")
    return ProcessOutput(int(process.returncode or 0), stdout, stderr)


def build_acceptance_command(
    docker_executable: str,
    container_identifier: str,
    scenario: str,
    task_id: str,
    run_token: str,
) -> list[str]:
    if scenario not in SCENARIOS_BY_NAME:
        raise MatrixError("An unknown ROS scenario was requested")
    if not re.fullmatch(r"matrix-[0-9a-f]{32}", task_id):
        raise MatrixError("The matrix task correlation identifier is invalid")
    if not re.fullmatch(r"[0-9a-f]{32}", run_token):
        raise MatrixError("The matrix child token is invalid")
    state_file, lock_file = acceptance_supervisor_paths(run_token)
    bootstrap = (
        "set -u; "
        "source /opt/ros/noetic/setup.bash; "
        "source /catkin_ws/devel/setup.bash; "
        "exec python3 -c \"$1\" run \"$2\" \"$3\" \"$4\" \"$5\" 0 -- \"${@:6}\""
    )
    command = [
        docker_executable,
        "exec",
        container_identifier,
        "/bin/bash",
        "-lc",
        bootstrap,
        "robotswarm-live-acceptance",
        ACCEPTANCE_SUPERVISOR_SOURCE,
        state_file,
        lock_file,
        task_id,
        LIVE_ACCEPTANCE_PATH,
        "/usr/bin/python3",
        LIVE_ACCEPTANCE_PATH,
        "--scenario",
        scenario,
        "--task-id",
        task_id,
        "--min-rtf",
        f"{MINIMUM_REAL_TIME_FACTOR:.2f}",
    ]
    if SCENARIOS_BY_NAME[scenario].behavior == "formation":
        command.extend(
            [
                "--formation-active-seconds",
                f"{MATRIX_FORMATION_ACTIVE_SECONDS:.1f}",
            ]
        )
    if SCENARIOS_BY_NAME[scenario].behavior == "follow":
        command.extend(
            [
                "--follow-active-seconds",
                f"{MATRIX_FOLLOW_ACTIVE_SECONDS:.1f}",
            ]
        )
    return command


def acceptance_supervisor_paths(run_token: str) -> tuple[str, str]:
    if not re.fullmatch(r"[0-9a-f]{32}", run_token):
        raise MatrixError("The matrix child token is invalid")
    prefix = f"/tmp/robotswarm-matrix-{run_token}"
    return prefix + ".state", prefix + ".lock"


ACCEPTANCE_SUPERVISOR_SOURCE = r'''\
import contextlib
import fcntl
import json
import math
import os
import signal
import stat
import subprocess
import sys
import time

mode, state_path, lock_path, task_id, expected_script = sys.argv[1:6]
owner_euid = os.geteuid()
terminal_states = {"aborted", "failed", "finished"}

def fail(code=4):
    raise SystemExit(code)

def secure_descriptor(path, flags, create=False):
    options = flags | os.O_CLOEXEC | os.O_NOFOLLOW
    if create:
        options |= os.O_CREAT | os.O_EXCL
    descriptor = os.open(path, options, 0o600)
    metadata = os.fstat(descriptor)
    if (
        not stat.S_ISREG(metadata.st_mode)
        or metadata.st_uid != owner_euid
        or stat.S_IMODE(metadata.st_mode) != 0o600
    ):
        os.close(descriptor)
        fail()
    return descriptor

def read_state(descriptor):
    os.lseek(descriptor, 0, os.SEEK_SET)
    raw = os.read(descriptor, 8192)
    if len(raw) == 0 or len(raw) >= 8192:
        fail()
    try:
        value = json.loads(raw.decode("utf-8"))
    except (UnicodeDecodeError, ValueError):
        fail()
    if (
        not isinstance(value, dict)
        or value.get("schema") != 1
        or value.get("task") != task_id
        or value.get("script") != expected_script
        or value.get("owner_euid") != owner_euid
    ):
        fail()
    return value

def write_state(descriptor, value):
    payload = json.dumps(
        value, sort_keys=True, separators=(",", ":")
    ).encode("utf-8")
    if len(payload) >= 8192:
        fail()
    temporary_path = "{}.tmp.{}.{}".format(
        state_path, os.getpid(), time.monotonic_ns()
    )
    temporary_descriptor = None
    replaced = False
    try:
        temporary_descriptor = secure_descriptor(
            temporary_path, os.O_RDWR, create=True
        )
        offset = 0
        while offset < len(payload):
            try:
                written = os.write(
                    temporary_descriptor, payload[offset:]
                )
            except InterruptedError:
                continue
            if written <= 0:
                fail()
            offset += written
        os.fsync(temporary_descriptor)
        opened = os.fstat(descriptor)
        current = os.stat(state_path, follow_symlinks=False)
        if (
            opened.st_dev != current.st_dev
            or opened.st_ino != current.st_ino
            or not stat.S_ISREG(current.st_mode)
            or current.st_uid != owner_euid
            or stat.S_IMODE(current.st_mode) != 0o600
        ):
            fail()
        os.replace(temporary_path, state_path)
        replaced = True
    except BaseException:
        if temporary_descriptor is not None:
            os.close(temporary_descriptor)
        if not replaced:
            try:
                os.unlink(temporary_path)
            except FileNotFoundError:
                pass
        raise
    with contextlib.suppress(OSError):
        os.close(descriptor)
    return temporary_descriptor

def open_locked_state():
    lock_descriptor = secure_descriptor(lock_path, os.O_RDWR)
    fcntl.flock(lock_descriptor, fcntl.LOCK_EX)
    try:
        state_descriptor = secure_descriptor(state_path, os.O_RDWR)
    except BaseException:
        fcntl.flock(lock_descriptor, fcntl.LOCK_UN)
        os.close(lock_descriptor)
        raise
    return lock_descriptor, state_descriptor

def close_locked(lock_descriptor, state_descriptor):
    os.close(state_descriptor)
    fcntl.flock(lock_descriptor, fcntl.LOCK_UN)
    os.close(lock_descriptor)

def process_arguments(pid):
    try:
        with open("/proc/{}/cmdline".format(pid), "rb") as stream:
            raw = stream.read(65536)
        with open("/proc/{}/status".format(pid), "r", encoding="ascii") as stream:
            status_text = stream.read(65536)
    except (FileNotFoundError, ProcessLookupError):
        return None
    arguments = [
        item.decode("utf-8", "replace")
        for item in raw.split(b"\0") if item
    ]
    effective_uid = None
    for line in status_text.splitlines():
        if line.startswith("Uid:"):
            fields = line.split()
            if len(fields) >= 3 and fields[2].isdigit():
                effective_uid = int(fields[2])
            break
    if effective_uid != owner_euid:
        return None
    return arguments

def expected_process(pid):
    arguments = process_arguments(pid)
    return bool(
        arguments
        and expected_script in arguments
        and task_id in arguments
    )

def process_start_time(pid):
    try:
        os.kill(pid, 0)
    except ProcessLookupError:
        return None
    try:
        with open("/proc/{}/stat".format(pid), "r", encoding="ascii") as stream:
            fields = stream.read(4096).rsplit(")", 1)[1].strip().split()
    except (FileNotFoundError, IndexError):
        return None
    if len(fields) < 20 or fields[0] == "Z":
        return None
    try:
        return int(fields[19])
    except ValueError:
        return None

def process_exists(pid):
    return process_start_time(pid) is not None

def stop_spawned_child(child):
    try:
        os.killpg(child.pid, signal.SIGKILL)
    except ProcessLookupError:
        pass
    try:
        child.wait(timeout=5.0)
    except subprocess.TimeoutExpired:
        try:
            os.killpg(child.pid, signal.SIGKILL)
        except ProcessLookupError:
            pass
        child.wait(timeout=5.0)

def unlink_current_state(descriptor):
    opened = os.fstat(descriptor)
    current = os.stat(state_path, follow_symlinks=False)
    if opened.st_dev != current.st_dev or opened.st_ino != current.st_ino:
        fail()
    os.unlink(state_path)

def stop_expected_supervisor(pid, expected_identity):
    identity = process_start_time(pid)
    if identity is None:
        return
    if identity != expected_identity or not expected_process(pid):
        fail()
    for number, grace in ((signal.SIGKILL, 2.0),):
        if process_start_time(pid) != identity:
            return
        try:
            os.kill(pid, number)
        except ProcessLookupError:
            return
        deadline = time.monotonic() + grace
        while time.monotonic() < deadline:
            if process_start_time(pid) != identity:
                return
            time.sleep(0.02)
    if process_start_time(pid) == identity:
        raise SystemExit(5)

if mode == "prepare":
    if len(sys.argv) != 6:
        fail()
    lock_descriptor = None
    state_descriptor = None
    created_lock = False
    created_state = False
    try:
        lock_descriptor = secure_descriptor(
            lock_path, os.O_RDWR, create=True
        )
        created_lock = True
        fcntl.flock(lock_descriptor, fcntl.LOCK_EX)
        state_descriptor = secure_descriptor(
            state_path, os.O_RDWR, create=True
        )
        created_state = True
        state_descriptor = write_state(state_descriptor, {
            "schema": 1,
            "status": "prepared",
            "task": task_id,
            "script": expected_script,
            "owner_euid": owner_euid,
            "supervisor_pid": None,
            "supervisor_start_time": None,
            "child_pid": None,
            "child_start_time": None,
        })
    except BaseException:
        if state_descriptor is not None:
            os.close(state_descriptor)
            state_descriptor = None
        if lock_descriptor is not None:
            with contextlib.suppress(OSError):
                fcntl.flock(lock_descriptor, fcntl.LOCK_UN)
            os.close(lock_descriptor)
            lock_descriptor = None
        owned_paths = []
        if created_state:
            owned_paths.append(state_path)
        if created_lock:
            owned_paths.append(lock_path)
        for path in owned_paths:
            try:
                os.unlink(path)
            except FileNotFoundError:
                pass
        raise
    os.close(state_descriptor)
    fcntl.flock(lock_descriptor, fcntl.LOCK_UN)
    os.close(lock_descriptor)
    print('SUPERVISOR_JSON {"confirmed":true,"state":"prepared"}', flush=True)
    raise SystemExit(0)

if mode == "run":
    try:
        separator = sys.argv.index("--", 6)
        spawn_delay = float(sys.argv[6])
    except (ValueError, TypeError):
        fail()
    child_arguments = sys.argv[separator + 1:]
    if (
        separator != 7
        or not child_arguments
        or not math.isfinite(spawn_delay)
        or not 0.0 <= spawn_delay <= 10.0
        or expected_script not in child_arguments
        or task_id not in child_arguments
    ):
        fail()

    lock_descriptor, state_descriptor = open_locked_state()
    state = read_state(state_descriptor)
    if state.get("status") != "prepared":
        close_locked(lock_descriptor, state_descriptor)
        raise SystemExit(125)
    state.update({
        "status": "starting",
        "supervisor_pid": os.getpid(),
        "supervisor_start_time": process_start_time(os.getpid()),
        "child_pid": None,
        "child_start_time": None,
    })
    state_descriptor = write_state(state_descriptor, state)
    close_locked(lock_descriptor, state_descriptor)

    if spawn_delay:
        time.sleep(spawn_delay)

    try:
        lock_descriptor, state_descriptor = open_locked_state()
    except FileNotFoundError:
        raise SystemExit(125)
    state = read_state(state_descriptor)
    if (
        state.get("status") != "starting"
        or state.get("supervisor_pid") != os.getpid()
    ):
        close_locked(lock_descriptor, state_descriptor)
        raise SystemExit(125)
    try:
        child = subprocess.Popen(child_arguments, start_new_session=True)
    except BaseException:
        state.update({"status": "failed", "child_pid": None})
        state_descriptor = write_state(state_descriptor, state)
        close_locked(lock_descriptor, state_descriptor)
        raise
    child_start_time = process_start_time(child.pid)
    if child_start_time is None or not expected_process(child.pid):
        with contextlib.suppress(ProcessLookupError):
            os.killpg(child.pid, signal.SIGKILL)
        child.wait()
        state.update({"status": "failed", "child_pid": None})
        state_descriptor = write_state(state_descriptor, state)
        close_locked(lock_descriptor, state_descriptor)
        fail()
    state.update({
        "status": "running",
        "child_pid": child.pid,
        "child_start_time": child_start_time,
    })
    try:
        state_descriptor = write_state(state_descriptor, state)
    except BaseException:
        child_cleanup_error = None
        try:
            stop_spawned_child(child)
        except BaseException as exc:
            child_cleanup_error = exc
        state.update({
            "status": "failed",
            "child_pid": None,
            "child_start_time": None,
        })
        try:
            state_descriptor = write_state(state_descriptor, state)
        except BaseException:
            pass
        close_locked(lock_descriptor, state_descriptor)
        if child_cleanup_error is not None:
            raise child_cleanup_error
        raise
    close_locked(lock_descriptor, state_descriptor)

    def forward(number, _frame):
        try:
            os.killpg(child.pid, number)
        except ProcessLookupError:
            pass

    for number in (signal.SIGINT, signal.SIGTERM, signal.SIGHUP):
        signal.signal(number, forward)
    return_code = child.wait()
    try:
        lock_descriptor, state_descriptor = open_locked_state()
    except FileNotFoundError:
        raise SystemExit(return_code)
    state = read_state(state_descriptor)
    if state.get("status") not in {"aborted", "aborting"}:
        state["status"] = "finished" if return_code >= 0 else "failed"
    else:
        state["status"] = "aborted"
    state["child_pid"] = None
    state["child_start_time"] = None
    state_descriptor = write_state(state_descriptor, state)
    close_locked(lock_descriptor, state_descriptor)
    raise SystemExit(return_code)

if mode == "abort":
    if len(sys.argv) != 6:
        fail()
    try:
        lock_descriptor, state_descriptor = open_locked_state()
    except FileNotFoundError:
        fail()
    state = read_state(state_descriptor)
    prior_state = state.get("status")
    supervisor_pid = state.get("supervisor_pid")
    supervisor_start_time = state.get("supervisor_start_time")
    child_pid = state.get("child_pid")
    child_start_time = state.get("child_start_time")
    if prior_state == "starting":
        if (
            not isinstance(supervisor_pid, int)
            or supervisor_pid <= 1
            or not isinstance(supervisor_start_time, int)
            or process_start_time(supervisor_pid) != supervisor_start_time
            or not expected_process(supervisor_pid)
        ):
            close_locked(lock_descriptor, state_descriptor)
            fail()
        state["status"] = "aborted"
        persistence_error = None
        try:
            state_descriptor = write_state(state_descriptor, state)
        except BaseException as exc:
            persistence_error = exc
            unlink_current_state(state_descriptor)
        supervisor_error = None
        try:
            stop_expected_supervisor(supervisor_pid, supervisor_start_time)
        except BaseException as exc:
            supervisor_error = exc
        if persistence_error is not None or supervisor_error is not None:
            close_locked(lock_descriptor, state_descriptor)
            if supervisor_error is not None:
                raise supervisor_error
            raise persistence_error
    elif prior_state == "running":
        if (
            not isinstance(supervisor_pid, int)
            or supervisor_pid <= 1
            or not isinstance(supervisor_start_time, int)
            or process_start_time(supervisor_pid) != supervisor_start_time
            or not expected_process(supervisor_pid)
            or not isinstance(child_pid, int)
            or child_pid <= 1
            or not isinstance(child_start_time, int)
        ):
            close_locked(lock_descriptor, state_descriptor)
            fail()
        current_child_start = process_start_time(child_pid)
        if current_child_start is not None and (
            current_child_start != child_start_time
            or not expected_process(child_pid)
        ):
            close_locked(lock_descriptor, state_descriptor)
            fail()
        state["status"] = "aborting"
        try:
            state_descriptor = write_state(state_descriptor, state)
        except BaseException:
            # The existing running record still identifies this exact child.
            # Cancellation must continue even when the diagnostic tombstone
            # cannot be persisted.
            pass
        for number, grace in (
            (signal.SIGINT, 25.0),
            (signal.SIGTERM, 10.0),
            (signal.SIGKILL, 5.0),
        ):
            if process_start_time(child_pid) != child_start_time:
                break
            try:
                os.killpg(child_pid, number)
            except ProcessLookupError:
                break
            deadline = time.monotonic() + grace
            while time.monotonic() < deadline:
                if process_start_time(child_pid) != child_start_time:
                    break
                if not expected_process(child_pid):
                    close_locked(lock_descriptor, state_descriptor)
                    fail()
                time.sleep(0.05)
            if process_start_time(child_pid) != child_start_time:
                break
        if process_start_time(child_pid) == child_start_time:
            close_locked(lock_descriptor, state_descriptor)
            raise SystemExit(5)
        state["status"] = "aborted"
        state["child_pid"] = None
        state["child_start_time"] = None
        persistence_error = None
        try:
            state_descriptor = write_state(state_descriptor, state)
        except BaseException as exc:
            persistence_error = exc
        supervisor_error = None
        try:
            stop_expected_supervisor(supervisor_pid, supervisor_start_time)
        except BaseException as exc:
            supervisor_error = exc
        if persistence_error is not None or supervisor_error is not None:
            close_locked(lock_descriptor, state_descriptor)
            if supervisor_error is not None:
                raise supervisor_error
            raise persistence_error
    elif prior_state == "prepared":
        state["status"] = "aborted"
        try:
            state_descriptor = write_state(state_descriptor, state)
        except BaseException:
            unlink_current_state(state_descriptor)
            close_locked(lock_descriptor, state_descriptor)
            raise
    elif prior_state not in terminal_states:
        close_locked(lock_descriptor, state_descriptor)
        fail()
    close_locked(lock_descriptor, state_descriptor)
    print("SUPERVISOR_JSON " + json.dumps({
        "confirmed": True,
        "late_start_blocked": True,
        "prior_state": prior_state,
    }, sort_keys=True, separators=(",", ":")), flush=True)
    raise SystemExit(0)

if mode == "finalize":
    if len(sys.argv) != 6:
        fail()
    lock_descriptor, state_descriptor = open_locked_state()
    state = read_state(state_descriptor)
    if state.get("status") not in terminal_states:
        close_locked(lock_descriptor, state_descriptor)
        fail()
    child_pid = state.get("child_pid")
    if isinstance(child_pid, int) and child_pid > 1 and process_exists(child_pid):
        close_locked(lock_descriptor, state_descriptor)
        fail()
    os.unlink(state_path)
    os.close(state_descriptor)
    fcntl.flock(lock_descriptor, fcntl.LOCK_UN)
    os.close(lock_descriptor)
    os.unlink(lock_path)
    print('SUPERVISOR_JSON {"confirmed":true,"state":"removed"}', flush=True)
    raise SystemExit(0)

fail()
'''


ROSTER_PROBE_SOURCE = """\
import json
import re
import sys

import rospy
from gazebo_msgs.msg import ModelStates

count = int(sys.argv[1])
rospy.init_node("robotswarm_roster_probe", anonymous=True, disable_signals=True)
message = rospy.wait_for_message("/gazebo/model_states", ModelStates, timeout=12.0)
models = {str(name) for name in message.name if re.fullmatch(r"tb3_\\d+", str(name))}
expected = {"tb3_{}".format(index) for index in range(count)}
print("ROSTER_JSON " + json.dumps({
    "expected_robot_count": count,
    "gazebo_model_count": len(models),
    "exact_roster": models == expected,
}, sort_keys=True, separators=(",", ":")), flush=True)
"""


TASK_ACTIVITY_PROBE_SOURCE = """\
import json
import sys
import threading
import time

import rospy
from std_msgs.msg import String

task_id = sys.argv[1]
behavior = sys.argv[2]
expected_count = int(sys.argv[3])
timeout = float(sys.argv[4])
topic = {
    "formation": "/formation/status",
    "follow": "/follow_leader/status",
    "transport": "/transport/status",
}[behavior]
lock = threading.Lock()
latest_task = None
latest_behavior = None

def decode(message):
    try:
        value = json.loads(message.data)
    except (TypeError, ValueError):
        return None
    return value if isinstance(value, dict) else None

def task_callback(message):
    global latest_task
    value = decode(message)
    if value is not None:
        with lock:
            latest_task = value.get("task")

def behavior_callback(message):
    global latest_behavior
    value = decode(message)
    if value is not None:
        with lock:
            latest_behavior = value

def behavior_is_active(value):
    if not isinstance(value, dict) or str(value.get("task_id") or "") != task_id:
        return False
    if behavior == "transport":
        return not bool(value.get("paused")) and value.get("phase") in {
            "SEARCH", "APPROACH", "PUSH",
        }
    if behavior == "follow":
        return bool(value.get("active")) and value.get("state") == "running"
    assignments = value.get("robot_assignments")
    try:
        robot_count = int(value.get("robot_count"))
        maximum_error = float(value.get("maximum_position_error"))
    except (TypeError, ValueError):
        return False
    return (
        not bool(value.get("paused"))
        and value.get("state") == "moving"
        and value.get("movement_mode") == "moving"
        and not value.get("error")
        and robot_count == expected_count
        and isinstance(assignments, dict)
        and len(assignments) == expected_count
        and maximum_error <= 0.12
        and not value.get("stale_odometry")
        and not value.get("waiting_for_odometry")
    )

rospy.init_node("robotswarm_matrix_activity_probe", anonymous=True, disable_signals=True)
rospy.Subscriber("/swarm/status", String, task_callback, queue_size=1)
rospy.Subscriber(topic, String, behavior_callback, queue_size=1)
deadline = time.monotonic() + timeout
while not rospy.is_shutdown() and time.monotonic() < deadline:
    with lock:
        task = dict(latest_task) if isinstance(latest_task, dict) else None
        detail = dict(latest_behavior) if isinstance(latest_behavior, dict) else None
    if (
        task is not None
        and str(task.get("task_id") or "") == task_id
        and task.get("status") == "running"
        and behavior_is_active(detail)
    ):
        print("TASK_ACTIVITY_JSON " + json.dumps({
            "behavior": behavior,
            "task_status": "running",
            "behavior_state": str(detail.get("state") or ""),
            "behavior_phase": str(detail.get("phase") or ""),
            "movement_mode": str(detail.get("movement_mode") or ""),
            "complete_formation": behavior != "formation" or (
                isinstance(detail.get("robot_assignments"), dict)
                and len(detail.get("robot_assignments")) == expected_count
            ),
            "confirmed": True,
        }, sort_keys=True, separators=(",", ":")), flush=True)
        raise SystemExit(0)
    if (
        task is not None
        and str(task.get("task_id") or "") == task_id
        and task.get("status") in {"completed", "failed", "stopped"}
    ):
        raise SystemExit(3)
    time.sleep(0.05)
raise SystemExit(2)
"""


STOP_TASK_SOURCE = """\
import json
import sys
import threading
import time

import rospy
from std_msgs.msg import String

task_id = sys.argv[1]
timeout = float(sys.argv[2])
latest = None
sample_count = 0
last_sample_at = None
lock = threading.Lock()

def callback(message):
    global latest, sample_count, last_sample_at
    try:
        value = json.loads(message.data)
    except (TypeError, ValueError):
        return
    if isinstance(value, dict):
        with lock:
            latest = value.get("task")
            sample_count += 1
            last_sample_at = time.monotonic()

rospy.init_node("robotswarm_matrix_stop_probe", anonymous=True, disable_signals=True)
publisher = rospy.Publisher("/swarm/commands", String, queue_size=1)
rospy.Subscriber("/swarm/status", String, callback, queue_size=1)
connect_deadline = time.monotonic() + min(5.0, timeout)
while publisher.get_num_connections() == 0 and time.monotonic() < connect_deadline:
    time.sleep(0.05)
if publisher.get_num_connections() == 0:
    raise SystemExit(3)
with lock:
    baseline_samples = sample_count
publisher.publish(String(data=json.dumps({
    "command": "stop_task",
    "parameters": {"task_id": task_id},
})))
deadline = time.monotonic() + timeout
quiet_samples = 0
quiet_started_at = None
observed_samples = baseline_samples
while not rospy.is_shutdown() and time.monotonic() < deadline:
    with lock:
        task = dict(latest) if isinstance(latest, dict) else None
        current_samples = sample_count
        sampled_at = last_sample_at
    if current_samples == observed_samples:
        time.sleep(0.05)
        continue
    observed_samples = current_samples
    if isinstance(task, dict):
        observed_id = str(task.get("task_id") or "")
        observed_status = task.get("status")
        if observed_id == task_id and observed_status in {"completed", "failed", "stopped"}:
            print("STOP_TASK_JSON " + json.dumps({
                "confirmed": True,
                "mode": "correlated_terminal",
                "post_publish_samples": current_samples - baseline_samples,
            }, sort_keys=True, separators=(",", ":")), flush=True)
            raise SystemExit(0)
        if observed_id and observed_id != task_id:
            raise SystemExit(4)
        quiet = not observed_id and observed_status in {"idle", "stopped"}
    else:
        quiet = False
    if quiet:
        if quiet_started_at is None:
            quiet_started_at = sampled_at
            quiet_samples = 1
        else:
            quiet_samples += 1
        if (
            quiet_samples >= 3
            and sampled_at is not None
            and quiet_started_at is not None
            and sampled_at - quiet_started_at >= 0.5
        ):
            print("STOP_TASK_JSON " + json.dumps({
                "confirmed": True,
                "mode": "fresh_quiescence",
                "post_publish_samples": current_samples - baseline_samples,
            }, sort_keys=True, separators=(",", ":")), flush=True)
            raise SystemExit(0)
    else:
        quiet_samples = 0
        quiet_started_at = None
    time.sleep(0.05)
raise SystemExit(2)
"""


def build_roster_probe_command(
    docker_executable: str,
    container_identifier: str,
    expected_robot_count: int,
) -> list[str]:
    bootstrap = (
        "source /opt/ros/noetic/setup.bash"
        " && source /catkin_ws/devel/setup.bash"
        " && exec python3 -c \"$1\" \"$2\""
    )
    return [
        docker_executable,
        "exec",
        container_identifier,
        "/bin/bash",
        "-lc",
        bootstrap,
        "robotswarm-roster-probe",
        ROSTER_PROBE_SOURCE,
        str(expected_robot_count),
    ]


def build_task_activity_probe_command(
    docker_executable: str,
    container_identifier: str,
    task_id: str,
    behavior: str,
    expected_robot_count: int,
    timeout: float,
) -> list[str]:
    if not re.fullmatch(r"matrix-[0-9a-f]{32}", task_id):
        raise MatrixError("The task activity correlation identifier is invalid")
    if behavior not in {"formation", "follow", "transport"}:
        raise MatrixError("The task activity behavior is invalid")
    if not 1 <= expected_robot_count <= 10:
        raise MatrixError("The task activity robot count is invalid")
    bootstrap = (
        "source /opt/ros/noetic/setup.bash"
        " && source /catkin_ws/devel/setup.bash"
        " && exec python3 -c \"$1\" \"$2\" \"$3\" \"$4\" \"$5\""
    )
    return [
        docker_executable,
        "exec",
        container_identifier,
        "/bin/bash",
        "-lc",
        bootstrap,
        "robotswarm-task-activity-probe",
        TASK_ACTIVITY_PROBE_SOURCE,
        task_id,
        behavior,
        str(expected_robot_count),
        f"{timeout:.3f}",
    ]


def build_supervisor_command(
    docker_executable: str,
    container_identifier: str,
    mode: str,
    task_id: str,
    run_token: str,
) -> list[str]:
    if mode not in {"prepare", "abort", "finalize"}:
        raise MatrixError("The matrix supervisor operation is invalid")
    if not re.fullmatch(r"matrix-[0-9a-f]{32}", task_id):
        raise MatrixError("The matrix task correlation identifier is invalid")
    state_file, lock_file = acceptance_supervisor_paths(run_token)
    return [
        docker_executable,
        "exec",
        container_identifier,
        "/usr/bin/python3",
        "-c",
        ACCEPTANCE_SUPERVISOR_SOURCE,
        mode,
        state_file,
        lock_file,
        task_id,
        LIVE_ACCEPTANCE_PATH,
    ]


class DockerHost:
    def __init__(self, executable: str, stop_event: threading.Event) -> None:
        self.executable = executable
        self.stop_event = stop_event

    def run(
        self,
        arguments: Sequence[str],
        *,
        timeout: float = 30,
        interruptible: bool = True,
    ) -> ProcessOutput:
        return run_command(
            [self.executable, *arguments],
            timeout=timeout,
            stop_event=self.stop_event if interruptible else None,
        )

    @staticmethod
    def _nonempty_lines(value: str) -> list[str]:
        return [line.strip() for line in value.splitlines() if line.strip()]

    def _container_ids(self, session_id: uuid.UUID, *, interruptible: bool) -> list[str]:
        result = self.run(
            [
                "ps",
                "-a",
                "--no-trunc",
                "--quiet",
                "--filter",
                f"label={MANAGED_LABEL}=true",
                "--filter",
                f"label={SESSION_LABEL}={session_id}",
            ],
            interruptible=interruptible,
        )
        if result.returncode != 0:
            raise MatrixError("Docker could not inspect the session container")
        identifiers = self._nonempty_lines(result.stdout)
        if any(not re.fullmatch(r"[0-9a-f]{64}", item) for item in identifiers):
            raise MatrixError("Docker returned an invalid managed container identifier")
        return identifiers

    def _network_ids(self, session_id: uuid.UUID, *, interruptible: bool) -> list[str]:
        result = self.run(
            [
                "network",
                "ls",
                "--no-trunc",
                "--quiet",
                "--filter",
                f"label={MANAGED_LABEL}=true",
                "--filter",
                f"label={SESSION_LABEL}={session_id}",
            ],
            interruptible=interruptible,
        )
        if result.returncode != 0:
            raise MatrixError("Docker could not inspect the session network")
        identifiers = self._nonempty_lines(result.stdout)
        if any(not re.fullmatch(r"[0-9a-f]{64}", item) for item in identifiers):
            raise MatrixError("Docker returned an invalid managed network identifier")
        return identifiers

    def _inspect_one(self, kind: str, identifier: str) -> dict[str, Any]:
        prefix = [kind] if kind in {"image", "network"} else []
        result = self.run([*prefix, "inspect", identifier], timeout=30)
        if result.returncode != 0:
            raise MatrixError(f"Docker could not inspect the managed {kind}")
        try:
            values = strict_json_loads(result.stdout)
        except (json.JSONDecodeError, ValueError) as exc:
            raise MatrixError(f"Docker returned malformed {kind} metadata") from exc
        if not isinstance(values, list) or len(values) != 1 or not isinstance(values[0], dict):
            raise MatrixError(f"Docker returned ambiguous {kind} metadata")
        return values[0]

    @staticmethod
    def _labels(document: dict[str, Any], *, network: bool = False) -> dict[str, str]:
        raw = document.get("Labels") if network else (document.get("Config") or {}).get("Labels")
        if not isinstance(raw, dict) or any(not isinstance(item, str) for item in raw.values()):
            raise MatrixError("Docker resource labels are invalid")
        return {str(key): item for key, item in raw.items()}

    def verify_session(
        self, session_id: uuid.UUID, deployment_commit: str
    ) -> tuple[ContainerHandle, dict[str, Any]]:
        identifiers = self._container_ids(session_id, interruptible=True)
        if len(identifiers) != 1:
            raise MatrixError("The account session does not own exactly one managed container")
        container = self._inspect_one("container", identifiers[0])
        labels = self._labels(container)
        expected_name = f"/robotswarm-{session_id.hex}"
        try:
            worker_id = uuid.UUID(str(labels.get(WORKER_LABEL)))
        except (ValueError, AttributeError) as exc:
            raise MatrixError("The managed container has an invalid worker owner") from exc
        if (
            labels.get(MANAGED_LABEL) != "true"
            or labels.get(SESSION_LABEL) != str(session_id)
            or container.get("Name") != expected_name
            or (container.get("State") or {}).get("Running") is not True
        ):
            raise MatrixError("The managed container does not match the live session")

        image_identifier = container.get("Image")
        configured_image = (container.get("Config") or {}).get("Image")
        match = IMAGE_ID_PATTERN.fullmatch(str(image_identifier or ""))
        if match is None or configured_image != image_identifier:
            raise MatrixError("The session container is not pinned to its immutable image ID")
        image = self._inspect_one("image", str(image_identifier))
        image_labels = self._labels(image)
        if image.get("Id") != image_identifier:
            raise MatrixError("Docker resolved a different immutable image")
        if image_labels.get(IMAGE_REVISION_LABEL) != deployment_commit:
            raise MatrixError("The ROS image revision does not match the requested deployment SHA")

        expected_version = f"{deployment_commit}+{match.group(1)[:12]}"
        if labels.get(IMAGE_VERSION_LABEL) != expected_version:
            raise MatrixError("The container image-version label does not match the deployment SHA")

        networks = self._network_ids(session_id, interruptible=True)
        if len(networks) != 1:
            raise MatrixError("The live session does not own exactly one private network")
        network = self._inspect_one("network", networks[0])
        network_labels = self._labels(network, network=True)
        expected_network_name = f"robotswarm-{session_id.hex}-net"
        if (
            network_labels.get(MANAGED_LABEL) != "true"
            or network_labels.get(SESSION_LABEL) != str(session_id)
            or network_labels.get(WORKER_LABEL) != str(worker_id)
            or network.get("Name") != expected_network_name
            or network.get("Internal") is not True
        ):
            raise MatrixError("The private Docker network does not match the live session")

        attached_networks = (container.get("NetworkSettings") or {}).get("Networks")
        if not isinstance(attached_networks, dict) or set(attached_networks) != {
            expected_network_name
        }:
            raise MatrixError("The managed container is not attached only to its private network")
        endpoint = attached_networks.get(expected_network_name)
        members = network.get("Containers")
        membership = members.get(identifiers[0]) if isinstance(members, dict) else None
        private_ip = endpoint.get("IPAddress") if isinstance(endpoint, dict) else None
        ipam = (network.get("IPAM") or {}).get("Config")
        gateway = ipam[0].get("Gateway") if isinstance(ipam, list) and ipam else None
        try:
            parsed_private_ip = ipaddress.ip_address(str(private_ip))
            parsed_gateway = ipaddress.ip_address(str(gateway))
        except ValueError as exc:
            raise MatrixError("The private Docker network has invalid IP endpoints") from exc
        if (
            not isinstance(endpoint, dict)
            or endpoint.get("NetworkID") != networks[0]
            or parsed_private_ip.version != 4
            or parsed_gateway.version != 4
            or not isinstance(members, dict)
            or set(members) != {identifiers[0]}
            or not isinstance(membership, dict)
            or membership.get("Name") != expected_name[1:]
        ):
            raise MatrixError("Docker could not prove exclusive container-network membership")

        evidence = {
            "managed": True,
            "running": True,
            "imagePinned": True,
            "imageRevision": deployment_commit,
            "imageVersion": expected_version,
            "privateNetworkVerified": True,
            "exclusiveNetworkAttachmentVerified": True,
            "privateMasterBindingReady": True,
        }
        return ContainerHandle(
            identifiers[0],
            expected_version,
            str(parsed_private_ip),
            str(parsed_gateway),
        ), evidence

    def run_acceptance(
        self,
        container: ContainerHandle,
        scenario: ScenarioSpec,
        timeout: float,
        task_id: str,
        run_token: str,
        *,
        cancel_event: threading.Event | None = None,
        on_started: Callable[[float], None] | None = None,
        on_exited: Callable[[float], None] | None = None,
    ) -> ProcessOutput:
        prepared = self._run_supervisor_operation(
            container,
            task_id,
            run_token,
            "prepare",
            timeout=10,
        )
        if prepared.get("state") != "prepared":
            raise CleanupError("The ROS acceptance supervisor was not prepared")
        try:
            if self.stop_event.is_set():
                raise KeyboardInterrupt
            if cancel_event is not None and cancel_event.is_set():
                raise MatrixError(
                    "The ROS acceptance process was cancelled before launch"
                )
            output = run_command(
                build_acceptance_command(
                    self.executable,
                    container.identifier,
                    scenario.name,
                    task_id,
                    run_token,
                ),
                timeout=timeout,
                stop_event=self.stop_event,
                cancel_event=cancel_event,
                on_started=on_started,
                on_exited=on_exited,
            )
        except BaseException:
            try:
                self.abort_acceptance(container, task_id, run_token)
            except Exception as cleanup_error:
                raise CleanupError(
                    "The cancelled ROS acceptance process or task remained active"
                ) from cleanup_error
            raise
        finalized = self._run_supervisor_operation(
            container,
            task_id,
            run_token,
            "finalize",
            timeout=10,
        )
        if finalized.get("state") != "removed" or not self.wait_acceptance_supervisor_absent(
            container, run_token, timeout=5
        ):
            raise CleanupError("The completed ROS acceptance supervisor retained state")
        return output

    def _run_supervisor_operation(
        self,
        container: ContainerHandle,
        task_id: str,
        run_token: str,
        mode: str,
        *,
        timeout: float,
    ) -> dict[str, Any]:
        output = run_command(
            build_supervisor_command(
                self.executable,
                container.identifier,
                mode,
                task_id,
                run_token,
            ),
            timeout=timeout,
        )
        entries = [
            line[len("SUPERVISOR_JSON ") :]
            for line in output.stdout.splitlines()
            if line.startswith("SUPERVISOR_JSON ")
        ]
        if output.returncode != 0 or len(entries) != 1:
            raise CleanupError(
                "The ROS acceptance supervisor operation could not be verified"
            )
        try:
            payload = strict_json_loads(entries[0])
        except (json.JSONDecodeError, ValueError) as exc:
            raise CleanupError(
                "The ROS acceptance supervisor proof was malformed"
            ) from exc
        if not isinstance(payload, dict) or payload.get("confirmed") is not True:
            raise CleanupError(
                "The ROS acceptance supervisor did not confirm its operation"
            )
        return payload

    def wait_acceptance_supervisor_absent(
        self,
        container: ContainerHandle,
        run_token: str,
        *,
        timeout: float,
    ) -> bool:
        state_file, lock_file = acceptance_supervisor_paths(run_token)
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            state_absent = run_command(
                [self.executable, "exec", container.identifier,
                 "/usr/bin/test", "!", "-e", state_file],
                timeout=5,
            )
            lock_absent = run_command(
                [self.executable, "exec", container.identifier,
                 "/usr/bin/test", "!", "-e", lock_file],
                timeout=5,
            )
            if state_absent.returncode == 0 and lock_absent.returncode == 0:
                return True
            time.sleep(0.1)
        return False

    def abort_acceptance(
        self,
        container: ContainerHandle,
        task_id: str,
        run_token: str,
    ) -> None:
        cancel: dict[str, Any] = {}
        cancel_error: BaseException | None = None
        try:
            cancel = self._run_supervisor_operation(
                container,
                task_id,
                run_token,
                "abort",
                timeout=50,
            )
        except BaseException as exc:
            cancel_error = exc
        stop_bootstrap = (
            "source /opt/ros/noetic/setup.bash"
            " && source /catkin_ws/devel/setup.bash"
            " && exec python3 -c \"$1\" \"$2\" \"$3\""
        )
        stopped: ProcessOutput | None = None
        stop_error: BaseException | None = None
        try:
            stopped = run_command(
                [
                    self.executable,
                    "exec",
                    container.identifier,
                    "/bin/bash",
                    "-lc",
                    stop_bootstrap,
                    "robotswarm-stop-task-probe",
                    STOP_TASK_SOURCE,
                    task_id,
                    "30.0",
                ],
                timeout=35,
            )
        except BaseException as exc:
            stop_error = exc
        stop_entries = [
            line[len("STOP_TASK_JSON ") :]
            for line in (stopped.stdout if stopped is not None else "").splitlines()
            if line.startswith("STOP_TASK_JSON ")
        ]
        stop_proof: dict[str, Any] | None = None
        if stopped is not None and stopped.returncode == 0 and len(stop_entries) == 1:
            try:
                candidate = strict_json_loads(stop_entries[0])
            except (json.JSONDecodeError, ValueError):
                candidate = None
            if isinstance(candidate, dict):
                stop_proof = candidate
        finalized: dict[str, Any] = {}
        finalize_error: BaseException | None = None
        try:
            finalized = self._run_supervisor_operation(
                container,
                task_id,
                run_token,
                "finalize",
                timeout=10,
            )
        except BaseException as exc:
            finalize_error = exc
        try:
            state_absent = self.wait_acceptance_supervisor_absent(
                container, run_token, timeout=5
            )
        except BaseException:
            state_absent = False
        try:
            post_publish_samples = int(
                (stop_proof or {}).get("post_publish_samples") or 0
            )
        except (TypeError, ValueError):
            post_publish_samples = 0
        valid_stop_mode = (
            stop_proof is not None
            and stop_proof.get("confirmed") is True
            and (
                stop_proof.get("mode") == "correlated_terminal"
                or (
                    stop_proof.get("mode") == "fresh_quiescence"
                    and cancel.get("prior_state") in {
                        "prepared", "starting"
                    }
                )
            )
            and post_publish_samples
            >= (3 if stop_proof.get("mode") == "fresh_quiescence" else 1)
        )
        if (
            cancel_error is not None
            or stop_error is not None
            or finalize_error is not None
            or cancel.get("late_start_blocked") is not True
            or finalized.get("state") != "removed"
            or not valid_stop_mode
            or not state_absent
        ):
            raise CleanupError("The ROS child cancellation could not be verified")

    def wait_task_active(
        self,
        container: ContainerHandle,
        task_id: str,
        scenario: ScenarioSpec,
        timeout: float,
    ) -> dict[str, Any]:
        output = run_command(
            build_task_activity_probe_command(
                self.executable,
                container.identifier,
                task_id,
                scenario.behavior,
                scenario.robot_count,
                timeout,
            ),
            timeout=timeout + 5,
            stop_event=self.stop_event,
        )
        entries = [
            line[len("TASK_ACTIVITY_JSON ") :]
            for line in output.stdout.splitlines()
            if line.startswith("TASK_ACTIVITY_JSON ")
        ]
        if output.returncode != 0 or len(entries) != 1:
            reason = {
                2: "activity_timeout",
                3: "task_terminal_before_activity",
            }.get(output.returncode, "probe_protocol_failure")
            raise MatrixError(
                "The correlated ROS task was not active during visual sampling "
                f"({reason})"
            )
        try:
            payload = strict_json_loads(entries[0])
        except (json.JSONDecodeError, ValueError) as exc:
            raise MatrixError("The ROS task activity proof was malformed") from exc
        if (
            not isinstance(payload, dict)
            or payload.get("confirmed") is not True
            or payload.get("behavior") != scenario.behavior
            or payload.get("task_status") != "running"
            or (
                scenario.behavior == "formation"
                and (
                    payload.get("behavior_state") != "moving"
                    or payload.get("movement_mode") != "moving"
                    or payload.get("complete_formation") is not True
                )
            )
        ):
            raise MatrixError("The ROS task activity proof did not match the scenario")
        return {
            "behavior": scenario.behavior,
            "taskStatus": "running",
            "behaviorState": redact_text(str(payload.get("behavior_state") or "")),
            "behaviorPhase": redact_text(str(payload.get("behavior_phase") or "")),
            "completeFormation": (
                payload.get("complete_formation") is True
                if scenario.behavior == "formation" else None
            ),
            "confirmed": True,
        }

    def copy_from_container(
        self,
        container: ContainerHandle,
        source: str,
        destination: Path,
        maximum_bytes: int,
    ) -> bytes:
        if destination.exists() or destination.is_symlink():
            raise MatrixError("An active-probe input path already exists")
        result = self.run(
            ["cp", f"{container.identifier}:{source}", str(destination)],
            timeout=30,
        )
        if result.returncode != 0:
            raise MatrixError("Docker could not copy an active-probe input")
        try:
            metadata = destination.lstat()
        except OSError as exc:
            raise MatrixError("Docker did not create the active-probe input") from exc
        if (
            not stat.S_ISREG(metadata.st_mode)
            or metadata.st_uid != os.getuid()
            or metadata.st_size <= 0
            or metadata.st_size > maximum_bytes
        ):
            raise MatrixError("Docker created an unsafe active-probe input")
        destination.chmod(0o500 if destination.suffix == ".py" else 0o400)
        return read_owned_bounded_file(destination, maximum_bytes)

    def verify_full_roster(
        self,
        container: ContainerHandle,
        expected_robot_count: int,
    ) -> dict[str, Any]:
        output = run_command(
            build_roster_probe_command(
                self.executable,
                container.identifier,
                expected_robot_count,
            ),
            timeout=30,
            stop_event=self.stop_event,
        )
        entries = [
            line[len("ROSTER_JSON ") :]
            for line in output.stdout.splitlines()
            if line.startswith("ROSTER_JSON ")
        ]
        if output.returncode != 0 or len(entries) != 1:
            raise MatrixError("The post-scenario Gazebo roster probe failed")
        try:
            payload = strict_json_loads(entries[0])
        except (json.JSONDecodeError, ValueError) as exc:
            raise MatrixError("The post-scenario Gazebo roster probe was malformed") from exc
        if (
            not isinstance(payload, dict)
            or payload.get("expected_robot_count") != expected_robot_count
            or payload.get("gazebo_model_count") != expected_robot_count
            or payload.get("exact_roster") is not True
        ):
            raise MatrixError("The full robot roster was not retained in Gazebo")
        return {
            "source": "/gazebo/model_states",
            "expectedRobots": expected_robot_count,
            "observedRobotModels": expected_robot_count,
            "exactRoster": True,
        }

    def resources_absent(self, session_id: uuid.UUID) -> tuple[bool, bool]:
        containers = self._container_ids(session_id, interruptible=False)
        networks = self._network_ids(session_id, interruptible=False)
        container_name = f"robotswarm-{session_id.hex}"
        network_name = f"robotswarm-{session_id.hex}-net"
        container_name_absent = self._named_resource_absent(
            "container", container_name
        )
        network_name_absent = self._named_resource_absent("network", network_name)
        return (
            not containers and container_name_absent,
            not networks and network_name_absent,
        )

    def _named_resource_absent(self, kind: str, name: str) -> bool:
        prefix = ["network"] if kind == "network" else []
        result = self.run(
            [*prefix, "inspect", name],
            timeout=15,
            interruptible=False,
        )
        if result.returncode == 0:
            return False
        diagnostic = result.stderr.lower()
        missing = (
            "no such object" in diagnostic
            or "no such container" in diagnostic
            or "no such network" in diagnostic
            or ("network" in diagnostic and "not found" in diagnostic)
        )
        if not missing:
            raise MatrixError(f"Docker could not confirm removal of the {kind}")
        return True


def parse_ros_protocol(output: ProcessOutput, expected: ScenarioSpec) -> RosEvidence:
    lines = output.stdout.splitlines()
    result_entries = [
        (index, line[len("RESULT_JSON ") :])
        for index, line in enumerate(lines)
        if line.startswith("RESULT_JSON ")
    ]
    summary_entries = [
        (index, line[len("SUMMARY_JSON ") :])
        for index, line in enumerate(lines)
        if line.startswith("SUMMARY_JSON ")
    ]
    result_payloads = [payload for _index, payload in result_entries]
    summary_payloads = [payload for _index, payload in summary_entries]
    if len(result_payloads) != 1 or len(summary_payloads) != 1:
        raise MatrixError("The ROS runner did not emit one RESULT_JSON and one SUMMARY_JSON")
    if result_entries[0][0] >= summary_entries[0][0]:
        raise MatrixError("The ROS runner emitted SUMMARY_JSON before its scenario result")
    try:
        result = strict_json_loads(result_payloads[0])
        summary = strict_json_loads(summary_payloads[0])
    except (json.JSONDecodeError, ValueError) as exc:
        raise MatrixError("The ROS runner emitted malformed JSON evidence") from exc
    if not isinstance(result, dict) or not isinstance(summary, dict):
        raise MatrixError("The ROS runner evidence has an invalid top-level type")
    if result.get("scenario") != expected.name:
        raise MatrixError("The ROS result belongs to a different scenario")
    if result.get("behavior") != expected.behavior or result.get("robot_count") != expected.robot_count:
        raise MatrixError("The ROS result does not match the requested behavior or robot count")
    return RosEvidence(
        result=result,
        summary=summary,
        result_sha256=hashlib.sha256(result_payloads[0].encode("utf-8")).hexdigest(),
        summary_sha256=hashlib.sha256(summary_payloads[0].encode("utf-8")).hexdigest(),
        returncode=output.returncode,
    )


def finite_number(value: Any, description: str) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise MatrixError(f"The {description} is not numeric")
    number = float(value)
    if not math.isfinite(number):
        raise MatrixError(f"The {description} is not finite")
    return number


def classify_active_probe_failure(output: ProcessOutput) -> dict[str, Any]:
    """Keep a useful failure category without retaining gzclient output."""
    diagnostic = f"{output.stdout}\n{output.stderr}".lower()
    categories = (
        ("timed out waiting for rendered frames", "render_report_timeout"),
        ("gzclient exited with status", "gzclient_exited_before_report"),
        ("diagnostic output exceeded", "diagnostic_output_limit"),
        ("renderer is not the expected gpu", "unexpected_gpu_renderer"),
        ("rendered fps", "render_fps_below_threshold"),
        ("post-render frame rate", "post_render_fps_below_threshold"),
        ("no physics real-time-factor samples", "physics_samples_missing"),
        ("physics real time factor", "physics_rtf_below_threshold"),
        ("physics real-time factor", "physics_rtf_below_threshold"),
        ("active user camera", "active_camera_missing"),
        ("gui probe plugin does not exist", "probe_plugin_missing"),
        ("private gzclient tmpdir", "private_runtime_failed"),
        ("process group", "gzclient_cleanup_failed"),
    )
    category = next(
        (name for marker, name in categories if marker in diagnostic),
        "unclassified_preflight_failure",
    )
    signal_markers = (
        ("robotswarm gui probe will write", "probe_plugin_loaded"),
        ("unable to create rendering window", "render_window_creation_failed"),
        ("failed to create d3d12", "d3d12_initialization_failed"),
        ("libgl error", "libgl_error"),
        ("could not initialize opengl", "opengl_initialization_failed"),
        ("failed to load system plugin", "probe_plugin_load_failed"),
        ("error loading system plugin", "probe_plugin_load_failed"),
        ("failed to load plugin", "probe_plugin_load_failed"),
        ("failed to map segment from shared object", "executable_mount_denied"),
        ("cannot connect to x server", "x11_connection_failed"),
        ("unable to connect to x server", "x11_connection_failed"),
        ("badaccess", "x11_bad_access"),
        ("badalloc", "x11_bad_allocation"),
        ("llvmpipe", "software_renderer_seen"),
        ("segmentation fault", "segmentation_fault"),
        ("out of memory", "out_of_memory"),
    )
    signals = sorted(
        {name for marker, name in signal_markers if marker in diagnostic}
    )
    return {
        "category": category,
        "diagnosticSignals": signals,
        "exitCode": output.returncode,
        "rawDiagnosticRetained": False,
    }


def validate_ros_evidence(evidence: RosEvidence) -> float:
    summary = evidence.summary
    if (
        evidence.returncode != 0
        or evidence.result.get("passed") is not True
        or evidence.result.get("cleanup_failures") not in ([], None)
        or summary.get("selected") != 1
        or summary.get("executed") != 1
        or summary.get("passed") != 1
        or summary.get("failed") != 0
        or summary.get("all_passed") is not True
        or summary.get("cleanup_passed") is not True
        or summary.get("failed_scenarios") not in ([], None)
        or summary.get("cleanup_failures") not in ([], None)
    ):
        raise MatrixError("The ROS scenario or its internal cleanup did not pass")
    metrics = evidence.result.get("metrics")
    if not isinstance(metrics, dict):
        raise MatrixError("The ROS result omitted its metrics")
    real_time_factor = finite_number(metrics.get("real_time_factor"), "ROS real-time factor")
    if real_time_factor < MINIMUM_REAL_TIME_FACTOR:
        raise MatrixError("The ROS real-time factor is below 2.90")
    return real_time_factor


def validate_transport_object_preference(
    result: dict[str, Any], robot_count: int
) -> dict[str, Any]:
    """Prove that every available payload contact is filled before a chain."""
    if (
        isinstance(robot_count, bool)
        or not isinstance(robot_count, int)
        or robot_count <= 0
    ):
        raise MatrixError("The transport object-preference count is invalid")

    expected = {f"tb3_{index}" for index in range(robot_count)}
    behavior = result.get("behavior_status")
    metrics = result.get("metrics")
    assignments = (
        behavior.get("robot_assignments")
        if isinstance(behavior, dict) else None
    )
    participation = (
        metrics.get("transport_participation")
        if isinstance(metrics, dict) else None
    )
    if not isinstance(assignments, dict) or set(assignments) != expected:
        raise MatrixError(
            "The transport object-preference evidence does not cover the exact roster"
        )
    if not isinstance(participation, dict) or set(participation) != expected:
        raise MatrixError(
            "The transport contact evidence does not cover the exact roster"
        )

    roots: set[str] = set()
    companions: set[str] = set()
    slots: dict[tuple[int, int], str] = {}
    normalized: dict[str, dict[str, Any]] = {}
    for robot in expected:
        assignment = assignments.get(robot)
        contact = participation.get(robot)
        if not isinstance(assignment, dict) or not isinstance(contact, dict):
            raise MatrixError("A transport assignment is not structured")
        role = assignment.get("role")
        chain_index = assignment.get("chain_index")
        depth = assignment.get("chain_depth", assignment.get("depth"))
        parent = assignment.get("parent_namespace")
        if (
            isinstance(chain_index, bool)
            or not isinstance(chain_index, int)
            or chain_index < 0
            or isinstance(depth, bool)
            or not isinstance(depth, int)
            or depth < 0
        ):
            raise MatrixError("A transport assignment has invalid chain coordinates")
        slot = (chain_index, depth)
        if slot in slots:
            raise MatrixError("Two transport robots occupy the same chain slot")
        slots[slot] = robot

        if role == "payload_push":
            if depth != 0 or parent not in (None, ""):
                raise MatrixError("A payload root has an invalid predecessor")
            if contact.get("role") != role or finite_number(
                contact.get("direct_contact_samples"),
                "payload-contact sample count",
            ) <= 0:
                raise MatrixError(
                    "A payload root has no measured direct object contact"
                )
            if contact.get("declared_parent_namespaces") not in ([], None):
                raise MatrixError("A payload root declared a robot predecessor")
            roots.add(robot)
        elif role == "companion_push":
            if depth < 1 or parent not in expected or parent == robot:
                raise MatrixError("A companion pusher has an invalid predecessor")
            declared_parents = contact.get("declared_parent_namespaces")
            if (
                contact.get("role") != role
                or not isinstance(declared_parents, list)
                or parent not in declared_parents
                or finite_number(
                    contact.get("companion_contact_samples"),
                    "companion-contact sample count",
                )
                <= 0
            ):
                raise MatrixError(
                    "A companion pusher has no measured predecessor contact"
                )
            companions.add(robot)
        else:
            raise MatrixError("A robot was not assigned a transport-push role")
        normalized[robot] = {
            "role": role,
            "chain_index": chain_index,
            "depth": depth,
            "parent": parent,
        }

    expected_root_count = min(2, robot_count)
    if len(roots) != expected_root_count or len(companions) != (
        robot_count - expected_root_count
    ):
        raise MatrixError(
            "Transport did not prefer every available direct payload contact"
        )
    if {
        normalized[robot]["chain_index"] for robot in roots
    } != set(range(expected_root_count)):
        raise MatrixError("The payload roots do not occupy the expected push lanes")

    for robot in companions:
        current = robot
        visited: set[str] = set()
        while normalized[current]["role"] == "companion_push":
            if current in visited:
                raise MatrixError("A transport companion chain contains a cycle")
            visited.add(current)
            parent = normalized[current]["parent"]
            parent_assignment = normalized[parent]
            if (
                parent_assignment["chain_index"]
                != normalized[current]["chain_index"]
                or parent_assignment["depth"]
                != normalized[current]["depth"] - 1
            ):
                raise MatrixError("A transport companion chain is not contiguous")
            current = parent
        if current not in roots:
            raise MatrixError("A transport companion chain does not reach the payload")

    return {
        "exactRoster": True,
        "objectContactPreference": True,
        "payloadRootCount": len(roots),
        "companionPusherCount": len(companions),
        "expectedPayloadRootCount": expected_root_count,
        "allRobotsAssigned": True,
        "allCompanionChainsReachPayload": True,
    }


def validate_transport_n2_contract(result: dict[str, Any]) -> dict[str, Any]:
    """Require complete collaborative evidence for the two-robot case."""
    expected = {"tb3_0", "tb3_1"}
    metrics = result.get("metrics")
    behavior = result.get("behavior_status")
    timeline = result.get("transport_phase_timeline")
    if not all(isinstance(item, dict) for item in (metrics, behavior, timeline)):
        raise MatrixError("The N=2 transport result omitted correlated evidence")

    transitions = timeline.get("transitions")
    if (
        not isinstance(transitions, list)
        or timeline.get("protocol_error") not in (None, "")
        or timeline.get("dropped_transitions") != 0
    ):
        raise MatrixError("The N=2 transport phase history is incomplete")
    phases = [
        str(item.get("phase") or "").upper()
        for item in transitions
        if isinstance(item, dict)
    ]
    required_phases = ("SEARCH", "APPROACH", "PUSH", "DONE")
    cursor = 0
    for phase in phases:
        if cursor < len(required_phases) and phase == required_phases[cursor]:
            cursor += 1
    if cursor != len(required_phases):
        raise MatrixError("The N=2 transport did not prove search, rendezvous, and push")

    travel = metrics.get("robot_travel_m")
    participation = metrics.get("transport_participation")
    assignments = behavior.get("robot_assignments")
    if not all(isinstance(item, dict) for item in (travel, participation, assignments)):
        raise MatrixError("The N=2 transport omitted its complete robot roster")
    if set(travel) != expected or set(participation) != expected or set(assignments) != expected:
        raise MatrixError("The N=2 transport evidence does not cover the exact roster")

    discovery = metrics.get("transport_discovery_response")
    if not isinstance(discovery, dict):
        raise MatrixError("The N=2 transport omitted discovery evidence")
    discovery_robots = discovery.get("robots")
    if (
        discovery.get("notice_observed") is not True
        or discovery.get("finder") not in expected
        or set(discovery.get("notice_recipients") or []) != expected
        or discovery.get("missing_notice_recipients") not in ([], None)
        or discovery.get("missing_status_acknowledgements") not in ([], None)
        or discovery.get("search_motion_window_supported") is not True
        or discovery.get("peak_simultaneous_movers") != 2
        or set(discovery.get("robots_observed_moving_during_search") or []) != expected
        or discovery.get("robots_below_required_search_travel") not in ([], None)
        or discovery.get("robots_below_required_travel") not in ([], None)
        or not isinstance(discovery_robots, dict)
        or set(discovery_robots) != expected
    ):
        raise MatrixError("The N=2 transport did not prove fleet search and notice")

    for robot in expected:
        response = discovery_robots[robot]
        assignment = assignments[robot]
        contribution = participation[robot]
        if not all(
            isinstance(item, dict)
            for item in (response, assignment, contribution)
        ):
            raise MatrixError("The N=2 transport robot evidence is malformed")
        if (
            response.get("notice_recipient") is not True
            or response.get("motion_detected") is not True
            or response.get("met_pre_push_path_requirement") is not True
            or assignment.get("notice_received") is not True
            or assignment.get("rendezvous_ready") is not True
            or assignment.get("role") != "payload_push"
            or assignment.get("parent_namespace") not in (None, "")
        ):
            raise MatrixError("The N=2 transport did not prove full-fleet rendezvous")
        useful_samples = contribution.get("useful_pushing_samples")
        if (
            contribution.get("role") != "payload_push"
            or contribution.get("declared_parent_namespaces") not in ([], None)
            or isinstance(useful_samples, bool)
            or not isinstance(useful_samples, int)
            or useful_samples <= 0
        ):
            raise MatrixError("The N=2 transport did not prove two useful payload roots")

    all_useful = metrics.get("transport_all_useful_samples")
    if (
        result.get("task_outcome") != "completed"
        or behavior.get("phase") != "DONE"
        or behavior.get("synchronized_push_started") is not True
        or metrics.get("transport_active_push_latched") is not True
        or isinstance(all_useful, bool)
        or not isinstance(all_useful, int)
        or all_useful <= 0
    ):
        raise MatrixError("The N=2 transport did not prove simultaneous useful pushing")

    return {
        "exactRoster": True,
        "phaseSequence": phases,
        "noticeRecipientCount": 2,
        "rendezvousRobotCount": 2,
        "payloadRootCount": 2,
        "companionPusherCount": 0,
        "usefulPusherCount": 2,
    }


def read_owned_bounded_file(path: Path, maximum_bytes: int) -> bytes:
    flags = os.O_RDONLY | os.O_CLOEXEC | os.O_NONBLOCK
    if hasattr(os, "O_NOFOLLOW"):
        flags |= os.O_NOFOLLOW
    try:
        descriptor = os.open(path, flags)
    except OSError as exc:
        raise MatrixError("The Gazebo render report is unavailable") from exc
    try:
        before = os.fstat(descriptor)
        if (
            not stat.S_ISREG(before.st_mode)
            or before.st_uid != os.getuid()
            or stat.S_IMODE(before.st_mode) & 0o022
            or before.st_size <= 0
            or before.st_size > maximum_bytes
        ):
            raise MatrixError("The Gazebo render report is not a safe bounded file")
        chunks: list[bytes] = []
        remaining = maximum_bytes + 1
        while remaining > 0:
            chunk = os.read(descriptor, remaining)
            if not chunk:
                break
            chunks.append(chunk)
            remaining -= len(chunk)
        after = os.fstat(descriptor)
    finally:
        os.close(descriptor)
    payload = b"".join(chunks)
    if (
        len(payload) > maximum_bytes
        or len(payload) != before.st_size
        or before.st_dev != after.st_dev
        or before.st_ino != after.st_ino
        or before.st_mtime_ns != after.st_mtime_ns
        or before.st_size != after.st_size
    ):
        raise MatrixError("The Gazebo render report changed while it was copied")
    return payload


def validate_viewer_startup_report(
    document: dict[str, Any], raw: bytes
) -> GazeboGuiEvidence:
    if document.get("schema_version") != 1:
        raise MatrixError("The Gazebo render report schema is unsupported")
    process = document.get("process")
    display = document.get("display")
    camera = document.get("camera")
    renderer = document.get("renderer")
    render = document.get("render_measurement")
    physics = document.get("physics_measurement")
    if not all(isinstance(item, dict) for item in (process, display, camera, renderer, render, physics)):
        raise MatrixError("The Gazebo render report is incomplete")
    executable = str(process.get("executable") or "")
    if not re.fullmatch(r"gzclient(?:-[0-9]+(?:\.[0-9]+)*)?", Path(executable).name):
        raise MatrixError("The render report was not produced by gzclient")
    if not display.get("x11") and not display.get("wayland"):
        raise MatrixError("The Gazebo render report has no private display")
    if not camera.get("name"):
        raise MatrixError("The Gazebo render report has no active camera")
    if finite_number(camera.get("viewport_width"), "viewport width") <= 0 or finite_number(
        camera.get("viewport_height"), "viewport height"
    ) <= 0:
        raise MatrixError("The Gazebo viewport is empty")

    renderer_text = " ".join(
        str(renderer.get(key) or "")
        for key in ("api", "device", "vendor", "gl_vendor", "gl_renderer")
    )
    normalized_renderer = renderer_text.lower()
    effective_renderer = " ".join(
        str(renderer.get(key) or "") for key in ("device", "gl_renderer")
    ).lower()
    if "d3d12" not in effective_renderer or "nvidia" not in effective_renderer:
        raise MatrixError("Gazebo did not report the required D3D12 NVIDIA renderer")
    if any(
        marker in normalized_renderer
        for marker in (
            "llvmpipe",
            "softpipe",
            "swrast",
            "lavapipe",
            "software rasterizer",
            "microsoft basic render driver",
            "gdi generic",
        )
    ):
        raise MatrixError("Gazebo selected a software renderer")

    if render.get("source") != "gazebo::rendering::Camera::AvgFPS":
        raise MatrixError("Gazebo FPS was not measured from the active camera")
    if finite_number(render.get("samples"), "render sample count") < 2:
        raise MatrixError("The Gazebo render report has too few FPS samples")
    average_fps = finite_number(render.get("average_fps"), "average Gazebo FPS")
    post_render_fps = finite_number(
        render.get("post_render_rate_fps"), "post-render Gazebo FPS"
    )
    if (
        average_fps < MINIMUM_STARTUP_SCENE_FPS
        or post_render_fps < MINIMUM_STARTUP_SCENE_FPS
    ):
        raise MatrixError("The Gazebo startup scene rendered below 45 FPS")

    expected_rtf_source = "gazebo.msgs.WorldStatistics delta(sim_time)/delta(real_time)"
    if physics.get("source") != expected_rtf_source:
        raise MatrixError("The physics RTF source is missing or ambiguous")
    if finite_number(physics.get("samples"), "physics sample count") < 1:
        raise MatrixError("The Gazebo render report has no physics samples")
    real_time_factor = finite_number(
        physics.get("real_time_factor"), "Gazebo physics real-time factor"
    )
    if real_time_factor < MINIMUM_REAL_TIME_FACTOR:
        raise MatrixError("The Gazebo physics real-time factor is below 2.90")
    adapter = str(renderer.get("gl_renderer") or renderer.get("device") or "NVIDIA")
    return GazeboGuiEvidence(
        document=document,
        raw=raw,
        adapter=adapter[:240],
        average_fps=average_fps,
        post_render_fps=post_render_fps,
        real_time_factor=real_time_factor,
    )


def load_viewer_startup_evidence(path: Path) -> GazeboGuiEvidence:
    raw = read_owned_bounded_file(path, MAXIMUM_RENDER_REPORT_BYTES)
    try:
        document = strict_json_loads(raw.decode("utf-8"))
    except (UnicodeDecodeError, json.JSONDecodeError, ValueError) as exc:
        raise MatrixError("The Gazebo render report is malformed") from exc
    if not isinstance(document, dict):
        raise MatrixError("The Gazebo render report is not a JSON object")
    return validate_viewer_startup_report(document, raw)


def load_active_probe_evidence(
    path: Path,
    expected_display: str,
    attestation: ActiveProbeAttestation,
) -> GazeboGuiEvidence:
    raw = read_owned_bounded_file(path, MAXIMUM_RENDER_REPORT_BYTES)
    try:
        document = strict_json_loads(raw.decode("utf-8"))
    except (UnicodeDecodeError, json.JSONDecodeError, ValueError) as exc:
        raise MatrixError("The active Gazebo GUI report is malformed") from exc
    if not isinstance(document, dict):
        raise MatrixError("The active Gazebo GUI report is not a JSON object")
    actual_sha256 = hashlib.sha256(raw).hexdigest()
    process = document.get("process") or {}
    if (
        not re.fullmatch(r"[0-9a-f]{64}", attestation.sha256)
        or actual_sha256 != attestation.sha256
        or process.get("pid") != attestation.process_id
        or process.get("start_ticks") != attestation.process_start_ticks
    ):
        raise MatrixError(
            "The active Gazebo GUI report is not bound to its live preflight process"
        )
    evidence = validate_viewer_startup_report(document, raw)
    render = document.get("render_measurement") or {}
    display = document.get("display") or {}
    warmup = finite_number(render.get("warmup_seconds"), "active-probe warmup")
    sample = finite_number(render.get("sample_seconds"), "active-probe sample window")
    if warmup < ACTIVE_PROBE_WARMUP_SECONDS * 0.95:
        raise MatrixError("The active Gazebo GUI warmup window was too short")
    if sample < ACTIVE_PROBE_SAMPLE_SECONDS * 0.98:
        raise MatrixError("The active Gazebo GUI sampling window was too short")
    if display.get("x11") != expected_display:
        raise MatrixError("The active Gazebo GUI probe used a different private display")
    return evidence


def active_probe_attestation(output: ProcessOutput) -> ActiveProbeAttestation:
    if output.returncode != 0:
        raise MatrixError("The active Gazebo GUI preflight did not complete")
    pattern = re.compile(
        re.escape(ACTIVE_PROBE_ATTESTATION_PREFIX)
        + r"([0-9a-f]{64}) ([1-9][0-9]*) ([1-9][0-9]*)"
    )
    matches = []
    for line in output.stdout.splitlines():
        match = pattern.fullmatch(line)
        if match is not None:
            matches.append(match.groups())
        elif line.startswith(ACTIVE_PROBE_ATTESTATION_PREFIX):
            raise MatrixError("The active Gazebo GUI attestation is malformed")
    if len(matches) != 1:
        raise MatrixError("The active Gazebo GUI attestation is missing or ambiguous")
    digest, process_id, start_ticks = matches[0]
    return ActiveProbeAttestation(digest, int(process_id), int(start_ticks))


def _argument_value(arguments: list[str], name: str) -> str | None:
    try:
        index = arguments.index(name)
    except ValueError:
        return None
    if index + 1 >= len(arguments):
        return None
    return arguments[index + 1]


def active_viewer_lease_directory(
    runtime_dir: Path,
    session_id: uuid.UUID,
    *,
    timeout: float,
    stop_event: threading.Event,
) -> Path:
    """Bind the lease directory to the publisher process for this session."""
    deadline = time.monotonic() + timeout
    expected_session = str(session_id)
    while time.monotonic() < deadline:
        if stop_event.is_set():
            raise KeyboardInterrupt
        matches: set[Path] = set()
        for process_path in Path("/proc").glob("[0-9]*"):
            try:
                if process_path.stat().st_uid != os.getuid():
                    continue
                raw = (process_path / "cmdline").read_bytes()
            except (FileNotFoundError, PermissionError, ProcessLookupError, OSError):
                continue
            if not raw or len(raw) > 64 * 1024:
                continue
            arguments = [
                item.decode("utf-8", errors="replace")
                for item in raw.rstrip(b"\0").split(b"\0")
            ]
            if not any(Path(item).name == "robotswarm-viewer-publisher" for item in arguments):
                continue
            if _argument_value(arguments, "--session-id") != expected_session:
                continue
            lease_text = _argument_value(arguments, "--lease-id")
            try:
                lease_id = uuid.UUID(str(lease_text))
            except (ValueError, AttributeError):
                raise MatrixError("The active viewer publisher has an invalid lease")
            candidate = runtime_dir / f"lease-{lease_id.hex}"
            if candidate.is_dir():
                matches.add(candidate)
        if len(matches) > 1:
            raise MatrixError("More than one viewer publisher owns the live session")
        if len(matches) == 1:
            directory = next(iter(matches))
            metadata = directory.lstat()
            if metadata.st_uid != os.getuid() or stat.S_IMODE(metadata.st_mode) != 0o700:
                raise MatrixError("The viewer lease runtime is not private")
            return directory
        time.sleep(0.2)
    raise MatrixError("The active HLS viewer lease could not be bound to the session")


def parse_process_environment(raw: bytes) -> dict[str, str]:
    if not raw or len(raw) > 256 * 1024:
        raise MatrixError("The viewer publisher environment is unavailable or too large")
    environment: dict[str, str] = {}
    for entry in raw.rstrip(b"\0").split(b"\0"):
        if b"=" not in entry:
            raise MatrixError("The viewer publisher environment is malformed")
        name, value = entry.split(b"=", 1)
        try:
            clean_name = name.decode("ascii")
            clean_value = value.decode("utf-8")
        except UnicodeDecodeError as exc:
            raise MatrixError("The viewer publisher environment is malformed") from exc
        if clean_name in environment:
            raise MatrixError("The viewer publisher environment has duplicate keys")
        environment[clean_name] = clean_value
    return environment


def bound_viewer_publisher(
    session_id: uuid.UUID,
    lease_directory: Path,
) -> BoundViewerPublisher:
    expected_session = str(session_id)
    if not lease_directory.name.startswith("lease-"):
        raise MatrixError("The bound viewer lease directory name is invalid")
    expected_lease = lease_directory.name[len("lease-") :]
    matches: list[BoundViewerPublisher] = []
    for process_path in Path("/proc").glob("[0-9]*"):
        try:
            if process_path.stat().st_uid != os.getuid():
                continue
            raw_arguments = (process_path / "cmdline").read_bytes()
            if not raw_arguments or len(raw_arguments) > 64 * 1024:
                continue
            arguments = [
                item.decode("utf-8", errors="replace")
                for item in raw_arguments.rstrip(b"\0").split(b"\0")
            ]
            publisher_argument = next(
                (
                    item
                    for item in arguments
                    if Path(item).name == "robotswarm-viewer-publisher"
                ),
                None,
            )
            if (
                publisher_argument is None
                or _argument_value(arguments, "--session-id") != expected_session
            ):
                continue
            lease_text = _argument_value(arguments, "--lease-id")
            if uuid.UUID(str(lease_text)).hex != expected_lease:
                continue
            raw_environment = (process_path / "environ").read_bytes()
        except (FileNotFoundError, PermissionError, ProcessLookupError, OSError, ValueError):
            continue
        publisher = Path(publisher_argument).resolve()
        if not publisher.is_file():
            continue
        matches.append(
            BoundViewerPublisher(
                process_path,
                publisher,
                parse_process_environment(raw_environment),
            )
        )
    if len(matches) != 1:
        raise MatrixError("The active viewer publisher runtime is ambiguous")
    return matches[0]


def _process_descendants(root: Path) -> list[Path]:
    pending = [root]
    visited: set[int] = set()
    descendants: list[Path] = []
    while pending:
        process_path = pending.pop()
        try:
            process_id = int(process_path.name)
        except ValueError:
            continue
        if process_id in visited:
            continue
        visited.add(process_id)
        try:
            children = (
                process_path / "task" / str(process_id) / "children"
            ).read_text(encoding="ascii")
        except (FileNotFoundError, PermissionError, ProcessLookupError, OSError):
            continue
        if process_path != root:
            descendants.append(process_path)
        pending.extend(
            Path("/proc") / value
            for value in children.split()
            if value.isdigit()
        )
    return descendants


def _namespace_process_ids(process_path: Path) -> set[int]:
    try:
        status = (process_path / "status").read_text(encoding="ascii")
    except (FileNotFoundError, PermissionError, ProcessLookupError, OSError):
        return set()
    match = re.search(r"^NSpid:\s+([0-9\s]+)$", status, re.MULTILINE)
    if match is None:
        return set()
    return {int(value) for value in match.group(1).split()}


def bound_viewer_gzclient(
    publisher: BoundViewerPublisher,
    startup: GazeboGuiEvidence,
) -> BoundViewerGzclient:
    process = startup.document.get("process")
    report_pid = process.get("pid") if isinstance(process, dict) else None
    report_executable = (
        process.get("executable") if isinstance(process, dict) else None
    )
    if isinstance(report_pid, bool) or not isinstance(report_pid, int) or report_pid <= 1:
        raise MatrixError("The viewer startup report has an invalid process identity")
    expected_executable = Path(
        _configured_executable(
            publisher.environment,
            "ROBOTSWARM_VIEWER_GZCLIENT",
            "gzclient",
        )
    )
    if (
        not isinstance(report_executable, str)
        or Path(report_executable).resolve() != expected_executable
    ):
        raise MatrixError("The startup report executable does not match gzclient")
    expected_display = str(
        (startup.document.get("display") or {}).get("x11") or ""
    )
    matches: list[BoundViewerGzclient] = []
    for process_path in _process_descendants(publisher.process_path):
        try:
            if process_path.stat().st_uid != os.getuid():
                continue
            executable = Path(os.readlink(process_path / "exe")).resolve()
            if executable != expected_executable:
                continue
            process_id = int(process_path.name)
            if report_pid not in ({process_id} | _namespace_process_ids(process_path)):
                continue
            arguments = (process_path / "cmdline").read_bytes().split(b"\0")
            if b"/viewer/plugin.so" not in arguments:
                continue
            environment = parse_process_environment(
                (process_path / "environ").read_bytes()
            )
        except (FileNotFoundError, PermissionError, ProcessLookupError, OSError):
            continue
        if environment.get("DISPLAY") != expected_display:
            continue
        matches.append(
            BoundViewerGzclient(process_path, executable, environment)
        )
    if len(matches) != 1:
        raise MatrixError("The startup report is not bound to one live viewer gzclient")
    return matches[0]


def _configured_executable(
    environment: dict[str, str],
    name: str,
    default: str,
) -> str:
    configured = environment.get(name, default)
    resolved = shutil.which(configured, path=environment.get("PATH") or os.defpath)
    if resolved is None:
        raise MatrixError("The active GUI probe is missing a required host executable")
    return str(Path(resolved).resolve())


def active_probe_runtime(
    lease_directory: Path,
    session_id: uuid.UUID,
    container: ContainerHandle,
    startup: GazeboGuiEvidence,
) -> ActiveProbeRuntime:
    publisher = bound_viewer_publisher(
        session_id, lease_directory
    )
    primary = bound_viewer_gzclient(publisher, startup)
    publisher_environment = publisher.environment
    display = str((startup.document.get("display") or {}).get("x11") or "")
    if not re.fullmatch(r":[1-9][0-9]{0,2}(?:\.0)?", display):
        raise MatrixError("The viewer startup report has an invalid private display")
    xauthority = lease_directory / "Xauthority"
    try:
        authority_metadata = xauthority.lstat()
    except OSError as exc:
        raise MatrixError("The private viewer Xauthority is unavailable") from exc
    if (
        not stat.S_ISREG(authority_metadata.st_mode)
        or authority_metadata.st_uid != os.getuid()
        or authority_metadata.st_size <= 0
        or authority_metadata.st_mode & 0o022
    ):
        raise MatrixError("The private viewer Xauthority is unsafe")
    try:
        private_ip = str(ipaddress.ip_address(container.private_ip))
        gateway = str(ipaddress.ip_address(container.network_gateway))
    except ValueError as exc:
        raise MatrixError("The active GUI probe has no verified private master") from exc
    if (
        primary.environment.get("ROS_MASTER_URI")
        != f"http://{private_ip}:11311"
        or primary.environment.get("GAZEBO_MASTER_URI")
        != f"http://{private_ip}:11345"
        or primary.environment.get("GZ_IP") != gateway
        or primary.environment.get("GAZEBO_IP") != gateway
        or primary.environment.get("XAUTHORITY") != "/viewer/Xauthority"
    ):
        raise MatrixError(
            "The primary viewer is not bound to the verified session masters"
        )

    asset_root = Path(
        publisher_environment.get(
            "ROBOTSWARM_VIEWER_ASSET_ROOT",
            str(publisher.executable.parent / "robotswarm-viewer-assets"),
        )
    ).resolve()
    ros_share = asset_root / "ros-share"
    model_paths = [asset_root / "models"]
    for raw_path in publisher_environment.get(
        "ROBOTSWARM_VIEWER_GAZEBO_MODEL_PATH", ""
    ).split(os.pathsep):
        if raw_path:
            model_paths.append(Path(raw_path).resolve())
    system_models = Path("/usr/share/gazebo-11/models")
    if system_models.is_dir():
        model_paths.append(system_models)
    model_paths = list(dict.fromkeys(model_paths))
    if not ros_share.is_dir() or not model_paths or any(
        not path.is_dir() for path in model_paths
    ):
        raise MatrixError("The active GUI probe assets are unavailable")

    sandbox = _configured_executable(
        publisher_environment, "ROBOTSWARM_VIEWER_SANDBOX", "bwrap"
    )
    gzclient = _configured_executable(
        publisher_environment, "ROBOTSWARM_VIEWER_GZCLIENT", "gzclient"
    )
    prefix = [
        sandbox,
        "--die-with-parent",
        "--unshare-all",
        "--share-net",
        "--new-session",
        "--ro-bind",
        "/usr",
        "/usr",
        "--symlink",
        "usr/bin",
        "/bin",
        "--symlink",
        "usr/sbin",
        "/sbin",
        "--symlink",
        "usr/lib",
        "/lib",
        "--symlink",
        "usr/lib64",
        "/lib64",
        "--ro-bind",
        "/etc",
        "/etc",
        "--proc",
        "/proc",
        "--dev",
        "/dev",
        "--dev-bind-try",
        "/dev/dxg",
        "/dev/dxg",
        "--dev-bind-try",
        "/dev/dri",
        "/dev/dri",
        "--ro-bind",
        "/sys",
        "/sys",
        "--tmpfs",
        "/tmp",
        "--tmpfs",
        "/run",
        "--dir",
        "/var",
        "--dir",
        "/var/cache",
        "--ro-bind-try",
        "/var/cache/fontconfig",
        "/var/cache/fontconfig",
        "--dir",
        "/opt",
        "--dir",
        "/opt/ros",
        "--dir",
        "/opt/ros/noetic",
        "--dir",
        "/opt/ros/noetic/share",
        "--ro-bind",
        str(ros_share),
        "/opt/ros/noetic/share",
        "--dir",
        "/viewer",
        "--bind",
        str(lease_directory),
        "/viewer",
    ]
    sandbox_model_paths = []
    for index, model_path in enumerate(model_paths):
        destination = f"/viewer/model-{index}"
        prefix.extend(["--ro-bind", str(model_path), destination])
        sandbox_model_paths.append(destination)
    prefix.extend(["--chdir", "/viewer", "--"])

    environment = {
        name: value
        for name, value in os.environ.items()
        if name in {"LANG", "LC_ALL", "LC_CTYPE", "PATH"}
    }
    for name in (
        "LD_LIBRARY_PATH",
        "LIBGL_DRIVERS_PATH",
        "MESA_D3D12_DEFAULT_ADAPTER_NAME",
        "MESA_LOADER_DRIVER_OVERRIDE",
    ):
        value = publisher_environment.get(name)
        if value:
            environment[name] = value
    probe_home = lease_directory / "matrix-active-probe-home"
    if probe_home.exists() or probe_home.is_symlink():
        raise MatrixError("The active GUI probe home is not fresh")
    try:
        probe_home.mkdir(mode=0o700)
        probe_home_metadata = probe_home.lstat()
    except OSError as exc:
        raise MatrixError("The active GUI probe home could not be created") from exc
    if (
        not stat.S_ISDIR(probe_home_metadata.st_mode)
        or probe_home_metadata.st_uid != os.getuid()
        or stat.S_IMODE(probe_home_metadata.st_mode) != 0o700
    ):
        raise MatrixError("The active GUI probe home is not private")
    environment.update(
        {
            "DISPLAY": display,
            "XAUTHORITY": "/viewer/Xauthority",
            "HOME": f"/viewer/{probe_home.name}",
            "ROS_HOME": f"/viewer/{probe_home.name}/.ros",
            "TMPDIR": "/tmp",
            "TMP": "/tmp",
            "TEMP": "/tmp",
            "ROS_MASTER_URI": f"http://{private_ip}:11311",
            "GAZEBO_MASTER_URI": f"http://{private_ip}:11345",
            "GZ_IP": gateway,
            "GAZEBO_IP": gateway,
            "GAZEBO_MODEL_PATH": os.pathsep.join(sandbox_model_paths),
            "GAZEBO_MODEL_DATABASE_URI": "",
            "QT_X11_NO_MITSHM": "1",
        }
    )
    environment.setdefault("MESA_D3D12_DEFAULT_ADAPTER_NAME", "NVIDIA")
    if not gzclient.startswith("/usr/"):
        raise MatrixError("The active GUI probe gzclient is outside the sandbox runtime")
    return ActiveProbeRuntime(
        tuple(prefix),
        environment,
        gzclient,
        primary.environment,
    )


def prepare_active_probe_inputs(
    docker: DockerHost,
    container: ContainerHandle,
    lease_directory: Path,
) -> tuple[Path, Path, dict[str, str]]:
    script = lease_directory / "matrix-active-gui-preflight.py"
    plugin = lease_directory / "matrix-active-gui-probe.so"
    script_raw = docker.copy_from_container(
        container,
        GUI_PREFLIGHT_PATH,
        script,
        MAXIMUM_PREFLIGHT_SCRIPT_BYTES,
    )
    plugin_raw = docker.copy_from_container(
        container,
        GUI_PROBE_PLUGIN_PATH,
        plugin,
        MAXIMUM_PROBE_PLUGIN_BYTES,
    )
    return script, plugin, {
        "preflightScriptSha256": hashlib.sha256(script_raw).hexdigest(),
        "probePluginSha256": hashlib.sha256(plugin_raw).hexdigest(),
    }


def create_active_probe_workspace() -> Path:
    workspace = Path(tempfile.mkdtemp(prefix="robotswarm-matrix-probe-", dir="/tmp"))
    try:
        workspace.chmod(0o700)
        metadata = workspace.lstat()
    except OSError as exc:
        with contextlib.suppress(OSError):
            workspace.rmdir()
        raise MatrixError("The active GUI probe workspace could not be secured") from exc
    if (
        not stat.S_ISDIR(metadata.st_mode)
        or metadata.st_uid != os.getuid()
        or stat.S_IMODE(metadata.st_mode) != 0o700
    ):
        with contextlib.suppress(OSError):
            workspace.rmdir()
        raise MatrixError("The active GUI probe workspace is not private")
    return workspace


def remove_active_probe_workspace(workspace: Path) -> bool:
    try:
        metadata = workspace.lstat()
    except FileNotFoundError:
        return True
    expected_parent = Path("/tmp").resolve()
    if (
        workspace.parent.resolve() != expected_parent
        or not re.fullmatch(r"robotswarm-matrix-probe-[A-Za-z0-9_-]+", workspace.name)
        or not stat.S_ISDIR(metadata.st_mode)
        or metadata.st_uid != os.getuid()
        or stat.S_IMODE(metadata.st_mode) != 0o700
    ):
        return False
    try:
        children = list(workspace.iterdir())
        for child in children:
            child_metadata = child.lstat()
            if (
                child.name not in {
                    "matrix-active-gui-preflight.py",
                    "matrix-active-gui-probe.so",
                }
                or not stat.S_ISREG(child_metadata.st_mode)
                or child_metadata.st_uid != os.getuid()
            ):
                return False
        for child in children:
            child.unlink()
        workspace.rmdir()
    except OSError:
        return False
    return not workspace.exists()


def _active_probe_process_identity(process_path: Path, token: str) -> tuple[int, int] | None:
    if re.fullmatch(r"[0-9a-f]{32}", token) is None:
        raise MatrixError("The active GUI probe token is invalid")
    try:
        process_id = int(process_path.name)
        if process_id <= 1 or process_id == os.getpid():
            return None
        if process_path.stat().st_uid != os.getuid():
            return None
        environment = (process_path / "environ").read_bytes().split(b"\0")
        marker = f"{ACTIVE_PROBE_TOKEN_ENV}={token}".encode("ascii")
        if marker not in environment:
            return None
        raw = (process_path / "stat").read_text(encoding="ascii")
        fields = raw[raw.rfind(")") + 2 :].split()
        if fields[0] == "Z":
            return None
        start_tick = int(fields[19])
    except (
        FileNotFoundError,
        IndexError,
        PermissionError,
        ProcessLookupError,
        OSError,
        ValueError,
    ):
        return None
    return process_id, start_tick


def active_probe_processes(token: str) -> list[tuple[int, int]]:
    matches: list[tuple[int, int]] = []
    for process_path in Path("/proc").glob("[0-9]*"):
        identity = _active_probe_process_identity(process_path, token)
        if identity is not None:
            matches.append(identity)
    return matches


def _x86_64_syscall(number: int, *arguments: Any) -> int:
    if os.uname().machine != "x86_64":
        raise OSError(
            errno.ENOSYS,
            "pidfd syscalls are not configured for this host",
        )
    libc = ctypes.CDLL(None, use_errno=True)
    syscall = libc.syscall
    syscall.restype = ctypes.c_long
    result = syscall(ctypes.c_long(number), *arguments)
    if result == -1:
        error_number = ctypes.get_errno()
        raise OSError(error_number, os.strerror(error_number))
    return int(result)


def _pidfd_open(process_id: int) -> int:
    return _x86_64_syscall(
        PIDFD_OPEN_SYSCALL_X86_64,
        ctypes.c_int(process_id),
        ctypes.c_uint(0),
    )


def _pidfd_send_signal(descriptor: int, number: int) -> None:
    _x86_64_syscall(
        PIDFD_SEND_SIGNAL_SYSCALL_X86_64,
        ctypes.c_int(descriptor),
        ctypes.c_int(number),
        ctypes.c_void_p(),
        ctypes.c_uint(0),
    )


def _signal_active_probe_process(
    identity: tuple[int, int], token: str, number: int
) -> None:
    process_id, _start_tick = identity
    try:
        descriptor = _pidfd_open(process_id)
    except OSError as exc:
        if exc.errno == errno.ESRCH:
            return
        raise CleanupError(
            "The active GUI probe could not be bound to a pidfd"
        ) from exc
    try:
        current = _active_probe_process_identity(
            Path("/proc") / str(process_id), token
        )
        if current != identity:
            return
        try:
            _pidfd_send_signal(descriptor, number)
        except OSError as exc:
            if exc.errno == errno.ESRCH:
                return
            raise CleanupError(
                "The active GUI probe could not be signalled through its pidfd"
            ) from exc
    finally:
        try:
            os.close(descriptor)
        except OSError as exc:
            raise CleanupError("The active GUI probe pidfd could not be closed") from exc


def stop_active_probe_processes(token: str, timeout: float = 8.0) -> bool:
    for identity in active_probe_processes(token):
        _signal_active_probe_process(identity, token, signal.SIGTERM)
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if not active_probe_processes(token):
            return True
        time.sleep(0.05)
    for identity in active_probe_processes(token):
        _signal_active_probe_process(identity, token, signal.SIGKILL)
    deadline = time.monotonic() + 3.0
    while time.monotonic() < deadline:
        if not active_probe_processes(token):
            return True
        time.sleep(0.05)
    return not active_probe_processes(token)


def build_active_probe_command(
    runtime: ActiveProbeRuntime,
    script: Path,
    plugin: Path,
    report: Path,
) -> list[str]:
    sandbox = list(runtime.command_prefix)
    try:
        chdir_index = sandbox.index("--chdir")
    except ValueError as exc:
        raise MatrixError("The active GUI sandbox has no private working directory") from exc
    sandbox[chdir_index:chdir_index] = [
        "--ro-bind",
        str(script),
        f"/viewer/{script.name}",
        "--ro-bind",
        str(plugin),
        f"/viewer/{plugin.name}",
    ]
    return [
        *sandbox,
        "/usr/bin/python3",
        f"/viewer/{script.name}",
        "--gzclient",
        runtime.gzclient,
        "--plugin",
        f"/viewer/{plugin.name}",
        "--report",
        f"/viewer/{report.name}",
        "--warmup-seconds",
        f"{ACTIVE_PROBE_WARMUP_SECONDS:.1f}",
        "--sample-seconds",
        f"{ACTIVE_PROBE_SAMPLE_SECONDS:.1f}",
        "--timeout-seconds",
        f"{ACTIVE_PROBE_TIMEOUT_SECONDS:.1f}",
        "--min-render-fps",
        f"{MINIMUM_STARTUP_SCENE_FPS:.1f}",
        "--min-real-time-factor",
        f"{MINIMUM_REAL_TIME_FACTOR:.2f}",
    ]


def _wait_for_timed_start(
    record: TimedCommand,
    thread: threading.Thread,
    stop_event: threading.Event,
    timeout: float,
) -> None:
    deadline = time.monotonic() + timeout
    while record.started is None and thread.is_alive() and time.monotonic() < deadline:
        if stop_event.is_set():
            raise KeyboardInterrupt
        time.sleep(0.02)
    if record.started is None:
        raise MatrixError("A concurrent acceptance process did not start")


def _join_timed_command(
    record: TimedCommand,
    thread: threading.Thread,
    stop_event: threading.Event,
    timeout: float,
) -> None:
    deadline = time.monotonic() + timeout
    while thread.is_alive() and time.monotonic() < deadline:
        thread.join(timeout=0.25)
        if stop_event.is_set():
            continue
    if thread.is_alive():
        raise MatrixError("A concurrent acceptance process did not terminate")
    if isinstance(record.error, KeyboardInterrupt):
        raise KeyboardInterrupt
    if record.error is not None:
        raise record.error


def run_active_scenario_gate(
    *,
    docker: DockerHost,
    container: ContainerHandle,
    scenario: ScenarioSpec,
    scenario_timeout: float,
    probe_command: Sequence[str],
    probe_environment: dict[str, str],
    ui: Any,
    stop_event: threading.Event,
) -> tuple[ProcessOutput, ProcessOutput, dict[str, Any], dict[str, Any]]:
    """Run the official GUI probe wholly inside the live scenario interval."""
    cancel_event = threading.Event()
    scenario_run = TimedCommand()
    probe_run = TimedCommand()
    run_token = uuid.uuid4().hex
    task_id = f"matrix-{run_token}"
    task_active_before: dict[str, Any] | None = None
    task_active_after: dict[str, Any] | None = None
    task_active_before_at: float | None = None
    task_active_after_at: float | None = None

    def run_scenario() -> None:
        try:
            scenario_run.output = docker.run_acceptance(
                container,
                scenario,
                scenario_timeout,
                task_id,
                run_token,
                cancel_event=cancel_event,
                on_started=lambda value: setattr(scenario_run, "started", value),
                on_exited=lambda value: setattr(scenario_run, "finished", value),
            )
        except BaseException as exc:
            scenario_run.error = exc
        finally:
            if scenario_run.finished is None:
                scenario_run.finished = time.monotonic()

    def run_probe() -> None:
        try:
            probe_run.output = run_command(
                probe_command,
                timeout=ACTIVE_PROBE_TIMEOUT_SECONDS + 10,
                stop_event=stop_event,
                cancel_event=cancel_event,
                environment=probe_environment,
                on_started=lambda value: setattr(probe_run, "started", value),
                on_exited=lambda value: setattr(probe_run, "finished", value),
            )
        except BaseException as exc:
            probe_run.error = exc
            cancel_event.set()
        finally:
            if probe_run.finished is None:
                probe_run.finished = time.monotonic()

    scenario_thread = threading.Thread(
        target=run_scenario,
        name="robotswarm-matrix-scenario",
    )
    probe_thread = threading.Thread(
        target=run_probe,
        name="robotswarm-matrix-active-gui-probe",
    )
    scenario_thread.start()
    try:
        _wait_for_timed_start(scenario_run, scenario_thread, stop_event, 10)
        task_active_before = docker.wait_task_active(
            container,
            task_id,
            scenario,
            min(120.0, scenario_timeout),
        )
        task_active_before_at = time.monotonic()
        if not scenario_thread.is_alive():
            raise MatrixError("The ROS scenario ended before active visual sampling")
        probe_thread.start()
        _wait_for_timed_start(probe_run, probe_thread, stop_event, 10)
        video_started = time.monotonic()
        try:
            video = validate_browser_video(
                ui.video_metrics(ACTIVE_SCENARIO_VIDEO_SECONDS),
                requested_seconds=ACTIVE_SCENARIO_VIDEO_SECONDS,
            )
        except BaseException:
            cancel_event.set()
            raise
        finally:
            video_finished = time.monotonic()
        _join_timed_command(
            probe_run,
            probe_thread,
            stop_event,
            ACTIVE_PROBE_TIMEOUT_SECONDS + 20,
        )
        task_active_after = docker.wait_task_active(
            container,
            task_id,
            scenario,
            3.0,
        )
        task_active_after_at = time.monotonic()
        if not scenario_thread.is_alive():
            raise MatrixError("The ROS scenario ended before its final activity proof")
        _join_timed_command(
            scenario_run,
            scenario_thread,
            stop_event,
            scenario_timeout + 50,
        )
    except BaseException:
        cancel_event.set()
        if probe_thread.ident is not None:
            probe_thread.join(timeout=50)
        scenario_thread.join(timeout=100)
        if scenario_thread.is_alive():
            raise CleanupError("The ROS scenario thread survived explicit cancellation")
        raise

    if (
        scenario_run.output is None
        or probe_run.output is None
        or scenario_run.started is None
        or scenario_run.finished is None
        or probe_run.started is None
        or probe_run.finished is None
        or task_active_before is None
        or task_active_after is None
        or task_active_before_at is None
        or task_active_after_at is None
    ):
        raise MatrixError("The concurrent acceptance evidence is incomplete")
    scenario_spans_probe = (
        scenario_run.started <= probe_run.started
        and scenario_run.finished >= probe_run.finished
    )
    video_is_overlapped = (
        scenario_run.started <= video_started
        and scenario_run.finished >= video_finished
        and probe_run.started <= video_started
        and probe_run.finished >= video_finished
    )

    probe_duration = probe_run.finished - probe_run.started
    scenario_overlap = max(
        0.0,
        min(scenario_run.finished, probe_run.finished)
        - max(scenario_run.started, probe_run.started),
    )
    video_duration = video_finished - video_started
    overlap = {
        "clock": "time.monotonic",
        "scenarioProcessActiveThroughoutProbe": scenario_spans_probe,
        "scenarioProcessActiveThroughoutVideo": video_is_overlapped,
        "probeActiveThroughoutVideo": video_is_overlapped,
        "taskActiveConfirmedBeforeProbe": task_active_before["confirmed"],
        "taskActiveConfirmedAfterProbe": task_active_after["confirmed"],
        "taskActivityCorrelation": "independentRosTaskAndBehaviorStatus",
        "taskActivityBeforeProbe": task_active_before,
        "taskActivityAfterProbe": task_active_after,
        "probeDurationSeconds": probe_duration,
        "scenarioProbeOverlapSeconds": scenario_overlap,
        "videoDurationSeconds": video_duration,
        "probeStartOffsetFromScenarioSeconds": (
            probe_run.started - scenario_run.started
        ),
        "probeStartAfterTaskActiveConfirmationSeconds": (
            probe_run.started - task_active_before_at
        ),
        "taskActiveConfirmationAfterProbeSeconds": (
            task_active_after_at - probe_run.finished
        ),
        "probeEndBeforeScenarioSeconds": (
            scenario_run.finished - probe_run.finished
        ),
        "videoStartOffsetFromScenarioSeconds": video_started - scenario_run.started,
        "videoEndBeforeScenarioSeconds": scenario_run.finished - video_finished,
    }
    video.update(
        {
            "phase": "duringActiveScenarioAndGuiProbe",
            "scope": "liveHlsUnderScenarioLoad",
            "scenarioProcessActiveThroughout": video_is_overlapped,
            "activeProbeThroughout": video_is_overlapped,
            "visualMotionMeasured": False,
        }
    )
    return scenario_run.output, probe_run.output, video, overlap


def validate_active_overlap(overlap: dict[str, Any]) -> None:
    if overlap.get("clock") != "time.monotonic":
        raise MatrixError("The active-load overlap did not use a monotonic clock")
    if (
        overlap.get("scenarioProcessActiveThroughoutProbe") is not True
        or overlap.get("scenarioProcessActiveThroughoutVideo") is not True
        or overlap.get("probeActiveThroughoutVideo") is not True
        or overlap.get("taskActiveConfirmedBeforeProbe") is not True
        or overlap.get("taskActiveConfirmedAfterProbe") is not True
        or overlap.get("taskActivityCorrelation")
        != "independentRosTaskAndBehaviorStatus"
    ):
        raise MatrixError("The ROS scenario did not span the complete active sampling window")
    probe_duration = finite_number(
        overlap.get("probeDurationSeconds"), "active-probe process duration"
    )
    scenario_overlap = finite_number(
        overlap.get("scenarioProbeOverlapSeconds"), "scenario/probe overlap"
    )
    video_duration = finite_number(
        overlap.get("videoDurationSeconds"), "active HLS wall interval"
    )
    probe_after_task = finite_number(
        overlap.get("probeStartAfterTaskActiveConfirmationSeconds"),
        "task activity to active-probe offset",
    )
    task_after_probe = finite_number(
        overlap.get("taskActiveConfirmationAfterProbeSeconds"),
        "active-probe to task activity offset",
    )
    if probe_duration < ACTIVE_PROBE_WARMUP_SECONDS + ACTIVE_PROBE_SAMPLE_SECONDS:
        raise MatrixError("The active GUI probe did not cover its complete sample budget")
    if scenario_overlap + 0.001 < probe_duration:
        raise MatrixError("The ROS scenario ended before the active GUI probe")
    if video_duration < ACTIVE_SCENARIO_VIDEO_SECONDS * 0.90:
        raise MatrixError("The active HLS measurement wall interval was too short")
    if probe_after_task < 0:
        raise MatrixError("The active GUI probe started before correlated task activity")
    if task_after_probe < 0:
        raise MatrixError("The final task activity proof preceded the Gazebo sample")


def require_one_session_uuid(ui: Any) -> uuid.UUID:
    sessions = ui._occupying_sessions()
    if len(sessions) != 1:
        raise MatrixError("Account A does not own exactly one active simulation session")
    try:
        parsed = uuid.UUID(str(sessions[0].get("id")))
    except (ValueError, AttributeError) as exc:
        raise MatrixError("The account session has an invalid private identifier") from exc
    if parsed.version not in {1, 2, 3, 4, 5}:
        raise MatrixError("The account session has an invalid private identifier")
    return parsed


def decoded_hls_fps(state: dict[str, Any]) -> float:
    if state.get("hlsInteractive") is not True or state.get("status") != "En vivo":
        raise MatrixError("The browser viewer is not a live interactive HLS stream")
    match = re.fullmatch(r"Video (\d+(?:\.\d+)?) FPS", str(state.get("fps") or ""))
    if match is None:
        raise MatrixError("The browser viewer did not expose decoded HLS FPS")
    value = float(match.group(1))
    if not math.isfinite(value) or value <= 0:
        raise MatrixError("The browser viewer decoded FPS value is invalid")
    return value


def validate_browser_video(
    metrics: dict[str, Any],
    requested_seconds: float = ACTIVE_SCENARIO_VIDEO_SECONDS,
) -> dict[str, Any]:
    """Validate decoded and presented HLS frames over a bounded interval."""
    if metrics.get("requestVideoFrameCallbackSupported") is not True:
        raise MatrixError("The browser cannot count presented HLS video frames")
    if metrics.get("getVideoPlaybackQualitySupported") is not True:
        raise MatrixError("The browser cannot measure decoded and dropped HLS frames")

    elapsed = finite_number(metrics.get("elapsedSeconds"), "browser video interval")
    media_advanced = finite_number(
        metrics.get("mediaTimeAdvancedSeconds"), "browser media-time advance"
    )
    callback_frames = finite_number(
        metrics.get("callbackFrames"), "presented browser frame count"
    )
    decoded_frames = finite_number(
        metrics.get("decodedFrames"), "decoded browser frame count"
    )
    callback_fps = finite_number(metrics.get("callbackFps"), "presented browser FPS")
    decoded_fps = finite_number(metrics.get("decodedFps"), "decoded browser FPS")
    dropped_frames = finite_number(
        metrics.get("droppedFrames"), "dropped browser frame count"
    )
    dropped_ratio = finite_number(metrics.get("droppedRatio"), "browser dropped ratio")
    playback_rate = finite_number(metrics.get("playbackRate"), "browser playback rate")
    visible_fraction = finite_number(
        metrics.get("visibleFraction"), "browser video visible fraction"
    )
    width = finite_number(metrics.get("width"), "browser video width")
    height = finite_number(metrics.get("height"), "browser video height")

    if elapsed < requested_seconds * 0.90:
        raise MatrixError("The browser video measurement interval was too short")
    if media_advanced < requested_seconds * 0.70:
        raise MatrixError("The HLS media time did not keep progressing")
    if callback_frames < 1 or decoded_frames < 1:
        raise MatrixError("The HLS video produced no frames")
    if (
        callback_fps < MINIMUM_BROWSER_VIDEO_FPS
        or decoded_fps < MINIMUM_BROWSER_VIDEO_FPS
    ):
        raise MatrixError("The HLS video stayed below the 30 FPS target")
    if dropped_frames < 0 or not 0 <= dropped_ratio <= MAXIMUM_BROWSER_DROPPED_RATIO:
        raise MatrixError("The HLS video exceeded its dropped-frame bound")
    if (
        int(metrics.get("readyState") or 0) < 2
        or metrics.get("paused") is not False
        or playback_rate <= 0
        or width <= 0
        or height <= 0
        or visible_fraction < 0.95
    ):
        raise MatrixError("The HLS video was not visibly playing")

    return {
        "measurementSeconds": requested_seconds,
        "elapsedSeconds": elapsed,
        "mediaTimeAdvancedSeconds": media_advanced,
        "targetFps": TARGET_BROWSER_VIDEO_FPS,
        "minimumAcceptedFps": MINIMUM_BROWSER_VIDEO_FPS,
        "presentedFrames": int(callback_frames),
        "presentedFps": callback_fps,
        "decodedFrames": int(decoded_frames),
        "decodedFps": decoded_fps,
        "droppedFrames": int(dropped_frames),
        "droppedRatio": dropped_ratio,
        "maximumDroppedRatio": MAXIMUM_BROWSER_DROPPED_RATIO,
        "mediaProgressing": True,
        "requestVideoFrameCallback": True,
        "getVideoPlaybackQuality": True,
    }


def close_viewer(ui: Any, timeout: float) -> dict[str, Any]:
    ui.normalize_viewer()
    if not ui.has_button("Cerrar visor"):
        already_closed = ui.cdp.evaluate(
            """
                !document.querySelector('[data-testid="private-viewer"]') &&
                !document.querySelector('[data-testid="viewer-closing"]')
            """
        )
        if already_closed is not True:
            raise MatrixError("The viewer could not be closed from its current state")
        return {"requested": False, "closed": True}
    ui.click_button("Cerrar visor")
    ui.wait_js(
        """
            !document.querySelector('[data-testid="private-viewer"]') &&
            !document.querySelector('[data-testid="viewer-closing"]') &&
            [...document.querySelectorAll('button')].some(button =>
                button.textContent.trim() === 'Abrir visor' && !button.disabled)
        """,
        timeout,
        "completed viewer close",
    )
    return {"requested": True, "closed": True}


def publisher_for_session_exists(session_id: uuid.UUID) -> bool:
    expected_session = str(session_id)
    for process_path in Path("/proc").glob("[0-9]*"):
        try:
            if process_path.stat().st_uid != os.getuid():
                continue
            raw = (process_path / "cmdline").read_bytes()
        except (FileNotFoundError, PermissionError, ProcessLookupError, OSError):
            continue
        arguments = [
            item.decode("utf-8", errors="replace")
            for item in raw.rstrip(b"\0").split(b"\0")
            if item
        ]
        if (
            any(Path(item).name == "robotswarm-viewer-publisher" for item in arguments)
            and _argument_value(arguments, "--session-id") == expected_session
        ):
            return True
    return False


def wait_for_resource_cleanup(
    docker: DockerHost,
    session_id: uuid.UUID,
    lease_directory: Path | None,
    timeout: float,
) -> dict[str, bool]:
    deadline = time.monotonic() + timeout
    latest = {
        "containerAbsent": False,
        "networkAbsent": False,
        "leaseRuntimeAbsent": lease_directory is None,
        "viewerPublisherAbsent": False,
    }
    while time.monotonic() < deadline:
        container_absent, network_absent = docker.resources_absent(session_id)
        latest = {
            "containerAbsent": container_absent,
            "networkAbsent": network_absent,
            "leaseRuntimeAbsent": lease_directory is None or not lease_directory.exists(),
            "viewerPublisherAbsent": not publisher_for_session_exists(session_id),
        }
        if all(latest.values()):
            return latest
        time.sleep(0.5)
    return latest


def cleanup_case(
    ui: Any,
    docker: DockerHost,
    session_id: uuid.UUID | None,
    lease_directory: Path | None,
    *,
    timeout: float,
) -> dict[str, Any]:
    """Close viewer first, then release the session and prove host cleanup."""
    cleanup_session_id = session_id
    resource_identity_known = session_id is not None or not ui.create_requested
    identity_error: str | None = None
    if cleanup_session_id is None and ui.create_requested:
        try:
            occupants = ui._occupying_sessions()
            if len(occupants) > 1:
                raise MatrixError("Uncertain creation exposed more than one active session")
            if len(occupants) == 1:
                cleanup_session_id = uuid.UUID(str(occupants[0].get("id")))
                resource_identity_known = True
        except Exception as exc:
            identity_error = redact_text(str(exc))

    try:
        viewer_was_present = bool(
            ui.cdp.evaluate(
                """
                    Boolean(
                        document.querySelector('[data-testid="private-viewer"]') ||
                        document.querySelector('[data-testid="viewer-closing"]')
                    )
                """
            )
        )
    except Exception:
        viewer_was_present = lease_directory is not None
    lease_identity_known = lease_directory is not None or not viewer_was_present

    result: dict[str, Any] = {
        "viewerClosed": False,
        "sessionStopped": False,
        "workspaceReleased": False,
        "resourceIdentityKnown": resource_identity_known,
        "leaseBindingKnown": lease_identity_known,
        "containerAbsent": resource_identity_known and cleanup_session_id is None,
        "networkAbsent": resource_identity_known and cleanup_session_id is None,
        "leaseRuntimeAbsent": lease_identity_known and lease_directory is None,
        "viewerPublisherAbsent": resource_identity_known and cleanup_session_id is None,
        "complete": False,
    }
    if identity_error:
        result["resourceIdentityError"] = identity_error
    original_event = ui.stop_event
    ui.stop_event = threading.Event()
    try:
        try:
            viewer = close_viewer(ui, min(timeout, 120))
            result["viewerClosed"] = bool(viewer.get("closed"))
        except Exception as exc:
            result["viewerError"] = redact_text(str(exc))

        try:
            stopped = ui.stop_created_session(timeout=timeout)
            result["sessionStopped"] = bool(stopped.get("released")) or not stopped.get(
                "requested", True
            )
            if result["sessionStopped"]:
                ui.wait_js(
                    """
                        [...document.querySelectorAll('button')].some(button =>
                            button.textContent.trim() === 'Crear simulación' &&
                            !button.disabled)
                    """,
                    min(timeout, 60),
                    "released simulation workspace",
                )
                result["workspaceReleased"] = True
        except Exception as exc:
            result["sessionError"] = redact_text(str(exc))

        if cleanup_session_id is not None:
            resources = wait_for_resource_cleanup(
                docker, cleanup_session_id, lease_directory, timeout
            )
            if not lease_identity_known:
                resources["leaseRuntimeAbsent"] = False
            result.update(resources)
        result["complete"] = all(
            result.get(key) is True
            for key in (
                "viewerClosed",
                "sessionStopped",
                "workspaceReleased",
                "resourceIdentityKnown",
                "leaseBindingKnown",
                "containerAbsent",
                "networkAbsent",
                "leaseRuntimeAbsent",
                "viewerPublisherAbsent",
            )
        )
        return result
    finally:
        ui.stop_event = original_event


def scenario_selection(names: Sequence[str] | None) -> list[ScenarioSpec]:
    if not names:
        return list(SCENARIOS)
    if len(set(names)) != len(names):
        raise MatrixError("Each partial-matrix scenario may be selected only once")
    try:
        return [SCENARIOS_BY_NAME[name] for name in names]
    except KeyError as exc:
        raise MatrixError("The partial matrix contains an unknown scenario") from exc


def evidence_bundle_hash(hashes: dict[str, str]) -> str:
    payload = json.dumps(hashes, sort_keys=True, separators=(",", ":")).encode("ascii")
    return hashlib.sha256(payload).hexdigest()


def json_evidence_hash(value: Any) -> str:
    payload = json.dumps(
        value,
        sort_keys=True,
        separators=(",", ":"),
        ensure_ascii=False,
        allow_nan=False,
    ).encode("utf-8")
    return hashlib.sha256(payload).hexdigest()


def case_artifact_prefix(index: int, scenario: ScenarioSpec) -> str:
    return f"{index:02d}-{scenario.name}"


def require_case_cleanup_before_next(case_report: dict[str, Any]) -> None:
    if not case_report.get("cleanup", {}).get("complete"):
        raise CleanupError(
            "A case did not release its viewer, session, and Docker resources"
        )


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Run the visible production ROS/Gazebo acceptance matrix"
    )
    parser.add_argument(
        "--execute-production",
        action="store_true",
        required=True,
        help="authorize production session, viewer, and Docker mutations",
    )
    parser.add_argument("--deployment-commit", required=True, help="full deployed Git SHA")
    parser.add_argument(
        "--credentials", type=Path, required=True, help="private 0600 test credentials"
    )
    parser.add_argument(
        "--chrome", type=Path, required=True, help="visible Windows Chrome executable"
    )
    parser.add_argument(
        "--profile-root",
        type=Path,
        required=True,
        help="Windows directory for the ephemeral profile",
    )
    parser.add_argument("--output", type=Path, required=True, help="sanitized 0600 matrix JSON")
    parser.add_argument("--url", default=DEFAULT_URL)
    parser.add_argument("--docker", default="docker")
    parser.add_argument("--viewer-runtime-dir", type=Path, default=DEFAULT_VIEWER_RUNTIME)
    parser.add_argument("--cdp-port", type=int, default=9342)
    parser.add_argument("--ready-timeout", type=float, default=480)
    parser.add_argument("--viewer-timeout", type=float, default=240)
    parser.add_argument("--scenario-timeout", type=float, default=900)
    parser.add_argument("--cleanup-timeout", type=float, default=300)
    parser.add_argument(
        "--scenario",
        action="append",
        choices=tuple(SCENARIOS_BY_NAME),
        help="run one named case; repeat for a partial matrix (default: all 14)",
    )
    return parser


def parse_args(argv: Sequence[str] | None = None) -> argparse.Namespace:
    parser = build_parser()
    args = parser.parse_args(argv)
    if not SHA_PATTERN.fullmatch(args.deployment_commit):
        parser.error("--deployment-commit must be a full lowercase Git SHA")
    if not 1024 <= args.cdp_port <= 65535:
        parser.error("--cdp-port must be a non-privileged TCP port")
    for name in ("ready_timeout", "viewer_timeout", "scenario_timeout", "cleanup_timeout"):
        value = getattr(args, name)
        if not math.isfinite(value) or value <= 0:
            parser.error(f"--{name.replace('_', '-')} must be greater than zero")
    try:
        args.selected_scenarios = scenario_selection(args.scenario)
    except MatrixError as exc:
        parser.error(str(exc))
    return args


def screenshot_evidence(details: dict[str, Any]) -> dict[str, Any]:
    return {
        "file": str(details.get("file") or ""),
        "sha256": str(details.get("sha256") or ""),
        "bytes": int(details.get("bytes") or 0),
        "sanitized": True,
    }


def run_one_case(
    *,
    index: int,
    scenario: ScenarioSpec,
    args: argparse.Namespace,
    ui: Any,
    docker: DockerHost,
    evidence_dir: Path,
    credentials: dict[str, str],
    stop_event: threading.Event,
) -> tuple[dict[str, Any], bool]:
    case_report: dict[str, Any] = {
        "scenario": scenario.name,
        "behavior": scenario.behavior,
        "robotCount": scenario.robot_count,
        "startedAt": utc_now(),
        "status": "running",
        "cleanup": {"complete": False},
        "hashes": {},
    }
    session_id: uuid.UUID | None = None
    lease_directory: Path | None = None
    probe_workspace: Path | None = None
    probe_token: str | None = None
    interrupted = False
    try:
        if ui._occupying_sessions():
            raise MatrixError("Account A already owns an active session")
        ui.create_session(scenario.robot_count)
        case_report["session"] = ui.wait_ready(scenario.robot_count, args.ready_timeout)
        session_id = require_one_session_uuid(ui)

        container, container_evidence = docker.verify_session(
            session_id, args.deployment_commit
        )
        case_report["container"] = container_evidence

        ui.open_viewer(args.viewer_timeout)
        initial_viewer = ui.require_interactive_hls()
        case_report["viewer"] = {
            "transport": "HLS",
            "interactive": True,
            "decodedFps": decoded_hls_fps(initial_viewer),
        }
        lease_directory = active_viewer_lease_directory(
            args.viewer_runtime_dir,
            session_id,
            timeout=args.viewer_timeout,
            stop_event=stop_event,
        )

        startup = load_viewer_startup_evidence(
            lease_directory / "render-report.json"
        )
        startup_name = (
            case_artifact_prefix(index, scenario)
            + "-viewer-startup-render-report.json"
        )
        write_bytes_secure(evidence_dir / startup_name, startup.raw)
        startup_sha = hashlib.sha256(startup.raw).hexdigest()
        case_report["viewerStartupScene"] = {
            "phase": "viewerStartupBeforeScenario",
            "scope": "startupGpuCapability",
            "activeScenarioMotionMeasured": False,
            "file": startup_name,
            "adapter": redact_text(startup.adapter),
            "nvidiaD3d12": True,
            "minimumStartupFps": MINIMUM_STARTUP_SCENE_FPS,
            "averageStartupFps": startup.average_fps,
            "postRenderStartupFps": startup.post_render_fps,
            "minimumStartupPhysicsRealTimeFactor": MINIMUM_REAL_TIME_FACTOR,
            "startupPhysicsRealTimeFactor": startup.real_time_factor,
        }
        case_report["hashes"]["viewerStartupSceneReportSha256"] = startup_sha

        runtime = active_probe_runtime(
            lease_directory,
            session_id,
            container,
            startup,
        )
        probe_workspace = create_active_probe_workspace()
        probe_token = uuid.uuid4().hex
        runtime.environment[ACTIVE_PROBE_TOKEN_ENV] = probe_token
        probe_script, probe_plugin, probe_input_hashes = prepare_active_probe_inputs(
            docker,
            container,
            probe_workspace,
        )
        probe_report_path = lease_directory / "matrix-active-gui-report.json"
        if probe_report_path.exists() or probe_report_path.is_symlink():
            raise MatrixError("The active Gazebo GUI report path is not fresh")
        probe_command = build_active_probe_command(
            runtime,
            probe_script,
            probe_plugin,
            probe_report_path,
        )
        roster_before = docker.verify_full_roster(container, scenario.robot_count)
        child, probe_output, video, overlap = run_active_scenario_gate(
            docker=docker,
            container=container,
            scenario=scenario,
            scenario_timeout=args.scenario_timeout,
            probe_command=probe_command,
            probe_environment=runtime.environment,
            ui=ui,
            stop_event=stop_event,
        )
        if active_probe_processes(probe_token):
            raise CleanupError("The active GUI probe sandbox left a live process")
        ros = parse_ros_protocol(child, scenario)
        safe_result = sanitize_report_value(
            ros.result, (credentials["email"], credentials["password"])
        )
        safe_summary = sanitize_report_value(
            ros.summary, (credentials["email"], credentials["password"])
        )
        case_report["ros"] = {
            "exitCode": ros.returncode,
            "protocolParsed": True,
            "taskCleanupVerified": False,
            "minimumRealTimeFactor": MINIMUM_REAL_TIME_FACTOR,
            "result": safe_result,
            "summary": safe_summary,
        }
        case_report["hashes"].update(
            {
                "resultJsonSha256": ros.result_sha256,
                "summaryJsonSha256": ros.summary_sha256,
                **probe_input_hashes,
            }
        )
        case_report["activeScenarioVideo"] = video
        case_report["activeScenarioOverlap"] = overlap
        case_report["hashes"]["activeScenarioVideoMetricsSha256"] = (
            json_evidence_hash(video)
        )
        if probe_output.returncode != 0:
            case_report["activeScenarioGuiProbeFailure"] = (
                classify_active_probe_failure(probe_output)
            )
            raise MatrixError("The official active Gazebo GUI preflight failed")

        probe_attestation = active_probe_attestation(probe_output)
        active_probe = load_active_probe_evidence(
            probe_report_path,
            str((startup.document.get("display") or {}).get("x11") or ""),
            probe_attestation,
        )
        active_probe_name = (
            case_artifact_prefix(index, scenario)
            + "-active-scenario-gui-report.json"
        )
        write_bytes_secure(evidence_dir / active_probe_name, active_probe.raw)
        active_probe_sha = hashlib.sha256(active_probe.raw).hexdigest()
        case_report["activeScenarioGuiProbe"] = {
            "phase": "duringScenarioProcess",
            "scope": "officialGazeboGuiPreflightUnderScenarioLoad",
            "officialPreflight": True,
            "samePrivateDisplayAndMaster": True,
            "file": active_probe_name,
            "exitCode": probe_output.returncode,
            "adapter": redact_text(active_probe.adapter),
            "nvidiaD3d12": True,
            "minimumFps": MINIMUM_STARTUP_SCENE_FPS,
            "averageFps": active_probe.average_fps,
            "postRenderFps": active_probe.post_render_fps,
            "minimumRealTimeFactor": MINIMUM_REAL_TIME_FACTOR,
            "realTimeFactor": active_probe.real_time_factor,
            "warmupSeconds": ACTIVE_PROBE_WARMUP_SECONDS,
            "sampleSeconds": ACTIVE_PROBE_SAMPLE_SECONDS,
            "scenarioProcessActiveThroughout": overlap[
                "scenarioProcessActiveThroughoutProbe"
            ],
            "visualMotionMeasured": False,
        }
        case_report["hashes"]["activeScenarioGuiReportSha256"] = active_probe_sha

        # Persist the bounded, sanitized protocol before applying pass/fail
        # gates.  A functional failure must keep its useful diagnostics.
        ros_rtf = validate_ros_evidence(ros)
        if scenario.behavior == "transport":
            case_report["transportObjectPreference"] = (
                validate_transport_object_preference(
                    ros.result, scenario.robot_count
                )
            )
            if scenario.name == "transport_grf_n2":
                case_report["transportN2Contract"] = (
                    validate_transport_n2_contract(ros.result)
                )
        validate_active_overlap(overlap)
        case_report["ros"].update(
            {
                "taskCleanupVerified": True,
                "fleetDeletionRequestedByRunner": False,
                "observedRealTimeFactor": ros_rtf,
            }
        )

        post_session = ui.wait_ready(
            scenario.robot_count, min(args.ready_timeout, 60)
        )
        roster_after = docker.verify_full_roster(container, scenario.robot_count)
        final_viewer = ui.require_interactive_hls()
        case_report["viewer"]["decodedFpsAfterScenario"] = decoded_hls_fps(final_viewer)
        case_report["postScenarioFullRoster"] = {
            "phase": "afterTaskCleanupBeforeSessionCleanup",
            "expectedRobots": scenario.robot_count,
            "uiActiveRobots": post_session["activeRobots"],
            "gazeboBeforeScenario": roster_before,
            "gazeboAfterScenario": roster_after,
            "retainedAcrossScenario": True,
        }

        screenshot_name = case_artifact_prefix(index, scenario) + "-browser.png"
        screenshot = ui.screenshot(
            evidence_dir / screenshot_name,
            credentials["email"],
            credentials["password"],
        )
        clean_screenshot = screenshot_evidence(screenshot)
        case_report["browserScreenshot"] = clean_screenshot
        case_report["hashes"]["browserPngSha256"] = clean_screenshot["sha256"]
        case_report["status"] = "passed"
    except KeyboardInterrupt:
        interrupted = True
        case_report["status"] = "interrupted"
        case_report["failure"] = "Interrupted by the operator"
    except Exception as exc:
        case_report["status"] = "failed"
        case_report["failure"] = redact_text(
            str(exc), (credentials["email"], credentials["password"])
        )
    finally:
        try:
            active_probe_process_absent = (
                probe_token is None or stop_active_probe_processes(probe_token)
            )
        except CleanupError:
            # Session cleanup still has to run when this kernel cannot provide
            # the only process-safe signalling primitive accepted here.
            active_probe_process_absent = False
        try:
            cleanup = cleanup_case(
                ui,
                docker,
                session_id,
                lease_directory,
                timeout=args.cleanup_timeout,
            )
        except Exception as exc:
            cleanup = {
                "viewerClosed": False,
                "sessionStopped": False,
                "workspaceReleased": False,
                "containerAbsent": False,
                "networkAbsent": False,
                "leaseRuntimeAbsent": False,
                "viewerPublisherAbsent": False,
                "complete": False,
                "error": redact_text(
                    str(exc), (credentials["email"], credentials["password"])
                ),
            }
        case_report["cleanup"] = cleanup
        case_report["cleanup"]["activeProbeProcessAbsent"] = (
            active_probe_process_absent
        )
        if not active_probe_process_absent:
            case_report["cleanup"]["complete"] = False
        probe_workspace_released = (
            probe_workspace is None
            or remove_active_probe_workspace(probe_workspace)
        )
        case_report["cleanup"]["probeWorkspaceReleased"] = (
            probe_workspace_released
        )
        if not probe_workspace_released:
            case_report["cleanup"]["complete"] = False
        case_report["finishedAt"] = utc_now()
        if case_report["hashes"]:
            case_report["hashes"]["evidenceBundleSha256"] = evidence_bundle_hash(
                case_report["hashes"]
            )
        if not cleanup.get("complete"):
            case_report["status"] = "cleanup-failed"
    return case_report, case_report["status"] == "passed"


def run_matrix(args: argparse.Namespace) -> int:
    visible = load_visible_driver()
    expected_origin = visible.validate_site(args.url)
    if not args.chrome.is_file():
        raise MatrixError("The visible Chrome executable was not found")
    if not args.profile_root.is_dir():
        raise MatrixError("The Chrome profile root does not exist")
    validate_output_target(args.output)
    run_id = dt.datetime.now(dt.timezone.utc).strftime("%Y%m%dT%H%M%SZ") + f"-{os.getpid()}"
    evidence_dir = args.output.parent / f"{args.output.stem}-evidence-{run_id}"
    validate_secure_directory(evidence_dir)
    credentials_by_account = visible.read_credentials(args.credentials)
    credentials = credentials_by_account["A"]
    secrets = (credentials["email"], credentials["password"])

    report: dict[str, Any] = {
        "schemaVersion": 1,
        "runId": run_id,
        "startedAt": utc_now(),
        "site": clean_site_url(args.url),
        "deploymentCommit": args.deployment_commit,
        "account": "A",
        "rendering": {
            "visibleChrome": True,
            "headless": False,
            "gpuDisabled": False,
            "viewerStartupScene": {
                "phase": "beforeScenario",
                "minimumGazeboFps": MINIMUM_STARTUP_SCENE_FPS,
                "capabilityOnly": True,
                "activeScenarioMotionMeasured": False,
            },
            "activeScenarioGuiProbe": {
                "phase": "duringScenarioProcess",
                "officialPreflight": True,
                "warmupSeconds": ACTIVE_PROBE_WARMUP_SECONDS,
                "sampleSeconds": ACTIVE_PROBE_SAMPLE_SECONDS,
                "minimumGazeboFps": MINIMUM_STARTUP_SCENE_FPS,
                "minimumRealTimeFactor": MINIMUM_REAL_TIME_FACTOR,
                "requiresCompleteMonotonicOverlap": True,
            },
            "activeScenarioVideo": {
                "phase": "duringScenarioAndGuiProbe",
                "measurementSeconds": ACTIVE_SCENARIO_VIDEO_SECONDS,
                "targetBrowserFps": TARGET_BROWSER_VIDEO_FPS,
                "minimumBrowserFps": MINIMUM_BROWSER_VIDEO_FPS,
                "maximumDroppedRatio": MAXIMUM_BROWSER_DROPPED_RATIO,
                "visualMotionMeasured": False,
            },
            "scenarioPhysics": {
                "source": "RESULT_JSON",
                "minimumRealTimeFactor": MINIMUM_REAL_TIME_FACTOR,
            },
        },
        "selectedScenarios": [item.name for item in args.selected_scenarios],
        "evidenceDirectory": evidence_dir.name,
        "cases": [],
        "cleanup": {"browserClosed": False, "profileRemoved": False},
        "success": False,
    }
    write_json_secure(args.output, report, secrets)

    stop_event = threading.Event()
    interrupted = False

    def note_signal(_number: int, _frame: Any) -> None:
        nonlocal interrupted
        interrupted = True
        stop_event.set()

    previous_int = signal.signal(signal.SIGINT, note_signal)
    previous_term = signal.signal(signal.SIGTERM, note_signal)

    profile = args.profile_root / f"robotswarm-ros-matrix-{run_id}"
    chrome = visible.OwnedChrome(
        "A", args.cdp_port, profile, run_id, args.chrome, args.url
    )
    ui: Any = None
    docker = DockerHost(args.docker, stop_event)
    cleanup_failed = False
    failure: str | None = None
    try:
        if not visible.port_is_free(args.cdp_port):
            raise MatrixError("The requested Chrome debugging port is occupied")
        chrome.launch()
        ui = visible.RobotSwarmUi(chrome, expected_origin, stop_event)
        ui.navigate(args.url)
        ui.login(credentials["email"], credentials["password"])
        report["browser"] = {"visible": True, "product": chrome.product}
        write_json_secure(args.output, report, secrets)

        for index, scenario in enumerate(args.selected_scenarios, start=1):
            if stop_event.is_set():
                raise KeyboardInterrupt
            position = f"[{index}/{len(args.selected_scenarios)}]"
            print(
                f"{position} Running {scenario.name} in a fresh visible session...",
                flush=True,
            )
            case_report, _passed = run_one_case(
                index=index,
                scenario=scenario,
                args=args,
                ui=ui,
                docker=docker,
                evidence_dir=evidence_dir,
                credentials=credentials,
                stop_event=stop_event,
            )
            report["cases"].append(case_report)
            try:
                require_case_cleanup_before_next(case_report)
                if case_report.get("status") == "interrupted" or interrupted:
                    raise KeyboardInterrupt
            except CleanupError:
                cleanup_failed = True
                raise
            finally:
                write_json_secure(args.output, report, secrets)
    except KeyboardInterrupt:
        interrupted = True
        failure = "Interrupted by the operator"
    except Exception as exc:
        failure = redact_text(str(exc), secrets)
        cleanup_failed = cleanup_failed or isinstance(exc, CleanupError)
    finally:
        if ui is not None and (ui.created_session or ui.create_requested):
            # A case normally performs this cleanup.  This covers failures
            # before run_one_case acquired the private session identifier.
            original_event = ui.stop_event
            ui.stop_event = threading.Event()
            try:
                fallback = ui.stop_created_session(timeout=args.cleanup_timeout)
                report["cleanup"]["fallbackSessionReleased"] = bool(
                    fallback.get("released")
                )
                cleanup_failed = cleanup_failed or not bool(fallback.get("released"))
            except Exception as exc:
                report["cleanup"]["fallbackSessionError"] = redact_text(str(exc), secrets)
                cleanup_failed = True
            finally:
                ui.stop_event = original_event
        if chrome.process is None:
            report["cleanup"]["browserClosed"] = True
            report["cleanup"]["profileRemoved"] = not profile.exists()
        else:
            try:
                browser_cleanup = chrome.close_owned()
                report["cleanup"]["browserClosed"] = bool(
                    browser_cleanup.get("processExited")
                    and browser_cleanup.get("portFree")
                )
                report["cleanup"]["cdpPortReleased"] = bool(
                    browser_cleanup.get("portFree")
                )
                report["cleanup"]["profileRemoved"] = bool(
                    browser_cleanup.get("profileRemoved")
                )
            except Exception as exc:
                report["cleanup"]["browserError"] = redact_text(str(exc), secrets)
                report["cleanup"]["browserClosed"] = False
                report["cleanup"]["profileRemoved"] = False
        cleanup_failed = cleanup_failed or not all(
            report["cleanup"].get(key)
            for key in ("browserClosed", "profileRemoved")
        )
        report["completedAt"] = utc_now()
        report["interrupted"] = interrupted
        if failure:
            report["failure"] = failure
        expected_count = len(args.selected_scenarios)
        report["success"] = (
            not interrupted
            and not cleanup_failed
            and len(report["cases"]) == expected_count
            and all(item.get("status") == "passed" for item in report["cases"])
            and all(item.get("cleanup", {}).get("complete") for item in report["cases"])
        )
        write_json_secure(args.output, report, secrets)
        signal.signal(signal.SIGINT, previous_int)
        signal.signal(signal.SIGTERM, previous_term)

    safe_output = redact_text(str(args.output), secrets)
    print(f"Sanitized ROS matrix report: {safe_output}", flush=True)
    if interrupted:
        return 130
    if cleanup_failed:
        return 3
    return 0 if report["success"] else 1


def main(argv: Sequence[str] | None = None) -> int:
    args = parse_args(argv)
    try:
        with HostRunLock():
            return run_matrix(args)
    except Exception as exc:
        print(f"ROS matrix preflight failed: {redact_text(str(exc))}", file=sys.stderr)
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
