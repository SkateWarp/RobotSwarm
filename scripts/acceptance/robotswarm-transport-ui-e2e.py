#!/usr/bin/env python3
"""Visible production smoke for the CollaborativeTransport web control path.

The operator must opt in explicitly.  One normal Chrome window creates one
fresh four-robot session, opens its real HLS viewer, and starts transport with
one physical click in the public frontend.  The report deliberately contains
no account, session, task, worker, container, network, or lease identifiers.
"""

from __future__ import annotations

import argparse
import contextlib
import dataclasses
import datetime as dt
import fcntl
import importlib.util
import ipaddress
import json
import math
import os
import re
import selectors
import signal
import shutil
import stat
import subprocess
import sys
import threading
import time
import uuid
from pathlib import Path
from types import ModuleType
from typing import Any, Iterable, Sequence


HERE = Path(__file__).resolve().parent
VISIBLE_DRIVER = HERE / "robotswarm-visible-e2e.py"
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

SCHEMA_VERSION = 1
ROBOT_COUNT = 4
TARGET_X = -3.0
TARGET_Y = -4.0
PLANNER = "grf"
TASK_TYPE = "CollaborativeTransport"
START_BUTTON = "Iniciar tarea"
MINIMUM_VIDEO_FPS = 27.0
MAXIMUM_DROPPED_RATIO = 0.10
MAXIMUM_CHILD_OUTPUT = 1024 * 1024
MAXIMUM_TASK_DOCUMENT = 512 * 1024
READ_CHUNK = 64 * 1024
MAXIMUM_OBSERVER_LINE = 16 * 1024
MAXIMUM_OBSERVER_EVIDENCE = 8 * 1024 * 1024
MAXIMUM_OBSERVER_DOCUMENTS = 6000
OBSERVER_SAMPLE_INTERVAL_SECONDS = 0.25
OBSERVER_MARKER_GRACE_SECONDS = 2.0
OBSERVER_SIGNAL_GRACE_SECONDS = 2.0
OBSERVER_KILL_GRACE_SECONDS = 3.0
OBSERVER_READER_JOIN_SECONDS = 2.0
OBSERVER_REMOTE_COMMAND_SECONDS = 8.0
START_PROBE_QUIET_SECONDS = 0.75
MINIMUM_PUSH_SAMPLES = 3
MINIMUM_PUSH_WINDOW_SECONDS = 0.40
MINIMUM_PUSH_PROGRESS_GAIN = 0.001
MINIMUM_SEARCH_TRAVEL_METRES = 0.015
MINIMUM_TASK_FRAME_CHANGE_RATIO = 0.0001

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
EMAIL_PATTERN = re.compile(r"[A-Za-z0-9._%+-]+@[A-Za-z0-9.-]+\.[A-Za-z]{2,}")
JWT_PATTERN = re.compile(
    r"\beyJ[A-Za-z0-9_-]{8,}\.[A-Za-z0-9_-]{8,}\.[A-Za-z0-9_-]{8,}\b"
)
PRIVATE_IP_PATTERN = re.compile(
    r"\b(?:10(?:\.[0-9]{1,3}){3}|192\.168(?:\.[0-9]{1,3}){2}|"
    r"172\.(?:1[6-9]|2[0-9]|3[01])(?:\.[0-9]{1,3}){2})\b"
)
IMAGE_ID_PATTERN = re.compile(r"sha256:([0-9a-f]{64})")
SHA_PATTERN = re.compile(r"[0-9a-f]{40}")
ROBOT_PATTERN = re.compile(r"tb3_[0-9]+")
TERMINAL_STATES = {"Completed", "Failed", "Cancelled"}
PHASE_LABELS = {
    "SEARCH": "Búsqueda",
    "APPROACH": "Reagrupación",
    "PUSH": "Empuje coordinado",
    "DONE": "Entrega completada",
}


class TransportSmokeError(RuntimeError):
    """Expected, sanitized failure in the operator-run smoke."""


@dataclasses.dataclass(frozen=True)
class ProcessOutput:
    returncode: int
    stdout: str
    stderr: str


@dataclasses.dataclass(frozen=True)
class LocalProcessIdentity:
    pid: int
    process_group: int
    session: int
    start_ticks: int

    @property
    def owns_private_group(self) -> bool:
        return (
            self.pid > 1
            and self.pid == self.process_group
            and self.pid == self.session
            and self.start_ticks > 0
        )


@dataclasses.dataclass(frozen=True)
class ContainerHandle:
    identifier: str
    worker_identifier: str = ""
    started_at: dt.datetime | None = None


@dataclasses.dataclass(frozen=True)
class ViewerBinding:
    directory: Path
    lease_id: uuid.UUID


@dataclasses.dataclass(frozen=True)
class StartProbeEvidence:
    report: dict[str, Any]
    task_id: str
    command_id: str
    task_created_at: str
    command_created_at: str
    click_epoch_ms: float
    request_epoch_ms: float
    response_epoch_ms: float


@dataclasses.dataclass(frozen=True)
class PhaseCapture:
    report: list[dict[str, Any]]
    task_id: str
    average_hash: str
    file_path: Path | None = None


def load_visible_driver() -> ModuleType:
    module_name = "robotswarm_visible_transport_shared"
    if module_name in sys.modules:
        return sys.modules[module_name]
    specification = importlib.util.spec_from_file_location(module_name, VISIBLE_DRIVER)
    if specification is None or specification.loader is None:
        raise TransportSmokeError("The visible RobotSwarm driver could not be loaded")
    module = importlib.util.module_from_spec(specification)
    sys.modules[module_name] = module
    specification.loader.exec_module(module)
    return module


VISIBLE = load_visible_driver()


def utc_now() -> str:
    return dt.datetime.now(dt.timezone.utc).isoformat().replace("+00:00", "Z")


def remaining_budget(deadline: float, label: str) -> float:
    remaining = deadline - time.monotonic()
    if remaining <= 0:
        raise TransportSmokeError(f"The {label} time budget was exhausted")
    return remaining


def parse_utc_timestamp(value: Any, label: str) -> dt.datetime:
    if not isinstance(value, str) or not value:
        raise TransportSmokeError(f"The {label} timestamp is unavailable")
    match = re.fullmatch(
        r"(?P<base>\d{4}-\d{2}-\d{2}T\d{2}:\d{2}:\d{2})"
        r"(?:\.(?P<fraction>\d{1,9}))?(?P<zone>Z|[+-]\d{2}:\d{2})",
        value,
    )
    if match is None:
        raise TransportSmokeError(f"The {label} timestamp is invalid")
    fraction = (match.group("fraction") or "0")[:6].ljust(6, "0")
    zone = "+00:00" if match.group("zone") == "Z" else match.group("zone")
    normalized = f"{match.group('base')}.{fraction}{zone}"
    try:
        parsed = dt.datetime.fromisoformat(normalized)
    except ValueError as exc:
        raise TransportSmokeError(f"The {label} timestamp is invalid") from exc
    if parsed.tzinfo is None:
        raise TransportSmokeError(f"The {label} timestamp has no timezone")
    return parsed.astimezone(dt.timezone.utc)


def sanitize_text(value: str, secrets: Iterable[str] = ()) -> str:
    clean = value
    for secret in secrets:
        if secret:
            clean = clean.replace(secret, "[REDACTED]")
    clean = JWT_PATTERN.sub("[TOKEN REDACTED]", clean)
    clean = EMAIL_PATTERN.sub("[ACCOUNT REDACTED]", clean)
    clean = UUID_PATTERN.sub("[IDENTIFIER REDACTED]", clean)
    clean = PRIVATE_IP_PATTERN.sub("[PRIVATE ADDRESS REDACTED]", clean)
    clean = re.sub(
        r"(?i)\b(?:worker|trabajador)\s*[:#=-]?\s*[A-Za-z0-9._-]+",
        "[WORKER REDACTED]",
        clean,
    )
    return clean


def sanitize_value(value: Any, secrets: Iterable[str] = ()) -> Any:
    if isinstance(value, dict):
        return {str(key): sanitize_value(item, secrets) for key, item in value.items()}
    if isinstance(value, list):
        return [sanitize_value(item, secrets) for item in value]
    if isinstance(value, tuple):
        return [sanitize_value(item, secrets) for item in value]
    if isinstance(value, str):
        return sanitize_text(value, secrets)
    return value


def assert_report_safe(document: Any, secrets: Iterable[str] = ()) -> None:
    serialized = json.dumps(document, ensure_ascii=False, allow_nan=False)
    if UUID_PATTERN.search(serialized):
        raise TransportSmokeError("A private identifier reached the report")
    if EMAIL_PATTERN.search(serialized) or JWT_PATTERN.search(serialized):
        raise TransportSmokeError("Account data reached the report")
    if PRIVATE_IP_PATTERN.search(serialized):
        raise TransportSmokeError("A private network address reached the report")
    for secret in secrets:
        if secret and secret in serialized:
            raise TransportSmokeError("A credential reached the report")


def validate_output_target(path: Path) -> None:
    VISIBLE.validate_secure_directory(path.parent)
    if not (path.exists() or path.is_symlink()):
        return
    details = path.lstat()
    if (
        stat.S_ISLNK(details.st_mode)
        or not stat.S_ISREG(details.st_mode)
        or details.st_uid != os.getuid()
        or stat.S_IMODE(details.st_mode) != 0o600
    ):
        raise TransportSmokeError("The existing report is not a private owned file")


def write_report(path: Path, report: dict[str, Any], secrets: Iterable[str]) -> None:
    safe = sanitize_value(report, secrets)
    assert_report_safe(safe, secrets)
    VISIBLE.write_json_secure(path, safe)


class HostRunLock:
    """Share the account-A lock used by the ROS matrix and load smoke."""

    def __init__(self, path: Path = Path("/tmp/robotswarm-ros-matrix-e2e.lock")) -> None:
        self.path = path
        self.stream: Any = None

    def __enter__(self) -> "HostRunLock":
        flags = os.O_RDWR | os.O_CREAT | os.O_CLOEXEC
        if hasattr(os, "O_NOFOLLOW"):
            flags |= os.O_NOFOLLOW
        descriptor = os.open(self.path, flags, 0o600)
        details = os.fstat(descriptor)
        if (
            not stat.S_ISREG(details.st_mode)
            or details.st_uid != os.getuid()
            or stat.S_IMODE(details.st_mode) != 0o600
        ):
            os.close(descriptor)
            raise TransportSmokeError("The shared account-A lock is not private")
        self.stream = os.fdopen(descriptor, "a+b", buffering=0)
        try:
            fcntl.flock(self.stream.fileno(), fcntl.LOCK_EX | fcntl.LOCK_NB)
        except BlockingIOError as exc:
            self.stream.close()
            self.stream = None
            raise TransportSmokeError("Another acceptance run already owns account A") from exc
        return self

    def __exit__(self, _kind: Any, _value: Any, _traceback: Any) -> None:
        if self.stream is not None:
            fcntl.flock(self.stream.fileno(), fcntl.LOCK_UN)
            self.stream.close()
            self.stream = None


def run_bounded(
    arguments: Sequence[str],
    *,
    timeout: float,
    stop_event: threading.Event | None,
) -> ProcessOutput:
    """Drain both child streams while enforcing one combined hard limit."""
    process = subprocess.Popen(
        list(arguments),
        stdin=subprocess.DEVNULL,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        start_new_session=True,
    )
    if process.stdout is None or process.stderr is None:
        raise TransportSmokeError("A bounded child process has no output pipes")
    selector = selectors.DefaultSelector()
    streams = {
        "stdout": (process.stdout, bytearray()),
        "stderr": (process.stderr, bytearray()),
    }
    for name, (stream, _buffer) in streams.items():
        os.set_blocking(stream.fileno(), False)
        selector.register(stream, selectors.EVENT_READ, data=name)

    deadline = time.monotonic() + timeout
    retained = 0
    reason: str | None = None
    try:
        while process.poll() is None or selector.get_map():
            if reason is None and stop_event is not None and stop_event.is_set():
                reason = "interrupted"
            if reason is None and time.monotonic() >= deadline:
                reason = "timeout"
            if reason is not None and process.poll() is None:
                with contextlib.suppress(ProcessLookupError):
                    os.killpg(process.pid, signal.SIGKILL)

            events = selector.select(0.15)
            for key, _mask in events:
                stream, buffer = streams[str(key.data)]
                try:
                    chunk = os.read(stream.fileno(), READ_CHUNK)
                except BlockingIOError:
                    continue
                if not chunk:
                    with contextlib.suppress(Exception):
                        selector.unregister(stream)
                    continue
                if retained + len(chunk) > MAXIMUM_CHILD_OUTPUT:
                    reason = "overflow"
                    room = max(0, MAXIMUM_CHILD_OUTPUT - retained)
                    buffer.extend(chunk[:room])
                    retained += room
                    continue
                buffer.extend(chunk)
                retained += len(chunk)
            if process.poll() is not None and not events:
                for stream, _buffer in streams.values():
                    with contextlib.suppress(Exception):
                        selector.unregister(stream)
                break
    finally:
        if process.poll() is None:
            with contextlib.suppress(ProcessLookupError):
                os.killpg(process.pid, signal.SIGKILL)
        with contextlib.suppress(subprocess.TimeoutExpired):
            process.wait(timeout=5)
        selector.close()
        process.stdout.close()
        process.stderr.close()

    if reason == "interrupted":
        raise KeyboardInterrupt
    if reason == "timeout":
        raise TransportSmokeError("A bounded child process timed out")
    if reason == "overflow":
        raise TransportSmokeError("A child process exceeded its output limit")
    return ProcessOutput(
        int(process.returncode or 0),
        bytes(streams["stdout"][1]).decode("utf-8", errors="replace"),
        bytes(streams["stderr"][1]).decode("utf-8", errors="replace"),
    )


class DockerProof:
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
        return run_bounded(
            [self.executable, *arguments],
            timeout=timeout,
            stop_event=self.stop_event if interruptible else None,
        )

    @staticmethod
    def _lines(value: str) -> list[str]:
        return [line.strip() for line in value.splitlines() if line.strip()]

    def _resource_ids(
        self,
        kind: str,
        session_id: uuid.UUID,
        *,
        interruptible: bool,
    ) -> list[str]:
        if kind == "network":
            arguments = [
                "network",
                "ls",
                "--no-trunc",
                "--quiet",
                "--filter",
                f"label={MANAGED_LABEL}=true",
                "--filter",
                f"label={SESSION_LABEL}={session_id}",
            ]
        else:
            arguments = [
                "ps",
                "-a",
                "--no-trunc",
                "--quiet",
                "--filter",
                f"label={MANAGED_LABEL}=true",
                "--filter",
                f"label={SESSION_LABEL}={session_id}",
            ]
        result = self.run(
            arguments,
            interruptible=interruptible,
        )
        if result.returncode != 0:
            raise TransportSmokeError(f"Docker could not list the managed {kind}")
        identifiers = self._lines(result.stdout)
        if any(re.fullmatch(r"[0-9a-f]{64}", item) is None for item in identifiers):
            raise TransportSmokeError(f"Docker returned an invalid {kind} identifier")
        return identifiers

    def _inspect(self, kind: str, identifier: str) -> dict[str, Any]:
        prefix = [kind] if kind in {"network", "image"} else []
        result = self.run([*prefix, "inspect", identifier])
        if result.returncode != 0:
            raise TransportSmokeError(f"Docker could not inspect the managed {kind}")
        try:
            documents = json.loads(result.stdout)
        except (json.JSONDecodeError, ValueError) as exc:
            raise TransportSmokeError(f"Docker returned malformed {kind} metadata") from exc
        if not isinstance(documents, list) or len(documents) != 1 or not isinstance(documents[0], dict):
            raise TransportSmokeError(f"Docker returned ambiguous {kind} metadata")
        return documents[0]

    @staticmethod
    def _labels(document: dict[str, Any], *, network: bool = False) -> dict[str, str]:
        raw = document.get("Labels") if network else (document.get("Config") or {}).get("Labels")
        if not isinstance(raw, dict) or any(not isinstance(value, str) for value in raw.values()):
            raise TransportSmokeError("Docker resource labels are invalid")
        return {str(key): value for key, value in raw.items()}

    def verify_session(
        self,
        session_id: uuid.UUID,
        deployment_commit: str,
    ) -> tuple[ContainerHandle, dict[str, Any]]:
        container_ids = self._resource_ids("container", session_id, interruptible=True)
        if len(container_ids) != 1:
            raise TransportSmokeError("The session does not own exactly one managed container")
        identifier = container_ids[0]
        container = self._inspect("container", identifier)
        labels = self._labels(container)
        expected_name = f"/robotswarm-{session_id.hex}"
        try:
            worker_id = uuid.UUID(str(labels.get(WORKER_LABEL)))
        except (ValueError, AttributeError) as exc:
            raise TransportSmokeError("The managed container has no valid owner") from exc
        if (
            labels.get(MANAGED_LABEL) != "true"
            or labels.get(SESSION_LABEL) != str(session_id)
            or container.get("Name") != expected_name
            or (container.get("State") or {}).get("Running") is not True
        ):
            raise TransportSmokeError("The managed container does not match the live session")
        started_at = parse_utc_timestamp(
            (container.get("State") or {}).get("StartedAt"),
            "managed container start",
        )

        image_id = container.get("Image")
        image_match = IMAGE_ID_PATTERN.fullmatch(str(image_id or ""))
        if image_match is None or (container.get("Config") or {}).get("Image") != image_id:
            raise TransportSmokeError("The session image is not pinned by immutable ID")
        image = self._inspect("image", str(image_id))
        image_labels = self._labels(image)
        if image.get("Id") != image_id:
            raise TransportSmokeError("Docker resolved a different immutable image")
        if image_labels.get(IMAGE_REVISION_LABEL) != deployment_commit:
            raise TransportSmokeError("The ROS image revision differs from the requested SHA")
        expected_version = f"{deployment_commit}+{image_match.group(1)[:12]}"
        if labels.get(IMAGE_VERSION_LABEL) != expected_version:
            raise TransportSmokeError("The container image-version label is inconsistent")

        network_ids = self._resource_ids("network", session_id, interruptible=True)
        if len(network_ids) != 1:
            raise TransportSmokeError("The session does not own exactly one private network")
        network = self._inspect("network", network_ids[0])
        network_labels = self._labels(network, network=True)
        expected_network = f"robotswarm-{session_id.hex}-net"
        if (
            network_labels.get(MANAGED_LABEL) != "true"
            or network_labels.get(SESSION_LABEL) != str(session_id)
            or network_labels.get(WORKER_LABEL) != str(worker_id)
            or network.get("Name") != expected_network
            or network.get("Internal") is not True
        ):
            raise TransportSmokeError("The private network does not match the session")

        attached = (container.get("NetworkSettings") or {}).get("Networks")
        members = network.get("Containers")
        if not isinstance(attached, dict) or set(attached) != {expected_network}:
            raise TransportSmokeError("The container is not attached only to its private network")
        endpoint = attached[expected_network]
        membership = members.get(identifier) if isinstance(members, dict) else None
        try:
            address = ipaddress.ip_address(str(endpoint.get("IPAddress")))
        except (ValueError, AttributeError) as exc:
            raise TransportSmokeError("The private network endpoint is invalid") from exc
        if (
            address.version != 4
            or not address.is_private
            or endpoint.get("NetworkID") != network_ids[0]
            or not isinstance(members, dict)
            or set(members) != {identifier}
            or not isinstance(membership, dict)
            or membership.get("Name") != expected_name[1:]
        ):
            raise TransportSmokeError("Exclusive private-network membership was not proven")

        return ContainerHandle(identifier, str(worker_id), started_at), {
            "managedContainer": True,
            "running": True,
            "immutableImage": True,
            "imageRevisionMatches": True,
            "workerLabelMatchesPrivateNetwork": True,
            "internalNetwork": True,
            "exclusiveNetworkAttachment": True,
            "privateEndpoint": True,
        }

    def resources_absent(self, session_id: uuid.UUID) -> tuple[bool, bool]:
        containers = self._resource_ids("container", session_id, interruptible=False)
        networks = self._resource_ids("network", session_id, interruptible=False)
        return not containers, not networks


TRANSPORT_OBSERVER_SOURCE = r'''\
import json
import math
import os
import re
import sys
import threading
import time

import rospy
from gazebo_msgs.msg import ModelStates
from std_msgs.msg import String

roster = tuple(item for item in sys.argv[1].split(',') if item)
stop_path = sys.argv[2]
maximum_seconds = float(sys.argv[3])
sample_interval_seconds = float(sys.argv[4])
if (
    not roster
    or len(set(roster)) != len(roster)
    or not all(re.fullmatch(r'tb3_[0-9]+', item) for item in roster)
    or not re.fullmatch(r'/tmp/robotswarm-transport-ui-[0-9a-f]{32}[.]stop', stop_path)
    or not math.isfinite(maximum_seconds)
    or maximum_seconds <= 0
    or not math.isfinite(sample_interval_seconds)
    or sample_interval_seconds <= 0
):
    raise SystemExit(4)

lock = threading.Lock()
active_task = None
active_phase = None
last_positions = {}
search_origins = {}
search_last = {}
search_paths = {item: 0.0 for item in roster}
last_emitted_at = 0.0
last_emitted_phase = None

def decode(message):
    try:
        value = json.loads(message.data)
    except (TypeError, ValueError):
        return None
    return value if isinstance(value, dict) else None

def model_callback(message):
    positions = {}
    for index, name in enumerate(message.name):
        if name in roster and index < len(message.pose):
            pose = message.pose[index].position
            positions[name] = (float(pose.x), float(pose.y))
    with lock:
        last_positions.update(positions)
        if active_phase != 'SEARCH' or not active_task:
            return
        for name, point in positions.items():
            if name not in search_origins:
                search_origins[name] = point
                search_last[name] = point
                continue
            previous = search_last.get(name, point)
            search_paths[name] += math.hypot(
                point[0] - previous[0], point[1] - previous[1]
            )
            search_last[name] = point

def status_callback(message):
    global active_task, active_phase, last_emitted_at, last_emitted_phase
    value = decode(message)
    if value is None:
        return
    task_id = str(value.get('task_id') or '')
    phase = str(value.get('phase') or '').upper()
    if not task_id or phase not in {'SEARCH', 'APPROACH', 'PUSH', 'DONE', 'FAILED'}:
        return
    discovery = value.get('discovery')
    selected_discovery = None
    if isinstance(discovery, dict):
        selected_discovery = {
            'event': discovery.get('event'),
            'task_id': discovery.get('task_id'),
            'announced': discovery.get('announced'),
            'finder': discovery.get('finder'),
            'notified_robots': discovery.get('notified_robots'),
        }
    observed_at = time.monotonic()
    with lock:
        active_task = task_id
        active_phase = phase
        if (
            phase not in {'DONE', 'FAILED'}
            and phase == last_emitted_phase
            and observed_at - last_emitted_at < sample_interval_seconds
        ):
            return
        last_emitted_at = observed_at
        last_emitted_phase = phase
        paths = dict(search_paths)
    document = {
        'observed_at': observed_at,
        'task_id': task_id,
        'phase': phase,
        'paused': value.get('paused'),
        'progress': value.get('progress'),
        'searching_robot_count': value.get('searching_robot_count'),
        'search_path_length_m': paths,
        'current_useful_pusher_count': value.get('current_useful_pusher_count'),
        'current_useful_pusher_ids': value.get('current_useful_pusher_ids'),
        'discovery': selected_discovery,
    }
    print(
        'TRANSPORT_OBSERVER_JSON '
        + json.dumps(document, sort_keys=True, separators=(',', ':')),
        flush=True,
    )

rospy.init_node(
    'robotswarm_transport_ui_observer', anonymous=True, disable_signals=True
)
rospy.Subscriber('/gazebo/model_states', ModelStates, model_callback, queue_size=1)
rospy.Subscriber('/transport/status', String, status_callback, queue_size=20)
print('TRANSPORT_OBSERVER_READY', flush=True)
deadline = time.monotonic() + maximum_seconds
try:
    while (
        not rospy.is_shutdown()
        and time.monotonic() < deadline
        and not os.path.exists(stop_path)
    ):
        time.sleep(0.05)
finally:
    try:
        os.unlink(stop_path)
    except FileNotFoundError:
        pass
'''

REMOTE_OBSERVER_STOP_SOURCE = r'''\
import glob
import os
import re
import stat
import time

stop_path = os.environ.get('ROBOTSWARM_OBSERVER_STOP_PATH', '')
if not re.fullmatch(
    r'/tmp/robotswarm-transport-ui-[0-9a-f]{32}[.]stop', stop_path
):
    raise SystemExit(4)

directory = '/tmp'
filename = stop_path.rsplit('/', 1)[-1]
directory_flags = os.O_RDONLY | os.O_CLOEXEC
if hasattr(os, 'O_DIRECTORY'):
    directory_flags |= os.O_DIRECTORY
try:
    directory_fd = os.open(directory, directory_flags)
except OSError:
    raise SystemExit(5)

flags = os.O_WRONLY | os.O_CREAT | os.O_CLOEXEC
if hasattr(os, 'O_NOFOLLOW'):
    flags |= os.O_NOFOLLOW
try:
    marker_fd = os.open(filename, flags, 0o600, dir_fd=directory_fd)
    details = os.fstat(marker_fd)
except OSError:
    os.close(directory_fd)
    raise SystemExit(5)
if (
    not stat.S_ISREG(details.st_mode)
    or details.st_uid != os.geteuid()
    or stat.S_IMODE(details.st_mode) != 0o600
    or details.st_nlink != 1
):
    os.close(marker_fd)
    os.close(directory_fd)
    raise SystemExit(5)

needle = stop_path.encode('utf-8')

def observer_count():
    count = 0
    for candidate in glob.glob('/proc/[0-9]*/cmdline'):
        try:
            with open(candidate, 'rb', buffering=0) as stream:
                raw = stream.read(65537)
        except (OSError, IOError):
            continue
        if len(raw) <= 65536 and needle in raw.split(b'\0'):
            count += 1
    return count

def remove_own_marker():
    try:
        current = os.stat(filename, dir_fd=directory_fd, follow_symlinks=False)
    except FileNotFoundError:
        return True
    except OSError:
        return False
    held = os.fstat(marker_fd)
    if (current.st_dev, current.st_ino) != (held.st_dev, held.st_ino):
        return False
    try:
        os.unlink(filename, dir_fd=directory_fd)
    except FileNotFoundError:
        return True
    except OSError:
        return False
    return True

deadline = time.monotonic() + 5.0
result = 'TRANSPORT_OBSERVER_ACTIVE'
exit_code = 6
while time.monotonic() < deadline:
    if observer_count() == 0:
        if remove_own_marker():
            result = 'TRANSPORT_OBSERVER_STOPPED'
            exit_code = 0
        else:
            result = 'TRANSPORT_OBSERVER_UNSAFE_MARKER'
            exit_code = 7
        break
    time.sleep(0.05)
os.close(marker_fd)
os.close(directory_fd)
print(result)
raise SystemExit(exit_code)
'''


class TransportStatusObserver:
    """Keep private ROS evidence that the reduced TaskRun result cannot expose."""

    def __init__(
        self,
        docker: DockerProof,
        container: ContainerHandle,
        roster: set[str],
        maximum_seconds: float,
    ) -> None:
        self.docker = docker
        self.container = container
        self.roster = set(roster)
        self.maximum_seconds = maximum_seconds
        self.token = uuid.uuid4().hex
        self.stop_path = f"/tmp/robotswarm-transport-ui-{self.token}.stop"
        self.process: subprocess.Popen[bytes] | None = None
        self.process_identity: LocalProcessIdentity | None = None
        self.local_group_terminated = False
        self.remote_observer_terminated = False
        self.documents: list[dict[str, Any]] = []
        self.document_bytes = 0
        self.stderr = bytearray()
        self.error: str | None = None
        self.ready = threading.Event()
        self.ready_seen = False
        self.lock = threading.Lock()
        self.threads: list[threading.Thread] = []

    def launch(self, timeout: float = 20) -> None:
        if self.process is not None:
            raise TransportSmokeError("The private ROS observer was already started")
        bootstrap = (
            "set -u; source /opt/ros/noetic/setup.bash; "
            "source /catkin_ws/devel/setup.bash; "
            "test ! -e \"$3\"; exec python3 -u -c \"$1\" \"$2\" \"$3\" \"$4\" \"$5\""
        )
        command = [
            self.docker.executable,
            "exec",
            self.container.identifier,
            "/bin/bash",
            "-lc",
            bootstrap,
            "robotswarm-transport-ui-observer",
            TRANSPORT_OBSERVER_SOURCE,
            ",".join(sorted(self.roster)),
            self.stop_path,
            f"{self.maximum_seconds:.3f}",
            f"{OBSERVER_SAMPLE_INTERVAL_SECONDS:.3f}",
        ]
        self.process = subprocess.Popen(
            command,
            stdin=subprocess.DEVNULL,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            start_new_session=True,
        )
        self.process_identity = self._read_process_identity(self.process.pid)
        if (
            self.process_identity is None
            or not self.process_identity.owns_private_group
        ):
            self.stop()
            raise TransportSmokeError(
                "The private ROS observer did not receive a private local process group"
            )
        if self.process.stdout is None or self.process.stderr is None:
            raise TransportSmokeError("The private ROS observer has no output pipes")
        self.threads = [
            threading.Thread(target=self._read_stdout, daemon=True),
            threading.Thread(target=self._read_stderr, daemon=True),
        ]
        for thread in self.threads:
            thread.start()
        if not self.ready.wait(timeout) or not self.ready_seen:
            self.stop()
            raise TransportSmokeError("The private ROS transport observer did not become ready")

    def _fail(self, message: str) -> None:
        with self.lock:
            if self.error is None:
                self.error = message

    @staticmethod
    def _bounded_lines(stream: Any) -> Iterable[bytes | None]:
        """Yield complete lines without ever accumulating an unbounded prefix."""
        pending = bytearray()
        dropping = False
        descriptor = stream.fileno()
        while True:
            try:
                chunk = os.read(descriptor, READ_CHUNK)
            except OSError:
                if stream.closed:
                    return
                raise
            if not chunk:
                break
            offset = 0
            while offset < len(chunk):
                newline = chunk.find(b"\n", offset)
                end = len(chunk) if newline < 0 else newline
                if dropping:
                    if newline < 0:
                        break
                    dropping = False
                    offset = newline + 1
                    continue

                piece = chunk[offset:end]
                if len(piece) > MAXIMUM_OBSERVER_LINE - len(pending):
                    pending.clear()
                    yield None
                    if newline < 0:
                        dropping = True
                        break
                    offset = newline + 1
                    continue
                pending.extend(piece)
                if newline < 0:
                    break
                yield bytes(pending)
                pending.clear()
                offset = newline + 1
        if pending and not dropping:
            yield bytes(pending)

    def _handle_stdout_line(self, raw: bytes) -> None:
        line = raw.decode("utf-8", errors="replace").strip()
        if line == "TRANSPORT_OBSERVER_READY":
            self.ready_seen = True
            self.ready.set()
            return
        prefix = "TRANSPORT_OBSERVER_JSON "
        if not line.startswith(prefix):
            return
        try:
            document = json.loads(line[len(prefix) :])
        except json.JSONDecodeError:
            self._fail("The private ROS observer returned malformed evidence")
            return
        retained_size = self._retained_size(document)
        with self.lock:
            if self.error is not None:
                return
            if not isinstance(document, dict):
                self.error = "The private ROS observer returned invalid evidence"
            elif (
                len(self.documents) >= MAXIMUM_OBSERVER_DOCUMENTS
                or self.document_bytes + retained_size > MAXIMUM_OBSERVER_EVIDENCE
            ):
                self.error = "The private ROS observer exceeded its evidence bound"
            else:
                self.documents.append(document)
                self.document_bytes += retained_size

    @staticmethod
    def _retained_size(value: Any) -> int:
        """Estimate the complete Python object graph retained for one JSON line."""
        total = 0
        pending = [value]
        seen: set[int] = set()
        while pending:
            item = pending.pop()
            identity = id(item)
            if identity in seen:
                continue
            seen.add(identity)
            total += sys.getsizeof(item)
            if isinstance(item, dict):
                pending.extend(item.keys())
                pending.extend(item.values())
            elif isinstance(item, (list, tuple)):
                pending.extend(item)
        return total

    def _read_stdout(self) -> None:
        assert self.process is not None and self.process.stdout is not None
        try:
            for raw in self._bounded_lines(self.process.stdout):
                if raw is None:
                    self._fail("The private ROS observer returned an oversized line")
                    continue
                self._handle_stdout_line(raw)
        except (OSError, ValueError):
            if not self.process.stdout.closed:
                self._fail("The private ROS observer output reader failed")
        finally:
            self.ready.set()

    def _read_stderr(self) -> None:
        assert self.process is not None and self.process.stderr is not None
        try:
            descriptor = self.process.stderr.fileno()
            while True:
                try:
                    chunk = os.read(descriptor, READ_CHUNK)
                except OSError:
                    if self.process.stderr.closed:
                        return
                    raise
                if not chunk:
                    return
                with self.lock:
                    room = max(0, MAXIMUM_CHILD_OUTPUT - len(self.stderr))
                    self.stderr.extend(chunk[:room])
                    if len(chunk) > room and self.error is None:
                        self.error = "The private ROS observer exceeded its diagnostic bound"
        except (OSError, ValueError):
            if not self.process.stderr.closed:
                self._fail("The private ROS observer diagnostic reader failed")

    def snapshot(self) -> list[dict[str, Any]]:
        with self.lock:
            if self.error:
                raise TransportSmokeError(self.error)
            return [dict(item) for item in self.documents]

    def wait_terminal(self, task_id: str, timeout: float) -> list[dict[str, Any]]:
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            documents = self.snapshot()
            if any(
                item.get("task_id") == task_id
                and item.get("phase") in {"DONE", "FAILED"}
                for item in documents
            ):
                return documents
            if self.process is not None and self.process.poll() is not None:
                break
            time.sleep(0.1)
        raise TransportSmokeError("The private ROS observer missed the terminal transport phase")

    @staticmethod
    def _read_process_identity(pid: int) -> LocalProcessIdentity | None:
        if isinstance(pid, bool) or not isinstance(pid, int) or pid <= 1:
            return None
        try:
            with open(f"/proc/{pid}/stat", "r", encoding="utf-8") as stream:
                raw = stream.read(8193)
        except (OSError, ValueError):
            return None
        if len(raw) > 8192 or ")" not in raw:
            return None
        head, tail = raw.rsplit(")", 1)
        fields = tail.split()
        try:
            recorded_pid = int(head.split(" ", 1)[0])
            process_group = int(fields[2])
            session = int(fields[3])
            start_ticks = int(fields[19])
        except (IndexError, TypeError, ValueError):
            return None
        if recorded_pid != pid:
            return None
        return LocalProcessIdentity(pid, process_group, session, start_ticks)

    @classmethod
    def _identity_matches(cls, expected: LocalProcessIdentity) -> bool:
        return cls._read_process_identity(expected.pid) == expected

    @classmethod
    def _private_group_absent(cls, expected: LocalProcessIdentity) -> bool:
        for candidate in Path("/proc").glob("[0-9]*"):
            try:
                pid = int(candidate.name)
            except ValueError:
                continue
            current = cls._read_process_identity(pid)
            if (
                current is not None
                and current.process_group == expected.process_group
                and current.session == expected.session
            ):
                return False
        try:
            os.killpg(expected.process_group, 0)
        except ProcessLookupError:
            return True
        except (PermissionError, OSError):
            return False
        return False

    @classmethod
    def _wait_for_process_group(
        cls,
        process: subprocess.Popen[bytes],
        identity: LocalProcessIdentity,
        timeout: float,
    ) -> bool:
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            process.poll()
            if cls._private_group_absent(identity):
                return True
            time.sleep(0.05)
        process.poll()
        return cls._private_group_absent(identity)

    def _prove_remote_observer_absent(self) -> bool:
        try:
            result = self.docker.run(
                [
                    "exec",
                    "--env",
                    f"ROBOTSWARM_OBSERVER_STOP_PATH={self.stop_path}",
                    self.container.identifier,
                    "python3",
                    "-c",
                    REMOTE_OBSERVER_STOP_SOURCE,
                ],
                timeout=OBSERVER_REMOTE_COMMAND_SECONDS,
                interruptible=False,
            )
        except Exception:
            return False
        return (
            result.returncode == 0
            and result.stdout.strip() == "TRANSPORT_OBSERVER_STOPPED"
        )

    def accept_container_absence_proof(self) -> None:
        """A removed container is conclusive evidence that its observer is gone."""
        self.remote_observer_terminated = True

    def stop(self) -> bool:
        process = self.process
        if process is None:
            return True

        if not self.remote_observer_terminated:
            self.remote_observer_terminated = self._prove_remote_observer_absent()

        identity = self.process_identity
        group_gone = self.local_group_terminated
        if identity is not None and identity.owns_private_group and not group_gone:
            group_gone = self._wait_for_process_group(
                process,
                identity,
                OBSERVER_MARKER_GRACE_SECONDS,
            )
            for number, grace in (
                (signal.SIGINT, OBSERVER_SIGNAL_GRACE_SECONDS),
                (signal.SIGTERM, OBSERVER_SIGNAL_GRACE_SECONDS),
                (signal.SIGKILL, OBSERVER_KILL_GRACE_SECONDS),
            ):
                if group_gone or not self._identity_matches(identity):
                    break
                try:
                    os.killpg(identity.process_group, number)
                except ProcessLookupError:
                    pass
                except (PermissionError, OSError):
                    break
                group_gone = self._wait_for_process_group(process, identity, grace)
        if group_gone:
            self.local_group_terminated = True

        if not self.remote_observer_terminated:
            self.remote_observer_terminated = self._prove_remote_observer_absent()

        process.poll()
        for stream in (process.stdout, process.stderr):
            if stream is not None and not stream.closed:
                with contextlib.suppress(OSError, ValueError):
                    stream.close()
        for thread in self.threads:
            if thread.ident is not None:
                thread.join(timeout=OBSERVER_READER_JOIN_SECONDS)
        readers_stopped = all(
            thread.ident is None or not thread.is_alive() for thread in self.threads
        )
        streams_closed = all(
            stream is None or stream.closed for stream in (process.stdout, process.stderr)
        )
        return (
            process.poll() is not None
            and group_gone
            and self.remote_observer_terminated
            and readers_stopped
            and streams_closed
        )


def finish_observer_cleanup(
    observer: TransportStatusObserver | None,
    already_exited: bool,
    cleanup: dict[str, Any],
) -> bool:
    if observer is None or already_exited:
        return True
    if cleanup.get("containerAbsent") is True:
        observer.accept_container_absence_proof()
    return observer.stop()


def _compressed(values: Iterable[str]) -> list[str]:
    result: list[str] = []
    for value in values:
        if not result or result[-1] != value:
            result.append(value)
    return result


def _require_monotonic_history(
    values: list[str],
    order: Sequence[str],
    label: str,
) -> list[str]:
    compact = _compressed(values)
    ranks = {value: index for index, value in enumerate(order)}
    if any(value not in ranks for value in compact):
        raise TransportSmokeError(f"The {label} history contains an unexpected value")
    if any(ranks[right] <= ranks[left] for left, right in zip(compact, compact[1:])):
        raise TransportSmokeError(f"The {label} history regressed")
    return compact


def validate_ros_transport_observation(
    documents: list[dict[str, Any]],
    task_id: str,
    roster: set[str],
) -> dict[str, Any]:
    if not documents:
        raise TransportSmokeError("The private ROS observer collected no transport evidence")
    foreign = {
        str(item.get("task_id") or "")
        for item in documents
        if item.get("task_id") and item.get("task_id") != task_id
    }
    if foreign:
        raise TransportSmokeError("The fresh container exposed transport evidence for another task")
    matched = [item for item in documents if item.get("task_id") == task_id]
    if not matched:
        raise TransportSmokeError("The private ROS observer did not match the one-click task")
    timestamps = [item.get("observed_at") for item in matched]
    if any(
        isinstance(value, bool)
        or not isinstance(value, (int, float))
        or not math.isfinite(float(value))
        for value in timestamps
    ) or any(float(right) < float(left) for left, right in zip(timestamps, timestamps[1:])):
        raise TransportSmokeError("The private ROS observation timeline is invalid")

    phases = _require_monotonic_history(
        [str(item.get("phase") or "") for item in matched],
        ("SEARCH", "APPROACH", "PUSH", "DONE"),
        "ROS transport phase",
    )
    if phases != ["SEARCH", "APPROACH", "PUSH", "DONE"]:
        raise TransportSmokeError(
            "The ROS transport automaton missed SEARCH, discovery/APPROACH, PUSH, or DONE"
        )

    search = [item for item in matched if item.get("phase") == "SEARCH"]
    if not search or any(item.get("searching_robot_count") != len(roster) for item in search):
        raise TransportSmokeError("The ROS SEARCH phase did not keep the authoritative fleet active")
    travel: dict[str, float] = {name: 0.0 for name in roster}
    for item in matched:
        raw_paths = item.get("search_path_length_m")
        if not isinstance(raw_paths, dict):
            continue
        for name in roster:
            value = raw_paths.get(name)
            if (
                not isinstance(value, bool)
                and isinstance(value, (int, float))
                and math.isfinite(float(value))
            ):
                travel[name] = max(travel[name], float(value))
    moving = {name for name, distance in travel.items() if distance >= MINIMUM_SEARCH_TRAVEL_METRES}
    if moving != roster:
        raise TransportSmokeError(
            "The ROS SEARCH phase did not prove physical movement for every robot"
        )

    approach = [item for item in matched if item.get("phase") == "APPROACH"]
    discovery_proven = False
    for item in approach:
        discovery = item.get("discovery")
        if not isinstance(discovery, dict):
            continue
        finder = discovery.get("finder")
        notified = discovery.get("notified_robots")
        if (
            discovery.get("event") == "payload_found"
            and discovery.get("announced") is True
            and discovery.get("task_id") == task_id
            and finder in roster
            and isinstance(notified, list)
            and len(notified) == len(roster) - 1
            and len(set(notified)) == len(notified)
            and set(notified) == roster - {finder}
        ):
            discovery_proven = True
            break
    if not discovery_proven:
        raise TransportSmokeError("APPROACH did not preserve the correlated fleet-wide notice")

    best: list[dict[str, Any]] = []
    current: list[dict[str, Any]] = []
    for item in matched:
        ids = item.get("current_useful_pusher_ids")
        exact_push = (
            item.get("phase") == "PUSH"
            and item.get("paused") is not True
            and item.get("current_useful_pusher_count") == len(roster)
            and isinstance(ids, list)
            and len(ids) == len(roster)
            and len(set(ids)) == len(ids)
            and set(ids) == roster
            and not isinstance(item.get("progress"), bool)
            and isinstance(item.get("progress"), (int, float))
            and math.isfinite(float(item["progress"]))
        )
        if exact_push:
            current.append(item)
            if len(current) > len(best):
                best = list(current)
        elif item.get("phase") == "PUSH":
            current = []
    duration = (
        float(best[-1]["observed_at"]) - float(best[0]["observed_at"])
        if len(best) >= 2
        else 0.0
    )
    progress_gain = (
        max(float(item["progress"]) for item in best)
        - min(float(item["progress"]) for item in best)
        if best
        else 0.0
    )
    if (
        len(best) < MINIMUM_PUSH_SAMPLES
        or duration < MINIMUM_PUSH_WINDOW_SECONDS
        or progress_gain < MINIMUM_PUSH_PROGRESS_GAIN
    ):
        raise TransportSmokeError(
            "PUSH did not sustain the exact current pusher roster with forward progress"
        )

    search_report: dict[str, Any] = {
        "allRobotsSearching": True,
        "movementDirectlyObserved": True,
        "robotsAboveMinimumTravel": len(moving),
        "minimumTravelMetres": MINIMUM_SEARCH_TRAVEL_METRES,
    }
    return {
        "samePrivateTask": True,
        "sameManagedWorkerContainer": True,
        "strictAutomaton": phases,
        "discoveryDuringApproach": True,
        "search": search_report,
        "push": {
            "exactCurrentPusherRoster": True,
            "sustainedSamples": len(best),
            "sustainedSeconds": round(duration, 3),
            "progressGain": round(progress_gain, 4),
        },
        "criticalEvidenceRetainedUntilDone": True,
    }


def _argument_value(arguments: list[str], name: str) -> str | None:
    try:
        index = arguments.index(name)
    except ValueError:
        return None
    return arguments[index + 1] if index + 1 < len(arguments) else None


def publisher_for_session_exists(session_id: uuid.UUID) -> bool:
    expected = str(session_id)
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
            if item
        ]
        if (
            any(Path(item).name == "robotswarm-viewer-publisher" for item in arguments)
            and _argument_value(arguments, "--session-id") == expected
        ):
            return True
    return False


def active_viewer_runtime(
    runtime_root: Path,
    session_id: uuid.UUID,
    *,
    timeout: float,
    stop_event: threading.Event,
) -> ViewerBinding:
    """Bind a private lease directory to this session's live publisher."""
    deadline = time.monotonic() + timeout
    expected = str(session_id)
    while time.monotonic() < deadline:
        if stop_event.is_set():
            raise KeyboardInterrupt
        matches: dict[Path, uuid.UUID] = {}
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
                if item
            ]
            if not any(
                Path(item).name == "robotswarm-viewer-publisher" for item in arguments
            ):
                continue
            if _argument_value(arguments, "--session-id") != expected:
                continue
            try:
                lease = uuid.UUID(str(_argument_value(arguments, "--lease-id")))
            except (ValueError, AttributeError) as exc:
                raise TransportSmokeError("The active viewer has an invalid lease") from exc
            candidate = runtime_root / f"lease-{lease.hex}"
            if candidate.is_dir():
                matches[candidate] = lease
        if len(matches) > 1:
            raise TransportSmokeError("More than one viewer runtime matches the session")
        if len(matches) == 1:
            directory, lease_id = next(iter(matches.items()))
            details = directory.lstat()
            if (
                not stat.S_ISDIR(details.st_mode)
                or details.st_uid != os.getuid()
                or stat.S_IMODE(details.st_mode) != 0o700
            ):
                raise TransportSmokeError("The viewer runtime is not private")
            return ViewerBinding(directory, lease_id)
        time.sleep(0.2)
    raise TransportSmokeError("The HLS publisher could not be bound to the session")


def viewer_lease_status(
    ui: Any,
    session_id: uuid.UUID,
    lease_id: uuid.UUID,
) -> dict[str, Any]:
    endpoint = (
        f"https://{VISIBLE.DEFAULT_API_HOST}/api/sessions/{session_id}"
        f"/viewer-lease/{lease_id}"
    )
    response = ui.cdp.evaluate(
        f"""
            (async () => {{
                const token = localStorage.getItem('jwt_access_token');
                if (!token) return {{authenticated: false, status: 0}};
                try {{
                    const response = await fetch({json.dumps(endpoint)}, {{
                        method: 'GET', cache: 'no-store', credentials: 'omit',
                        headers: {{Accept: 'application/json', Authorization: `Bearer ${{token}}`}},
                    }});
                    let body = null;
                    try {{ body = await response.json(); }} catch (_) {{}}
                    return {{authenticated: true, status: response.status, body}};
                }} catch (_) {{
                    return {{authenticated: true, status: 0}};
                }}
            }})()
        """,
        await_promise=True,
        timeout=30,
    )
    if (
        not isinstance(response, dict)
        or response.get("authenticated") is not True
        or response.get("status") != 200
        or not isinstance(response.get("body"), dict)
    ):
        raise TransportSmokeError("The private viewer lease status is unavailable")
    body = response["body"]
    if (
        str(body.get("leaseId") or "").lower() != str(lease_id).lower()
        or str(body.get("sessionId") or "").lower() != str(session_id).lower()
    ):
        raise TransportSmokeError("The private viewer lease status changed identity")
    return body


def require_active_viewer_lease(
    ui: Any,
    session_id: uuid.UUID,
    binding: ViewerBinding,
) -> dict[str, bool]:
    status = viewer_lease_status(ui, session_id, binding.lease_id)
    command = status.get("command")
    if (
        status.get("isReady") is not True
        or status.get("revokedAt") is not None
        or not isinstance(command, dict)
        or command.get("state") != "Completed"
    ):
        raise TransportSmokeError("The bound private viewer lease is not active")
    return {
        "privateLeasePreserved": True,
        "activeBeforeTask": True,
        "publisherCommandCompleted": True,
    }


def wait_viewer_lease_closed(
    ui: Any,
    session_id: uuid.UUID,
    binding: ViewerBinding,
    timeout: float,
) -> dict[str, bool]:
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        status = viewer_lease_status(ui, session_id, binding.lease_id)
        close_command = status.get("closeCommand")
        if (
            status.get("isReady") is False
            and isinstance(status.get("revokedAt"), str)
            and isinstance(close_command, dict)
            and close_command.get("state") == "Completed"
        ):
            return {
                "viewerLeaseInactive": True,
                "viewerLeaseRevoked": True,
                "viewerCloseCommandTerminal": True,
            }
        if isinstance(close_command, dict) and close_command.get("state") in {
            "Failed", "Cancelled"
        }:
            raise TransportSmokeError("The private viewer close command did not complete")
        time.sleep(0.25)
    raise TransportSmokeError("The private viewer lease did not reach a closed terminal state")


def close_viewer(ui: Any, timeout: float) -> bool:
    ui.normalize_viewer()
    if ui.has_button("Cerrar visor"):
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
    return True


def require_one_session(ui: Any) -> uuid.UUID:
    sessions = ui._occupying_sessions()
    if len(sessions) != 1:
        raise TransportSmokeError("Account A does not own exactly one fresh session")
    session = sessions[0]
    try:
        session_id = uuid.UUID(str(session.get("id")))
    except (ValueError, AttributeError) as exc:
        raise TransportSmokeError("The fresh session has an invalid private identifier") from exc
    desired = session.get("desiredRobotCount")
    active = session.get("activeRobotCount")
    if desired not in (None, ROBOT_COUNT) or active not in (None, ROBOT_COUNT):
        raise TransportSmokeError("The backend session does not expose exactly four robots")
    return session_id


def full_task_inventory(ui: Any, session_id: uuid.UUID) -> list[dict[str, Any]]:
    """Read bounded task documents while the login token remains in Chrome."""
    endpoint = f"https://{VISIBLE.DEFAULT_API_HOST}/api/sessions/{session_id}/tasks"
    response = ui.cdp.evaluate(
        f"""
            (async () => {{
                const token = localStorage.getItem('jwt_access_token');
                if (!token) return {{authenticated: false, status: 0}};
                try {{
                    const response = await fetch({json.dumps(endpoint)}, {{
                        method: 'GET', cache: 'no-store', credentials: 'omit',
                        headers: {{Accept: 'application/json', Authorization: `Bearer ${{token}}`}},
                    }});
                    let body = null;
                    try {{ body = await response.json(); }} catch (_) {{}}
                    const size = body === null ? 0 : JSON.stringify(body).length;
                    if (size > {MAXIMUM_TASK_DOCUMENT}) {{
                        return {{authenticated: true, status: response.status, oversized: true}};
                    }}
                    return {{authenticated: true, status: response.status, body, size}};
                }} catch (_) {{
                    return {{authenticated: true, status: 0}};
                }}
            }})()
        """,
        await_promise=True,
        timeout=30,
    )
    if not isinstance(response, dict) or response.get("authenticated") is not True:
        raise TransportSmokeError("The authenticated task inventory is unavailable")
    if response.get("oversized") is True:
        raise TransportSmokeError("The authenticated task inventory exceeded its size bound")
    body = response.get("body")
    if response.get("status") != 200 or not isinstance(body, list):
        raise TransportSmokeError("The authenticated task inventory request failed")
    if len(json.dumps(body, separators=(",", ":"), allow_nan=False)) > MAXIMUM_TASK_DOCUMENT:
        raise TransportSmokeError("The task inventory exceeded its local size bound")
    for task in body:
        if not isinstance(task, dict) or UUID_PATTERN.fullmatch(str(task.get("id") or "")) is None:
            raise TransportSmokeError("The task inventory contains an invalid task")
    return body


def authoritative_robot_roster(ui: Any, session_id: uuid.UUID) -> set[str]:
    endpoint = f"https://{VISIBLE.DEFAULT_API_HOST}/api/sessions/{session_id}/robots"
    response = ui.cdp.evaluate(
        f"""
            (async () => {{
                const token = localStorage.getItem('jwt_access_token');
                if (!token) return {{authenticated: false, status: 0}};
                try {{
                    const response = await fetch({json.dumps(endpoint)}, {{
                        method: 'GET', cache: 'no-store', credentials: 'omit',
                        headers: {{Accept: 'application/json', Authorization: `Bearer ${{token}}`}},
                    }});
                    let body = null;
                    try {{ body = await response.json(); }} catch (_) {{}}
                    return {{authenticated: true, status: response.status, body}};
                }} catch (_) {{
                    return {{authenticated: true, status: 0}};
                }}
            }})()
        """,
        await_promise=True,
        timeout=30,
    )
    if (
        not isinstance(response, dict)
        or response.get("authenticated") is not True
        or response.get("status") != 200
        or not isinstance(response.get("body"), list)
    ):
        raise TransportSmokeError("The authoritative session roster is unavailable")
    robots = response["body"]
    if len(robots) != ROBOT_COUNT:
        raise TransportSmokeError("The authoritative session roster is not exactly four robots")
    roster: set[str] = set()
    ordinals: set[int] = set()
    for robot in robots:
        if not isinstance(robot, dict):
            raise TransportSmokeError("The authoritative session roster is malformed")
        runtime_id = robot.get("runtimeId")
        ordinal = robot.get("ordinal")
        if (
            not isinstance(runtime_id, str)
            or ROBOT_PATTERN.fullmatch(runtime_id) is None
            or isinstance(ordinal, bool)
            or not isinstance(ordinal, int)
            or robot.get("namespace") != f"/{runtime_id}"
            or robot.get("state") not in {"Ready", "Active"}
        ):
            raise TransportSmokeError("The authoritative session roster has an invalid member")
        roster.add(runtime_id)
        ordinals.add(ordinal)
    if len(roster) != ROBOT_COUNT or ordinals != set(range(ROBOT_COUNT)):
        raise TransportSmokeError("The authoritative session roster contains duplicates or gaps")
    return roster


def install_task_watcher(ui: Any, session_id: uuid.UUID, baseline_ids: set[str]) -> None:
    """Poll in-page so short persisted state transitions are not lost to CDP calls."""
    endpoint = f"https://{VISIBLE.DEFAULT_API_HOST}/api/sessions/{session_id}/tasks"
    installed = ui.cdp.evaluate(
        f"""
            (() => {{
                if (window.__robotswarmTransportWatch) return false;
                const baseline = new Set({json.dumps(sorted(baseline_ids))});
                const endpoint = {json.dumps(endpoint)};
                let target = null;
                let targetId = null;
                let busy = false;
                let stopped = false;
                let error = null;
                let consecutiveFailures = 0;
                let polls = 0;
                let stateRegression = false;
                let phaseRegression = false;
                const states = [];
                const phases = [];
                const samples = [];
                const criticalSamples = [];
                const criticalPhases = new Set();
                const stateRank = new Map([
                    ['Queued', 0], ['Accepted', 1], ['Running', 2], ['Completed', 3],
                ]);
                const phaseRank = new Map([
                    ['SEARCH', 0], ['APPROACH', 1], ['PUSH', 2], ['DONE', 3],
                ]);
                const remember = (list, value, ranks, markRegression) => {{
                    if (typeof value === 'string' && value && list[list.length - 1] !== value) {{
                        const previous = list[list.length - 1];
                        if (previous && (!ranks.has(value) || ranks.get(value) <= ranks.get(previous))) {{
                            markRegression();
                        }}
                        list.push(value);
                    }}
                }};
                const sample = task => {{
                    const state = typeof task?.state === 'string' ? task.state : null;
                    const phaseValue = task?.result?.transport?.phase;
                    const phase = typeof phaseValue === 'string' ? phaseValue.toUpperCase() : null;
                    remember(states, state, stateRank, () => {{ stateRegression = true; }});
                    remember(phases, phase, phaseRank, () => {{ phaseRegression = true; }});
                    const discovery = task?.result?.transport?.discovery;
                    const searching = task?.result?.transport?.searching_robot_count;
                    const contributors = task?.result?.transport?.useful_contributor_count;
                    const currentPushers = task?.result?.transport?.current_useful_pusher_count;
                    const currentPusherIds = task?.result?.transport?.current_useful_pusher_ids;
                    const observed = {{
                        at: Math.round(performance.now()),
                        atEpochMs: Date.now(),
                        state,
                        phase,
                        progress: Number.isFinite(Number(task?.progress))
                            ? Number(task.progress) : null,
                        searchingRobotCount: Number.isInteger(searching) ? searching : null,
                        usefulContributorCount: Number.isInteger(contributors) ? contributors : null,
                        currentUsefulPusherCount:
                            Number.isInteger(currentPushers) ? currentPushers : null,
                        currentUsefulPusherIds:
                            Array.isArray(currentPusherIds) ? [...currentPusherIds] : null,
                        allPushersConfirmed:
                            task?.result?.transport?.all_pushers_confirmed === true,
                        discoveryAnnounced: discovery?.announced === true,
                        discoveryCorrelated: Boolean(
                            discovery && typeof discovery.task_id === 'string'
                            && discovery.task_id === task?.id
                        ),
                    }};
                    samples.push(observed);
                    const critical = (
                        phase === 'SEARCH' && searching === {ROBOT_COUNT}
                    ) || (
                        phase === 'APPROACH' && observed.discoveryCorrelated
                    ) || (
                        phase === 'PUSH' && observed.allPushersConfirmed
                    ) || phase === 'DONE';
                    if (critical && !criticalPhases.has(phase)) {{
                        criticalPhases.add(phase);
                        criticalSamples.push({{...observed}});
                    }}
                    if (samples.length > 6000) samples.shift();
                }};
                const poll = async () => {{
                    if (busy || stopped) return;
                    busy = true;
                    try {{
                        const token = localStorage.getItem('jwt_access_token');
                        if (!token) throw new Error('authentication unavailable');
                        const response = await fetch(endpoint, {{
                            method: 'GET', cache: 'no-store', credentials: 'omit',
                            headers: {{Accept: 'application/json', Authorization: `Bearer ${{token}}`}},
                        }});
                        if (!response.ok) throw new Error('task inventory unavailable');
                        const tasks = await response.json();
                        if (!Array.isArray(tasks)) throw new Error('task inventory malformed');
                        if (JSON.stringify(tasks).length > {MAXIMUM_TASK_DOCUMENT}) {{
                            throw new Error('task inventory oversized');
                        }}
                        const created = tasks.filter(task => !baseline.has(task?.id));
                        if (created.length > 1) throw new Error('multiple new tasks');
                        if (created.length === 1) {{
                            if (targetId !== null && created[0].id !== targetId) {{
                                throw new Error('new task identity changed');
                            }}
                            targetId = created[0].id;
                            target = created[0];
                            sample(target);
                        }}
                        consecutiveFailures = 0;
                        error = null;
                        polls += 1;
                    }} catch (failure) {{
                        consecutiveFailures += 1;
                        if (consecutiveFailures >= 3) {{
                            error = String(failure?.message || 'watch failed').slice(0, 120);
                        }}
                    }} finally {{
                        busy = false;
                    }}
                }};
                const timer = setInterval(poll, 100);
                poll();
                window.__robotswarmTransportWatch = {{
                    read: () => ({{
                        polls, error, taskCount: targetId === null ? 0 : 1,
                        states: [...states], phases: [...phases], samples: [...samples],
                        criticalSamples: criticalSamples.map(item => ({{...item}})),
                        stateRegression, phaseRegression,
                        latest: target,
                    }}),
                    stop: () => {{ stopped = true; clearInterval(timer); }},
                }};
                return true;
            }})()
        """
    )
    if installed is not True:
        raise TransportSmokeError("The task-state watcher could not be installed")


def read_task_watcher(ui: Any) -> dict[str, Any]:
    result = ui.cdp.evaluate("window.__robotswarmTransportWatch?.read() || null")
    if not isinstance(result, dict):
        raise TransportSmokeError("The task-state watcher is unavailable")
    if result.get("error"):
        raise TransportSmokeError(f"The task-state watcher failed: {result['error']}")
    if not isinstance(result.get("states"), list) or not isinstance(result.get("phases"), list):
        raise TransportSmokeError("The task-state watcher returned invalid history")
    if not isinstance(result.get("samples"), list) or not isinstance(
        result.get("criticalSamples"), list
    ):
        raise TransportSmokeError("The task-state watcher did not preserve critical samples")
    if result.get("stateRegression") is True or result.get("phaseRegression") is True:
        raise TransportSmokeError("The persisted transport automaton regressed")
    return result


def stop_task_watcher(ui: Any) -> None:
    with contextlib.suppress(Exception):
        ui.cdp.evaluate("window.__robotswarmTransportWatch?.stop()")


def install_start_probe(ui: Any, session_id: uuid.UUID) -> None:
    """Observe one trusted click and its retried POST without exporting IDs."""
    expected_path = f"/api/sessions/{session_id}/tasks"
    expected_origin = f"https://{VISIBLE.DEFAULT_API_HOST}"
    installed = ui.cdp.evaluate(
        f"""
            (() => {{
                if (window.__robotswarmTransportStartProbe) return false;
                const expectedPath = {json.dumps(expected_path)};
                const expectedOrigin = {json.dumps(expected_origin)};
                const originalOpen = XMLHttpRequest.prototype.open;
                const originalSend = XMLHttpRequest.prototype.send;
                const originalSetRequestHeader = XMLHttpRequest.prototype.setRequestHeader;
                const requests = new WeakMap();
                const taskIds = new Set();
                const commandIds = new Set();
                const responses = [];
                const clicks = [];
                let attempts = 0;
                let completed = 0;
                let stopped = false;
                const normalize = value => (value || '').replace(/\\s+/g, ' ').trim();
                const startButton = [...document.querySelectorAll('button')]
                    .find(item => normalize(item.textContent) === {json.dumps(START_BUTTON)}
                        && item.offsetParent !== null);
                if (!startButton) return false;
                const onClick = event => {{
                    const button = event.target?.closest?.('button');
                    if (button !== startButton) return;
                    clicks.push({{
                        atEpochMs: Date.now(),
                        atPerformanceMs: performance.now(),
                        trusted: event.isTrusted === true,
                        button: Number(event.button),
                        detail: Number(event.detail),
                    }});
                }};
                document.addEventListener('click', onClick, true);
                const fingerprint = async value => {{
                    if (typeof value !== 'string' || !value || !crypto?.subtle) return null;
                    const input = new TextEncoder().encode(
                        'robotswarm-transport-ui/idempotency/' + value
                    );
                    const digest = await crypto.subtle.digest('SHA-256', input);
                    return [...new Uint8Array(digest)]
                        .map(item => item.toString(16).padStart(2, '0'))
                        .join('').slice(0, 16);
                }};
                XMLHttpRequest.prototype.open = function(method, url, ...rest) {{
                    let path = '';
                    let origin = '';
                    try {{
                        const parsed = new URL(String(url), location.href);
                        path = parsed.pathname;
                        origin = parsed.origin;
                    }} catch (_) {{}}
                    requests.set(this, {{
                        method: String(method).toUpperCase(), path, origin,
                        idempotencyKey: null, sentAtEpochMs: null, attempt: null,
                    }});
                    return originalOpen.call(this, method, url, ...rest);
                }};
                XMLHttpRequest.prototype.setRequestHeader = function(name, value) {{
                    const request = requests.get(this);
                    if (request && String(name).toLowerCase() === 'idempotency-key') {{
                        request.idempotencyKey = String(value);
                    }}
                    return originalSetRequestHeader.call(this, name, value);
                }};
                XMLHttpRequest.prototype.send = function(body) {{
                    const request = requests.get(this);
                    if (
                        request?.method === 'POST'
                        && request.path === expectedPath
                        && request.origin === expectedOrigin
                    ) {{
                        attempts += 1;
                        request.attempt = attempts;
                        request.sentAtEpochMs = Date.now();
                        this.addEventListener('loadend', async () => {{
                            let payload = null;
                            try {{ payload = JSON.parse(this.responseText || 'null'); }} catch (_) {{}}
                            const task = payload?.task;
                            const command = payload?.command;
                            if (typeof task?.id === 'string') taskIds.add(task.id);
                            if (typeof command?.id === 'string') commandIds.add(command.id);
                            const keyFingerprint = await fingerprint(
                                request.idempotencyKey
                            );
                            responses.push({{
                                attempt: request.attempt,
                                status: Number(this.status) || 0,
                                requestEpochMs: request.sentAtEpochMs,
                                responseEpochMs: Date.now(),
                                gestureToRequestMs: clicks.length === 1
                                    ? request.sentAtEpochMs - clicks[0].atEpochMs : null,
                                keyFingerprint,
                                conflictCode: typeof payload?.code === 'string' ? payload.code : null,
                                conflictRetryable: payload?.retryable === true,
                                taskId: typeof task?.id === 'string' ? task.id : null,
                                commandId: typeof command?.id === 'string' ? command.id : null,
                                taskSessionId:
                                    typeof task?.sessionId === 'string' ? task.sessionId : null,
                                commandSessionId:
                                    typeof command?.sessionId === 'string' ? command.sessionId : null,
                                commandTaskRunId:
                                    typeof command?.taskRunId === 'string' ? command.taskRunId : null,
                                taskCreatedAt:
                                    typeof task?.createdAt === 'string' ? task.createdAt : null,
                                commandCreatedAt:
                                    typeof command?.createdAt === 'string' ? command.createdAt : null,
                                taskType: typeof task?.type === 'string' ? task.type : null,
                                taskState: typeof task?.state === 'string' ? task.state : null,
                                commandType: typeof command?.type === 'string' ? command.type : null,
                                commandState: typeof command?.state === 'string' ? command.state : null,
                                taskCommandCorrelated: Boolean(
                                    task?.id && task.id === command?.taskRunId
                                    && task.sessionId === command?.sessionId
                                    && String(task.sessionId).toLowerCase() ===
                                        {json.dumps(str(session_id).lower())}
                                ),
                                targetAccepted: Number(task?.parameters?.target_x) === {TARGET_X}
                                    && Number(task?.parameters?.target_y) === {TARGET_Y}
                                    && task?.parameters?.config?.transport_planner === {json.dumps(PLANNER)},
                            }});
                            responses.sort((left, right) => left.attempt - right.attempt);
                            completed += 1;
                        }}, {{once: true}});
                    }}
                    return originalSend.call(this, body);
                }};
                window.__robotswarmTransportStartProbe = {{
                    read: () => ({{
                        attempts, completed,
                        physicalClickCount: clicks.length,
                        clicks: clicks.map(item => ({{...item}})),
                        logicalTaskCount: taskIds.size,
                        logicalCommandCount: commandIds.size,
                        responses: responses.map(item => ({{...item}})),
                    }}),
                    stop: () => {{
                        if (stopped) return;
                        stopped = true;
                        document.removeEventListener('click', onClick, true);
                        XMLHttpRequest.prototype.open = originalOpen;
                        XMLHttpRequest.prototype.send = originalSend;
                        XMLHttpRequest.prototype.setRequestHeader = originalSetRequestHeader;
                    }},
                }};
                return true;
            }})()
        """
    )
    if installed is not True:
        raise TransportSmokeError("The one-click start probe could not be installed")


def stop_start_probe(ui: Any) -> None:
    with contextlib.suppress(Exception):
        ui.cdp.evaluate("window.__robotswarmTransportStartProbe?.stop()")


def read_start_probe(
    ui: Any,
    session_id: uuid.UUID,
    *,
    timeout: float = 15,
    quiet_period: float = START_PROBE_QUIET_SECONDS,
) -> StartProbeEvidence:
    deadline = time.monotonic() + timeout
    last: dict[str, Any] | None = None
    settled: dict[str, Any] | None = None
    while time.monotonic() < deadline:
        value = ui.cdp.evaluate("window.__robotswarmTransportStartProbe?.read() || null")
        if isinstance(value, dict):
            last = value
            if (
                int(value.get("attempts") or 0) > 0
                and value.get("completed") == value.get("attempts")
                and value.get("physicalClickCount") == 1
                and value.get("logicalTaskCount") == 1
                and value.get("logicalCommandCount") == 1
            ):
                settled = value
                break
        time.sleep(0.1)
    if not isinstance(settled, dict):
        if not isinstance(last, dict):
            raise TransportSmokeError("The frontend start request was not observed")
        settled = last
    quiet_deadline = time.monotonic() + max(0.0, quiet_period)
    while time.monotonic() < quiet_deadline:
        time.sleep(min(0.1, quiet_deadline - time.monotonic()))
        value = ui.cdp.evaluate("window.__robotswarmTransportStartProbe?.read() || null")
        if isinstance(value, dict):
            last = value
    if not isinstance(last, dict) or json.dumps(last, sort_keys=True) != json.dumps(
        settled, sort_keys=True
    ):
        raise TransportSmokeError("The delayed start-probe read found a late extra request or click")
    attempts = last.get("attempts")
    responses = last.get("responses")
    clicks = last.get("clicks")
    if (
        not isinstance(attempts, int)
        or not 1 <= attempts <= 4
        or last.get("completed") != attempts
        or last.get("physicalClickCount") != 1
        or last.get("logicalTaskCount") != 1
        or last.get("logicalCommandCount") != 1
        or not isinstance(responses, list)
        or len(responses) != attempts
        or not isinstance(clicks, list)
        or len(clicks) != 1
    ):
        raise TransportSmokeError("The UI start did not reconcile to one logical command and task")
    click = clicks[0]
    if (
        not isinstance(click, dict)
        or click.get("trusted") is not True
        or click.get("button") != 0
        or click.get("detail") != 1
        or isinstance(click.get("atEpochMs"), bool)
        or not isinstance(click.get("atEpochMs"), (int, float))
    ):
        raise TransportSmokeError("The start gesture was not exactly one trusted primary click")
    if [item.get("attempt") for item in responses if isinstance(item, dict)] != list(
        range(1, attempts + 1)
    ):
        raise TransportSmokeError("The frontend start attempts were not observed in order")
    statuses = [item.get("status") for item in responses if isinstance(item, dict)]
    if statuses != [409] * (attempts - 1) + [202]:
        raise TransportSmokeError("The UI start retries were not exact conflicts followed by one 202")
    fingerprints = {
        item.get("keyFingerprint") for item in responses if isinstance(item, dict)
    }
    if len(fingerprints) != 1 or re.fullmatch(r"[0-9a-f]{16}", str(next(iter(fingerprints), ""))) is None:
        raise TransportSmokeError("The retry sequence did not reuse one fingerprinted Idempotency-Key")
    for response in responses[:-1]:
        if (
            not isinstance(response, dict)
            or response.get("status") != 409
            or response.get("conflictCode") != "serialization_conflict"
            or response.get("conflictRetryable") is not True
            or any(
                response.get(key) is not None
                for key in (
                    "taskId", "commandId", "taskType", "taskState",
                    "commandType", "commandState", "taskCreatedAt", "commandCreatedAt",
                    "taskSessionId", "commandSessionId", "commandTaskRunId",
                )
            )
        ):
            raise TransportSmokeError("A retry was not an exact retryable serialization conflict")
    response = responses[-1]
    if not isinstance(response, dict):
        raise TransportSmokeError("The accepted frontend response is invalid")
    for item in responses:
        request_time = item.get("requestEpochMs")
        response_time = item.get("responseEpochMs")
        gesture_delay = item.get("gestureToRequestMs")
        if (
            isinstance(request_time, bool)
            or not isinstance(request_time, (int, float))
            or isinstance(response_time, bool)
            or not isinstance(response_time, (int, float))
            or request_time < click["atEpochMs"]
            or response_time < request_time
            or isinstance(gesture_delay, bool)
            or not isinstance(gesture_delay, (int, float))
            or not 0 <= gesture_delay <= 15000
        ):
            raise TransportSmokeError("The physical click and POST timeline is not causal")
    expected_session = str(session_id).lower()
    if (
        response.get("status") != 202
        or response.get("taskType") != TASK_TYPE
        or response.get("commandType") != "StartTask"
        or response.get("taskCommandCorrelated") is not True
        or response.get("targetAccepted") is not True
        or response.get("commandState") not in {
            "Pending", "Dispatched", "Acknowledged", "Running", "Completed"
        }
        or UUID_PATTERN.fullmatch(str(response.get("taskId") or "")) is None
        or UUID_PATTERN.fullmatch(str(response.get("commandId") or "")) is None
        or str(response.get("taskSessionId") or "").lower() != expected_session
        or str(response.get("commandSessionId") or "").lower() != expected_session
        or str(response.get("commandTaskRunId") or "").lower()
            != str(response.get("taskId") or "").lower()
        or not isinstance(response.get("taskCreatedAt"), str)
        or not isinstance(response.get("commandCreatedAt"), str)
    ):
        raise TransportSmokeError("The observed frontend start response is invalid")
    parse_utc_timestamp(response["taskCreatedAt"], "accepted task creation")
    parse_utc_timestamp(response["commandCreatedAt"], "accepted command creation")
    fingerprint_value = next(iter(fingerprints))
    return StartProbeEvidence(
        report={
            "httpAttempts": attempts,
            "retryableSerializationConflicts": attempts - 1,
            "physicalClicks": 1,
            "trustedPrimaryClick": True,
            "clickToRequestCausal": True,
            "delayedRereadStable": True,
            "logicalTaskRuns": 1,
            "logicalStartCommands": 1,
            "commandType": "StartTask",
            "initialCommandState": response["commandState"],
            "taskCommandCorrelation": True,
            "targetAccepted": True,
            "idempotency": {
                "sameKeyAcrossRetries": True,
                "fingerprint": fingerprint_value,
                "rawKeyExported": False,
            },
        },
        task_id=str(response["taskId"]),
        command_id=str(response["commandId"]),
        task_created_at=str(response["taskCreatedAt"]),
        command_created_at=str(response["commandCreatedAt"]),
        click_epoch_ms=float(click["atEpochMs"]),
        request_epoch_ms=float(response["requestEpochMs"]),
        response_epoch_ms=float(response["responseEpochMs"]),
    )


def fill_labeled_number(ui: Any, label_text: str, value: float) -> dict[str, Any]:
    result = ui.cdp.evaluate(
        f"""
            (() => {{
                const wanted = {json.dumps(label_text)};
                const normalize = value => (value || '').replace(/\\s+/g, ' ').trim();
                const label = [...document.querySelectorAll('label')]
                    .find(item => normalize(item.textContent).replace(/[*]$/, '').trim() === wanted
                        && item.offsetParent !== null);
                const input = (label?.htmlFor && document.getElementById(label.htmlFor))
                    || label?.parentElement?.querySelector('input');
                if (!(input instanceof HTMLInputElement) || input.type !== 'number') return null;
                const setter = Object.getOwnPropertyDescriptor(HTMLInputElement.prototype, 'value').set;
                setter.call(input, {json.dumps(f'{value:g}')});
                input.dispatchEvent(new Event('input', {{bubbles: true}}));
                input.dispatchEvent(new Event('change', {{bubbles: true}}));
                input.blur();
                return {{
                    value: input.value,
                    type: input.type,
                    minimum: input.min,
                    maximum: input.max,
                    step: input.step,
                    visible: input.offsetParent !== null,
                }};
            }})()
        """
    )
    if not isinstance(result, dict) or result.get("visible") is not True:
        raise TransportSmokeError(f"The real UI input {label_text} was not available")
    if (
        result.get("type") != "number"
        or float(result.get("value")) != value
        or float(result.get("minimum")) != -4
        or float(result.get("maximum")) != 4
        or float(result.get("step")) != 0.25
    ):
        raise TransportSmokeError(f"The UI input {label_text} did not keep its expected contract")
    ui.wait_js(
        f"""
            (() => {{
                const wanted = {json.dumps(label_text)};
                const normalize = value => (value || '').replace(/\\s+/g, ' ').trim();
                const label = [...document.querySelectorAll('label')]
                    .find(item => normalize(item.textContent).replace(/[*]$/, '').trim() === wanted);
                const input = (label?.htmlFor && document.getElementById(label.htmlFor))
                    || label?.parentElement?.querySelector('input');
                return Number(input?.value) === {value};
            }})()
        """,
        8,
        f"controlled value for {label_text}",
    )
    return {
        "label": label_text,
        "value": value,
        "nativeNumberInput": True,
        "constraintsVerified": True,
    }


def require_start_barrier(ui: Any) -> dict[str, bool]:
    result = ui.cdp.evaluate(
        f"""
            (() => {{
                const normalize = value => (value || '').replace(/\\s+/g, ' ').trim();
                const button = [...document.querySelectorAll('button')]
                    .find(item => normalize(item.textContent) === {json.dumps(START_BUTTON)}
                        && item.offsetParent !== null);
                if (!button || button.disabled || button.getAttribute('aria-disabled') === 'true') {{
                    return null;
                }}
                button.scrollIntoView({{block: 'center', inline: 'nearest'}});
                const box = button.getBoundingClientRect();
                const x = box.left + box.width / 2;
                const y = box.top + box.height / 2;
                const hit = document.elementFromPoint(x, y);
                const visibleBlocker = [...document.querySelectorAll(
                    '[role="dialog"], [role="listbox"], .MuiModal-root, '
                    + '.MuiPopover-root, .MuiBackdrop-root'
                )].some(element => {{
                    const style = getComputedStyle(element);
                    const rect = element.getBoundingClientRect();
                    return style.display !== 'none' && style.visibility !== 'hidden'
                        && style.pointerEvents !== 'none' && rect.width > 0 && rect.height > 0;
                }});
                return {{
                    enabled: true,
                    centerHitTest: Boolean(hit && button.contains(hit)),
                    overlayFree: !visibleBlocker,
                    validationClear: !document.querySelector('[data-testid="task-validation"]'),
                }};
            }})()
        """
    )
    if (
        not isinstance(result, dict)
        or result.get("enabled") is not True
        or result.get("centerHitTest") is not True
        or result.get("overlayFree") is not True
        or result.get("validationClear") is not True
    ):
        raise TransportSmokeError("The start button is disabled, covered, or blocked by validation")
    ui.wait_clickable_button(START_BUTTON)
    return {key: True for key in ("enabled", "centerHitTest", "overlayFree", "validationClear")}


def task_from_watch(watch: dict[str, Any]) -> dict[str, Any]:
    task = watch.get("latest")
    if not isinstance(task, dict):
        raise TransportSmokeError("The persisted CollaborativeTransport task was not observed")
    if task.get("type") != TASK_TYPE or UUID_PATTERN.fullmatch(str(task.get("id") or "")) is None:
        raise TransportSmokeError("The observed backend task has an invalid identity or type")
    return task


def correlate_private_start(
    start: StartProbeEvidence,
    task: dict[str, Any],
    session_id: uuid.UUID,
    container: ContainerHandle,
) -> dict[str, bool]:
    if (
        task.get("id") != start.task_id
        or str(task.get("sessionId") or "").lower() != str(session_id).lower()
    ):
        raise TransportSmokeError("The XHR response and persisted TaskRun do not match")
    task_created = parse_utc_timestamp(start.task_created_at, "XHR task creation")
    persisted_created = parse_utc_timestamp(task.get("createdAt"), "persisted task creation")
    command_created = parse_utc_timestamp(start.command_created_at, "command creation")
    click_time = dt.datetime.fromtimestamp(start.click_epoch_ms / 1000, tz=dt.timezone.utc)
    response_time = dt.datetime.fromtimestamp(start.response_epoch_ms / 1000, tz=dt.timezone.utc)
    if abs((persisted_created - task_created).total_seconds()) > 0.001:
        raise TransportSmokeError("The accepted and persisted TaskRun timestamps differ")
    if not (
        click_time - dt.timedelta(seconds=5)
        <= task_created
        <= response_time + dt.timedelta(seconds=5)
    ):
        raise TransportSmokeError("The TaskRun was not created in the one-click POST window")
    if not (
        click_time - dt.timedelta(seconds=5)
        <= command_created
        <= response_time + dt.timedelta(seconds=5)
    ) or abs((command_created - task_created).total_seconds()) > 5:
        raise TransportSmokeError("The StartTask command was not created with the one-click TaskRun")
    if (
        container.started_at is None
        or container.started_at > click_time + dt.timedelta(seconds=1)
        or UUID_PATTERN.fullmatch(container.worker_identifier) is None
    ):
        raise TransportSmokeError("The private worker container was not established before the click")
    return {
        "privateTaskIdMatched": True,
        "privateCommandTaskCorrelation": True,
        "taskAndCommandCreatedInPostWindow": True,
        "containerStartedBeforeGesture": True,
        "workerAndContainerPrivatelyCorrelated": True,
    }


def wait_for_task_state(ui: Any, state: str, timeout: float) -> dict[str, Any]:
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        ui.raise_if_interrupted()
        watch = read_task_watcher(ui)
        if watch.get("taskCount") == 1:
            task = task_from_watch(watch)
            current = task.get("state")
            if state in watch["states"]:
                return watch
            if current in {"Failed", "Cancelled"}:
                raise TransportSmokeError(f"Transport entered unexpected state {current}")
        time.sleep(0.1)
    raise TransportSmokeError(f"Timed out waiting for persisted task state {state}")


def wait_for_phase(ui: Any, phase: str, timeout: float) -> dict[str, Any]:
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        ui.raise_if_interrupted()
        watch = read_task_watcher(ui)
        if watch.get("taskCount") == 1:
            task = task_from_watch(watch)
            current = str(((task.get("result") or {}).get("transport") or {}).get("phase") or "").upper()
            if current == phase and task.get("state") == "Running":
                return watch
            if task.get("state") in TERMINAL_STATES:
                raise TransportSmokeError(f"Transport ended before live phase {phase} was captured")
        time.sleep(0.15)
    raise TransportSmokeError(f"Timed out waiting for correlated transport phase {phase}")


def safe_capture(details: dict[str, Any], phase: str) -> dict[str, Any]:
    return {
        "file": str(details.get("file") or ""),
        "sha256": str(details.get("sha256") or ""),
        "bytes": int(details.get("bytes") or 0),
        "phase": phase,
        "sanitized": True,
        "taskCorrelated": True,
        "humanReviewRequired": True,
    }


def capture_correlated_phase(
    ui: Any,
    phase: str,
    evidence_dir: Path,
    account: dict[str, str],
    *,
    timeout: float,
) -> PhaseCapture:
    before = wait_for_phase(ui, phase, timeout)
    before_task = task_from_watch(before)
    task_id = before_task["id"]
    ui.show_task_panel()
    ui.wait_js(
        "document.querySelector('[data-testid=\"transport-phase\"]')"
        f"?.innerText.includes({json.dumps(PHASE_LABELS[phase])})",
        5,
        f"visible {phase} task phase",
    )
    screen = ui.screenshot(
        evidence_dir / f"transport-{phase.lower()}-panel.png",
        account["email"],
        account["password"],
    )
    ui.cdp.evaluate("document.querySelector('[data-testid=\"private-viewer\"]')?.scrollIntoView({block:'center'})")
    clip = ui.capture_video_clip(evidence_dir / f"transport-{phase.lower()}-gazebo.png")
    after = read_task_watcher(ui)
    after_task = task_from_watch(after)
    after_phase = str(
        (((after_task.get("result") or {}).get("transport") or {}).get("phase") or "")
    ).upper()
    if after_task.get("id") != task_id or after_task.get("state") != "Running" or after_phase != phase:
        raise TransportSmokeError(f"The {phase} capture could not be correlated to the running task")
    average_hash = str(clip.get("averageHash") or "")
    if re.fullmatch(r"[0-9a-f]{64}", average_hash) is None:
        raise TransportSmokeError(f"The {phase} Gazebo frame has no valid visual fingerprint")
    return PhaseCapture(
        [safe_capture(screen, phase), safe_capture(clip, phase)],
        str(task_id),
        average_hash,
        evidence_dir / str(clip.get("file") or ""),
    )


def validate_visual_change(
    first: PhaseCapture,
    second: PhaseCapture,
) -> dict[str, Any]:
    if first.task_id != second.task_id:
        raise TransportSmokeError("The two Gazebo frames belong to different tasks")
    distance = VISIBLE.RobotSwarmUi._hash_distance(
        first.average_hash,
        second.average_hash,
    )
    changed_ratio: float | None = None
    if first.file_path is not None and second.file_path is not None:
        if not first.file_path.is_file() or not second.file_path.is_file():
            raise TransportSmokeError("A task-correlated Gazebo frame is missing")
        changed_ratio = VISIBLE.RobotSwarmUi._scene_difference(
            first.file_path,
            second.file_path,
        )
    if (
        changed_ratio is not None
        and changed_ratio < MINIMUM_TASK_FRAME_CHANGE_RATIO
    ) or (changed_ratio is None and distance < 1):
        raise TransportSmokeError("The two task-correlated Gazebo frames did not change visually")
    result = {
        "correlatedFrames": 2,
        "differentFrames": True,
        "averageHashDistance": distance,
        "samePrivateTask": True,
    }
    if changed_ratio is not None:
        result.update({
            "changedPixelRatio": round(changed_ratio, 6),
            "minimumChangedPixelRatio": MINIMUM_TASK_FRAME_CHANGE_RATIO,
        })
    return result


def video_evidence(ui: Any, seconds: float) -> dict[str, Any]:
    before = read_task_watcher(ui)
    before_task = task_from_watch(before)
    if before_task.get("state") != "Running":
        raise TransportSmokeError("The HLS measurement did not start while transport was running")
    task_id = before_task["id"]
    metrics = ui.video_metrics(seconds)
    VISIBLE.require_live_video(
        metrics,
        seconds,
        "CollaborativeTransport HLS",
        MINIMUM_VIDEO_FPS,
        MAXIMUM_DROPPED_RATIO,
    )
    after = read_task_watcher(ui)
    after_task = task_from_watch(after)
    if after_task.get("id") != task_id or after_task.get("state") != "Running":
        raise TransportSmokeError("Transport did not remain running for the full HLS interval")
    return {
        "measurementSeconds": seconds,
        "elapsedSeconds": float(metrics.get("elapsedSeconds") or 0),
        "mediaTimeAdvancedSeconds": float(metrics.get("mediaTimeAdvancedSeconds") or 0),
        "presentedFrames": int(metrics.get("callbackFrames") or 0),
        "presentedFps": float(metrics.get("callbackFps") or 0),
        "decodedFrames": int(metrics.get("decodedFrames") or 0),
        "decodedFps": float(metrics.get("decodedFps") or 0),
        "droppedFrames": (
            None if metrics.get("droppedFrames") is None else int(metrics["droppedFrames"])
        ),
        "droppedRatio": (
            None if metrics.get("droppedRatio") is None else float(metrics["droppedRatio"])
        ),
        "minimumAcceptedFps": MINIMUM_VIDEO_FPS,
        "maximumDroppedRatio": MAXIMUM_DROPPED_RATIO,
        "mediaProgressing": True,
        "measuredWhileRunning": True,
        "taskCorrelated": True,
    }


def wait_for_terminal(ui: Any, timeout: float) -> dict[str, Any]:
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        ui.raise_if_interrupted()
        watch = read_task_watcher(ui)
        if watch.get("taskCount") == 1:
            task = task_from_watch(watch)
            if task.get("state") in TERMINAL_STATES:
                return watch
        time.sleep(0.2)
    raise TransportSmokeError("Timed out waiting for terminal CollaborativeTransport evidence")


def _ordered_contains(values: list[str], expected: Sequence[str]) -> bool:
    try:
        positions = [values.index(value) for value in expected]
    except ValueError:
        return False
    return positions == sorted(positions) and len(set(positions)) == len(positions)


def _unexpected_failures(value: Any, path: str = "result") -> list[str]:
    failures: list[str] = []
    if isinstance(value, dict):
        for key, item in value.items():
            clean_key = str(key).lower()
            location = f"{path}.{key}"
            if clean_key in {"failure", "failures", "error", "errors"}:
                if item not in (None, "", [], {}):
                    failures.append(location)
            if "unexpected" in clean_key and ("collision" in clean_key or "contact" in clean_key):
                if (
                    (isinstance(item, bool) and item)
                    or (not isinstance(item, (bool, int, float)))
                    or (not isinstance(item, bool) and float(item) != 0)
                ):
                    failures.append(location)
            failures.extend(_unexpected_failures(item, location))
    elif isinstance(value, list):
        for index, item in enumerate(value):
            failures.extend(_unexpected_failures(item, f"{path}[{index}]") )
    return failures


def validate_transport_task(
    task: dict[str, Any],
    states: list[str],
    phases: list[str],
    session_id: uuid.UUID,
    samples: list[dict[str, Any]],
    roster: set[str],
) -> dict[str, Any]:
    if task.get("type") != TASK_TYPE:
        raise TransportSmokeError("The terminal task type changed")
    if str(task.get("sessionId") or "").lower() != str(session_id).lower():
        raise TransportSmokeError("The terminal task is not bound to the fresh session")
    if task.get("state") != "Completed" or task.get("outcomeState") != "Succeeded":
        raise TransportSmokeError("Collaborative transport did not complete successfully")
    if task.get("error") not in (None, "") or task.get("outcomeReason") not in (None, ""):
        raise TransportSmokeError("Collaborative transport persisted an error or failure reason")
    state_history = _require_monotonic_history(
        states,
        ("Queued", "Accepted", "Running", "Completed"),
        "persisted task state",
    )
    if not _ordered_contains(state_history, ("Accepted", "Running", "Completed")):
        raise TransportSmokeError("Persisted task history missed Accepted, Running, or Completed")
    phase_history = _require_monotonic_history(
        phases,
        ("SEARCH", "APPROACH", "PUSH", "DONE"),
        "persisted transport phase",
    )
    if not _ordered_contains(phase_history, ("SEARCH", "PUSH", "DONE")):
        raise TransportSmokeError("Correlated transport history missed SEARCH, PUSH, or DONE")
    if not isinstance(samples, list) or not all(isinstance(item, dict) for item in samples):
        raise TransportSmokeError("The correlated task sample history is invalid")
    all_searching = any(
        item.get("state") == "Running"
        and item.get("phase") == "SEARCH"
        and item.get("searchingRobotCount") == ROBOT_COUNT
        for item in samples
    )
    live_joint_push = any(
        item.get("state") == "Running"
        and item.get("phase") == "PUSH"
        and item.get("allPushersConfirmed") is True
        and item.get("usefulContributorCount") == ROBOT_COUNT
        for item in samples
    )
    if not all_searching:
        raise TransportSmokeError("The live SEARCH phase did not show all four robots searching")
    if not live_joint_push:
        raise TransportSmokeError("The live PUSH phase did not show all four useful pushers")

    parameters = task.get("parameters")
    if not isinstance(parameters, dict):
        raise TransportSmokeError("The persisted task parameters are unavailable")
    config = parameters.get("config")
    if (
        float(parameters.get("target_x")) != TARGET_X
        or float(parameters.get("target_y")) != TARGET_Y
        or not isinstance(config, dict)
        or config.get("transport_planner") != PLANNER
        or float(config.get("target_x")) != TARGET_X
        or float(config.get("target_y")) != TARGET_Y
    ):
        raise TransportSmokeError("The persisted transport target differs from the UI inputs")

    result = task.get("result")
    transport = result.get("transport") if isinstance(result, dict) else None
    discovery = transport.get("discovery") if isinstance(transport, dict) else None
    if not isinstance(transport, dict) or not isinstance(discovery, dict):
        raise TransportSmokeError("The terminal task has no transport discovery evidence")
    task_id = str(task.get("id") or "")
    if len(roster) != ROBOT_COUNT or any(ROBOT_PATTERN.fullmatch(item) is None for item in roster):
        raise TransportSmokeError("The authoritative robot roster is invalid")
    finder = discovery.get("finder")
    notified = discovery.get("notified_robots")
    contributors = transport.get("useful_contributor_ids")
    if (
        str(transport.get("phase") or "").upper() != "DONE"
        or discovery.get("event") != "payload_found"
        or discovery.get("announced") is not True
        or str(discovery.get("task_id") or "").lower() != task_id.lower()
        or finder not in roster
        or not isinstance(notified, list)
        or len(notified) != ROBOT_COUNT - 1
        or set(notified) != roster - {finder}
        or len(set(notified)) != len(notified)
        or transport.get("all_pushers_confirmed") is not True
        or transport.get("useful_contributor_count") != ROBOT_COUNT
        or not isinstance(contributors, list)
        or len(contributors) != ROBOT_COUNT
        or set(contributors) != roster
        or len(set(contributors)) != len(contributors)
    ):
        raise TransportSmokeError("The terminal result does not prove finder, notice, and four pushers")
    unexpected = _unexpected_failures(result)
    if unexpected:
        raise TransportSmokeError("The terminal result contains an unexpected collision or failure")
    return {
        "terminalState": "Completed",
        "outcome": "Succeeded",
        "workerCommandConsumptionProven": True,
        "workerCommandConsumptionBasis": "persisted Accepted and Running reports",
        "statesObserved": state_history,
        "phasesObserved": phase_history,
        "persistedApiObserver": {"pollIntervalMs": 100, "bounded": True},
        "search": {
            "activeSearchingRobots": ROBOT_COUNT,
            "allRobotsSearching": True,
            "movementClaimedByPersistedApi": False,
        },
        "target": {"x": TARGET_X, "y": TARGET_Y, "planner": PLANNER},
        "discovery": {
            "event": "payload_found",
            "announced": True,
            "taskCorrelated": True,
            "finderInAuthoritativeRoster": True,
            "notifiedTeammates": ROBOT_COUNT - 1,
            "completeRecipientFleet": True,
        },
        "push": {
            "allPushersConfirmed": True,
            "usefulContributors": ROBOT_COUNT,
            "exactAuthoritativeRoster": True,
        },
        "done": True,
        "noUnexpectedCollisionOrFailure": True,
        "collisionFailureBasis": (
            "terminal success, empty task errors, and no nonzero explicit unexpected-contact field"
        ),
    }


def persisted_terminal_read(
    ui: Any,
    session_id: uuid.UUID,
    task_id: str,
) -> dict[str, Any]:
    matches = [task for task in full_task_inventory(ui, session_id) if task.get("id") == task_id]
    if len(matches) != 1 or matches[0].get("state") != "Completed":
        raise TransportSmokeError("The completed task was not persisted exactly once")
    if len(full_task_inventory(ui, session_id)) != 1:
        raise TransportSmokeError("A single UI click created more than one backend TaskRun")
    return matches[0]


def decoded_hls_fps(state: dict[str, Any]) -> float:
    match = re.fullmatch(r"Video (\d+(?:\.\d+)?) FPS", str(state.get("fps") or ""))
    if state.get("hlsInteractive") is not True or state.get("status") != "En vivo" or match is None:
        raise TransportSmokeError("The browser did not expose a real interactive HLS stream")
    value = float(match.group(1))
    if not math.isfinite(value) or value <= 0:
        raise TransportSmokeError("The decoded HLS FPS value is invalid")
    return value


def wait_resource_cleanup(
    docker: DockerProof,
    session_id: uuid.UUID,
    binding: ViewerBinding | None,
    timeout: float,
) -> dict[str, bool]:
    deadline = time.monotonic() + timeout
    last = {
        "containerAbsent": False,
        "networkAbsent": False,
        "viewerRuntimeAbsent": binding is None,
        "viewerPublisherAbsent": False,
    }
    while time.monotonic() < deadline:
        container_absent, network_absent = docker.resources_absent(session_id)
        last = {
            "containerAbsent": container_absent,
            "networkAbsent": network_absent,
            "viewerRuntimeAbsent": binding is None or not binding.directory.exists(),
            "viewerPublisherAbsent": not publisher_for_session_exists(session_id),
        }
        if all(last.values()):
            return last
        time.sleep(0.5)
    return last


def cleanup_run(
    ui: Any,
    docker: DockerProof,
    session_id: uuid.UUID | None,
    binding: ViewerBinding | None,
    timeout: float,
    *,
    viewer_requested: bool,
) -> dict[str, Any]:
    original_event = ui.stop_event
    ui.stop_event = threading.Event()
    result: dict[str, Any] = {
        "viewerClosed": False,
        "sessionReleased": False,
        "workspaceReleased": False,
        "containerAbsent": session_id is None and not ui.create_requested,
        "networkAbsent": session_id is None and not ui.create_requested,
        "viewerRuntimeAbsent": binding is None and not viewer_requested,
        "viewerPublisherAbsent": session_id is None and not ui.create_requested,
        "viewerLeaseInactive": binding is None and not viewer_requested,
        "viewerLeaseRevoked": binding is None and not viewer_requested,
        "viewerCloseCommandTerminal": binding is None and not viewer_requested,
        "complete": False,
    }
    try:
        if session_id is None and ui.create_requested:
            with contextlib.suppress(Exception):
                occupants = ui._occupying_sessions()
                if len(occupants) == 1:
                    session_id = uuid.UUID(str(occupants[0].get("id")))
        if not ui.create_requested and session_id is None:
            result["viewerClosed"] = True
        else:
            try:
                result["viewerClosed"] = close_viewer(ui, min(timeout, 120))
                if session_id is not None and binding is not None:
                    result.update(
                        wait_viewer_lease_closed(
                            ui,
                            session_id,
                            binding,
                            min(timeout, 120),
                        )
                    )
            except Exception as exc:
                result["viewerError"] = str(exc)
        try:
            stopped = ui.stop_created_session(timeout=timeout)
            result["sessionReleased"] = bool(stopped.get("released")) or not stopped.get(
                "requested", True
            )
            if result["sessionReleased"]:
                ui.wait_js(
                    "[...document.querySelectorAll('button')].some(button => "
                    "button.textContent.trim() === 'Crear simulación' && !button.disabled)",
                    min(timeout, 60),
                    "released simulation workspace",
                )
                result["workspaceReleased"] = True
        except Exception as exc:
            result["sessionError"] = str(exc)
        if session_id is not None:
            result.update(wait_resource_cleanup(docker, session_id, binding, timeout))
            if viewer_requested and binding is None:
                result["viewerRuntimeAbsent"] = False
        result["complete"] = all(
            result.get(key) is True
            for key in (
                "viewerClosed",
                "sessionReleased",
                "workspaceReleased",
                "containerAbsent",
                "networkAbsent",
                "viewerRuntimeAbsent",
                "viewerPublisherAbsent",
                "viewerLeaseInactive",
                "viewerLeaseRevoked",
                "viewerCloseCommandTerminal",
            )
        )
        return result
    finally:
        ui.stop_event = original_event


def cleanup_unstarted_profile(chrome: Any) -> dict[str, Any]:
    """Remove only the profile marker created before a failed Popen call."""
    outcome = {
        "processExited": chrome.process is None,
        "portFree": VISIBLE.port_is_free(chrome.port),
        "profileRemoved": False,
    }
    profile = chrome.profile
    if not profile.exists() and not profile.is_symlink():
        outcome["profileRemoved"] = True
        return outcome
    try:
        details = profile.lstat()
        marker = json.loads(chrome.marker.read_text(encoding="utf-8"))
    except (OSError, ValueError):
        return outcome
    owned = (
        chrome.process is None
        and stat.S_ISDIR(details.st_mode)
        and not stat.S_ISLNK(details.st_mode)
        and details.st_uid == os.getuid()
        and marker.get("runId") == chrome.run_id
        and marker.get("label") == chrome.label
        and marker.get("port") == chrome.port
        and marker.get("pid") is None
    )
    if not owned:
        return outcome
    shutil.rmtree(profile)
    outcome["profileRemoved"] = not profile.exists()
    return outcome


def validate_options(args: argparse.Namespace) -> str:
    if not args.execute_production:
        raise TransportSmokeError("Refusing production without --execute-production")
    if SHA_PATTERN.fullmatch(args.deployment_commit) is None:
        raise TransportSmokeError("The deployment commit must be a full lowercase Git SHA")
    if not args.chrome.is_file():
        raise TransportSmokeError("The visible Chrome executable was not found")
    try:
        profile_details = args.profile_root.lstat()
    except OSError as exc:
        raise TransportSmokeError("The Chrome profile root is unavailable") from exc
    if (
        stat.S_ISLNK(profile_details.st_mode)
        or not stat.S_ISDIR(profile_details.st_mode)
        or profile_details.st_uid != os.getuid()
    ):
        raise TransportSmokeError("The Chrome profile root must be a real owned directory")
    if not 1024 <= args.cdp_port <= 65535:
        raise TransportSmokeError("The CDP port must be non-privileged")
    for name in ("ready_timeout", "viewer_timeout", "task_timeout", "cleanup_timeout"):
        value = float(getattr(args, name))
        if not math.isfinite(value) or value <= 0:
            raise TransportSmokeError(f"--{name.replace('_', '-')} must be positive")
    if not math.isfinite(args.video_seconds) or not 3 <= args.video_seconds <= 30:
        raise TransportSmokeError("--video-seconds must be between 3 and 30")
    validate_output_target(args.output)
    return VISIBLE.validate_site(args.url)


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Run the visible production CollaborativeTransport UI smoke"
    )
    parser.add_argument(
        "--execute-production",
        action="store_true",
        required=True,
        help="authorize one production session, viewer, and task",
    )
    parser.add_argument("--deployment-commit", required=True, help="full deployed Git SHA")
    parser.add_argument(
        "--credentials", type=Path, required=True, help="private 0600 account-A credentials"
    )
    parser.add_argument(
        "--chrome", type=Path, required=True, help="normal visible Windows Chrome executable"
    )
    parser.add_argument(
        "--profile-root", type=Path, required=True, help="directory for an ephemeral Chrome profile"
    )
    parser.add_argument("--output", type=Path, required=True, help="sanitized private JSON report")
    parser.add_argument("--url", default=DEFAULT_URL)
    parser.add_argument("--docker", default="docker")
    parser.add_argument("--viewer-runtime-dir", type=Path, default=DEFAULT_VIEWER_RUNTIME)
    parser.add_argument("--cdp-port", type=int, default=9352)
    parser.add_argument("--ready-timeout", type=float, default=480)
    parser.add_argument("--viewer-timeout", type=float, default=240)
    parser.add_argument("--task-timeout", type=float, default=600)
    parser.add_argument("--cleanup-timeout", type=float, default=300)
    parser.add_argument("--video-seconds", type=float, default=5)
    return parser


def parse_args(argv: Sequence[str] | None = None) -> argparse.Namespace:
    return build_parser().parse_args(argv)


def run_smoke(args: argparse.Namespace) -> int:
    expected_origin = validate_options(args)
    credentials = VISIBLE.read_credentials(args.credentials)["A"]
    secrets = (credentials["email"], credentials["password"])
    run_id = dt.datetime.now(dt.timezone.utc).strftime("%Y%m%dT%H%M%SZ") + f"-{os.getpid()}"
    evidence_dir = args.output.parent / f"{args.output.stem}-evidence-{run_id}"
    VISIBLE.validate_secure_directory(evidence_dir)
    profile = args.profile_root / f"robotswarm-transport-ui-{run_id}"

    report: dict[str, Any] = {
        "schemaVersion": SCHEMA_VERSION,
        "startedAt": utc_now(),
        "deploymentCommit": args.deployment_commit,
        "productionExecution": True,
        "account": "A",
        "scenario": {
            "taskType": TASK_TYPE,
            "robotCount": ROBOT_COUNT,
            "freshSessionRequired": True,
            "target": {"x": TARGET_X, "y": TARGET_Y, "planner": PLANNER},
        },
        "browser": {"visible": True, "headless": False, "gpuDisabled": False},
        "session": {"fresh": False, "ready": False},
        "captures": [],
        "cleanup": {"complete": False},
        "success": False,
    }
    write_report(args.output, report, secrets)

    stop_event = threading.Event()
    interrupted = False

    def note_signal(_number: int, _frame: Any) -> None:
        nonlocal interrupted
        interrupted = True
        stop_event.set()

    previous_int = signal.signal(signal.SIGINT, note_signal)
    previous_term = signal.signal(signal.SIGTERM, note_signal)
    chrome = VISIBLE.OwnedChrome(
        "A", args.cdp_port, profile, run_id, args.chrome, args.url
    )
    ui: Any = None
    docker = DockerProof(args.docker, stop_event)
    session_id: uuid.UUID | None = None
    container: ContainerHandle | None = None
    roster: set[str] | None = None
    binding: ViewerBinding | None = None
    observer: TransportStatusObserver | None = None
    viewer_requested = False
    failure: str | None = None
    cleanup_failed = False
    try:
        if not VISIBLE.port_is_free(args.cdp_port):
            raise TransportSmokeError("The requested Chrome debugging port is occupied")
        chrome.launch()
        ui = VISIBLE.RobotSwarmUi(chrome, expected_origin, stop_event)
        ui.navigate(args.url)
        ui.login(credentials["email"], credentials["password"])
        report["browser"]["product"] = chrome.product

        if ui._occupying_sessions():
            raise TransportSmokeError("Account A already owns a session; refusing to reuse it")
        ui.create_session(ROBOT_COUNT)
        ready = ui.wait_ready(ROBOT_COUNT, args.ready_timeout)
        session_id = require_one_session(ui)
        report["session"] = {
            "fresh": True,
            "ready": True,
            "requestedRobots": ROBOT_COUNT,
            "activeRobots": ready.get("activeRobots"),
        }
        container, report["container"] = docker.verify_session(
            session_id, args.deployment_commit
        )
        roster = authoritative_robot_roster(ui, session_id)
        report["session"]["authoritativeRoster"] = {
            "endpointRead": True,
            "exactCount": len(roster),
            "distinctRuntimeIds": len(roster),
        }

        viewer_requested = True
        viewer_deadline = time.monotonic() + args.viewer_timeout
        try:
            ui.request_viewer()
            binding = active_viewer_runtime(
                args.viewer_runtime_dir,
                session_id,
                timeout=remaining_budget(viewer_deadline, "private viewer startup"),
                stop_event=stop_event,
            )
            ui.wait_viewer_frame(
                remaining_budget(viewer_deadline, "private viewer startup")
            )
        except Exception:
            try:
                startup = ui.viewer_startup_state()
            except Exception:
                startup = {"diagnosticUnavailable": True}
            report["viewer"] = {
                "transport": "HLS",
                "live": False,
                "sessionBoundRuntime": binding is not None,
                "startup": startup,
            }
            raise
        viewer = ui.require_interactive_hls()
        report["viewer"] = {
            "transport": "HLS",
            "live": True,
            "interactive": True,
            "decodedFpsBeforeTask": decoded_hls_fps(viewer),
            "sessionBoundRuntime": True,
            **require_active_viewer_lease(ui, session_id, binding),
        }

        baseline = full_task_inventory(ui, session_id)
        if baseline:
            raise TransportSmokeError("The fresh session unexpectedly contains task history")
        ui.select_task_type(TASK_TYPE)
        fields = [
            fill_labeled_number(ui, "Destino X (m)", TARGET_X),
            fill_labeled_number(ui, "Destino Y (m)", TARGET_Y),
        ]
        install_task_watcher(ui, session_id, set())
        install_start_probe(ui, session_id)
        observer = TransportStatusObserver(
            docker,
            container,
            roster,
            args.task_timeout + 120,
        )
        observer.launch()
        barrier = require_start_barrier(ui)

        # This is the only physical start gesture in the complete harness.
        start_gesture = ui.click_button(START_BUTTON, require_trusted=True)
        start = read_start_probe(ui, session_id)
        report["start"] = {
            "selectedCard": TASK_TYPE,
            "fields": fields,
            "barrier": barrier,
            "gesture": start_gesture,
            **start.report,
        }

        accepted = wait_for_task_state(ui, "Accepted", 60)
        task_id = task_from_watch(accepted)["id"]
        if task_id != start.task_id:
            raise TransportSmokeError("The persisted TaskRun differs from the accepted POST")
        report["start"]["privateCorrelation"] = correlate_private_start(
            start,
            task_from_watch(accepted),
            session_id,
            container,
        )
        running = wait_for_task_state(ui, "Running", 60)
        if task_from_watch(running)["id"] != task_id:
            raise TransportSmokeError("The persisted task changed between Accepted and Running")

        search_capture = capture_correlated_phase(
            ui,
            "SEARCH",
            evidence_dir,
            credentials,
            timeout=min(90, args.task_timeout),
        )
        report["captures"].extend(search_capture.report)
        report["viewer"]["runningMeasurement"] = video_evidence(ui, args.video_seconds)
        push_capture = capture_correlated_phase(
            ui,
            "PUSH",
            evidence_dir,
            credentials,
            timeout=args.task_timeout,
        )
        report["captures"].extend(push_capture.report)
        report["viewer"]["taskVisualChange"] = validate_visual_change(
            search_capture,
            push_capture,
        )

        terminal_watch = wait_for_terminal(ui, args.task_timeout)
        terminal = task_from_watch(terminal_watch)
        if terminal.get("id") != task_id:
            raise TransportSmokeError("The terminal task differs from the one-click TaskRun")
        final_task = persisted_terminal_read(ui, session_id, task_id)
        time.sleep(1.0)
        repeated = persisted_terminal_read(ui, session_id, task_id)
        if repeated.get("updatedAt") != final_task.get("updatedAt"):
            raise TransportSmokeError("The terminal task changed after persistence verification")
        report["task"] = validate_transport_task(
            final_task,
            [str(item) for item in terminal_watch["states"]],
            [str(item).upper() for item in terminal_watch["phases"]],
            session_id,
            [*terminal_watch["criticalSamples"], *terminal_watch["samples"]],
            roster,
        )
        report["task"]["persistedReads"] = 2
        terminal_roster = authoritative_robot_roster(ui, session_id)
        if terminal_roster != roster:
            raise TransportSmokeError("The authoritative roster changed during transport")
        report["task"]["authoritativeRosterStable"] = True
        ros_documents = observer.wait_terminal(task_id, 20)
        report["task"]["privateRosObservation"] = validate_ros_transport_observation(
            ros_documents,
            task_id,
            roster,
        )
        if not observer.stop():
            raise TransportSmokeError("The private ROS observer did not stop after DONE")
        ui.show_task_panel()
        ui.wait_js(
            "document.querySelector('[data-testid=\"transport-phase\"]')?.innerText.includes('Entrega completada')",
            10,
            "visible terminal transport result",
        )
        final_capture = ui.screenshot(
            evidence_dir / "transport-done-panel.png",
            credentials["email"],
            credentials["password"],
        )
        report["captures"].append(safe_capture(final_capture, "DONE"))
        write_report(args.output, report, secrets)
    except KeyboardInterrupt:
        interrupted = True
        failure = "Interrupted by the operator"
    except Exception as exc:
        failure = sanitize_text(str(exc), secrets)
    finally:
        if ui is not None:
            stop_task_watcher(ui)
            stop_start_probe(ui)
            observer_exited = observer is None or observer.stop()
            try:
                report["cleanup"] = cleanup_run(
                    ui,
                    docker,
                    session_id,
                    binding,
                    args.cleanup_timeout,
                    viewer_requested=viewer_requested,
                )
            except Exception as exc:
                report["cleanup"] = {
                    "complete": False,
                    "error": sanitize_text(str(exc), secrets),
                }
            observer_exited = finish_observer_cleanup(
                observer,
                observer_exited,
                report["cleanup"],
            )
            report["cleanup"]["rosObserverExited"] = observer_exited
            report["cleanup"]["complete"] = bool(
                report["cleanup"].get("complete") and observer_exited
            )
            cleanup_failed = report["cleanup"].get("complete") is not True
        else:
            report["cleanup"] = {
                "complete": chrome.process is None,
                "sessionNotCreated": True,
            }

        if chrome.process is None:
            browser_cleanup = cleanup_unstarted_profile(chrome)
        else:
            try:
                browser_cleanup = chrome.close_owned()
            except Exception as exc:
                browser_cleanup = {"error": sanitize_text(str(exc), secrets)}
        report["cleanup"].update(
            {
                "browserExited": browser_cleanup.get("processExited") is True,
                "cdpPortReleased": browser_cleanup.get("portFree") is True,
                "profileRemoved": browser_cleanup.get("profileRemoved") is True,
            }
        )
        report["cleanup"]["complete"] = bool(
            report["cleanup"].get("complete")
            and report["cleanup"]["browserExited"]
            and report["cleanup"]["cdpPortReleased"]
            and report["cleanup"]["profileRemoved"]
        )
        cleanup_failed = report["cleanup"]["complete"] is not True
        report["completedAt"] = utc_now()
        report["interrupted"] = interrupted
        if failure:
            report["failure"] = failure
        report["success"] = bool(
            not interrupted
            and failure is None
            and report.get("task", {}).get("done") is True
            and report["cleanup"]["complete"]
        )
        write_report(args.output, report, secrets)
        signal.signal(signal.SIGINT, previous_int)
        signal.signal(signal.SIGTERM, previous_term)

    print(
        f"Sanitized transport UI report: {sanitize_text(str(args.output), secrets)}",
        flush=True,
    )
    if interrupted:
        return 130
    if cleanup_failed:
        return 3
    return 0 if report["success"] else 1


def main(argv: Sequence[str] | None = None) -> int:
    args = parse_args(argv)
    try:
        with HostRunLock():
            return run_smoke(args)
    except Exception as exc:
        print(
            f"Transport UI smoke preflight failed: {sanitize_text(str(exc))}",
            file=sys.stderr,
        )
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
