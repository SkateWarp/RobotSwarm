#!/usr/bin/env python3
"""Run the visible, loaded-payload N=4 production acceptance gate.

The normal ROS matrix proves the algorithms with the practice payload.  This
operator harness adds the physical-load gate without sharing a historical
session: account A creates one N=4 session through the public site, opens its
real HLS viewer, starts the official loaded-payload probe, and runs the deployed
GUI preflight against that viewer's private display while the heavy payload is
present in the same ROS/Gazebo session.

Private control-plane, Docker and viewer identifiers are used only in memory.
The retained report contains sanitized protocol documents and hashes, never
credentials, UUIDs, container IDs, network IDs or lease IDs.
"""

from __future__ import annotations

import argparse
import contextlib
import dataclasses
import datetime as dt
import hashlib
import importlib.util
import ipaddress
import json
import math
import os
import queue
import re
import shutil
import signal
import stat
import subprocess
import sys
import threading
import time
import uuid
from pathlib import Path
from types import ModuleType
from typing import Any, Iterable, Sequence
from urllib.parse import urlsplit


SCRIPT_DIR = Path(__file__).resolve().parent
MATRIX_DRIVER_PATH = SCRIPT_DIR / "robotswarm-ros-matrix-e2e.py"
DEFAULT_URL = "https://rs.zerav.la/apps/GTS/realtime"
DEPLOYED_PREFLIGHT_PATH = (
    "/catkin_ws/src/robot_swarm_bridge/scripts/gazebo_gui_preflight.py"
)
DEPLOYED_GUI_PLUGIN_PATH = (
    "/catkin_ws/devel/lib/librobotswarm_gazebo_gui_probe.so"
)
DEPLOYED_LOAD_PROBE_PATH = (
    "/catkin_ws/src/robot_swarm_bridge/test/robotswarm_payload_load_live.py"
)
MINIMUM_RENDER_FPS = 45.0
MINIMUM_REAL_TIME_FACTOR = 2.90
PRACTICE_PAYLOAD_MASS_KG = 0.25
LOADED_PAYLOAD_MASS_KG = 0.75
EXPECTED_ROBOT_COUNT = 4
MAXIMUM_PROTOCOL_OUTPUT_BYTES = 16 * 1024 * 1024
MAXIMUM_PREFLIGHT_OUTPUT_BYTES = 1024 * 1024
MAXIMUM_PREFLIGHT_SCRIPT_BYTES = 256 * 1024
MAXIMUM_GUI_PLUGIN_BYTES = 32 * 1024 * 1024
MAXIMUM_MASS_SAMPLE_GAP_SECONDS = 1.0
MAXIMUM_STATUS_SAMPLE_GAP_SECONDS = 1.0
MAXIMUM_LIVE_MASS_AGE_SECONDS = 0.75
LOADED_READY_TIMEOUT_SECONDS = 120.0
PREFLIGHT_WARMUP_SECONDS = 2.0
PREFLIGHT_SAMPLE_SECONDS = 5.0
LOADED_PREFLIGHT_HLS_SECONDS = 5.0
PREFLIGHT_SHUTDOWN_MARGIN_SECONDS = 5.0

CAPACITY_SINGLE_MAX_PROGRESS_M = 0.05
CAPACITY_ROOT_MAX_PROGRESS_M = 0.06
CAPACITY_FLEET_MIN_PROGRESS_M = 0.20
CAPACITY_ROBOT_MIN_PROGRESS_M = 0.05
CAPACITY_MINIMUM_GAIN = 4.0
CAPACITY_COMMAND_SPEED_MPS = 0.16
CAPACITY_PUSH_DURATION_SIM_S = 12.0
CAPACITY_PUSH_DURATION_TOLERANCE_SIM_S = 0.25
CAPACITY_RTF_MATCH_TOLERANCE = 0.01
CAPACITY_PAYLOAD_ROOT_MAX_CONTACT_DISTANCE_M = 0.12
CAPACITY_COMPANION_MAX_CONTACT_DISTANCE_M = 0.17
GRF_RTF_MATCH_TOLERANCE = 0.01
TASK_PROGRESS_REGRESSION_TOLERANCE = 0.01

GRF_MINIMUM_PUSH_DURATION_S = 0.75
GRF_MINIMUM_PUSH_SAMPLES = 5
GRF_MINIMUM_USEFUL_FRACTION = 0.50
GRF_MINIMUM_GOAL_EFFICIENCY = 0.50
GRF_CONFIGURED_GOAL_PROGRESS_M = 0.55
GRF_MINIMUM_GOAL_PROGRESS_M = 0.50
GRF_GOAL_ARRIVAL_TOLERANCE_M = 0.50
GRF_GOAL_PROGRESS_CONTRACT_EPSILON_M = 0.001
GRF_GOAL_PROGRESS_CONTRACT_BASIS = (
    "distance at synchronized PUSH start minus the controller arrival tolerance"
)
GRF_MINIMUM_SEARCH_TRAVEL_M = 0.05
GRF_MINIMUM_RENDEZVOUS_TRAVEL_M = 0.10
GRF_MINIMUM_ROOT_RENDEZVOUS_TRAVEL_M = 0.03
GRF_DIRECT_PAYLOAD_CLEARANCE_M = 0.075
GRF_COMPANION_CENTER_DISTANCE_M = 0.16
GRF_MINIMUM_CHAIN_CENTER_DISTANCE_M = 0.12
GRF_COMMAND_MINIMUM_SPEED_MPS = 0.015
GRF_COMMAND_MINIMUM_GOAL_COSINE = 0.50
GRF_COMMAND_MAXIMUM_AGE_S = 0.75
GRF_CONTRIBUTION_NOISE_FLOOR_MPS = 0.003
GRF_CONTRIBUTION_SPEED_TOLERANCE_MPS = 0.003
GRF_CONTRIBUTION_TRACKING_FRACTION = 0.75
GRF_CONTROL_GAP_TOLERANCE_S = 1.0

PAYLOAD_READY_PREFIX = "LOADED_PAYLOAD_READY_JSON "
PAYLOAD_MASS_PREFIX = "LOADED_MASS_SAMPLE_JSON "
GRF_ACTIVE_PREFIX = "LOADED_GRF_ACTIVE_JSON "
PUSH_LIVE_PREFIX = "LOADED_PUSH_LIVE_JSON "

SHA_PATTERN = re.compile(r"[0-9a-f]{40}")
CONTAINER_ID_PATTERN = re.compile(r"\b[0-9a-f]{64}\b", re.IGNORECASE)
EXPECTED_ROBOTS = tuple(f"tb3_{index}" for index in range(EXPECTED_ROBOT_COUNT))
EXPECTED_ROBOT_SET = frozenset(EXPECTED_ROBOTS)
ACTIVE_GRF_PHASES = frozenset({"SEARCH", "APPROACH", "PUSH"})


class LoadedGateError(RuntimeError):
    """An expected acceptance failure safe to show in a sanitized report."""


class LoadedCleanupError(LoadedGateError):
    """The harness could not prove removal of everything it allocated."""


class LoadedProbeEnded(LoadedGateError):
    """The official payload probe exited before a required live marker."""

    def __init__(self, description: str, output: Any) -> None:
        super().__init__(
            f"The loaded probe ended before {description} "
            f"(status {output.returncode})"
        )
        self.output = output


class MalformedLiveMarker(LoadedGateError):
    """Keep structural evidence for a bad live line without retaining its body."""

    def __init__(self, prefix: str, payload: str) -> None:
        super().__init__(f"The {prefix.strip()} marker is malformed")
        encoded = payload.encode("utf-8", errors="replace")
        stripped = payload.strip()
        self.diagnostic = {
            "marker": prefix.strip(),
            "bytes": len(encoded),
            "sha256": hashlib.sha256(encoded).hexdigest(),
            "startsWithObject": stripped.startswith("{"),
            "endsWithObject": stripped.endswith("}"),
            "embeddedKnownPrefixes": sorted(
                candidate.strip()
                for candidate in (
                    PAYLOAD_READY_PREFIX,
                    PAYLOAD_MASS_PREFIX,
                    GRF_ACTIVE_PREFIX,
                    PUSH_LIVE_PREFIX,
                )
                if candidate in payload
            ),
            "rawRetained": False,
        }


@dataclasses.dataclass(frozen=True)
class MarkerDocument:
    document: dict[str, Any]
    source_sha256: str
    line_number: int


@dataclasses.dataclass(frozen=True)
class TimedMarker:
    document: dict[str, Any]
    observed_at: float
    source_sha256: str


@dataclasses.dataclass(frozen=True)
class PushCapture:
    before: TimedMarker
    after: TimedMarker
    screenshot_started_at: float
    screenshot_finished_at: float
    decoded_fps_before: float
    decoded_fps_after: float


@dataclasses.dataclass(frozen=True)
class LoadedProtocol:
    load: MarkerDocument
    grf_result: MarkerDocument
    grf_summary: MarkerDocument
    post_cleanup: MarkerDocument
    returncode: int


@dataclasses.dataclass(frozen=True)
class ViewerRuntime:
    command_prefix: tuple[str, ...]
    environment: dict[str, str]
    gzclient: str
    publisher_render: Any
    binding: dict[str, bool]


def process_identity(pid: int) -> int | None:
    try:
        raw = (Path("/proc") / str(pid) / "stat").read_text(encoding="ascii")
        fields = raw[raw.rfind(")") + 2 :].split()
        return int(fields[19])
    except (IndexError, OSError, ValueError):
        return None


def living_process_group_members(process_group: int) -> list[int]:
    members: list[int] = []
    try:
        entries = list(Path("/proc").iterdir())
    except OSError:
        entries = []
    for entry in entries:
        if not entry.name.isdigit():
            continue
        try:
            raw = (entry / "stat").read_text(encoding="ascii")
            fields = raw[raw.rfind(")") + 2 :].split()
            state = fields[0]
            group_id = int(fields[2])
            session_id = int(fields[3])
        except (IndexError, OSError, ValueError):
            continue
        if (
            group_id == process_group
            and session_id == process_group
            and state != "Z"
        ):
            members.append(int(entry.name))
    return members


class BoundedChild:
    """Own one process group and drain both pipes without unbounded buffering."""

    def __init__(
        self,
        arguments: Sequence[str],
        *,
        maximum_output_bytes: int,
        environment: dict[str, str] | None = None,
        cwd: Path | None = None,
        umask: int | None = None,
        line_prefixes: Sequence[str] = (),
        line_streams: Sequence[str] = ("stdout",),
        strict_line_channel: bool = False,
    ) -> None:
        if maximum_output_bytes <= 0:
            raise ValueError("maximum_output_bytes must be positive")
        if umask is not None and not 0 <= umask <= 0o777:
            raise ValueError("umask must be an octal permission mask")
        if (
            not line_streams
            or len(set(line_streams)) != len(line_streams)
            or set(line_streams) - {"stdout", "stderr"}
        ):
            raise ValueError("line_streams must name unique stdout/stderr streams")
        if strict_line_channel and not line_prefixes:
            raise ValueError("a strict line channel requires approved prefixes")
        self.arguments = list(arguments)
        self.maximum_output_bytes = maximum_output_bytes
        self.environment = environment
        self.cwd = cwd
        self.umask = umask
        self.line_prefixes = tuple(item.encode("utf-8") for item in line_prefixes)
        self.line_streams = frozenset(line_streams)
        self.strict_line_channel = strict_line_channel
        self.process: subprocess.Popen[bytes] | None = None
        self.process_group: int | None = None
        self.process_identity: int | None = None
        self.started_at: float | None = None
        self.ended_at: float | None = None
        self.returncode: int | None = None
        self.done = threading.Event()
        self.overflow = threading.Event()
        self.line_events: queue.Queue[tuple[float, str]] = queue.Queue()
        self._lock = threading.Lock()
        self._kill_lock = threading.Lock()
        self._stdout = bytearray()
        self._stderr = bytearray()
        self._total = 0
        self._reader_errors: list[str] = []
        self._readers: list[threading.Thread] = []
        self._watcher: threading.Thread | None = None

    def start(self) -> "BoundedChild":
        if self.process is not None:
            raise LoadedGateError("A bounded child cannot be started twice")
        options: dict[str, Any] = {
            "stdin": subprocess.DEVNULL,
            "stdout": subprocess.PIPE,
            "stderr": subprocess.PIPE,
            "text": False,
            "env": self.environment,
            "cwd": str(self.cwd) if self.cwd is not None else None,
            "start_new_session": True,
        }
        launch_arguments = self.arguments
        if self.umask is not None:
            # Python 3.8 has no Popen(umask=...).  The fixed shell program
            # receives every real argument separately and immediately execs it,
            # so no command text from the child is evaluated by the shell.
            launch_arguments = [
                "/bin/sh",
                "-c",
                'umask "$1"; shift; exec "$@"',
                "robotswarm-private-child",
                f"{self.umask:03o}",
                *self.arguments,
            ]
        self.process = subprocess.Popen(launch_arguments, **options)
        self.process_group = self.process.pid
        try:
            observed_group = os.getpgid(self.process.pid)
        except ProcessLookupError:
            observed_group = self.process.pid if self.process.poll() is not None else None
        if observed_group != self.process_group or observed_group == os.getpgrp():
            if self.process.poll() is None:
                self.process.kill()
                self.process.wait(timeout=5.0)
            raise LoadedGateError("A bounded child did not get a private process group")
        self.process_identity = process_identity(self.process.pid)
        if self.process_identity is None:
            if self.process.poll() is None:
                self.process.kill()
                self.process.wait(timeout=5.0)
                raise LoadedGateError("A bounded child identity could not be captured")
            self.process_identity = -1
        self.started_at = time.monotonic()
        if self.process.stdout is None or self.process.stderr is None:
            self.kill_now()
            raise LoadedGateError("A bounded child did not expose both output pipes")
        for name, stream in (
            ("stdout", self.process.stdout),
            ("stderr", self.process.stderr),
        ):
            reader = threading.Thread(
                target=self._drain,
                args=(name, stream),
                name=f"robotswarm-loaded-{name}",
                daemon=True,
            )
            self._readers.append(reader)
            reader.start()
        self._watcher = threading.Thread(
            target=self._watch,
            name="robotswarm-loaded-process-watch",
            daemon=True,
        )
        self._watcher.start()
        return self

    def _watch(self) -> None:
        assert self.process is not None
        self.returncode = self.process.wait()
        self.ended_at = time.monotonic()
        self.done.set()

    def _accept(self, name: str, chunk: bytes) -> bytes:
        overflow_now = False
        with self._lock:
            remaining = max(0, self.maximum_output_bytes - self._total)
            accepted = chunk[:remaining]
            target = self._stdout if name == "stdout" else self._stderr
            target.extend(accepted)
            self._total += len(accepted)
            if len(accepted) != len(chunk) and not self.overflow.is_set():
                self.overflow.set()
                overflow_now = True
        if overflow_now:
            self.kill_now()
        return accepted

    def _drain(self, name: str, stream: Any) -> None:
        pending = bytearray()
        try:
            while True:
                chunk = os.read(stream.fileno(), 64 * 1024)
                if not chunk:
                    break
                accepted = self._accept(name, chunk)
                if name in self.line_streams and self.line_prefixes and accepted:
                    pending.extend(accepted)
                    while b"\n" in pending:
                        raw_line, _, remainder = pending.partition(b"\n")
                        pending = bytearray(remainder)
                        if any(raw_line.startswith(prefix) for prefix in self.line_prefixes):
                            self.line_events.put(
                                (
                                    time.monotonic(),
                                    raw_line.decode("utf-8", errors="replace"),
                                )
                            )
                        elif self.strict_line_channel:
                            with self._lock:
                                self._reader_errors.append(
                                    "dedicated line channel contained unapproved data"
                                )
                            self.kill_now()
                            return
        except (OSError, ValueError) as exc:
            if not self.done.is_set():
                with self._lock:
                    self._reader_errors.append(type(exc).__name__)
                self.kill_now()
        finally:
            if pending and self.strict_line_channel:
                with self._lock:
                    self._reader_errors.append(
                        "dedicated line channel ended with an incomplete line"
                    )
                if not self.done.is_set():
                    self.kill_now()
            with contextlib.suppress(OSError):
                stream.close()

    def _signal_group(self, number: int) -> None:
        process = self.process
        process_group = self.process_group
        identity = self.process_identity
        if process is None or process_group is None or identity is None:
            return
        if process_group <= 1 or process_group == os.getpgrp():
            raise LoadedCleanupError("Refusing to signal an unsafe bounded child group")
        if process.poll() is None:
            current_identity = process_identity(process.pid)
            try:
                current_group = os.getpgid(process.pid)
            except ProcessLookupError:
                current_group = None
            if (
                current_identity != identity or current_group != process_group
            ) and process.poll() is None:
                raise LoadedCleanupError(
                    "A bounded child PID or process group was reused during cleanup"
                )
        with contextlib.suppress(ProcessLookupError):
            os.killpg(process_group, number)

    def _wait_group_gone(self, timeout: float) -> bool:
        process_group = self.process_group
        if process_group is None:
            return self.done.is_set()
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            if self.done.is_set() and not living_process_group_members(process_group):
                return True
            time.sleep(0.02)
        return self.done.is_set() and not living_process_group_members(process_group)

    def _remove_orphaned_descendants(self) -> bool:
        process_group = self.process_group
        if (
            process_group is None
            or not self.done.is_set()
            or not living_process_group_members(process_group)
        ):
            return False
        self.kill_now()
        if not self._wait_group_gone(5.0):
            raise LoadedCleanupError(
                "A bounded child process group could not be removed"
            )
        return True

    def kill_now(self) -> None:
        """Kill the owned group immediately; used for output overflow."""
        with self._kill_lock:
            self._signal_group(signal.SIGKILL)

    def stop_gracefully(self) -> None:
        if self.done.is_set():
            if self._remove_orphaned_descendants():
                raise LoadedCleanupError("A bounded child left a live descendant")
            return
        self._signal_group(signal.SIGINT)
        if self._wait_group_gone(25.0):
            return
        self._signal_group(signal.SIGTERM)
        if self._wait_group_gone(10.0):
            return
        self.kill_now()
        if not self._wait_group_gone(5.0):
            raise LoadedCleanupError("A bounded child process could not be stopped")

    def _join_readers(self) -> None:
        for reader in self._readers:
            reader.join(timeout=5.0)
        if any(reader.is_alive() for reader in self._readers):
            self.kill_now()
            if self.process is not None:
                for stream in (self.process.stdout, self.process.stderr):
                    if stream is not None:
                        with contextlib.suppress(OSError):
                            stream.close()
            for reader in self._readers:
                reader.join(timeout=1.0)
        if any(reader.is_alive() for reader in self._readers):
            with self._lock:
                self._reader_errors.append("pipe did not close")

    def wait(
        self,
        *,
        timeout: float,
        stop_event: threading.Event | None,
    ) -> Any:
        if self.process is None or self.started_at is None:
            raise LoadedGateError("A bounded child was not started")
        deadline = self.started_at + timeout
        interrupted = False
        timed_out = False
        while not self.done.wait(0.05):
            if self.overflow.is_set():
                self.kill_now()
                continue
            if stop_event is not None and stop_event.is_set():
                interrupted = True
                self.stop_gracefully()
                break
            if time.monotonic() >= deadline:
                timed_out = True
                self.stop_gracefully()
                break
        if not self.done.is_set():
            self.kill_now()
            self.done.wait(5.0)
        orphaned = self._remove_orphaned_descendants()
        self._join_readers()
        if interrupted:
            raise KeyboardInterrupt
        if timed_out:
            raise LoadedGateError("A bounded child process timed out")
        if orphaned:
            raise LoadedCleanupError("A bounded child left a live descendant")
        return self.output()

    def output(self) -> Any:
        if not self.done.is_set():
            raise LoadedGateError("A bounded child is still running")
        orphaned = self._remove_orphaned_descendants()
        self._join_readers()
        if orphaned:
            raise LoadedCleanupError("A bounded child left a live descendant")
        if self.overflow.is_set():
            raise LoadedGateError("A child process exceeded the bounded output limit")
        if self._reader_errors:
            raise LoadedGateError("A child output pipe could not be drained safely")
        with self._lock:
            stdout = bytes(self._stdout).decode("utf-8", errors="replace")
            stderr = bytes(self._stderr).decode("utf-8", errors="replace")
        return MATRIX.ProcessOutput(int(self.returncode or 0), stdout, stderr)

    def pop_lines(self) -> list[tuple[float, str]]:
        values: list[tuple[float, str]] = []
        while True:
            try:
                values.append(self.line_events.get_nowait())
            except queue.Empty:
                return values


class LiveMarkerCollector:
    """Validate the small structured stream emitted beside the official probe."""

    def __init__(self) -> None:
        self.ready: list[TimedMarker] = []
        self.mass: list[TimedMarker] = []
        self.active: list[TimedMarker] = []
        self.push: list[TimedMarker] = []

    @staticmethod
    def _decode(observed_at: float, line: str, prefix: str) -> TimedMarker:
        payload = line[len(prefix) :]
        try:
            document = MATRIX.strict_json_loads(payload)
        except (json.JSONDecodeError, ValueError) as exc:
            raise MalformedLiveMarker(prefix, payload) from exc
        if not isinstance(document, dict):
            raise LoadedGateError(f"The {prefix.strip()} marker is not an object")
        return TimedMarker(
            document=document,
            observed_at=observed_at,
            source_sha256=hashlib.sha256(payload.encode("utf-8")).hexdigest(),
        )

    @staticmethod
    def _sequence(document: dict[str, Any], description: str) -> int:
        value = document.get("sequence")
        if isinstance(value, bool) or not isinstance(value, int) or value < 1:
            raise LoadedGateError(f"The {description} sequence is invalid")
        return value

    def ingest(self, child: BoundedChild) -> None:
        for observed_at, line in child.pop_lines():
            if line.startswith(PAYLOAD_READY_PREFIX):
                marker = self._decode(observed_at, line, PAYLOAD_READY_PREFIX)
                document = marker.document
                if (
                    document.get("schemaVersion") != 1
                    or document.get("profile") != "transport_crate_loaded"
                    or not math.isclose(
                        finite_number(document.get("payloadMassKg"), "ready mass"),
                        LOADED_PAYLOAD_MASS_KG,
                        abs_tol=1e-9,
                    )
                ):
                    raise LoadedGateError("The loaded-payload ready marker is invalid")
                self.ready.append(marker)
                if len(self.ready) != 1:
                    raise LoadedGateError("The probe emitted more than one loaded ready marker")
                continue

            if line.startswith(PAYLOAD_MASS_PREFIX):
                marker = self._decode(observed_at, line, PAYLOAD_MASS_PREFIX)
                document = marker.document
                if document.get("schemaVersion") != 1:
                    raise LoadedGateError("A live payload-mass marker has an invalid schema")
                sequence = self._sequence(document, "payload-mass")
                if self.mass and sequence <= self._sequence(
                    self.mass[-1].document, "payload-mass"
                ):
                    raise LoadedGateError("Live payload-mass markers are out of order")
                finite_number(document.get("payloadMassKg"), "live payload mass")
                self.mass.append(marker)
                continue

            if line.startswith(GRF_ACTIVE_PREFIX):
                marker = self._decode(observed_at, line, GRF_ACTIVE_PREFIX)
                document = marker.document
                task_id = str(document.get("taskId") or "")
                roster = document.get("roster")
                models = document.get("robotModels")
                progress = finite_number(document.get("taskProgress"), "GRF progress")
                mass_age = finite_number(
                    document.get("massSampleAgeSeconds"), "GRF mass-sample age"
                )
                self._sequence(document, "active GRF")
                self._sequence(
                    {"sequence": document.get("massSequence")}, "active GRF mass"
                )
                if (
                    document.get("schemaVersion") != 1
                    or re.fullmatch(r"loaded-grf-n4-[0-9a-f]{8}", task_id) is None
                    or document.get("taskStatus") != "running"
                    or document.get("phase") not in ACTIVE_GRF_PHASES
                    or document.get("robotCount") != EXPECTED_ROBOT_COUNT
                    or roster != list(EXPECTED_ROBOTS)
                    or models != list(EXPECTED_ROBOTS)
                    or not math.isclose(
                        finite_number(
                            document.get("payloadMassKg"), "active GRF payload mass"
                        ),
                        LOADED_PAYLOAD_MASS_KG,
                        abs_tol=1e-6,
                    )
                    or mass_age < 0.0
                    or mass_age > MAXIMUM_LIVE_MASS_AGE_SECONDS
                    or progress < 0.0
                    or progress > 1.0
                ):
                    raise LoadedGateError("An active loaded-GRF marker is invalid")
                if self.active and self._sequence(
                    document, "active GRF"
                ) <= self._sequence(self.active[-1].document, "active GRF"):
                    raise LoadedGateError("Active loaded-GRF markers are out of order")
                self.active.append(marker)
                continue

            if line.startswith(PUSH_LIVE_PREFIX):
                marker = self._decode(observed_at, line, PUSH_LIVE_PREFIX)
                document = marker.document
                contributor_ids = document.get("usefulContributorIds")
                task_id = str(document.get("taskId") or "")
                roster = document.get("roster")
                models = document.get("robotModels")
                mass_age = finite_number(
                    document.get("massSampleAgeSeconds"), "live PUSH mass-sample age"
                )
                progress = finite_number(
                    document.get("taskProgress"), "live PUSH task progress"
                )
                self._sequence(
                    {"sequence": document.get("massSequence")}, "live PUSH mass"
                )
                if (
                    document.get("schemaVersion") != 1
                    or document.get("taskStatus") != "running"
                    or document.get("phase") != "PUSH"
                    or document.get("robotCount") != EXPECTED_ROBOT_COUNT
                    or roster != list(EXPECTED_ROBOTS)
                    or models != list(EXPECTED_ROBOTS)
                    or document.get("allPushersConfirmed") is not True
                    or document.get("usefulContributorCount") != EXPECTED_ROBOT_COUNT
                    or not isinstance(contributor_ids, list)
                    or not all(isinstance(name, str) for name in contributor_ids)
                    or set(contributor_ids) != EXPECTED_ROBOT_SET
                    or len(contributor_ids) != EXPECTED_ROBOT_COUNT
                    or re.fullmatch(r"loaded-grf-n4-[0-9a-f]{8}", task_id) is None
                    or not math.isclose(
                        finite_number(
                            document.get("payloadMassKg"), "live PUSH payload mass"
                        ),
                        LOADED_PAYLOAD_MASS_KG,
                        abs_tol=1e-6,
                    )
                    or mass_age < 0.0
                    or mass_age > MAXIMUM_LIVE_MASS_AGE_SECONDS
                    or progress < 0.0
                    or progress > 1.0
                ):
                    raise LoadedGateError("A live all-fleet PUSH marker is invalid")
                sequence = self._sequence(document, "live PUSH")
                if self.push and sequence <= self._sequence(
                    self.push[-1].document, "live PUSH"
                ):
                    raise LoadedGateError("Live PUSH markers are out of order")
                self.push.append(marker)


def wait_for_live_marker(
    child: BoundedChild,
    markers: LiveMarkerCollector,
    predicate: Any,
    *,
    timeout: float,
    stop_event: threading.Event,
    description: str,
) -> Any:
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        markers.ingest(child)
        found = predicate(markers)
        if found is not None:
            return found
        if stop_event.is_set():
            with contextlib.suppress(Exception):
                child.stop_gracefully()
            raise KeyboardInterrupt
        if child.done.is_set():
            output = child.output()
            raise LoadedProbeEnded(description, output)
        time.sleep(0.05)
    with contextlib.suppress(Exception):
        child.stop_gracefully()
    raise LoadedGateError(f"Timed out waiting for {description}")


def load_matrix_driver() -> ModuleType:
    module_name = "robotswarm_ros_matrix_for_loaded_n4"
    existing = sys.modules.get(module_name)
    if existing is not None:
        return existing
    spec = importlib.util.spec_from_file_location(module_name, MATRIX_DRIVER_PATH)
    if spec is None or spec.loader is None:
        raise LoadedGateError("The ROS matrix support module could not be loaded")
    module = importlib.util.module_from_spec(spec)
    sys.modules[module_name] = module
    try:
        spec.loader.exec_module(module)
    except SystemExit as exc:
        sys.modules.pop(module_name, None)
        raise LoadedGateError("The ROS matrix dependencies are unavailable") from exc
    return module


MATRIX = load_matrix_driver()


def classify_loaded_probe_failure(output: Any) -> dict[str, Any]:
    protocol_lines = output.stdout.splitlines() if isinstance(output.stdout, str) else []
    marker_lines = output.stderr.splitlines() if isinstance(output.stderr, str) else []
    load_documents: list[dict[str, Any]] = []
    for line in protocol_lines:
        if not line.startswith("LOAD_RESULT_JSON "):
            continue
        try:
            document = MATRIX.strict_json_loads(
                line[len("LOAD_RESULT_JSON ") :]
            )
        except (json.JSONDecodeError, ValueError):
            continue
        if isinstance(document, dict):
            load_documents.append(document)

    categories: set[str] = set()
    if output.returncode == 90:
        categories.add("bootstrap_environment_failed")
    elif output.returncode == 91:
        categories.add("postcheck_failed")
    elif output.returncode == 92:
        categories.add("live_marker_supervision_failed")
    elif output.returncode != 0:
        categories.add("official_probe_failed")

    failure_count = 0
    if len(load_documents) == 1:
        failures = load_documents[0].get("failures")
        if isinstance(failures, list):
            failure_count = len(failures)
            for failure in failures:
                normalized = str(failure).lower()
                if "cleanup failed" in normalized:
                    categories.add("payload_cleanup_failed")
                if "loaded transport_grf" in normalized:
                    categories.add("loaded_grf_failed")
                if "serviceexception" in normalized or "service exception" in normalized:
                    categories.add("ros_service_failed")
                if any(
                    marker in normalized
                    for marker in (
                        "replace_payload",
                        "transport payload",
                        "transport_object",
                        "transport object",
                        "spawn model",
                        "delete model",
                    )
                ):
                    categories.add("payload_replace_or_visibility_failed")
                if "runtimeerror" in normalized or "runtime error" in normalized:
                    categories.add("runtime_failure")
                specific_failures = (
                    ("timeout waiting for empty fleet", "fleet_delete_timeout"),
                    ("old gazebo robot deletion", "gazebo_robot_delete_timeout"),
                    ("robot roster", "fleet_spawn_roster_timeout"),
                    ("gazebo robot models", "gazebo_robot_spawn_timeout"),
                    ("burger cmd_vel subscribers", "robot_command_subscriber_timeout"),
                    ("could not place", "model_placement_failed"),
                    ("gazebo telemetry disappeared", "gazebo_telemetry_lost"),
                    ("gazebo clock is unavailable", "gazebo_clock_unavailable"),
                    ("could not delete payload", "payload_delete_failed"),
                    ("could not spawn payload", "payload_spawn_failed"),
                    ("payload deletion", "payload_delete_timeout"),
                    ("payload spawn", "payload_spawn_timeout"),
                    ("swarm task status", "swarm_status_timeout"),
                    ("commands subscriber", "swarm_command_subscriber_timeout"),
                    ("gazebo telemetry", "gazebo_telemetry_timeout"),
                )
                categories.update(
                    category
                    for marker, category in specific_failures
                    if marker in normalized
                )
        if load_documents[0].get("passed") is not False:
            categories.add("load_result_status_inconsistent")
    elif not load_documents:
        categories.add("official_probe_result_missing")
    else:
        categories.add("official_probe_result_ambiguous")

    loaded_mass_count = 0
    practice_mass_count = 0
    for line in marker_lines:
        if not line.startswith(PAYLOAD_MASS_PREFIX):
            continue
        try:
            mass_document = MATRIX.strict_json_loads(
                line[len(PAYLOAD_MASS_PREFIX) :]
            )
            mass = float(mass_document.get("payloadMassKg"))
        except (AttributeError, TypeError, ValueError, json.JSONDecodeError):
            continue
        if math.isclose(mass, LOADED_PAYLOAD_MASS_KG, abs_tol=1e-6):
            loaded_mass_count += 1
        elif math.isclose(mass, PRACTICE_PAYLOAD_MASS_KG, abs_tol=1e-6):
            practice_mass_count += 1
    marker_counts = {
        "ready": sum(line.startswith(PAYLOAD_READY_PREFIX) for line in marker_lines),
        "mass": sum(line.startswith(PAYLOAD_MASS_PREFIX) for line in marker_lines),
        "loadedMass": loaded_mass_count,
        "practiceMass": practice_mass_count,
        "activeGrf": sum(
            line.startswith(GRF_ACTIVE_PREFIX) for line in marker_lines
        ),
        "livePush": sum(line.startswith(PUSH_LIVE_PREFIX) for line in marker_lines),
    }
    return {
        "returnCode": int(output.returncode),
        "categories": sorted(categories),
        "structuredLoadResultCount": len(load_documents),
        "structuredFailureCount": failure_count,
        "liveMarkerCounts": marker_counts,
        "rawDiagnosticRetained": False,
    }


class BoundedDockerHost(MATRIX.DockerHost):
    """Use this gate's hard output cap for every Docker helper call."""

    def run(
        self,
        arguments: Sequence[str],
        *,
        timeout: float = 30,
        interruptible: bool = True,
    ) -> Any:
        child = BoundedChild(
            [self.executable, *arguments],
            maximum_output_bytes=MAXIMUM_PROTOCOL_OUTPUT_BYTES,
        ).start()
        return child.wait(
            timeout=timeout,
            stop_event=self.stop_event if interruptible else None,
        )


def utc_now() -> str:
    return dt.datetime.now(dt.timezone.utc).isoformat(timespec="seconds").replace(
        "+00:00", "Z"
    )


def clean_failure(value: Any, secrets: Iterable[str] = ()) -> str:
    clean = MATRIX.redact_text(str(value), secrets)
    return CONTAINER_ID_PATTERN.sub("[CONTAINER ID REDACTED]", clean)


def finite_number(value: Any, description: str) -> float:
    try:
        return MATRIX.finite_number(value, description)
    except Exception as exc:
        raise LoadedGateError(str(exc)) from exc


def exact_id_list(
    value: Any,
    expected: set[str] | frozenset[str],
    description: str,
) -> list[str]:
    """Return an exact, duplicate-free identifier list."""
    if (
        not isinstance(value, list)
        or any(not isinstance(item, str) for item in value)
        or len(value) != len(set(value))
        or set(value) != set(expected)
    ):
        raise LoadedGateError(f"The {description} is not an exact identifier list")
    return value


def ordered_marker_stream(
    markers: Sequence[TimedMarker], description: str
) -> list[TimedMarker]:
    """Order a marker stream and reject repeated or backwards sequences."""
    ordered = sorted(markers, key=lambda marker: marker.observed_at)
    previous_sequence = 0
    for marker in ordered:
        if not math.isfinite(marker.observed_at):
            raise LoadedGateError(f"The {description} timestamp is invalid")
        sequence = LiveMarkerCollector._sequence(marker.document, description)
        if sequence <= previous_sequence:
            raise LoadedGateError(
                f"The {description} sequences are not strictly monotonic and unique"
            )
        previous_sequence = sequence
    return ordered


def loaded_mass_stream(
    markers: Sequence[TimedMarker], description: str
) -> tuple[list[TimedMarker], dict[int, TimedMarker]]:
    ordered = ordered_marker_stream(markers, description)
    by_sequence: dict[int, TimedMarker] = {}
    for marker in ordered:
        if marker.document.get("schemaVersion") != 1:
            raise LoadedGateError(f"The {description} schema is invalid")
        sequence = int(marker.document["sequence"])
        finite_number(marker.document.get("payloadMassKg"), description)
        by_sequence[sequence] = marker
    return ordered, by_sequence


def require_loaded_mass_reference(
    status_marker: TimedMarker,
    mass_by_sequence: dict[int, TimedMarker],
    description: str,
) -> TimedMarker:
    mass_sequence = LiveMarkerCollector._sequence(
        {"sequence": status_marker.document.get("massSequence")},
        f"{description} mass",
    )
    mass_marker = mass_by_sequence.get(mass_sequence)
    if mass_marker is None:
        raise LoadedGateError(f"The {description} does not cite a real mass marker")
    mass = finite_number(
        mass_marker.document.get("payloadMassKg"), f"{description} payload mass"
    )
    if not math.isclose(mass, LOADED_PAYLOAD_MASS_KG, abs_tol=1e-6):
        raise LoadedGateError(f"The {description} cites a mass marker that is not 0.75 kg")
    if (
        mass_marker.observed_at > status_marker.observed_at
        or status_marker.observed_at - mass_marker.observed_at
        > MAXIMUM_LIVE_MASS_AGE_SECONDS
    ):
        raise LoadedGateError(f"The {description} does not cite a fresh mass sample")
    return mass_marker


def validate_progress_high_water(
    markers: Sequence[TimedMarker], description: str
) -> float:
    """Reject a regression against any earlier high-water mark in the task."""
    high_water = -1.0
    for marker in ordered_marker_stream(markers, description):
        progress = finite_number(
            marker.document.get("taskProgress"), f"{description} progress"
        )
        if progress + TASK_PROGRESS_REGRESSION_TOLERANCE < high_water:
            raise LoadedGateError(
                f"The {description} progress regressed below its global high-water mark"
            )
        high_water = max(high_water, progress)
    return high_water


def validate_connection_graph(
    connections: dict[str, Any], expected_names: set[str], description: str
) -> tuple[set[str], set[str], int]:
    """Validate that every physical pusher has a path to the payload."""
    roots: set[str] = set()
    companions: set[str] = set()
    parents: dict[str, str] = {}
    connected_count = 0
    ordered_names = sorted(expected_names, key=lambda name: int(name.split("_")[-1]))
    expected_roots = set(ordered_names[: min(2, len(ordered_names))])
    expected_parent = {
        robot: ordered_names[index - 2]
        for index, robot in enumerate(ordered_names)
        if index >= 2
    }
    for robot in ordered_names:
        connection = connections.get(robot)
        if not isinstance(connection, dict):
            raise LoadedGateError(f"The {description} connection is not structured")
        role = connection.get("role")
        parent = connection.get("parent")
        if connection.get("connected") is not True:
            raise LoadedGateError(f"A Burger was not connected in the {description}")
        connected_count += 1
        distance = finite_number(
            connection.get("contact_distance_m"), f"{description} contact distance"
        )
        if distance < 0.0:
            raise LoadedGateError(f"The {description} contact distance is negative")
        if role == "payload_root":
            if robot not in expected_roots or parent != "transport_object":
                raise LoadedGateError(
                    f"A {description} payload root does not match transport_object"
                )
            if distance > CAPACITY_PAYLOAD_ROOT_MAX_CONTACT_DISTANCE_M:
                raise LoadedGateError(
                    f"A {description} payload root is too far from transport_object"
                )
            roots.add(robot)
        elif role == "companion":
            if (
                robot in expected_roots
                or parent not in expected_names
                or parent == robot
                or parent != expected_parent.get(robot)
            ):
                raise LoadedGateError(
                    f"A {description} companion has a non-equivalent robot parent"
                )
            if distance > CAPACITY_COMPANION_MAX_CONTACT_DISTANCE_M:
                raise LoadedGateError(
                    f"A {description} companion is too far from its parent"
                )
            companions.add(robot)
            parents[robot] = str(parent)
        else:
            raise LoadedGateError(f"A {description} pusher has an invalid physical role")

    expected_root_count = len(expected_roots)
    if roots != expected_roots or len(companions) != (
        len(expected_names) - expected_root_count
    ):
        raise LoadedGateError(
            f"The {description} chain has the wrong root/companion roles"
        )

    for robot in companions:
        cursor = robot
        visited: set[str] = set()
        while cursor not in roots:
            if cursor in visited or cursor not in parents:
                raise LoadedGateError(
                    f"The {description} companion graph is not connected to the payload"
                )
            visited.add(cursor)
            cursor = parents[cursor]
    return roots, companions, connected_count


def one_marker(lines: list[str], prefix: str) -> MarkerDocument:
    matches = [
        (index, line[len(prefix) :])
        for index, line in enumerate(lines)
        if line.startswith(prefix)
    ]
    if len(matches) != 1:
        raise LoadedGateError(f"The probe did not emit exactly one {prefix.strip()}")
    line_number, payload = matches[0]
    try:
        document = MATRIX.strict_json_loads(payload)
    except (json.JSONDecodeError, ValueError) as exc:
        raise LoadedGateError(f"The {prefix.strip()} document is malformed") from exc
    if not isinstance(document, dict):
        raise LoadedGateError(f"The {prefix.strip()} document is not a JSON object")
    return MarkerDocument(
        document=document,
        source_sha256=hashlib.sha256(payload.encode("utf-8")).hexdigest(),
        line_number=line_number,
    )


def parse_loaded_protocol(output: Any) -> LoadedProtocol:
    if not isinstance(output.stdout, str) or not isinstance(output.stderr, str):
        raise LoadedGateError("The loaded probe output is not textual")
    byte_count = len(output.stdout.encode("utf-8")) + len(output.stderr.encode("utf-8"))
    if byte_count > MAXIMUM_PROTOCOL_OUTPUT_BYTES:
        raise LoadedGateError("The loaded probe exceeded the bounded output limit")

    lines = output.stdout.splitlines()
    grf_result = one_marker(lines, "RESULT_JSON ")
    grf_summary = one_marker(lines, "SUMMARY_JSON ")
    load = one_marker(lines, "LOAD_RESULT_JSON ")
    post_cleanup = one_marker(lines, "POST_LOAD_CLEANUP_JSON ")
    order = (
        grf_result.line_number,
        grf_summary.line_number,
        load.line_number,
        post_cleanup.line_number,
    )
    if list(order) != sorted(order) or len(set(order)) != len(order):
        raise LoadedGateError("The loaded probe protocol was emitted out of order")
    return LoadedProtocol(
        load=load,
        grf_result=grf_result,
        grf_summary=grf_summary,
        post_cleanup=post_cleanup,
        returncode=int(output.returncode),
    )


def trial(document: dict[str, Any], name: str, count: int) -> dict[str, Any]:
    value = document.get(name)
    robot_count = value.get("robot_count") if isinstance(value, dict) else None
    if (
        not isinstance(value, dict)
        or isinstance(robot_count, bool)
        or robot_count != count
    ):
        raise LoadedGateError(f"The {name} evidence does not describe N={count}")
    return value


def capacity_trial_roster(
    item: dict[str, Any], count: int
) -> tuple[set[str], tuple[int, ...]]:
    """Return the exact fresh Burger namespace block used by one trial."""
    progress_by_robot = item.get("robot_forward_progress_m")
    connections_by_robot = item.get("final_push_connections")
    if not isinstance(progress_by_robot, dict) or not isinstance(
        connections_by_robot, dict
    ):
        raise LoadedGateError(
            f"The loaded N={count} trial does not contain structured Burger maps"
        )

    progress_names = set(progress_by_robot)
    connection_names = set(connections_by_robot)
    if progress_names != connection_names:
        raise LoadedGateError(
            f"The loaded N={count} trial maps do not contain the same Burger roster"
        )
    if len(progress_by_robot) != count or len(connections_by_robot) != count:
        raise LoadedGateError(
            f"The loaded N={count} trial does not contain exactly {count} Burgers"
        )

    indices: list[int] = []
    for name in progress_names:
        if not isinstance(name, str) or re.fullmatch(
            r"tb3_(0|[1-9][0-9]*)", name
        ) is None:
            raise LoadedGateError(
                f"The loaded N={count} trial contains an invalid Burger namespace"
            )
        indices.append(int(name[4:]))

    ordered_indices = tuple(sorted(indices))
    first_index = ordered_indices[0]
    expected_indices = tuple(range(first_index, first_index + count))
    if ordered_indices != expected_indices:
        raise LoadedGateError(
            f"The loaded N={count} trial Burger roster is not contiguous"
        )
    expected_names = {f"tb3_{index}" for index in expected_indices}
    if progress_names != expected_names:
        raise LoadedGateError(
            f"The loaded N={count} trial Burger roster is not canonical"
        )
    return expected_names, ordered_indices


def validate_loaded_capacity(document: dict[str, Any]) -> dict[str, Any]:
    if (
        document.get("profile") != "transport_crate_loaded"
        or not math.isclose(
            finite_number(document.get("profile_mass_kg"), "loaded payload mass"),
            LOADED_PAYLOAD_MASS_KG,
            abs_tol=1e-9,
        )
        or not math.isclose(
            finite_number(document.get("profile_friction"), "loaded friction"),
            0.25,
            abs_tol=1e-9,
        )
        or document.get("passed") is not True
        or document.get("failures") not in ([], None)
    ):
        raise LoadedGateError("The loaded capacity probe did not pass cleanly")

    thresholds = document.get("thresholds")
    if not isinstance(thresholds, dict):
        raise LoadedGateError("The loaded capacity thresholds are missing")
    pinned_thresholds = {
        "single_robot_maximum_progress_m": CAPACITY_SINGLE_MAX_PROGRESS_M,
        "two_root_maximum_progress_m": CAPACITY_ROOT_MAX_PROGRESS_M,
        "fleet_minimum_progress_m": CAPACITY_FLEET_MIN_PROGRESS_M,
        "minimum_robot_progress_m": CAPACITY_ROBOT_MIN_PROGRESS_M,
        "minimum_payload_progress_gain": CAPACITY_MINIMUM_GAIN,
        "minimum_real_time_factor": MINIMUM_REAL_TIME_FACTOR,
    }
    for name, expected_value in pinned_thresholds.items():
        observed_value = finite_number(thresholds.get(name), f"loaded {name}")
        if not math.isclose(observed_value, expected_value, abs_tol=1e-9):
            raise LoadedGateError("The loaded probe did not echo the pinned thresholds")

    single = trial(document, "single_robot_trial", 1)
    roots = trial(document, "root_only_trial", 2)
    fleet = trial(document, "fleet_trial", EXPECTED_ROBOT_COUNT)
    trial_rtfs: list[float] = []
    trial_durations: list[dict[str, float]] = []
    trial_connection_roles: dict[int, tuple[set[str], set[str]]] = {}
    trial_rosters: dict[int, set[str]] = {}
    prior_names: set[str] = set()
    next_expected_index: int | None = None
    for item in (single, roots, fleet):
        count = int(item["robot_count"])
        expected_names, ordered_indices = capacity_trial_roster(item, count)
        if prior_names.intersection(expected_names):
            raise LoadedGateError(
                "The loaded capacity trials reused a Burger namespace"
            )
        if (
            next_expected_index is not None
            and ordered_indices[0] != next_expected_index
        ):
            raise LoadedGateError(
                "The loaded capacity trial rosters do not follow the fleet "
                "manager's monotonic allocation sequence"
            )
        prior_names.update(expected_names)
        next_expected_index = ordered_indices[-1] + 1
        trial_rosters[count] = expected_names

        progress_by_robot = item["robot_forward_progress_m"]
        connections_by_robot = item.get("final_push_connections")
        root_ids, companion_ids, connected_count = validate_connection_graph(
            connections_by_robot,
            expected_names,
            f"loaded N={count}",
        )
        reported_connected = item.get("final_connected_robot_count")
        if (
            isinstance(reported_connected, bool)
            or not isinstance(reported_connected, int)
            or reported_connected != connected_count
            or connected_count != count
        ):
            raise LoadedGateError(
                "A loaded capacity trial's final connection count is inconsistent"
            )
        trial_connection_roles[count] = (root_ids, companion_ids)

        command_speed = finite_number(
            item.get("command_speed_mps"), "loaded trial command speed"
        )
        simulated = finite_number(
            item.get("push_duration_sim_s"), "loaded trial simulated duration"
        )
        wall = finite_number(
            item.get("push_duration_wall_s"), "loaded trial wall duration"
        )
        published_rtf = finite_number(
            item.get("real_time_factor"), "loaded trial published RTF"
        )
        if not math.isclose(command_speed, CAPACITY_COMMAND_SPEED_MPS, abs_tol=1e-9):
            raise LoadedGateError("A loaded capacity trial used a different command speed")
        if not math.isclose(
            simulated,
            CAPACITY_PUSH_DURATION_SIM_S,
            abs_tol=CAPACITY_PUSH_DURATION_TOLERANCE_SIM_S,
        ):
            raise LoadedGateError("A loaded capacity trial did not cover about 12 sim seconds")
        if wall <= 0.0:
            raise LoadedGateError("A loaded capacity trial has no positive wall duration")
        recalculated_rtf = simulated / wall
        if not math.isclose(
            recalculated_rtf,
            published_rtf,
            rel_tol=0.0,
            abs_tol=CAPACITY_RTF_MATCH_TOLERANCE,
        ):
            raise LoadedGateError("A loaded capacity trial published an inconsistent RTF")
        if recalculated_rtf < MINIMUM_REAL_TIME_FACTOR:
            raise LoadedGateError("A loaded capacity trial ran below RTF 2.90")
        trial_rtfs.append(recalculated_rtf)
        trial_durations.append(
            {
                "robotCount": count,
                "commandSpeedMps": command_speed,
                "simulatedSeconds": simulated,
                "wallSeconds": wall,
                "recalculatedRealTimeFactor": recalculated_rtf,
            }
        )

    single_progress = finite_number(
        single.get("payload_forward_progress_m"), "single-robot payload progress"
    )
    root_progress = finite_number(
        roots.get("payload_forward_progress_m"), "two-root payload progress"
    )
    fleet_progress = finite_number(
        fleet.get("payload_forward_progress_m"), "four-robot payload progress"
    )
    independent_gain = fleet_progress / max(single_progress, 0.005)
    if single_progress > CAPACITY_SINGLE_MAX_PROGRESS_M:
        raise LoadedGateError("One Burger moved the loaded payload beyond its limit")
    if root_progress > CAPACITY_ROOT_MAX_PROGRESS_M:
        raise LoadedGateError("Two payload roots moved the loaded payload beyond their limit")
    if fleet_progress < CAPACITY_FLEET_MIN_PROGRESS_M:
        raise LoadedGateError("Four Burgers did not move the loaded payload far enough")
    if independent_gain < CAPACITY_MINIMUM_GAIN:
        raise LoadedGateError("The four-robot payload gain was too small")
    # The official probe reports its own rounded gain for diagnostics.  The
    # gate deliberately does not use that echo for the decision: it derives
    # the ratio again from the raw trial progress above.
    finite_number(document.get("payload_progress_gain"), "payload progress gain")

    progress = fleet.get("robot_forward_progress_m")
    connections = fleet.get("final_push_connections")
    fleet_roster = trial_rosters[EXPECTED_ROBOT_COUNT]
    if not isinstance(progress, dict) or set(progress) != fleet_roster:
        raise LoadedGateError("Per-robot loaded progress does not cover all four Burgers")
    if not isinstance(connections, dict) or set(connections) != set(progress):
        raise LoadedGateError("Loaded push connections do not cover the same four Burgers")
    for robot, value in progress.items():
        if finite_number(value, "loaded robot progress") < CAPACITY_ROBOT_MIN_PROGRESS_M:
            raise LoadedGateError("A Burger did not advance during the loaded push")
    fleet_roots, fleet_companions = trial_connection_roles[EXPECTED_ROBOT_COUNT]

    linked = document.get("loaded_grf_trial")
    if not isinstance(linked, dict) or (
        linked.get("scenario") != "transport_grf_n4"
        or linked.get("passed") is not True
        or linked.get("exit_code") != 0
        or linked.get("fresh_cleanup_status_observed") is not True
    ):
        raise LoadedGateError("The loaded probe did not complete its GRF N=4 phase")

    return {
        "profileMassKg": LOADED_PAYLOAD_MASS_KG,
        "profileFriction": 0.25,
        "singleRobotProgressM": single_progress,
        "twoRootProgressM": root_progress,
        "fleetProgressM": fleet_progress,
        "payloadProgressGain": independent_gain,
        "connectedRobotCount": EXPECTED_ROBOT_COUNT,
        "payloadRootCount": len(fleet_roots),
        "companionCount": len(fleet_companions),
        "trialRealTimeFactors": trial_rtfs,
        "trialTiming": trial_durations,
    }


def _positive(value: Any, description: str) -> float:
    number = finite_number(value, description)
    if number <= 0:
        raise LoadedGateError(f"The {description} is not positive")
    return number


def validate_grf_n4(result: dict[str, Any], summary: dict[str, Any]) -> dict[str, Any]:
    evidence = MATRIX.RosEvidence(
        result=result,
        summary=summary,
        result_sha256="0" * 64,
        summary_sha256="0" * 64,
        returncode=0,
    )
    try:
        ros_rtf = MATRIX.validate_ros_evidence(evidence)
    except Exception as exc:
        raise LoadedGateError(str(exc)) from exc
    if (
        result.get("scenario") != "transport_grf_n4"
        or result.get("behavior") != "transport"
        or result.get("robot_count") != EXPECTED_ROBOT_COUNT
    ):
        raise LoadedGateError("The GRF evidence is not the requested N=4 transport")

    metrics = result.get("metrics")
    thresholds = result.get("transport_acceptance_thresholds")
    if not isinstance(metrics, dict) or not isinstance(thresholds, dict):
        raise LoadedGateError("The GRF N=4 metrics or thresholds are missing")
    published_rtf = finite_number(
        metrics.get("real_time_factor"), "GRF published real-time factor"
    )
    simulated_duration = finite_number(
        metrics.get("simulated_duration_s"), "GRF simulated duration"
    )
    wall_duration = finite_number(
        metrics.get("wall_duration_s"), "GRF wall duration"
    )
    if simulated_duration <= 0.0 or wall_duration <= 0.0:
        raise LoadedGateError("The GRF timing durations must both be positive")
    recalculated_rtf = simulated_duration / wall_duration
    if not math.isclose(
        recalculated_rtf,
        published_rtf,
        rel_tol=0.0,
        abs_tol=GRF_RTF_MATCH_TOLERANCE,
    ):
        raise LoadedGateError("The GRF task published an inconsistent RTF")
    if recalculated_rtf < MINIMUM_REAL_TIME_FACTOR:
        raise LoadedGateError("The GRF task ran below RTF 2.90")
    # MATRIX validates the wider ROS contract, but its RTF field is an echo.
    # Keep that check and make the physical timing ratio authoritative here.
    if not math.isclose(ros_rtf, published_rtf, rel_tol=0.0, abs_tol=1e-9):
        raise LoadedGateError("The GRF validators observed different RTF values")
    if thresholds.get("active_search_required") is not True:
        raise LoadedGateError("The loaded GRF case did not require active search")
    pinned_thresholds = {
        "direct_payload_surface_clearance_m": GRF_DIRECT_PAYLOAD_CLEARANCE_M,
        "companion_center_distance_m": GRF_COMPANION_CENTER_DISTANCE_M,
        "minimum_chain_center_distance_m": GRF_MINIMUM_CHAIN_CENTER_DISTANCE_M,
        "cmd_vel_min_forward_speed_mps": GRF_COMMAND_MINIMUM_SPEED_MPS,
        "cmd_vel_min_goal_alignment_cosine": GRF_COMMAND_MINIMUM_GOAL_COSINE,
        "cmd_vel_max_wall_age_s": GRF_COMMAND_MAXIMUM_AGE_S,
        "contribution_goal_velocity_noise_floor_mps": (
            GRF_CONTRIBUTION_NOISE_FLOOR_MPS
        ),
        "contribution_goal_velocity_tolerance_mps": (
            GRF_CONTRIBUTION_SPEED_TOLERANCE_MPS
        ),
        "contribution_minimum_tracking_fraction": (
            GRF_CONTRIBUTION_TRACKING_FRACTION
        ),
        "maximum_control_sample_gap_s": GRF_CONTROL_GAP_TOLERANCE_S,
        "minimum_continuous_useful_push_s": GRF_MINIMUM_PUSH_DURATION_S,
        "minimum_continuous_useful_push_samples": GRF_MINIMUM_PUSH_SAMPLES,
        "minimum_useful_push_fraction": GRF_MINIMUM_USEFUL_FRACTION,
        "minimum_goal_progress_efficiency": GRF_MINIMUM_GOAL_EFFICIENCY,
        "configured_minimum_goal_progress_m": GRF_CONFIGURED_GOAL_PROGRESS_M,
        "goal_arrival_tolerance_m": GRF_GOAL_ARRIVAL_TOLERANCE_M,
        "goal_progress_contract_epsilon_m": (
            GRF_GOAL_PROGRESS_CONTRACT_EPSILON_M
        ),
        "minimum_active_search_path_length_per_robot_m": (
            GRF_MINIMUM_SEARCH_TRAVEL_M
        ),
        "minimum_notified_robot_rendezvous_travel_m": (
            GRF_MINIMUM_RENDEZVOUS_TRAVEL_M
        ),
        "minimum_payload_root_rendezvous_travel_m": (
            GRF_MINIMUM_ROOT_RENDEZVOUS_TRAVEL_M
        ),
    }
    for name, expected_value in pinned_thresholds.items():
        observed_value = finite_number(thresholds.get(name), f"GRF {name}")
        if not math.isclose(observed_value, expected_value, abs_tol=1e-9):
            raise LoadedGateError("The GRF result did not echo the pinned thresholds")

    if thresholds.get("goal_progress_contract_basis") != (
        GRF_GOAL_PROGRESS_CONTRACT_BASIS
    ):
        raise LoadedGateError(
            "The GRF result omitted the pinned goal-progress contract basis"
        )
    push_initial_goal_distance = finite_number(
        metrics.get("transport_push_initial_goal_distance_m"),
        "initial PUSH goal distance",
    )
    if push_initial_goal_distance <= 0.0:
        raise LoadedGateError("The initial PUSH goal distance is not positive")
    contract_progress = max(
        GRF_GOAL_PROGRESS_CONTRACT_EPSILON_M,
        push_initial_goal_distance - GRF_GOAL_ARRIVAL_TOLERANCE_M,
    )
    expected_progress_echo = min(
        GRF_CONFIGURED_GOAL_PROGRESS_M,
        contract_progress,
    )
    observed_progress_echo = finite_number(
        thresholds.get("minimum_goal_progress_m"),
        "GRF dynamic goal-progress threshold",
    )
    if not math.isclose(
        observed_progress_echo,
        expected_progress_echo,
        rel_tol=0.0,
        abs_tol=1e-9,
    ):
        raise LoadedGateError(
            "The GRF result did not echo the dynamic goal-progress threshold"
        )

    response = metrics.get("transport_discovery_response")
    participation = metrics.get("transport_participation")
    if not isinstance(response, dict) or not isinstance(participation, dict):
        raise LoadedGateError("Search or push participation evidence is missing")
    robots = response.get("robots")
    if not isinstance(robots, dict) or set(robots) != EXPECTED_ROBOT_SET:
        raise LoadedGateError("Rendezvous evidence does not cover the exact N=4 roster")
    expected = set(EXPECTED_ROBOT_SET)
    if set(participation) != expected:
        raise LoadedGateError("Search and push evidence describe different fleets")

    finder = response.get("finder")
    recipients = response.get("notice_recipients")
    notified = response.get("notified_robots")
    search_movers = response.get("robots_observed_moving_during_search")
    if finder not in expected:
        raise LoadedGateError("The discovery finder is not in the exact N=4 roster")
    exact_id_list(recipients, expected, "finder notification recipient list")
    exact_id_list(notified, expected - {str(finder)}, "notified robot list")
    if (
        response.get("notice_observed") is not True
        or response.get("missing_notice_recipients") not in ([], None)
    ):
        raise LoadedGateError("Finder notification did not reach the complete N=4 fleet")
    if response.get("status_acknowledgement_available") is True:
        exact_id_list(
            response.get("status_acknowledged_robots"),
            expected,
            "status acknowledgement list",
        )
        if response.get("missing_status_acknowledgements") not in ([], None):
            raise LoadedGateError(
                "The complete fleet did not acknowledge the discovery notice"
            )

    exact_id_list(search_movers, expected, "SEARCH mover list")

    if (
        response.get("search_motion_window_supported") is not True
        or response.get("simultaneous_search_motion_window_supported") is not True
        or int(response.get("peak_simultaneous_search_movers") or 0)
        < EXPECTED_ROBOT_COUNT
        or response.get("robots_not_observed_moving_during_search") not in ([], None)
        or response.get("robots_below_required_search_travel") not in ([], None)
    ):
        raise LoadedGateError("All four Burgers were not proven active during SEARCH")

    search_paths = response.get("search_path_length_m")
    minimum_search = finite_number(
        response.get("minimum_search_path_length_m"), "minimum SEARCH travel"
    )
    if not math.isclose(
        minimum_search, GRF_MINIMUM_SEARCH_TRAVEL_M, abs_tol=1e-9
    ):
        raise LoadedGateError("SEARCH evidence did not use the pinned travel minimum")
    if not isinstance(search_paths, dict) or set(search_paths) != expected:
        raise LoadedGateError("Per-robot SEARCH paths are incomplete")
    if any(
        finite_number(search_paths[robot], "SEARCH path length") < minimum_search
        for robot in expected
    ):
        raise LoadedGateError("A Burger did not travel far enough while searching")

    for robot in expected:
        rendezvous = robots[robot]
        participation_item = participation[robot]
        if not isinstance(rendezvous, dict) or (
            rendezvous.get("notice_recipient") is not True
            or rendezvous.get("motion_detected") is not True
            or rendezvous.get("met_pre_push_path_requirement") is not True
        ):
            raise LoadedGateError("A Burger did not complete its notified rendezvous")
        if not isinstance(participation_item, dict):
            raise LoadedGateError("A Burger has no structured push evidence")
        if rendezvous.get("role") != participation_item.get("role"):
            raise LoadedGateError("Rendezvous and PUSH roles do not correlate")
        path_length = _positive(
            rendezvous.get("pre_push_path_length_m"), "rendezvous path length"
        )
        required_travel = (
            GRF_MINIMUM_ROOT_RENDEZVOUS_TRAVEL_M
            if rendezvous.get("role") == "payload_push" or robot == finder
            else GRF_MINIMUM_RENDEZVOUS_TRAVEL_M
        )
        echoed_travel = finite_number(
            rendezvous.get("required_pre_push_path_length_m"),
            "required rendezvous travel",
        )
        if not math.isclose(echoed_travel, required_travel, abs_tol=1e-9):
            raise LoadedGateError("Rendezvous evidence did not use the pinned minimum")
        if path_length < required_travel:
            raise LoadedGateError("A Burger did not meet the pinned rendezvous minimum")
    if response.get("robots_below_required_travel") not in ([], None) or response.get(
        "notified_robots_below_required_travel"
    ) not in ([], None):
        raise LoadedGateError("The complete fleet did not reach its pre-push rendezvous")

    roots: set[str] = set()
    companions: set[str] = set()
    companion_parents: dict[str, list[str]] = {}
    for robot in expected:
        item = participation[robot]
        if not isinstance(item, dict):
            raise LoadedGateError("A Burger has no structured push evidence")
        role = item.get("role")
        declared_parents = item.get("declared_parent_namespaces")
        if (
            not isinstance(declared_parents, list)
            or any(not isinstance(parent, str) for parent in declared_parents)
            or len(declared_parents) != len(set(declared_parents))
        ):
            raise LoadedGateError("A pusher has an invalid parent namespace list")
        if role == "payload_push":
            if declared_parents:
                raise LoadedGateError("A payload root declared a robot predecessor")
            roots.add(robot)
            _positive(item.get("direct_contact_samples"), "payload-contact samples")
        elif role == "companion_push":
            if (
                not declared_parents
                or any(
                    parent not in expected or parent == robot
                    for parent in declared_parents
                )
            ):
                raise LoadedGateError(
                    "A companion pusher has an invalid declared predecessor"
                )
            companions.add(robot)
            companion_parents[robot] = declared_parents
            _positive(item.get("companion_contact_samples"), "companion-contact samples")
        else:
            raise LoadedGateError("A Burger has no transport-push role")
        _positive(item.get("connected_samples"), "connected push samples")
        _positive(item.get("push_intent_samples"), "push-intent samples")
        if finite_number(
            item.get("maximum_continuous_useful_pushing_s"),
            "continuous useful-push duration",
        ) < GRF_MINIMUM_PUSH_DURATION_S:
            raise LoadedGateError("A Burger did not sustain useful pushing")
        if finite_number(
            item.get("maximum_continuous_useful_pushing_samples"),
            "continuous useful-push samples",
        ) < GRF_MINIMUM_PUSH_SAMPLES:
            raise LoadedGateError("A Burger did not sustain enough useful push samples")
        if finite_number(
            item.get("useful_pushing_fraction"), "useful-push fraction"
        ) < GRF_MINIMUM_USEFUL_FRACTION:
            raise LoadedGateError("A Burger did not contribute for enough payload progress")
    if not roots or len(roots) + len(companions) != EXPECTED_ROBOT_COUNT:
        raise LoadedGateError("The GRF push did not assign all four Burgers")
    if len(roots) != 2 or len(companions) != 2:
        raise LoadedGateError(
            "The loaded GRF push did not use exactly two payload roots and two companions"
        )

    def reaches_payload(robot: str, visiting: set[str]) -> bool:
        if robot in roots:
            return True
        if robot in visiting or robot not in companion_parents:
            return False
        next_visiting = set(visiting)
        next_visiting.add(robot)
        return all(
            reaches_payload(parent, next_visiting)
            for parent in companion_parents[robot]
        )

    if any(not reaches_payload(robot, set()) for robot in companions):
        raise LoadedGateError(
            "The GRF companion graph is not connected to a payload root"
        )

    if (
        metrics.get("transport_active_push_latched") is not True
        or finite_number(metrics.get("transport_grf_samples"), "GRF samples")
        < GRF_MINIMUM_PUSH_SAMPLES
        or finite_number(
            metrics.get("transport_all_useful_samples"), "all-fleet useful samples"
        ) < GRF_MINIMUM_PUSH_SAMPLES
        or finite_number(
            metrics.get("transport_maximum_continuous_all_useful_s"),
            "all-fleet useful duration",
        ) < GRF_MINIMUM_PUSH_DURATION_S
        or finite_number(
            metrics.get("transport_maximum_continuous_all_useful_samples"),
            "all-fleet continuous samples",
        ) < GRF_MINIMUM_PUSH_SAMPLES
        or finite_number(
            metrics.get("transport_all_useful_batch_fraction"),
            "all-fleet useful batch fraction",
        ) < GRF_MINIMUM_USEFUL_FRACTION
    ):
        raise LoadedGateError("All four Burgers were not proven pushing together")
    goal_progress = finite_number(
        metrics.get("transport_push_goal_progress_m"), "loaded GRF goal progress"
    )
    if (
        goal_progress + GRF_GOAL_PROGRESS_CONTRACT_EPSILON_M
        < GRF_MINIMUM_GOAL_PROGRESS_M
        or goal_progress + 1e-9 < observed_progress_echo
    ):
        raise LoadedGateError("The loaded payload did not make enough goal progress")
    goal_efficiency = finite_number(
        metrics.get("transport_push_goal_progress_efficiency"),
        "loaded GRF goal efficiency",
    )
    if goal_efficiency < GRF_MINIMUM_GOAL_EFFICIENCY:
        raise LoadedGateError("The loaded payload motion was not goal-directed enough")

    return {
        "observedRealTimeFactor": recalculated_rtf,
        "robotCount": EXPECTED_ROBOT_COUNT,
        "finderNoticeObserved": True,
        "noticeRecipientCount": EXPECTED_ROBOT_COUNT,
        "simultaneousSearchMoverCount": EXPECTED_ROBOT_COUNT,
        "rendezvousRobotCount": EXPECTED_ROBOT_COUNT,
        "usefulPusherCount": EXPECTED_ROBOT_COUNT,
        "payloadRootCount": len(roots),
        "companionPusherCount": len(companions),
        "grfSamples": int(metrics["transport_grf_samples"]),
        "allFleetUsefulSamples": int(metrics["transport_all_useful_samples"]),
        "contractGoalProgressM": observed_progress_echo,
        "physicalMinimumGoalProgressM": GRF_MINIMUM_GOAL_PROGRESS_M,
        "goalProgressM": goal_progress,
        "goalProgressEfficiency": goal_efficiency,
    }


def validate_post_cleanup(document: dict[str, Any]) -> dict[str, Any]:
    mass = finite_number(document.get("payloadMassKg"), "restored payload mass")
    if (
        document.get("schemaVersion") != 1
        or document.get("complete") is not True
        or document.get("taskTerminal") is not True
        or document.get("rosterEmpty") is not True
        or document.get("robotModelsAbsent") is not True
        or document.get("payloadPresent") is not True
        or not math.isclose(mass, PRACTICE_PAYLOAD_MASS_KG, abs_tol=1e-6)
    ):
        raise LoadedGateError(
            "The official probe did not prove task/roster cleanup and 0.25 kg restoration"
        )
    return {
        "taskTerminal": True,
        "rosterEmpty": True,
        "robotModelsAbsent": True,
        "practicePayloadRestored": True,
        "payloadMassKg": mass,
    }


def correlate_loaded_grf(load: dict[str, Any], grf: dict[str, Any]) -> None:
    linked = load.get("loaded_grf_trial")
    task = grf.get("task")
    if not isinstance(linked, dict) or not isinstance(task, dict):
        raise LoadedGateError("The loaded GRF task correlation is missing")
    load_task = str(linked.get("task_id") or "")
    result_task = str(grf.get("task_id") or "")
    status_task = str(task.get("task_id") or "")
    if (
        not re.fullmatch(r"loaded-grf-n4-[0-9a-f]{8}", load_task)
        or load_task != result_task
        or load_task != status_task
    ):
        raise LoadedGateError("The loaded probe and GRF result belong to different tasks")


def validate_protocol(protocol: LoadedProtocol) -> dict[str, Any]:
    capacity = validate_loaded_capacity(protocol.load.document)
    grf = validate_grf_n4(
        protocol.grf_result.document, protocol.grf_summary.document
    )
    correlate_loaded_grf(protocol.load.document, protocol.grf_result.document)
    cleanup = validate_post_cleanup(protocol.post_cleanup.document)
    if protocol.returncode != 0:
        raise LoadedGateError("The loaded probe process returned a failure status")
    return {"capacity": capacity, "grf": grf, "probeCleanup": cleanup}


def monitor_supervision_shell() -> str:
    """Return the bounded Bash lifecycle for the live-marker monitor job."""
    return r'''
cleanup_monitor_runtime() {
  rm -f -- "$monitor_stop" || return 1
  rmdir -- "$monitor_runtime" || return 1
}

monitor_is_running() {
  [ "$monitor_running" = true ] || return 1
  [ "$monitor_job" = '%1' ] || return 1
  jobs -p "$monitor_job" >/dev/null 2>&1
}

reap_monitor() {
  monitor_status=0
  # wait(1) on the PID captured from $! keeps the child's real status even
  # after Bash has retired the completed jobspec from `jobs` output.
  wait "$monitor_wait_pid" 2>/dev/null || monitor_status=$?
  monitor_running=false
  cleanup_monitor_runtime || return 1
  return "$monitor_status"
}

stop_monitor() {
  if [ "$monitor_running" != true ]; then
    cleanup_monitor_runtime
    return
  fi

  if ! monitor_is_running; then
    # A monitor that ended before we requested shutdown did not supervise the
    # complete loaded probe, even when its own exit status happened to be zero.
    reap_monitor || true
    return 1
  fi

  : > "$monitor_stop" || return 1
  for _attempt in $(seq 1 50); do
    if ! monitor_is_running; then
      reap_monitor
      return
    fi
    sleep 0.1
  done

  # A Bash jobspec is resolved against this shell's child-job table.  It cannot
  # turn into an unrelated process if the original numeric PID is later reused.
  kill -TERM "$monitor_job" 2>/dev/null || true
  for _attempt in $(seq 1 50); do
    if ! monitor_is_running; then
      reap_monitor
      return
    fi
    sleep 0.1
  done

  kill -KILL "$monitor_job" 2>/dev/null || true
  for _attempt in $(seq 1 50); do
    if ! monitor_is_running; then
      reap_monitor
      return
    fi
    sleep 0.1
  done
  return 1
}

interrupt_loaded_run() {
  interrupt_status="$1"
  if ! stop_monitor; then
    interrupt_status=92
  fi
  trap - EXIT
  exit "$interrupt_status"
}
'''.strip()


def build_payload_command(docker: str, container_identifier: str) -> list[str]:
    monitor = r'''
import json
import math
import os
import re
import signal
import sys
import threading
import time

import rospy
from gazebo_msgs.msg import ModelStates
from gazebo_msgs.srv import GetLinkProperties
from std_msgs.msg import String

running = True
state_lock = threading.Lock()
output_lock = threading.Lock()
latest_mass = None
latest_mass_at = None
mass_sequence = 0
active_sequence = 0
push_sequence = 0
roster = []
robot_models = []
expected_robots = ['tb3_0', 'tb3_1', 'tb3_2', 'tb3_3']
stop_path = sys.argv[1]


def request_stop(_number, _frame):
    global running
    running = False


def emit(prefix, document):
    encoded = (
        prefix + json.dumps(
            document, sort_keys=True, separators=(',', ':'), allow_nan=False
        ) + '\n'
    ).encode('utf-8')
    if len(encoded) > 4096:
        raise RuntimeError('live marker exceeds the atomic pipe-write limit')
    # This process owns the dedicated live-marker pipe.  One sub-PIPE_BUF write
    # also keeps callbacks indivisible if two ROS threads emit together.
    with output_lock:
        written = os.write(1, encoded)
    if written != len(encoded):
        raise RuntimeError('live marker write was incomplete')


def roster_callback(message):
    global roster
    names = sorted(
        item.strip('/') for item in message.data.split(',') if item.strip('/')
    )
    with state_lock:
        roster = names


def models_callback(message):
    global robot_models
    names = sorted(
        name for name in message.name if re.fullmatch(r'tb3_\d+', str(name))
    )
    with state_lock:
        robot_models = names


def status_callback(message):
    global active_sequence, push_sequence
    try:
        status = json.loads(message.data)
    except (TypeError, ValueError):
        return
    if not isinstance(status, dict) or status.get('robot_count') != 4:
        return
    task = status.get('task')
    if (
        not isinstance(task, dict)
        or task.get('task_type') != 'transport'
        or str(task.get('status') or '').lower() != 'running'
    ):
        return
    result = task.get('result')
    transport = result.get('transport') if isinstance(result, dict) else None
    if not isinstance(transport, dict):
        return
    task_id = str(task.get('task_id') or '')
    phase = transport.get('phase')
    progress = task.get('progress')
    if (
        re.fullmatch(r'loaded-grf-n4-[0-9a-f]{8}', task_id) is None
        or phase not in {'SEARCH', 'APPROACH', 'PUSH'}
        or isinstance(progress, bool)
        or not isinstance(progress, (int, float))
        or not math.isfinite(float(progress))
        or not 0.0 <= float(progress) <= 1.0
    ):
        return
    now = time.monotonic()
    with state_lock:
        mass = latest_mass
        sampled_at = latest_mass_at
        observed_mass_sequence = mass_sequence
        observed_roster = list(roster)
        observed_models = list(robot_models)
        if mass is None or sampled_at is None:
            return
        mass_age = now - sampled_at
        if (
            observed_roster != expected_robots
            or observed_models != expected_robots
            or not math.isclose(mass, 0.75, abs_tol=1e-6)
            or mass_age < 0.0
            or mass_age > 0.75
        ):
            return
        active_sequence += 1
        observed_active_sequence = active_sequence
    common = {
        'schemaVersion': 1,
        'sequence': observed_active_sequence,
        'taskId': task_id,
        'taskStatus': 'running',
        'taskProgress': float(progress),
        'phase': phase,
        'robotCount': 4,
        'roster': observed_roster,
        'robotModels': observed_models,
        'payloadMassKg': mass,
        'massSequence': observed_mass_sequence,
        'massSampleAgeSeconds': mass_age,
    }
    emit('LOADED_GRF_ACTIVE_JSON ', common)
    if phase != 'PUSH':
        return
    contributors = transport.get('useful_contributor_ids')
    if (
        transport.get('all_pushers_confirmed') is not True
        or transport.get('useful_contributor_count') != 4
        or not isinstance(contributors, list)
        or sorted(contributors) != expected_robots
    ):
        return
    with state_lock:
        push_sequence += 1
        sequence = push_sequence
    push = dict(common)
    push.update({
        'sequence': sequence,
        'allPushersConfirmed': True,
        'usefulContributorCount': 4,
        'usefulContributorIds': sorted(contributors),
    })
    emit('LOADED_PUSH_LIVE_JSON ', push)


signal.signal(signal.SIGINT, request_stop)
signal.signal(signal.SIGTERM, request_stop)
rospy.init_node('robotswarm_loaded_live_monitor', anonymous=True, disable_signals=True)
rospy.Subscriber('/swarm/status', String, status_callback, queue_size=20)
rospy.Subscriber('/fleet/robot_list', String, roster_callback, queue_size=5)
rospy.Subscriber('/gazebo/model_states', ModelStates, models_callback, queue_size=5)
rospy.wait_for_service('/gazebo/get_link_properties', timeout=20.0)
properties = rospy.ServiceProxy('/gazebo/get_link_properties', GetLinkProperties)
while running and not rospy.is_shutdown() and not os.path.exists(stop_path):
    try:
        response = properties('transport_object::link')
        if response.success:
            mass = float(response.mass)
            if math.isfinite(mass):
                with state_lock:
                    mass_sequence += 1
                    sequence = mass_sequence
                    sampled_at = time.monotonic()
                    emit('LOADED_MASS_SAMPLE_JSON ', {
                        'schemaVersion': 1,
                        'sequence': sequence,
                        'payloadMassKg': mass,
                    })
                    latest_mass = mass
                    latest_mass_at = sampled_at
    except (rospy.ROSException, rospy.ServiceException, TypeError, ValueError):
        pass
    time.sleep(0.20)
'''.strip()
    load_shim = r'''
import importlib.util
import inspect
import json
import math
import os
import sys
import xml.etree.ElementTree as ET

module_path = sys.argv[1]
official_arguments = sys.argv[2:]
spec = importlib.util.spec_from_file_location('robotswarm_deployed_load_probe', module_path)
if spec is None or spec.loader is None:
    raise RuntimeError('could not load the deployed payload probe')
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)

original_replace_payload = module.LoadProbe.replace_payload
original_reset_fleet = module.LoadProbe.reset_fleet
deployed_run_trial = getattr(module.LoadProbe, 'run_trial', None)
run_trial_refreshes_payload = (
    deployed_run_trial is not None
    and 'payload_xml' in inspect.signature(deployed_run_trial).parameters
)
ready_emitted = False
loaded_payload_xml = None
marker_fd_text = os.environ.get('ROBOTSWARM_LIVE_MARKER_FD', '1')
try:
    marker_fd = int(marker_fd_text)
except ValueError:
    raise RuntimeError('the live-marker file descriptor is invalid')
if marker_fd not in (1, 3):
    raise RuntimeError('the live-marker file descriptor is not approved')
if marker_fd != 1:
    os.set_inheritable(marker_fd, False)


def emit_ready_marker(document):
    encoded = (
        'LOADED_PAYLOAD_READY_JSON ' + json.dumps(
            document, sort_keys=True, separators=(',', ':'), allow_nan=False
        ) + '\n'
    ).encode('utf-8')
    if len(encoded) > 4096:
        raise RuntimeError('ready marker exceeds the atomic pipe-write limit')
    written = os.write(marker_fd, encoded)
    if written != len(encoded):
        raise RuntimeError('ready marker write was incomplete')


def is_loaded_payload(model_xml):
    try:
        root = ET.fromstring(model_xml)
        masses = [float(node.text) for node in root.findall('.//inertial/mass')]
    except (ET.ParseError, TypeError, ValueError):
        return False
    return any(math.isclose(mass, 0.75, abs_tol=1e-9) for mass in masses)


def replace_payload_with_ready_marker(probe, model_xml):
    global loaded_payload_xml, ready_emitted
    result = original_replace_payload(probe, model_xml)
    if is_loaded_payload(model_xml) and not ready_emitted:
        loaded_payload_xml = model_xml
        ready_emitted = True
        emit_ready_marker({
            'schemaVersion': 1,
            'profile': 'transport_crate_loaded',
            'payloadMassKg': 0.75,
        })
    return result


def reset_fleet_with_loaded_payload(probe, count):
    result = original_reset_fleet(probe, count)
    if loaded_payload_xml is not None:
        original_replace_payload(probe, loaded_payload_xml)
    return result


original_loaded_grf_command = module.loaded_grf_command


def loaded_grf_command_with_pinned_thresholds(args, task_id):
    command = original_loaded_grf_command(args, task_id)
    command.extend([
        '--transport-direct-contact-clearance', '0.075',
        '--transport-companion-contact-distance', '0.16',
        '--min-transport-chain-center-distance', '0.12',
        '--transport-cmd-min-speed', '0.015',
        '--transport-cmd-min-goal-cosine', '0.50',
        '--transport-cmd-max-age', '0.75',
        '--transport-contribution-noise-floor', '0.003',
        '--transport-contribution-speed-tolerance', '0.003',
        '--transport-contribution-tracking-fraction', '0.75',
        '--transport-control-gap-tolerance', '1.0',
        '--min-transport-push-duration', '0.75',
        '--min-transport-push-samples', '5',
        '--min-transport-useful-fraction', '0.50',
        '--min-transport-goal-efficiency', '0.50',
        '--min-transport-search-travel', '0.05',
        '--min-transport-rendezvous-travel', '0.10',
        '--min-transport-root-rendezvous-travel', '0.03',
    ])
    return command


module.LoadProbe.replace_payload = replace_payload_with_ready_marker
if not run_trial_refreshes_payload:
    module.LoadProbe.reset_fleet = reset_fleet_with_loaded_payload
module.loaded_grf_command = loaded_grf_command_with_pinned_thresholds
sys.argv = [module_path, *official_arguments]
raise SystemExit(module.main())
'''.strip()
    postcheck = r'''
import json
import math
import re

import rospy
from gazebo_msgs.msg import ModelStates
from gazebo_msgs.srv import GetLinkProperties
from std_msgs.msg import String

rospy.init_node('robotswarm_loaded_postcheck', anonymous=True, disable_signals=True)
roster_message = rospy.wait_for_message('/fleet/robot_list', String, timeout=12.0)
model_message = rospy.wait_for_message('/gazebo/model_states', ModelStates, timeout=12.0)
status_message = rospy.wait_for_message('/swarm/status', String, timeout=12.0)
rospy.wait_for_service('/gazebo/get_link_properties', timeout=12.0)
properties = rospy.ServiceProxy('/gazebo/get_link_properties', GetLinkProperties)(
    'transport_object::link'
)

roster = [item.strip('/') for item in roster_message.data.split(',') if item.strip('/')]
robot_models = [name for name in model_message.name if re.fullmatch(r'tb3_\d+', name)]
try:
    status = json.loads(status_message.data)
except (TypeError, ValueError):
    status = {}
task = status.get('task') if isinstance(status, dict) else {}
task_state = str((task or {}).get('status') or '').lower()
mass = float(properties.mass)
report = {
    'schemaVersion': 1,
    'taskTerminal': task_state in {'idle', 'completed', 'failed', 'stopped'},
    'rosterEmpty': not roster,
    'robotModelsAbsent': not robot_models,
    'payloadPresent': 'transport_object' in set(model_message.name),
    'payloadMassKg': mass,
}
report['complete'] = all(
    report[key]
    for key in ('taskTerminal', 'rosterEmpty', 'robotModelsAbsent', 'payloadPresent')
) and math.isclose(mass, 0.25, abs_tol=1e-6)
print('POST_LOAD_CLEANUP_JSON ' + json.dumps(
    report, sort_keys=True, separators=(',', ':'), allow_nan=False
), flush=True)
raise SystemExit(0 if report['complete'] else 1)
'''.strip()
    supervision = monitor_supervision_shell()
    bootstrap = f'''
# Keep the official probe protocol on stdout and reserve a separate, bounded
# Docker stream for live markers.  All ordinary stderr is intentionally dropped;
# persisted failures are classified from structured output instead of raw logs.
exec 4>&2
exec 2>/dev/null
source /opt/ros/noetic/setup.bash || exit 90
source /catkin_ws/devel/setup.bash || exit 90
# The monitor must be the only Bash background job, so %1 remains its stable
# jobspec for the complete lifecycle and cannot fall through to another child.
if [ -n "$(jobs -p)" ]; then
  exit 92
fi
umask 077
monitor_runtime=$(mktemp -d /tmp/robotswarm-loaded-monitor.XXXXXX) || exit 92
monitor_stop="$monitor_runtime/stop"
readonly monitor_job='%1'
monitor_wait_pid=
monitor_running=false
{supervision}
trap 'stop_monitor || true' EXIT
trap 'interrupt_loaded_run 130' INT
trap 'interrupt_loaded_run 143' TERM
python3 -u - "$monitor_stop" 1>&4 4>&- <<'PY_MONITOR' &
{monitor}
PY_MONITOR
monitor_wait_pid=$!
readonly monitor_wait_pid
monitor_running=true
load_rc=0
ROBOTSWARM_LIVE_MARKER_FD=3 python3 - {DEPLOYED_LOAD_PROBE_PATH} \
  --fleet-count 4 \
  --command-speed {CAPACITY_COMMAND_SPEED_MPS:.2f} \
  --push-duration {CAPACITY_PUSH_DURATION_SIM_S:.1f} \
  --single-max-progress {CAPACITY_SINGLE_MAX_PROGRESS_M:.2f} \
  --root-only-max-progress {CAPACITY_ROOT_MAX_PROGRESS_M:.2f} \
  --fleet-min-progress {CAPACITY_FLEET_MIN_PROGRESS_M:.2f} \
  --min-robot-progress {CAPACITY_ROBOT_MIN_PROGRESS_M:.2f} \
  --minimum-gain {CAPACITY_MINIMUM_GAIN:.1f} \
  --min-rtf 2.90 \
  --external-viewer-verified \
  --verify-grf-n4 3>&4 4>&- <<'PY_LOAD' || load_rc=$?
{load_shim}
PY_LOAD
if ! stop_monitor; then
  trap - EXIT INT TERM
  exit 92
fi
trap - EXIT INT TERM
exec 4>&-
post_rc=0
python3 - <<'PY' || post_rc=$?
{postcheck}
PY
if [ "$post_rc" -ne 0 ]; then
  exit 91
fi
exit "$load_rc"
'''.strip()
    return [
        docker,
        "exec",
        container_identifier,
        "/bin/bash",
        "-lc",
        bootstrap,
    ]


def build_preflight_command(
    runtime: ViewerRuntime,
    script: Path,
    plugin: Path,
    report: Path,
) -> list[str]:
    probe_runtime = MATRIX.ActiveProbeRuntime(
        runtime.command_prefix,
        runtime.environment,
        runtime.gzclient,
    )
    return MATRIX.build_active_probe_command(
        probe_runtime,
        script,
        plugin,
        report,
    )


def _master_endpoint(value: str, expected_port: int) -> ipaddress.IPv4Address:
    parsed = urlsplit(value)
    if (
        parsed.scheme != "http"
        or parsed.username is not None
        or parsed.password is not None
        or parsed.query
        or parsed.fragment
        or parsed.path not in ("", "/")
        or parsed.port != expected_port
    ):
        raise LoadedGateError("The viewer master URI has an unexpected form")
    try:
        address = ipaddress.ip_address(parsed.hostname or "")
    except ValueError as exc:
        raise LoadedGateError("The viewer master URI is not an IP endpoint") from exc
    if not isinstance(address, ipaddress.IPv4Address):
        raise LoadedGateError("The viewer master URI is not an IPv4 endpoint")
    return address


def validate_master_binding(
    container_address: str,
    viewer_environment: dict[str, str],
    container_environment: dict[str, str],
) -> dict[str, bool]:
    try:
        expected = ipaddress.ip_address(container_address)
    except ValueError as exc:
        raise LoadedGateError("The session container has no valid private address") from exc
    ros_address = _master_endpoint(viewer_environment.get("ROS_MASTER_URI", ""), 11311)
    gazebo_address = _master_endpoint(
        viewer_environment.get("GAZEBO_MASTER_URI", ""), 11345
    )
    if ros_address != expected or gazebo_address != expected:
        raise LoadedGateError("The viewer is not attached to the session container masters")
    if container_environment.get("ROS_MASTER_URI") != "http://127.0.0.1:11311":
        raise LoadedGateError("The payload probe would not use the session ROS master")
    configured_gazebo = container_environment.get("GAZEBO_MASTER_URI")
    if configured_gazebo not in (None, "", "http://127.0.0.1:11345", "http://localhost:11345"):
        raise LoadedGateError("The payload probe would not use the local Gazebo master")
    return {
        "sameSessionContainer": True,
        "sameRosMaster": True,
        "sameGazeboMaster": True,
        "privateNetworkEndpoint": True,
    }


def _container_environment(document: dict[str, Any]) -> dict[str, str]:
    values = (document.get("Config") or {}).get("Env")
    if not isinstance(values, list):
        raise LoadedGateError("The container environment is unavailable")
    result: dict[str, str] = {}
    for item in values:
        if not isinstance(item, str) or "=" not in item:
            raise LoadedGateError("The container environment is malformed")
        key, value = item.split("=", 1)
        if key in result:
            raise LoadedGateError("The container environment has duplicate keys")
        result[key] = value
    return result


def _container_address(document: dict[str, Any]) -> str:
    networks = (document.get("NetworkSettings") or {}).get("Networks")
    if not isinstance(networks, dict) or len(networks) != 1:
        raise LoadedGateError("The session container network is ambiguous")
    endpoint = next(iter(networks.values()))
    if not isinstance(endpoint, dict):
        raise LoadedGateError("The session network endpoint is malformed")
    return str(endpoint.get("IPAddress") or "")


def regular_private_file(path: Path, *, maximum_bytes: int | None = None) -> os.stat_result:
    try:
        metadata = path.lstat()
    except OSError as exc:
        raise LoadedGateError("A private runtime file is unavailable") from exc
    if (
        stat.S_ISLNK(metadata.st_mode)
        or not stat.S_ISREG(metadata.st_mode)
        or metadata.st_uid != os.getuid()
        or stat.S_IMODE(metadata.st_mode) & 0o077
        or metadata.st_size <= 0
        or (maximum_bytes is not None and metadata.st_size > maximum_bytes)
    ):
        raise LoadedGateError("A private runtime file failed ownership or mode checks")
    return metadata


def viewer_runtime(
    lease_directory: Path,
    session_id: uuid.UUID,
    docker: Any,
    container: Any,
) -> ViewerRuntime:
    publisher_render = MATRIX.load_viewer_startup_evidence(
        lease_directory / "render-report.json"
    )
    probe_runtime = MATRIX.active_probe_runtime(
        lease_directory,
        session_id,
        container,
        publisher_render,
    )
    if probe_runtime.primary_environment is None:
        raise LoadedGateError("The primary viewer process was not correlated")

    container_document = docker._inspect_one("container", container.identifier)
    binding = validate_master_binding(
        _container_address(container_document),
        probe_runtime.primary_environment,
        _container_environment(container_document),
    )
    binding["viewerLeaseBound"] = True
    binding["privateDisplayBound"] = True
    return ViewerRuntime(
        command_prefix=probe_runtime.command_prefix,
        environment=probe_runtime.environment,
        gzclient=probe_runtime.gzclient,
        publisher_render=publisher_render,
        binding=binding,
    )


def copy_deployed_asset(
    docker: Any,
    container: Any,
    source: str,
    destination: Path,
    maximum_bytes: int,
) -> None:
    if destination.exists() or destination.is_symlink():
        raise LoadedGateError("A deployed preflight asset already exists in the lease")
    result = docker.run(
        ["cp", f"{container.identifier}:{source}", str(destination)],
        timeout=45,
    )
    if result.returncode != 0:
        raise LoadedGateError("Docker could not copy a deployed preflight asset")
    try:
        metadata = destination.lstat()
        if stat.S_ISLNK(metadata.st_mode) or not stat.S_ISREG(metadata.st_mode):
            raise LoadedGateError("A deployed preflight asset is not a regular file")
        if metadata.st_uid != os.getuid() or not 0 < metadata.st_size <= maximum_bytes:
            raise LoadedGateError("A deployed preflight asset failed ownership or size checks")
        os.chmod(destination, 0o600)
        regular_private_file(destination, maximum_bytes=maximum_bytes)
    except Exception:
        with contextlib.suppress(OSError):
            destination.unlink()
        raise


def start_host_command(
    arguments: Sequence[str],
    *,
    environment: dict[str, str],
    cwd: Path,
) -> BoundedChild:
    return BoundedChild(
        arguments,
        maximum_output_bytes=MAXIMUM_PREFLIGHT_OUTPUT_BYTES,
        environment=environment,
        cwd=cwd,
        umask=0o077,
    ).start()


def start_payload_command(
    arguments: Sequence[str],
) -> BoundedChild:
    return BoundedChild(
        arguments,
        maximum_output_bytes=MAXIMUM_PROTOCOL_OUTPUT_BYTES,
        line_prefixes=(
            PAYLOAD_READY_PREFIX,
            PAYLOAD_MASS_PREFIX,
            GRF_ACTIVE_PREFIX,
            PUSH_LIVE_PREFIX,
        ),
        line_streams=("stderr",),
        strict_line_channel=True,
    ).start()


def wait_for_preflight_under_load(
    preflight: BoundedChild,
    payload: BoundedChild,
    *,
    timeout: float,
    stop_event: threading.Event,
) -> Any:
    if preflight.started_at is None:
        raise LoadedGateError("The GUI preflight has no monotonic start time")
    deadline = preflight.started_at + timeout
    while not preflight.done.wait(0.05):
        if stop_event.is_set():
            with contextlib.suppress(Exception):
                preflight.stop_gracefully()
            with contextlib.suppress(Exception):
                payload.stop_gracefully()
            raise KeyboardInterrupt
        if payload.done.is_set():
            with contextlib.suppress(Exception):
                preflight.stop_gracefully()
            # Preserve an overflow/pipe failure from the loaded process when
            # that is the reason the peer ended early.
            payload.output()
            raise LoadedGateError("The loaded probe ended during the GUI preflight")
        if time.monotonic() >= deadline:
            with contextlib.suppress(Exception):
                preflight.stop_gracefully()
            with contextlib.suppress(Exception):
                payload.stop_gracefully()
            raise LoadedGateError("The visible Gazebo preflight timed out")
    try:
        return preflight.output()
    except BaseException:
        with contextlib.suppress(Exception):
            payload.stop_gracefully()
        raise


def remove_private_runtime_files(paths: Iterable[Path]) -> dict[str, bool]:
    result: dict[str, bool] = {}
    for path in paths:
        try:
            if path.exists() or path.is_symlink():
                metadata = path.lstat()
                if metadata.st_uid != os.getuid() or not stat.S_ISREG(metadata.st_mode):
                    result[path.name] = False
                    continue
                path.unlink()
            result[path.name] = not path.exists() and not path.is_symlink()
        except OSError:
            result[path.name] = False
    return result


def json_artifact(
    path: Path,
    document: dict[str, Any],
    secrets: Iterable[str],
) -> tuple[str, int]:
    safe = MATRIX.sanitize_report_value(document, secrets)
    MATRIX.assert_report_safe(safe, secrets)
    payload = (
        json.dumps(
            safe,
            ensure_ascii=False,
            indent=2,
            sort_keys=True,
            allow_nan=False,
        )
        + "\n"
    ).encode("utf-8")
    MATRIX.write_bytes_secure(path, payload)
    return hashlib.sha256(payload).hexdigest(), len(payload)


def validate_preflight_report(
    path: Path,
    attestation: Any,
) -> tuple[Any, bytes]:
    regular_private_file(path, maximum_bytes=MATRIX.MAXIMUM_RENDER_REPORT_BYTES)
    raw = MATRIX.read_owned_bounded_file(path, MATRIX.MAXIMUM_RENDER_REPORT_BYTES)
    try:
        document = MATRIX.strict_json_loads(raw.decode("utf-8"))
    except (UnicodeDecodeError, json.JSONDecodeError, ValueError) as exc:
        raise LoadedGateError("The official preflight report is malformed") from exc
    if not isinstance(document, dict):
        raise LoadedGateError("The official preflight report is not a JSON object")
    try:
        process = document.get("process") or {}
        if (
            hashlib.sha256(raw).hexdigest() != attestation.sha256
            or process.get("pid") != attestation.process_id
            or process.get("start_ticks") != attestation.process_start_ticks
        ):
            raise LoadedGateError(
                "The official preflight report is not bound to its live process"
            )
        evidence = MATRIX.validate_viewer_startup_report(document, raw)
        MATRIX.assert_report_safe(document)
    except Exception as exc:
        raise LoadedGateError(str(exc)) from exc
    return evidence, raw


def wait_for_loaded_barrier(
    child: BoundedChild,
    markers: LiveMarkerCollector,
    stop_event: threading.Event,
    timeout: float = LOADED_READY_TIMEOUT_SECONDS,
) -> tuple[TimedMarker, TimedMarker]:
    def ready_and_mass(values: LiveMarkerCollector) -> Any:
        if len(values.ready) != 1:
            return None
        ready = values.ready[0]
        loaded = [
            marker
            for marker in values.mass
            if marker.observed_at >= ready.observed_at
            and math.isclose(
                finite_number(
                    marker.document.get("payloadMassKg"), "loaded barrier mass"
                ),
                LOADED_PAYLOAD_MASS_KG,
                abs_tol=1e-6,
            )
        ]
        return (ready, loaded[0]) if loaded else None

    return wait_for_live_marker(
        child,
        markers,
        ready_and_mass,
        timeout=timeout,
        stop_event=stop_event,
        description="the verified 0.75 kg payload barrier",
    )


def wait_for_loaded_mass_after(
    child: BoundedChild,
    markers: LiveMarkerCollector,
    timestamp: float,
    stop_event: threading.Event,
) -> TimedMarker:
    def loaded_after(values: LiveMarkerCollector) -> Any:
        for marker in values.mass:
            if marker.observed_at < timestamp:
                continue
            mass = finite_number(
                marker.document.get("payloadMassKg"), "post-preflight payload mass"
            )
            if math.isclose(mass, LOADED_PAYLOAD_MASS_KG, abs_tol=1e-6):
                return marker
        return None

    return wait_for_live_marker(
        child,
        markers,
        loaded_after,
        timeout=MAXIMUM_MASS_SAMPLE_GAP_SECONDS * 2.0,
        stop_event=stop_event,
        description="a loaded mass sample after the GUI preflight",
    )


def wait_for_loaded_grf_active(
    child: BoundedChild,
    markers: LiveMarkerCollector,
    stop_event: threading.Event,
    *,
    timeout: float,
    after_timestamp: float,
    after_sequence: int,
    after_mass_sequence: int,
) -> TimedMarker:
    def fresh_active(values: LiveMarkerCollector) -> Any:
        for marker in reversed(values.active):
            sequence = LiveMarkerCollector._sequence(
                marker.document, "active GRF"
            )
            if (
                marker.observed_at > after_timestamp
                and sequence > after_sequence
                and LiveMarkerCollector._sequence(
                    {"sequence": marker.document.get("massSequence")},
                    "active GRF mass",
                ) > after_mass_sequence
            ):
                return marker
        return None

    return wait_for_live_marker(
        child,
        markers,
        fresh_active,
        timeout=timeout,
        stop_event=stop_event,
        description="the real loaded GRF N=4 task",
    )


def wait_for_loaded_grf_after(
    child: BoundedChild,
    markers: LiveMarkerCollector,
    stop_event: threading.Event,
    *,
    task_id: str,
    timestamp: float,
    after_sequence: int,
    minimum_mass_sequence: int,
) -> TimedMarker:
    def same_task_after(values: LiveMarkerCollector) -> Any:
        for marker in reversed(values.active):
            if (
                marker.observed_at > timestamp
                and marker.document.get("taskId") == task_id
                and LiveMarkerCollector._sequence(
                    marker.document, "active GRF"
                ) > after_sequence
                and LiveMarkerCollector._sequence(
                    {"sequence": marker.document.get("massSequence")},
                    "active GRF mass",
                ) >= minimum_mass_sequence
            ):
                return marker
        return None

    return wait_for_live_marker(
        child,
        markers,
        same_task_after,
        timeout=MAXIMUM_STATUS_SAMPLE_GAP_SECONDS * 2.0,
        stop_event=stop_event,
        description="a fresh loaded GRF status after the GUI preflight",
    )


def validate_loaded_preflight_overlap(
    *,
    payload_started: float,
    payload_ended: float,
    preflight_started: float,
    preflight_ended: float,
    hls_started: float,
    hls_ended: float,
    ready: TimedMarker,
    mass_markers: Sequence[TimedMarker],
    active_markers: Sequence[TimedMarker],
    correlated_task_id: str,
    preflight_document: dict[str, Any],
) -> dict[str, Any]:
    timestamps = (
        payload_started,
        payload_ended,
        preflight_started,
        preflight_ended,
        hls_started,
        hls_ended,
        ready.observed_at,
    )
    if any(not math.isfinite(value) for value in timestamps):
        raise LoadedGateError("The loaded/preflight monotonic timeline is invalid")
    if not (
        payload_started <= ready.observed_at <= preflight_started
        <= hls_started < hls_ended <= preflight_ended <= payload_ended
    ):
        raise LoadedGateError(
            "The loaded process and GUI preflight did not span the HLS interval"
        )

    render = preflight_document.get("render_measurement")
    if not isinstance(render, dict):
        raise LoadedGateError("The preflight report has no render measurement window")
    warmup = finite_number(render.get("warmup_seconds"), "preflight warmup")
    sample = finite_number(render.get("sample_seconds"), "preflight sample window")
    if not math.isclose(warmup, PREFLIGHT_WARMUP_SECONDS, abs_tol=1e-6):
        raise LoadedGateError("The GUI preflight did not use the pinned warmup window")
    if sample < PREFLIGHT_SAMPLE_SECONDS * 0.98:
        raise LoadedGateError("The GUI preflight sampling window was too short")

    preflight_duration = preflight_ended - preflight_started
    if preflight_duration < (warmup + sample) * 0.98:
        raise LoadedGateError("The preflight process did not cover its reported windows")
    process_overlap = max(
        0.0,
        min(payload_ended, preflight_ended)
        - max(payload_started, preflight_started),
    )
    if process_overlap + 0.001 < preflight_duration:
        raise LoadedGateError("The loaded process did not cover the full preflight process")
    hls_duration = hls_ended - hls_started
    if hls_duration < LOADED_PREFLIGHT_HLS_SECONDS * 0.90:
        raise LoadedGateError("The concurrent HLS measurement interval was too short")

    ordered_mass, mass_by_sequence = loaded_mass_stream(
        mass_markers, "live payload-mass marker"
    )
    before = [
        marker for marker in ordered_mass if marker.observed_at <= preflight_started
    ]
    after = [
        marker for marker in ordered_mass if marker.observed_at >= preflight_ended
    ]
    if not before or not after:
        raise LoadedGateError("Loaded mass samples do not bracket the GUI preflight")
    first = before[-1]
    last = after[0]
    continuous = [
        marker
        for marker in ordered_mass
        if first.observed_at <= marker.observed_at <= last.observed_at
    ]
    if any(
        not math.isclose(
            finite_number(
                marker.document.get("payloadMassKg"), "overlap payload mass"
            ),
            LOADED_PAYLOAD_MASS_KG,
            abs_tol=1e-6,
        )
        for marker in continuous
    ):
        raise LoadedGateError(
            "Payload mass was not continuously 0.75 kg during preflight"
        )
    gaps = [
        later.observed_at - earlier.observed_at
        for earlier, later in zip(continuous, continuous[1:])
    ]
    maximum_gap = max(gaps, default=0.0)
    if (
        preflight_started - first.observed_at > MAXIMUM_MASS_SAMPLE_GAP_SECONDS
        or last.observed_at - preflight_ended > MAXIMUM_MASS_SAMPLE_GAP_SECONDS
        or maximum_gap > MAXIMUM_MASS_SAMPLE_GAP_SECONDS
    ):
        raise LoadedGateError("Loaded mass telemetry was not continuous during preflight")

    foreign_activity = [
        marker
        for marker in active_markers
        if preflight_started - MAXIMUM_STATUS_SAMPLE_GAP_SECONDS
        <= marker.observed_at
        <= preflight_ended + MAXIMUM_STATUS_SAMPLE_GAP_SECONDS
        and marker.document.get("taskId") != correlated_task_id
    ]
    if foreign_activity:
        raise LoadedGateError("More than one loaded GRF task overlaps the preflight")
    ordered_active = ordered_marker_stream(active_markers, "active GRF marker")
    same_task = [
        marker
        for marker in ordered_active
        if marker.document.get("taskId") == correlated_task_id
    ]
    validate_progress_high_water(same_task, "loaded GRF task")
    for marker in same_task:
        require_loaded_mass_reference(
            marker, mass_by_sequence, "active loaded-GRF marker"
        )
    active_before = [
        marker for marker in same_task if marker.observed_at <= preflight_started
    ]
    active_after = [
        marker for marker in same_task if marker.observed_at >= preflight_ended
    ]
    if not active_before or not active_after:
        raise LoadedGateError("Loaded GRF status does not bracket the GUI preflight")
    active_first = active_before[-1]
    active_last = active_after[0]
    continuous_activity = [
        marker
        for marker in same_task
        if active_first.observed_at <= marker.observed_at <= active_last.observed_at
    ]
    activity_gaps = [
        later.observed_at - earlier.observed_at
        for earlier, later in zip(continuous_activity, continuous_activity[1:])
    ]
    maximum_activity_gap = max(activity_gaps, default=0.0)
    if (
        preflight_started - active_first.observed_at
        > MAXIMUM_STATUS_SAMPLE_GAP_SECONDS
        or active_last.observed_at - preflight_ended
        > MAXIMUM_STATUS_SAMPLE_GAP_SECONDS
        or maximum_activity_gap > MAXIMUM_STATUS_SAMPLE_GAP_SECONDS
    ):
        raise LoadedGateError("Loaded GRF status was not continuous during preflight")

    phase_order = {"SEARCH": 0, "APPROACH": 1, "PUSH": 2}
    previous_phase = -1
    for marker in continuous_activity:
        document = marker.document
        phase_value = phase_order[str(document["phase"])]
        if phase_value < previous_phase:
            raise LoadedGateError("Loaded GRF phase regressed during preflight")
        previous_phase = phase_value

    origin = payload_started
    return {
        "clock": "time.monotonic",
        "loadedProcessSpansCompletePreflight": True,
        "loadedMassContinuousThroughoutPreflight": True,
        "loadedGrfActiveThroughoutPreflight": True,
        "sameGrfTaskThroughoutPreflight": True,
        "exactN4RosterAndModelsThroughout": True,
        "hlsMeasuredDuringLoadedPreflight": True,
        "payloadReadyOffsetSeconds": round(ready.observed_at - origin, 6),
        "preflightStartOffsetSeconds": round(preflight_started - origin, 6),
        "preflightEndOffsetSeconds": round(preflight_ended - origin, 6),
        "hlsStartOffsetSeconds": round(hls_started - origin, 6),
        "hlsEndOffsetSeconds": round(hls_ended - origin, 6),
        "payloadEndOffsetSeconds": round(payload_ended - origin, 6),
        "preflightProcessDurationSeconds": round(preflight_duration, 6),
        "processOverlapSeconds": round(process_overlap, 6),
        "hlsOverlapSeconds": round(hls_duration, 6),
        "officialWarmupSeconds": warmup,
        "officialSampleSeconds": sample,
        "guaranteedLoadedSampleOverlapSeconds": sample,
        "loadedMassSampleCount": len(continuous),
        "maximumMassSampleGapSeconds": round(maximum_gap, 6),
        "loadedGrfStatusSampleCount": len(continuous_activity),
        "maximumGrfStatusGapSeconds": round(maximum_activity_gap, 6),
        "grfPhasesObserved": sorted(
            {str(marker.document["phase"]) for marker in continuous_activity},
            key=phase_order.__getitem__,
        ),
        "firstGrfStatusMarkerSha256": active_first.source_sha256,
        "lastGrfStatusMarkerSha256": active_last.source_sha256,
        "firstLoadedMassMarkerSha256": first.source_sha256,
        "lastLoadedMassMarkerSha256": last.source_sha256,
    }


def capture_during_loaded_push(
    *,
    ui: Any,
    child: BoundedChild,
    markers: LiveMarkerCollector,
    destination: Path,
    email: str,
    password: str,
    correlated_task_id: str,
    timeout: float,
    stop_event: threading.Event,
) -> tuple[dict[str, Any], PushCapture]:
    deadline = time.monotonic() + timeout

    def remaining(maximum: float | None = None) -> float:
        value = max(0.001, deadline - time.monotonic())
        return min(value, maximum) if maximum is not None else value

    first = wait_for_live_marker(
        child,
        markers,
        lambda values: next(
            (
                marker
                for marker in reversed(values.push)
                if marker.document.get("taskId") == correlated_task_id
            ),
            None,
        ),
        timeout=remaining(),
        stop_event=stop_event,
        description="a correlated all-four-robots PUSH phase",
    )
    hls_before = ui.require_interactive_hls()
    hls_checked_at = time.monotonic()
    task_id = str(first.document["taskId"])

    def fresh_same_task(values: LiveMarkerCollector) -> Any:
        for marker in reversed(values.push):
            if (
                marker.observed_at >= hls_checked_at
                and marker.document.get("taskId") == task_id
            ):
                return marker
        return None

    before = wait_for_live_marker(
        child,
        markers,
        fresh_same_task,
        timeout=remaining(10.0),
        stop_event=stop_event,
        description="a fresh PUSH marker before the browser capture",
    )
    if child.done.is_set():
        raise LoadedGateError("The loaded probe ended before the browser capture")
    screenshot_started = time.monotonic()
    screenshot = ui.screenshot(destination, email, password)
    screenshot_finished = time.monotonic()
    before_mass_sequence = int(before.document["massSequence"])

    def marker_after_capture(values: LiveMarkerCollector) -> Any:
        for marker in reversed(values.push):
            if (
                marker.observed_at >= screenshot_finished
                and marker.document.get("taskId") == task_id
                and int(marker.document.get("massSequence") or 0)
                > before_mass_sequence
            ):
                return marker
        return None

    after = wait_for_live_marker(
        child,
        markers,
        marker_after_capture,
        timeout=remaining(10.0),
        stop_event=stop_event,
        description="a PUSH marker after the browser capture",
    )
    if child.done.is_set():
        raise LoadedGateError("The loaded probe ended during the browser capture")
    hls_after = ui.require_interactive_hls()
    return screenshot, PushCapture(
        before=before,
        after=after,
        screenshot_started_at=screenshot_started,
        screenshot_finished_at=screenshot_finished,
        decoded_fps_before=MATRIX.decoded_hls_fps(hls_before),
        decoded_fps_after=MATRIX.decoded_hls_fps(hls_after),
    )


def validate_push_capture(
    capture: PushCapture,
    *,
    correlated_task_id: str,
    payload_started: float,
    payload_ended: float,
    mass_markers: Sequence[TimedMarker],
    push_markers: Sequence[TimedMarker] | None = None,
) -> dict[str, Any]:
    before_task = str(capture.before.document.get("taskId") or "")
    after_task = str(capture.after.document.get("taskId") or "")
    if before_task != correlated_task_id or after_task != correlated_task_id:
        raise LoadedGateError("The browser capture and final GRF result do not correlate")
    if not (
        payload_started <= capture.before.observed_at
        <= capture.screenshot_started_at
        <= capture.screenshot_finished_at
        <= capture.after.observed_at <= payload_ended
    ):
        raise LoadedGateError("Live PUSH markers did not bracket the browser capture")
    for position, decoded_fps in (
        ("before", capture.decoded_fps_before),
        ("after", capture.decoded_fps_after),
    ):
        measured_fps = finite_number(
            decoded_fps, f"decoded HLS FPS {position} the PUSH capture"
        )
        if measured_fps < MATRIX.MINIMUM_BROWSER_VIDEO_FPS:
            raise LoadedGateError(
                "Decoded HLS video fell below the browser FPS threshold "
                f"{position} the PUSH capture: {measured_fps:.3f} < "
                f"{MATRIX.MINIMUM_BROWSER_VIDEO_FPS:.3f}"
            )
    before_mass_sequence = int(capture.before.document["massSequence"])
    after_mass_sequence = int(capture.after.document["massSequence"])
    if after_mass_sequence <= before_mass_sequence:
        raise LoadedGateError("The PUSH capture was not bracketed by two fresh mass samples")
    ordered_mass, mass_by_sequence = loaded_mass_stream(
        mass_markers, "live payload-mass marker"
    )
    history = list(push_markers) if push_markers is not None else [
        capture.before,
        capture.after,
    ]
    ordered_push = ordered_marker_stream(history, "live PUSH marker")
    same_task_push = [
        marker
        for marker in ordered_push
        if marker.document.get("taskId") == correlated_task_id
    ]
    if capture.before not in same_task_push or capture.after not in same_task_push:
        raise LoadedGateError("The browser capture markers are absent from PUSH history")
    validate_progress_high_water(same_task_push, "loaded PUSH task")
    referenced_mass: dict[int, TimedMarker] = {}
    for push_marker in same_task_push:
        if not math.isclose(
            finite_number(
                push_marker.document.get("payloadMassKg"),
                "live PUSH payload mass",
            ),
            LOADED_PAYLOAD_MASS_KG,
            abs_tol=1e-6,
        ):
            raise LoadedGateError("A live PUSH marker did not report 0.75 kg")
        mass_marker = require_loaded_mass_reference(
            push_marker, mass_by_sequence, "live PUSH marker"
        )
        referenced_mass[int(push_marker.document["massSequence"])] = mass_marker
    for push_marker, mass_sequence in (
        (capture.before, before_mass_sequence),
        (capture.after, after_mass_sequence),
    ):
        if mass_sequence not in referenced_mass:
            raise LoadedGateError("A PUSH marker does not cite its fresh mass sample")
    first_mass = referenced_mass[before_mass_sequence]
    mass_during_capture = [
        marker
        for marker in ordered_mass
        if (
            first_mass.observed_at
            <= marker.observed_at
            <= capture.after.observed_at
        )
    ]
    if any(
        not math.isclose(
            finite_number(
                marker.document.get("payloadMassKg"), "PUSH capture payload mass"
            ),
            LOADED_PAYLOAD_MASS_KG,
            abs_tol=1e-6,
        )
        for marker in mass_during_capture
    ):
        raise LoadedGateError(
            "Payload mass was not continuously 0.75 kg during the PUSH capture"
        )
    before_progress = finite_number(
        capture.before.document.get("taskProgress"), "pre-capture PUSH progress"
    )
    after_progress = finite_number(
        capture.after.document.get("taskProgress"), "post-capture PUSH progress"
    )
    if (
        after_progress + TASK_PROGRESS_REGRESSION_TOLERANCE
        < before_progress
    ):
        raise LoadedGateError("The loaded task progress regressed across the capture")
    return {
        "clock": "time.monotonic",
        "correlatedWithFinalGrfTask": True,
        "loadedProcessActiveThroughout": True,
        "allFourPushersConfirmedBeforeAndAfter": True,
        "runningPushStateBeforeAndAfter": True,
        "freshMassSamplesBeforeAndAfter": True,
        "payloadMassKg": LOADED_PAYLOAD_MASS_KG,
        "robotCount": EXPECTED_ROBOT_COUNT,
        "captureStartOffsetSeconds": round(
            capture.screenshot_started_at - payload_started, 6
        ),
        "captureEndOffsetSeconds": round(
            capture.screenshot_finished_at - payload_started, 6
        ),
        "captureDurationSeconds": round(
            capture.screenshot_finished_at - capture.screenshot_started_at, 6
        ),
        "decodedHlsFpsBefore": capture.decoded_fps_before,
        "decodedHlsFpsAfter": capture.decoded_fps_after,
        "minimumAcceptedHlsFps": MATRIX.MINIMUM_BROWSER_VIDEO_FPS,
        "taskProgressBefore": before_progress,
        "taskProgressAfter": after_progress,
        "massSequenceAdvanced": True,
        "beforePushMarkerSha256": capture.before.source_sha256,
        "afterPushMarkerSha256": capture.after.source_sha256,
    }


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Run the visible loaded-payload N=4 production acceptance gate"
    )
    parser.add_argument(
        "--execute-production",
        action="store_true",
        required=True,
        help="authorize one production session, viewer and loaded-payload mutation",
    )
    parser.add_argument("--deployment-commit", required=True, help="full deployed Git SHA")
    parser.add_argument(
        "--credentials", type=Path, required=True, help="private 0600 account-A credentials"
    )
    parser.add_argument(
        "--chrome", type=Path, required=True, help="visible Windows Chrome executable"
    )
    parser.add_argument(
        "--profile-root", type=Path, required=True, help="Windows ephemeral-profile root"
    )
    parser.add_argument("--output", type=Path, required=True, help="sanitized 0600 JSON report")
    parser.add_argument("--url", default=DEFAULT_URL)
    parser.add_argument("--docker", default="docker")
    parser.add_argument("--viewer-runtime-dir", type=Path, default=MATRIX.DEFAULT_VIEWER_RUNTIME)
    parser.add_argument("--cdp-port", type=int, default=9343)
    parser.add_argument("--ready-timeout", type=float, default=480)
    parser.add_argument("--viewer-timeout", type=float, default=240)
    parser.add_argument("--preflight-timeout", type=float, default=60)
    parser.add_argument("--probe-timeout", type=float, default=1200)
    parser.add_argument("--cleanup-timeout", type=float, default=300)
    return parser


def parse_args(argv: Sequence[str] | None = None) -> argparse.Namespace:
    parser = build_parser()
    args = parser.parse_args(argv)
    if not SHA_PATTERN.fullmatch(args.deployment_commit):
        parser.error("--deployment-commit must be a full lowercase Git SHA")
    if not 1024 <= args.cdp_port <= 65535:
        parser.error("--cdp-port must be a non-privileged TCP port")
    for name in (
        "ready_timeout",
        "viewer_timeout",
        "preflight_timeout",
        "probe_timeout",
        "cleanup_timeout",
    ):
        value = getattr(args, name)
        if not math.isfinite(value) or value <= 0:
            parser.error(f"--{name.replace('_', '-')} must be greater than zero")
    minimum_preflight_timeout = (
        MATRIX.ACTIVE_PROBE_TIMEOUT_SECONDS
        + LOADED_PREFLIGHT_HLS_SECONDS
        + PREFLIGHT_SHUTDOWN_MARGIN_SECONDS
    )
    if args.preflight_timeout < minimum_preflight_timeout:
        parser.error(
            "--preflight-timeout must cover the active probe, loaded HLS "
            f"sampling and shutdown margin (at least {minimum_preflight_timeout:.1f} "
            "seconds)"
        )
    return args


def require_owned_directory(path: Path, description: str) -> None:
    try:
        metadata = path.lstat()
    except OSError as exc:
        raise LoadedGateError(f"The {description} does not exist") from exc
    if (
        stat.S_ISLNK(metadata.st_mode)
        or not stat.S_ISDIR(metadata.st_mode)
        or metadata.st_uid != os.getuid()
    ):
        raise LoadedGateError(f"The {description} is not an owned real directory")


def run_loaded_gate(args: argparse.Namespace) -> int:
    visible = MATRIX.load_visible_driver()
    expected_origin = visible.validate_site(args.url)
    if not args.chrome.is_file():
        raise LoadedGateError("The visible Chrome executable was not found")
    require_owned_directory(args.profile_root, "Chrome profile root")
    if shutil.which(args.docker) is None:
        raise LoadedGateError("The Docker executable was not found")
    MATRIX.validate_output_target(args.output)

    run_id = dt.datetime.now(dt.timezone.utc).strftime("%Y%m%dT%H%M%SZ") + f"-{os.getpid()}"
    evidence_dir = args.output.parent / f"{args.output.stem}-evidence-{run_id}"
    MATRIX.validate_secure_directory(evidence_dir)
    credentials_by_account = visible.read_credentials(args.credentials)
    credentials = credentials_by_account["A"]
    secrets = (credentials["email"], credentials["password"])
    report: dict[str, Any] = {
        "schemaVersion": 1,
        "runId": run_id,
        "startedAt": utc_now(),
        "site": MATRIX.clean_site_url(args.url),
        "deploymentCommit": args.deployment_commit,
        "account": "A",
        "gate": "loaded-transport-grf-n4",
        "robotCount": EXPECTED_ROBOT_COUNT,
        "publicControlPlane": True,
        "rendering": {
            "visibleChrome": True,
            "headless": False,
            "gpuDisabled": False,
            "minimumGazeboFps": MINIMUM_RENDER_FPS,
            "minimumRealTimeFactor": MINIMUM_REAL_TIME_FACTOR,
        },
        "evidenceDirectory": evidence_dir.name,
        "status": "running",
        "cleanup": {
            "viewerClosed": False,
            "sessionStopped": False,
            "containerAbsent": False,
            "networkAbsent": False,
            "leaseRuntimeAbsent": False,
            "viewerPublisherAbsent": False,
            "taskRosterClean": False,
            "payloadProcessAbsent": False,
            "preflightFilesRemoved": False,
            "preflightWorkspaceReleased": False,
            "browserClosed": False,
            "profileRemoved": False,
            "complete": False,
        },
        "success": False,
    }
    MATRIX.write_json_secure(args.output, report, secrets)

    stop_event = threading.Event()
    interrupted = False

    def note_signal(_number: int, _frame: Any) -> None:
        nonlocal interrupted
        interrupted = True
        stop_event.set()

    previous_int = signal.signal(signal.SIGINT, note_signal)
    previous_term = signal.signal(signal.SIGTERM, note_signal)

    profile = args.profile_root / f"robotswarm-loaded-n4-{run_id}"
    chrome = visible.OwnedChrome(
        "A", args.cdp_port, profile, run_id, args.chrome, args.url
    )
    ui: Any = None
    docker = BoundedDockerHost(args.docker, stop_event)
    session_id: uuid.UUID | None = None
    lease_directory: Path | None = None
    preflight_workspace: Path | None = None
    preflight_script: Path | None = None
    preflight_plugin: Path | None = None
    preflight_report: Path | None = None
    preflight_token: str | None = None
    preflight_child: BoundedChild | None = None
    payload_child: BoundedChild | None = None
    before_screenshot: dict[str, Any] | None = None
    during_screenshot: dict[str, Any] | None = None
    failure: str | None = None
    cleanup_failed = False
    try:
        if not visible.port_is_free(args.cdp_port):
            raise LoadedGateError("The requested Chrome debugging port is occupied")
        chrome.launch()
        ui = visible.RobotSwarmUi(chrome, expected_origin, stop_event)
        ui.navigate(args.url)
        ui.login(credentials["email"], credentials["password"])
        report["browser"] = {
            "visible": True,
            "product": chrome.product,
            "mediaCapabilities": ui.require_hls_media_capabilities(),
        }
        if ui._occupying_sessions():
            raise LoadedGateError("Account A already owns an active session")

        print("Creating one fresh N=4 production session...", flush=True)
        ui.create_session(EXPECTED_ROBOT_COUNT)
        report["session"] = ui.wait_ready(EXPECTED_ROBOT_COUNT, args.ready_timeout)
        session_id = MATRIX.require_one_session_uuid(ui)
        container, container_evidence = docker.verify_session(
            session_id, args.deployment_commit
        )
        report["container"] = container_evidence

        viewer_requested_at = time.monotonic()
        ui.request_viewer()
        lease_directory = MATRIX.active_viewer_lease_directory(
            args.viewer_runtime_dir,
            session_id,
            timeout=args.viewer_timeout,
            stop_event=stop_event,
        )
        remaining_viewer_timeout = max(
            0.001,
            args.viewer_timeout - (time.monotonic() - viewer_requested_at),
        )
        try:
            ui.wait_viewer_frame(remaining_viewer_timeout)
        except Exception:
            with contextlib.suppress(Exception):
                report["viewerStartupFailure"] = ui.viewer_startup_state()
            raise
        initial_viewer = ui.require_interactive_hls()
        report["viewer"] = {
            "transport": "HLS",
            "interactive": True,
            "decodedFpsBefore": MATRIX.decoded_hls_fps(initial_viewer),
        }
        runtime = viewer_runtime(lease_directory, session_id, docker, container)
        preflight_token = uuid.uuid4().hex
        runtime.environment[MATRIX.ACTIVE_PROBE_TOKEN_ENV] = preflight_token
        report["runtimeBinding"] = runtime.binding
        report["viewer"]["publisherStartupGazebo"] = {
            "averageFps": runtime.publisher_render.average_fps,
            "postRenderFps": runtime.publisher_render.post_render_fps,
            "realTimeFactor": runtime.publisher_render.real_time_factor,
        }
        before_screenshot = ui.screenshot(
            evidence_dir / "before-loaded-probe-browser.png",
            credentials["email"],
            credentials["password"],
        )

        preflight_workspace = MATRIX.create_active_probe_workspace()
        preflight_script = (
            preflight_workspace / "matrix-active-gui-preflight.py"
        )
        preflight_plugin = preflight_workspace / "matrix-active-gui-probe.so"
        preflight_report = lease_directory / "loaded-gui-report.json"
        copy_deployed_asset(
            docker,
            container,
            DEPLOYED_PREFLIGHT_PATH,
            preflight_script,
            MAXIMUM_PREFLIGHT_SCRIPT_BYTES,
        )
        copy_deployed_asset(
            docker,
            container,
            DEPLOYED_GUI_PLUGIN_PATH,
            preflight_plugin,
            MAXIMUM_GUI_PLUGIN_BYTES,
        )
        if preflight_report.exists() or preflight_report.is_symlink():
            raise LoadedGateError("The private preflight report path is already occupied")

        preflight_command = build_preflight_command(
            runtime,
            preflight_script,
            preflight_plugin,
            preflight_report,
        )
        payload_command = build_payload_command(
            args.docker, container.identifier
        )

        print(
            "Waiting for the real loaded GRF N=4 phase before GUI preflight...",
            flush=True,
        )
        live_markers = LiveMarkerCollector()
        payload_child = start_payload_command(payload_command)
        if payload_child.started_at is None:
            raise LoadedGateError("The loaded probe has no monotonic start time")
        payload_deadline = payload_child.started_at + args.probe_timeout
        try:
            ready_marker, _barrier_mass = wait_for_loaded_barrier(
                payload_child,
                live_markers,
                stop_event,
                timeout=min(
                    LOADED_READY_TIMEOUT_SECONDS,
                    max(0.001, payload_deadline - time.monotonic()),
                ),
            )
        except LoadedProbeEnded as exc:
            report["loadedProbeFailure"] = classify_loaded_probe_failure(
                exc.output
            )
            raise
        active_anchor_timestamp = time.monotonic()
        active_anchor_sequence = max(
            (
                LiveMarkerCollector._sequence(marker.document, "active GRF")
                for marker in live_markers.active
            ),
            default=0,
        )
        active_before = wait_for_loaded_grf_active(
            payload_child,
            live_markers,
            stop_event,
            timeout=max(0.001, payload_deadline - time.monotonic()),
            after_timestamp=active_anchor_timestamp,
            after_sequence=active_anchor_sequence,
            after_mass_sequence=LiveMarkerCollector._sequence(
                _barrier_mass.document, "loaded barrier mass"
            ),
        )
        grf_task_id = str(active_before.document["taskId"])
        print(
            "Running the visible GUI preflight during the active loaded GRF task...",
            flush=True,
        )
        preflight_child = start_host_command(
            preflight_command,
            environment=runtime.environment,
            cwd=lease_directory,
        )
        if payload_child.done.is_set() or preflight_child.done.is_set():
            raise LoadedGateError("A loaded/preflight process ended before HLS sampling")
        loaded_hls_started = time.monotonic()
        try:
            loaded_hls_raw = ui.video_metrics(LOADED_PREFLIGHT_HLS_SECONDS)
            loaded_hls_ended = time.monotonic()
            loaded_hls_video = MATRIX.validate_browser_video(
                loaded_hls_raw,
                requested_seconds=LOADED_PREFLIGHT_HLS_SECONDS,
            )
        except BaseException:
            with contextlib.suppress(Exception):
                preflight_child.stop_gracefully()
            with contextlib.suppress(Exception):
                payload_child.stop_gracefully()
            raise
        if payload_child.done.is_set() or preflight_child.done.is_set():
            raise LoadedGateError("A loaded/preflight process ended during HLS sampling")
        preflight_output = wait_for_preflight_under_load(
            preflight_child,
            payload_child,
            timeout=args.preflight_timeout,
            stop_event=stop_event,
        )
        if preflight_output.returncode != 0:
            report["preflightFailure"] = MATRIX.classify_active_probe_failure(
                preflight_output
            )
            raise LoadedGateError("The official visible Gazebo preflight failed")
        if MATRIX.active_probe_processes(preflight_token):
            raise LoadedCleanupError(
                "The visible Gazebo preflight sandbox left a live process"
            )
        if (
            preflight_child.started_at is None
            or preflight_child.ended_at is None
            or payload_child.started_at is None
            or payload_child.done.is_set()
        ):
            raise LoadedGateError("The loaded process did not span the GUI preflight")
        post_preflight_mass = wait_for_loaded_mass_after(
            payload_child,
            live_markers,
            preflight_child.ended_at,
            stop_event,
        )
        post_preflight_grf_anchor = time.monotonic()
        post_preflight_grf_sequence = max(
            (
                LiveMarkerCollector._sequence(marker.document, "active GRF")
                for marker in live_markers.active
                if marker.document.get("taskId") == grf_task_id
            ),
            default=0,
        )
        wait_for_loaded_grf_after(
            payload_child,
            live_markers,
            stop_event,
            task_id=grf_task_id,
            timestamp=post_preflight_grf_anchor,
            after_sequence=post_preflight_grf_sequence,
            minimum_mass_sequence=LiveMarkerCollector._sequence(
                post_preflight_mass.document, "post-preflight payload mass"
            ),
        )
        preflight_attestation = MATRIX.active_probe_attestation(preflight_output)
        official_render, official_render_raw = validate_preflight_report(
            preflight_report,
            preflight_attestation,
        )
        if (
            (official_render.document.get("display") or {}).get("x11")
            != runtime.environment["DISPLAY"]
        ):
            raise LoadedGateError("The official preflight used a different private display")
        if MATRIX.active_probe_processes(preflight_token):
            raise LoadedCleanupError("The temporary preflight sandbox is still running")

        during_screenshot, push_capture = capture_during_loaded_push(
            ui=ui,
            child=payload_child,
            markers=live_markers,
            destination=evidence_dir / "during-loaded-grf-push-browser.png",
            email=credentials["email"],
            password=credentials["password"],
            correlated_task_id=grf_task_id,
            timeout=max(0.001, payload_deadline - time.monotonic()),
            stop_event=stop_event,
        )
        payload_output = payload_child.wait(
            timeout=args.probe_timeout,
            stop_event=stop_event,
        )
        live_markers.ingest(payload_child)
        if payload_child.ended_at is None:
            raise LoadedGateError("The loaded probe has no monotonic completion time")

        protocol = parse_loaded_protocol(payload_output)
        validation = validate_protocol(protocol)
        correlated_task = str(protocol.grf_result.document.get("task_id") or "")
        if correlated_task != grf_task_id:
            raise LoadedGateError(
                "The preflight and final GRF result belong to different tasks"
            )
        overlap = validate_loaded_preflight_overlap(
            payload_started=payload_child.started_at,
            payload_ended=payload_child.ended_at,
            preflight_started=preflight_child.started_at,
            preflight_ended=preflight_child.ended_at,
            hls_started=loaded_hls_started,
            hls_ended=loaded_hls_ended,
            ready=ready_marker,
            mass_markers=live_markers.mass,
            active_markers=live_markers.active,
            correlated_task_id=grf_task_id,
            preflight_document=official_render.document,
        )
        loaded_hls_video.update(
            {
                "phase": "duringLoadedGrfN4AndGuiPreflight",
                "scope": "liveHlsUnderLoadedGrfN4",
                "loadedProcessActiveThroughout": True,
                "loadedGrfTaskActiveThroughout": True,
                "exactN4RosterAndModelsThroughout": True,
                "activePreflightThroughout": True,
                "visualMotionMeasured": False,
                "algorithmMotionProvenByRos": True,
            }
        )
        push_evidence = validate_push_capture(
            push_capture,
            correlated_task_id=correlated_task,
            payload_started=payload_child.started_at,
            payload_ended=payload_child.ended_at,
            mass_markers=live_markers.mass,
            push_markers=live_markers.push,
        )

        # The production lease intentionally lasts five minutes.  A loaded run can
        # outlive it after we have already sampled the stream on both sides of the
        # PUSH screenshot, so do not turn the expected lease expiry into a false
        # algorithm failure.  Keep the old summary field for report readers, but
        # tie it to the last sample that was actually taken under load.
        report["viewer"]["decodedFpsAfter"] = push_capture.decoded_fps_after
        report["viewer"]["decodedFpsAfterLoadedPushCapture"] = (
            push_capture.decoded_fps_after
        )
        with contextlib.suppress(Exception):
            report["viewer"]["postTaskLeaseState"] = ui.viewer_state()
        report["viewer"]["loadedPreflightVideo"] = loaded_hls_video
        report["viewer"]["decodedFpsDuringLoadedPush"] = {
            "beforeCapture": push_capture.decoded_fps_before,
            "afterCapture": push_capture.decoded_fps_after,
        }
        report["preflight"] = {
            "officialDeployedScript": True,
            "privateSessionRuntime": True,
            "measuredDuringLoadedProcess": True,
            "measuredDuringLoadedGrfN4": True,
            "adapter": MATRIX.redact_text(official_render.adapter),
            "nvidiaD3d12": True,
            "minimumFps": MINIMUM_RENDER_FPS,
            "averageFps": official_render.average_fps,
            "postRenderFps": official_render.post_render_fps,
            "minimumRealTimeFactor": MINIMUM_REAL_TIME_FACTOR,
            "realTimeFactor": official_render.real_time_factor,
            "temporaryGzclientStopped": True,
        }
        report["loadedPreflightOverlap"] = overlap
        report["loadedPushBrowserCapture"] = push_evidence
        report.update(validation)

        artifact_documents = {
            "load-result.json": protocol.load.document,
            "grf-result.json": protocol.grf_result.document,
            "grf-summary.json": protocol.grf_summary.document,
            "post-load-cleanup.json": protocol.post_cleanup.document,
            "loaded-preflight-overlap.json": overlap,
            "loaded-push-browser-correlation.json": push_evidence,
            "loaded-preflight-hls.json": loaded_hls_video,
        }
        hashes: dict[str, str] = {
            "loadSourceProtocolSha256": protocol.load.source_sha256,
            "grfResultSourceProtocolSha256": protocol.grf_result.source_sha256,
            "grfSummarySourceProtocolSha256": protocol.grf_summary.source_sha256,
            "postCleanupSourceProtocolSha256": protocol.post_cleanup.source_sha256,
        }
        report["artifacts"] = {}
        for name, document in artifact_documents.items():
            digest, size = json_artifact(evidence_dir / name, document, secrets)
            stem = name[:-5] if name.endswith(".json") else name
            key = stem.replace("-", " ").title().replace(" ", "")
            hashes[f"{key}Sha256"] = digest
            report["artifacts"][name] = {
                "file": name,
                "sha256": digest,
                "bytes": size,
                "sanitized": True,
            }

        MATRIX.assert_report_safe(official_render.document)
        preflight_name = "gazebo-gui-preflight.json"
        MATRIX.write_bytes_secure(evidence_dir / preflight_name, official_render_raw)
        preflight_hash = hashlib.sha256(official_render_raw).hexdigest()
        hashes["gazeboGuiPreflightSha256"] = preflight_hash
        report["artifacts"][preflight_name] = {
            "file": preflight_name,
            "sha256": preflight_hash,
            "bytes": len(official_render_raw),
            "sanitized": True,
        }

        after_screenshot_name = "after-loaded-probe-browser.png"
        after_screenshot = ui.screenshot(
            evidence_dir / after_screenshot_name,
            credentials["email"],
            credentials["password"],
        )
        screenshots = (
            ("before-loaded-probe-browser.png", before_screenshot, "beforeBrowserPngSha256"),
            (
                "during-loaded-grf-push-browser.png",
                during_screenshot,
                "duringLoadedPushBrowserPngSha256",
            ),
            (after_screenshot_name, after_screenshot, "afterBrowserPngSha256"),
        )
        for screenshot_name, screenshot, hash_name in screenshots:
            if not isinstance(screenshot, dict):
                raise LoadedGateError("A sanitized browser screenshot is unavailable")
            screenshot_hash = str(screenshot.get("sha256") or "")
            if not re.fullmatch(r"[0-9a-f]{64}", screenshot_hash):
                raise LoadedGateError("A sanitized browser screenshot has no SHA-256")
            hashes[hash_name] = screenshot_hash
            report["artifacts"][screenshot_name] = {
                "file": screenshot_name,
                "sha256": screenshot_hash,
                "bytes": int(screenshot.get("bytes") or 0),
                "sanitized": True,
            }
        hashes["evidenceBundleSha256"] = MATRIX.evidence_bundle_hash(hashes)
        report["hashes"] = hashes
        report["status"] = "passed"
    except KeyboardInterrupt:
        interrupted = True
        report["status"] = "interrupted"
        failure = "Interrupted by the operator"
    except Exception as exc:
        report["status"] = "failed"
        failure = clean_failure(exc, secrets)
        if isinstance(exc, MalformedLiveMarker):
            report["liveMarkerFailure"] = exc.diagnostic
        if isinstance(exc, LoadedProbeEnded):
            report["loadedProbeFailure"] = classify_loaded_probe_failure(exc.output)
    finally:
        bounded_children_stopped = True
        for child in (preflight_child, payload_child):
            if child is None:
                continue
            try:
                if not child.done.is_set():
                    child.stop_gracefully()
                if child.done.is_set():
                    try:
                        child.output()
                    except LoadedGateError:
                        if not child.overflow.is_set():
                            bounded_children_stopped = False
            except Exception:
                bounded_children_stopped = False
        report["cleanup"]["boundedChildProcessesStopped"] = bounded_children_stopped
        preflight_process_clean = (
            preflight_token is None
            or MATRIX.stop_active_probe_processes(preflight_token)
        )
        report["cleanup"]["preflightProcessAbsent"] = preflight_process_clean
        runtime_paths = [
            path
            for path in (preflight_report, preflight_script, preflight_plugin)
            if path is not None
        ]
        runtime_cleanup = remove_private_runtime_files(runtime_paths)
        report["cleanup"]["preflightFilesRemoved"] = all(runtime_cleanup.values())
        report["cleanup"]["preflightFileCount"] = len(runtime_cleanup)
        report["cleanup"]["preflightWorkspaceReleased"] = (
            preflight_workspace is None
            or MATRIX.remove_active_probe_workspace(preflight_workspace)
        )

        if ui is None:
            for key in (
                "viewerClosed",
                "sessionStopped",
                "workspaceReleased",
                "containerAbsent",
                "networkAbsent",
                "leaseRuntimeAbsent",
                "viewerPublisherAbsent",
            ):
                report["cleanup"][key] = True
            report["cleanup"]["sessionResourcesComplete"] = True
        else:
            try:
                session_cleanup = MATRIX.cleanup_case(
                    ui,
                    docker,
                    session_id,
                    lease_directory,
                    timeout=args.cleanup_timeout,
                )
                for key in (
                    "viewerClosed",
                    "sessionStopped",
                    "workspaceReleased",
                    "containerAbsent",
                    "networkAbsent",
                    "leaseRuntimeAbsent",
                    "viewerPublisherAbsent",
                ):
                    report["cleanup"][key] = bool(session_cleanup.get(key))
                report["cleanup"]["sessionResourcesComplete"] = bool(
                    session_cleanup.get("complete")
                )
                if not session_cleanup.get("complete"):
                    report["cleanup"]["sessionError"] = "Session resource cleanup was incomplete"
            except Exception as exc:
                report["cleanup"]["sessionError"] = clean_failure(exc, secrets)

        probe_cleanup = report.get("probeCleanup")
        official_task_roster_cleanup = isinstance(probe_cleanup, dict) and all(
            probe_cleanup.get(key) is True
            for key in ("taskTerminal", "rosterEmpty", "robotModelsAbsent")
        )
        report["cleanup"]["officialProbeTaskRosterClean"] = (
            official_task_roster_cleanup
        )
        # If the probe failed before its structured postcheck, destruction of
        # the immutable session container is the only remaining proof that its
        # task, roster and child process no longer exist.
        report["cleanup"]["taskRosterClean"] = bool(
            official_task_roster_cleanup or report["cleanup"].get("containerAbsent")
        )
        report["cleanup"]["payloadProcessAbsent"] = bool(
            report["cleanup"].get("containerAbsent")
        )

        if ui is not None:
            with contextlib.suppress(Exception):
                report.setdefault("browser", {})["clickAudit"] = ui.click_evidence()

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
                report["cleanup"]["browserError"] = clean_failure(exc, secrets)

        required_cleanup = (
            "boundedChildProcessesStopped",
            "preflightProcessAbsent",
            "preflightFilesRemoved",
            "preflightWorkspaceReleased",
            "viewerClosed",
            "sessionStopped",
            "workspaceReleased",
            "containerAbsent",
            "networkAbsent",
            "leaseRuntimeAbsent",
            "viewerPublisherAbsent",
            "taskRosterClean",
            "payloadProcessAbsent",
            "browserClosed",
            "profileRemoved",
        )
        report["cleanup"]["complete"] = all(
            report["cleanup"].get(key) is True for key in required_cleanup
        )
        cleanup_failed = not report["cleanup"]["complete"]
        if cleanup_failed:
            report["status"] = "cleanup-failed"
        report["completedAt"] = utc_now()
        report["interrupted"] = interrupted
        if failure:
            report["failure"] = failure
        report["success"] = report["status"] == "passed" and not cleanup_failed
        MATRIX.write_json_secure(args.output, report, secrets)
        signal.signal(signal.SIGINT, previous_int)
        signal.signal(signal.SIGTERM, previous_term)

    print(f"Sanitized loaded N=4 report: {clean_failure(args.output, secrets)}", flush=True)
    if interrupted:
        return 130
    if cleanup_failed:
        return 3
    return 0 if report["success"] else 1


def main(argv: Sequence[str] | None = None) -> int:
    args = parse_args(argv)
    try:
        with MATRIX.HostRunLock():
            return run_loaded_gate(args)
    except Exception as exc:
        print(
            "Loaded N=4 preflight failed: " + clean_failure(exc),
            file=sys.stderr,
        )
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
