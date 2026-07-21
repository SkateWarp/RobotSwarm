#!/usr/bin/env python3
"""Offline contract tests for the visible ROS production matrix."""

from __future__ import annotations

import argparse
import copy
import contextlib
import fcntl
import importlib.util
import io
import json
import os
import re
import stat
import subprocess
import sys
import tempfile
import threading
import time
import unittest
import uuid
from pathlib import Path
from unittest import mock


SCRIPT = Path(__file__).with_name("robotswarm-ros-matrix-e2e.py")
LIVE_RUNNER = (
    Path(__file__).parents[2]
    / "swarm_ws/src/robot_swarm_bridge/test/robotswarm_live_acceptance.py"
)


def load_driver():
    spec = importlib.util.spec_from_file_location("robotswarm_ros_matrix_e2e", SCRIPT)
    if spec is None or spec.loader is None:
        raise RuntimeError("could not load the ROS matrix driver")
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


DRIVER = load_driver()


def passing_result(scenario=None, rtf=2.95):
    scenario = scenario or DRIVER.SCENARIOS[0]
    return {
        "scenario": scenario.name,
        "behavior": scenario.behavior,
        "robot_count": scenario.robot_count,
        "task_id": f"accept-{scenario.name}-1234abcd",
        "passed": True,
        "failures": [],
        "cleanup_failures": [],
        "metrics": {"real_time_factor": rtf},
    }


def passing_n2_result():
    scenario = next(
        item for item in DRIVER.SCENARIOS
        if item.name == "transport_grf_n2"
    )
    names = ["tb3_0", "tb3_1"]
    result = passing_result(scenario)
    result.update(
        {
            "task_outcome": "completed",
            "transport_phase_timeline": {
                "transitions": [
                    {"phase": phase, "status_sequence": index}
                    for index, phase in enumerate(
                        ("SEARCH", "APPROACH", "PUSH", "DONE"), start=1
                    )
                ],
                "dropped_transitions": 0,
                "protocol_error": None,
            },
            "behavior_status": {
                "phase": "DONE",
                "synchronized_push_started": True,
                "robot_assignments": {
                    name: {
                        "role": "payload_push",
                        "parent_namespace": None,
                        "notice_received": True,
                        "rendezvous_ready": True,
                    }
                    for name in names
                },
            },
        }
    )
    result["metrics"].update(
        {
            "robot_travel_m": {name: 1.0 for name in names},
            "transport_active_push_latched": True,
            "transport_all_useful_samples": 6,
            "transport_participation": {
                name: {
                    "role": "payload_push",
                    "declared_parent_namespaces": [],
                    "useful_pushing_samples": 6,
                }
                for name in names
            },
            "transport_discovery_response": {
                "notice_observed": True,
                "finder": "tb3_0",
                "notice_recipients": names,
                "missing_notice_recipients": [],
                "missing_status_acknowledgements": [],
                "search_motion_window_supported": True,
                "peak_simultaneous_movers": 2,
                "robots_observed_moving_during_search": names,
                "robots_below_required_search_travel": [],
                "robots_below_required_travel": [],
                "robots": {
                    name: {
                        "notice_recipient": True,
                        "motion_detected": True,
                        "met_pre_push_path_requirement": True,
                    }
                    for name in names
                },
            },
        }
    )
    return result


def passing_summary():
    return {
        "selected": 1,
        "executed": 1,
        "passed": 1,
        "failed": 0,
        "all_passed": True,
        "failed_scenarios": [],
        "cleanup_passed": True,
        "cleanup_failures": [],
    }


def child_output(result=None, summary=None, returncode=0):
    result = result or passing_result()
    summary = summary or passing_summary()
    return DRIVER.ProcessOutput(
        returncode,
        "diagnostic\n"
        + "RESULT_JSON "
        + json.dumps(result, separators=(",", ":"))
        + "\nSUMMARY_JSON "
        + json.dumps(summary, separators=(",", ":"))
        + "\n",
        "[acceptance] local diagnostic\n",
    )


def startup_render_report(
    fps=49.5,
    rtf=2.96,
    renderer="D3D12 (NVIDIA GeForce RTX 3080)",
):
    return {
        "schema_version": 1,
        "process": {"pid": 42, "executable": "/usr/bin/gzclient"},
        "display": {"x11": ":120", "wayland": ""},
        "camera": {
            "name": "gzclient_camera(0)",
            "viewport_width": 1280,
            "viewport_height": 720,
        },
        "renderer": {
            "api": "OpenGL Rendering Subsystem",
            "device": renderer,
            "vendor": "Microsoft Corporation",
            "gl_vendor": "Microsoft Corporation",
            "gl_renderer": renderer,
        },
        "render_measurement": {
            "source": "gazebo::rendering::Camera::AvgFPS",
            "warmup_seconds": 2.0,
            "sample_seconds": 5.0,
            "samples": 200,
            "average_fps": fps,
            "post_render_rate_fps": fps - 0.1,
            "configured_render_rate_fps": 50.0,
        },
        "physics_measurement": {
            "source": "gazebo.msgs.WorldStatistics delta(sim_time)/delta(real_time)",
            "samples": 100,
            "real_time_factor": rtf,
        },
    }


def passing_video_metrics(fps=29.8, dropped_ratio=0.02, media_advance=5.0):
    return {
        "requestVideoFrameCallbackSupported": True,
        "getVideoPlaybackQualitySupported": True,
        "elapsedSeconds": 5.0,
        "callbackFrames": 149,
        "callbackFps": fps,
        "decodedFrames": 150,
        "decodedFps": fps + 0.2,
        "droppedFrames": 3,
        "droppedRatio": dropped_ratio,
        "mediaTimeAdvancedSeconds": media_advance,
        "readyState": 4,
        "paused": False,
        "playbackRate": 1.0,
        "width": 1280,
        "height": 720,
        "visibleFraction": 1.0,
        "sampleChunks": 3,
    }


def passing_active_video():
    evidence = DRIVER.validate_browser_video(passing_video_metrics())
    evidence.update(
        {
            "phase": "duringActiveScenarioAndGuiProbe",
            "scope": "liveHlsUnderScenarioLoad",
            "scenarioProcessActiveThroughout": True,
            "activeProbeThroughout": True,
            "visualMotionMeasured": False,
        }
    )
    return evidence


def passing_overlap(overlapped=True):
    return {
        "clock": "time.monotonic",
        "scenarioProcessActiveThroughoutProbe": overlapped,
        "scenarioProcessActiveThroughoutVideo": overlapped,
        "probeActiveThroughoutVideo": overlapped,
        "taskActiveConfirmedBeforeProbe": overlapped,
        "taskActiveConfirmedAfterProbe": overlapped,
        "taskActivityCorrelation": "independentRosTaskAndBehaviorStatus",
        "taskActivityBeforeProbe": {
            "behavior": "formation",
            "taskStatus": "running",
            "behaviorState": "moving",
            "behaviorPhase": "",
            "confirmed": True,
        },
        "taskActivityAfterProbe": {
            "behavior": "formation",
            "taskStatus": "running",
            "behaviorState": "moving",
            "behaviorPhase": "",
            "confirmed": True,
        },
        "probeDurationSeconds": 7.2,
        "scenarioProbeOverlapSeconds": 7.2 if overlapped else 4.0,
        "videoDurationSeconds": 5.0,
        "probeStartOffsetFromScenarioSeconds": 0.2,
        "probeStartAfterTaskActiveConfirmationSeconds": 0.1,
        "taskActiveConfirmationAfterProbeSeconds": 0.1,
        "probeEndBeforeScenarioSeconds": 0.4 if overlapped else -0.2,
        "videoStartOffsetFromScenarioSeconds": 0.3,
        "videoEndBeforeScenarioSeconds": 2.0 if overlapped else -0.1,
    }


class FakeUi:
    def __init__(self):
        self.stop_event = threading.Event()
        self.created_session = False
        self.create_requested = False

    def _occupying_sessions(self):
        return []

    def create_session(self, _count):
        self.create_requested = True
        raise DRIVER.MatrixError("synthetic create failure")


class RosMatrixAcceptanceContractTests(unittest.TestCase):
    def test_catalog_matches_all_fourteen_live_runner_cases_in_order(self):
        source = LIVE_RUNNER.read_text(encoding="utf-8")
        block = source.split("SCENARIOS = [", 1)[1].split("\n]\n", 1)[0]
        live = [
            (name, behavior, int(count))
            for name, behavior, count in re.findall(
                r'scenario\("([^"]+)",\s*"(formation|follow|transport)",\s*(\d+),',
                block,
                flags=re.MULTILINE,
            )
        ]
        matrix = [(item.name, item.behavior, item.robot_count) for item in DRIVER.SCENARIOS]

        self.assertEqual(14, len(matrix))
        self.assertEqual(live, matrix)
        self.assertIn(
            ("transport_grf_n2", "transport", 2),
            matrix,
        )

    def test_default_selection_is_full_and_repeated_scenario_builds_partial_matrix(self):
        self.assertEqual(list(DRIVER.SCENARIOS), DRIVER.scenario_selection(None))
        selected = DRIVER.scenario_selection(
            ["formation_triangle_n3", "transport_grf_n10"]
        )
        self.assertEqual(
            ["formation_triangle_n3", "transport_grf_n10"],
            [item.name for item in selected],
        )
        with self.assertRaises(DRIVER.MatrixError):
            DRIVER.scenario_selection(["formation_triangle_n3"] * 2)

    def test_private_session_lookup_requires_exactly_one_uuid(self):
        expected = uuid.UUID("11111111-1111-4111-8111-111111111111")
        ui = mock.Mock()
        ui._occupying_sessions.return_value = [{"id": str(expected), "state": "Ready"}]

        self.assertEqual(expected, DRIVER.require_one_session_uuid(ui))

        ui._occupying_sessions.return_value = []
        with self.assertRaises(DRIVER.MatrixError):
            DRIVER.require_one_session_uuid(ui)
        ui._occupying_sessions.return_value = [
            {"id": str(expected)},
            {"id": "22222222-2222-4222-8222-222222222222"},
        ]
        with self.assertRaises(DRIVER.MatrixError):
            DRIVER.require_one_session_uuid(ui)

    def test_cli_requires_explicit_production_authorization_and_required_paths(self):
        required = [
            "--deployment-commit",
            "a" * 40,
            "--credentials",
            "/tmp/credentials",
            "--chrome",
            "/tmp/chrome.exe",
            "--profile-root",
            "/tmp/profile",
            "--output",
            "/tmp/report.json",
        ]
        with contextlib.redirect_stderr(io.StringIO()):
            with self.assertRaises(SystemExit) as missing_authorization:
                DRIVER.parse_args(required)
        self.assertEqual(2, missing_authorization.exception.code)

        parsed = DRIVER.parse_args(
            [
                "--execute-production",
                *required,
                "--scenario",
                "formation_triangle_n3",
                "--scenario",
                "follow_square_n6",
            ]
        )
        self.assertEqual(
            ["formation_triangle_n3", "follow_square_n6"],
            [item.name for item in parsed.selected_scenarios],
        )

    def test_child_command_keeps_the_fleet_for_active_and_post_roster_evidence(self):
        command = DRIVER.build_acceptance_command(
            "docker",
            "b" * 64,
            "formation_triangle_n3",
            "matrix-" + "c" * 32,
            "c" * 32,
        )

        self.assertEqual(command[:3], ["docker", "exec", "b" * 64])
        self.assertIs(command[7], DRIVER.ACCEPTANCE_SUPERVISOR_SOURCE)
        self.assertEqual(DRIVER.LIVE_ACCEPTANCE_PATH, command[11])
        self.assertEqual(command[-8:-2], [
            "--scenario",
            "formation_triangle_n3",
            "--task-id",
            "matrix-" + "c" * 32,
            "--min-rtf",
            "2.90",
        ])
        self.assertEqual(
            command[-2:], ["--formation-active-seconds", "15.0"]
        )
        self.assertNotIn("--delete-after", command)
        self.assertIn("--task-id", command)
        self.assertIn(
            "/tmp/robotswarm-matrix-" + "c" * 32 + ".state", command
        )
        self.assertIn(
            "/tmp/robotswarm-matrix-" + "c" * 32 + ".lock", command
        )

        transport = DRIVER.build_acceptance_command(
            "docker",
            "b" * 64,
            "transport_grf_n4",
            "matrix-" + "e" * 32,
            "e" * 32,
        )
        self.assertNotIn("--formation-active-seconds", transport)

    def test_every_formation_gets_the_same_bounded_active_window(self):
        formations = [
            scenario for scenario in DRIVER.SCENARIOS
            if scenario.behavior == "formation"
        ]
        self.assertEqual(6, len(formations))
        self.assertGreaterEqual(DRIVER.MATRIX_FORMATION_ACTIVE_SECONDS, 15.0)
        self.assertGreater(
            DRIVER.MATRIX_FORMATION_ACTIVE_SECONDS,
            DRIVER.ACTIVE_PROBE_WARMUP_SECONDS
            + DRIVER.ACTIVE_PROBE_SAMPLE_SECONDS,
        )
        for index, scenario in enumerate(formations):
            token = "{:032x}".format(index + 1)
            command = DRIVER.build_acceptance_command(
                "docker",
                "b" * 64,
                scenario.name,
                "matrix-" + token,
                token,
            )
            option = command.index("--formation-active-seconds")
            self.assertEqual(
                "{:.1f}".format(DRIVER.MATRIX_FORMATION_ACTIVE_SECONDS),
                command[option + 1],
            )

    @staticmethod
    def _supervisor_call(
        mode, state, lock, task_id, expected_script, timeout=10, source=None,
    ):
        return subprocess.run(
            [
                sys.executable,
                "-c",
                source or DRIVER.ACCEPTANCE_SUPERVISOR_SOURCE,
                mode,
                str(state),
                str(lock),
                task_id,
                expected_script,
            ],
            capture_output=True,
            check=False,
            text=True,
            timeout=timeout,
        )

    @staticmethod
    def _pid_start_time(pid):
        fields = Path("/proc/{}/stat".format(pid)).read_text(
            encoding="ascii"
        ).rsplit(")", 1)[1].strip().split()
        return int(fields[19])

    @staticmethod
    def _fake_runner_command(marker, expected_script, task_id, duration="60"):
        source = (
            "import os,pathlib,sys,time; "
            "pathlib.Path(sys.argv[3]).write_text(str(os.getpid()), "
            "encoding='ascii'); time.sleep(float(sys.argv[4]))"
        )
        return [
            sys.executable,
            "-c",
            source,
            expected_script,
            task_id,
            str(marker),
            duration,
        ]

    @staticmethod
    def _supervisor_with_write_failure(status, observed_pid_path=None):
        needle = "def write_state(descriptor, value):\n"
        observation = ""
        if observed_pid_path is not None:
            observation = (
                "        with open({!r}, 'w', encoding='ascii') as stream:\n"
                "            stream.write(str(value.get('child_pid')))\n"
            ).format(str(observed_pid_path))
        replacement = needle + (
            "    if value.get('status') == {!r}:\n"
            "{}"
            "        raise OSError('synthetic state write failure')\n"
        ).format(status, observation)
        source = DRIVER.ACCEPTANCE_SUPERVISOR_SOURCE
        if source.count(needle) != 1:
            raise AssertionError("supervisor write_state fixture changed")
        return source.replace(needle, replacement, 1)

    def test_supervisor_abort_before_start_blocks_a_late_non_root_runner(self):
        if os.geteuid() == 0:
            self.skipTest("the supervisor ownership gate needs a non-root process")

        task_id = "matrix-" + "d" * 32
        expected_script = "/tmp/robotswarm-live-fixture.py"
        with tempfile.TemporaryDirectory(dir="/var/tmp") as temporary:
            root = Path(temporary)
            state = root / "runner.state"
            lock = root / "runner.lock"
            marker = root / "runner.started"
            prepared = self._supervisor_call(
                "prepare", state, lock, task_id, expected_script
            )
            self.assertEqual(0, prepared.returncode, prepared.stderr)
            self.assertEqual(0o600, stat.S_IMODE(state.stat().st_mode))
            self.assertEqual(0o600, stat.S_IMODE(lock.stat().st_mode))
            self.assertEqual(os.geteuid(), state.stat().st_uid)

            aborted = self._supervisor_call(
                "abort", state, lock, task_id, expected_script
            )
            self.assertEqual(0, aborted.returncode, aborted.stderr)
            command = [
                sys.executable,
                "-c",
                DRIVER.ACCEPTANCE_SUPERVISOR_SOURCE,
                "run",
                str(state),
                str(lock),
                task_id,
                expected_script,
                "0",
                "--",
                *self._fake_runner_command(
                    marker, expected_script, task_id, "0.1"
                ),
            ]
            late = subprocess.run(
                command, capture_output=True, check=False, text=True, timeout=5
            )
            self.assertEqual(125, late.returncode, late.stderr)
            self.assertFalse(marker.exists())

            finalized = self._supervisor_call(
                "finalize", state, lock, task_id, expected_script
            )
            self.assertEqual(0, finalized.returncode, finalized.stderr)
            self.assertFalse(state.exists())
            self.assertFalse(lock.exists())

    def test_supervisor_abort_during_delayed_spawn_prevents_the_spawn(self):
        task_id = "matrix-" + "a" * 32
        expected_script = "/tmp/robotswarm-live-fixture.py"
        with tempfile.TemporaryDirectory(dir="/var/tmp") as temporary:
            root = Path(temporary)
            state = root / "runner.state"
            lock = root / "runner.lock"
            marker = root / "runner.started"
            self.assertEqual(
                0,
                self._supervisor_call(
                    "prepare", state, lock, task_id, expected_script
                ).returncode,
            )
            running = subprocess.Popen(
                [
                    sys.executable,
                    "-c",
                    DRIVER.ACCEPTANCE_SUPERVISOR_SOURCE,
                    "run",
                    str(state),
                    str(lock),
                    task_id,
                    expected_script,
                    "0.8",
                    "--",
                    *self._fake_runner_command(
                        marker, expected_script, task_id, "60"
                    ),
                ],
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                text=True,
            )
            try:
                deadline = time.monotonic() + 3
                while time.monotonic() < deadline:
                    if state.exists():
                        document = json.loads(state.read_text(encoding="utf-8"))
                        if document.get("status") == "starting":
                            break
                    time.sleep(0.02)
                else:
                    self.fail("supervisor did not enter its prepared spawn delay")

                aborted = self._supervisor_call(
                    "abort", state, lock, task_id, expected_script
                )
                self.assertEqual(0, aborted.returncode, aborted.stderr)
                stdout, stderr = running.communicate(timeout=3)
                self.assertNotEqual(0, running.returncode, stdout + stderr)
                time.sleep(0.9)
                self.assertFalse(marker.exists())
            finally:
                if running.poll() is None:
                    running.kill()
                    running.communicate(timeout=5)
            self.assertEqual(
                0,
                self._supervisor_call(
                    "finalize", state, lock, task_id, expected_script
                ).returncode,
            )

    def test_supervisor_abort_running_and_finished_children_is_correlated(self):
        task_id = "matrix-" + "b" * 32
        expected_script = "/tmp/robotswarm-live-fixture.py"
        with tempfile.TemporaryDirectory(dir="/var/tmp") as temporary:
            root = Path(temporary)
            for index, duration in enumerate(("60", "0")):
                state = root / "runner-{}.state".format(index)
                lock = root / "runner-{}.lock".format(index)
                marker = root / "runner-{}.started".format(index)
                self.assertEqual(
                    0,
                    self._supervisor_call(
                        "prepare", state, lock, task_id, expected_script
                    ).returncode,
                )
                runner = subprocess.Popen(
                    [
                        sys.executable,
                        "-c",
                        DRIVER.ACCEPTANCE_SUPERVISOR_SOURCE,
                        "run",
                        str(state),
                        str(lock),
                        task_id,
                        expected_script,
                        "0",
                        "--",
                        *self._fake_runner_command(
                            marker, expected_script, task_id, duration
                        ),
                    ],
                    stdout=subprocess.PIPE,
                    stderr=subprocess.PIPE,
                    text=True,
                )
                try:
                    deadline = time.monotonic() + 3
                    while not marker.exists() and time.monotonic() < deadline:
                        time.sleep(0.02)
                    self.assertTrue(marker.exists())
                    if duration == "0":
                        runner.communicate(timeout=3)
                    aborted = self._supervisor_call(
                        "abort", state, lock, task_id, expected_script
                    )
                    self.assertEqual(0, aborted.returncode, aborted.stderr)
                    proof = json.loads(
                        aborted.stdout.split("SUPERVISOR_JSON ", 1)[1]
                    )
                    self.assertEqual(
                        "running" if duration == "60" else "finished",
                        proof["prior_state"],
                    )
                    runner.communicate(timeout=5)
                    child_pid = int(marker.read_text(encoding="ascii"))
                    with self.assertRaises(ProcessLookupError):
                        os.kill(child_pid, 0)
                finally:
                    if runner.poll() is None:
                        runner.kill()
                        runner.communicate(timeout=5)
                self.assertEqual(
                    0,
                    self._supervisor_call(
                        "finalize", state, lock, task_id, expected_script
                    ).returncode,
                )

    def test_supervisor_kills_child_when_running_state_cannot_be_persisted(self):
        task_id = "matrix-" + "1" * 32
        expected_script = "/tmp/robotswarm-live-fixture.py"
        with tempfile.TemporaryDirectory(dir="/var/tmp") as temporary:
            root = Path(temporary)
            state = root / "runner.state"
            lock = root / "runner.lock"
            marker = root / "runner.started"
            observed_pid = root / "runner.observed-pid"
            faulty_source = self._supervisor_with_write_failure(
                "running", observed_pid
            )
            self.assertEqual(
                0,
                self._supervisor_call(
                    "prepare", state, lock, task_id, expected_script,
                    source=faulty_source,
                ).returncode,
            )
            runner = subprocess.run(
                [
                    sys.executable,
                    "-c",
                    faulty_source,
                    "run",
                    str(state),
                    str(lock),
                    task_id,
                    expected_script,
                    "0",
                    "--",
                    *self._fake_runner_command(
                        marker, expected_script, task_id, "60"
                    ),
                ],
                capture_output=True,
                check=False,
                text=True,
                timeout=8,
            )
            self.assertNotEqual(0, runner.returncode)
            self.assertTrue(observed_pid.exists())
            child_pid = int(observed_pid.read_text(encoding="ascii"))
            with self.assertRaises(ProcessLookupError):
                os.kill(child_pid, 0)
            document = json.loads(state.read_text(encoding="utf-8"))
            self.assertEqual("failed", document["status"])
            self.assertIsNone(document["child_pid"])
            self.assertEqual(
                0,
                self._supervisor_call(
                    "finalize", state, lock, task_id, expected_script
                ).returncode,
            )

    def test_abort_kills_child_even_when_aborting_tombstone_write_fails(self):
        task_id = "matrix-" + "0" * 32
        expected_script = "/tmp/robotswarm-live-fixture.py"
        faulty_abort = self._supervisor_with_write_failure("aborting")
        with tempfile.TemporaryDirectory(dir="/var/tmp") as temporary:
            root = Path(temporary)
            state = root / "runner.state"
            lock = root / "runner.lock"
            marker = root / "runner.started"
            self.assertEqual(
                0,
                self._supervisor_call(
                    "prepare", state, lock, task_id, expected_script
                ).returncode,
            )
            supervisor = subprocess.Popen(
                [
                    sys.executable,
                    "-c",
                    DRIVER.ACCEPTANCE_SUPERVISOR_SOURCE,
                    "run",
                    str(state),
                    str(lock),
                    task_id,
                    expected_script,
                    "0",
                    "--",
                    *self._fake_runner_command(
                        marker, expected_script, task_id, "60"
                    ),
                ],
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                text=True,
            )
            try:
                deadline = time.monotonic() + 3
                while not marker.exists() and time.monotonic() < deadline:
                    time.sleep(0.02)
                self.assertTrue(marker.exists())
                child_pid = int(marker.read_text(encoding="ascii"))
                aborted = self._supervisor_call(
                    "abort",
                    state,
                    lock,
                    task_id,
                    expected_script,
                    timeout=8,
                    source=faulty_abort,
                )
                self.assertEqual(0, aborted.returncode, aborted.stderr)
                supervisor.communicate(timeout=5)
                with self.assertRaises(ProcessLookupError):
                    os.kill(child_pid, 0)
                document = json.loads(state.read_text(encoding="utf-8"))
                self.assertEqual("aborted", document["status"])
                self.assertIsNone(document["child_pid"])
            finally:
                if supervisor.poll() is None:
                    supervisor.kill()
                    supervisor.communicate(timeout=5)
            self.assertEqual(
                0,
                self._supervisor_call(
                    "finalize", state, lock, task_id, expected_script
                ).returncode,
            )

    def test_supervisor_rejects_bad_mode_task_and_unrelated_cmdline(self):
        task_id = "matrix-" + "f" * 32
        expected_script = "/tmp/robotswarm-live-fixture.py"
        with tempfile.TemporaryDirectory(dir="/var/tmp") as temporary:
            root = Path(temporary)
            state = root / "runner.state"
            lock = root / "runner.lock"
            self.assertEqual(
                0,
                self._supervisor_call(
                    "prepare", state, lock, task_id, expected_script
                ).returncode,
            )
            state.chmod(0o644)
            rejected_mode = self._supervisor_call(
                "abort", state, lock, task_id, expected_script
            )
            self.assertEqual(4, rejected_mode.returncode)
            state.chmod(0o600)
            lock.chmod(0o644)
            rejected_lock_mode = self._supervisor_call(
                "abort", state, lock, task_id, expected_script
            )
            self.assertEqual(4, rejected_lock_mode.returncode)
            lock.chmod(0o600)
            wrong_task = self._supervisor_call(
                "abort", state, lock, "matrix-" + "0" * 32, expected_script
            )
            self.assertEqual(4, wrong_task.returncode)

            unrelated = subprocess.Popen(
                [sys.executable, "-c", "import time; time.sleep(60)"],
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL,
            )
            fake_supervisor = subprocess.Popen(
                [
                    sys.executable,
                    "-c",
                    "import time; time.sleep(60)",
                    expected_script,
                    task_id,
                ],
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL,
            )
            try:
                with lock.open("r+") as lock_stream:
                    fcntl.flock(lock_stream, fcntl.LOCK_EX)
                    document = json.loads(state.read_text(encoding="utf-8"))
                    document.update({
                        "status": "running",
                        "supervisor_pid": fake_supervisor.pid,
                        "supervisor_start_time": self._pid_start_time(
                            fake_supervisor.pid
                        ),
                        "child_pid": unrelated.pid,
                        "child_start_time": self._pid_start_time(
                            unrelated.pid
                        ),
                    })
                    state.write_text(
                        json.dumps(document, separators=(",", ":")),
                        encoding="utf-8",
                    )
                    state.chmod(0o600)
                    fcntl.flock(lock_stream, fcntl.LOCK_UN)
                rejected_process = self._supervisor_call(
                    "abort", state, lock, task_id, expected_script
                )
                self.assertEqual(4, rejected_process.returncode)
                self.assertIsNone(unrelated.poll())
                self.assertIsNone(fake_supervisor.poll())
            finally:
                unrelated.terminate()
                unrelated.wait(timeout=5)
                fake_supervisor.terminate()
                fake_supervisor.wait(timeout=5)

            with lock.open("r+") as lock_stream:
                fcntl.flock(lock_stream, fcntl.LOCK_EX)
                document = json.loads(state.read_text(encoding="utf-8"))
                document.update({
                    "status": "prepared",
                    "supervisor_pid": None,
                    "supervisor_start_time": None,
                    "child_pid": None,
                    "child_start_time": None,
                })
                state.write_text(
                    json.dumps(document, separators=(",", ":")),
                    encoding="utf-8",
                )
                state.chmod(0o600)
                fcntl.flock(lock_stream, fcntl.LOCK_UN)
            self.assertEqual(
                0,
                self._supervisor_call(
                    "abort", state, lock, task_id, expected_script
                ).returncode,
            )
            self.assertEqual(
                0,
                self._supervisor_call(
                    "finalize", state, lock, task_id, expected_script
                ).returncode,
            )

    @staticmethod
    def _run_stop_task_fixture(mode, timeout):
        fixture = r'''
import json
import sys
import threading
import time
import types

probe_source, mode, task_id, timeout = sys.argv[1:5]
callbacks = []

class String:
    def __init__(self, data=""):
        self.data = data

def emit(task):
    message = String(json.dumps({"task": task}))
    for callback in list(callbacks):
        callback(message)

class Publisher:
    def __init__(self, *_args, **_kwargs):
        pass

    def get_num_connections(self):
        return 1

    def publish(self, _message):
        if mode == "terminal":
            emit({"task_id": task_id, "status": "stopped"})
        elif mode == "fresh-idle":
            def publish_fresh_samples():
                for delay in (0.05, 0.35, 0.65):
                    time.sleep(delay if delay == 0.05 else 0.30)
                    emit({"task_id": "", "status": "idle"})
            threading.Thread(
                target=publish_fresh_samples, daemon=True
            ).start()

def Subscriber(_topic, _kind, callback, **_kwargs):
    callbacks.append(callback)
    callback(String(json.dumps({
        "task": {"task_id": "", "status": "idle"}
    })))
    return object()

rospy = types.ModuleType("rospy")
rospy.init_node = lambda *_args, **_kwargs: None
rospy.Publisher = Publisher
rospy.Subscriber = Subscriber
rospy.is_shutdown = lambda: False
std_msgs = types.ModuleType("std_msgs")
std_msgs_msg = types.ModuleType("std_msgs.msg")
std_msgs_msg.String = String
std_msgs.msg = std_msgs_msg
sys.modules["rospy"] = rospy
sys.modules["std_msgs"] = std_msgs
sys.modules["std_msgs.msg"] = std_msgs_msg
sys.argv = ["stop-probe", task_id, timeout]
exec(compile(probe_source, "<stop-probe>", "exec"), {"__name__": "__main__"})
'''
        return subprocess.run(
            [
                sys.executable,
                "-c",
                fixture,
                DRIVER.STOP_TASK_SOURCE,
                mode,
                "matrix-" + "9" * 32,
                str(timeout),
            ],
            capture_output=True,
            check=False,
            text=True,
            timeout=max(3, timeout + 2),
        )

    def test_stop_probe_rejects_one_old_idle_and_requires_fresh_evidence(self):
        stale = self._run_stop_task_fixture("stale-idle", 0.3)
        self.assertEqual(2, stale.returncode, stale.stderr)
        self.assertNotIn("STOP_TASK_JSON", stale.stdout)

        terminal = self._run_stop_task_fixture("terminal", 1.0)
        self.assertEqual(0, terminal.returncode, terminal.stderr)
        self.assertIn('"mode":"correlated_terminal"', terminal.stdout)

        fresh = self._run_stop_task_fixture("fresh-idle", 2.0)
        self.assertEqual(0, fresh.returncode, fresh.stderr)
        self.assertIn('"mode":"fresh_quiescence"', fresh.stdout)
        proof = json.loads(fresh.stdout.split("STOP_TASK_JSON ", 1)[1])
        self.assertGreaterEqual(proof["post_publish_samples"], 3)

    def test_activity_probe_requires_a_complete_low_error_moving_formation(self):
        command = DRIVER.build_task_activity_probe_command(
            "docker",
            "c" * 64,
            "matrix-" + "8" * 32,
            "formation",
            7,
            3.0,
        )
        self.assertEqual("7", command[-2])
        self.assertEqual("3.000", command[-1])
        source = DRIVER.TASK_ACTIVITY_PROBE_SOURCE
        for requirement in (
            'value.get("state") == "moving"',
            'value.get("movement_mode") == "moving"',
            'len(assignments) == expected_count',
            'maximum_error <= 0.12',
            'not value.get("error")',
        ):
            self.assertIn(requirement, source)

    def test_abort_acceptance_finalizes_state_and_validates_stop_protocol(self):
        docker = DRIVER.DockerHost("docker", threading.Event())
        container = DRIVER.ContainerHandle("c" * 64, "version")
        task_id = "matrix-" + "7" * 32
        token = "7" * 32
        stop = DRIVER.ProcessOutput(
            0,
            'STOP_TASK_JSON {"confirmed":true,"mode":'
            '"fresh_quiescence","post_publish_samples":3}\n',
            "",
        )
        absent = DRIVER.ProcessOutput(0, "", "")
        operations = []

        def operation(_container, _task, _token, mode, **_kwargs):
            operations.append(mode)
            if mode == "abort":
                return {
                    "confirmed": True,
                    "late_start_blocked": True,
                    "prior_state": "prepared",
                }
            return {"confirmed": True, "state": "removed"}

        with mock.patch.object(
            docker, "_run_supervisor_operation", side_effect=operation
        ), mock.patch.object(
            DRIVER, "run_command", side_effect=[stop, absent, absent]
        ):
            docker.abort_acceptance(container, task_id, token)

        self.assertEqual(["abort", "finalize"], operations)

    def test_running_child_requires_a_correlated_terminal_stop(self):
        docker = DRIVER.DockerHost("docker", threading.Event())
        container = DRIVER.ContainerHandle("c" * 64, "version")
        stop = DRIVER.ProcessOutput(
            0,
            'STOP_TASK_JSON {"confirmed":true,"mode":'
            '"fresh_quiescence","post_publish_samples":3}\n',
            "",
        )
        absent = DRIVER.ProcessOutput(0, "", "")

        def operation(_container, _task, _token, mode, **_kwargs):
            if mode == "abort":
                return {
                    "confirmed": True,
                    "late_start_blocked": True,
                    "prior_state": "running",
                }
            return {"confirmed": True, "state": "removed"}

        with mock.patch.object(
            docker, "_run_supervisor_operation", side_effect=operation
        ), mock.patch.object(
            DRIVER, "run_command", side_effect=[stop, absent, absent]
        ):
            with self.assertRaises(DRIVER.CleanupError):
                docker.abort_acceptance(
                    container, "matrix-" + "2" * 32, "2" * 32
                )

    def test_abort_acceptance_still_attempts_finalize_after_bad_stop_evidence(self):
        docker = DRIVER.DockerHost("docker", threading.Event())
        container = DRIVER.ContainerHandle("c" * 64, "version")
        operations = []

        def operation(_container, _task, _token, mode, **_kwargs):
            operations.append(mode)
            if mode == "abort":
                return {
                    "confirmed": True,
                    "late_start_blocked": True,
                    "prior_state": "prepared",
                }
            return {"confirmed": True, "state": "removed"}

        with mock.patch.object(
            docker, "_run_supervisor_operation", side_effect=operation
        ), mock.patch.object(
            DRIVER,
            "run_command",
            side_effect=[
                DRIVER.ProcessOutput(0, "", ""),
                DRIVER.ProcessOutput(0, "", ""),
                DRIVER.ProcessOutput(0, "", ""),
            ],
        ):
            with self.assertRaises(DRIVER.CleanupError):
                docker.abort_acceptance(
                    container, "matrix-" + "6" * 32, "6" * 32
                )
        self.assertEqual(["abort", "finalize"], operations)

    def test_run_acceptance_prepares_before_launch_and_finalizes_after_exit(self):
        docker = DRIVER.DockerHost("docker", threading.Event())
        container = DRIVER.ContainerHandle("c" * 64, "version")
        scenario = DRIVER.SCENARIOS[0]
        events = []

        def operation(_container, _task, _token, mode, **_kwargs):
            events.append(mode)
            return {
                "confirmed": True,
                "state": "prepared" if mode == "prepare" else "removed",
            }

        def launch_runner(*_args, **_kwargs):
            events.append("runner-docker-exec")
            return child_output()

        with mock.patch.object(
            docker, "_run_supervisor_operation", side_effect=operation
        ), mock.patch.object(
            DRIVER, "run_command", side_effect=launch_runner
        ) as launch, mock.patch.object(
            docker, "wait_acceptance_supervisor_absent", return_value=True
        ):
            output = docker.run_acceptance(
                container,
                scenario,
                5,
                "matrix-" + "5" * 32,
                "5" * 32,
            )

        self.assertEqual(0, output.returncode)
        self.assertEqual(
            ["prepare", "runner-docker-exec", "finalize"], events
        )
        self.assertEqual("docker", launch.call_args.args[0][0])
        self.assertIn(DRIVER.ACCEPTANCE_SUPERVISOR_SOURCE, launch.call_args.args[0])

    def test_run_acceptance_aborts_a_prepared_handshake_after_launch_failure(self):
        docker = DRIVER.DockerHost("docker", threading.Event())
        container = DRIVER.ContainerHandle("c" * 64, "version")
        scenario = DRIVER.SCENARIOS[0]
        task_id = "matrix-" + "4" * 32
        token = "4" * 32

        with mock.patch.object(
            docker,
            "_run_supervisor_operation",
            return_value={"confirmed": True, "state": "prepared"},
        ), mock.patch.object(
            DRIVER,
            "run_command",
            side_effect=DRIVER.MatrixError("synthetic launch failure"),
        ), mock.patch.object(docker, "abort_acceptance") as abort:
            with self.assertRaisesRegex(DRIVER.MatrixError, "launch failure"):
                docker.run_acceptance(
                    container, scenario, 5, task_id, token
                )

        abort.assert_called_once_with(container, task_id, token)

    def test_run_acceptance_never_launches_after_an_early_peer_abort(self):
        docker = DRIVER.DockerHost("docker", threading.Event())
        container = DRIVER.ContainerHandle("c" * 64, "version")
        scenario = DRIVER.SCENARIOS[0]
        task_id = "matrix-" + "3" * 32
        token = "3" * 32
        cancelled = threading.Event()
        cancelled.set()

        with mock.patch.object(
            docker,
            "_run_supervisor_operation",
            return_value={"confirmed": True, "state": "prepared"},
        ), mock.patch.object(DRIVER, "run_command") as launch, mock.patch.object(
            docker, "abort_acceptance"
        ) as abort:
            with self.assertRaisesRegex(DRIVER.MatrixError, "before launch"):
                docker.run_acceptance(
                    container,
                    scenario,
                    5,
                    task_id,
                    token,
                    cancel_event=cancelled,
                )

        launch.assert_not_called()
        abort.assert_called_once_with(container, task_id, token)

    def test_ros_protocol_requires_one_result_and_one_matching_summary(self):
        scenario = DRIVER.SCENARIOS[0]
        evidence = DRIVER.parse_ros_protocol(child_output(), scenario)

        self.assertGreaterEqual(DRIVER.validate_ros_evidence(evidence), 2.90)
        self.assertRegex(evidence.result_sha256, r"^[0-9a-f]{64}$")
        self.assertRegex(evidence.summary_sha256, r"^[0-9a-f]{64}$")

        duplicate = child_output()
        duplicate = DRIVER.ProcessOutput(
            duplicate.returncode,
            duplicate.stdout + duplicate.stdout,
            duplicate.stderr,
        )
        with self.assertRaises(DRIVER.MatrixError):
            DRIVER.parse_ros_protocol(duplicate, scenario)

        reversed_protocol = DRIVER.ProcessOutput(
            0,
            "SUMMARY_JSON "
            + json.dumps(passing_summary())
            + "\nRESULT_JSON "
            + json.dumps(passing_result())
            + "\n",
            "",
        )
        with self.assertRaises(DRIVER.MatrixError):
            DRIVER.parse_ros_protocol(reversed_protocol, scenario)

        malformed = DRIVER.ProcessOutput(
            0,
            "RESULT_JSON {\"scenario\":NaN}\n"
            + "SUMMARY_JSON "
            + json.dumps(passing_summary())
            + "\n",
            "",
        )
        with self.assertRaises(DRIVER.MatrixError):
            DRIVER.parse_ros_protocol(malformed, scenario)

    def test_ros_protocol_rejects_low_rtf_and_exit_summary_disagreement(self):
        low = DRIVER.parse_ros_protocol(
            child_output(result=passing_result(rtf=2.89)), DRIVER.SCENARIOS[0]
        )
        with self.assertRaisesRegex(DRIVER.MatrixError, "below 2.90"):
            DRIVER.validate_ros_evidence(low)

        bad_exit = DRIVER.parse_ros_protocol(
            child_output(returncode=1), DRIVER.SCENARIOS[0]
        )
        with self.assertRaises(DRIVER.MatrixError):
            DRIVER.validate_ros_evidence(bad_exit)

    def test_transport_n2_contract_proves_both_roots_across_every_phase(self):
        result = passing_n2_result()

        evidence = DRIVER.validate_transport_n2_contract(result)

        self.assertTrue(evidence["exactRoster"])
        self.assertEqual(
            ["SEARCH", "APPROACH", "PUSH", "DONE"],
            evidence["phaseSequence"],
        )
        self.assertEqual(2, evidence["noticeRecipientCount"])
        self.assertEqual(2, evidence["rendezvousRobotCount"])
        self.assertEqual(2, evidence["payloadRootCount"])
        self.assertEqual(0, evidence["companionPusherCount"])
        self.assertEqual(2, evidence["usefulPusherCount"])

        mutations = {
            "roster": lambda item: item["metrics"]["robot_travel_m"].pop("tb3_1"),
            "search": lambda item: item["metrics"][
                "transport_discovery_response"
            ].update({"peak_simultaneous_movers": 1}),
            "notice": lambda item: item["metrics"][
                "transport_discovery_response"
            ].update({"notice_recipients": ["tb3_0"]}),
            "rendezvous": lambda item: item["behavior_status"][
                "robot_assignments"
            ]["tb3_1"].update({"rendezvous_ready": False}),
            "role": lambda item: item["metrics"]["transport_participation"][
                "tb3_1"
            ].update({"role": "companion_push"}),
            "push": lambda item: item["metrics"].update(
                {"transport_all_useful_samples": 0}
            ),
        }
        for name, mutate in mutations.items():
            with self.subTest(name=name):
                incomplete = copy.deepcopy(result)
                mutate(incomplete)
                with self.assertRaises(DRIVER.MatrixError):
                    DRIVER.validate_transport_n2_contract(incomplete)

    def test_startup_report_is_only_the_gpu_scene_capability_gate(self):
        document = startup_render_report()
        raw = json.dumps(document).encode()
        evidence = DRIVER.validate_viewer_startup_report(document, raw)

        self.assertIsInstance(evidence, DRIVER.GazeboGuiEvidence)
        self.assertGreaterEqual(evidence.average_fps, 45.0)
        self.assertGreaterEqual(evidence.post_render_fps, 45.0)
        self.assertGreaterEqual(evidence.real_time_factor, 2.90)
        self.assertIn("NVIDIA", evidence.adapter)

        with self.assertRaisesRegex(DRIVER.MatrixError, "below 45 FPS"):
            DRIVER.validate_viewer_startup_report(
                startup_render_report(fps=44.9), raw
            )
        with self.assertRaisesRegex(DRIVER.MatrixError, "below 2.90"):
            DRIVER.validate_viewer_startup_report(
                startup_render_report(rtf=2.89), raw
            )
        with self.assertRaisesRegex(DRIVER.MatrixError, "NVIDIA"):
            DRIVER.validate_viewer_startup_report(
                startup_render_report(renderer="D3D12 (AMD Radeon)"), raw
            )

    def test_active_report_requires_full_window_and_the_bound_private_display(self):
        document = startup_render_report()
        with tempfile.TemporaryDirectory(dir="/var/tmp") as temporary:
            root = Path(temporary)
            root.chmod(0o700)
            report = root / "active-report.json"
            report.write_text(json.dumps(document), encoding="utf-8")
            report.chmod(0o600)

            evidence = DRIVER.load_active_probe_evidence(report, ":120")
            self.assertGreaterEqual(evidence.average_fps, 45.0)
            with self.assertRaisesRegex(DRIVER.MatrixError, "different private display"):
                DRIVER.load_active_probe_evidence(report, ":121")

            document["render_measurement"]["sample_seconds"] = 4.0
            report.write_text(json.dumps(document), encoding="utf-8")
            report.chmod(0o600)
            with self.assertRaisesRegex(DRIVER.MatrixError, "sampling window"):
                DRIVER.load_active_probe_evidence(report, ":120")

    def test_browser_viewer_must_be_live_hls_with_numeric_decoded_fps(self):
        state = {
            "hlsInteractive": True,
            "status": "En vivo",
            "fps": "Video 29.8 FPS",
        }
        self.assertEqual(29.8, DRIVER.decoded_hls_fps(state))

        with self.assertRaises(DRIVER.MatrixError):
            DRIVER.decoded_hls_fps({**state, "hlsInteractive": False})
        with self.assertRaises(DRIVER.MatrixError):
            DRIVER.decoded_hls_fps({**state, "fps": "Video sin métrica"})

    def test_browser_video_requires_real_progress_and_bounded_drops(self):
        evidence = DRIVER.validate_browser_video(passing_video_metrics())

        self.assertGreaterEqual(evidence["decodedFps"], 27.0)
        self.assertLessEqual(evidence["droppedRatio"], 0.10)
        self.assertTrue(evidence["requestVideoFrameCallback"])
        self.assertTrue(evidence["getVideoPlaybackQuality"])

        bad_quality = {
            **passing_video_metrics(),
            "getVideoPlaybackQualitySupported": False,
        }
        with self.assertRaisesRegex(DRIVER.MatrixError, "dropped HLS frames"):
            DRIVER.validate_browser_video(bad_quality)
        with self.assertRaisesRegex(DRIVER.MatrixError, "did not keep progressing"):
            DRIVER.validate_browser_video(
                passing_video_metrics(media_advance=2.0)
            )
        with self.assertRaisesRegex(DRIVER.MatrixError, "30 FPS target"):
            DRIVER.validate_browser_video(passing_video_metrics(fps=26.9))
        with self.assertRaisesRegex(DRIVER.MatrixError, "dropped-frame bound"):
            DRIVER.validate_browser_video(
                passing_video_metrics(dropped_ratio=0.101)
            )

    def test_active_probe_command_uses_official_thresholds_and_private_sandbox(self):
        runtime = DRIVER.ActiveProbeRuntime(
            ("/usr/bin/bwrap", "--share-net", "--bind", "/private", "/viewer", "--"),
            {
                "DISPLAY": ":120",
                "ROS_MASTER_URI": "http://172.20.0.4:11311",
                "GAZEBO_MASTER_URI": "http://172.20.0.4:11345",
            },
            "/usr/bin/gzclient",
        )
        command = DRIVER.build_active_probe_command(
            runtime,
            Path("/private/preflight.py"),
            Path("/private/probe.so"),
            Path("/private/report.json"),
        )

        self.assertEqual("/usr/bin/bwrap", command[0])
        self.assertIn("/viewer/preflight.py", command)
        self.assertIn("/viewer/probe.so", command)
        self.assertIn("/viewer/report.json", command)
        self.assertEqual("45.0", command[command.index("--min-render-fps") + 1])
        self.assertEqual("2.90", command[command.index("--min-real-time-factor") + 1])
        self.assertEqual("5.0", command[command.index("--sample-seconds") + 1])

    def test_active_probe_runtime_binds_verified_display_master_gateway_and_assets(self):
        session_id = uuid.UUID("11111111-1111-4111-8111-111111111111")
        document = startup_render_report()
        raw = json.dumps(document).encode("utf-8")
        startup = DRIVER.validate_viewer_startup_report(document, raw)
        container = DRIVER.ContainerHandle(
            "c" * 64, "version", "172.20.0.4", "172.20.0.1"
        )
        with tempfile.TemporaryDirectory(dir="/var/tmp") as temporary:
            root = Path(temporary)
            lease = root / f"lease-{session_id.hex}"
            lease.mkdir(mode=0o700)
            authority = lease / "Xauthority"
            authority.write_bytes(b"cookie")
            authority.chmod(0o600)
            publisher = root / "robotswarm-viewer-publisher"
            publisher.write_text("#!/usr/bin/env python3\n", encoding="utf-8")
            publisher.chmod(0o700)
            assets = root / "assets"
            (assets / "ros-share").mkdir(parents=True)
            (assets / "models").mkdir()
            environment = {
                "ROBOTSWARM_VIEWER_ASSET_ROOT": str(assets),
                "MESA_D3D12_DEFAULT_ADAPTER_NAME": "NVIDIA",
            }

            with mock.patch.object(
                DRIVER,
                "bound_viewer_publisher",
                return_value=(publisher, environment),
            ), mock.patch.object(
                DRIVER,
                "_configured_executable",
                side_effect=["/usr/bin/bwrap", "/usr/bin/gzclient"],
            ):
                runtime = DRIVER.active_probe_runtime(
                    lease,
                    session_id,
                    container,
                    startup,
                )

        self.assertEqual(":120", runtime.environment["DISPLAY"])
        self.assertEqual("/viewer/Xauthority", runtime.environment["XAUTHORITY"])
        self.assertEqual(
            "http://172.20.0.4:11311", runtime.environment["ROS_MASTER_URI"]
        )
        self.assertEqual(
            "http://172.20.0.4:11345", runtime.environment["GAZEBO_MASTER_URI"]
        )
        self.assertEqual("172.20.0.1", runtime.environment["GAZEBO_IP"])
        self.assertIn("--unshare-all", runtime.command_prefix)
        self.assertIn("--share-net", runtime.command_prefix)

    def test_active_gate_rejects_any_non_overlapping_scenario(self):
        DRIVER.validate_active_overlap(passing_overlap())
        with self.assertRaisesRegex(DRIVER.MatrixError, "complete active sampling"):
            DRIVER.validate_active_overlap(passing_overlap(overlapped=False))

    def test_case_orders_startup_active_probe_and_post_scenario_roster(self):
        scenario = DRIVER.SCENARIOS[0]
        session_id = uuid.UUID("11111111-1111-4111-8111-111111111111")
        events = []
        ui = mock.Mock()
        ui._occupying_sessions.side_effect = [[], [{"id": str(session_id)}]]
        ui.wait_ready.return_value = {
            "state": "Ready",
            "activeRobots": scenario.robot_count,
        }
        ui.require_interactive_hls.return_value = {
            "hlsInteractive": True,
            "status": "En vivo",
            "fps": "Video 29.8 FPS",
        }
        ui.screenshot.return_value = {
            "file": "browser.png",
            "sha256": "e" * 64,
            "bytes": 1024,
        }
        docker = mock.Mock()
        container = DRIVER.ContainerHandle(
            "c" * 64, "version", "172.20.0.4", "172.20.0.1"
        )
        docker.verify_session.return_value = (container, {"managed": True})
        docker.verify_full_roster.side_effect = lambda *_args: (
            events.append("roster")
            or {
                "source": "/gazebo/model_states",
                "expectedRobots": scenario.robot_count,
                "observedRobotModels": scenario.robot_count,
                "exactRoster": True,
            }
        )
        document = startup_render_report()
        raw = json.dumps(document).encode("utf-8")
        startup = DRIVER.validate_viewer_startup_report(document, raw)
        active_probe = DRIVER.validate_viewer_startup_report(document, raw)
        args = argparse.Namespace(
            ready_timeout=1,
            viewer_timeout=1,
            scenario_timeout=1,
            cleanup_timeout=1,
            viewer_runtime_dir=Path("/tmp/not-used"),
            deployment_commit="a" * 40,
        )
        cleanup = {
            "viewerClosed": True,
            "sessionStopped": True,
            "workspaceReleased": True,
            "resourceIdentityKnown": True,
            "leaseBindingKnown": True,
            "containerAbsent": True,
            "networkAbsent": True,
            "leaseRuntimeAbsent": True,
            "viewerPublisherAbsent": True,
            "complete": True,
        }

        def load_startup(_path):
            events.append("viewer-startup")
            return startup

        def run_active(**_kwargs):
            events.append("active-concurrent-gate")
            return (
                child_output(),
                DRIVER.ProcessOutput(0, "Gazebo GUI preflight passed\n", ""),
                passing_active_video(),
                passing_overlap(),
            )

        with tempfile.TemporaryDirectory() as temporary, mock.patch.object(
            DRIVER,
            "active_viewer_lease_directory",
            return_value=Path(temporary),
        ), mock.patch.object(
            DRIVER, "load_viewer_startup_evidence", side_effect=load_startup
        ), mock.patch.object(
            DRIVER,
            "active_probe_runtime",
            return_value=DRIVER.ActiveProbeRuntime((), {}, "/usr/bin/gzclient"),
        ), mock.patch.object(
            DRIVER,
            "prepare_active_probe_inputs",
            return_value=(
                Path(temporary) / "preflight.py",
                Path(temporary) / "probe.so",
                {
                    "preflightScriptSha256": "a" * 64,
                    "probePluginSha256": "b" * 64,
                },
            ),
        ), mock.patch.object(
            DRIVER, "build_active_probe_command", return_value=["active-probe"]
        ), mock.patch.object(
            DRIVER, "run_active_scenario_gate", side_effect=run_active
        ), mock.patch.object(
            DRIVER, "load_active_probe_evidence", return_value=active_probe
        ), mock.patch.object(
            DRIVER, "write_bytes_secure"
        ), mock.patch.object(
            DRIVER, "cleanup_case", return_value=cleanup
        ):
            report, passed = DRIVER.run_one_case(
                index=1,
                scenario=scenario,
                args=args,
                ui=ui,
                docker=docker,
                evidence_dir=Path(temporary),
                credentials={"email": "owner@example.test", "password": "secret"},
                stop_event=threading.Event(),
            )

        self.assertTrue(passed)
        self.assertEqual(
            [
                "viewer-startup",
                "roster",
                "active-concurrent-gate",
                "roster",
            ],
            events,
        )
        self.assertEqual("startupGpuCapability", report["viewerStartupScene"]["scope"])
        self.assertFalse(
            report["viewerStartupScene"]["activeScenarioMotionMeasured"]
        )
        self.assertTrue(
            report["activeScenarioOverlap"][
                "scenarioProcessActiveThroughoutProbe"
            ]
        )
        self.assertTrue(
            report["activeScenarioVideo"]["scenarioProcessActiveThroughout"]
        )
        self.assertTrue(
            report["postScenarioFullRoster"]["retainedAcrossScenario"]
        )
        self.assertTrue(report["ros"]["taskCleanupVerified"])

    def test_docker_evidence_checks_image_revision_version_and_private_network(self):
        commit = "a" * 40
        digest = "b" * 64
        container_id = "c" * 64
        network_id = "d" * 64
        session_id = uuid.UUID("11111111-1111-4111-8111-111111111111")
        worker_id = "22222222-2222-4222-8222-222222222222"
        network_name = f"robotswarm-{session_id.hex}-net"
        container_name = f"robotswarm-{session_id.hex}"
        docker = DRIVER.DockerHost("docker", threading.Event())
        documents = {
            "container": {
                "Name": f"/{container_name}",
                "Image": f"sha256:{digest}",
                "State": {"Running": True},
                "Config": {
                    "Image": f"sha256:{digest}",
                    "Labels": {
                        DRIVER.MANAGED_LABEL: "true",
                        DRIVER.SESSION_LABEL: str(session_id),
                        DRIVER.WORKER_LABEL: worker_id,
                        DRIVER.IMAGE_VERSION_LABEL: f"{commit}+{digest[:12]}",
                    },
                },
                "NetworkSettings": {
                    "Networks": {
                        network_name: {
                            "NetworkID": network_id,
                            "IPAddress": "172.20.0.4",
                        }
                    }
                },
            },
            "image": {
                "Id": f"sha256:{digest}",
                "Config": {"Labels": {DRIVER.IMAGE_REVISION_LABEL: commit}},
            },
            "network": {
                "Name": network_name,
                "Internal": True,
                "Labels": {
                    DRIVER.MANAGED_LABEL: "true",
                    DRIVER.SESSION_LABEL: str(session_id),
                    DRIVER.WORKER_LABEL: worker_id,
                },
                "Containers": {container_id: {"Name": container_name}},
                "IPAM": {"Config": [{"Gateway": "172.20.0.1"}]},
            },
        }
        with mock.patch.object(
            docker, "_container_ids", return_value=[container_id]
        ), mock.patch.object(
            docker, "_network_ids", return_value=[network_id]
        ), mock.patch.object(
            docker, "_inspect_one", side_effect=lambda kind, _identifier: documents[kind]
        ):
            handle, evidence = docker.verify_session(session_id, commit)

        self.assertEqual(container_id, handle.identifier)
        self.assertEqual(f"{commit}+{digest[:12]}", handle.image_version)
        self.assertEqual("172.20.0.4", handle.private_ip)
        self.assertEqual("172.20.0.1", handle.network_gateway)
        self.assertTrue(evidence["imagePinned"])
        self.assertTrue(evidence["exclusiveNetworkAttachmentVerified"])
        DRIVER.assert_report_safe(evidence)

        documents["container"]["NetworkSettings"]["Networks"]["bridge"] = {
            "NetworkID": "e" * 64
        }
        with mock.patch.object(
            docker, "_container_ids", return_value=[container_id]
        ), mock.patch.object(
            docker, "_network_ids", return_value=[network_id]
        ), mock.patch.object(
            docker, "_inspect_one", side_effect=lambda kind, _identifier: documents[kind]
        ):
            with self.assertRaisesRegex(DRIVER.MatrixError, "attached only"):
                docker.verify_session(session_id, commit)

    def test_cleanup_checks_deterministic_names_in_addition_to_labels(self):
        session_id = uuid.UUID("11111111-1111-4111-8111-111111111111")
        docker = DRIVER.DockerHost("docker", threading.Event())
        present = DRIVER.ProcessOutput(0, "[]\n", "")
        missing_container = DRIVER.ProcessOutput(
            1, "", "Error: No such object: managed-session"
        )
        missing_network = DRIVER.ProcessOutput(
            1, "", "Error: No such network: managed-network"
        )

        with mock.patch.object(
            docker, "_container_ids", return_value=[]
        ), mock.patch.object(
            docker, "_network_ids", return_value=[]
        ), mock.patch.object(
            docker,
            "run",
            side_effect=[missing_container, missing_network],
        ):
            self.assertEqual((True, True), docker.resources_absent(session_id))

        with mock.patch.object(
            docker, "_container_ids", return_value=[]
        ), mock.patch.object(
            docker, "_network_ids", return_value=[]
        ), mock.patch.object(
            docker,
            "run",
            side_effect=[present, missing_network],
        ):
            self.assertEqual((False, True), docker.resources_absent(session_id))

    def test_post_scenario_roster_probe_requires_the_exact_gazebo_fleet(self):
        docker = DRIVER.DockerHost("docker", threading.Event())
        container = DRIVER.ContainerHandle("c" * 64, "version")
        passing = DRIVER.ProcessOutput(
            0,
            'ROSTER_JSON {"exact_roster":true,"expected_robot_count":4,'
            '"gazebo_model_count":4}\n',
            "",
        )
        with mock.patch.object(DRIVER, "run_command", return_value=passing):
            evidence = docker.verify_full_roster(container, 4)
        self.assertEqual(4, evidence["observedRobotModels"])
        self.assertTrue(evidence["exactRoster"])

        incomplete = DRIVER.ProcessOutput(
            0,
            'ROSTER_JSON {"exact_roster":false,"expected_robot_count":4,'
            '"gazebo_model_count":3}\n',
            "",
        )
        with mock.patch.object(DRIVER, "run_command", return_value=incomplete):
            with self.assertRaisesRegex(DRIVER.MatrixError, "not retained"):
                docker.verify_full_roster(container, 4)

    def test_report_writer_redacts_identifiers_and_uses_mode_0600(self):
        with tempfile.TemporaryDirectory(dir="/var/tmp") as temporary:
            root = Path(temporary)
            root.chmod(0o700)
            destination = root / "matrix.json"
            secret = "super-secret-password"
            document = {
                "message": (
                    "owner@example.test "
                    "11111111-1111-4111-8111-111111111111 "
                    + secret
                ),
                "sessionId": "11111111-1111-4111-8111-111111111111",
                "publishToken": "opaque-publish-value",
                "worker_secret": "opaque-worker-value",
                "emailAddress": "not-even-shaped-like-an-email",
                "33333333-3333-4333-8333-333333333333": "private-key-name",
                "sha": "a" * 40,
            }

            DRIVER.write_json_secure(destination, document, [secret])

            payload = destination.read_text(encoding="utf-8")
            self.assertEqual(0o600, stat.S_IMODE(destination.stat().st_mode))
            self.assertNotIn(secret, payload)
            self.assertNotIn("owner@example.test", payload)
            self.assertNotIn("11111111-1111-4111-8111-111111111111", payload)
            self.assertNotIn("33333333-3333-4333-8333-333333333333", payload)
            self.assertNotIn("sessionId", payload)
            self.assertNotIn("publishToken", payload)
            self.assertNotIn("opaque-publish-value", payload)
            self.assertNotIn("worker_secret", payload)
            self.assertNotIn("emailAddress", payload)
            self.assertIn("a" * 40, payload)

    def test_secure_output_rejects_directory_and_report_symlinks(self):
        with tempfile.TemporaryDirectory(dir="/var/tmp") as temporary:
            root = Path(temporary)
            root.chmod(0o700)
            real_directory = root / "real"
            real_directory.mkdir(mode=0o700)
            linked_directory = root / "linked"
            linked_directory.symlink_to(real_directory, target_is_directory=True)

            with self.assertRaises(DRIVER.MatrixError):
                DRIVER.validate_secure_directory(linked_directory)

            target = root / "target.json"
            target.write_text("{}\n", encoding="utf-8")
            target.chmod(0o600)
            linked_report = root / "report.json"
            linked_report.symlink_to(target)
            with self.assertRaises(DRIVER.MatrixError):
                DRIVER.validate_output_target(linked_report)

    def test_case_cleanup_runs_even_when_session_creation_fails(self):
        ui = FakeUi()
        args = argparse.Namespace(
            ready_timeout=1,
            viewer_timeout=1,
            scenario_timeout=1,
            cleanup_timeout=1,
            viewer_runtime_dir=Path("/tmp/not-used"),
            deployment_commit="a" * 40,
        )
        cleanup = {
            "viewerClosed": True,
            "sessionStopped": True,
            "containerAbsent": True,
            "networkAbsent": True,
            "leaseRuntimeAbsent": True,
            "viewerPublisherAbsent": True,
            "complete": True,
        }
        with tempfile.TemporaryDirectory() as temporary, mock.patch.object(
            DRIVER, "cleanup_case", return_value=cleanup
        ) as cleanup_call:
            report, passed = DRIVER.run_one_case(
                index=1,
                scenario=DRIVER.SCENARIOS[0],
                args=args,
                ui=ui,
                docker=mock.Mock(),
                evidence_dir=Path(temporary),
                credentials={"email": "owner@example.test", "password": "secret"},
                stop_event=threading.Event(),
            )

        self.assertFalse(passed)
        self.assertEqual("failed", report["status"])
        self.assertTrue(report["cleanup"]["complete"])
        cleanup_call.assert_called_once()

    def test_failed_ros_gate_retains_sanitized_parsed_protocol_and_hashes(self):
        scenario = DRIVER.SCENARIOS[0]
        session_id = uuid.UUID("11111111-1111-4111-8111-111111111111")
        ui = mock.Mock()
        ui._occupying_sessions.side_effect = [[], [{"id": str(session_id)}]]
        ui.wait_ready.return_value = {
            "state": "Ready",
            "activeRobots": scenario.robot_count,
        }
        ui.require_interactive_hls.return_value = {
            "hlsInteractive": True,
            "status": "En vivo",
            "fps": "Video 29.8 FPS",
        }
        docker = mock.Mock()
        container = DRIVER.ContainerHandle(
            "c" * 64, "version", "172.20.0.4", "172.20.0.1"
        )
        docker.verify_session.return_value = (
            container,
            {"managed": True},
        )
        failed_result = passing_result(scenario)
        failed_result.update(
            {
                "passed": False,
                "failures": ["synthetic functional failure"],
                "diagnostic": (
                    "owner@example.test "
                    "33333333-3333-4333-8333-333333333333"
                ),
            }
        )
        failed_summary = {
            **passing_summary(),
            "passed": 0,
            "failed": 1,
            "all_passed": False,
            "failed_scenarios": [scenario.name],
        }
        failed_child = child_output(failed_result, failed_summary, returncode=1)
        docker.verify_full_roster.return_value = {
            "source": "/gazebo/model_states",
            "expectedRobots": scenario.robot_count,
            "observedRobotModels": scenario.robot_count,
            "exactRoster": True,
        }
        document = startup_render_report()
        raw = json.dumps(document).encode("utf-8")
        startup = DRIVER.validate_viewer_startup_report(document, raw)
        args = argparse.Namespace(
            ready_timeout=1,
            viewer_timeout=1,
            scenario_timeout=1,
            cleanup_timeout=1,
            viewer_runtime_dir=Path("/tmp/not-used"),
            deployment_commit="a" * 40,
        )
        cleanup = {
            "viewerClosed": True,
            "sessionStopped": True,
            "workspaceReleased": True,
            "resourceIdentityKnown": True,
            "leaseBindingKnown": True,
            "containerAbsent": True,
            "networkAbsent": True,
            "leaseRuntimeAbsent": True,
            "viewerPublisherAbsent": True,
            "complete": True,
        }

        with tempfile.TemporaryDirectory() as temporary, mock.patch.object(
            DRIVER,
            "active_viewer_lease_directory",
            return_value=Path(temporary),
        ), mock.patch.object(
            DRIVER, "load_viewer_startup_evidence", return_value=startup
        ), mock.patch.object(
            DRIVER,
            "active_probe_runtime",
            return_value=DRIVER.ActiveProbeRuntime((), {}, "/usr/bin/gzclient"),
        ), mock.patch.object(
            DRIVER,
            "prepare_active_probe_inputs",
            return_value=(
                Path(temporary) / "preflight.py",
                Path(temporary) / "probe.so",
                {
                    "preflightScriptSha256": "a" * 64,
                    "probePluginSha256": "b" * 64,
                },
            ),
        ), mock.patch.object(
            DRIVER, "build_active_probe_command", return_value=["active-probe"]
        ), mock.patch.object(
            DRIVER,
            "run_active_scenario_gate",
            return_value=(
                failed_child,
                DRIVER.ProcessOutput(0, "Gazebo GUI preflight passed\n", ""),
                passing_active_video(),
                passing_overlap(),
            ),
        ), mock.patch.object(
            DRIVER, "load_active_probe_evidence", return_value=startup
        ), mock.patch.object(
            DRIVER, "write_bytes_secure"
        ), mock.patch.object(
            DRIVER, "cleanup_case", return_value=cleanup
        ):
            report, passed = DRIVER.run_one_case(
                index=1,
                scenario=scenario,
                args=args,
                ui=ui,
                docker=docker,
                evidence_dir=Path(temporary),
                credentials={"email": "owner@example.test", "password": "secret"},
                stop_event=threading.Event(),
            )

        self.assertFalse(passed)
        self.assertEqual("failed", report["status"])
        self.assertIn("ros", report, report)
        self.assertEqual(1, report["ros"]["exitCode"])
        self.assertTrue(report["ros"]["protocolParsed"])
        self.assertFalse(report["ros"]["taskCleanupVerified"])
        serialized = json.dumps(report["ros"])
        self.assertNotIn("owner@example.test", serialized)
        self.assertNotIn(str(session_id), serialized)
        self.assertRegex(report["hashes"]["resultJsonSha256"], r"^[0-9a-f]{64}$")
        self.assertRegex(report["hashes"]["summaryJsonSha256"], r"^[0-9a-f]{64}$")
        self.assertRegex(report["hashes"]["evidenceBundleSha256"], r"^[0-9a-f]{64}$")

    def test_incomplete_cleanup_blocks_the_next_scenario(self):
        with self.assertRaises(DRIVER.CleanupError):
            DRIVER.require_case_cleanup_before_next(
                {
                    "scenario": "formation_triangle_n3",
                    "cleanup": {
                        "containerAbsent": True,
                        "networkAbsent": False,
                        "leaseRuntimeAbsent": True,
                        "complete": False,
                    },
                }
            )

        DRIVER.require_case_cleanup_before_next(
            {"scenario": "formation_triangle_n3", "cleanup": {"complete": True}}
        )

    def test_uncertain_creation_without_a_resource_uuid_fails_cleanup_closed(self):
        ui = mock.Mock()
        ui.stop_event = threading.Event()
        ui.create_requested = True
        ui._occupying_sessions.return_value = []
        ui.cdp.evaluate.return_value = False
        ui.has_button.return_value = False
        ui.stop_created_session.return_value = {"requested": True, "released": True}
        docker = mock.Mock()

        cleanup = DRIVER.cleanup_case(
            ui,
            docker,
            session_id=None,
            lease_directory=None,
            timeout=0.1,
        )

        self.assertFalse(cleanup["resourceIdentityKnown"])
        self.assertFalse(cleanup["complete"])
        docker.resources_absent.assert_not_called()

    def test_open_viewer_without_a_bound_lease_directory_fails_cleanup_closed(self):
        ui = mock.Mock()
        ui.stop_event = threading.Event()
        ui.create_requested = True
        ui.cdp.evaluate.return_value = True
        ui.has_button.return_value = False
        ui.stop_created_session.return_value = {"requested": True, "released": True}
        docker = mock.Mock()
        docker.resources_absent.return_value = (True, True)
        session_id = uuid.UUID("11111111-1111-4111-8111-111111111111")

        with mock.patch.object(DRIVER, "publisher_for_session_exists", return_value=False):
            cleanup = DRIVER.cleanup_case(
                ui,
                docker,
                session_id=session_id,
                lease_directory=None,
                timeout=0.1,
            )

        self.assertFalse(cleanup["leaseBindingKnown"])
        self.assertFalse(cleanup["leaseRuntimeAbsent"])
        self.assertFalse(cleanup["complete"])

    def test_child_capture_is_hard_bounded_and_kills_on_overflow(self):
        command = [
            sys.executable,
            "-c",
            (
                "import os,time; "
                "os.write(1, b'x' * 8192); "
                "os.write(2, b'y' * 8192); "
                "time.sleep(30)"
            ),
        ]
        with mock.patch.object(
            DRIVER, "MAXIMUM_CHILD_OUTPUT_BYTES", 1024
        ), mock.patch.object(
            DRIVER,
            "_signal_process_group",
            wraps=DRIVER._signal_process_group,
        ) as signal_group:
            with self.assertRaisesRegex(
                DRIVER.MatrixError,
                r"^A child process exceeded the bounded output limit$",
            ) as failure:
                DRIVER.run_command(command, timeout=5)

        self.assertLessEqual(len(str(failure.exception)), 80)
        self.assertTrue(
            any(call.args[1] == DRIVER.signal.SIGKILL for call in signal_group.call_args_list)
        )
        source = SCRIPT.read_text(encoding="utf-8")
        self.assertNotIn(".communicate(", source)

    def test_child_capture_records_actual_monotonic_spawn_and_exit(self):
        started = []
        exited = []
        result = DRIVER.run_command(
            [sys.executable, "-c", "import time; time.sleep(0.02); print('ok')"],
            timeout=2,
            on_started=started.append,
            on_exited=exited.append,
        )

        self.assertEqual(0, result.returncode)
        self.assertEqual("ok\n", result.stdout)
        self.assertEqual(1, len(started))
        self.assertEqual(1, len(exited))
        self.assertLessEqual(started[0], exited[0])

    def test_interrupt_escalation_still_has_int_term_kill_stages(self):
        self.assertEqual(
            [DRIVER.signal.SIGINT, DRIVER.signal.SIGTERM, DRIVER.signal.SIGKILL],
            [number for number, _grace in DRIVER.CHILD_GRACEFUL_STOP],
        )


if __name__ == "__main__":
    unittest.main()
