#!/usr/bin/env python3
"""Offline contract tests for the visible loaded-payload N=4 gate."""

from __future__ import annotations

import contextlib
import copy
import hashlib
import importlib.util
import io
import json
import os
import signal
import stat
import subprocess
import sys
import tempfile
import time
import unittest
from unittest import mock
from pathlib import Path


SCRIPT = Path(__file__).with_name("robotswarm-loaded-n4-e2e.py")


def load_driver():
    spec = importlib.util.spec_from_file_location("robotswarm_loaded_n4_e2e", SCRIPT)
    if spec is None or spec.loader is None:
        raise RuntimeError("could not load the loaded N=4 driver")
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


DRIVER = load_driver()


def preflight_render_report():
    return {
        "schema_version": 1,
        "process": {
            "pid": 741,
            "executable": "/usr/bin/gzclient",
            "start_ticks": 9981,
        },
        "display": {"x11": ":120", "wayland": ""},
        "camera": {
            "name": "gzclient_camera(0)",
            "viewport_width": 1280,
            "viewport_height": 720,
        },
        "renderer": {
            "api": "OpenGL Rendering Subsystem",
            "device": "D3D12 (NVIDIA GeForce RTX 3080)",
            "vendor": "Microsoft Corporation",
            "gl_vendor": "Microsoft Corporation",
            "gl_renderer": "D3D12 (NVIDIA GeForce RTX 3080)",
        },
        "render_measurement": {
            "source": "gazebo::rendering::Camera::AvgFPS",
            "warmup_seconds": 2.0,
            "sample_seconds": 5.0,
            "samples": 200,
            "average_fps": 49.5,
            "post_render_rate_fps": 49.4,
        },
        "physics_measurement": {
            "source": (
                "gazebo.msgs.WorldStatistics delta(sim_time)/delta(real_time)"
            ),
            "samples": 100,
            "real_time_factor": 2.96,
        },
    }


def process_is_live(pid):
    try:
        fields = (Path("/proc") / str(pid) / "stat").read_text(
            encoding="ascii"
        ).split()
    except OSError:
        return False
    return len(fields) > 2 and fields[2] != "Z"


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


def passing_grf():
    names = ["tb3_0", "tb3_1", "tb3_2", "tb3_3"]
    finder = names[0]
    robots = {
        name: {
            "role": "payload_push" if index < 2 else "companion_push",
            "finder": name == finder,
            "notified": name != finder,
            "notice_recipient": True,
            "status_acknowledged": True,
            "motion_detected": True,
            "pre_push_path_length_m": 0.45,
            "pre_push_displacement_m": 0.3,
            "required_pre_push_path_length_m": 0.03 if index < 2 else 0.1,
            "met_pre_push_path_requirement": True,
        }
        for index, name in enumerate(names)
    }
    participation = {}
    for index, name in enumerate(names):
        root = index < 2
        participation[name] = {
            "role": "payload_push" if root else "companion_push",
            "declared_parent_namespaces": [] if root else [names[index - 2]],
            "direct_contact_samples": 20 if root else 0,
            "companion_contact_samples": 0 if root else 20,
            "connected_samples": 20,
            "push_intent_samples": 20,
            "maximum_continuous_useful_pushing_s": 2.5,
            "maximum_continuous_useful_pushing_samples": 20,
            "useful_pushing_fraction": 0.75,
        }
    return {
        "scenario": "transport_grf_n4",
        "behavior": "transport",
        "robot_count": 4,
        "task_id": "loaded-grf-n4-1234abcd",
        "passed": True,
        "failures": [],
        "cleanup_failures": [],
        "task": {
            "task_id": "loaded-grf-n4-1234abcd",
            "status": "completed",
        },
        "transport_acceptance_thresholds": {
            "active_search_required": True,
            "direct_payload_surface_clearance_m": 0.075,
            "companion_center_distance_m": 0.16,
            "minimum_chain_center_distance_m": 0.12,
            "cmd_vel_min_forward_speed_mps": 0.015,
            "cmd_vel_min_goal_alignment_cosine": 0.50,
            "cmd_vel_max_wall_age_s": 0.75,
            "contribution_goal_velocity_noise_floor_mps": 0.003,
            "contribution_goal_velocity_tolerance_mps": 0.003,
            "contribution_minimum_tracking_fraction": 0.75,
            "maximum_control_sample_gap_s": 1.0,
            "minimum_continuous_useful_push_s": 0.75,
            "minimum_continuous_useful_push_samples": 5,
            "minimum_useful_push_fraction": 0.5,
            "minimum_goal_progress_efficiency": 0.5,
            "configured_minimum_goal_progress_m": 0.55,
            "goal_arrival_tolerance_m": 0.5,
            "goal_progress_contract_epsilon_m": 0.001,
            "goal_progress_contract_basis": (
                "distance at synchronized PUSH start minus the controller "
                "arrival tolerance"
            ),
            "minimum_goal_progress_m": 0.5,
            "minimum_active_search_path_length_per_robot_m": 0.05,
            "minimum_notified_robot_rendezvous_travel_m": 0.1,
            "minimum_payload_root_rendezvous_travel_m": 0.03,
        },
        "metrics": {
            "real_time_factor": 2.96,
            "simulated_duration_s": 10.0,
            "wall_duration_s": round(10.0 / 2.96, 6),
            "transport_discovery_response": {
                "notice_observed": True,
                "finder": finder,
                "notified_robots": names[1:],
                "notice_recipients": list(names),
                "missing_notice_recipients": [],
                "status_acknowledgement_available": True,
                "status_acknowledged_robots": list(names),
                "missing_status_acknowledgements": [],
                "search_motion_window_supported": True,
                "simultaneous_search_motion_window_supported": True,
                "peak_simultaneous_search_movers": 4,
                "robots_observed_moving_during_search": list(names),
                "robots_not_observed_moving_during_search": [],
                "minimum_search_path_length_m": 0.05,
                "search_path_length_m": {name: 0.25 for name in names},
                "robots_below_required_search_travel": [],
                "robots_below_required_travel": [],
                "notified_robots_below_required_travel": [],
                "robots": robots,
            },
            "transport_participation": participation,
            "transport_active_push_latched": True,
            "transport_grf_samples": 25,
            "transport_all_useful_samples": 20,
            "transport_maximum_continuous_all_useful_s": 2.2,
            "transport_maximum_continuous_all_useful_samples": 18,
            "transport_all_useful_batch_fraction": 0.72,
            "transport_push_initial_goal_distance_m": 1.0,
            "transport_push_goal_progress_m": 0.65,
            "transport_push_goal_progress_efficiency": 0.8,
        },
    }


def loaded_trial(count, payload_progress, rtf=2.95, start_index=0):
    names = [f"tb3_{index}" for index in range(start_index, start_index + count)]
    positions = {name: 0.20 for name in names}
    connections = {}
    for index, name in enumerate(names):
        root = index < min(2, count)
        connections[name] = {
            "role": "payload_root" if root else "companion",
            "parent": "transport_object" if root else names[index - 2],
            "contact_distance_m": 0.03 if root else 0.15,
            "connected": True,
        }
    return {
        "robot_count": count,
        "command_speed_mps": 0.16,
        "push_duration_sim_s": 12.0,
        "push_duration_wall_s": round(12.0 / rtf, 4),
        "real_time_factor": rtf,
        "payload_forward_progress_m": payload_progress,
        "robot_forward_progress_m": positions,
        "final_push_connections": connections,
        "final_connected_robot_count": count,
    }


def passing_load(start_index=0):
    return {
        "profile": "transport_crate_loaded",
        "profile_mass_kg": 0.75,
        "profile_friction": 0.25,
        "passed": True,
        "failures": [],
        "single_robot_trial": loaded_trial(1, 0.01, start_index=start_index),
        "root_only_trial": loaded_trial(
            2, 0.03, start_index=start_index + 1
        ),
        "fleet_trial": loaded_trial(4, 0.40, start_index=start_index + 3),
        "payload_progress_gain": 40.0,
        "thresholds": {
            "single_robot_maximum_progress_m": 0.05,
            "two_root_maximum_progress_m": 0.06,
            "fleet_minimum_progress_m": 0.20,
            "minimum_robot_progress_m": 0.05,
            "minimum_payload_progress_gain": 4.0,
            "minimum_real_time_factor": 2.90,
        },
        "loaded_grf_trial": {
            "scenario": "transport_grf_n4",
            "task_id": "loaded-grf-n4-1234abcd",
            "exit_code": 0,
            "passed": True,
            "fresh_cleanup_status_observed": True,
        },
    }


def passing_cleanup():
    return {
        "schemaVersion": 1,
        "taskTerminal": True,
        "rosterEmpty": True,
        "robotModelsAbsent": True,
        "payloadPresent": True,
        "payloadMassKg": 0.25,
        "complete": True,
    }


def protocol_output(
    *,
    grf=None,
    summary=None,
    load=None,
    cleanup=None,
    returncode=0,
):
    documents = (
        ("RESULT_JSON ", grf or passing_grf()),
        ("SUMMARY_JSON ", summary or passing_summary()),
        ("LOAD_RESULT_JSON ", load or passing_load()),
        ("POST_LOAD_CLEANUP_JSON ", cleanup or passing_cleanup()),
    )
    stdout = "diagnostic\n" + "\n".join(
        prefix + json.dumps(document, separators=(",", ":"))
        for prefix, document in documents
    ) + "\n"
    return DRIVER.MATRIX.ProcessOutput(returncode, stdout, "bounded diagnostic\n")


def timed_mass(observed_at, sequence, mass=0.75):
    return DRIVER.TimedMarker(
        document={
            "schemaVersion": 1,
            "sequence": sequence,
            "payloadMassKg": mass,
        },
        observed_at=observed_at,
        source_sha256=(f"{sequence:064x}"),
    )


def timed_active(
    observed_at,
    sequence,
    *,
    task_id="loaded-grf-n4-1234abcd",
    phase="SEARCH",
    progress=0.2,
    mass_sequence=None,
):
    return DRIVER.TimedMarker(
        document={
            "schemaVersion": 1,
            "sequence": sequence,
            "taskId": task_id,
            "taskStatus": "running",
            "taskProgress": progress,
            "phase": phase,
            "robotCount": 4,
            "roster": ["tb3_0", "tb3_1", "tb3_2", "tb3_3"],
            "robotModels": ["tb3_0", "tb3_1", "tb3_2", "tb3_3"],
            "payloadMassKg": 0.75,
            "massSequence": mass_sequence or sequence,
            "massSampleAgeSeconds": 0.05,
        },
        observed_at=observed_at,
        source_sha256=(f"{sequence + 50:064x}"),
    )


def timed_push(
    observed_at,
    sequence,
    task_id="loaded-grf-n4-1234abcd",
    *,
    mass_sequence=None,
    progress=0.7,
):
    return DRIVER.TimedMarker(
        document={
            "schemaVersion": 1,
            "sequence": sequence,
            "taskId": task_id,
            "taskStatus": "running",
            "taskProgress": progress,
            "phase": "PUSH",
            "robotCount": 4,
            "roster": ["tb3_0", "tb3_1", "tb3_2", "tb3_3"],
            "robotModels": ["tb3_0", "tb3_1", "tb3_2", "tb3_3"],
            "allPushersConfirmed": True,
            "usefulContributorCount": 4,
            "usefulContributorIds": ["tb3_0", "tb3_1", "tb3_2", "tb3_3"],
            "payloadMassKg": 0.75,
            "massSequence": mass_sequence or sequence,
            "massSampleAgeSeconds": 0.05,
        },
        observed_at=observed_at,
        source_sha256=(f"{sequence + 100:064x}"),
    )


class LoadedN4AcceptanceContractTests(unittest.TestCase):
    def test_cli_requires_explicit_authorization_full_sha_and_private_inputs(self):
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
            with self.assertRaises(SystemExit) as missing:
                DRIVER.parse_args(required)
        self.assertEqual(2, missing.exception.code)

        parsed = DRIVER.parse_args(["--execute-production", *required])
        self.assertEqual("a" * 40, parsed.deployment_commit)
        self.assertEqual(Path("/tmp/credentials"), parsed.credentials)
        minimum_preflight_timeout = (
            DRIVER.MATRIX.ACTIVE_PROBE_TIMEOUT_SECONDS
            + DRIVER.LOADED_PREFLIGHT_HLS_SECONDS
            + DRIVER.PREFLIGHT_SHUTDOWN_MARGIN_SECONDS
        )
        self.assertEqual(55.0, minimum_preflight_timeout)
        with contextlib.redirect_stderr(io.StringIO()):
            with self.assertRaises(SystemExit):
                DRIVER.parse_args(
                    [
                        "--execute-production",
                        *required,
                        "--preflight-timeout",
                        str(minimum_preflight_timeout - 0.1),
                    ]
                )
        boundary = DRIVER.parse_args(
            [
                "--execute-production",
                *required,
                "--preflight-timeout",
                str(minimum_preflight_timeout),
            ]
        )
        self.assertEqual(minimum_preflight_timeout, boundary.preflight_timeout)
        with contextlib.redirect_stderr(io.StringIO()):
            with self.assertRaises(SystemExit):
                DRIVER.parse_args(
                    ["--execute-production", *required[:1], "abc", *required[2:]]
                )

    def test_commands_fix_official_preflight_and_loaded_probe_thresholds(self):
        runtime = DRIVER.ViewerRuntime(
            command_prefix=("/usr/bin/bwrap", "--chdir", "/viewer", "--"),
            environment={"HOME": "/viewer/private-home"},
            gzclient="/usr/bin/gzclient",
            publisher_render=object(),
            binding={},
        )
        preflight = DRIVER.build_preflight_command(
            runtime,
            Path("/private/preflight.py"),
            Path("/private/probe.so"),
            Path("/private/report.json"),
        )
        self.assertIn("/viewer/preflight.py", preflight)
        self.assertEqual("45.0", preflight[preflight.index("--min-render-fps") + 1])
        self.assertEqual(
            "2.90", preflight[preflight.index("--min-real-time-factor") + 1]
        )
        self.assertEqual("2.0", preflight[preflight.index("--warmup-seconds") + 1])
        self.assertEqual("5.0", preflight[preflight.index("--sample-seconds") + 1])
        self.assertEqual(
            "45.0", preflight[preflight.index("--timeout-seconds") + 1]
        )
        self.assertIn("/viewer/private-home", runtime.environment.values())
        self.assertLess(
            preflight.index("/private/probe.so"), preflight.index("--chdir")
        )
        self.assertNotIn("--headless", preflight)
        self.assertNotIn("--disable-gpu", preflight)

        payload = DRIVER.build_payload_command("docker", "b" * 64)
        self.assertEqual(["docker", "exec", "b" * 64], payload[:3])
        shell = payload[-1]
        self.assertIn(DRIVER.DEPLOYED_LOAD_PROBE_PATH, shell)
        self.assertEqual(1, shell.count("--fleet-count 4"))
        self.assertEqual(1, shell.count("--min-rtf 2.90"))
        self.assertEqual(1, shell.count("--external-viewer-verified"))
        self.assertEqual(1, shell.count("--verify-grf-n4"))
        for text in (
            "--command-speed 0.16",
            "--push-duration 12.0",
            "--single-max-progress 0.05",
            "--root-only-max-progress 0.06",
            "--fleet-min-progress 0.20",
            "--min-robot-progress 0.05",
            "--minimum-gain 4.0",
            "'--min-transport-push-duration', '0.75'",
            "'--min-transport-push-samples', '5'",
            "'--min-transport-useful-fraction', '0.50'",
            "'--min-transport-goal-efficiency', '0.50'",
            "'--min-transport-search-travel', '0.05'",
            "'--min-transport-rendezvous-travel', '0.10'",
            "'--min-transport-root-rendezvous-travel', '0.03'",
            "'--transport-direct-contact-clearance', '0.075'",
            "'--transport-companion-contact-distance', '0.16'",
            "'--min-transport-chain-center-distance', '0.12'",
            "'--transport-cmd-min-speed', '0.015'",
            "'--transport-cmd-min-goal-cosine', '0.50'",
            "'--transport-cmd-max-age', '0.75'",
            "'--transport-contribution-noise-floor', '0.003'",
            "'--transport-contribution-speed-tolerance', '0.003'",
            "'--transport-contribution-tracking-fraction', '0.75'",
            "'--transport-control-gap-tolerance', '1.0'",
        ):
            self.assertEqual(1, shell.count(text), text)
        self.assertIn("LOADED_PAYLOAD_READY_JSON", shell)
        self.assertIn("LOADED_MASS_SAMPLE_JSON", shell)
        self.assertIn("LOADED_GRF_ACTIVE_JSON", shell)
        self.assertIn("LOADED_PUSH_LIVE_JSON", shell)
        self.assertIn("output_lock = threading.Lock()", shell)
        self.assertIn("os.write(1, encoded)", shell)
        self.assertIn("atomic pipe-write limit", shell)
        self.assertNotIn("print(prefix + json.dumps", shell)
        self.assertIn("exec 4>&2", shell)
        self.assertIn("exec 2>/dev/null", shell)
        self.assertIn('python3 -u - "$monitor_stop" 1>&4 4>&-', shell)
        self.assertIn("ROBOTSWARM_LIVE_MARKER_FD=3", shell)
        self.assertIn("--verify-grf-n4 3>&4 4>&-", shell)
        self.assertIn("os.write(marker_fd, encoded)", shell)
        self.assertNotIn("stdout is shared with the official probe", shell)
        self.assertIn("readonly monitor_job='%1'", shell)
        self.assertIn('if [ -n "$(jobs -p)" ]', shell)
        self.assertIn('kill -TERM "$monitor_job"', shell)
        self.assertIn('kill -KILL "$monitor_job"', shell)
        self.assertIn("monitor_wait_pid=$!", shell)
        self.assertIn('wait "$monitor_wait_pid"', shell)
        self.assertNotIn('kill -TERM "$monitor_wait_pid"', shell)
        self.assertNotIn('kill -KILL "$monitor_wait_pid"', shell)
        self.assertNotIn("kill -0", shell)
        self.assertIn("/fleet/robot_list", shell)
        self.assertIn("/gazebo/model_states", shell)
        self.assertIn("massSampleAgeSeconds", shell)
        self.assertIn("taskProgress", shell)
        self.assertIn("if not run_trial_refreshes_payload:", shell)
        self.assertEqual(
            1, shell.count("module.LoadProbe.reset_fleet =")
        )
        self.assertNotIn("def reset_exact_fleet", shell)
        self.assertIn(
            "expected_robots = ['tb3_0', 'tb3_1', 'tb3_2', 'tb3_3']",
            shell,
        )
        self.assertIn("POST_LOAD_CLEANUP_JSON", shell)
        self.assertIn("math.isclose(mass, 0.25", shell)
        for delimiter in ("PY_MONITOR", "PY_LOAD", "PY"):
            after_redirect = shell.split(f"<<'{delimiter}'", 1)[1]
            embedded = after_redirect.split("\n", 1)[1].split(
                f"\n{delimiter}\n", 1
            )[0]
            compile(embedded, f"<{delimiter.lower()}>", "exec")

    def test_load_shim_preserves_fresh_capacity_rosters_end_to_end(self):
        shell = DRIVER.build_payload_command("docker", "b" * 64)[-1]
        after_redirect = shell.split("<<'PY_LOAD'", 1)[1]
        load_shim = after_redirect.split("\n", 1)[1].split(
            "\nPY_LOAD\n", 1
        )[0]
        fake_probe = r'''
import json


class LoadProbe:
    next_robot_index = 11

    def __init__(self):
        self.roster = []

    def replace_payload(self, _model_xml):
        return None

    def reset_fleet(self, count):
        start = type(self).next_robot_index
        self.roster = [
            'tb3_{}'.format(index)
            for index in range(start, start + count)
        ]
        type(self).next_robot_index += count


def loaded_grf_command(_args, _task_id):
    return ['python3', 'robotswarm_live_acceptance.py']


def trial(probe, count, payload_progress):
    probe.reset_fleet(count)
    names = list(probe.roster)
    connections = {}
    for index, name in enumerate(names):
        is_root = index < min(2, count)
        connections[name] = {
            'role': 'payload_root' if is_root else 'companion',
            'parent': 'transport_object' if is_root else names[index - 2],
            'contact_distance_m': 0.03 if is_root else 0.15,
            'connected': True,
        }
    return {
        'robot_count': count,
        'command_speed_mps': 0.16,
        'push_duration_sim_s': 12.0,
        'push_duration_wall_s': round(12.0 / 2.95, 4),
        'real_time_factor': 2.95,
        'payload_forward_progress_m': payload_progress,
        'robot_forward_progress_m': {name: 0.20 for name in names},
        'final_push_connections': connections,
        'final_connected_robot_count': count,
    }


def main():
    probe = LoadProbe()
    probe.replace_payload('<sdf/>')
    single = trial(probe, 1, 0.01)
    roots = trial(probe, 2, 0.03)
    fleet = trial(probe, 4, 0.40)
    document = {
        'profile': 'transport_crate_loaded',
        'profile_mass_kg': 0.75,
        'profile_friction': 0.25,
        'passed': True,
        'failures': [],
        'single_robot_trial': single,
        'root_only_trial': roots,
        'fleet_trial': fleet,
        'payload_progress_gain': 40.0,
        'thresholds': {
            'single_robot_maximum_progress_m': 0.05,
            'two_root_maximum_progress_m': 0.06,
            'fleet_minimum_progress_m': 0.20,
            'minimum_robot_progress_m': 0.05,
            'minimum_payload_progress_gain': 4.0,
            'minimum_real_time_factor': 2.90,
        },
        'loaded_grf_trial': {
            'scenario': 'transport_grf_n4',
            'task_id': 'loaded-grf-n4-1234abcd',
            'exit_code': 0,
            'passed': True,
            'fresh_cleanup_status_observed': True,
        },
    }
    print('LOAD_RESULT_JSON ' + json.dumps(document, separators=(',', ':')))
    return 0
'''.strip()

        with tempfile.TemporaryDirectory() as temporary:
            probe_path = Path(temporary) / "fake_load_probe.py"
            probe_path.write_text(fake_probe, encoding="utf-8")
            completed = subprocess.run(
                [sys.executable, "-", str(probe_path)],
                input=load_shim,
                text=True,
                capture_output=True,
                timeout=10.0,
                check=False,
            )

        self.assertEqual(0, completed.returncode, completed.stderr)
        marker = next(
            line for line in completed.stdout.splitlines()
            if line.startswith("LOAD_RESULT_JSON ")
        )
        report = DRIVER.validate_loaded_capacity(
            json.loads(marker[len("LOAD_RESULT_JSON ") :])
        )
        self.assertEqual(4, report["connectedRobotCount"])

    def test_payload_child_publishes_active_grf_lines_to_the_collector(self):
        document = timed_active(1.0, 1).document
        source = (
            "import json, sys; print("
            + repr(DRIVER.GRF_ACTIVE_PREFIX)
            + " + json.dumps("
            + repr(document)
            + "), file=sys.stderr, flush=True)"
        )
        child = DRIVER.start_payload_command([sys.executable, "-c", source])
        child.wait(timeout=5.0, stop_event=None)
        collector = DRIVER.LiveMarkerCollector()
        collector.ingest(child)

        self.assertEqual(1, len(collector.active))
        self.assertEqual(document["taskId"], collector.active[0].document["taskId"])

    def test_payload_child_keeps_protocol_and_live_markers_on_separate_pipes(self):
        stdout_marker = timed_active(1.0, 1).document
        stderr_marker = timed_active(1.0, 2).document
        official_line = (
            DRIVER.GRF_ACTIVE_PREFIX
            + json.dumps(stdout_marker, separators=(",", ":"))
            + "\n"
            + "RESULT_JSON "
            + ("x" * (128 * 1024))
            + "\n"
        ).encode("utf-8")
        live_line = (
            DRIVER.GRF_ACTIVE_PREFIX
            + json.dumps(stderr_marker, separators=(",", ":"))
            + "\n"
        ).encode("utf-8")
        source = r"""
import os
import sys
import threading

official_marker = bytes.fromhex(sys.argv[1])
live = bytes.fromhex(sys.argv[2])

def write_official():
    os.write(1, official_marker)
    os.write(1, b'RESULT_JSON ')
    for _ in range(128):
        os.write(1, b'x' * 1024)
    os.write(1, b'\n')

left = threading.Thread(target=write_official)
right = threading.Thread(target=os.write, args=(2, live))
left.start()
right.start()
left.join()
right.join()
"""
        child = DRIVER.start_payload_command(
            [
                sys.executable,
                "-c",
                source,
                official_line.split(b"RESULT_JSON ", 1)[0].hex(),
                live_line.hex(),
            ]
        )
        output = child.wait(timeout=10.0, stop_event=None)
        collector = DRIVER.LiveMarkerCollector()
        collector.ingest(child)

        self.assertEqual(official_line.decode("utf-8"), output.stdout)
        self.assertEqual(live_line.decode("utf-8"), output.stderr)
        self.assertEqual(1, len(collector.active))
        self.assertEqual(2, collector.active[0].document["sequence"])

    def test_payload_child_rejects_unapproved_marker_channel_data_without_echo(self):
        child = DRIVER.start_payload_command(
            [
                sys.executable,
                "-c",
                "import sys; print('private diagnostic', file=sys.stderr, flush=True)",
            ]
        )
        with self.assertRaisesRegex(
            DRIVER.LoadedGateError, "could not be drained safely"
        ) as failure:
            child.wait(timeout=5.0, stop_event=None)

        self.assertNotIn("private diagnostic", str(failure.exception))

    def test_monitor_supervisor_cannot_signal_an_unrelated_numeric_pid(self):
        foreign = subprocess.Popen(
            [sys.executable, "-c", "import time; time.sleep(60)"],
            start_new_session=True,
        )
        try:
            with tempfile.TemporaryDirectory(dir="/tmp") as temporary:
                runtime = Path(temporary) / "monitor-runtime"
                runtime.mkdir(mode=0o700)
                script = "\n".join(
                    (
                        "monitor_runtime=" + str(runtime),
                        'monitor_stop="$monitor_runtime/stop"',
                        "monitor_job=" + str(foreign.pid),
                        "monitor_wait_pid=" + str(foreign.pid),
                        "monitor_running=true",
                        DRIVER.monitor_supervision_shell(),
                        "stop_monitor",
                    )
                )
                completed = subprocess.run(
                    ["/bin/bash", "-c", script],
                    text=True,
                    capture_output=True,
                    timeout=5.0,
                    check=False,
                )

            self.assertEqual(1, completed.returncode, completed.stderr)
            self.assertIsNone(foreign.poll())
        finally:
            with contextlib.suppress(ProcessLookupError):
                os.killpg(foreign.pid, signal.SIGKILL)
            foreign.wait(timeout=5.0)

    def test_monitor_supervisor_propagates_an_expected_stop_failure(self):
        with tempfile.TemporaryDirectory(dir="/tmp") as temporary:
            runtime = Path(temporary) / "monitor-runtime"
            runtime.mkdir(mode=0o700)
            script = "\n".join(
                (
                    "monitor_runtime=" + str(runtime),
                    'monitor_stop="$monitor_runtime/stop"',
                    "monitor_job='%1'",
                    "monitor_wait_pid=",
                    "monitor_running=false",
                    DRIVER.monitor_supervision_shell(),
                    "( while [ ! -f \"$monitor_stop\" ]; do sleep 0.01; done; exit 7 ) &",
                    "monitor_wait_pid=$!",
                    "monitor_running=true",
                    "stop_monitor",
                )
            )
            completed = subprocess.run(
                ["/bin/bash", "-c", script],
                text=True,
                capture_output=True,
                timeout=5.0,
                check=False,
            )

        self.assertEqual(7, completed.returncode, completed.stderr)
        self.assertFalse(runtime.exists())

    def test_monitor_supervisor_rejects_a_monitor_that_ended_early(self):
        with tempfile.TemporaryDirectory(dir="/tmp") as temporary:
            runtime = Path(temporary) / "monitor-runtime"
            runtime.mkdir(mode=0o700)
            script = "\n".join(
                (
                    "monitor_runtime=" + str(runtime),
                    'monitor_stop="$monitor_runtime/stop"',
                    "monitor_job='%1'",
                    "monitor_wait_pid=",
                    "monitor_running=false",
                    DRIVER.monitor_supervision_shell(),
                    "( exit 7 ) &",
                    "monitor_wait_pid=$!",
                    "monitor_running=true",
                    "sleep 0.05",
                    "stop_monitor",
                )
            )
            completed = subprocess.run(
                ["/bin/bash", "-c", script],
                text=True,
                capture_output=True,
                timeout=5.0,
                check=False,
            )
            self.assertFalse(runtime.exists())

        self.assertNotEqual(0, completed.returncode, completed.stderr)

    def test_loaded_preflight_report_requires_the_direct_child_attestation(self):
        document = preflight_render_report()
        with tempfile.TemporaryDirectory(dir="/tmp") as temporary:
            report = Path(temporary) / "loaded-gui-report.json"
            report.write_text(json.dumps(document), encoding="utf-8")
            report.chmod(0o600)
            raw = report.read_bytes()
            attestation = DRIVER.MATRIX.ActiveProbeAttestation(
                hashlib.sha256(raw).hexdigest(),
                document["process"]["pid"],
                document["process"]["start_ticks"],
            )

            evidence, accepted_raw = DRIVER.validate_preflight_report(
                report, attestation
            )
            self.assertEqual(raw, accepted_raw)
            self.assertEqual(49.5, evidence.average_fps)

            replay = DRIVER.MATRIX.ActiveProbeAttestation(
                attestation.sha256,
                attestation.process_id,
                attestation.process_start_ticks + 1,
            )
            with self.assertRaisesRegex(DRIVER.LoadedGateError, "live process"):
                DRIVER.validate_preflight_report(report, replay)

    def test_protocol_requires_one_ordered_load_grf_summary_and_cleanup_document(self):
        evidence = DRIVER.parse_loaded_protocol(protocol_output())
        result = DRIVER.validate_protocol(evidence)
        self.assertEqual(4, result["grf"]["usefulPusherCount"])
        self.assertTrue(result["probeCleanup"]["practicePayloadRestored"])

        duplicate = protocol_output()
        duplicate = DRIVER.MATRIX.ProcessOutput(
            0,
            duplicate.stdout
            + "LOAD_RESULT_JSON "
            + json.dumps(passing_load())
            + "\n",
            "",
        )
        with self.assertRaises(DRIVER.LoadedGateError):
            DRIVER.parse_loaded_protocol(duplicate)

        lines = protocol_output().stdout.splitlines()
        lines[1], lines[3] = lines[3], lines[1]
        with self.assertRaisesRegex(DRIVER.LoadedGateError, "out of order"):
            DRIVER.parse_loaded_protocol(
                DRIVER.MATRIX.ProcessOutput(0, "\n".join(lines) + "\n", "")
            )

    def test_capacity_gate_requires_heavy_profile_four_connected_pushers_and_rtf(self):
        report = DRIVER.validate_loaded_capacity(passing_load())
        self.assertEqual(0.75, report["profileMassKg"])
        self.assertEqual(4, report["connectedRobotCount"])
        self.assertEqual(2, report["payloadRootCount"])
        self.assertEqual(2, report["companionCount"])

        low_rtf = passing_load()
        low_rtf["fleet_trial"]["real_time_factor"] = 2.89
        low_rtf["fleet_trial"]["push_duration_wall_s"] = round(12.0 / 2.89, 4)
        with self.assertRaisesRegex(DRIVER.LoadedGateError, "below RTF 2.90"):
            DRIVER.validate_loaded_capacity(low_rtf)

        disconnected = passing_load()
        disconnected["fleet_trial"]["final_push_connections"]["tb3_3"][
            "connected"
        ] = False
        with self.assertRaisesRegex(DRIVER.LoadedGateError, "not connected"):
            DRIVER.validate_loaded_capacity(disconnected)

    def test_capacity_gate_accepts_a_fresh_contiguous_namespace_offset(self):
        report = DRIVER.validate_loaded_capacity(passing_load(start_index=27))

        self.assertEqual(4, report["connectedRobotCount"])

    def test_capacity_gate_rejects_reused_gapped_or_invalid_rosters(self):
        overlap = passing_load(start_index=10)
        overlap["root_only_trial"] = loaded_trial(2, 0.03, start_index=10)
        with self.assertRaisesRegex(DRIVER.LoadedGateError, "reused"):
            DRIVER.validate_loaded_capacity(overlap)

        cross_trial_gap = passing_load(start_index=10)
        cross_trial_gap["root_only_trial"] = loaded_trial(
            2, 0.03, start_index=12
        )
        cross_trial_gap["fleet_trial"] = loaded_trial(
            4, 0.40, start_index=14
        )
        with self.assertRaisesRegex(DRIVER.LoadedGateError, "monotonic allocation"):
            DRIVER.validate_loaded_capacity(cross_trial_gap)

        roster_gap = passing_load()
        trial = roster_gap["fleet_trial"]
        trial["robot_forward_progress_m"]["tb3_9"] = trial[
            "robot_forward_progress_m"
        ].pop("tb3_4")
        trial["final_push_connections"]["tb3_9"] = trial[
            "final_push_connections"
        ].pop("tb3_4")
        with self.assertRaisesRegex(DRIVER.LoadedGateError, "not contiguous"):
            DRIVER.validate_loaded_capacity(roster_gap)

        invalid = passing_load()
        trial = invalid["fleet_trial"]
        trial["robot_forward_progress_m"]["burger_4"] = trial[
            "robot_forward_progress_m"
        ].pop("tb3_4")
        trial["final_push_connections"]["burger_4"] = trial[
            "final_push_connections"
        ].pop("tb3_4")
        with self.assertRaisesRegex(DRIVER.LoadedGateError, "invalid Burger"):
            DRIVER.validate_loaded_capacity(invalid)

    def test_capacity_gate_requires_equal_progress_and_connection_maps(self):
        divergent = passing_load()
        connections = divergent["fleet_trial"]["final_push_connections"]
        connections["tb3_9"] = connections.pop("tb3_4")

        with self.assertRaisesRegex(DRIVER.LoadedGateError, "same Burger roster"):
            DRIVER.validate_loaded_capacity(divergent)

    def test_capacity_recalculates_timing_and_rejects_forged_trial_metadata(self):
        wrong_speed = passing_load()
        wrong_speed["single_robot_trial"]["command_speed_mps"] = 0.15
        with self.assertRaisesRegex(DRIVER.LoadedGateError, "command speed"):
            DRIVER.validate_loaded_capacity(wrong_speed)

        short = passing_load()
        short["root_only_trial"]["push_duration_sim_s"] = 10.0
        with self.assertRaisesRegex(DRIVER.LoadedGateError, "12 sim seconds"):
            DRIVER.validate_loaded_capacity(short)

        no_wall_clock = passing_load()
        no_wall_clock["fleet_trial"]["push_duration_wall_s"] = 0.0
        with self.assertRaisesRegex(DRIVER.LoadedGateError, "positive wall"):
            DRIVER.validate_loaded_capacity(no_wall_clock)

        forged_rtf = passing_load()
        forged_rtf["fleet_trial"]["real_time_factor"] = 4.0
        with self.assertRaisesRegex(DRIVER.LoadedGateError, "inconsistent RTF"):
            DRIVER.validate_loaded_capacity(forged_rtf)

        wrong_roster = passing_load()
        trial = wrong_roster["fleet_trial"]
        trial["robot_forward_progress_m"]["tb3_9"] = trial[
            "robot_forward_progress_m"
        ].pop("tb3_3")
        trial["final_push_connections"]["tb3_9"] = trial[
            "final_push_connections"
        ].pop("tb3_3")
        with self.assertRaisesRegex(DRIVER.LoadedGateError, "not contiguous"):
            DRIVER.validate_loaded_capacity(wrong_roster)

    def test_capacity_connections_are_real_payload_rooted_graphs(self):
        wrong_root = passing_load()
        wrong_root["fleet_trial"]["final_push_connections"]["tb3_3"][
            "parent"
        ] = "tb3_4"
        with self.assertRaisesRegex(DRIVER.LoadedGateError, "transport_object"):
            DRIVER.validate_loaded_capacity(wrong_root)

        cycle = passing_load()
        connections = cycle["fleet_trial"]["final_push_connections"]
        connections["tb3_5"]["parent"] = "tb3_6"
        connections["tb3_6"]["parent"] = "tb3_5"
        with self.assertRaisesRegex(DRIVER.LoadedGateError, "non-equivalent"):
            DRIVER.validate_loaded_capacity(cycle)

        invented_count = passing_load()
        invented_count["single_robot_trial"]["final_connected_robot_count"] = 0
        with self.assertRaisesRegex(DRIVER.LoadedGateError, "count is inconsistent"):
            DRIVER.validate_loaded_capacity(invented_count)

        impossible_distance = passing_load()
        for name in ("single_robot_trial", "root_only_trial", "fleet_trial"):
            for connection in impossible_distance[name][
                "final_push_connections"
            ].values():
                connection["contact_distance_m"] = 100.0
        with self.assertRaisesRegex(DRIVER.LoadedGateError, "too far"):
            DRIVER.validate_loaded_capacity(impossible_distance)

    def test_capacity_gate_rejects_symbolic_or_single_robot_success(self):
        boolean_count = passing_load()
        boolean_count["single_robot_trial"]["robot_count"] = True
        with self.assertRaisesRegex(DRIVER.LoadedGateError, "does not describe N=1"):
            DRIVER.validate_loaded_capacity(boolean_count)

        too_easy = passing_load()
        too_easy["single_robot_trial"]["payload_forward_progress_m"] = 0.07
        with self.assertRaisesRegex(DRIVER.LoadedGateError, "One Burger"):
            DRIVER.validate_loaded_capacity(too_easy)

        too_little = passing_load()
        too_little["fleet_trial"]["payload_forward_progress_m"] = 0.19
        with self.assertRaisesRegex(DRIVER.LoadedGateError, "did not move"):
            DRIVER.validate_loaded_capacity(too_little)

    def test_capacity_gate_does_not_trust_echoed_thresholds_or_gain(self):
        relaxed = passing_load()
        relaxed["thresholds"]["single_robot_maximum_progress_m"] = 0.5
        with self.assertRaisesRegex(DRIVER.LoadedGateError, "pinned thresholds"):
            DRIVER.validate_loaded_capacity(relaxed)

        invented_gain = passing_load()
        invented_gain["payload_progress_gain"] = 400.0
        result = DRIVER.validate_loaded_capacity(invented_gain)
        self.assertEqual(40.0, result["payloadProgressGain"])

    def test_grf_gate_proves_search_notice_rendezvous_and_all_four_pushers(self):
        result = DRIVER.validate_grf_n4(passing_grf(), passing_summary())
        self.assertEqual(4, result["simultaneousSearchMoverCount"])
        self.assertEqual(4, result["noticeRecipientCount"])
        self.assertEqual(4, result["rendezvousRobotCount"])
        self.assertEqual(4, result["usefulPusherCount"])
        self.assertGreaterEqual(result["observedRealTimeFactor"], 2.90)

    def test_grf_gate_recalculates_rtf_from_simulated_and_wall_time(self):
        forged = passing_grf()
        forged["metrics"]["real_time_factor"] = 3.0
        forged["metrics"]["simulated_duration_s"] = 1.0
        forged["metrics"]["wall_duration_s"] = 100.0
        with self.assertRaisesRegex(DRIVER.LoadedGateError, "inconsistent RTF"):
            DRIVER.validate_grf_n4(forged, passing_summary())

        slow = passing_grf()
        slow["metrics"]["real_time_factor"] = 2.89
        slow["metrics"]["simulated_duration_s"] = 10.0
        slow["metrics"]["wall_duration_s"] = 10.0 / 2.89
        with self.assertRaisesRegex(DRIVER.LoadedGateError, "below.*2.90"):
            DRIVER.validate_grf_n4(slow, passing_summary())

        no_wall_time = passing_grf()
        no_wall_time["metrics"]["wall_duration_s"] = 0.0
        with self.assertRaisesRegex(DRIVER.LoadedGateError, "durations"):
            DRIVER.validate_grf_n4(no_wall_time, passing_summary())

    def test_grf_gate_rejects_missing_search_or_notice_participant(self):
        missing_search = passing_grf()
        response = missing_search["metrics"]["transport_discovery_response"]
        response["robots_observed_moving_during_search"].remove("tb3_3")
        with self.assertRaisesRegex(DRIVER.LoadedGateError, "SEARCH"):
            DRIVER.validate_grf_n4(missing_search, passing_summary())

        missing_notice = passing_grf()
        response = missing_notice["metrics"]["transport_discovery_response"]
        response["notice_recipients"].remove("tb3_2")
        with self.assertRaisesRegex(DRIVER.LoadedGateError, "notification"):
            DRIVER.validate_grf_n4(missing_notice, passing_summary())

        wrong_identity = passing_grf()
        response = wrong_identity["metrics"]["transport_discovery_response"]
        response["robots"]["tb3_9"] = response["robots"].pop("tb3_3")
        with self.assertRaisesRegex(DRIVER.LoadedGateError, "exact N=4 roster"):
            DRIVER.validate_grf_n4(wrong_identity, passing_summary())

    def test_grf_identifier_lists_and_parent_graph_reject_duplicates_or_cycles(self):
        duplicate_recipient = passing_grf()
        duplicate_recipient["metrics"]["transport_discovery_response"][
            "notice_recipients"
        ].append("tb3_0")
        with self.assertRaisesRegex(DRIVER.LoadedGateError, "identifier list"):
            DRIVER.validate_grf_n4(duplicate_recipient, passing_summary())

        self_parent = passing_grf()
        self_parent["metrics"]["transport_participation"]["tb3_2"][
            "declared_parent_namespaces"
        ] = ["tb3_2"]
        with self.assertRaisesRegex(DRIVER.LoadedGateError, "predecessor"):
            DRIVER.validate_grf_n4(self_parent, passing_summary())

        cycle = passing_grf()
        participation = cycle["metrics"]["transport_participation"]
        participation["tb3_2"]["declared_parent_namespaces"] = ["tb3_3"]
        participation["tb3_3"]["declared_parent_namespaces"] = ["tb3_2"]
        with self.assertRaisesRegex(DRIVER.LoadedGateError, "not connected"):
            DRIVER.validate_grf_n4(cycle, passing_summary())

        root_with_robot_parent = passing_grf()
        root_with_robot_parent["metrics"]["transport_participation"]["tb3_0"][
            "declared_parent_namespaces"
        ] = ["tb3_1"]
        with self.assertRaisesRegex(DRIVER.LoadedGateError, "payload root"):
            DRIVER.validate_grf_n4(root_with_robot_parent, passing_summary())

    def test_grf_gate_requires_two_payload_roots_before_companion_chains(self):
        one_root = passing_grf()
        one_root["metrics"]["transport_discovery_response"]["robots"]["tb3_1"][
            "role"
        ] = "companion_push"
        one_root["metrics"]["transport_discovery_response"]["robots"]["tb3_1"][
            "required_pre_push_path_length_m"
        ] = DRIVER.GRF_MINIMUM_RENDEZVOUS_TRAVEL_M
        assignment = one_root["metrics"]["transport_participation"]["tb3_1"]
        assignment.update(
            {
                "role": "companion_push",
                "declared_parent_namespaces": ["tb3_0"],
                "direct_contact_samples": 0,
                "companion_contact_samples": 20,
            }
        )
        with self.assertRaisesRegex(DRIVER.LoadedGateError, "exactly two"):
            DRIVER.validate_grf_n4(one_root, passing_summary())

        three_roots = passing_grf()
        three_roots["metrics"]["transport_discovery_response"]["robots"]["tb3_2"][
            "role"
        ] = "payload_push"
        three_roots["metrics"]["transport_discovery_response"]["robots"]["tb3_2"][
            "required_pre_push_path_length_m"
        ] = DRIVER.GRF_MINIMUM_ROOT_RENDEZVOUS_TRAVEL_M
        assignment = three_roots["metrics"]["transport_participation"]["tb3_2"]
        assignment.update(
            {
                "role": "payload_push",
                "declared_parent_namespaces": [],
                "direct_contact_samples": 20,
                "companion_contact_samples": 0,
            }
        )
        with self.assertRaisesRegex(DRIVER.LoadedGateError, "exactly two"):
            DRIVER.validate_grf_n4(three_roots, passing_summary())

    def test_grf_gate_rejects_one_noncontributing_or_unconnected_robot(self):
        no_contribution = passing_grf()
        item = no_contribution["metrics"]["transport_participation"]["tb3_3"]
        item["useful_pushing_fraction"] = 0.0
        with self.assertRaisesRegex(DRIVER.LoadedGateError, "did not contribute"):
            DRIVER.validate_grf_n4(no_contribution, passing_summary())

        no_contact = passing_grf()
        item = no_contact["metrics"]["transport_participation"]["tb3_2"]
        item["companion_contact_samples"] = 0
        with self.assertRaisesRegex(DRIVER.LoadedGateError, "contact samples"):
            DRIVER.validate_grf_n4(no_contact, passing_summary())

    def test_grf_gate_does_not_trust_relaxed_echoed_thresholds(self):
        relaxed = passing_grf()
        relaxed["transport_acceptance_thresholds"][
            "minimum_continuous_useful_push_s"
        ] = 0.01
        with self.assertRaisesRegex(DRIVER.LoadedGateError, "pinned thresholds"):
            DRIVER.validate_grf_n4(relaxed, passing_summary())

        relaxed_geometry = passing_grf()
        relaxed_geometry["transport_acceptance_thresholds"][
            "direct_payload_surface_clearance_m"
        ] = 2.0
        with self.assertRaisesRegex(DRIVER.LoadedGateError, "pinned thresholds"):
            DRIVER.validate_grf_n4(relaxed_geometry, passing_summary())

        relaxed_contribution = passing_grf()
        relaxed_contribution["transport_acceptance_thresholds"][
            "cmd_vel_min_forward_speed_mps"
        ] = 0.0
        with self.assertRaisesRegex(DRIVER.LoadedGateError, "pinned thresholds"):
            DRIVER.validate_grf_n4(relaxed_contribution, passing_summary())

        short_goal = passing_grf()
        short_goal["metrics"]["transport_push_goal_progress_m"] = 0.49
        with self.assertRaisesRegex(DRIVER.LoadedGateError, "goal progress"):
            DRIVER.validate_grf_n4(short_goal, passing_summary())

    def test_grf_dynamic_echo_keeps_the_half_metre_physical_gate(self):
        rounded_contract = passing_grf()
        thresholds = rounded_contract["transport_acceptance_thresholds"]
        metrics = rounded_contract["metrics"]
        metrics["transport_push_initial_goal_distance_m"] = 0.9999
        thresholds["minimum_goal_progress_m"] = 0.4999

        result = DRIVER.validate_grf_n4(rounded_contract, passing_summary())
        self.assertEqual(0.4999, result["contractGoalProgressM"])
        self.assertEqual(0.5, result["physicalMinimumGoalProgressM"])

        long_contract = passing_grf()
        long_contract["metrics"][
            "transport_push_initial_goal_distance_m"
        ] = 1.2
        long_contract["transport_acceptance_thresholds"][
            "minimum_goal_progress_m"
        ] = 0.55
        result = DRIVER.validate_grf_n4(long_contract, passing_summary())
        self.assertEqual(0.55, result["contractGoalProgressM"])

        rounded_contract["metrics"]["transport_push_goal_progress_m"] = 0.4999
        result = DRIVER.validate_grf_n4(rounded_contract, passing_summary())
        self.assertEqual(0.4999, result["goalProgressM"])

        below_physical_gate = copy.deepcopy(rounded_contract)
        below_physical_gate["metrics"]["transport_push_goal_progress_m"] = 0.4989
        with self.assertRaisesRegex(DRIVER.LoadedGateError, "goal progress"):
            DRIVER.validate_grf_n4(below_physical_gate, passing_summary())

    def test_grf_dynamic_echo_requires_complete_consistent_contract_fields(self):
        missing_basis = passing_grf()
        missing_basis["transport_acceptance_thresholds"].pop(
            "goal_progress_contract_basis"
        )
        with self.assertRaisesRegex(DRIVER.LoadedGateError, "contract basis"):
            DRIVER.validate_grf_n4(missing_basis, passing_summary())

        relaxed_epsilon = passing_grf()
        relaxed_epsilon["transport_acceptance_thresholds"][
            "goal_progress_contract_epsilon_m"
        ] = 0.01
        with self.assertRaisesRegex(DRIVER.LoadedGateError, "pinned thresholds"):
            DRIVER.validate_grf_n4(relaxed_epsilon, passing_summary())

        missing_arrival_tolerance = passing_grf()
        missing_arrival_tolerance["transport_acceptance_thresholds"].pop(
            "goal_arrival_tolerance_m"
        )
        with self.assertRaisesRegex(DRIVER.LoadedGateError, "not numeric"):
            DRIVER.validate_grf_n4(
                missing_arrival_tolerance,
                passing_summary(),
            )

        forged_echo = passing_grf()
        forged_echo["transport_acceptance_thresholds"][
            "minimum_goal_progress_m"
        ] = 0.4999
        with self.assertRaisesRegex(DRIVER.LoadedGateError, "dynamic goal-progress"):
            DRIVER.validate_grf_n4(forged_echo, passing_summary())

        missing_initial_distance = passing_grf()
        missing_initial_distance["metrics"].pop(
            "transport_push_initial_goal_distance_m"
        )
        with self.assertRaisesRegex(DRIVER.LoadedGateError, "initial PUSH"):
            DRIVER.validate_grf_n4(
                missing_initial_distance,
                passing_summary(),
            )

    def test_probe_cleanup_requires_terminal_empty_and_restored_025kg_payload(self):
        cleanup = DRIVER.validate_post_cleanup(passing_cleanup())
        self.assertEqual(0.25, cleanup["payloadMassKg"])

        for key in ("taskTerminal", "rosterEmpty", "robotModelsAbsent"):
            invalid = passing_cleanup()
            invalid[key] = False
            with self.assertRaises(DRIVER.LoadedGateError):
                DRIVER.validate_post_cleanup(invalid)
        wrong_mass = passing_cleanup()
        wrong_mass["payloadMassKg"] = 0.75
        with self.assertRaises(DRIVER.LoadedGateError):
            DRIVER.validate_post_cleanup(wrong_mass)

    def test_loaded_and_grf_task_ids_must_correlate(self):
        load = passing_load()
        grf = passing_grf()
        DRIVER.correlate_loaded_grf(load, grf)
        grf["task"]["task_id"] = "loaded-grf-n4-deadbeef"
        with self.assertRaisesRegex(DRIVER.LoadedGateError, "different tasks"):
            DRIVER.correlate_loaded_grf(load, grf)

    def test_master_binding_requires_same_private_container_endpoint(self):
        viewer = {
            "ROS_MASTER_URI": "http://172.20.0.4:11311",
            "GAZEBO_MASTER_URI": "http://172.20.0.4:11345",
        }
        container = {"ROS_MASTER_URI": "http://127.0.0.1:11311"}
        binding = DRIVER.validate_master_binding("172.20.0.4", viewer, container)
        self.assertTrue(binding["sameRosMaster"])
        self.assertTrue(binding["sameGazeboMaster"])

        with self.assertRaises(DRIVER.LoadedGateError):
            DRIVER.validate_master_binding("172.20.0.5", viewer, container)
        with self.assertRaises(DRIVER.LoadedGateError):
            DRIVER.validate_master_binding(
                "172.20.0.4",
                {**viewer, "GAZEBO_MASTER_URI": "http://example.test:11345"},
                container,
            )

    def test_sanitized_artifact_is_private_and_drops_identifiers(self):
        with tempfile.TemporaryDirectory(dir="/tmp") as root:
            directory = Path(root)
            os.chmod(directory, 0o700)
            path = directory / "evidence.json"
            digest, size = DRIVER.json_artifact(
                path,
                {
                    "passed": True,
                    "session_id": "11111111-1111-4111-8111-111111111111",
                    "container_id": "b" * 64,
                    "note": "operator@example.test",
                },
                ("operator@example.test",),
            )
            metadata = path.stat()
            document = json.loads(path.read_text(encoding="utf-8"))
            self.assertEqual(0o600, stat.S_IMODE(metadata.st_mode))
            self.assertEqual(size, metadata.st_size)
            self.assertRegex(digest, r"^[0-9a-f]{64}$")
            self.assertNotIn("session_id", document)
            self.assertNotIn("container_id", document)
            self.assertNotIn("b" * 64, path.read_text(encoding="utf-8"))
            self.assertNotIn("operator@example.test", path.read_text(encoding="utf-8"))

        cleaned = DRIVER.clean_failure("docker failed for " + "c" * 64)
        self.assertNotIn("c" * 64, cleaned)

    def test_bounded_child_drains_both_streams_without_deadlock(self):
        source = """
import os
import threading

def write(fd, byte):
    for _ in range(128):
        os.write(fd, byte * 1024)

left = threading.Thread(target=write, args=(1, b'a'))
right = threading.Thread(target=write, args=(2, b'b'))
left.start()
right.start()
left.join()
right.join()
"""
        child = DRIVER.BoundedChild(
            [sys.executable, "-c", source],
            maximum_output_bytes=512 * 1024,
        ).start()
        output = child.wait(timeout=10.0, stop_event=None)
        self.assertEqual(0, output.returncode)
        self.assertEqual(128 * 1024, len(output.stdout))
        self.assertEqual(128 * 1024, len(output.stderr))

    def test_bounded_child_applies_private_umask_on_python_38(self):
        source = """
from pathlib import Path
import sys

Path(sys.argv[1]).write_text('private', encoding='ascii')
"""
        with tempfile.TemporaryDirectory(dir="/tmp") as root:
            artifact = Path(root) / "child-artifact.txt"
            child = DRIVER.BoundedChild(
                [sys.executable, "-c", source, str(artifact)],
                maximum_output_bytes=64 * 1024,
                umask=0o077,
            ).start()
            output = child.wait(timeout=10.0, stop_event=None)

            self.assertEqual(0, output.returncode)
            self.assertEqual(0o600, stat.S_IMODE(artifact.stat().st_mode))
            self.assertEqual("private", artifact.read_text(encoding="ascii"))

        driver_source = SCRIPT.read_text(encoding="utf-8")
        self.assertNotIn('options["umask"]', driver_source)
        self.assertIn('umask "$1"; shift; exec "$@"', driver_source)

    def test_bounded_child_rejects_and_kills_a_detached_live_descendant(self):
        source = """
import os
from pathlib import Path
import signal
import sys
import time

marker = Path(sys.argv[1])
child = os.fork()
if child == 0:
    os.close(1)
    os.close(2)
    signal.signal(signal.SIGINT, signal.SIG_IGN)
    signal.signal(signal.SIGTERM, signal.SIG_IGN)
    marker.write_text(str(os.getpid()), encoding='ascii')
    time.sleep(60)
    os._exit(0)
deadline = time.monotonic() + 2.0
while not marker.exists() and time.monotonic() < deadline:
    time.sleep(0.01)
"""
        with tempfile.TemporaryDirectory(dir="/tmp") as root:
            marker = Path(root) / "descendant.pid"
            child = DRIVER.BoundedChild(
                [sys.executable, "-c", source, str(marker)],
                maximum_output_bytes=64 * 1024,
            ).start()
            with self.assertRaisesRegex(
                DRIVER.LoadedCleanupError, "live descendant"
            ):
                child.wait(timeout=5.0, stop_event=None)
            self.assertTrue(marker.is_file())
            self.assertFalse(process_is_live(int(marker.read_text())))

    def test_loaded_probe_failure_is_classified_without_raw_output(self):
        result = {
            "passed": False,
            "failures": [
                "ServiceException: transport_object spawn model failed",
                "cleanup failed: private detail",
            ],
        }
        output = DRIVER.MATRIX.ProcessOutput(
            1,
            "LOAD_RESULT_JSON " + json.dumps(result) + "\n",
            "LOADED_MASS_SAMPLE_JSON {}\nsecret raw diagnostic",
        )
        classified = DRIVER.classify_loaded_probe_failure(output)
        self.assertEqual(1, classified["returnCode"])
        self.assertEqual(1, classified["structuredLoadResultCount"])
        self.assertEqual(2, classified["structuredFailureCount"])
        self.assertEqual(1, classified["liveMarkerCounts"]["mass"])
        self.assertIn("ros_service_failed", classified["categories"])
        self.assertIn(
            "payload_replace_or_visibility_failed", classified["categories"]
        )
        self.assertIn("payload_cleanup_failed", classified["categories"])
        self.assertNotIn("secret", json.dumps(classified))

        supervision = DRIVER.classify_loaded_probe_failure(
            DRIVER.MATRIX.ProcessOutput(92, "", "private raw diagnostic")
        )
        self.assertIn(
            "live_marker_supervision_failed", supervision["categories"]
        )
        self.assertNotIn("private", json.dumps(supervision))

    def test_bounded_child_kills_process_group_immediately_on_overflow(self):
        source = """
import os
import sys
import time

root_path, child_path = sys.argv[1:]
with open(root_path, 'w', encoding='ascii') as stream:
    stream.write(str(os.getpid()))
pid = os.fork()
if pid == 0:
    with open(child_path, 'w', encoding='ascii') as stream:
        stream.write(str(os.getpid()))
time.sleep(0.1)
fd = 1 if pid == 0 else 2
while True:
    os.write(fd, b'x' * 65536)
"""
        with tempfile.TemporaryDirectory(dir="/tmp") as root:
            root_path = Path(root) / "root.pid"
            child_path = Path(root) / "child.pid"
            child = DRIVER.BoundedChild(
                [sys.executable, "-c", source, str(root_path), str(child_path)],
                maximum_output_bytes=64 * 1024,
            ).start()
            started = time.monotonic()
            with self.assertRaisesRegex(DRIVER.LoadedGateError, "bounded output"):
                child.wait(timeout=10.0, stop_event=None)
            self.assertLess(time.monotonic() - started, 3.0)

            deadline = time.monotonic() + 3.0
            while time.monotonic() < deadline and not child_path.exists():
                time.sleep(0.01)
            pids = [int(root_path.read_text(encoding="ascii"))]
            if child_path.exists():
                pids.append(int(child_path.read_text(encoding="ascii")))
            for pid in pids:
                state_path = Path("/proc") / str(pid) / "stat"
                deadline = time.monotonic() + 3.0
                while state_path.exists() and time.monotonic() < deadline:
                    fields = state_path.read_text(encoding="ascii").split()
                    if len(fields) > 2 and fields[2] == "Z":
                        break
                    time.sleep(0.02)
                if state_path.exists():
                    fields = state_path.read_text(encoding="ascii").split()
                    self.assertEqual("Z", fields[2])

    def test_loaded_preflight_requires_complete_continuous_monotonic_overlap(self):
        ready = DRIVER.TimedMarker(
            document={
                "schemaVersion": 1,
                "profile": "transport_crate_loaded",
                "payloadMassKg": 0.75,
            },
            observed_at=11.0,
            source_sha256="a" * 64,
        )
        mass = [
            timed_mass(11.8 + index * 0.4, index + 1)
            for index in range(22)
        ]
        active = [
            timed_active(
                11.9 + index * 0.4,
                index + 1,
                progress=0.1 + index * 0.01,
                mass_sequence=index + 1,
            )
            for index in range(22)
        ]
        report = {
            "render_measurement": {
                "warmup_seconds": 2.0,
                "sample_seconds": 5.0,
            }
        }
        overlap = DRIVER.validate_loaded_preflight_overlap(
            payload_started=10.0,
            payload_ended=25.0,
            preflight_started=12.0,
            preflight_ended=20.0,
            hls_started=13.0,
            hls_ended=18.0,
            ready=ready,
            mass_markers=mass,
            active_markers=active,
            correlated_task_id="loaded-grf-n4-1234abcd",
            preflight_document=report,
        )
        self.assertTrue(overlap["loadedProcessSpansCompletePreflight"])
        self.assertTrue(overlap["hlsMeasuredDuringLoadedPreflight"])
        self.assertGreaterEqual(overlap["guaranteedLoadedSampleOverlapSeconds"], 5.0)

        with self.assertRaisesRegex(DRIVER.LoadedGateError, "span"):
            DRIVER.validate_loaded_preflight_overlap(
                payload_started=10.0,
                payload_ended=19.0,
                preflight_started=12.0,
                preflight_ended=20.0,
                hls_started=13.0,
                hls_ended=18.0,
                ready=ready,
                mass_markers=mass,
                active_markers=active,
                correlated_task_id="loaded-grf-n4-1234abcd",
                preflight_document=report,
            )
        with self.assertRaisesRegex(DRIVER.LoadedGateError, "HLS interval"):
            DRIVER.validate_loaded_preflight_overlap(
                payload_started=10.0,
                payload_ended=25.0,
                preflight_started=12.0,
                preflight_ended=20.0,
                hls_started=15.1,
                hls_ended=20.1,
                ready=ready,
                mass_markers=mass,
                active_markers=active,
                correlated_task_id="loaded-grf-n4-1234abcd",
                preflight_document=report,
            )
        gapped = [item for item in mass if not 14.0 < item.observed_at < 16.0]
        with self.assertRaisesRegex(DRIVER.LoadedGateError, "continuous"):
            DRIVER.validate_loaded_preflight_overlap(
                payload_started=10.0,
                payload_ended=25.0,
                preflight_started=12.0,
                preflight_ended=20.0,
                hls_started=13.0,
                hls_ended=18.0,
                ready=ready,
                mass_markers=gapped,
                active_markers=active,
                correlated_task_id="loaded-grf-n4-1234abcd",
                preflight_document=report,
            )

        regressed = copy.deepcopy(active)
        regressed[8] = timed_active(
            regressed[8].observed_at,
            9,
            progress=0.01,
            mass_sequence=9,
        )
        with self.assertRaisesRegex(DRIVER.LoadedGateError, "regressed"):
            DRIVER.validate_loaded_preflight_overlap(
                payload_started=10.0,
                payload_ended=25.0,
                preflight_started=12.0,
                preflight_ended=20.0,
                hls_started=13.0,
                hls_ended=18.0,
                ready=ready,
                mass_markers=mass,
                active_markers=regressed,
                correlated_task_id="loaded-grf-n4-1234abcd",
                preflight_document=report,
            )

        phase_regressed = copy.deepcopy(active)
        phase_regressed[7].document["phase"] = "PUSH"
        phase_regressed[8].document["phase"] = "APPROACH"
        with self.assertRaisesRegex(DRIVER.LoadedGateError, "phase regressed"):
            DRIVER.validate_loaded_preflight_overlap(
                payload_started=10.0,
                payload_ended=25.0,
                preflight_started=12.0,
                preflight_ended=20.0,
                hls_started=13.0,
                hls_ended=18.0,
                ready=ready,
                mass_markers=mass,
                active_markers=phase_regressed,
                correlated_task_id="loaded-grf-n4-1234abcd",
                preflight_document=report,
            )

        foreign = list(active)
        foreign[8] = timed_active(
            foreign[8].observed_at,
            9,
            task_id="loaded-grf-n4-deadbeef",
            progress=0.2,
            mass_sequence=9,
        )
        with self.assertRaisesRegex(DRIVER.LoadedGateError, "More than one"):
            DRIVER.validate_loaded_preflight_overlap(
                payload_started=10.0,
                payload_ended=25.0,
                preflight_started=12.0,
                preflight_ended=20.0,
                hls_started=13.0,
                hls_ended=18.0,
                ready=ready,
                mass_markers=mass,
                active_markers=foreign,
                correlated_task_id="loaded-grf-n4-1234abcd",
                preflight_document=report,
            )

    def test_loaded_overlap_rejects_hidden_mass_sequence_and_high_water_forgery(self):
        ready = DRIVER.TimedMarker(
            document={
                "schemaVersion": 1,
                "profile": "transport_crate_loaded",
                "payloadMassKg": 0.75,
            },
            observed_at=11.0,
            source_sha256="a" * 64,
        )
        mass = [timed_mass(11.8 + index * 0.4, index + 1) for index in range(22)]
        active = [
            timed_active(
                11.9 + index * 0.4,
                index + 1,
                progress=0.1 + index * 0.01,
                mass_sequence=index + 1,
            )
            for index in range(22)
        ]
        arguments = {
            "payload_started": 10.0,
            "payload_ended": 25.0,
            "preflight_started": 12.0,
            "preflight_ended": 20.0,
            "hls_started": 13.0,
            "hls_ended": 18.0,
            "ready": ready,
            "mass_markers": mass,
            "active_markers": active,
            "correlated_task_id": "loaded-grf-n4-1234abcd",
            "preflight_document": {
                "render_measurement": {
                    "warmup_seconds": 2.0,
                    "sample_seconds": 5.0,
                }
            },
        }

        wrong_mass = copy.deepcopy(mass)
        wrong_mass[8].document["payloadMassKg"] = 0.25
        with self.assertRaisesRegex(DRIVER.LoadedGateError, "continuously 0.75"):
            DRIVER.validate_loaded_preflight_overlap(
                **{**arguments, "mass_markers": wrong_mass}
            )

        repeated_mass_sequence = copy.deepcopy(mass)
        repeated_mass_sequence[8].document["sequence"] = 8
        with self.assertRaisesRegex(DRIVER.LoadedGateError, "strictly monotonic"):
            DRIVER.validate_loaded_preflight_overlap(
                **{**arguments, "mass_markers": repeated_mass_sequence}
            )

        repeated_active_sequence = copy.deepcopy(active)
        repeated_active_sequence[8].document["sequence"] = 8
        with self.assertRaisesRegex(DRIVER.LoadedGateError, "strictly monotonic"):
            DRIVER.validate_loaded_preflight_overlap(
                **{**arguments, "active_markers": repeated_active_sequence}
            )

        stale_real_mass = copy.deepcopy(active)
        stale_real_mass[5].document["massSequence"] = 4
        with self.assertRaisesRegex(DRIVER.LoadedGateError, "fresh mass"):
            DRIVER.validate_loaded_preflight_overlap(
                **{**arguments, "active_markers": stale_real_mass}
            )

        extra_mass = [*mass, timed_mass(20.8, 23, mass=0.25)]
        false_reference = [
            *active,
            timed_active(
                20.9,
                23,
                phase="PUSH",
                progress=0.31,
                mass_sequence=23,
            ),
        ]
        with self.assertRaisesRegex(DRIVER.LoadedGateError, "not 0.75 kg"):
            DRIVER.validate_loaded_preflight_overlap(
                **{
                    **arguments,
                    "mass_markers": extra_mass,
                    "active_markers": false_reference,
                }
            )

        hidden_high_water = [
            timed_active(11.81, 1, progress=0.9, mass_sequence=1),
            *[
                timed_active(
                    marker.observed_at,
                    index + 2,
                    progress=marker.document["taskProgress"],
                    mass_sequence=index + 1,
                )
                for index, marker in enumerate(active)
            ],
        ]
        with self.assertRaisesRegex(DRIVER.LoadedGateError, "global high-water"):
            DRIVER.validate_loaded_preflight_overlap(
                **{**arguments, "active_markers": hidden_high_water}
            )

    def test_browser_capture_must_be_bracketed_by_correlated_loaded_push(self):
        capture = DRIVER.PushCapture(
            before=timed_push(15.0, 1),
            after=timed_push(16.0, 2),
            screenshot_started_at=15.2,
            screenshot_finished_at=15.8,
            decoded_fps_before=30.0,
            decoded_fps_after=31.0,
        )
        evidence = DRIVER.validate_push_capture(
            capture,
            correlated_task_id="loaded-grf-n4-1234abcd",
            payload_started=10.0,
            payload_ended=20.0,
            mass_markers=[timed_mass(14.9, 1), timed_mass(15.9, 2)],
        )
        self.assertTrue(evidence["allFourPushersConfirmedBeforeAndAfter"])

        wrong = DRIVER.PushCapture(
            before=capture.before,
            after=timed_push(16.0, 2, "loaded-grf-n4-deadbeef"),
            screenshot_started_at=15.2,
            screenshot_finished_at=15.8,
            decoded_fps_before=30.0,
            decoded_fps_after=31.0,
        )
        with self.assertRaisesRegex(DRIVER.LoadedGateError, "correlate"):
            DRIVER.validate_push_capture(
                wrong,
                correlated_task_id="loaded-grf-n4-1234abcd",
                payload_started=10.0,
                payload_ended=20.0,
                mass_markers=[timed_mass(14.9, 1), timed_mass(15.9, 2)],
            )

        reused_mass = DRIVER.PushCapture(
            before=capture.before,
            after=timed_push(16.0, 2, mass_sequence=1),
            screenshot_started_at=15.2,
            screenshot_finished_at=15.8,
            decoded_fps_before=30.0,
            decoded_fps_after=31.0,
        )
        with self.assertRaisesRegex(DRIVER.LoadedGateError, "fresh mass samples"):
            DRIVER.validate_push_capture(
                reused_mass,
                correlated_task_id="loaded-grf-n4-1234abcd",
                payload_started=10.0,
                payload_ended=20.0,
                mass_markers=[timed_mass(14.9, 1)],
            )

        regressed = DRIVER.PushCapture(
            before=timed_push(15.0, 1, progress=0.8),
            after=timed_push(16.0, 2, progress=0.7),
            screenshot_started_at=15.2,
            screenshot_finished_at=15.8,
            decoded_fps_before=30.0,
            decoded_fps_after=31.0,
        )
        with self.assertRaisesRegex(DRIVER.LoadedGateError, "progress regressed"):
            DRIVER.validate_push_capture(
                regressed,
                correlated_task_id="loaded-grf-n4-1234abcd",
                payload_started=10.0,
                payload_ended=20.0,
                mass_markers=[timed_mass(14.9, 1), timed_mass(15.9, 2)],
            )

        below_fps_threshold = DRIVER.PushCapture(
            before=capture.before,
            after=capture.after,
            screenshot_started_at=15.2,
            screenshot_finished_at=15.8,
            decoded_fps_before=DRIVER.MATRIX.MINIMUM_BROWSER_VIDEO_FPS - 0.1,
            decoded_fps_after=DRIVER.MATRIX.MINIMUM_BROWSER_VIDEO_FPS,
        )
        with self.assertRaisesRegex(DRIVER.LoadedGateError, "FPS threshold"):
            DRIVER.validate_push_capture(
                below_fps_threshold,
                correlated_task_id="loaded-grf-n4-1234abcd",
                payload_started=10.0,
                payload_ended=20.0,
                mass_markers=[timed_mass(14.9, 1), timed_mass(15.9, 2)],
            )

    def test_push_capture_rejects_hidden_mass_sequences_and_global_regression(self):
        capture = DRIVER.PushCapture(
            before=timed_push(15.0, 1, mass_sequence=1),
            after=timed_push(16.0, 3, mass_sequence=3),
            screenshot_started_at=15.2,
            screenshot_finished_at=15.8,
            decoded_fps_before=30.0,
            decoded_fps_after=31.0,
        )
        mass = [
            timed_mass(14.9, 1),
            timed_mass(15.4, 2, mass=0.25),
            timed_mass(15.9, 3),
        ]
        with self.assertRaisesRegex(DRIVER.LoadedGateError, "continuously 0.75"):
            DRIVER.validate_push_capture(
                capture,
                correlated_task_id="loaded-grf-n4-1234abcd",
                payload_started=10.0,
                payload_ended=20.0,
                mass_markers=mass,
                push_markers=[capture.before, capture.after],
            )

        false_reference = DRIVER.PushCapture(
            before=capture.before,
            after=timed_push(16.0, 3, mass_sequence=2),
            screenshot_started_at=15.2,
            screenshot_finished_at=15.8,
            decoded_fps_before=30.0,
            decoded_fps_after=31.0,
        )
        with self.assertRaisesRegex(DRIVER.LoadedGateError, "not 0.75 kg"):
            DRIVER.validate_push_capture(
                false_reference,
                correlated_task_id="loaded-grf-n4-1234abcd",
                payload_started=10.0,
                payload_ended=20.0,
                mass_markers=mass,
                push_markers=[false_reference.before, false_reference.after],
            )

        stale_reference_capture = DRIVER.PushCapture(
            before=timed_push(15.0, 1, mass_sequence=1),
            after=timed_push(15.8, 2, mass_sequence=2),
            screenshot_started_at=15.2,
            screenshot_finished_at=15.7,
            decoded_fps_before=30.0,
            decoded_fps_after=31.0,
        )
        with self.assertRaisesRegex(DRIVER.LoadedGateError, "continuously 0.75"):
            DRIVER.validate_push_capture(
                stale_reference_capture,
                correlated_task_id="loaded-grf-n4-1234abcd",
                payload_started=10.0,
                payload_ended=20.0,
                mass_markers=[
                    timed_mass(14.9, 1),
                    timed_mass(15.1, 2),
                    timed_mass(15.5, 3, mass=0.25),
                ],
                push_markers=[
                    stale_reference_capture.before,
                    stale_reference_capture.after,
                ],
            )

        repeated_sequence = [
            timed_push(15.0, 1, mass_sequence=1),
            timed_push(15.5, 1, mass_sequence=1),
            timed_push(16.0, 3, mass_sequence=3),
        ]
        with self.assertRaisesRegex(DRIVER.LoadedGateError, "strictly monotonic"):
            DRIVER.validate_push_capture(
                capture,
                correlated_task_id="loaded-grf-n4-1234abcd",
                payload_started=10.0,
                payload_ended=20.0,
                mass_markers=[timed_mass(14.9, 1), timed_mass(15.9, 3)],
                push_markers=repeated_sequence,
            )

        high = timed_push(14.8, 1, mass_sequence=1, progress=0.9)
        before = timed_push(15.0, 2, mass_sequence=2, progress=0.7)
        after = timed_push(16.0, 3, mass_sequence=3, progress=0.7)
        high_water_capture = DRIVER.PushCapture(
            before=before,
            after=after,
            screenshot_started_at=15.2,
            screenshot_finished_at=15.8,
            decoded_fps_before=30.0,
            decoded_fps_after=31.0,
        )
        with self.assertRaisesRegex(DRIVER.LoadedGateError, "global high-water"):
            DRIVER.validate_push_capture(
                high_water_capture,
                correlated_task_id="loaded-grf-n4-1234abcd",
                payload_started=10.0,
                payload_ended=20.0,
                mass_markers=[
                    timed_mass(14.7, 1),
                    timed_mass(14.9, 2),
                    timed_mass(15.9, 3),
                ],
                push_markers=[high, before, after],
            )

    def test_live_push_marker_requires_the_exact_four_robot_roster(self):
        class Lines:
            def __init__(self, document):
                self.document = document

            def pop_lines(self):
                value = self.document
                self.document = None
                if value is None:
                    return []
                return [
                    (
                        12.0,
                        DRIVER.PUSH_LIVE_PREFIX
                        + json.dumps(value, separators=(",", ":")),
                    )
                ]

        valid = timed_push(12.0, 1).document
        collector = DRIVER.LiveMarkerCollector()
        collector.ingest(Lines(valid))
        self.assertEqual(1, len(collector.push))

        incomplete = copy.deepcopy(valid)
        incomplete["usefulContributorIds"] = ["tb3_0", "tb3_1", "tb3_2"]
        with self.assertRaisesRegex(DRIVER.LoadedGateError, "PUSH marker"):
            DRIVER.LiveMarkerCollector().ingest(Lines(incomplete))

        wrong_model = copy.deepcopy(valid)
        wrong_model["robotModels"][-1] = "tb3_9"
        with self.assertRaisesRegex(DRIVER.LoadedGateError, "PUSH marker"):
            DRIVER.LiveMarkerCollector().ingest(Lines(wrong_model))

        stale_mass = copy.deepcopy(valid)
        stale_mass["massSampleAgeSeconds"] = 0.76
        with self.assertRaisesRegex(DRIVER.LoadedGateError, "PUSH marker"):
            DRIVER.LiveMarkerCollector().ingest(Lines(stale_mass))

    def test_live_active_marker_requires_real_task_fresh_mass_and_exact_fleet(self):
        class Lines:
            def __init__(self, document):
                self.document = document

            def pop_lines(self):
                return [
                    (
                        12.0,
                        DRIVER.GRF_ACTIVE_PREFIX
                        + json.dumps(self.document, separators=(",", ":")),
                    )
                ]

        valid = timed_active(12.0, 1).document
        collector = DRIVER.LiveMarkerCollector()
        collector.ingest(Lines(valid))
        self.assertEqual(1, len(collector.active))

        for field, value in (
            ("taskStatus", "completed"),
            ("massSampleAgeSeconds", 0.76),
            ("robotModels", ["tb3_0", "tb3_1", "tb3_2", "tb3_9"]),
            ("roster", ["tb3_0", "tb3_1", "tb3_2"]),
        ):
            invalid = copy.deepcopy(valid)
            invalid[field] = value
            with self.assertRaisesRegex(DRIVER.LoadedGateError, "active loaded-GRF"):
                DRIVER.LiveMarkerCollector().ingest(Lines(invalid))

    def test_malformed_live_marker_keeps_only_structural_diagnostics(self):
        class Lines:
            def pop_lines(self):
                interleaved = (
                    DRIVER.GRF_ACTIVE_PREFIX
                    + json.dumps(timed_active(12.0, 1).document, separators=(",", ":"))
                    + DRIVER.PAYLOAD_MASS_PREFIX
                    + "{}"
                )
                return [(12.0, interleaved)]

        with self.assertRaises(DRIVER.MalformedLiveMarker) as failure:
            DRIVER.LiveMarkerCollector().ingest(Lines())

        diagnostic = failure.exception.diagnostic
        self.assertEqual(DRIVER.GRF_ACTIVE_PREFIX.strip(), diagnostic["marker"])
        self.assertEqual(
            [DRIVER.PAYLOAD_MASS_PREFIX.strip()],
            diagnostic["embeddedKnownPrefixes"],
        )
        self.assertRegex(diagnostic["sha256"], r"^[0-9a-f]{64}$")
        self.assertFalse(diagnostic["rawRetained"])
        self.assertNotIn("taskId", json.dumps(diagnostic))

    def test_active_grf_waits_require_new_time_and_sequence_anchors(self):
        collector = DRIVER.LiveMarkerCollector()
        collector.active.extend(
            [
                timed_active(10.6, 4),
                timed_active(10.0, 5),
            ]
        )

        def initial_wait(_child, values, predicate, **_kwargs):
            self.assertIsNone(predicate(values))
            values.active.append(timed_active(10.6, 6))
            return predicate(values)

        with mock.patch.object(
            DRIVER, "wait_for_live_marker", side_effect=initial_wait
        ):
            selected = DRIVER.wait_for_loaded_grf_active(
                object(),
                collector,
                object(),
                timeout=1.0,
                after_timestamp=10.5,
                after_sequence=5,
                after_mass_sequence=5,
            )
        self.assertEqual(6, selected.document["sequence"])

        def post_wait(_child, values, predicate, **_kwargs):
            self.assertIsNone(predicate(values))
            values.active.append(timed_active(11.1, 7))
            return predicate(values)

        with mock.patch.object(
            DRIVER, "wait_for_live_marker", side_effect=post_wait
        ):
            selected = DRIVER.wait_for_loaded_grf_after(
                object(),
                collector,
                object(),
                task_id="loaded-grf-n4-1234abcd",
                timestamp=11.0,
                after_sequence=6,
                minimum_mass_sequence=7,
            )
        self.assertEqual(7, selected.document["sequence"])

    def test_profile_root_must_be_an_owned_real_directory(self):
        with tempfile.TemporaryDirectory(dir="/tmp") as root:
            actual = Path(root) / "profiles"
            actual.mkdir()
            DRIVER.require_owned_directory(actual, "profile root")
            alias = Path(root) / "alias"
            alias.symlink_to(actual, target_is_directory=True)
            with self.assertRaisesRegex(DRIVER.LoadedGateError, "owned real"):
                DRIVER.require_owned_directory(alias, "profile root")

    def test_source_has_no_headless_or_gpu_disable_launch_path(self):
        source = SCRIPT.read_text(encoding="utf-8")
        self.assertNotIn('"--headless"', source)
        self.assertNotIn('"--disable-gpu"', source)
        self.assertIn("visible.OwnedChrome", source)
        request = source.index("ui.request_viewer()")
        bind = source.index("lease_directory = MATRIX.active_viewer_lease_directory", request)
        frame = source.index("ui.wait_viewer_frame", bind)
        self.assertLess(request, bind)
        self.assertLess(bind, frame)
        self.assertIn('report["viewerStartupFailure"]', source)
        self.assertIn("umask=0o077", source)
        self.assertIn("regular_private_file(path", source)
        self.assertIn("MATRIX.cleanup_case", source)
        self.assertIn("MATRIX.HostRunLock", source)
        self.assertNotIn(".communicate(", source)
        self.assertNotIn(".removesuffix(", source)
        self.assertIn("BoundedDockerHost", source)
        self.assertNotIn("final_viewer = ui.require_interactive_hls()", source)
        self.assertIn(
            'report["viewer"]["decodedFpsAfter"] = push_capture.decoded_fps_after',
            source,
        )


if __name__ == "__main__":
    unittest.main()
