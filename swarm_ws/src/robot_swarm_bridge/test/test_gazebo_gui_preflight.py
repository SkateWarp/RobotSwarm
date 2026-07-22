#!/usr/bin/env python3

import io
import os
from pathlib import Path
import signal
import sys
import tempfile
import textwrap
import unittest
from unittest import mock


SCRIPTS = Path(__file__).resolve().parents[1] / "scripts"
sys.path.insert(0, str(SCRIPTS))

from gazebo_gui_preflight import (
    DEFAULT_GPU_PATTERN,
    MAXIMUM_GZCLIENT_LOG_BYTES,
    PreflightError,
    REPORT_ATTESTATION_PREFIX,
    REPORT_SOURCE,
    RTF_SOURCE,
    _append_log_chunk,
    _signal_owned_process_group,
    active_gpu_description,
    build_parser,
    renderer_description,
    run_preflight,
    validate_report,
)


def good_report():
    return {
        "schema_version": 1,
        "process": {
            "pid": 741,
            "executable": "/usr/bin/gzclient-11.15.1",
            "start_ticks": 9981,
        },
        "display": {"x11": ":0", "wayland": "wayland-0"},
        "camera": {
            "name": "gzclient_camera",
            "viewport_width": 1280,
            "viewport_height": 720,
        },
        "renderer": {
            "api": "OpenGL Rendering Subsystem",
            "device": "D3D12 (NVIDIA GeForce RTX 3080)",
            "vendor": "microsoft",
            "gl_vendor": "Microsoft Corporation",
            "gl_renderer": "D3D12 (NVIDIA GeForce RTX 3080)",
            "gl_version": "4.2",
        },
        "render_measurement": {
            "source": REPORT_SOURCE,
            "warmup_seconds": 2.0,
            "sample_seconds": 5.0,
            "samples": 302,
            "average_fps": 60.4,
            "post_render_rate_fps": 60.2,
        },
        "physics_measurement": {
            "source": RTF_SOURCE,
            "topic": "/gazebo/swarm_arena/world_stats",
            "samples": 5,
            "real_time_factor": 2.97,
        },
    }


def process_is_live(pid):
    path = Path("/proc") / str(pid) / "stat"
    try:
        fields = path.read_text(encoding="ascii").split()
    except OSError:
        return False
    return len(fields) > 2 and fields[2] != "Z"


class GazeboGuiPreflightTests(unittest.TestCase):
    def test_production_performance_gates_are_the_defaults(self):
        args = build_parser().parse_args([])

        self.assertEqual(45.0, args.min_render_fps)
        self.assertEqual(2.90, args.min_real_time_factor)

    def test_accepts_visible_nvidia_client_and_keeps_fps_separate_from_rtf(self):
        report = good_report()

        errors = validate_report(
            report, expected_pid=741, expected_start_ticks=9981
        )

        self.assertEqual([], errors)
        self.assertNotEqual(
            report["render_measurement"]["source"],
            report["physics_measurement"]["source"],
        )

    def test_rejects_software_rendering_even_when_fps_is_high(self):
        report = good_report()
        # The actual OpenGL context wins over a stale or generic OGRE hint.
        report["renderer"]["gl_renderer"] = "llvmpipe (LLVM 12.0.0, 256 bits)"

        errors = validate_report(report, expected_pid=741)

        self.assertTrue(any("expected GPU" in error for error in errors))

    def test_rejects_missing_rtf_and_slow_user_camera(self):
        report = good_report()
        report["render_measurement"]["average_fps"] = 18.0
        report["render_measurement"]["post_render_rate_fps"] = 18.1
        report["physics_measurement"]["samples"] = 0
        report["physics_measurement"]["real_time_factor"] = None

        errors = validate_report(report, expected_pid=741)

        self.assertTrue(any("rendered FPS" in error for error in errors))
        self.assertTrue(any("post-render frame rate" in error for error in errors))
        self.assertTrue(any("real-time-factor samples" in error for error in errors))

    def test_rejects_stale_camera_average_when_frames_arrive_slowly(self):
        report = good_report()
        report["render_measurement"]["post_render_rate_fps"] = 8.0

        errors = validate_report(report, expected_pid=741)

        self.assertTrue(any("post-render frame rate" in error for error in errors))

    def test_rejects_nonfinite_measurements(self):
        report = good_report()
        report["render_measurement"]["average_fps"] = float("nan")
        report["physics_measurement"]["real_time_factor"] = float("inf")

        errors = validate_report(report, expected_pid=741)

        self.assertTrue(any("rendered FPS is missing" in error for error in errors))
        self.assertTrue(any("real-time factor is missing" in error for error in errors))

    def test_rejects_report_from_an_existing_or_headless_process(self):
        report = good_report()
        report["process"]["pid"] = 123
        report["display"] = {"x11": "", "wayland": ""}

        errors = validate_report(report, expected_pid=741)

        self.assertTrue(any("different gzclient" in error for error in errors))
        self.assertTrue(any("display" in error for error in errors))

    def test_rejects_replayed_pid_from_an_older_process_lifetime(self):
        report = good_report()

        errors = validate_report(
            report, expected_pid=741, expected_start_ticks=9982
        )

        self.assertTrue(any("process lifetime" in error for error in errors))
        self.assertEqual(
            "ROBOTSWARM_GUI_REPORT_ATTESTATION ", REPORT_ATTESTATION_PREFIX
        )

    def test_custom_gpu_pattern_must_be_valid(self):
        with self.assertRaises(PreflightError):
            validate_report(good_report(), gpu_pattern="[")

    def test_renderer_description_includes_wslg_adapter(self):
        description = renderer_description(good_report())

        self.assertRegex(description.lower(), DEFAULT_GPU_PATTERN)
        self.assertIn("D3D12", description)
        self.assertEqual(
            "D3D12 (NVIDIA GeForce RTX 3080)",
            active_gpu_description(good_report()),
        )

    def test_diagnostic_log_writer_stops_exactly_at_its_limit(self):
        stream = io.BytesIO()
        total = _append_log_chunk(stream, b"abcd", 0, maximum=8)
        total = _append_log_chunk(stream, b"efgh", total, maximum=8)
        self.assertEqual(8, total)
        with self.assertRaisesRegex(PreflightError, "1 MiB safety limit"):
            _append_log_chunk(stream, b"i", total, maximum=8)
        self.assertEqual(b"abcdefgh", stream.getvalue())
        self.assertEqual(1024 * 1024, MAXIMUM_GZCLIENT_LOG_BYTES)

    def test_group_signalling_rejects_self_group_and_reused_pid_identity(self):
        process = mock.Mock()
        process.pid = os.getpid()
        process.poll.return_value = None
        with self.assertRaisesRegex(PreflightError, "unsafe"):
            _signal_owned_process_group(
                process, os.getpgrp(), -1, signal.SIGTERM
            )
        with self.assertRaisesRegex(PreflightError, "reused"):
            _signal_owned_process_group(
                process, os.getpid() + 100000, -1, signal.SIGTERM
            )

    def test_run_uses_a_private_ephemeral_tmpdir_for_gzclient(self):
        with tempfile.TemporaryDirectory(dir="/tmp") as root:
            root_path = Path(root)
            plugin = root_path / "probe.so"
            plugin.write_bytes(b"probe")
            report_path = root_path / "report.json"
            executable = root_path / "fake-gzclient"
            report = good_report()
            source = textwrap.dedent(
                f"""\
                #!/usr/bin/env python3
                import json
                import os
                from pathlib import Path
                import stat
                import time

                document = {report!r}
                document['process']['pid'] = os.getpid()
                raw_stat = Path('/proc/self/stat').read_text(encoding='ascii')
                document['process']['start_ticks'] = int(
                    raw_stat[raw_stat.rfind(')') + 2:].split()[19]
                )
                private_tmp = Path(os.environ['TMPDIR'])
                document['test_tmpdir'] = str(private_tmp)
                document['test_tmpdir_mode'] = stat.S_IMODE(private_tmp.stat().st_mode)
                Path(os.environ['ROBOTSWARM_GUI_PROBE_REPORT']).write_text(
                    json.dumps(document), encoding='utf-8'
                )
                time.sleep(5)
                """
            )
            executable.write_text(source, encoding="utf-8")
            executable.chmod(0o700)
            args = build_parser().parse_args(
                [
                    "--gzclient",
                    str(executable),
                    "--plugin",
                    str(plugin),
                    "--report",
                    str(report_path),
                    "--timeout-seconds",
                    "2",
                ]
            )
            with mock.patch.dict(os.environ, {"DISPLAY": ":99"}, clear=False):
                observed, _, digest = run_preflight(args)
            private_tmp = Path(observed["test_tmpdir"])
            self.assertRegex(digest, r"^[0-9a-f]{64}$")
            self.assertEqual(0o700, observed["test_tmpdir_mode"])
            self.assertFalse(private_tmp.exists())

    def test_run_kills_an_execed_descendant_after_its_leader_exits(self):
        with tempfile.TemporaryDirectory(dir="/tmp") as root:
            root_path = Path(root)
            plugin = root_path / "probe.so"
            plugin.write_bytes(b"probe")
            report_path = root_path / "report.json"
            child_marker = root_path / "child.pid"
            executable = root_path / "orphaning-gzclient"
            report = good_report()
            executable.write_text(
                textwrap.dedent(
                    f"""\
                    #!/usr/bin/env python3
                    import json
                    import os
                    from pathlib import Path
                    import signal
                    import sys

                    child = os.fork()
                    if child == 0:
                        Path({str(child_marker)!r}).write_text(
                            str(os.getpid()), encoding='ascii'
                        )
                        signal.signal(signal.SIGTERM, signal.SIG_IGN)
                        os.execl(
                            sys.executable, sys.executable, '-c',
                            'import time; time.sleep(60)',
                        )
                    document = {report!r}
                    document['process']['pid'] = os.getpid()
                    raw_stat = Path('/proc/self/stat').read_text(encoding='ascii')
                    document['process']['start_ticks'] = int(
                        raw_stat[raw_stat.rfind(')') + 2:].split()[19]
                    )
                    Path(os.environ['ROBOTSWARM_GUI_PROBE_REPORT']).write_text(
                        json.dumps(document), encoding='utf-8'
                    )
                    os._exit(0)
                    """
                ),
                encoding="utf-8",
            )
            executable.chmod(0o700)
            args = build_parser().parse_args(
                [
                    "--gzclient",
                    str(executable),
                    "--plugin",
                    str(plugin),
                    "--report",
                    str(report_path),
                    "--timeout-seconds",
                    "2",
                ]
            )
            with mock.patch.dict(os.environ, {"DISPLAY": ":99"}, clear=False):
                observed, _, digest = run_preflight(args)
            self.assertEqual(1, observed["schema_version"])
            self.assertRegex(digest, r"^[0-9a-f]{64}$")
            self.assertTrue(child_marker.is_file())
            self.assertFalse(process_is_live(int(child_marker.read_text())))

    def test_run_rejects_and_cleans_up_a_flooding_gzclient(self):
        with tempfile.TemporaryDirectory(dir="/tmp") as root:
            root_path = Path(root)
            plugin = root_path / "probe.so"
            plugin.write_bytes(b"probe")
            tmpdir_marker = plugin.with_suffix(".tmpdir")
            child_marker = plugin.with_suffix(".child-pid")
            executable = root_path / "flooding-gzclient"
            executable.write_text(
                textwrap.dedent(
                    """\
                    #!/usr/bin/env python3
                    import os
                    from pathlib import Path
                    import signal
                    import sys
                    import time

                    Path(sys.argv[2]).with_suffix('.tmpdir').write_text(
                        os.environ['TMPDIR'], encoding='utf-8'
                    )
                    child = os.fork()
                    if child == 0:
                        Path(sys.argv[2]).with_suffix('.child-pid').write_text(
                            str(os.getpid()), encoding='ascii'
                        )
                        signal.signal(signal.SIGTERM, signal.SIG_IGN)
                        os.execl(
                            sys.executable, sys.executable, '-c',
                            'import time; time.sleep(60)',
                        )
                    deadline = time.monotonic() + 1.0
                    marker = Path(sys.argv[2]).with_suffix('.child-pid')
                    while not marker.exists() and time.monotonic() < deadline:
                        time.sleep(0.01)
                    block = b'x' * 65536
                    for _ in range(18):
                        os.write(1, block)
                    """
                ),
                encoding="utf-8",
            )
            executable.chmod(0o700)
            args = build_parser().parse_args(
                [
                    "--gzclient",
                    str(executable),
                    "--plugin",
                    str(plugin),
                    "--report",
                    str(root_path / "report.json"),
                    "--timeout-seconds",
                    "2",
                ]
            )
            with mock.patch.dict(os.environ, {"DISPLAY": ":99"}, clear=False):
                with self.assertRaisesRegex(PreflightError, "1 MiB safety limit"):
                    run_preflight(args)
            self.assertTrue(tmpdir_marker.is_file())
            private_tmp = Path(tmpdir_marker.read_text(encoding="utf-8"))
            self.assertFalse(private_tmp.exists())
            self.assertTrue(child_marker.is_file())
            self.assertFalse(process_is_live(int(child_marker.read_text())))


if __name__ == "__main__":
    unittest.main()
