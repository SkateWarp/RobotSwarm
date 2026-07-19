#!/usr/bin/env python3

from pathlib import Path
import sys
import unittest


SCRIPTS = Path(__file__).resolve().parents[1] / "scripts"
sys.path.insert(0, str(SCRIPTS))

from gazebo_gui_preflight import (
    DEFAULT_GPU_PATTERN,
    PreflightError,
    REPORT_SOURCE,
    RTF_SOURCE,
    active_gpu_description,
    renderer_description,
    validate_report,
)


def good_report():
    return {
        "schema_version": 1,
        "process": {"pid": 741, "executable": "/usr/bin/gzclient-11.15.1"},
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


class GazeboGuiPreflightTests(unittest.TestCase):
    def test_accepts_visible_nvidia_client_and_keeps_fps_separate_from_rtf(self):
        report = good_report()

        errors = validate_report(report, expected_pid=741)

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


if __name__ == "__main__":
    unittest.main()
