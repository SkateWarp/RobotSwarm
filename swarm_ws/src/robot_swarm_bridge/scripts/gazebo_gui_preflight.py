#!/usr/bin/env python3
"""Run a temporary visible gzclient and verify its real renderer and FPS."""

import argparse
import json
import math
import os
from pathlib import Path
import re
import shutil
import subprocess
import sys
import tempfile
import time


REPORT_SOURCE = "gazebo::rendering::Camera::AvgFPS"
RTF_SOURCE = "gazebo.msgs.WorldStatistics delta(sim_time)/delta(real_time)"
DEFAULT_GPU_PATTERN = r"(?:nvidia|geforce|rtx)"


class PreflightError(RuntimeError):
    pass


def _number(value):
    return (
        isinstance(value, (int, float))
        and not isinstance(value, bool)
        and math.isfinite(value)
    )


def renderer_description(report):
    renderer = report.get("renderer", {})
    parts = [
        renderer.get("device", ""),
        renderer.get("vendor", ""),
        renderer.get("gl_renderer", ""),
        renderer.get("gl_vendor", ""),
    ]
    return " | ".join(part.strip() for part in parts if part and part.strip())


def active_gpu_description(report):
    """Prefer the OpenGL context over OGRE's adapter hint."""
    renderer = report.get("renderer", {})
    return (renderer.get("gl_renderer") or renderer.get("device") or "").strip()


def validate_report(
    report,
    *,
    expected_pid=None,
    min_render_fps=30.0,
    min_real_time_factor=2.8,
    gpu_pattern=DEFAULT_GPU_PATTERN,
):
    """Return every reason a completed probe cannot be accepted."""
    errors = []
    if report.get("schema_version") != 1:
        errors.append("unsupported or missing report schema")

    process = report.get("process", {})
    executable = process.get("executable", "")
    executable_name = Path(executable).name
    if not re.fullmatch(r"gzclient(?:-[0-9]+(?:\.[0-9]+)*)?", executable_name):
        errors.append("report was not produced inside gzclient")
    if expected_pid is not None and process.get("pid") != expected_pid:
        errors.append("report came from a different gzclient process")

    display = report.get("display", {})
    if not display.get("x11") and not display.get("wayland"):
        errors.append("gzclient had no X11 or Wayland display")

    camera = report.get("camera", {})
    if not camera.get("name"):
        errors.append("Gazebo did not expose an active user camera")
    if not _number(camera.get("viewport_width")) or camera.get("viewport_width", 0) <= 0:
        errors.append("Gazebo user camera has no viewport width")
    if not _number(camera.get("viewport_height")) or camera.get("viewport_height", 0) <= 0:
        errors.append("Gazebo user camera has no viewport height")

    identity = active_gpu_description(report)
    try:
        expected_gpu = re.compile(gpu_pattern, re.IGNORECASE)
    except re.error as exc:
        raise PreflightError(f"invalid GPU regular expression: {exc}") from exc
    if not identity:
        errors.append("Gazebo did not report its active OpenGL renderer")
    elif not expected_gpu.search(identity):
        errors.append(f"renderer is not the expected GPU: {identity}")

    render = report.get("render_measurement", {})
    if render.get("source") != REPORT_SOURCE:
        errors.append("render FPS was not measured from Gazebo's active user camera")
    if not _number(render.get("samples")) or render.get("samples", 0) < 2:
        errors.append("too few rendered-frame samples")
    fps = render.get("average_fps")
    if not _number(fps):
        errors.append("rendered FPS is missing")
    elif fps < min_render_fps:
        errors.append(f"rendered FPS {fps:.2f} is below {min_render_fps:.2f}")
    frame_rate = render.get("post_render_rate_fps")
    if not _number(frame_rate):
        errors.append("post-render frame rate is missing")
    elif frame_rate < min_render_fps:
        errors.append(
            f"post-render frame rate {frame_rate:.2f} is below "
            f"{min_render_fps:.2f}"
        )

    physics = report.get("physics_measurement", {})
    if physics.get("source") != RTF_SOURCE:
        errors.append("physics RTF source is missing or ambiguous")
    rtf = physics.get("real_time_factor")
    if not _number(physics.get("samples")) or physics.get("samples", 0) < 1:
        errors.append("no physics real-time-factor samples were received")
    if not _number(rtf):
        errors.append("physics real-time factor is missing")
    elif rtf < min_real_time_factor:
        errors.append(
            f"physics real-time factor {rtf:.3f} is below "
            f"{min_real_time_factor:.3f}"
        )
    return errors


def plugin_candidates():
    name = "librobotswarm_gazebo_gui_probe.so"
    configured = os.environ.get("ROBOTSWARM_GUI_PROBE_PLUGIN")
    if configured:
        yield Path(configured)

    for prefix in os.environ.get("CMAKE_PREFIX_PATH", "").split(":"):
        if prefix:
            yield Path(prefix) / "lib" / name
    for directory in os.environ.get("LD_LIBRARY_PATH", "").split(":"):
        if directory:
            yield Path(directory) / name

    rospack = shutil.which("rospack")
    if rospack:
        found = subprocess.run(
            [rospack, "find", "robot_swarm_bridge"],
            check=False,
            capture_output=True,
            text=True,
        )
        if found.returncode == 0 and found.stdout.strip():
            package = Path(found.stdout.strip()).resolve()
            if package.parent.name == "src":
                workspace = package.parent.parent
                yield workspace / "devel" / "lib" / name
                yield workspace / "install" / "lib" / name


def find_plugin(requested=None):
    if requested:
        candidate = Path(requested).expanduser().resolve()
        if candidate.is_file():
            return candidate
        raise PreflightError(f"Gazebo GUI probe plugin does not exist: {candidate}")

    seen = set()
    for candidate in plugin_candidates():
        candidate = candidate.expanduser()
        if candidate in seen:
            continue
        seen.add(candidate)
        if candidate.is_file():
            return candidate.resolve()
    raise PreflightError(
        "could not find librobotswarm_gazebo_gui_probe.so; build the catkin "
        "workspace or pass --plugin"
    )


def _tail(path, line_count=18):
    try:
        lines = path.read_text(errors="replace").splitlines()
    except OSError:
        return ""
    return "\n".join(lines[-line_count:])


def run_preflight(args):
    gzclient = shutil.which(args.gzclient)
    if not gzclient:
        raise PreflightError(f"gzclient is not executable: {args.gzclient}")
    if not os.environ.get("DISPLAY") and not os.environ.get("WAYLAND_DISPLAY"):
        raise PreflightError("DISPLAY and WAYLAND_DISPLAY are both unset")

    plugin = find_plugin(args.plugin)
    temporary_report = args.report is None
    if temporary_report:
        report_path = Path(tempfile.gettempdir()) / (
            f"robotswarm-gazebo-gui-{os.getpid()}.json"
        )
    else:
        report_path = Path(args.report).expanduser().resolve()
    report_path.parent.mkdir(parents=True, exist_ok=True)
    report_path.unlink(missing_ok=True)

    environment = os.environ.copy()
    environment.update(
        {
            "ROBOTSWARM_GUI_PROBE_REPORT": str(report_path),
            "ROBOTSWARM_GUI_PROBE_WARMUP": str(args.warmup_seconds),
            "ROBOTSWARM_GUI_PROBE_SECONDS": str(args.sample_seconds),
        }
    )

    log_handle = tempfile.NamedTemporaryFile(
        prefix="robotswarm-gzclient-", suffix=".log", delete=False
    )
    log_path = Path(log_handle.name)
    process = None
    try:
        process = subprocess.Popen(
            [gzclient, "-g", str(plugin)],
            env=environment,
            stdout=log_handle,
            stderr=subprocess.STDOUT,
        )
        log_handle.close()
        deadline = time.monotonic() + args.timeout_seconds
        while time.monotonic() < deadline:
            if report_path.is_file():
                break
            return_code = process.poll()
            if return_code is not None:
                details = _tail(log_path)
                raise PreflightError(
                    f"gzclient exited with status {return_code} before reporting"
                    + (f":\n{details}" if details else "")
                )
            time.sleep(0.1)
        else:
            details = _tail(log_path)
            raise PreflightError(
                "timed out waiting for rendered frames"
                + (f":\n{details}" if details else "")
            )

        try:
            report = json.loads(report_path.read_text())
        except (OSError, json.JSONDecodeError) as exc:
            raise PreflightError(f"could not read GUI probe report: {exc}") from exc

        errors = validate_report(
            report,
            expected_pid=process.pid,
            min_render_fps=args.min_render_fps,
            min_real_time_factor=args.min_real_time_factor,
            gpu_pattern=args.gpu_pattern,
        )
        if errors:
            raise PreflightError("; ".join(errors))
        return report, report_path
    finally:
        if process is not None and process.poll() is None:
            process.terminate()
            try:
                process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                process.kill()
                process.wait(timeout=5)
        if not log_handle.closed:
            log_handle.close()
        log_path.unlink(missing_ok=True)
        if temporary_report:
            report_path.unlink(missing_ok=True)


def build_parser():
    parser = argparse.ArgumentParser(
        description=(
            "Launch a temporary visible gzclient and verify NVIDIA rendering, "
            "rendered FPS, and physics RTF."
        )
    )
    parser.add_argument("--gzclient", default="gzclient")
    parser.add_argument("--plugin", help="path to librobotswarm_gazebo_gui_probe.so")
    parser.add_argument("--report", help="keep the raw JSON report at this path")
    parser.add_argument("--warmup-seconds", type=float, default=2.0)
    parser.add_argument("--sample-seconds", type=float, default=5.0)
    parser.add_argument("--timeout-seconds", type=float, default=25.0)
    parser.add_argument("--min-render-fps", type=float, default=30.0)
    parser.add_argument("--min-real-time-factor", type=float, default=2.8)
    parser.add_argument("--gpu-pattern", default=DEFAULT_GPU_PATTERN)
    parser.add_argument("--json", action="store_true", help="also print the full report")
    return parser


def main(argv=None):
    parser = build_parser()
    args = parser.parse_args(argv)
    for name in (
        "warmup_seconds",
        "sample_seconds",
        "timeout_seconds",
        "min_render_fps",
        "min_real_time_factor",
    ):
        if not math.isfinite(getattr(args, name)) or getattr(args, name) <= 0:
            parser.error(f"--{name.replace('_', '-')} must be greater than zero")

    try:
        report, report_path = run_preflight(args)
    except PreflightError as exc:
        print(f"Gazebo GUI preflight failed: {exc}", file=sys.stderr)
        return 1

    render = report["render_measurement"]
    physics = report["physics_measurement"]
    camera = report["camera"]
    print("Gazebo GUI preflight passed")
    print(f"  renderer: {renderer_description(report)}")
    print(
        f"  viewport: {camera['name']} "
        f"{camera['viewport_width']}x{camera['viewport_height']}"
    )
    print(
        f"  rendered FPS: {render['average_fps']:.2f} "
        f"({render['source']}, {render['sample_seconds']:.1f} s)"
    )
    print(
        f"  post-render rate: {render['post_render_rate_fps']:.2f} FPS "
        "(observed callbacks over the same window)"
    )
    print(
        f"  physics RTF: {physics['real_time_factor']:.3f} "
        f"({physics['source']}; this is not FPS)"
    )
    if args.report:
        print(f"  report: {report_path}")
    if args.json:
        print(json.dumps(report, indent=2, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
