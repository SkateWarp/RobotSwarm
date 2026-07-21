#!/usr/bin/env python3
"""Run a temporary visible gzclient and verify its real renderer and FPS."""

import argparse
import contextlib
import json
import math
import os
from pathlib import Path
import re
import shutil
import signal
import stat
import subprocess
import sys
import tempfile
import time


REPORT_SOURCE = "gazebo::rendering::Camera::AvgFPS"
RTF_SOURCE = "gazebo.msgs.WorldStatistics delta(sim_time)/delta(real_time)"
DEFAULT_GPU_PATTERN = r"(?:nvidia|geforce|rtx)"
MAXIMUM_GZCLIENT_LOG_BYTES = 1024 * 1024


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
    min_render_fps=45.0,
    min_real_time_factor=2.90,
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


def _append_log_chunk(handle, chunk, total, maximum=MAXIMUM_GZCLIENT_LOG_BYTES):
    """Write one gzclient chunk without allowing its diagnostic log to grow."""
    if maximum <= 0:
        raise ValueError("maximum log size must be positive")
    remaining = max(0, maximum - total)
    accepted = chunk[:remaining]
    if accepted:
        handle.write(accepted)
    total += len(accepted)
    if len(accepted) != len(chunk):
        handle.flush()
        raise PreflightError(
            "gzclient diagnostic output exceeded the 1 MiB safety limit"
        )
    return total


def _drain_gzclient_output(stream, handle, total):
    """Drain every currently available byte from a nonblocking child pipe."""
    while True:
        try:
            chunk = os.read(stream.fileno(), 64 * 1024)
        except BlockingIOError:
            return total
        if not chunk:
            return total
        total = _append_log_chunk(handle, chunk, total)


def _process_identity(pid):
    """Return Linux's immutable start tick for one currently live PID."""
    try:
        raw = (Path("/proc") / str(pid) / "stat").read_text(encoding="ascii")
        fields = raw[raw.rfind(")") + 2 :].split()
        return int(fields[19])
    except (IndexError, OSError, ValueError):
        return None


def _living_process_group_members(process_group):
    """List non-zombie members without trusting a possibly reused leader PID."""
    members = []
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


def _capture_owned_process_group(process):
    """Capture the new session before the child PID can be mistaken for another."""
    try:
        process_group = os.getpgid(process.pid)
    except ProcessLookupError:
        if process.poll() is None:
            raise PreflightError(
                "gzclient vanished while its private process group was captured"
            )
        # start_new_session completed before Popen returned.  The leader may
        # already be gone, but an execed descendant still owns this PGID.
        process_group = process.pid
    if process_group != process.pid or process_group == os.getpgrp():
        raise PreflightError("gzclient did not start in an isolated process group")
    identity = _process_identity(process.pid)
    if identity is None:
        if process.poll() is None:
            raise PreflightError("could not capture the gzclient process identity")
        identity = -1
    return process_group, identity


def _signal_owned_process_group(process, process_group, identity, signal_number):
    if process_group <= 1 or process_group == os.getpgrp():
        raise PreflightError("refusing to signal an unsafe gzclient process group")
    if process.poll() is None:
        current_identity = _process_identity(process.pid)
        try:
            current_group = os.getpgid(process.pid)
        except ProcessLookupError:
            current_group = None
        if current_identity != identity or current_group != process_group:
            # The real leader can exit between poll() and the /proc reads.
            # In that case its descendants still reserve the original PGID.
            if process.poll() is None:
                raise PreflightError(
                    "refusing to signal a reused gzclient PID or process group"
                )
    try:
        os.killpg(process_group, signal_number)
    except ProcessLookupError:
        return


def _wait_process_group_gone(process, process_group, timeout):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        return_code = process.poll()
        if (
            return_code is not None
            and not _living_process_group_members(process_group)
        ):
            return True
        time.sleep(0.05)
    return (
        process.poll() is not None
        and not _living_process_group_members(process_group)
    )


def _stop_owned_process_group(process, process_group, identity):
    """Stop the whole gzclient session and prove that no live member remains."""
    if (
        process.poll() is not None
        and not _living_process_group_members(process_group)
    ):
        return
    _signal_owned_process_group(
        process, process_group, identity, signal.SIGTERM
    )
    if not _wait_process_group_gone(process, process_group, 5.0):
        _signal_owned_process_group(
            process, process_group, identity, signal.SIGKILL
        )
        if not _wait_process_group_gone(process, process_group, 5.0):
            raise PreflightError(
                "the temporary gzclient process group did not disappear"
            )
    if _living_process_group_members(process_group):
        raise PreflightError("the temporary gzclient process group is still running")


def run_preflight(args):
    gzclient = shutil.which(args.gzclient)
    if not gzclient:
        raise PreflightError(f"gzclient is not executable: {args.gzclient}")
    if not os.environ.get("DISPLAY") and not os.environ.get("WAYLAND_DISPLAY"):
        raise PreflightError("DISPLAY and WAYLAND_DISPLAY are both unset")

    plugin = find_plugin(args.plugin)
    private_runtime = tempfile.TemporaryDirectory(
        prefix="robotswarm-gui-preflight-", dir="/tmp"
    )
    runtime_path = Path(private_runtime.name)
    os.chmod(runtime_path, 0o700)
    runtime_metadata = runtime_path.lstat()
    if (
        stat.S_ISLNK(runtime_metadata.st_mode)
        or not stat.S_ISDIR(runtime_metadata.st_mode)
        or runtime_metadata.st_uid != os.getuid()
        or stat.S_IMODE(runtime_metadata.st_mode) != 0o700
    ):
        private_runtime.cleanup()
        raise PreflightError("could not create a private gzclient TMPDIR")
    try:
        temporary_report = args.report is None
        if temporary_report:
            report_path = runtime_path / "gazebo-gui-report.json"
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
                "TMPDIR": str(runtime_path),
            }
        )

        log_path = runtime_path / "gzclient.log"
        log_flags = os.O_WRONLY | os.O_CREAT | os.O_EXCL
        log_flags |= getattr(os, "O_NOFOLLOW", 0)
        log_fd = os.open(log_path, log_flags, 0o600)
        log_handle = os.fdopen(log_fd, "wb", buffering=0)
    except BaseException:
        private_runtime.cleanup()
        raise
    process = None
    process_group = None
    process_identity = None
    log_size = 0

    def stop_process():
        if process is None or process_group is None or process_identity is None:
            return
        _stop_owned_process_group(process, process_group, process_identity)

    try:
        process = subprocess.Popen(
            [gzclient, "-g", str(plugin)],
            env=environment,
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            start_new_session=True,
        )
        try:
            process_group, process_identity = _capture_owned_process_group(process)
        except BaseException:
            if process.poll() is None:
                process.kill()
                process.wait(timeout=5)
            raise
        if process.stdout is None:
            raise PreflightError("gzclient did not expose its diagnostic pipe")
        os.set_blocking(process.stdout.fileno(), False)
        deadline = time.monotonic() + args.timeout_seconds
        while time.monotonic() < deadline:
            log_size = _drain_gzclient_output(
                process.stdout, log_handle, log_size
            )
            if report_path.is_file():
                break
            return_code = process.poll()
            if return_code is not None:
                log_handle.flush()
                details = _tail(log_path)
                raise PreflightError(
                    f"gzclient exited with status {return_code} before reporting"
                    + (f":\n{details}" if details else "")
                )
            time.sleep(0.1)
        else:
            log_handle.flush()
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
        stop_process()
        log_size = _drain_gzclient_output(process.stdout, log_handle, log_size)
        return report, report_path
    finally:
        stop_process()
        if process is not None and process.stdout is not None:
            with contextlib.suppress(PreflightError, OSError, ValueError):
                _drain_gzclient_output(process.stdout, log_handle, log_size)
            with contextlib.suppress(OSError):
                process.stdout.close()
        if not log_handle.closed:
            log_handle.close()
        private_runtime.cleanup()


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
    parser.add_argument("--min-render-fps", type=float, default=45.0)
    parser.add_argument("--min-real-time-factor", type=float, default=2.90)
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
