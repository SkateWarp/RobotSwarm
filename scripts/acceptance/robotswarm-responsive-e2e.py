#!/usr/bin/env python3
"""Visible, read-only responsive acceptance for the public RobotSwarm control page."""

from __future__ import annotations

import argparse
import datetime as dt
import importlib.util
import json
import os
import re
import signal
import stat
import sys
import threading
from pathlib import Path
from typing import Any


HERE = Path(__file__).resolve().parent
VISIBLE_DRIVER = HERE / "robotswarm-visible-e2e.py"


def load_visible_driver():
    """Load the existing browser driver without duplicating its security code."""
    module_name = "robotswarm_visible_e2e_shared"
    if module_name in sys.modules:
        return sys.modules[module_name]

    specification = importlib.util.spec_from_file_location(module_name, VISIBLE_DRIVER)
    if specification is None or specification.loader is None:
        raise RuntimeError("The visible RobotSwarm driver could not be loaded")
    module = importlib.util.module_from_spec(specification)
    sys.modules[module_name] = module
    specification.loader.exec_module(module)
    return module


VISIBLE = load_visible_driver()
DriverError = VISIBLE.DriverError

DEFAULT_URL = "https://rs.zerav.la/apps/GTS/realtime"
DEFAULT_CREDENTIALS = VISIBLE.DEFAULT_CREDENTIALS
DEFAULT_PORT = 9350
VIEWPORT_WIDTHS = (360, 768, 1366, 1920)
VIEWPORT_HEIGHT = 900


def validate_options(args: argparse.Namespace) -> str:
    if not args.execute_production:
        raise DriverError("Refusing to open the public site without --execute-production")
    if not re.fullmatch(r"[0-9a-f]{40}", args.deployment_commit):
        raise DriverError("The deployment commit must be a full lowercase Git SHA")
    if not 1024 <= args.port <= 65535:
        raise DriverError("The CDP port must be non-privileged")
    if not 10 <= args.page_timeout <= 300:
        raise DriverError("The page timeout must be between 10 and 300 seconds")
    if not args.chrome.is_file():
        raise DriverError("The Chrome executable was not found")

    try:
        profile_details = args.profile_root.lstat()
    except OSError as exc:
        raise DriverError("The profile root is unavailable") from exc
    if stat.S_ISLNK(profile_details.st_mode) or not stat.S_ISDIR(profile_details.st_mode):
        raise DriverError("The profile root must be a real directory")
    if profile_details.st_uid != os.getuid():
        raise DriverError("The profile root has a different owner")

    return VISIBLE.validate_site(args.url)


def load_test_account(path: Path) -> dict[str, str]:
    """Reuse account A from the strict two-user credential file."""
    credentials = VISIBLE.read_credentials(path)
    account = credentials.get("A")
    if not isinstance(account, dict):
        raise DriverError("The responsive credential file has no account A")
    return account


def apply_device_metrics(cdp: Any, width: int, height: int = VIEWPORT_HEIGHT) -> None:
    """Set an exact CSS viewport while keeping the real Chrome window visible."""
    cdp.call(
        "Emulation.setDeviceMetricsOverride",
        {
            "width": width,
            "height": height,
            "deviceScaleFactor": 1,
            "mobile": False,
            "screenWidth": width,
            "screenHeight": height,
            "positionX": 0,
            "positionY": 0,
        },
    )
    settled = cdp.evaluate(
        """
            (async () => {
                if (document.fonts?.ready) await document.fonts.ready;
                await new Promise(resolve => requestAnimationFrame(
                    () => requestAnimationFrame(resolve)
                ));
                return true;
            })()
        """,
        await_promise=True,
        timeout=30,
    )
    if settled is not True:
        raise DriverError("The responsive viewport did not settle")


def inspect_control_page(cdp: Any) -> dict[str, Any]:
    """Return bounded layout facts; no page text or account data leaves Chrome."""
    result = cdp.evaluate(
        f"""
            (async () => {{
                const panel = document.querySelector({json.dumps(VISIBLE.WORKSPACE_SELECTOR)});
                if (!panel) return null;
                panel.scrollIntoView({{block: 'start', inline: 'nearest'}});
                await new Promise(resolve => requestAnimationFrame(
                    () => requestAnimationFrame(resolve)
                ));

                const panelBox = panel.getBoundingClientRect();
                const panelStyle = getComputedStyle(panel);
                const root = document.documentElement;
                const body = document.body;
                const clientWidth = root.clientWidth;
                const scrollWidth = Math.max(root.scrollWidth, body?.scrollWidth || 0);
                const layoutViewportWidth = innerWidth;
                const visualViewportWidth = window.visualViewport?.width ?? clientWidth;
                const visibleViewportWidth = Math.max(
                    0,
                    Math.min(layoutViewportWidth, visualViewportWidth, clientWidth)
                );
                const visibleWidth = Math.max(
                    0,
                    Math.min(panelBox.right, visibleViewportWidth)
                        - Math.max(panelBox.left, 0)
                );
                const visibleHeight = Math.max(
                    0,
                    Math.min(panelBox.bottom, innerHeight) - Math.max(panelBox.top, 0)
                );

                return {{
                    layoutViewportWidth,
                    innerHeight,
                    visualViewportWidth,
                    visibleViewportWidth,
                    devicePixelRatio,
                    clientWidth,
                    scrollWidth,
                    horizontalOverflowPx: Math.max(0, scrollWidth - clientWidth),
                    panelVisible:
                        panelStyle.display !== 'none'
                        && panelStyle.visibility !== 'hidden'
                        && visibleWidth > 0
                        && visibleHeight > 0,
                    panelInsideViewport:
                        panelBox.left >= -0.5
                        && panelBox.right <= visibleViewportWidth + 0.5,
                    panelVisibleWidth: Math.round(visibleWidth),
                    panelVisibleHeight: Math.round(visibleHeight),
                }};
            }})()
        """,
        await_promise=True,
        timeout=30,
    )
    if not isinstance(result, dict):
        raise DriverError("The simulation session panel is missing")
    return result


def validate_viewport_observation(expected_width: int, observation: dict[str, Any]) -> None:
    try:
        layout_width = float(observation["layoutViewportWidth"])
        visual_width = float(observation["visualViewportWidth"])
        client_width = float(observation["clientWidth"])
        visible_width = float(observation["visibleViewportWidth"])
        overflow = float(observation["horizontalOverflowPx"])
    except (KeyError, TypeError, ValueError) as exc:
        raise DriverError("The responsive layout metrics are invalid") from exc

    dimensions = (
        f"requested={expected_width}px, layout={layout_width:g}px, "
        f"visual={visual_width:g}px, client={client_width:g}px, "
        f"visible={visible_width:g}px"
    )
    if layout_width != expected_width:
        raise DriverError(
            f"Chrome exposed a different layout viewport ({dimensions})"
        )
    usable_widths = (visual_width, client_width, visible_width)
    if any(width <= 0 or width > layout_width for width in usable_widths):
        raise DriverError(f"Chrome exposed invalid visible viewport metrics ({dimensions})")
    expected_visible_width = min(layout_width, visual_width, client_width)
    if abs(visible_width - expected_visible_width) > 0.01:
        raise DriverError(f"Chrome exposed inconsistent visible viewport metrics ({dimensions})")
    if observation.get("panelVisible") is not True:
        raise DriverError(f"The session panel is not visible ({dimensions})")
    if observation.get("panelInsideViewport") is not True:
        raise DriverError(f"The session panel is clipped horizontally ({dimensions})")
    if overflow != 0:
        raise DriverError(
            f"The control page overflows horizontally by {overflow:g}px ({dimensions})"
        )


def capture_viewport(
    ui: Any,
    output_dir: Path,
    run_id: str,
    account: dict[str, str],
    width: int,
) -> dict[str, Any]:
    apply_device_metrics(ui.cdp, width)
    ui.wait_js(
        f"Boolean(document.querySelector({json.dumps(VISIBLE.WORKSPACE_SELECTOR)}))",
        20,
        "simulation session panel",
    )
    observation = inspect_control_page(ui.cdp)
    validate_viewport_observation(width, observation)

    destination = output_dir / f"{run_id}-control-{width}px.png"
    screenshot = ui.screenshot(destination, account["email"], account["password"])
    image = VISIBLE.png_details(destination.read_bytes(), destination.name)
    if image["width"] != width:
        raise DriverError(
            f"The {width}px responsive screenshot was written at {image['width']} pixels"
        )

    return {
        "requestedCssWidth": width,
        "viewportHeight": VIEWPORT_HEIGHT,
        "observation": observation,
        "screenshot": {**screenshot, "width": image["width"], "height": image["height"]},
    }


def browser_cleanup_passed(cleanup: dict[str, Any]) -> bool:
    return (
        cleanup.get("portFree") is True
        and cleanup.get("processExited") is True
        and cleanup.get("profileRemoved") is True
    )


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Capture the authenticated public RobotSwarm control page at four CSS widths"
    )
    parser.add_argument(
        "--execute-production",
        action="store_true",
        help="confirmación obligatoria antes de abrir el sitio público",
    )
    parser.add_argument("--deployment-commit", required=True)
    parser.add_argument("--chrome", type=Path, required=True)
    parser.add_argument("--profile-root", type=Path, required=True)
    parser.add_argument("--output-dir", type=Path, required=True)
    parser.add_argument("--output-json", type=Path)
    parser.add_argument("--credentials", type=Path, default=DEFAULT_CREDENTIALS)
    parser.add_argument("--url", default=DEFAULT_URL)
    parser.add_argument("--port", type=int, default=DEFAULT_PORT)
    parser.add_argument("--page-timeout", type=float, default=90)
    return parser


def main() -> int:
    args = build_parser().parse_args()
    expected_origin = validate_options(args)
    account = load_test_account(args.credentials)
    VISIBLE.validate_secure_directory(args.output_dir)

    run_id = dt.datetime.now(dt.timezone.utc).strftime("%Y%m%dT%H%M%SZ") + f"-{os.getpid()}"
    output_json = args.output_json or args.output_dir / f"responsive-e2e-{run_id}.json"
    VISIBLE.validate_secure_directory(output_json.parent)
    profile = args.profile_root / f"robotswarm-responsive-e2e-{run_id}"
    chrome = VISIBLE.OwnedChrome(
        "A",
        args.port,
        profile,
        run_id,
        args.chrome,
        args.url,
    )
    sanitizer = VISIBLE.Sanitizer([account["email"], account["password"]])
    stop_event = threading.Event()
    ui = None
    metrics_applied = False
    interrupted = False
    failure: BaseException | None = None
    cleanup_passed = False

    report: dict[str, Any] = {
        "schemaVersion": 1,
        "runId": run_id,
        "startedAt": VISIBLE.utc_now(),
        "site": VISIBLE.clean_url(args.url),
        "deploymentCommit": args.deployment_commit,
        "productionExecution": True,
        "readOnly": True,
        "mutatingActionsRequested": False,
        "browser": {
            "visible": True,
            "headless": False,
            "gpuDisabled": False,
            "cdpPort": args.port,
            "profileOwned": True,
        },
        "requestedCssWidths": list(VIEWPORT_WIDTHS),
        "captures": [],
        "cleanup": {},
        "success": False,
    }

    def note_signal(_number: int, _frame: Any) -> None:
        nonlocal interrupted
        interrupted = True
        stop_event.set()

    previous_int = signal.signal(signal.SIGINT, note_signal)
    previous_term = signal.signal(signal.SIGTERM, note_signal)
    try:
        print("Launching one owned, visible Chrome window for responsive acceptance…", flush=True)
        chrome.launch()
        ui = VISIBLE.RobotSwarmUi(chrome, expected_origin, stop_event)
        report["browser"].update(
            {
                "product": chrome.product,
                "profile": chrome.profile.name,
                "ownedPid": chrome.process.pid if chrome.process else None,
            }
        )

        ui.navigate(args.url)
        ui.login(account["email"], account["password"])
        ui.navigate(expected_origin + "/apps/GTS/realtime")
        ui.wait_js(
            f"Boolean(document.querySelector({json.dumps(VISIBLE.WORKSPACE_SELECTOR)}))",
            args.page_timeout,
            "public simulation control page",
        )

        for width in VIEWPORT_WIDTHS:
            ui.raise_if_interrupted()
            metrics_applied = True
            capture = capture_viewport(ui, args.output_dir, run_id, account, width)
            report["captures"].append(capture)
            print(f"Control page accepted at {width}px.", flush=True)

        if interrupted:
            raise KeyboardInterrupt
        report["success"] = True
    except BaseException as exc:
        failure = exc
        report["error"] = {
            "type": type(exc).__name__,
            "message": sanitizer.text(exc),
        }
    finally:
        stop_event.clear()
        metrics_cleanup_ok = not metrics_applied
        if metrics_applied and chrome.page:
            try:
                chrome.page.call("Emulation.clearDeviceMetricsOverride")
                metrics_cleanup_ok = True
            except Exception as exc:
                report["cleanup"]["deviceMetrics"] = {
                    "cleared": False,
                    "error": sanitizer.text(exc),
                }
        if "deviceMetrics" not in report["cleanup"]:
            report["cleanup"]["deviceMetrics"] = {
                "cleared": metrics_cleanup_ok,
                "reason": "not-applied" if not metrics_applied else "cleared",
            }

        try:
            browser_cleanup = chrome.close_owned()
        except Exception as exc:
            browser_cleanup = {
                "requested": True,
                "portFree": VISIBLE.port_is_free(args.port),
                "error": sanitizer.text(exc),
            }
        report["cleanup"]["browser"] = browser_cleanup
        cleanup_passed = metrics_cleanup_ok and browser_cleanup_passed(browser_cleanup)
        report["cleanup"]["passed"] = cleanup_passed
        if not cleanup_passed:
            report["success"] = False
            report.setdefault(
                "error",
                {
                    "type": "CleanupError",
                    "message": "Owned browser or responsive emulation cleanup did not finish",
                },
            )

        report["completedAt"] = VISIBLE.utc_now()
        report = sanitizer.value(report)
        VISIBLE.write_json_secure(output_json, report)
        signal.signal(signal.SIGINT, previous_int)
        signal.signal(signal.SIGTERM, previous_term)

    account.clear()
    print(f"Sanitized report: {output_json}", flush=True)
    exit_code = VISIBLE.result_exit_code(failure, cleanup_passed)
    if failure is not None:
        print(f"Responsive acceptance failed: {sanitizer.text(failure)}", file=sys.stderr, flush=True)
    elif exit_code != 0:
        print("Responsive acceptance passed, but cleanup did not finish.", file=sys.stderr, flush=True)
    return exit_code


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except DriverError as exc:
        print(f"Responsive acceptance could not start: {exc}", file=sys.stderr)
        raise SystemExit(2)
