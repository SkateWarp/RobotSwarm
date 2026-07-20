#!/usr/bin/env bash

set -Eeuo pipefail
IFS=$'\n\t'

project_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
publisher="$project_root/deploy/gpu-worker/robotswarm-viewer-publisher"
test_root="$(mktemp -d)"
fake_bin="$test_root/bin"
fake_state="$test_root/state"
runtime_dir="$test_root/runtime"
x_socket_dir="$test_root/x11"

cleanup() {
    local child
    for child in $(jobs -pr); do
        kill -TERM "$child" 2>/dev/null || true
    done
    wait 2>/dev/null || true
    if test "${KEEP_VIEWER_TEST_ARTIFACTS:-0}" = 1; then
        echo "viewer test artifacts: $test_root" >&2
    else
        rm -rf "$test_root"
    fi
}
trap cleanup EXIT

mkdir -p "$fake_bin" "$fake_state" "$x_socket_dir"

PUBLISHER_PATH="$publisher" python3 - <<'PY'
import datetime as dt
import importlib.machinery
import importlib.util
import os
import re
import sys

loader = importlib.machinery.SourceFileLoader(
    "robotswarm_viewer_timestamp", os.environ["PUBLISHER_PATH"]
)
spec = importlib.util.spec_from_loader(loader.name, loader)
module = importlib.util.module_from_spec(spec)
sys.modules[loader.name] = module
loader.exec_module(module)

expected = dt.datetime(2030, 1, 2, 3, 4, 5, 123456, tzinfo=dt.timezone.utc)
assert module.parse_utc_timestamp("2030-01-02T03:04:05.1234567Z") == expected
assert module.parse_utc_timestamp("2030-01-02T04:04:05.1234567+01:00") == expected
assert module.parse_utc_timestamp("2030-01-02T03:04:05.1Z") == dt.datetime(
    2030, 1, 2, 3, 4, 5, 100000, tzinfo=dt.timezone.utc
)
for invalid in ("2030-01-02T03:04:05.1234567", "not-a-timestamp"):
    try:
        module.parse_utc_timestamp(invalid)
    except module.PublisherError:
        pass
    else:
        raise AssertionError(f"invalid timestamp was accepted: {invalid}")
PY

stale_display=289
stale_x_dir="$test_root/stale-x-lock"
mkdir -m 0700 "$stale_x_dir"
PUBLISHER_PATH="$publisher" \
STALE_X_DIR="$stale_x_dir" \
STALE_DISPLAY="$stale_display" \
python3 - <<'PY'
import importlib.machinery
import importlib.util
import os
import sys
from pathlib import Path

loader = importlib.machinery.SourceFileLoader(
    "robotswarm_viewer_stale_x", os.environ["PUBLISHER_PATH"]
)
spec = importlib.util.spec_from_loader(loader.name, loader)
module = importlib.util.module_from_spec(spec)
sys.modules[loader.name] = module
loader.exec_module(module)
display = int(os.environ["STALE_DISPLAY"])
assert not module.DisplayLease._x_lock_is_busy(display), (
    "test display has a live or ambiguous X lock",
    display,
)
lease = object.__new__(module.DisplayLease)
lease.session_dir = Path(os.environ["STALE_X_DIR"])
lease._write_xauthority(display)
PY
stale_auth="$stale_x_dir/Xauthority"
/usr/bin/Xvfb ":$stale_display" \
    -screen 0 64x64x24 \
    -nolisten tcp \
    -listen local \
    -auth "$stale_auth" \
    >"$stale_x_dir/first.out" \
    2>"$stale_x_dir/first.err" &
stale_x_pid=$!
for _ in $(seq 1 40); do
    if DISPLAY=":$stale_display" XAUTHORITY="$stale_auth" \
        timeout 1 xdpyinfo >/dev/null 2>&1
    then
        break
    fi
    sleep 0.05
done
DISPLAY=":$stale_display" XAUTHORITY="$stale_auth" \
    timeout 1 xdpyinfo >/dev/null
kill -KILL "$stale_x_pid"
wait "$stale_x_pid" 2>/dev/null || true
test -f "/tmp/.X${stale_display}-lock"
PUBLISHER_PATH="$publisher" \
STALE_DISPLAY="$stale_display" \
python3 - <<'PY'
import importlib.machinery
import importlib.util
import os
import sys
from pathlib import Path

loader = importlib.machinery.SourceFileLoader(
    "robotswarm_viewer_stale_x_recovery", os.environ["PUBLISHER_PATH"]
)
spec = importlib.util.spec_from_loader(loader.name, loader)
module = importlib.util.module_from_spec(spec)
sys.modules[loader.name] = module
loader.exec_module(module)
display = int(os.environ["STALE_DISPLAY"])
assert not module.DisplayLease._x_lock_is_busy(display)
assert not Path(f"/tmp/.X{display}-lock").exists()
PY
/usr/bin/Xvfb ":$stale_display" \
    -screen 0 64x64x24 \
    -nolisten tcp \
    -listen local \
    -auth "$stale_auth" \
    >"$stale_x_dir/second.out" \
    2>"$stale_x_dir/second.err" &
stale_x_pid=$!
for _ in $(seq 1 40); do
    if DISPLAY=":$stale_display" XAUTHORITY="$stale_auth" \
        timeout 1 xdpyinfo >/dev/null 2>&1
    then
        break
    fi
    sleep 0.05
done
DISPLAY=":$stale_display" XAUTHORITY="$stale_auth" \
    timeout 1 xdpyinfo >/dev/null
kill -TERM "$stale_x_pid"
wait "$stale_x_pid"
test ! -e "/tmp/.X${stale_display}-lock"

PUBLISHER_PATH="$publisher" python3 - <<'PY'
import ctypes
import io
import importlib.machinery
import importlib.util
import os
import re
import sys
import threading

loader = importlib.machinery.SourceFileLoader(
    "robotswarm_viewer_input", os.environ["PUBLISHER_PATH"]
)
spec = importlib.util.spec_from_loader(loader.name, loader)
module = importlib.util.module_from_spec(spec)
sys.modules[loader.name] = module
loader.exec_module(module)


class FakeX11:
    def __init__(self):
        self.flushes = 0

    @staticmethod
    def XStringToKeysym(symbol):
        return 1 if symbol == b"w" else 0

    @staticmethod
    def XKeysymToKeycode(_display, keysym):
        return 25 if keysym else 0

    def XFlush(self, _display):
        self.flushes += 1
        return 1


class FakeXtst:
    def __init__(self):
        self.events = []

    def XTestFakeMotionEvent(self, _display, screen, x, y, delay):
        self.events.append(("motion", screen, x, y, delay))
        return 1

    def XTestFakeButtonEvent(self, _display, button, pressed, delay):
        self.events.append(("button", button, pressed, delay))
        return 1

    def XTestFakeKeyEvent(self, _display, keycode, pressed, delay):
        self.events.append(("key", keycode, pressed, delay))
        return 1


controller = object.__new__(module.ViewerInputController)
controller.width = 1280
controller.height = 720
controller._display = 123
controller._x11 = FakeX11()
controller._xtst = FakeXtst()
controller._input_lock = threading.Lock()
controller._pressed_buttons = set()
controller._pressed_keys = set()

controller._apply({"type": "pointerMove", "x": 0.5, "y": 0.25})
controller._apply({"type": "pointerDown", "x": 0.5, "y": 0.25, "button": 0})
controller._apply({"type": "pointerUp", "x": 0.5, "y": 0.25, "button": 0})
controller._apply({"type": "wheel", "x": 0.25, "y": 0.75, "deltaX": 0, "deltaY": -120})
controller._apply({"type": "keyDown", "code": "KeyW"})
controller._apply({"type": "keyUp", "code": "KeyW"})
controller._apply({"type": "pointerDown", "x": 0.4, "y": 0.6, "button": 2})
controller._apply({"type": "keyDown", "code": "KeyW"})
controller._apply({"type": "releaseAll"})
controller._apply({"type": "releaseAll"})

assert ("motion", 0, 640, 180, 0) in controller._xtst.events
assert ("button", 1, 1, 0) in controller._xtst.events
assert ("button", 1, 0, 0) in controller._xtst.events
assert ("button", 4, 1, 0) in controller._xtst.events
assert ("key", 25, 1, 0) in controller._xtst.events
assert ("key", 25, 0, 0) in controller._xtst.events
assert not controller._pressed_buttons
assert not controller._pressed_keys
assert controller._x11.flushes == 10


class RejectFirstReleaseXtst(FakeXtst):
    def __init__(self):
        super().__init__()
        self.reject_key = True
        self.reject_button = True

    def XTestFakeButtonEvent(self, display, button, pressed, delay):
        super().XTestFakeButtonEvent(display, button, pressed, delay)
        if not pressed and self.reject_button:
            self.reject_button = False
            return 0
        return 1

    def XTestFakeKeyEvent(self, display, keycode, pressed, delay):
        super().XTestFakeKeyEvent(display, keycode, pressed, delay)
        if not pressed and self.reject_key:
            self.reject_key = False
            return 0
        return 1


controller._xtst = RejectFirstReleaseXtst()
controller._pressed_buttons = {1}
controller._pressed_keys = {25}
try:
    controller._apply({"type": "releaseAll"})
except module.PublisherError:
    pass
else:
    raise AssertionError("a rejected viewer input release was accepted")
assert controller._pressed_buttons == {1}
assert controller._pressed_keys == {25}
controller._apply({"type": "releaseAll"})
assert not controller._pressed_buttons
assert not controller._pressed_keys

invalid_events = (
    {"type": "pointerMove", "x": -0.1, "y": 0.5},
    {"type": "pointerDown", "x": 0.5, "y": 0.5, "button": 8},
    {"type": "wheel", "x": 0.5, "y": 0.5, "deltaX": 0, "deltaY": float("nan")},
    {"type": "keyDown", "code": "LaunchCalculator"},
    {"type": "keyUp", "code": "KeyW", "extra": True},
    {"type": "releaseAll", "code": "KeyW"},
)
for event in invalid_events:
    try:
        controller._apply(event)
    except module.PublisherError:
        pass
    else:
        raise AssertionError(f"invalid viewer input was accepted: {event}")

eof_controller = object.__new__(module.ViewerInputController)
eof_controller._stopping = threading.Event()
eof_controller._error = None
eof_controller._error_lock = threading.Lock()
eof_controller._read_events(io.BytesIO(b""))
assert eof_controller._error == "viewer input pipe closed"

tree = """
  0x100001 "Terminal": ("terminal" "Terminal") 800x600+0+0
  0x100002 "Qt Selection Owner for gazebo": () 3x3+0+0
  0x200001 "gazebo": ("gazebo" "gazebo") 563x348+4+5
     0x200004 "gazebo": ("gazebo" "gazebo") 560x300+0+0
"""
assert module.select_gazebo_window_id(
    tree, re.compile("Gazebo", re.IGNORECASE)
) == 0x200001
try:
    module.select_gazebo_window_id(
        tree + '\n  0x200002 "gazebo": ("gazebo" "gazebo") 640x480+0+0',
        re.compile("Gazebo", re.IGNORECASE),
    )
except module.AmbiguousGazeboWindows:
    pass
else:
    raise AssertionError("ambiguous Gazebo windows were accepted")

assert module.window_is_viewable("Map State: IsViewable\n")
assert not module.window_is_viewable("Map State: IsUnMapped\n")
assert module.window_is_normal_nontransient(
    "_NET_WM_WINDOW_TYPE(ATOM) = _NET_WM_WINDOW_TYPE_NORMAL\n"
    "WM_TRANSIENT_FOR:  not found.\n"
)
assert not module.window_is_normal_nontransient(
    "_NET_WM_WINDOW_TYPE(ATOM) = _NET_WM_WINDOW_TYPE_SPLASH, "
    "_NET_WM_WINDOW_TYPE_NORMAL\n"
    "WM_TRANSIENT_FOR(WINDOW): window id # 0x200008\n"
)

geometry = """
  Absolute upper-left X:  0
  Absolute upper-left Y:  0
  Width: 1280
  Height: 720
"""
assert module.window_fills_display(geometry, 1280, 720)
assert not module.window_fills_display(geometry, 1920, 1080)


class FakeWindowX11:
    def __init__(self):
        self.calls = []

    def XMoveResizeWindow(self, display, window, x, y, width, height):
        self.calls.append(("resize", display.value, window, x, y, width, height))
        return 1

    def XRaiseWindow(self, display, window):
        self.calls.append(("raise", display.value, window))
        return 1

    def XSetInputFocus(self, display, window, revert, timestamp):
        self.calls.append(("focus", display.value, window, revert, timestamp))
        return 1

    def XFlush(self, display):
        self.calls.append(("flush", display.value))
        return 1


window_x11 = FakeWindowX11()
module.move_resize_x_window(
    window_x11, ctypes.c_void_p(123), 0x200001, 1280, 720
)
assert window_x11.calls == [
    ("resize", 123, 0x200001, 0, 0, 1280, 720),
    ("raise", 123, 0x200001),
    ("focus", 123, 0x200001, 2, 0),
    ("flush", 123),
]
PY

cat > "$fake_bin/Xvfb" <<'PY'
#!/usr/bin/env python3
import json
import os
import sys

if "ROBOTSWARM_MEDIA_TOKEN" in os.environ:
    raise SystemExit("display inherited the media token")
if any(name.lower().startswith("worker__") for name in os.environ):
    raise SystemExit("display inherited worker identity settings")

with open(os.path.join(os.environ["FAKE_STATE"], "xvfb.log"), "a", encoding="utf-8") as stream:
    stream.write(json.dumps(sys.argv[1:]) + "\n")
os.execv("/usr/bin/Xvfb", ["/usr/bin/Xvfb", *sys.argv[1:]])
PY

cat > "$fake_bin/gzclient-worker.py" <<'PY'
#!/usr/bin/env python3
import json
import os
import signal
import sys
import time

if "ROBOTSWARM_MEDIA_TOKEN" in os.environ:
    raise SystemExit("gzclient inherited the media token")
if any(name.lower().startswith("worker__") for name in os.environ):
    raise SystemExit("gzclient inherited worker identity settings")
expected_ip = os.environ["FAKE_CONTAINER_IP"]
assert os.environ["GAZEBO_MASTER_URI"] == f"http://{expected_ip}:11345"
assert os.environ["ROS_MASTER_URI"] == f"http://{expected_ip}:11311"
assert os.environ["GZ_IP"] == os.environ["FAKE_NETWORK_GATEWAY"]
assert os.environ["GAZEBO_IP"] == os.environ["FAKE_NETWORK_GATEWAY"]
assert os.environ["DISPLAY"].startswith(":")
assert os.environ["DISPLAY"][1:].isdigit()
assert os.environ["TMPDIR"] == "/tmp"
assert os.environ["TMP"] == "/tmp"
assert os.environ["TEMP"] == "/tmp"
assert os.environ["ROBOTSWARM_GUI_RENDER_RATE"] == "50.0"
assert os.environ["MESA_D3D12_DEFAULT_ADAPTER_NAME"] == "NVIDIA"
assert os.environ["GAZEBO_MODEL_DATABASE_URI"] == ""
assert "/viewer/model-0" in os.environ["GAZEBO_MODEL_PATH"]
assert sys.argv[1:3] == ["--verbose", "-g"]
with open(os.path.join(os.environ["FAKE_STATE"], "gzclient.log"), "a", encoding="utf-8") as stream:
    stream.write(json.dumps({
        "arguments": sys.argv[1:],
        "session": os.environ["FAKE_SESSION_ID"],
        "pid": int(os.environ["FAKE_GZCLIENT_PID"]),
        "processGroup": os.getpgrp(),
        "parent": os.getppid(),
        "display": os.environ["DISPLAY"],
        "gazeboMaster": os.environ["GAZEBO_MASTER_URI"],
        "modelPath": os.environ["GAZEBO_MODEL_PATH"],
    }) + "\n")
report = {
    "schema_version": 1,
    "process": {
        "pid": int(os.environ["FAKE_GZCLIENT_PID"]),
        "executable": os.environ["FAKE_GZCLIENT_EXE"],
    },
    "display": {"x11": os.environ["DISPLAY"], "wayland": ""},
    "camera": {
        "name": "gzclient_camera(0)",
        "viewport_width": 990,
        "viewport_height": 588,
    },
    "renderer": {
        "api": "OpenGL Rendering Subsystem",
        "device": "D3D12 (NVIDIA GeForce RTX 3080)",
        "vendor": "unknown",
        "gl_vendor": "Microsoft Corporation",
        "gl_renderer": "D3D12 (NVIDIA GeForce RTX 3080)",
    },
    "render_measurement": {
        "configured_render_rate_fps": 50.0,
        "average_fps": 49.8,
        "post_render_rate_fps": 49.7,
    },
}
report_path = os.environ["ROBOTSWARM_GUI_PROBE_REPORT"]
with open(report_path + ".tmp", "w", encoding="utf-8") as stream:
    json.dump(report, stream)
os.replace(report_path + ".tmp", report_path)
stopping = False

def stop(_signum, _frame):
    global stopping
    stopping = True

signal.signal(signal.SIGINT, stop)
signal.signal(signal.SIGTERM, stop)
while not stopping:
    time.sleep(0.05)
PY

cat > "$test_root/gzclient-launcher.c" <<'C'
#define _GNU_SOURCE
#include <errno.h>
#include <limits.h>
#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/wait.h>
#include <unistd.h>

static pid_t child_pid = -1;

static void forward_signal(int signal_number)
{
    if (child_pid > 0)
        kill(child_pid, signal_number);
}

int main(int argc, char **argv)
{
    if (argc == 2 && strcmp(argv[1], "--version") == 0)
    {
        const char *banner = getenv("FAKE_GZCLIENT_VERSION_BANNER");
        const char *status = getenv("FAKE_GZCLIENT_VERSION_RC");
        puts(banner && *banner
            ? banner
            : "Gazebo multi-robot simulator, version 11.15.1");
        return status && *status ? (int)strtol(status, NULL, 10) : 0;
    }

    char executable[PATH_MAX];
    if (!realpath(argv[0], executable))
        return 70;
    char worker[PATH_MAX];
    if (snprintf(worker, sizeof(worker), "%s", executable) >= (int)sizeof(worker))
        return 70;
    char *separator = strrchr(worker, '/');
    if (!separator)
        return 70;
    strcpy(separator + 1, "gzclient-worker.py");

    char process_id[32];
    snprintf(process_id, sizeof(process_id), "%ld", (long)getpid());
    if (setenv("FAKE_GZCLIENT_PID", process_id, 1) != 0 ||
        setenv("FAKE_GZCLIENT_EXE", executable, 1) != 0)
        return 70;

    child_pid = fork();
    if (child_pid < 0)
        return 70;
    if (child_pid == 0)
    {
        char **child_arguments = calloc((size_t)argc + 2, sizeof(char *));
        if (!child_arguments)
            _exit(70);
        child_arguments[0] = "/usr/bin/python3";
        child_arguments[1] = worker;
        for (int index = 1; index < argc; ++index)
            child_arguments[index + 1] = argv[index];
        execv(child_arguments[0], child_arguments);
        _exit(70);
    }

    signal(SIGINT, forward_signal);
    signal(SIGTERM, forward_signal);
    int status = 0;
    while (waitpid(child_pid, &status, 0) < 0)
    {
        if (errno != EINTR)
            return 70;
    }
    if (WIFEXITED(status))
        return WEXITSTATUS(status);
    if (WIFSIGNALED(status))
        return 128 + WTERMSIG(status);
    return 70;
}
C
cc -O2 -Wall -Wextra -Werror "$test_root/gzclient-launcher.c" \
    -o "$fake_bin/gzclient"

cat > "$fake_bin/bwrap" <<'PY'
#!/usr/bin/env python3
import json
import os
import sys

arguments = sys.argv[1:]
with open(os.path.join(os.environ["FAKE_STATE"], "bwrap.log"), "a", encoding="utf-8") as stream:
    stream.write(json.dumps(arguments) + "\n")
if "--" in arguments:
    viewer_source = None
    for index, value in enumerate(arguments[:-2]):
        if value == "--bind" and arguments[index + 2] == "/viewer":
            viewer_source = arguments[index + 1]
            break
    if viewer_source is None:
        raise SystemExit("fake bwrap received no private /viewer bind")
    report = os.environ.get("ROBOTSWARM_GUI_PROBE_REPORT", "")
    if report.startswith("/viewer/"):
        os.environ["ROBOTSWARM_GUI_PROBE_REPORT"] = os.path.join(
            viewer_source, report[len("/viewer/"):]
        )
    command = arguments[arguments.index("--") + 1:]
else:
    command = arguments[-1:]
if not command:
    raise SystemExit("fake bwrap received no command")
os.execv(command[0], command)
PY

cat > "$fake_bin/ldd" <<'SH'
#!/bin/sh
set -eu
test "$#" = 1
if [ "${FAKE_LDD_MISSING_PATH:-}" = "$1" ]; then
    printf '%s\n' 'libX11.so.6 => not found'
    exit 0
fi
printf '%s\n' 'libgazebo_gui.so.11 => /usr/lib/libgazebo_gui.so.11'
SH

cat > "$fake_bin/docker" <<'PY'
#!/usr/bin/env python3
import json
import os
import signal
import socket
import sys
import time
from urllib.parse import urlsplit

arguments = sys.argv[1:]
state = os.environ["FAKE_STATE"]
token = os.environ.get("ROBOTSWARM_MEDIA_TOKEN")
if token:
    raise SystemExit("docker inherited the media token")
if any(name.lower().startswith("worker__") for name in os.environ):
    raise SystemExit("docker inherited worker identity settings")

with open(os.path.join(state, "docker.log"), "a", encoding="utf-8") as stream:
    stream.write(json.dumps(arguments) + "\n")

if arguments[:1] == ["version"]:
    print("27.0.0")
    raise SystemExit(0)

if arguments[:3] == ["inspect", "--type", "container"]:
    target = arguments[3]
    if target == os.environ["FAKE_CONTAINER_ID"]:
        print(json.dumps([{
            "Id": target,
            "Name": "/" + os.environ["FAKE_CONTAINER_NAME"],
            "Image": "sha256:" + "b" * 64,
            "State": {"Running": True},
            "Config": {"Labels": {
                "io.robotswarm.managed": "true",
                "io.robotswarm.session-id": os.environ["FAKE_SESSION_ID"],
                "io.robotswarm.worker-id": os.environ["FAKE_WORKER_ID"],
            }},
            "NetworkSettings": {"Networks": {
                "robotswarm-" + os.environ["FAKE_SESSION_ID"].replace("-", "") + "-net": {
                    "NetworkID": os.environ["FAKE_NETWORK_ID"],
                    "IPAddress": os.environ["FAKE_CONTAINER_IP"],
                }
            }},
        }]))
        raise SystemExit(0)
    raise SystemExit(1)

if arguments[:2] == ["network", "inspect"]:
    network_id = arguments[2]
    if network_id != os.environ["FAKE_NETWORK_ID"]:
        raise SystemExit(1)
    print(json.dumps([{
        "Id": network_id,
        "Name": "robotswarm-" + os.environ["FAKE_SESSION_ID"].replace("-", "") + "-net",
        "Internal": True,
        "Labels": {
            "io.robotswarm.managed": "true",
            "io.robotswarm.session-id": os.environ["FAKE_SESSION_ID"],
            "io.robotswarm.worker-id": os.environ["FAKE_WORKER_ID"],
        },
        "IPAM": {"Config": [{
            "Subnet": os.environ["FAKE_NETWORK_SUBNET"],
            "Gateway": os.environ["FAKE_NETWORK_GATEWAY"],
        }]},
        "Containers": {
            os.environ["FAKE_CONTAINER_ID"]: {
                "Name": os.environ["FAKE_CONTAINER_NAME"],
                "IPv4Address": (
                    os.environ["FAKE_CONTAINER_IP"]
                    + "/"
                    + os.environ["FAKE_NETWORK_PREFIX"]
                ),
            }
        },
    }]))
    raise SystemExit(0)

raise SystemExit(2)
PY

cat > "$fake_bin/ffmpeg" <<'PY'
#!/usr/bin/env python3
import json
import os
import signal
import socket
import sys
import time
from urllib.parse import urlsplit

arguments = sys.argv[1:]
if any(name.lower().startswith("worker__") for name in os.environ):
    raise SystemExit("ffmpeg inherited worker identity settings")
if arguments == ["-hide_banner", "-encoders"]:
    encoders = os.environ.get("FAKE_ENCODERS", "h264_nvenc libx264")
    if "h264_nvenc" in encoders:
        print(" V....D h264_nvenc NVIDIA NVENC H.264 encoder")
    if "libx264" in encoders:
        print(" V....D libx264 libx264 H.264 encoder")
    raise SystemExit(0)

if "lavfi" in arguments:
    selected = arguments[arguments.index("-c:v") + 1]
    assert arguments[arguments.index("-pix_fmt") + 1] == "yuv420p"
    assert arguments[arguments.index("-i") + 1] == "color=c=black:s=1280x720:r=30"
    if selected == "h264_nvenc":
        assert arguments[arguments.index("-preset") + 1] == "p4"
        assert arguments[arguments.index("-tune") + 1] == "ll"
        assert arguments[arguments.index("-rc") + 1] == "cbr"
        assert arguments[arguments.index("-bf") + 1] == "0"
    else:
        assert selected == "libx264"
        assert arguments[arguments.index("-preset") + 1] == "veryfast"
        assert arguments[arguments.index("-tune") + 1] == "zerolatency"
    if selected == "h264_nvenc" and os.environ.get("FAKE_FAIL_NVENC") == "1":
        raise SystemExit(1)
    raise SystemExit(0)

state = os.environ["FAKE_STATE"]
if "ROBOTSWARM_MEDIA_TOKEN" in os.environ:
    open(os.path.join(state, "ffmpeg-leaked-token-env"), "w").close()
with open(os.path.join(state, "ffmpeg.log"), "a", encoding="utf-8") as stream:
    stream.write(json.dumps(arguments) + "\n")
target = arguments[-1]
inherited_token = os.environ.get("ROBOTSWARM_MEDIA_TOKEN", "")
if inherited_token and inherited_token in target:
    raise SystemExit("FFmpeg received the media token in its arguments")
parsed = urlsplit(target)
connection = socket.create_connection((parsed.hostname, parsed.port), timeout=2)
for sequence, (method, path, body) in enumerate(
    (
        ("ANNOUNCE", parsed.path, b"v=0\r\na=control:streamid=0\r\n"),
        ("SETUP", parsed.path + "/streamid=0", b""),
        ("RECORD", parsed.path, b""),
    ),
    start=1,
):
    request_target = f"rtsp://{parsed.hostname}:{parsed.port}{path}"
    header = (
        f"{method} {request_target} RTSP/1.0\r\n"
        f"CSeq: {sequence}\r\n"
        f"Content-Length: {len(body)}\r\n\r\n"
    ).encode("ascii")
    connection.sendall(header + body)
    response = bytearray()
    while b"\r\n\r\n" not in response:
        chunk = connection.recv(4096)
        if not chunk:
            raise SystemExit("RTSP proxy closed before replying")
        response.extend(chunk)
    if not response.startswith(b"RTSP/1.0 200"):
        raise SystemExit("RTSP proxy returned an error")
stopping = False
traffic_mode = os.environ.get("FAKE_FFMPEG_TRAFFIC", "continuous")
if traffic_mode not in {"continuous", "none", "stall"}:
    raise SystemExit("invalid fake FFmpeg traffic mode")

def stop(_signum, _frame):
    global stopping
    stopping = True

signal.signal(signal.SIGINT, stop)
signal.signal(signal.SIGTERM, stop)

sequence = 1
while not stopping and traffic_mode != "none":
    timestamp = sequence * 3000
    rtp = (
        b"\x80\xe0"
        + sequence.to_bytes(2, "big")
        + timestamp.to_bytes(4, "big")
        + b"\x12\x34\x56\x78"
        + b"test-video-payload"
    )
    connection.sendall(b"$\x00" + len(rtp).to_bytes(2, "big") + rtp)
    sequence += 1
    if traffic_mode == "stall" and sequence > 12:
        break
    time.sleep(0.025)
while not stopping:
    time.sleep(0.05)
connection.close()
PY

cat > "$fake_bin/xwininfo" <<'PY'
#!/usr/bin/env python3
import os
import sys

if "ROBOTSWARM_MEDIA_TOKEN" in os.environ:
    raise SystemExit("xwininfo inherited the media token")
if any(name.lower().startswith("worker__") for name in os.environ):
    raise SystemExit("xwininfo inherited worker identity settings")
if sys.argv[1:] == ["-version"]:
    print("xwininfo 1.1")
elif sys.argv[1:2] == ["-id"]:
    print("""Absolute upper-left X:  0
Absolute upper-left Y:  0
Width: 1280
Height: 720
Map State: IsViewable""")
else:
    mode = os.environ.get("FAKE_XWININFO_TREE_MODE", "single")
    counter_path = os.path.join(
        os.environ["FAKE_STATE"],
        "xwininfo-" + os.environ.get("FAKE_SESSION_ID", "probe") + ".count",
    )
    try:
        with open(counter_path, encoding="ascii") as stream:
            count = int(stream.read())
    except (FileNotFoundError, ValueError):
        count = 0
    with open(counter_path, "w", encoding="ascii") as stream:
        stream.write(str(count + 1))
    print('0x100001 "Terminal": ("terminal" "Terminal") 640x480+0+0')
    print('0x100002 "Qt Selection Owner for gazebo": () 3x3+0+0')
    print('0x200001 "gazebo": ("gazebo" "gazebo") 563x348+4+5')
    print('   0x200004 "gazebo": ("gazebo" "gazebo") 560x300+0+0')
    print('0x200003 "gazebo": ("gazebo" "gazebo") 640x480+0+0')
    if mode == "persistent_multiple" or (
        mode == "transient_multiple" and count < 3
    ):
        print('0x200002 "gazebo": ("gazebo" "gazebo") 640x480+0+0')
PY

cat > "$fake_bin/xprop" <<'PY'
#!/usr/bin/env python3
import json
import os
import sys

if "ROBOTSWARM_MEDIA_TOKEN" in os.environ:
    raise SystemExit("xprop inherited the media token")
if any(name.lower().startswith("worker__") for name in os.environ):
    raise SystemExit("xprop inherited worker identity settings")
if sys.argv[1:] == ["-version"]:
    if (
        os.environ.get("FAKE_XPROP_REQUIRES_DISPLAY") == "1"
        and not os.environ.get("DISPLAY")
    ):
        raise SystemExit("xprop could not open a display")
    print("xprop 1.2")
    raise SystemExit(0)
window_id = sys.argv[sys.argv.index("-id") + 1]
with open(os.path.join(os.environ["FAKE_STATE"], "xprop.log"), "a", encoding="utf-8") as stream:
    stream.write(json.dumps({"window": window_id, "arguments": sys.argv[1:]}) + "\n")
if window_id == "0x200003":
    print(
        "_NET_WM_WINDOW_TYPE(ATOM) = _NET_WM_WINDOW_TYPE_SPLASH, "
        "_NET_WM_WINDOW_TYPE_NORMAL"
    )
    print("WM_TRANSIENT_FOR(WINDOW): window id # 0x200008")
else:
    print("_NET_WM_WINDOW_TYPE(ATOM) = _NET_WM_WINDOW_TYPE_NORMAL")
    print("WM_TRANSIENT_FOR:  not found.")
PY

cat > "$fake_bin/mediamtx-test-server" <<'PY'
#!/usr/bin/env python3
import json
import os
import re
import socket
import threading

state = os.environ["FAKE_STATE"]
server_name = os.environ.get("FAKE_MEDIA_NAME", "media")
if re.fullmatch(r"[a-z0-9-]+", server_name) is None:
    raise SystemExit("invalid fake media server name")
mode = os.environ.get("FAKE_MEDIA_MODE", "accept")
if mode not in {"accept", "reject-record"}:
    raise SystemExit("invalid fake media server mode")
connection_count = int(os.environ.get("FAKE_MEDIA_CONNECTIONS", "2"))
if connection_count < 1 or connection_count > 8:
    raise SystemExit("invalid fake media server connection count")

listener = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
listener.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
listener.bind(("127.0.0.1", 0))
listener.listen(4)
with open(
    os.path.join(state, server_name + ".port"), "w", encoding="ascii"
) as stream:
    stream.write(str(listener.getsockname()[1]))

log_lock = threading.Lock()

def receive_packet(connection, buffer):
    while len(buffer) < 1:
        chunk = connection.recv(65536)
        if not chunk:
            return None
        buffer.extend(chunk)
    if buffer[0] == ord("$"):
        while len(buffer) < 4:
            chunk = connection.recv(65536)
            if not chunk:
                return None
            buffer.extend(chunk)
        length = int.from_bytes(buffer[2:4], "big")
        while len(buffer) < 4 + length:
            chunk = connection.recv(65536)
            if not chunk:
                return None
            buffer.extend(chunk)
        packet = bytes(buffer[: 4 + length])
        del buffer[: 4 + length]
        return packet

    while b"\r\n\r\n" not in buffer:
        chunk = connection.recv(65536)
        if not chunk:
            return None
        buffer.extend(chunk)
    header_end = buffer.index(b"\r\n\r\n")
    header = bytes(buffer[:header_end])
    content_length = 0
    for line in header.split(b"\r\n")[1:]:
        name, separator, value = line.partition(b":")
        if separator and name.strip().lower() == b"content-length":
            content_length = int(value.strip())
    message_length = header_end + 4 + content_length
    while len(buffer) < message_length:
        chunk = connection.recv(65536)
        if not chunk:
            return None
        buffer.extend(chunk)
    del buffer[:message_length]
    return header

def handle(connection):
    buffer = bytearray()
    traffic_packets = 0
    try:
        while True:
            packet = receive_packet(connection, buffer)
            if packet is None:
                return
            if packet.startswith(b"$"):
                traffic_packets += 1
                continue
            lines = packet.decode("ascii").split("\r\n")
            request_line = lines[0]
            method = request_line.split(" ", 1)[0]
            cseq = next(
                line.split(":", 1)[1].strip()
                for line in lines[1:]
                if line.lower().startswith("cseq:")
            )
            with log_lock:
                with open(
                    os.path.join(state, server_name + ".log"),
                    "a",
                    encoding="utf-8",
                ) as stream:
                    stream.write(json.dumps(request_line) + "\n")
            rejected = mode == "reject-record" and method == "RECORD"
            status = "401 Unauthorized" if rejected else "200 OK"
            transport = ""
            if status == "200 OK" and method == "SETUP":
                transport = (
                    "Transport: RTP/AVP/TCP;unicast;interleaved=0-1\r\n"
                )
            connection.sendall(
                (
                    f"RTSP/1.0 {status}\r\n"
                    f"CSeq: {cseq}\r\n"
                    f"{transport}"
                    "Session: viewer-test\r\n\r\n"
                ).encode("ascii")
            )
    except OSError:
        pass
    finally:
        with log_lock:
            with open(
                os.path.join(state, server_name + ".traffic"),
                "a",
                encoding="ascii",
            ) as stream:
                stream.write(str(traffic_packets) + "\n")
        connection.close()

threads = []
for _ in range(connection_count):
    connection, _ = listener.accept()
    thread = threading.Thread(target=handle, args=(connection,))
    thread.start()
    threads.append(thread)
listener.close()
for thread in threads:
    thread.join()
PY

cat > "$fake_bin/gazebo-master-test-server" <<'PY'
#!/usr/bin/env python3
import os
import signal
import socket

listener = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
listener.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
listener.bind(("0.0.0.0", 11345))
listener.listen(16)
listener.settimeout(0.1)
with open(os.path.join(os.environ["FAKE_STATE"], "gazebo-master.ready"), "w", encoding="ascii") as stream:
    stream.write("ready\n")
stopping = False

def stop(_signum, _frame):
    global stopping
    stopping = True

signal.signal(signal.SIGINT, stop)
signal.signal(signal.SIGTERM, stop)
while not stopping:
    try:
        connection, _ = listener.accept()
    except socket.timeout:
        continue
    connection.close()
listener.close()
PY

chmod 0755 \
    "$fake_bin/Xvfb" \
    "$fake_bin/bwrap" \
    "$fake_bin/docker" \
    "$fake_bin/ffmpeg" \
    "$fake_bin/gzclient" \
    "$fake_bin/gazebo-master-test-server" \
    "$fake_bin/ldd" \
    "$fake_bin/mediamtx-test-server" \
    "$fake_bin/xwininfo" \
    "$fake_bin/xprop"
chmod 0755 "$publisher"

asset_root="$test_root/release/robotswarm-viewer-assets"
plugin_path="$test_root/release/librobotswarm_gazebo_gui_probe.so"
mkdir -p \
    "$asset_root/ros-share/turtlebot3_description/meshes/bases" \
    "$asset_root/models/turtlebot3_burger"
printf 'fake mesh\n' > \
    "$asset_root/ros-share/turtlebot3_description/meshes/bases/burger_base.stl"
printf 'fake model\n' > "$asset_root/models/turtlebot3_burger/model.sdf"
printf 'fake plugin\n' > "$plugin_path"
chmod -R u=rwX,go=rX "$test_root/release"

PUBLISHER_PATH="$publisher" python3 - <<'PY'
import copy
import importlib.machinery
import importlib.util
import os
import sys
import tempfile
from pathlib import Path

loader = importlib.machinery.SourceFileLoader(
    "robotswarm_viewer_render_report", os.environ["PUBLISHER_PATH"]
)
spec = importlib.util.spec_from_loader(loader.name, loader)
module = importlib.util.module_from_spec(spec)
sys.modules[loader.name] = module
loader.exec_module(module)

with tempfile.TemporaryDirectory(dir="/tmp") as directory:
    report_path = Path(directory) / "render.json"
    assert module.read_render_report(report_path) is None
    report_path.write_text("{not-json", encoding="utf-8")
    report_path.chmod(0o600)
    try:
        module.read_render_report(report_path)
    except module.PublisherError as exc:
        assert "malformed" in str(exc)
    else:
        raise AssertionError("malformed render report was accepted")
    target_path = Path(directory) / "target.json"
    target_path.write_text("{}", encoding="utf-8")
    target_path.chmod(0o600)
    symlink_path = Path(directory) / "symlink.json"
    symlink_path.symlink_to(target_path)
    try:
        module.read_render_report(symlink_path)
    except module.PublisherError:
        pass
    else:
        raise AssertionError("symlink render report was accepted")
    fifo_path = Path(directory) / "report.fifo"
    os.mkfifo(fifo_path, 0o600)
    try:
        module.read_render_report(fifo_path)
    except module.PublisherError:
        pass
    else:
        raise AssertionError("FIFO render report was accepted")

base = {
    "schema_version": 1,
    "process": {"pid": 42, "executable": sys.executable},
    "display": {"x11": ":230", "wayland": ""},
    "camera": {
        "name": "gzclient_camera(0)",
        "viewport_width": 990,
        "viewport_height": 588,
    },
    "renderer": {
        "api": "OpenGL Rendering Subsystem",
        "device": "D3D12 (NVIDIA GeForce RTX 3080)",
        "vendor": "unknown",
        "gl_vendor": "Microsoft Corporation",
        "gl_renderer": "D3D12 (NVIDIA GeForce RTX 3080)",
    },
    "render_measurement": {
        "configured_render_rate_fps": 50.0,
        "average_fps": 49.9,
        "post_render_rate_fps": 49.8,
    },
}

def validate(document):
    module.validate_render_report_document(
        document,
        expected_namespace_pids={42},
        expected_executable=sys.executable,
        expected_adapter="NVIDIA",
        expected_display=":230",
        expected_width=1280,
        expected_height=720,
        configured_render_rate=50.0,
        minimum_render_rate=45.0,
    )

validate(base)
cases = []
software = copy.deepcopy(base)
software["renderer"]["gl_renderer"] = "llvmpipe (LLVM 12.0)"
cases.append((software, "software renderer"))
basic = copy.deepcopy(base)
basic["renderer"]["device"] = "Microsoft Basic Render Driver"
basic["renderer"]["gl_renderer"] = "GDI Generic"
cases.append((basic, "software renderer"))
native_gl = copy.deepcopy(base)
native_gl["renderer"]["device"] = "NVIDIA GeForce RTX 3080"
native_gl["renderer"]["gl_renderer"] = "NVIDIA GeForce RTX 3080/PCIe/SSE2"
cases.append((native_gl, "required D3D12 renderer"))
low = copy.deepcopy(base)
low["render_measurement"]["average_fps"] = 44.9
cases.append((low, "below the required minimum"))
cap_mismatch = copy.deepcopy(base)
cap_mismatch["render_measurement"]["configured_render_rate_fps"] = 30.0
cases.append((cap_mismatch, "cap does not match"))
over_cap = copy.deepcopy(base)
over_cap["render_measurement"]["average_fps"] = 100.0
cases.append((over_cap, "exceeded the configured cap"))
small_viewport = copy.deepcopy(base)
small_viewport["camera"]["viewport_width"] = 563
small_viewport["camera"]["viewport_height"] = 348
cases.append((small_viewport, "does not fill"))
wrong_display = copy.deepcopy(base)
wrong_display["display"]["x11"] = ":231"
cases.append((wrong_display, "another X display"))
pid_mismatch = copy.deepcopy(base)
pid_mismatch["process"]["pid"] = 43
cases.append((pid_mismatch, "another process"))
adapter_mismatch = copy.deepcopy(base)
adapter_mismatch["renderer"]["device"] = "D3D12 (AMD Radeon)"
adapter_mismatch["renderer"]["gl_renderer"] = "D3D12 (AMD Radeon)"
cases.append((adapter_mismatch, "configured GPU adapter"))
for document, expected_message in cases:
    try:
        validate(document)
    except module.PublisherError as exc:
        assert expected_message in str(exc), str(exc)
    else:
        raise AssertionError(f"invalid render report was accepted: {expected_message}")
PY

mapfile -t network_metadata < <(python3 - <<'PY'
import ipaddress
import fcntl
import socket
import struct

addresses = set()
probe = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
for _index, name in socket.if_nameindex():
    try:
        request = struct.pack("256s", name.encode("ascii")[:15])
        raw = fcntl.ioctl(probe.fileno(), 0x8915, request)
        address = ipaddress.ip_address(socket.inet_ntoa(raw[20:24]))
    except OSError:
        continue
    if address.is_private and not address.is_loopback and not address.is_link_local:
        addresses.add(address)
probe.close()
if len(addresses) < 2:
    raise SystemExit("viewer test host needs two assigned private IPv4 addresses")
gateway, address = sorted(addresses, key=int)[:2]
different_bits = (int(gateway) ^ int(address)).bit_length()
prefix = 32 - different_bits
network = ipaddress.ip_network((int(gateway), prefix), strict=False)
print(address)
print(network)
print(network.prefixlen)
print(gateway)
PY
)
test "${#network_metadata[@]}" = 4
test_private_ip="${network_metadata[0]}"
test_private_subnet="${network_metadata[1]}"
test_private_prefix="${network_metadata[2]}"
test_private_gateway="${network_metadata[3]}"

FAKE_STATE="$fake_state" "$fake_bin/gazebo-master-test-server" &
gazebo_master_pid=$!
for _ in $(seq 1 80); do
    if test -s "$fake_state/gazebo-master.ready"; then
        break
    fi
    sleep 0.05
done
test -s "$fake_state/gazebo-master.ready"

media_token="ABCDEFGHIJKLMNOPQRSTUVWXYZ_abcdefghijklmnop"
start_media_server() {
    local name="$1"
    local mode="$2"
    local connections="$3"
    local port_file="$fake_state/$name.port"

    rm -f "$port_file"
    FAKE_STATE="$fake_state" \
        FAKE_MEDIA_NAME="$name" \
        FAKE_MEDIA_MODE="$mode" \
        FAKE_MEDIA_CONNECTIONS="$connections" \
        "$fake_bin/mediamtx-test-server" &
    last_media_pid=$!
    for _ in $(seq 1 80); do
        if test -s "$port_file"; then
            break
        fi
        sleep 0.05
    done
    test -s "$port_file"
    last_media_port="$(cat "$port_file")"
    last_media_url="rtsp://127.0.0.1:$last_media_port"
}

last_media_pid=""
last_media_port=""
last_media_url=""
start_media_server media accept 2
media_server_pid="$last_media_pid"
media_port="$last_media_port"
publish_base_url="$last_media_url"
fake_worker_id="55555555-5555-4555-8555-555555555555"
common_environment=(
    "Worker__WorkerSecret=must-not-reach-viewer-children"
    "TMPDIR=/host/not-mounted/tmpdir"
    "TMP=/mnt/c/not-mounted/tmp"
    "TEMP=/mnt/c/not-mounted/temp"
    "ROBOTSWARM_VIEWER_DOCKER=$fake_bin/docker"
    "ROBOTSWARM_VIEWER_GZCLIENT=$fake_bin/gzclient"
    "ROBOTSWARM_VIEWER_SANDBOX=$fake_bin/bwrap"
    "ROBOTSWARM_VIEWER_LDD=$fake_bin/ldd"
    "ROBOTSWARM_VIEWER_DISPLAY_SERVER=$fake_bin/Xvfb"
    "ROBOTSWARM_VIEWER_FFMPEG=$fake_bin/ffmpeg"
    "ROBOTSWARM_VIEWER_XWININFO=$fake_bin/xwininfo"
    "ROBOTSWARM_VIEWER_XPROP=$fake_bin/xprop"
    "ROBOTSWARM_VIEWER_GUI_PLUGIN=$plugin_path"
    "ROBOTSWARM_VIEWER_ASSET_ROOT=$asset_root"
    "ROBOTSWARM_VIEWER_DISPLAY_TRANSPORT=unix"
    "ROBOTSWARM_VIEWER_FILL_DISPLAY=false"
    "ROBOTSWARM_VIEWER_RENDER_RATE=50"
    "ROBOTSWARM_VIEWER_MIN_RENDER_RATE=45"
    "ROBOTSWARM_VIEWER_GPU_ADAPTER_NAME=NVIDIA"
    "ROBOTSWARM_VIEWER_RUNTIME_DIR=$runtime_dir"
    "ROBOTSWARM_VIEWER_DISPLAY_MIN=230"
    "ROBOTSWARM_VIEWER_DISPLAY_MAX=232"
    "ROBOTSWARM_VIEWER_STARTUP_TIMEOUT=5"
    "ROBOTSWARM_VIEWER_SETTLE_SECONDS=0.2"
    "ROBOTSWARM_VIEWER_TRAFFIC_TIMEOUT=1"
    "FAKE_WORKER_ID=$fake_worker_id"
    "FAKE_NETWORK_ID=dddddddddddddddddddddddddddddddddddddddddddddddddddddddddddddddd"
    "FAKE_CONTAINER_IP=$test_private_ip"
    "FAKE_NETWORK_SUBNET=$test_private_subnet"
    "FAKE_NETWORK_PREFIX=$test_private_prefix"
    "FAKE_NETWORK_GATEWAY=$test_private_gateway"
    "FAKE_STATE=$fake_state"
)

env "${common_environment[@]}" PUBLISHER_PATH="$publisher" python3 - <<'PY'
import importlib.machinery
import importlib.util
import os
import sys

loader = importlib.machinery.SourceFileLoader(
    "robotswarm_viewer_defaults", os.environ["PUBLISHER_PATH"]
)
spec = importlib.util.spec_from_loader(loader.name, loader)
module = importlib.util.module_from_spec(spec)
sys.modules[loader.name] = module
loader.exec_module(module)
os.environ.pop("ROBOTSWARM_VIEWER_DISPLAY_TRANSPORT", None)
os.environ.pop("ROBOTSWARM_VIEWER_RENDER_RATE", None)
os.environ.pop("ROBOTSWARM_VIEWER_MIN_RENDER_RATE", None)
settings = module.Settings.load()
assert settings.display_transport == "unix"
assert settings.x_socket_dir == module.Path("/tmp/.X11-unix")
assert settings.render_rate == 50.0
assert settings.minimum_render_rate == 45.0
assert settings.frame_rate == 30
PY

probe_output="$(env "${common_environment[@]}" "$publisher" probe \
    --protocol-version 2 \
    --publish-base-url "$publish_base_url")"
test "$probe_output" = \
    '{"protocolVersion":2,"ready":true,"videoCodec":"H264","sources":["Scene"],"interactive":true}'

headless_probe_output="$(env -u DISPLAY -u XAUTHORITY \
    FAKE_XPROP_REQUIRES_DISPLAY=1 \
    "${common_environment[@]}" "$publisher" probe \
        --protocol-version 2 \
        --publish-base-url "$publish_base_url")"
test "$headless_probe_output" = "$probe_output"

if FAKE_LDD_MISSING_PATH="$fake_bin/xprop" \
    env -u DISPLAY -u XAUTHORITY "${common_environment[@]}" \
    "$publisher" probe \
        --protocol-version 2 \
        --publish-base-url "$publish_base_url" \
        >"$test_root/missing-xprop-library.out" \
        2>"$test_root/missing-xprop-library.err"
then
    echo "probe accepted xprop with an unresolved host dependency" >&2
    exit 1
fi
test ! -s "$test_root/missing-xprop-library.out"
grep -Fq "xprop has unresolved host dependencies" \
    "$test_root/missing-xprop-library.err"

rc255_probe_output="$(FAKE_GZCLIENT_VERSION_RC=255 \
    env "${common_environment[@]}" "$publisher" probe \
        --protocol-version 2 \
        --publish-base-url "$publish_base_url")"
test "$rc255_probe_output" = "$probe_output"

if FAKE_GZCLIENT_VERSION_BANNER='Gazebo simulator, version 12.0.0' \
    FAKE_GZCLIENT_VERSION_RC=0 env "${common_environment[@]}" \
    "$publisher" probe \
        --protocol-version 2 \
        --publish-base-url "$publish_base_url" \
        >"$test_root/non-gazebo11.out" 2>"$test_root/non-gazebo11.err"
then
    echo "probe accepted a non-Gazebo-11 banner with a zero exit status" >&2
    exit 1
fi
test ! -s "$test_root/non-gazebo11.out"
grep -Fq "host gzclient must be Gazebo Classic 11" \
    "$test_root/non-gazebo11.err"

software_probe_output="$(FAKE_ENCODERS=libx264 \
    ROBOTSWARM_VIEWER_ENCODER=libx264 \
    env "${common_environment[@]}" "$publisher" probe \
        --protocol-version 2 \
        --publish-base-url "$publish_base_url")"
test "$software_probe_output" = "$probe_output"

if FAKE_ENCODERS="mpeg4" env "${common_environment[@]}" \
    "$publisher" probe \
        --protocol-version 2 \
        --publish-base-url "$publish_base_url" \
        >"$test_root/no-encoder.out" 2>"$test_root/no-encoder.err"
then
    echo "probe accepted FFmpeg without an H.264 encoder" >&2
    exit 1
fi
test ! -s "$test_root/no-encoder.out"
grep -Fq "no supported H.264 encoder" "$test_root/no-encoder.err"
! grep -Fq "$media_token" "$test_root/no-encoder.err"

if FAKE_FAIL_NVENC=1 ROBOTSWARM_VIEWER_ENCODER=h264_nvenc \
    env "${common_environment[@]}" "$publisher" probe \
        --protocol-version 2 \
        --publish-base-url "$publish_base_url" \
        >"$test_root/bad-nvenc.out" 2>"$test_root/bad-nvenc.err"
then
    echo "probe accepted an NVENC encoder that could not initialize" >&2
    exit 1
fi
test ! -s "$test_root/bad-nvenc.out"
grep -Fq "cannot initialize h264_nvenc" "$test_root/bad-nvenc.err"

run_publisher() {
    local session_id="$1"
    local lease_id="$2"
    local container_id="$3"
    local output_file="$4"
    local error_file="$5"
    local target_url="${6:-$publish_base_url}"
    local traffic_mode="${7:-continuous}"
    local window_mode="${8:-single}"
    local compact_session="${session_id//-/}"
    local source_id="scene-${compact_session}"
    local container_name="robotswarm-${compact_session}"
    local input_file="$fake_state/input-${lease_id//-/}.fifo"
    local input_fd
    local expires_at
    expires_at="$(date --utc --date='+5 minutes' '+%Y-%m-%dT%H:%M:%SZ')"

    mkfifo "$input_file"
    env "${common_environment[@]}" \
        "FAKE_SESSION_ID=$session_id" \
        "FAKE_CONTAINER_ID=$container_id" \
        "FAKE_CONTAINER_NAME=$container_name" \
        "FAKE_FFMPEG_TRAFFIC=$traffic_mode" \
        "FAKE_XWININFO_TREE_MODE=$window_mode" \
        "$publisher" publish \
            --protocol-version 2 \
            --session-id "$session_id" \
            --worker-id "$fake_worker_id" \
            --lease-id "$lease_id" \
            --expires-at "$expires_at" \
            --container-id "$container_id" \
            --container-name "$container_name" \
            --source Scene \
            --source-id "$source_id" \
            --stream-path "session/${compact_session}/${source_id}" \
            --publish-base-url "$target_url" \
            <"$input_file" >"$output_file" 2>"$error_file" &
    last_pid=$!
    exec {input_fd}>"$input_file"
    printf '%s\n' "$media_token" >&"$input_fd"
    last_input_fd="$input_fd"
}

wait_ready() {
    local process_id="$1"
    local output_file="$2"
    local attempt
    for attempt in $(seq 1 120); do
        if grep -Fxq READY "$output_file"; then
            return 0
        fi
        if ! kill -0 "$process_id" 2>/dev/null; then
            echo "publisher exited before READY" >&2
            return 1
        fi
        sleep 0.05
    done
    echo "publisher did not report READY" >&2
    return 1
}

session_one="11111111-1111-4111-8111-111111111111"
session_two="22222222-2222-4222-8222-222222222222"
lease_one="33333333-3333-4333-8333-333333333333"
lease_two="44444444-4444-4444-8444-444444444444"
container_one="aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa"
container_two="cccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccc"

last_pid=""
last_input_fd=""
run_publisher "$session_one" "$lease_one" "$container_one" \
    "$test_root/one.out" "$test_root/one.err"
pid_one="$last_pid"
input_fd_one="$last_input_fd"
run_publisher "$session_two" "$lease_two" "$container_two" \
    "$test_root/two.out" "$test_root/two.err"
pid_two="$last_pid"
input_fd_two="$last_input_fd"

wait_ready "$pid_one" "$test_root/one.out"
wait_ready "$pid_two" "$test_root/two.out"
test ! -s "$test_root/one.err"
test ! -s "$test_root/two.err"

printf '%s\n' \
    '{"type":"pointerMove","x":0.25,"y":0.75}' \
    '{"type":"pointerDown","x":0.25,"y":0.75,"button":0}' \
    '{"type":"pointerUp","x":0.25,"y":0.75,"button":0}' \
    '{"type":"keyDown","code":"KeyW"}' \
    '{"type":"keyUp","code":"KeyW"}' \
    >&"$input_fd_one"
printf '%s\n' \
    '{"type":"pointerMove","x":0.75,"y":0.25}' \
    >&"$input_fd_two"
sleep 0.1

python3 - \
    "$fake_state/gzclient.log" \
    "$runtime_dir" \
    "$session_one" "$lease_one" 320 539 \
    "$session_two" "$lease_two" 959 180 <<'PY'
import ctypes
import json
import os
import sys
from pathlib import Path

with open(sys.argv[1], encoding="utf-8") as stream:
    displays = {
        record["session"]: record["display"]
        for record in map(json.loads, stream)
    }
runtime = Path(sys.argv[2])
x11 = ctypes.CDLL("libX11.so.6")
x11.XOpenDisplay.argtypes = [ctypes.c_char_p]
x11.XOpenDisplay.restype = ctypes.c_void_p
x11.XDefaultRootWindow.argtypes = [ctypes.c_void_p]
x11.XDefaultRootWindow.restype = ctypes.c_ulong
x11.XQueryPointer.argtypes = [
    ctypes.c_void_p,
    ctypes.c_ulong,
    ctypes.POINTER(ctypes.c_ulong),
    ctypes.POINTER(ctypes.c_ulong),
    ctypes.POINTER(ctypes.c_int),
    ctypes.POINTER(ctypes.c_int),
    ctypes.POINTER(ctypes.c_int),
    ctypes.POINTER(ctypes.c_int),
    ctypes.POINTER(ctypes.c_uint),
]
x11.XQueryPointer.restype = ctypes.c_int
x11.XCloseDisplay.argtypes = [ctypes.c_void_p]
x11.XCloseDisplay.restype = ctypes.c_int

arguments = sys.argv[3:]
assert len(arguments) % 4 == 0
for index in range(0, len(arguments), 4):
    session, lease, expected_x, expected_y = (
        arguments[index],
        arguments[index + 1],
        int(arguments[index + 2]),
        int(arguments[index + 3]),
    )
    display_name = displays[session]
    os.environ["XAUTHORITY"] = str(
        runtime / f"lease-{lease.replace('-', '')}" / "Xauthority"
    )
    display = x11.XOpenDisplay(display_name.encode("ascii"))
    assert display, display_name
    root = x11.XDefaultRootWindow(display)
    root_return = ctypes.c_ulong()
    child_return = ctypes.c_ulong()
    root_x = ctypes.c_int()
    root_y = ctypes.c_int()
    window_x = ctypes.c_int()
    window_y = ctypes.c_int()
    mask = ctypes.c_uint()
    try:
        assert x11.XQueryPointer(
            display,
            root,
            ctypes.byref(root_return),
            ctypes.byref(child_return),
            ctypes.byref(root_x),
            ctypes.byref(root_y),
            ctypes.byref(window_x),
            ctypes.byref(window_y),
            ctypes.byref(mask),
        )
        assert (root_x.value, root_y.value) == (expected_x, expected_y), (
            session,
            display_name,
            root_x.value,
            root_y.value,
        )
        assert mask.value == 0, (session, mask.value)
    finally:
        x11.XCloseDisplay(display)
PY

# WSLg prevents Xvfb from creating filesystem sockets in /tmp/.X11-unix, but
# the explicit local transport must provide authenticated abstract Unix
# endpoints and must never expose the conventional TCP ports.
python3 - <<'PY'
import socket

for display in (230, 231):
    unix_client = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
    unix_client.settimeout(1)
    try:
        unix_client.connect(f"\0/tmp/.X11-unix/X{display}")
    finally:
        unix_client.close()
    try:
        socket.create_connection(("127.0.0.1", 6000 + display), timeout=0.2)
    except OSError:
        pass
    else:
        raise AssertionError(f"Xvfb :{display} exposed an X11 TCP listener")
PY

kill -TERM "$pid_one" "$pid_two"
wait "$pid_one"
wait "$pid_two"
exec {input_fd_one}>&-
exec {input_fd_two}>&-
wait "$media_server_pid"

python3 - <<'PY'
import socket
from pathlib import Path

for display in (230, 231):
    client = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
    client.settimeout(0.2)
    try:
        client.connect(f"\0/tmp/.X11-unix/X{display}")
    except OSError:
        pass
    else:
        raise AssertionError(f"Xvfb :{display} survived publisher cleanup")
    finally:
        client.close()
    assert not Path(f"/tmp/.X{display}-lock").exists()
PY

test "$(cat "$test_root/one.out")" = READY
test "$(cat "$test_root/two.out")" = READY
test ! -e "$fake_state/ffmpeg-leaked-token-env"
test ! -d "$runtime_dir/lease-${lease_one//-/}"
test ! -d "$runtime_dir/lease-${lease_two//-/}"

python3 - "$fake_state/ffmpeg.log" "$media_token" <<'PY'
import json
import sys
from urllib.parse import urlsplit

with open(sys.argv[1], encoding="utf-8") as stream:
    commands = [json.loads(line) for line in stream]
assert len(commands) == 2, commands
inputs = {command[command.index("-i") + 1] for command in commands}
assert inputs == {":230.0", ":231.0"}, inputs
for command in commands:
    assert command[command.index("-c:v") + 1] == "h264_nvenc"
    serialized = json.dumps(command)
    assert sys.argv[2] not in serialized
    target = urlsplit(command[-1])
    assert target.scheme == "rtsp"
    assert target.hostname == "127.0.0.1"
    assert target.port is not None
    assert target.path.startswith("/session/")
    assert not target.query
PY

python3 - "$fake_state/media.log" "$media_token" "$media_port" <<'PY'
import json
import sys
from collections import Counter
from urllib.parse import parse_qs, urlsplit

with open(sys.argv[1], encoding="utf-8") as stream:
    request_lines = [json.loads(line) for line in stream]
assert len(request_lines) == 6, request_lines
methods = Counter()
session_paths = set()
for request_line in request_lines:
    method, target_text, protocol = request_line.split(" ")
    target = urlsplit(target_text)
    assert protocol == "RTSP/1.0"
    assert target.scheme == "rtsp"
    assert target.hostname == "127.0.0.1"
    assert target.port == int(sys.argv[3])
    assert parse_qs(target.query) == {"token": [sys.argv[2]]}
    methods[method] += 1
    session_paths.add("/".join(target.path.split("/")[:4]))
assert methods == {"ANNOUNCE": 2, "SETUP": 2, "RECORD": 2}, methods
assert len(session_paths) == 2, session_paths
PY

python3 - "$fake_state/media.traffic" <<'PY'
import sys

with open(sys.argv[1], encoding="ascii") as stream:
    packet_counts = [int(line) for line in stream]
assert len(packet_counts) == 2, packet_counts
assert all(count >= 3 for count in packet_counts), packet_counts
PY

python3 - "$fake_state/docker.log" "$media_token" "$container_one" "$container_two" <<'PY'
import json
import sys

with open(sys.argv[1], encoding="utf-8") as stream:
    commands = [json.loads(line) for line in stream]
serialized = json.dumps(commands)
assert sys.argv[2] not in serialized
assert not any(
    command and command[0] in {"run", "create", "start", "stop", "rm", "exec"}
    for command in commands
), commands
container_inspects = [
    command for command in commands
    if command[:3] == ["inspect", "--type", "container"]
]
assert {command[3] for command in container_inspects} == {sys.argv[3], sys.argv[4]}
network_inspects = [
    command for command in commands if command[:2] == ["network", "inspect"]
]
assert len(network_inspects) == 2, network_inspects
assert all(command[2] == "d" * 64 for command in network_inspects)
PY

python3 - "$fake_state/bwrap.log" "$fake_bin/gzclient" "$media_token" <<'PY'
import json
import sys

with open(sys.argv[1], encoding="utf-8") as stream:
    commands = [json.loads(line) for line in stream]
publish_commands = [
    command for command in commands
    if "--" in command and sys.argv[2] in command
]
assert len(publish_commands) == 2, publish_commands
for command in publish_commands:
    assert "--unshare-all" in command
    assert "--share-net" in command
    assert "--new-session" in command
    assert "--die-with-parent" in command
    assert ["--ro-bind", "/", "/"] not in [
        command[index:index + 3] for index in range(len(command) - 2)
    ]
    assert ["--ro-bind", "/home", "/home"] not in [
        command[index:index + 3] for index in range(len(command) - 2)
    ]
    assert ["--ro-bind", "/mnt", "/mnt"] not in [
        command[index:index + 3] for index in range(len(command) - 2)
    ]
    assert not any("docker.sock" in value for value in command)
    assert "/viewer" in command
    assert "/viewer/plugin.so" in command
    assert "/opt/ros/noetic/share" in command
    serialized = json.dumps(command)
    assert sys.argv[3] not in serialized
PY

python3 - "$fake_state/gzclient.log" "$media_token" <<'PY'
import json
import sys

with open(sys.argv[1], encoding="utf-8") as stream:
    processes = [json.loads(line) for line in stream]
assert len(processes) == 2, processes
assert len({process["processGroup"] for process in processes}) == 2
for process in processes:
    assert process["display"] in {":230", ":231"}
    assert process["gazeboMaster"].endswith(":11345")
    assert "/viewer/model-0" in process["modelPath"]
    assert sys.argv[2] not in json.dumps(process)
PY

python3 - "$fake_state/xvfb.log" <<'PY'
import json
import sys

with open(sys.argv[1], encoding="utf-8") as stream:
    commands = [json.loads(line) for line in stream]
assert len(commands) == 2, commands
assert {command[0] for command in commands} == {":230", ":231"}
for command in commands:
    assert command[command.index("-nolisten") + 1] == "tcp"
    assert command[command.index("-listen") + 1] == "local"
    assert "-extension" not in command
    assert "-auth" in command
PY

python3 - "$fake_state/xprop.log" <<'PY'
import json
import sys

with open(sys.argv[1], encoding="utf-8") as stream:
    windows = {json.loads(line)["window"] for line in stream}
assert "0x200001" in windows
assert "0x200003" in windows
assert "0x200004" not in windows, windows
PY

# A persistent splash and transient extra normal window can coexist with the
# main window.  Startup must ignore splash/nested widgets and wait until one
# stable, viewable, non-transient NORMAL top-level remains.
session_transient="61616161-6161-4161-8161-616161616161"
lease_transient="62626262-6262-4262-8262-626262626262"
container_transient="6161616161616161616161616161616161616161616161616161616161616161"
start_media_server transient-window accept 1
transient_server_pid="$last_media_pid"
run_publisher \
    "$session_transient" \
    "$lease_transient" \
    "$container_transient" \
    "$test_root/transient-window.out" \
    "$test_root/transient-window.err" \
    "$last_media_url" \
    continuous \
    transient_multiple
transient_publisher_pid="$last_pid"
transient_input_fd="$last_input_fd"
wait_ready "$transient_publisher_pid" "$test_root/transient-window.out"
kill -TERM "$transient_publisher_pid"
wait "$transient_publisher_pid"
exec {transient_input_fd}>&-
wait "$transient_server_pid"
test "$(cat "$test_root/transient-window.out")" = READY
test ! -s "$test_root/transient-window.err"
test "$(cat "$fake_state/xwininfo-$session_transient.count")" -ge 6

# Persistent ambiguity must consume only the bounded startup interval and
# return a specific diagnostic containing the last observed tree.
session_ambiguous="63636363-6363-4363-8363-636363636363"
lease_ambiguous="64646464-6464-4464-8464-646464646464"
container_ambiguous="6363636363636363636363636363636363636363636363636363636363636363"
run_publisher \
    "$session_ambiguous" \
    "$lease_ambiguous" \
    "$container_ambiguous" \
    "$test_root/persistent-window.out" \
    "$test_root/persistent-window.err" \
    "$publish_base_url" \
    continuous \
    persistent_multiple
ambiguous_publisher_pid="$last_pid"
ambiguous_input_fd="$last_input_fd"
for _ in $(seq 1 140); do
    if ! kill -0 "$ambiguous_publisher_pid" 2>/dev/null; then
        break
    fi
    sleep 0.05
done
if kill -0 "$ambiguous_publisher_pid" 2>/dev/null; then
    echo "publisher did not bound persistent Gazebo-window ambiguity" >&2
    exit 1
fi
if wait "$ambiguous_publisher_pid"; then
    echo "publisher accepted persistent Gazebo-window ambiguity" >&2
    exit 1
fi
exec {ambiguous_input_fd}>&-
test ! -s "$test_root/persistent-window.out"
grep -Fq \
    "did not converge to one stable viewable Gazebo main window" \
    "$test_root/persistent-window.err"
grep -Fq '0x200002' "$test_root/persistent-window.err"

foreign_session="56565656-5656-4656-8656-565656565656"
foreign_lease="57575757-5757-4757-8757-575757575757"
foreign_container="5656565656565656565656565656565656565656565656565656565656565656"
foreign_compact="${foreign_session//-/}"
if printf '%s\n' "$media_token" | env "${common_environment[@]}" \
    "FAKE_SESSION_ID=$foreign_session" \
    "FAKE_CONTAINER_ID=$foreign_container" \
    "FAKE_CONTAINER_NAME=robotswarm-$foreign_compact" \
    "$publisher" publish \
        --protocol-version 2 \
        --session-id "$foreign_session" \
        --worker-id "56565656-5656-4656-8656-565656565656" \
        --lease-id "$foreign_lease" \
        --expires-at "$(date --utc --date='+5 minutes' '+%Y-%m-%dT%H:%M:%SZ')" \
        --container-id "$foreign_container" \
        --container-name "robotswarm-$foreign_compact" \
        --source Scene \
        --source-id "scene-$foreign_compact" \
        --stream-path "session/$foreign_compact/scene-$foreign_compact" \
        --publish-base-url "$publish_base_url" \
        >"$test_root/foreign-worker.out" \
        2>"$test_root/foreign-worker.err"
then
    echo "publisher accepted a container owned by another worker" >&2
    exit 1
fi
test ! -s "$test_root/foreign-worker.out"
grep -Fq "session container worker label is invalid" \
    "$test_root/foreign-worker.err"

session_eof="58585858-5858-4858-8858-585858585858"
lease_eof="59595959-5959-4959-8959-595959595959"
container_eof="5858585858585858585858585858585858585858585858585858585858585858"
start_media_server eof accept 1
eof_server_pid="$last_media_pid"
run_publisher \
    "$session_eof" \
    "$lease_eof" \
    "$container_eof" \
    "$test_root/eof.out" \
    "$test_root/eof.err" \
    "$last_media_url"
eof_publisher_pid="$last_pid"
eof_input_fd="$last_input_fd"
wait_ready "$eof_publisher_pid" "$test_root/eof.out"
exec {eof_input_fd}>&-
if wait "$eof_publisher_pid"; then
    echo "publisher stayed ready after its interactive input pipe closed" >&2
    exit 1
fi
wait "$eof_server_pid"
test "$(cat "$test_root/eof.out")" = READY
grep -Fq "interactive viewer input stopped: viewer input pipe closed" \
    "$test_root/eof.err"

bad_session_compact="${session_one//-/}"
if env "${common_environment[@]}" "$publisher" publish \
    --protocol-version 2 \
    --session-id "$session_one" \
    --worker-id "$fake_worker_id" \
    --lease-id "$lease_one" \
    --expires-at "$(date --utc --date='+5 minutes' '+%Y-%m-%dT%H:%M:%SZ')" \
    --container-id "$container_one" \
    --container-name "robotswarm-$bad_session_compact" \
    --source Scene \
    --source-id "scene-$bad_session_compact" \
    --stream-path "session/not-the-session/scene-$bad_session_compact" \
    --publish-base-url "$publish_base_url" \
    >"$test_root/bad-path.out" 2>"$test_root/bad-path.err" \
    <<<"$media_token"
then
    echo "publisher accepted a non-canonical stream path" >&2
    exit 1
fi
test ! -s "$test_root/bad-path.out"
grep -Fq "viewer source path does not match" "$test_root/bad-path.err"
! grep -Fq "$media_token" "$test_root/bad-path.err"

if ROBOTSWARM_MEDIA_TOKEN="$media_token" \
    env "${common_environment[@]}" "$publisher" publish \
        --protocol-version 2 \
        --session-id "$session_one" \
        --worker-id "$fake_worker_id" \
        --lease-id "$lease_one" \
        --expires-at "$(date --utc --date='+5 minutes' '+%Y-%m-%dT%H:%M:%SZ')" \
        --container-id "$container_one" \
        --container-name "robotswarm-$bad_session_compact" \
        --source Scene \
        --source-id "scene-$bad_session_compact" \
        --stream-path "session/$bad_session_compact/scene-$bad_session_compact" \
        --publish-base-url "$publish_base_url" \
        >"$test_root/token-env.out" 2>"$test_root/token-env.err" \
        <<<"$media_token"
then
    echo "publisher accepted a token through its environment" >&2
    exit 1
fi
test ! -s "$test_root/token-env.out"
grep -Fq "must be provided on standard input" "$test_root/token-env.err"
! grep -Fq "$media_token" "$test_root/token-env.err"

env "${common_environment[@]}" \
    "PUBLISHER_PATH=$publisher" \
    "LEASE_TEST_ROOT=$test_root/lease-runtime-test" \
    python3 - <<'PY'
import datetime as dt
import importlib.machinery
import importlib.util
import os
import re
import sys
import uuid
from pathlib import Path

loader = importlib.machinery.SourceFileLoader(
    "robotswarm_viewer_publisher", os.environ["PUBLISHER_PATH"]
)
spec = importlib.util.spec_from_loader(loader.name, loader)
module = importlib.util.module_from_spec(spec)
sys.modules[loader.name] = module
loader.exec_module(module)

test_root = Path(os.environ["LEASE_TEST_ROOT"])
runtime = test_root / "runtime"
x_sockets = test_root / "x11"
x_sockets.mkdir(mode=0o700, parents=True)
settings = module.Settings(
    docker=os.environ["ROBOTSWARM_VIEWER_DOCKER"],
    gzclient=os.environ["ROBOTSWARM_VIEWER_GZCLIENT"],
    sandbox=os.environ["ROBOTSWARM_VIEWER_SANDBOX"],
    ldd=os.environ["ROBOTSWARM_VIEWER_LDD"],
    true_executable="/usr/bin/true",
    display_server=os.environ["ROBOTSWARM_VIEWER_DISPLAY_SERVER"],
    ffmpeg=os.environ["ROBOTSWARM_VIEWER_FFMPEG"],
    xwininfo=os.environ["ROBOTSWARM_VIEWER_XWININFO"],
    xprop=os.environ["ROBOTSWARM_VIEWER_XPROP"],
    gui_plugin=module.Path(os.environ["ROBOTSWARM_VIEWER_GUI_PLUGIN"]),
    ros_share_dir=(
        module.Path(os.environ["ROBOTSWARM_VIEWER_ASSET_ROOT"])
        / "ros-share"
    ),
    model_paths=((
        module.Path(os.environ["ROBOTSWARM_VIEWER_ASSET_ROOT"])
        / "models"
    ),),
    display_transport="unix",
    runtime_dir=runtime,
    x_socket_dir=x_sockets,
    display_min=240,
    display_max=241,
    width=1280,
    height=720,
    frame_rate=30,
    render_rate=50.0,
    minimum_render_rate=45.0,
    render_warmup_seconds=2.0,
    render_sample_seconds=5.0,
    bit_rate="4M",
    fill_display=True,
    gpu_adapter_name="NVIDIA",
    startup_timeout=4.0,
    settle_seconds=0.2,
    traffic_timeout=1.0,
    window_pattern=re.compile("Gazebo"),
)
request = module.PublishRequest(
    session_id=uuid.UUID("11111111-1111-4111-8111-111111111111"),
    worker_id=uuid.UUID("55555555-5555-4555-8555-555555555555"),
    lease_id=uuid.UUID("66666666-6666-4666-8666-666666666666"),
    expires_at=dt.datetime.now(dt.timezone.utc) + dt.timedelta(minutes=5),
    container_id="a" * 64,
    container_name="robotswarm-" + "1" * 32,
    source_id="scene-" + "1" * 32,
    stream_path="session/" + "1" * 32 + "/scene-" + "1" * 32,
    publish_base_url="rtsp://127.0.0.1:8554",
)
module.ensure_private_directory(runtime)
session_dir = runtime / f"lease-{request.lease_id.hex}"
session_dir.mkdir(mode=0o700)
(session_dir / "stale.marker").write_text("stale", encoding="ascii")

pipeline = module.ScenePipeline(settings, request, "A" * 43, "libx264")
assert not (session_dir / "stale.marker").exists()
try:
    module.ScenePipeline(settings, request, "A" * 43, "libx264")
except module.PublisherError as exc:
    assert "already has a local publisher" in str(exc)
else:
    raise AssertionError("a concurrent same-lease publisher acquired the lock")
pipeline.stop()

session_dir.mkdir(mode=0o755)
try:
    module.ScenePipeline(settings, request, "A" * 43, "libx264")
except module.PublisherError as exc:
    assert "not a private owned directory" in str(exc)
else:
    raise AssertionError("an ambiguous stale runtime was reclaimed")
PY

session_rejected="77777777-7777-4777-8777-777777777777"
lease_rejected="88888888-8888-4888-8888-888888888888"
container_rejected="eeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeee"
start_media_server rejected reject-record 1
rejected_server_pid="$last_media_pid"
run_publisher \
    "$session_rejected" \
    "$lease_rejected" \
    "$container_rejected" \
    "$test_root/rejected.out" \
    "$test_root/rejected.err" \
    "$last_media_url"
rejected_publisher_pid="$last_pid"
rejected_input_fd="$last_input_fd"
if wait "$rejected_publisher_pid"; then
    echo "publisher accepted an upstream RECORD rejection" >&2
    exit 1
fi
exec {rejected_input_fd}>&-
wait "$rejected_server_pid"
test ! -s "$test_root/rejected.out"
grep -Fq "upstream media server rejected RECORD (401)" \
    "$test_root/rejected.err"
! grep -Fq "$media_token" "$test_root/rejected.err"

session_silent="99999999-9999-4999-8999-999999999999"
lease_silent="aaaaaaaa-aaaa-4aaa-8aaa-aaaaaaaaaaaa"
container_silent="ffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffff"
start_media_server silent accept 1
silent_server_pid="$last_media_pid"
run_publisher \
    "$session_silent" \
    "$lease_silent" \
    "$container_silent" \
    "$test_root/silent.out" \
    "$test_root/silent.err" \
    "$last_media_url" \
    none
silent_publisher_pid="$last_pid"
silent_input_fd="$last_input_fd"
if wait "$silent_publisher_pid"; then
    echo "publisher accepted an RTSP connection without RTP video" >&2
    exit 1
fi
exec {silent_input_fd}>&-
wait "$silent_server_pid"
test ! -s "$test_root/silent.out"
grep -Fq "RTP video traffic stalled" "$test_root/silent.err"
! grep -Fq "$media_token" "$test_root/silent.err"
test "$(cat "$fake_state/silent.traffic")" = 0

session_stalled="bbbbbbbb-bbbb-4bbb-8bbb-bbbbbbbbbbbb"
lease_stalled="cccccccc-cccc-4ccc-8ccc-cccccccccccc"
container_stalled="abababababababababababababababababababababababababababababababab"
start_media_server stalled accept 1
stalled_server_pid="$last_media_pid"
run_publisher \
    "$session_stalled" \
    "$lease_stalled" \
    "$container_stalled" \
    "$test_root/stalled.out" \
    "$test_root/stalled.err" \
    "$last_media_url" \
    stall
stalled_publisher_pid="$last_pid"
stalled_input_fd="$last_input_fd"
wait_ready "$stalled_publisher_pid" "$test_root/stalled.out"
if wait "$stalled_publisher_pid"; then
    echo "publisher stayed ready after RTP video stopped" >&2
    exit 1
fi
exec {stalled_input_fd}>&-
wait "$stalled_server_pid"
test "$(cat "$test_root/stalled.out")" = READY
grep -Fq "RTP video traffic stalled" "$test_root/stalled.err"
! grep -Fq "$media_token" "$test_root/stalled.err"
test "$(cat "$fake_state/stalled.traffic")" -ge 3

echo "viewer publisher tests passed"
