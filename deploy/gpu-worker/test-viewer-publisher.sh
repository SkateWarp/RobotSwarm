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

cat > "$fake_bin/Xvfb" <<'PY'
#!/usr/bin/env python3
import json
import os
import signal
import socket
import sys
import time

if "ROBOTSWARM_MEDIA_TOKEN" in os.environ:
    raise SystemExit("display inherited the media token")
if any(name.lower().startswith("worker__") for name in os.environ):
    raise SystemExit("display inherited worker identity settings")

display = int(sys.argv[1].lstrip(":"))
arguments = sys.argv[2:]
tcp_transport = "-listen" in arguments and arguments[arguments.index("-listen") + 1] == "tcp"
socket_path = None
if tcp_transport:
    server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server.bind(("127.0.0.1", 6000 + display))
else:
    socket_path = os.path.join(
        os.environ["ROBOTSWARM_VIEWER_X_SOCKET_DIR"], f"X{display}"
    )
    server = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
    server.bind(socket_path)
server.listen(1)
with open(os.path.join(os.environ["FAKE_STATE"], "xvfb.log"), "a", encoding="utf-8") as stream:
    stream.write(json.dumps(sys.argv[1:]) + "\n")
stopping = False

def stop(_signum, _frame):
    global stopping
    stopping = True

signal.signal(signal.SIGINT, stop)
signal.signal(signal.SIGTERM, stop)
while not stopping:
    time.sleep(0.05)
server.close()
if socket_path is not None:
    try:
        os.unlink(socket_path)
    except FileNotFoundError:
        pass
PY

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
                    "Gateway": "",
                }
            }},
        }]))
        raise SystemExit(0)
    sidecar_file = os.path.join(state, "sidecar-" + target)
    if os.path.exists(sidecar_file):
        with open(sidecar_file, encoding="utf-8") as stream:
            metadata = json.load(stream)
        print(json.dumps([{
            "Config": {"Labels": {
                "io.robotswarm.viewer": "true",
                "io.robotswarm.viewer-lease": metadata["lease"],
                "io.robotswarm.session-id": metadata["session"],
            }}
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
            "Subnet": "172.31.0.0/16",
            "Gateway": os.environ["FAKE_NETWORK_GATEWAY"],
        }]},
    }]))
    raise SystemExit(0)

if arguments[:1] == ["run"]:
    name = arguments[arguments.index("--name") + 1]
    labels = [arguments[index + 1] for index, item in enumerate(arguments) if item == "--label"]
    label_map = dict(label.split("=", 1) for label in labels)
    sidecar_file = os.path.join(state, "sidecar-" + name)
    stop_file = os.path.join(state, "stop-" + name)
    with open(sidecar_file, "w", encoding="utf-8") as stream:
        json.dump({
            "lease": label_map["io.robotswarm.viewer-lease"],
            "session": label_map["io.robotswarm.session-id"],
        }, stream)
    stopping = False

    def stop(_signum, _frame):
        global stopping
        stopping = True

    signal.signal(signal.SIGINT, stop)
    signal.signal(signal.SIGTERM, stop)
    while not stopping and not os.path.exists(stop_file):
        time.sleep(0.05)
    for path in (sidecar_file, stop_file):
        try:
            os.unlink(path)
        except FileNotFoundError:
            pass
    raise SystemExit(0)

if arguments[:1] in (["stop"], ["rm"]):
    name = arguments[-1]
    open(os.path.join(state, "stop-" + name), "a", encoding="utf-8").close()
    if arguments[0] == "rm":
        try:
            os.unlink(os.path.join(state, "sidecar-" + name))
        except FileNotFoundError:
            pass
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

def stop(_signum, _frame):
    global stopping
    stopping = True

signal.signal(signal.SIGINT, stop)
signal.signal(signal.SIGTERM, stop)
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
else:
    print('0x200001 "Gazebo": ("gzclient" "Gazebo") 1280x720+0+0')
PY

cat > "$fake_bin/mediamtx-test-server" <<'PY'
#!/usr/bin/env python3
import json
import os
import socket
import threading

state = os.environ["FAKE_STATE"]
listener = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
listener.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
listener.bind(("127.0.0.1", 0))
listener.listen(4)
with open(os.path.join(state, "media.port"), "w", encoding="ascii") as stream:
    stream.write(str(listener.getsockname()[1]))

log_lock = threading.Lock()

def receive_request(connection, buffer):
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
    try:
        while True:
            header = receive_request(connection, buffer)
            if header is None:
                return
            lines = header.decode("ascii").split("\r\n")
            request_line = lines[0]
            cseq = next(
                line.split(":", 1)[1].strip()
                for line in lines[1:]
                if line.lower().startswith("cseq:")
            )
            with log_lock:
                with open(
                    os.path.join(state, "media.log"), "a", encoding="utf-8"
                ) as stream:
                    stream.write(json.dumps(request_line) + "\n")
            connection.sendall(
                (
                    "RTSP/1.0 200 OK\r\n"
                    f"CSeq: {cseq}\r\n"
                    "Session: viewer-test\r\n\r\n"
                ).encode("ascii")
            )
    finally:
        connection.close()

threads = []
for _ in range(2):
    connection, _ = listener.accept()
    thread = threading.Thread(target=handle, args=(connection,))
    thread.start()
    threads.append(thread)
listener.close()
for thread in threads:
    thread.join()
PY

chmod 0755 \
    "$fake_bin/Xvfb" \
    "$fake_bin/docker" \
    "$fake_bin/ffmpeg" \
    "$fake_bin/mediamtx-test-server" \
    "$fake_bin/xwininfo"
chmod 0755 "$publisher"

media_token="ABCDEFGHIJKLMNOPQRSTUVWXYZ_abcdefghijklmnop"
FAKE_STATE="$fake_state" "$fake_bin/mediamtx-test-server" &
media_server_pid=$!
for _ in $(seq 1 80); do
    if test -s "$fake_state/media.port"; then
        break
    fi
    sleep 0.05
done
test -s "$fake_state/media.port"
media_port="$(cat "$fake_state/media.port")"
publish_base_url="rtsp://127.0.0.1:$media_port"
common_environment=(
    "Worker__WorkerSecret=must-not-reach-viewer-children"
    "ROBOTSWARM_VIEWER_DOCKER=$fake_bin/docker"
    "ROBOTSWARM_VIEWER_DISPLAY_SERVER=$fake_bin/Xvfb"
    "ROBOTSWARM_VIEWER_FFMPEG=$fake_bin/ffmpeg"
    "ROBOTSWARM_VIEWER_XWININFO=$fake_bin/xwininfo"
    "ROBOTSWARM_VIEWER_DISPLAY_TRANSPORT=tcp"
    "ROBOTSWARM_VIEWER_RUNTIME_DIR=$runtime_dir"
    "ROBOTSWARM_VIEWER_X_SOCKET_DIR=$x_socket_dir"
    "ROBOTSWARM_VIEWER_DISPLAY_MIN=230"
    "ROBOTSWARM_VIEWER_DISPLAY_MAX=232"
    "ROBOTSWARM_VIEWER_STARTUP_TIMEOUT=4"
    "ROBOTSWARM_VIEWER_SETTLE_SECONDS=0.2"
    "ROBOTSWARM_VIEWER_GZCLIENT_GPU_REQUEST=none"
    "FAKE_WORKER_ID=55555555-5555-4555-8555-555555555555"
    "FAKE_NETWORK_ID=dddddddddddddddddddddddddddddddddddddddddddddddddddddddddddddddd"
    "FAKE_NETWORK_GATEWAY=172.31.0.1"
    "FAKE_STATE=$fake_state"
)

probe_output="$(env "${common_environment[@]}" "$publisher" probe \
    --protocol-version 1 \
    --publish-base-url "$publish_base_url")"
test "$probe_output" = \
    '{"protocolVersion":1,"ready":true,"videoCodec":"H264","sources":["Scene"]}'

software_probe_output="$(FAKE_ENCODERS=libx264 \
    ROBOTSWARM_VIEWER_ENCODER=libx264 \
    env "${common_environment[@]}" "$publisher" probe \
        --protocol-version 1 \
        --publish-base-url "$publish_base_url")"
test "$software_probe_output" = "$probe_output"

if FAKE_ENCODERS="mpeg4" env "${common_environment[@]}" \
    "$publisher" probe \
        --protocol-version 1 \
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
        --protocol-version 1 \
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
    local compact_session="${session_id//-/}"
    local source_id="scene-${compact_session}"
    local container_name="robotswarm-${compact_session}"
    local expires_at
    expires_at="$(date --utc --date='+5 minutes' '+%Y-%m-%dT%H:%M:%SZ')"

    printf '%s\n' "$media_token" | env "${common_environment[@]}" \
        "FAKE_SESSION_ID=$session_id" \
        "FAKE_CONTAINER_ID=$container_id" \
        "FAKE_CONTAINER_NAME=$container_name" \
        "$publisher" publish \
            --protocol-version 1 \
            --session-id "$session_id" \
            --lease-id "$lease_id" \
            --expires-at "$expires_at" \
            --container-id "$container_id" \
            --container-name "$container_name" \
            --source Scene \
            --source-id "$source_id" \
            --stream-path "session/${compact_session}/${source_id}" \
            --publish-base-url "$publish_base_url" \
            >"$output_file" 2>"$error_file" &
    last_pid=$!
}

wait_ready() {
    local process_id="$1"
    local output_file="$2"
    local attempt
    for attempt in $(seq 1 80); do
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
run_publisher "$session_one" "$lease_one" "$container_one" \
    "$test_root/one.out" "$test_root/one.err"
pid_one="$last_pid"
run_publisher "$session_two" "$lease_two" "$container_two" \
    "$test_root/two.out" "$test_root/two.err"
pid_two="$last_pid"

wait_ready "$pid_one" "$test_root/one.out"
wait_ready "$pid_two" "$test_root/two.out"
test ! -s "$test_root/one.err"
test ! -s "$test_root/two.err"

kill -TERM "$pid_one" "$pid_two"
wait "$pid_one"
wait "$pid_two"
wait "$media_server_pid"

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
assert inputs == {"127.0.0.1:230.0", "127.0.0.1:231.0"}, inputs
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

python3 - "$fake_state/docker.log" "$media_token" "$container_one" "$container_two" <<'PY'
import json
import sys

with open(sys.argv[1], encoding="utf-8") as stream:
    commands = [json.loads(line) for line in stream]
serialized = json.dumps(commands)
assert sys.argv[2] not in serialized
run_commands = [command for command in commands if command[:1] == ["run"]]
assert len(run_commands) == 2, run_commands
networks = {command[command.index("--network") + 1] for command in run_commands}
assert networks == {"container:" + sys.argv[3], "container:" + sys.argv[4]}
assert all("sha256:" + "b" * 64 in command for command in run_commands)
display_settings = {
    item
    for command in run_commands
    for item in command
    if item.startswith("DISPLAY=")
}
assert display_settings == {"DISPLAY=172.31.0.1:230", "DISPLAY=172.31.0.1:231"}
assert not any("/tmp/.X11-unix" in item for command in run_commands for item in command)
network_inspects = [
    command for command in commands if command[:2] == ["network", "inspect"]
]
assert len(network_inspects) == 2, network_inspects
assert all(command[2] == "d" * 64 for command in network_inspects)
PY

python3 - "$fake_state/xvfb.log" <<'PY'
import json
import sys

with open(sys.argv[1], encoding="utf-8") as stream:
    commands = [json.loads(line) for line in stream]
assert len(commands) == 2, commands
assert {command[0] for command in commands} == {":230", ":231"}
for command in commands:
    assert command[command.index("-nolisten") + 1] == "unix"
    assert command[command.index("-listen") + 1] == "tcp"
    assert "-auth" in command
PY

bad_session_compact="${session_one//-/}"
if env "${common_environment[@]}" "$publisher" publish \
    --protocol-version 1 \
    --session-id "$session_one" \
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
        --protocol-version 1 \
        --session-id "$session_one" \
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
    display_server=os.environ["ROBOTSWARM_VIEWER_DISPLAY_SERVER"],
    ffmpeg=os.environ["ROBOTSWARM_VIEWER_FFMPEG"],
    xwininfo=os.environ["ROBOTSWARM_VIEWER_XWININFO"],
    display_transport="unix",
    runtime_dir=runtime,
    x_socket_dir=x_sockets,
    display_min=240,
    display_max=241,
    width=1280,
    height=720,
    frame_rate=30,
    bit_rate="4M",
    gpu_request="none",
    startup_timeout=4.0,
    settle_seconds=0.2,
    window_pattern=re.compile("Gazebo"),
)
request = module.PublishRequest(
    session_id=uuid.UUID("11111111-1111-4111-8111-111111111111"),
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

echo "viewer publisher tests passed"
