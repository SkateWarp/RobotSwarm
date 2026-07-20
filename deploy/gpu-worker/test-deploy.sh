#!/usr/bin/env bash

set -euo pipefail

project_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
test_root="$(mktemp -d)"
fake_home="${test_root}/home"
fake_bin="${test_root}/bin"
fake_state="${test_root}/state"
publish_dir="${test_root}/publish"

cleanup() {
    rm -rf "$test_root"
}
trap cleanup EXIT

mkdir -p \
    "$fake_home/.config/robotswarm" \
    "$fake_bin" \
    "$fake_state" \
    "$publish_dir"

cat > "$fake_bin/docker" <<'EOF'
#!/usr/bin/env bash
set -euo pipefail

if [[ "${1:-}" == "info" || "${1:-}" == "version" ]]; then
    exit 0
fi

if [[ "${1:-}" == "ps" ]]; then
    if [[ "${FAKE_RUNNING_SESSION:-0}" == "1" \
        || ( "${FAKE_SESSION_AFTER_STOP:-0}" == "1" \
            && -f "$FAKE_STATE/stopped" ) ]]
    then
        echo "managed-session-container"
    fi
    exit 0
fi

if [[ "${1:-}" == "image" && "${2:-}" == "inspect" ]]; then
    format="${4:-}"
    if [[ "$format" == "{{.Id}}" ]]; then
        echo "$FAKE_IMAGE_ID"
    else
        echo "$FAKE_REVISION"
    fi
    exit 0
fi

exit 1
EOF

cat > "$fake_bin/systemctl" <<'EOF'
#!/usr/bin/env bash
set -euo pipefail

if [[ "${1:-}" == "--user" ]]; then
    shift
fi

case "${1:-}" in
    show-environment|daemon-reload)
        exit 0
        ;;
    is-active)
        test -f "$FAKE_STATE/active"
        ;;
    is-enabled)
        test -f "$FAKE_STATE/enabled"
        ;;
    enable)
        touch "$FAKE_STATE/enabled"
        ;;
    disable)
        rm -f "$FAKE_STATE/enabled"
        ;;
    start|restart)
        if [[ "${FAKE_RESTART_ALWAYS_FAIL:-0}" == "1" ]]; then
            exit 1
        fi
        if [[ "${FAKE_RESTART_FAIL_ONCE:-0}" == "1" \
            && ! -f "$FAKE_STATE/restart-failed" ]]
        then
            touch "$FAKE_STATE/restart-failed"
            exit 1
        fi
        touch "$FAKE_STATE/active"
        rm -f \
            "$FAKE_STATE/activating" \
            "$FAKE_STATE/failed" \
            "$FAKE_STATE/stopped"
        ;;
    stop)
        rm -f "$FAKE_STATE/active" "$FAKE_STATE/activating"
        touch "$FAKE_STATE/stopped"
        if [[ "${FAKE_FAILED_AFTER_STOP:-0}" == "1" ]]; then
            touch "$FAKE_STATE/failed"
        else
            rm -f "$FAKE_STATE/failed"
        fi
        if [[ "${FAKE_STOP_FAIL_ONCE:-0}" == "1" \
            && ! -f "$FAKE_STATE/stop-failed" ]]
        then
            touch "$FAKE_STATE/stop-failed"
            exit 1
        fi
        ;;
    show)
        property=""
        while (( $# > 0 )); do
            if [[ "$1" == "--property" ]]; then
                property="${2:-}"
                shift 2
            else
                shift
            fi
        done

        case "$property" in
            ActiveState)
                if [[ -f "$FAKE_STATE/active" ]]; then
                    echo active
                elif [[ -f "$FAKE_STATE/activating" ]]; then
                    echo activating
                elif [[ -f "$FAKE_STATE/failed" ]]; then
                    echo failed
                else
                    echo inactive
                fi
                ;;
            MainPID)
                if [[ -f "$FAKE_STATE/failed" ]]; then
                    echo "${FAKE_FAILED_MAIN_PID:-0}"
                else
                    echo 4242
                fi
                ;;
            *)
                exit 1
                ;;
        esac
        ;;
    status)
        exit 0
        ;;
    *)
        exit 1
        ;;
esac
EOF

cat > "$fake_bin/journalctl" <<'EOF'
#!/usr/bin/env bash
set -euo pipefail

if [[ "${FAKE_JOURNAL_FAIL:-0}" == "1" ]]; then
    exit 1
fi

for argument in "$@"; do
    if [[ "$argument" == "--show-cursor" ]]; then
        echo "-- cursor: fake-cursor"
        exit 0
    fi
done

release_environment="$HOME/.local/share/robotswarm-gpu-worker/current/gpu-worker-release.env"
image_version="$(
    sed -n 's/^Worker__ImageVersion="\([^"]*\)"$/\1/p' "$release_environment"
)"
echo "Worker ready for control-plane commands. ImageVersion=${image_version}"
EOF

chmod 0755 "$fake_bin/docker" "$fake_bin/systemctl" "$fake_bin/journalctl"

cat > "$fake_home/.config/robotswarm/gpu-worker-identity.env" <<'EOF'
Worker__BackendUrl="https://robot.zerav.la"
Worker__WorkerId="11111111-1111-1111-1111-111111111111"
Worker__WorkerSecret="abcdefghijklmnopqrstuvwxyzABCDEF_123456"
Worker__Name="test-gpu-worker"
Worker__MaxConcurrentSessions="2"
Worker__MaxRobotsPerSession="10"
EOF
chmod 0600 "$fake_home/.config/robotswarm/gpu-worker-identity.env"

cat > "$publish_dir/SwarmWorker" <<'EOF'
#!/usr/bin/env bash
exit 0
EOF
chmod 0755 "$publish_dir/SwarmWorker"

first_revision="1111111111111111111111111111111111111111"
first_image="sha256:aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa"
first_release="${first_revision}-1-1"
missing_helper_log="$test_root/missing-viewer-helper.log"

if "$project_root/deploy/gpu-worker/deploy.sh" \
    --publish-dir "$publish_dir" \
    --image-id "$first_image" \
    --revision "$first_revision" \
    --release-id "$first_release" \
    --gpu-request device=0 \
    --unit-file "$project_root/deploy/gpu-worker/robotswarm-gpu-worker.service" \
    >"$missing_helper_log" 2>&1
then
    echo "Expected a missing viewer helper to fail deployment." >&2
    exit 1
fi
grep -Fq "published viewer helper is missing" "$missing_helper_log"

cat > "$publish_dir/robotswarm-viewer-publisher" <<'EOF'
#!/usr/bin/env python3
print("first release viewer helper")
EOF
# deploy.sh owns the release mode rather than trusting the build artifact.
chmod 0644 "$publish_dir/robotswarm-viewer-publisher"

env -u XDG_RUNTIME_DIR -u DBUS_SESSION_BUS_ADDRESS \
    HOME="$fake_home" \
    PATH="$fake_bin:$PATH" \
    FAKE_STATE="$fake_state" \
    FAKE_IMAGE_ID="$first_image" \
    FAKE_REVISION="$first_revision" \
    "$project_root/deploy/gpu-worker/deploy.sh" \
        --publish-dir "$publish_dir" \
        --image-id "$first_image" \
        --revision "$first_revision" \
        --release-id "$first_release" \
        --gpu-request device=0 \
        --unit-file "$project_root/deploy/gpu-worker/robotswarm-gpu-worker.service"

current_link="$fake_home/.local/share/robotswarm-gpu-worker/current"
release_environment="$current_link/gpu-worker-release.env"
first_target="$(readlink "$current_link")"
first_viewer_publisher="$first_target/robotswarm-viewer-publisher"

test "$first_target" = \
    "$fake_home/.local/share/robotswarm-gpu-worker/releases/$first_release"
grep -Fq "Worker__SessionImage=\"$first_image\"" "$release_environment"
grep -Fq 'Worker__AllowMutableSessionImage="false"' "$release_environment"
! grep -Fq 'Worker__Viewer__Enabled=' "$release_environment"
grep -Fq \
    "Worker__Viewer__PublisherExecutable=\"$first_viewer_publisher\"" \
    "$release_environment"
grep -Fq "Worker__DockerExecutable=\"$fake_bin/docker\"" "$release_environment"
test "$(stat -c '%a' "$release_environment")" = "600"
test -f "$first_viewer_publisher"
test ! -L "$first_viewer_publisher"
test "$(stat -c '%a' "$first_viewer_publisher")" = "755"
grep -Fq "first release viewer helper" "$first_viewer_publisher"
cmp -s \
    "$fake_home/.config/systemd/user/robotswarm-gpu-worker.service" \
    "$project_root/deploy/gpu-worker/robotswarm-gpu-worker.service"

failed_stop_home="${test_root}/failed-stop-home"
failed_stop_state="${test_root}/failed-stop-state"
cp -a "$fake_home" "$failed_stop_home"
cp -a "$fake_state" "$failed_stop_state"
failed_stop_revision="abababababababababababababababababababab"
failed_stop_image="sha256:abababababababababababababababababababababababababababababababab"
failed_stop_release="${failed_stop_revision}-shutdown-regression"

HOME="$failed_stop_home" \
PATH="$fake_bin:$PATH" \
FAKE_STATE="$failed_stop_state" \
FAKE_IMAGE_ID="$failed_stop_image" \
FAKE_REVISION="$failed_stop_revision" \
FAKE_FAILED_AFTER_STOP=1 \
    "$project_root/deploy/gpu-worker/deploy.sh" \
        --publish-dir "$publish_dir" \
        --image-id "$failed_stop_image" \
        --revision "$failed_stop_revision" \
        --release-id "$failed_stop_release" \
        --gpu-request device=0 \
        --unit-file "$project_root/deploy/gpu-worker/robotswarm-gpu-worker.service"

test "$(readlink "$failed_stop_home/.local/share/robotswarm-gpu-worker/current")" = \
    "$failed_stop_home/.local/share/robotswarm-gpu-worker/releases/$failed_stop_release"
test -f "$failed_stop_state/active"
test ! -f "$failed_stop_state/failed"

failed_pid_revision="acacacacacacacacacacacacacacacacacacacac"
failed_pid_image="sha256:acacacacacacacacacacacacacacacacacacacacacacacacacacacacacacacac"
failed_pid_log="$test_root/failed-stop-live-pid.log"
if HOME="$failed_stop_home" \
    PATH="$fake_bin:$PATH" \
    FAKE_STATE="$failed_stop_state" \
    FAKE_IMAGE_ID="$failed_pid_image" \
    FAKE_REVISION="$failed_pid_revision" \
    FAKE_FAILED_AFTER_STOP=1 \
    FAKE_FAILED_MAIN_PID=8181 \
        "$project_root/deploy/gpu-worker/deploy.sh" \
            --publish-dir "$publish_dir" \
            --image-id "$failed_pid_image" \
            --revision "$failed_pid_revision" \
            --release-id "${failed_pid_revision}-unsafe-shutdown" \
            --gpu-request device=0 \
            --unit-file "$project_root/deploy/gpu-worker/robotswarm-gpu-worker.service" \
            >"$failed_pid_log" 2>&1
then
    echo "Expected a failed unit with a live PID to block deployment." >&2
    exit 1
fi

grep -Fq "failed worker service still has a running process" "$failed_pid_log"
test "$(readlink "$failed_stop_home/.local/share/robotswarm-gpu-worker/current")" = \
    "$failed_stop_home/.local/share/robotswarm-gpu-worker/releases/$failed_stop_release"
test -f "$failed_stop_state/active"

second_revision="2222222222222222222222222222222222222222"
second_image="sha256:bbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbb"
changed_unit="$test_root/changed-worker.service"
cp "$project_root/deploy/gpu-worker/robotswarm-gpu-worker.service" "$changed_unit"
echo "# simulated unit change" >> "$changed_unit"
sed -i 's/first release/second release/' \
    "$publish_dir/robotswarm-viewer-publisher"

if HOME="$fake_home" \
    PATH="$fake_bin:$PATH" \
    FAKE_STATE="$fake_state" \
    FAKE_IMAGE_ID="$second_image" \
    FAKE_REVISION="$second_revision" \
    FAKE_RESTART_FAIL_ONCE=1 \
        "$project_root/deploy/gpu-worker/deploy.sh" \
            --publish-dir "$publish_dir" \
            --image-id "$second_image" \
            --revision "$second_revision" \
            --release-id "${second_revision}-2-1" \
            --gpu-request device=0 \
            --unit-file "$changed_unit"
then
    echo "Expected the simulated restart failure to fail deployment." >&2
    exit 1
fi

test "$(readlink "$current_link")" = "$first_target"
grep -Fq "Worker__SessionImage=\"$first_image\"" "$release_environment"
grep -Fq \
    "Worker__Viewer__PublisherExecutable=\"$first_viewer_publisher\"" \
    "$release_environment"
grep -Fq "first release viewer helper" "$first_viewer_publisher"
cmp -s \
    "$fake_home/.config/systemd/user/robotswarm-gpu-worker.service" \
    "$project_root/deploy/gpu-worker/robotswarm-gpu-worker.service"
test -f "$fake_state/active"
test -f "$fake_state/enabled"

third_revision="3333333333333333333333333333333333333333"
third_image="sha256:cccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccc"

if HOME="$fake_home" \
    PATH="$fake_bin:$PATH" \
    FAKE_STATE="$fake_state" \
    FAKE_IMAGE_ID="$third_image" \
    FAKE_REVISION="$third_revision" \
    FAKE_JOURNAL_FAIL=1 \
        "$project_root/deploy/gpu-worker/deploy.sh" \
            --publish-dir "$publish_dir" \
            --image-id "$third_image" \
            --revision "$third_revision" \
            --release-id "${third_revision}-3-1" \
            --gpu-request device=0 \
            --unit-file "$changed_unit"
then
    echo "Expected the simulated journal failure to fail deployment." >&2
    exit 1
fi

test "$(readlink "$current_link")" = "$first_target"
test -f "$fake_state/active"
test ! -e \
    "$fake_home/.local/share/robotswarm-gpu-worker/releases/${third_revision}-3-1"

fourth_revision="4444444444444444444444444444444444444444"
fourth_image="sha256:dddddddddddddddddddddddddddddddddddddddddddddddddddddddddddddddd"

if HOME="$fake_home" \
    PATH="$fake_bin:$PATH" \
    FAKE_STATE="$fake_state" \
    FAKE_IMAGE_ID="$fourth_image" \
    FAKE_REVISION="$fourth_revision" \
    FAKE_RUNNING_SESSION=1 \
        "$project_root/deploy/gpu-worker/deploy.sh" \
            --publish-dir "$publish_dir" \
            --image-id "$fourth_image" \
            --revision "$fourth_revision" \
            --release-id "${fourth_revision}-4-1" \
            --gpu-request device=0 \
            --unit-file "$changed_unit"
then
    echo "Expected a running managed session to block deployment." >&2
    exit 1
fi

test "$(readlink "$current_link")" = "$first_target"
test -f "$fake_state/active"
test ! -e \
    "$fake_home/.local/share/robotswarm-gpu-worker/releases/${fourth_revision}-4-1"

fifth_revision="5555555555555555555555555555555555555555"
fifth_image="sha256:eeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeee"
rm -f "$fake_state/active"
touch "$fake_state/activating"

if HOME="$fake_home" \
    PATH="$fake_bin:$PATH" \
    FAKE_STATE="$fake_state" \
    FAKE_IMAGE_ID="$fifth_image" \
    FAKE_REVISION="$fifth_revision" \
    FAKE_SESSION_AFTER_STOP=1 \
        "$project_root/deploy/gpu-worker/deploy.sh" \
            --publish-dir "$publish_dir" \
            --image-id "$fifth_image" \
            --revision "$fifth_revision" \
            --release-id "${fifth_revision}-5-1" \
            --gpu-request device=0 \
            --unit-file "$changed_unit"
then
    echo "Expected a session appearing during drain to fail deployment." >&2
    exit 1
fi

test "$(readlink "$current_link")" = "$first_target"
test -f "$fake_state/active"
test ! -e \
    "$fake_home/.local/share/robotswarm-gpu-worker/releases/${fifth_revision}-5-1"

sixth_revision="6666666666666666666666666666666666666666"
sixth_image="sha256:ffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffff"
if HOME="$fake_home" \
    PATH="$fake_bin:$PATH" \
    FAKE_STATE="$fake_state" \
    FAKE_IMAGE_ID="$sixth_image" \
    FAKE_REVISION="$sixth_revision" \
    FAKE_STOP_FAIL_ONCE=1 \
        "$project_root/deploy/gpu-worker/deploy.sh" \
            --publish-dir "$publish_dir" \
            --image-id "$sixth_image" \
            --revision "$sixth_revision" \
            --release-id "${sixth_revision}-6-1" \
            --gpu-request device=0 \
            --unit-file "$changed_unit"
then
    echo "Expected the simulated stop failure to fail deployment." >&2
    exit 1
fi

test "$(readlink "$current_link")" = "$first_target"
test -f "$fake_state/active"
test -f "$fake_state/enabled"
test ! -e \
    "$fake_home/.local/share/robotswarm-gpu-worker/releases/${sixth_revision}-6-1"

seventh_revision="7777777777777777777777777777777777777777"
seventh_image="sha256:1212121212121212121212121212121212121212121212121212121212121212"
rollback_log="$test_root/rollback-failure.log"

if HOME="$fake_home" \
    PATH="$fake_bin:$PATH" \
    FAKE_STATE="$fake_state" \
    FAKE_IMAGE_ID="$seventh_image" \
    FAKE_REVISION="$seventh_revision" \
    FAKE_RESTART_ALWAYS_FAIL=1 \
        "$project_root/deploy/gpu-worker/deploy.sh" \
            --publish-dir "$publish_dir" \
            --image-id "$seventh_image" \
            --revision "$seventh_revision" \
            --release-id "${seventh_revision}-7-1" \
            --gpu-request device=0 \
            --unit-file "$changed_unit" \
            >"$rollback_log" 2>&1
then
    echo "Expected persistent restart failure to fail deployment." >&2
    exit 1
fi

test "$(readlink "$current_link")" = "$first_target"
test ! -f "$fake_state/active"
grep -Fq "CRITICAL: worker rollback is incomplete" "$rollback_log"

viewer_home="${test_root}/viewer-home"
viewer_state="${test_root}/viewer-state"
viewer_publish_dir="${test_root}/viewer-publish"
mkdir -p \
    "$viewer_home/.config/robotswarm" \
    "$viewer_state" \
    "$viewer_publish_dir"

cat > "$viewer_home/.config/robotswarm/gpu-worker-identity.env" <<'EOF'
Worker__BackendUrl="https://robot.zerav.la"
Worker__WorkerId="88888888-8888-8888-8888-888888888888"
Worker__WorkerSecret="abcdefghijklmnopqrstuvwxyzABCDEF_123456"
Worker__Name="viewer-gpu-worker"
Worker__MaxConcurrentSessions="2"
Worker__MaxRobotsPerSession="10"
Worker__Viewer__Enabled="true"
ROBOTSWARM_VIEWER_DISPLAY_TRANSPORT="unix"
ROBOTSWARM_VIEWER_RENDER_RATE="50"
ROBOTSWARM_VIEWER_MIN_RENDER_RATE="45"
ROBOTSWARM_VIEWER_GPU_ADAPTER_NAME="NVIDIA"
EOF
chmod 0600 "$viewer_home/.config/robotswarm/gpu-worker-identity.env"

cp "$publish_dir/SwarmWorker" "$viewer_publish_dir/SwarmWorker"
cat > "$viewer_publish_dir/robotswarm-viewer-publisher" <<'EOF'
#!/usr/bin/env bash
set -euo pipefail

if [[ "${1:-}" != "probe" ]]; then
    exit 1
fi
printf '%s\n' "$@" > "$FAKE_STATE/viewer-probe-arguments"
printf '%s\n' "$ROBOTSWARM_VIEWER_ENCODER" > "$FAKE_STATE/viewer-probe-encoder"
printf '%s\n' "$ROBOTSWARM_VIEWER_DISPLAY_TRANSPORT" \
    > "$FAKE_STATE/viewer-probe-display-transport"
printf '%s\n' "$ROBOTSWARM_VIEWER_RENDER_RATE" \
    > "$FAKE_STATE/viewer-probe-render-rate"
printf '%s\n' "$ROBOTSWARM_VIEWER_MIN_RENDER_RATE" \
    > "$FAKE_STATE/viewer-probe-min-render-rate"
printf '%s\n' "$ROBOTSWARM_VIEWER_GPU_ADAPTER_NAME" \
    > "$FAKE_STATE/viewer-probe-adapter"
touch "$FAKE_STATE/viewer-probed"
if [[ "${FAKE_VIEWER_PROBE_FAIL:-0}" == "1" ]]; then
    exit 1
fi
printf '%s\n' '{"protocolVersion":2,"ready":true,"videoCodec":"H264","sources":["Scene"],"interactive":true}'
EOF
chmod 0755 \
    "$viewer_publish_dir/SwarmWorker" \
    "$viewer_publish_dir/robotswarm-viewer-publisher"
mkdir -p \
    "$viewer_publish_dir/robotswarm-viewer-assets/ros-share/turtlebot3_description/meshes" \
    "$viewer_publish_dir/robotswarm-viewer-assets/models"
printf 'fake plugin\n' > \
    "$viewer_publish_dir/librobotswarm_gazebo_gui_probe.so"

viewer_revision="8888888888888888888888888888888888888888"
viewer_image="sha256:3434343434343434343434343434343434343434343434343434343434343434"
viewer_release="${viewer_revision}-8-1"
viewer_config_log="$test_root/viewer-config.log"

if HOME="$viewer_home" \
    PATH="$fake_bin:$PATH" \
    FAKE_STATE="$viewer_state" \
    FAKE_IMAGE_ID="$viewer_image" \
    FAKE_REVISION="$viewer_revision" \
        "$project_root/deploy/gpu-worker/deploy.sh" \
            --publish-dir "$viewer_publish_dir" \
            --image-id "$viewer_image" \
            --revision "$viewer_revision" \
            --release-id "$viewer_release" \
            --gpu-request device=1 \
            --unit-file "$project_root/deploy/gpu-worker/robotswarm-gpu-worker.service" \
            >"$viewer_config_log" 2>&1
then
    echo "Expected an enabled viewer without an RTSP endpoint to fail deployment." >&2
    exit 1
fi
grep -Fq \
    "Worker__Viewer__PublishBaseUrl is required" \
    "$viewer_config_log"
test ! -e "$viewer_home/.local/share/robotswarm-gpu-worker/releases/$viewer_release"

cat >> "$viewer_home/.config/robotswarm/gpu-worker-identity.env" <<'EOF'
Worker__Viewer__PublishBaseUrl="rtsp://127.0.0.1:8554/robotswarm"
EOF

sed -i \
    's/ROBOTSWARM_VIEWER_DISPLAY_TRANSPORT="unix"/ROBOTSWARM_VIEWER_DISPLAY_TRANSPORT="udp"/' \
    "$viewer_home/.config/robotswarm/gpu-worker-identity.env"
if HOME="$viewer_home" \
    PATH="$fake_bin:$PATH" \
    FAKE_STATE="$viewer_state" \
    FAKE_IMAGE_ID="$viewer_image" \
    FAKE_REVISION="$viewer_revision" \
        "$project_root/deploy/gpu-worker/deploy.sh" \
            --publish-dir "$viewer_publish_dir" \
            --image-id "$viewer_image" \
            --revision "$viewer_revision" \
            --release-id "$viewer_release" \
            --gpu-request device=1 \
            --unit-file "$project_root/deploy/gpu-worker/robotswarm-gpu-worker.service" \
            >"$viewer_config_log" 2>&1
then
    echo "Expected an unsupported viewer transport to fail deployment." >&2
    exit 1
fi
grep -Fq \
    "ROBOTSWARM_VIEWER_DISPLAY_TRANSPORT must be unix" \
    "$viewer_config_log"
sed -i \
    's/ROBOTSWARM_VIEWER_DISPLAY_TRANSPORT="udp"/ROBOTSWARM_VIEWER_DISPLAY_TRANSPORT="unix"/' \
    "$viewer_home/.config/robotswarm/gpu-worker-identity.env"

sed -i \
    's/ROBOTSWARM_VIEWER_MIN_RENDER_RATE="45"/ROBOTSWARM_VIEWER_MIN_RENDER_RATE="55"/' \
    "$viewer_home/.config/robotswarm/gpu-worker-identity.env"
if HOME="$viewer_home" \
    PATH="$fake_bin:$PATH" \
    FAKE_STATE="$viewer_state" \
    FAKE_IMAGE_ID="$viewer_image" \
    FAKE_REVISION="$viewer_revision" \
        "$project_root/deploy/gpu-worker/deploy.sh" \
            --publish-dir "$viewer_publish_dir" \
            --image-id "$viewer_image" \
            --revision "$viewer_revision" \
            --release-id "$viewer_release" \
            --gpu-request device=1 \
            --unit-file "$project_root/deploy/gpu-worker/robotswarm-gpu-worker.service" \
            >"$viewer_config_log" 2>&1
then
    echo "Expected a minimum render rate above the cap to fail deployment." >&2
    exit 1
fi
grep -Fq \
    "ROBOTSWARM_VIEWER_MIN_RENDER_RATE must not exceed" \
    "$viewer_config_log"
sed -i \
    's/ROBOTSWARM_VIEWER_MIN_RENDER_RATE="55"/ROBOTSWARM_VIEWER_MIN_RENDER_RATE="45"/' \
    "$viewer_home/.config/robotswarm/gpu-worker-identity.env"

HOME="$viewer_home" \
PATH="$fake_bin:$PATH" \
FAKE_STATE="$viewer_state" \
FAKE_IMAGE_ID="$viewer_image" \
FAKE_REVISION="$viewer_revision" \
    "$project_root/deploy/gpu-worker/deploy.sh" \
        --publish-dir "$viewer_publish_dir" \
        --image-id "$viewer_image" \
        --revision "$viewer_revision" \
        --release-id "$viewer_release" \
        --gpu-request device=1 \
        --unit-file "$project_root/deploy/gpu-worker/robotswarm-gpu-worker.service"

viewer_current="$viewer_home/.local/share/robotswarm-gpu-worker/current"
viewer_environment="$viewer_current/gpu-worker-release.env"
viewer_unit="$viewer_home/.config/systemd/user/robotswarm-gpu-worker.service"
test -f "$viewer_state/viewer-probed"
grep -Fxq 'auto' "$viewer_state/viewer-probe-encoder"
grep -Fxq 'unix' "$viewer_state/viewer-probe-display-transport"
grep -Fxq '50' "$viewer_state/viewer-probe-render-rate"
grep -Fxq '45' "$viewer_state/viewer-probe-min-render-rate"
grep -Fxq 'NVIDIA' "$viewer_state/viewer-probe-adapter"
grep -Fxq -- '--protocol-version' "$viewer_state/viewer-probe-arguments"
grep -Fxq -- 'rtsp://127.0.0.1:8554/robotswarm' \
    "$viewer_state/viewer-probe-arguments"
grep -Fxq 'PrivateTmp=true' "$viewer_unit"
! grep -Fxq 'PrivateTmp=false' "$viewer_unit"
grep -Fxq 'KillMode=control-group' "$viewer_unit"
grep -Fxq 'RestrictAddressFamilies=AF_UNIX AF_INET AF_INET6 AF_NETLINK' \
    "$viewer_unit"
grep -Fxq 'ExecStartPre=/usr/bin/python3 -c "import socket; socket.if_nameindex()"' \
    "$viewer_unit"
! grep -Fq 'Worker__Viewer__Enabled=' "$viewer_environment"
grep -Fxq 'ROBOTSWARM_VIEWER_ENCODER="auto"' "$viewer_environment"
grep -Fxq 'ROBOTSWARM_VIEWER_DISPLAY_TRANSPORT="unix"' \
    "$viewer_environment"
grep -Fxq 'ROBOTSWARM_VIEWER_RENDER_RATE="50"' \
    "$viewer_environment"
grep -Fxq 'ROBOTSWARM_VIEWER_MIN_RENDER_RATE="45"' \
    "$viewer_environment"
grep -Fxq 'ROBOTSWARM_VIEWER_GPU_ADAPTER_NAME="NVIDIA"' \
    "$viewer_environment"

sed -i \
    's/ROBOTSWARM_VIEWER_DISPLAY_TRANSPORT="unix"/ROBOTSWARM_VIEWER_DISPLAY_TRANSPORT="tcp"/' \
    "$viewer_home/.config/robotswarm/gpu-worker-identity.env"
tcp_revision="8989898989898989898989898989898989898989"
tcp_image="sha256:4545454545454545454545454545454545454545454545454545454545454545"
tcp_release="${tcp_revision}-8-2"
viewer_target_before_tcp="$(readlink "$viewer_current")"
if HOME="$viewer_home" \
    PATH="$fake_bin:$PATH" \
    FAKE_STATE="$viewer_state" \
    FAKE_IMAGE_ID="$tcp_image" \
    FAKE_REVISION="$tcp_revision" \
        "$project_root/deploy/gpu-worker/deploy.sh" \
            --publish-dir "$viewer_publish_dir" \
            --image-id "$tcp_image" \
            --revision "$tcp_revision" \
            --release-id "$tcp_release" \
            --gpu-request device=1 \
            --unit-file "$project_root/deploy/gpu-worker/robotswarm-gpu-worker.service" \
            >"$viewer_config_log" 2>&1
then
    echo "Expected network-exposed X11 TCP transport to fail deployment." >&2
    exit 1
fi
grep -Fq "ROBOTSWARM_VIEWER_DISPLAY_TRANSPORT must be unix" \
    "$viewer_config_log"
test "$(readlink "$viewer_current")" = "$viewer_target_before_tcp"
sed -i \
    's/ROBOTSWARM_VIEWER_DISPLAY_TRANSPORT="tcp"/ROBOTSWARM_VIEWER_DISPLAY_TRANSPORT="unix"/' \
    "$viewer_home/.config/robotswarm/gpu-worker-identity.env"

failed_probe_revision="9999999999999999999999999999999999999999"
failed_probe_image="sha256:5656565656565656565656565656565656565656565656565656565656565656"
failed_probe_release="${failed_probe_revision}-9-1"
viewer_target="$(readlink "$viewer_current")"
if HOME="$viewer_home" \
    PATH="$fake_bin:$PATH" \
    FAKE_STATE="$viewer_state" \
    FAKE_IMAGE_ID="$failed_probe_image" \
    FAKE_REVISION="$failed_probe_revision" \
    FAKE_VIEWER_PROBE_FAIL=1 \
        "$project_root/deploy/gpu-worker/deploy.sh" \
            --publish-dir "$viewer_publish_dir" \
            --image-id "$failed_probe_image" \
            --revision "$failed_probe_revision" \
            --release-id "$failed_probe_release" \
            --gpu-request device=1 \
            --unit-file "$project_root/deploy/gpu-worker/robotswarm-gpu-worker.service" \
            >/dev/null 2>&1
then
    echo "Expected a failed viewer host probe to block deployment." >&2
    exit 1
fi
test "$(readlink "$viewer_current")" = "$viewer_target"
test ! -e \
    "$viewer_home/.local/share/robotswarm-gpu-worker/releases/$failed_probe_release"

echo "GPU worker deployment and rollback tests passed."
