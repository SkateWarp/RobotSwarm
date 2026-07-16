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
        rm -f "$FAKE_STATE/activating" "$FAKE_STATE/stopped"
        ;;
    stop)
        rm -f "$FAKE_STATE/active" "$FAKE_STATE/activating"
        touch "$FAKE_STATE/stopped"
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
                else
                    echo inactive
                fi
                ;;
            MainPID)
                echo 4242
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

test "$first_target" = \
    "$fake_home/.local/share/robotswarm-gpu-worker/releases/$first_release"
grep -Fq "Worker__SessionImage=\"$first_image\"" "$release_environment"
grep -Fq 'Worker__AllowMutableSessionImage="false"' "$release_environment"
grep -Fq "Worker__DockerExecutable=\"$fake_bin/docker\"" "$release_environment"
test "$(stat -c '%a' "$release_environment")" = "600"

second_revision="2222222222222222222222222222222222222222"
second_image="sha256:bbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbb"
changed_unit="$test_root/changed-worker.service"
cp "$project_root/deploy/gpu-worker/robotswarm-gpu-worker.service" "$changed_unit"
echo "# simulated unit change" >> "$changed_unit"

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

echo "GPU worker deployment and rollback tests passed."
