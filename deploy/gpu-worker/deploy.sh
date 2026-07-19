#!/usr/bin/env bash

set -Eeuo pipefail
IFS=$'\n\t'
umask 077

service_name="robotswarm-gpu-worker.service"
viewer_publisher_name="robotswarm-viewer-publisher"
publish_dir=""
image_id=""
revision=""
release_id=""
gpu_request="device=0"
unit_file=""
verify_timeout=75

usage() {
    cat <<'EOF'
Usage:
  deploy.sh \
    --publish-dir PATH \
    --image-id sha256:IMAGE_ID \
    --revision GIT_SHA \
    --release-id RELEASE_ID \
    --gpu-request REQUEST \
    --unit-file PATH

The worker identity must already exist at:
  ~/.config/robotswarm/gpu-worker-identity.env
EOF
}

fail() {
    echo "gpu-worker deploy: $*" >&2
    exit 1
}

require_command() {
    command -v "$1" >/dev/null 2>&1 || fail "required command '$1' was not found"
}

read_identity_value() {
    local setting="$1"
    local raw_value
    local value
    local -a matches=()

    mapfile -t matches < <(grep -E "^${setting}=" "$identity_file" || true)
    if (( ${#matches[@]} == 0 )); then
        return 1
    fi
    if (( ${#matches[@]} != 1 )); then
        fail "the worker identity file defines '$setting' more than once"
    fi

    raw_value="${matches[0]#*=}"
    if [[ "$raw_value" == \"* ]]; then
        if (( ${#raw_value} < 2 )) \
            || [[ "${raw_value: -1}" != '"' ]]
        then
            fail "the worker identity setting '$setting' has invalid quoting"
        fi
        value="${raw_value:1:${#raw_value}-2}"
        [[ "$value" != *'"'* ]] \
            || fail "the worker identity setting '$setting' has invalid quoting"
    else
        [[ "$raw_value" != *[[:space:]#]* ]] \
            || fail "the worker identity setting '$setting' must use simple quoting"
        value="$raw_value"
    fi

    identity_value="$value"
}

write_atomic_file() {
    local source_file="$1"
    local destination="$2"
    local mode="$3"
    local destination_dir
    local temporary

    destination_dir="$(dirname "$destination")"
    temporary="$(mktemp "${destination_dir}/.$(basename "$destination").XXXXXX")"
    install -m "$mode" "$source_file" "$temporary"
    mv -fT "$temporary" "$destination"
}

replace_symlink() {
    local target="$1"
    local link_path="$2"
    local link_dir
    local temporary

    link_dir="$(dirname "$link_path")"
    temporary="${link_dir}/.$(basename "$link_path").${release_id}.$$"
    rm -f "$temporary"
    ln -s "$target" "$temporary"
    mv -fT "$temporary" "$link_path"
}

restore_file() {
    local existed="$1"
    local backup="$2"
    local destination="$3"
    local mode="$4"

    if [[ "$existed" == "true" ]]; then
        write_atomic_file "$backup" "$destination" "$mode"
    else
        rm -f "$destination"
    fi
}

assert_no_running_sessions() {
    local running_containers

    if ! running_containers="$(
        docker ps \
            --quiet \
            --filter "label=io.robotswarm.managed=true"
    )"
    then
        echo "Could not inspect managed simulation containers." >&2
        return 1
    fi

    if [[ -n "$running_containers" ]]; then
        echo "Managed simulation containers are still running; deployment is blocked." >&2
        return 1
    fi
}

wait_for_readiness() {
    local journal_cursor="$1"
    local expected_image_version="$2"
    local deadline=$((SECONDS + verify_timeout))
    local logs
    local latest_pid
    local main_pid

    while (( SECONDS < deadline )); do
        if systemctl --user is-active --quiet "$service_name"; then
            if ! main_pid="$(
                systemctl --user show \
                    --property MainPID \
                    --value \
                    "$service_name"
            )"
            then
                sleep 2
                continue
            fi

            if [[ ! "$main_pid" =~ ^[1-9][0-9]*$ ]]; then
                sleep 2
                continue
            fi

            logs="$(
                journalctl --user \
                    --unit "$service_name" \
                    --after-cursor "$journal_cursor" \
                    "_PID=${main_pid}" \
                    --no-pager \
                    --output cat 2>/dev/null || true
            )"

            if grep -Fq "Worker hub rejected a control-plane operation." <<<"$logs"; then
                echo "The backend rejected worker registration or heartbeat." >&2
                return 1
            fi

            if grep -Fq \
                "Worker ready for control-plane commands. ImageVersion=${expected_image_version}" \
                <<<"$logs"
            then
                if latest_pid="$(
                    systemctl --user show \
                        --property MainPID \
                        --value \
                        "$service_name"
                )" \
                    && [[ "$latest_pid" == "$main_pid" ]] \
                    && systemctl --user is-active --quiet "$service_name"
                then
                    return 0
                fi
            fi
        fi

        sleep 2
    done

    echo "The worker did not complete registration and its first heartbeat." >&2
    systemctl --user status "$service_name" --no-pager --full >&2 || true
    journalctl --user --unit "$service_name" --after-cursor "$journal_cursor" \
        --no-pager --lines 120 >&2 || true
    return 1
}

while (( $# > 0 )); do
    case "$1" in
        --publish-dir)
            publish_dir="${2:-}"
            shift 2
            ;;
        --image-id)
            image_id="${2:-}"
            shift 2
            ;;
        --revision)
            revision="${2:-}"
            shift 2
            ;;
        --release-id)
            release_id="${2:-}"
            shift 2
            ;;
        --gpu-request)
            gpu_request="${2:-}"
            shift 2
            ;;
        --unit-file)
            unit_file="${2:-}"
            shift 2
            ;;
        --verify-timeout)
            verify_timeout="${2:-}"
            shift 2
            ;;
        --help|-h)
            usage
            exit 0
            ;;
        *)
            usage >&2
            fail "unknown argument '$1'"
            ;;
    esac
done

[[ -d "$publish_dir" ]] || fail "publish directory does not exist"
[[ -x "$publish_dir/SwarmWorker" ]] || fail "published SwarmWorker executable is missing"
[[ -f "$publish_dir/$viewer_publisher_name" \
    && ! -L "$publish_dir/$viewer_publisher_name" ]] \
    || fail "published viewer helper is missing or is not a regular file"
[[ -f "$unit_file" ]] || fail "systemd unit file does not exist"
[[ "$image_id" =~ ^sha256:[a-f0-9]{64}$ ]] || fail "image ID must be an exact sha256 Docker image ID"
[[ "$revision" =~ ^[a-f0-9]{40}$ ]] || fail "revision must be a full 40-character Git SHA"
[[ "$release_id" =~ ^[A-Za-z0-9._-]+$ ]] || fail "release ID contains unsafe characters"
[[ "$verify_timeout" =~ ^[1-9][0-9]*$ ]] || fail "verify timeout must be a positive integer"
(( verify_timeout >= 15 && verify_timeout <= 300 )) || fail "verify timeout must be between 15 and 300 seconds"

if [[ ! "$gpu_request" =~ ^(all|device=([0-9]+|GPU-[A-Fa-f0-9-]+)(,([0-9]+|GPU-[A-Fa-f0-9-]+))*)$ ]]; then
    fail "GPU request must be 'all' or a comma-separated Docker device request"
fi

require_command docker
require_command id
require_command install
require_command journalctl
require_command mktemp
require_command systemctl

if [[ -z "${XDG_RUNTIME_DIR:-}" ]]; then
    XDG_RUNTIME_DIR="/run/user/$(id -u)"
fi
if [[ -z "${DBUS_SESSION_BUS_ADDRESS:-}" ]]; then
    DBUS_SESSION_BUS_ADDRESS="unix:path=${XDG_RUNTIME_DIR}/bus"
fi
export XDG_RUNTIME_DIR DBUS_SESSION_BUS_ADDRESS

[[ -d "$XDG_RUNTIME_DIR" && -S "$XDG_RUNTIME_DIR/bus" ]] \
    || fail "the systemd user bus is unavailable at '$XDG_RUNTIME_DIR/bus'"

docker_executable="$(command -v docker)"
[[ "$docker_executable" == /* ]] \
    || fail "Docker must resolve to an absolute executable path"
if [[ "$docker_executable" == *$'\n'* || "$docker_executable" == *'"'* ]]; then
    fail "the Docker executable path cannot be represented safely in systemd"
fi
docker_directory="$(dirname "$docker_executable")"

systemctl --user show-environment >/dev/null \
    || fail "the systemd user manager is not available for this runner user"
docker info >/dev/null \
    || fail "Docker is not available for this runner user"

inspected_image_id="$(docker image inspect --format '{{.Id}}' "$image_id")"
[[ "$inspected_image_id" == "$image_id" ]] \
    || fail "the exact ROS image ID is not available locally"

image_revision="$(
    docker image inspect \
        --format '{{index .Config.Labels "org.opencontainers.image.revision"}}' \
        "$image_id"
)"
[[ "$image_revision" == "$revision" ]] \
    || fail "the ROS image revision label does not match the requested Git SHA"

config_dir="$HOME/.config/robotswarm"
state_dir="$HOME/.local/share/robotswarm-gpu-worker"
releases_dir="${state_dir}/releases"
current_link="${state_dir}/current"
identity_file="${config_dir}/gpu-worker-identity.env"
unit_dir="$HOME/.config/systemd/user"
installed_unit="${unit_dir}/${service_name}"

install -d -m 0700 "$config_dir" "$state_dir" "$releases_dir" "$unit_dir"

[[ -f "$identity_file" && ! -L "$identity_file" ]] \
    || fail "create the regular identity file '$identity_file' before deploying"

identity_owner="$(stat -c '%u' "$identity_file")"
identity_mode="$(stat -c '%a' "$identity_file")"
[[ "$identity_owner" == "$(id -u)" ]] \
    || fail "the worker identity file must be owned by the runner user"
(( (8#$identity_mode & 077) == 0 )) \
    || fail "the worker identity file must not be readable by group or other users"

for setting in \
    Worker__BackendUrl \
    Worker__WorkerId \
    Worker__WorkerSecret \
    Worker__Name \
    Worker__MaxConcurrentSessions \
    Worker__MaxRobotsPerSession
do
    grep -Eq "^${setting}=" "$identity_file" \
        || fail "the worker identity file is missing '$setting'"
done

if grep -Eq 'replace-with|00000000-0000-0000-0000-000000000000' "$identity_file"; then
    fail "the worker identity file still contains template placeholders"
fi

viewer_enabled="false"
identity_value=""
if read_identity_value Worker__Viewer__Enabled
then
    configured_viewer_enabled="$identity_value"
    case "${configured_viewer_enabled,,}" in
        true|false)
            viewer_enabled="${configured_viewer_enabled,,}"
            ;;
        *)
            fail "Worker__Viewer__Enabled must be true or false"
            ;;
    esac
fi

viewer_publish_base_url=""
viewer_encoder="auto"
viewer_display_transport="unix"
if [[ "$viewer_enabled" == "true" ]]; then
    if ! read_identity_value Worker__Viewer__PublishBaseUrl \
        || [[ -z "$identity_value" ]]
    then
        fail "Worker__Viewer__PublishBaseUrl is required when viewer publishing is enabled"
    fi
    viewer_publish_base_url="$identity_value"

    if read_identity_value ROBOTSWARM_VIEWER_ENCODER
    then
        configured_viewer_encoder="$identity_value"
        case "$configured_viewer_encoder" in
            auto|h264_nvenc|libx264)
                viewer_encoder="$configured_viewer_encoder"
                ;;
            *)
                fail "ROBOTSWARM_VIEWER_ENCODER must be auto, h264_nvenc, or libx264"
            ;;
        esac
    fi

    if read_identity_value ROBOTSWARM_VIEWER_DISPLAY_TRANSPORT
    then
        configured_display_transport="$identity_value"
        case "$configured_display_transport" in
            unix|tcp)
                viewer_display_transport="$configured_display_transport"
                ;;
            *)
                fail "ROBOTSWARM_VIEWER_DISPLAY_TRANSPORT must be unix or tcp"
                ;;
        esac
    fi
fi

[[ ! -e "$current_link" || -L "$current_link" ]] \
    || fail "'$current_link' must be a symlink"

release_dir="${releases_dir}/${release_id}"
[[ ! -e "$release_dir" ]] || fail "release '$release_id' already exists"

backup_dir="$(mktemp -d "${state_dir}/.deploy-backup.XXXXXX")"
staging_dir="$(mktemp -d "${releases_dir}/.${release_id}.XXXXXX")"
rollback_armed="false"

cleanup() {
    local exit_status=$?

    trap - EXIT INT TERM
    if [[ "$rollback_armed" == "true" ]]; then
        rollback_armed="false"
        if ! rollback; then
            exit_status=1
        elif (( exit_status == 0 )); then
            exit_status=1
        fi
    fi

    rm -rf "$backup_dir" "$staging_dir"
    exit "$exit_status"
}
trap cleanup EXIT
trap 'exit 130' INT
trap 'exit 143' TERM

had_current="false"
previous_target=""
previous_image_version=""
if [[ -L "$current_link" ]]; then
    had_current="true"
    previous_target="$(readlink "$current_link")"
    if ! previous_release_dir="$(readlink -f "$current_link")"; then
        fail "the current worker release symlink cannot be resolved"
    fi

    previous_release_environment="${previous_release_dir}/gpu-worker-release.env"
    [[ -f "$previous_release_environment" ]] \
        || fail "the current worker release environment is missing"
    previous_image_version="$(
        sed -n \
            's/^Worker__ImageVersion="\([^"]*\)"$/\1/p' \
            "$previous_release_environment"
    )"
    [[ "$previous_image_version" =~ ^[a-f0-9]{40}\+[a-f0-9]{12}$ ]] \
        || fail "the current worker image version cannot be read"
fi

had_unit="false"
if [[ -f "$installed_unit" ]]; then
    had_unit="true"
    cp -p "$installed_unit" "$backup_dir/$service_name"
fi

was_running="false"
if systemctl --user is-active --quiet "$service_name"; then
    was_running="true"
elif [[ "$had_current" == "true" || "$had_unit" == "true" ]]; then
    if ! previous_active_state="$(
        systemctl --user show \
            --property ActiveState \
            --value \
            "$service_name"
    )"
    then
        fail "the current worker service state could not be inspected"
    fi

    case "$previous_active_state" in
        active|activating|reloading)
            was_running="true"
            ;;
        inactive|failed|deactivating)
            ;;
        *)
            fail "unsupported worker service state '$previous_active_state'"
            ;;
    esac
fi

was_enabled="false"
if systemctl --user is-enabled --quiet "$service_name"; then
    was_enabled="true"
fi

if [[ "$was_running" == "true" && "$had_current" != "true" ]]; then
    fail "the active worker service is not managed by the release symlink"
fi

rollback() {
    local rollback_failed=0

    echo "Deployment verification failed; restoring the previous worker release." >&2
    if ! systemctl --user stop "$service_name" >/dev/null 2>&1; then
        echo "Could not stop the failed worker service." >&2
        rollback_failed=1
    fi

    if [[ "$had_current" == "true" ]]; then
        if ! replace_symlink "$previous_target" "$current_link"; then
            echo "Could not restore the previous release symlink." >&2
            rollback_failed=1
        fi
    else
        if ! rm -f "$current_link"; then
            echo "Could not remove the failed release symlink." >&2
            rollback_failed=1
        fi
    fi

    if ! restore_file \
        "$had_unit" \
        "$backup_dir/$service_name" \
        "$installed_unit" \
        0644
    then
        echo "Could not restore the previous systemd unit." >&2
        rollback_failed=1
    fi

    if ! systemctl --user daemon-reload >/dev/null 2>&1; then
        echo "Could not reload systemd after rollback." >&2
        rollback_failed=1
    fi

    if [[ "$was_enabled" == "true" ]]; then
        if ! systemctl --user enable "$service_name" >/dev/null 2>&1; then
            echo "Could not restore the enabled service state." >&2
            rollback_failed=1
        fi
    else
        if ! systemctl --user disable "$service_name" >/dev/null 2>&1; then
            echo "Could not restore the disabled service state." >&2
            rollback_failed=1
        fi
    fi

    if [[ "$was_running" == "true" && "$had_current" == "true" ]]; then
        if ! systemctl --user restart "$service_name" >/dev/null 2>&1 \
            || ! wait_for_readiness "$journal_cursor" "$previous_image_version"
        then
            echo "The previous worker service did not recover readiness." >&2
            rollback_failed=1
        fi
    elif ! systemctl --user is-active --quiet "$service_name"; then
        :
    else
        echo "The worker service remained active after rollback." >&2
        rollback_failed=1
    fi

    if (( rollback_failed != 0 )); then
        echo "CRITICAL: worker rollback is incomplete; manual recovery is required." >&2
        return 1
    fi

    echo "The previous worker release was restored." >&2
    return 0
}

attempt_rollback() {
    rollback_armed="false"
    rollback
}

cp -a "$publish_dir/." "$staging_dir/"
test -x "$staging_dir/SwarmWorker"
install -m 0755 \
    "$publish_dir/$viewer_publisher_name" \
    "$staging_dir/$viewer_publisher_name"
test "$(stat -c '%a' "$staging_dir/$viewer_publisher_name")" = "755"

effective_unit_file="$unit_file"
if [[ "$viewer_enabled" == "true" \
    && "$viewer_display_transport" == "unix" ]]
then
    private_tmp_count="$(grep -Ec '^PrivateTmp=(true|false)$' "$unit_file" || true)"
    [[ "$private_tmp_count" == "1" ]] \
        || fail "the worker unit must define PrivateTmp exactly once"

    effective_unit_file="$backup_dir/${service_name}.viewer-enabled"
    sed 's/^PrivateTmp=true$/PrivateTmp=false/' \
        "$unit_file" > "$effective_unit_file"
    grep -Fxq 'PrivateTmp=false' "$effective_unit_file" \
        || fail "the viewer-enabled worker unit must expose the host X socket"
elif [[ "$viewer_enabled" == "true" ]]; then
    private_tmp_count="$(grep -Ec '^PrivateTmp=(true|false)$' "$unit_file" || true)"
    [[ "$private_tmp_count" == "1" ]] \
        || fail "the worker unit must define PrivateTmp exactly once"
    grep -Fxq 'PrivateTmp=true' "$unit_file" \
        || fail "the TCP viewer transport requires PrivateTmp=true"
fi

if [[ "$viewer_enabled" == "true" ]]; then
    if ! ROBOTSWARM_VIEWER_ENCODER="$viewer_encoder" \
        ROBOTSWARM_VIEWER_DISPLAY_TRANSPORT="$viewer_display_transport" \
        ROBOTSWARM_VIEWER_GZCLIENT_GPU_REQUEST="$gpu_request" \
        "$staging_dir/$viewer_publisher_name" probe \
            --protocol-version 1 \
            --publish-base-url "$viewer_publish_base_url" \
            >/dev/null
    then
        fail "the viewer helper host-readiness probe failed"
    fi
fi

image_digest="${image_id#sha256:}"
image_version="${revision}+${image_digest:0:12}"
cat > "$staging_dir/gpu-worker-release.env" <<EOF
Worker__SessionImage="${image_id}"
Worker__ImageVersion="${image_version}"
Worker__AllowMutableSessionImage="false"
Worker__EnableGpu="true"
Worker__GpuRequest="${gpu_request}"
Worker__DockerExecutable="${docker_executable}"
Worker__Viewer__PublisherExecutable="${release_dir}/${viewer_publisher_name}"
PATH="${docker_directory}:/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin"
EOF
if [[ "$viewer_enabled" == "true" ]]; then
    printf 'ROBOTSWARM_VIEWER_ENCODER="%s"\n' \
        "$viewer_encoder" >> "$staging_dir/gpu-worker-release.env"
    printf 'ROBOTSWARM_VIEWER_DISPLAY_TRANSPORT="%s"\n' \
        "$viewer_display_transport" >> "$staging_dir/gpu-worker-release.env"
    printf 'ROBOTSWARM_VIEWER_GZCLIENT_GPU_REQUEST="%s"\n' \
        "$gpu_request" >> "$staging_dir/gpu-worker-release.env"
fi
chmod 0600 "$staging_dir/gpu-worker-release.env"

cat > "$staging_dir/DEPLOYMENT" <<EOF
GIT_REVISION=${revision}
ROS_IMAGE_ID=${image_id}
RELEASE_ID=${release_id}
EOF
chmod 0644 "$staging_dir/DEPLOYMENT"

if ! journal_output="$(
    journalctl --user --lines 0 --show-cursor --no-pager
)"
then
    fail "could not read the systemd journal before deployment"
fi
journal_cursor="$(sed -n 's/^-- cursor: //p' <<<"$journal_output")"
if [[ -z "$journal_cursor" ]]; then
    fail "could not capture the systemd journal cursor"
fi

assert_no_running_sessions \
    || fail "drain all GPU sessions and disable new scheduling before retrying"

rollback_armed="true"
if [[ "$had_current" == "true" || "$had_unit" == "true" ]]; then
    if ! systemctl --user stop "$service_name"; then
        fail "the current worker service did not stop cleanly"
    fi

    if ! stopped_active_state="$(
        systemctl --user show \
            --property ActiveState \
            --value \
            "$service_name"
    )"
    then
        fail "the stopped worker service state could not be inspected"
    fi
    case "$stopped_active_state" in
        inactive)
            ;;
        failed)
            if ! stopped_main_pid="$(
                systemctl --user show \
                    --property MainPID \
                    --value \
                    "$service_name"
            )"
            then
                fail "the failed worker service process could not be inspected"
            fi
            [[ "$stopped_main_pid" == "0" ]] \
                || fail "the failed worker service still has a running process"
            echo "The previous worker reported a failed shutdown, but no process remains; deployment will continue." >&2
            ;;
        *)
            fail "the current worker service did not stop (state '$stopped_active_state')"
            ;;
    esac
fi

if ! assert_no_running_sessions; then
    if ! attempt_rollback; then
        :
    fi
    fail "a managed session appeared while entering the deployment window"
fi

if ! mv "$staging_dir" "$release_dir"; then
    if ! attempt_rollback; then
        :
    fi
    exit 1
fi
staging_dir=""

if ! write_atomic_file "$effective_unit_file" "$installed_unit" 0644 \
    || ! replace_symlink "$release_dir" "$current_link"
then
    if ! attempt_rollback; then
        :
    fi
    exit 1
fi

if ! systemctl --user daemon-reload \
    || ! systemctl --user enable "$service_name" \
    || ! systemctl --user start "$service_name" \
    || ! wait_for_readiness "$journal_cursor" "$image_version"
then
    if ! attempt_rollback; then
        :
    fi
    exit 1
fi

if ! active_target="$(readlink "$current_link")"; then
    if ! attempt_rollback; then
        :
    fi
    fail "the active release symlink could not be inspected"
fi
if [[ "$active_target" != "$release_dir" ]]; then
    if ! attempt_rollback; then
        :
    fi
    fail "the active release changed unexpectedly after verification"
fi

rollback_armed="false"
echo "GPU worker release '$release_id' is active."
echo "ROS sessions are pinned to local image '$image_id'."
