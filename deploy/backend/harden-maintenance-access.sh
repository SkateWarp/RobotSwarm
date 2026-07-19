#!/usr/bin/env bash

set -Eeuo pipefail
IFS=$'\n\t'
umask 077

cloudflared_unit=/etc/systemd/system/cloudflared.service
cloudflared_environment=/etc/cloudflared/robotswarm-tunnel.env
vnc_dropin=/etc/systemd/system/vncserver@.service.d/10-robotswarm-loopback.conf
websockify_dropin=/etc/systemd/system/websockify.service.d/10-robotswarm-loopback.conf
backup_root=/var/lib/robotswarm-hardening/backups
backup_dir=""
committed=false

fail() {
    echo "maintenance hardening: $*" >&2
    exit 1
}

require_command() {
    command -v "$1" >/dev/null 2>&1 \
        || fail "required command '$1' was not found"
}

backup_file() {
    local source=$1
    local key=$2

    if [[ -e "$source" || -L "$source" ]]; then
        cp -a -- "$source" "$backup_dir/$key"
    else
        : >"$backup_dir/$key.absent"
    fi
}

restore_file() {
    local destination=$1
    local key=$2

    if [[ -e "$backup_dir/$key.absent" ]]; then
        rm -f -- "$destination"
    else
        install -D -m "$(stat -c '%a' "$backup_dir/$key")" \
            "$backup_dir/$key" "$destination"
    fi
}

restart_services() {
    systemctl daemon-reload
    systemctl restart vncserver@1.service
    systemctl restart websockify.service
    systemctl restart cloudflared.service
}

rollback() {
    local status=$?

    trap - ERR INT TERM
    if [[ "$committed" == true || -z "$backup_dir" ]]; then
        exit "$status"
    fi

    echo "maintenance hardening: restoring the previous service configuration" >&2
    restore_file "$cloudflared_unit" cloudflared.service
    restore_file "$cloudflared_environment" robotswarm-tunnel.env
    restore_file "$vnc_dropin" vnc-loopback.conf
    restore_file "$websockify_dropin" websockify-loopback.conf
    restart_services || true
    exit "$status"
}

assert_loopback_listener() {
    local port=$1
    local addresses

    addresses="$(
        ss -H -lnt \
            | awk -v suffix=":$port" '$4 ~ suffix "$" { print $4 }'
    )"
    [[ -n "$addresses" ]] || fail "nothing is listening on TCP port $port"

    while IFS= read -r address; do
        case "$address" in
            "127.0.0.1:$port"|"[::1]:$port") ;;
            *) fail "TCP port $port is still exposed at $address" ;;
        esac
    done <<<"$addresses"
}

if [[ "${1:-}" != "--apply" || $# != 1 ]]; then
    echo "Usage: sudo $0 --apply" >&2
    exit 2
fi

[[ ${EUID:-$(id -u)} -eq 0 ]] || fail "run this script as root"

for command in \
    awk cp date dirname grep install mktemp python3 sed ss stat \
    systemctl systemd-analyze tr
do
    require_command "$command"
done

[[ -f "$cloudflared_unit" && ! -L "$cloudflared_unit" ]] \
    || fail "the cloudflared systemd unit is missing or is a symlink"

install -d -m 0700 "$backup_root"
backup_dir="$backup_root/$(date -u +%Y%m%dT%H%M%SZ)-$$"
install -d -m 0700 "$backup_dir"

backup_file "$cloudflared_unit" cloudflared.service
backup_file "$cloudflared_environment" robotswarm-tunnel.env
backup_file "$vnc_dropin" vnc-loopback.conf
backup_file "$websockify_dropin" websockify-loopback.conf
trap rollback ERR INT TERM

token="$(
    sed -nE \
        's/^ExecStart=.*[[:space:]]--token(=|[[:space:]]+)([^[:space:]]+).*$/\2/p' \
        "$cloudflared_unit"
)"

if [[ -n "$token" ]]; then
    [[ "$token" =~ ^[A-Za-z0-9._=-]{40,}$ ]] \
        || fail "the tunnel unit contains an unexpected token format"

    install -d -m 0700 "$(dirname "$cloudflared_environment")"
    token_file="$(mktemp "$backup_dir/tunnel-environment.XXXXXX")"
    printf 'TUNNEL_TOKEN=%s\n' "$token" >"$token_file"
    install -m 0600 "$token_file" "$cloudflared_environment"
elif [[ ! -s "$cloudflared_environment" ]]; then
    fail "no tunnel token was found in either the unit or protected environment file"
fi

sanitized_unit="$(mktemp "$backup_dir/cloudflared-unit.XXXXXX")"
python3 - "$cloudflared_unit" "$sanitized_unit" <<'PY'
import re
import sys

source, destination = sys.argv[1:]
lines = []
service_seen = False
environment_seen = False

with open(source, encoding="utf-8") as stream:
    for original in stream:
        line = re.sub(r"\s+--token(?:=|\s+)\S+", "", original.rstrip("\n"))
        if line == "[Service]":
            service_seen = True
        if line.startswith("EnvironmentFile=") and "robotswarm-tunnel.env" in line:
            environment_seen = True
        lines.append(line)

if not service_seen:
    raise SystemExit("cloudflared unit has no Service section")

if not environment_seen:
    service_index = lines.index("[Service]")
    lines.insert(
        service_index + 1,
        "EnvironmentFile=/etc/cloudflared/robotswarm-tunnel.env",
    )

with open(destination, "w", encoding="utf-8", newline="\n") as stream:
    stream.write("\n".join(lines) + "\n")
PY

grep -Fq 'EnvironmentFile=/etc/cloudflared/robotswarm-tunnel.env' "$sanitized_unit" \
    || fail "the protected tunnel environment was not added to the unit"
if grep -Eq -- '(^|[[:space:]])--token([=[:space:]]|$)' "$sanitized_unit"; then
    fail "the sanitized tunnel unit still contains a token argument"
fi
install -m 0644 "$sanitized_unit" "$cloudflared_unit"

install -d -m 0755 "$(dirname "$vnc_dropin")" "$(dirname "$websockify_dropin")"

websockify_binary="${WEBSOCKIFY_BIN:-$(
    systemctl cat websockify.service \
        | awk '/^ExecStart=/{value=$0} END{sub(/^ExecStart=/, "", value); split(value, part, /[[:space:]]+/); print part[1]}'
)}"
[[ "$websockify_binary" == /* && "$websockify_binary" != *[[:space:]]* ]] \
    || fail "could not determine an absolute websockify executable path"
[[ -x "$websockify_binary" ]] \
    || fail "websockify executable is not available at $websockify_binary"

vnc_config="$(mktemp "$backup_dir/vnc-loopback.XXXXXX")"
printf '%s\n' \
    '[Service]' \
    'ExecStart=' \
    'ExecStart=/usr/bin/vncserver -fg -depth 24 -geometry 1920x1080 -localhost yes :%i' \
    >"$vnc_config"
install -m 0644 "$vnc_config" "$vnc_dropin"

websockify_config="$(mktemp "$backup_dir/websockify-loopback.XXXXXX")"
printf '%s\n' \
    '[Service]' \
    'ExecStart=' \
    "ExecStart=$websockify_binary 127.0.0.1:6080 localhost:5901" \
    >"$websockify_config"
install -m 0644 "$websockify_config" "$websockify_dropin"

systemd-analyze verify \
    vncserver@1.service websockify.service cloudflared.service >/dev/null
restart_services

for service in vncserver@1.service websockify.service cloudflared.service; do
    systemctl is-active --quiet "$service" \
        || fail "$service did not return to the active state"
done

assert_loopback_listener 5901
assert_loopback_listener 6080

cloudflared_pid="$(systemctl show cloudflared.service --property MainPID --value)"
[[ "$cloudflared_pid" =~ ^[1-9][0-9]*$ ]] \
    || fail "cloudflared did not report a valid process ID"
if tr '\0' ' ' <"/proc/$cloudflared_pid/cmdline" \
    | grep -Eq -- '(^|[[:space:]])--token([=[:space:]]|$)'; then
    fail "the tunnel token is still present in the cloudflared process arguments"
fi

[[ "$(stat -c '%a:%U:%G' "$cloudflared_environment")" == "600:root:root" ]] \
    || fail "the protected tunnel environment has unexpected permissions"

committed=true
trap - ERR INT TERM
echo "maintenance hardening: applied; rollback backup is $backup_dir"
