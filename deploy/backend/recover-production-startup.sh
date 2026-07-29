#!/usr/bin/env bash

set -Eeuo pipefail
IFS=$'\n\t'

readonly project_name=robotswarm
readonly network_wait_seconds="${ROBOTSWARM_NETWORK_WAIT_SECONDS:-120}"
readonly health_wait_seconds="${ROBOTSWARM_HEALTH_WAIT_SECONDS:-180}"
readonly poll_seconds="${ROBOTSWARM_POLL_SECONDS:-2}"

readonly -a containers=(
    db_prod
    media_prod
    backend_prod
)

declare -Ar expected_services=(
    [db_prod]=db
    [media_prod]=media
    [backend_prod]=backend
)

declare -Ar published_ports=(
    [media_prod]=8554/tcp
    [backend_prod]=44336/tcp
)

configuration_error() {
    echo "production startup recovery: $*" >&2
    exit 78
}

temporary_error() {
    echo "production startup recovery: $*" >&2
    exit 75
}

require_nonnegative_integer() {
    local name=$1
    local value=$2

    [[ "$value" =~ ^[0-9]+$ ]] \
        || configuration_error "$name must be a non-negative integer"
}

inspect_value() {
    local container=$1
    local template=$2

    docker inspect --format "$template" "$container"
}

docker_daemon_ready() {
    docker info --format '{{.ServerVersion}}' >/dev/null 2>&1
}

listed_container_names() {
    local container=$1

    docker container ls \
        --all \
        --filter "name=^/${container}$" \
        --format '{{.Names}}'
}

validate_container_identity() {
    local container=$1
    local expected_service=${expected_services[$container]}
    local identity
    local listed_names

    if ! inspect_value "$container" '{{.Id}}' >/dev/null 2>&1; then
        if ! docker_daemon_ready; then
            temporary_error \
                "Docker is not ready while inspecting $container"
        fi

        if ! listed_names="$(listed_container_names "$container")"; then
            temporary_error \
                "Docker could not confirm whether $container exists"
        fi
        if [[ -z "$listed_names" ]]; then
            configuration_error "required container $container does not exist"
        fi
        temporary_error \
            "Docker lists $container but could not inspect its identity"
    fi

    if ! identity="$(
        inspect_value "$container" \
            '{{index .Config.Labels "com.docker.compose.project"}}|{{index .Config.Labels "com.docker.compose.service"}}'
    )"; then
        temporary_error \
            "Docker could not inspect the Compose identity for $container"
    fi

    if [[ "$identity" != "$project_name|$expected_service" ]]; then
        configuration_error \
            "$container does not belong to $project_name/$expected_service"
    fi
}

binding_address() {
    local container=$1
    local port=${published_ports[$container]}
    local template
    local address

    printf -v template \
        '{{with index .HostConfig.PortBindings "%s"}}{{range .}}{{println .HostIp}}{{end}}{{end}}' \
        "$port"
    if ! address="$(inspect_value "$container" "$template")"; then
        temporary_error \
            "Docker could not inspect the $port binding for $container"
    fi

    [[ -n "$address" ]] \
        || configuration_error "$container has no host binding for $port"
    [[ "$address" != *$'\n'* ]] \
        || configuration_error "$container has more than one host binding for $port"

    if ! python3 - "$address" <<'PY'
import ipaddress
import sys

try:
    address = ipaddress.IPv4Address(sys.argv[1])
except ipaddress.AddressValueError:
    raise SystemExit(1)
if address.is_unspecified or address.is_loopback or address.is_multicast:
    raise SystemExit(1)
PY
    then
        configuration_error \
            "$container has an unusable IPv4 binding for $port"
    fi

    printf '%s\n' "$address"
}

host_has_address() {
    local expected=$1

    ip -4 -o address show scope global \
        | awk -v expected="$expected" '
            {
                split($4, address, "/")
                if (address[1] == expected) {
                    found = 1
                }
            }
            END { exit found ? 0 : 1 }
        '
}

wait_for_address() {
    local address=$1
    local deadline=$((SECONDS + network_wait_seconds))

    while ! host_has_address "$address"; do
        if (( SECONDS >= deadline )); then
            temporary_error \
                "IPv4 address $address was not assigned within ${network_wait_seconds}s"
        fi
        sleep "$poll_seconds"
    done

    echo "production startup recovery: IPv4 address $address is assigned"
}

container_status() {
    local container=$1
    local value

    if ! value="$(inspect_value "$container" '{{.State.Status}}')"; then
        temporary_error "Docker could not inspect the state for $container"
    fi

    printf '%s\n' "$value"
}

container_has_healthcheck() {
    local container=$1
    local value

    if ! value="$(
        inspect_value "$container" \
            '{{if .Config.Healthcheck}}yes{{else}}no{{end}}'
    )"; then
        temporary_error \
            "Docker could not inspect the healthcheck for $container"
    fi

    [[ "$value" == yes ]]
}

container_health() {
    local container=$1
    local value

    if ! value="$(
        inspect_value "$container" \
            '{{if .State.Health}}{{.State.Health.Status}}{{else}}unknown{{end}}'
    )"; then
        temporary_error "Docker could not inspect the health for $container"
    fi

    printf '%s\n' "$value"
}

start_if_stopped() {
    local container=$1
    local status

    status="$(container_status "$container")"
    case "$status" in
        running)
            echo "production startup recovery: $container is already running"
            ;;
        created|exited)
            echo "production startup recovery: starting $container"
            if ! docker start "$container" >/dev/null; then
                temporary_error "Docker could not start $container"
            fi
            ;;
        *)
            temporary_error \
                "$container is in state '$status'; it was not treated as stopped"
            ;;
    esac
}

wait_until_ready() {
    local container=$1
    local deadline=$((SECONDS + health_wait_seconds))
    local status
    local health
    local has_healthcheck=false

    if container_has_healthcheck "$container"; then
        has_healthcheck=true
    fi

    while true; do
        status="$(container_status "$container")"

        if [[ "$status" == running ]]; then
            if [[ "$has_healthcheck" == false ]]; then
                echo "production startup recovery: $container is running"
                return
            fi

            health="$(container_health "$container")"
            if [[ "$health" == healthy ]]; then
                echo "production startup recovery: $container is healthy"
                return
            fi
        fi

        if (( SECONDS >= deadline )); then
            if [[ "$has_healthcheck" == true ]]; then
                temporary_error \
                    "$container did not become healthy within ${health_wait_seconds}s"
            fi
            temporary_error \
                "$container did not remain running within ${health_wait_seconds}s"
        fi

        sleep "$poll_seconds"
    done
}

append_unique_address() {
    local candidate=$1
    local existing

    for existing in "${publish_addresses[@]:-}"; do
        [[ "$existing" == "$candidate" ]] && return
    done
    publish_addresses+=("$candidate")
}

main() {
    local container
    local address
    local -a publish_addresses=()

    require_nonnegative_integer \
        ROBOTSWARM_NETWORK_WAIT_SECONDS "$network_wait_seconds"
    require_nonnegative_integer \
        ROBOTSWARM_HEALTH_WAIT_SECONDS "$health_wait_seconds"
    require_nonnegative_integer \
        ROBOTSWARM_POLL_SECONDS "$poll_seconds"

    for container in "${containers[@]}"; do
        validate_container_identity "$container"
    done

    for container in media_prod backend_prod; do
        address="$(binding_address "$container")"
        append_unique_address "$address"
    done

    for address in "${publish_addresses[@]}"; do
        wait_for_address "$address"
    done

    # Keep this order explicit. The database and media endpoint can recover
    # independently; the API is exposed only after both are ready.
    for container in "${containers[@]}"; do
        start_if_stopped "$container"
        wait_until_ready "$container"
    done

    echo "production startup recovery: production containers are ready"
}

main "$@"
