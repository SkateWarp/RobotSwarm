#!/usr/bin/env bash
set -Eeuo pipefail

container="robotswarm-postgres-races-${$}"
password="local-postgres-race-test"

cleanup() {
    docker rm --force "$container" >/dev/null 2>&1 || true
}
trap cleanup EXIT INT TERM

if ! command -v docker >/dev/null 2>&1; then
    echo "Docker is required to run the opt-in PostgreSQL concurrency tests." >&2
    exit 1
fi

docker run --detach --rm \
    --name "$container" \
    --publish 127.0.0.1::5432 \
    --env POSTGRES_DB=robotswarm_test \
    --env POSTGRES_USER=postgres \
    --env "POSTGRES_PASSWORD=$password" \
    postgres:17 >/dev/null

for _ in $(seq 1 60); do
    if docker exec "$container" pg_isready \
        --username postgres \
        --dbname robotswarm_test >/dev/null 2>&1; then
        break
    fi
    sleep 1
done

if ! docker exec "$container" pg_isready \
    --username postgres \
    --dbname robotswarm_test >/dev/null 2>&1; then
    echo "PostgreSQL did not become ready within 60 seconds." >&2
    exit 1
fi

port="$(docker port "$container" 5432/tcp | sed -n 's/.*://p' | head -n 1)"
if [[ -z "$port" ]]; then
    echo "Docker did not publish the PostgreSQL port." >&2
    exit 1
fi

connection="Host=127.0.0.1;Port=$port;Database=robotswarm_test;Username=postgres;Password=$password;Pooling=false;Timeout=5;Command Timeout=30"
server_version="$(docker exec "$container" psql \
    --username postgres \
    --dbname robotswarm_test \
    --tuples-only \
    --no-align \
    --command 'SHOW server_version;')"

echo "Running opt-in PostgreSQL $server_version concurrency acceptance tests..."
ROBOTSWARM_POSTGRES_TEST_CONNECTION="$connection" \
    dotnet test SwarmBackend.Tests/SwarmBackend.Tests.csproj \
    --configuration Release \
    --filter 'Category=PostgreSQL'
