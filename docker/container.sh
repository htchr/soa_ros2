#!/usr/bin/env bash
#
# container.sh — manage the soa_ros2 development container.
#
# One image, driven by docker/docker-compose.yml (the single source of truth for
# the container configuration). This is the terminal-based alternative to the VS
# Code devcontainer; both produce the same environment.
#
#   ./docker/container.sh build     Build the image (very long the first time)
#   ./docker/container.sh start     Create/start the container + apply device fixes
#   ./docker/container.sh enter     Open a shell inside the running container
#   ./docker/container.sh stop      Stop the container (keeps it for next time)
#   ./docker/container.sh rebuild   Rebuild the image from scratch and recreate
#   ./docker/container.sh down      Remove the container
#   ./docker/container.sh status    Show container status
#   ./docker/container.sh logs      Follow container logs
#
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
COMPOSE_FILE="$SCRIPT_DIR/docker-compose.yml"
SERVICE="soa"
PROJECT="soa_ros2"
IMAGE="soa_ros2:humble"

# Match the container user to the host user so files created inside the container
# stay editable from the host without sudo. Overrides the defaults in docker/.env.
HOST_UID="$(id -u)"; export HOST_UID
HOST_GID="$(id -g)"; export HOST_GID

# --- resolve the compose command (v2 plugin preferred, v1 fallback) ------------
if docker compose version >/dev/null 2>&1; then
    COMPOSE=(docker compose)
elif command -v docker-compose >/dev/null 2>&1; then
    COMPOSE=(docker-compose)
else
    echo "error: neither 'docker compose' nor 'docker-compose' is available." >&2
    exit 1
fi

dc() { "${COMPOSE[@]}" -f "$COMPOSE_FILE" -p "$PROJECT" "$@"; }

# --- helpers -------------------------------------------------------------------
running_id() { dc ps -q "$SERVICE" 2>/dev/null; }

allow_gui() {
    # Let container processes talk to the host X server (rviz, rerun, moveit).
    command -v xhost >/dev/null 2>&1 && xhost +local:root >/dev/null 2>&1 || true
}

image_exists() { docker image inspect "$IMAGE" >/dev/null 2>&1; }

runtime_setup() {
    dc exec -T "$SERVICE" bash -lc \
        '/home/ubuntu/colcon_ws/src/soa_ros2/docker/scripts/setup-runtime.sh'
}

first_build_if_needed() {
    if dc exec -T "$SERVICE" test -f /home/ubuntu/.soa_built 2>/dev/null; then
        return 0
    fi
    echo ">> First start detected — building the workspace (this can take a while)..."
    dc exec -T "$SERVICE" bash -lc \
        '/home/ubuntu/colcon_ws/src/soa_ros2/docker/scripts/build-workspace.sh'
}

ensure_up() {
    if ! image_exists; then
        echo ">> Image '$IMAGE' not found — building it first (this takes a long time)..."
        dc build
    fi
    allow_gui
    dc up -d
    runtime_setup
    first_build_if_needed
}

# --- commands ------------------------------------------------------------------
cmd_build()   { echo ">> Building image (first build is very long)..."; dc build "$@"; }
cmd_start()   { ensure_up; echo ">> Container is up. Enter it with: $0 enter"; }
cmd_stop()    { dc stop; }
cmd_status()  { dc ps; }
cmd_logs()    { dc logs -f "$@"; }
cmd_down()    { dc down "$@"; }

cmd_enter() {
    ensure_up
    echo ">> Entering container (service: $SERVICE). Type 'exit' to leave."
    dc exec -w /home/ubuntu/colcon_ws/src/soa_ros2 "$SERVICE" bash -l
}

cmd_rebuild() {
    echo ">> Rebuilding image from scratch (--no-cache). This takes a long time..."
    dc build --no-cache
    allow_gui
    dc up -d --force-recreate
    runtime_setup
    first_build_if_needed
    echo ">> Rebuild complete. Enter it with: $0 enter"
}

usage() {
    # Print the leading comment block (from line 3) as help, stopping at the
    # first line of actual code.
    awk 'NR<3{next} /^#/{sub(/^# ?/,""); print; next} {exit}' "${BASH_SOURCE[0]}"
}

main() {
    local cmd="${1:-}"; shift || true
    case "$cmd" in
        build)   cmd_build "$@" ;;
        start|up) cmd_start "$@" ;;
        enter|shell|exec) cmd_enter "$@" ;;
        stop)    cmd_stop "$@" ;;
        rebuild) cmd_rebuild "$@" ;;
        down)    cmd_down "$@" ;;
        status|ps) cmd_status "$@" ;;
        logs)    cmd_logs "$@" ;;
        ""|help|-h|--help) usage ;;
        *) echo "error: unknown command '$cmd'" >&2; echo >&2; usage; exit 1 ;;
    esac
}

main "$@"
