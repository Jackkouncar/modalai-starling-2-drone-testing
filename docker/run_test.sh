#!/usr/bin/env bash
set -euo pipefail

ROS_DISTRO="${ROS_DISTRO:-humble}"
ROS_WS="${ROS_WS:-/workspace/ros2_ws}"
SCRIPT="${1:-test_takeoff_land.py}"

if [ -f "/opt/ros/${ROS_DISTRO}/setup.bash" ]; then
    set +u
    # shellcheck disable=SC1090
    source "/opt/ros/${ROS_DISTRO}/setup.bash"
    set -u
fi

if [ -f "${ROS_WS}/install/setup.bash" ]; then
    set +u
    # shellcheck disable=SC1090
    source "${ROS_WS}/install/setup.bash"
    set -u
fi

cd /workspace/drone-tests

if [ ! -f "${SCRIPT}" ]; then
    echo "Could not find '${SCRIPT}' in /workspace/drone-tests."
    exit 1
fi

if [ "${DRONE_SIM_SKIP_STATUS_GATE:-}" = "1" ]; then
    echo "Sim-only mode: skipping PX4 vehicle_status preflight gate."
    echo "Starting sim-only MAVLink GCS heartbeat for PX4 preflight."
    python3 /workspace/drone-tests/docker/send_gcs_heartbeat.py &
    heartbeat_pid=$!
fi

cleanup() {
    if [ -n "${heartbeat_pid:-}" ] && kill -0 "${heartbeat_pid}" 2>/dev/null; then
        kill "${heartbeat_pid}" 2>/dev/null || true
    fi
}
trap cleanup EXIT INT TERM

python3 "${SCRIPT}"
