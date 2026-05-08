#!/usr/bin/env bash
set -euo pipefail

ROS_DISTRO="${ROS_DISTRO:-humble}"
ROS_WS="${ROS_WS:-/workspace/ros2_ws}"
PX4_HOME="${PX4_HOME:-/workspace/PX4-Autopilot}"
PX4_SIM_TARGET="${PX4_SIM_TARGET:-gz_x500_depth}"
PX4_GZ_WORLD="${PX4_GZ_WORLD:-}"
AGENT_PORT="${AGENT_PORT:-8888}"

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

cleanup() {
    if [ -n "${agent_pid:-}" ] && kill -0 "${agent_pid}" 2>/dev/null; then
        kill "${agent_pid}" 2>/dev/null || true
    fi
}
trap cleanup EXIT INT TERM

echo "Starting MicroXRCEAgent on UDP port ${AGENT_PORT}..."
MicroXRCEAgent udp4 -p "${AGENT_PORT}" &
agent_pid=$!
sleep 2

if ! kill -0 "${agent_pid}" 2>/dev/null; then
    echo "MicroXRCEAgent exited before PX4 started."
    wait "${agent_pid}"
    exit 1
fi

cd "${PX4_HOME}"
git config --global --add safe.directory "*" || true
git config --global --add safe.directory "${PX4_HOME}" || true
git config --global --add safe.directory "${PX4_HOME}/Tools/simulation/gz" || true
git submodule foreach --recursive \
    'git config --global --add safe.directory "$toplevel/$sm_path" || true' || true

if ! grep -R -m 1 "${PX4_SIM_TARGET}" ROMFS Tools src platforms >/dev/null 2>&1; then
    echo "Warning: PX4 target '${PX4_SIM_TARGET}' was not found in this PX4 tree."
    echo "Try a standard target such as PX4_SIM_TARGET=gz_x500_depth."
    echo "If your class requires Baylands, rebuild with PX4_REPO and PX4_REF set to the class fork and branch."
fi

echo "Starting PX4 SITL target: ${PX4_SIM_TARGET}"
if [ -n "${PX4_GZ_WORLD}" ]; then
    export PX4_GZ_WORLD
    echo "Using Gazebo world: ${PX4_GZ_WORLD}"
fi
make px4_sitl "${PX4_SIM_TARGET}"
