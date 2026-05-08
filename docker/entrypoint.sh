#!/usr/bin/env bash
set -e

ROS_DISTRO="${ROS_DISTRO:-humble}"
ROS_WS="${ROS_WS:-/workspace/ros2_ws}"

if [ -f "/opt/ros/${ROS_DISTRO}/setup.bash" ]; then
    # shellcheck disable=SC1090
    source "/opt/ros/${ROS_DISTRO}/setup.bash"
fi

if [ -f "${ROS_WS}/install/setup.bash" ]; then
    # shellcheck disable=SC1090
    source "${ROS_WS}/install/setup.bash"
fi

export DRONE_SIM="${DRONE_SIM:-1}"
export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_fastrtps_cpp}"

exec "$@"
