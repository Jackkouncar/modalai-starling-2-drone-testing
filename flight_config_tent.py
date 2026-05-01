"""Shared settings for the ModalAI Starling 2 indoor flight tests."""

import os


TARGET_DRONE = "ModalAI Starling 2"
TARGET_ROS_DISTRO = "foxy"

TAKEOFF_HEIGHT_M = 0.5
TAKEOFF_Z_NED = -TAKEOFF_HEIGHT_M

# How long each horizontal transit leg takes (seconds).
#
# Tent constraint: 7 ft × 7 ft (~2.13 m × 2.13 m), launching from center.
# Max safe reach from center = 2.13 / 2 = 1.06 m.
# TRANSIT_DISTANCE_M is set to 0.8 m to keep a ~0.26 m margin on every side
# for the drone body, prop wash drift, and positioning error.
# The square-test diagonal reach is 0.8 * sqrt(2) ≈ 1.13 m — still inside tent.
# 0.8 m over 5 s ≈ 0.16 m/s average (very gentle for a confined space).
TRANSIT_DISTANCE_M = 0.8
TRANSIT_DURATION_S = 5.0

# How long to wait for the drone to stabilize at a waypoint before moving on.
HOVER_SETTLE_S = 3.0

SOFT_LAND_FINAL_HEIGHT_M = 0.15
SOFT_LAND_FINAL_Z_NED = -SOFT_LAND_FINAL_HEIGHT_M
SOFT_LAND_DESCENT_SECONDS = 6.0   # slightly longer descent for safety
LAND_COMMAND_SECONDS = 4.0


def smooth_transit_xy(elapsed_s, x_start, y_start, x_end, y_end,
                      duration_s=TRANSIT_DURATION_S):
    """
    Return an (x, y) position setpoint that ramps smoothly from
    (x_start, y_start) to (x_end, y_end) over duration_s seconds using a
    smoothstep (3t² - 2t³) ease-in/out curve.

    Smoothstep has zero velocity at both endpoints, so the drone accelerates
    gently at the start of each leg and decelerates to a near-stop before the
    next hover phase — no abrupt velocity discontinuities.

    After duration_s the endpoint is returned unchanged (safe to call past end).
    """
    t = min(max(elapsed_s / duration_s, 0.0), 1.0)
    # Smoothstep: zero first-derivative at t=0 and t=1
    s = t * t * (3.0 - 2.0 * t)
    x = x_start + s * (x_end - x_start)
    y = y_start + s * (y_end - y_start)
    return x, y


def soft_landing_z(elapsed_seconds):
    """
    Ramp from takeoff height down close to the floor before PX4 land mode.
    Uses smoothstep so descent starts and ends gently.
    """
    t = min(max(elapsed_seconds / SOFT_LAND_DESCENT_SECONDS, 0.0), 1.0)
    s = t * t * (3.0 - 2.0 * t)   # smoothstep
    return TAKEOFF_Z_NED + s * (SOFT_LAND_FINAL_Z_NED - TAKEOFF_Z_NED)


def log_environment_check(node):
    """Log the ROS 2 distro so the team can catch Humble/Foxy mixups early."""
    ros_distro = os.environ.get("ROS_DISTRO")

    if ros_distro is None:
        node.get_logger().warn(
            f"ROS_DISTRO is not set. This repo is currently targeted for ROS 2 {TARGET_ROS_DISTRO}."
        )
        return

    if ros_distro != TARGET_ROS_DISTRO:
        node.get_logger().warn(
            f"ROS_DISTRO is '{ros_distro}', but these tests are targeted for ROS 2 {TARGET_ROS_DISTRO}."
        )
        return

    node.get_logger().info(f"ROS 2 distro check passed: {ros_distro}")