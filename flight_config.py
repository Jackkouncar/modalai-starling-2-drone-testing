"""Shared settings for the ModalAI Starling 2 Max indoor flight tests."""

import os


TARGET_DRONE = "ModalAI Starling 2 Max"
TARGET_ROS_DISTRO = "foxy"

TAKEOFF_HEIGHT_M = 1.0
TAKEOFF_Z_NED = -TAKEOFF_HEIGHT_M
TAKEOFF_RAMP_SECONDS = 5.0

# How long each horizontal transit leg takes. Longer is slower and safer indoors.
TRANSIT_DISTANCE_M = 1.5
TRANSIT_DURATION_S = 6.0

# How long to wait for the drone to stabilize at a waypoint before moving on.
HOVER_SETTLE_S = 3.0

# Preflight local-position gate.
STABILITY_REQUIRED_SECONDS = 2.0
STABILITY_MAX_DRIFT_M = 0.12

# In-flight guardrails for takeoff/hover/land testing.
SAFETY_MAX_HORIZONTAL_ERROR_M = 0.75
SAFETY_MAX_ALTITUDE_M = 1.6

# Used by the movement scripts and the final descent in takeoff/land.
SOFT_LAND_FINAL_HEIGHT_M = 0.15
SOFT_LAND_FINAL_Z_NED = -SOFT_LAND_FINAL_HEIGHT_M
SOFT_LAND_DESCENT_SECONDS = 6.0
LAND_COMMAND_SECONDS = 4.0


def smoothstep(elapsed_s, duration_s):
    """Return a 0..1 smoothstep progress value."""
    t = min(max(elapsed_s / duration_s, 0.0), 1.0)
    return t * t * (3.0 - 2.0 * t)


def smooth_transit_xy(elapsed_s, x_start, y_start, x_end, y_end,
                      duration_s=TRANSIT_DURATION_S):
    """
    Return an (x, y) position setpoint that ramps smoothly between points.

    Smoothstep has zero velocity at both endpoints, so the drone accelerates
    gently at the start of each leg and decelerates before the next hover.
    """
    s = smoothstep(elapsed_s, duration_s)
    x = x_start + s * (x_end - x_start)
    y = y_start + s * (y_end - y_start)
    return x, y


def soft_landing_z(elapsed_seconds):
    """Ramp from takeoff height down close to the floor before PX4 land mode."""
    s = smoothstep(elapsed_seconds, SOFT_LAND_DESCENT_SECONDS)
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
