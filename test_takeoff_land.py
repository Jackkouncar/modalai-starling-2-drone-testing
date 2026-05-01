#!/usr/bin/env python3
"""
Test Script: Takeoff and Land
- Waits for valid, stable PX4 local position
- Captures the current local position as home
- Enters offboard mode and arms
- Smoothly takes off relative to home
- Hovers for 5 seconds
- Smoothly descends near home height, then sends PX4 land
"""

import math
import time
from enum import Enum

import rclpy
from rclpy.node import Node

from flight_config import (
    LAND_COMMAND_SECONDS,
    SAFETY_MAX_ALTITUDE_M,
    SAFETY_MAX_HORIZONTAL_ERROR_M,
    SOFT_LAND_DESCENT_SECONDS,
    STABILITY_MAX_DRIFT_M,
    STABILITY_REQUIRED_SECONDS,
    TARGET_DRONE,
    TAKEOFF_HEIGHT_M,
    TAKEOFF_RAMP_SECONDS,
    log_environment_check,
)
from px4_msgs.msg import (
    OffboardControlMode,
    TrajectorySetpoint,
    VehicleCommand,
    VehicleLocalPosition,
    VehicleStatus,
)


class State(Enum):
    WAIT_FOR_POSITION = 0
    PRESTREAM = 1
    SET_OFFBOARD = 2
    ARM = 3
    TAKEOFF = 4
    HOVER = 5
    DESCEND = 6
    LAND = 7
    DONE = 8
    ABORT = 9


class TestTakeoffLand(Node):

    def __init__(self):
        super().__init__('test_takeoff_land')

        self.offboard_pub = self.create_publisher(
            OffboardControlMode, '/fmu/in/offboard_control_mode', 10)
        self.traj_pub = self.create_publisher(
            TrajectorySetpoint, '/fmu/in/trajectory_setpoint', 10)
        self.cmd_pub = self.create_publisher(
            VehicleCommand, '/fmu/in/vehicle_command', 10)

        self.local_pos_sub = self.create_subscription(
            VehicleLocalPosition,
            '/fmu/out/vehicle_local_position',
            self.local_position_cb,
            10,
        )
        self.status_sub = self.create_subscription(
            VehicleStatus,
            '/fmu/out/vehicle_status',
            self.vehicle_status_cb,
            10,
        )

        self.timer = self.create_timer(0.05, self.timer_cb)  # 20 Hz

        self.state = State.WAIT_FOR_POSITION
        self.state_start = time.time()
        self.log_counter = 0

        self.local_position = None
        self.local_position_seen_at = 0.0
        self.vehicle_status = None
        self.vehicle_status_seen_at = 0.0
        self.stability_samples = []

        self.home_x = None
        self.home_y = None
        self.home_z = None
        self.takeoff_z = None
        self.descent_final_z = None

        self.get_logger().info(f"=== {TARGET_DRONE} Test: Takeoff and Land ===")
        log_environment_check(self)

    def now_us(self):
        return self.get_clock().now().nanoseconds // 1000

    def local_position_cb(self, msg):
        self.local_position = msg
        self.local_position_seen_at = time.time()
        self.stability_samples.append((self.local_position_seen_at, msg.x, msg.y, msg.z))
        cutoff = self.local_position_seen_at - STABILITY_REQUIRED_SECONDS
        self.stability_samples = [
            sample for sample in self.stability_samples if sample[0] >= cutoff
        ]

    def vehicle_status_cb(self, msg):
        self.vehicle_status = msg
        self.vehicle_status_seen_at = time.time()

    def transition(self, new_state):
        self.state = new_state
        self.state_start = time.time()
        self.log_counter = 0
        self.get_logger().info(f"-> {new_state.name}")

    def publish_offboard_mode(self):
        msg = OffboardControlMode()
        msg.position = True
        msg.velocity = False
        msg.acceleration = False
        msg.attitude = False
        msg.body_rate = False
        msg.timestamp = self.now_us()
        self.offboard_pub.publish(msg)

    def publish_setpoint(self, x, y, z, yaw=0.0):
        msg = TrajectorySetpoint()
        msg.position = [x, y, z]
        msg.yaw = yaw
        msg.timestamp = self.now_us()
        self.traj_pub.publish(msg)

    def vehicle_command(self, command, param1=0.0, param2=0.0):
        msg = VehicleCommand()
        msg.command = command
        msg.param1 = float(param1)
        msg.param2 = float(param2)
        msg.target_system = 1
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        msg.timestamp = self.now_us()
        self.cmd_pub.publish(msg)

    def arm(self):
        self.vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0)

    def set_offboard_mode(self):
        self.vehicle_command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, 1.0, 6.0)

    def land(self):
        self.vehicle_command(VehicleCommand.VEHICLE_CMD_NAV_LAND)

    def log_throttled(self, message, every_n=40):
        self.log_counter += 1
        if self.log_counter % every_n == 1:
            self.get_logger().info(message)

    def local_position_fresh(self):
        return self.local_position is not None and time.time() - self.local_position_seen_at < 0.5

    def vehicle_status_fresh(self):
        return self.vehicle_status is not None and time.time() - self.vehicle_status_seen_at < 1.0

    def local_position_valid(self):
        if not self.local_position_fresh():
            return False

        pos = self.local_position
        finite = all(math.isfinite(value) for value in (pos.x, pos.y, pos.z, pos.vx, pos.vy, pos.vz))
        flags_ok = (
            getattr(pos, 'xy_valid', False)
            and getattr(pos, 'z_valid', False)
            and getattr(pos, 'v_xy_valid', False)
            and getattr(pos, 'v_z_valid', False)
        )
        return finite and flags_ok

    def local_position_stable(self):
        if not self.local_position_valid():
            return False

        if len(self.stability_samples) < 20:
            return False

        newest = self.stability_samples[-1][0]
        oldest = self.stability_samples[0][0]
        if newest - oldest < STABILITY_REQUIRED_SECONDS * 0.8:
            return False

        xs = [sample[1] for sample in self.stability_samples]
        ys = [sample[2] for sample in self.stability_samples]
        zs = [sample[3] for sample in self.stability_samples]
        drift = max(
            max(xs) - min(xs),
            max(ys) - min(ys),
            max(zs) - min(zs),
        )
        return drift <= STABILITY_MAX_DRIFT_M

    def status_allows_flight(self):
        if not self.vehicle_status_fresh():
            return False

        return not getattr(self.vehicle_status, 'failsafe', False)

    def in_offboard(self):
        return (
            self.vehicle_status_fresh()
            and self.vehicle_status.nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD
        )

    def is_armed(self):
        return (
            self.vehicle_status_fresh()
            and self.vehicle_status.arming_state == VehicleStatus.ARMING_STATE_ARMED
        )

    def capture_home(self):
        pos = self.local_position
        self.home_x = pos.x
        self.home_y = pos.y
        self.home_z = pos.z
        self.takeoff_z = self.home_z - TAKEOFF_HEIGHT_M
        self.descent_final_z = self.home_z - 0.15
        self.get_logger().info(
            f"Home captured: x={self.home_x:.2f}, y={self.home_y:.2f}, z={self.home_z:.2f}"
        )

    def smoothstep(self, elapsed, duration):
        t = min(max(elapsed / duration, 0.0), 1.0)
        return t * t * (3.0 - 2.0 * t)

    def z_between(self, start_z, end_z, elapsed, duration):
        progress = self.smoothstep(elapsed, duration)
        return start_z + progress * (end_z - start_z)

    def publish_home_setpoint(self, z):
        self.publish_setpoint(self.home_x, self.home_y, z)

    def outside_safety_box(self):
        if not self.local_position_valid() or self.home_x is None:
            return True

        pos = self.local_position
        horizontal_error = math.hypot(pos.x - self.home_x, pos.y - self.home_y)
        altitude = self.home_z - pos.z
        return (
            horizontal_error > SAFETY_MAX_HORIZONTAL_ERROR_M
            or altitude > SAFETY_MAX_ALTITUDE_M
        )

    def abort_to_land(self, reason):
        self.get_logger().error(reason)
        self.land()
        self.transition(State.ABORT)

    def timer_cb(self):
        dt = time.time() - self.state_start

        if self.state in (State.PRESTREAM, State.SET_OFFBOARD, State.ARM, State.TAKEOFF, State.HOVER, State.DESCEND):
            self.publish_offboard_mode()

        if self.state in (State.TAKEOFF, State.HOVER, State.DESCEND) and self.outside_safety_box():
            self.abort_to_land("Safety box exceeded or local position became invalid. Commanding land.")
            return

        if self.state == State.WAIT_FOR_POSITION:
            if not self.local_position_valid():
                self.log_throttled("Waiting for valid local position and velocity...")
                return

            if not self.status_allows_flight():
                self.log_throttled("Waiting for fresh vehicle status with no failsafe...")
                return

            if not self.local_position_stable():
                self.log_throttled("Waiting for local position to stabilize...")
                return

            self.capture_home()
            self.transition(State.PRESTREAM)

        elif self.state == State.PRESTREAM:
            self.publish_home_setpoint(self.home_z)
            self.log_throttled("Streaming hold setpoint before offboard...")
            if dt > 2.0:
                self.transition(State.SET_OFFBOARD)

        elif self.state == State.SET_OFFBOARD:
            self.publish_home_setpoint(self.home_z)
            self.set_offboard_mode()
            self.log_throttled("Requesting offboard mode...", every_n=10)
            if self.in_offboard():
                self.transition(State.ARM)
            elif dt > 4.0:
                self.abort_to_land("Offboard mode was not confirmed. Stopping test.")

        elif self.state == State.ARM:
            self.publish_home_setpoint(self.home_z)
            self.arm()
            self.log_throttled("Requesting arm...", every_n=10)
            if self.is_armed():
                self.transition(State.TAKEOFF)
            elif dt > 4.0:
                self.abort_to_land("Armed state was not confirmed. Stopping test.")

        elif self.state == State.TAKEOFF:
            z = self.z_between(self.home_z, self.takeoff_z, dt, TAKEOFF_RAMP_SECONDS)
            self.publish_home_setpoint(z)
            self.log_throttled(f"Smooth takeoff... z={z:.2f} target={self.takeoff_z:.2f}")
            if dt > TAKEOFF_RAMP_SECONDS:
                self.transition(State.HOVER)

        elif self.state == State.HOVER:
            self.publish_home_setpoint(self.takeoff_z)
            self.log_throttled(f"Hovering at home... {dt:.1f}s / 5.0s")
            if dt > 5.0:
                self.transition(State.DESCEND)

        elif self.state == State.DESCEND:
            z = self.z_between(self.takeoff_z, self.descent_final_z, dt, SOFT_LAND_DESCENT_SECONDS)
            self.publish_home_setpoint(z)
            self.log_throttled(f"Smooth descending... z={z:.2f}")
            if dt > SOFT_LAND_DESCENT_SECONDS:
                self.transition(State.LAND)

        elif self.state == State.LAND:
            self.land()
            self.log_throttled("Final PX4 land command...", every_n=10)
            if dt > 4.0:
                self.get_logger().info("=== Test Complete ===")
                raise SystemExit

        elif self.state == State.ABORT:
            self.land()
            self.log_throttled("Abort landing in progress...", every_n=10)
            if dt > 6.0:
                raise SystemExit


def main():
    rclpy.init()
    node = TestTakeoffLand()
    try:
        rclpy.spin(node)
    except SystemExit:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
