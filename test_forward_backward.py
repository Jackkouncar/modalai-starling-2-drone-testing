#!/usr/bin/env python3
"""
 Test Script: Forward and Backward Transit
- Takes off to 1m
- Flies forward 1.5m (positive X in NED) using smoothstep interpolation
- Hovers 3s
- Flies backward to origin using smoothstep interpolation
- Hovers 3s
- Lands

All horizontal motion is interpolated with smooth_transit_xy() from
flight_config.py.  The drone is given a gradually advancing position
setpoint rather than a sudden jump to the target, so it accelerates
gently at the start of each leg and decelerates to a near-stop before
the hover phase.
"""

import rclpy
from rclpy.node import Node
import time
from enum import Enum

from flight_config import (
    HOVER_SETTLE_S,
    LAND_COMMAND_SECONDS,
    SOFT_LAND_DESCENT_SECONDS,
    TARGET_DRONE,
    TAKEOFF_Z_NED,
    TRANSIT_DISTANCE_M,
    TRANSIT_DURATION_S,
    log_environment_check,
    smooth_transit_xy,
    soft_landing_z,
)
from px4_msgs.msg import (
    OffboardControlMode,
    TrajectorySetpoint,
    VehicleCommand
)


class State(Enum):
    INIT = 0
    TAKEOFF = 1
    HOVER_PRE = 2
    FORWARD = 3
    HOVER_FWD = 4
    BACKWARD = 5
    HOVER_BACK = 6
    LAND = 7


class TestForwardBackward(Node):

    def __init__(self):
        super().__init__('test_forward_backward')

        self.offboard_pub = self.create_publisher(
            OffboardControlMode, '/fmu/in/offboard_control_mode', 10)
        self.traj_pub = self.create_publisher(
            TrajectorySetpoint, '/fmu/in/trajectory_setpoint', 10)
        self.cmd_pub = self.create_publisher(
            VehicleCommand, '/fmu/in/vehicle_command', 10)

        self.timer = self.create_timer(0.05, self.timer_cb)  # 20 Hz

        self.takeoff_z = TAKEOFF_Z_NED
        self.d = TRANSIT_DISTANCE_M

        self.state = State.INIT
        self.state_start = time.time()
        self.log_counter = 0

        self.get_logger().info(f"=== {TARGET_DRONE} Test: Forward / Backward Transit ===")
        log_environment_check(self)

    def now_us(self):
        return self.get_clock().now().nanoseconds // 1000

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

    def timer_cb(self):
        self.publish_offboard_mode()
        dt = time.time() - self.state_start

        if self.state == State.INIT:
            self.publish_setpoint(0.0, 0.0, self.takeoff_z)
            if dt > 1.5:
                self.set_offboard_mode()
                self.arm()
                self.transition(State.TAKEOFF)

        elif self.state == State.TAKEOFF:
            self.publish_setpoint(0.0, 0.0, self.takeoff_z)
            self.log_throttled(f"Taking off... {dt:.1f}s")
            if dt > 4.0:
                self.transition(State.HOVER_PRE)

        elif self.state == State.HOVER_PRE:
            self.publish_setpoint(0.0, 0.0, self.takeoff_z)
            self.log_throttled(f"Stabilizing... {dt:.1f}s")
            if dt > HOVER_SETTLE_S:
                self.transition(State.FORWARD)

        elif self.state == State.FORWARD:
            # Smoothly interpolate from origin to (d, 0) over TRANSIT_DURATION_S
            x, y = smooth_transit_xy(dt, 0.0, 0.0, self.d, 0.0)
            self.publish_setpoint(x, y, self.takeoff_z)
            self.log_throttled(f"Moving forward -> ({x:.2f}, {y:.2f})  {dt:.1f}s")
            if dt > TRANSIT_DURATION_S:
                self.transition(State.HOVER_FWD)

        elif self.state == State.HOVER_FWD:
            self.publish_setpoint(self.d, 0.0, self.takeoff_z)
            self.log_throttled(f"Holding forward position... {dt:.1f}s")
            if dt > HOVER_SETTLE_S:
                self.transition(State.BACKWARD)

        elif self.state == State.BACKWARD:
            # Smoothly interpolate from (d, 0) back to origin
            x, y = smooth_transit_xy(dt, self.d, 0.0, 0.0, 0.0)
            self.publish_setpoint(x, y, self.takeoff_z)
            self.log_throttled(f"Moving backward -> ({x:.2f}, {y:.2f})  {dt:.1f}s")
            if dt > TRANSIT_DURATION_S:
                self.transition(State.HOVER_BACK)

        elif self.state == State.HOVER_BACK:
            self.publish_setpoint(0.0, 0.0, self.takeoff_z)
            self.log_throttled(f"Holding origin... {dt:.1f}s")
            if dt > HOVER_SETTLE_S:
                self.transition(State.LAND)

        elif self.state == State.LAND:
            self.publish_setpoint(0.0, 0.0, soft_landing_z(dt))
            if dt < SOFT_LAND_DESCENT_SECONDS:
                self.log_throttled(f"Soft descending... {dt:.1f}s")
                return

            self.land()
            self.log_throttled("Final landing command...")
            if dt > SOFT_LAND_DESCENT_SECONDS + LAND_COMMAND_SECONDS:
                self.get_logger().info("=== Test Complete ===")
                raise SystemExit


def main():
    rclpy.init()
    node = TestForwardBackward()
    try:
        rclpy.spin(node)
    except SystemExit:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()