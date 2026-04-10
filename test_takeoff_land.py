#!/usr/bin/env python3
"""
Test Script: Takeoff and Land
- Arms drone, enters offboard mode
- Takes off to 2m altitude
- Hovers for 5 seconds
- Lands
"""

import rclpy
from rclpy.node import Node
import time
from enum import Enum

from px4_msgs.msg import (
    OffboardControlMode,
    TrajectorySetpoint,
    VehicleCommand
)


class State(Enum):
    INIT = 0
    TAKEOFF = 1
    HOVER = 2
    LAND = 3


class TestTakeoffLand(Node):

    def __init__(self):
        super().__init__('test_takeoff_land')

        self.offboard_pub = self.create_publisher(
            OffboardControlMode, '/fmu/in/offboard_control_mode', 10)
        self.traj_pub = self.create_publisher(
            TrajectorySetpoint, '/fmu/in/trajectory_setpoint', 10)
        self.cmd_pub = self.create_publisher(
            VehicleCommand, '/fmu/in/vehicle_command', 10)

        self.timer = self.create_timer(0.05, self.timer_cb)  # 20 Hz

        self.takeoff_z = -2.0  # NED: negative = up
        self.state = State.INIT
        self.state_start = time.time()
        self.log_counter = 0  # throttle log messages

        self.get_logger().info("=== Test: Takeoff and Land ===")

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
        """Log once every every_n timer ticks (~2s at 20Hz)."""
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
                self.transition(State.HOVER)

        elif self.state == State.HOVER:
            self.publish_setpoint(0.0, 0.0, self.takeoff_z)
            self.log_throttled(f"Hovering... {dt:.1f}s / 5.0s")
            if dt > 5.0:
                self.transition(State.LAND)

        elif self.state == State.LAND:
            self.land()
            self.log_throttled("Landing...")
            if dt > 3.0:
                self.get_logger().info("=== Test Complete ===")
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
