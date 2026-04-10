#!/usr/bin/env python3
"""
Test Script: All-Directions Square Transit
- Takes off to 2m
- Flies forward 3m (+X)
- Flies right 3m (+Y)
- Flies backward 3m (back to X=0)
- Flies left 3m (back to Y=0, origin)
- Lands

Traces a square pattern: origin -> forward -> forward-right -> right -> origin -> land
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
    HOVER_START = 2
    FORWARD = 3
    HOVER_A = 4
    RIGHT = 5
    HOVER_B = 6
    BACKWARD = 7
    HOVER_C = 8
    LEFT = 9
    HOVER_D = 10
    LAND = 11


class TestAllDirections(Node):

    def __init__(self):
        super().__init__('test_all_directions')

        self.offboard_pub = self.create_publisher(
            OffboardControlMode, '/fmu/in/offboard_control_mode', 10)
        self.traj_pub = self.create_publisher(
            TrajectorySetpoint, '/fmu/in/trajectory_setpoint', 10)
        self.cmd_pub = self.create_publisher(
            VehicleCommand, '/fmu/in/vehicle_command', 10)

        self.timer = self.create_timer(0.05, self.timer_cb)

        self.takeoff_z = -2.0
        self.d = 3.0  # transit distance in meters

        self.state = State.INIT
        self.state_start = time.time()
        self.log_counter = 0

        # Waypoints: (x, y) for each leg of the square
        # Forward -> Forward+Right -> Right -> Origin
        self.wp = {
            State.FORWARD:  (self.d, 0.0),
            State.RIGHT:    (self.d, self.d),
            State.BACKWARD: (0.0,   self.d),
            State.LEFT:     (0.0,   0.0),
        }

        self.get_logger().info("=== Test: All-Directions Square ===")

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

    def current_target(self):
        """Return (x, y) for current move state, or origin."""
        return self.wp.get(self.state, (0.0, 0.0))

    def timer_cb(self):
        self.publish_offboard_mode()
        dt = time.time() - self.state_start

        # --- INIT: send setpoints, then arm & offboard ---
        if self.state == State.INIT:
            self.publish_setpoint(0.0, 0.0, self.takeoff_z)
            if dt > 1.5:
                self.set_offboard_mode()
                self.arm()
                self.transition(State.TAKEOFF)

        # --- TAKEOFF ---
        elif self.state == State.TAKEOFF:
            self.publish_setpoint(0.0, 0.0, self.takeoff_z)
            self.log_throttled(f"Taking off... {dt:.1f}s")
            if dt > 4.0:
                self.transition(State.HOVER_START)

        # --- HOVER at start ---
        elif self.state == State.HOVER_START:
            self.publish_setpoint(0.0, 0.0, self.takeoff_z)
            self.log_throttled(f"Stabilizing... {dt:.1f}s")
            if dt > 3.0:
                self.transition(State.FORWARD)

        # --- FORWARD (+X) ---
        elif self.state == State.FORWARD:
            x, y = self.current_target()
            self.publish_setpoint(x, y, self.takeoff_z)
            self.log_throttled(f"Forward to ({x},{y})... {dt:.1f}s")
            if dt > 5.0:
                self.transition(State.HOVER_A)

        elif self.state == State.HOVER_A:
            x, y = self.wp[State.FORWARD]
            self.publish_setpoint(x, y, self.takeoff_z)
            self.log_throttled(f"Holding ({x},{y})... {dt:.1f}s")
            if dt > 2.0:
                self.transition(State.RIGHT)

        # --- RIGHT (+Y) ---
        elif self.state == State.RIGHT:
            x, y = self.current_target()
            self.publish_setpoint(x, y, self.takeoff_z)
            self.log_throttled(f"Right to ({x},{y})... {dt:.1f}s")
            if dt > 5.0:
                self.transition(State.HOVER_B)

        elif self.state == State.HOVER_B:
            x, y = self.wp[State.RIGHT]
            self.publish_setpoint(x, y, self.takeoff_z)
            self.log_throttled(f"Holding ({x},{y})... {dt:.1f}s")
            if dt > 2.0:
                self.transition(State.BACKWARD)

        # --- BACKWARD (-X) ---
        elif self.state == State.BACKWARD:
            x, y = self.current_target()
            self.publish_setpoint(x, y, self.takeoff_z)
            self.log_throttled(f"Backward to ({x},{y})... {dt:.1f}s")
            if dt > 5.0:
                self.transition(State.HOVER_C)

        elif self.state == State.HOVER_C:
            x, y = self.wp[State.BACKWARD]
            self.publish_setpoint(x, y, self.takeoff_z)
            self.log_throttled(f"Holding ({x},{y})... {dt:.1f}s")
            if dt > 2.0:
                self.transition(State.LEFT)

        # --- LEFT (-Y, back to origin) ---
        elif self.state == State.LEFT:
            x, y = self.current_target()
            self.publish_setpoint(x, y, self.takeoff_z)
            self.log_throttled(f"Left to ({x},{y})... {dt:.1f}s")
            if dt > 5.0:
                self.transition(State.HOVER_D)

        elif self.state == State.HOVER_D:
            self.publish_setpoint(0.0, 0.0, self.takeoff_z)
            self.log_throttled(f"Back at origin... {dt:.1f}s")
            if dt > 2.0:
                self.transition(State.LAND)

        # --- LAND ---
        elif self.state == State.LAND:
            self.land()
            self.log_throttled("Landing...")
            if dt > 3.0:
                self.get_logger().info("=== Test Complete: Square pattern done ===")
                raise SystemExit


def main():
    rclpy.init()
    node = TestAllDirections()
    try:
        rclpy.spin(node)
    except SystemExit:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
