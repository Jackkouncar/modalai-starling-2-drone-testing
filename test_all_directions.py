#!/usr/bin/env python3
"""
 Test Script: All-Directions Square Transit
- Takes off to 1m
- Flies forward 1.5m (+X)
- Flies right 1.5m (+Y)
- Flies backward 1.5m (back to X=0)
- Flies left 1.5m (back to Y=0, origin)
- Lands

Traces a square: origin -> forward -> forward-right -> right -> origin

Each leg uses smooth_transit_xy() from flight_config.py so the drone
accelerates and decelerates gently (smoothstep curve) instead of jumping
instantly to each waypoint.
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


# (x_start, y_start, x_end, y_end) for each movement leg
_LEG = {
    State.FORWARD:  (0.0, 0.0, 1.0, 0.0),   # scaled by d below
    State.RIGHT:    (1.0, 0.0, 1.0, 1.0),
    State.BACKWARD: (1.0, 1.0, 0.0, 1.0),
    State.LEFT:     (0.0, 1.0, 0.0, 0.0),
}

# Waypoint to hold at after each leg
_HOLD = {
    State.HOVER_A: (1.0, 0.0),
    State.HOVER_B: (1.0, 1.0),
    State.HOVER_C: (0.0, 1.0),
    State.HOVER_D: (0.0, 0.0),
}


class TestAllDirections(Node):

    def __init__(self):
        super().__init__('test_all_directions')

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

        self.get_logger().info(f"=== {TARGET_DRONE} Test: All-Directions Square ===")
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

    def _publish_leg(self, dt):
        """
        Compute and publish a smoothly interpolated setpoint for the current
        movement leg.  Leg coordinates in _LEG are unit-scaled; multiply by
        self.d to get real-world metres.
        """
        xs, ys, xe, ye = _LEG[self.state]
        x, y = smooth_transit_xy(
            dt,
            xs * self.d, ys * self.d,
            xe * self.d, ye * self.d,
        )
        self.publish_setpoint(x, y, self.takeoff_z)
        self.log_throttled(f"{self.state.name} -> ({x:.2f}, {y:.2f})  {dt:.1f}s")

    def timer_cb(self):
        self.publish_offboard_mode()
        dt = time.time() - self.state_start

        # --- INIT ---
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
            if dt > HOVER_SETTLE_S:
                self.transition(State.FORWARD)

        # --- Movement legs ---
        elif self.state in _LEG:
            self._publish_leg(dt)
            if dt > TRANSIT_DURATION_S:
                # advance to the corresponding hover state
                next_hover = State(self.state.value + 1)
                self.transition(next_hover)

        # --- Hover / hold states ---
        elif self.state in _HOLD:
            hx, hy = _HOLD[self.state]
            self.publish_setpoint(hx * self.d, hy * self.d, self.takeoff_z)
            self.log_throttled(
                f"Holding ({hx * self.d:.2f}, {hy * self.d:.2f})... {dt:.1f}s"
            )
            if dt > HOVER_SETTLE_S:
                next_move = State(self.state.value + 1)
                self.transition(next_move)

        # --- LAND ---
        elif self.state == State.LAND:
            self.publish_setpoint(0.0, 0.0, soft_landing_z(dt))
            if dt < SOFT_LAND_DESCENT_SECONDS:
                self.log_throttled(f"Soft descending... {dt:.1f}s")
                return

            self.land()
            self.log_throttled("Final landing command...")
            if dt > SOFT_LAND_DESCENT_SECONDS + LAND_COMMAND_SECONDS:
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