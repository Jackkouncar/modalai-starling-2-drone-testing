#!/usr/bin/env python3
"""Shared guarded PX4 offboard mission runner for simple indoor tests."""

import math
import time
from enum import Enum

import rclpy
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)

from flight_config import (
    HOVER_SETTLE_S,
    LAND_COMMAND_SECONDS,
    SAFETY_MAX_ALTITUDE_M,
    SAFETY_VIOLATION_GRACE_SECONDS,
    SOFT_LAND_DESCENT_SECONDS,
    STABILITY_MAX_DRIFT_M,
    STABILITY_REQUIRED_SECONDS,
    TAKEOFF_HEIGHT_M,
    TAKEOFF_RAMP_SECONDS,
    TRANSIT_DURATION_S,
    log_environment_check,
    running_lab_simulation,
    safety_max_horizontal_error,
    smoothstep,
    smooth_transit_xy,
    skip_vehicle_status_gate,
)
from px4_msgs.msg import (
    OffboardControlMode,
    TrajectorySetpoint,
    VehicleCommand,
    VehicleLocalPosition,
    VehicleStatus,
)


PX4_QOS = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
)

LOCAL_POSITION_TOPICS = (
    '/fmu/out/vehicle_local_position',
    '/fmu/out/vehicle_local_position_v1',
)
VEHICLE_STATUS_TOPICS = (
    '/fmu/out/vehicle_status',
    '/fmu/out/vehicle_status_v1',
)


class MissionState(Enum):
    WAIT_FOR_POSITION = 0
    PRESTREAM = 1
    SET_OFFBOARD = 2
    ARM = 3
    TAKEOFF = 4
    HOVER_BEFORE_MISSION = 5
    TRANSIT = 6
    HOVER_AT_WAYPOINT = 7
    DESCEND = 8
    LAND = 9
    ABORT = 10


class GuardedMission(Node):
    """Run a home-relative waypoint mission with PX4 feedback checks."""

    def __init__(self, node_name, title, waypoints):
        super().__init__(node_name)

        self.title = title
        self.waypoints = waypoints
        self.waypoint_index = 0
        self.segment_start_x = 0.0
        self.segment_start_y = 0.0
        self.commanded_x = None
        self.commanded_y = None
        self.commanded_z = None

        self.offboard_pub = self.create_publisher(
            OffboardControlMode, '/fmu/in/offboard_control_mode', 10)
        self.traj_pub = self.create_publisher(
            TrajectorySetpoint, '/fmu/in/trajectory_setpoint', 10)
        self.cmd_pub = self.create_publisher(
            VehicleCommand, '/fmu/in/vehicle_command', 10)

        self.local_pos_subs = [
            self.create_subscription(
                VehicleLocalPosition, topic, self.local_position_cb, PX4_QOS)
            for topic in LOCAL_POSITION_TOPICS
        ]
        self.status_subs = [
            self.create_subscription(
                VehicleStatus, topic, self.vehicle_status_cb, PX4_QOS)
            for topic in VEHICLE_STATUS_TOPICS
        ]

        self.timer = self.create_timer(0.05, self.timer_cb)

        self.state = MissionState.WAIT_FOR_POSITION
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
        self.safety_violation_started_at = None

        self.get_logger().info(f"=== {self.title} ===")
        self.get_logger().info(
            f"Listening for PX4 local position on: {', '.join(LOCAL_POSITION_TOPICS)}"
        )
        self.get_logger().info(
            f"Safety limits: {safety_max_horizontal_error():.2f} m setpoint error, "
            f"{SAFETY_MAX_ALTITUDE_M:.2f} m altitude"
        )
        if running_lab_simulation():
            self.get_logger().info("Lab simulation mode detected; using wider Baylands drift allowance.")
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

    def log_throttled(self, message, every_n=40):
        self.log_counter += 1
        if self.log_counter % every_n == 1:
            self.get_logger().info(message)

    def publish_offboard_mode(self):
        msg = OffboardControlMode()
        msg.position = True
        msg.velocity = False
        msg.acceleration = False
        msg.attitude = False
        msg.body_rate = False
        msg.timestamp = self.now_us()
        self.offboard_pub.publish(msg)

    def publish_setpoint(self, rel_x, rel_y, z, yaw=0.0):
        x = self.home_x + rel_x
        y = self.home_y + rel_y
        msg = TrajectorySetpoint()
        msg.position = [x, y, z]
        msg.yaw = yaw
        msg.timestamp = self.now_us()
        self.traj_pub.publish(msg)
        self.commanded_x = x
        self.commanded_y = y
        self.commanded_z = z

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

    def local_position_fresh(self):
        return self.local_position is not None and time.time() - self.local_position_seen_at < 0.5

    def vehicle_status_fresh(self):
        return self.vehicle_status is not None and time.time() - self.vehicle_status_seen_at < 3.0

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
        if not self.local_position_valid() or len(self.stability_samples) < 20:
            return False

        newest = self.stability_samples[-1][0]
        oldest = self.stability_samples[0][0]
        if newest - oldest < STABILITY_REQUIRED_SECONDS * 0.8:
            return False

        xs = [sample[1] for sample in self.stability_samples]
        ys = [sample[2] for sample in self.stability_samples]
        zs = [sample[3] for sample in self.stability_samples]
        drift = max(max(xs) - min(xs), max(ys) - min(ys), max(zs) - min(zs))
        return drift <= STABILITY_MAX_DRIFT_M

    def status_allows_flight(self):
        if skip_vehicle_status_gate() and running_lab_simulation():
            return True

        if not self.vehicle_status_fresh():
            return False

        return (
            not getattr(self.vehicle_status, 'failsafe', False)
            and getattr(self.vehicle_status, 'pre_flight_checks_pass', True)
        )

    def vehicle_status_block_reason(self):
        if not self.vehicle_status_fresh():
            return "waiting for fresh vehicle status"

        failsafe = getattr(self.vehicle_status, 'failsafe', False)
        preflight = getattr(self.vehicle_status, 'pre_flight_checks_pass', True)

        if failsafe:
            return f"PX4 reports failsafe=true, preflight={preflight}"

        if not preflight:
            return "PX4 preflight checks are not passing"

        return "vehicle status is ready"

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

    def z_between(self, start_z, end_z, elapsed, duration):
        progress = smoothstep(elapsed, duration)
        return start_z + progress * (end_z - start_z)

    def safety_violation(self):
        if not self.local_position_fresh():
            return "local position is stale or missing"

        if not self.local_position_valid():
            pos = self.local_position
            return (
                "local position is invalid "
                f"(xy={getattr(pos, 'xy_valid', None)}, z={getattr(pos, 'z_valid', None)}, "
                f"vxy={getattr(pos, 'v_xy_valid', None)}, vz={getattr(pos, 'v_z_valid', None)})"
            )

        pos = self.local_position
        altitude = self.home_z - pos.z
        if altitude > SAFETY_MAX_ALTITUDE_M:
            return f"altitude {altitude:.2f} m exceeded {SAFETY_MAX_ALTITUDE_M:.2f} m"

        if self.commanded_x is not None:
            setpoint_error = math.hypot(pos.x - self.commanded_x, pos.y - self.commanded_y)
            max_error = safety_max_horizontal_error()
            if setpoint_error > max_error:
                return f"setpoint error {setpoint_error:.2f} m exceeded {max_error:.2f} m"

        return None

    def check_safety(self):
        reason = self.safety_violation()
        if reason is None:
            self.safety_violation_started_at = None
            return True

        if self.safety_violation_started_at is None:
            self.safety_violation_started_at = time.time()
            self.get_logger().warn(f"Safety warning: {reason}")
            return True

        if time.time() - self.safety_violation_started_at > SAFETY_VIOLATION_GRACE_SECONDS:
            self.abort_to_land(f"Safety box exceeded: {reason}. Commanding land.")
            return False

        return True

    def abort_to_land(self, reason):
        self.get_logger().error(reason)
        self.land()
        self.transition(MissionState.ABORT)

    def active_waypoint(self):
        return self.waypoints[self.waypoint_index]

    def timer_cb(self):
        dt = time.time() - self.state_start

        if self.state in (
            MissionState.PRESTREAM,
            MissionState.SET_OFFBOARD,
            MissionState.ARM,
            MissionState.TAKEOFF,
            MissionState.HOVER_BEFORE_MISSION,
            MissionState.TRANSIT,
            MissionState.HOVER_AT_WAYPOINT,
            MissionState.DESCEND,
        ):
            self.publish_offboard_mode()

        if self.state in (
            MissionState.TAKEOFF,
            MissionState.HOVER_BEFORE_MISSION,
            MissionState.TRANSIT,
            MissionState.HOVER_AT_WAYPOINT,
            MissionState.DESCEND,
        ) and not self.check_safety():
            return

        if self.state == MissionState.WAIT_FOR_POSITION:
            if not self.local_position_valid():
                self.log_throttled("Waiting for valid local position and velocity...")
                return
            if not self.status_allows_flight():
                self.log_throttled(
                    f"Waiting for vehicle status: {self.vehicle_status_block_reason()}..."
                )
                return
            if not self.local_position_stable():
                self.log_throttled("Waiting for local position to stabilize...")
                return
            self.capture_home()
            self.transition(MissionState.PRESTREAM)

        elif self.state == MissionState.PRESTREAM:
            self.publish_setpoint(0.0, 0.0, self.home_z)
            self.log_throttled("Streaming hold setpoint before offboard...")
            if dt > 2.0:
                self.transition(MissionState.SET_OFFBOARD)

        elif self.state == MissionState.SET_OFFBOARD:
            self.publish_setpoint(0.0, 0.0, self.home_z)
            self.set_offboard_mode()
            self.log_throttled("Requesting offboard mode...", every_n=10)
            if self.in_offboard():
                self.transition(MissionState.ARM)
            elif skip_vehicle_status_gate() and running_lab_simulation() and dt > 2.0:
                self.get_logger().warn("Sim-only mode: continuing without Offboard status confirmation.")
                self.transition(MissionState.ARM)
            elif dt > 4.0:
                self.abort_to_land("Offboard mode was not confirmed. Stopping test.")

        elif self.state == MissionState.ARM:
            self.publish_setpoint(0.0, 0.0, self.home_z)
            self.arm()
            self.log_throttled("Requesting arm...", every_n=10)
            if self.is_armed():
                self.transition(MissionState.TAKEOFF)
            elif skip_vehicle_status_gate() and running_lab_simulation() and dt > 2.0:
                self.get_logger().warn("Sim-only mode: continuing without armed status confirmation.")
                self.transition(MissionState.TAKEOFF)
            elif dt > 4.0:
                self.abort_to_land("Armed state was not confirmed. Stopping test.")

        elif self.state == MissionState.TAKEOFF:
            z = self.z_between(self.home_z, self.takeoff_z, dt, TAKEOFF_RAMP_SECONDS)
            self.publish_setpoint(0.0, 0.0, z)
            self.log_throttled(f"Smooth takeoff... z={z:.2f} target={self.takeoff_z:.2f}")
            if dt > TAKEOFF_RAMP_SECONDS:
                self.transition(MissionState.HOVER_BEFORE_MISSION)

        elif self.state == MissionState.HOVER_BEFORE_MISSION:
            self.publish_setpoint(self.segment_start_x, self.segment_start_y, self.takeoff_z)
            self.log_throttled(f"Stabilizing before mission... {dt:.1f}s")
            if dt > HOVER_SETTLE_S:
                self.transition(MissionState.TRANSIT)

        elif self.state == MissionState.TRANSIT:
            name, target_x, target_y = self.active_waypoint()
            x, y = smooth_transit_xy(
                dt, self.segment_start_x, self.segment_start_y, target_x, target_y)
            self.publish_setpoint(x, y, self.takeoff_z)
            self.log_throttled(f"{name} -> ({x:.2f}, {y:.2f}) {dt:.1f}s")
            if dt > TRANSIT_DURATION_S:
                self.segment_start_x = target_x
                self.segment_start_y = target_y
                self.transition(MissionState.HOVER_AT_WAYPOINT)

        elif self.state == MissionState.HOVER_AT_WAYPOINT:
            name, target_x, target_y = self.active_waypoint()
            self.publish_setpoint(target_x, target_y, self.takeoff_z)
            self.log_throttled(f"Holding {name}... {dt:.1f}s")
            if dt > HOVER_SETTLE_S:
                self.waypoint_index += 1
                if self.waypoint_index >= len(self.waypoints):
                    self.transition(MissionState.DESCEND)
                else:
                    self.transition(MissionState.TRANSIT)

        elif self.state == MissionState.DESCEND:
            z = self.z_between(self.takeoff_z, self.descent_final_z, dt, SOFT_LAND_DESCENT_SECONDS)
            self.publish_setpoint(self.segment_start_x, self.segment_start_y, z)
            self.log_throttled(f"Smooth descending... z={z:.2f}")
            if dt > SOFT_LAND_DESCENT_SECONDS:
                self.transition(MissionState.LAND)

        elif self.state == MissionState.LAND:
            self.land()
            self.log_throttled("Final PX4 land command...", every_n=10)
            if dt > LAND_COMMAND_SECONDS:
                self.get_logger().info("=== Test Complete ===")
                raise SystemExit

        elif self.state == MissionState.ABORT:
            self.land()
            self.log_throttled("Abort landing in progress...", every_n=10)
            if dt > 6.0:
                raise SystemExit


def run_guarded_mission(node_name, title, waypoints):
    rclpy.init()
    node = GuardedMission(node_name, title, waypoints)
    try:
        rclpy.spin(node)
    except (SystemExit, KeyboardInterrupt):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
