#!/usr/bin/env python3
"""Safe PX4 Offboard keyboard control over ROS 2 / Micro XRCE-DDS.

PX4 messages use NED/FRD.  Keyboard translations are body-heading relative:
W/S forward/back, A/D left/right, R/F up/down and Q/E yaw left/right.
"""

import argparse
import math
import os
import select
import sys
import termios
import time
import tty
from dataclasses import dataclass
from typing import Optional

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy

from px4_msgs.msg import (
    ActuatorOutputs,
    OffboardControlMode,
    TrajectorySetpoint,
    VehicleCommand,
    VehicleCommandAck,
    VehicleLocalPosition,
    VehicleOdometry,
    VehicleStatus,
)


def wrap_pi(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


@dataclass
class TargetNed:
    north: float = 0.0
    east: float = 0.0
    down: float = 0.0
    yaw: float = 0.0


class RawTerminal:
    def __init__(self) -> None:
        self.fd: Optional[int] = None
        self.old = None

    def __enter__(self):
        if sys.stdin.isatty():
            self.fd = sys.stdin.fileno()
            self.old = termios.tcgetattr(self.fd)
            tty.setcbreak(self.fd)
        return self

    def read_key(self) -> Optional[str]:
        if select.select([sys.stdin], [], [], 0.0)[0]:
            return os.read(sys.stdin.fileno(), 1).decode(errors="ignore").lower()
        return None

    def __exit__(self, *_args) -> None:
        if self.fd is not None and self.old is not None:
            termios.tcsetattr(self.fd, termios.TCSADRAIN, self.old)


class DdsWasdControl(Node):
    RATE_HZ = 20.0
    STATUS_TIMEOUT_S = 1.0
    PRESTREAM_S = 2.0
    MOVE_STEP_M = 0.35
    ALT_STEP_M = 0.20
    YAW_STEP_RAD = math.radians(12.0)
    TAKEOFF_HEIGHT_M = 1.2

    def __init__(self, arm_only: bool = False) -> None:
        super().__init__("my_drone_dds_wasd_control")
        self.arm_only = arm_only
        self.status: Optional[VehicleStatus] = None
        self.local: Optional[VehicleLocalPosition] = None
        self.odometry: Optional[VehicleOdometry] = None
        self.actuator_outputs: Optional[ActuatorOutputs] = None
        self.last_actuator_monotonic = 0.0
        self.last_status_monotonic = 0.0
        self.last_local_monotonic = 0.0
        self.target = TargetNed()
        self.target_initialized = False
        self.offboard_requested = False
        self.landing_requested = False
        self.prestream_started = 0.0
        self.arm_only_started = 0.0
        self.arm_only_seen_armed = False
        self.arm_only_disarm_sent = False
        self.exit_requested = False
        self.emergency_confirm_until = 0.0
        self.last_state_report = 0.0

        px4_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self.mode_pub = self.create_publisher(
            OffboardControlMode, "/fmu/in/offboard_control_mode", 10
        )
        self.setpoint_pub = self.create_publisher(
            TrajectorySetpoint, "/fmu/in/trajectory_setpoint", 10
        )
        self.command_pub = self.create_publisher(
            VehicleCommand, "/fmu/in/vehicle_command", 10
        )
        self.create_subscription(
            VehicleStatus, "/fmu/out/vehicle_status_v4", self._status_cb, px4_qos
        )
        self.create_subscription(
            VehicleLocalPosition,
            "/fmu/out/vehicle_local_position_v1",
            self._local_cb,
            px4_qos,
        )
        self.create_subscription(
            VehicleOdometry, "/fmu/out/vehicle_odometry", self._odometry_cb, px4_qos
        )
        self.create_subscription(
            VehicleCommandAck,
            "/fmu/out/vehicle_command_ack_v1",
            self._ack_cb,
            px4_qos,
        )
        self.create_subscription(
            ActuatorOutputs,
            "/fmu/out/actuator_outputs",
            self._actuator_outputs_cb,
            px4_qos,
        )
        self.timer = self.create_timer(1.0 / self.RATE_HZ, self._tick)

    def now_us(self) -> int:
        return self.get_clock().now().nanoseconds // 1000

    def _status_cb(self, msg: VehicleStatus) -> None:
        self.status = msg
        self.last_status_monotonic = time.monotonic()

    def _local_cb(self, msg: VehicleLocalPosition) -> None:
        self.local = msg
        self.last_local_monotonic = time.monotonic()
        if not self.target_initialized and msg.xy_valid and msg.z_valid:
            self.target = TargetNed(msg.x, msg.y, msg.z, msg.heading)
            self.target_initialized = True
            self.get_logger().info(
                f"NED target initialized: N={msg.x:.3f} E={msg.y:.3f} "
                f"D={msg.z:.3f} yaw={math.degrees(msg.heading):.1f} deg"
            )

    def _odometry_cb(self, msg: VehicleOdometry) -> None:
        self.odometry = msg

    def _ack_cb(self, msg: VehicleCommandAck) -> None:
        self.get_logger().info(f"PX4 command ack: command={msg.command} result={msg.result}")

    def _actuator_outputs_cb(self, msg: ActuatorOutputs) -> None:
        self.actuator_outputs = msg
        self.last_actuator_monotonic = time.monotonic()

    def state_fresh(self) -> bool:
        now = time.monotonic()
        return (
            self.status is not None
            and self.local is not None
            and now - self.last_status_monotonic < self.STATUS_TIMEOUT_S
            and now - self.last_local_monotonic < self.STATUS_TIMEOUT_S
            and self.local.xy_valid
            and self.local.z_valid
        )

    def publish_command(self, command: int, **params: float) -> None:
        msg = VehicleCommand()
        msg.timestamp = self.now_us()
        for index in range(1, 8):
            setattr(msg, f"param{index}", float(params.get(f"param{index}", 0.0)))
        msg.command = int(command)
        msg.target_system = 1
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.confirmation = 0
        msg.from_external = True
        self.command_pub.publish(msg)

    def publish_hold(self) -> None:
        mode = OffboardControlMode()
        mode.timestamp = self.now_us()
        mode.position = True
        self.mode_pub.publish(mode)

        sp = TrajectorySetpoint()
        sp.timestamp = mode.timestamp
        nan = float("nan")
        sp.position = [self.target.north, self.target.east, self.target.down]
        sp.velocity = [nan, nan, nan]
        sp.acceleration = [nan, nan, nan]
        sp.jerk = [nan, nan, nan]
        sp.yaw = self.target.yaw
        sp.yawspeed = nan
        self.setpoint_pub.publish(sp)

    def begin_takeoff(self) -> bool:
        if not self.state_fresh() or not self.target_initialized:
            self.get_logger().error("Cannot start: PX4 state/position is absent or stale")
            return False
        if self.status.failsafe or not self.status.pre_flight_checks_pass:
            self.get_logger().error("Cannot start: PX4 preflight checks/failsafe state is unsafe")
            return False
        self.target = TargetNed(
            self.local.x,
            self.local.y,
            self.local.z - self.TAKEOFF_HEIGHT_M,
            self.local.heading,
        )
        self.prestream_started = time.monotonic()
        self.offboard_requested = True
        self.get_logger().info("Prestreaming Offboard setpoints for 2 seconds")
        return True

    def begin_arm_only(self) -> bool:
        if not self.state_fresh() or self.status.failsafe or not self.status.pre_flight_checks_pass:
            self.get_logger().error("Arm-only rejected: PX4 state is not safe/fresh")
            return False
        self.publish_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=1.0)
        self.arm_only_started = time.monotonic()
        self.get_logger().info("Arm-only command sent; automatic disarm in 3 seconds")
        return True

    def land(self) -> None:
        if self.landing_requested:
            return
        self.publish_command(VehicleCommand.VEHICLE_CMD_NAV_LAND)
        self.landing_requested = True
        self.offboard_requested = False
        self.get_logger().warning("LAND requested; Offboard setpoint stream stopped")

    def exit_offboard(self) -> None:
        self.publish_command(
            VehicleCommand.VEHICLE_CMD_DO_SET_MODE, param1=1.0, param2=3.0
        )
        self.offboard_requested = False
        self.get_logger().warning("Requested POSCTL and stopped Offboard stream")

    def emergency_disarm(self) -> None:
        self.publish_command(
            VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM,
            param1=0.0,
            param2=21196.0,
        )
        self.offboard_requested = False
        self.get_logger().error("EMERGENCY FORCE DISARM sent")

    def move_key(self, key: str) -> None:
        if not self.target_initialized:
            return
        yaw = self.target.yaw
        forward_n, forward_e = math.cos(yaw), math.sin(yaw)
        right_n, right_e = -math.sin(yaw), math.cos(yaw)
        if key == "w":
            self.target.north += self.MOVE_STEP_M * forward_n
            self.target.east += self.MOVE_STEP_M * forward_e
        elif key == "s":
            self.target.north -= self.MOVE_STEP_M * forward_n
            self.target.east -= self.MOVE_STEP_M * forward_e
        elif key == "d":
            self.target.north += self.MOVE_STEP_M * right_n
            self.target.east += self.MOVE_STEP_M * right_e
        elif key == "a":
            self.target.north -= self.MOVE_STEP_M * right_n
            self.target.east -= self.MOVE_STEP_M * right_e
        elif key == "r":
            self.target.down -= self.ALT_STEP_M
        elif key == "f":
            self.target.down += self.ALT_STEP_M
        elif key == "q":
            self.target.yaw = wrap_pi(self.target.yaw - self.YAW_STEP_RAD)
        elif key == "e":
            self.target.yaw = wrap_pi(self.target.yaw + self.YAW_STEP_RAD)
        else:
            return
        self.get_logger().info(
            f"target NED=({self.target.north:.2f}, {self.target.east:.2f}, "
            f"{self.target.down:.2f}) yaw={math.degrees(self.target.yaw):.1f} deg"
        )

    def handle_key(self, key: str) -> None:
        if key == "t":
            self.begin_takeoff()
        elif key in "wasdrfqe":
            if self.status and self.status.nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD:
                self.move_key(key)
            else:
                self.get_logger().warning("Movement ignored: PX4 is not in Offboard")
        elif key == "l":
            self.land()
        elif key == "o":
            self.exit_offboard()
        elif key == "x":
            now = time.monotonic()
            if now <= self.emergency_confirm_until:
                self.emergency_disarm()
                self.emergency_confirm_until = 0.0
            else:
                self.emergency_confirm_until = now + 2.0
                self.get_logger().warning("Press X again within 2 seconds to FORCE DISARM")
        elif key == "z":
            if self.status and self.status.arming_state == VehicleStatus.ARMING_STATE_ARMED:
                self.land()
            else:
                self.exit_requested = True

    def _tick(self) -> None:
        now = time.monotonic()
        if self.status and self.local and now - self.last_state_report >= 1.0:
            self.last_state_report = now
            self.get_logger().info(
                "STATE "
                f"arm={self.status.arming_state} nav={self.status.nav_state} "
                f"NED=({self.local.x:.3f},{self.local.y:.3f},{self.local.z:.3f}) "
                f"vel=({self.local.vx:.3f},{self.local.vy:.3f},{self.local.vz:.3f}) "
                f"yaw_deg={math.degrees(self.local.heading):.1f} "
                f"failsafe={self.status.failsafe} "
                f"age_s=({now - self.last_status_monotonic:.2f},"
                f"{now - self.last_local_monotonic:.2f}) "
                f"motors={self._motor_report(now)}"
            )
        if (
            self.landing_requested
            and self.status
            and self.status.arming_state == VehicleStatus.ARMING_STATE_DISARMED
        ):
            self.publish_command(
                VehicleCommand.VEHICLE_CMD_DO_SET_MODE, param1=1.0, param2=3.0
            )
            self.get_logger().info("LANDING_DISARMED_CONFIRMED")
            self.exit_requested = True
            return

        if self.arm_only_started:
            if (
                self.status
                and self.status.arming_state == VehicleStatus.ARMING_STATE_ARMED
                and not self.arm_only_seen_armed
            ):
                self.arm_only_seen_armed = True
                self.get_logger().info("ARM_ONLY_ARMED_CONFIRMED")
            if now - self.arm_only_started >= 3.0 and not self.arm_only_disarm_sent:
                self.publish_command(
                    VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=0.0
                )
                self.arm_only_disarm_sent = True
                self.get_logger().info("Arm-only DISARM command sent")
            if (
                self.arm_only_disarm_sent
                and self.status
                and self.status.arming_state == VehicleStatus.ARMING_STATE_DISARMED
            ):
                self.get_logger().info("ARM_ONLY_DISARMED_CONFIRMED")
                self.exit_requested = True
            return

        if not self.offboard_requested:
            return
        if not self.state_fresh():
            self.get_logger().error("PX4 status timeout: stopping Offboard stream")
            self.offboard_requested = False
            return

        self.publish_hold()
        if self.prestream_started and now - self.prestream_started >= self.PRESTREAM_S:
            self.publish_command(
                VehicleCommand.VEHICLE_CMD_DO_SET_MODE, param1=1.0, param2=6.0
            )
            self.publish_command(
                VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=1.0
            )
            self.prestream_started = 0.0
            self.get_logger().info("OFFBOARD mode and ARM commands sent")

    def _motor_report(self, now: float) -> str:
        if self.actuator_outputs is None:
            return "absent"
        age = now - self.last_actuator_monotonic
        if age >= self.STATUS_TIMEOUT_S:
            return f"stale({age:.2f}s)"
        count = min(int(self.actuator_outputs.noutputs), 8)
        values = self.actuator_outputs.output[:count]
        return "[" + ",".join(f"{value:.2f}" for value in values) + "]"


def parse_args(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--arm-only",
        action="store_true",
        help="arm through DDS, wait 3 seconds, disarm and exit without Offboard/takeoff",
    )
    return parser.parse_args(argv)


def main(args=None) -> None:
    cli = parse_args(args)
    rclpy.init(args=None)
    node = DdsWasdControl(arm_only=cli.arm_only)
    print(
        "DDS WASD: T takeoff | W/S/A/D move | R/F up/down | Q/E yaw | "
        "L land | O exit Offboard | X twice emergency disarm | Z safe exit",
        flush=True,
    )
    try:
        with RawTerminal() as terminal:
            start = time.monotonic()
            arm_only_started = False
            while rclpy.ok() and not node.exit_requested:
                rclpy.spin_once(node, timeout_sec=0.02)
                if cli.arm_only and not arm_only_started and node.state_fresh():
                    arm_only_started = node.begin_arm_only()
                key = terminal.read_key()
                if key:
                    node.handle_key(key)
                if cli.arm_only and time.monotonic() - start > 15.0:
                    raise RuntimeError("arm-only test timed out")
    except KeyboardInterrupt:
        if node.status and node.status.arming_state == VehicleStatus.ARMING_STATE_ARMED:
            node.land()
            end = time.monotonic() + 1.0
            while time.monotonic() < end:
                rclpy.spin_once(node, timeout_sec=0.05)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
