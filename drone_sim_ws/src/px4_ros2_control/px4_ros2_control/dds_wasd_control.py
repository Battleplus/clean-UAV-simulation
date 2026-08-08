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

import numpy as np
import rclpy
from geometry_msgs.msg import AccelStamped, WrenchStamped
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

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
from std_msgs.msg import Bool, Float32MultiArray


def wrap_pi(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


def reaction_torque_safety_triggered(
    torque_body_nm: np.ndarray,
    age_s: float,
    limit_nm: float,
    armed: bool,
    freshness_s: float = 0.6,
) -> bool:
    """Return whether a fresh arm torque sample should trigger LAND."""
    values = np.asarray(torque_body_nm, dtype=float)
    if values.shape != (3,) or not np.all(np.isfinite(values)):
        return False
    return bool(
        armed
        and float(limit_nm) > 0.0
        and 0.0 <= float(age_s) < float(freshness_s)
        and float(np.linalg.norm(values)) > float(limit_nm)
    )


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
    # Small, repeatable increments leave control allocation headroom for the
    # canted X8.  Holding a key repeats the same increment, so WASD remains
    # continuous while a single key press cannot demand a near-saturation jump.
    # Identification-safe increments for the canted CAD X8.  A single
    # command must not demand a large lateral attitude transient while the
    # PX4 allocator and thrust margin are still being calibrated.
    MOVE_STEP_M = 0.08
    ALT_STEP_M = 0.12
    YAW_STEP_RAD = math.radians(3.0)
    # Keep the formal/default flight at 1.2 m.  Diagnostics can request a
    # lower hold height while tuning a near-limit thrust configuration without
    # changing the documented baseline.
    TAKEOFF_HEIGHT_M = float(os.environ.get("PX4_TAKEOFF_HEIGHT_M", "1.2"))
    MAX_POSITION_ERROR_M = 3.0
    MAX_VERTICAL_ERROR_M = 2.0
    OFFBOARD_LAND_HANDOFF_M = float(
        os.environ.get("PX4_OFFBOARD_LAND_HANDOFF_M", "0.18")
    )
    OFFBOARD_LAND_MIN_DURATION_S = float(
        os.environ.get("PX4_OFFBOARD_LAND_MIN_DURATION_S", "10.0")
    )
    OFFBOARD_LAND_MAX_VERTICAL_SPEED_M_S = float(
        os.environ.get("PX4_OFFBOARD_LAND_MAX_VERTICAL_SPEED_M_S", "0.25")
    )

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
        self.pending_takeoff = False
        self.offboard_requested = False
        self.landing_requested = False
        self.offboard_landing_active = False
        self.offboard_landing_started = 0.0
        self.takeoff_ground_down: Optional[float] = None
        self.prestream_started = 0.0
        self.arm_only_started = 0.0
        self.arm_only_seen_armed = False
        self.arm_only_disarm_sent = False
        self.exit_requested = False
        self.emergency_confirm_until = 0.0
        self.last_state_report = 0.0
        self.arm_feedforward_enabled = os.environ.get(
            "ARM_FEEDFORWARD_ENABLED", "false"
        ).lower() in {"1", "true", "yes", "on"}
        self.arm_feedforward_ned = [0.0, 0.0, 0.0]
        self.last_arm_feedforward_monotonic = 0.0
        self.arm_reaction_torque_body_nm = np.zeros(3)
        self.last_arm_reaction_monotonic = 0.0
        # The coupling monitor also publishes an explicit motion window.  A
        # freshly spawned arm can report a large numerical acceleration while
        # its gravity-loaded controller is settling; that startup transient
        # must not be classified as an airborne work-motion disturbance.
        self.arm_motion_active = False
        self.last_arm_motion_monotonic = 0.0
        self.arm_torque_abort_nm = float(os.environ.get("ARM_TORQUE_ABORT_NM", "0.5"))
        self.rl_joint_names = (
            "shoulder_pan", "shoulder_lift", "elbow_flex",
            "wrist_flex", "wrist_roll", "gripper",
        )
        self.rl_joint_lower = np.array([-1.91986, -1.74533, -1.69, -1.65806, -2.74385, -0.174533])
        self.rl_joint_upper = np.array([1.91986, 1.74533, 1.69, 1.65806, 2.84121, 1.74533])
        self.rl_joint_positions = np.zeros(6)
        self.rl_joint_velocities = np.zeros(6)
        # PX4 normally publishes VehicleLocalPosition.z in NED (down
        # positive).  Some Gazebo/PX4 bridge combinations used by this CAD
        # world expose the local vertical axis with the opposite sign.  Keep
        # the convention explicit and selectable for bring-up; horizontal
        # N/E and yaw remain unchanged.
        self.local_z_sign = 1.0 if os.environ.get(
            "PX4_LOCAL_Z_SIGN", "-1"
        ).strip().lower() in {"1", "+1", "up", "enu"} else -1.0

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
        self.landing_pub = self.create_publisher(
            Bool, "/my_drone/landing_requested", 10
        )
        self.rl_observation_pub = self.create_publisher(
            Float32MultiArray, "/my_drone/rl_observation", 10
        )
        self.rl_status_pub = self.create_publisher(
            Float32MultiArray, "/my_drone/rl_status", 10
        )
        self.rl_arm_pub = self.create_publisher(
            JointTrajectory, "/arm_controller/joint_trajectory", 10
        )
        self.rl_arm_motion_pub = self.create_publisher(
            Bool, "/my_drone/arm_motion_active", 10
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
        self.create_subscription(
            JointState, "/joint_states", self._joint_state_cb, 10
        )
        self.create_subscription(
            Float32MultiArray, "/my_drone/rl_action", self._rl_action_cb, 10
        )
        self.create_subscription(
            AccelStamped,
            "/my_drone/arm_feedforward_acceleration_ned",
            self._arm_feedforward_cb,
            10,
        )
        self.create_subscription(
            WrenchStamped,
            "/my_drone/arm_reaction_wrench_body",
            self._arm_reaction_wrench_cb,
            10,
        )
        self.create_subscription(
            Bool,
            "/my_drone/arm_motion_active",
            self._arm_motion_active_cb,
            10,
        )
        self.timer = self.create_timer(1.0 / self.RATE_HZ, self._tick)
        if self.arm_feedforward_enabled:
            self.get_logger().info(
                "ARM_FEEDFORWARD enabled; reading bounded NED acceleration"
            )

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
            # This is a one-time origin snapshot.  Logging it on every
            # VehicleLocalPosition callback made a healthy stream look like
            # repeated re-initialization and obscured coordinate-sign tests.
            self.get_logger().info(
                f"NED target initialized: N={msg.x:.3f} E={msg.y:.3f} "
                f"D={msg.z:.3f} yaw={math.degrees(msg.heading):.1f} deg"
            )
            self.get_logger().info(
                f"local vertical target sign={self.local_z_sign:+.0f} "
                "(target = current + sign * height)"
            )

    def _odometry_cb(self, msg: VehicleOdometry) -> None:
        self.odometry = msg

    def _ack_cb(self, msg: VehicleCommandAck) -> None:
        self.get_logger().info(f"PX4 command ack: command={msg.command} result={msg.result}")

    def _actuator_outputs_cb(self, msg: ActuatorOutputs) -> None:
        self.actuator_outputs = msg
        self.last_actuator_monotonic = time.monotonic()

    def _joint_state_cb(self, msg: JointState) -> None:
        positions = dict(zip(msg.name, msg.position))
        velocities = dict(zip(msg.name, msg.velocity))
        for index, name in enumerate(self.rl_joint_names):
            if name in positions and np.isfinite(positions[name]):
                self.rl_joint_positions[index] = float(positions[name])
            if name in velocities and np.isfinite(velocities[name]):
                self.rl_joint_velocities[index] = float(velocities[name])

    def _rl_action_cb(self, msg: Float32MultiArray) -> None:
        """Apply one bounded high-level action while PX4 remains the pilot."""
        values = np.asarray(msg.data, dtype=float)
        if values.shape != (11,) or not np.all(np.isfinite(values)):
            self.get_logger().error("RL action must contain 11 finite values")
            return
        values = np.clip(values, -1.0, 1.0)
        episode_command = float(values[10])
        if episode_command > 0.5:
            self.handle_key("t")
        elif episode_command < -0.5:
            self.land()
        if (
            self.target_initialized
            and self.status
            and self.status.nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD
        ):
            forward, right, up, yaw_rate = values[:4]
            yaw = self.target.yaw
            forward_n, forward_e = math.cos(yaw), math.sin(yaw)
            right_n, right_e = -math.sin(yaw), math.cos(yaw)
            self.target.north += self.MOVE_STEP_M * (forward * forward_n + right * right_n)
            self.target.east += self.MOVE_STEP_M * (forward * forward_e + right * right_e)
            self.target.down -= self.local_z_sign * self.ALT_STEP_M * up
            self.target.yaw = wrap_pi(self.target.yaw + self.YAW_STEP_RAD * yaw_rate)

        joint_velocity = values[4:10] * 0.20
        moving = bool(np.linalg.norm(joint_velocity) > 1.0e-5)
        if moving:
            duration_s = 0.25
            target = np.clip(
                self.rl_joint_positions + joint_velocity * duration_s,
                self.rl_joint_lower,
                self.rl_joint_upper,
            )
            trajectory = JointTrajectory()
            trajectory.joint_names = list(self.rl_joint_names)
            point = JointTrajectoryPoint()
            point.positions = target.tolist()
            # The configured joint_trajectory_controller requires the final
            # point velocity to be exactly zero.  The RL velocity action is
            # integrated into the next position target above; the controller
            # then executes that bounded increment and stops at the endpoint.
            point.velocities = np.zeros(6).tolist()
            point.time_from_start.sec = 0
            point.time_from_start.nanosec = int(duration_s * 1.0e9)
            trajectory.points = [point]
            self.rl_arm_pub.publish(trajectory)
        self.rl_arm_motion_pub.publish(Bool(data=moving))

    @staticmethod
    def _quaternion_wxyz_to_rpy(values) -> np.ndarray:
        q = np.asarray(values, dtype=float)
        if q.shape != (4,) or not np.all(np.isfinite(q)) or np.linalg.norm(q) < 1.0e-9:
            return np.zeros(3)
        w, x, y, z = q / np.linalg.norm(q)
        roll = math.atan2(2.0 * (w * x + y * z), 1.0 - 2.0 * (x * x + y * y))
        pitch = math.asin(float(np.clip(2.0 * (w * y - z * x), -1.0, 1.0)))
        yaw = math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))
        return np.array([roll, pitch, yaw], dtype=float)

    def publish_rl_observation(self) -> None:
        if self.local is None or self.status is None:
            return
        attitude = (
            self._quaternion_wxyz_to_rpy(self.odometry.q)
            if self.odometry is not None else np.array([0.0, 0.0, self.local.heading])
        )
        angular = (
            np.asarray(self.odometry.angular_velocity, dtype=float)
            if self.odometry is not None else np.zeros(3)
        )
        observation = np.r_[
            [self.local.x, self.local.y, self.local.z],
            [self.local.vx, self.local.vy, self.local.vz],
            attitude,
            angular,
            self.rl_joint_positions,
            self.rl_joint_velocities,
            [1.0],
        ]
        self.rl_observation_pub.publish(
            Float32MultiArray(data=observation.astype(np.float32).tolist())
        )
        self.rl_status_pub.publish(Float32MultiArray(data=[
            float(self.status.arming_state),
            float(self.status.nav_state),
            float(bool(self.status.failsafe)),
            float(bool(self.status.pre_flight_checks_pass)),
        ]))

    def _arm_feedforward_cb(self, msg: AccelStamped) -> None:
        values = np.asarray(
            [msg.accel.linear.x, msg.accel.linear.y, msg.accel.linear.z],
            dtype=float,
        )
        if np.all(np.isfinite(values)) and float(np.linalg.norm(values)) <= 2.0:
            self.arm_feedforward_ned = values.tolist()
            self.last_arm_feedforward_monotonic = time.monotonic()

    def _arm_reaction_wrench_cb(self, msg: WrenchStamped) -> None:
        values = np.asarray(
            [msg.wrench.torque.x, msg.wrench.torque.y, msg.wrench.torque.z],
            dtype=float,
        )
        if np.all(np.isfinite(values)):
            self.arm_reaction_torque_body_nm = values
            self.last_arm_reaction_monotonic = time.monotonic()

    def _arm_motion_active_cb(self, msg: Bool) -> None:
        self.arm_motion_active = bool(msg.data)
        self.last_arm_motion_monotonic = time.monotonic()

    def arm_motion_is_active(self, now: float | None = None) -> bool:
        """Return true only inside a fresh, explicitly announced arm motion."""
        current = time.monotonic() if now is None else float(now)
        age = current - self.last_arm_motion_monotonic
        return bool(self.arm_motion_active and 0.0 <= age < 1.0)

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
        if (
            self.arm_feedforward_enabled
            and self.arm_motion_is_active()
            and time.monotonic() - self.last_arm_feedforward_monotonic < 0.6
        ):
            sp.acceleration = list(self.arm_feedforward_ned)
        else:
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
        self.takeoff_ground_down = float(self.local.z)
        self.target = TargetNed(
            self.local.x,
            self.local.y,
            self.local.z + self.local_z_sign * self.TAKEOFF_HEIGHT_M,
            self.local.heading,
        )
        self.get_logger().info(
            f"target NED=({self.target.north:.2f}, {self.target.east:.2f}, "
            f"{self.target.down:.2f}) yaw={math.degrees(self.target.yaw):.1f} deg"
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

    def land(self, staged: bool = True) -> None:
        if self.landing_requested:
            return
        self.pending_takeoff = False
        self.landing_pub.publish(Bool(data=True))
        self.landing_requested = True
        if (
            staged
            and self.offboard_requested
            and self.target_initialized
            and self.local is not None
            and self.takeoff_ground_down is not None
        ):
            # The canted, near-thrust-limit airframe can produce a large pitch
            # transient when NAV_LAND takes over high above the ground.  Keep
            # PX4 in position-controlled Offboard while descending vertically;
            # the final native LAND handoff occurs close to the recorded
            # takeoff ground height, after the Gazebo support has had time to
            # restore without intersecting the aircraft.
            self.target.north = float(self.local.x)
            self.target.east = float(self.local.y)
            self.target.down = float(self.takeoff_ground_down)
            self.offboard_landing_active = True
            self.offboard_landing_started = time.monotonic()
            self.get_logger().warning(
                "OFFBOARD_LANDING_STARTED; holding XY and descending to the "
                "recorded takeoff ground height"
            )
            return
        self.publish_command(VehicleCommand.VEHICLE_CMD_NAV_LAND)
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
            self.target.down -= self.local_z_sign * self.ALT_STEP_M
        elif key == "f":
            self.target.down += self.local_z_sign * self.ALT_STEP_M
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
            if not self.begin_takeoff():
                # Preflight can remain false for a short interval while the
                # estimator receives its first valid heading.  Keep a single
                # request pending instead of requiring the operator/test to
                # guess the exact readiness instant.
                self.pending_takeoff = True
                self.get_logger().warning(
                    "Takeoff request queued until PX4 preflight is ready"
                )
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
        self.publish_rl_observation()
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
                f"arm_torque_nm={np.linalg.norm(self.arm_reaction_torque_body_nm):.3f} "
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

        if self.pending_takeoff and not self.offboard_requested:
            if self.state_fresh() and self.status and self.status.pre_flight_checks_pass:
                if not self.status.failsafe and self.begin_takeoff():
                    self.pending_takeoff = False

        if not self.offboard_requested:
            return
        if not self.state_fresh():
            self.get_logger().error("PX4 status timeout: stopping Offboard stream")
            self.offboard_requested = False
            return

        if (
            self.offboard_landing_active
            and self.takeoff_ground_down is not None
            and now - self.offboard_landing_started
            >= self.OFFBOARD_LAND_MIN_DURATION_S
            and abs(float(self.local.z) - self.takeoff_ground_down)
            <= self.OFFBOARD_LAND_HANDOFF_M
            and abs(float(self.local.vz))
            <= self.OFFBOARD_LAND_MAX_VERTICAL_SPEED_M_S
        ):
            self.publish_command(VehicleCommand.VEHICLE_CMD_NAV_LAND)
            self.offboard_landing_active = False
            self.offboard_requested = False
            self.get_logger().warning(
                "PX4_NATIVE_LAND_HANDOFF near recorded ground height; "
                "Offboard setpoint stream stopped"
            )
            return

        torque_age = now - self.last_arm_reaction_monotonic
        torque_norm = float(np.linalg.norm(self.arm_reaction_torque_body_nm))
        if self.arm_motion_is_active(now) and reaction_torque_safety_triggered(
            self.arm_reaction_torque_body_nm,
            torque_age,
            self.arm_torque_abort_nm,
            self.status.arming_state == VehicleStatus.ARMING_STATE_ARMED,
        ):
            self.get_logger().error(
                "Arm reaction-torque safety gate exceeded; requesting LAND "
                f"norm={torque_norm:.3f} N m limit={self.arm_torque_abort_nm:.3f}"
            )
            self.land(staged=False)
            return

        horizontal_error = math.hypot(
            self.local.x - self.target.north,
            self.local.y - self.target.east,
        )
        vertical_error = abs(self.local.z - self.target.down)
        if (
            self.status.arming_state == VehicleStatus.ARMING_STATE_ARMED
            and (
                horizontal_error > self.MAX_POSITION_ERROR_M
                or vertical_error > self.MAX_VERTICAL_ERROR_M
            )
        ):
            self.get_logger().error(
                "Position safety gate exceeded; requesting LAND "
                f"horizontal={horizontal_error:.2f} m vertical={vertical_error:.2f} m"
            )
            self.land(staged=False)
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
