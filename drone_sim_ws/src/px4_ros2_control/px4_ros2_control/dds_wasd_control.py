#!/usr/bin/env python3
"""Safe PX4 Offboard keyboard control over ROS 2 / Micro XRCE-DDS.

PX4 messages use NED/FRD.  Keyboard translations are body-heading relative:
W/S forward/back, A/D left/right, R/F up/down and Q/E yaw left/right.
"""

import argparse
import json
import math
import os
import select
import sys
import termios
import time
import tty
from dataclasses import dataclass
from enum import Enum
from typing import Optional

import numpy as np
import rclpy
from geometry_msgs.msg import AccelStamped, WrenchStamped
from nav_msgs.msg import Odometry as GazeboOdometry
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
from std_msgs.msg import Bool, Float32MultiArray, String


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


class FlightControlState(str, Enum):
    DISARMED = "DISARMED"
    POSITION_HOLD = "POSITION_HOLD"
    VELOCITY_CONTROL = "VELOCITY_CONTROL"
    LANDING = "LANDING"


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


def truth_hold_velocity_ned(
    target_enu: np.ndarray,
    position_enu: np.ndarray,
    velocity_enu: np.ndarray,
    *,
    position_gain_xy: float,
    position_gain_z: float,
    velocity_damping_xy: float,
    velocity_damping_z: float,
    maximum_speed_xy: float,
    maximum_speed_z: float,
) -> np.ndarray:
    """Outer-loop motion-capture position hold expressed as PX4 NED velocity."""
    target = np.asarray(target_enu, dtype=float)
    position = np.asarray(position_enu, dtype=float)
    velocity = np.asarray(velocity_enu, dtype=float)
    if any(value.shape != (3,) for value in (target, position, velocity)):
        raise ValueError("truth hold vectors must be three dimensional")
    if not all(np.all(np.isfinite(value)) for value in (target, position, velocity)):
        raise ValueError("truth hold vectors must be finite")
    error = target - position
    command_enu = np.array(
        [
            max(0.0, position_gain_xy) * error[0]
            - max(0.0, velocity_damping_xy) * velocity[0],
            max(0.0, position_gain_xy) * error[1]
            - max(0.0, velocity_damping_xy) * velocity[1],
            max(0.0, position_gain_z) * error[2]
            - max(0.0, velocity_damping_z) * velocity[2],
        ]
    )
    horizontal_norm = float(np.linalg.norm(command_enu[:2]))
    horizontal_limit = max(0.0, maximum_speed_xy)
    if horizontal_norm > horizontal_limit > 0.0:
        command_enu[:2] *= horizontal_limit / horizontal_norm
    elif horizontal_limit <= 0.0:
        command_enu[:2] = 0.0
    command_enu[2] = float(
        np.clip(command_enu[2], -max(0.0, maximum_speed_z), max(0.0, maximum_speed_z))
    )
    # Gazebo ENU [east,north,up] -> PX4 NED [north,east,down].
    return np.array([command_enu[1], command_enu[0], -command_enu[2]])


def truth_hold_position_ned(
    local_position_ned: np.ndarray,
    target_truth_enu: np.ndarray,
    position_truth_enu: np.ndarray,
    *,
    position_gain: float,
    maximum_offset_xy_m: float,
    maximum_offset_z_m: float,
) -> np.ndarray:
    """Translate motion-capture error into a PX4 local-position offset.

    The target is rebuilt around the current PX4 local position, so EKF
    origin/noise does not masquerade as a physical aircraft displacement.
    PX4 then closes its normal position, velocity, attitude and rate loops.
    """
    local = np.asarray(local_position_ned, dtype=float)
    target = np.asarray(target_truth_enu, dtype=float)
    position = np.asarray(position_truth_enu, dtype=float)
    if any(value.shape != (3,) for value in (local, target, position)):
        raise ValueError("truth position hold vectors must be three dimensional")
    if not all(np.all(np.isfinite(value)) for value in (local, target, position)):
        raise ValueError("truth position hold vectors must be finite")
    correction_enu = max(0.0, float(position_gain)) * (target - position)
    horizontal_norm = float(np.linalg.norm(correction_enu[:2]))
    horizontal_limit = max(0.0, float(maximum_offset_xy_m))
    if horizontal_norm > horizontal_limit > 0.0:
        correction_enu[:2] *= horizontal_limit / horizontal_norm
    elif horizontal_limit <= 0.0:
        correction_enu[:2] = 0.0
    correction_enu[2] = float(
        np.clip(
            correction_enu[2],
            -max(0.0, float(maximum_offset_z_m)),
            max(0.0, float(maximum_offset_z_m)),
        )
    )
    correction_ned = np.asarray(
        [correction_enu[1], correction_enu[0], -correction_enu[2]], dtype=float
    )
    return local + correction_ned


class DdsWasdControl(Node):
    RATE_HZ = 20.0
    STATE_REPORT_HZ = float(os.environ.get("PX4_STATE_REPORT_HZ", "1.0"))
    # vehicle_status is intentionally low rate.  A healthy half-second PX4
    # simulation interval can take several wall-clock seconds while Gazebo is
    # below real time during arm motion.  Keep this liveness window separate
    # from the high-rate local-position safety gate: a delayed status report
    # must not cause a false landing, while stale position must stop control.
    STATUS_TIMEOUT_S = float(os.environ.get("PX4_DDS_STATUS_TIMEOUT_S", "5.0"))
    LOCAL_POSITION_TIMEOUT_S = float(
        os.environ.get("PX4_DDS_LOCAL_POSITION_TIMEOUT_S", "1.0")
    )
    LANDING_LOCAL_POSITION_TIMEOUT_S = float(
        os.environ.get("PX4_DDS_LANDING_LOCAL_POSITION_TIMEOUT_S", "2.0")
    )
    ACTUATOR_OUTPUT_TIMEOUT_S = float(
        os.environ.get("PX4_DDS_ACTUATOR_OUTPUT_TIMEOUT_S", "1.0")
    )
    PRESTREAM_S = 2.0
    # RL actions still use bounded position increments.  Interactive keyboard
    # flight below is velocity controlled and never accumulates these steps.
    MOVE_STEP_M = 0.08
    ALT_STEP_M = 0.12
    YAW_STEP_RAD = math.radians(3.0)
    HORIZONTAL_SPEED_M_S = float(os.environ.get("PX4_WASD_HORIZONTAL_SPEED_M_S", "0.4"))
    VERTICAL_SPEED_M_S = float(os.environ.get("PX4_WASD_VERTICAL_SPEED_M_S", "0.15"))
    YAW_RATE_RAD_S = math.radians(
        float(os.environ.get("PX4_WASD_YAW_RATE_DEG_S", "15.0"))
    )
    KEY_RELEASE_TIMEOUT_S = float(
        os.environ.get("PX4_KEY_RELEASE_TIMEOUT_S", "0.20")
    )
    HORIZONTAL_ACCEL_LIMIT_M_S2 = float(
        # Clean Base 1 A/B: 0.30 m/s^2 produced 54% velocity overshoot,
        # whereas 0.15 m/s^2 held the same 0.40 m/s target to 9.1%.
        os.environ.get("PX4_WASD_HORIZONTAL_ACCEL_M_S2", "0.15")
    )
    HORIZONTAL_JERK_LIMIT_M_S3 = float(
        os.environ.get("PX4_WASD_HORIZONTAL_JERK_M_S3", "0.30")
    )
    VERTICAL_ACCEL_LIMIT_M_S2 = float(
        # 0.20 m/s^2 produced 11.3% overshoot in the clean R->F->Q
        # transition regression.  0.18 m/s^2 reduced it to 2.0% while
        # preserving the requested 0.15 m/s latched vertical speed.
        os.environ.get("PX4_WASD_VERTICAL_ACCEL_M_S2", "0.18")
    )
    VERTICAL_JERK_LIMIT_M_S3 = float(
        os.environ.get("PX4_WASD_VERTICAL_JERK_M_S3", "0.40")
    )
    YAW_ACCEL_LIMIT_RAD_S2 = math.radians(
        # Direct Q->E reversal with 30 deg/s^2 caused a 0.223 m/s vertical
        # coupling transient.  20 deg/s^2, together with the debug yaw-rate
        # integral tune, held the transient to 0.035 m/s and yaw overshoot to
        # 2.7%.  The 15 deg/s latched yaw-rate limit is unchanged.
        float(os.environ.get("PX4_WASD_YAW_ACCEL_DEG_S2", "20.0"))
    )
    RELEASE_HORIZONTAL_SPEED_M_S = float(
        os.environ.get("PX4_WASD_RELEASE_HORIZONTAL_SPEED_M_S", "0.08")
    )
    RELEASE_VERTICAL_SPEED_M_S = float(
        os.environ.get("PX4_WASD_RELEASE_VERTICAL_SPEED_M_S", "0.05")
    )
    RELEASE_YAW_RATE_RAD_S = math.radians(
        float(os.environ.get("PX4_WASD_RELEASE_YAW_RATE_DEG_S", "5.0"))
    )
    VELOCITY_ZERO_EPS = 1.0e-3
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
    TOUCHDOWN_DISARM_ENABLED = os.environ.get(
        "PX4_TOUCHDOWN_DISARM_ENABLED", "false"
    ).lower() in {"1", "true", "yes", "on"}
    TOUCHDOWN_DISARM_HEIGHT_M = float(
        os.environ.get("PX4_TOUCHDOWN_DISARM_HEIGHT_M", "0.05")
    )
    TOUCHDOWN_DISARM_HOLD_S = float(
        os.environ.get("PX4_TOUCHDOWN_DISARM_HOLD_S", "0.5")
    )
    TRUTH_HOLD_ENABLED = os.environ.get(
        "PX4_TRUTH_HOLD_ENABLED", "false"
    ).lower() in {"1", "true", "yes", "on"}
    TRUTH_HOLD_TIMEOUT_S = float(os.environ.get("PX4_TRUTH_HOLD_TIMEOUT_S", "1.0"))
    TRUTH_HOLD_XY_P = float(os.environ.get("PX4_TRUTH_HOLD_XY_P", "0.80"))
    TRUTH_HOLD_Z_P = float(os.environ.get("PX4_TRUTH_HOLD_Z_P", "1.30"))
    TRUTH_HOLD_XY_D = float(os.environ.get("PX4_TRUTH_HOLD_XY_D", "0.25"))
    TRUTH_HOLD_Z_D = float(os.environ.get("PX4_TRUTH_HOLD_Z_D", "0.45"))
    TRUTH_HOLD_VELOCITY_FILTER_TAU_S = float(
        os.environ.get("PX4_TRUTH_HOLD_VELOCITY_FILTER_TAU_S", "0.0")
    )
    TRUTH_HOLD_XY_MAX_M_S = float(
        os.environ.get("PX4_TRUTH_HOLD_XY_MAX_M_S", "0.08")
    )
    TRUTH_HOLD_Z_MAX_M_S = float(
        os.environ.get("PX4_TRUTH_HOLD_Z_MAX_M_S", "0.12")
    )
    TRUTH_HOLD_POSITION_GAIN = float(
        os.environ.get("PX4_TRUTH_HOLD_POSITION_GAIN", "1.0")
    )
    TRUTH_HOLD_POSITION_XY_MAX_M = float(
        os.environ.get("PX4_TRUTH_HOLD_POSITION_XY_MAX_M", "0.15")
    )
    TRUTH_HOLD_POSITION_Z_MAX_M = float(
        os.environ.get("PX4_TRUTH_HOLD_POSITION_Z_MAX_M", "0.12")
    )

    def __init__(self, arm_only: bool = False) -> None:
        super().__init__("my_drone_dds_wasd_control")
        self.arm_only = arm_only
        self.status: Optional[VehicleStatus] = None
        self.local: Optional[VehicleLocalPosition] = None
        self.odometry: Optional[VehicleOdometry] = None
        self.gazebo_truth_enu: Optional[np.ndarray] = None
        self.gazebo_truth_velocity_enu: Optional[np.ndarray] = None
        self.gazebo_truth_velocity_filtered_enu: Optional[np.ndarray] = None
        self.gazebo_truth_rpy: Optional[np.ndarray] = None
        self.gazebo_truth_angular_velocity: Optional[np.ndarray] = None
        self.last_gazebo_truth_monotonic = 0.0
        self.last_gazebo_truth_filter_monotonic = 0.0
        self.truth_hold_target_enu: Optional[np.ndarray] = None
        self.truth_hold_stale_reported = False
        self.actuator_outputs: Optional[ActuatorOutputs] = None
        self.last_actuator_monotonic = 0.0
        self.last_status_monotonic = 0.0
        self.last_local_monotonic = 0.0
        self.status_stale_reported = False
        self.target = TargetNed()
        self.target_initialized = False
        self.xy_reset_counter: Optional[int] = None
        self.z_reset_counter: Optional[int] = None
        self.heading_reset_counter: Optional[int] = None
        self.control_state = FlightControlState.DISARMED
        self.active_velocity_key: Optional[str] = None
        self.last_velocity_key_monotonic = 0.0
        self.hover_transition_pending = False
        self.velocity_command_ned = np.zeros(3)
        self.acceleration_command_ned = np.zeros(3)
        self.velocity_acceleration_feedforward_enabled = os.environ.get(
            "PX4_WASD_ACCELERATION_FEEDFORWARD_ENABLED", "true"
        ).lower() in {"1", "true", "yes", "on"}
        self.yaw_rate_command = 0.0
        self.yaw_hold_rad = float("nan")
        self.yaw_hold_pending = False
        self.last_control_tick_monotonic = time.monotonic()
        self.pending_takeoff = False
        self.offboard_requested = False
        self.landing_requested = False
        self.offboard_landing_active = False
        self.offboard_landing_started = 0.0
        self.touchdown_stable_since: Optional[float] = None
        self.touchdown_land_command_sent = 0.0
        self.touchdown_disarm_sent = False
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
        self.arm_reaction_force_norm_n = float("nan")
        self.arm_com_shift_norm_m = float("nan")
        self.arm_inertia_diag_kg_m2 = np.full(3, float("nan"))
        self.last_arm_coupling_state_monotonic = 0.0
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
            GazeboOdometry,
            "/model/my_drone/odometry",
            self._gazebo_truth_cb,
            10,
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
            String,
            "/my_drone/arm_coupling_state",
            self._arm_coupling_state_cb,
            10,
        )
        self.create_subscription(
            Bool,
            "/my_drone/arm_motion_active",
            self._arm_motion_active_cb,
            10,
        )
        self.create_subscription(
            Bool,
            "/my_drone/hover_request",
            self._hover_request_cb,
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
        self.status_stale_reported = False

    def _gazebo_truth_cb(self, msg: GazeboOdometry) -> None:
        now_monotonic = time.monotonic()
        self.gazebo_truth_enu = np.array(
            [
                float(msg.pose.pose.position.x),
                float(msg.pose.pose.position.y),
                float(msg.pose.pose.position.z),
            ]
        )
        self.gazebo_truth_velocity_enu = np.array(
            [
                float(msg.twist.twist.linear.x),
                float(msg.twist.twist.linear.y),
                float(msg.twist.twist.linear.z),
            ]
        )
        filter_tau_s = max(0.0, self.TRUTH_HOLD_VELOCITY_FILTER_TAU_S)
        filter_dt_s = now_monotonic - self.last_gazebo_truth_filter_monotonic
        if (
            self.gazebo_truth_velocity_filtered_enu is None
            or filter_tau_s <= 0.0
            or filter_dt_s <= 0.0
            or filter_dt_s > self.TRUTH_HOLD_TIMEOUT_S
        ):
            self.gazebo_truth_velocity_filtered_enu = (
                self.gazebo_truth_velocity_enu.copy()
            )
        else:
            # Exact first-order low-pass discretization.  Only the derivative
            # branch is filtered: the position error remains unfiltered so H
            # retains its steady-state accuracy while Gazebo velocity noise
            # cannot be amplified into an attitude oscillation by a larger D.
            filter_alpha = 1.0 - math.exp(-filter_dt_s / filter_tau_s)
            self.gazebo_truth_velocity_filtered_enu += filter_alpha * (
                self.gazebo_truth_velocity_enu
                - self.gazebo_truth_velocity_filtered_enu
            )
        self.last_gazebo_truth_filter_monotonic = now_monotonic
        orientation = msg.pose.pose.orientation
        self.gazebo_truth_rpy = self._quaternion_wxyz_to_rpy(
            [orientation.w, orientation.x, orientation.y, orientation.z]
        )
        self.gazebo_truth_angular_velocity = np.array(
            [
                float(msg.twist.twist.angular.x),
                float(msg.twist.twist.angular.y),
                float(msg.twist.twist.angular.z),
            ]
        )
        self.last_gazebo_truth_monotonic = now_monotonic

    def _local_cb(self, msg: VehicleLocalPosition) -> None:
        if self.target_initialized:
            if (
                self.xy_reset_counter is not None
                and int(msg.xy_reset_counter) != self.xy_reset_counter
            ):
                self.target.north += float(msg.delta_xy[0])
                self.target.east += float(msg.delta_xy[1])
                self.get_logger().warning(
                    "LOCAL_XY_RESET_APPLIED "
                    f"counter={int(msg.xy_reset_counter)} "
                    f"delta=({float(msg.delta_xy[0]):.3f},"
                    f"{float(msg.delta_xy[1]):.3f})"
                )
            if (
                self.z_reset_counter is not None
                and int(msg.z_reset_counter) != self.z_reset_counter
            ):
                delta_z = float(msg.delta_z)
                self.target.down += delta_z
                if self.takeoff_ground_down is not None:
                    self.takeoff_ground_down += delta_z
                self.get_logger().warning(
                    "LOCAL_Z_RESET_APPLIED "
                    f"counter={int(msg.z_reset_counter)} delta={delta_z:.3f}"
                )
            if (
                self.heading_reset_counter is not None
                and int(msg.heading_reset_counter) != self.heading_reset_counter
            ):
                delta_heading = float(msg.delta_heading)
                self.target.yaw = wrap_pi(self.target.yaw + delta_heading)
                if math.isfinite(self.yaw_hold_rad):
                    self.yaw_hold_rad = wrap_pi(self.yaw_hold_rad + delta_heading)
                self.get_logger().warning(
                    "LOCAL_HEADING_RESET_APPLIED "
                    f"counter={int(msg.heading_reset_counter)} "
                    f"delta_deg={math.degrees(delta_heading):.2f}"
                )

        self.xy_reset_counter = int(msg.xy_reset_counter)
        self.z_reset_counter = int(msg.z_reset_counter)
        self.heading_reset_counter = int(msg.heading_reset_counter)
        self.local = msg
        self.last_local_monotonic = time.monotonic()
        if not self.target_initialized and msg.xy_valid and msg.z_valid:
            self.target = TargetNed(msg.x, msg.y, msg.z, msg.heading)
            self.yaw_hold_rad = float(msg.heading)
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

    def _arm_coupling_state_cb(self, msg: String) -> None:
        """Accept only finite, dimensionally complete coupling diagnostics."""
        try:
            report = json.loads(msg.data)
            com_shift = np.asarray(report["com_shift_m"], dtype=float)
            inertia_diag = np.asarray(report["inertia_diag_kg_m2"], dtype=float)
            reaction_force = np.asarray(report["reaction_force_body_n"], dtype=float)
        except (KeyError, TypeError, ValueError, json.JSONDecodeError):
            return
        if any(values.shape != (3,) for values in (com_shift, inertia_diag, reaction_force)):
            return
        if not all(np.all(np.isfinite(values)) for values in (com_shift, inertia_diag, reaction_force)):
            return
        if np.any(inertia_diag <= 0.0):
            return
        self.arm_com_shift_norm_m = float(np.linalg.norm(com_shift))
        self.arm_inertia_diag_kg_m2 = inertia_diag
        self.arm_reaction_force_norm_n = float(np.linalg.norm(reaction_force))
        self.last_arm_coupling_state_monotonic = time.monotonic()

    def _arm_motion_active_cb(self, msg: Bool) -> None:
        self.arm_motion_active = bool(msg.data)
        self.last_arm_motion_monotonic = time.monotonic()

    def _hover_request_cb(self, msg: Bool) -> None:
        """Apply the same hover transition as the operator's H key."""
        if msg.data:
            self.hold_current_position()

    def arm_motion_is_active(self, now: float | None = None) -> bool:
        """Return true only inside a fresh, explicitly announced arm motion."""
        current = time.monotonic() if now is None else float(now)
        age = current - self.last_arm_motion_monotonic
        return bool(self.arm_motion_active and 0.0 <= age < 1.0)

    def status_fresh(self) -> bool:
        now = time.monotonic()
        return bool(
            self.status is not None
            and now - self.last_status_monotonic < self.STATUS_TIMEOUT_S
        )

    def local_position_fresh(self, timeout_s: float | None = None) -> bool:
        now = time.monotonic()
        maximum_age = (
            self.LOCAL_POSITION_TIMEOUT_S
            if timeout_s is None
            else float(timeout_s)
        )
        return (
            self.local is not None
            and now - self.last_local_monotonic < maximum_age
            and self.local.xy_valid
            and self.local.z_valid
        )

    def state_fresh(self) -> bool:
        """Strict gate used before arming: both status and position are fresh."""
        return self.status_fresh() and self.local_position_fresh()

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
        if (
            self.control_state == FlightControlState.POSITION_HOLD
            and self.TRUTH_HOLD_ENABLED
            and self.truth_hold_target_enu is not None
            and self.truth_hold_fresh()
        ):
            self.publish_truth_hold()
            return
        if (
            self.control_state == FlightControlState.POSITION_HOLD
            and
            self.TRUTH_HOLD_ENABLED
            and self.truth_hold_target_enu is not None
            and not self.truth_hold_stale_reported
        ):
            self.get_logger().warning(
                "TRUTH_HOLD_STALE; falling back to PX4 position hold"
            )
            self.truth_hold_stale_reported = True
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

    def truth_hold_fresh(self) -> bool:
        return bool(
            self.gazebo_truth_enu is not None
            and self.gazebo_truth_velocity_enu is not None
            and 0.0 <= time.monotonic() - self.last_gazebo_truth_monotonic
            < self.TRUTH_HOLD_TIMEOUT_S
        )

    def publish_truth_hold(self) -> None:
        """Hold the H snapshot with a motion-capture outer velocity loop.

        Gazebo truth plays the same role as a motion-capture/VIO measurement:
        this outer loop only asks for a bounded corrective velocity.  PX4
        remains responsible for velocity, acceleration, attitude, rate and
        actuator control.  Using velocity mode avoids rebuilding a moving
        local-position target from noisy EKF coordinates on every control
        tick, which previously produced an XY limit cycle.
        """
        mode = OffboardControlMode()
        self.truth_hold_stale_reported = False
        mode.timestamp = self.now_us()
        mode.velocity = True
        self.mode_pub.publish(mode)
        velocity_ned = truth_hold_velocity_ned(
            self.truth_hold_target_enu,
            self.gazebo_truth_enu,
            self.gazebo_truth_velocity_filtered_enu,
            position_gain_xy=self.TRUTH_HOLD_XY_P,
            position_gain_z=self.TRUTH_HOLD_Z_P,
            velocity_damping_xy=self.TRUTH_HOLD_XY_D,
            velocity_damping_z=self.TRUTH_HOLD_Z_D,
            maximum_speed_xy=self.TRUTH_HOLD_XY_MAX_M_S,
            maximum_speed_z=self.TRUTH_HOLD_Z_MAX_M_S,
        )
        nan = float("nan")
        sp = TrajectorySetpoint()
        sp.timestamp = mode.timestamp
        sp.position = [nan, nan, nan]
        sp.velocity = velocity_ned.tolist()
        sp.acceleration = [nan, nan, nan]
        sp.jerk = [nan, nan, nan]
        sp.yaw = self.target.yaw
        sp.yawspeed = nan
        self.setpoint_pub.publish(sp)

    def publish_velocity(self) -> None:
        """Publish a velocity-only setpoint; no position field is valid."""
        mode = OffboardControlMode()
        mode.timestamp = self.now_us()
        mode.velocity = True
        self.mode_pub.publish(mode)

        nan = float("nan")
        sp = TrajectorySetpoint()
        sp.timestamp = mode.timestamp
        sp.position = [nan, nan, nan]
        sp.velocity = self.velocity_command_ned.tolist()
        sp.acceleration = (
            self.acceleration_command_ned.tolist()
            if self.velocity_acceleration_feedforward_enabled
            else [nan, nan, nan]
        )
        sp.jerk = [nan, nan, nan]
        if abs(self.yaw_rate_command) > self.VELOCITY_ZERO_EPS:
            sp.yaw = nan
            sp.yawspeed = float(self.yaw_rate_command)
        else:
            sp.yaw = float(self.yaw_hold_rad)
            sp.yawspeed = nan
        self.setpoint_pub.publish(sp)

    def begin_takeoff(self) -> bool:
        if not self.state_fresh() or not self.target_initialized:
            self.get_logger().error("Cannot start: PX4 state/position is absent or stale")
            return False
        if self.status.failsafe or not self.status.pre_flight_checks_pass:
            self.get_logger().error("Cannot start: PX4 preflight checks/failsafe state is unsafe")
            return False
        self._refresh_disarmed_takeoff_target()
        self.control_state = FlightControlState.POSITION_HOLD
        self._clear_velocity_command()
        self.get_logger().info(
            f"target NED=({self.target.north:.2f}, {self.target.east:.2f}, "
            f"{self.target.down:.2f}) yaw={math.degrees(self.target.yaw):.1f} deg"
        )
        self.prestream_started = time.monotonic()
        self.offboard_requested = True
        self.get_logger().info("Prestreaming Offboard setpoints for 2 seconds")
        return True

    def _refresh_disarmed_takeoff_target(self) -> None:
        """Track a falling/unsettled vehicle throughout Offboard prestream.

        Gazebo can still move the unpowered model while PX4 receives the
        required two seconds of Offboard setpoints.  Locking the target at the
        first key press turns that motion into a large position error exactly
        when PX4 arms.  Refreshing from the latest local position keeps the
        commanded takeoff step equal to ``TAKEOFF_HEIGHT_M`` until arming.
        """
        self.takeoff_ground_down = float(self.local.z)
        self.target = TargetNed(
            float(self.local.x),
            float(self.local.y),
            float(self.local.z) + self.local_z_sign * self.TAKEOFF_HEIGHT_M,
            float(self.local.heading),
        )

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
        self.control_state = FlightControlState.LANDING
        self._clear_velocity_command()
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
        self.control_state = FlightControlState.POSITION_HOLD
        self._clear_velocity_command()
        self.get_logger().warning("Requested POSCTL and stopped Offboard stream")

    def emergency_disarm(self) -> None:
        self.publish_command(
            VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM,
            param1=0.0,
            param2=21196.0,
        )
        self.offboard_requested = False
        self.control_state = FlightControlState.DISARMED
        self._clear_velocity_command()
        self.get_logger().error("EMERGENCY FORCE DISARM sent")

    def _clear_velocity_command(self) -> None:
        self.hover_transition_pending = False
        self.active_velocity_key = None
        self.last_velocity_key_monotonic = 0.0
        self.velocity_command_ned = np.zeros(3)
        self.acceleration_command_ned = np.zeros(3)
        self.yaw_rate_command = 0.0
        self.yaw_hold_pending = False

    def stop_velocity_demand(self) -> None:
        """Latch zero demand; the S-curve state decelerates independently."""
        self.active_velocity_key = None
        self.last_velocity_key_monotonic = 0.0
        self.yaw_hold_pending = True

    @staticmethod
    def _ramp_scalar(current: float, target: float, max_delta: float) -> float:
        delta = float(target) - float(current)
        if abs(delta) <= max_delta:
            return float(target)
        return float(current) + math.copysign(max_delta, delta)

    @staticmethod
    def _ramp_horizontal(
        current: np.ndarray, target: np.ndarray, max_delta: float
    ) -> np.ndarray:
        current = np.asarray(current, dtype=float)
        target = np.asarray(target, dtype=float)
        delta = target - current
        norm = float(np.linalg.norm(delta))
        if norm <= max_delta or norm <= 1.0e-12:
            return target.copy()
        return current + delta * (max_delta / norm)

    @classmethod
    def _jerk_limited_vector_step(
        cls,
        velocity: np.ndarray,
        acceleration: np.ndarray,
        target_velocity: np.ndarray,
        acceleration_limit: float,
        jerk_limit: float,
        dt: float,
    ) -> tuple[np.ndarray, np.ndarray]:
        """Advance a bounded S-curve state without overshooting its target."""
        velocity = np.asarray(velocity, dtype=float).copy()
        acceleration = np.asarray(acceleration, dtype=float).copy()
        previous_acceleration = acceleration.copy()
        target_velocity = np.asarray(target_velocity, dtype=float)
        dt = max(0.0, min(float(dt), 0.25))
        if dt <= 0.0:
            return velocity, acceleration
        error = target_velocity - velocity
        error_norm = float(np.linalg.norm(error))
        if (
            error_norm <= cls.VELOCITY_ZERO_EPS + 1.0e-12
            and float(np.linalg.norm(acceleration)) <= jerk_limit * dt
        ):
            return target_velocity.copy(), np.zeros_like(acceleration)
        direction = error / max(error_norm, 1.0e-12)
        acceleration_toward_target = max(float(np.dot(acceleration, direction)), 0.0)
        braking_delta_velocity = (
            acceleration_toward_target * acceleration_toward_target
            / max(2.0 * jerk_limit, 1.0e-12)
        )
        # The continuous-time stopping distance lands exactly on the target
        # only when the control update is infinitesimal. Reserve one complete
        # update of velocity plus half a jerk step so a discrete S-curve starts
        # removing acceleration before it crosses the latched speed bound.
        braking_delta_velocity += (
            1.5 * acceleration_toward_target * dt
            + jerk_limit * dt * dt
        )
        if error_norm <= braking_delta_velocity:
            desired_acceleration = np.zeros_like(acceleration)
        else:
            desired_acceleration = direction * acceleration_limit
        acceleration = cls._ramp_horizontal(
            acceleration, desired_acceleration, jerk_limit * dt
        )
        acceleration_norm = float(np.linalg.norm(acceleration))
        if acceleration_norm > acceleration_limit > 0.0:
            acceleration *= acceleration_limit / acceleration_norm
        candidate = velocity + acceleration * dt
        if float(np.dot(target_velocity - candidate, error)) <= 0.0:
            crossing_acceleration = error / dt
            if float(np.linalg.norm(crossing_acceleration - previous_acceleration)) <= jerk_limit * dt + 1.0e-12:
                return target_velocity.copy(), crossing_acceleration
        return candidate, acceleration

    def set_velocity_key(self, key: str, now: float | None = None) -> None:
        """Latch one body-frame velocity target until H or another command."""
        if key not in "wasdrfqe":
            return
        self.hover_transition_pending = False
        # A real terminal emits repeated characters while a key is held.  The
        # latched command is edge-triggered: repeating the same key must not
        # restart any ramp or re-latch heading to a drifting measured value.
        # This makes a tap and a long press physically identical.
        if key == self.active_velocity_key:
            return
        self.active_velocity_key = key
        self.last_velocity_key_monotonic = time.monotonic() if now is None else float(now)
        self.get_logger().info(f"VELOCITY_DEMAND_LATCH key={key.upper()}")
        if self.control_state != FlightControlState.VELOCITY_CONTROL:
            self.control_state = FlightControlState.VELOCITY_CONTROL
            self.get_logger().info(f"VELOCITY_CONTROL_ENTER key={key.upper()}")
        if key not in "qe":
            self.yaw_hold_pending = True

    def _desired_velocity_ned(self, key_active: bool) -> tuple[np.ndarray, float]:
        desired = np.zeros(3)
        desired_yaw_rate = 0.0
        if not key_active or self.active_velocity_key is None:
            return desired, desired_yaw_rate
        heading = (
            float(self.local.heading)
            if self.local is not None and math.isfinite(float(self.local.heading))
            else float(self.target.yaw)
        )
        forward = np.array([math.cos(heading), math.sin(heading)])
        right = np.array([-math.sin(heading), math.cos(heading)])
        key = self.active_velocity_key
        if key == "w":
            desired[:2] = self.HORIZONTAL_SPEED_M_S * forward
        elif key == "s":
            desired[:2] = -self.HORIZONTAL_SPEED_M_S * forward
        elif key == "d":
            desired[:2] = self.HORIZONTAL_SPEED_M_S * right
        elif key == "a":
            desired[:2] = -self.HORIZONTAL_SPEED_M_S * right
        elif key == "r":
            desired[2] = self.local_z_sign * self.VERTICAL_SPEED_M_S
        elif key == "f":
            desired[2] = -self.local_z_sign * self.VERTICAL_SPEED_M_S
        elif key == "q":
            desired_yaw_rate = -self.YAW_RATE_RAD_S
        elif key == "e":
            desired_yaw_rate = self.YAW_RATE_RAD_S
        return desired, desired_yaw_rate

    def update_velocity_control(self, now: float, dt: float) -> None:
        if self.control_state != FlightControlState.VELOCITY_CONTROL:
            return
        # Manual velocity is deliberately latched.  Key release and elapsed
        # time do not alter the target; only H or a new direction command does.
        key_active = self.active_velocity_key is not None
        desired, desired_yaw_rate = self._desired_velocity_ned(key_active)
        horizontal, horizontal_acceleration = self._jerk_limited_vector_step(
            self.velocity_command_ned[:2],
            self.acceleration_command_ned[:2],
            desired[:2],
            self.HORIZONTAL_ACCEL_LIMIT_M_S2,
            self.HORIZONTAL_JERK_LIMIT_M_S3,
            dt,
        )
        vertical, vertical_acceleration = self._jerk_limited_vector_step(
            self.velocity_command_ned[2:3],
            self.acceleration_command_ned[2:3],
            desired[2:3],
            self.VERTICAL_ACCEL_LIMIT_M_S2,
            self.VERTICAL_JERK_LIMIT_M_S3,
            dt,
        )
        self.velocity_command_ned[:2] = horizontal
        self.acceleration_command_ned[:2] = horizontal_acceleration
        self.velocity_command_ned[2] = vertical[0]
        self.acceleration_command_ned[2] = vertical_acceleration[0]
        self.yaw_rate_command = self._ramp_scalar(
            self.yaw_rate_command,
            desired_yaw_rate,
            self.YAW_ACCEL_LIMIT_RAD_S2 * max(0.0, min(float(dt), 0.25)),
        )
        if (
            self.yaw_hold_pending
            and abs(self.yaw_rate_command) <= self.VELOCITY_ZERO_EPS
            and self.local is not None
            and math.isfinite(float(self.local.heading))
        ):
            self.yaw_hold_rad = float(self.local.heading)
            self.yaw_hold_pending = False

    def _measured_motion_settled(self) -> bool:
        """Wait for physical braking before latching a position setpoint."""
        if self.local is not None:
            vx = float(getattr(self.local, "vx", float("nan")))
            vy = float(getattr(self.local, "vy", float("nan")))
            vz = float(getattr(self.local, "vz", float("nan")))
            if math.isfinite(vx) and math.isfinite(vy):
                if math.hypot(vx, vy) > self.RELEASE_HORIZONTAL_SPEED_M_S:
                    return False
            if math.isfinite(vz) and abs(vz) > self.RELEASE_VERTICAL_SPEED_M_S:
                return False
        odometry = getattr(self, "odometry", None)
        if odometry is not None:
            angular = np.asarray(odometry.angular_velocity, dtype=float)
            if angular.shape == (3,) and np.all(np.isfinite(angular)):
                if abs(float(angular[2])) > self.RELEASE_YAW_RATE_RAD_S:
                    return False
        return True

    def hold_current_position(self) -> None:
        """Brake with the S-curve, then freeze a real position setpoint."""
        self.stop_velocity_demand()
        self.control_state = FlightControlState.VELOCITY_CONTROL
        self.hover_transition_pending = True
        self.get_logger().info("HOVER_BRAKING_TO_POSITION_HOLD")

    def complete_hover_transition_if_ready(self) -> bool:
        """Switch H from zero-velocity braking to a fixed NED position."""
        if not self.hover_transition_pending or self.local is None:
            return False
        command_stopped = bool(
            np.linalg.norm(self.velocity_command_ned) <= self.VELOCITY_ZERO_EPS
            and np.linalg.norm(self.acceleration_command_ned) <= self.VELOCITY_ZERO_EPS
            and abs(self.yaw_rate_command) <= self.VELOCITY_ZERO_EPS
        )
        if not command_stopped or not self._measured_motion_settled():
            return False
        self.target = TargetNed(
            float(self.local.x),
            float(self.local.y),
            float(self.local.z),
            float(self.local.heading),
        )
        self.yaw_hold_rad = float(self.local.heading)
        if self.TRUTH_HOLD_ENABLED and self.truth_hold_fresh():
            self.truth_hold_target_enu = self.gazebo_truth_enu.copy()
            self.get_logger().info(
                "TRUTH_HOLD_LOCKED ENU=("
                f"{self.truth_hold_target_enu[0]:.3f},"
                f"{self.truth_hold_target_enu[1]:.3f},"
                f"{self.truth_hold_target_enu[2]:.3f})"
            )
        else:
            self.truth_hold_target_enu = None
        self.control_state = FlightControlState.POSITION_HOLD
        self.hover_transition_pending = False
        self._clear_velocity_command()
        self.get_logger().info(
            "HOVER_POSITION_HOLD_LOCKED "
            f"NED=({self.target.north:.3f},{self.target.east:.3f},{self.target.down:.3f})"
        )
        return True

    def handle_key(self, key: str) -> None:
        if key == "t":
            if self.landing_requested or self.offboard_landing_active:
                self.get_logger().warning("Takeoff ignored: landing is in progress")
                return
            if self.offboard_requested or (
                self.status
                and self.status.arming_state == VehicleStatus.ARMING_STATE_ARMED
            ):
                self.get_logger().warning("Takeoff ignored: vehicle is already active")
                return
            if not self.begin_takeoff():
                # Preflight can remain false for a short interval while the
                # estimator receives its first valid heading.  Keep a single
                # request pending instead of requiring the operator/test to
                # guess the exact readiness instant.
                self.pending_takeoff = True
                self.get_logger().warning(
                    "Takeoff request queued until PX4 preflight is ready"
                )
        elif key in "wasdrfqeh":
            if self.status and self.status.nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD:
                if key == "h":
                    self.hold_current_position()
                else:
                    self.set_velocity_key(key)
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
        dt = now - self.last_control_tick_monotonic
        self.last_control_tick_monotonic = now
        self.publish_rl_observation()
        state_report_period_s = 1.0 / max(0.2, min(20.0, self.STATE_REPORT_HZ))
        if self.status and self.local and now - self.last_state_report >= state_report_period_s:
            self.last_state_report = now
            attitude_rpy = (
                self._quaternion_wxyz_to_rpy(self.odometry.q)
                if self.odometry is not None else np.zeros(3)
            )
            angular_velocity = (
                np.asarray(self.odometry.angular_velocity, dtype=float)
                if self.odometry is not None else np.zeros(3)
            )
            truth_report = (
                "truth_enu=(nan,nan,nan)"
                if self.gazebo_truth_enu is None
                else "truth_enu=("
                + ",".join(f"{value:.3f}" for value in self.gazebo_truth_enu)
                + ")"
            )
            truth_velocity_report = (
                "truth_vel_enu=(nan,nan,nan)"
                if self.gazebo_truth_velocity_enu is None
                else "truth_vel_enu=("
                + ",".join(f"{value:.3f}" for value in self.gazebo_truth_velocity_enu)
                + ")"
            )
            truth_attitude_report = (
                "truth_rpy_deg=(nan,nan,nan)"
                if self.gazebo_truth_rpy is None
                else "truth_rpy_deg=("
                + ",".join(
                    f"{math.degrees(value):.1f}" for value in self.gazebo_truth_rpy
                )
                + ")"
            )
            truth_rate_report = (
                "truth_body_rate_deg_s=(nan,nan,nan)"
                if self.gazebo_truth_angular_velocity is None
                else "truth_body_rate_deg_s=("
                + ",".join(
                    f"{math.degrees(value):.1f}"
                    for value in self.gazebo_truth_angular_velocity
                )
                + ")"
            )
            self.get_logger().info(
                "STATE "
                f"arm={self.status.arming_state} nav={self.status.nav_state} "
                f"NED=({self.local.x:.3f},{self.local.y:.3f},{self.local.z:.3f}) "
                f"vel=({self.local.vx:.3f},{self.local.vy:.3f},{self.local.vz:.3f}) "
                f"{truth_report} {truth_velocity_report} "
                f"{truth_attitude_report} {truth_rate_report} "
                f"control={self.control_state.value} "
                f"velocity_key={self.active_velocity_key or 'ZERO'} "
                f"velocity_sp=({self.velocity_command_ned[0]:.3f},"
                f"{self.velocity_command_ned[1]:.3f},"
                f"{self.velocity_command_ned[2]:.3f}) "
                f"acceleration_sp=({self.acceleration_command_ned[0]:.3f},"
                f"{self.acceleration_command_ned[1]:.3f},"
                f"{self.acceleration_command_ned[2]:.3f}) "
                f"yaw_deg={math.degrees(self.local.heading):.1f} "
                f"rpy_deg=({math.degrees(attitude_rpy[0]):.1f},"
                f"{math.degrees(attitude_rpy[1]):.1f},"
                f"{math.degrees(attitude_rpy[2]):.1f}) "
                f"body_rate_deg_s=({math.degrees(angular_velocity[0]):.1f},"
                f"{math.degrees(angular_velocity[1]):.1f},"
                f"{math.degrees(angular_velocity[2]):.1f}) "
                f"yaw_rate_sp_deg_s={math.degrees(self.yaw_rate_command):.1f} "
                f"failsafe={self.status.failsafe} "
                f"age_s=({now - self.last_status_monotonic:.2f},"
                f"{now - self.last_local_monotonic:.2f}) "
                f"arm_torque_nm={np.linalg.norm(self.arm_reaction_torque_body_nm):.3f} "
                f"arm_force_n={self.arm_reaction_force_norm_n:.6f} "
                f"arm_com_shift_m={self.arm_com_shift_norm_m:.6f} "
                "arm_inertia_diag=("
                + ",".join(f"{value:.9f}" for value in self.arm_inertia_diag_kg_m2)
                + ") "
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
            self.control_state = FlightControlState.DISARMED
            self._clear_velocity_command()
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

        # The canted eight-rotor debug allocator can retain an asymmetric yaw
        # command as soon as native AUTO LAND takes over at touchdown.  The
        # small temporary pads then see a large yaw impulse before PX4's land
        # detector reaches minimum thrust.  For the isolated debug profile
        # only, finish the already active LAND sequence after the measured
        # vehicle has stayed at the recorded ground height with low 3-D
        # velocity.  The force token is intentionally behind this physical
        # touchdown gate; formal flight keeps the entire workaround disabled.
        if (
            self.TOUCHDOWN_DISARM_ENABLED
            and self.landing_requested
            and not self.touchdown_disarm_sent
            and self.status
            and self.local
            and self.status.arming_state == VehicleStatus.ARMING_STATE_ARMED
            and self.takeoff_ground_down is not None
        ):
            touchdown_stable = (
                abs(float(self.local.z) - self.takeoff_ground_down)
                <= self.TOUCHDOWN_DISARM_HEIGHT_M
                and math.hypot(float(self.local.vx), float(self.local.vy)) < 0.25
                and abs(float(self.local.vz)) < 0.20
            )
            if touchdown_stable:
                if self.touchdown_stable_since is None:
                    self.touchdown_stable_since = now
                elif now - self.touchdown_stable_since >= self.TOUCHDOWN_DISARM_HOLD_S:
                    if not self.touchdown_land_command_sent:
                        self.publish_command(VehicleCommand.VEHICLE_CMD_NAV_LAND)
                        self.touchdown_land_command_sent = now
                        self.get_logger().warning(
                            "PX4_TOUCHDOWN_LAND_SENT after stable ground-height hold"
                        )
                    elif now - self.touchdown_land_command_sent >= 0.30:
                        self.publish_command(
                            VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM,
                            param1=0.0,
                            param2=21196.0,
                        )
                        self.touchdown_disarm_sent = True
                        self.offboard_landing_active = False
                        self.offboard_requested = False
                        self.get_logger().warning(
                            "PX4_TOUCHDOWN_FORCE_DISARM_SENT after stable "
                            "ground-height hold"
                        )
            else:
                self.touchdown_stable_since = None

        if self.pending_takeoff and not self.offboard_requested:
            if self.state_fresh() and self.status and self.status.pre_flight_checks_pass:
                if not self.status.failsafe and self.begin_takeoff():
                    self.pending_takeoff = False

        if not self.offboard_requested:
            return
        # Once airborne, the high-rate local position is the control-critical
        # watchdog. vehicle_status is deliberately low rate and can be delayed
        # by slow Gazebo lockstep; PX4 itself still owns native Offboard and
        # failsafe supervision, so a stale status report must not create a
        # false loss of thrust while position/attitude data remain healthy.
        local_timeout_s = (
            self.LANDING_LOCAL_POSITION_TIMEOUT_S
            if self.landing_requested or self.offboard_landing_active
            else self.LOCAL_POSITION_TIMEOUT_S
        )
        if not self.local_position_fresh(local_timeout_s):
            self.get_logger().error("PX4 local-position timeout: stopping Offboard stream")
            self.offboard_requested = False
            return
        if not self.status_fresh() and not self.status_stale_reported:
            self.get_logger().warning(
                "PX4 status report is delayed; continuing with fresh local position"
            )
            self.status_stale_reported = True

        if (
            self.prestream_started
            and self.status.arming_state == VehicleStatus.ARMING_STATE_DISARMED
        ):
            self._refresh_disarmed_takeoff_target()

        if (
            self.offboard_landing_active
            and not self.TOUCHDOWN_DISARM_ENABLED
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
        if (
            not self.landing_requested
            and self.arm_motion_is_active(now)
            and reaction_torque_safety_triggered(
                self.arm_reaction_torque_body_nm,
                torque_age,
                self.arm_torque_abort_nm,
                self.status.arming_state == VehicleStatus.ARMING_STATE_ARMED,
            )
        ):
            self.get_logger().error(
                "Arm reaction-torque safety gate exceeded; requesting LAND "
                f"norm={torque_norm:.3f} N m limit={self.arm_torque_abort_nm:.3f}"
            )
            # Keep the still-controllable vehicle in Offboard while descending
            # to the recorded ground height.  A direct high-altitude NAV_LAND
            # handoff can excite the near-thrust-limit canted airframe and turn
            # a bounded safety abort into a much larger horizontal excursion.
            self.land(staged=True)
            return

        horizontal_error = math.hypot(
            self.local.x - self.target.north,
            self.local.y - self.target.east,
        )
        vertical_error = abs(self.local.z - self.target.down)
        if (
            not self.landing_requested
            and self.control_state != FlightControlState.LANDING
            and self.status.arming_state == VehicleStatus.ARMING_STATE_ARMED
            and self.control_state != FlightControlState.VELOCITY_CONTROL
            and (
                horizontal_error > self.MAX_POSITION_ERROR_M
                or vertical_error > self.MAX_VERTICAL_ERROR_M
            )
        ):
            self.get_logger().error(
                "Position safety gate exceeded; requesting LAND "
                f"horizontal={horizontal_error:.2f} m vertical={vertical_error:.2f} m"
            )
            self.land(staged=True)
            return

        self.update_velocity_control(now, dt)
        self.complete_hover_transition_if_ready()
        if self.control_state == FlightControlState.VELOCITY_CONTROL:
            self.publish_velocity()
        else:
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
        if age >= self.ACTUATOR_OUTPUT_TIMEOUT_S:
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
        "DDS WASD: T takeoff | W/S/A/D move | R/F up/down | Q/E yaw | H hover | "
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
