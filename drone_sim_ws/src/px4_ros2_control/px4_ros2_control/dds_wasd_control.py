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
import threading
import time
import tty
import uuid
from dataclasses import dataclass
from enum import Enum
from typing import Optional

import numpy as np
import rclpy
from geometry_msgs.msg import AccelStamped, WrenchStamped
from nav_msgs.msg import Odometry as GazeboOdometry
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor
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


class TrajectorySetpointKeepalive:
    """Republish the latest authoritative PX4 setpoint on an isolated thread.

    The 20 Hz flight callback computes the command and remains its sole
    authority.  This helper only bridges short executor stalls: it refreshes
    the timestamp and republishes the latest value for a bounded source lease.
    One lock serialises both authoritative and repeated publications through
    the same DDS writer, so an old mixed-XY sample cannot overtake a newer
    finite-XY exit sample.
    """

    def __init__(
        self,
        publisher,
        *,
        active_callback,
        mixed_xy_repeat_callback=None,
        period_s: float = 0.02,
        maximum_source_age_s: float = 0.18,
        monotonic_callback=time.monotonic,
        timestamp_callback=None,
    ) -> None:
        self._publisher = publisher
        self._active_callback = active_callback
        self._mixed_xy_repeat_callback = mixed_xy_repeat_callback
        self._period_s = max(0.005, float(period_s))
        # This lease is deliberately below the unchanged 200 ms physical
        # direct-force lease and the 250 ms main-intent deadline.
        self._maximum_source_age_s = min(
            0.18, max(self._period_s, float(maximum_source_age_s))
        )
        self._monotonic = monotonic_callback
        self._timestamp_callback = timestamp_callback
        self._lock = threading.Lock()
        self._latest = None
        self._latest_source_monotonic = 0.0
        self._last_wire_monotonic = 0.0
        self._last_wire_timestamp_us = 0
        self._stop = threading.Event()
        self._thread = None

    @staticmethod
    def _clone(message: TrajectorySetpoint) -> TrajectorySetpoint:
        clone = TrajectorySetpoint()
        clone.timestamp = int(message.timestamp)
        clone.position = list(message.position)
        clone.velocity = list(message.velocity)
        clone.acceleration = list(message.acceleration)
        clone.jerk = list(message.jerk)
        clone.yaw = float(message.yaw)
        clone.yawspeed = float(message.yawspeed)
        return clone

    @staticmethod
    def _is_mixed_xy(message: TrajectorySetpoint) -> bool:
        return bool(
            not math.isfinite(float(message.position[0]))
            and not math.isfinite(float(message.position[1]))
            and not math.isfinite(float(message.velocity[0]))
            and not math.isfinite(float(message.velocity[1]))
            and math.isfinite(float(message.acceleration[0]))
            and math.isfinite(float(message.acceleration[1]))
        )

    def publish(self, message: TrajectorySetpoint) -> None:
        """Publish and atomically replace the command eligible for repeats."""
        now = self._monotonic()
        with self._lock:
            # The flight callback samples its timestamp before it waits for
            # this wire lock.  A repeat may therefore already have published
            # a newer timestamp.  Clamp here so the same DDS writer never
            # emits time backwards even when the command transition itself is
            # correctly ordered.
            message.timestamp = max(
                int(message.timestamp), self._last_wire_timestamp_us + 1
            )
            self._publisher.publish(message)
            self._latest = self._clone(message)
            self._latest_source_monotonic = now
            self._last_wire_monotonic = now
            self._last_wire_timestamp_us = int(message.timestamp)

    def pump_once(self, now: Optional[float] = None) -> bool:
        """Publish one bounded repeat; exposed for deterministic testing."""
        sample_time = self._monotonic() if now is None else float(now)
        repeated_mixed_xy = False
        with self._lock:
            source_age = sample_time - self._latest_source_monotonic
            wire_age = sample_time - self._last_wire_monotonic
            if (
                self._latest is None
                or not self._active_callback()
                or source_age < 0.0
                or source_age >= self._maximum_source_age_s
                or wire_age < 0.75 * self._period_s
            ):
                return False
            repeated = self._clone(self._latest)
            repeated.timestamp = max(
                self._last_wire_timestamp_us + 1,
                int(
                    self._timestamp_callback()
                    if self._timestamp_callback is not None
                    else self._latest.timestamp
                    + round(source_age * 1_000_000.0)
                ),
            )
            self._publisher.publish(repeated)
            self._last_wire_monotonic = sample_time
            self._last_wire_timestamp_us = int(repeated.timestamp)
            repeated_mixed_xy = self._is_mixed_xy(repeated)
        # Never invoke controller bookkeeping while holding the wire-order
        # lock.  The callback takes the independent force-lease lock.
        if repeated_mixed_xy and self._mixed_xy_repeat_callback is not None:
            self._mixed_xy_repeat_callback(sample_time)
        return True

    def _run(self) -> None:
        while not self._stop.wait(self._period_s):
            self.pump_once()

    def start(self) -> None:
        if self._thread is not None:
            return
        self._thread = threading.Thread(
            target=self._run,
            name="px4-setpoint-keepalive",
            daemon=True,
        )
        self._thread.start()

    def stop(self) -> None:
        self._stop.set()
        if self._thread is not None:
            self._thread.join(timeout=1.0)


class DirectXySafetyIngress(Node):
    """Receive ownership safety levels outside the busy flight executor.

    The main controller consumes several high-rate PX4 and Gazebo streams.  A
    dedicated node/executor prevents those streams from delaying the two
    fail-closed ownership subscriptions past the 40 ms watchdog.  The target
    callbacks parse each report once, update short latest-value caches, and
    publish at most one receipt ACK per ownership epoch.  They never enter the
    flight controller's ``control_lock`` while caching.  A prepared guardian
    level then makes one non-blocking attempt to commit that latest level and
    start the handoff immediately; it never queues behind flight work.
    """

    def __init__(self, controller) -> None:
        super().__init__("my_drone_dds_wasd_safety_ingress")
        self.controller = controller
        self.last_active_receipt_ack_epoch = -1
        # Reports and the lease heartbeat have distinct scheduling needs.
        # Keeping both in the default callback group on a single-threaded
        # executor allowed the always-ready 10 ms timer to delay an active
        # reallocator report for the complete 150 ms owner-entry budget.  Two
        # mutually-exclusive groups preserve ordering within each stream while
        # allowing the report ACK and lease refresh to make progress in
        # parallel.  The controller's lease lock remains the sole serialization
        # point for force-command publication.
        self.report_callback_group = MutuallyExclusiveCallbackGroup()
        self.lease_callback_group = MutuallyExclusiveCallbackGroup()
        # Both streams are replaceable safety levels, not event journals.
        # RELIABLE/KEEP_LAST(1) preserves the newest ownership level without
        # accumulating obsolete heartbeats behind a delayed callback.
        latest_safety_level_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self.create_subscription(
            Bool,
            "/my_drone/arm_motion_active",
            self._arm_motion_cb,
            1,
            callback_group=self.report_callback_group,
        )
        if controller.ARM_DIRECT_XY_EXTERNAL_GUARDIAN:
            # The process-isolated guardian is the sole force/state writer.
            # This process only caches its owner level; it does not subscribe
            # to the physical producer or run a lease publication timer.
            self.create_subscription(
                String,
                "/my_drone/arm_direct_xy_guardian/state",
                self._guardian_state_cb,
                latest_safety_level_qos,
                callback_group=self.report_callback_group,
            )
            return
        # This publisher carries receipt-time evidence that the reallocator is
        # already applying the requested epoch.  It avoids waiting for a busy
        # flight-control critical section merely to acknowledge a physical
        # ownership state that the dedicated executor has already observed.
        self.owner_state_pub = self.create_publisher(
            String, "/my_drone/arm_direct_xy_state", latest_safety_level_qos
        )
        # This is the sole writer for both enable and disable lease commands.
        # Main-control code only changes the atomic lease intent after it has
        # published the corresponding PX4 setpoint.  Serialising every publish
        # through the lease lock prevents an in-flight refresh from landing
        # after the finite-XY exit disable.
        self.force_command_pub = self.create_publisher(
            String,
            "/my_drone/arm_direct_xy_force_command",
            latest_safety_level_qos,
        )
        controller._attach_arm_direct_xy_force_lease_publisher(
            self.force_command_pub
        )
        self.create_subscription(
            String,
            "/my_drone/base1_reallocator/state",
            self._reallocator_state_cb,
            latest_safety_level_qos,
            callback_group=self.report_callback_group,
        )
        self.force_lease_timer = self.create_timer(
            min(0.01, 0.5 * controller.ARM_DIRECT_XY_WATCHDOG_S),
            self._refresh_force_lease,
            callback_group=self.lease_callback_group,
        )

    def _guardian_state_cb(self, msg: String) -> None:
        self.controller._locked_external_guardian_state_cb(msg)

    def _arm_motion_cb(self, msg: Bool) -> None:
        # Runtime ingress only records the level.  The 20 Hz control tick owns
        # all state transitions; executing them here can occupy this ordered
        # report callback long enough to hide the following physical ACK.
        self.controller._cache_arm_motion_active_cb(msg)

    def _reallocator_state_cb(self, msg: String) -> None:
        # Parse once so ACK, lease and cache judge the exact same producer and
        # receipt timestamps.  Repeated JSON work and repeated reliable ACK
        # publication previously made this 100 Hz safety callback vulnerable
        # to DDS backpressure.
        snapshot = self.controller._snapshot_arm_direct_xy_reallocator_state(msg)
        if snapshot is None:
            return
        # Physical receipt bookkeeping is safety-critical and must complete
        # before any reliable diagnostic/owner-state publication can block.
        self._observe_force_lease_snapshot(snapshot)
        self.controller._cache_arm_direct_xy_reallocator_snapshot(snapshot)
        self._publish_active_receipt_ack_snapshot(snapshot)

    def _observe_force_lease_report(self, msg: String) -> None:
        snapshot = self.controller._snapshot_arm_direct_xy_reallocator_state(msg)
        if snapshot is None:
            return
        self._observe_force_lease_snapshot(snapshot)

    def _observe_force_lease_snapshot(
        self, snapshot: tuple[float, float, dict]
    ) -> None:
        _producer_monotonic, received_monotonic, report = snapshot
        self.controller._observe_arm_direct_xy_force_lease_report(
            report, received_monotonic=received_monotonic
        )

    def _refresh_force_lease(self) -> None:
        # The controller samples the clock only after taking the lease lock.
        # Sampling here can race a newer mixed-setpoint timestamp written by
        # the control thread and turn an otherwise healthy lease age negative.
        self.controller._refresh_arm_direct_xy_force_lease()

    def _publish_active_receipt_ack(self, msg: String) -> None:
        snapshot = self.controller._snapshot_arm_direct_xy_reallocator_state(msg)
        if snapshot is None:
            return
        self._publish_active_receipt_ack_snapshot(snapshot)

    def _publish_active_receipt_ack_snapshot(
        self, snapshot: tuple[float, float, dict]
    ) -> None:
        controller = self.controller
        if not controller.ARM_DIRECT_XY_OWNERSHIP:
            return
        producer_monotonic, received_monotonic, report = snapshot
        try:
            force_epoch = int(report["direct_xy_force_epoch"])
        except (KeyError, TypeError, ValueError):
            return
        producer_age = received_monotonic - producer_monotonic
        actual_active = bool(
            0.0 <= producer_age < controller.ARM_DIRECT_XY_WATCHDOG_S
            and controller._direct_xy_reallocator_report_is_healthy(report)
            and report.get("event") == "allocated"
            and report.get("motion_active") is True
            and report.get("position_feedback_prepared") is True
            and report.get("position_target_latched") is True
            and report.get("position_feedback_active") is True
            and report.get("direct_xy_force_command_fresh") is True
            and report.get("direct_xy_force_enabled") is True
            and force_epoch == controller.arm_direct_xy_ownership_epoch
            and controller.arm_direct_xy_active
            and not controller.arm_direct_xy_abort_latched
            and controller.arm_motion_is_active(received_monotonic)
        )
        if not actual_active:
            return
        if force_epoch == self.last_active_receipt_ack_epoch:
            return
        state = {
            "schema": "my_drone.arm-direct-xy-state.v1",
            "ownership_epoch": force_epoch,
            "state": "direct_xy",
            "watchdog_reason": "",
            "watchdog_detail": "",
            "reallocator_fresh": True,
            "position_feedback_active": True,
            "position_feedback_prepared": True,
            "position_feedback_ready": True,
            "motion_active": True,
            "direct_xy_force_enabled_ack": True,
            "direct_xy_force_epoch_ack": force_epoch,
            "ack_source": "dedicated_safety_ingress",
        }
        self.owner_state_pub.publish(String(data=json.dumps(state, sort_keys=True)))
        # Record only after publish returns; a publisher exception must leave
        # the epoch eligible for retry on the next healthy report.
        self.last_active_receipt_ack_epoch = force_epoch


def wrap_pi(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


def quaternion_wxyz_to_rotation_body_to_world(values) -> np.ndarray:
    """Return the body-FLU to world-ENU rotation for a wxyz quaternion."""
    quaternion = np.asarray(values, dtype=float)
    if quaternion.shape != (4,) or not np.all(np.isfinite(quaternion)):
        raise ValueError("quaternion must contain four finite values")
    norm = float(np.linalg.norm(quaternion))
    if norm <= 0.0:
        raise ValueError("quaternion norm must be positive")
    w, x, y, z = quaternion / norm
    return np.asarray(
        [
            [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
        ],
        dtype=float,
    )


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

    def read_key(self, timeout_sec: float = 0.0) -> Optional[str]:
        if select.select([sys.stdin], [], [], max(0.0, float(timeout_sec)))[0]:
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
        # coupling transient. A later clean full-sequence run still reached
        # 17.1 deg/s with the 15 deg/s target, so use 15 deg/s^2 to retain the
        # target rate while reducing the reversal transient.
        float(os.environ.get("PX4_WASD_YAW_ACCEL_DEG_S2", "15.0"))
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
    # Some Gazebo-only profiles intentionally rest on an arm/gripper contact
    # whose height differs slightly from the pose recorded immediately before
    # takeoff.  Keep the formal default at zero and let only those profiles add
    # their measured contact-geometry allowance to the touchdown gate.
    TOUCHDOWN_CONTACT_ALLOWANCE_M = float(
        os.environ.get("PX4_TOUCHDOWN_CONTACT_ALLOWANCE_M", "0.0")
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
    # PX4 already closes the inner velocity loop.  Adding a second derivative
    # term around Gazebo truth duplicated velocity damping and produced a
    # visible fore/aft limit cycle while the arm retracted.  Clean A/B flight
    # on the 4 kg aircraft measured D=0.25 at 12 mm / 0.7 deg/s, D=0.40
    # diverging to 228 mm / 5.3 deg, and the cascaded P-only outer loop at
    # 6 mm / 0.1 deg/s.  Keep an environment override for experiments, but
    # make the standard cascaded position-P -> PX4-velocity-loop structure the
    # safe default.
    TRUTH_HOLD_XY_D = float(os.environ.get("PX4_TRUTH_HOLD_XY_D", "0.0"))
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
    TRUTH_HOLD_ARM_POSITION_OVERLAY = os.environ.get(
        "PX4_TRUTH_HOLD_ARM_POSITION_OVERLAY", "false"
    ).lower() in {"1", "true", "yes", "on"}
    TRUTH_HOLD_POSITION_GAIN = float(
        os.environ.get("PX4_TRUTH_HOLD_POSITION_GAIN", "1.0")
    )
    TRUTH_HOLD_POSITION_XY_MAX_M = float(
        os.environ.get("PX4_TRUTH_HOLD_POSITION_XY_MAX_M", "0.15")
    )
    TRUTH_HOLD_POSITION_Z_MAX_M = float(
        os.environ.get("PX4_TRUTH_HOLD_POSITION_Z_MAX_M", "0.12")
    )
    # Experimental mixed-axis ownership for the light 1.3 kg candidate only.
    # When enabled and all gates are healthy, PX4 owns Z velocity, yaw,
    # attitude and body rates while the canted-rotor reallocator is the sole
    # world-XY position outer loop.  The formal/Base1 default stays disabled.
    ARM_DIRECT_XY_OWNERSHIP = os.environ.get(
        "ARM_DIRECT_XY_OWNERSHIP", "false"
    ).lower() in {"1", "true", "yes", "on"}
    ARM_DIRECT_XY_EXTERNAL_GUARDIAN = os.environ.get(
        "ARM_DIRECT_XY_EXTERNAL_GUARDIAN", "false"
    ).lower() in {"1", "true", "yes", "on"}
    ARM_DIRECT_XY_WATCHDOG_S = min(
        0.04, max(0.02, float(os.environ.get("ARM_DIRECT_XY_WATCHDOG_S", "0.04")))
    )
    # The independent guardian owns the physical producer's strict 40 ms
    # watchdog.  This process only monitors delivery of the guardian's already
    # adjudicated owner level.  Keep that transport deadline inside both the
    # 200 ms physical force lease and PX4's 500 ms Offboard-loss boundary; a
    # 20 Hz control tick then restores finite XY no later than 200 ms after a
    # dead guardian.  This is not a relaxation of the physical 40 ms gate.
    ARM_DIRECT_XY_GUARDIAN_STATE_TIMEOUT_S = min(
        0.15,
        max(
            0.04,
            float(os.environ.get("ARM_DIRECT_XY_GUARDIAN_STATE_TIMEOUT_S", "0.15")),
        ),
    )
    ARM_DIRECT_XY_HEALTH_HOLD_S = max(
        0.0, float(os.environ.get("ARM_DIRECT_XY_HEALTH_HOLD_S", "0.25"))
    )
    ARM_DIRECT_XY_STABLE_HOLD_S = max(
        0.0, float(os.environ.get("ARM_DIRECT_XY_STABLE_HOLD_S", "0.50"))
    )
    # The arm-side ownership gate already defines a 150 ms entry budget.  Use
    # the same bounded window for both the prepared hand-off and the physical
    # force-owner acknowledgement.  The previous 100 ms controller deadline
    # could reject a valid DDS acknowledgement at ~110 ms even though it was
    # still inside the authoritative 150 ms owner-entry contract.  Runtime
    # freshness remains governed by the independent 40 ms watchdog below.
    ARM_DIRECT_XY_ENTRY_TIMEOUT_S = min(
        0.15,
        max(0.02, float(os.environ.get("ARM_DIRECT_XY_ENTRY_TIMEOUT_S", "0.15"))),
    )
    # A dedicated lease refresher may bridge a busy main callback, but it must
    # never keep the horizontal owner alive indefinitely after PX4 mixed-axis
    # setpoints stop.  This is intentionally below PX4's 0.5 s Offboard loss
    # boundary and does not relax either the 40 ms owner watchdog or the
    # reallocator's independent 0.20 s force-command timeout.
    ARM_DIRECT_XY_MAIN_SETPOINT_LEASE_S = 0.25
    # Keep the 10 ms timer for bootstrap, then reduce steady-state DDS load.
    # Eight nominal refresh opportunities still fit inside the unchanged
    # 0.20 s physical lease.
    ARM_DIRECT_XY_FORCE_REFRESH_S = 0.025
    ARM_DIRECT_XY_ENTRY_XY_ERROR_M = max(
        0.0, float(os.environ.get("ARM_DIRECT_XY_ENTRY_XY_ERROR_M", "0.05"))
    )
    ARM_DIRECT_XY_ENTRY_XY_SPEED_M_S = max(
        0.0, float(os.environ.get("ARM_DIRECT_XY_ENTRY_XY_SPEED_M_S", "0.08"))
    )
    ARM_DIRECT_XY_ENTRY_TILT_RAD = math.radians(
        max(0.0, float(os.environ.get("ARM_DIRECT_XY_ENTRY_TILT_DEG", "1.0")))
    )

    def __init__(self, arm_only: bool = False) -> None:
        super().__init__("my_drone_dds_wasd_control")
        # The terminal runs in the main thread.  ROS callbacks run continuously
        # on a background MultiThreadedExecutor so keyboard polling cannot starve
        # the 40 ms direct-XY safety heartbeat.  Separate callback groups keep
        # the reallocator subscription eligible independently of the 20 Hz
        # control timer; the lock makes their multi-step ownership transitions
        # atomic with operator commands.
        self.control_lock = threading.RLock()
        self.control_timer_group = MutuallyExclusiveCallbackGroup()
        self.diagnostic_timer_group = MutuallyExclusiveCallbackGroup()
        # A motion ownership edge must not occupy an executor worker while it
        # waits for the comparatively long control callback.  Cache only the
        # newest level under this short independent lock.
        self.arm_motion_report_cache_lock = threading.Lock()
        self.arm_motion_pending_snapshot = None
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
        self.last_joint_state_monotonic = 0.0
        self.arm_direct_xy_reallocator_healthy = False
        self.arm_direct_xy_position_feedback_active = False
        self.arm_direct_xy_position_feedback_prepared = False
        self.arm_direct_xy_position_feedback_ready = False
        self.arm_direct_xy_force_enabled_ack = False
        self.arm_direct_xy_force_epoch_ack = -1
        self.arm_direct_xy_force_commanded = False
        # Direct-force command publication is owned by DirectXySafetyIngress.
        # The main controller only mutates this compact lease intent after an
        # actual PX4 setpoint publication.  The independent lock must never be
        # replaced by control_lock: its purpose is to remain available while a
        # long main callback holds that lock.
        self.arm_direct_xy_force_lease_lock = threading.Lock()
        # Serialise motion edges with external guardian intent snapshots.
        # In particular, motion=False, finite PX4 XY publication and the
        # following exit intent form one atomic transition relative to the
        # isolated setpoint keepalive thread.  Without this lock a repeated
        # mixed setpoint could publish force_requested=True together with the
        # newly visible motion_active=False level.
        self.arm_direct_xy_intent_transition_lock = threading.RLock()
        # DDS publication can block behind a reliable reader.  Keep wire-order
        # serialization separate so healthy reallocator receipts remain able
        # to update the unchanged 40 ms watchdog while a publish is pending.
        self.arm_direct_xy_force_publish_lock = threading.Lock()
        self.arm_direct_xy_force_lease_publisher = None
        self.arm_direct_xy_force_lease_generation = 0
        self.arm_direct_xy_force_lease_requested = False
        self.arm_direct_xy_force_lease_epoch = -1
        self.arm_direct_xy_force_lease_mixed_setpoint_monotonic = 0.0
        self.arm_direct_xy_force_lease_first_active_receipt_monotonic = 0.0
        self.arm_direct_xy_force_lease_active_receipt_monotonic = 0.0
        self.arm_direct_xy_force_lease_active_report_epoch = -1
        self.arm_direct_xy_force_lease_latest_report_producer_monotonic = 0.0
        self.arm_direct_xy_force_lease_active_seen = False
        self.arm_direct_xy_force_ever_ack = False
        self.arm_direct_xy_exit_disable_pending = False
        self.arm_direct_xy_health_since = 0.0
        self.last_arm_direct_xy_state_monotonic = 0.0
        self.arm_direct_xy_stable_since = 0.0
        self.arm_direct_xy_active = False
        self.arm_direct_xy_entry_deadline = 0.0
        self.arm_direct_xy_prepared_monotonic = 0.0
        self.arm_direct_xy_force_ack_deadline = 0.0
        self.arm_direct_xy_force_ack_monotonic = 0.0
        self.last_arm_direct_xy_report_producer_monotonic = 0.0
        # Reallocator reports arrive on a background executor thread.  Never
        # let that thread take a DDS sample and then wait behind the relatively
        # long control callback: doing so ages an otherwise current sample past
        # the 40 ms watchdog before it is validated.  The callback publishes a
        # parsed latest-value snapshot here and only enters ``control_lock``
        # when it can do so immediately.  A control tick consumes any pending
        # snapshot at a deterministic point before evaluating ownership.
        self.arm_direct_xy_report_cache_lock = threading.Lock()
        self.arm_direct_xy_pending_report_snapshot = None
        self.arm_direct_xy_pending_fault_snapshot = None
        self.latest_arm_direct_xy_report_queued_producer_monotonic = 0.0
        # Receipt freshness is a safety heartbeat, not a control-state
        # transition.  Update it in the short cache critical section so a
        # healthy 50 Hz report cannot become stale merely because the main
        # controller is still completing its current tick.
        self.last_arm_direct_xy_healthy_receipt_monotonic = 0.0
        self.arm_direct_xy_guardian_cache_lock = threading.Lock()
        self.arm_direct_xy_guardian_pending_snapshot = None
        self.arm_direct_xy_guardian_session_id = uuid.uuid4().hex
        self.arm_direct_xy_abort_latched = False
        self.arm_direct_xy_last_fault = ""
        self.arm_direct_xy_last_fault_detail = ""
        self.arm_direct_xy_ownership_epoch = 0
        self.arm_direct_xy_preauthorized = False
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
        latest_safety_level_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self.mode_pub = self.create_publisher(
            OffboardControlMode, "/fmu/in/offboard_control_mode", 10
        )
        raw_setpoint_pub = self.create_publisher(
            TrajectorySetpoint, "/fmu/in/trajectory_setpoint", 10
        )
        # PX4's setpoint stream must not inherit occasional 100+ ms stalls
        # from arm/DDS callbacks.  The command itself is still calculated by
        # the normal flight tick; this bounded repeater merely maintains a
        # 50 Hz wire cadence through short scheduler stalls.
        self.setpoint_pub = TrajectorySetpointKeepalive(
            raw_setpoint_pub,
            active_callback=lambda: bool(self.offboard_requested),
            mixed_xy_repeat_callback=(
                self._note_arm_direct_xy_mixed_setpoint_published
            ),
            period_s=0.02,
            maximum_source_age_s=0.18,
            timestamp_callback=self.now_us,
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
        self.arm_direct_xy_ready_pub = self.create_publisher(
            Bool, "/my_drone/arm_direct_xy_ready", 10
        )
        self.arm_motion_inhibit_pub = self.create_publisher(
            Bool, "/my_drone/arm_motion_inhibit", 10
        )
        self.arm_direct_xy_state_pub = self.create_publisher(
            String, "/my_drone/arm_direct_xy_state", latest_safety_level_qos
        )
        self.arm_direct_xy_intent_pub = self.create_publisher(
            String,
            "/my_drone/arm_direct_xy_controller_intent",
            latest_safety_level_qos,
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
            Float32MultiArray,
            "/my_drone/rl_action",
            self._locked_rl_action_cb,
            10,
            callback_group=self.control_timer_group,
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
        # The arm-motion and reallocator ownership subscriptions live on
        # ``DirectXySafetyIngress`` and its dedicated executor.  Keeping them
        # off this node is what prevents the main PX4/Gazebo callback queue
        # from starving their 40 ms receipt heartbeat.
        self.create_subscription(
            Bool,
            "/my_drone/hover_request",
            self._locked_hover_request_cb,
            10,
            callback_group=self.control_timer_group,
        )
        self.timer = self.create_timer(
            1.0 / self.RATE_HZ,
            self._locked_tick,
            callback_group=self.control_timer_group,
        )
        self.state_report_timer = self.create_timer(
            1.0 / max(0.2, min(20.0, self.STATE_REPORT_HZ)),
            self._report_state,
            callback_group=self.diagnostic_timer_group,
        )
        if self.arm_feedforward_enabled:
            self.get_logger().info(
                "ARM_FEEDFORWARD enabled; reading bounded NED acceleration"
            )
        if self.ARM_DIRECT_XY_OWNERSHIP:
            self.get_logger().warning(
                "ARM_DIRECT_XY_OWNERSHIP enabled: PX4 XY position/velocity "
                "setpoints will become NaN only inside the gated arm-motion window"
            )
        # Start only after every controller field used by the activity and
        # mixed-owner callbacks has been initialised.
        self.setpoint_pub.start()

    def _locked_tick(self) -> None:
        with self.control_lock:
            self._consume_latest_arm_motion_snapshot()
            self._consume_external_guardian_state()
            self._consume_latest_arm_direct_xy_report_snapshot()
            self._tick()

    def stop_setpoint_keepalive(self) -> None:
        publisher = getattr(self, "setpoint_pub", None)
        if isinstance(publisher, TrajectorySetpointKeepalive):
            publisher.stop()

    def _cache_arm_motion_active_cb(self, msg: Bool) -> None:
        """Record a motion level without ever entering ``control_lock``."""
        snapshot = (time.monotonic(), bool(msg.data))
        with self.arm_motion_report_cache_lock:
            self.arm_motion_pending_snapshot = snapshot

    def _cache_external_guardian_state(self, msg: String) -> None:
        """Cache the independent process' authoritative owner level only."""
        if not self.ARM_DIRECT_XY_EXTERNAL_GUARDIAN:
            return
        received = time.monotonic()
        try:
            report = json.loads(msg.data)
            if report.get("schema") != "my_drone.arm-direct-xy-state.v1":
                return
            if str(report["controller_session_id"]) != self.arm_direct_xy_guardian_session_id:
                return
            int(report["ownership_epoch"])
            int(report["lease_generation"])
        except (KeyError, TypeError, ValueError, json.JSONDecodeError):
            return
        with self.arm_direct_xy_guardian_cache_lock:
            self.arm_direct_xy_guardian_pending_snapshot = (received, report)

    def _locked_external_guardian_state_cb(self, msg: String) -> None:
        """Commit a prepared guardian level immediately when control is free.

        The 20 Hz flight tick remains the fallback consumer and still owns
        every transition under ``control_lock``.  Waiting exclusively for
        that tick added up to 78.6 ms between a fresh prepared level and the
        mixed-setpoint handoff in v12.  This event path never waits for the
        lock, so it cannot stall the dedicated guardian ingress or hide the
        following physical ACK; a busy tick simply leaves the depth-one
        snapshot for the ordinary deterministic consume point.
        """
        self._cache_external_guardian_state(msg)
        if not self.control_lock.acquire(blocking=False):
            return
        try:
            # A motion rising edge and the prepared guardian report can arrive
            # on adjacent safety callbacks.  Establish the epoch first so the
            # exact session/epoch/generation checks below remain authoritative.
            self._consume_latest_arm_motion_snapshot()
            self._consume_external_guardian_state()
        finally:
            self.control_lock.release()

    def _consume_external_guardian_state(self) -> None:
        """Apply one guardian level under ``control_lock``."""
        if not self.ARM_DIRECT_XY_EXTERNAL_GUARDIAN:
            return
        with self.arm_direct_xy_guardian_cache_lock:
            snapshot = self.arm_direct_xy_guardian_pending_snapshot
            self.arm_direct_xy_guardian_pending_snapshot = None
        if snapshot is None:
            return
        received, report = snapshot
        epoch = int(report["ownership_epoch"])
        intent_generation = int(report["intent_generation"])
        self._ensure_arm_direct_xy_force_lease_state()
        with self.arm_direct_xy_force_lease_lock:
            expected_generation = int(self.arm_direct_xy_force_lease_generation)
        if (
            epoch != self.arm_direct_xy_ownership_epoch
            or intent_generation != expected_generation
        ):
            return
        state = str(report.get("state", "aborting"))
        healthy = bool(report.get("reallocator_fresh") is True)
        was_healthy = self.arm_direct_xy_reallocator_healthy
        self.arm_direct_xy_reallocator_healthy = healthy
        self.last_arm_direct_xy_state_monotonic = received
        if healthy:
            with self.arm_direct_xy_report_cache_lock:
                self.last_arm_direct_xy_healthy_receipt_monotonic = received
            if not was_healthy:
                self.arm_direct_xy_health_since = received
        else:
            self.arm_direct_xy_health_since = 0.0
        self.arm_direct_xy_position_feedback_prepared = bool(
            report.get("position_feedback_prepared") is True
        )
        if self.arm_direct_xy_position_feedback_prepared:
            if self.arm_direct_xy_prepared_monotonic <= 0.0:
                self.arm_direct_xy_prepared_monotonic = received
        else:
            self.arm_direct_xy_prepared_monotonic = 0.0
        self.arm_direct_xy_position_feedback_ready = bool(
            report.get("position_feedback_ready") is True
        )
        active = bool(
            state in {"direct_xy", "exit"}
            and epoch == self.arm_direct_xy_ownership_epoch
            and report.get("direct_xy_force_enabled_ack") is True
        )
        self.arm_direct_xy_position_feedback_active = active
        self.arm_direct_xy_force_enabled_ack = active
        self.arm_direct_xy_force_epoch_ack = epoch if active else -1
        if active:
            self.arm_direct_xy_force_ever_ack = True
            self.arm_direct_xy_force_ack_deadline = 0.0
            self.arm_direct_xy_force_ack_monotonic = received
        release_ack = bool(
            state == "px4_xy"
            and self.arm_direct_xy_exit_disable_pending
            and healthy
            and report.get("position_feedback_active") is False
            and report.get("direct_xy_force_enabled_ack") is False
        )
        if release_ack:
            # The guardian publishes px4_xy only after a fresh physical
            # disabled report matches this controller session, ownership
            # epoch and intent generation (all checked above).  Do not clear
            # exit pending on the earlier local False publication.
            self.arm_direct_xy_exit_disable_pending = False
            self.arm_direct_xy_force_enabled_ack = False
            self.arm_direct_xy_force_epoch_ack = -1
            self.arm_direct_xy_force_ever_ack = False
        if (
            state == "aborting"
            and self.arm_motion_is_active(received)
            and not self.arm_direct_xy_abort_latched
        ):
            self._abort_arm_for_direct_xy_fault(
                "guardian_abort", detail=str(report.get("watchdog_reason", ""))
            )
            self._restore_finite_xy_and_disable_direct_force()
            return
        if (
            healthy
            and self.arm_direct_xy_position_feedback_prepared
            and self.arm_motion_is_active(received)
            and not self.arm_direct_xy_active
            and not self.arm_direct_xy_abort_latched
        ):
            self._start_event_driven_direct_xy_handoff(received)

    def _locked_arm_motion_active_cb(self, msg: Bool) -> None:
        """Compatibility path that opportunistically commits cached motion."""
        self._cache_arm_motion_active_cb(msg)
        if not self.control_lock.acquire(blocking=False):
            return
        try:
            self._consume_latest_arm_motion_snapshot()
        finally:
            self.control_lock.release()

    def _consume_latest_arm_motion_snapshot(self) -> None:
        """Commit the latest arm ownership level under ``control_lock``."""
        cache_lock = getattr(self, "arm_motion_report_cache_lock", None)
        if cache_lock is None:
            return
        with cache_lock:
            snapshot = self.arm_motion_pending_snapshot
            self.arm_motion_pending_snapshot = None
        if snapshot is not None:
            received_monotonic, active = snapshot
            self._arm_motion_active_cb(
                Bool(data=active), observed_monotonic=received_monotonic
            )

    def _cache_arm_direct_xy_reallocator_state_cb(self, msg: String) -> None:
        """Record one reallocator report without running owner transitions."""
        snapshot = self._snapshot_arm_direct_xy_reallocator_state(msg)
        if snapshot is None:
            return
        self._cache_arm_direct_xy_reallocator_snapshot(snapshot)

    def _cache_arm_direct_xy_reallocator_snapshot(
        self, snapshot: tuple[float, float, dict]
    ) -> None:
        """Cache an already parsed report without running owner transitions."""
        producer_monotonic = snapshot[0]
        received_monotonic = snapshot[1]
        report = snapshot[2]
        received_healthy = bool(
            math.isfinite(producer_monotonic)
            and self._direct_xy_reallocator_report_is_healthy(report)
            and 0.0
            <= received_monotonic - producer_monotonic
            < self.ARM_DIRECT_XY_WATCHDOG_S
        )
        with self.arm_direct_xy_report_cache_lock:
            if (
                math.isfinite(producer_monotonic)
                and producer_monotonic
                <= self.latest_arm_direct_xy_report_queued_producer_monotonic
            ):
                return
            if math.isfinite(producer_monotonic):
                self.latest_arm_direct_xy_report_queued_producer_monotonic = (
                    producer_monotonic
                )
            pending = self.arm_direct_xy_pending_report_snapshot
            if received_healthy:
                self.last_arm_direct_xy_healthy_receipt_monotonic = max(
                    self.last_arm_direct_xy_healthy_receipt_monotonic,
                    received_monotonic,
                )
                if (
                    pending is None
                    or not math.isfinite(pending[0])
                    or producer_monotonic > pending[0]
                ):
                    self.arm_direct_xy_pending_report_snapshot = snapshot
            else:
                # A real producer/contract fault observed at callback receipt
                # is an event, not a replaceable level.  Keep it even if a
                # newer healthy level arrives before control_lock is free; an
                # active motion epoch must still fail closed.
                self.arm_direct_xy_pending_fault_snapshot = snapshot
                if (
                    pending is not None
                    and (
                        not math.isfinite(producer_monotonic)
                        or not math.isfinite(pending[0])
                        or pending[0] <= producer_monotonic
                    )
                ):
                    self.arm_direct_xy_pending_report_snapshot = None

    def _locked_arm_direct_xy_reallocator_state_cb(self, msg: String) -> None:
        """Compatibility path that may commit a cached report immediately."""
        self._cache_arm_direct_xy_reallocator_state_cb(msg)

        # Do not wait while holding the reallocator callback group.  If the
        # control path is busy, a later producer report may replace this cache
        # entry and the control path will consume that newest snapshot itself.
        if not self.control_lock.acquire(blocking=False):
            return
        try:
            # When both levels arrived while the control callback was busy,
            # establish the motion epoch before consuming its prepared/ACK
            # reallocator report.
            self._consume_latest_arm_motion_snapshot()
            self._consume_latest_arm_direct_xy_report_snapshot()
        finally:
            self.control_lock.release()

    def _consume_latest_arm_direct_xy_report_snapshot(self) -> None:
        """Commit the newest cached producer snapshot under ``control_lock``."""
        cache_lock = getattr(self, "arm_direct_xy_report_cache_lock", None)
        if cache_lock is None:
            return
        with cache_lock:
            fault_snapshot = self.arm_direct_xy_pending_fault_snapshot
            self.arm_direct_xy_pending_fault_snapshot = None
            snapshot = self.arm_direct_xy_pending_report_snapshot
            self.arm_direct_xy_pending_report_snapshot = None
        if fault_snapshot is not None:
            self._apply_arm_direct_xy_reallocator_snapshot(fault_snapshot)
        if snapshot is not None:
            self._apply_arm_direct_xy_reallocator_snapshot(snapshot)

    def _locked_rl_action_cb(self, msg: Float32MultiArray) -> None:
        with self.control_lock:
            self._rl_action_cb(msg)

    def _locked_hover_request_cb(self, msg: Bool) -> None:
        with self.control_lock:
            self._hover_request_cb(msg)

    def spin_callbacks(
        self, timeout_sec: float = 0.02, *, drain_limit: int = 32
    ) -> None:
        """Wait once, then drain a bounded batch of ready callbacks.

        This process consumes several high-rate PX4/Gazebo streams while
        ``rclpy.spin_once`` executes only one callback.  Calling it once per
        terminal loop allowed four consecutive healthy 50 Hz reallocator
        states to remain pending until the 40 ms ownership watchdog fired.
        A bounded zero-time drain keeps the latest safety acknowledgement
        observable without making keyboard polling unbounded.
        """
        rclpy.spin_once(self, timeout_sec=max(0.0, float(timeout_sec)))
        for _ in range(max(0, int(drain_limit))):
            rclpy.spin_once(self, timeout_sec=0.0)

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
        orientation = msg.pose.pose.orientation
        rotation_body_to_world = quaternion_wxyz_to_rotation_body_to_world(
            [orientation.w, orientation.x, orientation.y, orientation.z]
        )
        velocity_body_flu = np.array(
            [
                float(msg.twist.twist.linear.x),
                float(msg.twist.twist.linear.y),
                float(msg.twist.twist.linear.z),
            ]
        )
        # gz-sim OdometryPublisher expresses twist in robot_base_frame
        # (base_link FLU), while pose is in world ENU.  The truth-hold PD law
        # is a world-frame law, so rotate before filtering or damping.
        self.gazebo_truth_velocity_enu = rotation_body_to_world @ velocity_body_flu
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
        complete_finite_positions = True
        for index, name in enumerate(self.rl_joint_names):
            if name in positions and np.isfinite(positions[name]):
                self.rl_joint_positions[index] = float(positions[name])
            else:
                complete_finite_positions = False
            if name in velocities and np.isfinite(velocities[name]):
                self.rl_joint_velocities[index] = float(velocities[name])
        if complete_finite_positions:
            self.last_joint_state_monotonic = time.monotonic()

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

    def _arm_motion_active_cb(
        self, msg: Bool, *, observed_monotonic: float | None = None
    ) -> None:
        self._ensure_arm_direct_xy_force_lease_state()
        with self.arm_direct_xy_intent_transition_lock:
            self._apply_arm_motion_active_level(
                msg, observed_monotonic=observed_monotonic
            )

    def _apply_arm_motion_active_level(
        self, msg: Bool, *, observed_monotonic: float | None = None
    ) -> None:
        """Apply one motion level while guardian-intent snapshots are barred."""
        was_active = self.arm_motion_active
        self.arm_motion_active = bool(msg.data)
        now = (
            time.monotonic()
            if observed_monotonic is None
            else float(observed_monotonic)
        )
        self.last_arm_motion_monotonic = now
        if self.ARM_DIRECT_XY_OWNERSHIP and self.arm_motion_active and not was_active:
            self.arm_direct_xy_ownership_epoch += 1
            self.arm_direct_xy_force_commanded = False
            self.arm_direct_xy_force_ever_ack = False
            self.arm_direct_xy_exit_disable_pending = False
            self.arm_direct_xy_entry_deadline = now + self.ARM_DIRECT_XY_ENTRY_TIMEOUT_S
            self.arm_direct_xy_prepared_monotonic = 0.0
            self.arm_direct_xy_force_ack_deadline = 0.0
            self.arm_direct_xy_force_ack_monotonic = 0.0
            # ``ready`` is a pre-authorisation published while PX4 still owns
            # finite XY.  A compliant trajectory source starts only after
            # seeing it.  Keep inhibit false during the bounded hand-off while
            # the reallocator latches its target and reports active health.
            self.arm_motion_inhibit_pub.publish(
                Bool(data=not self.arm_direct_xy_preauthorized)
            )
        if not self.arm_motion_active:
            if (
                self.arm_direct_xy_active
                or self.arm_direct_xy_force_commanded
                or self.arm_direct_xy_force_enabled_ack
            ):
                self.arm_direct_xy_exit_disable_pending = True
            self.arm_direct_xy_active = False
            self.arm_direct_xy_entry_deadline = 0.0
            self.arm_direct_xy_prepared_monotonic = 0.0
            self.arm_direct_xy_force_ack_deadline = 0.0
            self.arm_direct_xy_force_ack_monotonic = 0.0
            self.arm_direct_xy_abort_latched = False
            self.arm_direct_xy_last_fault = ""
            self.arm_direct_xy_last_fault_detail = ""
            self.arm_direct_xy_preauthorized = False
            if self.ARM_DIRECT_XY_OWNERSHIP:
                # An idle edge clears the preceding action epoch, but it is
                # not itself a new motion grant.  Keep the arm inhibited until
                # the authoritative truth-hold tick has re-evaluated health
                # and the continuous stability hold for the new idle state.
                # Publishing inhibit=False here created a short false grant
                # before the next tick restored ready=False/inhibit=True.
                self.arm_direct_xy_ready_pub.publish(Bool(data=False))
                self.arm_motion_inhibit_pub.publish(Bool(data=True))
                self._restore_finite_xy_and_disable_direct_force()

    @staticmethod
    def _direct_xy_reallocator_report_is_healthy(report: dict) -> bool:
        """Validate the reallocator contract used for mixed-axis ownership."""
        try:
            event = str(report["event"])
            feasibility_scale = float(report["feasibility_scale"])
            residual_norm = float(report["residual_norm"])
            saturated = int(report["saturated"])
        except (KeyError, TypeError, ValueError):
            return False
        allocation_limited = bool(report.get("allocation_limited", False))
        return bool(
            event in {"allocated", "zero_overlay"}
            and report.get("source_fresh") is True
            and report.get("flight_allowed") is True
            and report.get("headroom_ok") is True
            and report.get("position_feedback_enabled") is True
            and report.get("position_feedback_ready") is True
            and report.get("truth_fresh") is True
            and math.isfinite(feasibility_scale)
            and feasibility_scale >= 1.0 - 1.0e-6
            and math.isfinite(residual_norm)
            and residual_norm <= 1.0e-3
            and saturated == 0
            and not allocation_limited
        )

    def _snapshot_arm_direct_xy_reallocator_state(
        self, msg: String
    ) -> tuple[float, float, dict] | None:
        """Parse one producer report without entering the control-state lock."""
        if not self.ARM_DIRECT_XY_OWNERSHIP:
            return None
        received_monotonic = time.monotonic()
        try:
            report = json.loads(msg.data)
        except (TypeError, json.JSONDecodeError):
            report = {}
        try:
            producer_monotonic = float(report.get("monotonic_s", float("nan")))
        except (TypeError, ValueError):
            producer_monotonic = float("nan")
        return producer_monotonic, received_monotonic, report

    def _apply_arm_direct_xy_reallocator_snapshot(
        self, snapshot: tuple[float, float, dict]
    ) -> None:
        """Apply a parsed snapshot; caller serializes control-state mutation."""
        producer_monotonic, received_monotonic, report = snapshot
        healthy = self._direct_xy_reallocator_report_is_healthy(report)
        if math.isfinite(producer_monotonic):
            if producer_monotonic <= self.last_arm_direct_xy_report_producer_monotonic:
                return
            self.last_arm_direct_xy_report_producer_monotonic = producer_monotonic
            # Judge transport freshness at callback receipt, before any wait
            # for the control-state lock.  Runtime freshness below still uses
            # this receipt timestamp, so queued snapshots cannot extend the
            # 40 ms watchdog artificially.
            producer_age = received_monotonic - producer_monotonic
            healthy = bool(
                healthy
                and 0.0 <= producer_age < self.ARM_DIRECT_XY_WATCHDOG_S
            )
        # Receipt time proves producer freshness and whether an ACK met its
        # deadline.  State transitions happen at commit time: a prepared
        # snapshot received before the deadline must not start a hand-off after
        # the control path finally becomes available past that deadline.
        transition_now = time.monotonic()
        position_feedback_prepared = bool(
            healthy
            and report.get("motion_active") is True
            and report.get("position_feedback_prepared") is True
            and report.get("position_target_latched") is True
        )
        try:
            force_epoch = int(report.get("direct_xy_force_epoch", -1))
        except (TypeError, ValueError):
            force_epoch = -1
        position_feedback_active = bool(
            position_feedback_prepared
            and str(report.get("event")) == "allocated"
            and report.get("position_feedback_active") is True
            and report.get("direct_xy_force_enabled") is True
            and force_epoch == self.arm_direct_xy_ownership_epoch
        )
        if position_feedback_active:
            # Prefer the safety ingress' atomic physical receipt evidence.  It
            # may already have observed this epoch while the normal controller
            # snapshot waited for control_lock.
            self._adopt_arm_direct_xy_force_lease_ack(transition_now)
            self.arm_direct_xy_force_ack_monotonic = received_monotonic
            if (
                self.arm_direct_xy_force_ack_deadline > 0.0
                and received_monotonic > self.arm_direct_xy_force_ack_deadline
            ):
                # The acknowledgement budget ends when DDS observes the ACK,
                # not when a later control tick happens to consume its state.
                self._abort_arm_for_direct_xy_fault("direct_force_ack_timeout")
                self._restore_finite_xy_and_disable_direct_force()
                return
            self.arm_direct_xy_force_ack_deadline = 0.0
        if position_feedback_prepared:
            if self.arm_direct_xy_prepared_monotonic <= 0.0:
                self.arm_direct_xy_prepared_monotonic = received_monotonic
        else:
            self.arm_direct_xy_prepared_monotonic = 0.0
        self.arm_direct_xy_position_feedback_prepared = position_feedback_prepared
        self.arm_direct_xy_position_feedback_active = position_feedback_active
        self.arm_direct_xy_force_enabled_ack = position_feedback_active
        self.arm_direct_xy_force_epoch_ack = force_epoch
        if position_feedback_active:
            self.arm_direct_xy_force_ever_ack = True
        self.arm_direct_xy_position_feedback_ready = bool(
            healthy and report.get("position_feedback_ready") is True
        )
        if (
            healthy
            and self.arm_direct_xy_abort_latched
            and self.arm_motion_is_active(transition_now)
        ):
            # An unhealthy report is a safety event, not merely a replaceable
            # sample.  Do not let a following healthy heartbeat re-authorise
            # the same arm-motion epoch; only its explicit falling edge clears
            # the abort latch.
            self.arm_direct_xy_reallocator_healthy = False
            self.arm_direct_xy_position_feedback_active = False
            self.arm_direct_xy_position_feedback_prepared = False
            self.arm_direct_xy_position_feedback_ready = False
            self.arm_direct_xy_force_enabled_ack = False
            self.arm_direct_xy_health_since = 0.0
            self.arm_direct_xy_prepared_monotonic = 0.0
            self.last_arm_direct_xy_state_monotonic = received_monotonic
            self.arm_direct_xy_ready_pub.publish(Bool(data=False))
            self.arm_motion_inhibit_pub.publish(Bool(data=True))
            return
        if healthy:
            if not self.arm_direct_xy_reallocator_healthy:
                self.arm_direct_xy_health_since = received_monotonic
            self.arm_direct_xy_reallocator_healthy = True
            self.last_arm_direct_xy_state_monotonic = received_monotonic
            if (
                position_feedback_prepared
                and self.arm_motion_is_active(transition_now)
                and not self.arm_direct_xy_active
                and not self.arm_direct_xy_abort_latched
            ):
                if transition_now > self.arm_direct_xy_entry_deadline:
                    self._abort_arm_for_direct_xy_fault("entry_gate_timeout")
                else:
                    self._start_event_driven_direct_xy_handoff(transition_now)
            if (
                position_feedback_active
                and self.arm_direct_xy_active
                and not self.arm_direct_xy_abort_latched
            ):
                heartbeat_fault = self._arm_direct_xy_reallocator_fault_reason(
                    transition_now
                )
                if heartbeat_fault is not None:
                    self._abort_arm_for_direct_xy_fault(
                        "runtime_watchdog", detail=heartbeat_fault
                    )
                    self._restore_finite_xy_and_disable_direct_force()
                else:
                    self.arm_direct_xy_ready_pub.publish(Bool(data=True))
                    self.arm_motion_inhibit_pub.publish(Bool(data=False))
                    self._publish_arm_direct_xy_state(
                        "direct_xy", now=transition_now
                    )
            elif self.arm_direct_xy_active and not position_feedback_prepared:
                self._abort_arm_for_direct_xy_fault("direct_force_ack_lost")
                self._restore_finite_xy_and_disable_direct_force()
            return

        self.arm_direct_xy_reallocator_healthy = False
        self.arm_direct_xy_position_feedback_active = False
        self.arm_direct_xy_position_feedback_prepared = False
        self.arm_direct_xy_position_feedback_ready = False
        self.arm_direct_xy_force_enabled_ack = False
        self.arm_direct_xy_health_since = 0.0
        self.arm_direct_xy_prepared_monotonic = 0.0
        self.last_arm_direct_xy_state_monotonic = received_monotonic
        if self.arm_motion_is_active(transition_now):
            self._abort_arm_for_direct_xy_fault("reallocator_unhealthy")
            self._restore_finite_xy_and_disable_direct_force()
        else:
            self.arm_direct_xy_preauthorized = False
            self.arm_direct_xy_ready_pub.publish(Bool(data=False))
            self.arm_motion_inhibit_pub.publish(Bool(data=True))
            self._publish_arm_direct_xy_state(
                "px4_xy",
                watchdog_reason="reallocator_unhealthy",
                now=transition_now,
            )

    def _arm_direct_xy_reallocator_state_cb(self, msg: String) -> None:
        """Synchronous compatibility path used by focused unit tests."""
        snapshot = self._snapshot_arm_direct_xy_reallocator_state(msg)
        if snapshot is not None:
            self._apply_arm_direct_xy_reallocator_snapshot(snapshot)

    def _arm_direct_xy_health_fresh(self, now: float) -> bool:
        return self._arm_direct_xy_reallocator_fault_reason(now) is None

    def _arm_direct_xy_reallocator_fault_reason(self, now: float) -> str | None:
        """Return the exact fail-closed owner-heartbeat fault, if any."""
        cache_lock = getattr(self, "arm_direct_xy_report_cache_lock", None)
        if cache_lock is None:
            healthy_receipt = self.last_arm_direct_xy_state_monotonic
            pending_fault = False
        else:
            with cache_lock:
                healthy_receipt = (
                    self.last_arm_direct_xy_healthy_receipt_monotonic
                )
                pending_fault = self.arm_direct_xy_pending_fault_snapshot is not None
        if pending_fault:
            return "reallocator_pending_fault"
        if not self.arm_direct_xy_reallocator_healthy:
            return "reallocator_unhealthy"
        if healthy_receipt <= 0.0:
            return "reallocator_receipt_missing"
        age = float(now) - float(healthy_receipt)
        if age < 0.0:
            return "reallocator_receipt_future"
        timeout_s = (
            self.ARM_DIRECT_XY_GUARDIAN_STATE_TIMEOUT_S
            if self.ARM_DIRECT_XY_EXTERNAL_GUARDIAN
            else self.ARM_DIRECT_XY_WATCHDOG_S
        )
        if age >= timeout_s:
            return (
                "guardian_state_stale"
                if self.ARM_DIRECT_XY_EXTERNAL_GUARDIAN
                else "reallocator_heartbeat_stale"
            )
        return None

    def _arm_direct_xy_runtime_fault_reason(
        self, now: float, *, require_handoff_buffers: bool = False
    ) -> str | None:
        """Explain the active mixed-axis safety gate without weakening it."""
        # Truth and PX4 status are written by executor threads.  Snapshot their
        # timestamps first, then sample the clock: if a callback updates one
        # between the caller's earlier clock sample and this check, comparing
        # that new stamp with the old ``now`` produces a false "timestamp
        # future" abort.  Resampling later only increases every watchdog age;
        # it therefore fixes the ordering race without relaxing any timeout.
        truth_stamp = float(self.last_gazebo_truth_monotonic)
        status_stamp = float(self.last_status_monotonic)
        evaluation_now = float(now)
        if truth_stamp > evaluation_now or status_stamp > evaluation_now:
            evaluation_now = max(evaluation_now, time.monotonic())
        reason = self._arm_direct_xy_reallocator_fault_reason(evaluation_now)
        if reason is not None:
            return reason
        if not self.arm_direct_xy_position_feedback_prepared:
            return "position_feedback_not_prepared"
        if self.gazebo_truth_enu is None:
            return "truth_position_missing"
        if self.gazebo_truth_velocity_enu is None:
            return "truth_velocity_missing"
        truth_age = evaluation_now - truth_stamp
        if truth_age < 0.0:
            return "truth_timestamp_future"
        if truth_age >= self.TRUTH_HOLD_TIMEOUT_S:
            return "truth_stale"
        if self.status is None:
            return "status_missing"
        status_age = evaluation_now - status_stamp
        if status_age < 0.0:
            return "status_timestamp_future"
        if status_age >= self.STATUS_TIMEOUT_S:
            return "status_stale"
        if bool(self.status.failsafe):
            return "px4_failsafe"
        if self.control_state != FlightControlState.POSITION_HOLD:
            return "not_position_hold"
        if require_handoff_buffers:
            if self.truth_hold_target_enu is None:
                return "truth_hold_target_missing"
            if self.gazebo_truth_velocity_filtered_enu is None:
                return "filtered_truth_velocity_missing"
        return None

    def _publish_arm_direct_xy_state(
        self,
        state: str,
        *,
        watchdog_reason: str = "",
        watchdog_detail: str = "",
        now: float | None = None,
    ) -> None:
        if not self.ARM_DIRECT_XY_OWNERSHIP:
            return
        current = time.monotonic() if now is None else float(now)
        report = {
            "schema": "my_drone.arm-direct-xy-state.v1",
            "ownership_epoch": int(self.arm_direct_xy_ownership_epoch),
            "state": str(state),
            "watchdog_reason": str(watchdog_reason),
            "watchdog_detail": str(watchdog_detail),
            "reallocator_fresh": self._arm_direct_xy_health_fresh(current),
            "position_feedback_active": bool(
                self.arm_direct_xy_position_feedback_active
            ),
            "position_feedback_prepared": bool(
                self.arm_direct_xy_position_feedback_prepared
            ),
            "position_feedback_ready": bool(
                self.arm_direct_xy_position_feedback_ready
            ),
            "motion_active": self.arm_motion_is_active(current),
            "direct_xy_force_enabled_ack": bool(
                self.arm_direct_xy_force_enabled_ack
            ),
            "direct_xy_force_epoch_ack": int(self.arm_direct_xy_force_epoch_ack),
        }
        self.arm_direct_xy_state_pub.publish(
            String(data=json.dumps(report, sort_keys=True))
        )

    def _publish_external_guardian_intent(self, state: str, now: float) -> None:
        """Publish a replaceable controller level; never a physical lease."""
        if not self.ARM_DIRECT_XY_EXTERNAL_GUARDIAN:
            return
        self._ensure_arm_direct_xy_force_lease_state()
        with self.arm_direct_xy_intent_transition_lock:
            with self.arm_direct_xy_force_lease_lock:
                # The mixed-setpoint repeater samples its wire time before it
                # publishes.  While its later intent callback waits here, the
                # main executor may commit a newer arm-motion heartbeat.  Use
                # a clock sampled under both ownership locks so that heartbeat
                # cannot look future-dated and falsely produce the inconsistent
                # force_requested=True/motion_active=False guardian level.
                intent_now = max(float(now), time.monotonic())
                generation = int(self.arm_direct_xy_force_lease_generation)
                requested = bool(
                    self.arm_direct_xy_force_commanded
                    and self.arm_direct_xy_active
                    and not self.arm_direct_xy_abort_latched
                )
                motion_active = self.arm_motion_is_active(intent_now)
                mixed_setpoint_active = bool(self.arm_direct_xy_active)
                ownership_epoch = int(self.arm_direct_xy_ownership_epoch)
            intent = {
                "schema": "my_drone.arm-direct-xy-controller-intent.v1",
                "monotonic_s": intent_now,
                "controller_session_id": self.arm_direct_xy_guardian_session_id,
                "ownership_epoch": ownership_epoch,
                "intent_generation": generation,
                "state": str(state),
                "motion_active": motion_active,
                "mixed_setpoint_active": mixed_setpoint_active,
                "force_requested": requested,
            }
            self.arm_direct_xy_intent_pub.publish(
                String(data=json.dumps(intent, sort_keys=True))
            )

    def _ensure_arm_direct_xy_force_lease_state(self) -> None:
        """Lazily initialise lease fields for focused object.__new__ tests."""
        if not hasattr(self, "arm_direct_xy_force_lease_lock"):
            self.arm_direct_xy_force_lease_lock = threading.Lock()
        if not hasattr(self, "arm_direct_xy_force_publish_lock"):
            self.arm_direct_xy_force_publish_lock = threading.Lock()
        if not hasattr(self, "arm_direct_xy_intent_transition_lock"):
            self.arm_direct_xy_intent_transition_lock = threading.RLock()
        defaults = {
            "arm_direct_xy_force_lease_publisher": None,
            "arm_direct_xy_force_lease_generation": 0,
            "arm_direct_xy_force_lease_requested": False,
            "arm_direct_xy_force_lease_epoch": -1,
            "arm_direct_xy_force_lease_mixed_setpoint_monotonic": 0.0,
            "arm_direct_xy_force_lease_first_active_receipt_monotonic": 0.0,
            "arm_direct_xy_force_lease_active_receipt_monotonic": 0.0,
            "arm_direct_xy_force_lease_active_report_epoch": -1,
            "arm_direct_xy_force_lease_latest_report_producer_monotonic": 0.0,
            "arm_direct_xy_force_lease_active_seen": False,
            "arm_direct_xy_force_lease_last_publish_monotonic": 0.0,
        }
        for name, value in defaults.items():
            if not hasattr(self, name):
                setattr(self, name, value)
        if self.arm_direct_xy_force_lease_publisher is None:
            # Existing focused tests attach an event recorder under the old
            # attribute.  Runtime assigns this alias only from the safety node,
            # so there is still exactly one ROS writer.
            publisher = getattr(self, "arm_direct_xy_force_command_pub", None)
            if publisher is not None:
                self.arm_direct_xy_force_lease_publisher = publisher

    def _attach_arm_direct_xy_force_lease_publisher(self, publisher) -> None:
        self._ensure_arm_direct_xy_force_lease_state()
        with self.arm_direct_xy_force_lease_lock:
            self.arm_direct_xy_force_lease_publisher = publisher
            # Compatibility/introspection alias; the publisher itself belongs
            # to DirectXySafetyIngress and is never duplicated on this node.
            self.arm_direct_xy_force_command_pub = publisher

    def _publish_arm_direct_xy_force_lease(
        self, enabled: bool, epoch: int, generation: int
    ) -> bool:
        """Publish a current generation without holding the receipt lock."""
        self._ensure_arm_direct_xy_force_lease_state()
        if self.ARM_DIRECT_XY_EXTERNAL_GUARDIAN:
            self._publish_external_guardian_intent(
                "direct_xy" if bool(enabled) else "exit", time.monotonic()
            )
            return True
        enabled = bool(enabled)
        epoch = int(epoch)
        generation = int(generation)
        with self.arm_direct_xy_force_publish_lock:
            with self.arm_direct_xy_force_lease_lock:
                publisher = self.arm_direct_xy_force_lease_publisher
                current = bool(
                    generation == self.arm_direct_xy_force_lease_generation
                    and epoch == self.arm_direct_xy_force_lease_epoch
                    and (
                        self.arm_direct_xy_force_lease_requested
                        if enabled
                        else not self.arm_direct_xy_force_lease_requested
                    )
                )
                if publisher is None:
                    raise RuntimeError(
                        "direct XY force lease publisher is not attached"
                    )
                if not current:
                    return False
                command = {
                    "schema": "my_drone.arm-direct-xy-force-command.v1",
                    "ownership_epoch": epoch,
                    "enabled": enabled,
                    "lease_generation": generation,
                }
            # Deliberately outside arm_direct_xy_force_lease_lock.  DDS work
            # must not starve physical receipt bookkeeping.
            publisher.publish(String(data=json.dumps(command, sort_keys=True)))
            published_monotonic = time.monotonic()
            with self.arm_direct_xy_force_lease_lock:
                if (
                    enabled
                    and generation == self.arm_direct_xy_force_lease_generation
                    and epoch == self.arm_direct_xy_force_lease_epoch
                    and self.arm_direct_xy_force_lease_requested
                ):
                    self.arm_direct_xy_force_lease_last_publish_monotonic = (
                        published_monotonic
                    )
        return True

    def _request_arm_direct_xy_force_after_mixed_setpoint(
        self, mixed_setpoint_monotonic: float
    ) -> None:
        """Atomically start a lease only after the mixed setpoint was sent."""
        self._ensure_arm_direct_xy_force_lease_state()
        stamp = float(mixed_setpoint_monotonic)
        if not math.isfinite(stamp) or stamp <= 0.0:
            raise ValueError("mixed setpoint timestamp must be finite and positive")
        epoch = int(self.arm_direct_xy_ownership_epoch)
        with self.arm_direct_xy_force_lease_lock:
            self.arm_direct_xy_force_lease_generation += 1
            self.arm_direct_xy_force_lease_requested = True
            self.arm_direct_xy_force_lease_epoch = epoch
            self.arm_direct_xy_force_lease_mixed_setpoint_monotonic = stamp
            self.arm_direct_xy_force_lease_first_active_receipt_monotonic = 0.0
            self.arm_direct_xy_force_lease_active_receipt_monotonic = 0.0
            self.arm_direct_xy_force_lease_active_report_epoch = -1
            self.arm_direct_xy_force_lease_latest_report_producer_monotonic = 0.0
            self.arm_direct_xy_force_lease_active_seen = False
            self.arm_direct_xy_force_lease_last_publish_monotonic = 0.0
            self.arm_direct_xy_force_commanded = True
            # The independent 10 ms safety timer owns both bootstrap and
            # refresh.  A reliable publish from this 20 Hz main-control thread
            # can return after the 150 ms deadline even when the allocator
            # applied the message immediately.
        if self.ARM_DIRECT_XY_EXTERNAL_GUARDIAN:
            # This call is deliberately after both the mixed PX4 setpoint and
            # the state mutation above.  Never let a diagnostic publication
            # send an active intent earlier than the actual ownership handoff.
            self._publish_external_guardian_intent("handoff", stamp)

    def _note_arm_direct_xy_mixed_setpoint_published(
        self, mixed_setpoint_monotonic: float
    ) -> bool:
        """Refresh main-controller liveness after an actual mixed publish."""
        self._ensure_arm_direct_xy_force_lease_state()
        stamp = float(mixed_setpoint_monotonic)
        if not math.isfinite(stamp) or stamp <= 0.0:
            return False
        epoch = int(self.arm_direct_xy_ownership_epoch)
        with self.arm_direct_xy_force_lease_lock:
            if not (
                self.arm_direct_xy_force_lease_requested
                and self.arm_direct_xy_force_lease_epoch == epoch
            ):
                return False
            self.arm_direct_xy_force_lease_mixed_setpoint_monotonic = stamp
        if self.ARM_DIRECT_XY_EXTERNAL_GUARDIAN:
            # A guardian heartbeat is valid only when it follows a real mixed
            # setpoint publication.  There is no independent timer in the
            # controller that can keep a stale flight intent alive.
            self._publish_external_guardian_intent("direct_xy", stamp)
        return True

    def _revoke_arm_direct_xy_force_lease_locked(
        self,
    ) -> tuple[bool, int, int] | None:
        """Update revoke state under lock and return a later wire command."""
        if not self.arm_direct_xy_force_lease_requested:
            return None
        epoch = int(self.arm_direct_xy_force_lease_epoch)
        self.arm_direct_xy_force_lease_generation += 1
        generation = int(self.arm_direct_xy_force_lease_generation)
        self.arm_direct_xy_force_lease_requested = False
        self.arm_direct_xy_force_lease_active_seen = False
        self.arm_direct_xy_force_lease_active_receipt_monotonic = 0.0
        self.arm_direct_xy_force_commanded = False
        return False, epoch, generation

    def _observe_arm_direct_xy_force_lease_report(
        self, report: dict, *, received_monotonic: float
    ) -> bool:
        """Record active evidence or serially revoke a lost physical owner."""
        if not self.ARM_DIRECT_XY_OWNERSHIP:
            return False
        self._ensure_arm_direct_xy_force_lease_state()
        received = float(received_monotonic)
        try:
            producer = float(report["monotonic_s"])
            force_epoch = int(report.get("direct_xy_force_epoch", -1))
        except (KeyError, TypeError, ValueError):
            producer = float("nan")
            force_epoch = -1
        producer_fresh = bool(
            math.isfinite(producer)
            and math.isfinite(received)
            and 0.0 <= received - producer < self.ARM_DIRECT_XY_WATCHDOG_S
        )
        contract_healthy = self._direct_xy_reallocator_report_is_healthy(report)
        revoke_command = None
        with self.arm_direct_xy_force_lease_lock:
            if not self.arm_direct_xy_force_lease_requested:
                return False
            if (
                math.isfinite(producer)
                and producer
                <= self.arm_direct_xy_force_lease_latest_report_producer_monotonic
            ):
                return False
            if math.isfinite(producer):
                self.arm_direct_xy_force_lease_latest_report_producer_monotonic = (
                    producer
                )
            expected_epoch = int(self.arm_direct_xy_force_lease_epoch)
            actual_active = bool(
                producer_fresh
                and contract_healthy
                and report.get("event") == "allocated"
                and report.get("motion_active") is True
                and report.get("position_feedback_prepared") is True
                and report.get("position_target_latched") is True
                and report.get("position_feedback_active") is True
                and report.get("direct_xy_force_command_fresh") is True
                and report.get("direct_xy_force_enabled") is True
                and force_epoch == expected_epoch
            )
            if actual_active:
                if not self.arm_direct_xy_force_lease_active_seen:
                    self.arm_direct_xy_force_lease_first_active_receipt_monotonic = (
                        received
                    )
                self.arm_direct_xy_force_lease_active_receipt_monotonic = received
                self.arm_direct_xy_force_lease_active_report_epoch = force_epoch
                self.arm_direct_xy_force_lease_active_seen = True
                return True
            if (
                self.arm_direct_xy_force_lease_active_seen
                and force_epoch == expected_epoch
            ):
                # Once this epoch has proven active, any newer same-session
                # report that retracts that fact is authoritative negative
                # evidence.  Revoke once and latch the generation so a later
                # queued positive sample cannot restart the old lease.
                revoke_command = self._revoke_arm_direct_xy_force_lease_locked()
        if revoke_command is not None:
            self._publish_arm_direct_xy_force_lease(*revoke_command)
        return False

    def _adopt_arm_direct_xy_force_lease_ack(
        self, now: float | None = None
    ) -> bool:
        """Commit safety-ingress physical evidence into main owner state.

        The dedicated ingress is authoritative for receipt time and epoch.  A
        busy main callback must not manufacture an ACK timeout merely because
        its normal reallocator snapshot has not yet been applied.  Entry still
        requires the first active receipt strictly inside 150 ms, while the
        latest receipt must satisfy the unchanged 40 ms runtime watchdog.
        """
        if not self.ARM_DIRECT_XY_OWNERSHIP:
            return False
        if self.ARM_DIRECT_XY_EXTERNAL_GUARDIAN:
            return bool(
                self.arm_direct_xy_force_enabled_ack
                and self.arm_direct_xy_force_epoch_ack
                == self.arm_direct_xy_ownership_epoch
            )
        self._ensure_arm_direct_xy_force_lease_state()
        current = time.monotonic() if now is None else float(now)
        deadline = float(self.arm_direct_xy_force_ack_deadline)
        expected_epoch = int(self.arm_direct_xy_ownership_epoch)
        with self.arm_direct_xy_force_lease_lock:
            first_receipt = float(
                self.arm_direct_xy_force_lease_first_active_receipt_monotonic
            )
            latest_receipt = float(
                self.arm_direct_xy_force_lease_active_receipt_monotonic
            )
            mixed_stamp = float(
                self.arm_direct_xy_force_lease_mixed_setpoint_monotonic
            )
            physical_epoch = int(
                self.arm_direct_xy_force_lease_active_report_epoch
            )
            evidence_matches = bool(
                self.arm_direct_xy_force_lease_requested
                and self.arm_direct_xy_force_lease_active_seen
                and self.arm_direct_xy_force_lease_epoch == expected_epoch
                and physical_epoch == expected_epoch
            )
        entered_on_time = bool(
            deadline > 0.0
            and mixed_stamp <= first_receipt < deadline
        )
        receipt_fresh = bool(
            0.0 <= current - latest_receipt < self.ARM_DIRECT_XY_WATCHDOG_S
        )
        if not (evidence_matches and entered_on_time and receipt_fresh):
            return False
        self.arm_direct_xy_force_enabled_ack = True
        self.arm_direct_xy_force_epoch_ack = expected_epoch
        self.arm_direct_xy_force_ever_ack = True
        self.arm_direct_xy_force_ack_monotonic = first_receipt
        self.arm_direct_xy_force_ack_deadline = 0.0
        return True

    def _refresh_arm_direct_xy_force_lease(
        self, now: float | None = None
    ) -> bool:
        """Refresh True independently of control_lock, or fail closed."""
        if not self.ARM_DIRECT_XY_OWNERSHIP:
            return False
        if self.ARM_DIRECT_XY_EXTERNAL_GUARDIAN:
            return False
        self._ensure_arm_direct_xy_force_lease_state()
        publish_command = None
        authorized = False
        with self.arm_direct_xy_force_lease_lock:
            # Runtime callers deliberately omit ``now``.  Read the clock only
            # after acquiring the same lock that protects the mixed-setpoint
            # timestamp; otherwise a control-thread update between the clock
            # read and this lock acquisition produces a false negative age and
            # an erroneous fail-closed revoke.  Explicit timestamps remain
            # available solely for deterministic unit tests.
            current = time.monotonic() if now is None else float(now)
            if not self.arm_direct_xy_force_lease_requested:
                return False
            epoch = int(self.arm_direct_xy_force_lease_epoch)
            setpoint_age = (
                current
                - self.arm_direct_xy_force_lease_mixed_setpoint_monotonic
            )
            receipt_age = (
                current
                - self.arm_direct_xy_force_lease_active_receipt_monotonic
            )
            controller_epoch_matches = bool(
                epoch == int(self.arm_direct_xy_ownership_epoch)
            )
            active_report_epoch_matches = bool(
                self.arm_direct_xy_force_lease_active_seen
                and self.arm_direct_xy_force_lease_active_report_epoch == epoch
            )
            main_setpoint_fresh = bool(
                0.0
                <= setpoint_age
                < self.ARM_DIRECT_XY_MAIN_SETPOINT_LEASE_S
            )
            report_fresh = bool(
                self.arm_direct_xy_force_lease_active_seen
                and 0.0 <= receipt_age < self.ARM_DIRECT_XY_WATCHDOG_S
            )
            motion_fresh = self.arm_motion_is_active(current)
            controller_intent_valid = bool(
                self.arm_direct_xy_active
                and not self.arm_direct_xy_abort_latched
                and motion_fresh
                and controller_epoch_matches
                and main_setpoint_fresh
            )
            # Before the first physical receipt this is the bootstrap retry;
            # afterwards it becomes the ordinary 40 ms-gated refresh.
            authorized = bool(
                controller_intent_valid
                and (
                    not self.arm_direct_xy_force_lease_active_seen
                    or (active_report_epoch_matches and report_fresh)
                )
            )
            if authorized:
                last_publish = float(
                    self.arm_direct_xy_force_lease_last_publish_monotonic
                )
                bootstrap = not self.arm_direct_xy_force_lease_active_seen
                refresh_due = bool(
                    last_publish <= 0.0
                    or current - last_publish >= self.ARM_DIRECT_XY_FORCE_REFRESH_S
                )
                if bootstrap or refresh_due:
                    publish_command = (
                        True,
                        epoch,
                        int(self.arm_direct_xy_force_lease_generation),
                    )
            hard_expiry = bool(
                setpoint_age < 0.0
                or setpoint_age >= self.ARM_DIRECT_XY_MAIN_SETPOINT_LEASE_S
                or (
                    self.arm_direct_xy_force_lease_active_seen
                    and (receipt_age < 0.0 or receipt_age >= self.ARM_DIRECT_XY_WATCHDOG_S)
                )
                or not controller_epoch_matches
                or (
                    self.arm_direct_xy_force_lease_active_seen
                    and not active_report_epoch_matches
                )
                or not motion_fresh
            )
            if hard_expiry:
                publish_command = self._revoke_arm_direct_xy_force_lease_locked()
            # controller inactive/abort is an orderly-exit state.  Do not race
            # its finite setpoint; simply stop refreshing until the main path
            # performs the serial disable (or the 0.25 s hard expiry fires).
        if publish_command is not None:
            published = self._publish_arm_direct_xy_force_lease(*publish_command)
            return bool(authorized and published)
        return False

    def _publish_arm_direct_xy_force_command(self, enabled: bool) -> None:
        """Compatibility wrapper; True is bootstrap, False is serial revoke."""
        self._ensure_arm_direct_xy_force_lease_state()
        if enabled:
            self._request_arm_direct_xy_force_after_mixed_setpoint(time.monotonic())
            return
        publish_command = None
        with self.arm_direct_xy_force_lease_lock:
            if self.arm_direct_xy_force_lease_requested:
                publish_command = self._revoke_arm_direct_xy_force_lease_locked()
            else:
                # Exit paths may need an explicit disable even if a receipt
                # fault already cleared intent.  Bump generation and publish
                # through the same safety-owned writer.
                epoch = int(self.arm_direct_xy_ownership_epoch)
                self.arm_direct_xy_force_lease_generation += 1
                generation = int(self.arm_direct_xy_force_lease_generation)
                self.arm_direct_xy_force_lease_epoch = epoch
                self.arm_direct_xy_force_commanded = False
                publish_command = (False, epoch, generation)
        if publish_command is not None:
            self._publish_arm_direct_xy_force_lease(*publish_command)

    def _disable_direct_xy_force_after_finite_px4(self) -> None:
        """Start exit after finite PX4 XY; complete on physical disabled ACK."""
        if not self.ARM_DIRECT_XY_OWNERSHIP:
            return
        if self.ARM_DIRECT_XY_EXTERNAL_GUARDIAN:
            self._ensure_arm_direct_xy_force_lease_state()
            with self.arm_direct_xy_force_lease_lock:
                lease_requested = bool(
                    self.arm_direct_xy_force_lease_requested
                )
            may_be_active = bool(
                self.arm_direct_xy_exit_disable_pending
                or self.arm_direct_xy_force_commanded
                or self.arm_direct_xy_force_enabled_ack
                or lease_requested
            )
            if may_be_active:
                self.arm_direct_xy_exit_disable_pending = True
                if lease_requested:
                    # This mutates the controller intent generation exactly
                    # once.  Subsequent finite setpoints refresh the same exit
                    # intent while the guardian retries its same-generation
                    # physical False command.
                    self._publish_arm_direct_xy_force_command(False)
                else:
                    self._publish_external_guardian_intent(
                        "aborting"
                        if self.arm_direct_xy_abort_latched
                        else "exit",
                        time.monotonic(),
                    )
            else:
                self._publish_external_guardian_intent(
                    "aborting" if self.arm_direct_xy_abort_latched else "px4_xy",
                    time.monotonic(),
                )
            return
        if (
            self.arm_direct_xy_exit_disable_pending
            or self.arm_direct_xy_force_commanded
            or self.arm_direct_xy_force_enabled_ack
        ):
            self._publish_arm_direct_xy_force_command(False)
            self.arm_direct_xy_exit_disable_pending = False
            self.arm_direct_xy_force_enabled_ack = False
            self.arm_direct_xy_force_ever_ack = False

    def _restore_finite_xy_and_disable_direct_force(self) -> None:
        """Immediately end mixed ownership in finite-before-disable order."""
        if not self.ARM_DIRECT_XY_OWNERSHIP:
            return
        direct_force_may_be_active = bool(
            self.arm_direct_xy_active
            or self.arm_direct_xy_force_commanded
            or self.arm_direct_xy_force_enabled_ack
            or self.arm_direct_xy_exit_disable_pending
        )
        if not direct_force_may_be_active:
            return
        self.arm_direct_xy_active = False
        self.arm_direct_xy_exit_disable_pending = True
        if self.offboard_requested and self.target_initialized:
            self.publish_hold()
        else:
            self._publish_arm_direct_xy_force_command(False)
            self.arm_direct_xy_exit_disable_pending = False
            self.arm_direct_xy_force_enabled_ack = False
            self.arm_direct_xy_force_ever_ack = False

    def _teardown_direct_xy_before_mode_exit(self, reason: str) -> None:
        """Fail closed before LAND, POSCTL, disarm, or Offboard loss.

        If the direct horizontal owner may be applying force, publish a finite
        PX4 horizontal setpoint before revoking it.  When no usable Offboard
        target exists, immediate force revocation is safer than waiting for the
        reallocator's command timeout.
        """
        if not self.ARM_DIRECT_XY_OWNERSHIP:
            return
        direct_force_may_be_active = bool(
            self.arm_direct_xy_active
            or self.arm_direct_xy_force_commanded
            or self.arm_direct_xy_force_enabled_ack
            or self.arm_direct_xy_exit_disable_pending
        )
        motion_active = self.arm_motion_is_active()
        if not direct_force_may_be_active and not motion_active:
            return
        self._abort_arm_for_direct_xy_fault(reason)
        if direct_force_may_be_active and self.offboard_requested and self.target_initialized:
            # abort_latched prevents publish_hold() from re-entering direct XY;
            # its ordinary path establishes finite PX4 XY and then revokes.
            self.publish_hold()
        else:
            self._publish_arm_direct_xy_force_command(False)
            self.arm_direct_xy_exit_disable_pending = False
            self.arm_direct_xy_force_enabled_ack = False
            self.arm_direct_xy_force_ever_ack = False

    def _update_arm_direct_xy_stability(self, now: float) -> bool:
        if (
            not self.ARM_DIRECT_XY_OWNERSHIP
            or self.control_state != FlightControlState.POSITION_HOLD
            or not self.TRUTH_HOLD_ENABLED
            or self.truth_hold_target_enu is None
            or not self.truth_hold_fresh()
            or not self.status_fresh()
            or self.status is None
            or self.status.arming_state != VehicleStatus.ARMING_STATE_ARMED
            or bool(self.status.failsafe)
            or self.gazebo_truth_velocity_enu is None
            or self.gazebo_truth_rpy is None
        ):
            self.arm_direct_xy_stable_since = 0.0
            return False
        xy_error = float(np.linalg.norm(
            self.truth_hold_target_enu[:2] - self.gazebo_truth_enu[:2]
        ))
        xy_speed = float(np.linalg.norm(self.gazebo_truth_velocity_enu[:2]))
        tilt = float(np.linalg.norm(self.gazebo_truth_rpy[:2]))
        stable = bool(
            xy_error <= self.ARM_DIRECT_XY_ENTRY_XY_ERROR_M
            and xy_speed <= self.ARM_DIRECT_XY_ENTRY_XY_SPEED_M_S
            and tilt <= self.ARM_DIRECT_XY_ENTRY_TILT_RAD
        )
        if stable:
            if self.arm_direct_xy_stable_since <= 0.0:
                self.arm_direct_xy_stable_since = now
        else:
            self.arm_direct_xy_stable_since = 0.0
        return stable

    def _arm_direct_xy_preentry_ready(self, now: float) -> bool:
        """Return the pre-authorisation gate while PX4 still owns finite XY."""
        if not self.ARM_DIRECT_XY_OWNERSHIP:
            return False
        if (
            self.arm_direct_xy_exit_disable_pending
            or self.arm_direct_xy_force_commanded
            or self.arm_direct_xy_force_enabled_ack
        ):
            # A new action must not reuse readiness until the preceding owner
            # has observed a finite PX4 XY setpoint and has been revoked.
            return False
        stable = self._update_arm_direct_xy_stability(now)
        health_held = bool(
            self._arm_direct_xy_health_fresh(now)
            and self.arm_direct_xy_position_feedback_ready
            and self.arm_direct_xy_health_since > 0.0
            and now - self.arm_direct_xy_health_since
            >= self.ARM_DIRECT_XY_HEALTH_HOLD_S
        )
        stability_held = bool(
            stable
            and self.arm_direct_xy_stable_since > 0.0
            and now - self.arm_direct_xy_stable_since
            >= self.ARM_DIRECT_XY_STABLE_HOLD_S
        )
        return health_held and stability_held

    def _arm_direct_xy_active_handoff_ready(self, now: float) -> bool:
        """Require the post-motion target latch before removing PX4 XY."""
        return bool(
            self.arm_direct_xy_preauthorized
            and self._arm_direct_xy_health_fresh(now)
            and self.arm_direct_xy_position_feedback_prepared
            and self.arm_direct_xy_prepared_monotonic > 0.0
            and self.arm_direct_xy_prepared_monotonic
            <= self.arm_direct_xy_entry_deadline
            and now <= self.arm_direct_xy_entry_deadline
        )

    def _start_event_driven_direct_xy_handoff(self, now: float) -> bool:
        """Publish mixed PX4 XY then enable direct force from prepared event."""
        if not self._arm_direct_xy_active_handoff_ready(now):
            return False
        handoff_fault = self._arm_direct_xy_runtime_fault_reason(
            now, require_handoff_buffers=True
        )
        if handoff_fault is not None:
            self._abort_arm_for_direct_xy_fault(
                "runtime_watchdog", detail=handoff_fault
            )
            return False
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
        mode = OffboardControlMode()
        mode.timestamp = self.now_us()
        mode.velocity = True
        self.mode_pub.publish(mode)
        nan = float("nan")
        sp = TrajectorySetpoint()
        sp.timestamp = mode.timestamp
        sp.position = [nan, nan, nan]
        sp.velocity = [nan, nan, float(velocity_ned[2])]
        sp.acceleration = [0.0, 0.0, nan]
        sp.jerk = [nan, nan, nan]
        sp.yaw = self.target.yaw
        sp.yawspeed = nan
        self.setpoint_pub.publish(sp)
        mixed_setpoint_monotonic = time.monotonic()
        self.arm_direct_xy_active = True
        self.arm_direct_xy_force_ack_deadline = (
            time.monotonic() + self.ARM_DIRECT_XY_ENTRY_TIMEOUT_S
        )
        self._request_arm_direct_xy_force_after_mixed_setpoint(
            mixed_setpoint_monotonic
        )
        self.arm_direct_xy_ready_pub.publish(Bool(data=True))
        self.arm_motion_inhibit_pub.publish(Bool(data=False))
        self._publish_arm_direct_xy_state("handoff", now=now)
        self.get_logger().info(
            "ARM_DIRECT_XY_HANDOFF_STARTED event-driven mixed-axis PX4 setpoint first"
        )
        return True

    def _abort_arm_for_direct_xy_fault(
        self, reason: str, *, detail: str = ""
    ) -> None:
        """Interrupt the active joint trajectory and latch an inhibit request."""
        if not self.ARM_DIRECT_XY_OWNERSHIP:
            return
        if (
            self.arm_direct_xy_active
            or self.arm_direct_xy_force_commanded
            or self.arm_direct_xy_force_enabled_ack
        ):
            self.arm_direct_xy_exit_disable_pending = True
        self.arm_direct_xy_active = False
        self.arm_direct_xy_entry_deadline = 0.0
        self.arm_direct_xy_prepared_monotonic = 0.0
        self.arm_direct_xy_force_ack_deadline = 0.0
        self.arm_direct_xy_force_ack_monotonic = 0.0
        self.arm_direct_xy_reallocator_healthy = False
        self.arm_direct_xy_health_since = 0.0
        self.arm_motion_inhibit_pub.publish(Bool(data=True))
        self.arm_direct_xy_ready_pub.publish(Bool(data=False))
        if self.arm_direct_xy_abort_latched:
            return
        self.arm_direct_xy_abort_latched = True
        self.arm_direct_xy_last_fault = str(reason)
        self.arm_direct_xy_last_fault_detail = str(detail)
        now = time.monotonic()
        self._publish_arm_direct_xy_state(
            "aborting",
            watchdog_reason=str(reason),
            watchdog_detail=str(detail),
            now=now,
        )
        if (
            self.last_joint_state_monotonic > 0.0
            and now - self.last_joint_state_monotonic < 0.20
            and np.all(np.isfinite(self.rl_joint_positions))
        ):
            stop = JointTrajectory()
            stop.joint_names = list(self.rl_joint_names)
            point = JointTrajectoryPoint()
            point.positions = self.rl_joint_positions.tolist()
            point.velocities = np.zeros(6).tolist()
            point.accelerations = np.zeros(6).tolist()
            point.time_from_start.sec = 0
            point.time_from_start.nanosec = 100_000_000
            stop.points = [point]
            self.rl_arm_pub.publish(stop)
        detail_text = "" if not detail else f" detail={detail}"
        self.get_logger().error(
            "ARM_DIRECT_XY_ABORT " + str(reason) + detail_text
            + "; restoring finite PX4 truth-hold XY"
        )

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
            self.ARM_DIRECT_XY_OWNERSHIP
            and self.arm_motion_is_active()
            and self.control_state == FlightControlState.POSITION_HOLD
            and self.TRUTH_HOLD_ENABLED
            and self.truth_hold_target_enu is not None
            and not self.truth_hold_fresh()
        ):
            self._abort_arm_for_direct_xy_fault("truth_hold_stale")
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
        # Truth may be stale during an abort.  The ordinary finite-position
        # fallback is still a valid PX4 XY owner, so publish it before sending
        # the direct-force revoke just like the truth-hold exit path.
        self._disable_direct_xy_force_after_finite_px4()

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
        # ``mode_pub.publish`` and the outer-loop calculation can occupy the
        # control lock long enough for one or more 50 Hz safety reports to
        # arrive.  Consume the newest lock-free snapshot immediately before
        # ownership freshness is evaluated.
        self._consume_latest_arm_motion_snapshot()
        # In external-guardian mode the authoritative owner heartbeat is a
        # different cache from the legacy raw reallocator report below.  It
        # must be consumed at this same just-in-time point: consuming it only
        # at the start of the 20 Hz control tick can age an otherwise healthy
        # 100 Hz guardian level past the unchanged 40 ms runtime watchdog while
        # this method is calculating and publishing the truth-hold setpoint.
        self._consume_external_guardian_state()
        self._consume_latest_arm_direct_xy_report_snapshot()
        now = time.monotonic()
        motion_active = self.arm_motion_is_active(now)
        preentry_ready = False
        if self.ARM_DIRECT_XY_OWNERSHIP:
            if motion_active:
                # Freeze the grant accepted on the motion rising edge.  Arm
                # motion may legitimately violate the entry-only low-speed
                # gate and must not revoke its own hand-off authorization.
                preentry_ready = self.arm_direct_xy_preauthorized
            else:
                preentry_ready = self._arm_direct_xy_preentry_ready(now)
                self.arm_direct_xy_preauthorized = preentry_ready
        if self.ARM_DIRECT_XY_OWNERSHIP and motion_active:
            if not self.arm_direct_xy_active:
                # Prepared and ACK transitions are event-driven in the
                # ownership callback group.  The timer only enforces the
                # bounded stage-one deadline while continuing finite PX4 XY.
                self.arm_motion_inhibit_pub.publish(
                    Bool(data=not self.arm_direct_xy_preauthorized)
                )
                if now >= self.arm_direct_xy_entry_deadline:
                    self._abort_arm_for_direct_xy_fault("entry_gate_timeout")
            if self.arm_direct_xy_active:
                # Synchronize the physical ACK captured by the dedicated
                # safety ingress before evaluating either runtime health or
                # the bounded entry deadline.  This keeps one physical source
                # of truth even when the main report snapshot is delayed.
                self._adopt_arm_direct_xy_force_lease_ack(now)
                runtime_fault = self._arm_direct_xy_runtime_fault_reason(now)
                if runtime_fault is not None:
                    self._abort_arm_for_direct_xy_fault(
                        "runtime_watchdog", detail=runtime_fault
                    )
                elif (
                    self.arm_direct_xy_force_ever_ack
                    and not self.arm_direct_xy_force_enabled_ack
                ):
                    # After the direct force owner has acknowledged once, any
                    # later loss is a runtime owner failure, not a new entry
                    # attempt.  Restore finite PX4 XY immediately.
                    self._abort_arm_for_direct_xy_fault("direct_force_ack_lost")
                elif (
                    not self.arm_direct_xy_force_enabled_ack
                    and self.arm_direct_xy_force_ack_deadline > 0.0
                    and now >= self.arm_direct_xy_force_ack_deadline
                ):
                    self._abort_arm_for_direct_xy_fault("direct_force_ack_timeout")
                else:
                    # Mixed-axis contract: no PX4 horizontal position or
                    # velocity command, zero horizontal acceleration
                    # feed-forward, while the existing truth-hold Z velocity
                    # and finite yaw remain valid.  ``mode.velocity`` is the
                    # highest required estimator/control level because Z is
                    # still a velocity-controlled axis; PX4 then consumes the
                    # finite acceleration values independently on X/Y.
                    nan = float("nan")
                    sp = TrajectorySetpoint()
                    sp.timestamp = mode.timestamp
                    sp.position = [nan, nan, nan]
                    sp.velocity = [nan, nan, float(velocity_ned[2])]
                    sp.acceleration = [0.0, 0.0, nan]
                    sp.jerk = [nan, nan, nan]
                    sp.yaw = self.target.yaw
                    sp.yawspeed = nan
                    # Atomic entry order is intentional: PX4 receives the
                    # mixed-axis setpoint before the reallocator is allowed to
                    # apply horizontal force.  The enable command is refreshed
                    # by the independent safety lease only after this actual
                    # mixed-setpoint publish updates main-controller liveness.
                    self.setpoint_pub.publish(sp)
                    self._note_arm_direct_xy_mixed_setpoint_published(
                        time.monotonic()
                    )
                    direct_ack = bool(
                        self.arm_direct_xy_force_enabled_ack
                        and self.arm_direct_xy_force_epoch_ack
                        == self.arm_direct_xy_ownership_epoch
                    )
                    if direct_ack:
                        self.arm_direct_xy_force_ack_deadline = 0.0
                    self.arm_direct_xy_ready_pub.publish(Bool(data=True))
                    self.arm_motion_inhibit_pub.publish(Bool(data=False))
                    self._publish_arm_direct_xy_state(
                        "direct_xy" if direct_ack else "handoff", now=now
                    )
                    return
        elif self.arm_direct_xy_active:
            self.arm_direct_xy_exit_disable_pending = bool(
                self.arm_direct_xy_force_commanded
                or self.arm_direct_xy_force_enabled_ack
            )
            self.arm_direct_xy_active = False
            self.get_logger().info(
                "ARM_DIRECT_XY_EXIT_STARTED; restoring finite PX4 XY before revoke"
            )

        if (
            not self.ARM_DIRECT_XY_OWNERSHIP
            and self.TRUTH_HOLD_ARM_POSITION_OVERLAY
            and motion_active
        ):
            # The canted-rotor reallocator owns horizontal world-position PD
            # during an arm session.  Request zero horizontal velocity from
            # PX4 so its normal velocity/attitude/rate loops stay active but
            # do not create a competing position-to-tilt loop.  Altitude
            # remains on the existing truth-hold branch.
            velocity_ned[:2] = 0.0
        if self.ARM_DIRECT_XY_OWNERSHIP:
            # ``ready`` is a pre-authorisation handshake and may therefore be
            # true while PX4 still publishes finite XY.  Actual ownership is
            # proven only by state=direct_xy plus the mixed-axis setpoint.
            grant = bool(
                not self.arm_direct_xy_abort_latched
                and (preentry_ready or self.arm_direct_xy_active)
            )
            self.arm_direct_xy_ready_pub.publish(Bool(data=grant))
            self.arm_motion_inhibit_pub.publish(Bool(data=not grant))
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
        # Atomic exit order is the inverse of entry: establish finite PX4 XY
        # first, then revoke the direct force owner.  There is therefore no
        # interval with neither horizontal outer loop active.
        self._disable_direct_xy_force_after_finite_px4()
        if self.ARM_DIRECT_XY_OWNERSHIP:
            self._publish_arm_direct_xy_state(
                "aborting"
                if self.arm_direct_xy_abort_latched
                else (
                    "exit"
                    if self.arm_direct_xy_exit_disable_pending
                    else "px4_xy"
                ),
                watchdog_reason=self.arm_direct_xy_last_fault,
                watchdog_detail=getattr(
                    self, "arm_direct_xy_last_fault_detail", ""
                ),
                now=now,
            )

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
        self._teardown_direct_xy_before_mode_exit("landing_requested")
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
        self._teardown_direct_xy_before_mode_exit("offboard_exit_requested")
        self.publish_command(
            VehicleCommand.VEHICLE_CMD_DO_SET_MODE, param1=1.0, param2=3.0
        )
        self.offboard_requested = False
        self.control_state = FlightControlState.POSITION_HOLD
        self._clear_velocity_command()
        self.get_logger().warning("Requested POSCTL and stopped Offboard stream")

    def emergency_disarm(self) -> None:
        self._teardown_direct_xy_before_mode_exit("emergency_disarm_requested")
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

    def _report_state(self) -> None:
        """Emit human-readable diagnostics outside the control callback group."""
        status = self.status
        local = self.local
        if status is None or local is None:
            return
        now = time.monotonic()
        attitude_rpy = (
            self._quaternion_wxyz_to_rpy(self.odometry.q)
            if self.odometry is not None
            else np.zeros(3)
        )
        angular_velocity = (
            np.asarray(self.odometry.angular_velocity, dtype=float)
            if self.odometry is not None
            else np.zeros(3)
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
            f"arm={status.arming_state} nav={status.nav_state} "
            f"NED=({local.x:.3f},{local.y:.3f},{local.z:.3f}) "
            f"vel=({local.vx:.3f},{local.vy:.3f},{local.vz:.3f}) "
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
            f"yaw_deg={math.degrees(local.heading):.1f} "
            f"rpy_deg=({math.degrees(attitude_rpy[0]):.1f},"
            f"{math.degrees(attitude_rpy[1]):.1f},"
            f"{math.degrees(attitude_rpy[2]):.1f}) "
            f"body_rate_deg_s=({math.degrees(angular_velocity[0]):.1f},"
            f"{math.degrees(angular_velocity[1]):.1f},"
            f"{math.degrees(angular_velocity[2]):.1f}) "
            f"yaw_rate_sp_deg_s={math.degrees(self.yaw_rate_command):.1f} "
            f"failsafe={status.failsafe} "
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

    def _tick(self) -> None:
        now = time.monotonic()
        dt = now - self.last_control_tick_monotonic
        self.last_control_tick_monotonic = now
        self.publish_rl_observation()
        # High-volume human-readable STATE logging used to run synchronously
        # inside this safety-critical control timer.  PTY/log backpressure
        # produced a measured 183 ms tick gap and allowed the direct-force
        # command heartbeat to expire.  Runtime state is already captured from
        # its authoritative ROS topics by acceptance_telemetry_recorder, so the
        # control path must remain free of console I/O.
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
            touchdown_height_gate_m = (
                self.TOUCHDOWN_DISARM_HEIGHT_M
                + self.TOUCHDOWN_CONTACT_ALLOWANCE_M
            )
            touchdown_stable = (
                abs(float(self.local.z) - self.takeoff_ground_down)
                <= touchdown_height_gate_m
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
                        self._teardown_direct_xy_before_mode_exit(
                            "touchdown_force_disarm"
                        )
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
            self._teardown_direct_xy_before_mode_exit("local_position_timeout")
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
            self._teardown_direct_xy_before_mode_exit("native_land_handoff")
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
    safety_node = DirectXySafetyIngress(node)
    # The main executor remains multi-threaded for PX4/Gazebo state and the
    # control timer.  Ownership ingress has its own executor below; adding more
    # main workers did not provide deterministic priority for the 40 ms ACK.
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    # One worker services ordered motion/reallocator reports while the other
    # keeps the independent force lease alive.  A single worker reintroduced
    # nondeterministic owner-entry timeouts under the 10 ms lease timer load.
    safety_executor = MultiThreadedExecutor(num_threads=2)
    safety_executor.add_node(safety_node)
    executor_thread = threading.Thread(
        target=executor.spin,
        name="dds-wasd-ros-executor",
    )
    safety_executor_thread = threading.Thread(
        target=safety_executor.spin,
        name="dds-wasd-safety-executor",
    )
    safety_executor_thread.start()
    executor_thread.start()
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
                with node.control_lock:
                    if cli.arm_only and not arm_only_started and node.state_fresh():
                        arm_only_started = node.begin_arm_only()
                key = terminal.read_key(0.02)
                if key:
                    with node.control_lock:
                        node.handle_key(key)
                if cli.arm_only and time.monotonic() - start > 15.0:
                    raise RuntimeError("arm-only test timed out")
    except KeyboardInterrupt:
        with node.control_lock:
            if node.status and node.status.arming_state == VehicleStatus.ARMING_STATE_ARMED:
                node.land()
            end = time.monotonic() + 1.0
        while time.monotonic() < end and rclpy.ok():
            time.sleep(0.05)
    finally:
        # Keep both nodes alive through any finite-PX4-before-force-disable
        # teardown above.  Stop executors before destroying their nodes to
        # avoid rclpy Destroyable callbacks racing node destruction.
        node.stop_setpoint_keepalive()
        executor.shutdown(timeout_sec=2.0)
        executor_thread.join(timeout=2.0)
        safety_executor.shutdown(timeout_sec=2.0)
        safety_executor_thread.join(timeout=2.0)
        safety_node.destroy_node()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
