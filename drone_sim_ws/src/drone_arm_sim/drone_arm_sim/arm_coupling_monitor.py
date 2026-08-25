"""ROS 2 monitor and bounded feed-forward for the aerial manipulator."""

from __future__ import annotations

import argparse
import json
from pathlib import Path
import time

import numpy as np

from drone_arm_sim.coupled_dynamics import CoupledArmDynamics, JOINT_NAMES, Payload
from drone_arm_sim.gazebo_wrench_controller import quaternion_matrix_xyzw

try:
    from control_msgs.msg import JointTrajectoryControllerState
    from geometry_msgs.msg import AccelStamped, WrenchStamped
    from nav_msgs.msg import Odometry
    import rclpy
    from rclpy.node import Node
    from sensor_msgs.msg import JointState
    from std_msgs.msg import String
except ModuleNotFoundError:  # Pure numerical tests do not require ROS.
    AccelStamped = WrenchStamped = Odometry = JointState = String = None
    JointTrajectoryControllerState = None
    rclpy = None
    Node = object


ENU_TO_NED = np.array([[0.0, 1.0, 0.0], [1.0, 0.0, 0.0], [0.0, 0.0, -1.0]])
ESTIMATOR_SOURCE_TIMEOUT_S = 0.10
REFERENCE_SOURCE_TIMEOUT_S = 0.10
REFERENCE_TRACKING_ERROR_RAD = 0.20


def estimator_source_is_fresh(age_s: float, timeout_s: float = ESTIMATOR_SOURCE_TIMEOUT_S) -> bool:
    return bool(
        np.isfinite(age_s)
        and age_s >= 0.0
        and np.isfinite(timeout_s)
        and timeout_s > 0.0
        and age_s <= timeout_s
    )


def validate_joint_state_sample(
    names: list[str] | tuple[str, ...],
    positions: list[float] | tuple[float, ...],
    velocities: list[float] | tuple[float, ...],
) -> tuple[dict[str, float], dict[str, float] | None, str | None]:
    """Validate one complete arm JointState sample without accepting NaN/Inf.

    ``sensor_msgs/JointState.velocity`` may legitimately be empty.  When it is
    absent the monitor derives velocity from position samples.  A partially
    populated velocity vector, a duplicate joint name, or a non-finite value
    is ambiguous and therefore fails closed.
    """
    sample_names = [str(name) for name in names]
    if len(sample_names) != len(set(sample_names)):
        return {}, None, "duplicate_joint_name"
    if len(positions) != len(sample_names):
        return {}, None, "position_size_mismatch"
    index = {name: offset for offset, name in enumerate(sample_names)}
    missing = [name for name in JOINT_NAMES if name not in index]
    if missing:
        return {}, None, "missing_required_joints:" + ",".join(missing)

    position_map = {
        name: float(positions[index[name]]) for name in JOINT_NAMES
    }
    if not all(np.isfinite(value) for value in position_map.values()):
        return {}, None, "non_finite_joint_position"

    if len(velocities) == 0:
        return position_map, None, None
    if len(velocities) != len(sample_names):
        return {}, None, "velocity_size_mismatch"
    velocity_map = {
        name: float(velocities[index[name]]) for name in JOINT_NAMES
    }
    if not all(np.isfinite(value) for value in velocity_map.values()):
        return {}, None, "non_finite_joint_velocity"
    return position_map, velocity_map, None


def validate_controller_reference(
    names: list[str] | tuple[str, ...],
    positions: list[float] | tuple[float, ...],
    velocities: list[float] | tuple[float, ...],
    accelerations: list[float] | tuple[float, ...],
) -> tuple[
    dict[str, float], dict[str, float], dict[str, float], str | None
]:
    """Validate and reorder one complete JTC desired-state sample.

    Dynamic feed-forward is useful only when position, velocity, and
    acceleration describe the same controller reference.  Missing vectors,
    partial vectors, duplicate names, and non-finite values therefore fail
    closed instead of falling back to differentiated measurements.
    """
    sample_names = [str(name) for name in names]
    if len(sample_names) != len(set(sample_names)):
        return {}, {}, {}, "reference_duplicate_joint_name"
    index = {name: offset for offset, name in enumerate(sample_names)}
    missing = [name for name in JOINT_NAMES if name not in index]
    if missing:
        return {}, {}, {}, "reference_missing_required_joints:" + ",".join(missing)
    vectors = (
        ("position", positions),
        ("velocity", velocities),
        ("acceleration", accelerations),
    )
    for label, values in vectors:
        if len(values) != len(sample_names):
            return {}, {}, {}, f"reference_{label}_size_mismatch"

    reordered = []
    for label, values in vectors:
        result = {name: float(values[index[name]]) for name in JOINT_NAMES}
        if not all(np.isfinite(value) for value in result.values()):
            return {}, {}, {}, f"reference_non_finite_joint_{label}"
        reordered.append(result)
    return reordered[0], reordered[1], reordered[2], None


def desired_reference_gate(
    enabled: bool,
    sample_valid: bool,
    invalid_reason: str | None,
    age_s: float,
    timeout_s: float,
    actual_positions: dict[str, float],
    reference_positions: dict[str, float],
    tracking_error_limit_rad: float,
) -> tuple[bool, str | None, float | None]:
    """Gate reference-only dynamic reaction without a measured-state fallback."""
    if not enabled:
        return False, "dynamic_reaction_disabled", None
    if not estimator_source_is_fresh(age_s, timeout_s):
        return False, "controller_reference_stale", None
    if not sample_valid:
        return False, invalid_reason or "invalid_controller_reference", None
    if not all(name in actual_positions for name in JOINT_NAMES):
        return False, "incomplete_joint_state_for_tracking_gate", None
    if not all(name in reference_positions for name in JOINT_NAMES):
        return False, "incomplete_controller_reference", None
    errors = [
        abs(float(reference_positions[name]) - float(actual_positions[name]))
        for name in JOINT_NAMES
    ]
    if not all(np.isfinite(value) for value in errors):
        return False, "non_finite_reference_tracking_error", None
    maximum_error = max(errors, default=0.0)
    limit = float(tracking_error_limit_rad)
    if not np.isfinite(limit) or limit <= 0.0:
        return False, "invalid_reference_tracking_error_limit", maximum_error
    if maximum_error > limit:
        return False, "reference_tracking_error_exceeded", maximum_error
    return True, None, maximum_error


def evaluate_coupling_sources(
    dynamics: CoupledArmDynamics,
    actual_positions: dict[str, float],
    reference_positions: dict[str, float],
    reference_velocities: dict[str, float],
    reference_accelerations: dict[str, float],
    dynamic_reaction_active: bool,
    payload: Payload | None,
) -> tuple[float, np.ndarray, np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
    """Combine actual configuration properties with reference-only dynamics."""
    mass_kg, center_of_mass_m, inertia_at_com_kg_m2 = dynamics.mass_properties(
        actual_positions, payload
    )
    home_center_m = (
        dynamics.home_com
        if payload is None
        else dynamics.mass_properties(dynamics.home_positions, payload)[1]
    )
    com_shift_m = center_of_mass_m - home_center_m
    reaction_force_body_n = np.zeros(3)
    reaction_torque_body_nm = np.zeros(3)
    if dynamic_reaction_active:
        reaction_state = dynamics.state(
            reference_positions,
            reference_velocities,
            reference_accelerations,
            payload,
        )
        reaction_force_body_n = reaction_state.reaction_force_body_n
        reaction_torque_body_nm = reaction_state.reaction_torque_body_nm
    return (
        mass_kg,
        center_of_mass_m,
        inertia_at_com_kg_m2,
        com_shift_m,
        reaction_force_body_n,
        reaction_torque_body_nm,
    )


def joint_motion_metrics(
    velocities: dict[str, float], accelerations: dict[str, float]
) -> dict[str, float]:
    """Return finite scalar motion gates used by quasi-static compensation."""
    velocity_values = [float(velocities.get(name, 0.0)) for name in JOINT_NAMES]
    acceleration_values = [
        float(accelerations.get(name, 0.0)) for name in JOINT_NAMES
    ]
    maximum_velocity = (
        max(abs(value) for value in velocity_values)
        if all(np.isfinite(value) for value in velocity_values)
        else float("nan")
    )
    acceleration_peak = (
        max(abs(value) for value in acceleration_values)
        if all(np.isfinite(value) for value in acceleration_values)
        else float("nan")
    )
    return {
        "maximum_joint_velocity_rad_s": float(maximum_velocity),
        "filtered_joint_acceleration_peak_rad_s2": float(acceleration_peak),
    }


def filtered_joint_acceleration_step(
    previous_filtered_rad_s2: float,
    raw_rad_s2: float,
    dt_s: float,
    time_constant_s: float,
    maximum_rad_s2: float,
) -> float:
    """Apply a sample-rate-independent first-order acceleration filter."""
    previous = (
        float(previous_filtered_rad_s2)
        if np.isfinite(previous_filtered_rad_s2)
        else 0.0
    )
    if not np.isfinite(raw_rad_s2) or not np.isfinite(dt_s) or dt_s <= 0.0:
        return previous
    tau = (
        max(0.0, float(time_constant_s))
        if np.isfinite(time_constant_s)
        else 0.0
    )
    alpha = 1.0 if tau <= 0.0 else 1.0 - np.exp(-float(dt_s) / tau)
    value = previous + float(alpha) * (float(raw_rad_s2) - previous)
    limit = (
        max(0.0, float(maximum_rad_s2))
        if np.isfinite(maximum_rad_s2)
        else 0.0
    )
    return float(np.clip(value, -limit, limit)) if limit > 0.0 else value


def bounded_compensation_ned(
    reaction_force_body_flu_n: np.ndarray,
    mass_kg: float,
    rotation_body_flu_to_world_enu: np.ndarray,
    limit_m_s2: float,
) -> np.ndarray:
    """Convert an arm base-reaction estimate to bounded PX4 NED acceleration."""
    if mass_kg <= 0.0:
        raise ValueError("mass_kg must be positive")
    body = -np.asarray(reaction_force_body_flu_n, dtype=float) / mass_kg
    ned = ENU_TO_NED @ np.asarray(rotation_body_flu_to_world_enu, dtype=float) @ body
    norm = float(np.linalg.norm(ned))
    limit = max(0.0, float(limit_m_s2))
    if limit > 0.0 and norm > limit:
        ned *= limit / norm
    return ned


class ArmCouplingMonitor(Node):
    def __init__(
        self,
        urdf: Path,
        motion_reference: Path,
        rate_hz: float,
        target_mass_kg: float,
        payload: Payload | None,
        feedforward_limit_m_s2: float,
        acceleration_filter_time_constant_s: float,
        maximum_joint_acceleration_rad_s2: float,
        dynamic_reaction_enabled: bool = False,
        reference_timeout_s: float = REFERENCE_SOURCE_TIMEOUT_S,
        reference_tracking_error_rad: float = REFERENCE_TRACKING_ERROR_RAD,
    ) -> None:
        super().__init__("my_drone_arm_coupling_monitor")
        self.dynamics = CoupledArmDynamics(
            urdf, motion_reference, target_mass_kg=target_mass_kg
        )
        self.positions: dict[str, float] = {}
        self.velocities: dict[str, float] = {}
        self.accelerations: dict[str, float] = {}
        self.previous_velocities: dict[str, float] = {}
        self.previous_joint_time: float | None = None
        self.last_joint_source_stamp_s: float | None = None
        self.last_joint_receipt_monotonic_s: float | None = None
        self.last_joint_sample_valid = False
        self.last_joint_invalid_reason = "no_joint_state"
        self.rotation = np.eye(3)
        self.payload = payload
        self.feedforward_limit_m_s2 = float(feedforward_limit_m_s2)
        self.acceleration_filter_time_constant_s = max(
            0.0, float(acceleration_filter_time_constant_s)
        )
        self.maximum_joint_acceleration_rad_s2 = max(
            0.0, float(maximum_joint_acceleration_rad_s2)
        )
        self.raw_joint_acceleration_peak_rad_s2 = 0.0
        self.dynamic_reaction_enabled = bool(dynamic_reaction_enabled)
        self.reference_timeout_s = float(reference_timeout_s)
        self.reference_tracking_error_rad = float(reference_tracking_error_rad)
        self.reference_positions: dict[str, float] = {}
        self.reference_velocities: dict[str, float] = {}
        self.reference_accelerations: dict[str, float] = {}
        self.last_reference_source_stamp_s: float | None = None
        self.last_reference_receipt_monotonic_s: float | None = None
        self.last_reference_sample_valid = False
        self.last_reference_invalid_reason = "no_controller_reference"
        self.last_log_time = 0.0
        self.wrench_publisher = self.create_publisher(
            WrenchStamped, "/my_drone/arm_reaction_wrench_body", 10
        )
        self.gravity_shift_publisher = self.create_publisher(
            WrenchStamped, "/my_drone/arm_gravity_shift_wrench_body", 10
        )
        self.feedforward_publisher = self.create_publisher(
            AccelStamped, "/my_drone/arm_feedforward_acceleration_ned", 10
        )
        self.state_publisher = self.create_publisher(
            String, "/my_drone/arm_coupling_state", 10
        )
        self.create_subscription(JointState, "/joint_states", self.on_joint_state, 10)
        self.create_subscription(
            JointTrajectoryControllerState,
            "/arm_controller/controller_state",
            self.on_controller_state,
            10,
        )
        self.create_subscription(
            Odometry, "/model/my_drone/odometry", self.on_odometry, 10
        )
        self.create_timer(1.0 / max(0.2, float(rate_hz)), self.on_timer)

    @staticmethod
    def _stamp_seconds(message: JointState) -> float:
        stamp = message.header.stamp
        value = float(stamp.sec) + 1.0e-9 * float(stamp.nanosec)
        return value if value > 0.0 else time.monotonic()

    def on_joint_state(self, message: JointState) -> None:
        now = self._stamp_seconds(message)
        self.last_joint_source_stamp_s = now
        self.last_joint_receipt_monotonic_s = time.monotonic()
        values, direct_velocity, invalid_reason = validate_joint_state_sample(
            message.name, message.position, message.velocity
        )
        if invalid_reason is not None:
            # Retain the last finite state for diagnostics, but mark the newest
            # sample invalid so it can neither refresh compensation nor enter
            # the dynamics calculation.
            self.last_joint_sample_valid = False
            self.last_joint_invalid_reason = invalid_reason
            return

        dt = None if self.previous_joint_time is None else now - self.previous_joint_time
        for name in JOINT_NAMES:
            previous_position = self.positions.get(name)
            self.positions[name] = float(values[name])
            if direct_velocity is not None:
                velocity = float(direct_velocity[name])
            elif previous_position is not None and dt is not None and 1.0e-4 <= dt <= 0.5:
                velocity = (self.positions[name] - previous_position) / dt
            else:
                velocity = self.velocities.get(name, 0.0)
            previous_velocity = self.velocities.get(name, velocity)
            self.velocities[name] = velocity
            if dt is not None and 1.0e-4 <= dt <= 0.5:
                raw_acceleration = (velocity - previous_velocity) / dt
                self.raw_joint_acceleration_peak_rad_s2 = max(
                    self.raw_joint_acceleration_peak_rad_s2,
                    abs(float(raw_acceleration)),
                )
                self.accelerations[name] = filtered_joint_acceleration_step(
                    self.accelerations.get(name, 0.0),
                    raw_acceleration,
                    dt,
                    self.acceleration_filter_time_constant_s,
                    self.maximum_joint_acceleration_rad_s2,
                )
        self.previous_velocities = dict(self.velocities)
        self.previous_joint_time = now
        self.last_joint_sample_valid = True
        self.last_joint_invalid_reason = None

    @staticmethod
    def _strict_stamp_seconds(message: JointTrajectoryControllerState) -> float | None:
        stamp = message.header.stamp
        value = float(stamp.sec) + 1.0e-9 * float(stamp.nanosec)
        return value if np.isfinite(value) and value > 0.0 else None

    def on_controller_state(self, message: JointTrajectoryControllerState) -> None:
        """Accept only ordered, complete JTC reference q/qd/qdd samples."""
        self.last_reference_receipt_monotonic_s = time.monotonic()
        source_stamp_s = self._strict_stamp_seconds(message)
        if source_stamp_s is None:
            self.last_reference_sample_valid = False
            self.last_reference_invalid_reason = "reference_invalid_source_stamp"
            return
        if (
            self.last_reference_source_stamp_s is not None
            and source_stamp_s <= self.last_reference_source_stamp_s
        ):
            self.last_reference_sample_valid = False
            self.last_reference_invalid_reason = "reference_out_of_order"
            return

        positions, velocities, accelerations, invalid_reason = (
            validate_controller_reference(
                message.joint_names,
                message.reference.positions,
                message.reference.velocities,
                message.reference.accelerations,
            )
        )
        if invalid_reason is not None:
            self.last_reference_sample_valid = False
            self.last_reference_invalid_reason = invalid_reason
            return
        self.reference_positions = positions
        self.reference_velocities = velocities
        self.reference_accelerations = accelerations
        self.last_reference_source_stamp_s = source_stamp_s
        self.last_reference_sample_valid = True
        self.last_reference_invalid_reason = None

    def on_odometry(self, message: Odometry) -> None:
        orientation = message.pose.pose.orientation
        quaternion = np.array(
            [orientation.x, orientation.y, orientation.z, orientation.w], dtype=float
        )
        if np.all(np.isfinite(quaternion)) and np.linalg.norm(quaternion) > 1.0e-9:
            self.rotation = quaternion_matrix_xyzw(quaternion)

    def on_timer(self) -> None:
        monotonic = time.monotonic()
        source_age_s = (
            float("inf")
            if self.last_joint_receipt_monotonic_s is None
            else monotonic - self.last_joint_receipt_monotonic_s
        )
        source_fresh = estimator_source_is_fresh(source_age_s)
        reference_age_s = (
            float("inf")
            if self.last_reference_receipt_monotonic_s is None
            else monotonic - self.last_reference_receipt_monotonic_s
        )
        (
            dynamic_reaction_active,
            dynamic_reaction_invalid_reason,
            reference_tracking_error_rad,
        ) = desired_reference_gate(
            self.dynamic_reaction_enabled,
            self.last_reference_sample_valid,
            self.last_reference_invalid_reason,
            reference_age_s,
            self.reference_timeout_s,
            self.positions,
            self.reference_positions,
            self.reference_tracking_error_rad,
        )
        reference_report = {
            "dynamic_reaction_enabled": self.dynamic_reaction_enabled,
            "dynamic_reaction_active": dynamic_reaction_active,
            "dynamic_reaction_invalid_reason": dynamic_reaction_invalid_reason,
            "controller_reference_valid": self.last_reference_sample_valid,
            "controller_reference_age_s": (
                reference_age_s if np.isfinite(reference_age_s) else None
            ),
            "controller_reference_timeout_s": self.reference_timeout_s,
            "controller_reference_stamp_s": self.last_reference_source_stamp_s,
            "reference_tracking_error_rad": reference_tracking_error_rad,
            "reference_tracking_error_limit_rad": self.reference_tracking_error_rad,
            "dynamic_reaction_source": (
                "arm_controller/controller_state.reference"
                if dynamic_reaction_active
                else "zero_fail_closed"
            ),
        }
        complete_state = all(name in self.positions for name in JOINT_NAMES)
        motion_metrics = joint_motion_metrics(self.velocities, self.accelerations)
        motion_state_finite = all(np.isfinite(value) for value in motion_metrics.values())
        report_motion_metrics = {
            name: value if np.isfinite(value) else None
            for name, value in motion_metrics.items()
        }
        estimator_valid = bool(
            source_fresh
            and complete_state
            and self.last_joint_sample_valid
            and motion_state_finite
        )
        if not estimator_valid:
            if not source_fresh:
                invalid_reason = "joint_state_stale"
            elif not complete_state:
                invalid_reason = "incomplete_joint_state"
            elif not self.last_joint_sample_valid:
                invalid_reason = self.last_joint_invalid_reason or "invalid_joint_state"
            else:
                invalid_reason = "non_finite_joint_motion_state"
            report = {
                "schema": "my_drone.arm-coupling-state.v2",
                "estimator_mode": "read_only",
                "estimator_valid": False,
                "source_fresh": source_fresh,
                "joint_state_valid": bool(
                    complete_state and self.last_joint_sample_valid and motion_state_finite
                ),
                "invalid_reason": invalid_reason,
                "source_age_s": source_age_s if np.isfinite(source_age_s) else None,
                "source_timeout_s": ESTIMATOR_SOURCE_TIMEOUT_S,
                "source_joint_state_stamp_s": self.last_joint_source_stamp_s,
                "output_frame": "base_link_flu",
                **report_motion_metrics,
                **reference_report,
            }
            self.state_publisher.publish(String(data=json.dumps(report, sort_keys=True)))
            if monotonic - self.last_log_time >= 1.0:
                self.last_log_time = monotonic
                self.get_logger().warning(
                    "ARM_COUPLING_STALE " + json.dumps(report, sort_keys=True)
                )
            # Do not refresh WrenchStamped/AccelStamped timestamps from stale
            # joint data.  Downstream safety gates must observe a real timeout.
            return
        # The actual joint configuration remains authoritative for COM,
        # inertia, and gravity shift.  Dynamic reaction uses only the JTC's
        # coherent desired q/qd/qdd triple.  On any gate failure it becomes
        # zero; measured velocity differentiation is intentionally never
        # reused as a fallback dynamic source.
        (
            mass_kg,
            center_of_mass_m,
            inertia_at_com_kg_m2,
            com_shift_m,
            reaction_force_body_n,
            reaction_torque_body_nm,
        ) = evaluate_coupling_sources(
            self.dynamics,
            self.positions,
            self.reference_positions,
            self.reference_velocities,
            self.reference_accelerations,
            dynamic_reaction_active,
            self.payload,
        )
        now = self.get_clock().now()
        wrench = WrenchStamped()
        wrench.header.stamp = now.to_msg()
        wrench.header.frame_id = "base_link"
        wrench.wrench.force.x, wrench.wrench.force.y, wrench.wrench.force.z = (
            float(value) for value in reaction_force_body_n
        )
        wrench.wrench.torque.x, wrench.wrench.torque.y, wrench.wrench.torque.z = (
            float(value) for value in reaction_torque_body_nm
        )
        self.wrench_publisher.publish(wrench)

        gravity_body_n = self.rotation.T @ np.array(
            [0.0, 0.0, -mass_kg * 9.80665], dtype=float
        )
        gravity_shift = WrenchStamped()
        gravity_shift.header.stamp = wrench.header.stamp
        gravity_shift.header.frame_id = "base_link"
        gravity_torque = np.cross(com_shift_m, gravity_body_n)
        gravity_shift.wrench.torque.x, gravity_shift.wrench.torque.y, gravity_shift.wrench.torque.z = (
            float(value) for value in gravity_torque
        )
        self.gravity_shift_publisher.publish(gravity_shift)

        acceleration = bounded_compensation_ned(
            reaction_force_body_n,
            mass_kg,
            self.rotation,
            self.feedforward_limit_m_s2,
        )
        feedforward = AccelStamped()
        feedforward.header.stamp = wrench.header.stamp
        feedforward.header.frame_id = "px4_ned"
        feedforward.accel.linear.x, feedforward.accel.linear.y, feedforward.accel.linear.z = (
            float(value) for value in acceleration
        )
        self.feedforward_publisher.publish(feedforward)

        report = {
            "schema": "my_drone.arm-coupling-state.v2",
            "estimator_mode": "read_only",
            "estimator_valid": True,
            "source_fresh": True,
            "joint_state_valid": True,
            "invalid_reason": None,
            "source_age_s": source_age_s,
            "source_timeout_s": ESTIMATOR_SOURCE_TIMEOUT_S,
            "source_joint_state_stamp_s": self.last_joint_source_stamp_s,
            "output_stamp_s": float(now.nanoseconds) * 1.0e-9,
            "output_frame": "base_link_flu",
            "mass_kg": mass_kg,
            "com_body_flu_m": center_of_mass_m.tolist(),
            "com_shift_m": com_shift_m.tolist(),
            "inertia_diag_kg_m2": np.diag(inertia_at_com_kg_m2).tolist(),
            "inertia_tensor_kg_m2": inertia_at_com_kg_m2.tolist(),
            "inertia_tensor_frame": "base_link_flu",
            "reaction_force_body_n": reaction_force_body_n.tolist(),
            "reaction_torque_body_nm": reaction_torque_body_nm.tolist(),
            "gravity_shift_torque_body_nm": gravity_torque.tolist(),
            "raw_joint_acceleration_peak_rad_s2": self.raw_joint_acceleration_peak_rad_s2,
            **motion_metrics,
            "feedforward_acceleration_ned_m_s2": acceleration.tolist(),
            **reference_report,
        }
        encoded_report = json.dumps(report, sort_keys=True)
        self.state_publisher.publish(String(data=encoded_report))

        if monotonic - self.last_log_time >= 1.0:
            self.last_log_time = monotonic
            self.get_logger().info("ARM_COUPLING_STATE " + encoded_report)
            self.raw_joint_acceleration_peak_rad_s2 = 0.0


def main() -> None:
    package = Path(__file__).resolve().parents[1]
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--urdf",
        type=Path,
        default=package / "urdf/my_drone_v3/my_drone_cad_formal_dynamic.urdf",
    )
    parser.add_argument(
        "--motion-reference",
        type=Path,
        default=package / "config/so101_motion_reference.json",
    )
    parser.add_argument("--rate-hz", type=float, default=100.0)
    parser.add_argument("--target-mass-kg", type=float, default=7.735)
    parser.add_argument("--payload-mass-kg", type=float, default=0.0)
    parser.add_argument("--payload-offset", nargs=3, type=float, default=(0.08, 0.0, 0.0))
    parser.add_argument("--feedforward-limit-m-s2", type=float, default=0.6)
    parser.add_argument(
        "--acceleration-filter-time-constant-s", type=float, default=0.20
    )
    parser.add_argument(
        "--maximum-joint-acceleration-rad-s2", type=float, default=4.0
    )
    parser.add_argument(
        "--dynamic-reaction-enabled",
        action="store_true",
        help=(
            "enable JTC-reference dynamic reaction feed-forward; disabled by "
            "default and fail-closed when the reference is unusable"
        ),
    )
    parser.add_argument(
        "--reference-timeout-s", type=float, default=REFERENCE_SOURCE_TIMEOUT_S
    )
    parser.add_argument(
        "--reference-tracking-error-rad",
        type=float,
        default=REFERENCE_TRACKING_ERROR_RAD,
    )
    parsed, ros_arguments = parser.parse_known_args()
    if rclpy is None:
        raise SystemExit("ROS 2 Python packages are not available")
    payload = (
        Payload(parsed.payload_mass_kg, tuple(parsed.payload_offset))
        if parsed.payload_mass_kg > 0.0
        else None
    )
    rclpy.init(args=ros_arguments)
    node = ArmCouplingMonitor(
        parsed.urdf,
        parsed.motion_reference,
        parsed.rate_hz,
        parsed.target_mass_kg,
        payload,
        parsed.feedforward_limit_m_s2,
        parsed.acceleration_filter_time_constant_s,
        parsed.maximum_joint_acceleration_rad_s2,
        parsed.dynamic_reaction_enabled,
        parsed.reference_timeout_s,
        parsed.reference_tracking_error_rad,
    )
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
