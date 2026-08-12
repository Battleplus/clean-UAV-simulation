"""Sensor-side arm torque disturbance observer for the 4 kg debug model.

The observer deliberately does not use Gazebo truth.  It compares rigid-body
torque inferred from PX4's FRD gyroscope with the torque predicted from the
actual eight actuator commands and the CAD allocation.  A slowly learned
static residual is frozen while the arm moves; only the bounded incremental
residual is published as a candidate compensation torque.

The node is experimental and is launched only when explicitly enabled.  Its
output is evidence for a paired A/B test, not a formal propeller model.
"""

from __future__ import annotations

import argparse
import json
from pathlib import Path
import time

import numpy as np

try:
    from actuator_msgs.msg import Actuators
    from geometry_msgs.msg import WrenchStamped
    from px4_msgs.msg import SensorCombined, VehicleStatus
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import (
        DurabilityPolicy,
        HistoryPolicy,
        QoSProfile,
        ReliabilityPolicy,
    )
    from std_msgs.msg import Bool, String
except ModuleNotFoundError:  # Pure functions remain importable in unit tests.
    Actuators = None
    WrenchStamped = None
    SensorCombined = None
    VehicleStatus = None
    rclpy = None
    Node = object
    Bool = None
    String = None

from drone_arm_sim.gazebo_direct_motor_model import (
    actuator_command_to_thrust_n,
    first_order_motor_step,
)


FLU_TO_FRD = np.diag([1.0, -1.0, -1.0])


def inertia_tensor_flu_to_frd(inertia_flu_kg_m2: np.ndarray) -> np.ndarray:
    """Transform a full inertia tensor from Gazebo FLU to PX4 FRD."""
    inertia = np.asarray(inertia_flu_kg_m2, dtype=float)
    if inertia.shape != (3, 3) or not np.all(np.isfinite(inertia)):
        raise ValueError("inertia tensor must be a finite 3x3 matrix")
    if not np.allclose(inertia, inertia.T, atol=1.0e-9):
        raise ValueError("inertia tensor must be symmetric")
    converted = FLU_TO_FRD @ inertia @ FLU_TO_FRD.T
    if np.min(np.linalg.eigvalsh(converted)) <= 0.0:
        raise ValueError("inertia tensor must be positive definite")
    return converted


def first_order_step(
    current: np.ndarray, target: np.ndarray, dt_s: float, time_constant_s: float
) -> np.ndarray:
    """Advance an exact first-order filter without sample-rate dependence."""
    current = np.asarray(current, dtype=float)
    target = np.asarray(target, dtype=float)
    if current.shape != target.shape:
        raise ValueError("current and target shapes must match")
    dt = max(0.0, float(dt_s))
    tau = max(0.0, float(time_constant_s))
    alpha = 1.0 if tau == 0.0 else 1.0 - np.exp(-dt / tau)
    return current + alpha * (target - current)


def bound_vector(values: np.ndarray, maximum_norm: float) -> np.ndarray:
    """Return a finite vector with its Euclidean norm bounded."""
    vector = np.asarray(values, dtype=float).copy()
    if vector.shape != (3,) or not np.all(np.isfinite(vector)):
        return np.zeros(3)
    limit = max(0.0, float(maximum_norm))
    norm = float(np.linalg.norm(vector))
    if limit == 0.0:
        return np.zeros(3)
    if norm > limit:
        vector *= limit / norm
    return vector


def motor_torque_about_dynamic_com_frd(
    config: dict,
    normalized_commands: np.ndarray,
    com_shift_body_flu_m: np.ndarray | None = None,
) -> np.ndarray:
    """Predict the eight-rotor torque about the current whole-vehicle COM.

    ``position_m`` is frozen about the retracted reference COM.  Moving the
    whole-vehicle COM by ``com_shift`` shortens every rotor lever arm by the
    same vector.  Reaction torque retains the configured Q/T assumption; this
    is why the observer remains debug evidence until propeller physics freezes.
    """
    commands = np.asarray(normalized_commands, dtype=float)
    if commands.shape != (8,) or not np.all(np.isfinite(commands)):
        raise ValueError("normalized_commands must contain eight finite values")
    shift_flu = (
        np.zeros(3)
        if com_shift_body_flu_m is None
        else np.asarray(com_shift_body_flu_m, dtype=float)
    )
    if shift_flu.shape != (3,) or not np.all(np.isfinite(shift_flu)):
        raise ValueError("com_shift_body_flu_m must be a finite 3-vector")
    shift_frd = FLU_TO_FRD @ shift_flu
    moment_ratio = float(config.get("reaction_moment_ratio_m") or 0.0)
    total = np.zeros(3)
    rotors = sorted(config["rotors"], key=lambda item: int(item["motor"]))
    if len(rotors) != 8:
        raise ValueError("observer requires exactly eight rotors")
    for rotor, command in zip(rotors, np.clip(commands, 0.0, 1.0), strict=True):
        axis = np.asarray(
            rotor.get("thrust_axis_body", rotor["axis_body"]), dtype=float
        )
        axis_norm = float(np.linalg.norm(axis))
        if axis.shape != (3,) or not np.isfinite(axis_norm) or axis_norm <= 1.0e-9:
            raise ValueError("each rotor requires a finite nonzero thrust axis")
        force = (
            actuator_command_to_thrust_n(config, float(command))
            * axis
            / axis_norm
        )
        lever = np.asarray(rotor["position_m"], dtype=float) - shift_frd
        direction = float(rotor["direction"])
        total += np.cross(lever, force) - direction * moment_ratio * force
    return total


def rigid_body_residual_torque_frd(
    angular_acceleration_frd_rad_s2: np.ndarray,
    angular_velocity_frd_rad_s: np.ndarray,
    inertia_diag_kg_m2: np.ndarray,
    predicted_motor_torque_frd_nm: np.ndarray,
) -> np.ndarray:
    """Estimate unmodelled body torque from Euler's rigid-body equation."""
    alpha = np.asarray(angular_acceleration_frd_rad_s2, dtype=float)
    omega = np.asarray(angular_velocity_frd_rad_s, dtype=float)
    inertia = np.asarray(inertia_diag_kg_m2, dtype=float)
    motor = np.asarray(predicted_motor_torque_frd_nm, dtype=float)
    if alpha.shape != (3,) or omega.shape != (3,) or motor.shape != (3,):
        raise ValueError("alpha, omega and motor torque must be 3-vectors")
    if inertia.shape == (3,):
        if np.any(inertia <= 0.0):
            raise ValueError("inertia diagonal must be positive")
        inertia_matrix = np.diag(inertia)
    elif inertia.shape == (3, 3):
        if not np.allclose(inertia, inertia.T, atol=1.0e-9):
            raise ValueError("inertia tensor must be symmetric")
        if np.min(np.linalg.eigvalsh(inertia)) <= 0.0:
            raise ValueError("inertia tensor must be positive definite")
        inertia_matrix = inertia
    else:
        raise ValueError("inertia must be a 3-vector or 3x3 tensor")
    if not all(np.all(np.isfinite(vector)) for vector in (alpha, omega, inertia_matrix, motor)):
        raise ValueError("all rigid-body inputs must be finite")
    angular_momentum = inertia_matrix @ omega
    return inertia_matrix @ alpha + np.cross(omega, angular_momentum) - motor


class BoundedTorqueDisturbanceObserver:
    """Learn a static model residual and expose only arm-motion increments."""

    def __init__(
        self,
        *,
        angular_acceleration_time_constant_s: float = 0.08,
        baseline_time_constant_s: float = 4.0,
        estimate_time_constant_s: float = 0.18,
        decay_time_constant_s: float = 0.15,
        maximum_torque_nm: float = 0.08,
        maximum_angular_acceleration_rad_s2: float = 20.0,
    ) -> None:
        self.angular_acceleration_time_constant_s = max(
            0.0, float(angular_acceleration_time_constant_s)
        )
        self.baseline_time_constant_s = max(0.0, float(baseline_time_constant_s))
        self.estimate_time_constant_s = max(0.0, float(estimate_time_constant_s))
        self.decay_time_constant_s = max(0.0, float(decay_time_constant_s))
        self.maximum_torque_nm = max(0.0, float(maximum_torque_nm))
        self.maximum_angular_acceleration_rad_s2 = max(
            0.0, float(maximum_angular_acceleration_rad_s2)
        )
        self.previous_gyro: np.ndarray | None = None
        self.filtered_angular_acceleration = np.zeros(3)
        self.baseline_residual_torque = np.zeros(3)
        self.estimated_disturbance_torque = np.zeros(3)
        self.baseline_samples = 0
        self.baseline_duration_s = 0.0
        self.last_raw_residual = np.zeros(3)

    def reset(self) -> None:
        self.previous_gyro = None
        self.filtered_angular_acceleration[:] = 0.0
        self.baseline_residual_torque[:] = 0.0
        self.estimated_disturbance_torque[:] = 0.0
        self.baseline_samples = 0
        self.baseline_duration_s = 0.0
        self.last_raw_residual[:] = 0.0

    def step(
        self,
        *,
        angular_velocity_frd_rad_s: np.ndarray,
        predicted_motor_torque_frd_nm: np.ndarray,
        inertia_diag_kg_m2: np.ndarray,
        dt_s: float,
        armed: bool,
        arm_motion_active: bool,
        sensor_clipped: bool = False,
    ) -> tuple[np.ndarray, bool]:
        """Return bounded disturbance torque and whether it is usable."""
        omega = np.asarray(angular_velocity_frd_rad_s, dtype=float)
        dt = float(dt_s)
        valid = (
            omega.shape == (3,)
            and np.all(np.isfinite(omega))
            and 1.0e-4 <= dt <= 0.1
            and not sensor_clipped
        )
        if not valid:
            self.previous_gyro = omega.copy() if omega.shape == (3,) else None
            self.estimated_disturbance_torque[:] = 0.0
            return np.zeros(3), False
        if self.previous_gyro is None:
            self.previous_gyro = omega.copy()
            return np.zeros(3), False

        raw_alpha = (omega - self.previous_gyro) / dt
        self.previous_gyro = omega.copy()
        raw_alpha = bound_vector(
            raw_alpha, self.maximum_angular_acceleration_rad_s2
        )
        self.filtered_angular_acceleration = first_order_step(
            self.filtered_angular_acceleration,
            raw_alpha,
            dt,
            self.angular_acceleration_time_constant_s,
        )
        try:
            residual = rigid_body_residual_torque_frd(
                self.filtered_angular_acceleration,
                omega,
                inertia_diag_kg_m2,
                predicted_motor_torque_frd_nm,
            )
        except ValueError:
            self.estimated_disturbance_torque[:] = 0.0
            return np.zeros(3), False
        self.last_raw_residual = residual

        if not armed:
            self.baseline_samples = 0
            self.baseline_duration_s = 0.0
            self.baseline_residual_torque[:] = 0.0
            self.estimated_disturbance_torque[:] = 0.0
            return np.zeros(3), False

        if not arm_motion_active:
            self.baseline_residual_torque = first_order_step(
                self.baseline_residual_torque,
                residual,
                dt,
                self.baseline_time_constant_s,
            )
            self.baseline_samples += 1
            self.baseline_duration_s += dt
            self.estimated_disturbance_torque = first_order_step(
                self.estimated_disturbance_torque,
                np.zeros(3),
                dt,
                self.decay_time_constant_s,
            )
            return np.zeros(3), False

        # Do not engage before at least 0.2 s of armed, arm-static baseline.
        if self.baseline_duration_s < 0.2:
            self.estimated_disturbance_torque[:] = 0.0
            return np.zeros(3), False
        increment = residual - self.baseline_residual_torque
        self.estimated_disturbance_torque = first_order_step(
            self.estimated_disturbance_torque,
            increment,
            dt,
            self.estimate_time_constant_s,
        )
        self.estimated_disturbance_torque = bound_vector(
            self.estimated_disturbance_torque, self.maximum_torque_nm
        )
        return self.estimated_disturbance_torque.copy(), True


class ArmDisturbanceObserverNode(Node):
    def __init__(
        self,
        config_path: Path,
        *,
        velocity_command_scale: float,
        maximum_torque_nm: float,
    ) -> None:
        super().__init__("my_drone_arm_disturbance_observer")
        self.config = json.loads(config_path.read_text(encoding="utf-8"))
        self.velocity_command_scale = max(1.0e-9, float(velocity_command_scale))
        self.observer = BoundedTorqueDisturbanceObserver(
            maximum_torque_nm=maximum_torque_nm
        )
        self.motor_commands = np.zeros(8)
        self.filtered_motor_commands = np.zeros(8)
        self.last_motor_monotonic = 0.0
        self.inertia_tensor = np.full((3, 3), float("nan"))
        self.com_shift_flu = np.zeros(3)
        self.last_coupling_monotonic = 0.0
        self.arm_motion_active = False
        self.last_motion_monotonic = 0.0
        self.armed = False
        self.last_status_monotonic = 0.0
        self.last_sensor_timestamp_us: int | None = None
        self.last_report_monotonic = 0.0

        px4_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self.publisher = self.create_publisher(
            WrenchStamped,
            "/my_drone/arm_observer_disturbance_wrench_body_frd",
            10,
        )
        self.diagnostic_publisher = self.create_publisher(
            String, "/my_drone/arm_observer_state", 10
        )
        self.create_subscription(
            Actuators,
            "/my_drone/command/motor_speed",
            self._motor_cb,
            20,
        )
        self.create_subscription(
            SensorCombined,
            "/fmu/out/sensor_combined",
            self._sensor_cb,
            px4_qos,
        )
        self.create_subscription(
            VehicleStatus,
            "/fmu/out/vehicle_status_v4",
            self._status_cb,
            px4_qos,
        )
        self.create_subscription(
            String,
            "/my_drone/arm_coupling_state",
            self._coupling_cb,
            10,
        )
        self.create_subscription(
            Bool,
            "/my_drone/arm_motion_active",
            self._motion_cb,
            10,
        )
        self.get_logger().warning(
            "experimental torque disturbance observer active; output remains "
            "bounded and arm-motion gated"
        )

    def _motor_cb(self, message) -> None:
        normalized = np.asarray(message.normalized, dtype=float)
        if normalized.size >= 8 and np.all(np.isfinite(normalized[:8])):
            self.motor_commands = np.clip(normalized[:8], 0.0, 1.0)
        else:
            velocity = np.asarray(message.velocity, dtype=float)
            if velocity.size < 8 or not np.all(np.isfinite(velocity[:8])):
                return
            self.motor_commands = np.clip(
                velocity[:8] / self.velocity_command_scale, 0.0, 1.0
            )
        self.last_motor_monotonic = time.monotonic()

    def _status_cb(self, message) -> None:
        self.armed = bool(
            message.arming_state == VehicleStatus.ARMING_STATE_ARMED
            and not message.failsafe
        )
        self.last_status_monotonic = time.monotonic()

    def _motion_cb(self, message) -> None:
        self.arm_motion_active = bool(message.data)
        self.last_motion_monotonic = time.monotonic()

    def _coupling_cb(self, message) -> None:
        try:
            report = json.loads(message.data)
            if "inertia_tensor_kg_m2" in report:
                if report.get("inertia_tensor_frame") != "base_link_flu":
                    return
                inertia = inertia_tensor_flu_to_frd(
                    np.asarray(report["inertia_tensor_kg_m2"], dtype=float)
                )
            else:
                inertia = np.diag(
                    np.asarray(report["inertia_diag_kg_m2"], dtype=float)
                )
            shift = np.asarray(report["com_shift_m"], dtype=float)
        except (KeyError, TypeError, ValueError, json.JSONDecodeError):
            return
        if (
            inertia.shape == (3, 3)
            and shift.shape == (3,)
            and np.all(np.isfinite(inertia))
            and np.allclose(inertia, inertia.T, atol=1.0e-9)
            and np.min(np.linalg.eigvalsh(inertia)) > 0.0
            and np.all(np.isfinite(shift))
        ):
            self.inertia_tensor = inertia
            self.com_shift_flu = shift
            self.last_coupling_monotonic = time.monotonic()

    def _publish(self, torque_frd: np.ndarray, active: bool, now: float) -> None:
        message = WrenchStamped()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = "px4_frd"
        message.wrench.torque.x = float(torque_frd[0])
        message.wrench.torque.y = float(torque_frd[1])
        message.wrench.torque.z = float(torque_frd[2])
        self.publisher.publish(message)
        if now - self.last_report_monotonic >= 1.0:
            self.last_report_monotonic = now
            report = {
                "active": bool(active),
                "armed": bool(self.armed),
                "arm_motion_active": bool(self.arm_motion_active),
                "baseline_samples": int(self.observer.baseline_samples),
                "baseline_torque_frd_nm": self.observer.baseline_residual_torque.tolist(),
                "raw_residual_torque_frd_nm": self.observer.last_raw_residual.tolist(),
                "estimated_disturbance_torque_frd_nm": torque_frd.tolist(),
            }
            self.diagnostic_publisher.publish(String(data=json.dumps(report)))
            self.get_logger().info("ARM_DOB_STATE " + json.dumps(report))

    def _sensor_cb(self, message) -> None:
        now = time.monotonic()
        timestamp_us = int(message.timestamp)
        if self.last_sensor_timestamp_us is None:
            dt_s = 0.0
        else:
            dt_s = 1.0e-6 * (timestamp_us - self.last_sensor_timestamp_us)
        self.last_sensor_timestamp_us = timestamp_us
        data_fresh = bool(
            now - self.last_motor_monotonic < 0.2
            and now - self.last_status_monotonic < 0.5
            and now - self.last_coupling_monotonic < 0.6
        )
        motion_active = bool(
            self.arm_motion_active and now - self.last_motion_monotonic < 1.0
        )
        if not data_fresh:
            self._publish(np.zeros(3), False, now)
            return
        try:
            self.filtered_motor_commands = first_order_motor_step(
                self.filtered_motor_commands,
                self.motor_commands,
                dt_s,
                self.config,
            )
            motor_torque = motor_torque_about_dynamic_com_frd(
                self.config, self.filtered_motor_commands, self.com_shift_flu
            )
        except ValueError:
            self._publish(np.zeros(3), False, now)
            return
        torque, active = self.observer.step(
            angular_velocity_frd_rad_s=np.asarray(message.gyro_rad, dtype=float),
            predicted_motor_torque_frd_nm=motor_torque,
            inertia_diag_kg_m2=self.inertia_tensor,
            dt_s=dt_s,
            armed=self.armed,
            arm_motion_active=motion_active,
            sensor_clipped=bool(message.gyro_clipping),
        )
        self._publish(torque if active else np.zeros(3), active, now)


def parse_cli_and_ros_args(args=None):
    """Parse node options while preserving ROS arguments for ``rclpy.init``.

    ROS 2 launch appends arguments such as ``--ros-args`` and node remaps to
    every executable.  Treating the complete command line as application-only
    arguments made the observer exit before publishing a single estimate.
    """
    package = Path(__file__).resolve().parents[1]
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--config",
        type=Path,
        default=package / "config" / "my_drone_v3_cad_debug_4kg.json",
    )
    parser.add_argument("--velocity-command-scale", type=float, default=1000.0)
    parser.add_argument("--maximum-torque-nm", type=float, default=0.08)
    return parser.parse_known_args(args)


def main(args=None) -> None:
    parsed, ros_arguments = parse_cli_and_ros_args(args)
    if rclpy is None:
        raise SystemExit("ROS 2 Python packages are required")
    rclpy.init(args=ros_arguments)
    node = ArmDisturbanceObserverNode(
        parsed.config,
        velocity_command_scale=parsed.velocity_command_scale,
        maximum_torque_nm=parsed.maximum_torque_nm,
    )
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
