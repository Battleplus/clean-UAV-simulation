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
    from geometry_msgs.msg import AccelStamped, WrenchStamped
    from nav_msgs.msg import Odometry
    import rclpy
    from rclpy.node import Node
    from sensor_msgs.msg import JointState
    from std_msgs.msg import String
except ModuleNotFoundError:  # Pure numerical tests do not require ROS.
    AccelStamped = WrenchStamped = Odometry = JointState = String = None
    rclpy = None
    Node = object


ENU_TO_NED = np.array([[0.0, 1.0, 0.0], [1.0, 0.0, 0.0], [0.0, 0.0, -1.0]])


def filtered_joint_acceleration_step(
    previous_filtered_rad_s2: float,
    raw_rad_s2: float,
    dt_s: float,
    time_constant_s: float,
    maximum_rad_s2: float,
) -> float:
    """Apply a sample-rate-independent first-order acceleration filter."""
    if not np.isfinite(raw_rad_s2) or not np.isfinite(dt_s) or dt_s <= 0.0:
        return float(previous_filtered_rad_s2)
    tau = max(0.0, float(time_constant_s))
    alpha = 1.0 if tau <= 0.0 else 1.0 - np.exp(-float(dt_s) / tau)
    value = float(previous_filtered_rad_s2) + float(alpha) * (
        float(raw_rad_s2) - float(previous_filtered_rad_s2)
    )
    limit = max(0.0, float(maximum_rad_s2))
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
        values = dict(zip(message.name, message.position))
        direct_velocity = dict(zip(message.name, message.velocity))
        dt = None if self.previous_joint_time is None else now - self.previous_joint_time
        for name in JOINT_NAMES:
            if name not in values:
                continue
            previous_position = self.positions.get(name)
            self.positions[name] = float(values[name])
            if name in direct_velocity and np.isfinite(direct_velocity[name]):
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

    def on_odometry(self, message: Odometry) -> None:
        orientation = message.pose.pose.orientation
        quaternion = np.array(
            [orientation.x, orientation.y, orientation.z, orientation.w], dtype=float
        )
        if np.all(np.isfinite(quaternion)) and np.linalg.norm(quaternion) > 1.0e-9:
            self.rotation = quaternion_matrix_xyzw(quaternion)

    def on_timer(self) -> None:
        if any(name not in self.positions for name in JOINT_NAMES):
            return
        state = self.dynamics.state(
            self.positions, self.velocities, self.accelerations, self.payload
        )
        now = self.get_clock().now()
        wrench = WrenchStamped()
        wrench.header.stamp = now.to_msg()
        wrench.header.frame_id = "base_link"
        wrench.wrench.force.x, wrench.wrench.force.y, wrench.wrench.force.z = (
            float(value) for value in state.reaction_force_body_n
        )
        wrench.wrench.torque.x, wrench.wrench.torque.y, wrench.wrench.torque.z = (
            float(value) for value in state.reaction_torque_body_nm
        )
        self.wrench_publisher.publish(wrench)

        gravity_body_n = self.rotation.T @ np.array(
            [0.0, 0.0, -state.mass_kg * 9.80665], dtype=float
        )
        gravity_shift = WrenchStamped()
        gravity_shift.header.stamp = wrench.header.stamp
        gravity_shift.header.frame_id = "base_link"
        gravity_torque = np.cross(state.com_shift_m, gravity_body_n)
        gravity_shift.wrench.torque.x, gravity_shift.wrench.torque.y, gravity_shift.wrench.torque.z = (
            float(value) for value in gravity_torque
        )
        self.gravity_shift_publisher.publish(gravity_shift)

        acceleration = bounded_compensation_ned(
            state.reaction_force_body_n,
            state.mass_kg,
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
            "mass_kg": state.mass_kg,
            "com_body_flu_m": state.center_of_mass_m.tolist(),
            "com_shift_m": state.com_shift_m.tolist(),
            "inertia_diag_kg_m2": np.diag(state.inertia_at_com_kg_m2).tolist(),
            "inertia_tensor_kg_m2": state.inertia_at_com_kg_m2.tolist(),
            "inertia_tensor_frame": "base_link_flu",
            "reaction_force_body_n": state.reaction_force_body_n.tolist(),
            "reaction_torque_body_nm": state.reaction_torque_body_nm.tolist(),
            "raw_joint_acceleration_peak_rad_s2": self.raw_joint_acceleration_peak_rad_s2,
            "filtered_joint_acceleration_peak_rad_s2": max(
                (abs(value) for value in self.accelerations.values()), default=0.0
            ),
            "feedforward_acceleration_ned_m_s2": acceleration.tolist(),
        }
        encoded_report = json.dumps(report, sort_keys=True)
        self.state_publisher.publish(String(data=encoded_report))

        monotonic = time.monotonic()
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
    parser.add_argument("--rate-hz", type=float, default=3.0)
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
    )
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
