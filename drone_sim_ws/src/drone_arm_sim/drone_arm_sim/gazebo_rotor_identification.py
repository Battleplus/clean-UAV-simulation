"""Identify the Gazebo wrench direction of each virtual rotor.

The node alternates a zero-command baseline window and a short single-rotor
pulse.  Velocity changes are converted to a body-FLU wrench about the model
origin.  The result is intended to catch axis, ordering, and reaction-torque
sign errors before tuning a flight controller or PX4 control allocation.
"""

from __future__ import annotations

import argparse
from pathlib import Path

from actuator_msgs.msg import Actuators
from nav_msgs.msg import Odometry
import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node

from drone_arm_sim.gazebo_wrench_controller import quaternion_matrix_xyzw
from drone_arm_sim.model_analysis import UrdfModel, _default_urdf


class RotorIdentifier(Node):
    def __init__(
        self,
        urdf: Path,
        pulse_speed: float,
        warmup_duration: float,
        baseline_duration: float,
        pulse_duration: float,
        rotor_index: int | None,
    ):
        super().__init__("my_drone_rotor_identifier")
        model = UrdfModel(urdf)
        self.mass, self.center_flu, self.inertia_flu = model.mass_properties({})
        self.pulse_speed = pulse_speed
        self.warmup_duration = warmup_duration
        self.baseline_duration = baseline_duration
        self.pulse_duration = pulse_duration
        self.rotor = 0 if rotor_index is None else rotor_index
        self.last_rotor = 7 if rotor_index is None else rotor_index
        self.phase = "warmup"
        self.phase_start = None
        self.phase_start_state = None
        self.baseline_linear_acceleration = None
        self.baseline_angular_acceleration = None
        self.columns: list[np.ndarray] = []
        self.publisher = self.create_publisher(
            Actuators, "/my_drone/command/motor_speed", 10
        )
        self.subscription = self.create_subscription(
            Odometry, "/model/my_drone/odometry", self.on_odometry, 50
        )

    @staticmethod
    def stamp_seconds(message: Odometry) -> float:
        stamp = message.header.stamp
        return float(stamp.sec) + 1e-9 * float(stamp.nanosec)

    @staticmethod
    def state(message: Odometry) -> tuple[np.ndarray, ...]:
        pose = message.pose.pose
        twist = message.twist.twist
        position = np.array([pose.position.x, pose.position.y, pose.position.z])
        quaternion = np.array(
            [
                pose.orientation.x,
                pose.orientation.y,
                pose.orientation.z,
                pose.orientation.w,
            ]
        )
        velocity = np.array(
            [twist.linear.x, twist.linear.y, twist.linear.z]
        )
        angular_velocity = np.array(
            [twist.angular.x, twist.angular.y, twist.angular.z]
        )
        return position, quaternion, velocity, angular_velocity

    def publish_command(self) -> None:
        message = Actuators()
        speed = np.zeros(8)
        if self.phase == "pulse":
            speed[self.rotor] = self.pulse_speed
        message.velocity = speed.tolist()
        self.publisher.publish(message)

    def acceleration(
        self,
        start: tuple[np.ndarray, ...],
        end: tuple[np.ndarray, ...],
        duration: float,
    ) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
        _, quaternion_start, velocity_start, omega_start = start
        _, quaternion_end, velocity_end, omega_end = end
        rotation_flu_to_enu = quaternion_matrix_xyzw(quaternion_start)
        rotation_end_flu_to_enu = quaternion_matrix_xyzw(quaternion_end)
        # nav_msgs/Odometry twist is expressed in child_frame_id (body FLU).
        velocity_start_enu = rotation_flu_to_enu @ velocity_start
        velocity_end_enu = rotation_end_flu_to_enu @ velocity_end
        linear_acceleration_enu = (
            velocity_end_enu - velocity_start_enu
        ) / duration
        angular_acceleration_flu = (omega_end - omega_start) / duration
        return (
            rotation_flu_to_enu,
            linear_acceleration_enu,
            angular_acceleration_flu,
        )

    def finish(self) -> None:
        matrix = np.column_stack(self.columns)
        normalized = matrix.copy()
        for index in range(normalized.shape[1]):
            force_norm = np.linalg.norm(normalized[:3, index])
            if force_norm > 1e-9:
                normalized[:, index] /= force_norm
        np.set_printoptions(precision=6, suppress=True, linewidth=180)
        self.get_logger().info(
            "Measured Gazebo wrench columns [Fx,Fy,Fz,Mx,My,Mz] in body FLU:\n"
            + str(matrix)
        )
        self.get_logger().info(
            "Force-normalized Gazebo effectiveness:\n" + str(normalized)
        )
        self.get_logger().info("ROTOR_IDENTIFICATION_COMPLETE")
        rclpy.shutdown()

    def on_odometry(self, message: Odometry) -> None:
        current_time = self.stamp_seconds(message)
        current_state = self.state(message)
        flat_state = np.concatenate(current_state)
        if not np.all(np.isfinite(flat_state)):
            return
        self.publish_command()
        if self.phase_start is None:
            self.phase_start = current_time
            self.phase_start_state = current_state
            return

        duration = current_time - self.phase_start
        if self.phase == "warmup":
            target_duration = self.warmup_duration
        elif self.phase == "baseline":
            target_duration = self.baseline_duration
        else:
            target_duration = self.pulse_duration
        if duration < target_duration:
            return

        if self.phase == "warmup":
            self.phase = "baseline"
            self.phase_start = current_time
            self.phase_start_state = current_state
            return

        rotation, linear_acceleration, angular_acceleration = self.acceleration(
            self.phase_start_state, current_state, duration
        )
        if self.phase == "baseline":
            self.baseline_linear_acceleration = linear_acceleration
            self.baseline_angular_acceleration = angular_acceleration
            self.phase = "pulse"
            self.phase_start = current_time
            self.phase_start_state = current_state
            self.publish_command()
            return

        delta_acceleration_enu = (
            linear_acceleration - self.baseline_linear_acceleration
        )
        force_flu = (
            rotation.T @ (self.mass * delta_acceleration_enu)
        )
        delta_alpha_flu = (
            angular_acceleration - self.baseline_angular_acceleration
        )
        omega = current_state[3]
        torque_com_flu = (
            self.inertia_flu @ delta_alpha_flu
            + np.cross(omega, self.inertia_flu @ omega)
        )
        torque_origin_flu = torque_com_flu + np.cross(
            self.center_flu, force_flu
        )
        self.columns.append(np.concatenate((force_flu, torque_origin_flu)))
        self.get_logger().info(
            f"rotor {self.rotor + 1}: "
            f"{self.columns[-1].round(6).tolist()}"
        )

        self.rotor += 1
        if self.rotor > self.last_rotor:
            self.phase = "baseline"
            self.publish_command()
            self.finish()
            return
        self.phase = "baseline"
        self.phase_start = current_time
        self.phase_start_state = current_state
        self.publish_command()


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--urdf", type=Path, default=_default_urdf())
    parser.add_argument("--pulse-speed", type=float, default=400.0)
    parser.add_argument("--warmup-duration", type=float, default=0.5)
    parser.add_argument("--baseline-duration", type=float, default=0.08)
    parser.add_argument("--pulse-duration", type=float, default=0.12)
    parser.add_argument(
        "--rotor-index",
        type=int,
        choices=range(1, 9),
        help="Identify only this one-based rotor index.",
    )
    parsed, ros_arguments = parser.parse_known_args()
    rclpy.init(args=ros_arguments)
    node = RotorIdentifier(
        parsed.urdf,
        parsed.pulse_speed,
        parsed.warmup_duration,
        parsed.baseline_duration,
        parsed.pulse_duration,
        None if parsed.rotor_index is None else parsed.rotor_index - 1,
    )
    try:
        rclpy.spin(node)
    except ExternalShutdownException:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
