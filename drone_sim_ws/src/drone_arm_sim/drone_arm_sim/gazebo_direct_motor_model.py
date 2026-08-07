"""Apply eight normalized direct-thrust commands to the Gazebo aircraft.

Input ``Actuators.normalized`` is indexed by PX4 motor number minus one.
Each command is clamped to [0, 1] and mapped through the supplied static
thrust table when present (falling back to ``T = u * maximum_thrust_n``).
Optional first-order rise/fall dynamics are advanced on simulation time.  The
eight forces and moments are combined into
an equivalent wrench at ``drone_base_link``.  This retains the real base/arm
multibody response without inventing an RPM or motorConstant.
"""

from __future__ import annotations

import argparse
import json
from pathlib import Path

import numpy as np

try:
    from actuator_msgs.msg import Actuators
    from nav_msgs.msg import Odometry
    import rclpy
    from rclpy.node import Node
    from ros_gz_interfaces.msg import Entity, EntityWrench
except ModuleNotFoundError:
    Actuators = None
    Odometry = None
    rclpy = None
    Node = object
    Entity = None
    EntityWrench = None

from drone_arm_sim.gazebo_wrench_controller import quaternion_matrix_xyzw


FRD_TO_FLU = np.diag([1.0, -1.0, -1.0])


def command_to_thrust_n(config: dict, normalized_command: float) -> float:
    """Map one normalized command to thrust, preserving legacy configs."""
    command = float(np.clip(normalized_command, 0.0, 1.0))
    model = config.get("static_thrust_model")
    points = model.get("points", []) if isinstance(model, dict) else []
    if isinstance(model, dict) and not points and model.get("throttle_percent"):
        throttle_values = model["throttle_percent"]
        thrust_values = model["rated_capped_thrust_n"]
        points = [
            {"throttle_percent": throttle, "rated_capped_thrust_n": thrust}
            for throttle, thrust in zip(throttle_values, thrust_values, strict=True)
        ]
    if not points:
        maximum_value = config.get("maximum_thrust_n")
        if maximum_value is None:
            maximum_value = config["maximum_rated_thrust_per_motor_n"]
        maximum = float(maximum_value)
        return maximum * command

    throttle = np.asarray(
        [float(point["throttle_percent"]) for point in points], dtype=float
    )
    thrust = np.asarray(
        [
            float(
                point["rated_capped_thrust_n"]
                if "rated_capped_thrust_n" in point
                else point["measured_thrust_n"]
            )
            for point in points
        ],
        dtype=float,
    )
    order = np.argsort(throttle)
    value = float(np.interp(command * 100.0, throttle[order], thrust[order]))
    rated_cap = model.get("rated_cap_n")
    if rated_cap is not None:
        value = min(value, float(rated_cap))
    return max(0.0, value)


def first_order_motor_step(
    current: np.ndarray, target: np.ndarray, dt_s: float, config: dict
) -> np.ndarray:
    """Advance normalized motor commands using optional asymmetric dynamics."""
    current = np.clip(np.asarray(current, dtype=float), 0.0, 1.0)
    target = np.clip(np.asarray(target, dtype=float), 0.0, 1.0)
    if current.shape != target.shape:
        raise ValueError("Current and target motor command shapes must match")
    dynamics = config.get("motor_dynamics")
    if not isinstance(dynamics, dict):
        return target.copy()
    rise = float(dynamics.get("rise_time_constant_s", 0.0))
    fall = float(dynamics.get("fall_time_constant_s", rise))
    tau = np.where(target >= current, rise, fall)
    dt_s = max(0.0, float(dt_s))
    alpha = np.where(tau > 0.0, 1.0 - np.exp(-dt_s / np.maximum(tau, 1e-12)), 1.0)
    return np.clip(current + alpha * (target - current), 0.0, 1.0)


def direct_wrench_flu(
    config: dict, normalized_by_motor: np.ndarray
) -> tuple[np.ndarray, np.ndarray]:
    """Return base-origin force and torque in Gazebo body FLU."""
    commands = np.clip(
        np.asarray(normalized_by_motor, dtype=float), 0.0, 1.0
    )
    if commands.shape != (8,):
        raise ValueError("Expected exactly eight motor commands")

    total_force = np.zeros(3)
    total_torque = np.zeros(3)
    moment_ratio = config.get("reaction_moment_ratio_m")
    for rotor in config["rotors"]:
        motor_index = int(rotor["motor"]) - 1
        thrust = command_to_thrust_n(config, commands[motor_index])
        position = FRD_TO_FLU @ np.asarray(rotor["position_m"], dtype=float)
        axis = FRD_TO_FLU @ np.asarray(rotor["axis_body"], dtype=float)
        axis /= np.linalg.norm(axis)
        force = thrust * axis
        total_force += force
        total_torque += np.cross(position, force)
        if moment_ratio is not None:
            # Match the PX4 effectiveness sign convention used elsewhere in
            # this package: +1 is CCW, moment = -direction * ratio * force.
            total_torque -= (
                float(rotor["direction"]) * float(moment_ratio) * force
            )
    return total_force, total_torque


class DirectMotorModel(Node):
    def __init__(
        self,
        config_path: Path,
        entity_name: str,
        command_topic: str,
        velocity_command_scale: float,
        reaction_moment_ratio_m: float | None = None,
    ):
        super().__init__("my_drone_gazebo_direct_motor_model")
        self.config = json.loads(config_path.read_text(encoding="utf-8"))
        if reaction_moment_ratio_m is not None and reaction_moment_ratio_m >= 0.0:
            self.config["reaction_moment_ratio_m"] = reaction_moment_ratio_m
            self.get_logger().info(
                f"reaction moment trial enabled: Q/T={reaction_moment_ratio_m:.6g} m"
            )
        self.entity_name = entity_name
        self.velocity_command_scale = velocity_command_scale
        self.rotation_flu_to_world: np.ndarray | None = None
        self.target_commands = np.zeros(8)
        self.filtered_commands = np.zeros(8)
        self.last_simulation_time_s: float | None = None
        self.logged_first_nonzero_command = False
        self.publisher = self.create_publisher(
            EntityWrench, "/world/flight_world/wrench", 10
        )
        self.create_subscription(
            Odometry,
            "/model/my_drone/odometry",
            self.on_odometry,
            20,
        )
        self.create_subscription(
            Actuators,
            command_topic,
            self.on_command,
            20,
        )

    def on_odometry(self, message) -> None:
        simulation_time_s = (
            float(message.header.stamp.sec)
            + 1e-9 * float(message.header.stamp.nanosec)
        )
        if self.last_simulation_time_s is None:
            dt_s = 0.0
        else:
            # Large jumps mean a reset/pause rather than a physical motor step.
            dt_s = min(max(simulation_time_s - self.last_simulation_time_s, 0.0), 0.1)
        self.last_simulation_time_s = simulation_time_s
        self.filtered_commands = first_order_motor_step(
            self.filtered_commands, self.target_commands, dt_s, self.config
        )
        quaternion = np.array(
            [
                message.pose.pose.orientation.x,
                message.pose.pose.orientation.y,
                message.pose.pose.orientation.z,
                message.pose.pose.orientation.w,
            ]
        )
        if np.all(np.isfinite(quaternion)) and np.linalg.norm(quaternion) > 1e-9:
            self.rotation_flu_to_world = quaternion_matrix_xyzw(quaternion)
            # ApplyLinkWrench's non-persistent topic acts for one simulation
            # update only.  Odometry is emitted at the model update rate, so
            # publishing here applies thrust on every physics step regardless
            # of real-time factor.  A wall-clock timer would under-apply force
            # whenever physics runs faster than the timer.
            self.publish_wrench()

    def on_command(self, message) -> None:
        values = np.asarray(message.normalized, dtype=float)
        if values.size >= 8 and np.all(np.isfinite(values[:8])):
            self.target_commands = np.clip(values[:8], 0.0, 1.0)
            if (
                not self.logged_first_nonzero_command
                and np.max(self.target_commands) > 1e-6
            ):
                self.get_logger().info(
                    "first nonzero normalized motor command: "
                    f"{self.target_commands.tolist()}"
                )
                self.logged_first_nonzero_command = True
            return
        # PX4's Gazebo actuator bridge publishes its scaled outputs in the
        # velocity field.  EC_MAX=1000 therefore maps linearly to u=1 here;
        # no RPM interpretation or quadratic motorConstant is introduced.
        values = np.asarray(message.velocity, dtype=float)
        if values.size >= 8 and np.all(np.isfinite(values[:8])):
            self.target_commands = np.clip(
                values[:8] / self.velocity_command_scale, 0.0, 1.0
            )
            if (
                not self.logged_first_nonzero_command
                and np.max(self.target_commands) > 1e-6
            ):
                self.get_logger().info(
                    "first nonzero velocity motor command: "
                    f"raw={values[:8].tolist()}, "
                    f"normalized={self.target_commands.tolist()}"
                )
                self.logged_first_nonzero_command = True

    def publish_wrench(self) -> None:
        if self.rotation_flu_to_world is None:
            return
        force_body, torque_body = direct_wrench_flu(
            self.config, self.filtered_commands
        )
        rotation = self.rotation_flu_to_world
        message = EntityWrench()
        message.header.stamp = self.get_clock().now().to_msg()
        message.entity.name = self.entity_name
        message.entity.type = Entity.LINK
        force_world = rotation @ force_body
        torque_world = rotation @ torque_body
        message.wrench.force.x = float(force_world[0])
        message.wrench.force.y = float(force_world[1])
        message.wrench.force.z = float(force_world[2])
        message.wrench.torque.x = float(torque_world[0])
        message.wrench.torque.y = float(torque_world[1])
        message.wrench.torque.z = float(torque_world[2])
        self.publisher.publish(message)


def main() -> None:
    package = Path(__file__).resolve().parents[1]
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--config",
        type=Path,
        default=package / "config" / "my_drone_measured_mounts.json",
    )
    parser.add_argument("--entity-name", default="drone_base_link")
    parser.add_argument(
        "--command-topic",
        default="/my_drone/command/motor_speed",
    )
    parser.add_argument("--velocity-command-scale", type=float, default=1000.0)
    parser.add_argument("--reaction-moment-ratio-m", type=float, default=-1.0)
    parsed, ros_arguments = parser.parse_known_args()
    if rclpy is None:
        raise SystemExit("ROS 2 Python packages are not available")
    rclpy.init(args=ros_arguments)
    node = DirectMotorModel(
        parsed.config,
        parsed.entity_name,
        parsed.command_topic,
        parsed.velocity_command_scale,
        parsed.reaction_moment_ratio_m,
    )
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
