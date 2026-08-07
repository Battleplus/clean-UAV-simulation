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
from collections import deque
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
from drone_arm_sim.allocation_analysis import rotor_wrench_frd


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


def motor_wrench_frd(
    config: dict,
    motor_number: int,
    normalized_command: float,
    thrust_scale: float = 1.0,
) -> tuple[float, np.ndarray, np.ndarray]:
    """Map one normalized motor command to thrust, force and torque in PX4 FRD.

    The function intentionally exposes the current model boundary: when RPM is
    unavailable, thrust is obtained from the measured normalized static table and
    reaction torque uses the configured temporary Q/T ratio.
    """
    motor = int(motor_number)
    rotor = next((item for item in config["rotors"] if int(item["motor"]) == motor), None)
    if rotor is None:
        raise ValueError(f"unknown motor number: {motor_number}")
    thrust = command_to_thrust_n(config, normalized_command)
    thrust *= max(0.0, float(thrust_scale))
    force, torque = rotor_wrench_frd(
        config, rotor, thrust, position_key="wrench_position_m"
    )
    return float(thrust), force, torque


def per_motor_wrench_frd(
    config: dict,
    normalized_by_motor: np.ndarray,
    thrust_scale: float = 1.0,
) -> list[dict]:
    """Return force/torque records for all eight actuator inputs."""
    commands = np.asarray(normalized_by_motor, dtype=float)
    if commands.shape != (8,):
        raise ValueError("Expected exactly eight motor commands")
    records = []
    for motor in range(1, 9):
        thrust, force, torque = motor_wrench_frd(
            config, motor, commands[motor - 1], thrust_scale
        )
        records.append({
            "motor": motor,
            "command": float(np.clip(commands[motor - 1], 0.0, 1.0)),
            "thrust_n": thrust,
            "force_frd_n": force,
            "torque_frd_nm": torque,
        })
    return records


def direct_wrench_flu(
    config: dict, normalized_by_motor: np.ndarray, thrust_scale: float = 1.0
) -> tuple[np.ndarray, np.ndarray]:
    """Return base-origin force and torque in Gazebo body FLU."""
    commands = np.clip(
        np.asarray(normalized_by_motor, dtype=float), 0.0, 1.0
    )
    if commands.shape != (8,):
        raise ValueError("Expected exactly eight motor commands")

    total_force = np.zeros(3)
    total_torque = np.zeros(3)
    for record in per_motor_wrench_frd(config, commands, thrust_scale):
        total_force += FRD_TO_FLU @ record["force_frd_n"]
        total_torque += FRD_TO_FLU @ record["torque_frd_nm"]
    return total_force, total_torque


def environment_wrench_world(
    config: dict,
    rotation_flu_to_world: np.ndarray,
    linear_velocity_body_flu: np.ndarray,
    angular_velocity_body_flu: np.ndarray,
) -> tuple[np.ndarray, np.ndarray]:
    """Return estimated air-drag force/torque in world ENU.

    The odometry twist used by this model is expressed in the child link's FLU
    frame.  Wind is configured in world ENU and transformed into the body before
    applying an axis-wise quadratic drag model.  Coefficients are Cd*A (m^2),
    deliberately exposed in JSON because the current CAD has no aerodynamic
    identification data.
    """
    model = config.get("environment_dynamics")
    if not isinstance(model, dict):
        return np.zeros(3), np.zeros(3)
    rotation = np.asarray(rotation_flu_to_world, dtype=float)
    velocity_body = np.asarray(linear_velocity_body_flu, dtype=float)
    angular_body = np.asarray(angular_velocity_body_flu, dtype=float)
    wind_world = np.asarray(
        model.get("wind_velocity_world_enu_m_s", [0.0, 0.0, 0.0]), dtype=float
    )
    relative_body = velocity_body - rotation.T @ wind_world
    rho = float(model.get("air_density_kg_m3", 1.225))
    cd_area = np.asarray(
        model.get("quadratic_drag_area_cd_m2_body_flu", [0.0, 0.0, 0.0]),
        dtype=float,
    )
    damping = np.asarray(
        model.get("angular_damping_n_m_per_rad_s_body_flu", [0.0, 0.0, 0.0]),
        dtype=float,
    )
    drag_body = -0.5 * rho * cd_area * np.abs(relative_body) * relative_body
    damping_torque_body = -damping * angular_body
    return rotation @ drag_body, rotation @ damping_torque_body


def ground_effect_thrust_scale(config: dict, height_m: float) -> float:
    """Bounded empirical thrust multiplier near a flat ground plane."""
    environment = config.get("environment_dynamics")
    effect = environment.get("ground_effect") if isinstance(environment, dict) else None
    if not isinstance(effect, dict) or not bool(effect.get("enabled", False)):
        return 1.0
    gain = max(0.0, float(effect.get("maximum_thrust_gain", 0.0)))
    decay = max(1e-6, float(effect.get("decay_height_m", 0.25)))
    return 1.0 + gain * np.exp(-max(0.0, float(height_m)) / decay)


def command_to_current_a(config: dict, normalized_command: float) -> float:
    """Interpolate one motor's measured 14.8 V current curve."""
    model = config.get("static_current_model")
    points = model.get("points", []) if isinstance(model, dict) else []
    if not points:
        return 0.0
    throttle = np.asarray(
        [float(point["throttle_percent"]) for point in points], dtype=float
    )
    current = np.asarray([float(point["current_a"]) for point in points], dtype=float)
    order = np.argsort(throttle)
    return max(
        0.0,
        float(np.interp(np.clip(normalized_command, 0.0, 1.0) * 100.0,
                        throttle[order], current[order])),
    )


def battery_step(
    config: dict, normalized_commands: np.ndarray, state_of_charge: float, dt_s: float
) -> tuple[float, float, float, float]:
    """Advance a simple Thevenin battery and return SOC, voltage, scale, amps."""
    model = config.get("battery_dynamics")
    if not isinstance(model, dict) or not bool(model.get("enabled", False)):
        reference = float(model.get("reference_voltage_v", 14.8)) if isinstance(model, dict) else 14.8
        return float(np.clip(state_of_charge, 0.0, 1.0)), reference, 1.0, 0.0
    commands = np.clip(np.asarray(normalized_commands, dtype=float), 0.0, 1.0)
    total_current = sum(command_to_current_a(config, value) for value in commands)
    capacity_ah = max(1e-9, float(model.get("capacity_ah", 1.0)))
    soc = float(np.clip(
        state_of_charge - total_current * max(0.0, float(dt_s)) / (3600.0 * capacity_ah),
        0.0, 1.0,
    ))
    full = float(model.get("full_voltage_v", 16.8))
    empty = float(model.get("empty_voltage_v", 13.2))
    open_circuit = empty + soc * (full - empty)
    resistance = max(0.0, float(model.get("pack_internal_resistance_ohm", 0.0)))
    minimum = float(model.get("minimum_loaded_voltage_v", 0.0))
    loaded = max(minimum, open_circuit - total_current * resistance)
    reference = max(1e-9, float(model.get("reference_voltage_v", 14.8)))
    exponent = float(model.get("thrust_voltage_exponent", 2.0))
    # The static table is already capped at the user's 1.2 kgf rating.  Voltage
    # sag may reduce that curve, but a battery model must not lift the cap.
    thrust_scale = min(1.0, max(0.0, (loaded / reference) ** exponent))
    return soc, loaded, thrust_scale, total_current


def delayed_command_step(
    pending_commands,
    current_command: np.ndarray,
    simulation_time_s: float,
    delay_s: float,
) -> np.ndarray:
    """Release timestamped actuator commands after a simulation-time delay."""
    released = np.asarray(current_command, dtype=float)
    release_before = float(simulation_time_s) - max(0.0, float(delay_s))
    while pending_commands and float(pending_commands[0][0]) <= release_before:
        _, released = pending_commands.popleft()
    return np.asarray(released, dtype=float).copy()


class DirectMotorModel(Node):
    def __init__(
        self,
        config_path: Path,
        entity_name: str,
        command_topic: str,
        velocity_command_scale: float,
        reaction_moment_ratio_m: float | None = None,
        wind_velocity_world_enu_m_s: list[float] | None = None,
        battery_dynamics_enabled: bool | None = None,
        battery_overrides: dict[str, float] | None = None,
    ):
        super().__init__("my_drone_gazebo_direct_motor_model")
        self.config = json.loads(config_path.read_text(encoding="utf-8"))
        if reaction_moment_ratio_m is not None and reaction_moment_ratio_m >= 0.0:
            self.config["reaction_moment_ratio_m"] = reaction_moment_ratio_m
            self.get_logger().info(
                f"reaction moment trial enabled: Q/T={reaction_moment_ratio_m:.6g} m"
            )
        if (
            wind_velocity_world_enu_m_s is not None
            and np.all(np.isfinite(wind_velocity_world_enu_m_s))
        ):
            environment = self.config.setdefault("environment_dynamics", {})
            environment["wind_velocity_world_enu_m_s"] = [
                float(value) for value in wind_velocity_world_enu_m_s
            ]
            self.get_logger().info(
                "wind override ENU m/s: "
                f"{environment['wind_velocity_world_enu_m_s']}"
            )
        if battery_dynamics_enabled is not None or battery_overrides:
            battery = self.config.setdefault("battery_dynamics", {})
            if battery_dynamics_enabled is not None:
                battery["enabled"] = bool(battery_dynamics_enabled)
            for key, value in (battery_overrides or {}).items():
                if np.isfinite(value):
                    battery[key] = float(value)
            self.get_logger().info(
                "battery dynamics override: "
                f"enabled={bool(battery.get('enabled', False))} "
                f"R={float(battery.get('pack_internal_resistance_ohm', 0.0)):.6g} ohm "
                f"capacity={float(battery.get('capacity_ah', 0.0)):.6g} Ah"
            )
        self.entity_name = entity_name
        self.velocity_command_scale = velocity_command_scale
        self.rotation_flu_to_world: np.ndarray | None = None
        self.position_world_z = 0.0
        self.linear_velocity_body_flu = np.zeros(3)
        self.angular_velocity_body_flu = np.zeros(3)
        self.target_commands = np.zeros(8)
        self.received_commands = np.zeros(8)
        self.pending_commands = deque(maxlen=2500)
        self.filtered_commands = np.zeros(8)
        self.last_simulation_time_s: float | None = None
        self.logged_first_nonzero_command = False
        self.battery_state_of_charge = 1.0
        self.battery_loaded_voltage_v = 14.8
        self.battery_thrust_scale = 1.0
        self.battery_current_a = 0.0
        self.last_dynamics_log_time_s: float | None = None
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
        delay_s = float(
            self.config.get("actuator_transport_delay_s", 0.0)
        )
        self.target_commands = delayed_command_step(
            self.pending_commands,
            self.target_commands,
            simulation_time_s,
            delay_s,
        )
        self.filtered_commands = first_order_motor_step(
            self.filtered_commands, self.target_commands, dt_s, self.config
        )
        (
            self.battery_state_of_charge,
            self.battery_loaded_voltage_v,
            self.battery_thrust_scale,
            self.battery_current_a,
        ) = battery_step(
            self.config,
            self.filtered_commands,
            self.battery_state_of_charge,
            dt_s,
        )
        if self.last_dynamics_log_time_s is None or (
            simulation_time_s - self.last_dynamics_log_time_s >= 1.0
        ):
            self.last_dynamics_log_time_s = simulation_time_s
            self.get_logger().info(
                "MOTOR_DYNAMIC_STATE "
                f"sim_s={simulation_time_s:.3f} "
                f"cmd_max={float(np.max(self.target_commands)):.4f} "
                f"filtered_max={float(np.max(self.filtered_commands)):.4f} "
                f"voltage_v={self.battery_loaded_voltage_v:.4f} "
                f"thrust_scale={self.battery_thrust_scale:.6f} "
                f"current_a={self.battery_current_a:.4f}"
            )
        quaternion = np.array(
            [
                message.pose.pose.orientation.x,
                message.pose.pose.orientation.y,
                message.pose.pose.orientation.z,
                message.pose.pose.orientation.w,
            ]
        )
        self.position_world_z = float(message.pose.pose.position.z)
        self.linear_velocity_body_flu = np.array(
            [message.twist.twist.linear.x, message.twist.twist.linear.y,
             message.twist.twist.linear.z], dtype=float
        )
        self.angular_velocity_body_flu = np.array(
            [message.twist.twist.angular.x, message.twist.twist.angular.y,
             message.twist.twist.angular.z], dtype=float
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
            self.received_commands = np.clip(values[:8], 0.0, 1.0)
            command_time = self.last_simulation_time_s or 0.0
            self.pending_commands.append(
                (command_time, self.received_commands.copy())
            )
            if (
                not self.logged_first_nonzero_command
                and np.max(self.received_commands) > 1e-6
            ):
                self.get_logger().info(
                    "first nonzero normalized motor command: "
                    f"{self.received_commands.tolist()}"
                )
                self.logged_first_nonzero_command = True
            return
        # PX4's Gazebo actuator bridge publishes its scaled outputs in the
        # velocity field.  EC_MAX=1000 therefore maps linearly to u=1 here;
        # no RPM interpretation or quadratic motorConstant is introduced.
        values = np.asarray(message.velocity, dtype=float)
        if values.size >= 8 and np.all(np.isfinite(values[:8])):
            self.received_commands = np.clip(
                values[:8] / self.velocity_command_scale, 0.0, 1.0
            )
            command_time = self.last_simulation_time_s or 0.0
            self.pending_commands.append(
                (command_time, self.received_commands.copy())
            )
            if (
                not self.logged_first_nonzero_command
                and np.max(self.received_commands) > 1e-6
            ):
                self.get_logger().info(
                    "first nonzero velocity motor command: "
                    f"raw={values[:8].tolist()}, "
                    f"normalized={self.received_commands.tolist()}"
                )
                self.logged_first_nonzero_command = True

    def publish_wrench(self) -> None:
        if self.rotation_flu_to_world is None:
            return
        thrust_scale = (
            ground_effect_thrust_scale(self.config, self.position_world_z)
            * self.battery_thrust_scale
        )
        force_body, torque_body = direct_wrench_flu(
            self.config, self.filtered_commands, thrust_scale=thrust_scale
        )
        rotation = self.rotation_flu_to_world
        message = EntityWrench()
        message.header.stamp = self.get_clock().now().to_msg()
        message.entity.name = self.entity_name
        message.entity.type = Entity.LINK
        environment_force_world, environment_torque_world = environment_wrench_world(
            self.config,
            rotation,
            self.linear_velocity_body_flu,
            self.angular_velocity_body_flu,
        )
        force_world = rotation @ force_body + environment_force_world
        torque_world = rotation @ torque_body + environment_torque_world
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
    parser.add_argument("--wind-enu", type=float, nargs=3, default=None)
    parser.add_argument(
        "--battery-dynamics-enabled",
        default=None,
        choices=("true", "false"),
        help="override battery_dynamics.enabled without editing the baseline JSON",
    )
    parser.add_argument("--battery-internal-resistance-ohm", type=float, default=float("nan"))
    parser.add_argument("--battery-capacity-ah", type=float, default=float("nan"))
    parser.add_argument("--battery-full-voltage-v", type=float, default=float("nan"))
    parser.add_argument("--battery-empty-voltage-v", type=float, default=float("nan"))
    parser.add_argument(
        "--battery-minimum-loaded-voltage-v", type=float, default=float("nan")
    )
    parser.add_argument("--battery-thrust-voltage-exponent", type=float, default=float("nan"))
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
        parsed.wind_enu,
        None if parsed.battery_dynamics_enabled is None else parsed.battery_dynamics_enabled == "true",
        {
            "pack_internal_resistance_ohm": parsed.battery_internal_resistance_ohm,
            "capacity_ah": parsed.battery_capacity_ah,
            "full_voltage_v": parsed.battery_full_voltage_v,
            "empty_voltage_v": parsed.battery_empty_voltage_v,
            "minimum_loaded_voltage_v": parsed.battery_minimum_loaded_voltage_v,
            "thrust_voltage_exponent": parsed.battery_thrust_voltage_exponent,
        },
    )
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
