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
import subprocess
import time

import numpy as np

try:
    from actuator_msgs.msg import Actuators
    from geometry_msgs.msg import WrenchStamped
    from nav_msgs.msg import Odometry
    from geometry_msgs.msg import WrenchStamped
    import rclpy
    from rclpy.executors import ExternalShutdownException
    from rclpy.node import Node
    from ros_gz_interfaces.msg import Entity, EntityWrench
    from std_msgs.msg import Bool
except ModuleNotFoundError:
    Actuators = None
    WrenchStamped = None
    Odometry = None
    WrenchStamped = None
    rclpy = None
    ExternalShutdownException = RuntimeError
    Node = object
    Entity = None
    EntityWrench = None
    Bool = None

from drone_arm_sim.gazebo_wrench_controller import quaternion_matrix_xyzw
from drone_arm_sim.allocation_analysis import allocation_matrix, rotor_wrench_frd


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


def actuator_command_to_thrust_n(config: dict, normalized_command: float) -> float:
    """Map the PX4 actuator signal to thrust.

    PX4's control allocator emits a normalized *thrust* request.  The supplied
    static table, in contrast, maps ESC throttle to thrust and reaches the
    rated cap before 100 percent throttle.  Feeding the PX4 value directly
    through that table creates a very high gain near hover followed by a dead
    band from the rated-cap throttle to 1.0.  Formal flight configs therefore
    use a linear normalized-thrust interface; the static table remains the
    evidence used to calculate the equivalent ESC throttle and current.
    """
    command = float(np.clip(normalized_command, 0.0, 1.0))
    input_model = config.get("actuator_input_model")
    if input_model == "hover_scaled_linear_thrust_with_rated_cap":
        mapping = config["actuator_normalization"]
        gain = float(mapping["linear_thrust_per_command_n"])
        maximum = float(config["maximum_thrust_n"])
        return min(maximum, gain * command)
    if input_model == "hover_anchored_normalized_thrust":
        mapping = config["actuator_normalization"]
        hover_command = float(mapping["px4_hover_command"])
        hover_thrust = float(mapping["physical_hover_thrust_n"])
        maximum = float(config["maximum_thrust_n"])
        return float(np.interp(
            command, [0.0, hover_command, 1.0], [0.0, hover_thrust, maximum]
        ))
    if input_model == "normalized_thrust":
        maximum_value = config.get("maximum_thrust_n")
        if maximum_value is None:
            maximum_value = config["maximum_rated_thrust_per_motor_n"]
        maximum = float(maximum_value)
        return maximum * command
    return command_to_thrust_n(config, command)


def thrust_to_actuator_command(config: dict, thrust_n: float) -> float:
    """Invert :func:`actuator_command_to_thrust_n` with physical bounds."""
    requested = max(0.0, float(thrust_n))
    input_model = config.get("actuator_input_model")
    if input_model == "hover_scaled_linear_thrust_with_rated_cap":
        mapping = config["actuator_normalization"]
        gain = float(mapping["linear_thrust_per_command_n"])
        maximum = float(config["maximum_thrust_n"])
        return 0.0 if gain <= 0.0 else float(
            np.clip(min(requested, maximum) / gain, 0.0, 1.0)
        )
    if input_model == "hover_anchored_normalized_thrust":
        mapping = config["actuator_normalization"]
        hover_command = float(mapping["px4_hover_command"])
        hover_thrust = float(mapping["physical_hover_thrust_n"])
        maximum = float(config["maximum_thrust_n"])
        return float(np.clip(np.interp(
            requested, [0.0, hover_thrust, maximum], [0.0, hover_command, 1.0]
        ), 0.0, 1.0))
    if input_model == "normalized_thrust":
        maximum_value = config.get("maximum_thrust_n")
        if maximum_value is None:
            maximum_value = config["maximum_rated_thrust_per_motor_n"]
        maximum = float(maximum_value)
        return 0.0 if maximum <= 0.0 else float(
            np.clip(requested / maximum, 0.0, 1.0)
        )
    return thrust_to_command(config, requested)


def thrust_to_command(config: dict, thrust_n: float) -> float:
    """Invert the static thrust curve for a bounded force command.

    The formal model uses a measured piecewise-linear throttle table.  The
    inverse is only used by the optional arm torque feed-forward path; without
    a table it reduces to the legacy linear mapping.
    """
    requested = max(0.0, float(thrust_n))
    model = config.get("static_thrust_model")
    points = model.get("points", []) if isinstance(model, dict) else []
    if not points:
        maximum = float(config.get("maximum_thrust_n", config.get(
            "maximum_rated_thrust_per_motor_n", 0.0)))
        return 0.0 if maximum <= 0.0 else float(np.clip(requested / maximum, 0.0, 1.0))
    throttle = np.asarray([float(p["throttle_percent"]) for p in points], dtype=float)
    thrust = np.asarray([
        float(p.get("rated_capped_thrust_n", p.get("measured_thrust_n", 0.0)))
        for p in points
    ], dtype=float)
    order = np.argsort(thrust, kind="stable")
    thrust = thrust[order]
    throttle = throttle[order]
    # A rated cap can make the last table entries duplicate.  np.interp is
    # well-defined for the duplicates and the result is clamped below.
    return float(np.clip(np.interp(requested, thrust, throttle) / 100.0, 0.0, 1.0))


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
    thrust = actuator_command_to_thrust_n(config, normalized_command)
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


def takeoff_support_release_ready(
    wrench_frd: np.ndarray,
    minimum_up_force_n: float,
    maximum_horizontal_force_n: float,
    maximum_com_torque_nm: float,
) -> bool:
    """Return whether the COM wrench is sufficiently balanced for liftoff.

    FRD uses positive down, so upward force is ``-Fz``.  Requiring the full
    COM wrench (rather than only total vertical force) prevents the temporary
    Gazebo support from being removed while the allocator is still settling.
    """
    wrench = np.asarray(wrench_frd, dtype=float)
    if wrench.shape != (6,) or not np.all(np.isfinite(wrench)):
        return False
    return bool(
        -wrench[2] >= float(minimum_up_force_n)
        and np.linalg.norm(wrench[:2]) <= float(maximum_horizontal_force_n)
        and np.linalg.norm(wrench[3:]) <= float(maximum_com_torque_nm)
    )


def landing_support_restore_ready(
    current_world_z_m: float,
    release_world_z_m: float | None,
    clearance_m: float,
) -> bool:
    """Return whether a descending aircraft is close enough to restore its fixture.

    The support is almost as tall as the CAD arm.  Creating it immediately on
    a LAND command can intersect the still-airborne vehicle and inject a large
    contact impulse.  The release height records the supported ground pose, so
    restoring only shortly above that pose leaves time for the Gazebo service
    call while avoiding an airborne collision.
    """
    if release_world_z_m is None:
        return False
    values = np.asarray([current_world_z_m, release_world_z_m, clearance_m], dtype=float)
    if not np.all(np.isfinite(values)):
        return False
    return bool(current_world_z_m <= release_world_z_m + max(0.0, clearance_m))


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
    # Current measurements are indexed by ESC throttle.  Convert the PX4
    # normalized-thrust request back through the measured static curve first.
    if config.get("actuator_input_model") in {
        "normalized_thrust", "hover_anchored_normalized_thrust",
        "hover_scaled_linear_thrust_with_rated_cap"
    }:
        equivalent_throttle = [
            thrust_to_command(config, actuator_command_to_thrust_n(config, value))
            for value in commands
        ]
    else:
        equivalent_throttle = commands
    total_current = sum(command_to_current_a(config, value)
                        for value in equivalent_throttle)
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


def torque_feedforward_command_delta(
    config: dict,
    normalized_commands: np.ndarray,
    reaction_torque_frd_nm: np.ndarray,
    matrix_pinv: np.ndarray | None = None,
    max_delta_n: float = 2.0,
) -> tuple[np.ndarray, np.ndarray]:
    """Return motor commands that counter an arm reaction torque.

    ``reaction_torque_frd_nm`` is the torque currently applied by the arm to
    the aircraft body.  The requested rotor wrench is its exact negative with
    zero net force.  A bounded pseudoinverse solution is used so this feature
    remains a feed-forward trim and cannot consume the whole flight margin.
    """
    commands = np.clip(np.asarray(normalized_commands, dtype=float), 0.0, 1.0)
    torque = np.asarray(reaction_torque_frd_nm, dtype=float)
    if commands.shape != (8,) or torque.shape != (3,) or not np.all(np.isfinite(torque)):
        return commands.copy(), np.zeros(8)
    matrix = allocation_matrix(config, position_key="wrench_position_m")
    pinv = np.linalg.pinv(matrix) if matrix_pinv is None else np.asarray(matrix_pinv)
    desired = np.concatenate((np.zeros(3), -torque))
    delta_n = pinv @ desired
    limit = max(0.0, float(max_delta_n))
    if limit > 0.0:
        delta_n = np.clip(delta_n, -limit, limit)
    target_thrust = np.array([
        actuator_command_to_thrust_n(config, command) + delta
        for command, delta in zip(commands, delta_n, strict=True)
    ])
    compensated = np.array([
        thrust_to_actuator_command(config, thrust)
        for thrust in target_thrust
    ])
    # Do not allow the trim to turn a stopped motor on or exceed its rated
    # output.  The base PX4 command remains the source of the main thrust.
    compensated = np.clip(compensated, 0.0, 1.0)
    return compensated, compensated - commands


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
        arm_torque_feedforward_enabled: bool = False,
        arm_torque_feedforward_max_delta_n: float = 2.0,
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
        self.position_world_xyz = np.zeros(3)
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
        self.arm_torque_feedforward_enabled = bool(arm_torque_feedforward_enabled)
        self.arm_torque_feedforward_max_delta_n = max(
            0.0, float(arm_torque_feedforward_max_delta_n)
        )
        self.arm_reaction_torque_flu = np.zeros(3)
        self.arm_reaction_torque_stamp_s: float | None = None
        self.arm_motion_active_stamp_s: float | None = None
        self.arm_torque_matrix_pinv = np.linalg.pinv(
            allocation_matrix(self.config, position_key="wrench_position_m")
        )
        self.last_dynamics_log_time_s: float | None = None
        support_release = self.config.get("takeoff_support_release", {})
        self.support_release_enabled = bool(support_release.get("enabled", False))
        self.support_release_model_name = str(
            support_release.get("model_name", "my_drone_bringup_landing_support")
        )
        self.support_release_up_force_n = max(
            0.0, float(support_release.get("release_up_force_n", float("inf")))
        )
        self.support_release_max_horizontal_force_n = max(
            0.0,
            float(support_release.get("maximum_horizontal_force_n", float("inf"))),
        )
        self.support_release_max_com_torque_nm = max(
            0.0,
            float(support_release.get("maximum_com_torque_nm", float("inf"))),
        )
        self.support_release_hold_time_s = max(
            0.0, float(support_release.get("hold_time_s", 0.12))
        )
        self.support_release_candidate_s: float | None = None
        self.support_release_process: subprocess.Popen | None = None
        self.support_release_requested = False
        self.support_release_world_z_m: float | None = None
        self.support_restore_on_land = bool(
            support_release.get("restore_on_land", False)
        )
        self.support_restore_clearance_m = max(
            0.0, float(support_release.get("restore_clearance_m", 0.45))
        )
        restore_filename = Path(
            support_release.get("restore_sdf_filename", "../worlds/landing_support.sdf")
        )
        if not restore_filename.is_absolute():
            restore_filename = config_path.parent / restore_filename
        self.support_restore_sdf_filename = restore_filename.resolve()
        self.landing_requested = False
        self.support_restore_process: subprocess.Popen | None = None
        self.support_restore_requested = False
        normalization = self.config.get("actuator_normalization", {})
        self.rated_thrust_command = float(
            normalization.get("rated_thrust_command", 0.999)
        )
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
        self.create_subscription(
            WrenchStamped,
            "/my_drone/arm_reaction_wrench_body",
            self.on_arm_reaction_wrench,
            10,
        )
        self.create_subscription(
            Bool,
            "/my_drone/arm_motion_active",
            self.on_arm_motion_active,
            10,
        )
        self.create_subscription(
            Bool,
            "/my_drone/landing_requested",
            self.on_landing_requested,
            10,
        )
        if self.arm_torque_feedforward_enabled:
            self.get_logger().info(
                "arm torque feed-forward enabled: "
                f"max_delta={self.arm_torque_feedforward_max_delta_n:.3f} N"
            )

    def on_odometry(self, message) -> None:
        # Gazebo can deliver one last odometry callback while ROS is tearing
        # down.  Do not touch clocks, loggers, or publishers after shutdown.
        if rclpy is None or not rclpy.ok():
            return
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
        self.position_world_xyz = np.asarray(
            [
                message.pose.pose.position.x,
                message.pose.pose.position.y,
                message.pose.pose.position.z,
            ],
            dtype=float,
        )
        self.position_world_z = float(self.position_world_xyz[2])
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
        release_scale = (
            ground_effect_thrust_scale(self.config, self.position_world_z)
            * self.battery_thrust_scale
        )
        release_records = per_motor_wrench_frd(
            self.config, self.filtered_commands, release_scale
        )
        release_thrust_n = np.asarray(
            [record["thrust_n"] for record in release_records], dtype=float
        )
        release_wrench_frd = allocation_matrix(self.config) @ release_thrust_n
        self.update_takeoff_support_release(simulation_time_s, release_wrench_frd)
        self.update_landing_support_restore()
        if self.last_dynamics_log_time_s is None or (
            simulation_time_s - self.last_dynamics_log_time_s >= 1.0
        ):
            self.last_dynamics_log_time_s = simulation_time_s
            diagnostic_scale = (
                ground_effect_thrust_scale(self.config, self.position_world_z)
                * self.battery_thrust_scale
            )
            diagnostic_records = per_motor_wrench_frd(
                self.config, self.filtered_commands, diagnostic_scale
            )
            diagnostic_force, _ = direct_wrench_flu(
                self.config, self.filtered_commands, diagnostic_scale
            )
            equivalent_throttles = [
                thrust_to_command(self.config, record["thrust_n"])
                for record in diagnostic_records
            ]
            self.get_logger().info(
                "MOTOR_DYNAMIC_STATE "
                f"sim_s={simulation_time_s:.3f} "
                f"cmd_min={float(np.min(self.target_commands)):.4f} "
                f"cmd_mean={float(np.mean(self.target_commands)):.4f} "
                f"cmd_max={float(np.max(self.target_commands)):.4f} "
                f"filtered_mean={float(np.mean(self.filtered_commands)):.4f} "
                f"filtered_max={float(np.max(self.filtered_commands)):.4f} "
                f"esc_throttle_mean={float(np.mean(equivalent_throttles)):.4f} "
                f"total_thrust_n={sum(record['thrust_n'] for record in diagnostic_records):.4f} "
                f"body_up_force_n={float(diagnostic_force[2]):.4f} "
                f"saturated_motors={int(np.count_nonzero(self.filtered_commands >= self.rated_thrust_command))} "
                f"rated_cap_command={self.rated_thrust_command:.4f} "
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

    def update_takeoff_support_release(
        self, simulation_time_s: float, wrench_frd: np.ndarray
    ) -> None:
        """Remove the four Gazebo bring-up supports simultaneously at liftoff."""
        if self.support_release_process is not None:
            status = self.support_release_process.poll()
            if status is not None:
                output = self.support_release_process.stdout.read().strip()
                level = self.get_logger().info if status == 0 else self.get_logger().error
                level(
                    "TAKEOFF_SUPPORT_RELEASE_RESULT "
                    f"status={status} output={output or '<empty>'}"
                )
                self.support_release_process = None
            return
        if not self.support_release_enabled or self.support_release_requested:
            return
        balanced = takeoff_support_release_ready(
            wrench_frd,
            self.support_release_up_force_n,
            self.support_release_max_horizontal_force_n,
            self.support_release_max_com_torque_nm,
        )
        if not balanced:
            self.support_release_candidate_s = None
            return
        if self.support_release_candidate_s is None:
            self.support_release_candidate_s = simulation_time_s
            return
        if simulation_time_s - self.support_release_candidate_s < self.support_release_hold_time_s:
            return
        request = f'name: "{self.support_release_model_name}" type: MODEL'
        self.support_release_process = subprocess.Popen(
            [
                "gz", "service", "-s", "/world/flight_world/remove",
                "--reqtype", "gz.msgs.Entity", "--reptype", "gz.msgs.Boolean",
                "--timeout", "3000", "--req", request,
            ],
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            text=True,
        )
        self.support_release_requested = True
        self.support_release_world_z_m = float(self.position_world_xyz[2])
        body_up_force_n = -float(wrench_frd[2])
        horizontal_force_n = float(np.linalg.norm(wrench_frd[:2]))
        com_torque_nm = float(np.linalg.norm(wrench_frd[3:]))
        self.get_logger().info(
            "TAKEOFF_SUPPORT_RELEASE_REQUESTED "
            f"model={self.support_release_model_name} "
            f"body_up_force_n={body_up_force_n:.4f} "
            f"horizontal_force_n={horizontal_force_n:.4f} "
            f"com_torque_nm={com_torque_nm:.4f} "
            f"threshold_n={self.support_release_up_force_n:.4f}"
        )

    def on_landing_requested(self, message) -> None:
        """Latch the explicit PX4/WASD LAND request for fixture restoration."""
        if bool(message.data):
            self.landing_requested = True

    def update_landing_support_restore(self) -> None:
        """Respawn the temporary support beneath the aircraft for landing."""
        if self.support_restore_process is not None:
            status = self.support_restore_process.poll()
            if status is not None:
                output = self.support_restore_process.stdout.read().strip()
                level = self.get_logger().info if status == 0 else self.get_logger().error
                level(
                    "LANDING_SUPPORT_RESTORE_RESULT "
                    f"status={status} output={output or '<empty>'}"
                )
                self.support_restore_process = None
            return
        if (
            not self.support_restore_on_land
            or not self.landing_requested
            or not self.support_release_requested
            or self.support_restore_requested
        ):
            return
        if not landing_support_restore_ready(
            float(self.position_world_xyz[2]),
            self.support_release_world_z_m,
            self.support_restore_clearance_m,
        ):
            return
        if not self.support_restore_sdf_filename.is_file():
            self.get_logger().error(
                "LANDING_SUPPORT_RESTORE_MISSING "
                f"file={self.support_restore_sdf_filename}"
            )
            self.support_restore_requested = True
            return
        x_m, y_m = (float(value) for value in self.position_world_xyz[:2])
        request = (
            f'sdf_filename: "{self.support_restore_sdf_filename}" '
            f'name: "{self.support_release_model_name}" allow_renaming: false '
            f'pose {{ position {{ x: {x_m:.9g} y: {y_m:.9g} z: 0 }} }}'
        )
        self.support_restore_process = subprocess.Popen(
            [
                "gz", "service", "-s", "/world/flight_world/create",
                "--reqtype", "gz.msgs.EntityFactory",
                "--reptype", "gz.msgs.Boolean",
                "--timeout", "3000", "--req", request,
            ],
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            text=True,
        )
        self.support_restore_requested = True
        self.get_logger().info(
            "LANDING_SUPPORT_RESTORE_REQUESTED "
            f"model={self.support_release_model_name} x_m={x_m:.4f} y_m={y_m:.4f}"
        )

    def on_command(self, message) -> None:
        if rclpy is None or not rclpy.ok():
            return
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

    def on_arm_reaction_wrench(self, message) -> None:
        """Cache a fresh arm reaction torque expressed in Gazebo FLU."""
        if rclpy is None or not rclpy.ok():
            return
        values = np.asarray(
            [message.wrench.torque.x, message.wrench.torque.y, message.wrench.torque.z],
            dtype=float,
        )
        if values.shape == (3,) and np.all(np.isfinite(values)):
            self.arm_reaction_torque_flu = values
            self.arm_reaction_torque_stamp_s = time.monotonic()

    def on_arm_motion_active(self, message) -> None:
        """Accept only fresh motion heartbeats; false clears immediately."""
        self.arm_motion_active_stamp_s = time.monotonic() if bool(message.data) else None

    def publish_wrench(self) -> None:
        if rclpy is None or not rclpy.ok() or self.rotation_flu_to_world is None:
            return
        thrust_scale = (
            ground_effect_thrust_scale(self.config, self.position_world_z)
            * self.battery_thrust_scale
        )
        commands_for_wrench = self.filtered_commands
        if (
            self.arm_torque_feedforward_enabled
            and self.arm_motion_active_stamp_s is not None
            and time.monotonic() - self.arm_motion_active_stamp_s < 1.0
            and self.arm_reaction_torque_stamp_s is not None
            and time.monotonic() - self.arm_reaction_torque_stamp_s < 0.6
        ):
            # FRD/FLU share x and invert y/z.  The rotor allocator is in FRD,
            # while the coupled-dynamics monitor publishes base_link FLU.
            reaction_frd = FRD_TO_FLU @ self.arm_reaction_torque_flu
            commands_for_wrench, _ = torque_feedforward_command_delta(
                self.config,
                self.filtered_commands,
                reaction_frd,
                self.arm_torque_matrix_pinv,
                self.arm_torque_feedforward_max_delta_n,
            )
        force_body, torque_body = direct_wrench_flu(
            self.config, commands_for_wrench, thrust_scale=thrust_scale
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
        try:
            self.publisher.publish(message)
        except Exception as exc:  # ROS context may close between the guard and publish.
            if rclpy is None or not rclpy.ok() or "context is invalid" in str(exc).lower():
                return
            raise


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
    parser.add_argument(
        "--arm-torque-feedforward-enabled",
        default="false",
        choices=("true", "false"),
        help="counter fresh SO101 reaction torque through rotor allocation",
    )
    parser.add_argument(
        "--arm-torque-feedforward-max-delta-n", type=float, default=2.0
    )
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
        parsed.arm_torque_feedforward_enabled == "true",
        parsed.arm_torque_feedforward_max_delta_n,
    )
    try:
        rclpy.spin(node)
    except ExternalShutdownException:
        pass
    finally:
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()


if __name__ == "__main__":
    main()
