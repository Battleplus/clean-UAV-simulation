"""Optional pre-allocation arm compensation for the protected Base 1 model.

The node is deliberately an overlay.  PX4's original eight motor commands are
decoded back to the 6D rotor wrench, a bounded arm compensation wrench is
added, and one constrained 6x8 allocation produces the commands consumed by
the unchanged Base 1 Gazebo motor model.  When compensation is disabled or
has ramped to zero, the incoming ROS message is forwarded without alteration.
"""

from __future__ import annotations

import argparse
from copy import deepcopy
import json
from pathlib import Path
import time

import numpy as np
from scipy.optimize import lsq_linear

from drone_arm_sim.allocation_analysis import allocation_matrix
from drone_arm_sim.gazebo_direct_motor_model import (
    actuator_command_to_thrust_n,
    thrust_to_actuator_command,
)

try:
    from actuator_msgs.msg import Actuators
    from geometry_msgs.msg import WrenchStamped
    from nav_msgs.msg import Odometry
    from px4_msgs.msg import VehicleStatus
    import rclpy
    from rclpy.executors import ExternalShutdownException
    from rclpy.node import Node
    from rclpy.qos import (
        DurabilityPolicy,
        HistoryPolicy,
        QoSProfile,
        ReliabilityPolicy,
    )
    from std_msgs.msg import Bool, String
except ModuleNotFoundError:  # Pure numerical tests do not require ROS 2.
    Actuators = WrenchStamped = Odometry = VehicleStatus = Bool = String = None
    DurabilityPolicy = HistoryPolicy = QoSProfile = ReliabilityPolicy = None
    rclpy = None
    ExternalShutdownException = RuntimeError
    Node = object


FLU_TO_FRD = np.diag([1.0, -1.0, -1.0])
ZERO_EPSILON = 1.0e-12


def flight_state_allows_compensation(
    armed: bool,
    offboard: bool,
    status_age_s: float,
    timeout_s: float,
) -> bool:
    """Fail closed unless a fresh PX4 status proves armed Offboard flight."""
    return bool(
        armed
        and offboard
        and np.isfinite(status_age_s)
        and 0.0 <= float(status_age_s) <= max(0.0, float(timeout_s))
    )


def motor_order_to_config_order(config: dict, values: np.ndarray) -> np.ndarray:
    """Convert motor-number order (PX4 index) to allocation-column order."""
    motor_values = np.asarray(values, dtype=float)
    if motor_values.shape != (8,):
        raise ValueError("motor commands must contain exactly eight values")
    indices = np.asarray([int(rotor["motor"]) - 1 for rotor in config["rotors"]])
    if sorted(indices.tolist()) != list(range(8)):
        raise ValueError("config rotors must map each PX4 motor 1..8 exactly once")
    return motor_values[indices]


def config_order_to_motor_order(config: dict, values: np.ndarray) -> np.ndarray:
    """Convert allocation-column order to PX4 motor-number order."""
    config_values = np.asarray(values, dtype=float)
    if config_values.shape != (8,):
        raise ValueError("rotor values must contain exactly eight values")
    result = np.zeros(8, dtype=float)
    for index, rotor in enumerate(config["rotors"]):
        result[int(rotor["motor"]) - 1] = config_values[index]
    return result


def commands_to_config_thrust_n(config: dict, commands: np.ndarray) -> np.ndarray:
    ordered = motor_order_to_config_order(config, commands)
    return np.asarray(
        [actuator_command_to_thrust_n(config, value) for value in ordered],
        dtype=float,
    )


def config_thrust_n_to_commands(config: dict, thrust_n: np.ndarray) -> np.ndarray:
    commands = np.asarray(
        [thrust_to_actuator_command(config, value) for value in thrust_n],
        dtype=float,
    )
    return config_order_to_motor_order(config, commands)


def slew_vector(
    current: np.ndarray,
    target: np.ndarray,
    dt_s: float,
    force_rate_n_s: float,
    torque_rate_nm_s: float,
) -> np.ndarray:
    """Slew each force/torque component; stale data therefore decays to zero."""
    value = np.asarray(current, dtype=float)
    goal = np.asarray(target, dtype=float)
    if value.shape != (6,) or goal.shape != (6,):
        raise ValueError("wrench vectors must contain six values")
    if not np.all(np.isfinite(value)) or not np.all(np.isfinite(goal)):
        raise ValueError("wrench vectors must be finite")
    dt = max(0.0, float(dt_s))
    limits = np.asarray(
        [max(0.0, force_rate_n_s)] * 3 + [max(0.0, torque_rate_nm_s)] * 3,
        dtype=float,
    ) * dt
    result = value + np.clip(goal - value, -limits, limits)
    result[np.abs(result) < ZERO_EPSILON] = 0.0
    return result


def bounded_vector(values: np.ndarray, norm_limit: float) -> np.ndarray:
    result = np.asarray(values, dtype=float).copy()
    if result.shape != (3,) or not np.all(np.isfinite(result)):
        raise ValueError("vector must be finite and three dimensional")
    limit = max(0.0, float(norm_limit))
    norm = float(np.linalg.norm(result))
    if limit <= 0.0:
        return np.zeros(3)
    if norm > limit:
        result *= limit / norm
    return result


def compensation_wrench_frd(
    reaction_wrench_flu: np.ndarray,
    gravity_wrench_flu: np.ndarray,
    *,
    reaction_force_gain: float,
    reaction_torque_gain: float,
    gravity_torque_gain: float,
    force_limit_n: float,
    reaction_torque_limit_nm: float,
    gravity_torque_limit_nm: float,
) -> np.ndarray:
    """Return the bounded wrench that opposes the estimated arm disturbance."""
    reaction = np.asarray(reaction_wrench_flu, dtype=float)
    gravity = np.asarray(gravity_wrench_flu, dtype=float)
    if reaction.shape != (6,) or gravity.shape != (6,):
        raise ValueError("source wrenches must contain six values")
    if not np.all(np.isfinite(reaction)) or not np.all(np.isfinite(gravity)):
        raise ValueError("source wrenches must be finite")
    force_frd = FLU_TO_FRD @ bounded_vector(
        reaction[:3], force_limit_n
    )
    reaction_torque_frd = FLU_TO_FRD @ bounded_vector(
        reaction[3:], reaction_torque_limit_nm
    )
    gravity_torque_frd = FLU_TO_FRD @ bounded_vector(
        gravity[3:], gravity_torque_limit_nm
    )
    # Both estimator outputs describe a disturbance exerted on the aircraft;
    # the rotor feed-forward request is their equal and opposite wrench.
    return np.concatenate(
        (
            -float(np.clip(reaction_force_gain, 0.0, 1.0)) * force_frd,
            -float(np.clip(reaction_torque_gain, 0.0, 1.0)) * reaction_torque_frd
            - float(np.clip(gravity_torque_gain, 0.0, 1.0)) * gravity_torque_frd,
        )
    )


def relative_gravity_wrench(
    gravity_wrench_flu: np.ndarray,
    reference_wrench_flu: np.ndarray | None,
) -> np.ndarray:
    """Return only the arm-pose gravity increment relative to takeoff trim.

    Base 1 already trims the gravity moment of the pose present before arming.
    Feeding that constant moment forward again changes a proven hover trim and
    caused the earlier A/B overlay to degrade.  Compensation therefore starts
    at zero and follows only pose-induced changes.
    """
    current = np.asarray(gravity_wrench_flu, dtype=float)
    if current.shape != (6,) or not np.all(np.isfinite(current)):
        raise ValueError("gravity wrench must contain six finite values")
    if reference_wrench_flu is None:
        return np.zeros(6)
    reference = np.asarray(reference_wrench_flu, dtype=float)
    if reference.shape != (6,) or not np.all(np.isfinite(reference)):
        raise ValueError("gravity reference must contain six finite values")
    return current - reference


def position_feedback_force_frd(
    target_world_enu: np.ndarray,
    position_world_enu: np.ndarray,
    velocity_body_flu: np.ndarray,
    rotation_body_to_world: np.ndarray,
    *,
    position_gain_n_m: float,
    velocity_gain_n_s_m: float,
    horizontal_limit_n: float,
    vertical_limit_n: float,
) -> np.ndarray:
    """Return a bounded body-FRD restoring force for an arm-motion snapshot.

    Gazebo odometry provides pose in ``world`` and twist in ``base_link``.
    The PD law is formed in the world ENU frame, limited there so horizontal
    and vertical authority stay explicit, and then rotated into body FLU/FRD
    for the 6D rotor allocator.
    """
    target = np.asarray(target_world_enu, dtype=float)
    position = np.asarray(position_world_enu, dtype=float)
    velocity_body = np.asarray(velocity_body_flu, dtype=float)
    rotation = np.asarray(rotation_body_to_world, dtype=float)
    if target.shape != (3,) or position.shape != (3,) or velocity_body.shape != (3,):
        raise ValueError("position feedback vectors must be three dimensional")
    if rotation.shape != (3, 3):
        raise ValueError("body-to-world rotation must be 3x3")
    if not all(
        np.all(np.isfinite(value))
        for value in (target, position, velocity_body, rotation)
    ):
        raise ValueError("position feedback inputs must be finite")
    velocity_world = rotation @ velocity_body
    force_world = (
        max(0.0, float(position_gain_n_m)) * (target - position)
        - max(0.0, float(velocity_gain_n_s_m)) * velocity_world
    )
    force_world[:2] = bounded_vector(force_world[:2].tolist() + [0.0], horizontal_limit_n)[:2]
    force_world[2] = float(
        np.clip(
            force_world[2],
            -max(0.0, float(vertical_limit_n)),
            max(0.0, float(vertical_limit_n)),
        )
    )
    force_body_flu = rotation.T @ force_world
    return FLU_TO_FRD @ force_body_flu


def quaternion_xyzw_to_rotation_body_to_world(values: np.ndarray) -> np.ndarray:
    quaternion = np.asarray(values, dtype=float)
    if quaternion.shape != (4,) or not np.all(np.isfinite(quaternion)):
        raise ValueError("quaternion must contain four finite values")
    norm = float(np.linalg.norm(quaternion))
    if norm <= ZERO_EPSILON:
        raise ValueError("quaternion norm must be positive")
    x, y, z, w = quaternion / norm
    return np.asarray(
        [
            [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
        ],
        dtype=float,
    )


def allocate_total_wrench(
    config: dict,
    base_commands_motor_order: np.ndarray,
    compensation_wrench: np.ndarray,
    *,
    maximum_motor_delta_n: float,
    force_limit_n: float = 60.0,
    reaction_torque_limit_nm: float = 1.0,
    gravity_torque_limit_nm: float = 1.0,
    regularization: float = 1.0e-7,
) -> dict:
    """Perform one bounded 6x8 allocation around the PX4 Base 1 solution."""
    base_commands = np.asarray(base_commands_motor_order, dtype=float)
    compensation = np.asarray(compensation_wrench, dtype=float)
    if base_commands.shape != (8,) or not np.all(np.isfinite(base_commands)):
        raise ValueError("base_commands_motor_order must be eight finite values")
    if compensation.shape != (6,) or not np.all(np.isfinite(compensation)):
        raise ValueError("compensation_wrench must be six finite values")
    base_thrust = commands_to_config_thrust_n(config, base_commands)
    matrix = allocation_matrix(config, position_key="wrench_position_m")
    base_wrench = matrix @ base_thrust
    desired_wrench = base_wrench + compensation
    maximum = float(config["maximum_thrust_n"])
    delta = max(0.0, float(maximum_motor_delta_n))
    lower = np.maximum(0.0, base_thrust - delta)
    upper = np.minimum(maximum, base_thrust + delta)

    # The tiny second term selects the closest-to-PX4 solution in the 2D
    # allocation nullspace without turning this into a post-allocation offset.
    lam = max(0.0, float(regularization))
    solve_matrix = np.vstack((matrix, lam * np.eye(8)))
    solve_target = np.concatenate((desired_wrench, lam * base_thrust))
    result = lsq_linear(
        solve_matrix,
        solve_target,
        bounds=(lower, upper),
        lsmr_tol="auto",
    )
    thrust = np.asarray(result.x, dtype=float)
    realized = matrix @ thrust
    residual = realized - desired_wrench
    commands = config_thrust_n_to_commands(config, thrust)
    tolerance = max(1.0e-9, 1.0e-6 * maximum)

    # Dimensionless weighted residual: force components normalized by
    # force_limit_n, torque components by max(reaction, gravity) limit.
    # This avoids mixing N and N·m in a single L2 norm.
    torque_limit = max(reaction_torque_limit_nm, gravity_torque_limit_nm, 1.0e-9)
    weights = np.array([
        force_limit_n, force_limit_n, force_limit_n,
        torque_limit, torque_limit, torque_limit,
    ])
    residual_normalized = residual / weights
    residual_norm = float(np.linalg.norm(residual_normalized))

    return {
        "commands_motor_order": commands,
        "base_thrust_config_order_n": base_thrust,
        "thrust_config_order_n": thrust,
        "base_wrench_frd": base_wrench,
        "desired_wrench_frd": desired_wrench,
        "realized_wrench_frd": realized,
        "residual_frd": residual,
        "residual_normalized": residual_normalized.tolist(),
        "residual_norm": residual_norm,
        "residual_force_norm_n": float(np.linalg.norm(residual[:3])),
        "residual_torque_norm_nm": float(np.linalg.norm(residual[3:])),
        "success": bool(result.success),
        "saturated_low": thrust <= lower + tolerance,
        "saturated_high": thrust >= upper - tolerance,
    }


def _message_wrench(message) -> np.ndarray:
    return np.asarray(
        [
            message.wrench.force.x,
            message.wrench.force.y,
            message.wrench.force.z,
            message.wrench.torque.x,
            message.wrench.torque.y,
            message.wrench.torque.z,
        ],
        dtype=float,
    )


class Base1WrenchReallocator(Node):
    def __init__(self, arguments: argparse.Namespace) -> None:
        super().__init__("base1_arm_wrench_reallocator")
        self.config = json.loads(arguments.config.read_text(encoding="utf-8"))
        if abs(float(self.config.get("estimated_all_up_mass_kg", -1.0)) - 4.0) > 1.0e-9:
            raise ValueError("Base 1 reallocator refuses any config other than 4.0 kg")
        self.enabled = bool(arguments.enabled)
        self.velocity_scale = float(arguments.velocity_command_scale)
        self.source_timeout_s = max(0.01, float(arguments.source_timeout_s))
        self.flight_state_timeout_s = max(
            0.05, float(arguments.flight_state_timeout_s)
        )
        self.reaction_force_gain = float(arguments.reaction_force_gain)
        self.reaction_torque_gain = float(arguments.reaction_torque_gain)
        self.gravity_torque_gain = float(arguments.gravity_torque_gain)
        self.force_limit_n = float(arguments.force_limit_n)
        self.reaction_torque_limit_nm = float(arguments.reaction_torque_limit_nm)
        self.gravity_torque_limit_nm = float(arguments.gravity_torque_limit_nm)
        self.force_slew_rate_n_s = float(arguments.force_slew_rate_n_s)
        self.torque_slew_rate_nm_s = float(arguments.torque_slew_rate_nm_s)
        self.maximum_motor_delta_n = float(arguments.maximum_motor_delta_n)
        self.minimum_headroom_n = float(arguments.minimum_headroom_n)
        self.maximum_residual_norm = max(
            0.0, float(arguments.maximum_residual_norm)
        )
        self.position_feedback_enabled = bool(arguments.position_feedback_enabled)
        self.position_gain_n_m = max(0.0, float(arguments.position_gain_n_m))
        self.velocity_gain_n_s_m = max(0.0, float(arguments.velocity_gain_n_s_m))
        self.position_horizontal_limit_n = max(
            0.0, float(arguments.position_horizontal_limit_n)
        )
        self.position_vertical_limit_n = max(
            0.0, float(arguments.position_vertical_limit_n)
        )
        self.truth_timeout_s = max(0.01, float(arguments.truth_timeout_s))
        self.arm_motion_timeout_s = max(0.05, float(arguments.arm_motion_timeout_s))
        self.current_compensation = np.zeros(6)
        self.reaction_wrench_flu = np.zeros(6)
        self.gravity_wrench_flu = np.zeros(6)
        self.gravity_reference_flu: np.ndarray | None = None
        self.reaction_stamp_s: float | None = None
        self.gravity_stamp_s: float | None = None
        self.valid_state_stamp_s: float | None = None
        self.flight_state_stamp_s: float | None = None
        self.flight_armed = False
        self.flight_offboard = False
        self.truth_position_world_enu: np.ndarray | None = None
        self.truth_velocity_body_flu: np.ndarray | None = None
        self.rotation_body_to_world: np.ndarray | None = None
        self.truth_stamp_s: float | None = None
        self.arm_motion_active = False
        self.arm_motion_stamp_s: float | None = None
        self.position_target_world_enu: np.ndarray | None = None
        self.last_command_s: float | None = None
        self.last_log_s = 0.0
        self.publisher = self.create_publisher(Actuators, arguments.output_topic, 20)
        self.create_subscription(Actuators, arguments.input_topic, self.on_command, 20)
        self.create_subscription(
            WrenchStamped, arguments.reaction_topic, self.on_reaction, 20
        )
        self.create_subscription(
            WrenchStamped, arguments.gravity_topic, self.on_gravity, 20
        )
        self.create_subscription(String, arguments.state_topic, self.on_state, 20)
        self.create_subscription(Odometry, arguments.truth_topic, self.on_truth, 20)
        self.create_subscription(
            Bool, arguments.arm_motion_topic, self.on_arm_motion, 20
        )
        px4_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self.create_subscription(
            VehicleStatus,
            arguments.vehicle_status_topic,
            self.on_vehicle_status,
            px4_qos,
        )
        self.get_logger().info(
            "BASE1_REALLOCATOR_READY "
            + json.dumps(
                {
                    "enabled": self.enabled,
                    "reaction_force_gain": self.reaction_force_gain,
                    "reaction_torque_gain": self.reaction_torque_gain,
                    "gravity_torque_gain": self.gravity_torque_gain,
                    "position_feedback_enabled": self.position_feedback_enabled,
                    "input_topic": arguments.input_topic,
                    "output_topic": arguments.output_topic,
                },
                sort_keys=True,
            )
        )

    def on_reaction(self, message) -> None:
        values = _message_wrench(message)
        if str(message.header.frame_id) == "base_link" and np.all(np.isfinite(values)):
            self.reaction_wrench_flu = values
            self.reaction_stamp_s = time.monotonic()

    def on_gravity(self, message) -> None:
        values = _message_wrench(message)
        if str(message.header.frame_id) == "base_link" and np.all(np.isfinite(values)):
            self.gravity_wrench_flu = values
            self.gravity_stamp_s = time.monotonic()
            # Continuously learn the exact trim pose only while disarmed.  The
            # last ground sample is then frozen for the entire armed flight.
            if not self.flight_armed:
                self.gravity_reference_flu = values.copy()

    def on_state(self, message) -> None:
        try:
            state = json.loads(message.data)
        except (TypeError, json.JSONDecodeError):
            self.valid_state_stamp_s = None
            return
        if bool(state.get("estimator_valid")) and bool(state.get("source_fresh")):
            self.valid_state_stamp_s = time.monotonic()
        else:
            self.valid_state_stamp_s = None

    def on_vehicle_status(self, message) -> None:
        self.flight_armed = bool(
            int(message.arming_state) == int(VehicleStatus.ARMING_STATE_ARMED)
        )
        self.flight_offboard = bool(
            int(message.nav_state) == int(VehicleStatus.NAVIGATION_STATE_OFFBOARD)
        )
        self.flight_state_stamp_s = time.monotonic()

    def on_truth(self, message) -> None:
        pose = message.pose.pose
        twist = message.twist.twist
        position = np.asarray(
            [pose.position.x, pose.position.y, pose.position.z], dtype=float
        )
        velocity = np.asarray(
            [twist.linear.x, twist.linear.y, twist.linear.z], dtype=float
        )
        quaternion = np.asarray(
            [pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w],
            dtype=float,
        )
        if not np.all(np.isfinite(position)) or not np.all(np.isfinite(velocity)):
            return
        try:
            rotation = quaternion_xyzw_to_rotation_body_to_world(quaternion)
        except ValueError:
            return
        self.truth_position_world_enu = position
        self.truth_velocity_body_flu = velocity
        self.rotation_body_to_world = rotation
        self.truth_stamp_s = time.monotonic()
        if self.arm_motion_active and self.position_target_world_enu is None:
            self.position_target_world_enu = position.copy()

    def on_arm_motion(self, message) -> None:
        active = bool(message.data)
        now_s = time.monotonic()
        if active and not self.arm_motion_active:
            self.position_target_world_enu = (
                None
                if self.truth_position_world_enu is None
                else self.truth_position_world_enu.copy()
            )
        elif not active:
            self.position_target_world_enu = None
        self.arm_motion_active = active
        self.arm_motion_stamp_s = now_s if active else None

    def _fresh(self, stamp_s: float | None, now_s: float) -> bool:
        return bool(stamp_s is not None and 0.0 <= now_s - stamp_s <= self.source_timeout_s)

    def _extract_commands(self, message) -> tuple[np.ndarray | None, str | None]:
        normalized = np.asarray(message.normalized, dtype=float)
        if normalized.size >= 8 and np.all(np.isfinite(normalized[:8])):
            return np.clip(normalized[:8], 0.0, 1.0), "normalized"
        velocity = np.asarray(message.velocity, dtype=float)
        if velocity.size >= 8 and np.all(np.isfinite(velocity[:8])):
            return np.clip(velocity[:8] / self.velocity_scale, 0.0, 1.0), "velocity"
        return None, None

    def on_command(self, message) -> None:
        # This exact branch is the non-regression contract: with the overlay
        # disabled, Base 1 sees a byte-for-byte-equivalent ROS message.
        if not self.enabled:
            self.publisher.publish(message)
            return
        commands, field = self._extract_commands(message)
        if commands is None:
            self.publisher.publish(message)
            return
        now_s = time.monotonic()
        dt_s = 0.0 if self.last_command_s is None else min(now_s - self.last_command_s, 0.1)
        self.last_command_s = now_s
        source_stamps = [self.valid_state_stamp_s]
        if self.reaction_force_gain > 0.0 or self.reaction_torque_gain > 0.0:
            source_stamps.append(self.reaction_stamp_s)
        if self.gravity_torque_gain > 0.0:
            source_stamps.append(self.gravity_stamp_s)
        source_fresh = all(self._fresh(stamp, now_s) for stamp in source_stamps)
        motion_active = bool(
            self.arm_motion_active
            and self.arm_motion_stamp_s is not None
            and 0.0 <= now_s - self.arm_motion_stamp_s <= self.arm_motion_timeout_s
        )
        truth_fresh = bool(
            self.truth_stamp_s is not None
            and 0.0 <= now_s - self.truth_stamp_s <= self.truth_timeout_s
        )
        flight_allowed = flight_state_allows_compensation(
            self.flight_armed,
            self.flight_offboard,
            float("inf")
            if self.flight_state_stamp_s is None
            else now_s - self.flight_state_stamp_s,
            self.flight_state_timeout_s,
        )
        base_thrust = commands_to_config_thrust_n(self.config, commands)
        maximum = float(self.config["maximum_thrust_n"])
        has_headroom = bool(
            np.min(maximum - base_thrust) >= self.minimum_headroom_n
            and np.min(base_thrust) >= self.minimum_headroom_n
        )

        # ── Hard safety gate: bypass immediately ─────────────────────────
        # Disarmed / not Offboard / no headroom → zero compensation and
        # forward the unmodified PX4 message on the very next frame.
        # Estimator source stale → controlled fade-out via slew.
        if not flight_allowed or not has_headroom:
            self.current_compensation[:] = 0.0
            self.publisher.publish(message)
            return

        target = np.zeros(6)
        if source_fresh:
            gravity_delta = relative_gravity_wrench(
                self.gravity_wrench_flu, self.gravity_reference_flu
            )
            target = compensation_wrench_frd(
                self.reaction_wrench_flu,
                gravity_delta,
                reaction_force_gain=self.reaction_force_gain,
                reaction_torque_gain=self.reaction_torque_gain,
                gravity_torque_gain=self.gravity_torque_gain,
                force_limit_n=self.force_limit_n,
                reaction_torque_limit_nm=self.reaction_torque_limit_nm,
                gravity_torque_limit_nm=self.gravity_torque_limit_nm,
            )
        position_force_frd = np.zeros(3)
        if (
            self.position_feedback_enabled
            and motion_active
            and truth_fresh
            and self.position_target_world_enu is not None
            and self.truth_position_world_enu is not None
            and self.truth_velocity_body_flu is not None
            and self.rotation_body_to_world is not None
        ):
            position_force_frd = position_feedback_force_frd(
                self.position_target_world_enu,
                self.truth_position_world_enu,
                self.truth_velocity_body_flu,
                self.rotation_body_to_world,
                position_gain_n_m=self.position_gain_n_m,
                velocity_gain_n_s_m=self.velocity_gain_n_s_m,
                horizontal_limit_n=self.position_horizontal_limit_n,
                vertical_limit_n=self.position_vertical_limit_n,
            )
            target[:3] += position_force_frd
        self.current_compensation = slew_vector(
            self.current_compensation,
            target,
            dt_s,
            self.force_slew_rate_n_s,
            self.torque_slew_rate_nm_s,
        )
        if not np.any(self.current_compensation):
            self.publisher.publish(message)
            return
        allocation = allocate_total_wrench(
            self.config,
            commands,
            self.current_compensation,
            maximum_motor_delta_n=self.maximum_motor_delta_n,
            force_limit_n=self.force_limit_n,
            reaction_torque_limit_nm=self.reaction_torque_limit_nm,
            gravity_torque_limit_nm=self.gravity_torque_limit_nm,
        )
        if (
            not allocation["success"]
            or not np.all(np.isfinite(allocation["commands_motor_order"]))
            or allocation["residual_norm"] > self.maximum_residual_norm
        ):
            # Never freeze or replay the last compensated motor vector.
            self.current_compensation = np.zeros(6)
            self.publisher.publish(message)
            return
        output = deepcopy(message)
        compensated = allocation["commands_motor_order"].tolist()
        if field == "normalized":
            output.normalized = compensated
        else:
            output.velocity = (self.velocity_scale * np.asarray(compensated)).tolist()
        self.publisher.publish(output)
        if now_s - self.last_log_s >= 1.0:
            self.last_log_s = now_s
            self.get_logger().info(
                "BASE1_COMPENSATION_STATE "
                + json.dumps(
                    {
                        "source_fresh": source_fresh,
                        "flight_allowed": flight_allowed,
                        "headroom_ok": has_headroom,
                        "motion_active": motion_active,
                        "truth_fresh": truth_fresh,
                        "position_target_world_enu": None
                        if self.position_target_world_enu is None
                        else self.position_target_world_enu.tolist(),
                        "position_feedback_force_frd": position_force_frd.tolist(),
                        "gravity_reference_ready": self.gravity_reference_flu is not None,
                        "gravity_delta_wrench_flu": relative_gravity_wrench(
                            self.gravity_wrench_flu, self.gravity_reference_flu
                        ).tolist(),
                        "compensation_wrench_frd": self.current_compensation.tolist(),
                        "residual_norm": allocation["residual_norm"],
                        "saturated": int(
                            np.count_nonzero(
                                allocation["saturated_low"] | allocation["saturated_high"]
                            )
                        ),
                    },
                    sort_keys=True,
                )
            )


def main() -> None:
    package = Path(__file__).resolve().parents[1]
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--config",
        type=Path,
        default=package / "config/my_drone_v3_cad_debug_4kg.json",
    )
    parser.add_argument("--enabled", action="store_true")
    parser.add_argument("--input-topic", default="/my_drone/command/motor_speed")
    parser.add_argument(
        "--output-topic", default="/my_drone/base1_compensated/command/motor_speed"
    )
    parser.add_argument(
        "--reaction-topic", default="/my_drone/base1_estimator/reaction_wrench_body"
    )
    parser.add_argument(
        "--gravity-topic", default="/my_drone/base1_estimator/gravity_shift_wrench_body"
    )
    parser.add_argument(
        "--state-topic", default="/my_drone/base1_estimator/coupling_state"
    )
    parser.add_argument("--velocity-command-scale", type=float, default=1000.0)
    parser.add_argument("--source-timeout-s", type=float, default=0.50)
    parser.add_argument("--flight-state-timeout-s", type=float, default=5.0)
    parser.add_argument(
        "--vehicle-status-topic", default="/fmu/out/vehicle_status_v4"
    )
    parser.add_argument("--reaction-force-gain", type=float, default=0.0)
    parser.add_argument("--reaction-torque-gain", type=float, default=0.0)
    parser.add_argument("--gravity-torque-gain", type=float, default=0.0)
    parser.add_argument("--force-limit-n", type=float, default=1.0)
    parser.add_argument("--reaction-torque-limit-nm", type=float, default=0.10)
    parser.add_argument("--gravity-torque-limit-nm", type=float, default=0.10)
    parser.add_argument("--force-slew-rate-n-s", type=float, default=1.0)
    parser.add_argument("--torque-slew-rate-nm-s", type=float, default=0.10)
    parser.add_argument("--maximum-motor-delta-n", type=float, default=0.50)
    parser.add_argument("--minimum-headroom-n", type=float, default=0.25)
    parser.add_argument("--maximum-residual-norm", type=float, default=0.02)
    parser.add_argument("--position-feedback-enabled", action="store_true")
    parser.add_argument("--truth-topic", default="/model/my_drone/odometry")
    parser.add_argument("--arm-motion-topic", default="/my_drone/arm_motion_active")
    parser.add_argument("--truth-timeout-s", type=float, default=0.50)
    parser.add_argument("--arm-motion-timeout-s", type=float, default=1.0)
    parser.add_argument("--position-gain-n-m", type=float, default=4.0)
    parser.add_argument("--velocity-gain-n-s-m", type=float, default=2.0)
    parser.add_argument("--position-horizontal-limit-n", type=float, default=0.20)
    parser.add_argument("--position-vertical-limit-n", type=float, default=0.15)
    parsed, ros_arguments = parser.parse_known_args()
    if rclpy is None:
        raise SystemExit("ROS 2 Python packages are not available")
    rclpy.init(args=ros_arguments)
    node = Base1WrenchReallocator(parsed)
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
