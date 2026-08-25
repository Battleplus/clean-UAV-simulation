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
from drone_arm_sim.slow_adaptive_wrench import (
    SlowAdaptiveWrenchTrim,
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
FAST_PATH_REGULARIZATION = 1.0e-7
RECOVERABLE_COUPLING_STATE_REASONS = frozenset({"joint_state_stale"})


def classify_coupling_state_sample(
    state: dict,
    *,
    now_s: float,
    last_valid_stamp_s: float | None,
    hold_timeout_s: float,
) -> tuple[str, str | None, float | None, float | None]:
    """Classify one estimator report without turning a soft stale frame into a fault.

    A single ``joint_state_stale`` report can occur when the 100 Hz estimator
    timer lands just beyond its 100 ms input deadline.  The reallocator already
    owns a longer, explicit source lease.  Keep the last *validated* state only
    inside that lease; malformed, incomplete, or non-finite samples still fail
    closed immediately.
    """
    if not isinstance(state, dict):
        return "rejected", "invalid_state_payload", None, None
    valid = bool(
        state.get("estimator_valid")
        and state.get("source_fresh")
        and state.get("joint_state_valid", True)
    )
    if valid:
        try:
            maximum_velocity = float(state["maximum_joint_velocity_rad_s"])
            maximum_acceleration = float(
                state["filtered_joint_acceleration_peak_rad_s2"]
            )
        except (KeyError, TypeError, ValueError):
            return "rejected", "invalid_joint_motion_metrics", None, None
        if not np.all(np.isfinite([maximum_velocity, maximum_acceleration])):
            return "rejected", "non_finite_joint_motion_metrics", None, None
        return "accepted", None, maximum_velocity, maximum_acceleration

    reason = str(state.get("invalid_reason") or "invalid_coupling_state")
    last_age_s = (
        float("inf")
        if last_valid_stamp_s is None
        else float(now_s) - float(last_valid_stamp_s)
    )
    if (
        reason in RECOVERABLE_COUPLING_STATE_REASONS
        and np.isfinite(last_age_s)
        and 0.0 <= last_age_s <= max(0.0, float(hold_timeout_s))
    ):
        return "held_last_valid", reason, None, None
    return "rejected", reason, None, None


def direct_xy_force_command_is_fresh(
    enabled: bool,
    stamp_s: float | None,
    now_s: float,
    timeout_s: float,
) -> bool:
    """Return whether the independent direct-XY force grant is usable."""
    return bool(
        enabled
        and stamp_s is not None
        and np.isfinite(now_s)
        and np.isfinite(stamp_s)
        and 0.0 <= float(now_s) - float(stamp_s) <= max(0.0, float(timeout_s))
    )


def direct_xy_protocol_phase(
    *,
    position_feedback_enabled: bool,
    motion_active: bool,
    target_latched: bool,
    prepared: bool,
    force_active: bool,
) -> str:
    """Describe the fail-closed two-stage ownership state for telemetry."""
    if not position_feedback_enabled:
        return "disabled"
    if force_active:
        return "active"
    if motion_active and target_latched:
        return "prepared" if prepared else "blocked"
    return "idle"


def clear_revoked_direct_xy_force(
    current_wrench_frd: np.ndarray,
    non_direct_target_wrench_frd: np.ndarray,
    *,
    revoke: bool,
) -> np.ndarray:
    """Drop only stale direct-XY force while preserving all other axes."""
    current = np.asarray(current_wrench_frd, dtype=float)
    target = np.asarray(non_direct_target_wrench_frd, dtype=float)
    if current.shape != (6,) or target.shape != (6,):
        raise ValueError("wrench vectors must contain six values")
    if not np.all(np.isfinite(current)) or not np.all(np.isfinite(target)):
        raise ValueError("wrench vectors must be finite")
    result = current.copy()
    if revoke:
        result[:2] = target[:2]
    return result


class WrenchEffortBaseline:
    """Capture one time-weighted PX4 raw-wrench baseline per armed flight."""

    def __init__(self, hold_s: float = 5.0) -> None:
        self.hold_s = max(0.1, float(hold_s))
        self.duration_s = 0.0
        self.integral = np.zeros(6)
        self.value: np.ndarray | None = None

    def reset(self) -> None:
        self.duration_s = 0.0
        self.integral[:] = 0.0
        self.value = None

    def step(self, wrench_frd: np.ndarray, dt_s: float, *, eligible: bool) -> np.ndarray | None:
        if self.value is not None:
            return self.value.copy()
        try:
            wrench = np.asarray(wrench_frd, dtype=float)
            raw_dt = float(dt_s)
        except (TypeError, ValueError):
            wrench = np.empty(0)
            raw_dt = float("nan")
        dt = max(0.0, min(raw_dt, 0.1)) if np.isfinite(raw_dt) else 0.0
        if (
            not eligible
            or dt <= 0.0
            or wrench.shape != (6,)
            or not np.all(np.isfinite(wrench))
        ):
            self.duration_s = 0.0
            self.integral[:] = 0.0
            return None
        self.duration_s += dt
        self.integral += dt * wrench
        if self.duration_s + 1.0e-12 >= self.hold_s:
            self.value = self.integral / self.duration_s
            return self.value.copy()
        return None


def adaptive_quasi_static_gate(
    maximum_joint_velocity_rad_s: float | None,
    maximum_joint_acceleration_rad_s2: float | None,
    velocity_body_flu: np.ndarray | None,
    angular_velocity_body_flu: np.ndarray | None,
    *,
    joint_velocity_limit_rad_s: float,
    joint_acceleration_limit_rad_s2: float,
    body_speed_limit_m_s: float,
    angular_rate_limit_rad_s: float,
) -> bool:
    """Fail closed unless joints and aircraft are continuously quasi-static."""
    if maximum_joint_velocity_rad_s is None or maximum_joint_acceleration_rad_s2 is None:
        return False
    velocity = np.asarray(velocity_body_flu, dtype=float)
    angular = np.asarray(angular_velocity_body_flu, dtype=float)
    scalars = np.asarray(
        [maximum_joint_velocity_rad_s, maximum_joint_acceleration_rad_s2],
        dtype=float,
    )
    return bool(
        velocity.shape == (3,)
        and angular.shape == (3,)
        and np.all(np.isfinite(scalars))
        and np.all(np.isfinite(velocity))
        and np.all(np.isfinite(angular))
        and float(maximum_joint_velocity_rad_s) <= max(0.0, float(joint_velocity_limit_rad_s))
        and float(maximum_joint_acceleration_rad_s2) <= max(0.0, float(joint_acceleration_limit_rad_s2))
        and float(np.linalg.norm(velocity)) <= max(0.0, float(body_speed_limit_m_s))
        and float(np.linalg.norm(angular)) <= max(0.0, float(angular_rate_limit_rad_s))
    )


def adaptive_application_gate(
    enabled: bool,
    source_fresh: bool,
    truth_fresh: bool,
    flight_allowed: bool,
    has_headroom: bool,
) -> bool:
    """Fail closed before a retained adaptive state can reach the motors."""
    return bool(
        enabled
        and source_fresh
        and truth_fresh
        and flight_allowed
        and has_headroom
    )


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


def unconstrained_allocation_delta_map(config: dict) -> np.ndarray:
    """Return the default regularized thrust-delta map for fixed geometry.

    The matrix is computed once while the node starts, before flight.  Small
    direct-XY corrections normally remain strictly inside every motor bound,
    so applying this map avoids invoking SciPy's bounded solver in the 100 Hz
    motor callback.  Bound checks in ``allocate_total_wrench`` remain
    authoritative and fall back to the original solver near saturation.
    """
    matrix = allocation_matrix(config, position_key="position_m")
    if matrix.shape != (6, 8) or not np.all(np.isfinite(matrix)):
        raise ValueError("allocation matrix must be finite and 6x8")
    if int(np.linalg.matrix_rank(matrix)) != 6:
        raise ValueError("allocation matrix must have full wrench rank")
    # This is the closed-form solution of the same unconstrained objective as
    # ``lsq_linear([A; lambda I], [w; 0])`` used below.  Keeping lambda here
    # preserves the original closest-to-PX4 regularization on the fast path.
    lam = FAST_PATH_REGULARIZATION
    result = matrix.T @ np.linalg.solve(
        matrix @ matrix.T + (lam * lam) * np.eye(6),
        np.eye(6),
    )
    if result.shape != (8, 6) or not np.all(np.isfinite(result)):
        raise ValueError("allocation delta map must be finite and 8x6")
    return result


def allocate_total_wrench(
    config: dict,
    base_commands_motor_order: np.ndarray,
    compensation_wrench: np.ndarray,
    *,
    maximum_motor_delta_n: float,
    regularization: float = 1.0e-7,
    allocation_delta_map: np.ndarray | None = None,
) -> dict:
    """Perform one bounded 6x8 allocation around the PX4 Base 1 solution."""
    base_commands = np.asarray(base_commands_motor_order, dtype=float)
    compensation = np.asarray(compensation_wrench, dtype=float)
    if base_commands.shape != (8,) or not np.all(np.isfinite(base_commands)):
        raise ValueError("base_commands_motor_order must be eight finite values")
    if compensation.shape != (6,) or not np.all(np.isfinite(compensation)):
        raise ValueError("compensation_wrench must be six finite values")
    base_thrust = commands_to_config_thrust_n(config, base_commands)
    # Compensation is specified as a wrench about the composite vehicle CoM.
    # The direct Gazebo motor model publishes the equivalent wrench about the
    # base-link origin, but the allocator must use the CoM-relative rotor arms
    # (``position_m``).  Using ``wrench_position_m`` here makes a requested
    # pure horizontal force carry an uncommanded pitch/roll moment at the CoM.
    matrix = allocation_matrix(config, position_key="position_m")
    base_wrench = matrix @ base_thrust
    desired_wrench = base_wrench + compensation
    maximum = float(config["maximum_thrust_n"])
    delta = max(0.0, float(maximum_motor_delta_n))
    lower = np.maximum(0.0, base_thrust - delta)
    upper = np.minimum(maximum, base_thrust + delta)

    solver_success = False
    thrust: np.ndarray | None = None
    # The cached map is built for the node's fixed default objective.  A
    # library caller that requests a different Tikhonov regularization keeps
    # the original bounded-solver semantics instead of silently using a map
    # for another objective.
    fast_path_regularization_matches = bool(
        np.isclose(
            float(regularization),
            FAST_PATH_REGULARIZATION,
            rtol=0.0,
            atol=1.0e-15,
        )
    )
    if allocation_delta_map is not None and fast_path_regularization_matches:
        delta_map = np.asarray(allocation_delta_map, dtype=float)
        if delta_map.shape != (8, 6) or not np.all(np.isfinite(delta_map)):
            raise ValueError("allocation_delta_map must be finite and 8x6")
        unconstrained = base_thrust + delta_map @ compensation
        bound_tolerance = 1.0e-10
        if bool(
            np.all(unconstrained >= lower - bound_tolerance)
            and np.all(unconstrained <= upper + bound_tolerance)
        ):
            thrust = np.clip(unconstrained, lower, upper)
            solver_success = True

    if thrust is None:
        # The tiny second term selects the closest-to-PX4 solution in the 2D
        # allocation nullspace without turning this into a post-allocation
        # offset.  This bounded path is retained for real motor constraints.
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
        solver_success = bool(result.success)
    realized = matrix @ thrust
    residual = realized - desired_wrench
    commands = config_thrust_n_to_commands(config, thrust)
    tolerance = max(1.0e-9, 1.0e-6 * maximum)
    return {
        "commands_motor_order": commands,
        "base_thrust_config_order_n": base_thrust,
        "thrust_config_order_n": thrust,
        "base_wrench_frd": base_wrench,
        "desired_wrench_frd": desired_wrench,
        "realized_wrench_frd": realized,
        "residual_frd": residual,
        "residual_norm": float(np.linalg.norm(residual)),
        "success": solver_success,
        "saturated_low": thrust <= lower + tolerance,
        "saturated_high": thrust >= upper - tolerance,
    }


def allocate_feasible_compensation(
    config: dict,
    base_commands_motor_order: np.ndarray,
    requested_compensation_wrench: np.ndarray,
    *,
    maximum_motor_delta_n: float,
    maximum_residual_norm: float,
    boundary_margin: float = 0.98,
    maximum_backoff_steps: int = 6,
    allocation_delta_map: np.ndarray | None = None,
) -> dict:
    """Allocate a continuous, direction-preserving feasible compensation.

    A transiently infeasible arm wrench must never make the overlay disappear
    in one callback.  First try the complete wrench.  If the motor box cannot
    realize it, project the bounded result onto the requested six-dimensional
    wrench direction and re-allocate that scaled request with a small boundary
    margin.  Additional bounded backoff is only a numerical fallback.
    """
    requested = np.asarray(requested_compensation_wrench, dtype=float)
    if requested.shape != (6,) or not np.all(np.isfinite(requested)):
        raise ValueError("requested_compensation_wrench must contain six finite values")
    residual_limit = max(0.0, float(maximum_residual_norm))

    def acceptable(result: dict) -> bool:
        return bool(
            result["success"]
            and np.all(np.isfinite(result["commands_motor_order"]))
            and float(result["residual_norm"]) <= residual_limit
        )

    full = allocate_total_wrench(
        config,
        base_commands_motor_order,
        requested,
        maximum_motor_delta_n=maximum_motor_delta_n,
        allocation_delta_map=allocation_delta_map,
    )
    if acceptable(full) or float(np.linalg.norm(requested)) <= ZERO_EPSILON:
        return {
            "allocation": full,
            "requested_compensation_wrench_frd": requested,
            "delivered_compensation_wrench_frd": requested.copy(),
            "feasibility_scale": 1.0,
            "limited": False,
        }

    realized_increment = (
        np.asarray(full["realized_wrench_frd"], dtype=float)
        - np.asarray(full["base_wrench_frd"], dtype=float)
    )
    denominator = float(np.dot(requested, requested))
    projected_scale = float(np.dot(realized_increment, requested) / denominator)
    margin = float(np.clip(boundary_margin, 0.0, 1.0))
    scale = float(np.clip(projected_scale, 0.0, 1.0) * margin)

    for _ in range(max(1, int(maximum_backoff_steps))):
        delivered = scale * requested
        candidate = allocate_total_wrench(
            config,
            base_commands_motor_order,
            delivered,
            maximum_motor_delta_n=maximum_motor_delta_n,
            allocation_delta_map=allocation_delta_map,
        )
        if acceptable(candidate):
            return {
                "allocation": candidate,
                "requested_compensation_wrench_frd": requested,
                "delivered_compensation_wrench_frd": delivered,
                "feasibility_scale": scale,
                "limited": True,
            }
        scale *= 0.8

    # Zero is always inside the per-motor delta box because it reconstructs
    # the PX4 base solution.  This branch is an explicit solver-failure safety
    # fallback, not the normal infeasible-wrench path above.
    delivered = np.zeros(6)
    candidate = allocate_total_wrench(
        config,
        base_commands_motor_order,
        delivered,
        maximum_motor_delta_n=maximum_motor_delta_n,
        allocation_delta_map=allocation_delta_map,
    )
    return {
        "allocation": candidate,
        "requested_compensation_wrench_frd": requested,
        "delivered_compensation_wrench_frd": delivered,
        "feasibility_scale": 0.0,
        "limited": True,
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
        expected_mass_kg = float(getattr(arguments, "expected_mass_kg", 4.0))
        configured_mass_kg = float(
            self.config.get("estimated_all_up_mass_kg", -1.0)
        )
        if (
            not np.isfinite(expected_mass_kg)
            or expected_mass_kg <= 0.0
            or abs(configured_mass_kg - expected_mass_kg) > 1.0e-9
        ):
            raise ValueError(
                "wrench reallocator mass guard mismatch: "
                f"config={configured_mass_kg} expected={expected_mass_kg}"
            )
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
        # Build and validate the fixed-geometry interior solution before any
        # subscriptions or READY marker exist.  Besides removing SciPy from
        # the normal 100 Hz direct-XY path, this eagerly initializes NumPy's
        # linear-algebra backend outside flight.
        self.allocation_delta_map = (
            unconstrained_allocation_delta_map(self.config)
            if self.enabled
            else None
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
        self.adaptive_enabled = bool(arguments.adaptive_enabled)
        self.adaptive_baseline = WrenchEffortBaseline(arguments.adaptive_baseline_hold_s)
        self.adaptive_joint_velocity_limit_rad_s = max(
            0.0, float(arguments.adaptive_joint_velocity_limit_rad_s)
        )
        self.adaptive_joint_acceleration_limit_rad_s2 = max(
            0.0, float(arguments.adaptive_joint_acceleration_limit_rad_s2)
        )
        self.adaptive_body_speed_limit_m_s = max(
            0.0, float(arguments.adaptive_body_speed_limit_m_s)
        )
        self.adaptive_angular_rate_limit_rad_s = max(
            0.0, float(arguments.adaptive_angular_rate_limit_rad_s)
        )
        self.adaptive_update_residual_limit = max(
            0.0, float(arguments.adaptive_update_residual_limit)
        )
        self.adaptive_trim = SlowAdaptiveWrenchTrim(
            time_constant_s=arguments.adaptive_time_constant_s,
            leak_time_constant_s=arguments.adaptive_leak_time_constant_s,
            warmup_s=arguments.adaptive_warmup_s,
            force_deadband_n=arguments.adaptive_force_deadband_n,
            torque_deadband_nm=arguments.adaptive_torque_deadband_nm,
            horizontal_force_limit_n=arguments.adaptive_horizontal_force_limit_n,
            vertical_force_limit_n=arguments.adaptive_vertical_force_limit_n,
            torque_limit_nm=arguments.adaptive_torque_limit_nm,
            horizontal_force_rate_n_s=arguments.adaptive_horizontal_force_rate_n_s,
            vertical_force_rate_n_s=arguments.adaptive_vertical_force_rate_n_s,
            torque_rate_nm_s=arguments.adaptive_torque_rate_nm_s,
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
        self.coupling_state_receive_stamp_s: float | None = None
        self.coupling_state_status = "missing"
        self.coupling_state_invalid_reason: str | None = "no_coupling_state"
        self.maximum_joint_velocity_rad_s: float | None = None
        self.maximum_joint_acceleration_rad_s2: float | None = None
        self.flight_state_stamp_s: float | None = None
        self.flight_armed = False
        self.flight_offboard = False
        self.truth_position_world_enu: np.ndarray | None = None
        self.truth_velocity_body_flu: np.ndarray | None = None
        self.truth_angular_velocity_body_flu: np.ndarray | None = None
        self.rotation_body_to_world: np.ndarray | None = None
        self.truth_stamp_s: float | None = None
        self.arm_motion_active = False
        self.arm_motion_stamp_s: float | None = None
        self.position_target_world_enu: np.ndarray | None = None
        self.direct_xy_force_enabled = False
        self.direct_xy_force_session_id = ""
        self.direct_xy_retired_sessions: set[str] = set()
        self.direct_xy_force_epoch = -1
        self.direct_xy_force_lease_generation = -1
        self.direct_xy_minimum_enable_epoch = 0
        self.direct_xy_force_stamp_s: float | None = None
        self.direct_xy_force_timeout_s = max(
            0.05, float(getattr(arguments, "direct_xy_force_timeout_s", 0.20))
        )
        self.direct_xy_force_clear_pending = False
        self.position_feedback_was_active = False
        self.last_allocation_limited = False
        self.last_allocation_residual_norm = 0.0
        self.last_command_s: float | None = None
        # Cache the newest valid PX4 actuator level.  Gazebo already applies
        # zero-order hold to actuator commands; the reallocator must do the
        # same at its own fixed 100 Hz producer boundary instead of making its
        # physical output/report cadence depend on an upstream DDS callback.
        # A short upstream scheduling jitter can therefore no longer be
        # misclassified as a dead reallocator by the unchanged 40 ms guardian
        # watchdog.  All force/source leases below remain independently
        # fail-closed.
        self.latest_command_message = None
        self.last_log_s = 0.0
        self.last_diagnostic_s = 0.0
        self.last_diagnostic_protocol_signature = None
        self.diagnostic_period_s = 1.0 / max(
            1.0, min(200.0, float(arguments.diagnostic_rate_hz))
        )
        self.publisher = self.create_publisher(Actuators, arguments.output_topic, 20)
        self.diagnostic_publisher = self.create_publisher(
            String, arguments.diagnostic_state_topic, 20
        )
        self.command_refresh_timer = self.create_timer(
            self.diagnostic_period_s,
            self._on_command_refresh_timer,
        )
        # Motor commands are a latest-state stream, not an event journal.  A
        # deep reliable reader queue replays stale actuator levels after any
        # transient callback delay and can keep the 40 ms owner watchdog
        # unhealthy long after the original stall has ended.  The ros_gz
        # RELIABLE writer is DDS-compatible with this BEST_EFFORT reader; keep
        # only the newest sample so recovery always resumes at current PX4
        # output rather than draining historical motor commands.
        motor_input_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        direct_xy_safety_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self.create_subscription(
            Actuators,
            arguments.input_topic,
            self.on_command,
            motor_input_qos,
        )
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
        self.create_subscription(
            String,
            getattr(
                arguments,
                "direct_xy_force_command_topic",
                "/my_drone/arm_direct_xy_force_command",
            ),
            self.on_direct_xy_force_command,
            direct_xy_safety_qos,
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
                    "adaptive_enabled": self.adaptive_enabled,
                    "input_topic": arguments.input_topic,
                    "output_topic": arguments.output_topic,
                },
                sort_keys=True,
            )
        )

    def _publish_diagnostic_state(
        self,
        now_s: float,
        *,
        event: str,
        source_fresh: bool,
        flight_allowed: bool,
        headroom_ok: bool,
        motion_active: bool,
        input_commands: np.ndarray,
        output_commands: np.ndarray,
        requested_wrench_frd: np.ndarray,
        delivered_wrench_frd: np.ndarray,
        feasibility_scale: float,
        residual_norm: float,
        saturated: int,
        truth_fresh: bool,
        position_feedback_active: bool,
        position_feedback_prepared: bool,
    ) -> None:
        """Publish compact high-rate evidence without increasing text-log load."""
        protocol_signature = (
            str(event),
            bool(source_fresh),
            bool(motion_active),
            bool(position_feedback_prepared),
            bool(position_feedback_active),
            bool(self.direct_xy_force_enabled),
            str(getattr(self, "direct_xy_force_session_id", "legacy")),
            int(self.direct_xy_force_epoch),
            int(getattr(self, "direct_xy_force_lease_generation", 0)),
            bool(self.position_target_world_enu is not None),
        )
        protocol_changed = (
            protocol_signature
            != getattr(self, "last_diagnostic_protocol_signature", None)
        )
        if (
            not protocol_changed
            and now_s - self.last_diagnostic_s < self.diagnostic_period_s
        ):
            return
        self.last_diagnostic_s = now_s
        self.last_diagnostic_protocol_signature = protocol_signature
        def source_age(stamp_s: float | None) -> float | None:
            if stamp_s is None:
                return None
            age_s = now_s - stamp_s
            return float(age_s) if np.isfinite(age_s) and age_s >= 0.0 else None

        diagnostic_timeout_s = max(
            0.0, float(getattr(self, "source_timeout_s", 0.5))
        )

        def diagnostic_fresh(stamp_s: float | None) -> bool:
            age_s = source_age(stamp_s)
            return bool(age_s is not None and age_s <= diagnostic_timeout_s)

        coupling_fresh = diagnostic_fresh(
            getattr(self, "valid_state_stamp_s", None)
        )
        reaction_required = bool(
            getattr(self, "reaction_force_gain", 0.0) > 0.0
            or getattr(self, "reaction_torque_gain", 0.0) > 0.0
        )
        gravity_required = bool(getattr(self, "gravity_torque_gain", 0.0) > 0.0)
        reaction_fresh = diagnostic_fresh(
            getattr(self, "reaction_stamp_s", None)
        )
        gravity_fresh = diagnostic_fresh(
            getattr(self, "gravity_stamp_s", None)
        )
        invalid_sources = []
        if not coupling_fresh:
            invalid_sources.append("coupling_state")
        if reaction_required and not reaction_fresh:
            invalid_sources.append("reaction_wrench")
        if gravity_required and not gravity_fresh:
            invalid_sources.append("gravity_wrench")
        report = {
            "schema": "my_drone.base1-reallocator-state.v1",
            "monotonic_s": now_s,
            "event": event,
            "source_fresh": bool(source_fresh),
            "source_invalid_reasons": invalid_sources,
            "coupling_state_status": str(
                getattr(self, "coupling_state_status", "missing")
            ),
            "coupling_state_invalid_reason": getattr(
                self, "coupling_state_invalid_reason", "no_coupling_state"
            ),
            "coupling_state_age_s": source_age(
                getattr(self, "valid_state_stamp_s", None)
            ),
            "coupling_state_receive_age_s": source_age(
                getattr(self, "coupling_state_receive_stamp_s", None)
            ),
            "coupling_state_fresh": bool(coupling_fresh),
            "reaction_wrench_required": reaction_required,
            "reaction_wrench_age_s": source_age(
                getattr(self, "reaction_stamp_s", None)
            ),
            "reaction_wrench_fresh": bool(reaction_fresh),
            "gravity_wrench_required": gravity_required,
            "gravity_wrench_age_s": source_age(
                getattr(self, "gravity_stamp_s", None)
            ),
            "gravity_wrench_fresh": bool(gravity_fresh),
            "flight_allowed": bool(flight_allowed),
            "headroom_ok": bool(headroom_ok),
            "motion_active": bool(motion_active),
            "position_feedback_enabled": bool(self.position_feedback_enabled),
            "position_feedback_ready": bool(
                self.position_feedback_enabled
                and source_fresh
                and truth_fresh
                and flight_allowed
                and headroom_ok
            ),
            "position_feedback_active": bool(position_feedback_active),
            "position_feedback_prepared": bool(position_feedback_prepared),
            "direct_xy_protocol_phase": direct_xy_protocol_phase(
                position_feedback_enabled=self.position_feedback_enabled,
                motion_active=motion_active,
                target_latched=self.position_target_world_enu is not None,
                prepared=position_feedback_prepared,
                force_active=position_feedback_active,
            ),
            "direct_xy_force_enabled": bool(self.direct_xy_force_enabled),
            "direct_xy_force_session_id": str(
                getattr(self, "direct_xy_force_session_id", "")
            ),
            "direct_xy_force_epoch": int(self.direct_xy_force_epoch),
            "direct_xy_force_lease_generation": int(
                getattr(self, "direct_xy_force_lease_generation", 0)
            ),
            "direct_xy_minimum_enable_epoch": int(
                self.direct_xy_minimum_enable_epoch
            ),
            "direct_xy_force_command_fresh": direct_xy_force_command_is_fresh(
                self.direct_xy_force_enabled,
                self.direct_xy_force_stamp_s,
                now_s,
                self.direct_xy_force_timeout_s,
            ),
            "direct_xy_force_command_age_s": None
            if self.direct_xy_force_stamp_s is None
            else max(0.0, now_s - self.direct_xy_force_stamp_s),
            "position_target_latched": bool(
                self.position_target_world_enu is not None
            ),
            "truth_fresh": bool(truth_fresh),
            "allocation_limited": bool(self.last_allocation_limited),
            "input_commands": np.asarray(input_commands, dtype=float).tolist(),
            "output_commands": np.asarray(output_commands, dtype=float).tolist(),
            "requested_compensation_wrench_frd": np.asarray(
                requested_wrench_frd, dtype=float
            ).tolist(),
            "delivered_compensation_wrench_frd": np.asarray(
                delivered_wrench_frd, dtype=float
            ).tolist(),
            "feasibility_scale": float(feasibility_scale),
            "residual_norm": float(residual_norm),
            "saturated": int(saturated),
        }
        self.diagnostic_publisher.publish(
            String(data=json.dumps(report, sort_keys=True))
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
        now_s = time.monotonic()
        self.coupling_state_receive_stamp_s = now_s
        try:
            state = json.loads(message.data)
        except (TypeError, json.JSONDecodeError):
            self.valid_state_stamp_s = None
            self.coupling_state_status = "rejected"
            self.coupling_state_invalid_reason = "malformed_state_json"
            self.maximum_joint_velocity_rad_s = None
            self.maximum_joint_acceleration_rad_s2 = None
            return
        status, reason, maximum_velocity, maximum_acceleration = (
            classify_coupling_state_sample(
                state,
                now_s=now_s,
                last_valid_stamp_s=self.valid_state_stamp_s,
                hold_timeout_s=self.source_timeout_s,
            )
        )
        self.coupling_state_status = status
        self.coupling_state_invalid_reason = reason
        if status == "accepted":
            self.valid_state_stamp_s = now_s
            self.maximum_joint_velocity_rad_s = maximum_velocity
            self.maximum_joint_acceleration_rad_s2 = maximum_acceleration
        elif status == "rejected":
            self.valid_state_stamp_s = None
            self.maximum_joint_velocity_rad_s = None
            self.maximum_joint_acceleration_rad_s2 = None

    def on_vehicle_status(self, message) -> None:
        was_armed = self.flight_armed
        self.flight_armed = bool(
            int(message.arming_state) == int(VehicleStatus.ARMING_STATE_ARMED)
        )
        self.flight_offboard = bool(
            int(message.nav_state) == int(VehicleStatus.NAVIGATION_STATE_OFFBOARD)
        )
        self.flight_state_stamp_s = time.monotonic()
        if was_armed and not self.flight_armed:
            # Learned trim belongs to one payload/armed session.  Never carry
            # it into a later takeoff where the payload may have changed.
            self.adaptive_baseline.reset()
            self.adaptive_trim.reset()
            self.last_allocation_limited = False
            self.last_allocation_residual_norm = 0.0

    def on_truth(self, message) -> None:
        pose = message.pose.pose
        twist = message.twist.twist
        position = np.asarray(
            [pose.position.x, pose.position.y, pose.position.z], dtype=float
        )
        velocity = np.asarray(
            [twist.linear.x, twist.linear.y, twist.linear.z], dtype=float
        )
        angular_velocity = np.asarray(
            [twist.angular.x, twist.angular.y, twist.angular.z], dtype=float
        )
        quaternion = np.asarray(
            [pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w],
            dtype=float,
        )
        if not all(
            np.all(np.isfinite(value))
            for value in (position, velocity, angular_velocity)
        ):
            return
        try:
            rotation = quaternion_xyzw_to_rotation_body_to_world(quaternion)
        except ValueError:
            return
        self.truth_position_world_enu = position
        self.truth_velocity_body_flu = velocity
        self.truth_angular_velocity_body_flu = angular_velocity
        self.rotation_body_to_world = rotation
        self.truth_stamp_s = time.monotonic()
        if self.arm_motion_active and self.position_target_world_enu is None:
            self.position_target_world_enu = position.copy()

    def on_arm_motion(self, message) -> None:
        active = bool(message.data)
        now_s = time.monotonic()
        if active and not self.arm_motion_active:
            # A motion edge only prepares a target.  It must never inherit the
            # previous ownership grant: the DDS owner sends a newer epoch only
            # after its finite-XY loop has completed the hand-off.
            if self.direct_xy_force_enabled or self.position_feedback_was_active:
                self.direct_xy_force_clear_pending = True
            self.direct_xy_force_enabled = False
            self.direct_xy_force_stamp_s = None
            self.direct_xy_minimum_enable_epoch = self.direct_xy_force_epoch + 1
            self.position_target_world_enu = (
                None
                if self.truth_position_world_enu is None
                else self.truth_position_world_enu.copy()
            )
        elif not active and not self.direct_xy_force_enabled:
            self.position_target_world_enu = None
        self.arm_motion_active = active
        self.arm_motion_stamp_s = now_s if active else None

    def on_direct_xy_force_command(self, message) -> None:
        try:
            command = json.loads(message.data)
            if command.get("schema") != "my_drone.arm-direct-xy-force-command.v1":
                return
            epoch = int(command["ownership_epoch"])
            if type(command["enabled"]) is not bool:
                return
            enabled = command["enabled"]
            lease_generation = int(command.get("lease_generation", 0))
            session_id = str(command.get("controller_session_id", "legacy"))
        except (KeyError, TypeError, ValueError, json.JSONDecodeError):
            return
        if not session_id or len(session_id) > 128:
            return
        current_session = str(
            getattr(self, "direct_xy_force_session_id", "")
        )
        retired_sessions = getattr(self, "direct_xy_retired_sessions", set())
        if session_id in retired_sessions:
            return
        if session_id != current_session:
            # A process restart establishes its boot/session only with a
            # finite disable handshake.  Never let a delayed True from an old
            # DDS writer become the first grant after either process restarts.
            if enabled and session_id != "legacy":
                return
            if current_session:
                retired_sessions.add(current_session)
            self.direct_xy_retired_sessions = retired_sessions
            self.direct_xy_force_session_id = session_id
            if not (enabled and session_id == "legacy"):
                self.direct_xy_force_lease_generation = -1
                self.direct_xy_force_epoch = -1
                self.direct_xy_minimum_enable_epoch = 0
        latest_generation = int(
            getattr(self, "direct_xy_force_lease_generation", -1)
        )
        if lease_generation < latest_generation:
            return
        if epoch < self.direct_xy_force_epoch:
            return
        if enabled and epoch < self.direct_xy_minimum_enable_epoch:
            return
        if enabled and (
            not self.arm_motion_active
            or self.arm_motion_stamp_s is None
            or time.monotonic() - self.arm_motion_stamp_s
            > self.arm_motion_timeout_s
            or self.position_target_world_enu is None
        ):
            return
        was_enabled = self.direct_xy_force_enabled
        self.direct_xy_force_lease_generation = lease_generation
        self.direct_xy_force_session_id = session_id
        self.direct_xy_force_epoch = epoch
        self.direct_xy_force_enabled = enabled
        self.direct_xy_force_stamp_s = time.monotonic()
        if was_enabled and not enabled:
            self.direct_xy_force_clear_pending = True
        if not enabled and not self.arm_motion_active:
            self.position_target_world_enu = None

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

    def _on_command_refresh_timer(self) -> None:
        """Maintain the physical producer cadence from the latest PX4 level.

        The subscription callback still publishes with minimum latency.  This
        timer only fills a missed upstream interval, so it neither queues old
        actuator samples nor changes the command value.  Running in the same
        single-threaded executor also means a genuinely blocked allocator
        cannot fake a healthy heartbeat: both output and report stop together
        and the existing 40 ms guardian gate still trips.
        """
        if not self.enabled or self.latest_command_message is None:
            return
        now_s = time.monotonic()
        if (
            self.last_command_s is not None
            and now_s - self.last_command_s < self.diagnostic_period_s
        ):
            return
        self._process_command(self.latest_command_message, now_s=now_s)

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
        # ROS message callbacks receive independent message instances.  Keep a
        # deep copy so the fixed-rate refresh never observes later mutation by
        # middleware or test fixtures.
        self.latest_command_message = deepcopy(message)
        self._process_command(message, now_s=time.monotonic(), extracted=(commands, field))

    def _process_command(self, message, *, now_s: float, extracted=None) -> None:
        if extracted is None:
            commands, field = self._extract_commands(message)
            if commands is None:
                return
        else:
            commands, field = extracted
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
        direct_force_fresh = direct_xy_force_command_is_fresh(
            self.direct_xy_force_enabled,
            self.direct_xy_force_stamp_s,
            now_s,
            self.direct_xy_force_timeout_s,
        )
        if self.direct_xy_force_enabled and not direct_force_fresh:
            self.direct_xy_force_enabled = False
            self.direct_xy_force_clear_pending = True
            if not motion_active:
                self.position_target_world_enu = None
        if self.arm_motion_active and not motion_active:
            # Close a lost/stale motion session.  A later True message must
            # create a fresh edge instead of inheriting stale targets.
            self.arm_motion_active = False
            self.arm_motion_stamp_s = None
            if not self.direct_xy_force_enabled:
                self.position_target_world_enu = None
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
        raw_wrench_frd = (
            allocation_matrix(self.config, position_key="wrench_position_m")
            @ base_thrust
        )
        maximum = float(self.config["maximum_thrust_n"])
        has_headroom = bool(
            np.min(maximum - base_thrust) >= self.minimum_headroom_n
            and np.min(base_thrust) >= self.minimum_headroom_n
        )
        target = np.zeros(6)
        if source_fresh and flight_allowed and has_headroom:
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
        position_feedback_prepared = bool(
            self.position_feedback_enabled
            and (motion_active or direct_force_fresh)
            and truth_fresh
            and flight_allowed
            and has_headroom
            and self.position_target_world_enu is not None
            and self.truth_position_world_enu is not None
            and self.truth_velocity_body_flu is not None
            and self.rotation_body_to_world is not None
        )
        position_feedback_active = bool(
            position_feedback_prepared and direct_force_fresh
        )
        if position_feedback_active:
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
            # PX4 retains finite Z velocity ownership in the mixed-axis mode;
            # this overlay therefore owns XY only and must not form a second
            # vertical loop.
            position_force_frd[2] = 0.0
            target[:2] += position_force_frd[:2]
        quasi_static = adaptive_quasi_static_gate(
            self.maximum_joint_velocity_rad_s,
            self.maximum_joint_acceleration_rad_s2,
            self.truth_velocity_body_flu,
            self.truth_angular_velocity_body_flu,
            joint_velocity_limit_rad_s=self.adaptive_joint_velocity_limit_rad_s,
            joint_acceleration_limit_rad_s2=self.adaptive_joint_acceleration_limit_rad_s2,
            body_speed_limit_m_s=self.adaptive_body_speed_limit_m_s,
            angular_rate_limit_rad_s=self.adaptive_angular_rate_limit_rad_s,
        )
        gravity_delta_norm = float(
            np.linalg.norm(
                relative_gravity_wrench(
                    self.gravity_wrench_flu, self.gravity_reference_flu
                )[3:]
            )
        )
        baseline_eligible = bool(
            self.adaptive_enabled
            and source_fresh
            and truth_fresh
            and flight_allowed
            and has_headroom
            and quasi_static
            and not motion_active
            and gravity_delta_norm <= 0.01
        )
        adaptive_baseline_frd = self.adaptive_baseline.step(
            raw_wrench_frd, dt_s, eligible=baseline_eligible
        )
        adaptive_candidate_frd = (
            np.zeros(6)
            if adaptive_baseline_frd is None
            else raw_wrench_frd - adaptive_baseline_frd
        )
        adaptive_eligible = bool(
            self.adaptive_enabled
            and adaptive_baseline_frd is not None
            and source_fresh
            and truth_fresh
            and flight_allowed
            and has_headroom
            and quasi_static
            and not self.last_allocation_limited
            and self.last_allocation_residual_norm
            <= self.adaptive_update_residual_limit
        )
        adaptive_wrench_state_frd = self.adaptive_trim.step(
            adaptive_candidate_frd,
            dt_s,
            eligible=adaptive_eligible,
            allocation_limited=self.last_allocation_limited,
            quasi_static=quasi_static,
        )
        adaptive_application_allowed = adaptive_application_gate(
            self.adaptive_enabled,
            source_fresh,
            truth_fresh,
            flight_allowed,
            has_headroom,
        )
        adaptive_wrench_frd = (
            adaptive_wrench_state_frd
            if adaptive_application_allowed
            else np.zeros(6)
        )
        target += adaptive_wrench_frd
        direct_xy_force_ended = bool(
            self.direct_xy_force_clear_pending
            or (self.position_feedback_was_active and not position_feedback_active)
        )
        # Remove the direct XY component on the first motor callback after
        # revoke.  Do not slew an obsolete outer-loop force into the PX4
        # finite-XY restore window.  Other model/adaptive targets remain.
        self.current_compensation = clear_revoked_direct_xy_force(
            self.current_compensation,
            target,
            revoke=direct_xy_force_ended,
        )
        if direct_xy_force_ended:
            self.direct_xy_force_clear_pending = False
        self.position_feedback_was_active = position_feedback_active
        self.current_compensation = slew_vector(
            self.current_compensation,
            target,
            dt_s,
            self.force_slew_rate_n_s,
            self.torque_slew_rate_nm_s,
        )
        if not np.any(self.current_compensation):
            # Zero overlay is an exactly feasible allocation.  Do not leave a
            # previous transient limit latched forever after compensation has
            # returned continuously to zero.
            self.last_allocation_limited = False
            self.last_allocation_residual_norm = 0.0
            self.publisher.publish(message)
            self._publish_diagnostic_state(
                now_s,
                # Once the position loop has latched its motion target, an
                # exact zero correction is still a valid allocated owner
                # output.  Reporting it as allocated completes the two-stage
                # hand-off before the joint trajectory starts; idle zero
                # passthrough remains explicitly ``zero_overlay``.
                event="allocated" if position_feedback_active else "zero_overlay",
                source_fresh=source_fresh,
                flight_allowed=flight_allowed,
                headroom_ok=has_headroom,
                motion_active=motion_active,
                input_commands=commands,
                output_commands=commands,
                requested_wrench_frd=target,
                delivered_wrench_frd=self.current_compensation,
                feasibility_scale=1.0,
                residual_norm=0.0,
                saturated=0,
                truth_fresh=truth_fresh,
                position_feedback_active=position_feedback_active,
                position_feedback_prepared=position_feedback_prepared,
            )
            return
        feasible = allocate_feasible_compensation(
            self.config,
            commands,
            self.current_compensation,
            maximum_motor_delta_n=self.maximum_motor_delta_n,
            maximum_residual_norm=self.maximum_residual_norm,
            allocation_delta_map=self.allocation_delta_map,
        )
        allocation = feasible["allocation"]
        self.last_allocation_limited = bool(feasible["limited"])
        self.last_allocation_residual_norm = float(allocation["residual_norm"])
        adaptive_wrench_delivered_frd = (
            float(feasible["feasibility_scale"]) * adaptive_wrench_frd
        )
        adaptive_backcalculated = False
        if (
            not allocation["success"]
            or not np.all(np.isfinite(allocation["commands_motor_order"]))
            or allocation["residual_norm"] > self.maximum_residual_norm
        ):
            # A true numerical solver failure remains fail-safe. Ordinary
            # motor-bound infeasibility is handled by nonzero scaling above.
            self.publisher.publish(message)
            self._publish_diagnostic_state(
                now_s,
                event="allocation_failure",
                source_fresh=source_fresh,
                flight_allowed=flight_allowed,
                headroom_ok=has_headroom,
                motion_active=motion_active,
                input_commands=commands,
                output_commands=commands,
                requested_wrench_frd=self.current_compensation,
                delivered_wrench_frd=np.zeros(6),
                feasibility_scale=0.0,
                residual_norm=float(allocation["residual_norm"]),
                saturated=int(
                    np.count_nonzero(
                        allocation["saturated_low"] | allocation["saturated_high"]
                    )
                ),
                truth_fresh=truth_fresh,
                position_feedback_active=position_feedback_active,
                position_feedback_prepared=position_feedback_prepared,
            )
            return
        if self.last_allocation_limited and np.any(adaptive_wrench_frd):
            # The common feasibility scaler affects every overlay component.
            # Synchronise only the adaptive share to its delivered fraction.
            self.adaptive_trim.back_calculate(adaptive_wrench_delivered_frd)
            adaptive_backcalculated = True
        self.current_compensation = np.asarray(
            feasible["delivered_compensation_wrench_frd"], dtype=float
        )
        output = deepcopy(message)
        compensated = allocation["commands_motor_order"].tolist()
        if field == "normalized":
            output.normalized = compensated
        else:
            output.velocity = (self.velocity_scale * np.asarray(compensated)).tolist()
        self.publisher.publish(output)
        self._publish_diagnostic_state(
            now_s,
            event="allocated",
            source_fresh=source_fresh,
            flight_allowed=flight_allowed,
            headroom_ok=has_headroom,
            motion_active=motion_active,
            input_commands=commands,
            output_commands=np.asarray(compensated, dtype=float),
            requested_wrench_frd=feasible[
                "requested_compensation_wrench_frd"
            ],
            delivered_wrench_frd=self.current_compensation,
            feasibility_scale=float(feasible["feasibility_scale"]),
            residual_norm=float(allocation["residual_norm"]),
            saturated=int(
                np.count_nonzero(
                    allocation["saturated_low"] | allocation["saturated_high"]
                )
            ),
            truth_fresh=truth_fresh,
            position_feedback_active=position_feedback_active,
            position_feedback_prepared=position_feedback_prepared,
        )
        # Never format or write verbose text from the motor callback.  A
        # measured first-use log write blocked this single-threaded path for
        # 146 ms.  The compact diagnostic topic above is the authoritative
        # runtime record and can be persisted by an out-of-band recorder.


def main() -> None:
    package = Path(__file__).resolve().parents[1]
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--config",
        type=Path,
        default=package / "config/my_drone_v3_cad_debug_4kg.json",
    )
    parser.add_argument(
        "--expected-mass-kg",
        type=float,
        default=4.0,
        help="Fail closed unless the selected config has this all-up mass.",
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
    parser.add_argument(
        "--direct-xy-force-command-topic",
        default="/my_drone/arm_direct_xy_force_command",
    )
    parser.add_argument("--direct-xy-force-timeout-s", type=float, default=0.20)
    parser.add_argument("--truth-timeout-s", type=float, default=0.50)
    parser.add_argument("--arm-motion-timeout-s", type=float, default=1.0)
    parser.add_argument("--position-gain-n-m", type=float, default=4.0)
    parser.add_argument("--velocity-gain-n-s-m", type=float, default=2.0)
    parser.add_argument("--position-horizontal-limit-n", type=float, default=0.20)
    parser.add_argument("--position-vertical-limit-n", type=float, default=0.15)
    parser.add_argument("--adaptive-enabled", action="store_true")
    parser.add_argument("--adaptive-baseline-hold-s", type=float, default=5.0)
    parser.add_argument("--adaptive-time-constant-s", type=float, default=15.0)
    parser.add_argument("--adaptive-leak-time-constant-s", type=float, default=60.0)
    # Deprecated CLI compatibility only.  The old fast action-end decay is
    # intentionally not used by the residual-effort trim.
    parser.add_argument("--adaptive-decay-time-constant-s", type=float, default=60.0)
    parser.add_argument("--adaptive-warmup-s", type=float, default=3.0)
    parser.add_argument("--adaptive-force-deadband-n", type=float, default=0.02)
    parser.add_argument("--adaptive-torque-deadband-nm", type=float, default=0.005)
    parser.add_argument("--adaptive-horizontal-force-limit-n", type=float, default=0.06)
    parser.add_argument("--adaptive-vertical-force-limit-n", type=float, default=0.04)
    parser.add_argument("--adaptive-torque-limit-nm", type=float, default=0.02)
    parser.add_argument("--adaptive-horizontal-force-rate-n-s", type=float, default=0.005)
    parser.add_argument("--adaptive-vertical-force-rate-n-s", type=float, default=0.003)
    parser.add_argument("--adaptive-torque-rate-nm-s", type=float, default=0.002)
    parser.add_argument("--adaptive-joint-velocity-limit-rad-s", type=float, default=0.005)
    parser.add_argument(
        "--adaptive-joint-acceleration-limit-rad-s2", type=float, default=0.02
    )
    parser.add_argument("--adaptive-body-speed-limit-m-s", type=float, default=0.03)
    parser.add_argument(
        "--adaptive-angular-rate-limit-rad-s", type=float, default=0.008726646
    )
    parser.add_argument("--adaptive-update-residual-limit", type=float, default=0.005)
    parser.add_argument(
        "--diagnostic-state-topic",
        default="/my_drone/base1_reallocator/state",
    )
    parser.add_argument("--diagnostic-rate-hz", type=float, default=50.0)
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
