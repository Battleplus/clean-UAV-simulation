"""Six-DoF hover simulation with ideal wrench or eight lagged motors.

Frames follow PX4 conventions: world NED and body FRD.  Rotor axes in the
configuration point along the force that a positive rotor command applies.
"""

from __future__ import annotations

import argparse
import csv
import json
from pathlib import Path

import numpy as np
from scipy.optimize import lsq_linear

from drone_arm_sim.allocation_analysis import _default_config, allocation_matrix
from drone_arm_sim.model_analysis import UrdfModel, _default_urdf


def _skew(vector: np.ndarray) -> np.ndarray:
    x, y, z = vector
    return np.array([[0.0, -z, y], [z, 0.0, -x], [-y, x, 0.0]])


def _vee(matrix: np.ndarray) -> np.ndarray:
    return np.array([matrix[2, 1], matrix[0, 2], matrix[1, 0]])


def _rotation_exp(vector: np.ndarray) -> np.ndarray:
    angle = float(np.linalg.norm(vector))
    if angle < 1e-10:
        return np.eye(3) + _skew(vector)
    axis_skew = _skew(vector / angle)
    return (
        np.eye(3)
        + np.sin(angle) * axis_skew
        + (1.0 - np.cos(angle)) * axis_skew @ axis_skew
    )


def _rpy_matrix(rpy: np.ndarray) -> np.ndarray:
    roll, pitch, yaw = rpy
    cr, sr = np.cos(roll), np.sin(roll)
    cp, sp = np.cos(pitch), np.sin(pitch)
    cy, sy = np.cos(yaw), np.sin(yaw)
    return np.array(
        [
            [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
            [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
            [-sp, cp * sr, cp * cr],
        ]
    )


def _attitude_error(rotation: np.ndarray, desired: np.ndarray) -> np.ndarray:
    return 0.5 * _vee(desired.T @ rotation - rotation.T @ desired)


def allocate_bounded(
    matrix: np.ndarray,
    desired_wrench: np.ndarray,
    lower: float,
    upper: float,
    previous: np.ndarray,
) -> tuple[np.ndarray, np.ndarray]:
    """Bounded weighted least squares with mild command regularization."""
    wrench_scale = np.array([1.0, 1.0, 1.0, 5.0, 5.0, 5.0])
    weighted_matrix = wrench_scale[:, None] * matrix
    weighted_wrench = wrench_scale * desired_wrench
    regularization = 2e-3
    augmented_matrix = np.vstack(
        (weighted_matrix, regularization * np.eye(matrix.shape[1]))
    )
    augmented_target = np.concatenate(
        (weighted_wrench, regularization * previous)
    )
    result = lsq_linear(
        augmented_matrix,
        augmented_target,
        bounds=(lower, upper),
        method="trf",
        lsmr_tol="auto",
    )
    command = result.x
    return command, matrix @ command - desired_wrench


def simulate(
    model: UrdfModel,
    config: dict,
    mode: str,
    duration: float,
    dt: float,
) -> tuple[dict[str, float | np.ndarray], list[list[float]]]:
    mass, _, inertia = model.mass_properties({})
    matrix = allocation_matrix(config)
    gravity = float(config.get("gravity_m_s2", 9.81))
    lower = float(config.get("minimum_thrust_n", 0.0))
    upper = float(config.get("maximum_thrust_n", np.inf))
    motor_tau = float(config.get("motor_time_constant_s", 0.035))

    position = np.array([0.25, -0.20, 0.0])
    velocity = np.zeros(3)
    rotation = _rpy_matrix(np.deg2rad(np.array([5.0, -4.0, 3.0])))
    angular_velocity = np.zeros(3)
    desired_position = np.array([0.0, 0.0, -1.0])
    desired_rotation = np.eye(3)

    thrust = np.zeros(8)
    thrust_command = np.zeros(8)
    maximum_allocation_residual = 0.0
    maximum_thrust = 0.0
    log: list[list[float]] = []

    position_kp = np.array([3.2, 3.2, 4.5])
    position_kd = np.array([3.4, 3.4, 4.0])
    attitude_kp = np.array([0.55, 0.55, 0.35])
    attitude_kd = np.array([0.16, 0.16, 0.10])
    gravity_world = np.array([0.0, 0.0, gravity])

    steps = int(round(duration / dt))
    for step in range(steps + 1):
        time = step * dt
        position_error = desired_position - position
        desired_acceleration = (
            position_kp * position_error
            - position_kd * velocity
        )
        force_world = mass * (desired_acceleration - gravity_world)
        force_body = rotation.T @ force_world

        attitude_error = _attitude_error(rotation, desired_rotation)
        torque_body = (
            -attitude_kp * attitude_error
            - attitude_kd * angular_velocity
            + np.cross(angular_velocity, inertia @ angular_velocity)
        )
        desired_wrench = np.concatenate((force_body, torque_body))

        if mode == "direct":
            applied_wrench = desired_wrench
            allocation_residual = np.zeros(6)
        else:
            thrust_command, allocation_residual = allocate_bounded(
                matrix, desired_wrench, lower, upper, thrust_command
            )
            thrust += dt * (thrust_command - thrust) / motor_tau
            thrust = np.clip(thrust, lower, upper)
            applied_wrench = matrix @ thrust
            maximum_thrust = max(maximum_thrust, float(np.max(thrust)))
            maximum_allocation_residual = max(
                maximum_allocation_residual,
                float(np.linalg.norm(allocation_residual)),
            )

        force = applied_wrench[:3]
        torque = applied_wrench[3:]
        acceleration = gravity_world + rotation @ force / mass
        angular_acceleration = np.linalg.solve(
            inertia,
            torque - np.cross(angular_velocity, inertia @ angular_velocity),
        )

        velocity += acceleration * dt
        position += velocity * dt
        angular_velocity += angular_acceleration * dt
        rotation = rotation @ _rotation_exp(angular_velocity * dt)
        # Keep numerical integration on SO(3).
        u_matrix, _, vt_matrix = np.linalg.svd(rotation)
        rotation = u_matrix @ vt_matrix

        if step % max(1, int(round(0.02 / dt))) == 0:
            log.append(
                [
                    time,
                    *position,
                    *velocity,
                    *attitude_error,
                    *angular_velocity,
                    *thrust,
                ]
            )

    summary: dict[str, float | np.ndarray] = {
        "position": position,
        "position_error": desired_position - position,
        "velocity": velocity,
        "attitude_error": _attitude_error(rotation, desired_rotation),
        "angular_velocity": angular_velocity,
        "maximum_thrust": maximum_thrust,
        "maximum_allocation_residual": maximum_allocation_residual,
    }
    return summary, log


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--urdf", type=Path, default=_default_urdf())
    parser.add_argument("--config", type=Path, default=_default_config())
    parser.add_argument("--mode", choices=("direct", "motors"), default="motors")
    parser.add_argument("--duration", type=float, default=8.0)
    parser.add_argument("--dt", type=float, default=0.002)
    parser.add_argument("--csv", type=Path)
    args = parser.parse_args()

    model = UrdfModel(args.urdf)
    config = json.loads(args.config.read_text(encoding="utf-8"))
    summary, rows = simulate(model, config, args.mode, args.duration, args.dt)

    if args.csv:
        args.csv.parent.mkdir(parents=True, exist_ok=True)
        with args.csv.open("w", encoding="utf-8", newline="") as stream:
            writer = csv.writer(stream)
            writer.writerow(
                [
                    "time_s",
                    "north_m",
                    "east_m",
                    "down_m",
                    "vn_m_s",
                    "ve_m_s",
                    "vd_m_s",
                    "attitude_error_x",
                    "attitude_error_y",
                    "attitude_error_z",
                    "p_rad_s",
                    "q_rad_s",
                    "r_rad_s",
                    *[f"rotor_{index}_thrust_n" for index in range(1, 9)],
                ]
            )
            writer.writerows(rows)

    np.set_printoptions(precision=7, suppress=True)
    print("frames: world NED, body FRD")
    print("mode:", args.mode)
    print("final position NED [m]:", summary["position"])
    print("final position error [m]:", summary["position_error"])
    print("final velocity [m/s]:", summary["velocity"])
    print("final attitude error:", summary["attitude_error"])
    print("final angular velocity [rad/s]:", summary["angular_velocity"])
    if args.mode == "motors":
        print("maximum rotor thrust [N]:", summary["maximum_thrust"])
        print(
            "maximum allocation residual norm:",
            summary["maximum_allocation_residual"],
        )

    if np.linalg.norm(summary["position_error"]) > 0.04:
        raise SystemExit("Position controller did not settle within 4 cm.")
    if np.linalg.norm(summary["attitude_error"]) > np.deg2rad(1.0):
        raise SystemExit("Attitude controller did not settle within 1 degree.")


if __name__ == "__main__":
    main()
