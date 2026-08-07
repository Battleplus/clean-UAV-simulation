"""Validate the example eight-rotor wrench allocation matrix."""

from __future__ import annotations

import argparse
import json
from pathlib import Path

import numpy as np
from scipy.optimize import lsq_linear


def _default_config() -> Path:
    try:
        from ament_index_python.packages import get_package_share_directory

        return (
            Path(get_package_share_directory("drone_arm_sim"))
            / "config"
            / "octorotor_example.json"
        )
    except Exception:
        return (
            Path(__file__).resolve().parents[1]
            / "config"
            / "octorotor_example.json"
        )


def rotor_wrench_frd(
    config: dict,
    rotor: dict,
    thrust_n: float,
    position_key: str = "position_m",
) -> tuple[np.ndarray, np.ndarray]:
    """Return one rotor's force and torque in PX4 FRD for a positive thrust.

    ``position_m`` is the vehicle-COM allocation reference.  Runtime Gazebo
    application can request ``wrench_position_m`` to apply the same force at
    the URDF base-link origin.
    """
    position = np.asarray(
        rotor.get(position_key, rotor.get("position_m")), dtype=float
    )
    axis = np.asarray(rotor["axis_body"], dtype=float)
    axis /= np.linalg.norm(axis)
    force = float(thrust_n) * axis
    moment_ratio = float(config.get("reaction_moment_ratio_m") or 0.0)
    direction = float(rotor["direction"])
    torque = np.cross(position, force) - direction * moment_ratio * force
    return force, torque


def allocation_matrix(
    config: dict, position_key: str = "position_m"
) -> np.ndarray:
    """Map non-negative rotor thrusts [N] to body wrench [N, N m]."""
    # A canted-rotor geometry can have full 6D authority before propeller
    # reaction torque is known.  Treat an explicit JSON null as "not modelled"
    # instead of silently borrowing an unrelated example coefficient.
    columns = []
    for rotor in config["rotors"]:
        force, moment = rotor_wrench_frd(config, rotor, 1.0, position_key)
        columns.append(np.concatenate((force, moment)))
    return np.column_stack(columns)


def allocate_bounded_wrench(
    config: dict,
    desired_wrench: np.ndarray,
    lower_thrust_n: float = 0.0,
    upper_thrust_n: float | None = None,
) -> dict:
    """Solve bounded non-negative thrust allocation and expose saturation data."""
    desired = np.asarray(desired_wrench, dtype=float)
    if desired.shape != (6,) or not np.all(np.isfinite(desired)):
        raise ValueError("desired_wrench must be a finite 6-element vector")
    upper = (
        float(upper_thrust_n)
        if upper_thrust_n is not None
        else float(config.get("maximum_thrust_n", config.get("maximum_rated_thrust_per_motor_n")))
    )
    lower = float(lower_thrust_n)
    if not np.isfinite(lower) or not np.isfinite(upper) or lower < 0.0 or upper < lower:
        raise ValueError("invalid thrust bounds")
    matrix = allocation_matrix(config)
    result = lsq_linear(matrix, desired, bounds=(lower, upper), lsmr_tol="auto")
    thrust = np.asarray(result.x, dtype=float)
    residual = matrix @ thrust - desired
    tolerance = max(1e-8, 1e-6 * max(1.0, upper))
    saturated_low = thrust <= lower + tolerance
    saturated_high = thrust >= upper - tolerance
    return {
        "thrust_n": thrust,
        "wrench": matrix @ thrust,
        "residual": residual,
        "residual_norm": float(np.linalg.norm(residual)),
        "success": bool(result.success),
        "feasible": bool(result.success and np.linalg.norm(residual) <= 1e-8),
        "saturated_low": saturated_low,
        "saturated_high": saturated_high,
        "saturated_any": saturated_low | saturated_high,
        "matrix": matrix,
    }


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--config", type=Path, default=_default_config())
    args = parser.parse_args()

    config = json.loads(args.config.read_text(encoding="utf-8"))
    matrix = allocation_matrix(config)
    singular_values = np.linalg.svd(matrix, compute_uv=False)
    rank = int(np.linalg.matrix_rank(matrix))
    mass = float(config["example_mass_kg"])
    gravity = float(config.get("gravity_m_s2", 9.81))
    hover_wrench = np.array([0.0, 0.0, -mass * gravity, 0.0, 0.0, 0.0])
    hover_thrust = np.linalg.pinv(matrix) @ hover_wrench
    residual = matrix @ hover_thrust - hover_wrench

    np.set_printoptions(precision=6, suppress=True)
    print("WARNING: this geometry is an example, not measured aircraft data.")
    print("allocation matrix Gamma (6 x 8):")
    print(matrix)
    print("singular values:", singular_values)
    print("rank:", rank)
    print("condition number:", singular_values[0] / singular_values[-1])
    print("unconstrained hover thrusts [N]:", hover_thrust)
    print("minimum hover thrust [N]:", float(np.min(hover_thrust)))
    print("hover wrench residual:", residual)
    if rank != 6:
        raise SystemExit("Allocation matrix is not full row rank.")
    if np.min(hover_thrust) < 0:
        raise SystemExit(
            "Hover requires negative rotor thrust; geometry or allocation must change."
        )


if __name__ == "__main__":
    main()
