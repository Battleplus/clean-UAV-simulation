"""Validate the example eight-rotor wrench allocation matrix."""

from __future__ import annotations

import argparse
import json
from pathlib import Path

import numpy as np


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


def allocation_matrix(config: dict) -> np.ndarray:
    """Map non-negative rotor thrusts [N] to body wrench [N, N m]."""
    # A canted-rotor geometry can have full 6D authority before propeller
    # reaction torque is known.  Treat an explicit JSON null as "not modelled"
    # instead of silently borrowing an unrelated example coefficient.
    moment_ratio = float(config.get("reaction_moment_ratio_m") or 0.0)
    columns = []
    for rotor in config["rotors"]:
        position = np.asarray(rotor["position_m"], dtype=float)
        axis = np.asarray(rotor["axis_body"], dtype=float)
        axis /= np.linalg.norm(axis)
        # direction follows PX4 CA_ROTOR*_KM: +1 is CCW / positive KM.
        # PX4 effectiveness uses moment = r x (CT*n) - CT*KM*n.
        direction = float(rotor["direction"])
        force = axis
        moment = np.cross(position, force) - direction * moment_ratio * force
        columns.append(np.concatenate((force, moment)))
    return np.column_stack(columns)


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
