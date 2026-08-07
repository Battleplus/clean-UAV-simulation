"""Estimate free-floating base reaction from conservation of momentum."""

from __future__ import annotations

import argparse
from pathlib import Path

import numpy as np

from drone_arm_sim.model_analysis import UrdfModel, _default_urdf


def _vee(skew: np.ndarray) -> np.ndarray:
    return np.array([skew[2, 1], skew[0, 2], skew[1, 0]])


def _assignments(values: list[str]) -> dict[str, float]:
    result = {}
    for assignment in values:
        name, value = assignment.split("=", 1)
        result[name] = float(value)
    return result


def reaction_twist(
    model: UrdfModel,
    positions: dict[str, float],
    velocities: dict[str, float],
    finite_difference_step: float = 1e-6,
) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """Return base twist, relative momentum and momentum residual.

    The base twist [linear, angular] is the instantaneous velocity required
    to keep total linear and angular momentum equal to zero when no external
    wrench is applied.
    """
    next_positions = {
        name: value + finite_difference_step * velocities.get(name, 0.0)
        for name, value in positions.items()
    }
    for name, velocity in velocities.items():
        next_positions.setdefault(name, finite_difference_step * velocity)

    transforms = model.link_transforms(positions)
    next_transforms = model.link_transforms(next_positions)
    current_entries = {
        name: (mass, center, inertia)
        for name, mass, center, inertia in model.inertial_entries(positions)
    }
    next_entries = {
        name: (mass, center, inertia)
        for name, mass, center, inertia in model.inertial_entries(next_positions)
    }

    relative: dict[str, tuple[np.ndarray, np.ndarray]] = {}
    for name in current_entries:
        current_rotation = transforms[name][:3, :3]
        next_rotation = next_transforms[name][:3, :3]
        rotation_rate = (
            (next_rotation - current_rotation)
            / finite_difference_step
            @ current_rotation.T
        )
        angular = _vee(0.5 * (rotation_rate - rotation_rate.T))
        linear = (
            next_entries[name][1] - current_entries[name][1]
        ) / finite_difference_step
        relative[name] = (linear, angular)

    def momentum(
        base_linear: np.ndarray,
        base_angular: np.ndarray,
        include_relative: bool,
    ) -> np.ndarray:
        linear_momentum = np.zeros(3)
        angular_momentum = np.zeros(3)
        for name, (mass, center, inertia) in current_entries.items():
            relative_linear, relative_angular = relative[name]
            if not include_relative:
                relative_linear = np.zeros(3)
                relative_angular = np.zeros(3)
            link_linear = (
                base_linear
                + np.cross(base_angular, center)
                + relative_linear
            )
            link_angular = base_angular + relative_angular
            link_linear_momentum = mass * link_linear
            linear_momentum += link_linear_momentum
            angular_momentum += (
                inertia @ link_angular
                + np.cross(center, link_linear_momentum)
            )
        return np.concatenate((linear_momentum, angular_momentum))

    relative_momentum = momentum(np.zeros(3), np.zeros(3), True)
    coupling = np.zeros((6, 6))
    for column in range(6):
        base_linear = np.zeros(3)
        base_angular = np.zeros(3)
        if column < 3:
            base_linear[column] = 1.0
        else:
            base_angular[column - 3] = 1.0
        coupling[:, column] = momentum(base_linear, base_angular, False)

    base_twist = -np.linalg.solve(coupling, relative_momentum)
    residual = (
        coupling @ base_twist + relative_momentum
    )
    return base_twist, relative_momentum, residual


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--urdf", type=Path, default=_default_urdf())
    parser.add_argument("--joint", action="append", default=[], metavar="NAME=RAD")
    parser.add_argument(
        "--velocity",
        action="append",
        default=[],
        metavar="NAME=RAD_S",
    )
    args = parser.parse_args()

    positions = _assignments(args.joint)
    velocities = _assignments(args.velocity)
    if not velocities:
        velocities = {
            "shoulder_pan": 0.4,
            "shoulder_lift": -0.6,
            "elbow_flex": 0.8,
            "wrist_flex": -0.5,
            "wrist_roll": 0.3,
        }

    model = UrdfModel(args.urdf)
    base_twist, relative_momentum, residual = reaction_twist(
        model, positions, velocities
    )

    home_mass, home_center, _ = model.mass_properties({})
    _, current_center, _ = model.mass_properties(positions)
    quasi_static_translation = -(current_center - home_center)

    np.set_printoptions(precision=8, suppress=True)
    print(f"total mass [kg]: {home_mass:.6f}")
    print("joint positions [rad]:", positions)
    print("joint velocities [rad/s]:", velocities)
    print("arm-induced momentum [linear; angular]:", relative_momentum)
    print("reaction base linear velocity [m/s]:", base_twist[:3])
    print("reaction base angular velocity [rad/s]:", base_twist[3:])
    print("zero-momentum residual:", residual)
    print(
        "quasi-static base translation from home [m]:",
        quasi_static_translation,
    )
    if np.linalg.norm(residual) > 1e-7:
        raise SystemExit("Momentum conservation residual is too large.")


if __name__ == "__main__":
    main()
