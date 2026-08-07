"""Damped-least-squares inverse kinematics for the five-axis SO-101 chain."""

from __future__ import annotations

import argparse
from pathlib import Path

import numpy as np

from drone_arm_sim.model_analysis import UrdfModel, _default_urdf, _transform


ARM_JOINTS = [
    "shoulder_pan",
    "shoulder_lift",
    "elbow_flex",
    "wrist_flex",
    "wrist_roll",
]


def orientation_error(current: np.ndarray, target: np.ndarray) -> np.ndarray:
    """Orientation error expressed in the root frame."""
    return 0.5 * sum(
        np.cross(current[:, index], target[:, index]) for index in range(3)
    )


def solve_ik(
    model: UrdfModel,
    target: np.ndarray,
    seed: dict[str, float] | None = None,
    end_link: str = "gripper_frame_link",
    max_iterations: int = 500,
    damping: float = 0.04,
    orientation_weight: float = 0.25,
) -> tuple[dict[str, float], dict[str, float | int | bool]]:
    """Solve a weighted 6D task; orientation is approximate for a 5-DoF arm."""
    positions = {name: 0.0 for name in ARM_JOINTS}
    if seed:
        positions.update(seed)
    weights = np.diag([1.0, 1.0, 1.0, orientation_weight, orientation_weight,
                       orientation_weight])

    position_norm = np.inf
    orientation_norm = np.inf
    for iteration in range(max_iterations):
        current, active = model.forward_kinematics(end_link, positions)
        names = [item[0] for item in active]
        jacobian = model.jacobian(end_link, positions)
        position_delta = target[:3, 3] - current[:3, 3]
        rotation_delta = orientation_error(current[:3, :3], target[:3, :3])
        position_norm = float(np.linalg.norm(position_delta))
        orientation_norm = float(np.linalg.norm(rotation_delta))
        if position_norm < 1e-4 and orientation_norm < 2e-3:
            break

        error = np.concatenate((position_delta, rotation_delta))
        weighted_jacobian = weights @ jacobian
        weighted_error = weights @ error
        normal = (
            weighted_jacobian @ weighted_jacobian.T
            + damping * damping * np.eye(6)
        )
        delta = weighted_jacobian.T @ np.linalg.solve(normal, weighted_error)
        delta_norm = np.linalg.norm(delta)
        if delta_norm > 0.15:
            delta *= 0.15 / delta_norm

        for name, increment in zip(names, delta):
            lower, upper = model.joint_limits(name)
            positions[name] = float(
                np.clip(positions[name] + increment, lower, upper)
            )

    result = {
        "converged": position_norm < 1e-4 and orientation_norm < 2e-3,
        "iterations": iteration + 1,
        "position_error_m": position_norm,
        "orientation_error_rad": orientation_norm,
    }
    return positions, result


def _assignments(values: list[str]) -> dict[str, float]:
    result = {}
    for assignment in values:
        name, value = assignment.split("=", 1)
        result[name] = float(value)
    return result


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--urdf", type=Path, default=_default_urdf())
    parser.add_argument("--end-link", default="gripper_frame_link")
    parser.add_argument("--xyz", nargs=3, type=float)
    parser.add_argument("--rpy", nargs=3, type=float, default=[0.0, 0.0, 0.0])
    parser.add_argument("--seed", action="append", default=[], metavar="NAME=RAD")
    parser.add_argument(
        "--from-joints",
        action="append",
        default=[],
        metavar="NAME=RAD",
        help="Generate a reachable target pose from known joint angles.",
    )
    args = parser.parse_args()

    model = UrdfModel(args.urdf)
    if args.from_joints:
        source = _assignments(args.from_joints)
        target, _ = model.forward_kinematics(args.end_link, source)
        print("target generated from:", source)
    elif args.xyz:
        target = _transform(np.asarray(args.xyz), np.asarray(args.rpy))
    else:
        parser.error("provide --xyz X Y Z or one or more --from-joints NAME=RAD")

    solution, status = solve_ik(
        model,
        target,
        seed=_assignments(args.seed),
        end_link=args.end_link,
    )
    achieved, _ = model.forward_kinematics(args.end_link, solution)

    np.set_printoptions(precision=6, suppress=True)
    print("solution [rad]:")
    for name in ARM_JOINTS:
        print(f"  {name}: {solution[name]: .6f}")
    print("status:", status)
    print("target transform:")
    print(target)
    print("achieved transform:")
    print(achieved)
    if not status["converged"]:
        raise SystemExit(
            "IK did not meet the strict full-pose tolerance. "
            "A five-axis arm cannot reach every arbitrary 6D pose."
        )


if __name__ == "__main__":
    main()
