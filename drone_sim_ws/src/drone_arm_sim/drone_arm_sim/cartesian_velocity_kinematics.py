"""ROS-independent small-step IK used by Cartesian velocity control."""

from __future__ import annotations

import math

import numpy as np


def _solve_soft_axis_ik(
    model,
    target_position: np.ndarray,
    target_forward: np.ndarray,
    seed: dict[str, float],
    tool_forward_axis_local: np.ndarray,
    *,
    position_tolerance_m: float,
    max_iterations: int = 1200,
    damping: float = 0.02,
) -> tuple[dict[str, float], dict[str, object]]:
    """Mirror the demo's position-priority, soft-tool-axis IK without ROS."""
    positions = dict(seed)
    position_error = math.inf
    direction_error = math.inf
    for iteration in range(max_iterations):
        current, active = model.forward_kinematics("gripper_link", positions)
        names = [item[0] for item in active]
        jacobian = model.jacobian("gripper_link", positions)
        current_forward = current[:3, :3] @ tool_forward_axis_local
        delta_position = target_position - current[:3, 3]
        delta_direction = np.cross(current_forward, target_forward)
        position_error = float(np.linalg.norm(delta_position))
        direction_error = float(np.linalg.norm(delta_direction))
        if position_error < position_tolerance_m:
            break
        projector = np.eye(3) - np.outer(current_forward, current_forward)
        task_jacobian = np.vstack((jacobian[:3], projector @ jacobian[3:]))
        axis_weight = 0.04
        error = np.concatenate((delta_position, axis_weight * delta_direction))
        weighted_jacobian = task_jacobian.copy()
        weighted_jacobian[3:] *= axis_weight
        normal = (
            weighted_jacobian @ weighted_jacobian.T
            + damping * damping * np.eye(6)
        )
        increment = weighted_jacobian.T @ np.linalg.solve(normal, error)
        norm = float(np.linalg.norm(increment))
        if norm > 0.12:
            increment *= 0.12 / norm
        for name, delta in zip(names, increment):
            if name == "wrist_roll":
                continue
            lower, upper = model.joint_limits(name)
            positions[name] = float(np.clip(positions[name] + delta, lower, upper))
    return positions, {
        "converged": position_error < position_tolerance_m,
        "iterations": iteration + 1,
        "position_error_m": position_error,
        "tool_axis_error_rad": math.asin(min(1.0, direction_error)),
    }


def solve_velocity_step_ik(
    model,
    target_position: np.ndarray,
    target_forward: np.ndarray,
    seed: dict[str, float],
    tool_forward_axis_local: np.ndarray,
    *,
    position_tolerance_m: float,
) -> tuple[dict[str, float], dict[str, object]]:
    """Refine XYZ at the unchanged gate when the soft axis is unattainable."""
    start_transform, _ = model.forward_kinematics("gripper_link", seed)
    requested_distance = float(
        np.linalg.norm(np.asarray(target_position) - start_transform[:3, 3])
    )
    solve_tolerance = max(
        1.0e-6,
        min(float(position_tolerance_m), 0.05 * requested_distance),
    )
    solution, status = _solve_soft_axis_ik(
        model,
        target_position,
        target_forward,
        seed,
        tool_forward_axis_local,
        position_tolerance_m=solve_tolerance,
    )
    if bool(status["converged"]):
        return solution, {**status, "axis_refinement": False}

    intermediate, _ = model.forward_kinematics("gripper_link", solution)
    attainable_forward = intermediate[:3, :3] @ tool_forward_axis_local
    attainable_forward /= np.linalg.norm(attainable_forward)
    refined, refined_status = _solve_soft_axis_ik(
        model,
        target_position,
        attainable_forward,
        solution,
        tool_forward_axis_local,
        position_tolerance_m=float(position_tolerance_m),
    )
    final_transform, _ = model.forward_kinematics("gripper_link", refined)
    final_forward = final_transform[:3, :3] @ tool_forward_axis_local
    final_forward /= np.linalg.norm(final_forward)
    original_axis_error = math.acos(
        float(np.clip(np.dot(final_forward, target_forward), -1.0, 1.0))
    )
    return refined, {
        **refined_status,
        "axis_refinement": True,
        "initial_position_error_m": float(status["position_error_m"]),
        "tool_axis_error_rad": original_axis_error,
    }
