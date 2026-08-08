#!/usr/bin/env python3
"""Deterministic task-level regression for the offline my_drone RL interface.

The smoke test only checks that a constant hover action is finite.  This
regression drives each supported curriculum task with a small, deterministic
controller and records the actual arm-pose and end-effector errors.  It is
still an offline model test; PX4/Gazebo remains the authority for flight and
contact validation.
"""

from __future__ import annotations

import argparse
import json
from pathlib import Path

import numpy as np

from drone_arm_sim.coupled_dynamics import JOINT_NAMES
from drone_arm_sim.gazebo_direct_motor_model import FRD_TO_FLU
from drone_arm_sim.rl_env import make_default_env


def _motor_action(env, correction_wrench: np.ndarray | None = None) -> np.ndarray:
    commands = env.hover_action()[:8].copy()
    if correction_wrench is not None:
        thrust_delta = np.linalg.pinv(env.allocation) @ np.asarray(correction_wrench, dtype=float)
        hover_thrust = np.asarray(env.config["bounded_hover_thrust_n"], dtype=float)
        commands = np.asarray(
            [env._thrust_to_command(t) for t in hover_thrust + thrust_delta],
            dtype=float,
        )
    return np.r_[np.clip(commands, 0.0, 1.0), np.zeros(6)]


def _joint_controller(
    env,
    target: np.ndarray,
    gain: float = 2.0,
    acceleration_limit_rad_s2: float = 0.6,
) -> np.ndarray:
    # The environment maps normalized joint actions to ±2 rad/s.
    desired_velocity = np.clip(gain * (target - env.joints), -2.0, 2.0)
    maximum_delta = acceleration_limit_rad_s2 * env.dt_s
    velocity = env.joint_velocities + np.clip(
        desired_velocity - env.joint_velocities, -maximum_delta, maximum_delta
    )
    action = _motor_action(env, _arm_stabilizing_wrench(env, velocity))
    action[8:] = velocity / 2.0
    return action


def _arm_stabilizing_wrench(env, desired_joint_velocity: np.ndarray) -> np.ndarray:
    q = dict(zip(JOINT_NAMES, env.joints))
    qd = dict(zip(JOINT_NAMES, desired_joint_velocity))
    qdd = dict(
        zip(
            JOINT_NAMES,
            (desired_joint_velocity - env.joint_velocities) / env.dt_s,
        )
    )
    state = env.dynamics.state(q, qd, qdd)
    reaction_frd = np.r_[
        FRD_TO_FLU @ state.reaction_force_body_n,
        FRD_TO_FLU @ state.reaction_torque_body_nm,
    ]
    attitude_feedback = np.r_[
        np.zeros(3),
        -1.2 * env.attitude_rpy - 0.35 * env.angular_velocity,
    ]
    return -reaction_frd + attitude_feedback


def _ee_controller(env, gain: float = 4.0) -> np.ndarray:
    q = env.joints.copy()
    target = env._target_ee_position()
    current = env._ee_position(q)
    h = 1.0e-5
    jacobian = np.zeros((3, 6))
    for index in range(6):
        plus = q.copy(); plus[index] += h
        minus = q.copy(); minus[index] -= h
        jacobian[:, index] = (env._ee_position(plus) - env._ee_position(minus)) / (2.0 * h)
    qd = np.linalg.pinv(jacobian) @ (gain * (target - current))
    qd = np.clip(qd, -2.0, 2.0)
    maximum_delta = 0.8 * env.dt_s
    qd = env.joint_velocities + np.clip(
        qd - env.joint_velocities, -maximum_delta, maximum_delta
    )
    action = _motor_action(env, _arm_stabilizing_wrench(env, qd))
    action[8:] = qd / 2.0
    return action


def run_task(task: str, steps: int, wind: np.ndarray | None = None) -> dict:
    env = make_default_env(task)
    if wind is not None:
        # Recreate with the same formal model but an explicit constant wind
        # acceleration, keeping the public helper as the single source of data.
        env.wind_ned_m_s2[:] = wind
    reset = env.reset(seed=17)
    _ = reset[0] if isinstance(reset, tuple) else reset
    position_norms: list[float] = []
    attitude_norms: list[float] = []
    joint_errors: list[float] = []
    ee_errors: list[float] = []
    terminated = truncated = False
    for _ in range(max(1, steps)):
        if task == "wind_hover":
            # Cancel the known disturbance through the same 6×8 allocation
            # matrix used by the flight model.
            correction = np.r_[-env.wind_ned_m_s2 * env.mass_kg, np.zeros(3)]
            action = _motor_action(env, correction)
        elif task == "arm_pose":
            action = _joint_controller(env, env._target_joints())
        elif task == "joint_trajectory":
            action = _joint_controller(
                env,
                env._target_joints(),
                gain=4.0,
                acceleration_limit_rad_s2=1.2,
            )
        elif task == "ee_trajectory":
            action = _ee_controller(env)
        else:
            action = _motor_action(env)
        observation, reward, terminated, truncated, info = env.step(action)
        if not np.all(np.isfinite(observation)) or not np.isfinite(reward):
            raise AssertionError(f"{task}: non-finite state")
        position_norms.append(float(np.linalg.norm(env.position_ned)))
        attitude_norms.append(float(np.linalg.norm(env.attitude_rpy)))
        joint_errors.append(float(np.linalg.norm(env.joints - info["target_joints_rad"])))
        ee_errors.append(float(info["end_effector_error_norm_m"]))
        if terminated or truncated:
            break
    return {
        "task": task,
        "steps": len(position_norms),
        "terminated": bool(terminated),
        "truncated": bool(truncated),
        "max_position_norm_m": max(position_norms),
        "max_attitude_norm_rad": max(attitude_norms),
        "final_joint_error_rad": joint_errors[-1],
        "max_ee_error_m": max(ee_errors),
        "final_ee_error_m": ee_errors[-1],
        "mass_kg": float(info["mass_kg"]),
        "px4_in_the_loop": False,
    }


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--steps", type=int, default=200)
    parser.add_argument(
        "--tasks",
        default="hover,wind_hover,arm_pose,joint_trajectory,ee_trajectory",
        help="comma-separated task list",
    )
    parser.add_argument("--output", type=Path)
    args = parser.parse_args()
    requested = [item.strip() for item in args.tasks.split(",") if item.strip()]
    reports = []
    for task in requested:
        reports.append(
            run_task(
                task,
                args.steps,
                np.array([0.15, -0.10, 0.0]) if task == "wind_hover" else None,
            )
        )
    # The regression is a numerical wiring check, not a tuned flight claim.
    # Safety termination or NaN is a hard failure; task errors are reported so
    # the next controller-tuning pass has a concrete baseline.
    by_task = {item["task"]: item for item in reports}
    gates = {
        "no_safety_termination": not any(item["terminated"] for item in reports),
        "hover_position_m": by_task.get("hover", {}).get("max_position_norm_m", 0.0) < 0.10,
        "wind_position_m": by_task.get("wind_hover", {}).get("max_position_norm_m", 0.0) < 0.50,
        "wind_attitude_rad": by_task.get("wind_hover", {}).get("max_attitude_norm_rad", 0.0) < 0.50,
        "arm_pose_error_rad": by_task.get("arm_pose", {}).get("final_joint_error_rad", 0.0) < 0.10,
        "arm_pose_position_m": by_task.get("arm_pose", {}).get("max_position_norm_m", 0.0) < 0.50,
        "arm_pose_attitude_rad": by_task.get("arm_pose", {}).get("max_attitude_norm_rad", 0.0) < 0.50,
        "joint_trajectory_error_rad": by_task.get("joint_trajectory", {}).get("final_joint_error_rad", 0.0) < 0.20,
        "joint_trajectory_position_m": by_task.get("joint_trajectory", {}).get("max_position_norm_m", 0.0) < 0.50,
        "joint_trajectory_attitude_rad": by_task.get("joint_trajectory", {}).get("max_attitude_norm_rad", 0.0) < 0.50,
        "ee_trajectory_error_m": by_task.get("ee_trajectory", {}).get("final_ee_error_m", 0.0) < 0.03,
        "ee_trajectory_position_m": by_task.get("ee_trajectory", {}).get("max_position_norm_m", 0.0) < 0.50,
        "ee_trajectory_attitude_rad": by_task.get("ee_trajectory", {}).get("max_attitude_norm_rad", 0.0) < 0.50,
    }
    if not all(gates.values()):
        reports.append({"acceptance_gates": gates})
        raise AssertionError(json.dumps(reports, indent=2))
    rendered = json.dumps({"schema": 1, "reports": reports, "acceptance_gates": gates}, indent=2)
    print("RL_TASK_REGRESSION_PASS " + rendered)
    if args.output:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(rendered + "\n", encoding="utf-8")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
