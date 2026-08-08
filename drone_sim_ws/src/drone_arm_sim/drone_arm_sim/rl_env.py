"""Dependency-light RL environment for the validated my_drone model.

This is an offline training interface, not a replacement for Gazebo/PX4.
It shares the formal allocation, thrust curve, battery model and arm mass
properties so policies can be prototyped without silently using the legacy
virtual-octorotor geometry.  PX4-in-the-loop and contact calibration remain
separate validation gates.
"""

from __future__ import annotations

from pathlib import Path
from typing import Any

import numpy as np

from drone_arm_sim.coupled_dynamics import CoupledArmDynamics, JOINT_NAMES
from drone_arm_sim.allocation_analysis import allocation_matrix
from drone_arm_sim.gazebo_direct_motor_model import (
    FRD_TO_FLU,
    actuator_command_to_thrust_n,
    battery_step,
    thrust_to_actuator_command,
)

try:
    import gymnasium as gym
    from gymnasium import spaces
except ModuleNotFoundError:
    gym = None
    spaces = None


class _BoxSpec:
    def __init__(self, low, high, shape, dtype=np.float32):
        self.low = np.broadcast_to(np.asarray(low, dtype=dtype), shape).copy()
        self.high = np.broadcast_to(np.asarray(high, dtype=dtype), shape).copy()
        self.shape = tuple(shape)
        self.dtype = dtype

    def sample(self):
        return np.random.uniform(self.low, self.high).astype(self.dtype)


BaseEnv = gym.Env if gym is not None else object


class MyDroneArmEnv(BaseEnv):
    """Small-step hover/arm environment with safety termination."""

    metadata = {"render_modes": []}
    observation_size = 25
    action_size = 14  # 8 normalized motor commands + 6 normalized joint velocities

    def __init__(
        self,
        config: dict,
        dynamics: CoupledArmDynamics,
        task: str = "hover",
        dt_s: float = 0.01,
        max_episode_steps: int = 2000,
        wind_ned_m_s2: np.ndarray | None = None,
    ) -> None:
        if task not in {"hover", "wind_hover", "arm_pose", "joint_trajectory", "ee_trajectory"}:
            raise ValueError(f"unknown task {task}")
        self.config = config
        self.dynamics = dynamics
        self.task = task
        self.dt_s = float(dt_s)
        self.max_episode_steps = int(max_episode_steps)
        self.wind_ned_m_s2 = np.zeros(3) if wind_ned_m_s2 is None else np.asarray(wind_ned_m_s2, dtype=float)
        if self.wind_ned_m_s2.shape != (3,):
            raise ValueError("wind_ned_m_s2 must have shape (3,)")
        self.mass_kg = float(config.get("estimated_all_up_mass_kg", 7.735))
        _, _, self.inertia = dynamics.mass_properties({})
        self.allocation = allocation_matrix(config)
        if spaces is not None:
            self.action_space = spaces.Box(
                low=np.r_[np.zeros(8), -np.ones(6)],
                high=np.r_[np.ones(8), np.ones(6)],
                dtype=np.float32,
            )
            self.observation_space = spaces.Box(
                low=-np.full(self.observation_size, np.inf, dtype=np.float32),
                high=np.full(self.observation_size, np.inf, dtype=np.float32),
                dtype=np.float32,
            )
        else:
            self.action_space = _BoxSpec(
                np.r_[np.zeros(8), -np.ones(6)],
                np.ones(self.action_size),
                (self.action_size,),
            )
            self.observation_space = _BoxSpec(-np.inf, np.inf, (self.observation_size,))
        self.np_random = np.random.default_rng()
        self.reset()

    def _hover_command(self) -> float:
        return float(self.config.get("hover_command", 0.8737))

    def _thrust_to_command(self, thrust_n: float) -> float:
        return thrust_to_actuator_command(self.config, thrust_n)

    def hover_action(self) -> np.ndarray:
        """Return the bounded per-motor hover command from the formal allocation."""
        hover = np.asarray(self.config.get("bounded_hover_thrust_n", []), dtype=float)
        if hover.shape == (8,):
            motors = np.asarray([self._thrust_to_command(value) for value in hover])
        else:
            motors = np.full(8, self._hover_command())
        return np.r_[motors, np.zeros(6)]

    def _target_joints(self) -> np.ndarray:
        if self.task == "arm_pose":
            return np.asarray(self.config.get("rl_work_pose_rad", [0.4, -0.6, 0.8, -0.5, 0.3, 0.8]), dtype=float)
        if self.task == "joint_trajectory":
            work = np.asarray(self.config.get("rl_work_pose_rad", [0.4, -0.6, 0.8, -0.5, 0.3, 0.8]), dtype=float)
            return self._trajectory_progress() * work
        if self.task == "ee_trajectory":
            return self.joints.copy()
        return np.zeros(6)

    def _ee_position(self, joints: np.ndarray) -> np.ndarray:
        positions = dict(zip(JOINT_NAMES, np.asarray(joints, dtype=float)))
        transform = self.dynamics.model.link_transforms(positions).get("gripper_link")
        if transform is None:
            raise ValueError("formal URDF does not contain gripper_link")
        return np.asarray(transform[:3, 3], dtype=float)

    def _trajectory_progress(self) -> float:
        duration = float(self.config.get("rl_trajectory_duration_s", 2.0))
        return float(np.clip(self.steps * self.dt_s / max(duration, self.dt_s), 0.0, 1.0))

    def _target_ee_position(self) -> np.ndarray:
        progress = self._trajectory_progress()
        target = (1.0 - progress) * self._home_ee_position + progress * self._work_ee_position
        if self.task == "ee_trajectory":
            target = target.copy()
            target[1] += 0.02 * np.sin(np.pi * progress)
        return target

    def _observation(self) -> np.ndarray:
        return np.concatenate(
            (
                self.position_ned,
                self.velocity_ned,
                self.attitude_rpy,
                self.angular_velocity,
                self.joints,
                self.joint_velocities,
                np.array([self.soc]),
            )
        ).astype(np.float32)

    def reset(self, *, seed: int | None = None, options: dict | None = None):
        if seed is not None:
            self.np_random = np.random.default_rng(seed)
        self.position_ned = np.zeros(3)
        self.velocity_ned = np.zeros(3)
        self.attitude_rpy = np.zeros(3)
        self.angular_velocity = np.zeros(3)
        self.joints = np.zeros(6)
        self.joint_velocities = np.zeros(6)
        self.previous_joint_velocities = np.zeros(6)
        self.soc = 1.0
        self.steps = 0
        self.last_action = self.hover_action()
        # End-effector trajectory endpoints are derived from the formal URDF,
        # so they stay aligned with the CAD arm if the model is regenerated.
        self._home_ee_position = self._ee_position(self.joints)
        work_pose = np.asarray(
            self.config.get("rl_work_pose_rad", [0.4, -0.6, 0.8, -0.5, 0.3, 0.8]),
            dtype=float,
        )
        self._work_ee_position = self._ee_position(work_pose)
        observation = self._observation()
        info = {"task": self.task, "px4_in_the_loop": False, "mass_kg": self.mass_kg}
        return (observation, info) if gym is not None else observation

    def step(self, action: np.ndarray):
        action = np.asarray(action, dtype=float)
        if action.shape != (self.action_size,):
            raise ValueError(f"expected action shape {(self.action_size,)}, got {action.shape}")
        if not np.all(np.isfinite(action)):
            raise ValueError("action contains non-finite values")
        action = np.clip(action, np.r_[np.zeros(8), -np.ones(6)], 1.0)
        motor_commands = action[:8]
        self.previous_joint_velocities = self.joint_velocities.copy()
        self.joint_velocities = action[8:] * 2.0
        joint_accelerations = (
            self.joint_velocities - self.previous_joint_velocities
        ) / self.dt_s
        self.joints += self.joint_velocities * self.dt_s
        for index, name in enumerate(JOINT_NAMES):
            lower, upper = self.dynamics.model.joint_limits(name)
            self.joints[index] = np.clip(self.joints[index], lower, upper)
        positions = dict(zip(JOINT_NAMES, self.joints))
        velocities = dict(zip(JOINT_NAMES, self.joint_velocities))
        arm_state = self.dynamics.state(
            positions,
            velocities,
            dict(zip(JOINT_NAMES, joint_accelerations)),
        )
        soc, _, thrust_scale, _ = battery_step(
            self.config, motor_commands, self.soc, self.dt_s
        )
        self.soc = soc
        thrusts = np.asarray(
            [actuator_command_to_thrust_n(self.config, value) for value in motor_commands]
        ) * thrust_scale
        body_wrench_frd = self.allocation @ thrusts
        # CoupledArmDynamics reports the reaction wrench in ROS FLU while
        # the allocation matrix is expressed in PX4 FRD.  Keep the arm
        # disturbance in the offline base dynamics without pretending this
        # replaces Gazebo/PX4 contact and attitude physics.
        reaction_wrench_frd = np.r_[
            FRD_TO_FLU @ arm_state.reaction_force_body_n,
            FRD_TO_FLU @ arm_state.reaction_torque_body_nm,
        ]
        total_wrench_frd = body_wrench_frd + reaction_wrench_frd
        acceleration_ned = (
            total_wrench_frd[:3] / arm_state.mass_kg
            + np.array([0.0, 0.0, 9.80665])
        )
        acceleration_ned += self.wind_ned_m_s2
        # The offline model is intentionally small-angle; it is used for RL
        # curriculum and reward wiring, while Gazebo remains the authority for
        # full attitude/contact physics.
        self.velocity_ned += acceleration_ned * self.dt_s
        self.position_ned += self.velocity_ned * self.dt_s
        angular_acceleration = np.linalg.solve(
            arm_state.inertia_at_com_kg_m2, total_wrench_frd[3:]
        )
        self.angular_velocity += angular_acceleration * self.dt_s
        self.attitude_rpy += self.angular_velocity * self.dt_s
        self.steps += 1
        self.last_action = action
        target_position = np.zeros(3)
        target_joints = self._target_joints()
        ee_position = self._ee_position(self.joints)
        ee_target = self._target_ee_position()
        ee_error = ee_position - ee_target
        reward = -(
            float(np.dot(self.position_ned - target_position, self.position_ned - target_position))
            + 0.1 * float(np.dot(self.velocity_ned, self.velocity_ned))
            + 0.2 * float(np.dot(self.attitude_rpy, self.attitude_rpy))
            + 0.01 * float(np.dot(action[8:], action[8:]))
            + 0.5 * float(np.dot(self.joints - target_joints, self.joints - target_joints))
            + (2.0 * float(np.dot(ee_error, ee_error)) if self.task == "ee_trajectory" else 0.0)
        )
        terminated = bool(
            np.linalg.norm(self.position_ned) > 5.0
            or np.max(np.abs(self.attitude_rpy)) > 0.8
            or self.soc <= 0.05
        )
        truncated = self.steps >= self.max_episode_steps
        if np.linalg.norm(self.position_ned) < 0.15 and np.linalg.norm(self.attitude_rpy) < 0.15:
            reward += 1.0
        info = {
            "task": self.task,
            "mass_kg": arm_state.mass_kg,
            "com_shift_m": arm_state.com_shift_m.copy(),
            "reaction_force_body_n": arm_state.reaction_force_body_n.copy(),
            "reaction_torque_body_nm": arm_state.reaction_torque_body_nm.copy(),
            "battery_thrust_scale": thrust_scale,
            "end_effector_position_m": ee_position.copy(),
            "end_effector_target_m": ee_target.copy(),
            "end_effector_error_m": ee_error.copy(),
            "end_effector_error_norm_m": float(np.linalg.norm(ee_error)),
            "target_joints_rad": target_joints.copy(),
            "safety_terminated": terminated,
        }
        observation = self._observation()
        if gym is not None:
            return observation, float(reward), terminated, truncated, info
        return observation, float(reward), terminated, truncated, info


def make_default_env(task: str = "hover") -> MyDroneArmEnv:
    import json

    package = Path(__file__).resolve().parents[1]
    config = json.loads(
        (package / "config/my_drone_v3_cad_7p735_flight.json").read_text(encoding="utf-8")
    )
    reference = json.loads(
        (package / "config/so101_motion_reference.json").read_text(encoding="utf-8")
    )
    dynamics = CoupledArmDynamics(
        package / "urdf/my_drone_v3/my_drone_cad_formal_dynamic.urdf",
        reference,
        target_mass_kg=7.735,
    )
    return MyDroneArmEnv(config, dynamics, task=task)
