#!/usr/bin/env python3
"""Run a dependency-light smoke test for the offline my_drone RL environment."""

from __future__ import annotations

import argparse
import json
from pathlib import Path

import numpy as np

from drone_arm_sim.rl_env import make_default_env


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--steps", type=int, default=200)
    parser.add_argument("--task", default="hover")
    args = parser.parse_args()
    env = make_default_env(args.task)
    reset = env.reset(seed=17)
    observation = reset[0] if isinstance(reset, tuple) else reset
    if observation.shape != env.observation_space.shape:
        raise AssertionError((observation.shape, env.observation_space.shape))
    action = env.hover_action()
    rewards = []
    terminated = truncated = False
    last_info = {}
    for _ in range(max(1, args.steps)):
        observation, reward, terminated, truncated, last_info = env.step(action)
        if not np.all(np.isfinite(observation)) or not np.isfinite(reward):
            raise AssertionError("non-finite RL state")
        rewards.append(float(reward))
        if terminated or truncated:
            break
    report = {
        "task": args.task,
        "steps": len(rewards),
        "observation_size": int(observation.size),
        "action_size": int(env.action_space.shape[0]),
        "terminated": bool(terminated),
        "truncated": bool(truncated),
        "minimum_reward": min(rewards),
        "mass_kg": float(last_info["mass_kg"]),
        "battery_thrust_scale": float(last_info["battery_thrust_scale"]),
        "px4_in_the_loop": False,
    }
    print("RL_ENV_SMOKE_PASS " + json.dumps(report, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
