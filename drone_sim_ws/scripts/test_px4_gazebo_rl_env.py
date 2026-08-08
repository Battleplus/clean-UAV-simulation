#!/usr/bin/env python3
"""Live smoke test for the PX4/Gazebo RL observation and action bridge."""

from __future__ import annotations

import argparse
import json
import os
import pty
import select
import signal
import subprocess
import time

import numpy as np

from drone_arm_sim.px4_gazebo_rl_env import Px4GazeboArmEnv
from ros2_test_utils import ros2_child_environment


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--timeout", type=float, default=100.0)
    args = parser.parse_args()
    master, slave = pty.openpty()
    controller = subprocess.Popen(
        ["ros2", "run", "px4_ros2_control", "dds_wasd_control"],
        stdin=slave,
        stdout=slave,
        stderr=slave,
        start_new_session=True,
        close_fds=True,
        env=ros2_child_environment(),
    )
    os.close(slave)
    output = ""
    env = Px4GazeboArmEnv(observation_timeout_s=15.0)
    observations = []
    start = time.monotonic()
    try:
        observation, info = env.reset(options={"takeoff": True})
        observations.append(observation)
        pre_action_joint = None
        reached_offboard = False
        reached_height = False
        action_phase = False
        action_steps = 0
        settle_steps = 0
        causal_joint_delta = 0.0
        while time.monotonic() - start < args.timeout:
            if select.select([master], [], [], 0.0)[0]:
                try:
                    data = os.read(master, 65536).decode(errors="replace")
                except OSError:
                    data = ""
                output += data
            status = env._status
            if status is not None:
                reached_offboard |= bool(status[0] == 2.0 and status[1] == 14.0)
            action = np.zeros(env.action_size)
            if reached_offboard and len(observations) > 30 and action_steps < 5:
                # Bound the perturbation: five small arm endpoints are enough
                # to prove the command path without accumulating a large COM
                # shift. Exercise flight translation only for the first two.
                if pre_action_joint is None:
                    pre_action_joint = observation[12:18].copy()
                if action_steps < 2:
                    action[0] = 0.02
                action[4] = 0.20
                action_steps += 1
                action_phase = True
            elif action_steps >= 5:
                settle_steps += 1
            observation, _, terminated, _, step_info = env.step(action)
            observations.append(observation)
            reached_height |= abs(float(observation[2] - observations[0][2])) > 0.60
            if pre_action_joint is not None:
                causal_joint_delta = abs(
                    float(observation[12] - pre_action_joint[0])
                )
            if terminated:
                raise RuntimeError(f"live RL environment safety termination: {step_info}")
            if (
                reached_offboard
                and reached_height
                and action_phase
                and settle_steps >= 10
                and causal_joint_delta > 0.01
            ):
                break
            time.sleep(0.18)
        env.land()
        deadline = time.monotonic() + 30.0
        while time.monotonic() < deadline and controller.poll() is None:
            if select.select([master], [], [], 0.1)[0]:
                try:
                    output += os.read(master, 65536).decode(errors="replace")
                except OSError:
                    break
        final = observations[-1]
        joint_delta = (
            np.zeros(6)
            if pre_action_joint is None
            else final[12:18].astype(float) - pre_action_joint.astype(float)
        )
        report = {
            "px4_in_the_loop": True,
            "gazebo_in_the_loop": True,
            "observations": len(observations),
            "reached_offboard": reached_offboard,
            "reached_height": reached_height,
            "failsafe_seen": "failsafe=True" in output,
            "action_steps": action_steps,
            "commanded_shoulder_delta_rad": causal_joint_delta,
            "uncommanded_joint_delta_norm_rad": float(np.linalg.norm(joint_delta[1:])),
            "normal_disarm": "LANDING_DISARMED_CONFIRMED" in output,
        }
        print("PX4_GAZEBO_RL_METRICS " + json.dumps(report, sort_keys=True))
        passed = (
            report["reached_offboard"]
            and report["reached_height"]
            and not report["failsafe_seen"]
            and report["commanded_shoulder_delta_rad"] > 1.0e-4
            and report["normal_disarm"]
        )
        print("PX4_GAZEBO_RL_PASS" if passed else "PX4_GAZEBO_RL_FAIL")
        return 0 if passed else 1
    except Exception:
        # Preserve the controller-side reason in CI/log files when the live
        # environment aborts before the normal metrics block.
        if select.select([master], [], [], 0.2)[0]:
            try:
                output += os.read(master, 65536).decode(errors="replace")
            except OSError:
                pass
        print("PX4_GAZEBO_RL_CONTROLLER_TAIL")
        print(output[-8000:])
        raise
    finally:
        env.close()
        if controller.poll() is None:
            os.killpg(controller.pid, signal.SIGTERM)
            try:
                controller.wait(timeout=5)
            except subprocess.TimeoutExpired:
                os.killpg(controller.pid, signal.SIGKILL)
        os.close(master)


if __name__ == "__main__":
    raise SystemExit(main())
