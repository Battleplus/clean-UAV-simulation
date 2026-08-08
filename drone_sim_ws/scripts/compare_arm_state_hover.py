#!/usr/bin/env python3
"""Compare hover allocation for retracted, expanded and working arm states."""

from __future__ import annotations

import argparse
import json
from pathlib import Path

import numpy as np

from drone_arm_sim.allocation_analysis import allocate_bounded_wrench
from drone_arm_sim.coupled_dynamics import CoupledArmDynamics, JOINT_NAMES
from drone_arm_sim.gazebo_direct_motor_model import FRD_TO_FLU


def thrust_to_command(config: dict, thrust_n: float) -> float:
    points = config["static_thrust_model"]["points"]
    thrust = np.asarray([
        float(item.get("rated_capped_thrust_n", item["measured_thrust_n"]))
        for item in points
    ])
    throttle = np.asarray([float(item["throttle_percent"]) for item in points])
    return float(np.clip(np.interp(thrust_n, thrust, throttle) / 100.0, 0.0, 1.0))


def main() -> int:
    workspace = Path(__file__).resolve().parents[1]
    package = workspace / "src/drone_arm_sim"
    parser = argparse.ArgumentParser()
    parser.add_argument("--output", type=Path, default=workspace / "analysis/arm_state_hover_comparison.json")
    args = parser.parse_args()
    config = json.loads((package / "config/my_drone_v3_cad_7p735_flight.json").read_text(encoding="utf-8"))
    reference = json.loads((package / "config/so101_motion_reference.json").read_text(encoding="utf-8"))
    dynamics = CoupledArmDynamics(
        package / "urdf/my_drone_v3/my_drone_cad_formal_dynamic.urdf",
        reference,
        target_mass_kg=7.735,
    )
    states = {
        "retracted": ("retracted", None, None),
        "expanded_work_a": ("work_a", None, None),
        "working_work_a": (
            "work_a",
            (0.25, -0.20, 0.30, -0.20, 0.15, 0.10),
            (0.50, -0.40, 0.60, -0.40, 0.30, 0.20),
        ),
        "working_work_b": (
            "work_b",
            (-0.20, 0.16, -0.24, 0.16, -0.12, -0.08),
            (-0.40, 0.32, -0.48, 0.32, -0.24, -0.16),
        ),
    }
    report_states = {}
    for label, (preset, velocity, acceleration) in states.items():
        positions = dict(zip(JOINT_NAMES, reference["presets"][preset]))
        velocities = None if velocity is None else dict(zip(JOINT_NAMES, velocity))
        accelerations = None if acceleration is None else dict(zip(JOINT_NAMES, acceleration))
        state = dynamics.state(positions, velocities, accelerations)
        reaction_frd = np.r_[
            FRD_TO_FLU @ state.reaction_force_body_n,
            FRD_TO_FLU @ state.reaction_torque_body_nm,
        ]
        hover_target = np.array([0.0, 0.0, -state.mass_kg * 9.80665, 0.0, 0.0, 0.0])
        force_only_target = hover_target - np.r_[reaction_frd[:3], np.zeros(3)]
        full_target = hover_target - reaction_frd
        force_only = allocate_bounded_wrench(config, force_only_target)
        allocation = allocate_bounded_wrench(config, full_target)
        commands = np.asarray([thrust_to_command(config, value) for value in allocation["thrust_n"]])
        force_only_commands = np.asarray([
            thrust_to_command(config, value) for value in force_only["thrust_n"]
        ])
        report_states[label] = {
            "preset": preset,
            "mass_kg": state.mass_kg,
            "center_of_mass_body_flu_m": state.center_of_mass_m.tolist(),
            "com_shift_norm_m": float(np.linalg.norm(state.com_shift_m)),
            "reaction_force_body_flu_n": state.reaction_force_body_n.tolist(),
            "reaction_torque_body_flu_nm": state.reaction_torque_body_nm.tolist(),
            "reaction_torque_norm_nm": float(np.linalg.norm(state.reaction_torque_body_nm)),
            "hover_target_wrench_frd": full_target.tolist(),
            "force_only_allocation_residual_norm": float(force_only["residual_norm"]),
            "force_only_hover_command_normalized": force_only_commands.tolist(),
            "force_only_saturated_high_count": int(np.count_nonzero(force_only["saturated_high"])),
            "torque_compensation_residual_increment": float(
                allocation["residual_norm"] - force_only["residual_norm"]
            ),
            "hover_thrust_n": allocation["thrust_n"].tolist(),
            "hover_command_normalized": commands.tolist(),
            "hover_command_mean": float(np.mean(commands)),
            "hover_command_max": float(np.max(commands)),
            "saturated_high_count": int(np.count_nonzero(allocation["saturated_high"])),
            "saturated_low_count": int(np.count_nonzero(allocation["saturated_low"])),
            "allocation_residual_norm": float(allocation["residual_norm"]),
        }
    report = {
        "schema": 1,
        "mass_baseline_kg": 7.735,
        "config": str(package / "config/my_drone_v3_cad_7p735_flight.json"),
        "interpretation": "Static allocation is a coupling comparison, not PX4/Gazebo flight acceptance.",
        "states": report_states,
    }
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(report, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    print("ARM_STATE_HOVER_COMPARISON_PASS " + json.dumps({
        key: {
            "mass_kg": value["mass_kg"],
            "hover_command_mean": value["hover_command_mean"],
            "hover_command_max": value["hover_command_max"],
            "reaction_torque_norm_nm": value["reaction_torque_norm_nm"],
            "force_only_residual_norm": value["force_only_allocation_residual_norm"],
            "allocation_residual_norm": value["allocation_residual_norm"],
        }
        for key, value in report_states.items()
    }, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
