"""Generate a PX4 airframe from the authoritative CAD rotor config."""

from __future__ import annotations

import json
import argparse
from pathlib import Path

import numpy as np

from drone_arm_sim.allocation_analysis import allocation_matrix


HEADER = """#!/bin/sh
#
# @name Gazebo my_drone CAD canted octorotor
#
# @type Octocopter
# @class Copter
#
# Generated from the selected CAD flight configuration. Output 0..1000 is a
# normalized command mapped through the static-thrust table in Gazebo.

. ${R}etc/init.d/rc.mc_defaults

PX4_SIMULATOR=${PX4_SIMULATOR:=gz}
PX4_GZ_WORLD=${PX4_GZ_WORLD:=flight_world}
PX4_SIM_MODEL=${PX4_SIM_MODEL:=my_drone}

param set-default SIM_GZ_EN 1
param set-default CA_AIRFRAME 0
param set-default CA_ROTOR_COUNT 8
"""


def generate(config_path: Path, output: Path) -> None:
    config = json.loads(config_path.read_text(encoding="utf-8"))
    rotors = sorted(config["rotors"], key=lambda rotor: int(rotor["motor"]))
    matrix = allocation_matrix(config)
    mass = float(config["estimated_all_up_mass_kg"])
    desired = np.array([0.0, 0.0, -mass * 9.80665, 0.0, 0.0, 0.0])
    hover = np.linalg.pinv(matrix) @ desired
    hover_command = _thrust_to_command(config, float(np.mean(hover)))
    reaction_moment_ratio = config.get("reaction_moment_ratio_m")
    if reaction_moment_ratio is None:
        reaction_moment_ratio = 0.0
    lines = [HEADER.rstrip(), "", "# PX4 FRD; rotor index = motor number - 1."]
    for index, rotor in enumerate(rotors):
        position = rotor["position_m"]
        axis = rotor["axis_body"]
        lines.extend(
            [
                f"param set-default CA_ROTOR{index}_PX {position[0]:.9f}",
                f"param set-default CA_ROTOR{index}_PY {position[1]:.9f}",
                f"param set-default CA_ROTOR{index}_PZ {position[2]:.9f}",
                f"param set-default CA_ROTOR{index}_AX {axis[0]:.9f}",
                f"param set-default CA_ROTOR{index}_AY {axis[1]:.9f}",
                f"param set-default CA_ROTOR{index}_AZ {axis[2]:.9f}",
                f"param set-default CA_ROTOR{index}_KM "
                f"{float(rotor['direction']) * float(reaction_moment_ratio):.9f}",
                "",
            ]
        )
    for motor in range(1, 9):
        lines.append(f"param set-default SIM_GZ_EC_FUNC{motor} {100 + motor}")
    lines.append("")
    for motor in range(1, 9):
        lines.append(f"param set-default SIM_GZ_EC_MIN{motor} 0")
    for motor in range(1, 9):
        lines.append(f"param set-default SIM_GZ_EC_MAX{motor} 1000")
    lines.extend(
        [
            "",
            f"param set-default MPC_THR_HOVER {hover_command:.4f}",
            "param set-default CA_METHOD 0",
            "param set-default NAV_DLL_ACT 0",
            "param set SENS_IMU_MODE 0",
            "param set EKF2_MULTI_IMU 1",
            "param set EKF2_MAG_TYPE 0",
            "",
        ]
    )
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text("\n".join(lines), encoding="utf-8", newline="\n")
    print(output)
    print(f"hover command={hover_command:.6f}")


def _thrust_to_command(config: dict, thrust_n: float) -> float:
    """Invert the configured static thrust curve for a normalized command."""
    model = config.get("static_thrust_model")
    points = model.get("points", []) if isinstance(model, dict) else []
    if not points:
        maximum = float(config.get("maximum_thrust_n", 0.0))
        return float(np.clip(thrust_n / maximum, 0.0, 1.0)) if maximum > 0.0 else 0.0
    throttle = np.asarray([float(point["throttle_percent"]) for point in points])
    thrust = np.asarray([
        float(point.get("rated_capped_thrust_n", point["measured_thrust_n"]))
        for point in points
    ])
    order = np.argsort(thrust, kind="stable")
    thrust = thrust[order]
    throttle = throttle[order]
    unique_thrust, unique_indices = np.unique(thrust, return_index=True)
    throttle = throttle[unique_indices]
    return float(np.clip(np.interp(thrust_n, unique_thrust, throttle) / 100.0, 0.0, 1.0))


def main() -> None:
    package = Path(__file__).resolve().parents[1]
    workspace = package.parents[1]
    parser = argparse.ArgumentParser()
    parser.add_argument("--config", type=Path, default=package / "config" / "my_drone_v2_cad.json")
    parser.add_argument("--output", type=Path, default=workspace / "px4" / "airframes" / "4015_gz_my_drone_octorotor")
    args = parser.parse_args()
    generate(
        args.config,
        args.output,
    )


if __name__ == "__main__":
    main()
