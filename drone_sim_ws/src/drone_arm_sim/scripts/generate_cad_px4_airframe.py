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
    # The bridge anchors the calculated physical hover force below PX4's 0.9
    # internal hover-state ceiling while preserving u=0 -> 0 N and u=1 -> the
    # rated maximum thrust.  It is a control-interface calibration, not a
    # change to the motor's physical force limit.
    hover_command = float(config["actuator_normalization"]["px4_hover_command"])
    reaction_moment_ratio = config.get("reaction_moment_ratio_m")
    if reaction_moment_ratio is None:
        reaction_moment_ratio = 0.0
    lines = [HEADER.rstrip(), "", "# PX4 FRD; rotor index = motor number - 1."]
    for index, rotor in enumerate(rotors):
        position = rotor["position_m"]
        # Prefer a signed propeller thrust axis when pitch-accurate CAD or
        # manufacturer data are available; the current formal config falls
        # back to the CAD shaft axis under its explicit pitch hypothesis.
        axis = rotor.get("thrust_axis_body", rotor["axis_body"])
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
            # The formal CAD vehicle has only about 7.5% vertical thrust
            # margin at 7.735 kg.  Keep PX4's position loop inside that
            # envelope during the first takeoff instead of allowing the
            # default 3 m/s climb target and 45 deg tilt limit to consume all
            # available thrust.  These are bring-up defaults, not a substitute
            # for flight-log tuning.
            "param set-default MPC_ALT_MODE 0",
            "param set-default MPC_XY_P 0.15",
            "param set-default MPC_XY_VEL_P_ACC 0.90",
            "param set-default MPC_XY_VEL_I_ACC 0.00",
            "param set-default MPC_XY_VEL_D_ACC 0.40",
            "param set-default MPC_XY_VEL_MAX 0.20",
            "param set-default MPC_Z_P 0.15",
            "param set-default MPC_Z_VEL_P_ACC 2.00",
            "param set-default MPC_Z_VEL_I_ACC 0.80",
            "param set-default MPC_TKO_RAMP_T 0.80",
            "param set-default MPC_Z_VEL_MAX_UP 0.20",
            "param set-default MPC_Z_VEL_MAX_DN 0.30",
            "param set-default MPC_TILTMAX_AIR 10",
            "param set-default MPC_TILTMAX_LND 5",
            "param set-default MPC_THR_MIN 0.50",
            "param set-default MPC_THR_MAX 1.00",
            # ULog identification on the formal 0.40 kg m^2 airframe showed
            # the stock multicopter gains driving a 1.5 s pitch limit cycle:
            # pitch reached 29 deg and motor allocation repeatedly saturated
            # before horizontal position diverged.  These conservative
            # bring-up gains cap the demanded body rates and reduce integral
            # buildup while retaining derivative damping.  They must be
            # replaced by measured/system-identified gains before claiming
            # controller fidelity to the physical aircraft.
            "param set-default MC_ROLL_P 2.00",
            "param set-default MC_PITCH_P 2.00",
            "param set-default MC_YAW_P 1.50",
            "param set-default MC_ROLLRATE_MAX 70.0",
            "param set-default MC_PITCHRATE_MAX 70.0",
            "param set-default MC_YAWRATE_MAX 60.0",
            "param set-default MC_ROLLRATE_P 0.080",
            "param set-default MC_PITCHRATE_P 0.080",
            "param set-default MC_YAWRATE_P 0.120",
            "param set-default MC_ROLLRATE_I 0.050",
            "param set-default MC_PITCHRATE_I 0.050",
            "param set-default MC_YAWRATE_I 0.050",
            "param set-default MC_ROLLRATE_D 0.003",
            "param set-default MC_PITCHRATE_D 0.003",
            # With only 7.5% ideal vertical margin, a hover-thrust estimate
            # biased by brief support contact can make takeoff impossible.
            # Keep the estimator close to the calculated 7.735 kg trim until
            # real in-air data can replace the provisional mass/thrust model.
            "param set-default HTE_THR_RANGE 0.01",
            "param set-default HTE_HT_ERR_INIT 0.00",
            "param set-default HTE_HT_NOISE 0.0001",
            "param set-default MC_AIRMODE 1",
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
