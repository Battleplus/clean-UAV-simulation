#!/usr/bin/env python3
"""Recompute COM-relative rotor allocation for a disposable payload URDF."""

from __future__ import annotations

import argparse
import json
from pathlib import Path
import re
import sys

import numpy as np

from drone_arm_sim.allocation_analysis import allocate_bounded_wrench
from drone_arm_sim.model_analysis import UrdfModel

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src/drone_arm_sim/scripts"))
from generate_cad_px4_airframe import generate as generate_airframe


FLU_TO_FRD = np.diag([1.0, -1.0, -1.0])


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--base-config", type=Path, required=True)
    parser.add_argument("--payload-urdf", type=Path, required=True)
    parser.add_argument("--output-config", type=Path, required=True)
    parser.add_argument("--output-airframe", type=Path, required=True)
    parser.add_argument("--hover-command-bias", type=float, default=0.0)
    parser.add_argument(
        "--px4-param", action="append", default=[], metavar="NAME=VALUE",
        help="replace an existing param set-default line in the generated airframe",
    )
    args = parser.parse_args()

    config = json.loads(args.base_config.read_text(encoding="utf-8"))
    mass, center_flu, inertia_flu = UrdfModel(args.payload_urdf).mass_properties({})
    center_frd = FLU_TO_FRD @ center_flu
    inertia_frd = FLU_TO_FRD @ inertia_flu @ FLU_TO_FRD

    for rotor in config["rotors"]:
        base_position = np.asarray(rotor["wrench_position_m"], dtype=float)
        rotor["position_m"] = (base_position - center_frd).tolist()

    motor_table = config.get("motor_dynamics_table", {}).get("motors", [])
    rotor_by_motor = {int(item["motor"]): item for item in config["rotors"]}
    for record in motor_table:
        rotor = rotor_by_motor[int(record["motor"])]
        record["position_frd_m"] = list(rotor["position_m"])
        record["wrench_position_frd_m"] = list(rotor["wrench_position_m"])

    config["estimated_all_up_mass_kg"] = float(mass)
    config["temporary_fixed_mass_kg"] = float(mass)
    config["mass_override_source"] = (
        f"aggregate inertials from disposable payload URDF {args.payload_urdf}"
    )
    config["formal_urdf"] = str(args.payload_urdf)
    config["description"] = (
        "Disposable payload flight configuration derived from the formal CAD "
        "model; motor thrust and uncalibrated Q/T assumptions are unchanged."
    )

    desired = np.array([0.0, 0.0, -mass * 9.80665, 0.0, 0.0, 0.0])
    allocation = allocate_bounded_wrench(config, desired)
    hover = np.asarray(allocation["thrust_n"], dtype=float)
    residual = np.asarray(allocation["residual"], dtype=float)
    config["bounded_hover_thrust_n"] = hover.tolist()
    config["bounded_hover_residual"] = residual.tolist()
    config["bounded_hover_residual_norm"] = float(np.linalg.norm(residual))

    normalization = config["actuator_normalization"]
    gain = float(normalization["linear_thrust_per_command_n"])
    average_hover = float(np.mean(hover))
    unbiased_hover_command = average_hover / gain
    normalization["px4_hover_command"] = float(np.clip(
        unbiased_hover_command + args.hover_command_bias, 0.0, 0.9
    ))
    normalization["physical_hover_thrust_n"] = average_hover
    normalization["physical_hover_fraction"] = (
        average_hover / float(config["maximum_thrust_n"])
    )
    normalization["status"] = (
        "payload-specific COM allocation and hover command; the physical "
        "command-to-thrust gain remains the unloaded calibrated interface"
    )
    config["actuator_input_model_status"] = (
        "payload-specific hover point with unchanged single-slope physical "
        "command-to-thrust gain and rated motor cap"
    )
    release = config.get("takeoff_support_release", {})
    release["release_up_force_n"] = 0.95 * mass * 9.80665
    restore = release.get("restore_sdf_filename")
    if restore and not Path(restore).is_absolute():
        release["restore_sdf_filename"] = str(
            (args.base_config.parent / restore).resolve()
        )

    config["payload_flight_override"] = {
        "source_urdf": str(args.payload_urdf),
        "aggregate_mass_kg": float(mass),
        "center_of_mass_flu_m": center_flu.tolist(),
        "center_of_mass_frd_m": center_frd.tolist(),
        "inertia_at_com_frd_kg_m2": inertia_frd.tolist(),
        "hover_command": float(normalization["px4_hover_command"]),
        "unbiased_hover_command": float(unbiased_hover_command),
        "hover_command_bias": float(args.hover_command_bias),
        "allocation_residual_norm": float(np.linalg.norm(residual)),
        "status": "temporary payload-specific flight calibration",
        "px4_parameter_overrides": list(args.px4_param),
    }

    args.output_config.parent.mkdir(parents=True, exist_ok=True)
    args.output_config.write_text(
        json.dumps(config, ensure_ascii=False, indent=2) + "\n", encoding="utf-8"
    )
    generate_airframe(args.output_config, args.output_airframe)
    airframe_text = args.output_airframe.read_text(encoding="utf-8")
    for assignment in args.px4_param:
        if "=" not in assignment:
            raise ValueError(f"invalid --px4-param {assignment!r}; expected NAME=VALUE")
        name, value = assignment.split("=", 1)
        pattern = rf"(?m)^param set-default {re.escape(name)}\s+\S+\s*$"
        replacement = f"param set-default {name} {value}"
        airframe_text, count = re.subn(pattern, replacement, airframe_text)
        if count != 1:
            raise ValueError(
                f"PX4 parameter {name!r} was not present exactly once in generated airframe"
            )
    args.output_airframe.write_text(airframe_text, encoding="utf-8")
    print(
        "PAYLOAD_FLIGHT_CONFIG_READY "
        f"mass_kg={mass:.9f} "
        f"com_frd_m={center_frd.tolist()} "
        f"hover_command={normalization['px4_hover_command']:.9f} "
        f"residual_norm={np.linalg.norm(residual):.9g}"
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
