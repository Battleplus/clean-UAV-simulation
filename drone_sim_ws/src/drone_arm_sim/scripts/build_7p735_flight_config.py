"""Build the 7.735 kg opposite-pitch formal flight-test configuration."""

from __future__ import annotations

import json
from pathlib import Path

import numpy as np
from scipy.optimize import lsq_linear

from drone_arm_sim.allocation_analysis import allocation_matrix


def main() -> None:
    package = Path(__file__).resolve().parents[1]
    prototype = json.loads(
        (package / "config/my_drone_v2_cad_flight_pitch_corrected.json")
        .read_text(encoding="utf-8")
    )
    physical = json.loads(
        (package / "config/my_drone_v3_cad_physical.json").read_text(encoding="utf-8")
    )
    scenario = physical["allocation_results"][1]
    if not scenario["hover_feasible_at_rated_1p2_kgf"]:
        raise ValueError("The 7.735 kg opposite-pitch scenario is not hover-feasible")
    axes = {
        int(rotor["motor"]): rotor
        for rotor in physical["scenarios"]["opposite_pitch_all_up_hypothesis"]
    }
    config = dict(prototype)
    config["description"] = (
        "Temporary 7.735 kg whole-aircraft flight-test scenario. CAD shaft geometry "
        "and COM-relative PX4 allocation are retained; motors 4/5/7/8 require "
        "opposite-pitch propellers because pitch sign is unresolved in the flat CAD."
    )
    config["scenario"] = "temporary_7p735kg_opposite_pitch_ideal_14p8v_supply"
    config["estimated_all_up_mass_kg"] = float(physical["estimated_mass_kg"])
    config["temporary_fixed_mass_kg"] = float(physical["estimated_mass_kg"])
    config["mass_override_source"] = physical["mass_override_source"]
    config["formal_urdf"] = "urdf/my_drone_v3/my_drone_cad_formal_dynamic.urdf"
    config["allocation_reference"] = "vehicle COM in PX4 FRD"
    config["wrench_reference"] = "URDF base_link origin in PX4 FRD"
    config["rotors"] = []
    for prototype_rotor in prototype["rotors"]:
        motor = int(prototype_rotor["motor"])
        rotor = dict(prototype_rotor)
        rotor["wrench_position_m"] = list(prototype_rotor["position_m"])
        rotor["position_m"] = list(axes[motor]["position_m"])
        rotor["axis_body"] = list(axes[motor]["axis_body"])
        rotor["thrust_sign_source"] = axes[motor]["thrust_sign_source"]
        config["rotors"].append(rotor)
    config["maximum_vertical_force_n"] = scenario["maximum_upward_vertical_force_n"]
    config["maximum_supported_mass_kg"] = scenario["maximum_supported_mass_kg"]
    config["estimated_vertical_thrust_to_weight"] = scenario["maximum_vertical_thrust_to_weight"]
    hover_target = np.array(
        [0.0, 0.0, -config["estimated_all_up_mass_kg"] * 9.80665, 0.0, 0.0, 0.0]
    )
    matrix = allocation_matrix(config)
    bounded = lsq_linear(
        matrix,
        hover_target,
        bounds=(0.0, float(config["maximum_thrust_n"])),
        lsmr_tol="auto",
    )
    residual = matrix @ bounded.x - hover_target
    if not bounded.success or np.linalg.norm(residual) > 1e-8:
        raise ValueError("The complete flight allocation cannot trim hover")
    config["bounded_hover_thrust_n"] = bounded.x.tolist()
    config["bounded_hover_residual"] = residual.tolist()
    config["bounded_hover_residual_norm"] = float(np.linalg.norm(residual))
    config["flight_feasibility_nonreversible"] = "FEASIBLE_WITH_OPPOSITE_PITCH_HYPOTHESIS"
    config["battery_dynamics"] = dict(prototype["battery_dynamics"])
    config["battery_dynamics"]["enabled"] = False
    config["battery_dynamics"]["status"] = (
        "disabled only for the first 7.735 kg ideal-14.8V feasibility flight; "
        "the current provisional 5 mOhm sag estimate removes hover margin"
    )
    output = package / "config/my_drone_v3_cad_7p735_flight.json"
    output.write_text(json.dumps(config, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    print(output)
    print(
        f"mass={config['estimated_all_up_mass_kg']:.3f} kg "
        f"T/W={config['estimated_vertical_thrust_to_weight']:.6f} "
        f"max_motor={max(config['bounded_hover_thrust_n']):.6f} N"
    )


if __name__ == "__main__":
    main()
