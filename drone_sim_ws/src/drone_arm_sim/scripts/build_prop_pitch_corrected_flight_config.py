#!/usr/bin/env python3
"""Create the explicit all-up flight hypothesis used for controller debugging.

The CAD contains no signed blade pitch.  This script must therefore preserve
the distinction between a flyable numerical hypothesis and physical evidence.
"""

from __future__ import annotations

import json
from pathlib import Path


PACKAGE = Path(__file__).resolve().parents[1]
SOURCE = PACKAGE / "config" / "my_drone_v2_cad.json"
OUTPUT = PACKAGE / "config" / "my_drone_v2_cad_flight_pitch_corrected.json"
OPPOSITE_PITCH_REQUIRED_IF_ALL_UP = {4, 5, 7, 8}


def main() -> None:
    config = json.loads(SOURCE.read_text(encoding="utf-8"))
    config["description"] = (
        "All-up controller-debug hypothesis derived from CAD positions and "
        "signless shaft lines. Positive thrust direction and opposite-pitch "
        "propellers on motors 4, 5, 7 and 8 remain unverified."
    )
    config["scenario"] = "all_up_axis_hypothesis_4_5_7_8_pitch_unverified"
    config["authoritative_source"] = "config/my_drone_v2_cad.json"
    config["axis_assumption"] = "ALL_UP_HYPOTHESIS_NOT_PHYSICAL_FACT"
    config["positive_thrust_direction_status"] = "UNRESOLVED_FOR_ALL_MOTORS"
    config.pop("upward_thrust_motors", None)
    config.pop("downward_thrust_motors", None)
    body_frame = config.get("body_frame_frozen", {})
    body_frame.pop("upward_thrust_motors", None)
    body_frame.pop("downward_thrust_motors", None)
    body_frame["positive_thrust_direction_status"] = "UNRESOLVED_FOR_ALL_MOTORS"
    for rotor in config["rotors"]:
        motor = int(rotor["motor"])
        if motor in OPPOSITE_PITCH_REQUIRED_IF_ALL_UP:
            rotor["axis_body"] = [-float(value) for value in rotor["axis_body"]]
        rotor["axis_role"] = "ALL_UP_HYPOTHESIS_NOT_PHYSICAL_FACT"
        rotor["thrust_sign_status"] = (
            "UNRESOLVED_PROP_PITCH_OR_SIGNED_TEST_REQUIRED"
        )
        rotor["vertical_thrust_direction"] = "upward_in_debug_hypothesis"
        rotor["opposite_pitch_required_if_all_up"] = (
            motor in OPPOSITE_PITCH_REQUIRED_IF_ALL_UP
        )
    maximum = float(config["maximum_thrust_n"])
    vertical = maximum * sum(-float(r["axis_body"][2]) for r in config["rotors"])
    mass = float(config["estimated_all_up_mass_kg"])
    config["maximum_vertical_force_n"] = vertical
    config["maximum_supported_mass_kg"] = vertical / 9.80665
    config["estimated_vertical_thrust_to_weight"] = vertical / (mass * 9.80665)
    config["flight_feasibility_nonreversible"] = "UNRESOLVED_PROP_PITCH"
    config["debug_hypothesis_vertical_capacity"] = "FEASIBLE"
    config["reaction_moment_ratio_m"] = 0.001
    config["reaction_moment_estimate"] = {
        "formula": "Q = (Q/T) * T",
        "q_over_t_m": 0.001,
        "status": "initial formal estimate; not a measured propeller parameter",
        "validation": "the isolated 4 kg debug generator overrides this to its separately tested 0.005 m value",
        "calibration_required": "replace with measured RPM-thrust-torque data or C_Q/C_T and propeller diameter",
        "not_inherited_from_legacy_example": True,
    }
    config["environment_dynamics"] = {
        "air_density_kg_m3": 1.225,
        "quadratic_drag_area_cd_m2_body_flu": [0.10, 0.10, 0.18],
        "angular_damping_n_m_per_rad_s_body_flu": [0.025, 0.025, 0.040],
        "wind_velocity_world_enu_m_s": [0.0, 0.0, 0.0],
        "ground_effect": {
            "enabled": True,
            "maximum_thrust_gain": 0.08,
            "decay_height_m": 0.25,
            "status": "initial bounded estimate; replace after propeller diameter and test data are available",
        },
        "status": "initial estimates for runtime testing; drag and ground-effect coefficients require identification",
    }
    config["battery_dynamics"] = {
        "enabled": True,
        "reference_voltage_v": 14.8,
        "full_voltage_v": 14.8,
        "empty_voltage_v": 13.2,
        "minimum_loaded_voltage_v": 12.0,
        "capacity_ah": 10.0,
        "pack_internal_resistance_ohm": 0.005,
        "thrust_voltage_exponent": 2.0,
        "status": "capacity and pack resistance are initial estimates; replace with the actual battery specification and load test",
    }
    config["actuator_transport_delay_s"] = 0.008
    config["actuator_transport_delay_status"] = (
        "initial two-physics-step estimate; replace with measured PX4-output-to-ESC response latency"
    )
    OUTPUT.write_text(json.dumps(config, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    print(OUTPUT)
    print(f"vertical_force={vertical:.6f} N, T/W={config['estimated_vertical_thrust_to_weight']:.6f}")


if __name__ == "__main__":
    main()
