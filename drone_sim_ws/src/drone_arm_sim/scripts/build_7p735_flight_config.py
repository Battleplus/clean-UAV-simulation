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
    config["rl_work_pose_rad"] = [0.4, -0.6, 0.8, -0.5, 0.3, 0.8]
    config["rl_trajectory_duration_s"] = 2.0
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
    thrust_points = config["static_thrust_model"]["points"]
    positive_points = [
        point for point in thrust_points
        if float(point.get("rated_capped_thrust_n", point["measured_thrust_n"])) > 0.0
    ]
    first_effective = positive_points[0]
    config["motor_dynamics_table"] = {
        "schema": 1,
        "input": "PX4 normalized_thrust in [0,1]; no RPM telemetry is available",
        "model_status": (
            "normalized static thrust and first-order actuator model; real k_f, "
            "C_T, C_Q and RPM response remain uncalibrated"
        ),
        "maximum_thrust_n": float(config["maximum_thrust_n"]),
        "minimum_effective_command": float(first_effective["throttle_percent"]) / 100.0,
        "minimum_effective_thrust_n": float(
            first_effective.get("rated_capped_thrust_n", first_effective["measured_thrust_n"])
        ),
        "reaction_torque_model": {
            "method": "Q = (Q/T) * T",
            "q_over_t_m": float(config["reaction_moment_ratio_m"]),
            "status": "temporary estimate; replace with measured C_Q/C_T or RPM-thrust-torque table",
        },
        "motors": [
            {
                "motor": int(rotor["motor"]),
                "px4_output": int(rotor["motor"]) - 1,
                "max_thrust_n": float(config["maximum_thrust_n"]),
                "min_effective_thrust_n": float(
                    first_effective.get("rated_capped_thrust_n", first_effective["measured_thrust_n"])
                ),
                "thrust_axis_frd": list(rotor["axis_body"]),
                "position_frd_m": list(rotor["position_m"]),
                "wrench_position_frd_m": list(rotor["wrench_position_m"]),
                "turning_direction": rotor["turning_direction"],
                "thrust_coefficient_kf": None,
                "thrust_coefficient_status": "unavailable_without RPM",
                "reaction_torque_coefficient_kq": None,
                "reaction_torque_coefficient_status": "represented only by temporary Q/T ratio",
                "rise_time_constant_s": float(config["motor_dynamics"]["rise_time_constant_s"]),
                "fall_time_constant_s": float(config["motor_dynamics"]["fall_time_constant_s"]),
            }
            for rotor in config["rotors"]
        ],
    }
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
    # PX4 PositionControl clamps its internal hover-thrust state to <=0.9,
    # while this canted 7.735 kg geometry physically needs 0.9304 of rated
    # rotor thrust.  Keep the physical endpoints exact and introduce a
    # piecewise-linear control normalization with a documented hover anchor.
    # This changes control resolution, not mass, geometry, or maximum thrust.
    px4_hover_command = 0.85
    physical_hover_thrust = float(np.mean(bounded.x))
    linear_thrust_per_command = physical_hover_thrust / px4_hover_command
    rated_command = float(config["maximum_thrust_n"]) / linear_thrust_per_command
    config["actuator_input_model"] = "hover_scaled_linear_thrust_with_rated_cap"
    config["actuator_normalization"] = {
        "px4_hover_command": px4_hover_command,
        "physical_hover_thrust_n": physical_hover_thrust,
        "physical_hover_fraction": physical_hover_thrust / float(config["maximum_thrust_n"]),
        "linear_thrust_per_command_n": linear_thrust_per_command,
        "rated_thrust_command": rated_command,
        "maximum_command": 1.0,
        "maximum_thrust_n": float(config["maximum_thrust_n"]),
        "status": (
            "control-interface calibration required because PX4 PositionControl "
            "limits its hover state to 0.9; physical thrust endpoints are unchanged"
        ),
    }
    config["actuator_input_model_status"] = (
        "single-slope PX4 command-to-thrust map through the calculated 7.735 kg "
        "hover force, capped at rated thrust above rated_thrust_command; this "
        "preserves allocator linearity around every unequal hover motor while the "
        "measured 14.8 V table is used only for equivalent ESC throttle/current"
    )
    config["takeoff_support_release"] = {
        "enabled": True,
        "model_name": "my_drone_bringup_landing_support",
        "release_up_force_n": float(config["estimated_all_up_mass_kg"]) * 9.80665 * 0.95,
        "maximum_horizontal_force_n": 0.50,
        "maximum_com_torque_nm": 0.05,
        "hold_time_s": 0.15,
        "restore_on_land": True,
        "restore_clearance_m": 0.05,
        "restore_sdf_filename": "../worlds/landing_support.sdf",
        "status": (
            "Four thin ground-level pads are released together after 95 percent "
            "of vehicle weight and a near-balanced COM wrench are held; the "
            "pads are restored near the recorded ground height for landing."
        ),
    }
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
