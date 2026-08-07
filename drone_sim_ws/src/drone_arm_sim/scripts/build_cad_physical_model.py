"""Build the formal CAD/physical configuration and allocation evidence."""

from __future__ import annotations

import json
import math
from pathlib import Path

import numpy as np
from scipy.optimize import lsq_linear


CW_MOTORS = {1, 2, 7, 8}
CCW_MOTORS = {3, 4, 5, 6}
USER_UPWARD_MOTORS = {1, 2, 3, 6}
MAX_THRUST_N = 1.2 * 9.80665
GRAVITY = 9.80665
TEMPORARY_FIXED_MASS_KG = 7.735
THRUST_TABLE_GRAMS = {
    0: 0, 15: 26, 20: 79, 25: 142, 30: 160, 35: 254, 40: 372,
    45: 457, 50: 545, 55: 562, 60: 630, 65: 673, 70: 725,
    75: 831, 80: 909, 85: 1041, 90: 1283,
}


def cad_to_frd(vector):
    x, y, z = np.asarray(vector, dtype=float)
    return np.asarray([-z, x, -y])


def unit(vector):
    vector = np.asarray(vector, dtype=float)
    return vector / np.linalg.norm(vector)


def allocation(rotors, moment_ratio=0.0):
    columns = []
    for rotor in rotors:
        position = np.asarray(rotor["position_m"], dtype=float)
        axis = unit(rotor["axis_body"])
        spin = float(rotor["direction"])
        moment = np.cross(position, axis) - spin * moment_ratio * axis
        columns.append(np.concatenate((axis, moment)))
    return np.column_stack(columns)


def analyse_scenario(name, rotors, mass):
    matrix = allocation(rotors)
    singular = np.linalg.svd(matrix, compute_uv=False)
    rank = int(np.linalg.matrix_rank(matrix, tol=1e-9))
    condition = float(singular[0] / singular[-1]) if singular[-1] > 1e-12 else math.inf
    target = np.asarray([0.0, 0.0, -mass * GRAVITY, 0.0, 0.0, 0.0])
    unconstrained = np.linalg.pinv(matrix) @ target
    bounded = lsq_linear(matrix, target, bounds=(0.0, MAX_THRUST_N), lsmr_tol="auto")
    max_vertical = MAX_THRUST_N * sum(max(0.0, -float(r["axis_body"][2])) for r in rotors)
    return {
        "name": name,
        "allocation_matrix_6x8": matrix.tolist(),
        "rank": rank,
        "singular_values": singular.tolist(),
        "condition_number": condition,
        "hover_target_wrench": target.tolist(),
        "unconstrained_hover_thrust_n": unconstrained.tolist(),
        "unconstrained_all_nonnegative": bool(np.all(unconstrained >= -1e-8)),
        "unconstrained_within_rated_max": bool(np.all(unconstrained <= MAX_THRUST_N + 1e-8)),
        "bounded_hover_thrust_n": bounded.x.tolist(),
        "bounded_hover_residual": (matrix @ bounded.x - target).tolist(),
        "bounded_hover_residual_norm": float(np.linalg.norm(matrix @ bounded.x - target)),
        "maximum_upward_vertical_force_n": max_vertical,
        "maximum_supported_mass_kg": max_vertical / GRAVITY,
        "maximum_vertical_thrust_to_weight": max_vertical / (mass * GRAVITY),
        "hover_feasible_at_rated_1p2_kgf": bool(
            bounded.cost < 1e-8
            and np.all(bounded.x <= MAX_THRUST_N + 1e-8)
            and np.all(bounded.x >= -1e-8)
        ),
    }


def main() -> None:
    project = Path(__file__).resolve().parents[4]
    analysis = project / "drone_sim_ws" / "analysis" / "cad_direct"
    config_dir = project / "drone_sim_ws" / "src" / "drone_arm_sim" / "config"
    axes = json.loads((analysis / "motor_axis_evidence.json").read_text(encoding="utf-8"))
    pitch = json.loads((analysis / "propeller_pitch_geometry_evidence.json").read_text(encoding="utf-8"))
    physical = json.loads((analysis / "cad_mass_properties.json").read_text(encoding="utf-8"))
    cad_com = np.asarray(physical["estimated_cad_com_m"], dtype=float)
    rotors_unsigned = []
    declared_rotors = []
    all_up_rotors = []
    for item in sorted(axes["motors"], key=lambda x: x["motor"]):
        motor = item["motor"]
        shaft_cad = np.asarray(item["assembly_axis_point_mm"], dtype=float) / 1000.0
        position = cad_to_frd(shaft_cad - cad_com)
        prop_side = unit(item["cad_propeller_side_axis_frd"])
        declared_axis = prop_side.copy()
        wants_up = motor in USER_UPWARD_MOTORS
        if (declared_axis[2] < 0.0) != wants_up:
            declared_axis *= -1.0
        all_up_axis = prop_side.copy()
        if all_up_axis[2] > 0.0:
            all_up_axis *= -1.0
        common = {
            "motor": motor,
            "position_m": position.tolist(),
            "turning_direction": "CW" if motor in CW_MOTORS else "CCW",
            "direction": -1 if motor in CW_MOTORS else 1,
            "propulsion_mode": item["propulsion_mode"],
            "cad_shaft_axis_line_frd": prop_side.tolist(),
            "cad_axis_point_mm": item["assembly_axis_point_mm"],
            "motor_propeller_line_offset_mm": item["motor_propeller_line_offset_mm"],
            "motor_propeller_axis_angle_deg": item["motor_propeller_axis_angle_deg"],
            "pitch_geometry_status": pitch["result"],
        }
        rotors_unsigned.append(common)
        declared_rotors.append(
            common | {
                "axis_body": declared_axis.tolist(),
                "thrust_sign_source": "user-declared CW/CCW + puller/pusher table; not resolved by flat CAD propeller geometry",
            }
        )
        all_up_rotors.append(
            common | {
                "axis_body": all_up_axis.tolist(),
                "thrust_sign_source": "explicit opposite-pitch flight hypothesis; not present in current CAD propeller geometry",
            }
        )
    cad_density_mass = float(physical["estimated_total_mass_kg"])
    mass = TEMPORARY_FIXED_MASS_KG
    inertia_scale = mass / cad_density_mass
    working_inertia = (
        np.asarray(
            physical["estimated_ros_flu_inertia_at_com_kg_m2"], dtype=float
        ) * inertia_scale
    )
    scenarios = [
        analyse_scenario("user_declared_installed_propellers", declared_rotors, mass),
        analyse_scenario("opposite_pitch_all_up_hypothesis", all_up_rotors, mass),
    ]
    thrust_curve = [
        {
            "throttle_percent": throttle,
            "measured_thrust_gf": grams,
            "measured_thrust_n": grams / 1000.0 * GRAVITY,
            "rated_capped_thrust_n": min(grams / 1000.0 * GRAVITY, MAX_THRUST_N),
        }
        for throttle, grams in THRUST_TABLE_GRAMS.items()
    ]
    payload = {
        "schema": 3,
        "description": "Formal CAD physical baseline. It is intentionally separate from the lower-mass flight prototype.",
        "source_assembly": str(project / "零件" / "完整零件" / "组合无人机.SLDASM"),
        "geometry_evidence": "analysis/cad_direct/motor_axis_evidence.json",
        "pitch_evidence": "analysis/cad_direct/propeller_pitch_geometry_evidence.json",
        "physical_evidence": "analysis/cad_direct/cad_mass_properties.json",
        "coordinate_frames": {
            "cad_assembly": "immutable SolidWorks assembly origin",
            "ros_flu": "+X nose, +Y left, +Z up; origin at provisional CAD-derived COM",
            "px4_frd": "+X nose, +Y right, +Z down; origin at provisional CAD-derived COM",
            "cad_to_ros_flu": "X=-CAD_Z, Y=-CAD_X, Z=CAD_Y",
            "cad_to_px4_frd": "X=-CAD_Z, Y=CAD_X, Z=-CAD_Y",
        },
        "estimated_mass_kg": mass,
        "cad_density_estimated_mass_kg": cad_density_mass,
        "temporary_fixed_mass_kg": mass,
        "mass_override_source": "user fixed the temporary whole-aircraft mass at 7.735 kg on 2026-08-07",
        "inertia_scaling_from_cad_density_estimate": inertia_scale,
        "estimated_cad_com_m": physical["estimated_cad_com_m"],
        "estimated_ros_flu_inertia_at_com_kg_m2": working_inertia.tolist(),
        "cad_density_estimated_ros_flu_inertia_at_com_kg_m2": physical["estimated_ros_flu_inertia_at_com_kg_m2"],
        "mass_status": "temporary user-fixed whole-aircraft mass; CAD COM retained and CAD inertia scaled uniformly by mass ratio until measured inertia is available",
        "maximum_rated_thrust_per_motor_n": MAX_THRUST_N,
        "static_thrust_model": {
            "method": "piecewise-linear interpolation of supplied 14.8 V static test table",
            "rated_cap_n": MAX_THRUST_N,
            "note": "90% measured point is 1.283 kgf but output is capped at the user-rated 1.2 kgf until test conditions are reconciled",
            "points": thrust_curve,
        },
        "motor_dynamics": {
            "rise_time_constant_s": 0.035,
            "fall_time_constant_s": 0.035,
            "status": "initial estimate only; must be replaced by measured step response",
        },
        "reaction_torque": {
            "modelled_in_allocation_report": False,
            "reason": "no RPM-torque or C_Q evidence; zero used for geometry authority audit",
        },
        "rotor_axis_lines_unsigned": rotors_unsigned,
        "scenarios": {
            "user_declared_installed_propellers": declared_rotors,
            "opposite_pitch_all_up_hypothesis": all_up_rotors,
        },
        "allocation_results": scenarios,
    }
    config_path = config_dir / "my_drone_v3_cad_physical.json"
    report_path = analysis / "cad_physical_allocation_report.json"
    config_path.write_text(json.dumps(payload, ensure_ascii=False, indent=2), encoding="utf-8")
    report_path.write_text(json.dumps(scenarios, ensure_ascii=False, indent=2), encoding="utf-8")
    print(config_path)
    for result in scenarios:
        print(
            result["name"],
            "rank", result["rank"],
            "condition", result["condition_number"],
            "T/W", result["maximum_vertical_thrust_to_weight"],
            "hover", result["hover_feasible_at_rated_1p2_kgf"],
            "residual", result["bounded_hover_residual_norm"],
        )


if __name__ == "__main__":
    main()
