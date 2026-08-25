#!/usr/bin/env python3
"""Build an isolated 1.3 kg candidate without modifying Base 1 / 4 kg.

The user-provided approximate mass split is treated as the current source of
truth: 0.6 kg for the complete SO101 arm and 0.7 kg for everything else.
Within each group, the existing CAD-volume mass and inertia ratios are kept.
Geometry, joint frames, meshes and rotor shaft axes are not changed.
"""

from __future__ import annotations

import copy
import json
import re
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np


WORKSPACE = Path(__file__).resolve().parents[1]
PACKAGE = WORKSPACE / "src/drone_arm_sim"
sys.path.insert(0, str(PACKAGE))

from drone_arm_sim.allocation_analysis import allocate_bounded_wrench  # noqa: E402
from drone_arm_sim.model_analysis import UrdfModel  # noqa: E402


TOTAL_MASS_KG = 1.3
ARM_MASS_KG = 0.6
AIRFRAME_MASS_KG = TOTAL_MASS_KG - ARM_MASS_KG
GRAVITY_M_S2 = 9.80665
FLU_TO_FRD = np.diag([1.0, -1.0, -1.0])

# Base 1's rate loop was identified around a much larger inertia.  These
# candidate-local initial values scale it down conservatively for the 1.3 kg
# model; Base 1 stays untouched and dynamic acceptance is still required.
CANDIDATE_RATE_GAINS = {
    "MC_ROLLRATE_P": 0.030,
    "MC_PITCHRATE_P": 0.030,
    "MC_YAWRATE_P": 0.040,
    "MC_ROLLRATE_I": 0.018,
    "MC_PITCHRATE_I": 0.018,
    "MC_YAWRATE_I": 0.010,
    "MC_ROLLRATE_D": 0.0009,
    "MC_PITCHRATE_D": 0.0009,
}

ARM_LINKS = {
    "arm_base_link",
    "shoulder_link",
    "upper_arm_link",
    "lower_arm_link",
    "wrist_link",
    "gripper_link",
    "moving_jaw_link",
}


def _scale_link_inertial(link: ET.Element, scale: float) -> float:
    inertial = link.find("inertial")
    if inertial is None:
        return 0.0
    mass = inertial.find("mass")
    inertia = inertial.find("inertia")
    if mass is None or inertia is None:
        return 0.0
    value = float(mass.get("value")) * scale
    mass.set("value", f"{value:.12g}")
    for key in ("ixx", "ixy", "ixz", "iyy", "iyz", "izz"):
        inertia.set(key, f"{float(inertia.get(key)) * scale:.12g}")
    return value


def build_urdf(source: Path, output: Path) -> dict:
    tree = ET.parse(source)
    root = tree.getroot()
    root.set("name", "my_drone_v3_cad_candidate_1p3kg")
    links = root.findall("link")
    source_arm = sum(
        float(link.find("inertial/mass").get("value"))
        for link in links
        if link.get("name") in ARM_LINKS and link.find("inertial/mass") is not None
    )
    source_airframe = sum(
        float(link.find("inertial/mass").get("value"))
        for link in links
        if link.get("name") not in ARM_LINKS and link.find("inertial/mass") is not None
    )
    if source_arm <= 0.0 or source_airframe <= 0.0:
        raise ValueError("invalid arm/airframe source mass partition")
    arm_scale = ARM_MASS_KG / source_arm
    airframe_scale = AIRFRAME_MASS_KG / source_airframe
    arm_total = 0.0
    airframe_total = 0.0
    for link in links:
        if link.get("name") in ARM_LINKS:
            arm_total += _scale_link_inertial(link, arm_scale)
        else:
            airframe_total += _scale_link_inertial(link, airframe_scale)
    root.insert(
        0,
        ET.Comment(
            " 1.3 kg MASS CANDIDATE: user estimate, not per-part scale data. "
            "SO101 links total 0.6 kg; remaining aircraft links total 0.7 kg. "
            "Base 1 and the 4 kg debug URDF are unchanged. "
        ),
    )
    output.parent.mkdir(parents=True, exist_ok=True)
    ET.indent(tree, space="  ")
    tree.write(output, encoding="utf-8", xml_declaration=True)
    if not np.isclose(arm_total, ARM_MASS_KG, atol=1.0e-10, rtol=0.0):
        raise AssertionError((arm_total, ARM_MASS_KG))
    if not np.isclose(airframe_total, AIRFRAME_MASS_KG, atol=1.0e-10, rtol=0.0):
        raise AssertionError((airframe_total, AIRFRAME_MASS_KG))
    return {
        "source_arm_mass_kg": source_arm,
        "source_airframe_mass_kg": source_airframe,
        "arm_scale": arm_scale,
        "airframe_scale": airframe_scale,
        "arm_mass_kg": arm_total,
        "airframe_mass_kg": airframe_total,
        "total_mass_kg": arm_total + airframe_total,
    }


def _folded_positions(reference_path: Path) -> dict[str, float]:
    reference = json.loads(reference_path.read_text(encoding="utf-8"))
    names = [row["name"] for row in reference["joints"]]
    return dict(zip(names, reference["presets"]["retracted"]))


def build_config(source: Path, urdf: Path, reference: Path, output: Path) -> dict:
    config = copy.deepcopy(json.loads(source.read_text(encoding="utf-8")))
    folded = _folded_positions(reference)
    mass, center_flu, inertia_flu = UrdfModel(urdf).mass_properties(folded)
    if not np.isclose(mass, TOTAL_MASS_KG, atol=1.0e-9, rtol=0.0):
        raise AssertionError((mass, TOTAL_MASS_KG))
    center_frd = FLU_TO_FRD @ center_flu
    inertia_frd = FLU_TO_FRD @ inertia_flu @ FLU_TO_FRD

    config["description"] = (
        "Independent 1.3 kg all-up mass candidate with a 0.6 kg SO101 arm "
        "and a 0.7 kg remaining aircraft. Geometry and rotor axes are inherited "
        "from Base 1; the mass split is an approximate user input, not measured "
        "per-link inertia evidence."
    )
    config["scenario"] = "candidate_1p3kg_arm_0p6kg_ideal_linear_supply"
    config["estimated_all_up_mass_kg"] = TOTAL_MASS_KG
    config["temporary_fixed_mass_kg"] = TOTAL_MASS_KG
    config["formal_urdf"] = "urdf/my_drone_v3/my_drone_cad_candidate_1p3kg.urdf"
    config["mass_override_source"] = (
        "user approximate split: all-up 1.3 kg including SO101 arm 0.6 kg; "
        "group-internal link mass/inertia ratios inherited from Base 1 CAD volumes"
    )
    config["mass_partition"] = {
        "status": "APPROXIMATE_USER_INPUT_NOT_COMPONENT_SCALE_DATA",
        "all_up_mass_kg": TOTAL_MASS_KG,
        "arm_mass_kg": ARM_MASS_KG,
        "remaining_aircraft_mass_kg": AIRFRAME_MASS_KG,
        "arm_links": sorted(ARM_LINKS),
        "reference_pose": "retracted/folded ground pose",
        "center_of_mass_flu_m": center_flu.tolist(),
        "center_of_mass_frd_m": center_frd.tolist(),
        "inertia_at_com_flu_kg_m2": inertia_flu.tolist(),
        "inertia_at_com_frd_kg_m2": inertia_frd.tolist(),
    }
    installation = config.setdefault("arm_installation", {})
    installation["status"] = (
        "1.3 kg candidate reuses the connected Base 1 CAD geometry and joint "
        "re-indexing; Base 1 / 4 kg files remain unchanged"
    )
    # The Base 1 restore asset is the obsolete 0.797 m high bring-up stand.
    # The candidate starts from the 2 cm ground support in the world and must
    # land back on that surface, rather than spawning the old stand in flight.
    config.setdefault("takeoff_support_release", {})["restore_on_land"] = False

    # wrench_position_m is the immutable CAD rotor position about base_link.
    # PX4 CA_ROTORn_P* must instead be relative to the folded vehicle COM.
    for rotor in config["rotors"]:
        origin_position = np.asarray(rotor["wrench_position_m"], dtype=float)
        rotor["position_m"] = (origin_position - center_frd).tolist()
    rotor_by_motor = {int(row["motor"]): row for row in config["rotors"]}
    for row in config.get("motor_dynamics_table", {}).get("motors", []):
        rotor = rotor_by_motor[int(row["motor"])]
        row["position_frd_m"] = list(rotor["position_m"])
        row["wrench_position_frd_m"] = list(rotor["wrench_position_m"])

    desired = np.array([0.0, 0.0, -mass * GRAVITY_M_S2, 0.0, 0.0, 0.0])
    allocation = allocate_bounded_wrench(config, desired)
    if not allocation["feasible"]:
        raise ValueError(
            f"1.3 kg folded hover is infeasible: residual={allocation['residual_norm']}"
        )
    hover = np.asarray(allocation["thrust_n"], dtype=float)
    maximum_thrust = float(config["maximum_thrust_n"])
    hover_fractions = hover / maximum_thrust
    average_hover = float(np.mean(hover))
    config["bounded_hover_thrust_n"] = hover.tolist()
    config["bounded_hover_residual"] = allocation["residual"].tolist()
    config["bounded_hover_residual_norm"] = float(allocation["residual_norm"])
    config["estimated_vertical_thrust_to_weight"] = (
        float(config["maximum_vertical_force_n"]) / (mass * GRAVITY_M_S2)
    )
    normalization = config["actuator_normalization"]
    normalization["px4_hover_command"] = average_hover / maximum_thrust
    normalization["physical_hover_thrust_n"] = average_hover
    normalization["physical_hover_fraction"] = average_hover / maximum_thrust
    normalization["status"] = (
        "1.3 kg candidate folded-COM hover allocation with the unchanged "
        "ideal linear Base 1 motor interface"
    )
    config["candidate_hover_analysis"] = {
        "reference_pose": "retracted/folded",
        "per_motor_thrust_n": hover.tolist(),
        "per_motor_command_fraction": hover_fractions.tolist(),
        "minimum_command_fraction": float(np.min(hover_fractions)),
        "maximum_command_fraction": float(np.max(hover_fractions)),
        "mean_command_fraction": float(np.mean(hover_fractions)),
        "allocation_residual_norm": float(allocation["residual_norm"]),
        "vertical_thrust_to_weight": config["estimated_vertical_thrust_to_weight"],
    }
    config["candidate_compensation_limits"] = {
        "gravity_torque_limit_nm": 0.90,
        "maximum_motor_delta_n": 1.25,
        "minimum_motor_headroom_n": 0.25,
        # The arm is about 46% of the 1.3 kg all-up mass.  A pose that is only
        # statically allocatable is not a safe flight pose: PX4 still needs
        # differential thrust to arrest the transient produced while the arm
        # enters and leaves it.  Keep 0.30 N per-motor overlay authority in
        # reserve; the first dynamic directional run proved that the previous
        # 0.05 N floor admitted a left-side pose with just 0.074 N remaining
        # and no usable recovery margin.
        "minimum_delta_headroom_n": 0.30,
        "basis": (
            "standard 25,725-pose static scan: maximum collision-free gravity "
            "torque 0.82446 N m and maximum required motor delta 1.18362 N"
        ),
        "status": "OFFLINE_STATIC_LIMITS_NOT_DYNAMIC_FLIGHT_ACCEPTED",
    }
    release = config.setdefault("takeoff_support_release", {})
    release["release_up_force_n"] = mass * GRAVITY_M_S2 * 1.02
    release["status"] = (
        "candidate support release threshold is 102 percent of the 1.3 kg "
        "estimated weight; dynamic validation remains required"
    )
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(
        json.dumps(config, ensure_ascii=False, indent=2) + "\n",
        encoding="utf-8",
    )
    return config


def _replace_parameter(text: str, name: str, value: str) -> str:
    text, count = re.subn(
        rf"(?m)^(param set(?:-default)? {re.escape(name)}\s+)\S+\s*$",
        rf"\g<1>{value}",
        text,
    )
    if count != 1:
        raise ValueError(f"expected exactly one {name}, found {count}")
    return text


def build_airframe(source: Path, config: dict, output: Path) -> None:
    text = source.read_text(encoding="utf-8")
    text = text.replace(
        "Gazebo my_drone CAD debug 4kg ideal",
        "Gazebo my_drone CAD candidate 1.3kg arm 0.6kg",
    )
    for index, rotor in enumerate(sorted(config["rotors"], key=lambda r: int(r["motor"]))):
        for axis, value in zip("XYZ", rotor["position_m"]):
            text = _replace_parameter(text, f"CA_ROTOR{index}_P{axis}", f"{value:.9f}")
    hover = float(config["actuator_normalization"]["px4_hover_command"])
    text = _replace_parameter(text, "MPC_THR_HOVER", f"{hover:.4f}")
    for name, value in CANDIDATE_RATE_GAINS.items():
        text = _replace_parameter(text, name, f"{value:.4f}")
    text = text.replace(
        "# Debug-only 4 kg profile has about 2.08:1 vertical thrust-to-weight.",
        "# 1.3 kg candidate uses the user-provided approximate mass split.",
    )
    text += (
        "\n# Candidate-only rate gains are inertia-scaled initial values and require "
        "fresh PX4/Gazebo identification.\n"
    )
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(text, encoding="utf-8", newline="\n")


def main() -> None:
    source_urdf = PACKAGE / "urdf/my_drone_v3/my_drone_cad_debug_4kg.urdf"
    source_config = PACKAGE / "config/my_drone_v3_cad_debug_4kg.json"
    reference = PACKAGE / "config/so101_motion_reference_4kg.json"
    source_airframe = WORKSPACE / "px4/airframes/4027_gz_my_drone_octorotor_debug_4kg"
    output_urdf = PACKAGE / "urdf/my_drone_v3/my_drone_cad_candidate_1p3kg.urdf"
    output_config = PACKAGE / "config/my_drone_v3_cad_candidate_1p3kg.json"
    output_airframe = WORKSPACE / "px4/airframes/4028_gz_my_drone_octorotor_candidate_1p3kg"
    report_path = WORKSPACE / "analysis/base1/candidate_1p3kg_mass_allocation.json"

    partition = build_urdf(source_urdf, output_urdf)
    config = build_config(source_config, output_urdf, reference, output_config)
    build_airframe(source_airframe, config, output_airframe)
    folded = _folded_positions(reference)
    source_mass, _, source_inertia = UrdfModel(source_urdf).mass_properties(folded)
    candidate_inertia = np.asarray(
        config["mass_partition"]["inertia_at_com_flu_kg_m2"], dtype=float
    )
    report = {
        "status": "GENERATED_OFFLINE_NOT_GAZEBO_FLIGHT_ACCEPTED",
        "source_profile": "Base 1 / 4 kg protected debug profile",
        "mass_partition": partition,
        "computed_mass_properties": config["mass_partition"],
        "hover_analysis": config["candidate_hover_analysis"],
        "candidate_rate_gains": CANDIDATE_RATE_GAINS,
        "candidate_rate_gain_basis": {
            "source_folded_mass_kg": float(source_mass),
            "source_folded_inertia_diag_kg_m2": np.diag(source_inertia).tolist(),
            "candidate_folded_inertia_diag_kg_m2": np.diag(candidate_inertia).tolist(),
            "candidate_to_source_inertia_ratio": (
                np.diag(candidate_inertia) / np.diag(source_inertia)
            ).tolist(),
            "status": "INERTIA_SCALED_INITIAL_TUNING_NOT_DYNAMICALLY_IDENTIFIED",
        },
        "outputs": {
            "urdf": str(output_urdf),
            "config": str(output_config),
            "airframe": str(output_airframe),
        },
        "limitations": [
            "1.3 kg and 0.6 kg are approximate user inputs, not scale measurements",
            "per-link mass ratios and all inertias are provisional CAD-volume scaling",
            "motor thrust axes and opposite-pitch all-up hypothesis are unchanged",
            "candidate rate gains are inertia-scaled initial values and require fresh dynamic identification",
            "no Gazebo/PX4 flight acceptance is implied by this offline build",
        ],
    }
    report_path.parent.mkdir(parents=True, exist_ok=True)
    report_path.write_text(
        json.dumps(report, ensure_ascii=False, indent=2) + "\n",
        encoding="utf-8",
    )
    print(json.dumps(report, ensure_ascii=False, indent=2))


if __name__ == "__main__":
    main()
