#!/usr/bin/env python3
"""Build an isolated 4 kg ideal-control profile from the formal 7.735 kg baseline."""

from __future__ import annotations

import copy
import json
import re
import xml.etree.ElementTree as ET
from pathlib import Path


FORMAL_MASS_KG = 7.735
DEBUG_MASS_KG = 4.0
SCALE = DEBUG_MASS_KG / FORMAL_MASS_KG
GRAVITY = 9.80665


def scale_urdf(source: Path, output: Path) -> float:
    tree = ET.parse(source)
    root = tree.getroot()
    root.set("name", "my_drone_v3_cad_debug_4kg")
    total = 0.0
    for link in root.findall("link"):
        inertial = link.find("inertial")
        if inertial is None:
            continue
        mass = inertial.find("mass")
        inertia = inertial.find("inertia")
        if mass is None or inertia is None:
            continue
        scaled_mass = float(mass.get("value")) * SCALE
        mass.set("value", f"{scaled_mass:.12g}")
        total += scaled_mass
        for key in ("ixx", "ixy", "ixz", "iyy", "iyz", "izz"):
            inertia.set(key, f"{float(inertia.get(key)) * SCALE:.12g}")
    output.parent.mkdir(parents=True, exist_ok=True)
    ET.indent(tree, space="  ")
    tree.write(output, encoding="utf-8", xml_declaration=True)
    return total


def build_config(source: Path, output: Path) -> tuple[dict, float]:
    config = json.loads(source.read_text(encoding="utf-8"))
    config = copy.deepcopy(config)
    config["description"] = (
        "Isolated 4.000 kg ideal-control debug profile. Geometry, COM locations, "
        "per-link mass ratios, inertia ratios, CAD rotor locations and thrust axes "
        "are retained from the protected 7.735 kg formal model."
    )
    config["scenario"] = "debug_4kg_ideal_linear_supply"
    config["estimated_all_up_mass_kg"] = DEBUG_MASS_KG
    config["temporary_fixed_mass_kg"] = DEBUG_MASS_KG
    config["mass_override_source"] = (
        "debug-only uniform scaling from protected 7.735 kg baseline; scale="
        f"{SCALE:.12f}"
    )
    config["formal_urdf"] = "urdf/my_drone_v3/my_drone_cad_debug_4kg.urdf"

    # The same allocation matrix is linear in demanded weight, so the already
    # balanced formal hover solution scales exactly with vehicle mass.
    hover = [float(value) * SCALE for value in config["bounded_hover_thrust_n"]]
    config["bounded_hover_thrust_n"] = hover
    config["bounded_hover_residual"] = [0.0] * 6
    config["bounded_hover_residual_norm"] = 0.0
    maximum_thrust = float(config["maximum_thrust_n"])
    physical_hover = sum(hover) / len(hover)
    hover_command = physical_hover / maximum_thrust
    config["estimated_vertical_thrust_to_weight"] = (
        float(config["maximum_vertical_force_n"]) / (DEBUG_MASS_KG * GRAVITY)
    )
    config["actuator_input_model"] = "ideal_linear_thrust"
    config["actuator_normalization"] = {
        "px4_hover_command": hover_command,
        "physical_hover_thrust_n": physical_hover,
        "physical_hover_fraction": hover_command,
        "linear_thrust_per_command_n": maximum_thrust,
        "rated_thrust_command": 1.0,
        "maximum_command": 1.0,
        "maximum_thrust_n": maximum_thrust,
        "status": "debug-only ideal linear command-to-thrust mapping",
    }
    config["actuator_input_model_status"] = (
        "debug-only ideal mapping u=0..1 to 0..rated thrust; no voltage sag, "
        "ESC dead zone, transport delay or nonlinear static-thrust interpolation"
    )
    config["actuator_transport_delay_s"] = 0.0
    config["actuator_transport_delay_status"] = "disabled in 4 kg control-logic debug profile"
    motor_dynamics = config.setdefault("motor_dynamics", {})
    motor_dynamics["rise_time_constant_s"] = 0.001
    motor_dynamics["fall_time_constant_s"] = 0.001
    motor_dynamics["status"] = "near-instant ideal response for control-logic validation"
    for motor in config.get("motor_dynamics_table", {}).get("motors", []):
        motor["rise_time_constant_s"] = 0.001
        motor["fall_time_constant_s"] = 0.001

    battery = config.setdefault("battery_dynamics", {})
    battery["enabled"] = False
    battery["status"] = "disabled in 4 kg ideal control-logic debug profile"
    environment = config.setdefault("environment_dynamics", {})
    environment.setdefault("wind", {})["enabled"] = False
    environment.setdefault("ground_effect", {})["enabled"] = False
    # Yaw authority cannot be validated with zero propeller reaction torque.
    # Retain the small closed-loop bring-up estimate used by the formal
    # profile; it remains explicitly provisional rather than a real C_Q/C_T.
    config["reaction_moment_ratio_m"] = 0.001
    config["reaction_moment_estimate"] = {
        "value_m": 0.001,
        "status": "temporary debug Q/T estimate retained only to validate yaw control",
    }
    config.setdefault("motor_dynamics_table", {}).setdefault(
        "reaction_torque_model", {}
    )["q_over_t_m"] = 0.001
    release = config.setdefault("takeoff_support_release", {})
    release.update(
        {
            "enabled": True,
            "release_up_force_n": DEBUG_MASS_KG * GRAVITY * 1.02,
            "maximum_horizontal_force_n": 0.50,
            "maximum_com_torque_nm": 0.05,
            "hold_time_s": 0.15,
            "restore_on_land": True,
            "status": (
                "the tall debug spawn fixture is removed after 102 percent of "
                "weight and a balanced wrench are held, avoiding a release-induced "
                "free-fall transient; an equivalent-height four-pad "
                "fixture is restored below the aircraft for landing"
            ),
        }
    )
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(json.dumps(config, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    return config, hover_command


def build_airframe(source: Path, output: Path, hover_command: float) -> None:
    text = source.read_text(encoding="utf-8")
    text = text.replace("Gazebo my_drone CAD canted octorotor", "Gazebo my_drone CAD debug 4kg ideal")
    text = re.sub(
        r"param set-default MPC_THR_HOVER\s+[-+0-9.eE]+",
        f"param set-default MPC_THR_HOVER {hover_command:.4f}",
        text,
    )
    text = re.sub(
        r"param set-default MPC_THR_MIN\s+[-+0-9.eE]+",
        "param set-default MPC_THR_MIN 0.10",
        text,
    )
    text = re.sub(
        r"param set-default MPC_THR_MAX\s+[-+0-9.eE]+",
        "param set-default MPC_THR_MAX 1.00",
        text,
    )
    # The formal airframe is intentionally sluggish because it has only 7.5%
    # vertical margin.  The isolated 4 kg profile has >2:1 T/W, so use
    # moderate bring-up gains and useful velocity limits.  These remain
    # debug parameters, not a tune for the eventual physical 7.735 kg craft.
    debug_parameters = {
        "EKF2_HGT_REF": "1",
        "EKF2_GPS_CTRL": "7",
        "EKF2_BARO_CTRL": "0",
        "EKF2_BARO_DELAY": "20",
        "MPC_XY_P": "2.20",
        "MPC_XY_VEL_P_ACC": "1.00",
        "MPC_XY_VEL_I_ACC": "0.10",
        "MPC_XY_VEL_D_ACC": "0.35",
        "MPC_XY_VEL_MAX": "0.40",
        "MPC_Z_P": "0.35",
        "MPC_Z_VEL_P_ACC": "2.20",
        "MPC_Z_VEL_I_ACC": "0.20",
        "MPC_Z_VEL_D_ACC": "0.20",
        "MPC_TKO_RAMP_T": "1.00",
        "MPC_Z_VEL_MAX_UP": "0.25",
        "MPC_Z_VEL_MAX_DN": "0.25",
        "MPC_TILTMAX_AIR": "20",
        "MPC_TILTMAX_LND": "10",
        "MC_ROLL_P": "3.00",
        "MC_PITCH_P": "3.00",
        "MC_YAW_P": "1.50",
        "MC_ROLLRATE_MAX": "90.0",
        "MC_PITCHRATE_MAX": "90.0",
        "MC_YAWRATE_MAX": "90.0",
        "MC_ROLLRATE_P": "0.100",
        "MC_PITCHRATE_P": "0.100",
        "MC_YAWRATE_P": "0.120",
        "MC_ROLLRATE_I": "0.060",
        "MC_PITCHRATE_I": "0.060",
        "MC_YAWRATE_I": "0.060",
    }
    for name, value in debug_parameters.items():
        text, count = re.subn(
            rf"param set-default {name}\s+[-+0-9.eE]+",
            f"param set-default {name} {value}",
            text,
        )
        if count == 0 and name.startswith("EKF2_"):
            text, count = re.subn(
                r"(param set-default MPC_ALT_MODE\s+[-+0-9.eE]+)",
                rf"\1\nparam set-default {name} {value}",
                text,
            )
        if count == 0 and name == "MPC_Z_VEL_D_ACC":
            text, count = re.subn(
                r"(param set-default MPC_Z_VEL_I_ACC\s+[-+0-9.eE]+)",
                rf"\1\nparam set-default {name} {value}",
                text,
            )
        if count != 1:
            raise ValueError(f"expected one {name} parameter in source airframe, found {count}")
    text = text.replace(
        "# The formal CAD vehicle has only about 7.5% vertical thrust",
        "# Debug-only 4 kg profile has about 2.08:1 vertical thrust-to-weight.\n# The protected 7.735 kg formal profile is not modified.\n# Original note: the formal CAD vehicle has only about 7.5% vertical thrust",
    )
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(text, encoding="utf-8", newline="\n")


def build_debug_world(source: Path, output: Path) -> None:
    """Create an isolated stable bench-contact world for flight-logic tests."""
    text = source.read_text(encoding="utf-8")
    text = text.replace(
        "<pose>0.17 0.17 -0.01 0 0 0</pose>",
        "<pose>0.17 0.17 0.4085 0 0 0</pose>",
    ).replace(
        "<pose>0.17 -0.17 -0.01 0 0 0</pose>",
        "<pose>0.17 -0.17 0.4085 0 0 0</pose>",
    ).replace(
        "<pose>-0.17 0.17 -0.01 0 0 0</pose>",
        "<pose>-0.17 0.17 0.4085 0 0 0</pose>",
    ).replace(
        "<pose>-0.17 -0.17 -0.01 0 0 0</pose>",
        "<pose>-0.17 -0.17 0.4085 0 0 0</pose>",
    ).replace(
        "<box><size>0.05 0.05 0.02</size></box>",
        "<box><size>0.05 0.05 0.817</size></box>",
    )
    text = text.replace(
        "Ground-level bring-up contacts.",
        "Isolated 4 kg flight-logic bench contacts.",
    )
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(text, encoding="utf-8", newline="\n")


def main() -> None:
    workspace = Path(__file__).resolve().parents[1]
    package = workspace / "src/drone_arm_sim"
    formal_urdf = package / "urdf/my_drone_v3/my_drone_cad_formal_dynamic.urdf"
    formal_config = package / "config/my_drone_v3_cad_7p735_flight.json"
    formal_airframe = workspace / "px4/airframes/4026_gz_my_drone_octorotor_7p735"
    debug_urdf = package / "urdf/my_drone_v3/my_drone_cad_debug_4kg.urdf"
    debug_config = package / "config/my_drone_v3_cad_debug_4kg.json"
    debug_airframe = workspace / "px4/airframes/4027_gz_my_drone_octorotor_debug_4kg"
    debug_world = package / "worlds/flight_world_debug_4kg.sdf"

    total = scale_urdf(formal_urdf, debug_urdf)
    config, hover_command = build_config(formal_config, debug_config)
    build_airframe(formal_airframe, debug_airframe, hover_command)
    build_debug_world(package / "worlds/flight_world_250hz.sdf", debug_world)
    print(f"scale={SCALE:.12f}")
    print(f"urdf_total_mass_kg={total:.12f}")
    print(f"vertical_thrust_to_weight={config['estimated_vertical_thrust_to_weight']:.6f}")
    print(f"px4_hover_command={hover_command:.6f}")
    print(debug_urdf)
    print(debug_config)
    print(debug_airframe)
    print(debug_world)


if __name__ == "__main__":
    main()
