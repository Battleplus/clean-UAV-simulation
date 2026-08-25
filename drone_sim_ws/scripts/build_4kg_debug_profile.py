#!/usr/bin/env python3
"""Build an isolated 4 kg ideal-control profile from the formal 7.735 kg baseline."""

from __future__ import annotations

import copy
import json
import re
import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np
from scipy.optimize import lsq_linear
from scipy.spatial.transform import Rotation

from drone_arm_sim.allocation_analysis import allocation_matrix


FORMAL_MASS_KG = 7.735
DEBUG_MASS_KG = 4.0
SCALE = DEBUG_MASS_KG / FORMAL_MASS_KG
GRAVITY = 9.80665

# The arm_mount remains exactly at the CAD assembly interface: the base-body
# and SO101-base meshes share the original mating vertices.  Only the servo
# coordinate zero is re-indexed in this isolated 4 kg profile.  The midpoint
# between the original folded pose and the requested nose-forward straight
# pose is used so *both* endpoints remain inside the measured joint limits.
# The formal 7.735 kg CAD URDF and its motion reference remain untouched.
ARM_MOUNT_XYZ = np.zeros(3)
ARM_MOUNT_RPY = np.zeros(3)
FORMAL_FOLDED_RAD = {
    "shoulder_pan": 0.0,
    "shoulder_lift": 0.0,
    "elbow_flex": 0.0,
    "wrist_flex": 0.0,
    "wrist_roll": -np.pi / 2.0,
    "gripper": 0.0,
}
FORMAL_FORWARD_STRAIGHT_RAD = {
    # Keep shoulder_pan at the folded physical angle: the base must not yaw.
    # Flipping the planar shoulder solution by -pi makes the remaining chain
    # unfold along ROS FLU +X (the frozen nose direction).
    "shoulder_pan": 0.0,
    "shoulder_lift": 0.25068950 - np.pi,
    "elbow_flex": 2.81363492,
    "wrist_flex": 0.02542253,
    "wrist_roll": 0.0,
    "gripper": 0.0,
}
ARM_ZERO_OFFSETS_RAD = {
    name: 0.5 * (
        FORMAL_FOLDED_RAD[name] + FORMAL_FORWARD_STRAIGHT_RAD[name]
    )
    for name in FORMAL_FOLDED_RAD
    if name != "gripper"
}
DEBUG_FOLDED_RAD = {
    name: FORMAL_FOLDED_RAD[name] - ARM_ZERO_OFFSETS_RAD.get(name, 0.0)
    for name in FORMAL_FOLDED_RAD
}
DEBUG_FORWARD_STRAIGHT_RAD = {
    name: FORMAL_FORWARD_STRAIGHT_RAD[name]
    - ARM_ZERO_OFFSETS_RAD.get(name, 0.0)
    for name in FORMAL_FORWARD_STRAIGHT_RAD
}


def _numbers(values: np.ndarray) -> str:
    return " ".join(f"{float(value):.12g}" for value in values)


def _reindex_joint_zero(joint: ET.Element, offset_rad: float) -> None:
    """Bake one rotation about the measured joint axis into its zero frame."""
    origin = joint.find("origin")
    axis_node = joint.find("axis")
    if origin is None or axis_node is None:
        raise ValueError(f"joint {joint.get('name')} lacks origin or axis")
    old_rpy = np.array(
        [float(value) for value in origin.get("rpy", "0 0 0").split()],
        dtype=float,
    )
    axis = np.array(
        [float(value) for value in axis_node.get("xyz", "1 0 0").split()],
        dtype=float,
    )
    axis /= np.linalg.norm(axis)
    rotation = (
        Rotation.from_euler("xyz", old_rpy).as_matrix()
        @ Rotation.from_rotvec(axis * float(offset_rad)).as_matrix()
    )
    origin.set("rpy", _numbers(Rotation.from_matrix(rotation).as_euler("xyz")))


def install_reindexed_base1_arm(root: ET.Element) -> None:
    """Keep the CAD mount connected and preserve folded/straight endpoints."""
    joints = {joint.get("name"): joint for joint in root.findall("joint")}
    mount = joints.get("arm_mount")
    if mount is None or mount.find("origin") is None:
        raise ValueError("arm_mount is missing from the 4 kg URDF")
    mount_origin = mount.find("origin")
    mount_origin.set("xyz", _numbers(ARM_MOUNT_XYZ))
    mount_origin.set("rpy", _numbers(ARM_MOUNT_RPY))
    for name, offset in ARM_ZERO_OFFSETS_RAD.items():
        joint = joints.get(name)
        if joint is None:
            raise ValueError(f"{name} is missing from the 4 kg URDF")
        _reindex_joint_zero(joint, offset)
    controls = root.find("./ros2_control")
    if controls is None:
        raise ValueError("ros2_control is missing from the 4 kg URDF")
    for name, value in DEBUG_FOLDED_RAD.items():
        state = controls.find(
            f"./joint[@name='{name}']/state_interface[@name='position']"
            "/param[@name='initial_value']"
        )
        if state is None:
            raise ValueError(f"initial position is missing for joint {name}")
        state.text = f"{float(value):.12g}"


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
    install_reindexed_base1_arm(root)
    # Keep the validated Base 1 low-noise barometer.  Regenerating the debug
    # profile must not silently restore the formal model's provisional 3 Pa
    # noise and undo the saved hover baseline.
    baro_stddev = root.find(
        "./gazebo/sensor[@name='air_pressure_sensor']"
        "/air_pressure/pressure/noise/stddev"
    )
    if baro_stddev is None:
        raise ValueError("air_pressure_sensor Gaussian stddev is missing")
    baro_stddev.text = "0.2"
    output.parent.mkdir(parents=True, exist_ok=True)
    ET.indent(tree, space="  ")
    tree.write(output, encoding="utf-8", xml_declaration=True)
    return total


def build_config(source: Path, output: Path) -> tuple[dict, float]:
    config = json.loads(source.read_text(encoding="utf-8"))
    config = copy.deepcopy(config)
    config["description"] = (
        "Isolated 4.000 kg ideal-control debug profile. Geometry, COM locations, "
        "per-link mass ratios, inertia ratios and CAD rotor locations are retained "
        "from the protected 7.735 kg model; axis_body remains the explicit all-up "
        "debug hypothesis pending propeller handedness or signed thrust tests."
    )
    config["scenario"] = "debug_4kg_ideal_linear_supply"
    config["estimated_all_up_mass_kg"] = DEBUG_MASS_KG
    config["temporary_fixed_mass_kg"] = DEBUG_MASS_KG
    config["mass_override_source"] = (
        "debug-only uniform scaling from protected 7.735 kg baseline; scale="
        f"{SCALE:.12f}"
    )
    config["formal_urdf"] = "urdf/my_drone_v3/my_drone_cad_debug_4kg.urdf"
    config["arm_installation"] = {
        "profile": "base1_cad_connected_folded_ground_nose_forward_straight_flight",
        "ground_spawn_z_m": 0.289,
        "mount_xyz_m": ARM_MOUNT_XYZ.tolist(),
        "mount_rpy_rad": ARM_MOUNT_RPY.tolist(),
        "joint_zero_offsets_rad": {
            name: float(value)
            for name, value in ARM_ZERO_OFFSETS_RAD.items()
        },
        "folded_command_rad": DEBUG_FOLDED_RAD,
        "nose_forward_straight_command_rad": DEBUG_FORWARD_STRAIGHT_RAD,
        "status": (
            "4 kg Base 1 CAD-connected installation; movable links are "
            "re-indexed between the original folded ground pose and the "
            "nose-forward / body +X straight flight pose with zero base yaw; "
            "formal 7.735 kg CAD "
            "assembly and motion reference are unchanged"
        ),
    }

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
    config["reaction_moment_ratio_m"] = 0.005
    config["reaction_moment_estimate"] = {
        "value_m": 0.005,
        "status": "temporary debug Q/T estimate retained only to validate yaw control",
    }
    config.setdefault("motor_dynamics_table", {}).setdefault(
        "reaction_torque_model", {}
    )["q_over_t_m"] = 0.005
    # The formal hover solution was calculated with its separate 0.001 m
    # reaction-torque estimate.  Re-solve the 4 kg trim after applying the
    # debug-only 0.005 m value; scaling the old thrust vector leaves a small
    # residual body torque and contaminates observer baselines.
    hover_target = np.array(
        [0.0, 0.0, -DEBUG_MASS_KG * GRAVITY, 0.0, 0.0, 0.0]
    )
    bounded = lsq_linear(
        allocation_matrix(config),
        hover_target,
        bounds=(0.0, maximum_thrust),
        lsmr_tol="auto",
    )
    residual = allocation_matrix(config) @ bounded.x - hover_target
    if not bounded.success or float(np.linalg.norm(residual)) > 1.0e-8:
        raise ValueError("4 kg debug allocation cannot trim hover")
    hover = bounded.x.tolist()
    config["bounded_hover_thrust_n"] = hover
    config["bounded_hover_residual"] = residual.tolist()
    config["bounded_hover_residual_norm"] = float(np.linalg.norm(residual))
    physical_hover = float(np.mean(bounded.x))
    hover_command = physical_hover / maximum_thrust
    config["actuator_normalization"]["px4_hover_command"] = hover_command
    config["actuator_normalization"]["physical_hover_thrust_n"] = physical_hover
    config["actuator_normalization"]["physical_hover_fraction"] = hover_command
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
                "ground-level contact pads are removed after 102 percent of weight "
                "and a balanced wrench are held; the retracted CAD gripper is the "
                "aircraft's initial table contact"
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
    text, km_count = re.subn(
        r"(param set-default CA_ROTOR\d+_KM\s+)([-+0-9.eE]+)",
        lambda match: (
            f"{match.group(1)}"
            f"{(-0.005 if float(match.group(2)) < 0.0 else 0.005):.9f}"
        ),
        text,
    )
    if km_count != 8:
        raise ValueError(f"expected eight CA_ROTORn_KM parameters, found {km_count}")
    # The formal airframe is intentionally sluggish because it has only 7.5%
    # vertical margin.  The isolated 4 kg profile has >2:1 T/W, so use
    # moderate bring-up gains and useful velocity limits.  These remain
    # debug parameters, not a tune for the eventual physical 7.735 kg craft.
    debug_parameters = {
        "EKF2_HGT_REF": "0",
        "EKF2_GPS_CTRL": "5",
        "EKF2_BARO_CTRL": "1",
        "EKF2_BARO_DELAY": "0",
        "EKF2_EV_CTRL": "12",
        "EKF2_EVA_NOISE": "0.05",
        "EKF2_EVV_NOISE": "0.05",
        "MPC_XY_P": "0.95",
        "MPC_XY_VEL_P_ACC": "1.80",
        "MPC_XY_VEL_I_ACC": "0.40",
        "MPC_XY_VEL_D_ACC": "0.20",
        "MPC_XY_VEL_MAX": "0.40",
        "MPC_Z_P": "1.00",
        "MPC_Z_VEL_P_ACC": "4.00",
        "MPC_Z_VEL_I_ACC": "2.00",
        "MPC_Z_VEL_D_ACC": "0.00",
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
        "MC_YAWRATE_I": "0.030",
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
        "Ground-level bring-up contacts.",
        "Isolated 4 kg ground-level bring-up contacts.",
    )
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(text, encoding="utf-8", newline="\n")


def build_motion_reference(source: Path, output: Path) -> None:
    """Preserve formal physical presets under the isolated joint re-index."""
    reference = json.loads(source.read_text(encoding="utf-8"))
    names = [item["name"] for item in reference["joints"]]
    offsets = np.array([ARM_ZERO_OFFSETS_RAD.get(name, 0.0) for name in names])
    for name, values in reference["presets"].items():
        reference["presets"][name] = (
            np.asarray(values, dtype=float) - offsets
        ).tolist()
    reference["presets"]["flight_straight_forward"] = [
        DEBUG_FORWARD_STRAIGHT_RAD[name] for name in names
    ]
    reference["purpose"] = (
        "4 kg Base 1 motion reference: original CAD folded ground pose and "
        "airborne straight pose toward ROS FLU body +X / nose with zero base yaw"
    )
    reference["zero_configuration"] = (
        "Servo coordinates are centered between the original folded pose and "
        "the requested nose-forward (body +X) zero-base-yaw flight pose"
    )
    reference["formal_reference_unchanged"] = True
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(
        json.dumps(reference, ensure_ascii=False, indent=2) + "\n",
        encoding="utf-8",
    )


def main() -> None:
    workspace = Path(__file__).resolve().parents[1]
    package = workspace / "src/drone_arm_sim"
    formal_urdf = package / "urdf/my_drone_v3/my_drone_cad_formal_dynamic.urdf"
    formal_config = package / "config/my_drone_v3_cad_7p735_flight.json"
    formal_airframe = workspace / "px4/airframes/4026_gz_my_drone_octorotor_7p735"
    debug_urdf = package / "urdf/my_drone_v3/my_drone_cad_debug_4kg.urdf"
    debug_config = package / "config/my_drone_v3_cad_debug_4kg.json"
    debug_motion_reference = package / "config/so101_motion_reference_4kg.json"
    debug_airframe = workspace / "px4/airframes/4027_gz_my_drone_octorotor_debug_4kg"
    debug_world = package / "worlds/flight_world_debug_4kg.sdf"

    total = scale_urdf(formal_urdf, debug_urdf)
    build_motion_reference(
        package / "config/so101_motion_reference.json", debug_motion_reference
    )
    config, hover_command = build_config(formal_config, debug_config)
    build_airframe(formal_airframe, debug_airframe, hover_command)
    build_debug_world(package / "worlds/flight_world_250hz.sdf", debug_world)
    print(f"scale={SCALE:.12f}")
    print(f"urdf_total_mass_kg={total:.12f}")
    print(f"vertical_thrust_to_weight={config['estimated_vertical_thrust_to_weight']:.6f}")
    print(f"px4_hover_command={hover_command:.6f}")
    print(debug_urdf)
    print(debug_config)
    print(debug_motion_reference)
    print(debug_airframe)
    print(debug_world)


if __name__ == "__main__":
    main()
