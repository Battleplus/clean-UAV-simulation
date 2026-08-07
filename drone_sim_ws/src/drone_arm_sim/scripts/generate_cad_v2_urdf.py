"""Generate the split my_drone_v2 URDF directly from CAD-exported STL files."""

from __future__ import annotations

import json
from pathlib import Path
import re
import shutil
import struct
import xml.etree.ElementTree as ET

import numpy as np
from scipy.spatial.transform import Rotation


DENSITY_KG_M3 = 1300.0
MAXIMUM_THRUST_N = 11.76798
THRUST_TABLE_GF = {
    0: 0, 15: 26, 20: 79, 25: 142, 30: 160, 35: 254,
    40: 372, 45: 457, 50: 545, 55: 562, 60: 630, 65: 673,
    70: 725, 75: 831, 80: 909, 85: 1041, 90: 1283,
}
CURRENT_TABLE_A = {
    0: 0.0, 15: 0.66, 20: 1.30, 25: 2.12, 30: 2.87, 35: 4.75,
    40: 7.23, 45: 8.91, 50: 10.40, 55: 11.65, 60: 12.96,
    65: 14.16, 70: 15.92, 75: 18.07, 80: 21.51, 85: 25.25,
    90: 32.01,
}
CAD_TO_FLU = np.array(
    [[0.0, 0.0, -1.0], [-1.0, 0.0, 0.0], [0.0, 1.0, 0.0]]
)
# (frame-quartet instance, local motor instance) -> PX4 motor number.
PX4_MOTOR_MAP = {
    (2, 2): 1,
    (2, 3): 2,
    (1, 3): 3,
    (2, 4): 4,
    (2, 1): 5,
    (1, 2): 6,
    (1, 1): 7,
    (1, 4): 8,
}
CW_MOTORS = {1, 2, 7, 8}


def read_stl(path: Path) -> tuple[int, bytes]:
    data = path.read_bytes()
    if len(data) < 84:
        raise ValueError(f"Invalid STL: {path}")
    count = struct.unpack_from("<I", data, 80)[0]
    if len(data) != 84 + count * 50:
        raise ValueError(f"Only binary STL is supported: {path}")
    return count, data[84:]


def merge_stls(paths: list[Path], output: Path) -> None:
    blocks = []
    count = 0
    for path in paths:
        child_count, block = read_stl(path)
        count += child_count
        blocks.append(block)
    header = b"my_drone_v2 CAD group".ljust(80, b"\0")
    output.write_bytes(header + struct.pack("<I", count) + b"".join(blocks))


def mesh_properties(path: Path) -> tuple[np.ndarray, np.ndarray, float]:
    count, block = read_stl(path)
    low = np.full(3, np.inf)
    high = np.full(3, -np.inf)
    signed_volume_mm3 = 0.0
    for index in range(count):
        values = struct.unpack_from("<12f", block, index * 50)
        triangle = np.asarray(
            [values[3:6], values[6:9], values[9:12]], dtype=float
        )
        low = np.minimum(low, triangle.min(axis=0))
        high = np.maximum(high, triangle.max(axis=0))
        signed_volume_mm3 += (
            np.dot(triangle[0], np.cross(triangle[1], triangle[2])) / 6.0
        )
    return low, high, abs(signed_volume_mm3) * 1e-9


def element(parent: ET.Element, tag: str, text: str | None = None, **attrs):
    child = ET.SubElement(parent, tag, {key: str(value) for key, value in attrs.items()})
    child.text = text
    return child


def fmt(values) -> str:
    return " ".join(f"{float(value):.9g}" for value in values)


def inertia_box(mass: float, size: np.ndarray) -> tuple[float, float, float]:
    x, y, z = size
    return (
        mass * (y * y + z * z) / 12.0,
        mass * (x * x + z * z) / 12.0,
        mass * (x * x + y * y) / 12.0,
    )


def add_inertial(
    link: ET.Element, mass: float, center: np.ndarray, size: np.ndarray
) -> None:
    inertial = element(link, "inertial")
    element(inertial, "origin", xyz=fmt(center), rpy="0 0 0")
    element(inertial, "mass", value=f"{mass:.9g}")
    ixx, iyy, izz = inertia_box(mass, size)
    element(
        inertial,
        "inertia",
        ixx=f"{ixx:.9g}",
        ixy="0",
        ixz="0",
        iyy=f"{iyy:.9g}",
        iyz="0",
        izz=f"{izz:.9g}",
    )


def rpy(matrix: np.ndarray) -> np.ndarray:
    return Rotation.from_matrix(matrix).as_euler("xyz")


def add_visual(
    link: ET.Element,
    name: str,
    mesh_uri: str,
    mesh_rotation: np.ndarray,
    mesh_translation: np.ndarray,
    color: str,
) -> None:
    visual = element(link, "visual", name=name)
    element(
        visual,
        "origin",
        xyz=fmt(mesh_translation),
        rpy=fmt(rpy(mesh_rotation)),
    )
    geometry = element(visual, "geometry")
    element(geometry, "mesh", filename=mesh_uri, scale="0.001 0.001 0.001")
    material = element(visual, "material", name=f"{name}_material")
    element(material, "color", rgba=color)


def z_axis_rotation(axis: np.ndarray) -> np.ndarray:
    axis = axis / np.linalg.norm(axis)
    helper = np.array([1.0, 0.0, 0.0])
    if abs(float(helper @ axis)) > 0.9:
        helper = np.array([0.0, 1.0, 0.0])
    x_axis = helper - axis * float(helper @ axis)
    x_axis /= np.linalg.norm(x_axis)
    y_axis = np.cross(axis, x_axis)
    return np.column_stack((x_axis, y_axis, axis))


def instance_ids(path: Path, component: str) -> tuple[int, int]:
    frame = re.search(r"f机架总成-(\d+)", path.name)
    item = re.search(rf"f{component}-(\d+)\.STL$", path.name)
    if frame is None or item is None:
        raise ValueError(f"Could not parse component instance: {path.name}")
    return int(frame.group(1)), int(item.group(1))


def generate(package: Path) -> None:
    visual_dir = package / "meshes" / "my_drone_v2" / "visual"
    component_files = sorted(visual_dir.glob("full_assembly - *.STL"))
    motor_files = [path for path in component_files if "f电机-" in path.name]
    rotor_files = [path for path in component_files if "f螺旋桨-" in path.name]
    arm_files = [path for path in component_files if "fSO101 Assembly-1" in path.name]
    excluded = set(motor_files + rotor_files + arm_files)
    base_files = [path for path in component_files if path not in excluded]
    if len(motor_files) != 8 or len(rotor_files) != 8:
        raise ValueError(
            f"Expected 8 motors and 8 rotors, got {len(motor_files)} and "
            f"{len(rotor_files)}"
        )

    base_mesh = visual_dir / "base_body.stl"
    arm_mesh = visual_dir / "arm_fixed.stl"
    merge_stls(base_files, base_mesh)
    merge_stls(arm_files, arm_mesh)

    motor_by_key = {instance_ids(path, "电机"): path for path in motor_files}
    rotor_by_key = {instance_ids(path, "螺旋桨"): path for path in rotor_files}
    motor_centers_cad = {
        key: sum(mesh_properties(path)[:2]) / 2.0
        for key, path in motor_by_key.items()
    }
    rotor_centers_cad = {
        key: sum(mesh_properties(path)[:2]) / 2.0
        for key, path in rotor_by_key.items()
    }
    cad_origin_mm = np.mean(list(motor_centers_cad.values()), axis=0)
    mesh_translation_base = -CAD_TO_FLU @ (cad_origin_mm * 0.001)

    rotor_records = []
    for key, motor_number in PX4_MOTOR_MAP.items():
        motor_cad = motor_centers_cad[key]
        rotor_cad = rotor_centers_cad[key]
        position_flu = CAD_TO_FLU @ ((motor_cad - cad_origin_mm) * 0.001)
        rotor_position_flu = CAD_TO_FLU @ (
            (rotor_cad - cad_origin_mm) * 0.001
        )
        axis_flu = CAD_TO_FLU @ (rotor_cad - motor_cad)
        axis_flu /= np.linalg.norm(axis_flu)
        # CAD locates the propeller on opposite sides for the two frame
        # quartets. Standard one-direction PX4 motors must all produce a
        # positive body-Z (up) component for hover, which fixes this sign.
        if axis_flu[2] < 0.0:
            axis_flu *= -1.0
        axis_frd = axis_flu * np.array([1.0, -1.0, -1.0])
        position_frd = position_flu * np.array([1.0, -1.0, -1.0])
        rotor_records.append(
            {
                "motor": motor_number,
                "name": f"rotor_{motor_number}_link",
                "cad_instance": motor_by_key[key].stem.removeprefix(
                    "full_assembly - "
                ),
                "position_m": position_frd.tolist(),
                "axis_body": axis_frd.tolist(),
                "direction": -1 if motor_number in CW_MOTORS else 1,
                "turning_direction": "CW" if motor_number in CW_MOTORS else "CCW",
                "tilt_deg": float(np.degrees(np.arccos(axis_flu[2]))),
                "_key": key,
                "_position_flu": position_flu,
                "_rotor_position_flu": rotor_position_flu,
                "_axis_flu": axis_flu,
            }
        )
    rotor_records.sort(key=lambda record: record["motor"])

    # Replace the early centre-to-centre prototype geometry with formal
    # SolidWorks cylindrical-face evidence.
    evidence_path = package.parents[1] / "analysis" / "cad_direct" / "motor_axis_evidence.json"
    if not evidence_path.exists():
        raise FileNotFoundError(f"Exact SolidWorks motor-axis evidence is required: {evidence_path}")
    exact_payload = json.loads(evidence_path.read_text(encoding="utf-8"))
    exact_by_motor = {item["motor"]: item for item in exact_payload["motors"]}
    for record in rotor_records:
        item = exact_by_motor[record["motor"]]
        position_frd = np.asarray(item["px4_frd_position_m"], dtype=float)
        prop_position_frd = np.asarray(item["px4_frd_propeller_position_m"], dtype=float)
        axis_frd = np.asarray(item["px4_frd_thrust_axis"], dtype=float)
        record["position_m"] = position_frd.tolist()
        record["axis_body"] = axis_frd.tolist()
        record["tilt_deg"] = item["tilt_deg"]
        record["azimuth_deg"] = item["azimuth_deg"]
        record["axis_source"] = item["axis_source"]
        record["cad_propeller_side_axis_body"] = item["cad_propeller_side_axis_frd"]
        record["thrust_sign_status"] = item["thrust_sign_status"]
        record["propulsion_mode"] = item["propulsion_mode"]
        record["motor_propeller_line_offset_mm"] = item["motor_propeller_line_offset_mm"]
        record["motor_propeller_axis_angle_deg"] = item["motor_propeller_axis_angle_deg"]
        record["_position_flu"] = position_frd * np.array([1.0, -1.0, -1.0])
        record["_rotor_position_flu"] = prop_position_frd * np.array([1.0, -1.0, -1.0])
        record["_axis_flu"] = axis_frd * np.array([1.0, -1.0, -1.0])

    component_volumes = {
        path: mesh_properties(path)[2] for path in component_files
    }
    total_volume = sum(component_volumes.values())
    estimated_mass = total_volume * DENSITY_KG_M3
    # The exact CAD evidence contains four upward and four downward thrust
    # axes.  Commands are non-negative for ordinary fixed-pitch propellers, so
    # the signed all-motors-at-maximum sum is not a useful lift-capacity
    # metric.  Keep it as an audit value, and separately compute the optimistic
    # upper bound obtained by turning every downward-pointing rotor off.  If
    # even that bound is below weight, a hover solution is mathematically
    # impossible regardless of moment balancing or controller tuning.
    full_throttle_signed_vertical_force = MAXIMUM_THRUST_N * sum(
        float(record["_axis_flu"][2]) for record in rotor_records
    )
    best_case_upward_force = MAXIMUM_THRUST_N * sum(
        max(0.0, float(record["_axis_flu"][2])) for record in rotor_records
    )
    weight = estimated_mass * 9.80665
    best_case_thrust_to_weight = best_case_upward_force / weight
    nonreversible_feasibility = (
        "INFEASIBLE" if best_case_upward_force <= weight else "UNPROVEN"
    )
    config = {
        "description": "Formal rotor lines extracted from SolidWorks cylindrical faces; bounding-box centre-to-centre axes are not used.",
        "source_manifest": "analysis/cad_direct/assembly_manifest.json",
        "axis_source": "analysis/cad_direct/motor_axis_evidence.json",
        "axis_extraction_method": exact_payload["method"],
        "body_frame_frozen": exact_payload["body_frame_frozen"],
        "assembly_sha256": "0cbd1455f3bf9c4acee91a4f5ee997f8fc823cd7817ba7f4a217f9fc029ee202",
        "coordinate_frame": "PX4 body FRD; ROS FLU uses X=-CAD Z, Y=-CAD X, Z=CAD Y",
        "cad_origin_mm": cad_origin_mm.tolist(),
        "density_kg_m3": DENSITY_KG_M3,
        "mesh_volume_m3": total_volume,
        "estimated_all_up_mass_kg": estimated_mass,
        "maximum_thrust_n": MAXIMUM_THRUST_N,
        "minimum_thrust_n": 0.0,
        "command_model": "piecewise-linear interpolation of supplied 14.8 V static thrust table; capped at rated 1.2 kgf",
        "static_thrust_model": {
            "method": "piecewise-linear interpolation of supplied 14.8 V static test table",
            "rated_cap_n": MAXIMUM_THRUST_N,
            "points": [
                {
                    "throttle_percent": throttle,
                    "measured_thrust_gf": grams,
                    "measured_thrust_n": grams / 1000.0 * 9.80665,
                    "rated_capped_thrust_n": min(
                        grams / 1000.0 * 9.80665, MAXIMUM_THRUST_N
                    ),
                }
                for throttle, grams in THRUST_TABLE_GF.items()
            ],
            "status": "measured table supplied by user; 90 percent point capped from 1.283 kgf to rated 1.2 kgf",
        },
        "motor_dynamics": {
            "rise_time_constant_s": 0.035,
            "fall_time_constant_s": 0.035,
            "status": "initial estimate only; replace with measured step response",
        },
        "static_current_model": {
            "method": "piecewise-linear interpolation of supplied 14.8 V static test table",
            "points": [
                {"throttle_percent": throttle, "current_a": current}
                for throttle, current in CURRENT_TABLE_A.items()
            ],
        },
        "reaction_moment_ratio_m": None,
        "maximum_vertical_force_n": full_throttle_signed_vertical_force,
        "maximum_supported_mass_kg": full_throttle_signed_vertical_force / 9.80665,
        "estimated_vertical_thrust_to_weight": full_throttle_signed_vertical_force
        / weight,
        "full_throttle_signed_vertical_force_n": full_throttle_signed_vertical_force,
        "best_case_upward_force_n_nonreversible": best_case_upward_force,
        "best_case_supported_mass_kg_nonreversible": best_case_upward_force / 9.80665,
        "best_case_thrust_to_weight_nonreversible": best_case_thrust_to_weight,
        "flight_feasibility_nonreversible": nonreversible_feasibility,
        "flight_feasibility_reason": (
            "Even the optimistic upward-only thrust bound is below vehicle weight; "
            "downward-pointing fixed-pitch rotors cannot be used for lift."
            if nonreversible_feasibility == "INFEASIBLE"
            else "Vertical capacity alone is sufficient, but full six-axis hover "
            "allocation still requires a bounded feasibility solve."
        ),
        "upward_thrust_motors": [
            int(record["motor"])
            for record in rotor_records
            if float(record["_axis_flu"][2]) > 0.0
        ],
        "downward_thrust_motors": [
            int(record["motor"])
            for record in rotor_records
            if float(record["_axis_flu"][2]) < 0.0
        ],
        "rotors": [
            {key: value for key, value in record.items() if not key.startswith("_")}
            for record in rotor_records
        ],
    }
    config_path = package / "config" / "my_drone_v2_cad.json"
    config_path.write_text(
        json.dumps(config, ensure_ascii=False, indent=2), encoding="utf-8"
    )

    root = ET.Element("robot", {"name": "my_drone_v2_cad_dynamic"})
    root.append(
        ET.Comment(
            " Geometry is generated only from E:/清洁无人机/零件. "
            "Mass is a density-based estimate. "
        )
    )
    base_low, base_high, base_volume = mesh_properties(base_mesh)
    base_center = CAD_TO_FLU @ (
        ((base_low + base_high) / 2.0 - cad_origin_mm) * 0.001
    )
    base_size = (base_high - base_low)[[2, 0, 1]] * 0.001
    base_link = element(root, "link", name="base_link")
    add_inertial(
        base_link, base_volume * DENSITY_KG_M3, base_center, base_size
    )
    add_visual(
        base_link,
        "base_cad_visual",
        "package://drone_arm_sim/meshes/my_drone_v2/visual/base_body.stl",
        CAD_TO_FLU,
        mesh_translation_base,
        "0.60 0.64 0.70 1",
    )
    collision = element(base_link, "collision", name="base_box_collision")
    element(collision, "origin", xyz=fmt(base_center), rpy="0 0 0")
    geometry = element(collision, "geometry")
    element(geometry, "box", size=fmt(base_size))

    arm_low, arm_high, arm_volume = mesh_properties(arm_mesh)
    joint_evidence = json.loads(
        (package.parents[1] / "analysis" / "cad_direct" / "so101_joint_evidence.json").read_text(encoding="utf-8")
    )
    motion_reference = json.loads(
        (package / "config" / "so101_motion_reference.json").read_text(encoding="utf-8")
    )
    evidence_by_name = {item["joint"]: item for item in joint_evidence["joints"]}
    motion_by_name = {item["name"]: item for item in motion_reference["joints"]}
    joint_names = [item["name"] for item in motion_reference["joints"]]
    frame_position = {"arm_base_link": np.zeros(3)}
    child_by_joint = {
        "shoulder_pan": "shoulder_link",
        "shoulder_lift": "upper_arm_link",
        "elbow_flex": "lower_arm_link",
        "wrist_flex": "wrist_link",
        "wrist_roll": "gripper_link",
        "gripper": "moving_jaw_link",
    }
    parent_by_joint = {
        "shoulder_pan": "arm_base_link",
        "shoulder_lift": "shoulder_link",
        "elbow_flex": "upper_arm_link",
        "wrist_flex": "lower_arm_link",
        "wrist_roll": "wrist_link",
        "gripper": "gripper_link",
    }
    for joint_name in joint_names:
        frame_position[child_by_joint[joint_name]] = np.asarray(
            evidence_by_name[joint_name]["ros_flu_axis_point_m"], dtype=float
        )

    group_by_link = {
        "arm_base_link": "arm_base",
        "shoulder_link": "shoulder",
        "upper_arm_link": "upper_arm",
        "lower_arm_link": "lower_arm",
        "wrist_link": "wrist",
        "gripper_link": "gripper",
        "moving_jaw_link": "moving_jaw",
    }
    for link_name, group_name in group_by_link.items():
        group_mesh = visual_dir / f"so101_{group_name}.stl"
        if not group_mesh.exists():
            raise FileNotFoundError(
                f"Run group_so101_cad_meshes.py first: {group_mesh}"
            )
        low, high, volume = mesh_properties(group_mesh)
        center_base = CAD_TO_FLU @ (((low + high) * 0.5 - cad_origin_mm) * 0.001)
        size = (high - low)[[2, 0, 1]] * 0.001
        link = element(root, "link", name=link_name)
        add_inertial(
            link,
            max(volume * DENSITY_KG_M3, 0.001),
            center_base - frame_position[link_name],
            size,
        )
        add_visual(
            link,
            f"{group_name}_cad_visual",
            f"package://drone_arm_sim/meshes/my_drone_v2/visual/so101_{group_name}.stl",
            CAD_TO_FLU,
            mesh_translation_base - frame_position[link_name],
            "0.95 0.45 0.05 1",
        )

    joint = element(root, "joint", name="arm_mount", type="fixed")
    element(joint, "origin", xyz="0 0 0", rpy="0 0 0")
    element(joint, "parent", link="base_link")
    element(joint, "child", link="arm_base_link")
    arm_joint_gazebo = element(root, "gazebo", reference="arm_mount")
    element(arm_joint_gazebo, "preserveFixedJoint", "true")

    for joint_name in joint_names:
        parent_name = parent_by_joint[joint_name]
        child_name = child_by_joint[joint_name]
        spec = motion_by_name[joint_name]
        evidence = evidence_by_name[joint_name]
        arm_joint = element(root, "joint", name=joint_name, type="revolute")
        element(
            arm_joint,
            "origin",
            xyz=fmt(frame_position[child_name] - frame_position[parent_name]),
            rpy="0 0 0",
        )
        element(arm_joint, "parent", link=parent_name)
        element(arm_joint, "child", link=child_name)
        element(arm_joint, "axis", xyz=fmt(evidence["ros_flu_axis"]))
        element(
            arm_joint,
            "limit",
            lower=spec["lower_rad"],
            upper=spec["upper_rad"],
            effort=spec["effort_nm"],
            velocity=spec["velocity_rad_s"],
        )
        element(
            arm_joint,
            "dynamics",
            damping=spec["damping"],
            friction=spec["friction"],
        )

    for record in rotor_records:
        motor_number = record["motor"]
        key = record["_key"]
        motor_source = motor_by_key[key]
        rotor_source = rotor_by_key[key]
        motor_output = visual_dir / f"motor_{motor_number}.stl"
        rotor_output = visual_dir / f"rotor_{motor_number}.stl"
        shutil.copyfile(motor_source, motor_output)
        shutil.copyfile(rotor_source, rotor_output)
        motor_low, motor_high, motor_volume = mesh_properties(motor_output)
        rotor_low, rotor_high, rotor_volume = mesh_properties(rotor_output)
        motor_mass = motor_volume * DENSITY_KG_M3
        rotor_mass = rotor_volume * DENSITY_KG_M3
        motor_size = (motor_high - motor_low)[[2, 0, 1]] * 0.001
        rotor_size = (rotor_high - rotor_low)[[2, 0, 1]] * 0.001
        motor_position = record["_position_flu"]
        rotor_position = record["_rotor_position_flu"]

        motor_link = element(root, "link", name=f"motor_{motor_number}_link")
        add_inertial(motor_link, motor_mass, np.zeros(3), motor_size)
        add_visual(
            motor_link,
            f"motor_{motor_number}_visual",
            f"package://drone_arm_sim/meshes/my_drone_v2/visual/motor_{motor_number}.stl",
            CAD_TO_FLU,
            mesh_translation_base - motor_position,
            "0.18 0.20 0.23 1",
        )
        motor_joint = element(
            root, "joint", name=f"motor_{motor_number}_mount", type="fixed"
        )
        element(motor_joint, "origin", xyz=fmt(motor_position), rpy="0 0 0")
        element(motor_joint, "parent", link="base_link")
        element(motor_joint, "child", link=f"motor_{motor_number}_link")

        rotor_rotation = z_axis_rotation(record["_axis_flu"])
        rotor_link = element(root, "link", name=f"rotor_{motor_number}_link")
        radius = float(np.max(rotor_size) / 2.0)
        add_inertial(
            rotor_link,
            rotor_mass,
            np.zeros(3),
            np.full(3, 2.0 * radius),
        )
        add_visual(
            rotor_link,
            f"rotor_{motor_number}_visual",
            f"package://drone_arm_sim/meshes/my_drone_v2/visual/rotor_{motor_number}.stl",
            rotor_rotation.T @ CAD_TO_FLU,
            rotor_rotation.T @ (mesh_translation_base - rotor_position),
            "0.08 0.18 0.72 0.85",
        )
        rotor_joint = element(
            root,
            "joint",
            name=f"rotor_{motor_number}_joint",
            type="continuous",
        )
        element(
            rotor_joint,
            "origin",
            xyz=fmt(rotor_position - motor_position),
            rpy=fmt(rpy(rotor_rotation)),
        )
        element(rotor_joint, "parent", link=f"motor_{motor_number}_link")
        element(rotor_joint, "child", link=f"rotor_{motor_number}_link")
        element(rotor_joint, "axis", xyz="0 0 1")
        element(rotor_joint, "dynamics", damping="0.0001", friction="0")

    gazebo_reference = element(root, "gazebo", reference="base_link")
    for name, sensor_type, rate in (
        ("air_pressure_sensor", "air_pressure", "50"),
        ("magnetometer_sensor", "magnetometer", "100"),
        ("imu_sensor", "imu", "250"),
        ("navsat_sensor", "navsat", "30"),
    ):
        sensor = element(gazebo_reference, "sensor", name=name, type=sensor_type)
        element(sensor, "gz_frame_id", "base_link")
        element(sensor, "always_on", "1")
        element(sensor, "update_rate", rate)
        raw_topics = {
            "air_pressure": "/my_drone/raw/air_pressure",
            "magnetometer": "/my_drone/raw/magnetometer",
            "imu": "/my_drone/raw/imu",
            "navsat": "/my_drone/raw/navsat",
        }
        element(sensor, "topic", raw_topics[sensor_type])
        if sensor_type == "air_pressure":
            air_pressure = element(sensor, "air_pressure")
            pressure = element(air_pressure, "pressure")
            noise = element(pressure, "noise", type="gaussian")
            element(noise, "mean", "0")
            element(noise, "stddev", "3")
        elif sensor_type == "magnetometer":
            magnetometer = element(sensor, "magnetometer")
            for axis in ("x", "y", "z"):
                axis_element = element(magnetometer, axis)
                noise = element(axis_element, "noise", type="gaussian")
                element(noise, "stddev", "0.0001")
        elif sensor_type == "imu":
            imu = element(sensor, "imu")
            angular_velocity = element(imu, "angular_velocity")
            for axis in ("x", "y", "z"):
                axis_element = element(angular_velocity, axis)
                noise = element(axis_element, "noise", type="gaussian")
                element(noise, "mean", "0.0")
                element(noise, "stddev", "0.0008726646")
            linear_acceleration = element(imu, "linear_acceleration")
            for axis, standard_deviation in (
                ("x", "0.00637"),
                ("y", "0.00637"),
                ("z", "0.00686"),
            ):
                axis_element = element(linear_acceleration, axis)
                noise = element(axis_element, "noise", type="gaussian")
                element(noise, "mean", "0.0")
                element(noise, "stddev", standard_deviation)
    ros2_control = element(
        root, "ros2_control", name="GazeboSimSystem", type="system"
    )
    hardware = element(ros2_control, "hardware")
    element(hardware, "plugin", "gz_ros2_control/GazeboSimSystem")
    retracted = motion_reference["presets"]["retracted"]
    for index, joint_name in enumerate(joint_names):
        spec = motion_by_name[joint_name]
        control_joint = element(ros2_control, "joint", name=joint_name)
        command = element(control_joint, "command_interface", name="position")
        element(command, "param", str(spec["lower_rad"]), name="min")
        element(command, "param", str(spec["upper_rad"]), name="max")
        position = element(control_joint, "state_interface", name="position")
        element(position, "param", str(retracted[index]), name="initial_value")
        element(control_joint, "state_interface", name="velocity")

    gazebo = element(root, "gazebo")
    arm_controller = element(
        gazebo,
        "plugin",
        filename="libgz_ros2_control-system.so",
        name="gz_ros2_control::GazeboSimROS2ControlPlugin",
    )
    element(
        arm_controller,
        "parameters",
        "$(find drone_arm_sim)/config/so101_ros2_control.yaml",
    )
    element(arm_controller, "hold_joints", "true")
    element(arm_controller, "position_proportional_gain", "0.15")
    state_publisher = element(
        gazebo,
        "plugin",
        filename="gz-sim-joint-state-publisher-system",
        name="gz::sim::systems::JointStatePublisher",
    )
    for joint_name in joint_names:
        element(state_publisher, "joint_name", joint_name)

    plugin = element(
        gazebo,
        "plugin",
        filename="gz-sim-odometry-publisher-system",
        name="gz::sim::systems::OdometryPublisher",
    )
    element(plugin, "dimensions", "3")
    element(plugin, "odom_publish_frequency", "250")
    element(plugin, "odom_topic", "/model/my_drone/odometry")
    element(plugin, "odom_frame", "world")
    element(plugin, "robot_base_frame", "base_link")

    output = package / "urdf" / "my_drone_v2" / "my_drone_cad_dynamic.urdf"
    output.parent.mkdir(parents=True, exist_ok=True)
    tree = ET.ElementTree(root)
    ET.indent(tree, space="  ")
    tree.write(output, encoding="utf-8", xml_declaration=True)
    print(config_path)
    print(output)
    print(
        f"mass={estimated_mass:.6f} kg, "
        f"signed_full_throttle_vertical_force={full_throttle_signed_vertical_force:.6f} N, "
        f"T/W={config['estimated_vertical_thrust_to_weight']:.6f}"
    )


def main() -> None:
    package = Path(__file__).resolve().parents[1]
    generate(package)


if __name__ == "__main__":
    main()
