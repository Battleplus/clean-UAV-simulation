"""Convert SolidWorks cylindrical-face evidence into PX4 rotor geometry."""

from __future__ import annotations

import json
import math
from pathlib import Path
import re

import numpy as np


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
PUSHER_MOTORS = {3, 6, 7, 8}
UPWARD_THRUST_MOTORS = {1, 2, 3, 6}
DOWNWARD_THRUST_MOTORS = {4, 5, 7, 8}


def key(instance: str) -> tuple[int, int]:
    frame = re.search(r"f机架总成-(\d+)", instance)
    item = re.search(r"/(?:f电机|f螺旋桨)-(\d+)$", instance)
    if frame is None or item is None:
        raise ValueError(f"Unrecognized instance: {instance}")
    return int(frame.group(1)), int(item.group(1))


def unit(values) -> np.ndarray:
    result = np.asarray(values, dtype=float)
    return result / np.linalg.norm(result)


def assembly_to_frd(values) -> np.ndarray:
    x, y, z = np.asarray(values, dtype=float)
    return np.asarray([-z, x, -y])


def angle_degrees(first, second, unsigned=False) -> float:
    dot = float(unit(first) @ unit(second))
    if unsigned:
        dot = abs(dot)
    return math.degrees(math.acos(np.clip(dot, -1.0, 1.0)))


def generate(project: Path) -> None:
    analysis = project / "drone_sim_ws" / "analysis" / "cad_direct"
    probe_path = analysis / "axis_face_probe.json"
    records = json.loads(probe_path.read_text(encoding="utf-8"))
    motors = {key(record["instance"]): record for record in records if "f电机-" in record["instance"]}
    props = {key(record["instance"]): record for record in records if "f螺旋桨-" in record["instance"]}
    if set(motors) != set(PX4_MOTOR_MAP) or set(props) != set(PX4_MOTOR_MAP):
        raise ValueError("Expected eight matched motor/propeller CAD instances")

    bbox_origin = np.mean(
        [record["assembly_bbox_center_m"] for record in motors.values()], axis=0
    )
    evidence = []
    for cad_key, motor_number in sorted(PX4_MOTOR_MAP.items(), key=lambda item: item[1]):
        motor = motors[cad_key]
        prop = props[cad_key]
        shaft_candidates = [
            face for face in motor["cylinders"]
            if abs(float(face["radius_m"]) - 0.0025) < 1e-8
        ]
        if len(shaft_candidates) != 1 or len(prop["cylinders"]) != 1:
            raise ValueError(f"Ambiguous cylindrical axis for {cad_key}")
        shaft = shaft_candidates[0]
        hub = prop["cylinders"][0]
        shaft_origin = np.asarray(shaft["assembly_origin_m"], dtype=float)
        prop_origin = np.asarray(hub["assembly_origin_m"], dtype=float)
        shaft_axis = unit(shaft["assembly_axis"])
        prop_axis = unit(hub["assembly_axis"])
        outward = prop_origin - shaft_origin
        if float(outward @ shaft_axis) < 0.0:
            shaft_axis *= -1.0
        if float(outward @ prop_axis) < 0.0:
            prop_axis *= -1.0
        perpendicular = outward - shaft_axis * float(outward @ shaft_axis)
        coaxial_faces = [
            face for face in motor["cylinders"]
            if abs(abs(float(unit(face["assembly_axis"]) @ shaft_axis)) - 1.0) < 1e-10
            and np.linalg.norm(
                (np.asarray(face["assembly_origin_m"]) - shaft_origin)
                - shaft_axis * float((np.asarray(face["assembly_origin_m"]) - shaft_origin) @ shaft_axis)
            ) < 1e-9
        ]
        prop_side_frd = unit(assembly_to_frd(shaft_axis))
        operational_frd = prop_side_frd.copy()
        wants_up = motor_number in UPWARD_THRUST_MOTORS
        is_up = operational_frd[2] < 0.0
        if wants_up != is_up:
            operational_frd *= -1.0
        position_frd = assembly_to_frd(shaft_origin - bbox_origin)
        prop_position_frd = assembly_to_frd(prop_origin - bbox_origin)
        evidence.append({
            "motor": motor_number,
            "cad_key": list(cad_key),
            "motor_instance": motor["instance"],
            "propeller_instance": prop["instance"],
            "axis_source": "motor shaft r=2.5 mm cylindrical face; cross-checked against rotor/body cylinders and propeller cylindrical face",
            "assembly_axis_point_mm": (shaft_origin * 1000.0).tolist(),
            "assembly_propeller_axis_point_mm": (prop_origin * 1000.0).tolist(),
            "assembly_thrust_axis": shaft_axis.tolist(),
            "cad_propeller_side_axis_frd": prop_side_frd.tolist(),
            "ros_flu_thrust_axis": (operational_frd * np.asarray([1.0, -1.0, -1.0])).tolist(),
            "px4_frd_thrust_axis": operational_frd.tolist(),
            "px4_frd_position_m": position_frd.tolist(),
            "px4_frd_propeller_position_m": prop_position_frd.tolist(),
            "tilt_deg": angle_degrees(operational_frd, [0.0, 0.0, -1.0]),
            "azimuth_deg": math.degrees(math.atan2(operational_frd[1], operational_frd[0])),
            "propeller_side_tilt_deg": angle_degrees(prop_side_frd, [0.0, 0.0, -1.0]),
            "propulsion_mode": "pusher" if motor_number in PUSHER_MOTORS else "puller",
            "vertical_thrust_direction": "up" if wants_up else "down",
            "thrust_sign_status": "CONFIRMED by user rotation/mount table: " + ("upward" if wants_up else "downward"),
            "motor_propeller_line_offset_mm": float(np.linalg.norm(perpendicular) * 1000.0),
            "motor_propeller_axis_angle_deg": angle_degrees(shaft_axis, prop_axis),
            "coaxial_motor_face_radii_mm": sorted(float(face["radius_m"]) * 1000.0 for face in coaxial_faces),
        })

    payload = {
        "schema": 1,
        "source_assembly": str(project / "零件" / "完整零件" / "组合无人机.SLDASM"),
        "source_probe": str(probe_path),
        "method": "SolidWorks IFace2.GetSurface / ISurface.IsCylinder / CylinderParams",
        "body_origin": "mean SolidWorks component bounding-box centre of eight f电机 instances",
        "assembly_to_ros_flu": "X=-CAD_Z, Y=-CAD_X, Z=CAD_Y",
        "assembly_to_px4_frd": "X=-CAD_Z, Y=CAD_X, Z=-CAD_Y",
        "body_frame_frozen": {
            "ros_flu": "+X nose, +Y left, +Z up",
            "px4_frd": "+X nose, +Y right, +Z down",
            "clockwise_motor_order_from_nose_right": [1, 3, 8, 4, 2, 6, 7, 5],
            "pusher_motors": sorted(PUSHER_MOTORS),
            "puller_motors": sorted(set(PX4_MOTOR_MAP.values()) - PUSHER_MOTORS),
            "upward_thrust_motors": sorted(UPWARD_THRUST_MOTORS),
            "downward_thrust_motors": sorted(DOWNWARD_THRUST_MOTORS),
        },
        "assembly_bbox_origin_m": bbox_origin.tolist(),
        "motors": evidence,
    }
    evidence_path = analysis / "motor_axis_evidence.json"
    evidence_path.write_text(json.dumps(payload, ensure_ascii=False, indent=2), encoding="utf-8")

    config_path = project / "drone_sim_ws" / "src" / "drone_arm_sim" / "config" / "my_drone_v2_cad.json"
    config = json.loads(config_path.read_text(encoding="utf-8"))
    by_motor = {item["motor"]: item for item in evidence}
    config["description"] = "Formal CAD rotor geometry extracted from SolidWorks cylindrical faces; no bounding-box centre-to-centre axis estimate is used."
    config["axis_source"] = "analysis/cad_direct/motor_axis_evidence.json"
    config["axis_extraction_method"] = payload["method"]
    for rotor in config["rotors"]:
        item = by_motor[rotor["motor"]]
        rotor["position_m"] = item["px4_frd_position_m"]
        rotor["axis_body"] = item["px4_frd_thrust_axis"]
        rotor["tilt_deg"] = item["tilt_deg"]
        rotor["azimuth_deg"] = item["azimuth_deg"]
        rotor["axis_source"] = item["axis_source"]
        rotor["cad_propeller_side_axis_body"] = item["cad_propeller_side_axis_frd"]
        rotor["thrust_sign_status"] = item["thrust_sign_status"]
        rotor["propulsion_mode"] = item["propulsion_mode"]
        rotor["axis_evidence_motor"] = item["motor_instance"]
        rotor["axis_evidence_propeller"] = item["propeller_instance"]
        rotor["motor_propeller_line_offset_mm"] = item["motor_propeller_line_offset_mm"]
        rotor["motor_propeller_axis_angle_deg"] = item["motor_propeller_axis_angle_deg"]
    vertical_force = config["maximum_thrust_n"] * sum(-rotor["axis_body"][2] for rotor in config["rotors"])
    best_case_upward_force = config["maximum_thrust_n"] * sum(
        max(0.0, -rotor["axis_body"][2]) for rotor in config["rotors"]
    )
    config["maximum_vertical_force_n"] = vertical_force
    config["maximum_supported_mass_kg"] = vertical_force / 9.80665
    config["estimated_vertical_thrust_to_weight"] = vertical_force / (config["estimated_all_up_mass_kg"] * 9.80665)
    config["best_case_upward_force_n_nonreversible"] = best_case_upward_force
    config["best_case_supported_mass_kg_nonreversible"] = best_case_upward_force / 9.80665
    config["best_case_thrust_to_weight_nonreversible"] = best_case_upward_force / (config["estimated_all_up_mass_kg"] * 9.80665)
    config["flight_feasibility_nonreversible"] = "INFEASIBLE" if best_case_upward_force <= config["estimated_all_up_mass_kg"] * 9.80665 else "FEASIBLE"
    config_path.write_text(json.dumps(config, ensure_ascii=False, indent=2), encoding="utf-8")

    report = ["# SolidWorks exact motor-axis evidence", ""]
    for item in evidence:
        report.extend([
            f"## motor_{item['motor']}", "",
            f"- 轴线来源：{item['axis_source']}",
            f"- 总装轴心（mm）：{item['assembly_axis_point_mm']}",
            f"- ROS FLU 推力轴：{item['ros_flu_thrust_axis']}",
            f"- PX4 FRD 推力轴：{item['px4_frd_thrust_axis']}",
            f"- CAD 螺旋桨所在侧轴：{item['cad_propeller_side_axis_frd']}",
            f"- 倾角：{item['tilt_deg']:.9f}°", f"- 方位角：{item['azimuth_deg']:.9f}°",
            f"- 电机轴与螺旋桨轴偏差：{item['motor_propeller_line_offset_mm']:.9f} mm / {item['motor_propeller_axis_angle_deg']:.9f}°",
            f"- 推力正负号状态：{item['thrust_sign_status']}",
            f"- 电机同轴圆柱半径（mm）：{item['coaxial_motor_face_radii_mm']}", "",
        ])
    (analysis / "motor_axis_evidence.md").write_text("\n".join(report), encoding="utf-8")
    print(evidence_path)
    print(config_path)


if __name__ == "__main__":
    generate(Path(__file__).resolve().parents[4])
