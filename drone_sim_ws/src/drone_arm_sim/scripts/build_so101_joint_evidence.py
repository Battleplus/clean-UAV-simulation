#!/usr/bin/env python3
"""Build traceable SO101 joint axes from SolidWorks cylindrical faces."""

from __future__ import annotations

import json
import math
import re
from pathlib import Path


ROOT = Path(__file__).resolve().parents[3]
ANALYSIS = ROOT / "analysis" / "cad_direct"
PROBE = ANALYSIS / "axis_face_probe.json"
OUTPUT_JSON = ANALYSIS / "so101_joint_evidence.json"
OUTPUT_MD = ANALYSIS / "so101_joint_evidence.md"

# Frozen from the exact motor-axis extraction. This is the CAD point used as
# base_link origin, expressed in the SolidWorks assembly frame.
BODY_ORIGIN_CAD_M = (-0.06443653447931824, 0.037365550279939826, -0.0003434681115362017)

JOINT_BY_SERVO = {
    "SCS215_7": "shoulder_pan",
    "SCS215_7_1": "shoulder_lift",
    "SCS215_7_2": "elbow_flex",
    "SCS215_7_3": "wrist_flex",
    "SCS215_7_4": "wrist_roll",
    "SCS215_7_5": "gripper",
}


def cad_vector_to_ros(v: list[float] | tuple[float, ...]) -> list[float]:
    # Frozen mapping: ROS FLU = [-CAD_Z, -CAD_X, CAD_Y].
    return [-v[2], -v[0], v[1]]


def unit(v: list[float]) -> list[float]:
    n = math.sqrt(sum(x * x for x in v))
    return [x / n for x in v]


def main() -> None:
    records = json.loads(PROBE.read_text(encoding="utf-8"))
    joints = []
    for record in records:
        match = re.search(r"/(SCS215_7(?:_[1-5])?)\.step-1/", record["instance"])
        if not match:
            continue
        servo = match.group(1)
        central = [
            face for face in record["cylinders"]
            if abs(face["radius_m"] - 0.0096) <= 1e-7
        ]
        if len(central) != 1:
            raise RuntimeError(f"{servo}: expected one r=9.6 mm cylinder, got {len(central)}")
        face = central[0]
        p_cad = face["assembly_origin_m"]
        rel_cad = [p_cad[i] - BODY_ORIGIN_CAD_M[i] for i in range(3)]
        axis_ros = unit(cad_vector_to_ros(face["assembly_axis"]))
        point_ros = cad_vector_to_ros(rel_cad)
        joints.append({
            "joint": JOINT_BY_SERVO[servo],
            "servo_instance": servo,
            "axis_source": "SolidWorks central drive-horn cylindrical face (r=9.6 mm)",
            "source_component": record["instance"],
            "assembly_axis_point_m": p_cad,
            "assembly_axis": face["assembly_axis"],
            "ros_flu_axis_point_m": point_ros,
            "ros_flu_axis": axis_ros,
            "source_radius_mm": face["radius_m"] * 1000.0,
        })

    joints.sort(key=lambda j: list(JOINT_BY_SERVO.values()).index(j["joint"]))
    if len(joints) != 6 or {j["joint"] for j in joints} != set(JOINT_BY_SERVO.values()):
        raise RuntimeError(f"Incomplete SO101 extraction: {len(joints)} joints")

    payload = {
        "schema": 1,
        "source_assembly": r"E:\清洁无人机\零件\完整零件\组合无人机.SLDASM",
        "source_probe": str(PROBE),
        "body_origin_cad_m": BODY_ORIGIN_CAD_M,
        "assembly_to_ros_flu": "X=-CAD_Z, Y=-CAD_X, Z=CAD_Y",
        "zero_configuration": "CAD retracted assembly pose",
        "mapping_basis": "SO101 serial kinematic order and matching drive-servo instance suffix",
        "joints": joints,
    }
    OUTPUT_JSON.write_text(json.dumps(payload, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")

    lines = [
        "# SO101 exact joint-axis evidence", "",
        f"Source assembly: `{payload['source_assembly']}`", "",
        "All axes come from the r=9.6 mm cylindrical face of the corresponding drive horn.", "",
        "| Joint | Servo | ROS FLU axis point (m) | ROS FLU unit axis |", "|---|---|---|---|",
    ]
    for j in joints:
        p = ", ".join(f"{x:.9f}" for x in j["ros_flu_axis_point_m"])
        a = ", ".join(f"{x:.9f}" for x in j["ros_flu_axis"])
        lines.append(f"| {j['joint']} | {j['servo_instance']} | [{p}] | [{a}] |")
    OUTPUT_MD.write_text("\n".join(lines) + "\n", encoding="utf-8")
    print(f"wrote {OUTPUT_JSON}")
    print(f"wrote {OUTPUT_MD}")


if __name__ == "__main__":
    main()
