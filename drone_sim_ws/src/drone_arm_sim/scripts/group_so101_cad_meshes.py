#!/usr/bin/env python3
"""Group assembly-coordinate SO101 CAD components into moving rigid links."""

from __future__ import annotations

import json
import re
from pathlib import Path

import numpy as np

from generate_cad_v2_urdf import merge_stls, mesh_properties


PACKAGE = Path(__file__).resolve().parents[1]
WORKSPACE = PACKAGE.parents[1]
VISUAL = PACKAGE / "meshes" / "my_drone_v2" / "visual"
EVIDENCE = WORKSPACE / "analysis" / "cad_direct" / "so101_joint_evidence.json"
MANIFEST = WORKSPACE / "analysis" / "cad_direct" / "so101_link_mesh_groups.json"

ORDER = ["arm_base", "shoulder", "upper_arm", "lower_arm", "wrist", "gripper", "moving_jaw"]
SERVO_PARENT = {
    "SCS215_7": "arm_base", "SCS215_7_1": "shoulder", "SCS215_7_2": "upper_arm",
    "SCS215_7_3": "lower_arm", "SCS215_7_4": "wrist", "SCS215_7_5": "gripper",
}
SERVO_CHILD = {
    "SCS215_7": "shoulder", "SCS215_7_1": "upper_arm", "SCS215_7_2": "lower_arm",
    "SCS215_7_3": "wrist", "SCS215_7_4": "gripper", "SCS215_7_5": "moving_jaw",
}


def servo_id(name: str) -> str | None:
    hits = re.findall(r"SCS215_7(?:_[1-5])?", name)
    return hits[-1] if hits else None


def choose_group(path: Path, centers_mm: dict[str, np.ndarray]) -> tuple[str, str]:
    name = path.name
    sid = servo_id(name)
    if sid:
        if "驱动" in name or "Moving_Jaw" in name:
            return SERVO_CHILD[sid], "drive horn / driven child"
        return SERVO_PARENT[sid], "servo assembly / parent body"

    explicit = (
        ("Moving_Jaw", "moving_jaw"),
        ("f插头-", "gripper"),
        ("f相机支架-", "gripper"),
        ("f测力天平-", "gripper"),
        ("Wrist_Roll_Follower", "gripper"),
        ("Wrist_Roll_Pitch", "wrist"),
        ("Motor_holder_SO101_Wrist", "lower_arm"),
        ("Under_arm_SO101", "lower_arm"),
        ("Upper_arm_SO101", "upper_arm"),
        ("Rotation_Pitch_SO101", "shoulder"),
        ("Motor_holder_SO101_Base", "shoulder"),
        ("fbase2-", "arm_base"),
    )
    for token, group in explicit:
        if token in name:
            return group, f"semantic component: {token}"

    low, high, _ = mesh_properties(path)
    center = (low + high) * 0.5
    group = min(centers_mm, key=lambda key: float(np.linalg.norm(center - centers_mm[key])))
    return group, "nearest rigid-link reference center"


def main() -> None:
    evidence = json.loads(EVIDENCE.read_text(encoding="utf-8"))
    points = {
        item["joint"]: np.asarray(item["assembly_axis_point_m"], dtype=float) * 1000.0
        for item in evidence["joints"]
    }
    # SolidWorks' STL exporter adds a fixed translation to every assembly
    # component. Recover it from the same six drive horns used as axis
    # evidence so API geometry and STL geometry share one frame.
    probe = json.loads((WORKSPACE / "analysis" / "cad_direct" / "axis_face_probe.json").read_text(encoding="utf-8"))
    offsets = []
    all_files = sorted(VISUAL.glob("full_assembly - fSO101 Assembly-1*.STL"))
    for item in evidence["joints"]:
        sid = item["servo_instance"]
        candidates = [f for f in all_files if f"{sid}.step-1" in f.name and "驱动" in f.name]
        if len(candidates) != 1:
            raise RuntimeError(f"{sid}: expected one drive horn STL, got {len(candidates)}")
        lo, hi, _ = mesh_properties(candidates[0])
        stl_center = (lo + hi) * 0.5
        rec = next(r for r in probe if f"/{sid}.step-1/" in r["instance"])
        bbox = np.asarray(rec["assembly_bbox_m"], dtype=float) * 1000.0
        cad_center = (bbox[:3] + bbox[3:]) * 0.5
        offsets.append(stl_center - cad_center)
    stl_offset_mm = np.mean(offsets, axis=0)
    if float(np.max(np.linalg.norm(np.asarray(offsets) - stl_offset_mm, axis=1))) > 0.05:
        raise RuntimeError("STL/API frame offset is not rigid")
    points = {name: value + stl_offset_mm for name, value in points.items()}
    # Reference centres lie inside each rigid span in the retracted CAD pose.
    p = [points[j] for j in ("shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll", "gripper")]
    centers = {
        "arm_base": p[0] + (p[0] - p[1]) * 0.35,
        "shoulder": (p[0] + p[1]) * 0.5,
        "upper_arm": (p[1] + p[2]) * 0.5,
        "lower_arm": (p[2] + p[3]) * 0.5,
        "wrist": (p[3] + p[4]) * 0.5,
        "gripper": (p[4] + p[5]) * 0.5,
        "moving_jaw": p[5] + (p[5] - p[4]) * 0.35,
    }
    groups: dict[str, list[Path]] = {name: [] for name in ORDER}
    records = []
    files = all_files
    for path in files:
        group, reason = choose_group(path, centers)
        groups[group].append(path)
        records.append({"file": path.name, "link": group, "reason": reason})
    if len(files) != 134 or any(not value for value in groups.values()):
        raise RuntimeError({name: len(value) for name, value in groups.items()})
    for name, paths in groups.items():
        merge_stls(paths, VISUAL / f"so101_{name}.stl")
    payload = {
        "schema": 1,
        "source": r"E:\清洁无人机\零件\完整零件\组合无人机.SLDASM",
        "method": "servo parent/drive-horn child semantics, named structural parts, then nearest rigid-span centre",
        "solidworks_stl_translation_mm": stl_offset_mm.tolist(),
        "counts": {name: len(paths) for name, paths in groups.items()},
        "components": records,
    }
    MANIFEST.write_text(json.dumps(payload, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    print(json.dumps(payload["counts"], ensure_ascii=False))


if __name__ == "__main__":
    main()
