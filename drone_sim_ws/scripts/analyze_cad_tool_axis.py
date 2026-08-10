#!/usr/bin/env python3
"""Extract a reproducible gripper longitudinal-axis estimate from the CAD STL."""

from __future__ import annotations

import argparse
import json
from pathlib import Path
import struct

import numpy as np


CAD_TO_FLU = np.array([[0.0, 0.0, -1.0], [-1.0, 0.0, 0.0], [0.0, 1.0, 0.0]])
WRIST_POINT = np.array([0.110957550, -0.007061805, -0.257019339])
GRIPPER_JOINT_POINT = np.array([0.137886413, -0.022764883, -0.276993867])
GRIPPER_COM_LOCAL = np.array([0.0278397323, -0.000475002761, 0.0098625121])


def vertices(path: Path) -> np.ndarray:
    data = path.read_bytes()
    count = struct.unpack_from("<I", data, 80)[0]
    if len(data) != 84 + count * 50:
        raise ValueError("only binary STL is supported")
    result = np.empty((count * 3, 3))
    for index in range(count):
        values = struct.unpack_from("<12f", data, 84 + index * 50)
        result[index * 3:(index + 1) * 3] = np.asarray(values[3:12]).reshape(3, 3)
    return result


def unit(vector: np.ndarray) -> np.ndarray:
    return vector / np.linalg.norm(vector)


def main() -> None:
    workspace = Path(__file__).resolve().parents[1]
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--mesh", type=Path,
        default=workspace / "src/drone_arm_sim/meshes/my_drone_v2/visual/so101_gripper.stl",
    )
    parser.add_argument(
        "--output", type=Path,
        default=workspace / "analysis/cad_direct/tool_axis_evidence.json",
    )
    args = parser.parse_args()
    raw = vertices(args.mesh)
    centered = raw - raw.mean(axis=0)
    covariance = centered.T @ centered / len(centered)
    values, vectors = np.linalg.eigh(covariance)
    pca_raw = vectors[:, int(np.argmax(values))]
    pca_flu = unit(CAD_TO_FLU @ pca_raw)
    com_direction = unit(GRIPPER_COM_LOCAL)
    if float(pca_flu @ com_direction) < 0.0:
        pca_flu *= -1.0
    wrist_to_gripper = unit(GRIPPER_JOINT_POINT - WRIST_POINT)
    result = {
        "schema": 1,
        "source_mesh": str(args.mesh),
        "method": "area-unweighted STL vertex PCA, sign selected toward gripper CAD center of mass",
        "principal_axis_ros_flu": pca_flu.tolist(),
        "gripper_com_direction_local": com_direction.tolist(),
        "wrist_to_gripper_joint_direction_ros_flu": wrist_to_gripper.tolist(),
        "agreement_deg": {
            "pca_vs_com": float(np.degrees(np.arccos(np.clip(pca_flu @ com_direction, -1, 1)))),
            "pca_vs_joint": float(np.degrees(np.arccos(np.clip(pca_flu @ wrist_to_gripper, -1, 1)))),
        },
        "note": "PCA is geometric evidence, not a CAD datum axis; retain the source vectors for audit.",
    }
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(result, indent=2), encoding="utf-8")
    print(json.dumps(result, indent=2))


if __name__ == "__main__":
    main()
