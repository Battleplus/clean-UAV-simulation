"""Extract the eight motor mounting frames embedded in ``drone_body.stl``.

The body STL contains eight disconnected, congruent circular mount solids.
This script finds connected triangle components, selects the repeated group of
eight, and obtains each mount's symmetry axis using PCA.  The rotor origin is
the centre of the uppermost vertex plane along the upward-pointing axis.
"""

from __future__ import annotations

import argparse
from collections import Counter
import json
from pathlib import Path
import struct

import numpy as np


PX4_ORDER_CLOCKWISE = [1, 3, 8, 4, 2, 6, 7, 5]
PX4_DIRECTION = {
    1: "CW",
    2: "CW",
    3: "CCW",
    4: "CCW",
    5: "CCW",
    6: "CCW",
    7: "CW",
    8: "CW",
}


def read_binary_stl(path: Path) -> np.ndarray:
    data = path.read_bytes()
    if len(data) < 84:
        raise ValueError(f"{path} is too short to be a binary STL")
    triangle_count = struct.unpack_from("<I", data, 80)[0]
    expected_size = 84 + triangle_count * 50
    if len(data) != expected_size:
        raise ValueError(
            f"{path} is not the expected binary STL layout "
            f"({len(data)} bytes, expected {expected_size})"
        )
    dtype = np.dtype(
        [
            ("normal", "<f4", (3,)),
            ("vertices", "<f4", (3, 3)),
            ("attribute", "<u2"),
        ]
    )
    return np.frombuffer(
        data, dtype=dtype, offset=84, count=triangle_count
    )["vertices"].astype(float)


def connected_components(triangles: np.ndarray) -> list[np.ndarray]:
    parent = np.arange(len(triangles))

    def find(index: int) -> int:
        while parent[index] != index:
            parent[index] = parent[parent[index]]
            index = int(parent[index])
        return index

    def union(left: int, right: int) -> None:
        left_root, right_root = find(left), find(right)
        if left_root != right_root:
            parent[right_root] = left_root

    owner: dict[tuple[float, float, float], int] = {}
    for triangle_index, triangle in enumerate(triangles):
        for vertex in triangle:
            key = tuple(vertex)
            if key in owner:
                union(triangle_index, owner[key])
            else:
                owner[key] = triangle_index

    groups: dict[int, list[int]] = {}
    for triangle_index in range(len(triangles)):
        groups.setdefault(find(triangle_index), []).append(triangle_index)
    return [np.asarray(indices) for indices in groups.values()]


def select_mount_components(
    components: list[np.ndarray],
) -> list[np.ndarray]:
    face_counts = Counter(len(component) for component in components)
    candidates = [
        face_count
        for face_count, repetitions in face_counts.items()
        if repetitions == 8
    ]
    if len(candidates) != 1:
        raise ValueError(
            "Expected exactly one triangle-count class repeated eight times; "
            f"found {face_counts}"
        )
    return [
        component
        for component in components
        if len(component) == candidates[0]
    ]


def mount_frame(triangles: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    points = np.unique(triangles.reshape(-1, 3), axis=0)
    covariance = np.cov((points - points.mean(axis=0)).T)
    eigenvalues, eigenvectors = np.linalg.eigh(covariance)
    axis = eigenvectors[:, int(np.argmax(eigenvalues))]
    if axis[2] < 0:
        axis = -axis
    axis /= np.linalg.norm(axis)

    axial_position = points @ axis
    upper_plane = points[
        np.isclose(axial_position, axial_position.max(), atol=1e-6)
    ]
    if len(upper_plane) < 8:
        raise ValueError("Could not identify a circular upper mounting plane")
    return upper_plane.mean(axis=0), axis


def extract(path: Path) -> list[dict]:
    triangles = read_binary_stl(path)
    components = connected_components(triangles)
    mounts = select_mount_components(components)
    frames = [mount_frame(triangles[component]) for component in mounts]

    # Clockwise angle in PX4 FRD: FRD y (right) equals negative FLU y.
    frames.sort(
        key=lambda frame: float(
            np.arctan2(-frame[0][1], frame[0][0]) % (2 * np.pi)
        )
    )
    result = []
    for motor, (position_flu, axis_flu) in zip(
        PX4_ORDER_CLOCKWISE, frames, strict=True
    ):
        position_frd = position_flu * np.array([1.0, -1.0, -1.0])
        axis_frd = axis_flu * np.array([1.0, -1.0, -1.0])
        result.append(
            {
                "motor": motor,
                "direction": PX4_DIRECTION[motor],
                "position_flu_m": position_flu.tolist(),
                "axis_flu": axis_flu.tolist(),
                "position_frd_m": position_frd.tolist(),
                "axis_frd": axis_frd.tolist(),
                "tilt_deg": float(
                    np.degrees(np.arccos(np.clip(axis_flu[2], -1.0, 1.0)))
                ),
            }
        )
    return result


def main() -> None:
    package = Path(__file__).resolve().parents[1]
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--mesh",
        type=Path,
        default=package / "meshes" / "drone_body.stl",
    )
    parser.add_argument("--output", type=Path)
    args = parser.parse_args()
    result = {
        "source_mesh": str(args.mesh),
        "coordinate_convention": {
            "mesh": "Gazebo/ROS FLU",
            "px4": "FRD",
            "body_forward": "+X (arm extension direction)",
        },
        "mounts": extract(args.mesh),
    }
    text = json.dumps(result, indent=2) + "\n"
    if args.output:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(text, encoding="utf-8")
    else:
        print(text, end="")


if __name__ == "__main__":
    main()
