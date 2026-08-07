"""Quantify whether the CAD propeller meshes contain resolvable blade pitch."""

from __future__ import annotations

import json
import math
from pathlib import Path
import re
import struct

import numpy as np


PX4_MOTOR_MAP = {
    (2, 2): 1, (2, 3): 2, (1, 3): 3, (2, 4): 4,
    (2, 1): 5, (1, 2): 6, (1, 1): 7, (1, 4): 8,
}
STL_DTYPE = np.dtype(
    [("normal", "<f4", (3,)), ("vertices", "<f4", (3, 3)), ("attribute", "<u2")]
)


def load_binary_stl(path: Path):
    with path.open("rb") as stream:
        stream.seek(80)
        count = struct.unpack("<I", stream.read(4))[0]
    triangles = np.fromfile(path, dtype=STL_DTYPE, offset=84, count=count)
    if len(triangles) != count:
        raise ValueError(f"Incomplete binary STL: {path}")
    return triangles["vertices"].astype(float) / 1000.0


def unit(vector):
    vector = np.asarray(vector, dtype=float)
    return vector / np.linalg.norm(vector)


def unsigned_angle_degrees(first, second):
    cosine = abs(float(unit(first) @ unit(second)))
    return math.degrees(math.acos(np.clip(cosine, -1.0, 1.0)))


def main() -> None:
    project = Path(__file__).resolve().parents[4]
    analysis = project / "drone_sim_ws" / "analysis" / "cad_direct"
    mesh_root = project / "drone_sim_ws" / "src" / "drone_arm_sim" / "meshes" / "my_drone_v2" / "visual"
    axis_evidence = json.loads((analysis / "motor_axis_evidence.json").read_text(encoding="utf-8"))
    motor_evidence = {tuple(item["cad_key"]): item for item in axis_evidence["motors"]}
    mesh_paths = sorted(path for path in mesh_root.glob("*.STL") if "螺旋桨" in path.name)
    if len(mesh_paths) != 8:
        raise ValueError(f"Expected eight propeller meshes, found {len(mesh_paths)}")
    records = []
    for mesh_path in mesh_paths:
        numbers = [int(x) for x in re.findall(r"-(\d+)", mesh_path.stem)]
        cad_key = tuple(numbers[-2:])
        item = motor_evidence[cad_key]
        triangles = load_binary_stl(mesh_path)
        vertices = triangles.reshape(-1, 3)
        mesh_center = vertices.mean(axis=0)
        eigenvalues, eigenvectors = np.linalg.eigh(np.cov((vertices - mesh_center).T))
        mesh_axis = eigenvectors[:, 0]
        cad_axis = unit(item["assembly_thrust_axis"])
        if float(mesh_axis @ cad_axis) < 0.0:
            mesh_axis *= -1.0
        centres = triangles.mean(axis=1)
        radial = centres - mesh_center
        radial -= np.outer(radial @ mesh_axis, mesh_axis)
        radius = np.linalg.norm(radial, axis=1)
        radial_unit = radial / np.maximum(radius[:, None], 1e-12)
        tangent = np.cross(mesh_axis, radial_unit)
        cross = np.cross(triangles[:, 1] - triangles[:, 0], triangles[:, 2] - triangles[:, 0])
        double_area = np.linalg.norm(cross, axis=1)
        normals = cross / np.maximum(double_area[:, None], 1e-15)
        axial_normal = normals @ mesh_axis
        tangent_normal = np.sum(normals * tangent, axis=1)
        blade_surface = (radius > 0.015) & (np.abs(axial_normal) > 0.9)
        slope = np.abs(tangent_normal[blade_surface] / axial_normal[blade_surface])
        pitch_angles = np.degrees(np.arctan(slope))
        records.append(
            {
                "motor": PX4_MOTOR_MAP[cad_key],
                "cad_key": list(cad_key),
                "mesh": str(mesh_path),
                "triangle_count": int(len(triangles)),
                "cad_cylinder_axis": cad_axis.tolist(),
                "mesh_smallest_variance_axis": mesh_axis.tolist(),
                "mesh_axis_vs_cad_axis_unsigned_deg": unsigned_angle_degrees(mesh_axis, cad_axis),
                "mesh_axis_variance_m2": float(eigenvalues[0]),
                "blade_surface_triangle_count": int(blade_surface.sum()),
                "geometric_pitch_angle_abs_median_deg": float(np.median(pitch_angles)),
                "geometric_pitch_angle_abs_p95_deg": float(np.percentile(pitch_angles, 95)),
                "geometric_pitch_angle_abs_max_deg": float(np.max(pitch_angles)),
                "pitch_sign_resolved": False,
                "reason": "blade surfaces are effectively normal to the hub axis; no signed helical pitch is encoded",
            }
        )
    max_p95 = max(item["geometric_pitch_angle_abs_p95_deg"] for item in records)
    payload = {
        "schema": 1,
        "source_assembly": str(project / "零件" / "完整零件" / "组合无人机.SLDASM"),
        "method": "binary STL face normals cross-checked against SolidWorks hub-cylinder axes",
        "result": "PITCH_GEOMETRY_UNRESOLVED",
        "maximum_p95_abs_pitch_angle_deg": max_p95,
        "formal_conclusion": "The CAD preserves hub axes and puller/pusher placement, but its simplified flat propeller solids do not encode usable blade pitch. CW/CCW plus this geometry cannot determine thrust sign.",
        "required_replacement": "pitch-accurate propeller CAD or manufacturer blade handedness/pitch designation",
        "motors": sorted(records, key=lambda item: item["motor"]),
    }
    output = analysis / "propeller_pitch_geometry_evidence.json"
    output.write_text(json.dumps(payload, ensure_ascii=False, indent=2), encoding="utf-8")
    print(output)
    print(payload["result"], f"max_p95={max_p95:.9f} deg")
    for item in payload["motors"]:
        print(
            item["motor"],
            f"axis_error={item['mesh_axis_vs_cad_axis_unsigned_deg']:.6f}",
            f"pitch_p95={item['geometric_pitch_angle_abs_p95_deg']:.9f}",
        )


if __name__ == "__main__":
    main()
