#!/usr/bin/env python3
"""Build a non-destructive rigid-payload variant of the formal CAD URDF."""

from __future__ import annotations

import argparse
from pathlib import Path
import xml.etree.ElementTree as ET

import numpy as np


def payload_fragment(mass_kg: float, size_m: float, offset: tuple[float, float, float]) -> str:
    if mass_kg <= 0.0:
        raise ValueError("payload mass must be positive")
    if size_m <= 0.0:
        raise ValueError("payload size must be positive")
    x, y, z = offset
    inertia = mass_kg * size_m * size_m / 6.0
    return f"""
  <!-- Generic rigid payload fixture; not a measured cleaning-tool model. -->
  <link name="test_payload_link">
    <inertial>
      <origin xyz="0 0 0" rpy="0 0 0" />
      <mass value="{mass_kg:.12g}" />
      <inertia ixx="{inertia:.12g}" ixy="0" ixz="0"
               iyy="{inertia:.12g}" iyz="0" izz="{inertia:.12g}" />
    </inertial>
    <visual name="test_payload_visual">
      <geometry><box size="{size_m:.12g} {size_m:.12g} {size_m:.12g}" /></geometry>
      <material name="test_payload_material"><color rgba="0.15 0.55 0.95 1" /></material>
    </visual>
    <collision name="test_payload_collision">
      <geometry><box size="{size_m:.12g} {size_m:.12g} {size_m:.12g}" /></geometry>
    </collision>
  </link>
  <joint name="test_payload_fixed_joint" type="fixed">
    <origin xyz="{x:.12g} {y:.12g} {z:.12g}" rpy="0 0 0" />
    <parent link="gripper_link" />
    <child link="test_payload_link" />
  </joint>
  <gazebo reference="test_payload_fixed_joint"><preserveFixedJoint>true</preserveFixedJoint></gazebo>
"""


def _inertia_matrix(element: ET.Element) -> np.ndarray:
    return np.array([
        [float(element.get("ixx")), float(element.get("ixy")), float(element.get("ixz"))],
        [float(element.get("ixy")), float(element.get("iyy")), float(element.get("iyz"))],
        [float(element.get("ixz")), float(element.get("iyz")), float(element.get("izz"))],
    ])


def _parallel_axis(mass: float, displacement: np.ndarray) -> np.ndarray:
    return mass * (
        float(displacement @ displacement) * np.eye(3)
        - np.outer(displacement, displacement)
    )


def _merge_into_gripper(text: str, mass_kg: float, size_m: float,
                        offset: tuple[float, float, float]) -> str:
    root = ET.fromstring(text)
    gripper = root.find("./link[@name='gripper_link']")
    if gripper is None:
        raise ValueError("source has no gripper_link")
    inertial = gripper.find("inertial")
    if inertial is None:
        raise ValueError("gripper_link has no inertial element")
    mass_node = inertial.find("mass")
    origin_node = inertial.find("origin")
    inertia_node = inertial.find("inertia")
    if mass_node is None or origin_node is None or inertia_node is None:
        raise ValueError("gripper_link inertial is incomplete")
    original_mass = float(mass_node.get("value"))
    original_center = np.fromstring(origin_node.get("xyz", ""), sep=" ")
    if original_center.shape != (3,):
        raise ValueError("gripper inertial origin must have three coordinates")
    payload_center = np.asarray(offset, dtype=float)
    combined_mass = original_mass + mass_kg
    combined_center = (
        original_mass * original_center + mass_kg * payload_center
    ) / combined_mass
    payload_center_inertia = np.eye(3) * mass_kg * size_m * size_m / 6.0
    combined_inertia = (
        _inertia_matrix(inertia_node)
        + _parallel_axis(original_mass, original_center - combined_center)
        + payload_center_inertia
        + _parallel_axis(mass_kg, payload_center - combined_center)
    )
    mass_node.set("value", f"{combined_mass:.12g}")
    origin_node.set("xyz", " ".join(f"{value:.12g}" for value in combined_center))
    for row, column, name in (
        (0, 0, "ixx"), (0, 1, "ixy"), (0, 2, "ixz"),
        (1, 1, "iyy"), (1, 2, "iyz"), (2, 2, "izz"),
    ):
        inertia_node.set(name, f"{combined_inertia[row, column]:.12g}")
    xyz = " ".join(f"{value:.12g}" for value in payload_center)
    visual = ET.SubElement(gripper, "visual", {"name": "test_payload_visual"})
    ET.SubElement(visual, "origin", {"xyz": xyz, "rpy": "0 0 0"})
    geometry = ET.SubElement(visual, "geometry")
    ET.SubElement(geometry, "box", {"size": f"{size_m} {size_m} {size_m}"})
    material = ET.SubElement(visual, "material", {"name": "test_payload_material"})
    ET.SubElement(material, "color", {"rgba": "0.15 0.55 0.95 1"})
    collision = ET.SubElement(gripper, "collision", {"name": "test_payload_collision"})
    ET.SubElement(collision, "origin", {"xyz": xyz, "rpy": "0 0 0"})
    geometry = ET.SubElement(collision, "geometry")
    ET.SubElement(geometry, "box", {"size": f"{size_m} {size_m} {size_m}"})
    ET.indent(root, space="  ")
    return ET.tostring(root, encoding="unicode")


def build(source: Path, output: Path, mass_kg: float, size_m: float,
          offset: tuple[float, float, float], attachment_mode: str) -> None:
    text = source.read_text(encoding="utf-8")
    if "test_payload_" in text:
        raise ValueError("source already contains the test payload fixture")
    if attachment_mode == "merged":
        result = _merge_into_gripper(text, mass_kg, size_m, offset)
    else:
        marker = "</robot>"
        if text.count(marker) != 1:
            raise ValueError("source must contain exactly one closing robot tag")
        result = text.replace(marker, payload_fragment(mass_kg, size_m, offset) + marker)
    ET.fromstring(result)
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(result, encoding="utf-8")


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--source", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--mass-kg", type=float, default=0.25)
    parser.add_argument("--size-m", type=float, default=0.06)
    parser.add_argument("--offset", type=float, nargs=3, default=(0.08, 0.0, 0.0))
    parser.add_argument(
        "--attachment-mode", choices=("merged", "fixed"), default="merged"
    )
    args = parser.parse_args()
    build(
        args.source, args.output, args.mass_kg, args.size_m,
        tuple(args.offset), args.attachment_mode
    )
    print(
        f"PAYLOAD_URDF_READY path={args.output} "
        f"payload_mass_kg={args.mass_kg:.6f} mode={args.attachment_mode} "
        "total_mass_must_be_recomputed_from_output_urdf=true"
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
