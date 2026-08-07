"""URDF-based forward kinematics, Jacobian and center-of-mass checks."""

from __future__ import annotations

import argparse
from pathlib import Path
import xml.etree.ElementTree as ET

import numpy as np


def _vector(text: str | None, default: tuple[float, float, float]) -> np.ndarray:
    if not text:
        return np.asarray(default, dtype=float)
    return np.asarray([float(value) for value in text.split()], dtype=float)


def _rpy_matrix(rpy: np.ndarray) -> np.ndarray:
    roll, pitch, yaw = rpy
    cr, sr = np.cos(roll), np.sin(roll)
    cp, sp = np.cos(pitch), np.sin(pitch)
    cy, sy = np.cos(yaw), np.sin(yaw)
    rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])
    ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
    rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]])
    return rz @ ry @ rx


def _transform(xyz: np.ndarray, rpy: np.ndarray) -> np.ndarray:
    result = np.eye(4)
    result[:3, :3] = _rpy_matrix(rpy)
    result[:3, 3] = xyz
    return result


def _axis_rotation(axis: np.ndarray, angle: float) -> np.ndarray:
    axis = axis / np.linalg.norm(axis)
    x, y, z = axis
    c, s = np.cos(angle), np.sin(angle)
    cross = np.array([[0, -z, y], [z, 0, -x], [-y, x, 0]])
    rotation = c * np.eye(3) + (1 - c) * np.outer(axis, axis) + s * cross
    result = np.eye(4)
    result[:3, :3] = rotation
    return result


class UrdfModel:
    """Small dependency-free URDF tree evaluator for model validation."""

    def __init__(self, path: Path):
        self.path = path
        root = ET.parse(path).getroot()
        self.name = root.attrib["name"]
        self.links = {link.attrib["name"]: link for link in root.findall("link")}
        self.joints = {joint.attrib["name"]: joint for joint in root.findall("joint")}
        self.parent_joint = {
            joint.find("child").attrib["link"]: joint for joint in self.joints.values()
        }
        self.children: dict[str, list[ET.Element]] = {}
        for joint in self.joints.values():
            parent = joint.find("parent").attrib["link"]
            self.children.setdefault(parent, []).append(joint)
        child_links = set(self.parent_joint)
        roots = set(self.links) - child_links
        if len(roots) != 1:
            raise ValueError(f"Expected one root link, found {sorted(roots)}")
        self.root_link = roots.pop()

    @staticmethod
    def joint_origin(joint: ET.Element) -> np.ndarray:
        origin = joint.find("origin")
        if origin is None:
            return np.eye(4)
        return _transform(
            _vector(origin.attrib.get("xyz"), (0.0, 0.0, 0.0)),
            _vector(origin.attrib.get("rpy"), (0.0, 0.0, 0.0)),
        )

    def chain(self, end_link: str) -> list[ET.Element]:
        result = []
        current = end_link
        while current != self.root_link:
            joint = self.parent_joint.get(current)
            if joint is None:
                raise ValueError(f"{end_link} is not connected to {self.root_link}")
            result.append(joint)
            current = joint.find("parent").attrib["link"]
        return list(reversed(result))

    def joint_limits(self, joint_name: str) -> tuple[float, float]:
        joint = self.joints[joint_name]
        if joint.attrib["type"] == "continuous":
            return -np.inf, np.inf
        limit = joint.find("limit")
        if limit is None:
            return -np.inf, np.inf
        return (
            float(limit.attrib.get("lower", "-inf")),
            float(limit.attrib.get("upper", "inf")),
        )

    def link_transforms(self, positions: dict[str, float]) -> dict[str, np.ndarray]:
        """Return every link pose relative to the root link."""
        transforms: dict[str, np.ndarray] = {}

        def visit(link_name: str, link_transform: np.ndarray) -> None:
            transforms[link_name] = link_transform
            for joint in self.children.get(link_name, []):
                child_transform = link_transform @ self.joint_origin(joint)
                if joint.attrib["type"] in {"revolute", "continuous"}:
                    axis = _vector(
                        joint.find("axis").attrib.get("xyz"), (1.0, 0.0, 0.0)
                    )
                    child_transform = child_transform @ _axis_rotation(
                        axis, positions.get(joint.attrib["name"], 0.0)
                    )
                visit(joint.find("child").attrib["link"], child_transform)

        visit(self.root_link, np.eye(4))
        return transforms

    def forward_kinematics(
        self, end_link: str, positions: dict[str, float]
    ) -> tuple[np.ndarray, list[tuple[str, np.ndarray, np.ndarray]]]:
        transform = np.eye(4)
        active = []
        for joint in self.chain(end_link):
            transform = transform @ self.joint_origin(joint)
            joint_type = joint.attrib["type"]
            if joint_type in {"revolute", "continuous"}:
                axis = _vector(joint.find("axis").attrib.get("xyz"), (1.0, 0.0, 0.0))
                axis_world = transform[:3, :3] @ axis
                position_world = transform[:3, 3].copy()
                active.append((joint.attrib["name"], position_world, axis_world))
                transform = transform @ _axis_rotation(
                    axis, positions.get(joint.attrib["name"], 0.0)
                )
        return transform, active

    def jacobian(self, end_link: str, positions: dict[str, float]) -> np.ndarray:
        transform, active = self.forward_kinematics(end_link, positions)
        end_position = transform[:3, 3]
        columns = []
        for _, joint_position, axis_world in active:
            linear = np.cross(axis_world, end_position - joint_position)
            columns.append(np.concatenate((linear, axis_world)))
        return np.column_stack(columns)

    def inertial_entries(
        self, positions: dict[str, float]
    ) -> list[tuple[str, float, np.ndarray, np.ndarray]]:
        """Return (link, mass, CoM position, CoM inertia) in the root frame."""
        entries = []
        for link_name, link_transform in self.link_transforms(positions).items():
            inertial = self.links[link_name].find("inertial")
            if inertial is None:
                continue
            mass = float(inertial.find("mass").attrib["value"])
            if mass <= 0:
                continue
            origin = inertial.find("origin")
            inertial_transform = np.eye(4)
            if origin is not None:
                inertial_transform = _transform(
                    _vector(origin.attrib.get("xyz"), (0.0, 0.0, 0.0)),
                    _vector(origin.attrib.get("rpy"), (0.0, 0.0, 0.0)),
                )
            inertia_node = inertial.find("inertia")
            inertia_local = np.array(
                [
                    [
                        float(inertia_node.attrib["ixx"]),
                        float(inertia_node.attrib["ixy"]),
                        float(inertia_node.attrib["ixz"]),
                    ],
                    [
                        float(inertia_node.attrib["ixy"]),
                        float(inertia_node.attrib["iyy"]),
                        float(inertia_node.attrib["iyz"]),
                    ],
                    [
                        float(inertia_node.attrib["ixz"]),
                        float(inertia_node.attrib["iyz"]),
                        float(inertia_node.attrib["izz"]),
                    ],
                ]
            )
            com_transform = link_transform @ inertial_transform
            rotation = com_transform[:3, :3]
            inertia_root = rotation @ inertia_local @ rotation.T
            entries.append(
                (link_name, mass, com_transform[:3, 3].copy(), inertia_root)
            )
        return entries

    def mass_properties(
        self, positions: dict[str, float]
    ) -> tuple[float, np.ndarray, np.ndarray]:
        """Return total mass, CoM and inertia about CoM in the root frame."""
        entries = self.inertial_entries(positions)
        total_mass = sum(entry[1] for entry in entries)
        if total_mass <= 0:
            raise ValueError("URDF has no positive link mass")
        center = sum(entry[1] * entry[2] for entry in entries) / total_mass
        inertia = np.zeros((3, 3))
        for _, mass, link_center, link_inertia in entries:
            offset = link_center - center
            parallel_axis = mass * (
                np.dot(offset, offset) * np.eye(3) - np.outer(offset, offset)
            )
            inertia += link_inertia + parallel_axis
        return total_mass, center, inertia

    def center_of_mass(
        self, positions: dict[str, float]
    ) -> tuple[float, np.ndarray]:
        mass, center, _ = self.mass_properties(positions)
        return mass, center


def _default_urdf() -> Path:
    try:
        from ament_index_python.packages import get_package_share_directory

        return (
            Path(get_package_share_directory("drone_arm_sim"))
            / "urdf"
            / "drone_with_arm_controlled.urdf"
        )
    except Exception:
        return Path(__file__).resolve().parents[1] / "urdf" / "drone_with_arm_controlled.urdf"


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--urdf", type=Path, default=_default_urdf())
    parser.add_argument("--end-link", default="gripper_frame_link")
    parser.add_argument(
        "--joint",
        action="append",
        default=[],
        metavar="NAME=RAD",
        help="Set a joint angle; may be repeated.",
    )
    args = parser.parse_args()
    positions = {}
    for assignment in args.joint:
        name, value = assignment.split("=", 1)
        positions[name] = float(value)

    model = UrdfModel(args.urdf)
    end_transform, active = model.forward_kinematics(args.end_link, positions)
    jacobian = model.jacobian(args.end_link, positions)
    mass, center, inertia = model.mass_properties(positions)

    np.set_printoptions(precision=6, suppress=True)
    print(f"robot: {model.name}")
    print(f"root link: {model.root_link}")
    print(f"end link: {args.end_link}")
    print("active chain joints:", ", ".join(item[0] for item in active))
    print(f"total mass [kg]: {mass:.6f}")
    print("center of mass in base frame [m]:", center)
    print("composite inertia about CoM [kg m^2]:")
    print(inertia)
    print("end-effector transform:")
    print(end_transform)
    print("geometric Jacobian [linear; angular]:")
    print(jacobian)
    print(f"Jacobian rank: {np.linalg.matrix_rank(jacobian)}")


if __name__ == "__main__":
    main()
