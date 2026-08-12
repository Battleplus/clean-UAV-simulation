"""Runtime mass-property and reaction-wrench model for the CAD SO101 arm.

The Gazebo multibody solver already applies the inertial reaction of every
URDF link.  This module is the explicit, auditable coupling layer used for
feed-forward compensation and for comparing retracted/work/payload states.
It deliberately does not replace Gazebo's contact solver or duplicate its
forces: the returned wrench is an estimate of the wrench the moving arm
transmits to a base whose pose is held fixed.
"""

from __future__ import annotations

from dataclasses import dataclass
import json
from pathlib import Path
from typing import Mapping

import numpy as np

from drone_arm_sim.model_analysis import UrdfModel, _axis_rotation, _transform, _vector


JOINT_NAMES = (
    "shoulder_pan",
    "shoulder_lift",
    "elbow_flex",
    "wrist_flex",
    "wrist_roll",
    "gripper",
)


@dataclass(frozen=True)
class Payload:
    """Rigid payload attached to the gripper frame (all dimensions in SI)."""

    mass_kg: float
    offset_gripper_m: tuple[float, float, float] = (0.08, 0.0, 0.0)
    inertia_gripper_kg_m2: tuple[tuple[float, float, float], ...] = (
        (1.0e-4, 0.0, 0.0),
        (0.0, 1.0e-4, 0.0),
        (0.0, 0.0, 1.0e-4),
    )

    def __post_init__(self) -> None:
        if self.mass_kg < 0.0:
            raise ValueError("payload mass must be non-negative")


@dataclass(frozen=True)
class CoupledState:
    mass_kg: float
    center_of_mass_m: np.ndarray
    inertia_at_com_kg_m2: np.ndarray
    com_shift_m: np.ndarray
    reaction_force_body_n: np.ndarray
    reaction_torque_body_nm: np.ndarray
    damping_torque_body_nm: np.ndarray
    joint_resisting_torque_nm: np.ndarray


def _skew(vector: np.ndarray) -> np.ndarray:
    x, y, z = vector
    return np.array([[0.0, -z, y], [z, 0.0, -x], [-y, x, 0.0]])


def _vee(matrix: np.ndarray) -> np.ndarray:
    return np.array([matrix[2, 1], matrix[0, 2], matrix[1, 0]])


def _aggregate(
    entries: list[tuple[str, float, np.ndarray, np.ndarray]],
) -> tuple[float, np.ndarray, np.ndarray]:
    mass = sum(item[1] for item in entries)
    if mass <= 0.0:
        raise ValueError("aggregate model has no positive mass")
    center = sum((item[1] * item[2] for item in entries), start=np.zeros(3)) / mass
    inertia = np.zeros((3, 3))
    for _, item_mass, item_center, item_inertia in entries:
        offset = item_center - center
        inertia += item_inertia + item_mass * (
            np.dot(offset, offset) * np.eye(3) - np.outer(offset, offset)
        )
    return float(mass), center, 0.5 * (inertia + inertia.T)


class CoupledArmDynamics:
    """Evaluate CAD mass properties and arm-induced base reaction."""

    def __init__(
        self,
        urdf: Path,
        motion_reference: Path | Mapping | None = None,
        target_mass_kg: float | None = 7.735,
        finite_difference_step: float = 1.0e-5,
    ) -> None:
        self.model = UrdfModel(Path(urdf))
        self.finite_difference_step = float(finite_difference_step)
        if self.finite_difference_step <= 0.0:
            raise ValueError("finite_difference_step must be positive")
        if motion_reference is None:
            self.motion = {}
        elif isinstance(motion_reference, Mapping):
            self.motion = dict(motion_reference)
        else:
            self.motion = json.loads(Path(motion_reference).read_text(encoding="utf-8"))
        self.joints = {
            item["name"]: item
            for item in self.motion.get("joints", [])
            if isinstance(item, Mapping)
        }
        self._link_inertials = {}
        for link_name, link in self.model.links.items():
            inertial = link.find("inertial")
            if inertial is None:
                continue
            mass = float(inertial.find("mass").attrib["value"])
            if mass <= 0.0:
                continue
            origin = inertial.find("origin")
            local_transform = (
                np.eye(4)
                if origin is None
                else _transform(
                    _vector(origin.attrib.get("xyz"), (0.0, 0.0, 0.0)),
                    _vector(origin.attrib.get("rpy"), (0.0, 0.0, 0.0)),
                )
            )
            node = inertial.find("inertia")
            local_inertia = np.array(
                [
                    [float(node.attrib["ixx"]), float(node.attrib["ixy"]), float(node.attrib["ixz"])],
                    [float(node.attrib["ixy"]), float(node.attrib["iyy"]), float(node.attrib["iyz"])],
                    [float(node.attrib["ixz"]), float(node.attrib["iyz"]), float(node.attrib["izz"])],
                ]
            )
            self._link_inertials[link_name] = (mass, local_transform, local_inertia)
        active = set(self.active_joint_names)
        self._ancestor_joint_names = {
            link_name: tuple(
                joint.attrib["name"]
                for joint in self.model.chain(link_name)
                if joint.attrib["name"] in active
            )
            for link_name in self.model.links
        }
        self._children_kinematics = {}
        for parent, joints in self.model.children.items():
            prepared = []
            for joint in joints:
                axis_node = joint.find("axis")
                axis = _vector(
                    None if axis_node is None else axis_node.attrib.get("xyz"),
                    (1.0, 0.0, 0.0),
                )
                prepared.append(
                    (
                        joint.attrib["name"],
                        joint.find("child").attrib["link"],
                        joint.attrib["type"],
                        self.model.joint_origin(joint),
                        axis,
                    )
                )
            self._children_kinematics[parent] = tuple(prepared)
        base_mass, _, _ = self.model.mass_properties({})
        self.mass_scale = (
            float(target_mass_kg) / base_mass
            if target_mass_kg is not None
            else 1.0
        )
        if self.mass_scale <= 0.0:
            raise ValueError("target mass must be positive")
        _, self.home_com, self.home_inertia = self.mass_properties({})

    @property
    def active_joint_names(self) -> tuple[str, ...]:
        return tuple(name for name in JOINT_NAMES if name in self.model.joints)

    def _normalized_positions(self, positions: Mapping[str, float]) -> dict[str, float]:
        return {name: float(positions.get(name, 0.0)) for name in self.active_joint_names}

    def _entries(
        self, positions: Mapping[str, float], payload: Payload | None = None
    ) -> list[tuple[str, float, np.ndarray, np.ndarray]]:
        q = self._normalized_positions(positions)
        transforms, _ = self._tree_snapshot(q)
        return self._entries_from_transforms(transforms, payload)

    def _entries_from_transforms(
        self, transforms: Mapping[str, np.ndarray], payload: Payload | None
    ) -> list[tuple[str, float, np.ndarray, np.ndarray]]:
        entries = []
        for name, (mass, local_transform, local_inertia) in self._link_inertials.items():
            com_transform = transforms[name] @ local_transform
            rotation = com_transform[:3, :3]
            entries.append(
                (
                    name,
                    mass * self.mass_scale,
                    com_transform[:3, 3].copy(),
                    rotation @ (local_inertia * self.mass_scale) @ rotation.T,
                )
            )
        if payload is not None and payload.mass_kg > 0.0:
            gripper = transforms.get("gripper_link")
            if gripper is None:
                raise ValueError("payload requires a gripper_link in the URDF")
            local_offset = np.asarray(payload.offset_gripper_m, dtype=float)
            center = gripper[:3, :3] @ local_offset + gripper[:3, 3]
            inertia_local = np.asarray(payload.inertia_gripper_kg_m2, dtype=float)
            inertia = gripper[:3, :3] @ inertia_local @ gripper[:3, :3].T
            entries.append(("__payload__", float(payload.mass_kg), center, inertia))
        return entries

    def _tree_snapshot(
        self, positions: Mapping[str, float]
    ) -> tuple[dict[str, np.ndarray], dict[str, tuple[np.ndarray, np.ndarray]]]:
        """Evaluate link transforms and body-frame joint axes in one traversal."""
        transforms = {}
        axes = {}

        def visit(link_name: str, link_transform: np.ndarray) -> None:
            transforms[link_name] = link_transform
            for name, child, kind, origin, local_axis in self._children_kinematics.get(
                link_name, ()
            ):
                joint_transform = link_transform @ origin
                if kind in {"revolute", "continuous"}:
                    unit_axis = local_axis / np.linalg.norm(local_axis)
                    axes[name] = (
                        joint_transform[:3, 3].copy(),
                        joint_transform[:3, :3] @ unit_axis,
                    )
                    child_transform = joint_transform @ _axis_rotation(
                        local_axis, float(positions.get(name, 0.0))
                    )
                else:
                    child_transform = joint_transform
                visit(child, child_transform)

        visit(self.model.root_link, np.eye(4))
        return transforms, axes

    def mass_properties(
        self, positions: Mapping[str, float], payload: Payload | None = None
    ) -> tuple[float, np.ndarray, np.ndarray]:
        return _aggregate(self._entries(positions, payload))

    def _joint_axes(self, positions: Mapping[str, float]) -> dict[str, tuple[np.ndarray, np.ndarray]]:
        """Return each joint's axis and origin in the root/body frame."""
        q = self._normalized_positions(positions)
        result: dict[str, tuple[np.ndarray, np.ndarray]] = {}

        def visit(link_name: str, link_transform: np.ndarray) -> None:
            for joint in self.model.children.get(link_name, []):
                origin_transform = link_transform @ self.model.joint_origin(joint)
                joint_name = joint.attrib["name"]
                if joint.attrib["type"] in {"revolute", "continuous"}:
                    axis = _vector(
                        joint.find("axis").attrib.get("xyz"), (1.0, 0.0, 0.0)
                    )
                    result[joint_name] = (
                        origin_transform[:3, 3].copy(),
                        origin_transform[:3, :3] @ (axis / np.linalg.norm(axis)),
                    )
                    child_transform = origin_transform @ _axis_rotation(
                        axis, q.get(joint_name, 0.0)
                    )
                else:
                    child_transform = origin_transform
                visit(joint.find("child").attrib["link"], child_transform)

        visit(self.model.root_link, np.eye(4))
        return result

    def _kinematic_maps(self, positions: Mapping[str, float]):
        q = self._normalized_positions(positions)
        return (
            self.model.link_transforms(q),
            {item[0]: item for item in self.model.inertial_entries(q)},
        )

    @staticmethod
    def _jacobian_from_mapset(
        mapset, link_name: str, names: tuple[str, ...], h: float
    ) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
        """Finite-difference COM/orientation Jacobians from cached trees."""
        transforms, entries = mapset[0]
        center = entries[link_name][2]
        rotation = transforms[link_name][:3, :3]
        jv = np.zeros((3, len(names)))
        jw = np.zeros((3, len(names)))
        for index, name in enumerate(names):
            plus_transforms, plus_entries = mapset[1 + 2 * index]
            minus_transforms, minus_entries = mapset[2 + 2 * index]
            jv[:, index] = (plus_entries[link_name][2] - minus_entries[link_name][2]) / (2.0 * h)
            r_plus = plus_transforms[link_name][:3, :3]
            r_minus = minus_transforms[link_name][:3, :3]
            rotation_rate = ((r_plus - r_minus) / (2.0 * h)) @ rotation.T
            jw[:, index] = _vee(0.5 * (rotation_rate - rotation_rate.T))
        return center, jv, jw

    def _mapset(self, positions: Mapping[str, float]):
        """Cache one kinematic tree and all joint finite-difference trees."""
        q = self._normalized_positions(positions)
        result = [self._kinematic_maps(q)]
        h = self.finite_difference_step
        for name in self.active_joint_names:
            plus = dict(q); plus[name] += h
            minus = dict(q); minus[name] -= h
            result.extend((self._kinematic_maps(plus), self._kinematic_maps(minus)))
        return result

    def _analytical_snapshot(
        self, positions: Mapping[str, float], payload: Payload | None
    ):
        """Return inertial entries and exact geometric Jacobians for one pose.

        The original implementation rebuilt a complete URDF tree for every
        joint perturbation.  A revolute-joint COM Jacobian is available
        directly from the joint axis and origin, so one pose needs only one
        inertial traversal and one joint-axis traversal.
        """
        q = self._normalized_positions(positions)
        transforms, axes = self._tree_snapshot(q)
        entries = self._entries_from_transforms(transforms, payload)
        names = self.active_joint_names
        jacobians = {}
        for link_name, _, center, _ in entries:
            jacobian_link = "gripper_link" if link_name == "__payload__" else link_name
            ancestors = set(self._ancestor_joint_names[jacobian_link])
            jv = np.zeros((3, len(names)))
            jw = np.zeros((3, len(names)))
            for index, name in enumerate(names):
                if name not in ancestors:
                    continue
                origin, axis = axes[name]
                jv[:, index] = np.cross(axis, center - origin)
                jw[:, index] = axis
            jacobians[link_name] = (jv, jw)
        return entries, jacobians, axes

    def state(
        self,
        positions: Mapping[str, float],
        velocities: Mapping[str, float] | None = None,
        accelerations: Mapping[str, float] | None = None,
        payload: Payload | None = None,
    ) -> CoupledState:
        """Evaluate the coupled state using cached-tree geometric Jacobians."""
        q = self._normalized_positions(positions)
        names = self.active_joint_names
        qd = np.asarray([float((velocities or {}).get(name, 0.0)) for name in names])
        qdd = np.asarray([float((accelerations or {}).get(name, 0.0)) for name in names])
        h = self.finite_difference_step

        entries, jacobians, axes = self._analytical_snapshot(q, payload)
        q_plus_velocity = {
            name: q[name] + h * qd[index] for index, name in enumerate(names)
        }
        q_minus_velocity = {
            name: q[name] - h * qd[index] for index, name in enumerate(names)
        }
        _, plus_jacobians, _ = self._analytical_snapshot(q_plus_velocity, payload)
        _, minus_jacobians, _ = self._analytical_snapshot(q_minus_velocity, payload)

        mass, center, inertia = _aggregate(entries)
        reaction_force = np.zeros(3)
        reaction_torque = np.zeros(3)
        for link_name, link_mass, link_center, link_inertia in entries:
            jv, jw = jacobians[link_name]
            jv_plus, jw_plus = plus_jacobians[link_name]
            jv_minus, jw_minus = minus_jacobians[link_name]
            djv_dt = (jv_plus - jv_minus) @ qd / (2.0 * h)
            djw_dt = (jw_plus - jw_minus) @ qd / (2.0 * h)
            linear_acceleration = jv @ qdd + djv_dt
            angular_velocity = jw @ qd
            angular_acceleration = jw @ qdd + djw_dt
            force = link_mass * linear_acceleration
            reaction_force -= force
            reaction_torque -= (
                np.cross(link_center, force)
                + link_inertia @ angular_acceleration
                + np.cross(angular_velocity, link_inertia @ angular_velocity)
            )

        d_torque = np.zeros(3)
        resisting = np.zeros(len(names))
        for index, name in enumerate(names):
            item = self.joints.get(name, {})
            damping = float(item.get("damping", 0.0))
            friction = float(item.get("friction", 0.0))
            resisting[index] = -damping * qd[index] - friction * np.tanh(
                qd[index] / 1.0e-3
            )
            axis = axes.get(name, (np.zeros(3), np.zeros(3)))[1]
            d_torque -= axis * resisting[index]
        reaction_torque += d_torque
        home_center = (
            self.home_com
            if payload is None
            else self.mass_properties({}, payload)[1]
        )
        return CoupledState(
            mass_kg=mass,
            center_of_mass_m=center,
            inertia_at_com_kg_m2=inertia,
            com_shift_m=center - home_center,
            reaction_force_body_n=reaction_force,
            reaction_torque_body_nm=reaction_torque,
            damping_torque_body_nm=d_torque,
            joint_resisting_torque_nm=resisting,
        )

    def _state_finite_difference_reference(
        self,
        positions: Mapping[str, float],
        velocities: Mapping[str, float] | None = None,
        accelerations: Mapping[str, float] | None = None,
        payload: Payload | None = None,
    ) -> CoupledState:
        q = self._normalized_positions(positions)
        qd = np.asarray([float((velocities or {}).get(name, 0.0)) for name in self.active_joint_names])
        qdd = np.asarray([float((accelerations or {}).get(name, 0.0)) for name in self.active_joint_names])
        mass, center, inertia = self.mass_properties(q, payload)
        reaction_force = np.zeros(3)
        reaction_torque = np.zeros(3)
        d_torque = np.zeros(3)
        resisting = np.zeros(len(self.active_joint_names))
        axes = self._joint_axes(q)
        h = self.finite_difference_step
        names = self.active_joint_names
        base_mapset = self._mapset(q)
        q_plus_velocity = {
            name: q[name] + h * qd[index] for index, name in enumerate(names)
        }
        q_minus_velocity = {
            name: q[name] - h * qd[index] for index, name in enumerate(names)
        }
        plus_mapset = self._mapset(q_plus_velocity)
        minus_mapset = self._mapset(q_minus_velocity)
        for link_name, link_mass, link_center, link_inertia in self._entries(q, payload):
            # Payload uses the gripper Jacobian plus a rigid local offset.
            jacobian_link = "gripper_link" if link_name == "__payload__" else link_name
            _, jv, jw = self._jacobian_from_mapset(base_mapset, jacobian_link, names, h)
            _, jv_plus, jw_plus = self._jacobian_from_mapset(plus_mapset, jacobian_link, names, h)
            _, jv_minus, jw_minus = self._jacobian_from_mapset(minus_mapset, jacobian_link, names, h)
            if link_name == "__payload__":
                offset = np.asarray(payload.offset_gripper_m, dtype=float)
                transform = base_mapset[0][0]["gripper_link"]
                transform_plus = plus_mapset[0][0]["gripper_link"]
                transform_minus = minus_mapset[0][0]["gripper_link"]
                jv = jv - _skew(transform[:3, :3] @ offset) @ jw
                jv_plus = jv_plus - _skew(transform_plus[:3, :3] @ offset) @ jw_plus
                jv_minus = jv_minus - _skew(transform_minus[:3, :3] @ offset) @ jw_minus
            djv_dt = (jv_plus - jv_minus) @ qd / (2.0 * h)
            djw_dt = (jw_plus - jw_minus) @ qd / (2.0 * h)
            linear_acceleration = jv @ qdd + djv_dt
            angular_velocity = jw @ qd
            angular_acceleration = jw @ qdd + djw_dt
            force = link_mass * linear_acceleration
            reaction_force -= force
            reaction_torque -= (
                np.cross(link_center, force)
                + link_inertia @ angular_acceleration
                + np.cross(angular_velocity, link_inertia @ angular_velocity)
            )

        for index, name in enumerate(self.active_joint_names):
            item = self.joints.get(name, {})
            damping = float(item.get("damping", 0.0))
            friction = float(item.get("friction", 0.0))
            resisting[index] = -damping * qd[index] - friction * np.tanh(qd[index] / 1.0e-3)
            axis = axes.get(name, (np.zeros(3), np.zeros(3)))[1]
            # Equal/opposite internal joint torque transmitted to the base.
            d_torque -= axis * resisting[index]
        reaction_torque += d_torque
        _, home_center, _ = self.mass_properties({}, payload)
        return CoupledState(
            mass_kg=mass,
            center_of_mass_m=center,
            inertia_at_com_kg_m2=inertia,
            com_shift_m=center - home_center,
            reaction_force_body_n=reaction_force,
            reaction_torque_body_nm=reaction_torque,
            damping_torque_body_nm=d_torque,
            joint_resisting_torque_nm=resisting,
        )

    def contact_delta_twist(
        self,
        positions: Mapping[str, float],
        impulse_body_ns: np.ndarray,
        payload: Payload | None = None,
    ) -> np.ndarray:
        """Return free-body [linear, angular] velocity jump from a contact impulse."""
        impulse = np.asarray(impulse_body_ns, dtype=float)
        if impulse.shape != (3,):
            raise ValueError("impulse_body_ns must have shape (3,)")
        mass, center, inertia = self.mass_properties(positions, payload)
        if payload is None:
            point = center
        else:
            transform = self.model.link_transforms(self._normalized_positions(positions))["gripper_link"]
            point = transform[:3, :3] @ np.asarray(payload.offset_gripper_m) + transform[:3, 3]
        angular = np.linalg.solve(inertia, np.cross(point - center, impulse))
        return np.concatenate((impulse / mass, angular))


def preset_state(
    dynamics: CoupledArmDynamics,
    preset: str,
    reference: Mapping,
    payload: Payload | None = None,
) -> CoupledState:
    target = reference["presets"][preset]
    positions = dict(zip(JOINT_NAMES, target))
    return dynamics.state(positions, payload=payload)
