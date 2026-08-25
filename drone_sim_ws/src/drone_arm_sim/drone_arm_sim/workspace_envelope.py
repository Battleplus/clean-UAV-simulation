"""Offline flight-feasibility envelope for the articulated SO101 arm.

The runtime compensation node is deliberately action agnostic.  This module
uses the same CAD mass model and the same 6x8 rotor allocator to decide which
static arm poses leave enough motor and compensation authority for flight.
It also performs a deterministic oriented-box collision proxy check from the
URDF.  The proxy is intentionally conservative and its calibrated exclusions
are written into every report; it is not presented as a replacement for a
mesh/contact simulation.
"""

from __future__ import annotations

from dataclasses import dataclass
import hashlib
from itertools import combinations, product
import json
from pathlib import Path
import re
import struct
from typing import Iterable, Mapping
import xml.etree.ElementTree as ET

import numpy as np
from scipy.optimize import lsq_linear

from drone_arm_sim.allocation_analysis import allocation_matrix
from drone_arm_sim.base1_wrench_reallocator import (
    FLU_TO_FRD,
    allocate_total_wrench,
    config_thrust_n_to_commands,
)
from drone_arm_sim.coupled_dynamics import CoupledArmDynamics
from drone_arm_sim.model_analysis import _transform, _vector


DEFAULT_GRID_COUNTS = {
    "shoulder_pan": 7,
    "shoulder_lift": 7,
    "elbow_flex": 7,
    "wrist_flex": 5,
    "wrist_roll": 5,
    "gripper": 3,
}

HORIZONTAL_DIRECTIONS = (
    "front",
    "front_left",
    "left",
    "rear_left",
    "rear",
    "rear_right",
    "right",
    "front_right",
)
DIAGONAL_DIRECTIONS = (
    "front_left",
    "rear_left",
    "rear_right",
    "front_right",
)
RADIAL_BAND_LIMITS_M = (0.15, 0.25)
STRUCTURAL_PROXY_EXCLUSIONS = {
    ("arm_base_link", "upper_arm_link"),
    ("base_link", "shoulder_link"),
}


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with Path(path).open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


@dataclass(frozen=True)
class OrientedBox:
    link: str
    name: str
    local_transform: np.ndarray
    half_extent_m: np.ndarray
    source: str = "urdf_collision_box"


def _mesh_path(urdf: Path, filename: str) -> Path:
    prefix = "package://drone_arm_sim/"
    if not filename.startswith(prefix):
        raise ValueError(f"unsupported mesh URI for offline scan: {filename}")
    package = Path(urdf).resolve().parents[2]
    return package / filename[len(prefix):]


def _stl_vertices(path: Path) -> np.ndarray:
    """Read binary or ASCII STL vertices without adding a mesh dependency."""
    data = Path(path).read_bytes()
    if len(data) >= 84:
        triangle_count = struct.unpack_from("<I", data, 80)[0]
        if 84 + 50 * triangle_count == len(data):
            vertices = np.empty((triangle_count * 3, 3), dtype=float)
            cursor = 0
            for triangle in range(triangle_count):
                offset = 84 + 50 * triangle + 12
                for vertex in range(3):
                    vertices[cursor] = struct.unpack_from(
                        "<fff", data, offset + 12 * vertex
                    )
                    cursor += 1
            return vertices
    text = data.decode("utf-8", errors="ignore")
    matches = re.findall(
        r"\bvertex\s+([-+0-9.eE]+)\s+([-+0-9.eE]+)\s+([-+0-9.eE]+)",
        text,
    )
    if not matches:
        raise ValueError(f"STL contains no readable vertices: {path}")
    return np.asarray(matches, dtype=float)


def _rotor_swept_volume_proxies(
    urdf: Path, clearance_m: float = 0.005
) -> tuple[OrientedBox, ...]:
    """Create conservative swept-volume boxes from the eight rotor STL files.

    Rotor links have visual meshes but no URDF collision geometry.  Treating a
    blade at its zero angle as collision geometry would also miss most of its
    rotation.  The proxy therefore revolves every transformed mesh vertex
    about the rotor-link Z axis and encloses the resulting disc with an OBB.
    """
    root = ET.parse(urdf).getroot()
    result: list[OrientedBox] = []
    for link in root.findall("link"):
        link_name = link.attrib["name"]
        if re.fullmatch(r"rotor_[1-8]_link", link_name) is None:
            continue
        visual = link.find("visual")
        geometry = None if visual is None else visual.find("geometry")
        mesh = None if geometry is None else geometry.find("mesh")
        if visual is None or mesh is None:
            raise ValueError(f"{link_name} has no visual mesh for swept-volume proxy")
        origin = visual.find("origin")
        visual_transform = np.eye(4)
        if origin is not None:
            visual_transform = _transform(
                _vector(origin.attrib.get("xyz"), (0.0, 0.0, 0.0)),
                _vector(origin.attrib.get("rpy"), (0.0, 0.0, 0.0)),
            )
        scale = _vector(mesh.attrib.get("scale"), (1.0, 1.0, 1.0))
        vertices = _stl_vertices(_mesh_path(Path(urdf), mesh.attrib["filename"]))
        points = (visual_transform[:3, :3] @ (vertices * scale).T).T
        points += visual_transform[:3, 3]
        radial = float(np.max(np.linalg.norm(points[:, :2], axis=1)))
        z_min = float(np.min(points[:, 2]))
        z_max = float(np.max(points[:, 2]))
        if not np.isfinite(radial) or radial <= 0.0 or z_max <= z_min:
            raise ValueError(f"invalid swept-volume bounds for {link_name}")
        local = np.eye(4)
        local[2, 3] = 0.5 * (z_min + z_max)
        result.append(
            OrientedBox(
                link_name,
                f"{link_name}_swept_volume",
                local,
                np.asarray(
                    [
                        radial + clearance_m,
                        radial + clearance_m,
                        0.5 * (z_max - z_min) + clearance_m,
                    ]
                ),
                source="rotor_visual_swept_volume",
            )
        )
    if len(result) != 8:
        raise ValueError(f"expected 8 rotor swept-volume proxies, got {len(result)}")
    return tuple(result)


def _box_collision_proxies(urdf: Path) -> tuple[OrientedBox, ...]:
    root = ET.parse(urdf).getroot()
    result: list[OrientedBox] = []
    for link in root.findall("link"):
        link_name = link.attrib["name"]
        for index, collision in enumerate(link.findall("collision")):
            name = collision.attrib.get("name", f"collision_{index}")
            # This is a tabletop support aid, not aircraft self geometry.
            if "ground_support" in name:
                continue
            geometry = collision.find("geometry")
            box = None if geometry is None else geometry.find("box")
            if box is None:
                continue
            origin = collision.find("origin")
            local = np.eye(4)
            if origin is not None:
                local = _transform(
                    _vector(origin.attrib.get("xyz"), (0.0, 0.0, 0.0)),
                    _vector(origin.attrib.get("rpy"), (0.0, 0.0, 0.0)),
                )
            size = _vector(box.attrib.get("size"), (0.0, 0.0, 0.0))
            if np.any(size <= 0.0):
                raise ValueError(f"invalid collision box {link_name}/{name}")
            result.append(OrientedBox(link_name, name, local, 0.5 * size))
    return tuple(result)


def oriented_boxes_intersect(
    first_transform: np.ndarray,
    first_half_extent: np.ndarray,
    second_transform: np.ndarray,
    second_half_extent: np.ndarray,
    epsilon: float = 1.0e-10,
) -> bool:
    """Separating-axis test for two 3-D oriented boxes."""
    ra = np.asarray(first_transform, dtype=float)[:3, :3]
    rb = np.asarray(second_transform, dtype=float)[:3, :3]
    ca = np.asarray(first_transform, dtype=float)[:3, 3]
    cb = np.asarray(second_transform, dtype=float)[:3, 3]
    a = np.asarray(first_half_extent, dtype=float)
    b = np.asarray(second_half_extent, dtype=float)
    rotation = ra.T @ rb
    absolute = np.abs(rotation) + float(epsilon)
    translation = ra.T @ (cb - ca)

    for axis in range(3):
        if abs(translation[axis]) > a[axis] + float(absolute[axis] @ b):
            return False
    for axis in range(3):
        projected = abs(float(translation @ rotation[:, axis]))
        if projected > float(a @ absolute[:, axis]) + b[axis]:
            return False
    for i in range(3):
        i1, i2 = (i + 1) % 3, (i + 2) % 3
        for j in range(3):
            j1, j2 = (j + 1) % 3, (j + 2) % 3
            projected = abs(
                translation[i2] * rotation[i1, j]
                - translation[i1] * rotation[i2, j]
            )
            radius = (
                a[i1] * absolute[i2, j]
                + a[i2] * absolute[i1, j]
                + b[j1] * absolute[i, j2]
                + b[j2] * absolute[i, j1]
            )
            if projected > radius:
                return False
    return True


def _adjacent_link_pairs(model) -> set[tuple[str, str]]:
    result = set()
    for joint in model.joints.values():
        parent = joint.find("parent").attrib["link"]
        child = joint.find("child").attrib["link"]
        result.add(tuple(sorted((parent, child))))
    return result


def collision_pairs(
    dynamics: CoupledArmDynamics,
    boxes: Iterable[OrientedBox],
    positions: Mapping[str, float],
    ignored_link_pairs: set[tuple[str, str]] | None = None,
) -> tuple[tuple[str, str], ...]:
    """Return non-adjacent URDF box-proxy collision pairs for one pose."""
    ignored = set() if ignored_link_pairs is None else set(ignored_link_pairs)
    ignored |= _adjacent_link_pairs(dynamics.model)
    moving_links = {
        link
        for link, ancestors in dynamics._ancestor_joint_names.items()
        if ancestors
    }
    transforms = dynamics.model.link_transforms(dict(positions))
    world_boxes = [
        (item, transforms[item.link] @ item.local_transform) for item in boxes
    ]
    result = set()
    for (first, first_tf), (second, second_tf) in combinations(world_boxes, 2):
        if first.link == second.link:
            continue
        # Fixed aircraft geometry is allowed to overlap other fixed aircraft
        # proxies: those relationships do not change with an arm pose and are
        # not a workspace/self-collision decision.  Retain every pair that
        # contains at least one link downstream of an active arm joint.
        if first.link not in moving_links and second.link not in moving_links:
            continue
        pair = tuple(sorted((first.link, second.link)))
        if pair in ignored:
            continue
        if oriented_boxes_intersect(
            first_tf,
            first.half_extent_m,
            second_tf,
            second.half_extent_m,
        ):
            result.add(pair)
    return tuple(sorted(result))


def calibrated_proxy_exclusions(
    dynamics: CoupledArmDynamics,
    boxes: Iterable[OrientedBox],
    accepted_poses: Iterable[Mapping[str, float]],
) -> set[tuple[str, str]]:
    """Record coarse-box overlaps present in already validated CAD poses."""
    exclusions = _adjacent_link_pairs(dynamics.model)
    for pose in accepted_poses:
        exclusions.update(collision_pairs(dynamics, boxes, pose, exclusions))
    return exclusions


def _hover_baseline(config: dict, mass_kg: float, gravity_m_s2: float) -> dict:
    matrix = allocation_matrix(config, position_key="wrench_position_m")
    target = np.asarray([0.0, 0.0, -mass_kg * gravity_m_s2, 0.0, 0.0, 0.0])
    maximum = float(config["maximum_thrust_n"])
    solved = lsq_linear(matrix, target, bounds=(0.0, maximum), lsmr_tol="auto")
    thrust = np.asarray(solved.x, dtype=float)
    residual = matrix @ thrust - target
    if not solved.success or float(np.linalg.norm(residual)) > 1.0e-8:
        raise ValueError("4 kg baseline hover is not exactly allocatable")
    return {
        "target_wrench_frd": target,
        "thrust_config_order_n": thrust,
        "commands_motor_order": config_thrust_n_to_commands(config, thrust),
        "matrix": matrix,
        "matrix_pseudoinverse": np.linalg.pinv(matrix),
    }


def _horizontal_direction(position: np.ndarray, deadband_m: float = 0.025) -> str:
    x, y = float(position[0]), float(position[1])
    if np.hypot(x, y) <= deadband_m:
        return "center"
    angle = float(np.arctan2(y, x))
    sector = int(np.floor((angle + np.pi / 8.0) / (np.pi / 4.0))) % 8
    return (
        "front",
        "front_left",
        "left",
        "rear_left",
        "rear",
        "rear_right",
        "right",
        "front_right",
    )[sector]


def _vertical_direction(z_delta_m: float, deadband_m: float = 0.025) -> str:
    if z_delta_m > deadband_m:
        return "up"
    if z_delta_m < -deadband_m:
        return "down"
    return "level"


def _radial_band(radial_distance_m: float) -> str:
    near_limit, middle_limit = RADIAL_BAND_LIMITS_M
    if radial_distance_m <= near_limit:
        return "near"
    if radial_distance_m <= middle_limit:
        return "middle"
    return "far"


def _gripper_state(
    value_rad: float, lower_rad: float, upper_rad: float
) -> str:
    normalized = (value_rad - lower_rad) / max(1.0e-12, upper_rad - lower_rad)
    if normalized <= 1.0 / 3.0:
        return "closed"
    if normalized >= 2.0 / 3.0:
        return "open"
    return "middle"


def _static_joint_gravity_torque(
    dynamics: CoupledArmDynamics,
    positions: Mapping[str, float],
    gravity_m_s2: float,
) -> np.ndarray:
    """Generalized gravity load on each active joint from the CAD mass model."""
    normalized = dynamics._normalized_positions(positions)
    entries, jacobians, _ = dynamics._analytical_snapshot(normalized, None)
    torque = np.zeros(len(dynamics.active_joint_names))
    gravity = np.asarray([0.0, 0.0, -float(gravity_m_s2)])
    for link_name, mass_kg, _, _ in entries:
        linear_jacobian, _ = jacobians[link_name]
        torque += linear_jacobian.T @ (float(mass_kg) * gravity)
    return torque


def _pose_values(
    dynamics: CoupledArmDynamics,
    grid_counts: Mapping[str, int],
    limit_margin_fraction: float,
) -> Iterable[dict[str, float]]:
    names = dynamics.active_joint_names
    axes = []
    for name in names:
        lower, upper = dynamics.model.joint_limits(name)
        if not np.isfinite(lower) or not np.isfinite(upper):
            raise ValueError(f"finite joint limits required for {name}")
        margin = max(0.0, float(limit_margin_fraction)) * (upper - lower)
        count = max(2, int(grid_counts.get(name, 3)))
        axes.append(np.linspace(lower + margin, upper - margin, count))
    for values in product(*axes):
        yield {name: float(value) for name, value in zip(names, values, strict=True)}


def evaluate_static_pose(
    dynamics: CoupledArmDynamics,
    config: dict,
    baseline: dict,
    boxes: Iterable[OrientedBox],
    ignored_collision_pairs: set[tuple[str, str]],
    positions: Mapping[str, float],
    *,
    gravity_m_s2: float,
    gravity_torque_limit_nm: float,
    maximum_motor_delta_n: float,
    minimum_motor_headroom_n: float,
    minimum_delta_headroom_n: float,
    allocation_residual_limit: float,
    joint_effort_reserve_fraction: float,
) -> dict:
    mass, center, inertia = dynamics.mass_properties(positions)
    com_shift = center - dynamics.home_com
    gravity_force_flu = np.asarray([0.0, 0.0, -mass * gravity_m_s2])
    gravity_torque_flu = np.cross(com_shift, gravity_force_flu)
    compensation_frd = np.concatenate(
        (np.zeros(3), -FLU_TO_FRD @ gravity_torque_flu)
    )
    base_thrust = np.asarray(baseline["thrust_config_order_n"], dtype=float)
    maximum = float(config["maximum_thrust_n"])
    collisions = collision_pairs(
        dynamics, boxes, positions, ignored_collision_pairs
    )
    raw_torque_norm = float(np.linalg.norm(gravity_torque_flu))
    joint_gravity_torque = _static_joint_gravity_torque(
        dynamics, positions, gravity_m_s2
    )
    joint_effort_limits = np.asarray(
        [
            float(dynamics.joints.get(name, {}).get("effort_nm", np.inf))
            for name in dynamics.active_joint_names
        ]
    )
    usable_joint_effort = joint_effort_limits * (
        1.0 - float(joint_effort_reserve_fraction)
    )
    joint_effort_margin = usable_joint_effort - np.abs(joint_gravity_torque)
    joint_effort_feasible = bool(np.all(joint_effort_margin >= -1.0e-9))

    # Most static samples are exactly solved by the closest-to-Base-1
    # pseudoinverse increment.  Reuse that analytical solution and invoke the
    # bounded SciPy allocator only when a non-colliding, within-torque sample
    # needs nullspace redistribution at a motor bound.  This makes a 25k-pose
    # standard scan practical without changing the acceptance calculation.
    matrix = np.asarray(baseline["matrix"], dtype=float)
    candidate = base_thrust + np.asarray(
        baseline["matrix_pseudoinverse"], dtype=float
    ) @ compensation_frd
    lower = np.maximum(0.0, base_thrust - maximum_motor_delta_n)
    upper = np.minimum(maximum, base_thrust + maximum_motor_delta_n)
    direct_residual = float(
        np.linalg.norm(matrix @ candidate - (matrix @ base_thrust + compensation_frd))
    )
    direct_inside = bool(
        np.all(candidate >= lower - 1.0e-9)
        and np.all(candidate <= upper + 1.0e-9)
    )
    if (
        not direct_inside
        and not collisions
        and raw_torque_norm <= gravity_torque_limit_nm + 1.0e-9
    ):
        allocation = allocate_total_wrench(
            config,
            baseline["commands_motor_order"],
            compensation_frd,
            maximum_motor_delta_n=maximum_motor_delta_n,
        )
        thrust = np.asarray(allocation["thrust_config_order_n"], dtype=float)
        allocation_success = bool(allocation["success"])
        residual = float(allocation["residual_norm"])
        motor_commands = np.asarray(allocation["commands_motor_order"], dtype=float)
    else:
        thrust = candidate
        allocation_success = direct_inside
        residual = direct_residual
        motor_commands = config_thrust_n_to_commands(config, thrust)
    motor_headroom = float(min(np.min(thrust), np.min(maximum - thrust)))
    delta_headroom = float(maximum_motor_delta_n - np.max(np.abs(thrust - base_thrust)))
    end_transform, _ = dynamics.model.forward_kinematics(
        "gripper_link", dict(positions)
    )
    endpoint = end_transform[:3, 3].copy()
    gripper_lower, gripper_upper = dynamics.model.joint_limits("gripper")
    allowed = bool(
        not collisions
        and raw_torque_norm <= gravity_torque_limit_nm + 1.0e-9
        and joint_effort_feasible
        and allocation_success
        and residual <= allocation_residual_limit
        and motor_headroom >= minimum_motor_headroom_n
        and delta_headroom >= minimum_delta_headroom_n
    )
    reasons = []
    if collisions:
        reasons.append("collision_proxy")
    if raw_torque_norm > gravity_torque_limit_nm + 1.0e-9:
        reasons.append("gravity_torque_limit")
    if not joint_effort_feasible:
        reasons.append("joint_effort_reserve")
    if not allocation_success or residual > allocation_residual_limit:
        reasons.append("allocation_residual")
    if motor_headroom < minimum_motor_headroom_n:
        reasons.append("physical_motor_headroom")
    if delta_headroom < minimum_delta_headroom_n:
        reasons.append("overlay_delta_headroom")
    return {
        "positions_rad": {name: float(positions[name]) for name in dynamics.active_joint_names},
        "endpoint_body_flu_m": endpoint.tolist(),
        "horizontal_direction": _horizontal_direction(endpoint),
        # Reclassified after the scan against the actually flight-allowed
        # vertical span.  The retracted pose happens to be near the top of the
        # SO101 workspace and is therefore not a meaningful up/down origin.
        "vertical_direction": "unclassified",
        "radial_distance_m": float(np.linalg.norm(endpoint[:2])),
        "radial_band": _radial_band(float(np.linalg.norm(endpoint[:2]))),
        "gripper_state": _gripper_state(
            float(positions["gripper"]), gripper_lower, gripper_upper
        ),
        "mass_kg": float(mass),
        "center_of_mass_body_flu_m": center.tolist(),
        "com_shift_body_flu_m": com_shift.tolist(),
        "com_shift_norm_m": float(np.linalg.norm(com_shift)),
        "inertia_at_com_kg_m2": inertia.tolist(),
        "gravity_torque_body_flu_nm": gravity_torque_flu.tolist(),
        "gravity_torque_norm_nm": raw_torque_norm,
        "joint_gravity_torque_nm": {
            name: float(joint_gravity_torque[index])
            for index, name in enumerate(dynamics.active_joint_names)
        },
        "joint_usable_effort_nm": {
            name: float(usable_joint_effort[index])
            for index, name in enumerate(dynamics.active_joint_names)
        },
        "joint_effort_margin_nm": {
            name: float(joint_effort_margin[index])
            for index, name in enumerate(dynamics.active_joint_names)
        },
        "minimum_joint_effort_margin_nm": float(np.min(joint_effort_margin)),
        "requested_compensation_wrench_frd": compensation_frd.tolist(),
        "motor_thrust_n": thrust.tolist(),
        "motor_command_normalized": motor_commands.tolist(),
        "minimum_physical_motor_headroom_n": motor_headroom,
        "remaining_overlay_delta_headroom_n": delta_headroom,
        "allocation_residual_norm": residual,
        "collision_proxy_pairs": [list(pair) for pair in collisions],
        "flight_allowed": allowed,
        "rejection_reasons": reasons,
    }


def scan_workspace(
    urdf: Path,
    motion_reference: Path,
    flight_config: Path,
    *,
    target_mass_kg: float = 4.0,
    grid_counts: Mapping[str, int] = DEFAULT_GRID_COUNTS,
    limit_margin_fraction: float = 0.03,
    gravity_torque_limit_nm: float = 1.35,
    maximum_motor_delta_n: float = 1.60,
    minimum_motor_headroom_n: float = 0.25,
    minimum_delta_headroom_n: float = 0.05,
    allocation_residual_limit: float = 1.0e-6,
    joint_effort_reserve_fraction: float = 0.10,
    rotor_clearance_m: float = 0.005,
    include_samples: bool = False,
) -> dict:
    if not 0.0 <= float(joint_effort_reserve_fraction) < 1.0:
        raise ValueError("joint_effort_reserve_fraction must be in [0, 1)")
    if float(rotor_clearance_m) < 0.0:
        raise ValueError("rotor_clearance_m must be non-negative")
    reference = json.loads(Path(motion_reference).read_text(encoding="utf-8"))
    config = json.loads(Path(flight_config).read_text(encoding="utf-8"))
    dynamics = CoupledArmDynamics(
        Path(urdf), reference, target_mass_kg=target_mass_kg
    )
    gravity = float(config.get("gravity_m_s2", 9.80665))
    baseline = _hover_baseline(config, target_mass_kg, gravity)
    urdf_boxes = _box_collision_proxies(Path(urdf))
    rotor_boxes = _rotor_swept_volume_proxies(
        Path(urdf), clearance_m=rotor_clearance_m
    )
    boxes = urdf_boxes + rotor_boxes
    names = dynamics.active_joint_names
    anchors = {
        name: {joint: float(value) for joint, value in zip(names, values, strict=True)}
        for name, values in reference.get("presets", {}).items()
        if isinstance(values, list) and len(values) == len(names)
    }
    accepted_anchor_names = [
        name for name in ("retracted", "flight_straight_forward") if name in anchors
    ]
    # These two pairs overlap in every documented pose because the coarse box
    # proxies cover nested mounting hardware.  Do not learn global exclusions
    # from accepted poses: doing so previously hid an upper-arm/wrist overlap
    # everywhere merely because the folded anchor was accepted once.
    ignored_pairs = _adjacent_link_pairs(dynamics.model) | set(
        STRUCTURAL_PROXY_EXCLUSIONS
    )
    anchor_collision_overrides = {
        name: calibrated_proxy_exclusions(
            dynamics, urdf_boxes, (anchors[name],)
        ) - ignored_pairs
        for name in accepted_anchor_names
    }

    common = dict(
        gravity_m_s2=gravity,
        gravity_torque_limit_nm=gravity_torque_limit_nm,
        maximum_motor_delta_n=maximum_motor_delta_n,
        minimum_motor_headroom_n=minimum_motor_headroom_n,
        minimum_delta_headroom_n=minimum_delta_headroom_n,
        allocation_residual_limit=allocation_residual_limit,
        joint_effort_reserve_fraction=joint_effort_reserve_fraction,
    )
    samples = []
    for positions in _pose_values(dynamics, grid_counts, limit_margin_fraction):
        samples.append(
            evaluate_static_pose(
                dynamics, config, baseline, boxes, ignored_pairs, positions, **common
            )
        )
    anchor_results = {}
    for name, positions in anchors.items():
        anchor_ignored = ignored_pairs | anchor_collision_overrides.get(name, set())
        result = evaluate_static_pose(
            dynamics, config, baseline, boxes, anchor_ignored, positions, **common
        )
        result["anchor_collision_override_pairs"] = [
            list(pair) for pair in sorted(anchor_collision_overrides.get(name, set()))
        ]
        anchor_results[name] = result
    allowed = [item for item in samples if item["flight_allowed"]]
    allowed_z = [item["endpoint_body_flu_m"][2] for item in allowed]
    if allowed_z:
        z_min, z_max = min(allowed_z), max(allowed_z)
        z_reference = float(
            anchor_results.get("retracted", {}).get(
                "endpoint_body_flu_m", [0.0, 0.0, 0.5 * (z_min + z_max)]
            )[2]
        )
        vertical_deadband = min(0.025, max(0.005, (z_max - z_min) / 8.0))
        for item in (*samples, *anchor_results.values()):
            item["vertical_direction"] = _vertical_direction(
                item["endpoint_body_flu_m"][2] - z_reference,
                vertical_deadband,
            )
    else:
        z_min = z_max = z_reference = vertical_deadband = None
    directions = HORIZONTAL_DIRECTIONS
    coverage = {}
    for direction in directions:
        group = [item for item in allowed if item["horizontal_direction"] == direction]
        coverage[direction] = {
            "allowed_samples": len(group),
            "maximum_radial_distance_m": max(
                (item["radial_distance_m"] for item in group), default=None
            ),
        }
    for vertical in ("up", "level", "down"):
        group = [item for item in allowed if item["vertical_direction"] == vertical]
        coverage[vertical] = {"allowed_samples": len(group)}
    radial_bands = ("near", "middle", "far")
    gripper_states = ("closed", "middle", "open")
    coverage["radial_bands"] = {
        band: {
            "allowed_samples": sum(item["radial_band"] == band for item in allowed)
        }
        for band in radial_bands
    }
    coverage["gripper_states"] = {
        state: {
            "allowed_samples": sum(item["gripper_state"] == state for item in allowed)
        }
        for state in gripper_states
    }
    coverage["direction_vertical_matrix"] = {
        direction: {
            vertical: sum(
                item["horizontal_direction"] == direction
                and item["vertical_direction"] == vertical
                for item in allowed
            )
            for vertical in ("up", "level", "down")
        }
        for direction in directions
    }
    coverage["direction_radial_matrix"] = {
        direction: {
            band: sum(
                item["horizontal_direction"] == direction
                and item["radial_band"] == band
                for item in allowed
            )
            for band in radial_bands
        }
        for direction in directions
    }
    coverage["direction_gripper_matrix"] = {
        direction: {
            state: sum(
                item["horizontal_direction"] == direction
                and item["gripper_state"] == state
                for item in allowed
            )
            for state in gripper_states
        }
        for direction in directions
    }
    coverage_checks = {
        "eight_horizontal_directions": all(
            coverage[name]["allowed_samples"] > 0 for name in directions
        ),
        "four_horizontal_diagonals": all(
            coverage[name]["allowed_samples"] > 0 for name in DIAGONAL_DIRECTIONS
        ),
        "up_level_down": all(
            coverage[name]["allowed_samples"] > 0 for name in ("up", "level", "down")
        ),
        "near_middle_far": all(
            coverage["radial_bands"][name]["allowed_samples"] > 0
            for name in radial_bands
        ),
        "closed_middle_open_gripper": all(
            coverage["gripper_states"][name]["allowed_samples"] > 0
            for name in gripper_states
        ),
        "at_least_two_distances_in_each_horizontal_direction": all(
            sum(value > 0 for value in coverage["direction_radial_matrix"][name].values())
            >= 2
            for name in directions
        ),
        "all_gripper_states_in_each_horizontal_direction": all(
            all(value > 0 for value in coverage["direction_gripper_matrix"][name].values())
            for name in directions
        ),
    }
    rejection_counts: dict[str, int] = {}
    for item in samples:
        for reason in item["rejection_reasons"]:
            rejection_counts[reason] = rejection_counts.get(reason, 0) + 1
    extrema_source = allowed if allowed else samples
    def compact(item: dict) -> dict:
        return {
            "positions_rad": item["positions_rad"],
            "endpoint_body_flu_m": item["endpoint_body_flu_m"],
            "horizontal_direction": item["horizontal_direction"],
            "vertical_direction": item["vertical_direction"],
            "radial_band": item["radial_band"],
            "gripper_state": item["gripper_state"],
            "com_shift_norm_m": item["com_shift_norm_m"],
            "gravity_torque_norm_nm": item["gravity_torque_norm_nm"],
            "minimum_physical_motor_headroom_n": item[
                "minimum_physical_motor_headroom_n"
            ],
            "remaining_overlay_delta_headroom_n": item[
                "remaining_overlay_delta_headroom_n"
            ],
            "allocation_residual_norm": item["allocation_residual_norm"],
            "minimum_joint_effort_margin_nm": item[
                "minimum_joint_effort_margin_nm"
            ],
            "flight_allowed": item["flight_allowed"],
            "rejection_reasons": item["rejection_reasons"],
        }

    boundary_samples = {
        "maximum_radius_by_direction": {
            direction: compact(max(
                (
                    item for item in allowed
                    if item["horizontal_direction"] == direction
                ),
                key=lambda item: item["radial_distance_m"],
            ))
            for direction in directions
            if any(item["horizontal_direction"] == direction for item in allowed)
        },
        "highest_allowed": None if not allowed else compact(max(
            allowed, key=lambda item: item["endpoint_body_flu_m"][2]
        )),
        "lowest_allowed": None if not allowed else compact(min(
            allowed, key=lambda item: item["endpoint_body_flu_m"][2]
        )),
        # A static extremum can have an unsafe continuous path from the folded
        # pose.  Preserve a small ranked candidate set so the trajectory
        # planner can reject that extremum and try a nearby kinematic branch
        # without storing the full 25,725-pose scan in the evidence file.
        "highest_candidates": [
            compact(item)
            for item in sorted(
                allowed,
                key=lambda item: item["endpoint_body_flu_m"][2],
                reverse=True,
            )[:512]
        ],
        "lowest_candidates": [
            compact(item)
            for item in sorted(
                allowed,
                key=lambda item: item["endpoint_body_flu_m"][2],
            )[:512]
        ],
        "maximum_gravity_torque_allowed": None if not allowed else compact(max(
            allowed, key=lambda item: item["gravity_torque_norm_nm"]
        )),
        "minimum_overlay_headroom_allowed": None if not allowed else compact(min(
            allowed, key=lambda item: item["remaining_overlay_delta_headroom_n"]
        )),
        "rejection_witness": {
            reason: compact(next(
                item for item in samples if reason in item["rejection_reasons"]
            ))
            for reason in sorted(rejection_counts)
        },
    }
    summary = {
        "sample_count": len(samples),
        "flight_allowed_count": len(allowed),
        "flight_allowed_fraction": len(allowed) / max(1, len(samples)),
        "rejection_counts": rejection_counts,
        "maximum_allowed_com_shift_m": max(
            (item["com_shift_norm_m"] for item in extrema_source), default=None
        ),
        "maximum_allowed_gravity_torque_nm": max(
            (item["gravity_torque_norm_nm"] for item in extrema_source), default=None
        ),
        "minimum_allowed_physical_motor_headroom_n": min(
            (item["minimum_physical_motor_headroom_n"] for item in extrema_source),
            default=None,
        ),
        "minimum_allowed_overlay_delta_headroom_n": min(
            (item["remaining_overlay_delta_headroom_n"] for item in extrema_source),
            default=None,
        ),
        "minimum_allowed_joint_effort_margin_nm": min(
            (item["minimum_joint_effort_margin_nm"] for item in extrema_source),
            default=None,
        ),
        "allowed_endpoint_z_min_m": z_min,
        "allowed_endpoint_z_max_m": z_max,
        "allowed_endpoint_vertical_span_m": (
            None if z_min is None else z_max - z_min
        ),
        "vertical_classification_reference": "retracted_endpoint_body_flu_z",
        "vertical_classification_reference_m": z_reference,
        "vertical_classification_deadband_m": vertical_deadband,
        "coverage_checks": coverage_checks,
        "required_direction_coverage_complete": all(coverage_checks.values()),
    }
    joint_sampling = {}
    for name in dynamics.active_joint_names:
        lower, upper = dynamics.model.joint_limits(name)
        margin = max(0.0, float(limit_margin_fraction)) * (upper - lower)
        count = max(2, int(grid_counts.get(name, 3)))
        sampled_span = (upper - margin) - (lower + margin)
        joint_sampling[name] = {
            "count": count,
            "sampled_min_rad": float(lower + margin),
            "sampled_max_rad": float(upper - margin),
            "maximum_grid_step_rad": float(sampled_span / (count - 1)),
        }
    report = {
        "schema": 2,
        "status": "DISCRETE_STATIC_ENVELOPE",
        "interpretation": (
            "Discrete static CAD/allocator/joint-effort screening with conservative "
            "box and rotor swept-volume proxies. Category coverage does not prove "
            "continuous-space safety. Flight acceptance still requires trajectory "
            "preflight and PX4/Gazebo dynamic/contact validation."
        ),
        "model_assumptions": {
            "positive_thrust_direction_status": config.get(
                "positive_thrust_direction_status", "UNSPECIFIED"
            ),
            "rotor_axis_assumption": config.get("axis_assumption", "UNSPECIFIED"),
            "motor_model_status": config.get(
                "actuator_input_model_status", "UNSPECIFIED"
            ),
            "joint_effort_source": "motion_reference joints[*].effort_nm",
            "joint_effort_measurement_status": "DOCUMENTED_NOT_BENCH_VERIFIED",
            "static_pose_only": True,
            "continuous_trajectory_guarantee": False,
            "formal_physical_release_ready": False,
        },
        "inputs": {
            "urdf": str(Path(urdf)),
            "motion_reference": str(Path(motion_reference)),
            "flight_config": str(Path(flight_config)),
            "sha256": {
                "urdf": _sha256(Path(urdf)),
                "motion_reference": _sha256(Path(motion_reference)),
                "flight_config": _sha256(Path(flight_config)),
            },
            "target_mass_kg": target_mass_kg,
            "grid_counts": dict(grid_counts),
            "joint_sampling": joint_sampling,
            "limit_margin_fraction": limit_margin_fraction,
            "radial_band_limits_m": list(RADIAL_BAND_LIMITS_M),
        },
        "limits": {
            "gravity_torque_limit_nm": gravity_torque_limit_nm,
            "maximum_motor_delta_n": maximum_motor_delta_n,
            "minimum_motor_headroom_n": minimum_motor_headroom_n,
            "minimum_delta_headroom_n": minimum_delta_headroom_n,
            "allocation_residual_limit": allocation_residual_limit,
            "joint_effort_reserve_fraction": joint_effort_reserve_fraction,
        },
        "collision_proxy": {
            "box_count": len(boxes),
            "urdf_box_count": len(urdf_boxes),
            "rotor_swept_volume_count": len(rotor_boxes),
            "rotor_clearance_m": rotor_clearance_m,
            "accepted_calibration_anchors": accepted_anchor_names,
            "ignored_link_pairs": [list(pair) for pair in sorted(ignored_pairs)],
            "structural_proxy_exclusions": [
                list(pair) for pair in sorted(STRUCTURAL_PROXY_EXCLUSIONS)
            ],
            "anchor_only_collision_overrides": {
                name: [list(pair) for pair in sorted(pairs)]
                for name, pairs in anchor_collision_overrides.items()
            },
            "calibration_excludes_rotor_swept_volumes": True,
            "mesh_contact_validation_required": True,
        },
        "baseline_hover": {
            "thrust_config_order_n": baseline["thrust_config_order_n"].tolist(),
            "commands_motor_order": baseline["commands_motor_order"].tolist(),
        },
        "coverage": coverage,
        "summary": summary,
        "boundary_samples": boundary_samples,
        "anchors": anchor_results,
    }
    if include_samples:
        report["samples"] = samples
    return report
