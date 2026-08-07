"""Build the non-flyable formal-mass URDF from CAD aggregate evidence."""

from __future__ import annotations

import argparse
import json
from pathlib import Path
import tempfile
import xml.etree.ElementTree as ET

import numpy as np

from drone_arm_sim.model_analysis import UrdfModel


CAD_TO_FLU = np.array(
    [[0.0, 0.0, -1.0], [-1.0, 0.0, 0.0], [0.0, 1.0, 0.0]]
)


def _matrix(node: ET.Element) -> np.ndarray:
    return np.array([
        [float(node.attrib["ixx"]), float(node.attrib["ixy"]), float(node.attrib["ixz"])],
        [float(node.attrib["ixy"]), float(node.attrib["iyy"]), float(node.attrib["iyz"])],
        [float(node.attrib["ixz"]), float(node.attrib["iyz"]), float(node.attrib["izz"])],
    ])


def _set_matrix(node: ET.Element, inertia: np.ndarray) -> None:
    values = {
        "ixx": inertia[0, 0], "ixy": inertia[0, 1], "ixz": inertia[0, 2],
        "iyy": inertia[1, 1], "iyz": inertia[1, 2], "izz": inertia[2, 2],
    }
    for key, value in values.items():
        node.attrib[key] = f"{float(value):.12g}"


def _parallel_axis(mass: float, offset: np.ndarray) -> np.ndarray:
    return mass * (
        float(np.dot(offset, offset)) * np.eye(3) - np.outer(offset, offset)
    )


def build_formal_urdf(
    source_urdf: Path,
    output_urdf: Path,
    physical_evidence: Path,
    physical_config: Path,
    geometry_config: Path,
    arm_mesh_groups: Path,
) -> dict:
    physical = json.loads(physical_evidence.read_text(encoding="utf-8"))
    active_physical = json.loads(physical_config.read_text(encoding="utf-8"))
    geometry = json.loads(geometry_config.read_text(encoding="utf-8"))
    mesh_groups = json.loads(arm_mesh_groups.read_text(encoding="utf-8"))
    source_model = UrdfModel(source_urdf)
    source_mass, source_center, _ = source_model.mass_properties({})
    target_mass = float(active_physical["estimated_mass_kg"])
    scale = target_mass / source_mass

    cad_center = np.asarray(physical["estimated_cad_com_m"], dtype=float)
    stl_translation = np.asarray(
        mesh_groups["solidworks_stl_translation_mm"], dtype=float
    ) * 0.001
    urdf_cad_origin = np.asarray(geometry["cad_origin_mm"], dtype=float) * 0.001
    target_center = CAD_TO_FLU @ (
        cad_center + stl_translation - urdf_cad_origin
    )
    target_inertia = np.asarray(
        active_physical["estimated_ros_flu_inertia_at_com_kg_m2"], dtype=float
    )

    tree = ET.parse(source_urdf)
    root = tree.getroot()
    root.attrib["name"] = "my_drone_v3_cad_formal_dynamic"
    root.insert(0, ET.Comment(
        " FORMAL CAD GEOMETRY WITH CURRENT ENGINEERING MASS/INERTIA OVERRIDE. "
        "Per-link distribution is scaled from CAD mesh volumes; base inertial is "
        "closed to the exact aggregate CAD mass, COM and inertia. "
    ))
    for inertial in root.findall("link/inertial"):
        mass_node = inertial.find("mass")
        mass_node.attrib["value"] = f"{float(mass_node.attrib['value']) * scale:.12g}"
        inertia_node = inertial.find("inertia")
        _set_matrix(inertia_node, _matrix(inertia_node) * scale)

    # Non-base entries keep the mesh-volume distribution. Solve the base-link
    # COM and tensor so the complete zero-pose model exactly conserves the CAD
    # aggregate mass properties instead of merely matching total mass.
    with tempfile.NamedTemporaryFile(suffix=".urdf", delete=False) as handle:
        temporary = Path(handle.name)
    try:
        tree.write(temporary, encoding="utf-8", xml_declaration=True)
        scaled_model = UrdfModel(temporary)
        entries = scaled_model.inertial_entries({})
    finally:
        temporary.unlink(missing_ok=True)

    base_entry = next(entry for entry in entries if entry[0] == "base_link")
    base_mass = base_entry[1]
    nonbase = [entry for entry in entries if entry[0] != "base_link"]
    nonbase_first_moment = sum(
        (mass * center for _, mass, center, _ in nonbase), start=np.zeros(3)
    )
    base_center = (target_mass * target_center - nonbase_first_moment) / base_mass
    nonbase_inertia_at_target = np.zeros((3, 3))
    for _, mass, center, inertia in nonbase:
        nonbase_inertia_at_target += inertia + _parallel_axis(
            mass, center - target_center
        )
    base_inertia = (
        target_inertia
        - nonbase_inertia_at_target
        - _parallel_axis(base_mass, base_center - target_center)
    )
    base_inertia = 0.5 * (base_inertia + base_inertia.T)
    eigenvalues = np.linalg.eigvalsh(base_inertia)
    if float(np.min(eigenvalues)) <= 0.0:
        raise ValueError(
            f"CAD closure requires non-physical base inertia eigenvalues {eigenvalues}"
        )

    base_inertial = root.find("link[@name='base_link']/inertial")
    base_origin = base_inertial.find("origin")
    base_origin.attrib["xyz"] = " ".join(f"{value:.12g}" for value in base_center)
    base_origin.attrib["rpy"] = "0 0 0"
    _set_matrix(base_inertial.find("inertia"), base_inertia)
    output_urdf.parent.mkdir(parents=True, exist_ok=True)
    ET.indent(tree, space="  ")
    tree.write(output_urdf, encoding="utf-8", xml_declaration=True)

    result_model = UrdfModel(output_urdf)
    result_mass, result_center, result_inertia = result_model.mass_properties({})
    if not np.isclose(result_mass, target_mass, rtol=0.0, atol=1e-8):
        raise AssertionError((result_mass, target_mass))
    if not np.allclose(result_center, target_center, rtol=0.0, atol=1e-8):
        raise AssertionError((result_center, target_center))
    if not np.allclose(result_inertia, target_inertia, rtol=0.0, atol=1e-8):
        raise AssertionError((result_inertia, target_inertia))
    return {
        "source_urdf": str(source_urdf),
        "physical_evidence": str(physical_evidence),
        "physical_config": str(physical_config),
        "mass_override_source": active_physical.get("mass_override_source"),
        "mass_distribution_method": (
            "all link inertials scaled from CAD mesh-volume distribution; "
            "base link closed to exact aggregate CAD mass/COM/inertia"
        ),
        "source_mass_kg": source_mass,
        "scale": scale,
        "formal_mass_kg": result_mass,
        "formal_com_ros_flu_m": result_center.tolist(),
        "formal_inertia_at_com_ros_flu_kg_m2": result_inertia.tolist(),
        "base_mass_kg": base_mass,
        "base_com_ros_flu_m": base_center.tolist(),
        "base_inertia_at_com_ros_flu_kg_m2": base_inertia.tolist(),
        "base_inertia_eigenvalues": eigenvalues.tolist(),
        "flight_status_at_1p2_kgf_per_motor": (
            "FEASIBLE_WITH_OPPOSITE_PITCH_HYPOTHESIS"
            if active_physical["allocation_results"][1]["hover_feasible_at_rated_1p2_kgf"]
            else "INFEASIBLE"
        ),
    }


def main() -> None:
    package = Path(__file__).resolve().parents[1]
    workspace = package.parents[1]
    parser = argparse.ArgumentParser()
    parser.add_argument("--source", type=Path, default=package / "urdf/my_drone_v2/my_drone_cad_dynamic.urdf")
    parser.add_argument("--output", type=Path, default=package / "urdf/my_drone_v3/my_drone_cad_formal_dynamic.urdf")
    parser.add_argument("--physical", type=Path, default=workspace / "analysis/cad_direct/cad_mass_properties.json")
    parser.add_argument("--physical-config", type=Path, default=package / "config/my_drone_v3_cad_physical.json")
    parser.add_argument("--geometry", type=Path, default=package / "config/my_drone_v2_cad.json")
    parser.add_argument("--arm-groups", type=Path, default=workspace / "analysis/cad_direct/so101_link_mesh_groups.json")
    parser.add_argument("--report", type=Path, default=package / "config/my_drone_v3_cad_formal_urdf.json")
    args = parser.parse_args()
    report = build_formal_urdf(
        args.source, args.output, args.physical, args.physical_config,
        args.geometry, args.arm_groups
    )
    args.report.write_text(json.dumps(report, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    print(args.output)
    print(args.report)
    print(
        f"formal_mass={report['formal_mass_kg']:.9f} kg "
        f"status={report['flight_status_at_1p2_kgf_per_motor']}"
    )


if __name__ == "__main__":
    main()
