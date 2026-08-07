"""Build measured-geometry movable-arm URDF variants.

The resulting URDF intentionally adds no guessed rotor mass, propeller visual,
RPM model, or motor constant.  Direct forces are applied by the companion
``gazebo_direct_motor_model`` node using the measured-mount configuration.

Two explicit mass profiles are supported.  ``physical_estimate`` records the
current 10 kg CAD/density estimate even though the available motors cannot
hover it.  ``flight_test`` is a clearly labelled bring-up mass sized for a 1.8
vertical thrust-to-weight ratio.  Only the base-link mass and inertia are
scaled; the original movable-arm link properties are retained.
"""

from __future__ import annotations

import argparse
import json
from pathlib import Path
import xml.etree.ElementTree as ET

import numpy as np


def element(
    parent: ET.Element, tag: str, text: str | None = None, **attributes: str
) -> ET.Element:
    child = ET.SubElement(parent, tag, attributes)
    child.text = text
    return child


def z_axis_rpy(axis: np.ndarray) -> np.ndarray:
    """Return RPY for a frame whose local +Z is ``axis``."""
    axis = axis / np.linalg.norm(axis)
    reference = np.array([0.0, 0.0, 1.0])
    cosine = float(np.clip(reference @ axis, -1.0, 1.0))
    cross = np.cross(reference, axis)
    sine = float(np.linalg.norm(cross))
    if sine < 1e-12:
        rotation = (
            np.eye(3)
            if cosine > 0
            else np.diag([1.0, -1.0, -1.0])
        )
    else:
        skew = np.array(
            [
                [0.0, -cross[2], cross[1]],
                [cross[2], 0.0, -cross[0]],
                [-cross[1], cross[0], 0.0],
            ]
        )
        rotation = (
            np.eye(3)
            + skew
            + skew @ skew * ((1.0 - cosine) / sine**2)
        )
    pitch = np.arcsin(np.clip(-rotation[2, 0], -1.0, 1.0))
    return np.array(
        [
            np.arctan2(rotation[2, 1], rotation[2, 2]),
            pitch,
            np.arctan2(rotation[1, 0], rotation[0, 0]),
        ]
    )


def apply_mass_profile(
    root: ET.Element, config: dict, mass_profile: str
) -> tuple[float, float]:
    masses = []
    for link in root.findall("link"):
        mass = link.find("inertial/mass")
        if mass is not None:
            masses.append((link.attrib["name"], mass, float(mass.attrib["value"])))
    source_total = sum(value for _, _, value in masses)
    if mass_profile == "source":
        return source_total, source_total

    target_total = float(config["mass_profiles"][mass_profile]["all_up_mass_kg"])
    base = root.find("./link[@name='drone_base_link']/inertial")
    if base is None:
        raise ValueError("drone_base_link has no inertial element")
    base_mass_element = base.find("mass")
    inertia = base.find("inertia")
    if base_mass_element is None or inertia is None:
        raise ValueError("drone_base_link inertial data is incomplete")
    source_base_mass = float(base_mass_element.attrib["value"])
    other_mass = source_total - source_base_mass
    target_base_mass = target_total - other_mass
    if target_base_mass <= 0.0:
        raise ValueError(
            f"Target mass {target_total} kg is below retained arm mass "
            f"{other_mass} kg"
        )
    scale = target_base_mass / source_base_mass
    base_mass_element.attrib["value"] = f"{target_base_mass:.9g}"
    for attribute in ("ixx", "ixy", "ixz", "iyy", "iyz", "izz"):
        inertia.attrib[attribute] = f"{float(inertia.attrib[attribute]) * scale:.9g}"
    return source_total, target_total


def add_px4_sensors(root: ET.Element) -> None:
    gazebo = ET.Element("gazebo", {"reference": "drone_base_link"})
    definitions = (
        ("air_pressure_sensor", "air_pressure", "50"),
        ("magnetometer_sensor", "magnetometer", "100"),
        ("imu_sensor", "imu", "250"),
        ("navsat_sensor", "navsat", "30"),
    )
    for name, sensor_type, rate in definitions:
        sensor = element(gazebo, "sensor", name=name, type=sensor_type)
        element(sensor, "always_on", "1")
        element(sensor, "update_rate", rate)
        element(sensor, "gz_frame_id", "drone_base_link")
        if sensor_type == "imu":
            element(sensor, "imu")
    root.append(gazebo)


def generate(
    source: Path,
    config_path: Path,
    output: Path,
    mass_profile: str = "flight_test",
) -> None:
    tree = ET.parse(source)
    root = tree.getroot()
    config = json.loads(config_path.read_text(encoding="utf-8"))
    source_mass, target_mass = apply_mass_profile(root, config, mass_profile)
    root.attrib["name"] = f"my_drone_measured_{mass_profile}"
    add_px4_sensors(root)
    for frequency in root.findall(".//odom_publish_frequency"):
        frequency.text = "250"

    insertion_index = next(
        (
            index
            for index, child in enumerate(root)
            if child.tag == "gazebo"
        ),
        len(root),
    )
    for rotor in config["rotors"]:
        name = rotor["name"]
        position_frd = np.asarray(rotor["position_m"], dtype=float)
        axis_frd = np.asarray(rotor["axis_body"], dtype=float)
        position_flu = position_frd * np.array([1.0, -1.0, -1.0])
        axis_flu = axis_frd * np.array([1.0, -1.0, -1.0])
        rpy = z_axis_rpy(axis_flu)

        # Empty links are intentional coordinate frames and add no invented
        # rotor mass or inertia to the original dynamic model.
        link = ET.Element("link", {"name": name})
        root.insert(insertion_index, link)
        insertion_index += 1

        joint = ET.Element(
            "joint", {"name": f"{name}_joint", "type": "fixed"}
        )
        element(
            joint,
            "origin",
            xyz=" ".join(f"{value:.9g}" for value in position_flu),
            rpy=" ".join(f"{value:.9g}" for value in rpy),
        )
        element(joint, "parent", link="drone_base_link")
        element(joint, "child", link=name)
        root.insert(insertion_index, joint)
        insertion_index += 1

    root.insert(
        insertion_index,
        ET.Comment(
            " Rotor forces use config/my_drone_measured_mounts.json and "
            "gazebo_direct_motor_model; no RPM coefficient is assumed. "
            f"Mass profile: {mass_profile}, all-up mass: {target_mass:.9g} kg "
            f"(source URDF: {source_mass:.9g} kg). "
            "Odometry is 250 Hz to match the direct-thrust physics world. "
        ),
    )
    ET.indent(tree, space="  ")
    output.parent.mkdir(parents=True, exist_ok=True)
    tree.write(output, encoding="utf-8", xml_declaration=True)


def main() -> None:
    package = Path(__file__).resolve().parents[1]
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--source",
        type=Path,
        default=package / "urdf" / "drone_with_arm_controlled.urdf",
    )
    parser.add_argument(
        "--config",
        type=Path,
        default=package / "config" / "my_drone_measured_mounts.json",
    )
    parser.add_argument(
        "--output",
        type=Path,
        default=None,
    )
    parser.add_argument(
        "--mass-profile",
        choices=("source", "physical_estimate", "flight_test"),
        default="flight_test",
    )
    args = parser.parse_args()
    output = args.output or (
        package
        / "urdf"
        / f"drone_with_arm_measured_{args.mass_profile}.urdf"
    )
    generate(args.source, args.config, output, args.mass_profile)
    print(output)


if __name__ == "__main__":
    main()
