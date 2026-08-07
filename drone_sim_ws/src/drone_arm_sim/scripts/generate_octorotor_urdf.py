"""Add eight virtual rotor links and Gazebo motor plugins to my_drone."""

from __future__ import annotations

import argparse
import json
from pathlib import Path
import xml.etree.ElementTree as ET

import numpy as np


def _element(parent: ET.Element, tag: str, text: str | None = None, **attrs):
    child = ET.SubElement(parent, tag, attrs)
    child.text = text
    return child


def _z_axis_rpy(axis: np.ndarray) -> np.ndarray:
    """Return URDF RPY whose local positive Z axis equals ``axis``."""
    z_axis = np.array([0.0, 0.0, 1.0])
    cosine = float(np.clip(np.dot(z_axis, axis), -1.0, 1.0))
    cross = np.cross(z_axis, axis)
    sine = float(np.linalg.norm(cross))
    if sine < 1e-12:
        rotation = np.eye(3) if cosine > 0 else np.diag([1.0, -1.0, -1.0])
    else:
        skew = np.array(
            [
                [0.0, -cross[2], cross[1]],
                [cross[2], 0.0, -cross[0]],
                [-cross[1], cross[0], 0.0],
            ]
        )
        rotation = np.eye(3) + skew + skew @ skew * ((1.0 - cosine) / sine**2)
    pitch = np.arcsin(np.clip(-rotation[2, 0], -1.0, 1.0))
    roll = np.arctan2(rotation[2, 1], rotation[2, 2])
    yaw = np.arctan2(rotation[1, 0], rotation[0, 0])
    return np.array([roll, pitch, yaw])


def generate(source: Path, config_path: Path, destination: Path) -> None:
    tree = ET.parse(source)
    root = tree.getroot()
    root.attrib["name"] = "my_drone_octorotor"
    # PX4 GZBridge currently hard-codes all primary sensor topics under a
    # link named "base_link".  Keep the geometry unchanged while giving the
    # aircraft root that name and disambiguating the SO-101 pedestal link.
    link_name_map = {
        "drone_base_link": "base_link",
        "base_link": "arm_base_link",
    }
    for link in root.findall("link"):
        link.attrib["name"] = link_name_map.get(
            link.attrib["name"], link.attrib["name"]
        )
    for joint in root.findall("joint"):
        for endpoint in ("parent", "child"):
            node = joint.find(endpoint)
            node.attrib["link"] = link_name_map.get(
                node.attrib["link"], node.attrib["link"]
            )
    for frame in root.findall(".//robot_base_frame"):
        if frame.text in link_name_map:
            frame.text = link_name_map[frame.text]
    config = json.loads(config_path.read_text(encoding="utf-8"))
    motor_constant = float(config["motor_constant_n_per_rad_s_squared"])
    time_constant = float(config["motor_time_constant_s"])
    max_speed = float(config["maximum_rotor_speed_rad_s"])
    moment_constant = float(config["reaction_moment_ratio_m"])
    slowdown = int(config["rotor_velocity_slowdown_sim"])
    rotor_mass = float(config["virtual_rotor_mass_kg"])

    gazebo = root.find("gazebo")
    if gazebo is None:
        gazebo = ET.SubElement(root, "gazebo")

    sensor_extension = ET.Element("gazebo", {"reference": "base_link"})
    for sensor_name, sensor_type, update_rate in (
        ("air_pressure_sensor", "air_pressure", "50"),
        ("magnetometer_sensor", "magnetometer", "100"),
        ("imu_sensor", "imu", "250"),
        ("navsat_sensor", "navsat", "30"),
    ):
        sensor = _element(
            sensor_extension, "sensor", name=sensor_name, type=sensor_type
        )
        _element(sensor, "always_on", "1")
        _element(sensor, "update_rate", update_rate)
        _element(sensor, "gz_frame_id", "base_link")
        if sensor_type == "imu":
            _element(sensor, "imu")
    root.insert(list(root).index(gazebo), sensor_extension)

    for index, rotor in enumerate(config["rotors"]):
        link_name = rotor["name"]
        joint_name = f"{link_name}_joint"
        position_frd = np.asarray(rotor["position_m"], dtype=float)
        position_flu = np.array(
            [position_frd[0], -position_frd[1], -position_frd[2]]
        )
        axis_frd = np.asarray(rotor["axis_body"], dtype=float)
        # ROS / Gazebo body frame is FLU. PX4 config is FRD.
        axis_flu = np.array([axis_frd[0], -axis_frd[1], -axis_frd[2]])
        axis_flu /= np.linalg.norm(axis_flu)
        rotor_rpy = _z_axis_rpy(axis_flu)

        link = ET.Element("link", {"name": link_name})
        inertial = _element(link, "inertial")
        _element(inertial, "origin", xyz="0 0 0", rpy="0 0 0")
        _element(inertial, "mass", value=f"{rotor_mass:.8g}")
        _element(
            inertial,
            "inertia",
            ixx="1e-7",
            ixy="0",
            ixz="0",
            iyy="1e-7",
            iyz="0",
            izz="2e-7",
        )
        visual = _element(link, "visual")
        _element(visual, "origin", xyz="0 0 0", rpy="0 0 0")
        geometry = _element(visual, "geometry")
        _element(geometry, "cylinder", radius="0.085", length="0.003")
        material = _element(visual, "material", name="virtual_rotor")
        _element(material, "color", rgba="0.08 0.08 0.08 0.75")
        root.insert(list(root).index(gazebo), link)

        joint = ET.Element(
            "joint", {"name": joint_name, "type": "continuous"}
        )
        _element(
            joint,
            "origin",
            xyz=" ".join(f"{value:.9g}" for value in position_flu),
            rpy=" ".join(f"{value:.9g}" for value in rotor_rpy),
        )
        _element(joint, "parent", link="base_link")
        _element(joint, "child", link=link_name)
        _element(
            joint,
            "axis",
            xyz="0 0 1",
        )
        _element(joint, "dynamics", damping="0.00001", friction="0")
        root.insert(list(root).index(gazebo), joint)

        plugin = _element(
            gazebo,
            "plugin",
            filename="gz-sim-multicopter-motor-model-system",
            name="gz::sim::systems::MulticopterMotorModel",
        )
        _element(plugin, "jointName", joint_name)
        _element(plugin, "linkName", link_name)
        _element(
            plugin,
            "turningDirection",
            "ccw" if float(rotor["direction"]) > 0 else "cw",
        )
        _element(plugin, "timeConstantUp", f"{time_constant:.9g}")
        _element(plugin, "timeConstantDown", f"{time_constant:.9g}")
        _element(plugin, "maxRotVelocity", f"{max_speed:.9g}")
        _element(plugin, "motorConstant", f"{motor_constant:.9g}")
        _element(plugin, "momentConstant", f"{moment_constant:.9g}")
        _element(plugin, "commandSubTopic", "command/motor_speed")
        _element(plugin, "actuator_number", str(index))
        _element(plugin, "rotorDragCoefficient", "0")
        _element(plugin, "rollingMomentCoefficient", "0")
        _element(plugin, "rotorVelocitySlowdownSim", str(slowdown))
        _element(plugin, "motorType", "velocity")

    ET.indent(tree, space="  ")
    destination.parent.mkdir(parents=True, exist_ok=True)
    tree.write(destination, encoding="utf-8", xml_declaration=True)


def main() -> None:
    parser = argparse.ArgumentParser()
    directory = Path(__file__).resolve().parents[1]
    parser.add_argument(
        "--source",
        type=Path,
        default=directory / "urdf" / "drone_with_arm_controlled.urdf",
    )
    parser.add_argument(
        "--config",
        type=Path,
        default=directory / "config" / "octorotor_example.json",
    )
    parser.add_argument(
        "--output",
        type=Path,
        default=directory / "urdf" / "my_drone_octorotor_example.urdf",
    )
    args = parser.parse_args()
    generate(args.source, args.config, args.output)
    print(args.output)


if __name__ == "__main__":
    main()
