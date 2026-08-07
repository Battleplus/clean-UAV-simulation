"""Publish the static hover rotor speed for Gazebo motor plumbing tests."""

from __future__ import annotations

import argparse
import json
from math import sqrt
from pathlib import Path

from actuator_msgs.msg import Actuators
import rclpy
from rclpy.node import Node

from drone_arm_sim.allocation_analysis import _default_config


class HoverPublisher(Node):
    def __init__(self, rotor_speed: float):
        super().__init__("my_drone_motor_hover_demo")
        self.rotor_speed = rotor_speed
        self.publisher = self.create_publisher(
            Actuators, "/my_drone/command/motor_speed", 10
        )
        self.timer = self.create_timer(0.02, self.publish)

    def publish(self) -> None:
        message = Actuators()
        message.velocity = [self.rotor_speed] * 8
        self.publisher.publish(message)


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--config", type=Path, default=_default_config())
    parsed, ros_arguments = parser.parse_known_args()
    config = json.loads(parsed.config.read_text(encoding="utf-8"))
    thrust_each = (
        float(config["example_mass_kg"])
        * float(config.get("gravity_m_s2", 9.81))
        / (8.0 * 0.965926)
    )
    speed = sqrt(
        thrust_each / float(config["motor_constant_n_per_rad_s_squared"])
    )
    rclpy.init(args=ros_arguments)
    node = HoverPublisher(speed)
    node.get_logger().warning(
        "Open-loop smoke test only: it cannot stabilize pose or arm reaction. "
        f"Publishing {speed:.3f} rad/s to all eight virtual rotors."
    )
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
