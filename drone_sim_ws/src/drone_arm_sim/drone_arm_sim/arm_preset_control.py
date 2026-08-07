"""Command and verify SO101 arm presets through ros2_control."""

from __future__ import annotations

import argparse
import json
from pathlib import Path
import sys
import time

from ament_index_python.packages import get_package_share_directory
from builtin_interfaces.msg import Duration
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


JOINT_NAMES = [
    "shoulder_pan",
    "shoulder_lift",
    "elbow_flex",
    "wrist_flex",
    "wrist_roll",
    "gripper",
]


class ArmPresetCommander(Node):
    def __init__(self) -> None:
        super().__init__("arm_preset_control")
        self.publisher = self.create_publisher(
            JointTrajectory, "/arm_controller/joint_trajectory", 10
        )
        self.latest_positions: dict[str, float] = {}
        self.create_subscription(JointState, "/joint_states", self.on_state, 10)

    def on_state(self, message: JointState) -> None:
        self.latest_positions.update(zip(message.name, message.position))

    def send(self, positions: list[float], duration_s: float) -> None:
        message = JointTrajectory()
        message.joint_names = JOINT_NAMES
        point = JointTrajectoryPoint()
        point.positions = positions
        seconds = int(duration_s)
        point.time_from_start = Duration(
            sec=seconds, nanosec=int((duration_s - seconds) * 1_000_000_000)
        )
        message.points = [point]
        self.publisher.publish(message)

    def maximum_error(self, target: list[float]) -> float | None:
        if any(name not in self.latest_positions for name in JOINT_NAMES):
            return None
        return max(
            abs(self.latest_positions[name] - expected)
            for name, expected in zip(JOINT_NAMES, target)
        )


def load_presets() -> dict[str, list[float]]:
    config = (
        Path(get_package_share_directory("drone_arm_sim"))
        / "config"
        / "so101_motion_reference.json"
    )
    return json.loads(config.read_text(encoding="utf-8"))["presets"]


def main(args=None) -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--preset",
        choices=("retracted", "work_a", "work_b", "flight_work_a", "flight_work_b"),
        required=True,
    )
    parser.add_argument("--duration", type=float, default=3.0)
    parser.add_argument("--wait", action="store_true")
    parser.add_argument("--tolerance", type=float, default=0.04)
    parsed = parser.parse_args(args)
    if parsed.duration <= 0.0:
        parser.error("--duration must be positive")

    target = load_presets()[parsed.preset]
    rclpy.init()
    node = ArmPresetCommander()
    try:
        connection_deadline = time.monotonic() + 10.0
        while node.publisher.get_subscription_count() == 0:
            if time.monotonic() >= connection_deadline:
                raise RuntimeError("arm_controller trajectory subscriber not found")
            rclpy.spin_once(node, timeout_sec=0.1)
        node.send(target, parsed.duration)
        node.get_logger().info(
            f"Sent preset {parsed.preset} over /arm_controller/joint_trajectory"
        )
        if not parsed.wait:
            return
        deadline = time.monotonic() + parsed.duration + 5.0
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.05)
            error = node.maximum_error(target)
            if error is not None and error <= parsed.tolerance:
                node.get_logger().info(
                    f"ARM_PRESET_REACHED preset={parsed.preset} max_error={error:.6f} rad"
                )
                return
        error = node.maximum_error(target)
        raise RuntimeError(
            f"preset {parsed.preset} was not reached; max_error={error} rad"
        )
    except RuntimeError as error:
        node.get_logger().error(str(error))
        sys.exit(1)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
