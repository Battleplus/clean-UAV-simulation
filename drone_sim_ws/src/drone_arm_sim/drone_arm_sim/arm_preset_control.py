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
from std_msgs.msg import Bool
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
        self.motion_publisher = self.create_publisher(
            Bool, "/my_drone/arm_motion_active", 10
        )
        self.create_subscription(JointState, "/joint_states", self.on_state, 10)

    def on_state(self, message: JointState) -> None:
        self.latest_positions.update(zip(message.name, message.position))

    def send(self, positions: list[float], duration_s: float) -> None:
        message = JointTrajectory()
        message.joint_names = JOINT_NAMES
        # A single far-away endpoint leaves the Gazebo joint controller to
        # choose its own interpolation/initial velocity.  During flight that
        # can create a large first acceleration even when the endpoint time is
        # long.  Publish a sampled quintic (smoothstep) trajectory instead:
        # position, velocity and acceleration are all continuous and zero at
        # both ends, while the requested preset and total duration are kept
        # unchanged.  The first sample is the measured current pose whenever
        # available, so an interrupted command cannot inject a position jump.
        start = [
            float(self.latest_positions.get(name, 0.0))
            for name in JOINT_NAMES
        ]
        segment_count = max(4, int(round(float(duration_s))))
        points = []
        for index in range(segment_count + 1):
            tau = index / segment_count
            smooth = 10.0 * tau**3 - 15.0 * tau**4 + 6.0 * tau**5
            smooth_rate = (30.0 * tau**2 - 60.0 * tau**3 + 30.0 * tau**4) / duration_s
            smooth_accel = (
                60.0 * tau - 180.0 * tau**2 + 120.0 * tau**3
            ) / (duration_s * duration_s)
            point = JointTrajectoryPoint()
            point.positions = [
                current + smooth * (target - current)
                for current, target in zip(start, positions)
            ]
            point.velocities = [
                smooth_rate * (target - current)
                for current, target in zip(start, positions)
            ]
            point.accelerations = [
                smooth_accel * (target - current)
                for current, target in zip(start, positions)
            ]
            elapsed = float(duration_s) * tau
            seconds = int(elapsed)
            point.time_from_start = Duration(
                sec=seconds,
                nanosec=int(round((elapsed - seconds) * 1_000_000_000)),
            )
            points.append(point)
        message.points = points
        self.publisher.publish(message)

    def publish_motion_active(self, active: bool) -> None:
        message = Bool()
        message.data = bool(active)
        self.motion_publisher.publish(message)

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


def load_motion_reference() -> dict:
    config = (
        Path(get_package_share_directory("drone_arm_sim"))
        / "config"
        / "so101_motion_reference.json"
    )
    return json.loads(config.read_text(encoding="utf-8"))


def main(args=None) -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--preset",
        choices=(
            "retracted",
            "work_a",
            "work_b",
            "flight_work_a",
            "flight_work_b",
            "flight_micro_a",
            "flight_micro_b",
        ),
        required=True,
    )
    parser.add_argument("--duration", type=float, default=3.0)
    parser.add_argument("--wait", action="store_true")
    parser.add_argument("--tolerance", type=float, default=0.04)
    parsed = parser.parse_args(args)
    if parsed.duration <= 0.0:
        parser.error("--duration must be positive")

    reference = load_motion_reference()
    target = reference["presets"][parsed.preset]
    limits = {item["name"]: item for item in reference.get("joints", [])}
    for name, value in zip(JOINT_NAMES, target):
        limit = limits.get(name)
        if limit is None:
            raise RuntimeError(f"no safety limit is defined for joint {name}")
        if not float(limit["lower_rad"]) <= float(value) <= float(limit["upper_rad"]):
            raise RuntimeError(f"preset {parsed.preset} exceeds limit for joint {name}")
    # A cubic trajectory can peak above average speed.  Reserve half of the
    # configured joint velocity limit for a conservative ground/flight command.
    min_duration = max(
        abs(float(value) - float(reference["presets"]["retracted"][index]))
        / max(1e-6, 0.5 * float(limits[name]["velocity_rad_s"]))
        for index, (name, value) in enumerate(zip(JOINT_NAMES, target))
    )
    if parsed.duration < min_duration:
        parser.error(
            f"{parsed.preset} needs duration >= {min_duration:.3f}s for the 0.5x velocity safety limit"
        )
    rclpy.init()
    node = ArmPresetCommander()
    try:
        connection_deadline = time.monotonic() + 10.0
        while node.publisher.get_subscription_count() == 0:
            if time.monotonic() >= connection_deadline:
                raise RuntimeError("arm_controller trajectory subscriber not found")
            rclpy.spin_once(node, timeout_sec=0.1)
        node.publish_motion_active(True)
        node.send(target, parsed.duration)
        node.get_logger().info(
            f"Sent preset {parsed.preset} over /arm_controller/joint_trajectory"
        )
        if not parsed.wait:
            return
        deadline = time.monotonic() + parsed.duration + 5.0
        last_motion_heartbeat = 0.0
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.05)
            if time.monotonic() - last_motion_heartbeat >= 0.25:
                node.publish_motion_active(True)
                last_motion_heartbeat = time.monotonic()
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
        node.publish_motion_active(False)
        rclpy.spin_once(node, timeout_sec=0.1)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
