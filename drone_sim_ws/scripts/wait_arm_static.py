#!/usr/bin/env python3
"""Require an arm preset to be reached and motionless for a continuous window."""

from __future__ import annotations

import argparse
import json
import math
from pathlib import Path
import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState


def evaluate_joint_sample(
    names,
    positions,
    velocities,
    target_by_name: dict[str, float],
) -> tuple[bool, float, float]:
    """Return completeness, maximum position error and maximum speed."""
    if len(names) != len(positions) or len(names) != len(velocities):
        return False, float("inf"), float("inf")
    observed = {
        str(name): (float(position), float(velocity))
        for name, position, velocity in zip(
            names, positions, velocities, strict=True
        )
    }
    if not set(target_by_name).issubset(observed):
        return False, float("inf"), float("inf")
    values = [observed[name] for name in target_by_name]
    if not all(math.isfinite(value) for pair in values for value in pair):
        return False, float("inf"), float("inf")
    maximum_error = max(
        abs(observed[name][0] - target) for name, target in target_by_name.items()
    )
    maximum_speed = max(abs(observed[name][1]) for name in target_by_name)
    return True, maximum_error, maximum_speed


class ArmStaticMonitor(Node):
    def __init__(
        self,
        topic: str,
        target_by_name: dict[str, float],
        position_tolerance: float,
        velocity_limit: float,
        hold_s: float,
    ) -> None:
        super().__init__("my_drone_arm_static_monitor")
        self.target_by_name = target_by_name
        self.position_tolerance = float(position_tolerance)
        self.velocity_limit = float(velocity_limit)
        self.hold_s = float(hold_s)
        self.stable_since: float | None = None
        self.last_sample = 0.0
        self.sample_count = 0
        self.maximum_error = float("inf")
        self.maximum_speed = float("inf")
        self.create_subscription(JointState, topic, self.on_joint_state, 20)

    def on_joint_state(self, message: JointState) -> None:
        complete, error, speed = evaluate_joint_sample(
            message.name, message.position, message.velocity, self.target_by_name
        )
        now = time.monotonic()
        self.last_sample = now
        self.sample_count += 1
        self.maximum_error = error
        self.maximum_speed = speed
        if (
            complete
            and error <= self.position_tolerance
            and speed <= self.velocity_limit
        ):
            if self.stable_since is None:
                self.stable_since = now
        else:
            self.stable_since = None

    def ready(self) -> bool:
        now = time.monotonic()
        return bool(
            self.stable_since is not None
            and now - self.stable_since >= self.hold_s
            and now - self.last_sample < 0.5
        )


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--reference", required=True, type=Path)
    parser.add_argument("--preset", default="retracted")
    parser.add_argument("--topic", default="/joint_states")
    parser.add_argument("--position-tolerance", type=float, default=0.08)
    parser.add_argument("--velocity-limit", type=float, default=0.03)
    parser.add_argument("--hold", type=float, default=2.0)
    parser.add_argument("--timeout", type=float, default=30.0)
    args = parser.parse_args()

    reference = json.loads(args.reference.read_text(encoding="utf-8"))
    joint_names = [str(item["name"]) for item in reference["joints"]]
    target = reference["presets"].get(args.preset)
    if target is None or len(target) != len(joint_names):
        raise SystemExit(f"invalid preset in motion reference: {args.preset}")
    target_by_name = dict(zip(joint_names, map(float, target), strict=True))

    rclpy.init()
    node = ArmStaticMonitor(
        args.topic,
        target_by_name,
        args.position_tolerance,
        args.velocity_limit,
        args.hold,
    )
    deadline = time.monotonic() + args.timeout
    try:
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
            if node.ready():
                print(
                    "ARM_STATIC_GATE_PASS "
                    f"preset={args.preset} samples={node.sample_count} "
                    f"max_error_rad={node.maximum_error:.6f} "
                    f"max_speed_rad_s={node.maximum_speed:.6f}",
                    flush=True,
                )
                return 0
        print(
            "ARM_STATIC_GATE_FAIL "
            f"preset={args.preset} samples={node.sample_count} "
            f"max_error_rad={node.maximum_error:.6f} "
            f"max_speed_rad_s={node.maximum_speed:.6f}",
            flush=True,
        )
        return 1
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
