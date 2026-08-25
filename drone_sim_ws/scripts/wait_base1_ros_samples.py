#!/usr/bin/env python3
"""Wait for stable Base1 ROS samples with one DDS participant."""

from __future__ import annotations

import argparse
import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import String


class SampleGate(Node):
    def __init__(self, joint_samples: int, coupling_samples: int) -> None:
        super().__init__("base1_startup_sample_gate")
        self.required_joint_samples = joint_samples
        self.required_coupling_samples = coupling_samples
        self.joint_samples = 0
        self.coupling_samples = 0
        if joint_samples:
            self.create_subscription(
                JointState, "/joint_states", self._on_joint_state, 10
            )
        if coupling_samples:
            self.create_subscription(
                String,
                "/my_drone/base1_estimator/coupling_state",
                self._on_coupling_state,
                10,
            )

    def _on_joint_state(self, message: JointState) -> None:
        if message.name and len(message.name) == len(message.position):
            self.joint_samples += 1

    def _on_coupling_state(self, message: String) -> None:
        if message.data:
            self.coupling_samples += 1

    def ready(self) -> bool:
        return (
            self.joint_samples >= self.required_joint_samples
            and self.coupling_samples >= self.required_coupling_samples
        )


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--joint-samples", type=int, default=0)
    parser.add_argument("--coupling-samples", type=int, default=0)
    parser.add_argument("--timeout", type=float, default=8.0)
    args = parser.parse_args()
    if args.joint_samples < 0 or args.coupling_samples < 0:
        parser.error("sample counts must be non-negative")
    if args.joint_samples + args.coupling_samples == 0:
        parser.error("at least one sample count must be positive")
    if args.timeout <= 0.0:
        parser.error("timeout must be positive")

    rclpy.init()
    node = SampleGate(args.joint_samples, args.coupling_samples)
    deadline = time.monotonic() + args.timeout
    try:
        while rclpy.ok() and not node.ready() and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
        if node.ready():
            return 0
        print(
            "BASE1_SAMPLE_GATE_TIMEOUT "
            f"joint={node.joint_samples}/{args.joint_samples} "
            f"coupling={node.coupling_samples}/{args.coupling_samples}",
            flush=True,
        )
        return 1
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
