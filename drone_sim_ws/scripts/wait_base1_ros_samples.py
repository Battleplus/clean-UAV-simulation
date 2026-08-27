#!/usr/bin/env python3
"""Wait for stable Base1 ROS samples with one DDS participant."""

from __future__ import annotations

import argparse
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from px4_msgs.msg import VehicleStatus
from sensor_msgs.msg import JointState
from std_msgs.msg import String


class SampleGate(Node):
    def __init__(
        self,
        joint_samples: int,
        coupling_samples: int,
        vehicle_status_samples: int = 0,
    ) -> None:
        super().__init__("base1_startup_sample_gate")
        self.required_joint_samples = joint_samples
        self.required_coupling_samples = coupling_samples
        self.required_vehicle_status_samples = vehicle_status_samples
        self.joint_samples = 0
        self.coupling_samples = 0
        self.vehicle_status_samples = 0
        self.latest_arming_state: int | None = None
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
        if vehicle_status_samples:
            px4_qos = QoSProfile(
                reliability=ReliabilityPolicy.BEST_EFFORT,
                durability=DurabilityPolicy.TRANSIENT_LOCAL,
                history=HistoryPolicy.KEEP_LAST,
                depth=1,
            )
            self.create_subscription(
                VehicleStatus,
                "/fmu/out/vehicle_status_v4",
                self._on_vehicle_status,
                px4_qos,
            )

    def _on_joint_state(self, message: JointState) -> None:
        if message.name and len(message.name) == len(message.position):
            self.joint_samples += 1

    def _on_coupling_state(self, message: String) -> None:
        if message.data:
            self.coupling_samples += 1

    def _on_vehicle_status(self, message: VehicleStatus) -> None:
        self.latest_arming_state = int(message.arming_state)
        self.vehicle_status_samples += 1

    def ready(self) -> bool:
        return (
            self.joint_samples >= self.required_joint_samples
            and self.coupling_samples >= self.required_coupling_samples
            and self.vehicle_status_samples >= self.required_vehicle_status_samples
        )


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--joint-samples", type=int, default=0)
    parser.add_argument("--coupling-samples", type=int, default=0)
    parser.add_argument("--vehicle-status-samples", type=int, default=0)
    parser.add_argument(
        "--print-arming-state",
        action="store_true",
        help="print the latest numeric PX4 arming state after the gate passes",
    )
    parser.add_argument("--timeout", type=float, default=8.0)
    args = parser.parse_args()
    if (
        args.joint_samples < 0
        or args.coupling_samples < 0
        or args.vehicle_status_samples < 0
    ):
        parser.error("sample counts must be non-negative")
    if (
        args.joint_samples
        + args.coupling_samples
        + args.vehicle_status_samples
        == 0
    ):
        parser.error("at least one sample count must be positive")
    if args.timeout <= 0.0:
        parser.error("timeout must be positive")
    if args.print_arming_state and args.vehicle_status_samples <= 0:
        parser.error("--print-arming-state requires --vehicle-status-samples")

    rclpy.init()
    node = SampleGate(
        args.joint_samples,
        args.coupling_samples,
        args.vehicle_status_samples,
    )
    deadline = time.monotonic() + args.timeout
    try:
        while rclpy.ok() and not node.ready() and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
        if node.ready():
            if args.print_arming_state:
                print(
                    f"VEHICLE_ARMING_STATE state={node.latest_arming_state}",
                    flush=True,
                )
            return 0
        print(
            "BASE1_SAMPLE_GATE_TIMEOUT "
            f"joint={node.joint_samples}/{args.joint_samples} "
            f"coupling={node.coupling_samples}/{args.coupling_samples} "
            "vehicle_status="
            f"{node.vehicle_status_samples}/{args.vehicle_status_samples}",
            flush=True,
        )
        return 1
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
