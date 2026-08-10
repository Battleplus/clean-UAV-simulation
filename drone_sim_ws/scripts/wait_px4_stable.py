#!/usr/bin/env python3
"""Wait until PX4 local velocity remains inside a strict stability window."""

import argparse
import math
import time

import rclpy
from px4_msgs.msg import VehicleLocalPosition
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy


class StabilityGate(Node):
    def __init__(self, horizontal: float, vertical: float, hold: float) -> None:
        super().__init__("my_drone_stability_gate")
        self.horizontal = horizontal
        self.vertical = vertical
        self.hold = hold
        self.stable_since = None
        self.last_sample = None
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self.create_subscription(
            VehicleLocalPosition,
            "/fmu/out/vehicle_local_position_v1",
            self._position_cb,
            qos,
        )

    def _position_cb(self, message: VehicleLocalPosition) -> None:
        now = time.monotonic()
        horizontal_speed = math.hypot(float(message.vx), float(message.vy))
        vertical_speed = abs(float(message.vz))
        self.last_sample = (now, horizontal_speed, vertical_speed)
        inside = (
            math.isfinite(horizontal_speed)
            and math.isfinite(vertical_speed)
            and horizontal_speed < self.horizontal
            and vertical_speed < self.vertical
        )
        if inside:
            if self.stable_since is None:
                self.stable_since = now
        else:
            self.stable_since = None

    def accepted(self) -> bool:
        return bool(
            self.stable_since is not None
            and time.monotonic() - self.stable_since >= self.hold
        )


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--horizontal", type=float, default=0.10)
    parser.add_argument("--vertical", type=float, default=0.08)
    parser.add_argument("--hold", type=float, default=5.0)
    parser.add_argument("--timeout", type=float, default=90.0)
    args = parser.parse_args()
    rclpy.init()
    node = StabilityGate(args.horizontal, args.vertical, args.hold)
    started = time.monotonic()
    last_report = 0.0
    try:
        while time.monotonic() - started < args.timeout:
            rclpy.spin_once(node, timeout_sec=0.1)
            if node.accepted():
                _, horizontal, vertical = node.last_sample
                print(
                    "PX4_STABILITY_GATE_PASS "
                    f"horizontal_speed={horizontal:.3f}m/s "
                    f"vertical_speed={vertical:.3f}m/s hold={args.hold:.1f}s"
                )
                return 0
            if node.last_sample and time.monotonic() - last_report >= 1.0:
                _, horizontal, vertical = node.last_sample
                stable_for = (
                    0.0 if node.stable_since is None
                    else time.monotonic() - node.stable_since
                )
                print(
                    "PX4_STABILITY_GATE_WAIT "
                    f"horizontal_speed={horizontal:.3f}m/s "
                    f"vertical_speed={vertical:.3f}m/s stable_for={stable_for:.1f}s",
                    flush=True,
                )
                last_report = time.monotonic()
        print("PX4_STABILITY_GATE_TIMEOUT")
        return 1
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
