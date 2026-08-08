#!/usr/bin/env python3
"""Wait until Gazebo odometry proves the CAD aircraft is stationary."""

from __future__ import annotations

import argparse
import math
import time

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node


class SettleMonitor(Node):
    def __init__(self, linear_limit: float, angular_limit: float, hold_s: float) -> None:
        super().__init__("my_drone_settle_monitor")
        self.linear_limit = float(linear_limit)
        self.angular_limit = float(angular_limit)
        self.hold_s = float(hold_s)
        self.stable_since: float | None = None
        self.last_sample = 0.0
        self.sample_count = 0
        self.last_linear = float("inf")
        self.last_angular = float("inf")
        self.create_subscription(
            Odometry, "/model/my_drone/odometry", self.on_odometry, 20
        )

    def on_odometry(self, message: Odometry) -> None:
        linear = message.twist.twist.linear
        angular = message.twist.twist.angular
        self.last_linear = math.sqrt(linear.x**2 + linear.y**2 + linear.z**2)
        self.last_angular = math.sqrt(angular.x**2 + angular.y**2 + angular.z**2)
        now = time.monotonic()
        self.last_sample = now
        self.sample_count += 1
        if self.last_linear <= self.linear_limit and self.last_angular <= self.angular_limit:
            if self.stable_since is None:
                self.stable_since = now
        else:
            self.stable_since = None

    def settled(self) -> bool:
        return bool(
            self.stable_since is not None
            and time.monotonic() - self.stable_since >= self.hold_s
            and time.monotonic() - self.last_sample < 0.5
        )


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--timeout", type=float, default=45.0)
    parser.add_argument("--hold", type=float, default=2.0)
    parser.add_argument("--linear-limit", type=float, default=0.08)
    parser.add_argument("--angular-limit", type=float, default=0.08)
    args = parser.parse_args()
    rclpy.init()
    node = SettleMonitor(args.linear_limit, args.angular_limit, args.hold)
    deadline = time.monotonic() + args.timeout
    try:
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
            if node.settled():
                print(
                    "MODEL_SETTLED "
                    f"samples={node.sample_count} "
                    f"linear_m_s={node.last_linear:.5f} "
                    f"angular_rad_s={node.last_angular:.5f}",
                    flush=True,
                )
                return 0
        print(
            "MODEL_NOT_SETTLED "
            f"samples={node.sample_count} "
            f"linear_m_s={node.last_linear:.5f} "
            f"angular_rad_s={node.last_angular:.5f}",
            flush=True,
        )
        return 1
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
