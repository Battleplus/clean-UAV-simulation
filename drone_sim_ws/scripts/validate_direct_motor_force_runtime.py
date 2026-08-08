#!/usr/bin/env python3
"""Runtime sign/sanity check for the direct eight-motor Gazebo wrench path.

This deliberately bypasses PX4 and holds all eight normalized actuator inputs
at a fixed value for a short interval.  It is useful for distinguishing a
control-loop tuning problem from a force-frame/sign problem.
"""

from __future__ import annotations

import argparse
import math
import time

import rclpy
from actuator_msgs.msg import Actuators
from nav_msgs.msg import Odometry
from rclpy.node import Node


class DirectMotorRuntimeCheck(Node):
    def __init__(self, command: float, hold_s: float) -> None:
        super().__init__("my_drone_direct_motor_runtime_check")
        self.command = float(command)
        self.hold_s = float(hold_s)
        self.publisher = self.create_publisher(
            Actuators, "/my_drone/command/motor_speed", 10
        )
        self.create_subscription(
            Odometry, "/model/my_drone/odometry", self._odometry_cb, 20
        )
        self.started = time.monotonic()
        self.last_report = 0.0
        self.samples: list[tuple[float, float, float, float]] = []
        self.timer = self.create_timer(0.02, self._tick)

    def _odometry_cb(self, msg: Odometry) -> None:
        now = time.monotonic() - self.started
        self.samples.append(
            (
                now,
                float(msg.pose.pose.position.x),
                float(msg.pose.pose.position.y),
                float(msg.pose.pose.position.z),
            )
        )

    def _tick(self) -> None:
        elapsed = time.monotonic() - self.started
        message = Actuators()
        value = self.command if elapsed <= self.hold_s else 0.0
        message.normalized = [value] * 8
        self.publisher.publish(message)
        if elapsed - self.last_report >= 1.0:
            self.last_report = elapsed
            if self.samples:
                sample = self.samples[-1]
                self.get_logger().info(
                    "DIRECT_MOTOR_RUNTIME t=%.2f command=%.3f world_xyz=(%.4f, %.4f, %.4f)"
                    % (elapsed, value, sample[1], sample[2], sample[3])
                )
        if elapsed > self.hold_s + 2.0:
            self.get_logger().info(self.summary())
            raise SystemExit(0)

    def summary(self) -> str:
        if not self.samples:
            return "DIRECT_MOTOR_RUNTIME no odometry samples"
        before = [sample for sample in self.samples if sample[0] < 0.5]
        during = [sample for sample in self.samples if 0.5 <= sample[0] <= self.hold_s]
        after = [sample for sample in self.samples if sample[0] > self.hold_s]
        reference = (before or self.samples)[-1][3]
        peak = max(sample[3] for sample in during or self.samples)
        minimum = min(sample[3] for sample in during or self.samples)
        final = (after or self.samples)[-1][3]
        return (
            "DIRECT_MOTOR_RUNTIME_SUMMARY "
            f"command={self.command:.3f} reference_z={reference:.6f} "
            f"hold_z_min={minimum:.6f} hold_z_max={peak:.6f} final_z={final:.6f} "
            f"delta_z={final-reference:.6f}"
        )


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--command", type=float, default=0.9)
    parser.add_argument("--hold-seconds", type=float, default=4.0)
    args = parser.parse_args()
    rclpy.init()
    node = DirectMotorRuntimeCheck(args.command, args.hold_seconds)
    try:
        rclpy.spin(node)
    except SystemExit:
        return 0
    finally:
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
