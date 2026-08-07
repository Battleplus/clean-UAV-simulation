"""Exit successfully only when Gazebo ``my_drone`` has settled in hover."""

from __future__ import annotations

import argparse
import math

import numpy as np
import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node


class HoverAcceptance(Node):
    def __init__(
        self,
        target: np.ndarray,
        settle_time: float,
        position_tolerance: float,
        attitude_tolerance_deg: float,
    ):
        super().__init__("my_drone_hover_acceptance")
        self.target = target
        self.settle_time = settle_time
        self.position_tolerance = position_tolerance
        self.attitude_tolerance = math.radians(attitude_tolerance_deg)
        self.start_stamp = None
        self.result = None
        self.subscription = self.create_subscription(
            Odometry,
            "/model/my_drone/odometry",
            self.on_odometry,
            10,
        )

    @staticmethod
    def _stamp_seconds(message: Odometry) -> float:
        stamp = message.header.stamp
        return float(stamp.sec) + float(stamp.nanosec) * 1e-9

    def on_odometry(self, message: Odometry) -> None:
        stamp = self._stamp_seconds(message)
        if self.start_stamp is None:
            self.start_stamp = stamp
            return
        if stamp - self.start_stamp < self.settle_time:
            return

        pose = message.pose.pose
        twist = message.twist.twist
        position = np.array(
            [pose.position.x, pose.position.y, pose.position.z]
        )
        position_error = float(np.linalg.norm(self.target - position))
        quaternion = np.array(
            [
                pose.orientation.x,
                pose.orientation.y,
                pose.orientation.z,
                pose.orientation.w,
            ]
        )
        quaternion /= np.linalg.norm(quaternion)
        attitude_error = 2.0 * math.acos(
            min(1.0, abs(float(quaternion[3])))
        )
        speed = float(
            np.linalg.norm(
                [twist.linear.x, twist.linear.y, twist.linear.z]
            )
        )
        angular_speed = float(
            np.linalg.norm(
                [twist.angular.x, twist.angular.y, twist.angular.z]
            )
        )
        velocity = [
            float(twist.linear.x),
            float(twist.linear.y),
            float(twist.linear.z),
        ]
        angular_velocity = [
            float(twist.angular.x),
            float(twist.angular.y),
            float(twist.angular.z),
        ]
        self.result = {
            "position_enu_m": position.round(6).tolist(),
            "position_error_m": position_error,
            "attitude_error_deg": math.degrees(attitude_error),
            "velocity_body_m_s": np.round(velocity, 6).tolist(),
            "speed_m_s": speed,
            "angular_velocity_body_rad_s": np.round(
                angular_velocity, 6
            ).tolist(),
            "angular_speed_rad_s": angular_speed,
        }
        passed = (
            position_error <= self.position_tolerance
            and attitude_error <= self.attitude_tolerance
            and speed <= 0.05
            and angular_speed <= 0.05
        )
        self.get_logger().info(
            f"hover acceptance: {self.result}, passed={passed}"
        )
        self.result["passed"] = passed
        rclpy.shutdown()


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--target", nargs=3, type=float, default=[0.0, 0.0, 1.0])
    parser.add_argument("--settle-time", type=float, default=8.0)
    parser.add_argument("--position-tolerance", type=float, default=0.05)
    parser.add_argument("--attitude-tolerance-deg", type=float, default=2.0)
    parsed, ros_arguments = parser.parse_known_args()

    rclpy.init(args=ros_arguments)
    node = HoverAcceptance(
        np.asarray(parsed.target, dtype=float),
        parsed.settle_time,
        parsed.position_tolerance,
        parsed.attitude_tolerance_deg,
    )
    try:
        rclpy.spin(node)
    finally:
        result = node.result
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    if result is None:
        raise SystemExit("No complete odometry acceptance window was received.")
    if not result["passed"]:
        raise SystemExit(f"Hover acceptance failed: {result}")


if __name__ == "__main__":
    main()
