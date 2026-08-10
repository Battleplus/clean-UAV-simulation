"""Interactive bounded Cartesian jogging for the SO101 end effector."""

from __future__ import annotations

import argparse
import math
import select
import sys
import termios
import time
import tty

from builtin_interfaces.msg import Duration
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from drone_arm_sim.cartesian_arm_demo import (
    JOINT_NAMES,
    TOOL_FORWARD_AXIS_LOCAL,
    _formal_urdf,
    _solve_tool_axis_ik,
)
from drone_arm_sim.model_analysis import UrdfModel


HALF_POSE = {
    "shoulder_pan": 0.0,
    "shoulder_lift": -0.40,
    "elbow_flex": 0.825,
    "wrist_flex": 0.0,
    "wrist_roll": 0.0,
    "gripper": 0.60,
}
RETRACTED_POSE = {name: 0.0 for name in JOINT_NAMES}


class CartesianJogNode(Node):
    def __init__(self, step_m: float, duration_s: float) -> None:
        super().__init__("cartesian_arm_jog")
        self.step_m = step_m
        self.duration_s = duration_s
        self.positions: dict[str, float] = {}
        self.model = UrdfModel(_formal_urdf())
        self.publisher = self.create_publisher(
            JointTrajectory, "/arm_controller/joint_trajectory", 10
        )
        self.create_subscription(JointState, "/joint_states", self._state_cb, 10)

    def _state_cb(self, message: JointState) -> None:
        self.positions.update(zip(message.name, message.position))

    def ready(self) -> bool:
        return self.publisher.get_subscription_count() > 0 and all(
            name in self.positions for name in JOINT_NAMES
        )

    def _publish(self, target: dict[str, float], duration_s: float | None = None) -> None:
        message = JointTrajectory()
        message.joint_names = JOINT_NAMES
        point = JointTrajectoryPoint()
        point.positions = [float(target[name]) for name in JOINT_NAMES]
        point.velocities = [0.0] * len(JOINT_NAMES)
        elapsed = self.duration_s if duration_s is None else duration_s
        seconds = int(elapsed)
        point.time_from_start = Duration(
            sec=seconds, nanosec=int(round((elapsed - seconds) * 1e9))
        )
        message.points = [point]
        self.publisher.publish(message)

    def command_pose(self, target: dict[str, float], duration_s: float = 4.0) -> None:
        self._publish(target, duration_s)
        self.get_logger().info("ARM_JOG_POSE_SENT")

    def jog(self, delta_base: np.ndarray) -> bool:
        seed = {name: float(self.positions[name]) for name in JOINT_NAMES[:-1]}
        transform, _ = self.model.forward_kinematics("gripper_link", seed)
        target_position = transform[:3, 3] + delta_base
        target_forward = transform[:3, :3] @ TOOL_FORWARD_AXIS_LOCAL
        target_forward /= np.linalg.norm(target_forward)
        solution, status = _solve_tool_axis_ik(
            self.model, target_position, target_forward, seed
        )
        if not bool(status["converged"]):
            self.get_logger().warning(
                f"ARM_JOG_REJECTED IK error={status['position_error_m']:.4f} m"
            )
            return False
        margin = math.radians(4.0)
        for name, value in solution.items():
            lower, upper = self.model.joint_limits(name)
            if not lower + margin <= value <= upper - margin:
                self.get_logger().warning(f"ARM_JOG_REJECTED {name} near limit")
                return False
            if abs(value - seed[name]) > 0.25:
                self.get_logger().warning(f"ARM_JOG_REJECTED {name} step too large")
                return False
        target = {**solution, "gripper": float(self.positions["gripper"])}
        self._publish(target)
        self.get_logger().info(
            "ARM_JOG_SENT delta_base_m=" + np.array2string(delta_base, precision=3)
        )
        return True

    def gripper(self, increment: float) -> None:
        lower, upper = self.model.joint_limits("gripper")
        target = dict(self.positions)
        target["gripper"] = float(
            np.clip(float(target["gripper"]) + increment, lower, upper)
        )
        self._publish(target, 0.8)


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--step", type=float, default=0.01)
    parser.add_argument("--duration", type=float, default=0.8)
    args, ros_args = parser.parse_known_args()
    if not 0.002 <= args.step <= 0.03:
        parser.error("--step must be between 0.002 and 0.03 m")
    rclpy.init(args=ros_args)
    node = CartesianJogNode(args.step, args.duration)
    deadline = time.monotonic() + 15.0
    while rclpy.ok() and not node.ready() and time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.1)
    if not node.ready():
        node.destroy_node()
        rclpy.shutdown()
        raise SystemExit("arm controller or joint states are unavailable")

    controls = {
        "i": np.array([args.step, 0.0, 0.0]),
        "k": np.array([-args.step, 0.0, 0.0]),
        "j": np.array([0.0, args.step, 0.0]),
        "l": np.array([0.0, -args.step, 0.0]),
        "u": np.array([0.0, 0.0, args.step]),
        "o": np.array([0.0, 0.0, -args.step]),
    }
    print("SO101 Cartesian jog: B half pose | I/K forward/back | J/L left/right")
    print("U/O up/down | [/] gripper | X retract | Q quit")
    settings = termios.tcgetattr(sys.stdin)
    try:
        tty.setcbreak(sys.stdin.fileno())
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.02)
            if not select.select([sys.stdin], [], [], 0.02)[0]:
                continue
            key = sys.stdin.read(1).lower()
            if key in controls:
                node.jog(controls[key])
            elif key == "b":
                node.command_pose(HALF_POSE)
            elif key == "x":
                node.command_pose(RETRACTED_POSE)
            elif key == "[":
                node.gripper(-0.05)
            elif key == "]":
                node.gripper(0.05)
            elif key == "q":
                break
    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
