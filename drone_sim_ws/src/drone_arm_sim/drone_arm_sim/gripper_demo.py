"""Open and close the gripper while preserving every arm joint position."""

import argparse
import time

from builtin_interfaces.msg import Duration
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from drone_arm_sim.cartesian_arm_demo import JOINT_NAMES


class GripperDemo(Node):
    def __init__(self) -> None:
        super().__init__("gripper_demo")
        self.positions = {}
        self.trajectory_pub = self.create_publisher(
            JointTrajectory, "/arm_controller/joint_trajectory", 10
        )
        self.motion_pub = self.create_publisher(Bool, "/my_drone/arm_motion_active", 10)
        self.create_subscription(JointState, "/joint_states", self._state_cb, 10)

    def _state_cb(self, message: JointState) -> None:
        self.positions.update(zip(message.name, message.position))


def _point(values: list[float], elapsed: float) -> JointTrajectoryPoint:
    point = JointTrajectoryPoint()
    point.positions = values
    point.velocities = [0.0] * len(values)
    seconds = int(elapsed)
    point.time_from_start = Duration(
        sec=seconds, nanosec=int(round((elapsed - seconds) * 1_000_000_000))
    )
    return point


def main(args=None) -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--open", type=float, default=1.2)
    parser.add_argument("--duration", type=float, default=3.0)
    parser.add_argument("--hold", type=float, default=2.0)
    parsed = parser.parse_args(args)
    if parsed.duration <= 0.0 or parsed.hold < 0.0:
        parser.error("duration must be positive and hold cannot be negative")

    rclpy.init()
    node = GripperDemo()
    try:
        deadline = time.monotonic() + 10.0
        while node.trajectory_pub.get_subscription_count() == 0 or any(
            name not in node.positions for name in JOINT_NAMES
        ):
            if time.monotonic() >= deadline:
                raise RuntimeError("arm controller or complete joint state is unavailable")
            rclpy.spin_once(node, timeout_sec=0.1)
        start = [float(node.positions[name]) for name in JOINT_NAMES]
        opened = list(start)
        opened[-1] = parsed.open
        message = JointTrajectory()
        message.joint_names = JOINT_NAMES
        message.points = [
            _point(start, 0.05),
            _point(opened, parsed.duration),
            _point(opened, parsed.duration + parsed.hold),
            _point(start, 2.0 * parsed.duration + parsed.hold),
        ]
        node.motion_pub.publish(Bool(data=True))
        node.trajectory_pub.publish(message)
        node.get_logger().info(
            f"GRIPPER_DEMO_BEGIN start={start[-1]:.3f} open={parsed.open:.3f}"
        )
        finished = time.monotonic() + 2.0 * parsed.duration + parsed.hold
        while time.monotonic() < finished:
            rclpy.spin_once(node, timeout_sec=0.1)
            node.motion_pub.publish(Bool(data=True))
        error = abs(float(node.positions["gripper"]) - start[-1])
        node.get_logger().info(f"GRIPPER_DEMO_COMPLETE return_error={error:.6f}rad")
        if error > 0.08:
            raise RuntimeError(f"gripper did not return to start: {error:.3f} rad")
    except RuntimeError as error:
        node.get_logger().error(str(error))
        raise SystemExit(1)
    finally:
        node.motion_pub.publish(Bool(data=False))
        rclpy.spin_once(node, timeout_sec=0.1)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
