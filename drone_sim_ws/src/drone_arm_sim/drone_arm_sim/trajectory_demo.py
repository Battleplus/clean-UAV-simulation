"""Publish a repeatable six-joint ros2_control trajectory command."""

from builtin_interfaces.msg import Duration
import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


JOINT_NAMES = [
    "shoulder_pan",
    "shoulder_lift",
    "elbow_flex",
    "wrist_flex",
    "wrist_roll",
    "gripper",
]


class TrajectoryDemo(Node):
    def __init__(self) -> None:
        super().__init__("trajectory_demo")
        self.publisher = self.create_publisher(
            JointTrajectory,
            "/arm_controller/joint_trajectory",
            10,
        )
        self.timer = self.create_timer(2.0, self.publish_once)

    @staticmethod
    def point(positions: list[float], seconds: int) -> JointTrajectoryPoint:
        point = JointTrajectoryPoint()
        point.positions = positions
        point.time_from_start = Duration(sec=seconds)
        return point

    def publish_once(self) -> None:
        message = JointTrajectory()
        message.joint_names = JOINT_NAMES
        message.points = [
            self.point([0.0, 0.0, 0.0, 0.0, 0.0, 0.2], 1),
            self.point([0.4, -0.6, 0.8, -0.5, 0.3, 0.8], 4),
            self.point([-0.4, -0.3, 0.5, 0.2, -0.3, 0.3], 7),
            self.point([0.0, 0.0, 0.0, 0.0, 0.0, 0.2], 10),
        ]
        self.publisher.publish(message)
        self.get_logger().info(
            "Published a 10 s trajectory to "
            "/arm_controller/joint_trajectory"
        )
        self.timer.cancel()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = TrajectoryDemo()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
