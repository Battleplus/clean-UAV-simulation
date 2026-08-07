"""Closed-loop 6D wrench hover controller for the Gazebo ``my_drone`` model.

Gazebo uses an ENU world and an FLU body.  ``ApplyLinkWrench`` expects force
and torque in world coordinates.  The controller compensates for applying
force at the base-link origin instead of at the composite center of mass.
"""

from __future__ import annotations

import argparse
from pathlib import Path

import numpy as np

from drone_arm_sim.model_analysis import UrdfModel, _default_urdf

try:
    from nav_msgs.msg import Odometry
    import rclpy
    from rclpy.node import Node
    from ros_gz_interfaces.msg import Entity, EntityWrench
except ModuleNotFoundError:  # Allows pure controller tests outside ROS.
    Odometry = None
    rclpy = None
    Node = object
    Entity = None
    EntityWrench = None


def quaternion_matrix_xyzw(quaternion: np.ndarray) -> np.ndarray:
    quaternion = quaternion / np.linalg.norm(quaternion)
    x, y, z, w = quaternion
    return np.array(
        [
            [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
        ]
    )


def _vee(matrix: np.ndarray) -> np.ndarray:
    return np.array([matrix[2, 1], matrix[0, 2], matrix[1, 0]])


def compute_wrench_enu(
    mass: float,
    inertia_body: np.ndarray,
    center_of_mass_body: np.ndarray,
    position_world: np.ndarray,
    velocity_world: np.ndarray,
    rotation_body_to_world: np.ndarray,
    angular_velocity_body: np.ndarray,
    target_world: np.ndarray,
) -> tuple[np.ndarray, np.ndarray]:
    """Return world force and origin-applied world torque."""
    position_kp = np.array([3.2, 3.2, 4.5])
    position_kd = np.array([3.4, 3.4, 4.0])
    attitude_kp = np.array([0.55, 0.55, 0.35])
    attitude_kd = np.array([0.16, 0.16, 0.10])

    acceleration_command = (
        position_kp * (target_world - position_world)
        - position_kd * velocity_world
    )
    force_world = mass * (
        acceleration_command + np.array([0.0, 0.0, 9.81])
    )

    attitude_error_body = 0.5 * _vee(
        rotation_body_to_world - rotation_body_to_world.T
    )
    torque_body_com = (
        -attitude_kp * attitude_error_body
        - attitude_kd * angular_velocity_body
        + np.cross(
            angular_velocity_body,
            inertia_body @ angular_velocity_body,
        )
    )
    torque_world_com = rotation_body_to_world @ torque_body_com

    # ApplyLinkWrench applies force at the canonical link origin.  Compensate
    # the moment arm so the commanded net moment is about the composite CoM.
    origin_to_com_world = rotation_body_to_world @ center_of_mass_body
    torque_world_origin = (
        torque_world_com + np.cross(origin_to_com_world, force_world)
    )
    return force_world, torque_world_origin


class WrenchController(Node):
    def __init__(
        self,
        urdf: Path,
        target: np.ndarray,
        entity_name: str,
        persistent_bootstrap: bool = False,
    ):
        super().__init__("my_drone_gazebo_wrench_controller")
        model = UrdfModel(urdf)
        self.mass, self.center, self.inertia = model.mass_properties({})
        self.target = target
        self.entity_name = entity_name
        self.persistent_bootstrap = persistent_bootstrap
        self.last_message = None
        self.publisher = self.create_publisher(
            EntityWrench,
            (
                "/world/flight_world/wrench/persistent"
                if persistent_bootstrap
                else "/world/flight_world/wrench"
            ),
            10,
        )
        self.clear_publisher = (
            self.create_publisher(
                Entity,
                "/world/flight_world/wrench/clear",
                10,
            )
            if persistent_bootstrap
            else None
        )
        self.bootstrap_timer = (
            self.create_timer(0.01, self.publish_bootstrap_once)
            if persistent_bootstrap
            else None
        )
        self.subscription = self.create_subscription(
            Odometry,
            "/model/my_drone/odometry",
            self.on_odometry,
            10,
        )
        self.get_logger().info(
            f"Controlling {entity_name} to ENU {target.tolist()}, "
            f"mass={self.mass:.6f} kg"
        )

    def _message(self, force: np.ndarray, torque: np.ndarray):
        message = EntityWrench()
        message.header.stamp = self.get_clock().now().to_msg()
        message.entity.name = self.entity_name
        message.entity.type = Entity.LINK
        message.wrench.force.x = float(force[0])
        message.wrench.force.y = float(force[1])
        message.wrench.force.z = float(force[2])
        message.wrench.torque.x = float(torque[0])
        message.wrench.torque.y = float(torque[1])
        message.wrench.torque.z = float(torque[2])
        return message

    def publish_bootstrap_once(self) -> None:
        if (
            self.last_message is not None
            or self.publisher.get_subscription_count() == 0
        ):
            return
        force = self.mass * np.array([0.0, 0.0, 9.81])
        torque = np.cross(self.center, force)
        self.last_message = self._message(force, torque)
        self.publisher.publish(self.last_message)
        self.bootstrap_timer.cancel()

    def on_odometry(self, message) -> None:
        if self.persistent_bootstrap:
            return
        pose = message.pose.pose
        twist = message.twist.twist
        position = np.array(
            [pose.position.x, pose.position.y, pose.position.z]
        )
        velocity_body = np.array(
            [twist.linear.x, twist.linear.y, twist.linear.z]
        )
        quaternion = np.array(
            [
                pose.orientation.x,
                pose.orientation.y,
                pose.orientation.z,
                pose.orientation.w,
            ]
        )
        rotation = quaternion_matrix_xyzw(quaternion)
        # nav_msgs/Odometry expresses twist in child_frame_id (body FLU),
        # while the position loop and commanded wrench use world ENU.
        velocity = rotation @ velocity_body
        angular_velocity = np.array(
            [twist.angular.x, twist.angular.y, twist.angular.z]
        )
        state = np.concatenate(
            [position, velocity, quaternion, angular_velocity]
        )
        if not np.all(np.isfinite(state)) or np.linalg.norm(quaternion) < 1e-9:
            self.get_logger().error(
                "Ignoring non-finite Gazebo odometry; no wrench was published."
            )
            return
        force, torque = compute_wrench_enu(
            self.mass,
            self.inertia,
            self.center,
            position,
            velocity,
            rotation,
            angular_velocity,
            self.target,
        )
        self.last_message = self._message(force, torque)
        self.publisher.publish(self.last_message)

    def stop_wrench(self) -> None:
        if not rclpy.ok():
            return
        if self.persistent_bootstrap:
            message = Entity()
            message.name = self.entity_name
            message.type = Entity.LINK
            for _ in range(3):
                self.clear_publisher.publish(message)
                rclpy.spin_once(self, timeout_sec=0.05)
        else:
            self.publisher.publish(
                self._message(np.zeros(3), np.zeros(3))
            )


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--urdf", type=Path, default=_default_urdf())
    parser.add_argument("--target", nargs=3, type=float, default=[0.0, 0.0, 1.0])
    parser.add_argument("--entity-name", default="my_drone::drone_base_link")
    parser.add_argument("--persistent-bootstrap", action="store_true")
    parsed, ros_arguments = parser.parse_known_args()
    if rclpy is None:
        raise SystemExit("ROS 2 Python packages are not available.")
    rclpy.init(args=ros_arguments)
    node = WrenchController(
        parsed.urdf,
        np.asarray(parsed.target, dtype=float),
        parsed.entity_name,
        parsed.persistent_bootstrap,
    )
    try:
        rclpy.spin(node)
    finally:
        node.stop_wrench()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
