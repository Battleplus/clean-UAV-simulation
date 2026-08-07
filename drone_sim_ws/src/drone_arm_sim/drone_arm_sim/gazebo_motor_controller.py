"""Closed-loop eight-motor controller for the Gazebo ``my_drone`` model.

This is a Gazebo plumbing and allocation validator, not a replacement for
PX4.  Control is computed in PX4 NED/FRD coordinates and converted from the
Gazebo ENU/FLU odometry frame.
"""

from __future__ import annotations

import argparse
import json
from pathlib import Path

from actuator_msgs.msg import Actuators
from nav_msgs.msg import Odometry
import numpy as np
import rclpy
from rclpy.node import Node

from drone_arm_sim.allocation_analysis import _default_config, allocation_matrix
from drone_arm_sim.flight_control_demo import _attitude_error, allocate_bounded
from drone_arm_sim.gazebo_wrench_controller import quaternion_matrix_xyzw
from drone_arm_sim.model_analysis import UrdfModel, _default_urdf


ENU_TO_NED = np.array([[0.0, 1.0, 0.0], [1.0, 0.0, 0.0], [0.0, 0.0, -1.0]])
FRD_TO_FLU = np.diag([1.0, -1.0, -1.0])
LEVEL_EAST_FACING_FRD_TO_NED = ENU_TO_NED @ FRD_TO_FLU


class GazeboMotorController(Node):
    def __init__(
        self,
        urdf: Path,
        config_path: Path,
        target_ned: np.ndarray,
        direct_thrust: bool = False,
    ):
        super().__init__("my_drone_gazebo_motor_controller")
        config = json.loads(config_path.read_text(encoding="utf-8"))
        model = UrdfModel(urdf)
        self.mass, center_flu, inertia_flu = model.mass_properties({})
        self.center_frd = FRD_TO_FLU @ center_flu
        self.inertia_frd = FRD_TO_FLU @ inertia_flu @ FRD_TO_FLU
        self.matrix = allocation_matrix(config)
        self.motor_indices = np.asarray(
            [int(rotor.get("motor", index + 1)) - 1
             for index, rotor in enumerate(config["rotors"])],
            dtype=int,
        )
        self.direct_thrust = direct_thrust
        self.motor_constant = (
            None
            if direct_thrust
            else float(config["motor_constant_n_per_rad_s_squared"])
        )
        self.minimum_thrust = float(config.get("minimum_thrust_n", 0.0))
        self.maximum_thrust = float(config["maximum_thrust_n"])
        self.gravity = float(config.get("gravity_m_s2", 9.81))
        self.target_ned = target_ned
        self.previous_thrust = np.zeros(8)
        self.logged_first_allocation = False
        self.publisher = self.create_publisher(
            Actuators,
            (
                "/model/my_drone/command/motor_speed"
                if direct_thrust
                else "/my_drone/command/motor_speed"
            ),
            10,
        )
        self.subscription = self.create_subscription(
            Odometry, "/model/my_drone/odometry", self.on_odometry, 20
        )
        self.get_logger().info(
            f"Controlling eight motors to NED {target_ned.tolist()}, "
            f"mass={self.mass:.6f} kg"
        )

    def on_odometry(self, message: Odometry) -> None:
        pose = message.pose.pose
        twist = message.twist.twist
        position_enu = np.array(
            [pose.position.x, pose.position.y, pose.position.z]
        )
        velocity_enu = np.array(
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
        angular_velocity_flu = np.array(
            [twist.angular.x, twist.angular.y, twist.angular.z]
        )
        state = np.concatenate(
            [position_enu, velocity_enu, quaternion, angular_velocity_flu]
        )
        if not np.all(np.isfinite(state)) or np.linalg.norm(quaternion) < 1e-9:
            return
        # Gazebo's first odometry sample after spawning at a non-zero pose can
        # encode the insertion jump as roughly 1000 m/s.  It is not aircraft
        # motion and causes a destructive derivative kick.  Wait one update
        # for physically meaningful twist instead.
        if (
            np.linalg.norm(velocity_enu) > 10.0
            or np.linalg.norm(angular_velocity_flu) > 10.0
        ):
            self.get_logger().warning(
                "Ignoring Gazebo spawn transient: "
                f"linear={velocity_enu.tolist()}, "
                f"angular={angular_velocity_flu.tolist()}"
            )
            return

        position_ned = ENU_TO_NED @ position_enu
        velocity_ned = ENU_TO_NED @ velocity_enu
        rotation_flu_to_enu = quaternion_matrix_xyzw(quaternion)
        rotation_frd_to_ned = (
            ENU_TO_NED @ rotation_flu_to_enu @ FRD_TO_FLU
        )
        angular_velocity_frd = FRD_TO_FLU @ angular_velocity_flu

        acceleration_command = (
            np.array([3.2, 3.2, 4.5]) * (self.target_ned - position_ned)
            - np.array([3.4, 3.4, 4.0]) * velocity_ned
        )
        gravity_ned = np.array([0.0, 0.0, self.gravity])
        force_ned = self.mass * (acceleration_command - gravity_ned)
        force_frd = rotation_frd_to_ned.T @ force_ned

        attitude_error = _attitude_error(
            rotation_frd_to_ned, LEVEL_EAST_FACING_FRD_TO_NED
        )
        torque_com_frd = (
            -np.array([0.55, 0.55, 0.35]) * attitude_error
            - np.array([0.16, 0.16, 0.10]) * angular_velocity_frd
            + np.cross(
                angular_velocity_frd,
                self.inertia_frd @ angular_velocity_frd,
            )
        )
        torque_origin_frd = torque_com_frd + np.cross(
            self.center_frd, force_frd
        )
        desired_wrench = np.concatenate((force_frd, torque_origin_frd))
        thrust, _ = allocate_bounded(
            self.matrix,
            desired_wrench,
            self.minimum_thrust,
            self.maximum_thrust,
            self.previous_thrust,
        )
        if not self.logged_first_allocation:
            self.get_logger().info(
                "first allocation: "
                f"position_ned={position_ned.tolist()}, "
                f"velocity_ned={velocity_ned.tolist()}, "
                f"force_frd={force_frd.tolist()}, "
                f"torque_origin_frd={torque_origin_frd.tolist()}, "
                f"thrust_config_order={thrust.tolist()}"
            )
            self.logged_first_allocation = True
        self.previous_thrust = thrust
        output = Actuators()
        if self.direct_thrust:
            # Allocation columns follow config order, which need not be
            # numeric motor order (the measured clockwise order is
            # 1,3,8,4,2,6,7,5).  The direct motor topic is indexed by
            # PX4 motor number minus one.
            normalized = np.zeros(8)
            normalized[self.motor_indices] = (
                np.maximum(thrust, 0.0) / self.maximum_thrust
            )
            output.normalized = normalized.tolist()
        else:
            speed = np.sqrt(
                np.maximum(thrust, 0.0) / self.motor_constant
            )
            output.velocity = speed.tolist()
        self.publisher.publish(output)


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--urdf", type=Path, default=_default_urdf())
    parser.add_argument("--config", type=Path, default=_default_config())
    parser.add_argument("--target", nargs=3, type=float, default=[0.0, 0.0, -1.0])
    parser.add_argument("--direct-thrust", action="store_true")
    parsed, ros_arguments = parser.parse_known_args()
    rclpy.init(args=ros_arguments)
    node = GazeboMotorController(
        parsed.urdf,
        parsed.config,
        np.asarray(parsed.target, dtype=float),
        parsed.direct_thrust,
    )
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
