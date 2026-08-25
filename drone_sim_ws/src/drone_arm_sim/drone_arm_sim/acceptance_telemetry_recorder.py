"""Record high-rate flight, motor and compensation evidence as JSON lines."""

from __future__ import annotations

import argparse
import json
import os
from pathlib import Path
import time

from nav_msgs.msg import Odometry
from px4_msgs.msg import (
    ActuatorOutputs,
    OffboardControlMode,
    TrajectorySetpoint,
    VehicleStatus,
)
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from std_msgs.msg import Bool, String


class AcceptanceTelemetryRecorder(Node):
    def __init__(self, output: Path, maximum_rate_hz: float) -> None:
        super().__init__("acceptance_telemetry_recorder")
        output.parent.mkdir(parents=True, exist_ok=True)
        self.stream = output.open("w", encoding="utf-8", buffering=1)
        self.minimum_interval_s = 1.0 / max(1.0, maximum_rate_hz)
        self.last_odom_s = 0.0
        self.last_motor_s = 0.0
        self.sequence = 0
        self.create_subscription(
            Odometry, "/model/my_drone/odometry", self._on_odometry, 50
        )
        px4_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )
        latest_safety_level_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self.create_subscription(
            ActuatorOutputs,
            "/fmu/out/actuator_outputs",
            self._on_motor,
            px4_qos,
        )
        self.create_subscription(
            VehicleStatus,
            "/fmu/out/vehicle_status_v4",
            self._on_vehicle_status,
            px4_qos,
        )
        self.create_subscription(
            String,
            "/my_drone/base1_reallocator/state",
            self._on_reallocator,
            50,
        )
        self.create_subscription(
            Bool, "/my_drone/arm_motion_active", self._on_arm_motion, 20
        )
        self.create_subscription(
            Bool, "/my_drone/arm_direct_xy_ready", self._on_direct_xy_ready, 20
        )
        self.create_subscription(
            Bool, "/my_drone/arm_motion_inhibit", self._on_arm_motion_inhibit, 20
        )
        self.create_subscription(
            String,
            (
                "/my_drone/arm_direct_xy_guardian/state"
                if os.environ.get("ARM_DIRECT_XY_EXTERNAL_GUARDIAN", "false").lower()
                in {"1", "true", "yes", "on"}
                else "/my_drone/arm_direct_xy_state"
            ),
            self._on_direct_xy_state,
            latest_safety_level_qos,
        )
        self.create_subscription(
            String,
            "/my_drone/arm_direct_xy_controller_intent",
            self._on_direct_xy_controller_intent,
            latest_safety_level_qos,
        )
        # These are the actual ROS -> PX4 input messages.  Recording their
        # finite-value masks is the authoritative proof that mixed-axis mode
        # releases XY velocity control while retaining Z/yaw control.
        self.create_subscription(
            OffboardControlMode,
            "/fmu/in/offboard_control_mode",
            self._on_offboard_control_mode,
            50,
        )
        self.create_subscription(
            TrajectorySetpoint,
            "/fmu/in/trajectory_setpoint",
            self._on_trajectory_setpoint,
            50,
        )
        self._write({"kind": "recorder", "event": "started"})

    def _write(self, sample: dict) -> None:
        sample = {
            "sequence": self.sequence,
            "recorder_monotonic_s": time.monotonic(),
            **sample,
        }
        self.sequence += 1
        self.stream.write(json.dumps(sample, separators=(",", ":")) + "\n")

    def _on_odometry(self, message: Odometry) -> None:
        now_s = time.monotonic()
        if now_s - self.last_odom_s < self.minimum_interval_s:
            return
        self.last_odom_s = now_s
        pose = message.pose.pose
        twist = message.twist.twist
        self._write(
            {
                "kind": "odometry",
                "position_world_enu_m": [
                    pose.position.x,
                    pose.position.y,
                    pose.position.z,
                ],
                "quaternion_body_to_world_xyzw": [
                    pose.orientation.x,
                    pose.orientation.y,
                    pose.orientation.z,
                    pose.orientation.w,
                ],
                "linear_velocity_body_flu_m_s": [
                    twist.linear.x,
                    twist.linear.y,
                    twist.linear.z,
                ],
                "angular_velocity_body_flu_rad_s": [
                    twist.angular.x,
                    twist.angular.y,
                    twist.angular.z,
                ],
            }
        )

    def _on_motor(self, message: ActuatorOutputs) -> None:
        now_s = time.monotonic()
        if now_s - self.last_motor_s < self.minimum_interval_s:
            return
        self.last_motor_s = now_s
        count = min(int(message.noutputs), 8)
        self._write(
            {
                "kind": "px4_actuator_outputs",
                "outputs": [float(value) for value in message.output[:count]],
            }
        )

    def _on_vehicle_status(self, message: VehicleStatus) -> None:
        self._write(
            {
                "kind": "vehicle_status",
                "arming_state": int(message.arming_state),
                "nav_state": int(message.nav_state),
                "failsafe": bool(message.failsafe),
            }
        )

    def _on_reallocator(self, message: String) -> None:
        try:
            state = json.loads(message.data)
        except (TypeError, json.JSONDecodeError):
            self._write({"kind": "reallocator_invalid"})
            return
        self._write({"kind": "reallocator", "state": state})

    def _on_arm_motion(self, message: Bool) -> None:
        self._write({"kind": "arm_motion", "active": bool(message.data)})

    def _on_direct_xy_ready(self, message: Bool) -> None:
        self._write({"kind": "arm_direct_xy_ready", "ready": bool(message.data)})

    def _on_arm_motion_inhibit(self, message: Bool) -> None:
        self._write({"kind": "arm_motion_inhibit", "inhibit": bool(message.data)})

    def _on_direct_xy_state(self, message: String) -> None:
        try:
            state = json.loads(message.data)
        except (TypeError, json.JSONDecodeError):
            self._write({"kind": "arm_direct_xy_state_invalid"})
            return
        self._write({"kind": "arm_direct_xy_state", "state": state})

    def _on_direct_xy_controller_intent(self, message: String) -> None:
        try:
            state = json.loads(message.data)
        except (TypeError, json.JSONDecodeError):
            self._write({"kind": "arm_direct_xy_controller_intent_invalid"})
            return
        self._write({"kind": "arm_direct_xy_controller_intent", "state": state})

    def _on_offboard_control_mode(self, message: OffboardControlMode) -> None:
        self._write(
            {
                "kind": "px4_offboard_control_mode_input",
                "px4_timestamp_us": int(message.timestamp),
                "position": bool(message.position),
                "velocity": bool(message.velocity),
                "acceleration": bool(message.acceleration),
                "attitude": bool(message.attitude),
                "body_rate": bool(message.body_rate),
                "thrust_and_torque": bool(message.thrust_and_torque),
                "direct_actuator": bool(message.direct_actuator),
            }
        )

    def _on_trajectory_setpoint(self, message: TrajectorySetpoint) -> None:
        self._write(
            {
                "kind": "px4_trajectory_setpoint_input",
                "px4_timestamp_us": int(message.timestamp),
                "position": [float(value) for value in message.position],
                "velocity": [float(value) for value in message.velocity],
                "acceleration": [float(value) for value in message.acceleration],
                "jerk": [float(value) for value in message.jerk],
                "yaw": float(message.yaw),
                "yawspeed": float(message.yawspeed),
            }
        )

    def close(self) -> None:
        if not self.stream.closed:
            self._write({"kind": "recorder", "event": "stopped"})
            self.stream.close()


def main(args=None) -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--maximum-rate-hz", type=float, default=100.0)
    parsed, ros_arguments = parser.parse_known_args(args)
    if parsed.maximum_rate_hz <= 0.0:
        parser.error("--maximum-rate-hz must be positive")
    rclpy.init(args=ros_arguments)
    node = AcceptanceTelemetryRecorder(parsed.output, parsed.maximum_rate_hz)
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
