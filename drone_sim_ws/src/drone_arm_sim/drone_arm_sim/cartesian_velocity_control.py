"""Persistent jerk-limited Cartesian velocity control for the SO101 tool.

Keyboard commands latch an endpoint velocity; they do not encode a position
step.  A persistent node integrates that velocity into short minimum-jerk
joint segments, runs every segment through the Base1 trajectory preflight,
and stops safely when IK, collision or motor authority is unavailable.
"""

from __future__ import annotations

import argparse
import json
import math
from pathlib import Path
import time

from builtin_interfaces.msg import Duration
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, String
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from drone_arm_sim.arm_preset_control import flight_config_path, motion_reference_path
from drone_arm_sim.cartesian_velocity_profile import (
    COMMAND_DIRECTIONS,
    JerkLimitedVelocity3D,
    joint_velocity_sample_is_stable,
)
from drone_arm_sim.cartesian_velocity_kinematics import solve_velocity_step_ik
from drone_arm_sim.cartesian_arm_demo import (
    IK_POSITION_TOLERANCE_M,
    JOINT_NAMES,
    TOOL_FORWARD_AXIS_LOCAL,
    _formal_urdf,
)
from drone_arm_sim.model_analysis import UrdfModel
from drone_arm_sim.trajectory_preflight import TrajectoryPreflight


JOINT_SETTLE_VELOCITY_RAD_S = 0.01
JOINT_SETTLE_SAMPLE_COUNT = 5
JOINT_STATE_FRESHNESS_S = 0.5


class CartesianVelocityControl(Node):
    def __init__(
        self,
        urdf: Path,
        *,
        maximum_speed_m_s: float,
        maximum_acceleration_m_s2: float,
        maximum_jerk_m_s3: float,
        horizon_s: float,
        control_hz: float,
    ) -> None:
        super().__init__("cartesian_arm_velocity_control")
        self.model = UrdfModel(urdf)
        self.preflight = TrajectoryPreflight(
            urdf, motion_reference_path(), flight_config_path()
        )
        self.limiter = JerkLimitedVelocity3D(
            maximum_speed_m_s, maximum_acceleration_m_s2, maximum_jerk_m_s3
        )
        self.horizon_s = max(0.4, float(horizon_s))
        self.positions: dict[str, float] = {}
        self.velocities: dict[str, float] = {}
        self.last_joint_state_monotonic = -math.inf
        self.last_velocity_state_monotonic = -math.inf
        self.settled_joint_samples = 0
        self.closing_event: str | None = None
        self.next_segment_monotonic = 0.0
        self.last_tick_monotonic = time.monotonic()
        self.active = False
        self.last_motion_publish_monotonic = 0.0
        self.publisher = self.create_publisher(
            JointTrajectory, "/arm_controller/joint_trajectory", 10
        )
        self.motion_publisher = self.create_publisher(
            Bool, "/my_drone/arm_motion_active", 10
        )
        self.status_publisher = self.create_publisher(
            String, "/my_drone/arm_cartesian_velocity_state", 10
        )
        self.create_subscription(JointState, "/joint_states", self._state_cb, 20)
        self.create_subscription(
            String,
            "/my_drone/arm_cartesian_velocity_command",
            self._command_cb,
            20,
        )
        self.create_timer(1.0 / max(2.0, float(control_hz)), self._tick)
        self.get_logger().info(
            "ARM_CARTESIAN_VELOCITY_READY "
            + json.dumps(
                {
                    "maximum_speed_m_s": self.limiter.maximum_speed,
                    "maximum_acceleration_m_s2": self.limiter.maximum_acceleration,
                    "maximum_jerk_m_s3": self.limiter.maximum_jerk,
                    "horizon_s": self.horizon_s,
                },
                sort_keys=True,
            )
        )

    def _state_cb(self, message: JointState) -> None:
        now = time.monotonic()
        previous_positions = dict(self.positions)
        previous_time = self.last_joint_state_monotonic
        for name, value in zip(message.name, message.position):
            if math.isfinite(float(value)):
                self.positions[name] = float(value)
        measured_velocities = {
            name: float(value)
            for name, value in zip(message.name, message.velocity)
            if math.isfinite(float(value))
        }
        if (
            len(message.velocity) == len(message.name)
            and all(name in measured_velocities for name in JOINT_NAMES)
        ):
            self.velocities = measured_velocities
            self.last_velocity_state_monotonic = now
        elif (
            math.isfinite(previous_time)
            and now > previous_time
            and all(name in previous_positions for name in JOINT_NAMES)
            and all(name in self.positions for name in JOINT_NAMES)
        ):
            dt = now - previous_time
            self.velocities = {
                name: (self.positions[name] - previous_positions[name]) / dt
                for name in JOINT_NAMES
            }
            self.last_velocity_state_monotonic = now
        else:
            self.velocities = {}
        self.last_joint_state_monotonic = now

        # Count only samples received after Cartesian generation has stopped.
        # This prevents an old pre-motion run of zeroes from closing a session
        # while the controller still has a physical segment in flight.
        if self.active and self.limiter.stopped():
            if joint_velocity_sample_is_stable(self.velocities, JOINT_NAMES):
                self.settled_joint_samples += 1
            else:
                self.settled_joint_samples = 0
        else:
            self.settled_joint_samples = 0

    def _publish_state(self, event: str, **extra) -> None:
        report = {
            "event": event,
            "active": self.active,
            "target_velocity_body_flu_m_s": self.limiter.target.tolist(),
            "velocity_body_flu_m_s": self.limiter.velocity.tolist(),
            **extra,
        }
        self.status_publisher.publish(String(data=json.dumps(report, sort_keys=True)))
        self.get_logger().info(
            "ARM_CARTESIAN_VELOCITY_STATE "
            + json.dumps(report, sort_keys=True)
        )

    def _command_cb(self, message: String) -> None:
        command = str(message.data).strip().lower()
        if command in COMMAND_DIRECTIONS:
            self.limiter.set_direction(COMMAND_DIRECTIONS[command])
            self.active = True
            self.closing_event = None
            self.settled_joint_samples = 0
            self._publish_state("velocity_latched", command=command)
        elif command in {"stop", "zero"}:
            self.limiter.set_direction(None)
            if self.active:
                self.closing_event = "stopped"
                self.settled_joint_samples = 0
            self._publish_state("decelerating")
        elif command in {"disable", "exit"}:
            self.limiter.set_direction(None)
            self.limiter.velocity[:] = 0.0
            self.limiter.acceleration[:] = 0.0
            if self.active:
                self.closing_event = "disabled"
                self.settled_joint_samples = 0
                self._publish_state("disabling")
            else:
                self._publish_state("disabled")
        else:
            self._publish_state("command_rejected", command=command)

    def _reject_and_stop(self, reason: str, **extra) -> None:
        self.limiter.set_direction(None)
        self.limiter.velocity[:] = 0.0
        self.limiter.acceleration[:] = 0.0
        self.closing_event = "stopped_after_rejection"
        self.settled_joint_samples = 0
        self._publish_state("segment_rejected", reason=reason, **extra)

    def _publish_segment(self, target: dict[str, float], duration_s: float) -> None:
        message = JointTrajectory()
        message.joint_names = JOINT_NAMES
        point = JointTrajectoryPoint()
        point.positions = [float(target[name]) for name in JOINT_NAMES]
        point.velocities = [0.0] * len(JOINT_NAMES)
        point.accelerations = [0.0] * len(JOINT_NAMES)
        seconds = int(duration_s)
        point.time_from_start = Duration(
            sec=seconds, nanosec=int(round((duration_s - seconds) * 1.0e9))
        )
        message.points = [point]
        self.publisher.publish(message)

    def _tick(self) -> None:
        now = time.monotonic()
        dt = max(0.0, min(0.2, now - self.last_tick_monotonic))
        self.last_tick_monotonic = now
        velocity = self.limiter.step(dt)
        moving = self.active and not self.limiter.stopped()
        # This persistent server shares the motion-session topic with preset
        # and directional executors.  Publishing False while this server is
        # idle races an active executor's True heartbeat and repeatedly
        # destroys its world-position snapshot.  Heartbeat only while this
        # server owns an active Cartesian motion; the explicit stop/disable
        # paths below still publish the single closing False edge.
        if self.active and now - self.last_motion_publish_monotonic >= 0.2:
            self.motion_publisher.publish(Bool(data=True))
            self.last_motion_publish_monotonic = now
        if self.active and self.limiter.stopped():
            velocity_state_fresh = (
                now - self.last_velocity_state_monotonic
                <= JOINT_STATE_FRESHNESS_S
            )
            if (
                velocity_state_fresh
                and self.settled_joint_samples >= JOINT_SETTLE_SAMPLE_COUNT
            ):
                event = self.closing_event or "stopped"
                self.active = False
                self.closing_event = None
                self.motion_publisher.publish(Bool(data=False))
                self._publish_state(
                    event,
                    settled_joint_samples=self.settled_joint_samples,
                    maximum_measured_joint_velocity_rad_s=max(
                        abs(float(self.velocities[name])) for name in JOINT_NAMES
                    ),
                )
            return
        if not moving or now < self.next_segment_monotonic:
            return
        if not all(name in self.positions for name in JOINT_NAMES):
            self._reject_and_stop("joint_state_unavailable")
            return
        start = {name: float(self.positions[name]) for name in JOINT_NAMES}
        seed = {name: start[name] for name in JOINT_NAMES[:-1]}
        transform, _ = self.model.forward_kinematics("gripper_link", seed)
        target_position = transform[:3, 3] + velocity * self.horizon_s
        target_forward = transform[:3, :3] @ TOOL_FORWARD_AXIS_LOCAL
        target_forward /= np.linalg.norm(target_forward)
        solution, status = solve_velocity_step_ik(
            self.model,
            target_position,
            target_forward,
            seed,
            TOOL_FORWARD_AXIS_LOCAL,
            position_tolerance_m=IK_POSITION_TOLERANCE_M,
        )
        if not bool(status["converged"]):
            self._reject_and_stop(
                "ik", position_error_m=float(status["position_error_m"])
            )
            return
        margin = math.radians(4.0)
        for name, value in solution.items():
            lower, upper = self.model.joint_limits(name)
            if not lower + margin <= value <= upper - margin:
                self._reject_and_stop("joint_limit", joint=name)
                return
            if abs(float(value) - start[name]) > 0.25:
                self._reject_and_stop("joint_step", joint=name)
                return
        target = {**solution, "gripper": start["gripper"]}
        maximum_joint_step = max(
            abs(float(target[name]) - float(start[name])) for name in JOINT_NAMES
        )
        # Bound the interpolation gap rather than relying on a fixed sample
        # count: each segment is checked at <= 0.02 rad joint-space spacing,
        # with at least nine samples for dynamic extrema.
        preflight_sample_count = max(
            9, int(math.ceil(maximum_joint_step / 0.02)) + 1
        )
        preflight = self.preflight.adapt_quintic(
            start,
            target,
            self.horizon_s,
            # Full distance is always tried at every duration first.  The
            # preflight may then shorten an outward/manual segment, but it
            # suppresses shortening automatically for a safety retraction.
            allow_distance_scaling=True,
            time_scales=(1.0, 1.5, 2.0, 3.0, 4.0, 6.0, 8.0, 12.0),
            distance_scales=(0.9, 0.8, 0.7, 0.6, 0.5),
            sample_count=preflight_sample_count,
        )
        if not preflight["accepted"]:
            attempts = preflight["attempts"]
            failure_counts = (
                {}
                if not attempts
                else attempts[-1]["evaluation"]["failure_counts"]
            )
            self._reject_and_stop("preflight", failure_counts=failure_counts)
            return
        selected = preflight["selected"]
        duration = float(selected["effective_duration_s"])
        selected_target = {
            name: float(selected["selected_target"][name]) for name in JOINT_NAMES
        }
        # Publishing the original IK target here would bypass a successful
        # distance-reduction decision.  Always publish the exact target that
        # the dynamics/allocation preflight evaluated.
        self._publish_segment(selected_target, duration)
        self.next_segment_monotonic = now + 0.75 * duration
        selected_seed = {name: selected_target[name] for name in JOINT_NAMES[:-1]}
        selected_transform, _ = self.model.forward_kinematics(
            "gripper_link", selected_seed
        )
        selected_endpoint_delta = selected_transform[:3, 3] - transform[:3, 3]
        self._publish_state(
            "segment_sent",
            duration_s=duration,
            time_scale=float(selected["time_scale"]),
            distance_scale=float(selected["distance_scale"]),
            velocity_scale=float(selected["velocity_scale"]),
            acceleration_scale=float(selected["acceleration_scale"]),
            jerk_scale=float(selected["jerk_scale"]),
            preflight_sample_count=preflight_sample_count,
            is_retraction=bool(preflight["is_retraction"]),
            axis_refinement=bool(status["axis_refinement"]),
            tool_axis_error_rad=float(status["tool_axis_error_rad"]),
            requested_endpoint_delta_body_flu_m=(
                velocity * self.horizon_s
            ).tolist(),
            selected_endpoint_delta_body_flu_m=selected_endpoint_delta.tolist(),
            maximum_joint_velocity_rad_s=float(
                selected["evaluation"]["maximum_joint_velocity_rad_s"]
            ),
            maximum_joint_acceleration_rad_s2=float(
                selected["evaluation"]["maximum_joint_acceleration_rad_s2"]
            ),
            maximum_joint_jerk_rad_s3=float(
                selected["evaluation"]["maximum_joint_jerk_rad_s3"]
            ),
            minimum_overlay_delta_headroom_n=float(
                selected["evaluation"]["minimum_overlay_delta_headroom_n"]
            ),
        )


def main(args=None) -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--urdf", type=Path, default=_formal_urdf())
    parser.add_argument("--speed", type=float, default=0.010)
    parser.add_argument("--acceleration", type=float, default=0.010)
    parser.add_argument("--jerk", type=float, default=0.020)
    parser.add_argument("--horizon", type=float, default=1.0)
    parser.add_argument("--control-hz", type=float, default=10.0)
    parsed, ros_arguments = parser.parse_known_args(args)
    if not 0.001 <= parsed.speed <= 0.03:
        parser.error("--speed must be in [0.001, 0.03] m/s")
    if parsed.acceleration <= 0.0 or parsed.jerk <= 0.0:
        parser.error("--acceleration and --jerk must be positive")
    rclpy.init(args=ros_arguments)
    node = CartesianVelocityControl(
        parsed.urdf,
        maximum_speed_m_s=parsed.speed,
        maximum_acceleration_m_s2=parsed.acceleration,
        maximum_jerk_m_s3=parsed.jerk,
        horizon_s=parsed.horizon,
        control_hz=parsed.control_hz,
    )
    try:
        rclpy.spin(node)
    finally:
        node.motion_publisher.publish(Bool(data=False))
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
