"""Rule-based safety supervisor for the aerial manipulator.

PX4 remains responsible for flight stabilization.  This node observes the
Gazebo vehicle, arm reaction wrench and motor commands, then publishes a
bounded arm-speed recommendation and a high-level safety state.  It is
advisory by design: no motor, arm or PX4 command is published from this node.
"""

from __future__ import annotations

import argparse
from dataclasses import asdict, dataclass
from enum import IntEnum
import json
import math
import time

import numpy as np


class SupervisorState(IntEnum):
    NORMAL = 0
    SLOW = 1
    PAUSE = 2
    RETRACT = 3
    LAND = 4


@dataclass(frozen=True)
class SupervisorInput:
    horizontal_error_m: float = 0.0
    vertical_error_m: float = 0.0
    tilt_rad: float = 0.0
    arm_torque_nm: float = 0.0
    motor_saturation_fraction: float = 0.0


@dataclass(frozen=True)
class SupervisorDecision:
    state: SupervisorState
    arm_speed_scale: float
    reasons: tuple[str, ...]


_LIMITS = {
    SupervisorState.SLOW: SupervisorInput(0.30, 0.25, 0.18, 0.15, 0.125),
    SupervisorState.PAUSE: SupervisorInput(0.60, 0.50, 0.30, 0.30, 0.375),
    SupervisorState.RETRACT: SupervisorInput(0.90, 0.80, 0.40, 0.45, 0.50),
    SupervisorState.LAND: SupervisorInput(2.00, 1.50, 0.70, 0.80, 0.75),
}

_SPEED_SCALE = {
    SupervisorState.NORMAL: 1.0,
    SupervisorState.SLOW: 0.45,
    SupervisorState.PAUSE: 0.0,
    SupervisorState.RETRACT: 0.0,
    SupervisorState.LAND: 0.0,
}


def _validate(sample: SupervisorInput) -> None:
    values = np.asarray(list(asdict(sample).values()), dtype=float)
    if not np.all(np.isfinite(values)) or np.any(values < 0.0):
        raise ValueError("supervisor inputs must be finite and non-negative")
    if sample.motor_saturation_fraction > 1.0:
        raise ValueError("motor saturation fraction must be <= 1")


def evaluate_supervisor(sample: SupervisorInput) -> SupervisorDecision:
    """Classify one observation using the most severe exceeded boundary."""
    _validate(sample)
    metrics = asdict(sample)
    labels = {
        "horizontal_error_m": "horizontal drift",
        "vertical_error_m": "vertical drift",
        "tilt_rad": "vehicle tilt",
        "arm_torque_nm": "arm reaction torque",
        "motor_saturation_fraction": "motor saturation",
    }
    selected = SupervisorState.NORMAL
    reasons: list[str] = []
    for state in (
        SupervisorState.SLOW,
        SupervisorState.PAUSE,
        SupervisorState.RETRACT,
        SupervisorState.LAND,
    ):
        limits = asdict(_LIMITS[state])
        exceeded = [labels[name] for name, value in metrics.items()
                    if value >= limits[name]]
        if exceeded:
            selected = state
            reasons = exceeded
    return SupervisorDecision(selected, _SPEED_SCALE[selected], tuple(reasons))


class RuleSupervisorPolicy:
    """Add recovery dwell so noisy samples cannot rapidly resume arm motion."""

    def __init__(self, recovery_hold_s: float = 2.0) -> None:
        self.recovery_hold_s = max(0.0, float(recovery_hold_s))
        self.state = SupervisorState.NORMAL
        self.recovery_started_s: float | None = None

    def update(self, sample: SupervisorInput, now_s: float | None = None) -> SupervisorDecision:
        now = time.monotonic() if now_s is None else float(now_s)
        requested = evaluate_supervisor(sample)
        if requested.state >= self.state:
            self.state = requested.state
            self.recovery_started_s = None
        elif self.recovery_started_s is None:
            self.recovery_started_s = now
        elif now - self.recovery_started_s >= self.recovery_hold_s:
            self.state = requested.state
            self.recovery_started_s = None
        reasons = requested.reasons if self.state == requested.state else ("recovery hold",)
        return SupervisorDecision(self.state, _SPEED_SCALE[self.state], reasons)


def quaternion_tilt_rad(x: float, y: float, z: float, w: float) -> float:
    q = np.asarray([x, y, z, w], dtype=float)
    if not np.all(np.isfinite(q)) or np.linalg.norm(q) < 1.0e-9:
        return 0.0
    x, y, z, w = q / np.linalg.norm(q)
    body_up_z = 1.0 - 2.0 * (x * x + y * y)
    return math.acos(float(np.clip(body_up_z, -1.0, 1.0)))


def main() -> None:
    # ROS imports stay local so the policy can be unit-tested without a ROS graph.
    import rclpy
    from actuator_msgs.msg import Actuators
    from geometry_msgs.msg import WrenchStamped
    from nav_msgs.msg import Odometry
    from rclpy.node import Node
    from std_msgs.msg import Float32, String

    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--rate-hz", type=float, default=5.0)
    parser.add_argument("--recovery-hold-s", type=float, default=2.0)
    parser.add_argument("--duration-s", type=float, default=0.0,
                        help="Exit after this wall-clock duration; 0 runs continuously")
    parsed, ros_args = parser.parse_known_args()
    rclpy.init(args=ros_args)

    class SupervisorNode(Node):
        def __init__(self) -> None:
            super().__init__("my_drone_rule_supervisor")
            self.policy = RuleSupervisorPolicy(parsed.recovery_hold_s)
            self.anchor: np.ndarray | None = None
            self.position = np.zeros(3)
            self.tilt = 0.0
            self.torque = 0.0
            self.saturation = 0.0
            self.last_odom = 0.0
            self.last_wrench = 0.0
            self.last_motors = 0.0
            self.started = time.monotonic()
            self.previous_state: SupervisorState | None = None
            self.scale_pub = self.create_publisher(
                Float32, "/my_drone/supervisor/arm_speed_scale", 10
            )
            self.action_pub = self.create_publisher(
                String, "/my_drone/supervisor/action", 10
            )
            self.create_subscription(
                Odometry, "/model/my_drone/odometry", self.on_odometry, 20
            )
            self.create_subscription(
                WrenchStamped,
                "/my_drone/arm_reaction_wrench_body",
                self.on_wrench,
                20,
            )
            self.create_subscription(
                Actuators, "/my_drone/command/motor_speed", self.on_motors, 20
            )
            self.create_timer(1.0 / max(1.0, parsed.rate_hz), self.tick)

        def on_odometry(self, message: Odometry) -> None:
            pose = message.pose.pose
            self.position = np.asarray(
                [pose.position.x, pose.position.y, pose.position.z], dtype=float
            )
            if self.anchor is None:
                self.anchor = self.position.copy()
            q = pose.orientation
            self.tilt = quaternion_tilt_rad(q.x, q.y, q.z, q.w)
            self.last_odom = time.monotonic()

        def on_wrench(self, message: WrenchStamped) -> None:
            torque = message.wrench.torque
            self.torque = float(np.linalg.norm([torque.x, torque.y, torque.z]))
            self.last_wrench = time.monotonic()

        def on_motors(self, message: Actuators) -> None:
            values = np.asarray(message.velocity[:8], dtype=float)
            self.saturation = (
                float(np.mean(values >= 999.0)) if values.size else 0.0
            )
            self.last_motors = time.monotonic()

        def tick(self) -> None:
            now = time.monotonic()
            if parsed.duration_s > 0.0 and now - self.started >= parsed.duration_s:
                rclpy.shutdown()
                return
            if self.anchor is None or now - self.last_odom > 1.0:
                return
            delta = self.position - self.anchor
            sample = SupervisorInput(
                horizontal_error_m=float(np.linalg.norm(delta[:2])),
                vertical_error_m=abs(float(delta[2])),
                tilt_rad=self.tilt,
                arm_torque_nm=self.torque if now - self.last_wrench < 0.8 else 0.0,
                motor_saturation_fraction=(
                    self.saturation if now - self.last_motors < 0.8 else 0.0
                ),
            )
            decision = self.policy.update(sample, now)
            self.scale_pub.publish(Float32(data=decision.arm_speed_scale))
            payload = {
                "state": decision.state.name,
                "arm_speed_scale": decision.arm_speed_scale,
                "reasons": list(decision.reasons),
                "metrics": asdict(sample),
                "advisory_only": True,
            }
            self.action_pub.publish(String(data=json.dumps(payload, sort_keys=True)))
            if decision.state != self.previous_state:
                self.get_logger().warning(
                    "SUPERVISOR_STATE " + json.dumps(payload, sort_keys=True)
                )
                self.previous_state = decision.state

    node = SupervisorNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
