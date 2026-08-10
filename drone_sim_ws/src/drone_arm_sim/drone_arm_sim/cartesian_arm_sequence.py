"""Run multiple Cartesian arm cycles from one persistent ROS node."""

from __future__ import annotations

import argparse
import math
from pathlib import Path
import sys
import time

import rclpy
from px4_msgs.msg import VehicleLocalPosition
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool

from drone_arm_sim.cartesian_arm_demo import (
    CartesianDemoNode,
    _formal_urdf,
    execute_cartesian_cycle,
    execute_cartesian_cycle_gated,
)
from drone_arm_sim.model_analysis import UrdfModel


class CartesianSequenceNode(CartesianDemoNode):
    def __init__(self) -> None:
        super().__init__()
        self.local: VehicleLocalPosition | None = None
        self.last_local_monotonic = 0.0
        self.filtered_velocity = [math.nan, math.nan, math.nan]
        self.last_velocity_filter_monotonic = 0.0
        self.hover_pub = self.create_publisher(Bool, "/my_drone/hover_request", 10)
        px4_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self.create_subscription(
            VehicleLocalPosition,
            "/fmu/out/vehicle_local_position_v1",
            self._local_cb,
            px4_qos,
        )

    def _local_cb(self, message: VehicleLocalPosition) -> None:
        now = time.monotonic()
        raw = [float(message.vx), float(message.vy), float(message.vz)]
        if not all(math.isfinite(value) for value in self.filtered_velocity):
            self.filtered_velocity = raw
        else:
            dt = max(0.0, min(0.1, now - self.last_velocity_filter_monotonic))
            alpha = dt / (0.25 + dt)
            self.filtered_velocity = [
                previous + alpha * (current - previous)
                for previous, current in zip(self.filtered_velocity, raw)
            ]
        self.local = message
        self.last_local_monotonic = now
        self.last_velocity_filter_monotonic = now

    def wait_for_flight_stability(
        self,
        horizontal_limit: float,
        vertical_limit: float,
        hold_s: float,
        timeout_s: float,
        reference_xy: tuple[float, float] | None = None,
        horizontal_position_limit: float | None = None,
    ) -> None:
        stable_since = None
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)
            fresh = time.monotonic() - self.last_local_monotonic < 0.5
            stable = (
                fresh
                and self.local is not None
                and math.hypot(
                    self.filtered_velocity[0], self.filtered_velocity[1]
                ) < horizontal_limit
                and abs(self.filtered_velocity[2]) < vertical_limit
                and (
                    reference_xy is None
                    or horizontal_position_limit is None
                    or math.hypot(
                        float(self.local.x) - reference_xy[0],
                        float(self.local.y) - reference_xy[1],
                    ) < horizontal_position_limit
                )
            )
            if stable:
                if stable_since is None:
                    stable_since = time.monotonic()
                elif time.monotonic() - stable_since >= hold_s:
                    self.get_logger().info(
                        "CARTESIAN_SEQUENCE_STABILITY_PASS "
                        f"horizontal_limit={horizontal_limit:.3f}m/s "
                        f"vertical_limit={vertical_limit:.3f}m/s hold={hold_s:.1f}s"
                    )
                    return
            else:
                stable_since = None
            self.publish_motion(False)
        raise RuntimeError("PX4 did not meet the between-cycle stability gate")


def main(args=None) -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--urdf", type=Path, default=_formal_urdf())
    parser.add_argument("--cycles", type=int, default=2)
    parser.add_argument("--distance", type=float, default=0.12)
    parser.add_argument("--step", type=float, default=0.005)
    parser.add_argument("--duration", type=float, default=8.0)
    parser.add_argument("--hold", type=float, default=3.0)
    parser.add_argument("--joint-margin-deg", type=float, default=5.0)
    parser.add_argument("--require-px4-stable", action="store_true")
    parser.add_argument("--stable-horizontal", type=float, default=0.10)
    parser.add_argument("--stable-vertical", type=float, default=0.08)
    parser.add_argument("--stable-hold", type=float, default=5.0)
    parser.add_argument("--stable-timeout", type=float, default=45.0)
    parser.add_argument("--event-file", type=Path)
    parser.add_argument("--flight-gated-waypoints", action="store_true")
    parser.add_argument("--waypoint-position-limit", type=float, default=0.10)
    parser.add_argument("--waypoint-stable-hold", type=float, default=0.5)
    parser.add_argument("--waypoint-stable-timeout", type=float, default=20.0)
    parsed = parser.parse_args(args)
    if parsed.cycles < 1:
        parser.error("cycles must be at least one")
    if parsed.duration <= 0.0 or parsed.hold < 0.0:
        parser.error("duration must be positive and hold cannot be negative")

    model = UrdfModel(parsed.urdf)
    if parsed.event_file is not None:
        parsed.event_file.parent.mkdir(parents=True, exist_ok=True)
        parsed.event_file.write_text("", encoding="utf-8")

    def record_event(message: str) -> None:
        print(message, flush=True)
        if parsed.event_file is not None:
            with parsed.event_file.open("a", encoding="utf-8") as stream:
                stream.write(message + "\n")
                stream.flush()

    rclpy.init()
    node = CartesianSequenceNode()
    try:
        for cycle in range(1, parsed.cycles + 1):
            if parsed.flight_gated_waypoints:
                while node.local is None or time.monotonic() - node.last_local_monotonic >= 0.5:
                    rclpy.spin_once(node, timeout_sec=0.05)
                cycle_reference_xy = (float(node.local.x), float(node.local.y))

                def waypoint_gate() -> None:
                    node.wait_for_flight_stability(
                        parsed.stable_horizontal,
                        parsed.stable_vertical,
                        parsed.waypoint_stable_hold,
                        parsed.waypoint_stable_timeout,
                        cycle_reference_xy,
                        parsed.waypoint_position_limit,
                    )

                executor = execute_cartesian_cycle_gated
                extra = {"waypoint_gate": waypoint_gate}
            else:
                executor = execute_cartesian_cycle
                extra = {}
            result = executor(
                node,
                model,
                distance=parsed.distance,
                step=parsed.step,
                duration=parsed.duration,
                hold=parsed.hold,
                joint_margin_rad=math.radians(parsed.joint_margin_deg),
                cycle_label=str(cycle),
                **extra,
            )
            record_event(
                "CARTESIAN_SEQUENCE_CYCLE_INTERVAL "
                f"cycle={cycle} start={result['started_monotonic']:.6f} "
                f"end={result['completed_monotonic']:.6f}"
            )
            record_event(f"CARTESIAN_SEQUENCE_CYCLE_COMPLETE cycle={cycle}")
            if cycle < parsed.cycles:
                if parsed.require_px4_stable:
                    node.hover_pub.publish(Bool(data=True))
                    node.get_logger().info("CARTESIAN_SEQUENCE_HOVER_REQUESTED")
                    node.wait_for_flight_stability(
                        parsed.stable_horizontal,
                        parsed.stable_vertical,
                        parsed.stable_hold,
                        parsed.stable_timeout,
                    )
                    record_event("CARTESIAN_SEQUENCE_STABILITY_PASS")
                else:
                    deadline = time.monotonic() + parsed.stable_hold
                    while time.monotonic() < deadline:
                        rclpy.spin_once(node, timeout_sec=0.05)
                        node.publish_motion(False)
        record_event(f"CARTESIAN_SEQUENCE_COMPLETE cycles={parsed.cycles}")
    except RuntimeError as error:
        record_event(f"CARTESIAN_SEQUENCE_ERROR error={error}")
        node.get_logger().error(str(error))
        sys.exit(1)
    finally:
        node.publish_motion(False)
        rclpy.spin_once(node, timeout_sec=0.1)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
