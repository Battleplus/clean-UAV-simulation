#!/usr/bin/env python3
"""Exercise all six Cartesian velocity directions through one ROS publisher."""

from __future__ import annotations

import json
import time

import numpy as np
import rclpy
from rclpy.node import Node
from std_msgs.msg import String


COMMAND_TOPIC = "/my_drone/arm_cartesian_velocity_command"
STATE_TOPIC = "/my_drone/arm_cartesian_velocity_state"
DIRECTIONS = {
    "forward": np.array([1.0, 0.0, 0.0]),
    "back": np.array([-1.0, 0.0, 0.0]),
    "left": np.array([0.0, 1.0, 0.0]),
    "right": np.array([0.0, -1.0, 0.0]),
    "up": np.array([0.0, 0.0, 1.0]),
    "down": np.array([0.0, 0.0, -1.0]),
}


class GroundSmoke(Node):
    def __init__(self) -> None:
        super().__init__("cartesian_velocity_ground_smoke")
        self.publisher = self.create_publisher(String, COMMAND_TOPIC, 10)
        self.events: list[dict] = []
        self.create_subscription(String, STATE_TOPIC, self._state_cb, 20)

    def _state_cb(self, message: String) -> None:
        try:
            event = json.loads(message.data)
        except json.JSONDecodeError:
            return
        self.events.append(event)
        print("CARTESIAN_LIVE_STATE " + json.dumps(event, sort_keys=True))

    def publish(self, command: str) -> None:
        self.publisher.publish(String(data=command))
        print(f"CARTESIAN_LIVE_COMMAND {command}", flush=True)

    def spin_for(self, duration_s: float) -> None:
        deadline = time.monotonic() + duration_s
        while time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)

    def wait_for_event(self, event_name: str, start_index: int, timeout_s: float) -> bool:
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)
            if any(
                event.get("event") == event_name
                for event in self.events[start_index:]
            ):
                return True
        return False


def main() -> int:
    rclpy.init()
    node = GroundSmoke()
    failures: list[str] = []
    try:
        deadline = time.monotonic() + 12.0
        while node.publisher.get_subscription_count() == 0:
            if time.monotonic() >= deadline:
                print("CARTESIAN_GROUND_FAIL no command subscriber")
                return 1
            rclpy.spin_once(node, timeout_sec=0.1)

        for command, expected_direction in DIRECTIONS.items():
            start_index = len(node.events)
            node.publish(command)
            # The folded/retracted pose is intentionally close to the body
            # collision boundary.  This smoke test validates command routing,
            # direction and measured stopping with one tiny safe segment; the
            # full-distance workspace test starts from an interior pose.
            node.spin_for(0.25)
            node.publish("stop")
            if not node.wait_for_event("stopped", start_index, 12.0):
                failures.append(f"{command}: did not reach measured stop")
                break
            events = node.events[start_index:]
            if any(event.get("event") == "segment_rejected" for event in events):
                failures.append(f"{command}: segment rejected")
                break
            segments = [
                event for event in events if event.get("event") == "segment_sent"
            ]
            if not segments:
                failures.append(f"{command}: no segment sent")
                break
            selected = np.asarray(
                segments[-1]["selected_endpoint_delta_body_flu_m"], dtype=float
            )
            projection = float(np.dot(selected, expected_direction))
            if projection <= 0.0:
                failures.append(
                    f"{command}: endpoint moved against command ({projection:.6g} m)"
                )
                break
            print(
                f"CARTESIAN_DIRECTION_PASS command={command} "
                f"projection_m={projection:.6f}",
                flush=True,
            )
        node.publish("disable")
        node.spin_for(0.5)
        if failures:
            print("CARTESIAN_GROUND_FAIL " + "; ".join(failures))
            return 1
        print("CARTESIAN_GROUND_SIX_DIRECTION_PASS")
        return 0
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
