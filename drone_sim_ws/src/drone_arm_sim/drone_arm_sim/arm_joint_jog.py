"""Move one SO101 joint by a bounded, smooth keyboard-sized increment."""

from __future__ import annotations

import argparse
import sys
import time

import rclpy

from .arm_preset_control import ArmPresetCommander, JOINT_NAMES, load_motion_reference


def bounded_jog_target(
    current: list[float],
    joint_name: str,
    delta_rad: float,
    reference: dict,
) -> list[float]:
    """Return a jog target, rejecting rather than clipping an unsafe command."""
    if joint_name not in JOINT_NAMES:
        raise ValueError(f"unknown joint: {joint_name}")
    if len(current) != len(JOINT_NAMES):
        raise ValueError("current position vector must contain all SO101 joints")
    limits = {item["name"]: item for item in reference.get("joints", [])}
    limit = limits.get(joint_name)
    if limit is None:
        raise ValueError(f"no safety limit is defined for joint {joint_name}")
    target = list(map(float, current))
    index = JOINT_NAMES.index(joint_name)
    requested = target[index] + float(delta_rad)
    lower = float(limit["lower_rad"])
    upper = float(limit["upper_rad"])
    if not lower <= requested <= upper:
        raise ValueError(
            f"jog rejected: {joint_name} target {requested:.4f} rad is outside "
            f"[{lower:.4f}, {upper:.4f}]"
        )
    target[index] = requested
    return target


def main(args=None) -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--joint", choices=JOINT_NAMES, required=True)
    parser.add_argument("--delta", type=float, required=True)
    parser.add_argument("--duration", type=float, default=2.5)
    parser.add_argument("--tolerance", type=float, default=0.01)
    parsed = parser.parse_args(args)
    if parsed.duration <= 0.0:
        parser.error("--duration must be positive")
    if parsed.delta == 0.0:
        parser.error("--delta must be non-zero")

    reference = load_motion_reference()
    limits = {item["name"]: item for item in reference.get("joints", [])}
    velocity_limit = float(limits[parsed.joint]["velocity_rad_s"])
    # Quintic smoothstep peak speed is 1.875 times its average speed.  Keep
    # keyboard jogging below 20% of the documented joint limit in flight.
    minimum_duration = 1.875 * abs(parsed.delta) / max(1e-6, 0.2 * velocity_limit)
    if parsed.duration < minimum_duration:
        parser.error(
            f"jog needs duration >= {minimum_duration:.3f}s for the 0.2x velocity limit"
        )

    rclpy.init()
    node = ArmPresetCommander()
    try:
        connection_deadline = time.monotonic() + 10.0
        while node.publisher.get_subscription_count() == 0:
            if time.monotonic() >= connection_deadline:
                raise RuntimeError("arm_controller trajectory subscriber not found")
            rclpy.spin_once(node, timeout_sec=0.1)

        state_deadline = time.monotonic() + 5.0
        while any(name not in node.latest_positions for name in JOINT_NAMES):
            if time.monotonic() >= state_deadline:
                raise RuntimeError("complete /joint_states sample unavailable")
            rclpy.spin_once(node, timeout_sec=0.1)

        current = [float(node.latest_positions[name]) for name in JOINT_NAMES]
        target = bounded_jog_target(current, parsed.joint, parsed.delta, reference)
        node.publish_motion_active(True)
        node.send(target, parsed.duration)
        node.get_logger().info(
            f"ARM_JOG_SENT joint={parsed.joint} delta={parsed.delta:.4f}rad "
            f"duration={parsed.duration:.2f}s"
        )

        deadline = time.monotonic() + parsed.duration + 5.0
        last_heartbeat = 0.0
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.05)
            if time.monotonic() - last_heartbeat >= 0.25:
                node.publish_motion_active(True)
                last_heartbeat = time.monotonic()
            error = node.maximum_error(target)
            if error is not None and error <= parsed.tolerance:
                node.get_logger().info(
                    f"ARM_JOG_REACHED joint={parsed.joint} "
                    f"max_error={error:.6f}rad"
                )
                return
        raise RuntimeError(
            f"jog target was not reached; max_error={node.maximum_error(target)}"
        )
    except (RuntimeError, ValueError) as error:
        node.get_logger().error(str(error))
        sys.exit(1)
    finally:
        node.publish_motion_active(False)
        rclpy.spin_once(node, timeout_sec=0.1)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
