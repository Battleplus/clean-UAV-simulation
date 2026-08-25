"""Persistent terminal publisher for SO101 Cartesian velocity commands."""

from __future__ import annotations

import argparse
from contextlib import contextmanager
import sys
import time
from typing import Callable, Iterator, TextIO


COMMAND_TOPIC = "/my_drone/arm_cartesian_velocity_command"
KEY_COMMANDS = {
    "i": "forward",
    "k": "back",
    "j": "left",
    "l": "right",
    "u": "up",
    "o": "down",
    " ": "stop",
    "x": "disable",
}


def command_for_key(key: str) -> str | None:
    """Translate one terminal key into a body-FLU velocity command."""
    return KEY_COMMANDS.get(str(key).lower())


def run_key_loop(
    read_key: Callable[[], str],
    publish_command: Callable[[str], None],
    keep_running: Callable[[], bool],
) -> None:
    """Publish mapped keys until X disables the velocity controller."""
    while keep_running():
        key = read_key()
        if not key:
            return
        command = command_for_key(key)
        if command is None:
            continue
        publish_command(command)
        print(f"CARTESIAN_TARGET_VELOCITY {command}", flush=True)
        if command == "disable":
            return


@contextmanager
def cbreak_terminal(stream: TextIO) -> Iterator[None]:
    """Read single keys while guaranteeing restoration of terminal settings."""
    import termios
    import tty

    if not stream.isatty():
        raise RuntimeError("Cartesian velocity keyboard requires an interactive TTY")
    descriptor = stream.fileno()
    settings = termios.tcgetattr(descriptor)
    try:
        tty.setcbreak(descriptor)
        yield
    finally:
        termios.tcsetattr(descriptor, termios.TCSADRAIN, settings)


def wait_for_subscriber(node, publisher, rclpy_api, timeout_s: float) -> None:
    """Wait for the persistent velocity server before accepting commands."""
    deadline = time.monotonic() + timeout_s
    print(f"Waiting for a subscriber on {COMMAND_TOPIC}...", flush=True)
    while rclpy_api.ok() and publisher.get_subscription_count() == 0:
        if time.monotonic() >= deadline:
            raise RuntimeError(
                "Cartesian velocity server did not subscribe to the command topic"
            )
        rclpy_api.spin_once(node, timeout_sec=0.1)


def main(args=None) -> None:
    import rclpy
    from std_msgs.msg import String

    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--subscriber-timeout",
        type=float,
        default=15.0,
        help="seconds to wait for the Cartesian velocity server",
    )
    parsed, ros_arguments = parser.parse_known_args(args)
    if parsed.subscriber_timeout <= 0.0:
        parser.error("--subscriber-timeout must be positive")

    rclpy.init(args=ros_arguments)
    node = rclpy.create_node("cartesian_arm_velocity_keyboard")
    publisher = node.create_publisher(String, COMMAND_TOPIC, 10)
    disabled = False

    def publish(command: str) -> None:
        nonlocal disabled
        publisher.publish(String(data=command))
        disabled = command == "disable"
        # Let DDS service discovery and delivery progress without replacing
        # this long-lived node or publisher between keystrokes.
        rclpy.spin_once(node, timeout_sec=0.02)

    try:
        wait_for_subscriber(node, publisher, rclpy, parsed.subscriber_timeout)
        print("CARTESIAN VELOCITY KEYBOARD CONNECTED", flush=True)
        print("I/K forward/back | J/L left/right | U/O up/down", flush=True)
        print("SPACE stop | X disable and exit", flush=True)
        with cbreak_terminal(sys.stdin):
            run_key_loop(lambda: sys.stdin.read(1), publish, rclpy.ok)
    except RuntimeError as error:
        node.get_logger().error(str(error))
        raise SystemExit(1) from error
    finally:
        if publisher.get_subscription_count() > 0 and not disabled:
            publisher.publish(String(data="disable"))
            rclpy.spin_once(node, timeout_sec=0.1)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
