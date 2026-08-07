#!/usr/bin/env python3
"""Continuously command a local-NED PX4 hover setpoint over MAVLink."""

from __future__ import annotations

import argparse
import time

from pymavlink import mavutil


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--connection", default="udpin:0.0.0.0:14550")
    parser.add_argument("--north", type=float, default=0.0)
    parser.add_argument("--east", type=float, default=0.0)
    parser.add_argument("--down", type=float, default=-1.0)
    parser.add_argument("--duration", type=float, default=80.0)
    args = parser.parse_args()

    master = mavutil.mavlink_connection(args.connection)
    heartbeat = master.wait_heartbeat(timeout=20)
    if heartbeat is None:
        raise SystemExit("Timed out waiting for the PX4 MAVLink heartbeat.")

    # Use position only; velocity, acceleration, yaw, and yaw-rate are ignored.
    type_mask = (
        (1 << 3)
        | (1 << 4)
        | (1 << 5)
        | (1 << 6)
        | (1 << 7)
        | (1 << 8)
        | (1 << 10)
        | (1 << 11)
    )
    started = time.monotonic()
    last_mode_request = 0.0
    while time.monotonic() - started < args.duration:
        elapsed = time.monotonic() - started
        master.mav.set_position_target_local_ned_send(
            int(elapsed * 1000.0),
            master.target_system,
            master.target_component,
            mavutil.mavlink.MAV_FRAME_LOCAL_NED,
            type_mask,
            args.north,
            args.east,
            args.down,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
        )
        if elapsed > 1.0 and elapsed - last_mode_request >= 1.0:
            master.mav.command_long_send(
                master.target_system,
                master.target_component,
                mavutil.mavlink.MAV_CMD_DO_SET_MODE,
                0,
                mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
                6,  # PX4_CUSTOM_MAIN_MODE_OFFBOARD
                0,
                0,
                0,
                0,
                0,
            )
            last_mode_request = elapsed
        time.sleep(0.05)


if __name__ == "__main__":
    main()
