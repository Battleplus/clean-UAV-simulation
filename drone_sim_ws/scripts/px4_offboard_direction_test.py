#!/usr/bin/env python3
"""Fly a forward-right-back-left local-NED box and verify each corner."""

from __future__ import annotations

import json
import math
import time

from pymavlink import mavutil


WAYPOINTS = (
    ("center_climb", 0.0, 0.0, -1.0, 22.0),
    ("forward_north", 1.0, 0.0, -1.0, 22.0),
    ("right_east", 1.0, 1.0, -1.0, 22.0),
    ("back_south", 0.0, 1.0, -1.0, 22.0),
    ("left_west", 0.0, 0.0, -1.0, 22.0),
)


def send_position(master, elapsed: float, north: float, east: float, down: float):
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
    master.mav.set_position_target_local_ned_send(
        int(elapsed * 1000.0),
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_FRAME_LOCAL_NED,
        type_mask,
        north,
        east,
        down,
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
    )


def request_offboard(master) -> None:
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


def main() -> None:
    master = mavutil.mavlink_connection("udpin:0.0.0.0:14550")
    heartbeat = master.wait_heartbeat(timeout=20)
    if heartbeat is None:
        raise SystemExit("Timed out waiting for the PX4 MAVLink heartbeat.")

    # Request LOCAL_POSITION_NED at 20 Hz.
    master.mav.command_long_send(
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,
        0,
        mavutil.mavlink.MAVLINK_MSG_ID_LOCAL_POSITION_NED,
        50_000,
        0,
        0,
        0,
        0,
        0,
    )

    overall_start = time.monotonic()
    last_mode_request = 0.0
    latest_position = None
    results = []

    for name, north, east, down, duration in WAYPOINTS:
        phase_start = time.monotonic()
        while time.monotonic() - phase_start < duration:
            now = time.monotonic()
            elapsed = now - overall_start
            send_position(master, elapsed, north, east, down)
            if elapsed > 1.0 and elapsed - last_mode_request >= 1.0:
                request_offboard(master)
                last_mode_request = elapsed

            while True:
                message = master.recv_match(
                    type="LOCAL_POSITION_NED", blocking=False
                )
                if message is None:
                    break
                latest_position = message
            time.sleep(0.05)

        if latest_position is None:
            raise SystemExit(f"No LOCAL_POSITION_NED received for {name}.")
        error = math.sqrt(
            (latest_position.x - north) ** 2
            + (latest_position.y - east) ** 2
            + (latest_position.z - down) ** 2
        )
        speed = math.sqrt(
            latest_position.vx**2
            + latest_position.vy**2
            + latest_position.vz**2
        )
        result = {
            "waypoint": name,
            "target_ned_m": [north, east, down],
            "actual_ned_m": [
                round(latest_position.x, 4),
                round(latest_position.y, 4),
                round(latest_position.z, 4),
            ],
            "error_m": round(error, 4),
            "speed_m_s": round(speed, 4),
        }
        print(json.dumps(result, ensure_ascii=False), flush=True)
        results.append((error, speed))

    if any(error > 0.4 or speed > 0.25 for error, speed in results):
        raise SystemExit("DIRECTION_TEST_FAIL")
    print("DIRECTION_TEST_PASS", flush=True)


if __name__ == "__main__":
    main()
