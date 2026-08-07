#!/usr/bin/env python3
"""Interactive body-relative WASD position control for PX4 Offboard."""

from __future__ import annotations

import argparse
import math
import os
import select
import sys
import termios
import time
import tty

if __name__ == "__main__" and os.environ.get("ALLOW_LEGACY_PYMAVLINK", "0") != "1":
    raise SystemExit(
        "Legacy pymavlink WASD is disabled to prevent competing Offboard "
        "publishers. Use scripts/run_ros2_dds_wasd.sh. For historical "
        "diagnostics only, set ALLOW_LEGACY_PYMAVLINK=1 explicitly."
    )

from nav_msgs.msg import Odometry
from pymavlink import mavutil
import rclpy
from rclpy.qos import qos_profile_sensor_data
from rclpy.signals import SignalHandlerOptions


def command_mode_offboard(master) -> None:
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


def command_arm(master) -> None:
    master.mav.command_long_send(
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
        0,
        1,
        0,
        0,
        0,
        0,
        0,
        0,
    )


def command_land(master) -> None:
    master.mav.command_long_send(
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_CMD_NAV_LAND,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
    )


def send_target(master, elapsed, north, east, vertical_velocity, yaw) -> None:
    # Use horizontal position, vertical velocity, and yaw.  Keeping altitude
    # out of the local-z position setpoint avoids following an EKF z reset.
    type_mask = (
        (1 << 2)
        | (1 << 3)
        | (1 << 4)
        | (1 << 6)
        | (1 << 7)
        | (1 << 8)
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
        0.0,
        0.0,
        0.0,
        vertical_velocity,
        0.0,
        0.0,
        0.0,
        yaw,
        0.0,
    )


def drain_state(master, position, yaw):
    while True:
        message = master.recv_match(blocking=False)
        if message is None:
            break
        message_type = message.get_type()
        if message_type == "LOCAL_POSITION_NED":
            position = message
        elif message_type == "ATTITUDE":
            yaw = float(message.yaw)
    return position, yaw


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--connection", default="udpin:0.0.0.0:14550")
    parser.add_argument("--step", type=float, default=0.15)
    parser.add_argument("--vertical-step", type=float, default=0.10)
    parser.add_argument("--yaw-step-deg", type=float, default=8.0)
    parser.add_argument("--takeoff-height", type=float, default=1.0)
    args = parser.parse_args()

    if not sys.stdin.isatty():
        raise SystemExit("WASD control requires an interactive terminal.")

    master = mavutil.mavlink_connection(args.connection)
    if master.wait_heartbeat(timeout=20) is None:
        raise SystemExit("Timed out waiting for PX4 MAVLink heartbeat.")

    for message_id in (
        mavutil.mavlink.MAVLINK_MSG_ID_LOCAL_POSITION_NED,
        mavutil.mavlink.MAVLINK_MSG_ID_ATTITUDE,
    ):
        master.mav.command_long_send(
            master.target_system,
            master.target_component,
            mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,
            0,
            message_id,
            50_000,
            0,
            0,
            0,
            0,
            0,
        )

    position = None
    measured_yaw = 0.0
    world_altitude = None
    world_altitude_updated = 0.0
    rclpy.init(
        args=None,
        signal_handler_options=SignalHandlerOptions.NO,
    )
    ros_node = rclpy.create_node("my_drone_wasd_world_altitude")

    def on_odometry(message: Odometry) -> None:
        nonlocal world_altitude, world_altitude_updated
        world_altitude = float(message.pose.pose.position.z)
        world_altitude_updated = time.monotonic()

    odometry_subscription = ros_node.create_subscription(
        Odometry,
        "/model/my_drone/odometry",
        on_odometry,
        qos_profile_sensor_data,
    )
    deadline = time.monotonic() + 10.0
    while (
        (position is None or world_altitude is None)
        and time.monotonic() < deadline
    ):
        position, measured_yaw = drain_state(
            master, position, measured_yaw
        )
        rclpy.spin_once(ros_node, timeout_sec=0.01)
        time.sleep(0.05)
    if position is None:
        raise SystemExit("No PX4 LOCAL_POSITION_NED data received.")
    if world_altitude is None:
        raise SystemExit("No Gazebo world odometry data received.")

    target_north = float(position.x)
    target_east = float(position.y)
    target_world_altitude = world_altitude + args.takeoff_height
    target_yaw = measured_yaw
    yaw_step = math.radians(args.yaw_step_deg)
    started = time.monotonic()
    last_mode_request = 0.0
    old_terminal = termios.tcgetattr(sys.stdin.fileno())

    print(
        "\nW/S 前后  A/D 左右  R/F 升降  Q/E 左右转向  "
        "空格锁定当前位置  L 降落退出\n"
    )
    try:
        tty.setcbreak(sys.stdin.fileno())
        while True:
            now = time.monotonic()
            elapsed = now - started
            position, measured_yaw = drain_state(
                master, position, measured_yaw
            )
            rclpy.spin_once(ros_node, timeout_sec=0.0)

            if elapsed - last_mode_request >= 1.0:
                command_mode_offboard(master)
                if elapsed < 8.0:
                    command_arm(master)
                last_mode_request = elapsed

            readable, _, _ = select.select([sys.stdin], [], [], 0.0)
            if readable:
                key = os.read(sys.stdin.fileno(), 1).decode(
                    errors="ignore"
                ).lower()
                forward_north = math.cos(measured_yaw)
                forward_east = math.sin(measured_yaw)
                right_north = -math.sin(measured_yaw)
                right_east = math.cos(measured_yaw)
                if key == "w":
                    target_north += args.step * forward_north
                    target_east += args.step * forward_east
                elif key == "s":
                    target_north -= args.step * forward_north
                    target_east -= args.step * forward_east
                elif key == "d":
                    target_north += args.step * right_north
                    target_east += args.step * right_east
                elif key == "a":
                    target_north -= args.step * right_north
                    target_east -= args.step * right_east
                elif key == "r":
                    target_world_altitude += args.vertical_step
                elif key == "f":
                    target_world_altitude -= args.vertical_step
                elif key == "q":
                    target_yaw -= yaw_step
                elif key == "e":
                    target_yaw += yaw_step
                elif key == " ":
                    target_north = float(position.x)
                    target_east = float(position.y)
                    target_world_altitude = world_altitude
                    target_yaw = measured_yaw
                elif key in ("l", "\x1b"):
                    command_land(master)
                    break

            altitude_error = target_world_altitude - world_altitude
            if now - world_altitude_updated > 0.5:
                vertical_velocity = 0.0
            else:
                vertical_velocity = max(
                    -0.5, min(0.5, -0.8 * altitude_error)
                )
                if abs(altitude_error) < 0.03:
                    vertical_velocity = 0.0
            send_target(
                master,
                elapsed,
                target_north,
                target_east,
                vertical_velocity,
                target_yaw,
            )
            print(
                "\r目标 NED "
                f"N={target_north:+.2f} E={target_east:+.2f} "
                f"world_z={world_altitude:.2f}/"
                f"{target_world_altitude:.2f} m "
                f"vz={vertical_velocity:+.2f} "
                f"yaw={math.degrees(target_yaw):+.1f}°"
                "  ",
                end="",
                flush=True,
            )
            time.sleep(0.05)
    except KeyboardInterrupt:
        command_land(master)
    finally:
        termios.tcsetattr(
            sys.stdin.fileno(), termios.TCSADRAIN, old_terminal
        )
        ros_node.destroy_node()
        rclpy.shutdown()
        print("\n已发送降落命令。")


if __name__ == "__main__":
    main()
