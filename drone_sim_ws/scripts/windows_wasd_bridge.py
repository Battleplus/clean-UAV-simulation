"""Windows key-down/key-up bridge for the WSL PX4 WASD controller."""

from __future__ import annotations

import ctypes
from pathlib import Path
import subprocess
import sys
import time


MOTION_KEYS = ("W", "S", "A", "D", "R", "F", "Q", "E")
ONE_SHOT_KEYS = ("T", "H", "L", "O", "Z", "X")
VK = {key: ord(key) for key in MOTION_KEYS + ONE_SHOT_KEYS}


def pressed(user32: object, key: str) -> bool:
    return bool(user32.GetAsyncKeyState(VK[key]) & 0x8000)


def send(child: subprocess.Popen[bytes], key: str) -> None:
    if child.stdin is None:
        raise RuntimeError("controller stdin is unavailable")
    child.stdin.write(key.lower().encode("ascii"))
    child.stdin.flush()


def main() -> int:
    # A closed Windows host console can leave its WSL ros2 child alive. Two
    # Offboard publishers fight over velocity/position setpoints and make
    # both hover and WASD appear unstable. Enforce exactly one controller.
    subprocess.run(
        [
            "wsl.exe", "-d", "Ubuntu-24.04", "--", "bash", "-lc",
            "pkill -TERM -f '[d]ds_wasd_control' 2>/dev/null || true",
        ],
        check=False,
    )
    time.sleep(0.5)
    command = [
        "wsl.exe", "-d", "Ubuntu-24.04", "--cd", "/home/asus/my_drone_ws",
        "bash", "-lc",
        "source /opt/ros/jazzy/setup.bash && "
        "source /home/asus/ros2_px4_build_ws/install/setup.bash && "
        "source install/setup.bash && "
        "export PX4_WASD_HORIZONTAL_SPEED_M_S=0.40 && "
        "export PX4_WASD_VERTICAL_SPEED_M_S=0.15 && "
        "export PX4_KEY_RELEASE_TIMEOUT_S=0.25 && "
        "export PX4_TOUCHDOWN_DISARM_ENABLED=true && "
        "export PX4_TOUCHDOWN_DISARM_HEIGHT_M=0.05 && "
        "export PX4_TOUCHDOWN_DISARM_HOLD_S=0.5 && "
        "exec ros2 run px4_ros2_control dds_wasd_control",
    ]
    workspace = Path(__file__).resolve().parents[1]
    commit = subprocess.run(
        ["git", "-C", str(workspace), "rev-parse", "--short", "HEAD"],
        check=False, capture_output=True, text=True,
    ).stdout.strip() or "unknown"
    print("my_drone true key-state WASD controller")
    print(f"LATEST_WORKSPACE={workspace}")
    print(f"GIT_COMMIT={commit}")
    print("W/S forward/back: 0.40 m/s | A/D left/right: 0.40 m/s")
    print("R up: 0.15 m/s | F down: 0.15 m/s | Q/E yaw: 15 deg/s")
    print("S-curve: horizontal a=0.30 m/s^2 jerk=0.60 m/s^3")
    print("S-curve: vertical a=0.18 m/s^2 jerk=0.40 m/s^3")
    print("Tap/hold a motion key -> latch one velocity target")
    print("Release does nothing | H -> S-curve brake to zero + heading hold")
    print("T takeoff | L land | keep this window focused while flying")
    child = subprocess.Popen(command, stdin=subprocess.PIPE)
    user32 = ctypes.windll.user32
    console_hwnd = ctypes.windll.kernel32.GetConsoleWindow()
    previous = {key: False for key in MOTION_KEYS + ONE_SHOT_KEYS}
    try:
        while child.poll() is None:
            focused = bool(console_hwnd) and user32.GetForegroundWindow() == console_hwnd
            current = {
                key: focused and pressed(user32, key)
                for key in MOTION_KEYS + ONE_SHOT_KEYS
            }
            # Every command is edge-triggered.  A motion key latches one
            # velocity target in the ROS controller; holding it must never
            # generate repeats and releasing it must never command zero.
            for key in MOTION_KEYS + ONE_SHOT_KEYS:
                if current[key] and not previous[key]:
                    send(child, key)
            previous = current
            time.sleep(0.01)
    except KeyboardInterrupt:
        if child.poll() is None:
            send(child, "H")
    finally:
        if child.poll() is None:
            child.terminate()
            try:
                child.wait(timeout=5)
            except subprocess.TimeoutExpired:
                child.kill()
    return int(child.returncode or 0)


if __name__ == "__main__":
    raise SystemExit(main())
