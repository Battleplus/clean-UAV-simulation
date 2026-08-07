#!/usr/bin/env python3
"""End-to-end PTY regression for the interactive PX4 WASD launcher."""

from __future__ import annotations

import math
import os
import pty
import select
import signal
import subprocess
import sys
import time

import yaml


WORKSPACE = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))


def drain(fd: int, duration: float) -> bytes:
    deadline = time.monotonic() + duration
    output = bytearray()
    while time.monotonic() < deadline:
        readable, _, _ = select.select([fd], [], [], 0.1)
        if readable:
            try:
                output.extend(os.read(fd, 8192))
            except OSError:
                break
    return bytes(output)


def wait_for(fd: int, marker: bytes, timeout: float) -> bytes:
    deadline = time.monotonic() + timeout
    output = bytearray()
    while time.monotonic() < deadline:
        readable, _, _ = select.select([fd], [], [], 0.2)
        if readable:
            try:
                chunk = os.read(fd, 8192)
            except OSError as error:
                raise RuntimeError(
                    "WASD launcher exited before the keyboard prompt:\n"
                    + output.decode(errors="replace")
                ) from error
            output.extend(chunk)
            if marker in output:
                return bytes(output)
    raise TimeoutError(f"Timed out waiting for terminal marker {marker!r}")


def sample_odometry() -> tuple[float, float, float, float]:
    result = subprocess.run(
        [
            "ros2",
            "topic",
            "echo",
            "/model/my_drone/odometry",
            "--once",
        ],
        check=True,
        capture_output=True,
        text=True,
        timeout=15,
    )
    yaml_start = result.stdout.find("header:")
    if yaml_start < 0:
        raise RuntimeError(
            "Could not find odometry YAML in ros2 output:\n"
            + result.stdout
        )
    data = yaml.safe_load(
        result.stdout[yaml_start:].split("---", 1)[0]
    )
    pose = data["pose"]["pose"]
    position = pose["position"]
    quaternion = pose["orientation"]
    x = float(quaternion["x"])
    y = float(quaternion["y"])
    z = float(quaternion["z"])
    w = float(quaternion["w"])
    yaw = math.atan2(
        2.0 * (w * z + x * y),
        1.0 - 2.0 * (y * y + z * z),
    )
    return (
        float(position["x"]),
        float(position["y"]),
        float(position["z"]),
        yaw,
    )


def dot(delta, direction) -> float:
    return delta[0] * direction[0] + delta[1] * direction[1]


def main() -> None:
    pid, fd = pty.fork()
    if pid == 0:
        os.chdir(WORKSPACE)
        os.execvp("bash", ["bash", "scripts/wsl_px4_wasd.sh"])

    transcript = bytearray()
    samples = []
    try:
        transcript.extend(wait_for(fd, b"W/S", 300))
        transcript.extend(drain(fd, 15))
        samples.append(sample_odometry())

        for keys in (b"wwww", b"dddd", b"ssss", b"aaaa"):
            os.write(fd, keys)
            transcript.extend(drain(fd, 12))
            samples.append(sample_odometry())

        os.write(fd, b"l")
        transcript.extend(drain(fd, 12))
        deadline = time.monotonic() + 30
        while time.monotonic() < deadline:
            waited, status = os.waitpid(pid, os.WNOHANG)
            if waited == pid:
                if os.waitstatus_to_exitcode(status) != 0:
                    raise SystemExit("WASD launcher exited with an error.")
                break
            transcript.extend(drain(fd, 0.2))
        else:
            raise TimeoutError("WASD launcher did not exit after landing.")
    finally:
        try:
            os.kill(pid, signal.SIGINT)
        except ProcessLookupError:
            pass
        with open("/tmp/my_drone_wasd_pty_transcript.log", "wb") as stream:
            stream.write(transcript)

    initial_yaw = samples[0][3]
    forward = (math.cos(initial_yaw), math.sin(initial_yaw))
    right = (math.sin(initial_yaw), -math.cos(initial_yaw))
    expected = (forward, right, (-forward[0], -forward[1]), (-right[0], -right[1]))
    labels = ("W-forward", "D-right", "S-back", "A-left")
    results = []
    for index, (label, direction) in enumerate(zip(labels, expected)):
        before = samples[index]
        after = samples[index + 1]
        delta = (after[0] - before[0], after[1] - before[1])
        progress = dot(delta, direction)
        altitude_change = abs(after[2] - samples[0][2])
        results.append((label, delta, progress, altitude_change))
        print(
            f"{label}: delta_enu=({delta[0]:+.3f}, {delta[1]:+.3f}), "
            f"body_progress={progress:.3f} m, "
            f"altitude_change={altitude_change:.3f} m"
        )

    if any(progress < 0.25 for _, _, progress, _ in results):
        raise SystemExit("WASD_PTY_TEST_FAIL: directional movement too small")
    if any(altitude_change > 0.35 for _, _, _, altitude_change in results):
        raise SystemExit("WASD_PTY_TEST_FAIL: altitude was not maintained")
    print("WASD_PTY_TEST_PASS")


if __name__ == "__main__":
    main()
