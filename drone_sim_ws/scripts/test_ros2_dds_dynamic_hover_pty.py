#!/usr/bin/env python3
"""Takeoff/hover/land regression for the enabled motor-battery dynamics."""

from __future__ import annotations

import argparse
import errno
import json
import os
import pty
import re
import select
import signal
import subprocess
import sys
import time

import numpy as np


STATE_RE = re.compile(
    r"STATE arm=(\d+) nav=(\d+) NED=\(([-+0-9.e]+),([-+0-9.e]+),([-+0-9.e]+)\)"
)
TARGET_RE = re.compile(
    r"target NED=\(([-+0-9.e]+),\s*([-+0-9.e]+),\s*([-+0-9.e]+)\)"
)
INIT_TARGET_RE = re.compile(
    r"NED target initialized: N=([-+0-9.e]+) E=([-+0-9.e]+) D=([-+0-9.e]+)"
)
MOTOR_RE = re.compile(r"motors=\[([^\]]+)\]")


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--timeout", type=float, default=50.0)
    parser.add_argument("--hover-seconds", type=float, default=18.0)
    args = parser.parse_args()

    master, slave = pty.openpty()
    controller = subprocess.Popen(
        ["ros2", "run", "px4_ros2_control", "dds_wasd_control"],
        stdin=slave,
        stdout=slave,
        stderr=slave,
        start_new_session=True,
        close_fds=True,
    )
    os.close(slave)
    start = time.monotonic()
    output = ""
    states: list[tuple[float, float, float, float, float, float]] = []
    targets: list[tuple[float, float, float, float]] = []
    motors: list[float] = []
    initialized = False
    first_state_time = None
    offboard_time = None
    land_sent = False
    try:
        while time.monotonic() - start < args.timeout:
            if select.select([master], [], [], 0.1)[0]:
                try:
                    data = os.read(master, 65536).decode(errors="replace")
                except OSError as exc:
                    if exc.errno == errno.EIO:
                        break
                    raise
                sys.stdout.write(data)
                sys.stdout.flush()
                output += data
                now = time.monotonic()
                for match in STATE_RE.finditer(data):
                    states.append((now,) + tuple(float(v) for v in match.groups()))
                targets.extend(
                    (now,) + tuple(float(v) for v in match.groups())
                    for match in TARGET_RE.finditer(data)
                )
                targets.extend(
                    (now,) + tuple(float(v) for v in match.groups())
                    for match in INIT_TARGET_RE.finditer(data)
                )
                for match in MOTOR_RE.finditer(data):
                    motors.extend(float(v.strip()) for v in match.group(1).split(","))
                if "STATE arm=" in output and first_state_time is None:
                    first_state_time = now
                if "OFFBOARD mode and ARM commands sent" in output and offboard_time is None:
                    offboard_time = now

            if first_state_time is not None and not initialized and time.monotonic() - first_state_time >= 2.0:
                os.write(master, b"t")
                initialized = True
            if offboard_time is not None and not land_sent and time.monotonic() - offboard_time >= args.hover_seconds:
                os.write(master, b"l")
                land_sent = True
                print("DYNAMIC_HOVER_SENT_LAND", flush=True)
            if controller.poll() is not None:
                break
        if controller.poll() is None:
            raise RuntimeError("dynamic hover controller did not finish before timeout")
    finally:
        if controller.poll() is None:
            os.killpg(controller.pid, signal.SIGTERM)
            try:
                controller.wait(timeout=5)
            except subprocess.TimeoutExpired:
                os.killpg(controller.pid, signal.SIGKILL)
        os.close(master)

    offboard_states = [state for state in states if int(state[2]) == 14]
    target_errors = []
    steady_target_errors = []
    for state in offboard_states:
        prior = [target for target in targets if target[0] <= state[0]]
        if prior:
            target = prior[-1]
            error = np.array([state[3] - target[1], state[4] - target[2], state[5] - target[3]])
            target_errors.append(error)
            if offboard_time is not None and 8.0 <= state[0] - offboard_time <= args.hover_seconds:
                steady_target_errors.append(error)
    evaluation_errors = steady_target_errors or target_errors
    horizontal = [float(np.linalg.norm(error[:2])) for error in evaluation_errors]
    height = [abs(float(error[2])) for error in evaluation_errors]
    hover_states = (
        [state for state in offboard_states if offboard_time is None or state[0] - offboard_time <= args.hover_seconds]
    )
    max_step = 0.0
    for previous, current in zip(hover_states, hover_states[1:]):
        max_step = max(max_step, float(np.linalg.norm(np.asarray(current[3:6]) - np.asarray(previous[3:6]))))
    report = {
        "offboard_samples": len(offboard_states),
        "steady_hover_samples": len(steady_target_errors),
        "max_horizontal_error_m": max(horizontal, default=None),
        "max_height_error_m": max(height, default=None),
        "takeoff_transient_max_height_error_m": max(
            (abs(float(error[2])) for error in target_errors), default=None
        ),
        "max_state_step_m": max_step,
        "max_motor_output": max(motors, default=None),
        "motor_saturation_fraction": (sum(value >= 999.0 for value in motors) / len(motors) if motors else None),
        "failsafe_seen": "failsafe=True" in output,
    }
    print("DDS_DYNAMIC_HOVER_METRICS " + json.dumps(report, sort_keys=True))
    required = ("OFFBOARD mode and ARM commands sent", "PX4 command ack: command=21 result=0", "LANDING_DISARMED_CONFIRMED")
    missing = [item for item in required if item not in output]
    # These are deliberately modest first-pass gates; tuning is a later step.
    passed = not missing and bool(offboard_states) and not report["failsafe_seen"] and max_step < 0.55
    if not passed:
        print(f"DDS_DYNAMIC_HOVER_FAIL missing={missing} report={report}", file=sys.stderr)
        return 1
    print("DDS_DYNAMIC_HOVER_PASS")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
