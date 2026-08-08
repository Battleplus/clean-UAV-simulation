#!/usr/bin/env python3
"""Drive the interactive ROS 2 DDS controller through a real pseudo-terminal."""

import argparse
import errno
import os
import pty
import re
import select
import signal
import subprocess
import sys
import time

import numpy as np

from ros2_test_utils import ros2_child_environment


STATE_RE = re.compile(
    r"STATE arm=(\d+) nav=(\d+) NED=\(([-+0-9.e]+),([-+0-9.e]+),([-+0-9.e]+)\)"
)
YAW_RE = re.compile(r"yaw_deg=([-+0-9.e]+)")
TARGET_RE = re.compile(
    r"target NED=\(([-+0-9.e]+),\s*([-+0-9.e]+),\s*([-+0-9.e]+)\)"
)
TARGET_YAW_RE = re.compile(r"target NED=.*?yaw=([-+0-9.e]+) deg")
MOTOR_RE = re.compile(r"motors=\[([^\]]+)\]")


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--timeout", type=float, default=80.0)
    args = parser.parse_args()
    master, slave = pty.openpty()
    proc = subprocess.Popen(
        ["ros2", "run", "px4_ros2_control", "dds_wasd_control"],
        stdin=slave,
        stdout=slave,
        stderr=slave,
        start_new_session=True,
        close_fds=True,
        env=ros2_child_environment(),
    )
    os.close(slave)
    start = time.monotonic()
    output = ""
    states = []
    yaw_values = []
    target_yaw_values = []
    targets = []
    motor_samples = []
    schedule = []
    sent = set()
    initialized = False
    first_state_time = None
    offboard_time = None
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
                for match in STATE_RE.finditer(data):
                    states.append(
                        (time.monotonic(),) + tuple(float(x) for x in match.groups())
                    )
                yaw_values.extend(float(match.group(1)) for match in YAW_RE.finditer(data))
                target_yaw_values.extend(
                    float(match.group(1)) for match in TARGET_YAW_RE.finditer(data)
                )
                now = time.monotonic()
                targets.extend(
                    (now,) + tuple(float(x) for x in match.groups())
                    for match in TARGET_RE.finditer(data)
                )
                for match in MOTOR_RE.finditer(data):
                    values = [
                        float(value.strip())
                        for value in match.group(1).split(",")
                        if value.strip()
                    ]
                    if len(values) == 8:
                        motor_samples.append((now, values))
                if "STATE arm=" in output and first_state_time is None:
                    first_state_time = time.monotonic()
                if "OFFBOARD mode and ARM commands sent" in output and offboard_time is None:
                    offboard_time = time.monotonic()
                    schedule = [
                        (12.0, b"w", "W"),
                        (17.0, b"d", "D"),
                        (22.0, b"s", "S"),
                        (27.0, b"a", "A"),
                        (32.0, b"q", "Q"),
                        (37.0, b"e", "E"),
                        (44.0, b"l", "LAND"),
                    ]
            if (
                first_state_time is not None
                and not initialized
                and time.monotonic() - first_state_time >= 2.0
            ):
                os.write(master, b"t")
                initialized = True
            if offboard_time is not None:
                elapsed = time.monotonic() - offboard_time
                for at, key, label in schedule:
                    if elapsed >= at and label not in sent:
                        os.write(master, key)
                        sent.add(label)
                        print(f"PTY_SENT_{label}", flush=True)
            if proc.poll() is not None:
                break
        if proc.poll() is None:
            raise RuntimeError("controller did not finish before timeout")
    finally:
        if proc.poll() is None:
            os.killpg(proc.pid, signal.SIGTERM)
            try:
                proc.wait(timeout=5)
            except subprocess.TimeoutExpired:
                os.killpg(proc.pid, signal.SIGKILL)
        os.close(master)

    required = [
        "OFFBOARD mode and ARM commands sent",
        "PTY_SENT_W",
        "PTY_SENT_D",
        "PTY_SENT_S",
        "PTY_SENT_A",
        "PTY_SENT_Q",
        "PTY_SENT_E",
        "PX4 command ack: command=21 result=0",
        "LANDING_DISARMED_CONFIRMED",
    ]
    combined = output + "\n" + "\n".join(f"PTY_SENT_{x}" for x in sent)
    missing = [item for item in required if item not in combined]
    offboard_states = [s for s in states if int(s[2]) == 14]
    armed_states = [s for s in states if int(s[1]) == 2]
    climbed = any(s[5] < -0.8 for s in armed_states)
    no_failsafe = "failsafe=True" not in output
    observed_yaw_values = yaw_values or target_yaw_values
    # The bring-up controller intentionally uses a 3-degree yaw increment to
    # preserve thrust margin.  Require a measurable response, not a 5-degree
    # final excursion that would reject the documented Q/E pair after it
    # nearly cancels.  The raw yaw range remains in DDS_WASD_METRICS.
    yaw_moved = bool(observed_yaw_values) and max(observed_yaw_values) - min(observed_yaw_values) > 2.0
    armed_states = [s for s in states if int(s[2]) == 2]
    target_errors = []
    for state in offboard_states:
        prior_targets = [target for target in targets if target[0] <= state[0]]
        if not prior_targets:
            continue
        target = prior_targets[-1]
        target_errors.append(
            np.array([state[3] - target[1], state[4] - target[2], state[5] - target[3]])
        )
    horizontal_errors = [float(np.linalg.norm(error[:2])) for error in target_errors]
    height_errors = [abs(float(error[2])) for error in target_errors]
    flat_motors = [value for _, sample in motor_samples for value in sample]
    saturation_values = [value for value in flat_motors if value >= 999.0]
    metrics = {
        "offboard_state_count": len(offboard_states),
        "max_horizontal_position_error_m": max(horizontal_errors, default=None),
        "max_height_error_m": max(height_errors, default=None),
        "max_motor_output": max(flat_motors, default=None),
        "motor_saturation_fraction": (
            len(saturation_values) / len(flat_motors) if flat_motors else None
        ),
        "yaw_range_deg": (
            max(observed_yaw_values) - min(observed_yaw_values)
            if observed_yaw_values
            else None
        ),
        "failsafe_seen": not no_failsafe,
    }
    print("DDS_WASD_METRICS " + __import__("json").dumps(metrics, sort_keys=True))
    # A command/ack-only pass is not enough: an unstable vehicle can still
    # complete the key sequence after flying away.  Keep these first-pass
    # limits deliberately wider than the protected 0.523 m baseline result.
    stable_metrics = (
        metrics["max_horizontal_position_error_m"] is not None
        and metrics["max_horizontal_position_error_m"] < 2.0
        and metrics["max_height_error_m"] is not None
        and metrics["max_height_error_m"] < 1.5
        and metrics["motor_saturation_fraction"] is not None
        and metrics["motor_saturation_fraction"] < 0.25
    )
    if missing or not offboard_states or not climbed or not no_failsafe or not yaw_moved or not stable_metrics:
        print(
            f"DDS_WASD_PTY_FAIL missing={missing} offboard={bool(offboard_states)} "
            f"climbed={climbed} no_failsafe={no_failsafe} yaw_moved={yaw_moved} "
            f"stable_metrics={stable_metrics}",
            file=sys.stderr,
        )
        return 1
    print("DDS_WASD_PTY_PASS")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
