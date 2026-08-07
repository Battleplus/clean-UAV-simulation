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


STATE_RE = re.compile(
    r"STATE arm=(\d+) nav=(\d+) NED=\(([-+0-9.e]+),([-+0-9.e]+),([-+0-9.e]+)\)"
)
YAW_RE = re.compile(r"yaw_deg=([-+0-9.e]+)")


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
    )
    os.close(slave)
    start = time.monotonic()
    output = ""
    states = []
    yaw_values = []
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
    yaw_moved = bool(yaw_values) and max(yaw_values) - min(yaw_values) > 5.0
    if missing or not offboard_states or not climbed or not no_failsafe or not yaw_moved:
        print(
            f"DDS_WASD_PTY_FAIL missing={missing} offboard={bool(offboard_states)} "
            f"climbed={climbed} no_failsafe={no_failsafe} yaw_moved={yaw_moved}",
            file=sys.stderr,
        )
        return 1
    print("DDS_WASD_PTY_PASS")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
