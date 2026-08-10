#!/usr/bin/env python3
"""End-to-end PTY check for heartbeat-based velocity WASD control."""

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

from ros2_test_utils import ros2_child_environment


STATE_RE = re.compile(
    r"STATE arm=(\d+) nav=(\d+) NED=\(([-+0-9.e]+),([-+0-9.e]+),([-+0-9.e]+)\) "
    r"vel=\(([-+0-9.e]+),([-+0-9.e]+),([-+0-9.e]+)\).*?"
    r"control=([A-Z_]+).*?motors=\[([^]]+)\]"
)


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--vertical-only", action="store_true")
    parser.add_argument("--timeout", type=float, default=140.0)
    args = parser.parse_args()
    master, slave = pty.openpty()
    child_environment = ros2_child_environment()
    if args.vertical_only:
        child_environment.update({
            "PX4_TOUCHDOWN_DISARM_ENABLED": "true",
            "PX4_TOUCHDOWN_DISARM_HEIGHT_M": "0.05",
            "PX4_TOUCHDOWN_DISARM_HOLD_S": "0.5",
            "ARM_FEEDFORWARD_ENABLED": "false",
        })
    proc = subprocess.Popen(
        ["ros2", "run", "px4_ros2_control", "dds_wasd_control"],
        stdin=slave,
        stdout=slave,
        stderr=slave,
        start_new_session=True,
        close_fds=True,
        env=child_environment,
    )
    os.close(slave)
    started = time.monotonic()
    output = ""
    states: list[tuple[float, tuple[str, ...]]] = []
    first_state_at = None
    offboard_at = None
    latest_state = None
    settled_since = None
    hover_settled = False
    action_started_at = None
    takeoff_sent = False
    phase = 0
    phase_started = None
    # Each motion key is held by a 10 Hz heartbeat.  The H phase deliberately
    # interrupts W before its heartbeat timeout to prove immediate braking.
    phases = (
        [
            ("r", 1.5, 3.0),
            ("f", 1.5, 3.0),
            ("r", 1.0, 0.0),
            ("h", 0.1, 4.0),
        ]
        if args.vertical_only else [
            ("w", 2.0, 2.0), ("s", 2.0, 2.0),
            ("a", 2.0, 2.0), ("d", 2.0, 2.0),
            ("r", 1.5, 2.0), ("f", 1.5, 2.0),
            ("q", 1.5, 2.0), ("e", 1.5, 2.0),
            ("w", 1.0, 0.0), ("h", 0.1, 3.0),
        ]
    )
    last_heartbeat = 0.0
    land_sent = False
    try:
        while time.monotonic() - started < args.timeout:
            now = time.monotonic()
            if select.select([master], [], [], 0.05)[0]:
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
                    latest_state = match.groups()
                    states.append((now, latest_state))
                if first_state_at is None and "STATE arm=" in output:
                    first_state_at = now
                if offboard_at is None and "OFFBOARD mode and ARM commands sent" in output:
                    offboard_at = now
            if first_state_at and not takeoff_sent and now - first_state_at >= 2.0:
                os.write(master, b"t")
                takeoff_sent = True
            if offboard_at and phase_started is None:
                if args.vertical_only:
                    # Do not mix the takeoff transient into the R/F/H check.
                    # The 4 kg profile commands about 1.2 m above its local
                    # origin, so require both altitude capture and low vz for
                    # three continuous seconds before applying any key.
                    settled = (
                        latest_state is not None
                        and float(latest_state[4]) <= -1.0
                        and abs(float(latest_state[7])) < 0.08
                    )
                    if settled:
                        settled_since = settled_since or now
                    else:
                        settled_since = None
                    if settled_since is not None and now - settled_since >= 3.0:
                        print("VELOCITY_TEST_HOVER_SETTLED", flush=True)
                        hover_settled = True
                        phase_started = now
                        action_started_at = now
                elif now - offboard_at >= 9.0:
                    phase_started = now
                    action_started_at = now
            if phase_started is not None and phase < len(phases):
                key, hold_s, rest_s = phases[phase]
                elapsed = now - phase_started
                if elapsed < hold_s:
                    if now - last_heartbeat >= 0.10:
                        os.write(master, key.encode())
                        last_heartbeat = now
                elif elapsed >= hold_s + rest_s:
                    print(f"VELOCITY_PHASE_DONE key={key.upper()}", flush=True)
                    phase += 1
                    phase_started = now
                    last_heartbeat = 0.0
            elif phase == len(phases) and not land_sent:
                os.write(master, b"l")
                land_sent = True
            if proc.poll() is not None:
                break
    finally:
        if proc.poll() is None:
            os.killpg(proc.pid, signal.SIGTERM)
            try:
                proc.wait(timeout=5)
            except subprocess.TimeoutExpired:
                os.killpg(proc.pid, signal.SIGKILL)
        os.close(master)

    measured_states = [
        (timestamp, state) for timestamp, state in states
        if action_started_at is None or timestamp >= action_started_at
    ]
    position_states = [s for _, s in measured_states if s[8] == "POSITION_HOLD"]
    velocity_states = [s for _, s in measured_states if s[8] == "VELOCITY_CONTROL"]
    horizontal_speed = [
        (float(s[5]) ** 2 + float(s[6]) ** 2) ** 0.5 for _, s in measured_states
    ]
    vertical_speed = [abs(float(s[7])) for _, s in measured_states]
    motor_values = [
        float(value.strip())
        for _, state in measured_states
        for value in state[9].split(",")
    ]
    required = [
        "OFFBOARD mode and ARM commands sent",
        ("VELOCITY_CONTROL_ENTER key=R" if args.vertical_only else "VELOCITY_CONTROL_ENTER key=W"),
        "VELOCITY_RELEASE_HOLD",
        "HOVER_HOLD_CURRENT",
        "PX4 command ack: command=21 result=0",
        "LANDING_DISARMED_CONFIRMED",
    ]
    if args.vertical_only:
        required.append("VELOCITY_CONTROL_ENTER key=F")
    metrics = {
        "state_samples": len(states),
        "position_hold_samples": len(position_states),
        "velocity_control_samples": len(velocity_states),
        "max_horizontal_speed_m_s": max(horizontal_speed, default=None),
        "max_vertical_speed_m_s": max(vertical_speed, default=None),
        "velocity_enters": output.count("VELOCITY_CONTROL_ENTER"),
        "release_holds": output.count("VELOCITY_RELEASE_HOLD"),
        "failsafe_seen": "failsafe=True" in output,
        "position_safety_land_seen": "Position safety gate exceeded" in output,
        "max_motor_output": max(motor_values, default=None),
        "motor_saturation_fraction": (
            sum(value >= 999.0 for value in motor_values) / len(motor_values)
            if motor_values else None
        ),
        "vertical_only": args.vertical_only,
    }
    print("DDS_VELOCITY_WASD_METRICS " + json.dumps(metrics, sort_keys=True))
    missing = [value for value in required if value not in output]
    passed = (
        not missing
        and (hover_settled if args.vertical_only else True)
        # STATE is emitted at 1 Hz while key heartbeats run at 10 Hz. Four
        # seconds of vertical key time can legitimately yield only four
        # sampled VELOCITY_CONTROL states depending on phase alignment.
        and len(velocity_states) >= (4 if args.vertical_only else 6)
        and len(position_states) >= 6
        and metrics["velocity_enters"] >= (3 if args.vertical_only else 9)
        and metrics["release_holds"] >= (2 if args.vertical_only else 8)
        and not metrics["failsafe_seen"]
        and not metrics["position_safety_land_seen"]
        and metrics["motor_saturation_fraction"] == 0.0
    )
    if not passed:
        print(f"DDS_VELOCITY_WASD_FAIL missing={missing}", file=sys.stderr)
        return 1
    print("DDS_VELOCITY_WASD_PASS")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
