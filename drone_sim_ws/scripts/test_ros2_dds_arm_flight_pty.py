#!/usr/bin/env python3
"""Fly with PX4 DDS while moving the ros2_control SO101 arm."""

import argparse
import errno
import math
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


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--timeout", type=float, default=95.0)
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
    states = []
    initialized = False
    first_state_time = None
    offboard_time = None
    sent = set()
    arm_process = None
    arm_label = None
    arm_results = {}
    profile = os.environ.get("ARM_FLIGHT_PROFILE", "safe")
    if profile == "full_a":
        # Diagnostic only: full work_a at a deliberately slow 30 s trajectory.
        arm_schedule = [(14.0, "work_a"), (74.0, "retracted")]
        arm_duration = "30"
        land_after_s = 120.0
    elif profile == "micro":
        # Flight-safe identification motion: 25% of the reduced flight_work
        # presets, sent over a long trajectory.
        arm_schedule = [
            (14.0, "flight_micro_a"),
            (34.0, "flight_micro_b"),
            (54.0, "retracted"),
        ]
        arm_duration = "14"
        land_after_s = 72.0
    else:
        # The 7.735 kg configuration has only a small thrust margin.  Use the
        # documented slow-work profile so arm acceleration is a measured
        # disturbance rather than an unrealistic step impulse.
        arm_schedule = [
            (14.0, "flight_work_a"),
            (34.0, "flight_work_b"),
            (54.0, "retracted"),
        ]
        arm_duration = "8"
        land_after_s = 72.0
    safety_abort = False
    safety_abort_reason = ""
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
                if "STATE arm=" in output and first_state_time is None:
                    first_state_time = time.monotonic()
                if "OFFBOARD mode and ARM commands sent" in output and offboard_time is None:
                    offboard_time = time.monotonic()

            if (
                first_state_time is not None
                and not initialized
                and time.monotonic() - first_state_time >= 2.0
            ):
                os.write(master, b"t")
                initialized = True

            if arm_process is not None and arm_process.poll() is not None:
                arm_stdout, _ = arm_process.communicate()
                arm_results[arm_label] = (arm_process.returncode, arm_stdout)
                print(arm_stdout, end="", flush=True)
                arm_process = None
                arm_label = None

            if offboard_time is not None:
                elapsed = time.monotonic() - offboard_time
                if not safety_abort and elapsed > 8.0:
                    flight_states = [
                        state for state in states
                        if 0.0 <= state[0] - offboard_time <= elapsed
                    ]
                    if flight_states:
                        north0, east0, down0 = flight_states[0][3:6]
                        latest = flight_states[-1]
                        drift = math.hypot(latest[3] - north0, latest[4] - east0)
                        altitude_error = abs(latest[5] - down0)
                        if drift > 2.0 or altitude_error > 1.8:
                            safety_abort = True
                            safety_abort_reason = (
                                f"drift={drift:.3f}m altitude_delta={altitude_error:.3f}m"
                            )
                            print(
                                f"ARM_FLIGHT_SAFETY_ABORT {safety_abort_reason}",
                                flush=True,
                            )
                            if arm_process is not None and arm_process.poll() is None:
                                arm_process.terminate()
                                arm_process = None
                                arm_label = None
                            if "retracted" not in sent:
                                arm_label = "retracted"
                                arm_process = subprocess.Popen(
                                    [
                                        "ros2", "run", "drone_arm_sim", "arm_preset_control",
                                        "--preset", "retracted", "--duration", "10", "--wait",
                                        "--tolerance", "0.08",
                                    ],
                                    stdout=subprocess.PIPE,
                                    stderr=subprocess.STDOUT,
                                    text=True,
                                )
                                sent.add("retracted")
                            os.write(master, b"l")
                            sent.add("LAND")
                            print("ARM_FLIGHT_SAFETY_LAND", flush=True)
                for at, preset in arm_schedule:
                    if not safety_abort and elapsed >= at and preset not in sent and arm_process is None:
                        arm_label = preset
                        arm_process = subprocess.Popen(
                            [
                                "ros2", "run", "drone_arm_sim", "arm_preset_control",
                                "--preset", preset, "--duration", arm_duration, "--wait",
                                "--tolerance", "0.06",
                            ],
                            stdout=subprocess.PIPE,
                            stderr=subprocess.STDOUT,
                            text=True,
                        )
                        sent.add(preset)
                        print(f"ARM_FLIGHT_SENT_{preset}", flush=True)
                        break
                if not safety_abort and elapsed >= land_after_s and "LAND" not in sent:
                    os.write(master, b"l")
                    sent.add("LAND")
                    print("ARM_FLIGHT_SENT_LAND", flush=True)
            if controller.poll() is not None:
                break
        if controller.poll() is None:
            raise RuntimeError("flight controller did not finish before timeout")
    finally:
        if arm_process is not None and arm_process.poll() is None:
            arm_process.terminate()
        if controller.poll() is None:
            os.killpg(controller.pid, signal.SIGTERM)
            try:
                controller.wait(timeout=5)
            except subprocess.TimeoutExpired:
                os.killpg(controller.pid, signal.SIGKILL)
        os.close(master)

    missing = []
    for item in (
        "OFFBOARD mode and ARM commands sent",
        "PX4 command ack: command=21 result=0",
        "LANDING_DISARMED_CONFIRMED",
    ):
        if item not in output:
            missing.append(item)
    required_presets = (
        ("work_a", "retracted") if profile == "full_a"
        else (
            ("flight_micro_a", "flight_micro_b", "retracted")
            if profile == "micro"
            else ("flight_work_a", "flight_work_b", "retracted")
        )
    )
    for preset in required_presets:
        result = arm_results.get(preset)
        if result is None or result[0] != 0 or "ARM_PRESET_REACHED" not in result[1]:
            missing.append(f"arm:{preset}")

    armed_states = [state for state in states if int(state[1]) == 2]
    climbed = any(state[5] < -0.8 for state in armed_states)
    no_failsafe = "failsafe=True" not in output
    arm_window = []
    if offboard_time is not None:
        arm_window = [
            state for state in states
            if 0.0 <= state[0] - offboard_time <= 70.0
        ]
    max_horizontal_drift = float("inf")
    altitude_span = float("inf")
    if arm_window:
        north0, east0 = arm_window[0][3], arm_window[0][4]
        max_horizontal_drift = max(
            math.hypot(state[3] - north0, state[4] - east0) for state in arm_window
        )
        down_values = [state[5] for state in arm_window]
        altitude_span = max(down_values) - min(down_values)
    stable = max_horizontal_drift < 1.5 and altitude_span < 1.5
    print(
        "ARM_FLIGHT_METRICS "
        f"horizontal_drift_m={max_horizontal_drift:.3f} "
        f"altitude_span_m={altitude_span:.3f} samples={len(arm_window)}"
    )
    if safety_abort:
        print(f"ARM_FLIGHT_ABORTED reason={safety_abort_reason}")
    if missing or not climbed or not no_failsafe or not stable:
        print(
            f"DDS_ARM_FLIGHT_FAIL missing={missing} climbed={climbed} "
            f"no_failsafe={no_failsafe} stable={stable}",
            file=sys.stderr,
        )
        return 1
    print("DDS_ARM_FLIGHT_PASS")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
