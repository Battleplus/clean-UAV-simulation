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

from ros2_test_utils import ros2_child_environment


STATE_RE = re.compile(
    r"STATE arm=(\d+) nav=(\d+) NED=\(([-+0-9.e]+),([-+0-9.e]+),([-+0-9.e]+)\)"
)
FULL_STATE_RE = re.compile(
    r"STATE arm=(\d+) nav=(\d+) NED=\(([-+0-9.e]+),([-+0-9.e]+),([-+0-9.e]+)\) "
    r"vel=\(([-+0-9.e]+),([-+0-9.e]+),([-+0-9.e]+)\) "
    r"truth_enu=\(([-+0-9.e]+),([-+0-9.e]+),([-+0-9.e]+)\) "
    r"truth_vel_enu=\(([-+0-9.e]+),([-+0-9.e]+),([-+0-9.e]+)\)"
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
    parser.add_argument(
        "--settled-hover-seconds", type=float, default=0.0,
        help="when positive, start this hover clock only after height/vertical speed settle",
    )
    args = parser.parse_args()

    master, slave = pty.openpty()
    controller_environment = ros2_child_environment()
    profile = os.environ.get("ARM_FLIGHT_PROFILE", "")
    if profile.endswith("_4kg") or profile == "cartesian_formal_7p735":
        # Keep the test harness configurable.  PX4 local NED can retain a
        # small ground offset after a real Gazebo touchdown; callers may use
        # a larger end-of-test-only threshold without changing flight or
        # hover control.  Never overwrite an explicitly supplied value.
        controller_environment.setdefault("PX4_TOUCHDOWN_DISARM_ENABLED", "true")
        controller_environment.setdefault("PX4_TOUCHDOWN_DISARM_HEIGHT_M", "0.05")
        controller_environment.setdefault("PX4_TOUCHDOWN_DISARM_HOLD_S", "0.5")
    controller = subprocess.Popen(
        ["ros2", "run", "px4_ros2_control", "dds_wasd_control"],
        stdin=slave,
        stdout=slave,
        stderr=slave,
        start_new_session=True,
        close_fds=True,
        env=controller_environment,
    )
    os.close(slave)
    start = time.monotonic()
    output = ""
    states: list[tuple[float, float, float, float, float, float]] = []
    targets: list[tuple[float, float, float, float]] = []
    motor_samples: list[tuple[float, list[float]]] = []
    detailed_states: list[tuple[float, ...]] = []
    initialized = False
    first_state_time = None
    offboard_time = None
    land_sent = False
    land_sent_time = None
    hover_settle_candidate = None
    settled_hover_start = None
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
                for match in FULL_STATE_RE.finditer(data):
                    detailed_states.append(
                        (now,) + tuple(float(v) for v in match.groups())
                    )
                targets.extend(
                    (now,) + tuple(float(v) for v in match.groups())
                    for match in TARGET_RE.finditer(data)
                )
                targets.extend(
                    (now,) + tuple(float(v) for v in match.groups())
                    for match in INIT_TARGET_RE.finditer(data)
                )
                for match in MOTOR_RE.finditer(data):
                    motor_samples.append((
                        now,
                        [float(v.strip()) for v in match.group(1).split(",")],
                    ))
                if "STATE arm=" in output and first_state_time is None:
                    first_state_time = now
                if "OFFBOARD mode and ARM commands sent" in output and offboard_time is None:
                    offboard_time = now

            if first_state_time is not None and not initialized and time.monotonic() - first_state_time >= 2.0:
                os.write(master, b"t")
                initialized = True
            if args.settled_hover_seconds > 0.0 and detailed_states and targets:
                latest = detailed_states[-1]
                prior = [target for target in targets if target[0] <= latest[0]]
                target = prior[-1] if prior else None
                settled_now = (
                    int(latest[2]) == 14
                    and target is not None
                    and abs(latest[5] - target[3]) < 0.15
                    and abs(latest[8]) < 0.08
                )
                if settled_now:
                    if hover_settle_candidate is None:
                        hover_settle_candidate = time.monotonic()
                    elif (
                        settled_hover_start is None
                        and time.monotonic() - hover_settle_candidate >= 3.0
                    ):
                        settled_hover_start = time.monotonic()
                        print("DYNAMIC_HOVER_STEADY_WINDOW_STARTED", flush=True)
                else:
                    hover_settle_candidate = None
            hover_clock_complete = (
                settled_hover_start is not None
                and time.monotonic() - settled_hover_start >= args.settled_hover_seconds
            )
            legacy_clock_complete = (
                args.settled_hover_seconds <= 0.0
                and offboard_time is not None
                and time.monotonic() - offboard_time >= args.hover_seconds
            )
            if not land_sent and (hover_clock_complete or legacy_clock_complete):
                os.write(master, b"l")
                land_sent = True
                land_sent_time = time.monotonic()
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
    # Keep a little sampling margin around the nominal final 10-second
    # evaluation period.  State output is approximately 1 Hz and the exact
    # endpoints are not guaranteed to be observed, so a 10-second inclusive
    # window can contain only nine samples even after a complete run.
    steady_window_duration_s = (
        args.settled_hover_seconds if args.settled_hover_seconds > 0.0 else 12.0
    )
    steady_window_start_s = (
        0.0 if args.settled_hover_seconds > 0.0
        else max(8.0, args.hover_seconds - steady_window_duration_s)
    )
    evaluation_origin = settled_hover_start or offboard_time
    for state in offboard_states:
        prior = [target for target in targets if target[0] <= state[0]]
        if prior:
            target = prior[-1]
            error = np.array([state[3] - target[1], state[4] - target[2], state[5] - target[3]])
            target_errors.append(error)
            if (
                evaluation_origin is not None
                and steady_window_start_s <= state[0] - evaluation_origin <= steady_window_duration_s
            ):
                steady_target_errors.append(error)
    evaluation_errors = steady_target_errors or target_errors
    horizontal = [float(np.linalg.norm(error[:2])) for error in evaluation_errors]
    height = [abs(float(error[2])) for error in evaluation_errors]
    hover_states = [
        state for state in offboard_states
        if evaluation_origin is not None
        and steady_window_start_s <= state[0] - evaluation_origin <= steady_window_duration_s
    ]
    max_step = 0.0
    for previous, current in zip(hover_states, hover_states[1:]):
        max_step = max(max_step, float(np.linalg.norm(np.asarray(current[3:6]) - np.asarray(previous[3:6]))))
    landing_states = [
        state for state in states
        if land_sent_time is not None and state[0] >= land_sent_time
    ]
    landing_horizontal_excursion = 0.0
    landing_max_step = 0.0
    if landing_states:
        landing_origin = np.asarray(landing_states[0][3:5])
        landing_horizontal_excursion = max(
            float(np.linalg.norm(np.asarray(state[3:5]) - landing_origin))
            for state in landing_states
        )
        for previous, current in zip(landing_states, landing_states[1:]):
            landing_max_step = max(
                landing_max_step,
                float(np.linalg.norm(np.asarray(current[3:6]) - np.asarray(previous[3:6]))),
            )
    steady_motors = [
        value
        for stamp, values in motor_samples
        if evaluation_origin is not None
        and steady_window_start_s <= stamp - evaluation_origin <= steady_window_duration_s
        for value in values
    ]
    evaluation_motors = steady_motors or [
        value for _, values in motor_samples for value in values
    ]
    report = {
        "offboard_samples": len(offboard_states),
        "steady_hover_samples": len(steady_target_errors),
        "steady_window_start_s": steady_window_start_s,
        "steady_window_duration_s": steady_window_duration_s,
        "max_horizontal_error_m": max(horizontal, default=None),
        "max_height_error_m": max(height, default=None),
        "takeoff_transient_max_height_error_m": max(
            (abs(float(error[2])) for error in target_errors), default=None
        ),
        "max_state_step_m": max_step,
        "landing_horizontal_excursion_m": landing_horizontal_excursion,
        "landing_max_state_step_m": landing_max_step,
        "max_motor_output": max(evaluation_motors, default=None),
        "mean_motor_output": (
            float(np.mean(evaluation_motors)) if evaluation_motors else None
        ),
        "motor_saturation_fraction": (
            sum(value >= 999.0 for value in evaluation_motors) / len(evaluation_motors)
            if evaluation_motors else None
        ),
        "failsafe_seen": "failsafe=True" in output,
    }
    detailed_steady = [
        state for state in detailed_states
        if evaluation_origin is not None
        and steady_window_start_s <= state[0] - evaluation_origin <= steady_window_duration_s
        and int(state[2]) == 14
    ]
    if detailed_steady:
        px4_z = [state[5] for state in detailed_steady]
        px4_vz = [state[8] for state in detailed_steady]
        truth_z = [state[11] for state in detailed_steady]
        truth_vz = [state[14] for state in detailed_steady]
        report.update({
            "px4_height_peak_to_peak_m": max(px4_z) - min(px4_z),
            "truth_height_peak_to_peak_m": max(truth_z) - min(truth_z),
            "px4_vertical_speed_abs_p90_m_s": float(np.percentile(np.abs(px4_vz), 90)),
            "px4_vertical_speed_abs_max_m_s": max(abs(value) for value in px4_vz),
            "truth_vertical_speed_abs_p90_m_s": float(np.percentile(np.abs(truth_vz), 90)),
            "truth_vertical_speed_abs_max_m_s": max(abs(value) for value in truth_vz),
            "px4_truth_height_correlation": (
                # PX4 local Z is down-positive while Gazebo ENU Z is up-positive.
                float(np.corrcoef(-np.asarray(px4_z), truth_z)[0, 1])
                if len(px4_z) >= 3 and np.std(px4_z) > 1e-9 and np.std(truth_z) > 1e-9
                else None
            ),
        })
    print("DDS_DYNAMIC_HOVER_METRICS " + json.dumps(report, sort_keys=True))
    touchdown_profile = profile.endswith("_4kg") or profile == "cartesian_formal_7p735"
    required = (
        "OFFBOARD mode and ARM commands sent",
        "OFFBOARD_LANDING_STARTED",
        (
            "PX4_TOUCHDOWN_LAND_SENT"
            if touchdown_profile else "PX4_NATIVE_LAND_HANDOFF"
        ),
        "PX4 command ack: command=21 result=0",
        "LANDING_DISARMED_CONFIRMED",
    )
    missing = [item for item in required if item not in output]
    minimum_steady_samples = max(5, min(10, int(steady_window_duration_s)))
    # These are deliberately modest first-pass gates; tuning is a later step.
    passed = (
        not missing
        and len(steady_target_errors) >= minimum_steady_samples
        and bool(horizontal)
        and bool(height)
        and max(horizontal) < 1.0
        and max(height) < 0.8
        and min(height) < 0.60
        and not report["failsafe_seen"]
        and max_step < 0.55
        and landing_horizontal_excursion < 1.0
        and landing_max_step < 0.55
    )
    if args.settled_hover_seconds > 0.0:
        passed = (
            passed
            and settled_hover_start is not None
            and report.get("px4_height_peak_to_peak_m", float("inf")) < 0.15
            and report.get("truth_height_peak_to_peak_m", float("inf")) < 0.15
            and report.get("px4_vertical_speed_abs_p90_m_s", float("inf")) < 0.08
            and report["motor_saturation_fraction"] == 0.0
        )
    if not passed:
        print(f"DDS_DYNAMIC_HOVER_FAIL missing={missing} report={report}", file=sys.stderr)
        return 1
    print("DDS_DYNAMIC_HOVER_PASS")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
