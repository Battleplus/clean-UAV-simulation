#!/usr/bin/env python3
"""End-to-end PTY check for latched, jerk-limited velocity WASD control."""

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
VELOCITY_SP_RE = re.compile(
    r"velocity_key=([^ ]+) velocity_sp=\(([-+0-9.e]+),([-+0-9.e]+),([-+0-9.e]+)\)"
)
YAW_RATE_RE = re.compile(
    r"body_rate_deg_s=\(([-+0-9.e]+),([-+0-9.e]+),([-+0-9.e]+)\) "
    r"yaw_rate_sp_deg_s=([-+0-9.e]+)"
)
TRUTH_ATTITUDE_RE = re.compile(
    r"truth_rpy_deg=\(([-+0-9.e]+),([-+0-9.e]+),([-+0-9.e]+)\) "
    r"truth_body_rate_deg_s=\(([-+0-9.e]+),([-+0-9.e]+),([-+0-9.e]+)\)"
)
TRUTH_VELOCITY_RE = re.compile(
    r"truth_vel_enu=\(([-+0-9.e]+),([-+0-9.e]+),([-+0-9.e]+)\)"
)
PX4_ATTITUDE_RE = re.compile(
    r"(?<!truth_)rpy_deg=\(([-+0-9.e]+),([-+0-9.e]+),([-+0-9.e]+)\)"
)
TARGET_RE = re.compile(
    r"target NED=\(([-+0-9.e]+), ([-+0-9.e]+), ([-+0-9.e]+)\)"
)


def _phase_response_metrics(samples: list[dict], target: float) -> dict:
    """Return command-response metrics for one latched vertical phase."""
    if not samples:
        return {"samples": 0, "target_m_s": target}
    actual = [sample["actual_vz"] for sample in samples]
    command = [sample["command_vz"] for sample in samples]
    elapsed = [sample["elapsed_s"] for sample in samples]
    direction = 1.0 if target >= 0.0 else -1.0
    target_magnitude = abs(target)
    directed = [value * direction for value in actual]
    peak = max(directed)
    overshoot = (
        max(0.0, peak - target_magnitude) / target_magnitude
        if target_magnitude > 1.0e-6 else max(abs(value) for value in actual)
    )
    rise_time = next(
        (t for t, value in zip(elapsed, directed) if value >= 0.9 * target_magnitude),
        None,
    ) if target_magnitude > 1.0e-6 else None
    tolerance = max(0.015, 0.1 * target_magnitude)
    settling_time = None
    for index, value in enumerate(actual):
        if all(abs(later - target) <= tolerance for later in actual[index:]):
            settling_time = elapsed[index]
            break
    tail_count = min(2, len(actual))
    steady_actual = sum(actual[-tail_count:]) / tail_count
    collective = [sample.get("motor_collective") for sample in samples]
    collective = [value for value in collective if value is not None]
    truth_actual = [
        sample["truth_vz_ned"] for sample in samples
        if sample.get("truth_vz_ned") is not None
    ]
    truth_directed = [value * direction for value in truth_actual]
    return {
        "samples": len(samples),
        "target_m_s": target,
        "command_peak_m_s": max(abs(value) for value in command),
        "actual_peak_directed_m_s": peak,
        "rise_time_s": rise_time,
        "overshoot_fraction": overshoot,
        "settling_time_s": settling_time,
        "steady_actual_m_s": steady_actual,
        "steady_error_m_s": target - steady_actual,
        "min_motor_collective_output": min(collective, default=None),
        "max_motor_collective_output": max(collective, default=None),
        "truth_actual_peak_directed_m_s": max(truth_directed, default=None),
        "truth_max_abs_vertical_speed_m_s": max(
            (abs(value) for value in truth_actual), default=None
        ),
    }


def _scalar_phase_response_metrics(
    samples: list[dict],
    target: float,
    actual_key: str,
    command_key: str,
    unit_suffix: str,
    tolerance_floor: float,
) -> dict:
    """Return identical response metrics for any signed scalar control phase."""
    target_name = f"target_{unit_suffix}"
    if not samples:
        return {"samples": 0, target_name: target}
    actual = [float(sample[actual_key]) for sample in samples]
    command = [float(sample[command_key]) for sample in samples]
    elapsed = [float(sample["elapsed_s"]) for sample in samples]
    direction = 1.0 if target >= 0.0 else -1.0
    target_magnitude = abs(target)
    directed = [value * direction for value in actual]
    peak = max(directed)
    overshoot = (
        max(0.0, peak - target_magnitude) / target_magnitude
        if target_magnitude > 1.0e-6 else max(abs(value) for value in actual)
    )
    rise_time = next(
        (t for t, value in zip(elapsed, directed) if value >= 0.9 * target_magnitude),
        None,
    ) if target_magnitude > 1.0e-6 else None
    tolerance = max(tolerance_floor, 0.1 * target_magnitude)
    settling_time = None
    for index, value in enumerate(actual):
        if all(abs(later - target) <= tolerance for later in actual[index:]):
            settling_time = elapsed[index]
            break
    tail_count = min(2, len(actual))
    steady_actual = sum(actual[-tail_count:]) / tail_count
    return {
        "samples": len(samples),
        target_name: target,
        f"command_peak_abs_{unit_suffix}": max(abs(value) for value in command),
        f"actual_peak_directed_{unit_suffix}": peak,
        "rise_time_s": rise_time,
        "overshoot_fraction": overshoot,
        "settling_time_s": settling_time,
        f"steady_actual_{unit_suffix}": steady_actual,
        f"steady_error_{unit_suffix}": target - steady_actual,
    }


def _horizontal_phase_response_metrics(samples: list[dict], target: float) -> dict:
    """Project horizontal response onto the final commanded NED direction."""
    if not samples:
        return {"samples": 0, "target_m_s": target}
    direction = None
    if abs(target) > 1.0e-6:
        # Use the latest non-zero command direction.  During W->S or A->D
        # reversal, early samples still point along the previous command and
        # must not be mistaken for progress toward the new target.
        for sample in reversed(samples):
            vx = float(sample["command_vx"])
            vy = float(sample["command_vy"])
            norm = (vx * vx + vy * vy) ** 0.5
            if norm > 1.0e-3:
                direction = (vx / norm, vy / norm)
                break
    projected = []
    for sample in samples:
        actual_vx = float(sample["actual_vx"])
        actual_vy = float(sample["actual_vy"])
        command_vx = float(sample["command_vx"])
        command_vy = float(sample["command_vy"])
        if direction is None:
            actual = (actual_vx * actual_vx + actual_vy * actual_vy) ** 0.5
            command = (command_vx * command_vx + command_vy * command_vy) ** 0.5
        else:
            actual = actual_vx * direction[0] + actual_vy * direction[1]
            command = command_vx * direction[0] + command_vy * direction[1]
        projected.append({
            "elapsed_s": sample["elapsed_s"],
            "actual": actual,
            "command": command,
        })
    metrics = _scalar_phase_response_metrics(
        projected, target, "actual", "command", "m_s", 0.04
    )
    metrics["projection_direction_ned"] = list(direction) if direction else None
    return metrics


def _wrapped_degrees(value: float) -> float:
    return (float(value) + 180.0) % 360.0 - 180.0


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--vertical-only", action="store_true")
    parser.add_argument("--yaw-only", action="store_true")
    parser.add_argument("--yaw-reversal", action="store_true")
    parser.add_argument("--horizontal-vertical-transition", action="store_true")
    parser.add_argument("--vertical-yaw-transition", action="store_true")
    parser.add_argument("--timeout", type=float, default=140.0)
    args = parser.parse_args()
    if sum((args.vertical_only, args.yaw_only, args.yaw_reversal,
            args.horizontal_vertical_transition,
            args.vertical_yaw_transition)) > 1:
        parser.error("diagnostic profile flags are mutually exclusive")
    master, slave = pty.openpty()
    child_environment = ros2_child_environment()
    child_environment.update({
        "PX4_TOUCHDOWN_DISARM_ENABLED": "true",
        "PX4_TOUCHDOWN_DISARM_HEIGHT_M": "0.05",
        "PX4_TOUCHDOWN_DISARM_HOLD_S": "0.5",
        "ARM_FEEDFORWARD_ENABLED": "false",
    })
    if args.vertical_only or args.vertical_yaw_transition:
        child_environment.update({
            "PX4_WASD_VERTICAL_SPEED_M_S": "0.15",
        })
    # One-Hz state reports are too sparse for rise/settling/steady-state
    # identification and can make a three-second H phase contain only three
    # samples.  All diagnostic profiles use 10 Hz; this changes logging only,
    # not the 20-Hz control update.
    child_environment["PX4_STATE_REPORT_HZ"] = "10"
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
    pending_state_text = ""
    states: list[tuple[float, tuple[str, ...]]] = []
    phase_samples: dict[int, list[dict]] = {}
    attitude_samples: list[dict] = []
    first_state_at = None
    pre_takeoff_settled_since = None
    offboard_at = None
    latest_state = None
    takeoff_target_z = None
    settled_since = None
    hover_settled = False
    action_started_at = None
    takeoff_sent = False
    phase = 0
    phase_started = None
    # Each motion key is sent once.  Demand remains latched throughout the
    # phase and changes only on the next key or H.
    if args.vertical_only:
        phases = [
            ("r", 1.5, 3.0), ("f", 1.5, 3.0),
            ("r", 1.0, 0.0), ("h", 0.1, 4.0),
        ]
    elif args.vertical_yaw_transition:
        phases = [
            ("r", 1.5, 2.0), ("f", 1.5, 2.0),
            ("q", 3.0, 1.0), ("h", 0.1, 4.0),
        ]
    elif args.yaw_only:
        phases = [
            ("q", 3.0, 1.0), ("h", 0.1, 3.0),
            ("e", 3.0, 1.0), ("h", 0.1, 3.0),
        ]
    elif args.yaw_reversal:
        phases = [("q", 2.0, 1.0), ("e", 2.0, 1.0), ("h", 0.1, 3.0)]
    elif args.horizontal_vertical_transition:
        phases = [("d", 3.0, 1.0), ("r", 3.0, 1.0), ("h", 0.1, 3.0)]
    else:
        phases = [
            ("w", 2.0, 2.0), ("s", 2.0, 2.0),
            ("a", 2.0, 2.0), ("d", 2.0, 2.0),
            ("r", 1.5, 2.0), ("f", 1.5, 2.0),
            ("q", 1.5, 2.0), ("e", 1.5, 2.0),
            ("w", 1.0, 0.0), ("h", 0.1, 3.0),
        ]
    phase_command_sent = False
    land_sent = False
    aborted_by_safety = False
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
                pending_state_text += data
                state_lines = pending_state_text.splitlines(keepends=True)
                if state_lines and not state_lines[-1].endswith(("\n", "\r")):
                    pending_state_text = state_lines.pop()
                else:
                    pending_state_text = ""
                for state_line in state_lines:
                    match = STATE_RE.search(state_line)
                    if match is None:
                        continue
                    latest_state = match.groups()
                    states.append((now, latest_state))
                    velocity_match = VELOCITY_SP_RE.search(state_line)
                    yaw_rate_match = YAW_RATE_RE.search(state_line)
                    truth_velocity_match = TRUTH_VELOCITY_RE.search(state_line)
                    truth_attitude_match = TRUTH_ATTITUDE_RE.search(state_line)
                    px4_attitude_match = PX4_ATTITUDE_RE.search(state_line)
                    if truth_attitude_match and px4_attitude_match:
                        attitude_samples.append({
                            "timestamp": now,
                            "control": latest_state[8],
                            "px4_rpy_deg": [
                                float(px4_attitude_match.group(index))
                                for index in range(1, 4)
                            ],
                            "truth_rpy_deg": [
                                float(truth_attitude_match.group(index))
                                for index in range(1, 4)
                            ],
                            "truth_body_rate_deg_s": [
                                float(truth_attitude_match.group(index))
                                for index in range(4, 7)
                            ],
                        })
                    if phase_started is not None and phase < len(phases):
                        motor_collective = sum(
                            float(value.strip()) for value in latest_state[9].split(",")
                        )
                        motor_outputs = [
                            float(value.strip()) for value in latest_state[9].split(",")
                        ]
                        phase_samples.setdefault(phase, []).append({
                            "elapsed_s": now - phase_started,
                            "actual_vx": float(latest_state[5]),
                            "actual_vy": float(latest_state[6]),
                            "actual_vz": float(latest_state[7]),
                            "command_vx": (
                                float(velocity_match.group(2)) if velocity_match else 0.0
                            ),
                            "command_vy": (
                                float(velocity_match.group(3)) if velocity_match else 0.0
                            ),
                            "command_vz": (
                                float(velocity_match.group(4)) if velocity_match else 0.0
                            ),
                            "actual_yaw_rate_deg_s": (
                                float(yaw_rate_match.group(3)) if yaw_rate_match else 0.0
                            ),
                            "command_yaw_rate_deg_s": (
                                float(yaw_rate_match.group(4)) if yaw_rate_match else 0.0
                            ),
                            "truth_vz_ned": (
                                -float(truth_velocity_match.group(3))
                                if truth_velocity_match else None
                            ),
                            "motor_collective": motor_collective,
                            "motor_saturated": any(
                                value <= 1.0 or value >= 999.0 for value in motor_outputs
                            ),
                            "truth_tilt_deg": (
                                max(
                                    abs(float(truth_attitude_match.group(1))),
                                    abs(float(truth_attitude_match.group(2))),
                                )
                                if truth_attitude_match else None
                            ),
                        })
                if first_state_at is None and "STATE arm=" in output:
                    first_state_at = now
                if offboard_at is None and "OFFBOARD mode and ARM commands sent" in output:
                    offboard_at = now
                target_match = TARGET_RE.search(data)
                if target_match:
                    takeoff_target_z = float(target_match.group(3))
                if (
                    not aborted_by_safety
                    and not land_sent
                    and phase < len(phases)
                    and ("Position safety gate exceeded" in data or "control=LANDING" in data)
                ):
                    aborted_by_safety = True
                    print("VELOCITY_TEST_ABORT_SAFETY_LANDING", flush=True)
            if first_state_at and not takeoff_sent and latest_state is not None:
                pre_takeoff_stable = (
                    (float(latest_state[5]) ** 2 + float(latest_state[6]) ** 2) ** 0.5
                    < 0.10
                    and abs(float(latest_state[7])) < 0.08
                )
                if pre_takeoff_stable:
                    pre_takeoff_settled_since = pre_takeoff_settled_since or now
                else:
                    pre_takeoff_settled_since = None
                if (
                    pre_takeoff_settled_since is not None
                    and now - pre_takeoff_settled_since >= 3.0
                ):
                    print("VELOCITY_TEST_PRE_TAKEOFF_STABLE", flush=True)
                    os.write(master, b"t")
                    takeoff_sent = True
            if offboard_at and phase_started is None:
                # Do not mix the takeoff transient into any velocity check.
                # The takeoff height is relative to the estimator value at
                # command time.  EKF local D does not necessarily start at
                # zero, so compare with the logged target rather than a
                # hard-coded absolute height or an elapsed-time delay.
                settled = (
                    latest_state is not None
                    and takeoff_target_z is not None
                    and abs(float(latest_state[4]) - takeoff_target_z) <= 0.15
                    and abs(float(latest_state[7])) < 0.08
                    and (float(latest_state[5]) ** 2 + float(latest_state[6]) ** 2) ** 0.5 < 0.10
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
            if aborted_by_safety:
                if not land_sent:
                    os.write(master, b"l")
                    land_sent = True
            elif phase_started is not None and phase < len(phases):
                key, hold_s, rest_s = phases[phase]
                elapsed = now - phase_started
                if not phase_command_sent:
                    os.write(master, key.encode())
                    phase_command_sent = True
                elif elapsed >= hold_s + rest_s:
                    print(f"VELOCITY_PHASE_DONE key={key.upper()}", flush=True)
                    phase += 1
                    phase_started = now
                    phase_command_sent = False
            elif phase == len(phases) and not land_sent:
                os.write(master, b"l")
                land_sent = True
            if proc.poll() is not None:
                break
        # A diagnostic timeout must never abandon an armed Offboard vehicle.
        # Request the normal staged landing and keep the controller alive long
        # enough to acknowledge/disarm before terminating the PTY child.
        if takeoff_sent and not land_sent and proc.poll() is None:
            os.write(master, b"l")
            land_sent = True
            print("VELOCITY_TEST_TIMEOUT_LAND_REQUESTED", flush=True)
            cleanup_deadline = time.monotonic() + 20.0
            while time.monotonic() < cleanup_deadline and proc.poll() is None:
                if not select.select([master], [], [], 0.1)[0]:
                    continue
                try:
                    data = os.read(master, 65536).decode(errors="replace")
                except OSError as exc:
                    if exc.errno == errno.EIO:
                        break
                    raise
                sys.stdout.write(data)
                sys.stdout.flush()
                output += data
                if "LANDING_DISARMED_CONFIRMED" in data:
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
    yaw_rate_samples = []
    for line in output.splitlines():
        if "control=VELOCITY_CONTROL" not in line:
            continue
        match = YAW_RATE_RE.search(line)
        if match:
            yaw_rate_samples.append({
                "actual_deg_s": float(match.group(3)),
                "command_deg_s": float(match.group(4)),
            })
    command_states = [s for _, s in measured_states if s[8] == "VELOCITY_CONTROL"]
    horizontal_speed = [
        (float(s[5]) ** 2 + float(s[6]) ** 2) ** 0.5 for s in command_states
    ]
    vertical_speed = [abs(float(s[7])) for s in command_states]
    motor_values = [
        float(value.strip())
        for state in command_states
        for value in state[9].split(",")
    ]
    motor_collective = [
        sum(float(value.strip()) for value in state[9].split(","))
        for state in command_states
    ]
    phase_truth_vertical = [
        sample["truth_vz_ned"]
        for samples in phase_samples.values()
        for sample in samples
        if sample.get("truth_vz_ned") is not None
    ]
    phase_estimator_truth_vz_error = [
        sample["actual_vz"] - sample["truth_vz_ned"]
        for samples in phase_samples.values()
        for sample in samples
        if sample.get("truth_vz_ned") is not None
    ]
    measured_attitude_samples = [
        sample for sample in attitude_samples
        if action_started_at is None or sample["timestamp"] >= action_started_at
    ]
    command_attitude_samples = [
        sample for sample in measured_attitude_samples
        if sample["control"] == "VELOCITY_CONTROL"
    ]
    truth_tilt_deg = [
        max(abs(sample["truth_rpy_deg"][0]), abs(sample["truth_rpy_deg"][1]))
        for sample in command_attitude_samples
    ]
    # Gazebo reports ENU/FLU while PX4 reports NED/FRD.  Their absolute yaw
    # zero and yaw/pitch signs therefore differ.  Compare changes from the
    # first command sample after applying the FLU->FRD axis signs; comparing
    # the raw Euler numbers creates a false ~100 degree attitude error.
    px4_truth_attitude_error_deg = []
    if command_attitude_samples:
        reference = command_attitude_samples[0]
        for sample in command_attitude_samples:
            px4_delta = [
                _wrapped_degrees(sample["px4_rpy_deg"][axis] - reference["px4_rpy_deg"][axis])
                for axis in range(3)
            ]
            truth_delta = [
                _wrapped_degrees(sample["truth_rpy_deg"][axis] - reference["truth_rpy_deg"][axis])
                for axis in range(3)
            ]
            truth_delta_frd = [truth_delta[0], -truth_delta[1], -truth_delta[2]]
            px4_truth_attitude_error_deg.append(max(
                abs(_wrapped_degrees(px4_delta[axis] - truth_delta_frd[axis]))
                for axis in range(3)
            ))
    required = [
        "OFFBOARD mode and ARM commands sent",
        (
            "VELOCITY_DEMAND_LATCH key=R"
            if (args.vertical_only or args.vertical_yaw_transition)
            else (
                "VELOCITY_DEMAND_LATCH key=Q"
                if (args.yaw_only or args.yaw_reversal)
                else (
                    "VELOCITY_DEMAND_LATCH key=D"
                    if args.horizontal_vertical_transition
                    else "VELOCITY_DEMAND_LATCH key=W"
                )
            )
        ),
        "HOVER_ZERO_VELOCITY_DEMAND",
        "PX4 command ack: command=21 result=0",
        "LANDING_DISARMED_CONFIRMED",
    ]
    if args.vertical_only:
        required.append("VELOCITY_DEMAND_LATCH key=F")
    metrics = {
        "state_samples": len(states),
        "position_hold_samples": len(position_states),
        "velocity_control_samples": len(velocity_states),
        "max_horizontal_speed_m_s": max(horizontal_speed, default=None),
        "max_vertical_speed_m_s": max(vertical_speed, default=None),
        "max_truth_vertical_speed_m_s": max(
            (abs(value) for value in phase_truth_vertical), default=None
        ),
        "max_estimator_truth_vz_error_m_s": max(
            (abs(value) for value in phase_estimator_truth_vz_error), default=None
        ),
        "max_zero_vertical_disturbance_m_s": (
            max(vertical_speed, default=None)
            if (args.yaw_only or args.yaw_reversal) else None
        ),
        "velocity_enters": output.count("VELOCITY_CONTROL_ENTER"),
        "velocity_demand_latches": output.count("VELOCITY_DEMAND_LATCH"),
        "failsafe_seen": "failsafe=True" in output,
        "position_safety_land_seen": "Position safety gate exceeded" in output,
        "aborted_by_safety": aborted_by_safety,
        "max_motor_output": max(motor_values, default=None),
        "min_motor_collective_output": min(motor_collective, default=None),
        "max_motor_collective_output": max(motor_collective, default=None),
        "motor_saturation_fraction": (
            sum(value <= 1.0 or value >= 999.0 for value in motor_values) / len(motor_values)
            if motor_values else None
        ),
        "truth_attitude_samples": len(command_attitude_samples),
        "max_truth_roll_pitch_deg": max(truth_tilt_deg, default=None),
        "max_frame_aligned_px4_truth_attitude_error_deg": max(
            px4_truth_attitude_error_deg, default=None
        ),
        "vertical_only": args.vertical_only,
        "yaw_only": args.yaw_only,
        "yaw_reversal": args.yaw_reversal,
        "horizontal_vertical_transition": args.horizontal_vertical_transition,
        "vertical_yaw_transition": args.vertical_yaw_transition,
    }
    if args.vertical_only:
        phase_vertical_targets = (-0.15, 0.15, -0.15, 0.0)
    elif args.horizontal_vertical_transition:
        phase_vertical_targets = (0.0, -0.15, 0.0)
    elif args.vertical_yaw_transition:
        phase_vertical_targets = (-0.15, 0.15, 0.0, 0.0)
    elif args.yaw_only:
        phase_vertical_targets = (0.0,) * 4
    elif args.yaw_reversal:
        phase_vertical_targets = (0.0,) * 3
    else:
        # Full W/S/A/D/R/F/Q/E/W/H sequence.  Keeping a per-phase vertical
        # response exposes cross-axis coupling instead of reducing the whole
        # flight to one maximum that cannot identify the triggering command.
        phase_vertical_targets = (
            0.0, 0.0, 0.0, 0.0, -0.15,
            0.15, 0.0, 0.0, 0.0, 0.0,
        )
    metrics["phase_vertical_response"] = {
        f"{phases[index][0].upper()}_{index}": _phase_response_metrics(
            phase_samples.get(index, []), phase_vertical_targets[index]
        )
        for index in range(len(phases))
    }
    phase_horizontal_targets = tuple(
        0.40 if phase_key in "wasd" else 0.0 for phase_key, _, _ in phases
    )
    metrics["phase_horizontal_response"] = {
        f"{phases[index][0].upper()}_{index}": _horizontal_phase_response_metrics(
            phase_samples.get(index, []), phase_horizontal_targets[index]
        )
        for index in range(len(phases))
    }
    phase_yaw_targets = tuple(
        -15.0 if phase_key == "q" else 15.0 if phase_key == "e" else 0.0
        for phase_key, _, _ in phases
    )
    metrics["phase_yaw_response"] = {
        f"{phases[index][0].upper()}_{index}": _scalar_phase_response_metrics(
            phase_samples.get(index, []),
            phase_yaw_targets[index],
            "actual_yaw_rate_deg_s",
            "command_yaw_rate_deg_s",
            "deg_s",
            1.5,
        )
        for index in range(len(phases))
    }
    metrics["phase_quality"] = {
        f"{phases[index][0].upper()}_{index}": {
            "motor_saturation_fraction": (
                sum(bool(sample.get("motor_saturated")) for sample in phase_samples.get(index, []))
                / len(phase_samples.get(index, []))
                if phase_samples.get(index) else None
            ),
            "max_truth_roll_pitch_deg": max(
                (
                    float(sample["truth_tilt_deg"])
                    for sample in phase_samples.get(index, [])
                    if sample.get("truth_tilt_deg") is not None
                ),
                default=None,
            ),
            "tail_1s_max_truth_roll_pitch_deg": max(
                (
                    float(sample["truth_tilt_deg"])
                    for sample in phase_samples.get(index, [])
                    if sample.get("truth_tilt_deg") is not None
                    and sample["elapsed_s"] >= max(
                        0.0, phases[index][1] + phases[index][2] - 1.0
                    )
                ),
                default=None,
            ),
        }
        for index in range(len(phases))
    }
    metrics["horizontal_speed_overshoot_fraction"] = (
        max(0.0, metrics["max_horizontal_speed_m_s"] - 0.40) / 0.40
        if metrics["max_horizontal_speed_m_s"] is not None else None
    )
    metrics["vertical_speed_overshoot_fraction"] = (
        max(0.0, metrics["max_vertical_speed_m_s"] - 0.15) / 0.15
        if metrics["max_vertical_speed_m_s"] is not None else None
    )
    metrics["truth_vertical_speed_overshoot_fraction"] = (
        max(0.0, metrics["max_truth_vertical_speed_m_s"] - 0.15) / 0.15
        if metrics["max_truth_vertical_speed_m_s"] is not None else None
    )
    metrics["max_yaw_rate_deg_s"] = max(
        (abs(sample["actual_deg_s"]) for sample in yaw_rate_samples), default=None
    )
    metrics["yaw_rate_overshoot_fraction"] = (
        max(0.0, metrics["max_yaw_rate_deg_s"] - 15.0) / 15.0
        if metrics["max_yaw_rate_deg_s"] is not None else None
    )
    print("DDS_VELOCITY_WASD_METRICS " + json.dumps(metrics, sort_keys=True))
    missing = [value for value in required if value not in output]
    passed = (
        not missing
        and hover_settled
        # STATE is emitted at 1 Hz. Four seconds of vertical key time can
        # legitimately yield only four
        # sampled VELOCITY_CONTROL states depending on phase alignment.
        and len(velocity_states) >= (
            4 if (args.vertical_only or args.yaw_only or args.yaw_reversal
                  or args.horizontal_vertical_transition
                  or args.vertical_yaw_transition) else 6
        )
        and metrics["velocity_enters"] >= 1
        and metrics["velocity_demand_latches"] >= (
            3 if args.vertical_only else (
                3 if args.vertical_yaw_transition
                else (
                    2 if (args.yaw_only or args.yaw_reversal)
                    else (2 if args.horizontal_vertical_transition else 9)
                )
            )
        )
        and not metrics["failsafe_seen"]
        and not metrics["position_safety_land_seen"]
        and metrics["motor_saturation_fraction"] == 0.0
        and metrics["horizontal_speed_overshoot_fraction"] <= 0.15
        and metrics["vertical_speed_overshoot_fraction"] <= 0.10 + 1.0e-9
        and (
            metrics["truth_vertical_speed_overshoot_fraction"] is None
            or metrics["truth_vertical_speed_overshoot_fraction"] <= 0.10 + 1.0e-9
        )
        and (
            not (args.yaw_only or args.yaw_reversal)
            or metrics["max_zero_vertical_disturbance_m_s"] <= 0.10
        )
        and (
            metrics["max_yaw_rate_deg_s"] is None
            or metrics["yaw_rate_overshoot_fraction"] <= 0.10
        )
    )
    if not passed:
        print(f"DDS_VELOCITY_WASD_FAIL missing={missing}", file=sys.stderr)
        return 1
    print("DDS_VELOCITY_WASD_PASS")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
