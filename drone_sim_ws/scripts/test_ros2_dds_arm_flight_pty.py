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

from ros2_test_utils import ros2_child_environment


STATE_RE = re.compile(
    r"STATE arm=(\d+) nav=(\d+) NED=\(([-+0-9.e]+),([-+0-9.e]+),([-+0-9.e]+)\) "
    r"vel=\(([-+0-9.e]+),([-+0-9.e]+),([-+0-9.e]+)\)"
)
TARGET_RE = re.compile(
    r"target NED=\(([-+0-9.e]+),\s*([-+0-9.e]+),\s*([-+0-9.e]+)\)"
)


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--timeout", type=float, default=140.0)
    args = parser.parse_args()
    master, slave = pty.openpty()
    controller = subprocess.Popen(
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
    initialized = False
    first_state_time = None
    offboard_time = None
    target_ned = None
    flight_ready_time = None
    stable_candidate_time = None
    sent = set()
    arm_process = None
    arm_label = None
    arm_results = {}
    profile = os.environ.get("ARM_FLIGHT_PROFILE", "safe")
    if profile == "full_a":
        # Diagnostic only: wait for the PX4 takeoff transient to settle before
        # commanding full work_a at a deliberately slow 30 s trajectory.
        arm_schedule = [(5.0, "work_a"), (45.0, "retracted")]
        arm_duration = "30"
        land_after_s = 82.0
    elif profile == "full_a_slow":
        # Diagnostic only: the full pose is deliberately spread over 90 s.
        # This is not the default flight profile; it exists to separate
        # acceleration-induced reaction torque from the steady full-pose load.
        # Keep the prior 30 s full_a run as an independent failure baseline.
        arm_schedule = [(5.0, "work_a"), (105.0, "retracted")]
        arm_duration = "90"
        land_after_s = 205.0
    elif profile == "full_b_slow":
        # Independent full-pose acceptance run.  Keeping work_b separate from
        # work_a makes any failure attributable to this pose and preserves a
        # complete retracted -> work_b -> retracted evidence chain.
        arm_schedule = [(5.0, "work_b"), (105.0, "retracted")]
        arm_duration = "90"
        land_after_s = 205.0
    elif profile == "full_ab_slow":
        # Diagnostic only: exercise both complete work poses with a long,
        # bounded trajectory, then return to the documented retracted pose.
        arm_schedule = [
            (5.0, "work_a"),
            (105.0, "work_b"),
            (205.0, "retracted"),
        ]
        arm_duration = "90"
        land_after_s = 310.0
    elif profile == "micro":
        # Flight-safe identification motion: 25% of the reduced flight_work
        # presets, sent over a long trajectory.  Leave a settling window after
        # PX4 enters Offboard so the arm disturbance is not conflated with the
        # vehicle's own takeoff transient.
        arm_schedule = [
            (5.0, "flight_micro_a"),
            (25.0, "flight_micro_b"),
            (45.0, "retracted"),
        ]
        arm_duration = "14"
        land_after_s = 67.0
    else:
        # The 7.735 kg configuration has only a small thrust margin.  Use the
        # documented slow-work profile so arm acceleration is a measured
        # disturbance rather than an unrealistic step impulse.
        arm_schedule = [
            (5.0, "flight_work_a"),
            (25.0, "flight_work_b"),
            (45.0, "retracted"),
        ]
        arm_duration = "8"
        land_after_s = 62.0
    safety_abort = False
    safety_abort_reason = ""
    internal_land = False
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
                if (
                    "Arm reaction-torque safety gate exceeded" in output
                    or "Position safety gate exceeded" in output
                    or (
                        (
                            "LAND requested; Offboard setpoint stream stopped" in output
                            or "OFFBOARD_LANDING_STARTED" in output
                        )
                        and "LAND" not in sent
                    )
                ):
                    internal_land = True
                for match in STATE_RE.finditer(data):
                    states.append(
                        (time.monotonic(),) + tuple(float(x) for x in match.groups())
                    )
                targets = list(TARGET_RE.finditer(data))
                if targets:
                    target_ned = tuple(float(value) for value in targets[-1].groups())
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
                # The 7.735 kg vehicle has little thrust margin and can need a
                # long takeoff transient.  Start arm motion only after the
                # measured PX4 position has approached the commanded hold and
                # remained in a bounded window for three seconds.
                if flight_ready_time is None and target_ned is not None and elapsed > 10.0:
                    recent = [
                        state for state in states
                        if time.monotonic() - state[0] <= 5.0 and int(state[1]) == 2
                    ]
                    ready_now = False
                    if len(recent) >= 4:
                        latest = recent[-1]
                        horizontal_error = math.hypot(
                            latest[3] - target_ned[0], latest[4] - target_ned[1]
                        )
                        vertical_error = abs(latest[5] - target_ned[2])
                        speed = math.sqrt(
                            latest[6] * latest[6]
                            + latest[7] * latest[7]
                            + latest[8] * latest[8]
                        )
                        ranges = [
                            max(state[index] for state in recent)
                            - min(state[index] for state in recent)
                            for index in (3, 4, 5)
                        ]
                        # Do not begin arm motion while PX4 is still climbing.
                        # The 7.735 kg vehicle has only a small vertical thrust
                        # margin, so the previous broad window could accept a
                        # state nearly half a metre below the hold target.
                        # This is a motion-start gate, not the flight pass
                        # criterion.  The formal vehicle can hold a repeatable
                        # horizontal offset from its takeoff target because its
                        # canted rotors have very little remaining authority.
                        # Gate on completed climb and measured stationarity so
                        # that such a steady bias cannot block the test forever.
                        ready_now = (
                            latest[5] <= target_ned[2] + 0.40
                            and horizontal_error < 0.75
                            and vertical_error < 0.40
                            and max(ranges) < 0.20
                            and speed < 0.15
                        )
                    if ready_now:
                        if stable_candidate_time is None:
                            stable_candidate_time = time.monotonic()
                        elif time.monotonic() - stable_candidate_time >= 3.0:
                            flight_ready_time = time.monotonic()
                            print(
                                "ARM_FLIGHT_HOVER_READY "
                                f"offboard_elapsed_s={elapsed:.1f}",
                                flush=True,
                            )
                    else:
                        stable_candidate_time = None
                # Position excursions during PX4's normal landing are not a
                # flight-hold failure.  Stop evaluating the airborne safety
                # gate as soon as this runner has deliberately requested LAND.
                if (
                    not safety_abort
                    and "LAND" not in sent
                    and flight_ready_time is not None
                    and elapsed > 12.0
                ):
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
                                    env=ros2_child_environment(),
                                )
                                sent.add("retracted")
                            os.write(master, b"l")
                            sent.add("LAND")
                            print("ARM_FLIGHT_SAFETY_LAND", flush=True)
                motion_elapsed = (
                    None if flight_ready_time is None
                    else time.monotonic() - flight_ready_time
                )
                for at, preset in arm_schedule:
                    if (
                        not safety_abort
                        and not internal_land
                        and motion_elapsed is not None
                        and motion_elapsed >= at
                        and preset not in sent
                        and arm_process is None
                    ):
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
                            env=ros2_child_environment(),
                        )
                        sent.add(preset)
                        print(f"ARM_FLIGHT_SENT_{preset}", flush=True)
                        break
                if (
                    not safety_abort
                    and not internal_land
                    and motion_elapsed is not None
                    and motion_elapsed >= land_after_s
                    and "LAND" not in sent
                ):
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
    if profile in {"full_a", "full_a_slow"}:
        required_presets = ("work_a", "retracted")
    elif profile == "full_b_slow":
        required_presets = ("work_b", "retracted")
    elif profile == "full_ab_slow":
        required_presets = ("work_a", "work_b", "retracted")
    elif profile == "micro":
        required_presets = ("flight_micro_a", "flight_micro_b", "retracted")
    else:
        required_presets = ("flight_work_a", "flight_work_b", "retracted")
    for preset in required_presets:
        result = arm_results.get(preset)
        if result is None or result[0] != 0 or "ARM_PRESET_REACHED" not in result[1]:
            missing.append(f"arm:{preset}")

    armed_states = [state for state in states if int(state[1]) == 2]
    climbed = any(state[5] < -0.8 for state in armed_states)
    no_failsafe = "failsafe=True" not in output
    arm_window = []
    if flight_ready_time is not None:
        arm_window = [
            state for state in states
            if 0.0 <= state[0] - flight_ready_time <= land_after_s
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
    if internal_land:
        print("ARM_FLIGHT_INTERNAL_LAND_DETECTED")
    if (
        missing
        or not climbed
        or not no_failsafe
        or not stable
        or internal_land
        or flight_ready_time is None
    ):
        print(
            f"DDS_ARM_FLIGHT_FAIL missing={missing} climbed={climbed} "
            f"no_failsafe={no_failsafe} stable={stable} "
            f"internal_land={internal_land} hover_ready={flight_ready_time is not None}",
            file=sys.stderr,
        )
        return 1
    print("DDS_ARM_FLIGHT_PASS")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
