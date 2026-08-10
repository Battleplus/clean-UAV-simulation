#!/usr/bin/env python3
"""Fly with PX4 DDS while moving the ros2_control SO101 arm."""

import argparse
import errno
import json
import math
import os
import pty
from pathlib import Path
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
DIAGNOSTIC_RE = re.compile(
    r"arm_torque_nm=([-+0-9.e]+) motors=\[([^\]]*)\]"
)
CARTESIAN_BEGIN_RE = re.compile(
    r"CARTESIAN_DEMO_BEGIN cycle=(\d+) monotonic=([-+0-9.e]+)"
)
CARTESIAN_COMPLETE_RE = re.compile(
    r"CARTESIAN_DEMO_COMPLETE cycle=(\d+) monotonic=([-+0-9.e]+)"
)


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--timeout", type=float, default=140.0)
    args = parser.parse_args()
    master, slave = pty.openpty()
    profile = os.environ.get("ARM_FLIGHT_PROFILE", "safe")
    cartesian_distance_m = float(os.environ.get("ARM_CARTESIAN_DISTANCE_M", "0.10"))
    if not 0.0 < cartesian_distance_m <= 0.25:
        parser.error("ARM_CARTESIAN_DISTANCE_M must be in (0, 0.25]")
    rated_motor_output = 999.0
    if profile.endswith("_4kg") or profile == "cartesian_formal_7p735":
        config_name = (
            "my_drone_v3_cad_debug_4kg.json"
            if profile.endswith("_4kg")
            else "my_drone_v3_cad_7p735_flight.json"
        )
        config_path = (
            Path(__file__).resolve().parents[1]
            / "src" / "drone_arm_sim" / "config" / config_name
        )
        config = json.loads(config_path.read_text(encoding="utf-8"))
        rated_motor_output = 1000.0 * float(
            config["actuator_normalization"]["rated_thrust_command"]
        )
    controller_environment = ros2_child_environment()
    if profile.endswith("_4kg") or profile == "cartesian_formal_7p735":
        controller_environment.update(
            {
                "PX4_TOUCHDOWN_DISARM_ENABLED": "true",
                "PX4_TOUCHDOWN_DISARM_HEIGHT_M": "0.05",
                "PX4_TOUCHDOWN_DISARM_HOLD_S": "0.5",
            }
        )
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
    states = []
    diagnostics = []
    initialized = False
    first_state_time = None
    takeoff_stable_since = None
    offboard_time = None
    target_ned = None
    flight_ready_time = None
    stable_candidate_time = None
    sent = set()
    arm_process = None
    arm_label = None
    arm_results = {}
    cartesian_action_started = {}
    cartesian_action_completed = {}
    first_cartesian_completed_time = None
    between_cycle_stable_since = None
    between_cycle_hover_sent = False
    final_cartesian_stable_since = None
    velocity_burst_last_sent = {}
    post_arm_stable_since = None
    post_arm_burst_start = None
    post_arm_burst_end = None
    post_velocity_stable_since = None
    cartesian_sequence_event_file = Path("/tmp/my_drone_cartesian_sequence.events")
    cartesian_sequence_event_file.unlink(missing_ok=True)
    cartesian_sequence_events_seen = set()
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
    elif profile in {"micro", "micro_4kg"}:
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
    elif profile == "combined_4kg":
        # Integrated 4 kg bring-up: deliberately overlap bounded arm motion
        # with paired position and yaw commands.  Opposing key pairs return
        # the target to the original hover point before landing.
        arm_schedule = [
            (5.0, "flight_work_a"),
            (25.0, "flight_work_b"),
            (45.0, "retracted"),
        ]
        arm_duration = "12"
        land_after_s = 70.0
    elif profile == "demo_extended_4kg":
        # Reproduce the visible keyboard-6 cycle under instrumented flight.
        # The first H confirms that hover-reset remains live before the arm
        # moves; the second H cancels any accumulated pose error mid-cycle.
        arm_schedule = [
            (5.0, "demo_extended"),
            (20.0, "retracted"),
        ]
        arm_duration = "8"
        land_after_s = 40.0
    elif profile == "demo_extended_twice_4kg":
        # Two consecutive visible cycles reproduce the reported manual case.
        # Schedule ids are distinct even though the same two presets repeat.
        arm_schedule = [
            (5.0, "demo_extended", "demo_extended_1"),
            (24.0, "retracted", "retracted_1"),
            (53.0, "demo_extended", "demo_extended_2"),
            (72.0, "retracted", "retracted_2"),
        ]
        arm_duration = "16"
        land_after_s = 96.0
    elif profile == "cartesian_demo_4kg":
        arm_schedule = [(6.0, "cartesian_demo")]
        arm_duration = "30"
        land_after_s = 95.0
    elif profile == "cartesian_demo_twice_4kg":
        arm_schedule = [(6.0, "cartesian_sequence", "cartesian_sequence_twice")]
        arm_duration = "30"
        land_after_s = 160.0
    elif profile == "cartesian_velocity_4kg":
        # Full joint acceptance: velocity flight, automatic release hold, an
        # explicit H hold, one complete Cartesian arm cycle, another velocity
        # flight and a normal PX4 landing.  The second burst is event driven so
        # it cannot overlap arm motion merely because a trajectory ran slowly.
        arm_schedule = [(12.0, "cartesian_sequence", "cartesian_sequence_once")]
        arm_duration = "30"
        land_after_s = 180.0
    elif profile == "cartesian_formal_7p735":
        # Formal CAD vehicle distance ladder.  Each distance is a separate
        # clean-start acceptance run because this airframe has little thrust
        # margin and a shorter run must not inherit state from a prior rung.
        arm_schedule = [(6.0, "cartesian_sequence", "cartesian_sequence_once")]
        # The formal vehicle cannot yet meet the operator-facing 20 second
        # motion target without saturating.  Keep the last demonstrated safe
        # 40 second diagnostic trajectory until actuator authority or motion
        # shaping is improved; do not expose the failed faster trajectory as
        # an accepted user command.
        arm_duration = os.environ.get("ARM_FORMAL_DURATION_S", "40")
        land_after_s = 140.0
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
    controller_timed_out = False
    landing_disarmed_time = None
    flight_key_schedule = []
    if profile == "combined_4kg":
        flight_key_schedule = [
            (12.0, b"w", "W"),
            (20.0, b"d", "D"),
            (32.0, b"s", "S"),
            (40.0, b"a", "A"),
            (50.0, b"q", "Q"),
            (56.0, b"e", "E"),
        ]
    elif profile == "demo_extended_4kg":
        flight_key_schedule = [
            (3.0, b"h", "H_INITIAL"),
            (12.0, b"h", "H_ARM_MOVING"),
        ]
    elif profile == "demo_extended_twice_4kg":
        flight_key_schedule = [
            (3.0, b"h", "H_INITIAL"),
            (12.0, b"h", "H_ARM_MOVING"),
            (51.0, b"h", "H_BEFORE_SECOND"),
        ]
    elif profile in {"cartesian_demo_4kg", "cartesian_demo_twice_4kg"}:
        flight_key_schedule = [
            (0.2, b"h", "H_INITIAL"),
        ]
    elif profile == "cartesian_velocity_4kg":
        flight_key_schedule = [(8.0, b"h", "H_BEFORE_ARM")]
    elif profile == "cartesian_formal_7p735":
        flight_key_schedule = [(0.2, b"h", "H_INITIAL")]
    velocity_bursts = []
    if profile == "cartesian_velocity_4kg":
        velocity_bursts = [(2.0, 4.0, b"w", "W_BURST")]
    try:
        while time.monotonic() - start < args.timeout:
            if cartesian_sequence_event_file.exists():
                for event in cartesian_sequence_event_file.read_text(
                    encoding="utf-8", errors="replace"
                ).splitlines():
                    if event in cartesian_sequence_events_seen:
                        continue
                    cartesian_sequence_events_seen.add(event)
                    print(event, flush=True)
                    interval = re.match(
                        r"CARTESIAN_SEQUENCE_CYCLE_INTERVAL cycle=(\d+) "
                        r"start=([-+0-9.e]+) end=([-+0-9.e]+)",
                        event,
                    )
                    if interval:
                        cycle = int(interval.group(1))
                        label = f"cartesian_demo_{cycle}"
                        cartesian_action_started[label] = float(interval.group(2))
                        cartesian_action_completed[label] = float(interval.group(3))
                    if event.startswith("CARTESIAN_SEQUENCE_ERROR"):
                        safety_abort = True
                        safety_abort_reason = event
                        if "LAND" not in sent:
                            os.write(master, b"l")
                            sent.add("LAND")
                            print("ARM_FLIGHT_SEQUENCE_ERROR_LAND", flush=True)
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
                    "LANDING_DISARMED_CONFIRMED" in output
                    and landing_disarmed_time is None
                ):
                    landing_disarmed_time = time.monotonic()
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
                for match in DIAGNOSTIC_RE.finditer(data):
                    motors = [
                        float(value) for value in match.group(2).split(",") if value
                    ]
                    diagnostics.append(
                        (time.monotonic(), float(match.group(1)), motors)
                    )
                targets = list(TARGET_RE.finditer(data))
                if targets:
                    target_ned = tuple(float(value) for value in targets[-1].groups())
                if "STATE arm=" in output and first_state_time is None:
                    first_state_time = time.monotonic()
                if "OFFBOARD mode and ARM commands sent" in output and offboard_time is None:
                    offboard_time = time.monotonic()

            if first_state_time is not None and not initialized:
                takeoff_ready = time.monotonic() - first_state_time >= 2.0
                if profile == "cartesian_formal_7p735":
                    recent_ground = [
                        state for state in states
                        if time.monotonic() - state[0] <= 5.0 and int(state[1]) == 1
                    ]
                    takeoff_ready = False
                    if len(recent_ground) >= 4:
                        latest = recent_ground[-1]
                        position_ranges = [
                            max(state[index] for state in recent_ground)
                            - min(state[index] for state in recent_ground)
                            for index in (3, 4, 5)
                        ]
                        speed = math.sqrt(
                            latest[6] * latest[6]
                            + latest[7] * latest[7]
                            + latest[8] * latest[8]
                        )
                        stable_now = max(position_ranges) < 0.15 and speed < 0.10
                        if stable_now:
                            if takeoff_stable_since is None:
                                takeoff_stable_since = time.monotonic()
                            takeoff_ready = time.monotonic() - takeoff_stable_since >= 5.0
                        else:
                            takeoff_stable_since = None
                if takeoff_ready:
                    os.write(master, b"t")
                    initialized = True
                    print("ARM_FLIGHT_TAKEOFF_ESTIMATOR_READY", flush=True)

            if arm_process is not None and arm_process.poll() is not None:
                arm_stdout, _ = arm_process.communicate()
                arm_results[arm_label] = (arm_process.returncode, arm_stdout)
                print(arm_stdout, end="", flush=True)
                if arm_label in {"cartesian_sequence_twice", "cartesian_sequence_once"}:
                    starts = {
                        int(cycle): float(timestamp)
                        for cycle, timestamp in CARTESIAN_BEGIN_RE.findall(arm_stdout)
                    }
                    completes = {
                        int(cycle): float(timestamp)
                        for cycle, timestamp in CARTESIAN_COMPLETE_RE.findall(arm_stdout)
                    }
                    expected_cycles = (1, 2) if arm_label.endswith("twice") else (1,)
                    for cycle in expected_cycles:
                        label = f"cartesian_demo_{cycle}"
                        if cycle in starts:
                            cartesian_action_started[label] = starts[cycle]
                        if cycle in completes:
                            cartesian_action_completed[label] = completes[cycle]
                        arm_results[label] = (arm_process.returncode, arm_stdout)
                    if 1 in completes:
                        first_cartesian_completed_time = completes[1]
                        print("ARM_FLIGHT_FIRST_CYCLE_COMPLETE", flush=True)
                if (
                    arm_label is not None
                    and arm_label.startswith("cartesian_demo")
                    and arm_process.returncode == 0
                    and "CARTESIAN_DEMO_COMPLETE" in arm_stdout
                ):
                    cartesian_action_completed[arm_label] = time.monotonic()
                if (
                    profile == "cartesian_demo_twice_4kg"
                    and arm_label == "cartesian_demo_1"
                    and arm_process.returncode == 0
                    and "CARTESIAN_DEMO_COMPLETE" in arm_stdout
                ):
                    first_cartesian_completed_time = time.monotonic()
                    print("ARM_FLIGHT_FIRST_CYCLE_COMPLETE", flush=True)
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
                        if profile in {
                            "cartesian_demo_4kg", "cartesian_demo_twice_4kg",
                            "cartesian_velocity_4kg", "cartesian_formal_7p735",
                        }:
                            ready_now = (
                                ready_now
                                and math.hypot(latest[6], latest[7]) < 0.10
                                and abs(latest[8]) < 0.08
                            )
                    if ready_now:
                        if stable_candidate_time is None:
                            stable_candidate_time = time.monotonic()
                        elif time.monotonic() - stable_candidate_time >= (
                            5.0 if profile in {
                                "cartesian_demo_4kg", "cartesian_demo_twice_4kg",
                                "cartesian_velocity_4kg", "cartesian_formal_7p735",
                            } else 3.0
                        ):
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
                for at, key, label in flight_key_schedule:
                    sent_label = f"KEY_{label}"
                    if (
                        not safety_abort
                        and not internal_land
                        and motion_elapsed is not None
                        and motion_elapsed >= at
                        and sent_label not in sent
                    ):
                        os.write(master, key)
                        sent.add(sent_label)
                        print(f"ARM_FLIGHT_PTY_SENT_{label}", flush=True)
                for burst_start, burst_end, key, label in velocity_bursts:
                    if (
                        not safety_abort
                        and not internal_land
                        and motion_elapsed is not None
                        and burst_start <= motion_elapsed < burst_end
                        and time.monotonic() - velocity_burst_last_sent.get(label, 0.0)
                        >= 0.08
                    ):
                        os.write(master, key)
                        velocity_burst_last_sent[label] = time.monotonic()
                        sent.add(f"BURST_{label}")
                    if (
                        motion_elapsed is not None
                        and motion_elapsed >= burst_end
                        and f"BURST_END_{label}" not in sent
                    ):
                        sent.add(f"BURST_END_{label}")
                        print(f"ARM_FLIGHT_VELOCITY_BURST_COMPLETE {label}", flush=True)
                for schedule_item in arm_schedule:
                    at, preset = schedule_item[:2]
                    schedule_id = (
                        schedule_item[2] if len(schedule_item) >= 3 else preset
                    )
                    if (
                        not safety_abort
                        and not internal_land
                        and motion_elapsed is not None
                        and motion_elapsed >= at
                        and schedule_id not in sent
                        and arm_process is None
                    ):
                        arm_label = schedule_id
                        arm_command = [
                            "ros2", "run", "drone_arm_sim", "arm_preset_control",
                            "--preset", preset, "--duration", arm_duration, "--wait",
                            "--tolerance", "0.06",
                        ]
                        if preset == "cartesian_demo":
                            cartesian_action_started[schedule_id] = time.monotonic()
                            arm_command = [
                                "ros2", "run", "drone_arm_sim", "cartesian_arm_demo",
                                "--distance", f"{cartesian_distance_m:.6f}",
                                "--step", "0.005",
                                "--duration", arm_duration, "--hold", "3",
                            ]
                        elif preset == "cartesian_sequence":
                            cartesian_sequence_event_file.unlink(missing_ok=True)
                            arm_command = [
                                "ros2", "run", "drone_arm_sim",
                                "cartesian_arm_sequence",
                                "--cycles", (
                                    "1" if profile in {
                                        "cartesian_velocity_4kg",
                                        "cartesian_formal_7p735",
                                    } else "2"
                                ), "--distance", f"{cartesian_distance_m:.6f}",
                                "--step", "0.005", "--duration", arm_duration,
                                "--hold", "3", "--require-px4-stable",
                                "--stable-horizontal", "0.10",
                                "--stable-vertical", "0.08",
                                "--stable-hold", "5",
                                "--stable-timeout", "60",
                                "--event-file", str(cartesian_sequence_event_file),
                            ]
                            if profile == "cartesian_formal_7p735":
                                arm_command.extend([
                                    "--flight-gated-waypoints",
                                    "--waypoint-position-limit", "0.10",
                                    "--waypoint-stable-hold", "0.5",
                                    "--waypoint-stable-timeout", "20",
                                ])
                        arm_process = subprocess.Popen(
                            arm_command,
                            stdout=subprocess.PIPE,
                            stderr=subprocess.STDOUT,
                            text=True,
                            env=ros2_child_environment(),
                        )
                        sent.add(schedule_id)
                        print(f"ARM_FLIGHT_SENT_{schedule_id}", flush=True)
                        break
                if profile == "cartesian_velocity_4kg":
                    latest = states[-1] if states else None
                    arm_complete = "cartesian_demo_1" in cartesian_action_completed
                    stable_now = (
                        arm_complete
                        and arm_process is None
                        and latest is not None
                        and time.monotonic() - latest[0] < 1.5
                        and math.hypot(latest[6], latest[7]) < 0.10
                        and abs(latest[8]) < 0.08
                    )
                    if post_arm_burst_start is None:
                        if stable_now:
                            if post_arm_stable_since is None:
                                post_arm_stable_since = time.monotonic()
                            elif time.monotonic() - post_arm_stable_since >= 5.0:
                                post_arm_burst_start = time.monotonic()
                                post_arm_burst_end = post_arm_burst_start + 2.0
                                print("ARM_FLIGHT_POST_ARM_STABLE", flush=True)
                        else:
                            post_arm_stable_since = None
                    elif time.monotonic() < post_arm_burst_end:
                        if time.monotonic() - velocity_burst_last_sent.get("D_BURST", 0.0) >= 0.08:
                            os.write(master, b"d")
                            velocity_burst_last_sent["D_BURST"] = time.monotonic()
                            sent.add("BURST_D_BURST")
                    else:
                        if "BURST_END_D_BURST" not in sent:
                            sent.add("BURST_END_D_BURST")
                            print("ARM_FLIGHT_VELOCITY_BURST_COMPLETE D_BURST", flush=True)
                        if stable_now:
                            if post_velocity_stable_since is None:
                                post_velocity_stable_since = time.monotonic()
                        else:
                            post_velocity_stable_since = None
                cartesian_land_ready = True
                if profile in {
                    "cartesian_demo_4kg", "cartesian_demo_twice_4kg",
                    "cartesian_formal_7p735",
                }:
                    required_cartesian = (
                        ("cartesian_demo",)
                        if profile == "cartesian_demo_4kg"
                        else (
                            ("cartesian_demo_1",)
                            if profile == "cartesian_formal_7p735"
                            else ("cartesian_demo_1", "cartesian_demo_2")
                        )
                    )
                    all_cartesian_complete = all(
                        label in cartesian_action_completed
                        for label in required_cartesian
                    )
                    latest = states[-1] if states else None
                    final_stable_now = (
                        all_cartesian_complete
                        and arm_process is None
                        and latest is not None
                        and time.monotonic() - latest[0] < 1.5
                        and math.hypot(latest[6], latest[7]) < 0.10
                        and abs(latest[8]) < 0.08
                    )
                    if final_stable_now:
                        if final_cartesian_stable_since is None:
                            final_cartesian_stable_since = time.monotonic()
                    else:
                        final_cartesian_stable_since = None
                    cartesian_land_ready = (
                        final_cartesian_stable_since is not None
                        and time.monotonic() - final_cartesian_stable_since >= 5.0
                    )
                elif profile == "cartesian_velocity_4kg":
                    cartesian_land_ready = (
                        post_velocity_stable_since is not None
                        and time.monotonic() - post_velocity_stable_since >= 5.0
                    )
                if (
                    not safety_abort
                    and not internal_land
                    and cartesian_land_ready
                    and motion_elapsed is not None
                    and (
                        motion_elapsed >= land_after_s
                        or profile == "cartesian_velocity_4kg"
                    )
                    and "LAND" not in sent
                ):
                    os.write(master, b"l")
                    sent.add("LAND")
                    print("ARM_FLIGHT_SENT_LAND", flush=True)
            if controller.poll() is not None:
                break
            # The ROS 2 launcher occasionally keeps the PTY child alive after
            # the controller has already confirmed a complete PX4 landing and
            # disarm.  Give normal teardown a short grace period, then let the
            # runner's finally block terminate the stale launcher wrapper.
            # Flight acceptance below still requires the explicit PX4 LAND
            # acknowledgement and LANDING_DISARMED_CONFIRMED markers.
            if (
                landing_disarmed_time is not None
                and time.monotonic() - landing_disarmed_time >= 2.0
            ):
                print("ARM_FLIGHT_CONTROLLER_EXIT_GRACE_EXPIRED", flush=True)
                break
        if controller.poll() is None:
            controller_timed_out = True
            print("ARM_FLIGHT_CONTROLLER_TIMEOUT", flush=True)
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
    if controller_timed_out:
        missing.append("flight-controller-timeout")
    if profile in {"full_a", "full_a_slow"}:
        required_presets = ("work_a", "retracted")
    elif profile == "full_b_slow":
        required_presets = ("work_b", "retracted")
    elif profile == "full_ab_slow":
        required_presets = ("work_a", "work_b", "retracted")
    elif profile in {"micro", "micro_4kg"}:
        required_presets = ("flight_micro_a", "flight_micro_b", "retracted")
    elif profile in {"demo_extended_4kg", "demo_extended_twice_4kg"}:
        required_presets = ("demo_extended", "retracted")
    elif profile == "cartesian_demo_4kg":
        required_presets = ("cartesian_demo",)
    elif profile == "cartesian_demo_twice_4kg":
        required_presets = ("cartesian_demo_1", "cartesian_demo_2")
    elif profile == "cartesian_velocity_4kg":
        required_presets = ("cartesian_demo_1",)
    elif profile == "cartesian_formal_7p735":
        required_presets = ("cartesian_demo_1",)
    else:
        required_presets = ("flight_work_a", "flight_work_b", "retracted")
    for preset in required_presets:
        result = arm_results.get(preset)
        required_marker = (
            "CARTESIAN_DEMO_COMPLETE"
            if preset.startswith("cartesian_demo") else "ARM_PRESET_REACHED"
        )
        if result is None or result[0] != 0 or required_marker not in result[1]:
            missing.append(f"arm:{preset}")
    if profile == "combined_4kg":
        for label in ("W", "D", "S", "A", "Q", "E"):
            if f"KEY_{label}" not in sent:
                missing.append(f"key:{label}")
    elif profile == "cartesian_velocity_4kg":
        for label in ("W_BURST", "D_BURST"):
            if f"BURST_{label}" not in sent or f"BURST_END_{label}" not in sent:
                missing.append(f"velocity-burst:{label}")
        for marker in (
            "VELOCITY_CONTROL_ENTER key=W",
            "VELOCITY_CONTROL_ENTER key=D",
            "VELOCITY_RELEASE_HOLD",
            "HOVER_HOLD_CURRENT",
        ):
            if marker not in output:
                missing.append(f"controller-marker:{marker}")

    armed_states = [state for state in states if int(state[1]) == 2]
    climbed = any(state[5] < -0.8 for state in armed_states)
    no_failsafe = "failsafe=True" not in output
    cartesian_labels = ()
    if profile == "cartesian_demo_4kg":
        cartesian_labels = ("cartesian_demo",)
    elif profile == "cartesian_demo_twice_4kg":
        cartesian_labels = ("cartesian_demo_1", "cartesian_demo_2")
    elif profile == "cartesian_velocity_4kg":
        cartesian_labels = ("cartesian_demo_1",)
    elif profile == "cartesian_formal_7p735":
        cartesian_labels = ("cartesian_demo_1",)

    action_intervals = []
    arm_window = []
    if cartesian_labels:
        for label in cartesian_labels:
            action_start = cartesian_action_started.get(label)
            action_end = cartesian_action_completed.get(label)
            if action_start is None or action_end is None:
                continue
            action_intervals.append((label, action_start, action_end))
            arm_window.extend(
                state for state in states
                if action_start <= state[0] <= action_end
            )
    elif flight_ready_time is not None:
        arm_window = [
            state for state in states
            if 0.0 <= state[0] - flight_ready_time <= land_after_s
        ]
    max_horizontal_drift = float("inf")
    altitude_span = float("inf")
    if action_intervals:
        cycle_drifts = []
        cycle_altitude_spans = []
        for label, action_start, action_end in action_intervals:
            cycle_window = [
                state for state in states
                if action_start <= state[0] <= action_end
            ]
            if not cycle_window:
                continue
            north0, east0 = cycle_window[0][3], cycle_window[0][4]
            cycle_drift = max(
                math.hypot(state[3] - north0, state[4] - east0)
                for state in cycle_window
            )
            cycle_down = [state[5] for state in cycle_window]
            cycle_altitude_span = max(cycle_down) - min(cycle_down)
            cycle_drifts.append(cycle_drift)
            cycle_altitude_spans.append(cycle_altitude_span)
            print(
                "ARM_FLIGHT_CYCLE_METRICS "
                f"label={label} horizontal_drift_m={cycle_drift:.3f} "
                f"altitude_span_m={cycle_altitude_span:.3f} "
                f"samples={len(cycle_window)}"
            )
        if len(cycle_drifts) == len(cartesian_labels):
            max_horizontal_drift = max(cycle_drifts)
            altitude_span = max(cycle_altitude_spans)
    elif arm_window:
        north0, east0 = arm_window[0][3], arm_window[0][4]
        max_horizontal_drift = max(
            math.hypot(state[3] - north0, state[4] - east0) for state in arm_window
        )
        down_values = [state[5] for state in arm_window]
        altitude_span = max(down_values) - min(down_values)
    diagnostic_window = []
    if action_intervals:
        diagnostic_window = [
            sample for sample in diagnostics
            if any(start <= sample[0] <= end for _, start, end in action_intervals)
        ]
    elif flight_ready_time is not None:
        diagnostic_window = [
            sample for sample in diagnostics
            if 0.0 <= sample[0] - flight_ready_time <= land_after_s
        ]
    max_arm_torque = max(
        (sample[1] for sample in diagnostic_window), default=float("inf")
    )
    motor_samples = [
        motors for _, _, motors in diagnostic_window if len(motors) >= 8
    ]
    saturation_samples = sum(
        1
        for motors in motor_samples
        if any(value >= rated_motor_output for value in motors[:8])
    )
    saturation_rate = (
        saturation_samples / len(motor_samples) if motor_samples else float("inf")
    )
    stable = max_horizontal_drift < 1.5 and altitude_span < 1.5
    if profile in {
        "cartesian_demo_4kg", "cartesian_demo_twice_4kg",
        "cartesian_velocity_4kg", "cartesian_formal_7p735",
    }:
        stable = (
            max_horizontal_drift < 0.15
            and altitude_span < 0.30
            and max_arm_torque < 0.50
            and saturation_samples == 0
        )
    print(
        "ARM_FLIGHT_METRICS "
        f"horizontal_drift_m={max_horizontal_drift:.3f} "
        f"altitude_span_m={altitude_span:.3f} "
        f"max_arm_torque_nm={max_arm_torque:.3f} "
        f"motor_saturation_samples={saturation_samples}/{len(motor_samples)} "
        f"motor_saturation_rate={saturation_rate:.6f} "
        f"rated_motor_output={rated_motor_output:.3f} "
        f"samples={len(arm_window)}"
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
