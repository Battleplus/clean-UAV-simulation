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
    r"arm_torque_nm=([-+0-9.e]+) "
    r"arm_force_n=([-+0-9.e]+) "
    r"arm_com_shift_m=([-+0-9.e]+) "
    r"arm_inertia_diag=\(([^)]*)\) motors=\[([^\]]*)\]"
)
TRUTH_RPY_RE = re.compile(
    r"truth_rpy_deg=\(([-+0-9.e]+),([-+0-9.e]+),([-+0-9.e]+)\)"
)
TRUTH_ENU_RE = re.compile(
    r"truth_enu=\(([-+0-9.e]+),([-+0-9.e]+),([-+0-9.e]+)\)"
)
CARTESIAN_BEGIN_RE = re.compile(
    r"CARTESIAN_DEMO_BEGIN cycle=(\d+) monotonic=([-+0-9.e]+)"
)
CARTESIAN_COMPLETE_RE = re.compile(
    r"CARTESIAN_DEMO_COMPLETE cycle=(\d+) monotonic=([-+0-9.e]+)"
)
DIRECTIONAL_STAGE_RE = re.compile(
    r"DIRECTIONAL_STAGE_INTERVAL direction=([a-z_]+) phase=(extend|hold|retract) "
    r"start=([-+0-9.e]+) end=([-+0-9.e]+)"
)
DIRECTIONAL_ORDER = (
    "front", "rear", "left", "right", "up", "down",
    "front_left", "front_right", "rear_left", "rear_right",
)
DIRECTIONAL_RECOVERY_LABEL = "directional_abort_retracted"
DIRECTIONAL_RECOVERY_STABLE_HOLD_S = 2.0


def directional_abort_recovery_command() -> list[str]:
    """Return the one allowed arm command after a directional-motion abort."""
    return cpu_role_command("arm", [
        "ros2", "run", "drone_arm_sim", "arm_preset_control",
        "--preset", "retracted", "--duration", "20", "--wait",
        "--tolerance", "0.08", "--flight-preflight",
    ])


def cpu_role_command(role: str, command: list[str]) -> list[str]:
    """Wrap a child in the candidate's non-root inherited CPU affinity."""
    runner = Path(__file__).resolve().parent / "run_with_cpu_role.sh"
    return [str(runner), role, *command]


def directional_abort_recovery_reached(returncode: int, output: str) -> bool:
    """Require both a successful child and measured retracted-pose evidence."""
    return bool(
        int(returncode) == 0
        and "ARM_PRESET_REACHED preset=retracted" in str(output)
    )


def directional_abort_recovery_stable(latest, now_s: float) -> bool:
    """Validate the fresh PX4 hold required before abort-recovery LAND."""
    return bool(
        latest is not None
        and float(now_s) - float(latest[0]) < 1.5
        and int(latest[1]) == 2
        and int(latest[2]) == 14
        and math.hypot(float(latest[6]), float(latest[7])) < 0.10
        and abs(float(latest[8])) < 0.08
    )


def _ros_topic_sample(topic: str, timeout_s: float) -> str:
    """Return one live ROS sample or raise before an automated flight can arm."""
    result = subprocess.run(
        ["ros2", "topic", "echo", "--once", topic],
        capture_output=True,
        text=True,
        timeout=max(0.1, timeout_s),
        env=ros2_child_environment(),
        check=False,
    )
    if result.returncode != 0 or not result.stdout.strip():
        detail = (result.stderr or result.stdout).strip().replace("\n", " ")
        raise RuntimeError(f"no live sample on {topic}: {detail[:240]}")
    return result.stdout


def require_live_backend(timeout_s: float = 8.0) -> None:
    """Reject a stale ROS graph left behind after Gazebo or PX4 exited.

    Topic names alone are insufficient because bridges and overlay nodes can
    survive a crashed Gazebo server.  Every automated arm-flight run therefore
    requires fresh physics, arm, truth-odometry and PX4 samples before it opens
    the Offboard controller PTY.
    """
    for topic in (
        "/clock",
        "/joint_states",
        "/model/my_drone/odometry",
        "/fmu/out/vehicle_status_v4",
    ):
        deadline = time.monotonic() + max(timeout_s, 1.0)
        last_error = None
        while time.monotonic() < deadline:
            try:
                # A fresh `ros2 topic echo` process can spend several seconds
                # in DDS discovery on WSL even while a high-rate publisher is
                # healthy.  Retry bounded probes inside one overall startup
                # deadline; this does not relax any in-flight freshness gate.
                _ros_topic_sample(topic, min(8.0, deadline - time.monotonic()))
                break
            except (RuntimeError, subprocess.TimeoutExpired) as exc:
                last_error = exc
                time.sleep(0.25)
        else:
            if last_error is not None:
                raise last_error
            raise RuntimeError(f"no live sample on {topic}")


def climbed_relative_to_takeoff_ground(states, minimum_climb_m=0.8):
    """Use relative NED displacement because EKF local origin is arbitrary."""
    first_armed_index = next(
        (index for index, state in enumerate(states) if int(state[1]) == 2), None
    )
    if first_armed_index is None:
        return False
    ground_states = [
        state for state in states[:first_armed_index] if int(state[1]) == 1
    ]
    armed_states = [state for state in states if int(state[1]) == 2]
    if not ground_states or not armed_states:
        return False
    # Median-like central sample avoids a single noisy EKF ground reading.
    ground_down = sorted(state[5] for state in ground_states)[len(ground_states) // 2]
    return ground_down - min(state[5] for state in armed_states) >= minimum_climb_m


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--timeout", type=float, default=140.0)
    args = parser.parse_args()
    master, slave = pty.openpty()
    profile = os.environ.get("ARM_FLIGHT_PROFILE", "safe")
    cartesian_distance_m = float(os.environ.get("ARM_CARTESIAN_DISTANCE_M", "0.10"))
    cartesian_hold_s = float(os.environ.get("ARM_CARTESIAN_HOLD_S", "3.0"))
    if cartesian_distance_m == 0.0 or abs(cartesian_distance_m) > 0.25:
        parser.error("ARM_CARTESIAN_DISTANCE_M magnitude must be in (0, 0.25]")
    if not 0.0 <= cartesian_hold_s <= 30.0:
        parser.error("ARM_CARTESIAN_HOLD_S must be in [0, 30]")
    try:
        require_live_backend(
            float(os.environ.get("ARM_FLIGHT_BACKEND_SAMPLE_TIMEOUT_S", "30"))
        )
    except (RuntimeError, subprocess.TimeoutExpired) as exc:
        print(f"ARM_FLIGHT_BACKEND_NOT_READY {exc}", file=sys.stderr, flush=True)
        return 2
    print("ARM_FLIGHT_BACKEND_READY clock joint_states odometry px4_status", flush=True)
    rated_motor_output = 999.0
    selected_flight_config = os.environ.get("MY_DRONE_FLIGHT_CONFIG", "").strip()
    if selected_flight_config:
        config_path = Path(selected_flight_config).expanduser().resolve()
        config = json.loads(config_path.read_text(encoding="utf-8"))
        rated_motor_output = 1000.0 * float(
            config["actuator_normalization"]["rated_thrust_command"]
        )
    elif profile.endswith("_4kg") or profile == "cartesian_formal_7p735":
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
    arm_environment = ros2_child_environment()
    if profile.endswith("_4kg") or profile == "cartesian_formal_7p735":
        controller_environment.update(
            {
                "PX4_TOUCHDOWN_DISARM_ENABLED": "true",
                "PX4_TOUCHDOWN_DISARM_HEIGHT_M": os.environ.get(
                    "PX4_TOUCHDOWN_DISARM_HEIGHT_M", "0.05"
                ),
                "PX4_TOUCHDOWN_DISARM_HOLD_S": os.environ.get(
                    "PX4_TOUCHDOWN_DISARM_HOLD_S", "0.5"
                ),
            }
        )
    if profile.endswith("_4kg"):
        # The Base 1 backend and this PTY controller are separate processes;
        # exports made by the backend launcher do not reach this child.  Keep
        # the ideal-motion-capture H hold explicit for Base 1 while allowing
        # callers to disable it for an A/B run.
        controller_environment["PX4_TRUTH_HOLD_ENABLED"] = os.environ.get(
            "PX4_TRUTH_HOLD_ENABLED", "true"
        )
        # The debug model re-indexes all joint zeroes while preserving the
        # formal CAD geometry.  Never let an automated flight silently fall
        # back to the formal 7.735 kg preset file: identically named presets
        # would then command a different physical pose.
        arm_environment["SO101_MOTION_REFERENCE"] = str(
            Path(__file__).resolve().parents[1]
            / "src/drone_arm_sim/config/so101_motion_reference_4kg.json"
        )
    # Formal acceptance must execute the controller from this checkout.  A
    # second PX4 message workspace is sourced for px4_msgs and can expose an
    # older px4_ros2_control console entry point through Python distribution
    # metadata even when the local colcon prefix is first.  Launching the
    # authoritative source file also makes the process identity auditable in
    # `ps` and prevents a stale installed controller from silently running.
    controller_source = (
        Path(__file__).resolve().parents[1]
        / "src/px4_ros2_control/px4_ros2_control/dds_wasd_control.py"
    )
    if not controller_source.is_file():
        parser.error(f"current-workspace controller not found: {controller_source}")
    print(f"ARM_FLIGHT_CONTROLLER_SOURCE path={controller_source}", flush=True)
    controller = subprocess.Popen(
        cpu_role_command("controller", [sys.executable, str(controller_source)]),
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
    truth_attitudes = []
    truth_positions = []
    initialized = False
    seen_armed = False
    prehover_disarm = False
    first_state_time = None
    takeoff_stable_since = None
    ground_stable_hold_s = float(
        os.environ.get("ARM_FLIGHT_GROUND_STABLE_HOLD_S", "5.0")
    )
    offboard_time = None
    target_ned = None
    flight_ready_time = None
    stable_candidate_time = None
    sent = set()
    arm_process = None
    arm_label = None
    arm_return_duration = None
    directional_plan_path = None
    directional_order = DIRECTIONAL_ORDER
    arm_results = {}
    arm_action_started = {}
    arm_action_completed = {}
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
    directional_event_file = Path("/tmp/my_drone_directional_workspace.events")
    directional_event_file.unlink(missing_ok=True)
    directional_events_seen = set()
    directional_stage_intervals = {}
    directional_recovery_pending = False
    directional_recovery_started = False
    directional_recovery_reached = False
    directional_recovery_stable_since = None
    directional_recovery_blocked = False
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
    elif profile == "gripper_4kg":
        # First coupling rung: move only the gripper while every upstream arm
        # joint remains at the CAD retracted pose.
        arm_schedule = [
            (5.0, "gripper_open"),
            (20.0, "gripper_closed"),
        ]
        arm_duration = "10"
        land_after_s = 38.0
    elif profile == "wrist_roll_4kg":
        # Second coupling rung: only wrist_roll moves; the gripper remains
        # closed and every upstream joint remains retracted.  Use a slower
        # trajectory than the first 0.30 rad identification run; that run is
        # retained in analysis as a failed strict-gate data point.
        arm_schedule = [
            (5.0, "wrist_roll_test"),
            (22.0, "wrist_roll_home"),
        ]
        arm_duration = "12"
        land_after_s = 42.0
    elif profile == "shoulder_pan_4kg":
        # Third coupling rung: rotate the complete downstream arm chain about
        # shoulder_pan, but keep every other joint at the retracted pose.  The
        # deliberately small 0.10 rad move over 15 s identifies upstream-joint
        # coupling before any multi-joint trajectory is authorized.
        arm_schedule = [
            (5.0, "shoulder_pan_slow_test"),
            (25.0, "shoulder_pan_home"),
        ]
        arm_duration = "15"
        land_after_s = 47.0
    elif profile == "multi_joint_slow_4kg":
        # Fourth coupling rung: all six joints make the already bounded
        # flight_work_a move together, but over 30 s in each direction.  This
        # is intentionally separate from the full work_a/extended trajectory.
        arm_schedule = [
            (5.0, "flight_work_a"),
            (40.0, "retracted"),
        ]
        arm_duration = "30"
        land_after_s = 76.0
    elif profile == "full_extend_slow_4kg":
        # Fifth coupling rung: use the complete, visibly extended target with
        # no amplitude reduction.  Ninety seconds each way isolates the
        # static COM/inertia change from an unnecessarily aggressive joint
        # acceleration before payload testing is considered.
        arm_schedule = [
            (5.0, "demo_extended"),
            (100.0, "retracted"),
        ]
        arm_duration = "90"
        land_after_s = 196.0
    elif profile == "nose_forward_straight_4kg":
        # Instrument the exact operator key-6 pose: zero shoulder-pan yaw,
        # complete nose-forward extension, hold through preset completion and
        # a lower-acceleration return to the CAD folded pose.  The aircraft is
        # dynamically asymmetric: measured pitch-rate during the old 90 s
        # return was over twice the extension value as articulated inertia
        # decreased.  Keep the proven extension unchanged and validate the
        # return with an independent duration.
        arm_duration = os.environ.get(
            "ARM_NOSE_FORWARD_EXTENSION_DURATION_S", "90"
        )
        arm_hold_duration = os.environ.get(
            "ARM_NOSE_FORWARD_HOLD_S", "5"
        )
        arm_return_duration = os.environ.get(
            "ARM_NOSE_FORWARD_RETURN_DURATION_S", "120"
        )
        if (
            float(arm_duration) <= 0.0
            or float(arm_hold_duration) < 0.0
            or float(arm_return_duration) <= 0.0
        ):
            parser.error(
                "nose-forward extension/return must be positive and hold nonnegative"
            )
        return_start_s = 5.0 + float(arm_duration) + float(arm_hold_duration)
        arm_schedule = [
            (5.0, "flight_straight_forward"),
            (return_start_s, "retracted"),
        ]
        land_after_s = return_start_s + float(arm_return_duration) + 6.0
    elif profile == "directional_workspace_4kg":
        # One clean PX4/Gazebo flight covers ten endpoint directions.  The
        # dedicated child returns home after every direction and writes exact
        # extend/hold/retract monotonic intervals for per-stage acceptance.
        directional_plan_path = Path(
            os.environ.get(
                "ARM_DIRECTIONAL_PLAN",
                str(
                    Path(__file__).resolve().parents[1]
                    / "analysis/base1/directional_workspace_flight_plan_4kg.json"
                ),
            )
        ).resolve()
        if not directional_plan_path.is_file():
            parser.error(f"directional plan not found: {directional_plan_path}")
        directional_plan = json.loads(
            directional_plan_path.read_text(encoding="utf-8")
        )
        if directional_plan.get("direction_order") != list(DIRECTIONAL_ORDER):
            parser.error("directional plan does not contain the required ten directions")
        requested_directions = [
            item.strip()
            for item in os.environ.get("ARM_DIRECTIONAL_ONLY", "").split(",")
            if item.strip()
        ]
        if requested_directions:
            if len(set(requested_directions)) != len(requested_directions):
                parser.error("ARM_DIRECTIONAL_ONLY contains duplicate directions")
            unknown = [
                direction for direction in requested_directions
                if direction not in DIRECTIONAL_ORDER
            ]
            if unknown:
                parser.error(
                    "ARM_DIRECTIONAL_ONLY contains unknown directions: "
                    + ",".join(unknown)
                )
            directional_order = tuple(requested_directions)
        arm_schedule = [(5.0, "directional_workspace_sequence")]
        arm_duration = "0"
        selected_legs = [
            leg for leg in directional_plan["legs"]
            if leg["direction"] in directional_order
        ]
        selected_motion_s = sum(
            float(leg["outward"]["effective_duration_s"])
            + float(leg["hold_s"])
            + float(leg["return"]["effective_duration_s"])
            + float(leg["settle_s"])
            for leg in selected_legs
        )
        land_after_s = 5.0 + selected_motion_s + 45.0
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
        arm_duration = os.environ.get("ARM_CARTESIAN_DURATION_S", "30")
        if float(arm_duration) <= 0.0:
            parser.error("ARM_CARTESIAN_DURATION_S must be positive")
        land_after_s = 2.0 * float(arm_duration) + cartesian_hold_s + 35.0
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
    controller_stream_stopped = False
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
    elif profile in {"nose_forward_straight_4kg", "directional_workspace_4kg"}:
        flight_key_schedule = [(0.2, b"h", "H_INITIAL")]
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
            if directional_event_file.exists():
                for event in directional_event_file.read_text(
                    encoding="utf-8", errors="replace"
                ).splitlines():
                    if event in directional_events_seen:
                        continue
                    directional_events_seen.add(event)
                    print(event, flush=True)
                    interval = DIRECTIONAL_STAGE_RE.fullmatch(event)
                    if interval:
                        direction, phase, interval_start, interval_end = interval.groups()
                        directional_stage_intervals[(direction, phase)] = (
                            float(interval_start), float(interval_end)
                        )
                    if event == "DIRECTIONAL_EMERGENCY_RETRACT_COMPLETE":
                        directional_recovery_reached = True
                        directional_recovery_pending = False
                        directional_recovery_started = True
                    elif event.startswith(
                        "DIRECTIONAL_EMERGENCY_RETRACT_BLOCKED"
                    ):
                        directional_recovery_blocked = True
                        directional_recovery_pending = False
                        directional_recovery_started = True
                    if event.startswith("DIRECTIONAL_SEQUENCE_ERROR"):
                        safety_abort = True
                        safety_abort_reason = event
                        if (
                            not directional_recovery_pending
                            and not directional_recovery_blocked
                            and not directional_recovery_reached
                        ):
                            directional_recovery_pending = True
                            print(
                                "ARM_FLIGHT_DIRECTIONAL_RECOVERY_PENDING "
                                f"reason={event}",
                                flush=True,
                            )
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
                if "PX4 status timeout: stopping Offboard stream" in data:
                    controller_stream_stopped = True
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
                    state = (
                        (time.monotonic(),)
                        + tuple(float(x) for x in match.groups())
                    )
                    states.append(state)
                    arming_state = int(state[1])
                    if arming_state == 2:
                        seen_armed = True
                    elif (
                        seen_armed
                        and arming_state == 1
                        and flight_ready_time is None
                    ):
                        prehover_disarm = True
                for match in DIAGNOSTIC_RE.finditer(data):
                    motors = [
                        float(value) for value in match.group(5).split(",") if value
                    ]
                    inertia_diag = [
                        float(value) for value in match.group(4).split(",") if value
                    ]
                    if len(inertia_diag) == 3:
                        diagnostics.append({
                            "timestamp": time.monotonic(),
                            "reaction_torque_norm_nm": float(match.group(1)),
                            "reaction_force_norm_n": float(match.group(2)),
                            "com_shift_norm_m": float(match.group(3)),
                            "inertia_diag_kg_m2": inertia_diag,
                            "motors": motors,
                        })
                for match in TRUTH_RPY_RE.finditer(data):
                    truth_attitudes.append(
                        (time.monotonic(),) + tuple(
                            float(value) for value in match.groups()
                        )
                    )
                for match in TRUTH_ENU_RE.finditer(data):
                    truth_positions.append(
                        (time.monotonic(),) + tuple(
                            float(value) for value in match.groups()
                        )
                    )
                targets = list(TARGET_RE.finditer(data))
                if targets:
                    target_ned = tuple(float(value) for value in targets[-1].groups())
                if "STATE arm=" in output and first_state_time is None:
                    first_state_time = time.monotonic()
                if "OFFBOARD mode and ARM commands sent" in output and offboard_time is None:
                    offboard_time = time.monotonic()

            if prehover_disarm:
                print(
                    "ARM_FLIGHT_PREHOVER_DISARM "
                    "vehicle disarmed after arming but before hover-ready gate",
                    flush=True,
                )
                break

            if controller_stream_stopped:
                controller_timed_out = True
                print(
                    "ARM_FLIGHT_CONTROLLER_STREAM_STOPPED "
                    "PX4 status timed out; aborting before further flight actions",
                    flush=True,
                )
                break

            if first_state_time is not None and not initialized:
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
                    position_limit = (
                        0.15 if profile == "cartesian_formal_7p735" else 0.08
                    )
                    stable_now = max(position_ranges) < position_limit and speed < 0.08
                    if stable_now:
                        if takeoff_stable_since is None:
                            takeoff_stable_since = time.monotonic()
                        takeoff_ready = (
                            time.monotonic() - takeoff_stable_since
                            >= ground_stable_hold_s
                        )
                    else:
                        takeoff_stable_since = None
                if takeoff_ready:
                    os.write(master, b"t")
                    initialized = True
                    print("ARM_FLIGHT_TAKEOFF_ESTIMATOR_READY", flush=True)

            if arm_process is not None and arm_process.poll() is not None:
                completed_arm_label = arm_label
                completed_arm_returncode = int(arm_process.returncode)
                arm_stdout, _ = arm_process.communicate()
                arm_results[completed_arm_label] = (
                    completed_arm_returncode,
                    arm_stdout,
                )
                arm_action_completed[completed_arm_label] = time.monotonic()
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
                if (
                    profile == "directional_workspace_4kg"
                    and completed_arm_label == "directional_workspace_sequence"
                    and completed_arm_returncode != 0
                ):
                    safety_abort = True
                    safety_abort_reason = (
                        f"directional-sequence-exit={completed_arm_returncode}"
                    )
                    if "DIRECTIONAL_EMERGENCY_RETRACT_COMPLETE" in arm_stdout:
                        directional_recovery_reached = True
                        directional_recovery_pending = False
                        directional_recovery_started = True
                        print(
                            "ARM_FLIGHT_DIRECTIONAL_RECOVERY_RETRACTED_REACHED",
                            flush=True,
                        )
                    elif "DIRECTIONAL_EMERGENCY_RETRACT_BLOCKED" in arm_stdout:
                        directional_recovery_blocked = True
                        directional_recovery_pending = False
                        directional_recovery_started = True
                        print(
                            "ARM_FLIGHT_DIRECTIONAL_RECOVERY_BLOCKED "
                            "reason=internal_preflighted_return_blocked",
                            flush=True,
                        )
                    elif not directional_recovery_pending:
                        directional_recovery_pending = True
                        print(
                            "ARM_FLIGHT_DIRECTIONAL_RECOVERY_PENDING "
                            f"reason={safety_abort_reason}",
                            flush=True,
                        )
                if completed_arm_label == DIRECTIONAL_RECOVERY_LABEL:
                    directional_recovery_reached = (
                        directional_abort_recovery_reached(
                            completed_arm_returncode,
                            arm_stdout,
                        )
                    )
                    if directional_recovery_reached:
                        print(
                            "ARM_FLIGHT_DIRECTIONAL_RECOVERY_RETRACTED_REACHED",
                            flush=True,
                        )
                    else:
                        directional_recovery_blocked = True
                        print(
                            "ARM_FLIGHT_DIRECTIONAL_RECOVERY_BLOCKED "
                            f"reason=retracted_exit_{completed_arm_returncode}",
                            flush=True,
                        )
                arm_process = None
                arm_label = None

            if (
                profile == "directional_workspace_4kg"
                and directional_recovery_pending
                and not directional_recovery_started
                and not directional_recovery_blocked
                and not directional_recovery_reached
                and arm_process is None
            ):
                arm_label = DIRECTIONAL_RECOVERY_LABEL
                arm_process = subprocess.Popen(
                    directional_abort_recovery_command(),
                    stdout=subprocess.PIPE,
                    stderr=subprocess.STDOUT,
                    text=True,
                    env=arm_environment,
                )
                arm_action_started[arm_label] = time.monotonic()
                directional_recovery_started = True
                print("ARM_FLIGHT_DIRECTIONAL_RECOVERY_STARTED", flush=True)

            if directional_recovery_blocked:
                # Keep the joints frozen and the controller alive.  v7 proved
                # that automatic LAND with the arm extended can destabilize
                # this airframe catastrophically.  Only a completed retract
                # followed by the existing measured stable hold may land.
                pass

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
                            "directional_workspace_4kg",
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
                                "directional_workspace_4kg",
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
                                    cpu_role_command("arm", [
                                        "ros2", "run", "drone_arm_sim", "arm_preset_control",
                                        "--preset", "retracted", "--duration", "10", "--wait",
                                        "--tolerance", "0.08",
                                    ]),
                                    stdout=subprocess.PIPE,
                                    stderr=subprocess.STDOUT,
                                    text=True,
                                    env=arm_environment,
                                )
                                sent.add("retracted")
                            os.write(master, b"l")
                            sent.add("LAND")
                            print("ARM_FLIGHT_SAFETY_LAND", flush=True)
                motion_elapsed = (
                    None if flight_ready_time is None
                    else time.monotonic() - flight_ready_time
                )
                if (
                    profile == "directional_workspace_4kg"
                    and directional_recovery_reached
                    and "LAND" not in sent
                ):
                    recovery_now = time.monotonic()
                    recovery_stable_now = directional_abort_recovery_stable(
                        states[-1] if states else None,
                        recovery_now,
                    )
                    if recovery_stable_now:
                        if directional_recovery_stable_since is None:
                            directional_recovery_stable_since = recovery_now
                        elif (
                            recovery_now - directional_recovery_stable_since
                            >= DIRECTIONAL_RECOVERY_STABLE_HOLD_S
                        ):
                            os.write(master, b"l")
                            sent.add("LAND")
                            print(
                                "ARM_FLIGHT_DIRECTIONAL_RECOVERY_STABLE_LAND",
                                flush=True,
                            )
                    else:
                        directional_recovery_stable_since = None
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
                        command_duration = (
                            arm_return_duration
                            if profile == "nose_forward_straight_4kg"
                            and preset == "retracted"
                            else arm_duration
                        )
                        arm_command = [
                            "ros2", "run", "drone_arm_sim", "arm_preset_control",
                            "--preset", preset, "--duration", command_duration, "--wait",
                            "--tolerance", "0.06",
                        ]
                        if profile == "nose_forward_straight_4kg":
                            # This strict profile validates the same airborne
                            # preflight path used by the operator keyboard.
                            # Both extension and safety-critical full return
                            # must be accepted before any trajectory is sent.
                            arm_command.append("--flight-preflight")
                        if preset == "cartesian_demo":
                            cartesian_action_started[schedule_id] = time.monotonic()
                            arm_command = [
                                "ros2", "run", "drone_arm_sim", "cartesian_arm_demo",
                                "--distance", f"{cartesian_distance_m:.6f}",
                                "--step", "0.005",
                                "--duration", arm_duration,
                                "--hold", f"{cartesian_hold_s:.6f}",
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
                        elif preset == "directional_workspace_sequence":
                            directional_event_file.unlink(missing_ok=True)
                            arm_command = [
                                sys.executable,
                                str(
                                    Path(__file__).resolve().parent
                                    / "directional_workspace_flight_sequence.py"
                                ),
                                "--plan", str(directional_plan_path),
                                "--event-file", str(directional_event_file),
                                "--tolerance", "0.06",
                                "--stability-timeout", "35",
                            ]
                            if directional_order != DIRECTIONAL_ORDER:
                                arm_command.extend(
                                    ["--directions", ",".join(directional_order)]
                                )
                        arm_process = subprocess.Popen(
                            cpu_role_command("arm", arm_command),
                            stdout=subprocess.PIPE,
                            stderr=subprocess.STDOUT,
                            text=True,
                            env=arm_environment,
                        )
                        arm_action_started[schedule_id] = time.monotonic()
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
                elif profile == "directional_workspace_4kg":
                    latest = states[-1] if states else None
                    sequence_complete = (
                        f"DIRECTIONAL_SEQUENCE_COMPLETE directions={len(directional_order)}"
                        in directional_events_seen
                    )
                    final_stable_now = (
                        sequence_complete
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
            if landing_disarmed_time is None:
                controller_timed_out = True
                print("ARM_FLIGHT_CONTROLLER_TIMEOUT", flush=True)
            else:
                # PX4 has acknowledged LAND and the controller has observed
                # the disarmed state.  A lingering `ros2 run` wrapper is a
                # teardown artifact, not a flight-controller timeout; the
                # finally block below still terminates it deterministically.
                print("ARM_FLIGHT_WRAPPER_STALE_AFTER_DISARM", flush=True)
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
    if prehover_disarm:
        missing.append("prehover-disarm")
    if profile in {"full_a", "full_a_slow"}:
        required_presets = ("work_a", "retracted")
    elif profile == "full_b_slow":
        required_presets = ("work_b", "retracted")
    elif profile == "full_ab_slow":
        required_presets = ("work_a", "work_b", "retracted")
    elif profile in {"micro", "micro_4kg"}:
        required_presets = ("flight_micro_a", "flight_micro_b", "retracted")
    elif profile == "gripper_4kg":
        required_presets = ("gripper_open", "gripper_closed")
    elif profile == "wrist_roll_4kg":
        required_presets = ("wrist_roll_test", "wrist_roll_home")
    elif profile == "shoulder_pan_4kg":
        required_presets = ("shoulder_pan_slow_test", "shoulder_pan_home")
    elif profile == "multi_joint_slow_4kg":
        required_presets = ("flight_work_a", "retracted")
    elif profile == "full_extend_slow_4kg":
        required_presets = ("demo_extended", "retracted")
    elif profile == "nose_forward_straight_4kg":
        required_presets = ("flight_straight_forward", "retracted")
    elif profile == "directional_workspace_4kg":
        required_presets = ("directional_workspace_sequence",)
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
            if preset.startswith("cartesian_demo")
            else (
                "DIRECTIONAL_SEQUENCE_COMPLETE"
                if preset == "directional_workspace_sequence"
                else "ARM_PRESET_REACHED"
            )
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
    climbed = climbed_relative_to_takeoff_ground(states)
    no_failsafe = re.search(
        r"(?<![A-Za-z0-9_])failsafe=True\b", output
    ) is None
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
    if profile == "directional_workspace_4kg":
        for direction in directional_order:
            for phase in ("extend", "hold", "retract"):
                interval = directional_stage_intervals.get((direction, phase))
                if interval is None:
                    missing.append(f"directional-stage:{direction}:{phase}")
                    continue
                action_start, action_end = interval
                label = f"{direction}:{phase}"
                action_intervals.append((label, action_start, action_end))
                arm_window.extend(
                    state for state in states
                    if action_start <= state[0] <= action_end
                )
        for direction in directional_order:
            marker = f"DIRECTIONAL_DIRECTION_COMPLETE direction={direction}"
            if marker not in directional_events_seen:
                missing.append(f"directional-return:{direction}")
    elif cartesian_labels:
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
    elif profile in {
        "gripper_4kg", "wrist_roll_4kg", "shoulder_pan_4kg",
        "multi_joint_slow_4kg", "full_extend_slow_4kg",
        "nose_forward_straight_4kg",
    }:
        for label in required_presets:
            action_start = arm_action_started.get(label)
            action_end = arm_action_completed.get(label)
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
        # Isolated gripper/wrist profiles do not have Cartesian labels.  The
        # old comparison against cartesian_labels therefore left both values
        # at infinity even when every scheduled arm interval was measured.
        if len(cycle_drifts) == len(action_intervals):
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
            if any(
                start <= sample["timestamp"] <= end
                for _, start, end in action_intervals
            )
        ]
    elif flight_ready_time is not None:
        diagnostic_window = [
            sample for sample in diagnostics
            if 0.0 <= sample["timestamp"] - flight_ready_time <= land_after_s
        ]
    max_arm_torque = max(
        (sample["reaction_torque_norm_nm"] for sample in diagnostic_window),
        default=float("inf"),
    )
    max_arm_force = max(
        (sample["reaction_force_norm_n"] for sample in diagnostic_window),
        default=float("inf"),
    )
    max_com_shift = max(
        (sample["com_shift_norm_m"] for sample in diagnostic_window),
        default=float("inf"),
    )
    motor_samples = [
        sample["motors"] for sample in diagnostic_window
        if len(sample["motors"]) >= 8
    ]
    saturation_samples = sum(
        1
        for motors in motor_samples
        if any(value >= rated_motor_output for value in motors[:8])
    )
    saturation_rate = (
        saturation_samples / len(motor_samples) if motor_samples else float("inf")
    )
    inertia_reference = None
    if action_intervals:
        first_action_start = min(start for _, start, _ in action_intervals)
        baseline_samples = [
            sample for sample in diagnostics
            if sample["timestamp"] < first_action_start
        ]
        if baseline_samples:
            inertia_reference = baseline_samples[-1]["inertia_diag_kg_m2"]
    elif diagnostics:
        inertia_reference = diagnostics[0]["inertia_diag_kg_m2"]
    max_inertia_diag_change = float("inf")
    if inertia_reference is not None and diagnostic_window:
        max_inertia_diag_change = max(
            math.sqrt(sum(
                (value - reference) ** 2
                for value, reference in zip(
                    sample["inertia_diag_kg_m2"], inertia_reference
                )
            ))
            for sample in diagnostic_window
        )
    if action_intervals:
        truth_attitude_window = [
            sample for sample in truth_attitudes
            if any(start <= sample[0] <= end for _, start, end in action_intervals)
        ]
    elif flight_ready_time is not None:
        truth_attitude_window = [
            sample for sample in truth_attitudes
            if 0.0 <= sample[0] - flight_ready_time <= land_after_s
        ]
    else:
        truth_attitude_window = []
    max_truth_tilt_deg = max(
        (math.hypot(sample[1], sample[2]) for sample in truth_attitude_window),
        default=float("inf"),
    )
    truth_tilt_deg = [
        math.hypot(sample[1], sample[2]) for sample in truth_attitude_window
    ]
    rms_truth_tilt_deg = (
        math.sqrt(sum(value * value for value in truth_tilt_deg) / len(truth_tilt_deg))
        if truth_tilt_deg
        else float("inf")
    )
    if action_intervals:
        truth_position_window = [
            sample for sample in truth_positions
            if any(start <= sample[0] <= end for _, start, end in action_intervals)
        ]
    elif flight_ready_time is not None:
        truth_position_window = [
            sample for sample in truth_positions
            if 0.0 <= sample[0] - flight_ready_time <= land_after_s
        ]
    else:
        truth_position_window = []
    if truth_position_window:
        truth_x = [sample[1] for sample in truth_position_window]
        truth_y = [sample[2] for sample in truth_position_window]
        truth_z = [sample[3] for sample in truth_position_window]
        truth_x_peak_to_peak_m = max(truth_x) - min(truth_x)
        truth_y_peak_to_peak_m = max(truth_y) - min(truth_y)
        truth_xy_peak_to_peak_m = max(
            truth_x_peak_to_peak_m, truth_y_peak_to_peak_m
        )
        truth_altitude_peak_to_peak_m = max(truth_z) - min(truth_z)
    else:
        truth_x_peak_to_peak_m = float("inf")
        truth_y_peak_to_peak_m = float("inf")
        truth_xy_peak_to_peak_m = float("inf")
        truth_altitude_peak_to_peak_m = float("inf")
    directional_stages_stable = True
    directional_stage_results = {}
    if profile == "directional_workspace_4kg":
        stage_horizontal_limit = float(
            os.environ.get("ARM_FLIGHT_ACCEPT_HORIZONTAL_M", "0.05")
        )
        stage_altitude_limit = float(
            os.environ.get("ARM_FLIGHT_ACCEPT_ALTITUDE_M", "0.05")
        )
        stage_tilt_limit = float(
            os.environ.get("ARM_FLIGHT_ACCEPT_TILT_DEG", "1.0")
        )
        for direction in directional_order:
            direction_pass = True
            for phase in ("extend", "hold", "retract"):
                interval = directional_stage_intervals.get((direction, phase))
                if interval is None:
                    direction_pass = False
                    continue
                interval_start, interval_end = interval
                positions = [
                    sample for sample in truth_positions
                    if interval_start <= sample[0] <= interval_end
                ]
                attitudes = [
                    sample for sample in truth_attitudes
                    if interval_start <= sample[0] <= interval_end
                ]
                stage_diagnostics = [
                    sample for sample in diagnostics
                    if interval_start <= sample["timestamp"] <= interval_end
                ]
                if positions:
                    x_span = max(item[1] for item in positions) - min(item[1] for item in positions)
                    y_span = max(item[2] for item in positions) - min(item[2] for item in positions)
                    xy_span = max(x_span, y_span)
                    altitude_stage_span = (
                        max(item[3] for item in positions)
                        - min(item[3] for item in positions)
                    )
                else:
                    xy_span = altitude_stage_span = float("inf")
                stage_tilt = max(
                    (math.hypot(item[1], item[2]) for item in attitudes),
                    default=float("inf"),
                )
                stage_motor_samples = [
                    item["motors"] for item in stage_diagnostics
                    if len(item["motors"]) >= 8
                ]
                stage_saturation = sum(
                    1 for motors in stage_motor_samples
                    if any(value >= rated_motor_output for value in motors[:8])
                )
                stage_pass = (
                    xy_span <= stage_horizontal_limit
                    and altitude_stage_span <= stage_altitude_limit
                    and stage_tilt <= stage_tilt_limit
                    and stage_saturation == 0
                    and bool(stage_motor_samples)
                    and bool(attitudes)
                    and bool(positions)
                    and no_failsafe
                )
                direction_pass = direction_pass and stage_pass
                directional_stage_results[(direction, phase)] = stage_pass
                print(
                    "DIRECTIONAL_FLIGHT_STAGE_METRICS "
                    f"direction={direction} phase={phase} "
                    f"xy_peak_to_peak_m={xy_span:.3f} "
                    f"altitude_peak_to_peak_m={altitude_stage_span:.3f} "
                    f"max_truth_tilt_deg={stage_tilt:.3f} "
                    f"motor_saturation_samples={stage_saturation}/{len(stage_motor_samples)} "
                    f"no_failsafe={no_failsafe} pass={stage_pass}"
                )
            returned = (
                f"DIRECTIONAL_DIRECTION_COMPLETE direction={direction}"
                in directional_events_seen
            )
            direction_pass = direction_pass and returned
            print(
                "DIRECTIONAL_FLIGHT_DIRECTION_RESULT "
                f"direction={direction} returned={returned} pass={direction_pass}"
            )
            directional_stages_stable = directional_stages_stable and direction_pass
    stable = max_horizontal_drift < 1.5 and altitude_span < 1.5
    if profile in {
        "cartesian_demo_4kg", "cartesian_demo_twice_4kg",
        "cartesian_velocity_4kg", "cartesian_formal_7p735", "gripper_4kg",
        "wrist_roll_4kg", "shoulder_pan_4kg", "multi_joint_slow_4kg",
        "full_extend_slow_4kg", "nose_forward_straight_4kg",
        "directional_workspace_4kg",
    }:
        acceptance_horizontal_m = float(
            os.environ.get("ARM_FLIGHT_ACCEPT_HORIZONTAL_M", "0.15")
        )
        acceptance_altitude_m = float(
            os.environ.get("ARM_FLIGHT_ACCEPT_ALTITUDE_M", "0.30")
        )
        acceptance_tilt_deg = float(
            os.environ.get("ARM_FLIGHT_ACCEPT_TILT_DEG", "3.0")
        )
        stable = (
            truth_xy_peak_to_peak_m <= acceptance_horizontal_m
            and truth_altitude_peak_to_peak_m <= acceptance_altitude_m
            and max_arm_torque < 0.50
            and saturation_samples == 0
            and max_truth_tilt_deg <= acceptance_tilt_deg
            and math.isfinite(max_arm_force)
            and math.isfinite(max_com_shift)
            and math.isfinite(max_inertia_diag_change)
            and len(diagnostic_window) > 0
        )
        if profile == "directional_workspace_4kg":
            stable = stable and directional_stages_stable
    print(
        "ARM_FLIGHT_METRICS "
        f"horizontal_drift_m={max_horizontal_drift:.3f} "
        f"altitude_span_m={altitude_span:.3f} "
        f"max_com_shift_m={max_com_shift:.6f} "
        f"max_inertia_diag_change_kg_m2={max_inertia_diag_change:.9f} "
        f"max_arm_force_n={max_arm_force:.6f} "
        f"max_arm_torque_nm={max_arm_torque:.3f} "
        f"motor_saturation_samples={saturation_samples}/{len(motor_samples)} "
        f"motor_saturation_rate={saturation_rate:.6f} "
        f"rated_motor_output={rated_motor_output:.3f} "
        f"max_truth_tilt_deg={max_truth_tilt_deg:.3f} "
        f"rms_truth_tilt_deg={rms_truth_tilt_deg:.3f} "
        f"samples={len(arm_window)}"
    )
    print(
        "ARM_FLIGHT_TRUTH_METRICS "
        f"x_peak_to_peak_m={truth_x_peak_to_peak_m:.3f} "
        f"y_peak_to_peak_m={truth_y_peak_to_peak_m:.3f} "
        f"xy_peak_to_peak_m={truth_xy_peak_to_peak_m:.3f} "
        f"altitude_peak_to_peak_m={truth_altitude_peak_to_peak_m:.3f}"
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
