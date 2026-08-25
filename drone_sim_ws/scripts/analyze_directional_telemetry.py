#!/usr/bin/env python3
"""Apply strict per-stage gates to high-rate directional flight telemetry."""

from __future__ import annotations

import argparse
import json
import math
from pathlib import Path
import re

import numpy as np


INTERVAL_RE = re.compile(
    r"DIRECTIONAL_STAGE_INTERVAL direction=(\S+) phase=(\S+) "
    r"start=([0-9.]+) end=([0-9.]+)"
)

FORMAL_DIRECTIONS = (
    "front",
    "rear",
    "left",
    "right",
    "up",
    "down",
    "front_left",
    "front_right",
    "rear_left",
    "rear_right",
)
FORMAL_PHASES = ("extend", "hold", "retract")
CONTROLLER_INTENT_SCHEMA = "my_drone.arm-direct-xy-controller-intent.v1"
CONTROLLER_INTENT_MAXIMUM_GAP_S = 0.250


def quaternion_tilt_deg(quaternion_xyzw: list[float]) -> float:
    x, y, z, w = (float(value) for value in quaternion_xyzw)
    norm = math.sqrt(x * x + y * y + z * z + w * w)
    if norm <= 1.0e-12:
        return float("inf")
    x, y, z, w = x / norm, y / norm, z / norm, w / norm
    roll = math.atan2(2.0 * (w * x + y * z), 1.0 - 2.0 * (x * x + y * y))
    pitch = math.asin(max(-1.0, min(1.0, 2.0 * (w * y - z * x))))
    return math.degrees(math.hypot(roll, pitch))


def load_interval_records(
    path: Path,
) -> list[tuple[str, str, float, float]]:
    records = []
    for line in path.read_text(encoding="utf-8").splitlines():
        match = INTERVAL_RE.search(line)
        if match:
            records.append(
                (
                    match.group(1),
                    match.group(2),
                    float(match.group(3)),
                    float(match.group(4)),
                )
            )
    return records


def load_intervals(path: Path) -> dict[tuple[str, str], tuple[float, float]]:
    intervals = {}
    for direction, phase, start, end in load_interval_records(path):
        intervals[(direction, phase)] = (start, end)
    return intervals


def load_samples(path: Path) -> list[dict]:
    samples = []
    with path.open(encoding="utf-8") as stream:
        for line_number, line in enumerate(stream, 1):
            if not line.strip():
                continue
            try:
                samples.append(json.loads(line))
            except json.JSONDecodeError as error:
                raise RuntimeError(
                    f"invalid telemetry JSON at line {line_number}: {error}"
                ) from error
    return samples


def in_interval(sample: dict, start: float, end: float) -> bool:
    stamp = float(sample.get("recorder_monotonic_s", -1.0))
    return start <= stamp <= end


def compensation_continuity(reallocator: list[dict]) -> dict:
    failures = 0
    slew_violations = 0
    maximum_gap_s = 0.0
    previous = None
    for sample in reallocator:
        state = sample["state"]
        requested = np.asarray(
            state["requested_compensation_wrench_frd"], dtype=float
        )
        delivered = np.asarray(
            state["delivered_compensation_wrench_frd"], dtype=float
        )
        if (
            state.get("event") == "allocation_failure"
            or float(state.get("feasibility_scale", 0.0)) <= 0.0
            and np.linalg.norm(requested) > 1.0e-4
            or np.linalg.norm(requested) > 1.0e-4
            and np.linalg.norm(delivered) < 0.01 * np.linalg.norm(requested)
        ):
            failures += 1
        if previous is not None:
            dt = float(sample["recorder_monotonic_s"]) - float(
                previous["recorder_monotonic_s"]
            )
            maximum_gap_s = max(maximum_gap_s, dt)
            if dt > 0.0:
                prior = np.asarray(
                    previous["state"]["delivered_compensation_wrench_frd"],
                    dtype=float,
                )
                change = np.abs(delivered - prior)
                # Configured slew limits are 1 N/s and 0.1 N m/s.  Small
                # tolerances cover DDS timestamp jitter and float roundoff.
                bounds = np.asarray([1.0, 1.0, 1.0, 0.1, 0.1, 0.1]) * dt
                if np.any(change > bounds + 0.005):
                    slew_violations += 1
        previous = sample
    return {
        "allocation_failure_samples": failures,
        "slew_violation_samples": slew_violations,
        "maximum_sample_gap_s": maximum_gap_s,
    }


def _finite_vector(values, length: int) -> list[bool]:
    try:
        array = np.asarray(values, dtype=float)
    except (TypeError, ValueError):
        return [False] * length
    if array.shape != (length,):
        return [False] * length
    return [bool(np.isfinite(value)) for value in array]


def mixed_axis_setpoint_is_valid(sample: dict) -> bool:
    """True only for PX4 XY-release / Z-velocity mixed-axis semantics."""
    position = _finite_vector(sample.get("position"), 3)
    velocity = _finite_vector(sample.get("velocity"), 3)
    acceleration = _finite_vector(sample.get("acceleration"), 3)
    jerk = _finite_vector(sample.get("jerk"), 3)
    try:
        acceleration_values = np.asarray(sample["acceleration"], dtype=float)
        yaw_finite = math.isfinite(float(sample["yaw"]))
        yawspeed_finite = math.isfinite(float(sample["yawspeed"]))
    except (KeyError, TypeError, ValueError):
        return False
    return bool(
        position == [False, False, False]
        and velocity == [False, False, True]
        and acceleration == [True, True, False]
        and np.allclose(acceleration_values[:2], 0.0, atol=1.0e-9)
        and jerk == [False, False, False]
        and yaw_finite
        and not yawspeed_finite
    )


def finite_px4_xy_setpoint_is_valid(sample: dict) -> bool:
    """Return whether PX4 has an unambiguous finite horizontal setpoint.

    PX4 may own XY through either its position loop or its velocity loop.  A
    partially finite horizontal pair is not accepted: it cannot prove that
    both horizontal axes have one well-defined owner.
    """
    position = _finite_vector(sample.get("position"), 3)
    velocity = _finite_vector(sample.get("velocity"), 3)
    return bool(
        position[:2] == [True, True]
        or velocity[:2] == [True, True]
    )


def direct_xy_controller_intent_evidence(
    window: list[dict],
    *,
    maximum_gap_s: float = CONTROLLER_INTENT_MAXIMUM_GAP_S,
) -> dict:
    """Audit the controller intent lease while one motion stage is active.

    Entry/exit levels may legitimately appear at stage boundaries.  The
    watchdog-relevant active interval therefore begins at the first requested
    level and ends at the last requested level.  Every level inside that
    interval must remain one internally consistent direct-XY request.
    """
    samples = sorted(
        (
            sample
            for sample in window
            if sample.get("kind") == "arm_direct_xy_controller_intent"
        ),
        key=lambda sample: (
            float(sample.get("recorder_monotonic_s", -1.0)),
            int(sample.get("sequence", 0)),
        ),
    )
    invalid_transport_samples = sum(
        1
        for sample in window
        if sample.get("kind") == "arm_direct_xy_controller_intent_invalid"
    )
    parsed = []
    malformed_samples = 0
    for sample in samples:
        value = sample.get("state")
        try:
            if not isinstance(value, dict):
                raise TypeError
            producer_stamp = float(value["monotonic_s"])
            session = str(value["controller_session_id"])
            epoch = int(value["ownership_epoch"])
            generation = int(value["intent_generation"])
            state = str(value["state"])
            requested = value["force_requested"]
            mixed = value["mixed_setpoint_active"]
            motion = value["motion_active"]
            if (
                value.get("schema") != CONTROLLER_INTENT_SCHEMA
                or not math.isfinite(producer_stamp)
                or not session
                or type(requested) is not bool
                or type(mixed) is not bool
                or type(motion) is not bool
            ):
                raise ValueError
        except (KeyError, TypeError, ValueError):
            malformed_samples += 1
            continue
        parsed.append(
            {
                "recorder_stamp": float(sample["recorder_monotonic_s"]),
                "producer_stamp": producer_stamp,
                "session": session,
                "epoch": epoch,
                "generation": generation,
                "state": state,
                "requested": requested,
                "mixed": mixed,
                "motion": motion,
            }
        )

    requested_indices = [
        index for index, item in enumerate(parsed) if item["requested"] is True
    ]
    active = (
        parsed[requested_indices[0] : requested_indices[-1] + 1]
        if requested_indices
        else []
    )
    active_stamps = [item["recorder_stamp"] for item in active]
    maximum_active_gap_s = (
        max(np.diff(active_stamps), default=float("inf"))
        if len(active_stamps) >= 2
        else float("inf")
    )
    effective_rate_hz = (
        (len(active_stamps) - 1) / (active_stamps[-1] - active_stamps[0])
        if len(active_stamps) >= 2 and active_stamps[-1] > active_stamps[0]
        else 0.0
    )
    inconsistent_active_samples = sum(
        1
        for item in active
        if not (
            item["requested"] is True
            and item["mixed"] is True
            and item["motion"] is True
            and item["state"] in {"handoff", "direct_xy"}
        )
    )
    session_changes = sum(
        current["session"] != previous["session"]
        for previous, current in zip(parsed, parsed[1:])
    )
    producer_time_regressions = sum(
        current["producer_stamp"] <= previous["producer_stamp"]
        for previous, current in zip(parsed, parsed[1:])
    )
    epoch_regressions = sum(
        current["epoch"] < previous["epoch"]
        for previous, current in zip(parsed, parsed[1:])
        if current["session"] == previous["session"]
    )
    generation_regressions = sum(
        current["generation"] < previous["generation"]
        for previous, current in zip(parsed, parsed[1:])
        if current["session"] == previous["session"]
    )
    report = {
        "intent_samples": len(samples),
        "active_intent_samples": len(active),
        "effective_active_rate_hz": float(effective_rate_hz),
        "maximum_active_gap_s": float(maximum_active_gap_s),
        "maximum_allowed_gap_s": float(maximum_gap_s),
        "invalid_transport_samples": invalid_transport_samples,
        "malformed_samples": malformed_samples,
        "inconsistent_active_samples": inconsistent_active_samples,
        "controller_session_changes": session_changes,
        "producer_time_regressions": producer_time_regressions,
        "ownership_epoch_regressions": epoch_regressions,
        "intent_generation_regressions": generation_regressions,
        "controller_sessions": sorted({item["session"] for item in parsed}),
    }
    report["pass"] = bool(
        len(active) >= 2
        and maximum_active_gap_s < maximum_gap_s
        and invalid_transport_samples == 0
        and malformed_samples == 0
        and inconsistent_active_samples == 0
        and session_changes == 0
        and producer_time_regressions == 0
        and epoch_regressions == 0
        and generation_regressions == 0
    )
    return report


def velocity_offboard_mode_is_valid(sample: dict) -> bool:
    return bool(
        sample.get("velocity") is True
        and sample.get("position") is False
        and sample.get("acceleration") is False
        and sample.get("attitude") is False
        and sample.get("body_rate") is False
        and sample.get("thrust_and_torque") is False
        and sample.get("direct_actuator") is False
    )


def _latest_before(samples: list[dict], stamp: float) -> dict | None:
    latest = None
    for sample in samples:
        sample_stamp = float(sample.get("recorder_monotonic_s", -1.0))
        if sample_stamp > stamp:
            break
        latest = sample
    return latest


def direct_xy_handoff_evidence(
    samples: list[dict], *, maximum_transition_lag_s: float = 0.10
) -> dict:
    """Prove the ordered PX4/direct-force XY ownership hand-off.

    Entry must publish the mixed-axis PX4 setpoint before enabling direct XY
    force.  Exit must restore a finite PX4 XY setpoint before disabling direct
    XY force.  The short interval between the two independently recorded DDS
    samples is treated as one hand-off corridor and reported as latency; it is
    not evidence of a second steady-state owner.  Dual-owner or unowned states
    anywhere outside such a correctly ordered corridor remain hard failures.
    """
    events = []
    invalid_setpoint_samples = 0
    for sample in samples:
        kind = sample.get("kind")
        stamp = float(sample.get("recorder_monotonic_s", -1.0))
        sequence = int(sample.get("sequence", 0))
        if kind == "px4_trajectory_setpoint_input":
            if mixed_axis_setpoint_is_valid(sample):
                value = "mixed"
            elif finite_px4_xy_setpoint_is_valid(sample):
                value = "finite"
            else:
                invalid_setpoint_samples += 1
                value = "invalid"
            events.append((stamp, sequence, "setpoint", value))
        elif kind == "reallocator":
            active = sample.get("state", {}).get("position_feedback_active") is True
            events.append((stamp, sequence, "force", bool(active)))
    events.sort(key=lambda item: (item[0], item[1]))

    setpoint = None
    force_active = None
    phase = "initial"
    pending_stamp = None
    entry_handoffs = []
    exit_handoffs = []
    entry_force_before_mixed = 0
    exit_force_off_before_finite = 0
    dual_owner_outside_handoff = 0
    unowned_xy_outside_handoff = 0
    initial_state_invalid = False

    def stable_anomaly() -> None:
        nonlocal dual_owner_outside_handoff, unowned_xy_outside_handoff
        if setpoint == "finite" and force_active is True:
            dual_owner_outside_handoff += 1
        elif setpoint == "mixed" and force_active is False:
            unowned_xy_outside_handoff += 1

    for stamp, _sequence, kind, value in events:
        changed = False
        if kind == "setpoint":
            if value == "invalid":
                continue
            changed = value != setpoint
            setpoint = value
        else:
            changed = value != force_active
            force_active = value
        if setpoint is None or force_active is None:
            continue

        if phase == "initial":
            if setpoint == "finite" and force_active is False:
                phase = "px4"
            else:
                initial_state_invalid = True
                stable_anomaly()
            continue
        if not changed:
            if phase in {"px4", "direct"}:
                stable_anomaly()
            continue

        if phase == "px4":
            if kind == "setpoint" and value == "mixed" and force_active is False:
                phase = "entering"
                pending_stamp = stamp
            elif kind == "force" and value is True and setpoint == "finite":
                entry_force_before_mixed += 1
                phase = "entry_wrong_order"
                pending_stamp = stamp
            else:
                stable_anomaly()
        elif phase == "entering":
            if kind == "force" and value is True:
                entry_handoffs.append(
                    {
                        "mixed_setpoint_s": float(pending_stamp),
                        "direct_force_on_s": stamp,
                        "lag_s": stamp - float(pending_stamp),
                    }
                )
                phase = "direct"
                pending_stamp = None
            elif kind == "setpoint" and value == "finite":
                unowned_xy_outside_handoff += 1
                phase = "px4"
                pending_stamp = None
        elif phase == "entry_wrong_order":
            if kind == "setpoint" and value == "mixed":
                dual_owner_outside_handoff += 1
                phase = "direct"
                pending_stamp = None
        elif phase == "direct":
            if kind == "setpoint" and value == "finite" and force_active is True:
                phase = "exiting"
                pending_stamp = stamp
            elif kind == "force" and value is False and setpoint == "mixed":
                exit_force_off_before_finite += 1
                phase = "exit_wrong_order"
                pending_stamp = stamp
            else:
                stable_anomaly()
        elif phase == "exiting":
            if kind == "force" and value is False:
                exit_handoffs.append(
                    {
                        "finite_setpoint_s": float(pending_stamp),
                        "direct_force_off_s": stamp,
                        "lag_s": stamp - float(pending_stamp),
                    }
                )
                phase = "px4"
                pending_stamp = None
            elif kind == "setpoint" and value == "mixed":
                dual_owner_outside_handoff += 1
                phase = "direct"
                pending_stamp = None
        elif phase == "exit_wrong_order":
            if kind == "setpoint" and value == "finite":
                unowned_xy_outside_handoff += 1
                phase = "px4"
                pending_stamp = None

    incomplete_handoff = phase in {
        "entering",
        "exiting",
        "entry_wrong_order",
        "exit_wrong_order",
    }
    entry_lags = [item["lag_s"] for item in entry_handoffs]
    exit_lags = [item["lag_s"] for item in exit_handoffs]
    maximum_entry_lag = max(entry_lags, default=0.0)
    maximum_exit_lag = max(exit_lags, default=0.0)
    report = {
        "entry_handoffs": entry_handoffs,
        "exit_handoffs": exit_handoffs,
        "entry_count": len(entry_handoffs),
        "exit_count": len(exit_handoffs),
        "maximum_entry_lag_s": float(maximum_entry_lag),
        "maximum_exit_lag_s": float(maximum_exit_lag),
        "maximum_transition_lag_s": float(maximum_transition_lag_s),
        "entry_force_before_mixed_count": entry_force_before_mixed,
        "exit_force_off_before_finite_count": exit_force_off_before_finite,
        "dual_owner_outside_handoff_samples": dual_owner_outside_handoff,
        "unowned_xy_outside_handoff_samples": unowned_xy_outside_handoff,
        "invalid_setpoint_samples": invalid_setpoint_samples,
        "initial_state_invalid": initial_state_invalid,
        "incomplete_handoff": incomplete_handoff,
        "final_phase": phase,
    }
    report["pass"] = bool(
        len(entry_handoffs) > 0
        and len(entry_handoffs) == len(exit_handoffs)
        and maximum_entry_lag <= maximum_transition_lag_s
        and maximum_exit_lag <= maximum_transition_lag_s
        and entry_force_before_mixed == 0
        and exit_force_off_before_finite == 0
        and dual_owner_outside_handoff == 0
        and unowned_xy_outside_handoff == 0
        and invalid_setpoint_samples == 0
        and not initial_state_invalid
        and not incomplete_handoff
        and phase == "px4"
    )
    return report


def direct_xy_ownership_evidence(
    window: list[dict],
    start: float,
    end: float,
    *,
    validated_handoffs: dict | None = None,
    maximum_sample_gap_s: float = 0.10,
) -> dict:
    """Prove continuous, unique XY ownership within one motion stage.

    ``validated_handoffs`` is the independently reconstructed result from
    :func:`direct_xy_handoff_evidence`.  Only samples inside one of its
    correctly ordered, bounded entry/exit corridors are exempted from the
    steady-state dual/unowned counters.  This does not exempt setpoint gaps,
    extend the 100 ms corridor, or accept a corridor inferred from this stage
    window alone.
    """
    maximum_sample_gap_s = max(0.0, float(maximum_sample_gap_s))
    # A repeated setpoint is valid for no longer than the publisher's fixed
    # 180 ms source lease, even when the wider evidence profile permits a
    # larger generic sample gap.  OffboardControlMode is a level/heartbeat,
    # not a message that PX4 requires to share every TrajectorySetpoint
    # timestamp.
    setpoint_gap_limit_s = min(maximum_sample_gap_s, 0.18)
    setpoints = sorted(
        (
            sample
            for sample in window
            if sample.get("kind") == "px4_trajectory_setpoint_input"
            and start
            <= float(sample.get("recorder_monotonic_s", -1.0))
            <= end
        ),
        key=lambda sample: float(sample["recorder_monotonic_s"]),
    )
    modes = sorted(
        (
            sample
            for sample in window
            if sample.get("kind") == "px4_offboard_control_mode_input"
        ),
        key=lambda sample: float(sample["recorder_monotonic_s"]),
    )
    reallocators = sorted(
        (sample for sample in window if sample.get("kind") == "reallocator"),
        key=lambda sample: float(sample["recorder_monotonic_s"]),
    )
    owner_states = sorted(
        (
            sample
            for sample in window
            if sample.get("kind") == "arm_direct_xy_state"
        ),
        key=lambda sample: float(sample["recorder_monotonic_s"]),
    )
    inhibits = [
        sample
        for sample in window
        if sample.get("kind") == "arm_motion_inhibit"
        and bool(sample.get("inhibit"))
        and start
        <= float(sample.get("recorder_monotonic_s", -1.0))
        <= end
    ]
    mode_by_timestamp = {
        int(sample.get("px4_timestamp_us", -1)): sample for sample in modes
    }

    valid_samples = []
    continuous_owner_samples = []
    dual_owner_samples = 0
    unowned_samples = 0
    validated_corridor_samples = 0
    paired_samples = 0
    valid_mode_pairs = 0
    fresh_velocity_mode_samples = 0
    mode_ages_s = []
    handoffs = validated_handoffs or {}
    entry_corridors = [
        (
            float(item["mixed_setpoint_s"]),
            float(item["direct_force_on_s"]),
        )
        for item in handoffs.get("entry_handoffs", [])
        if 0.0 <= float(item.get("lag_s", float("inf"))) <= 0.10
    ]
    exit_corridors = [
        (
            float(item["finite_setpoint_s"]),
            float(item["direct_force_off_s"]),
        )
        for item in handoffs.get("exit_handoffs", [])
        if 0.0 <= float(item.get("lag_s", float("inf"))) <= 0.10
    ]

    def inside(stamp: float, corridors: list[tuple[float, float]]) -> bool:
        return any(begin <= stamp <= finish for begin, finish in corridors)

    for setpoint in setpoints:
        stamp = float(setpoint["recorder_monotonic_s"])
        mixed = mixed_axis_setpoint_is_valid(setpoint)
        exact_mode = mode_by_timestamp.get(
            int(setpoint.get("px4_timestamp_us", -1))
        )
        paired = exact_mode is not None
        paired_samples += int(paired)
        valid_mode_pairs += int(
            paired and velocity_offboard_mode_is_valid(exact_mode)
        )
        # The 50 Hz setpoint keepalive intentionally refreshes only the
        # TrajectorySetpoint timestamp; the authoritative flight loop publishes
        # OffboardControlMode at 20 Hz.  Associate each setpoint with the most
        # recent non-future mode level, while rejecting a stale level using the
        # declared evidence sample-gap budget.
        mode = _latest_before(modes, stamp)
        mode_age_s = (
            stamp - float(mode["recorder_monotonic_s"])
            if mode is not None
            else float("inf")
        )
        mode_fresh = bool(
            mode is not None
            and 0.0 <= mode_age_s <= maximum_sample_gap_s
        )
        mode_valid = bool(mode_fresh and velocity_offboard_mode_is_valid(mode))
        fresh_velocity_mode_samples += int(mode_valid)
        mode_ages_s.append(mode_age_s)
        reallocator = _latest_before(reallocators, stamp)
        reallocator_fresh = bool(
            reallocator is not None
            and 0.0
            <= stamp - float(reallocator["recorder_monotonic_s"])
            < 0.08
        )
        reallocator_state = {} if reallocator is None else reallocator.get("state", {})
        position_explicitly_inactive = bool(
            reallocator_fresh
            and reallocator_state.get("position_feedback_active") is False
        )
        position_active = bool(
            reallocator_fresh
            and reallocator_state.get("position_feedback_enabled") is True
            and reallocator_state.get("position_feedback_active") is True
            and reallocator_state.get("position_target_latched") is True
            and reallocator_state.get("truth_fresh") is True
            and reallocator_state.get("allocation_limited") is False
        )
        owner = _latest_before(owner_states, stamp)
        owner_fresh = bool(
            owner is not None
            and 0.0 <= stamp - float(owner["recorder_monotonic_s"]) < 0.08
        )
        owner_state = {} if owner is None else owner.get("state", {})
        direct_owner = bool(
            owner_fresh
            and owner_state.get("state") == "direct_xy"
            and owner_state.get("position_feedback_active") is True
            and owner_state.get("reallocator_fresh") is True
        )
        finite_px4 = finite_px4_xy_setpoint_is_valid(setpoint)
        in_entry_corridor = bool(
            mixed
            and position_explicitly_inactive
            and inside(stamp, entry_corridors)
        )
        in_exit_corridor = bool(
            finite_px4 and position_active and inside(stamp, exit_corridors)
        )
        in_validated_corridor = in_entry_corridor or in_exit_corridor
        validated_corridor_samples += int(in_validated_corridor)
        if position_active and not mixed and not in_exit_corridor:
            dual_owner_samples += 1
        if mixed and not position_active and not in_entry_corridor:
            unowned_samples += 1
        direct_valid = bool(mixed and position_active and direct_owner and mode_valid)
        # Before a validated entry (or after a validated exit), finite PX4 XY
        # with the direct force physically inactive is also a unique owner.
        px4_valid = bool(
            finite_px4 and position_explicitly_inactive and mode_valid
        )
        valid_samples.append(direct_valid)
        continuous_owner_samples.append(
            bool(direct_valid or px4_valid or (in_validated_corridor and mode_valid))
        )

    setpoint_stamps = [
        float(sample["recorder_monotonic_s"]) for sample in setpoints
    ]
    maximum_setpoint_gap_s = (
        max(np.diff(setpoint_stamps), default=float("inf"))
        if len(setpoint_stamps) >= 2
        else float("inf")
    )
    # Include the stage boundaries so a delayed acquire/release is measured,
    # not hidden by looking only at received setpoint samples.
    maximum_invalid_window_s = 0.0
    invalid_start = start
    for stamp, valid in zip(setpoint_stamps, continuous_owner_samples):
        if valid:
            if invalid_start is not None:
                maximum_invalid_window_s = max(
                    maximum_invalid_window_s, stamp - invalid_start
                )
                invalid_start = None
        elif invalid_start is None:
            invalid_start = stamp
    if invalid_start is not None:
        maximum_invalid_window_s = max(maximum_invalid_window_s, end - invalid_start)
    elif setpoint_stamps:
        maximum_invalid_window_s = max(
            maximum_invalid_window_s, end - setpoint_stamps[-1]
        )

    count = len(setpoints)
    coverage = sum(valid_samples) / count if count else 0.0
    continuous_coverage = (
        sum(continuous_owner_samples) / count if count else 0.0
    )
    paired_rate = paired_samples / count if count else 0.0
    valid_mode_rate = valid_mode_pairs / count if count else 0.0
    fresh_mode_rate = (
        fresh_velocity_mode_samples / count if count else 0.0
    )
    report = {
        "setpoint_samples": count,
        "valid_unique_owner_samples": sum(valid_samples),
        "direct_owner_coverage": coverage,
        "continuous_unique_owner_coverage": continuous_coverage,
        "mode_setpoint_timestamp_pair_rate": paired_rate,
        "valid_velocity_mode_pair_rate": valid_mode_rate,
        "fresh_velocity_mode_coverage": fresh_mode_rate,
        "maximum_velocity_mode_age_s": float(
            max(mode_ages_s, default=float("inf"))
        ),
        "velocity_mode_freshness_limit_s": maximum_sample_gap_s,
        "setpoint_gap_limit_s": setpoint_gap_limit_s,
        "maximum_setpoint_gap_s": float(maximum_setpoint_gap_s),
        "maximum_ownership_anomaly_window_s": float(maximum_invalid_window_s),
        "dual_owner_samples": dual_owner_samples,
        "unowned_xy_samples": unowned_samples,
        "validated_handoff_corridor_samples": validated_corridor_samples,
        "inhibit_true_samples": len(inhibits),
    }
    report["pass"] = bool(
        count > 0
        and coverage >= 0.99
        and continuous_coverage >= 0.99
        and fresh_mode_rate >= 0.99
        and maximum_setpoint_gap_s <= setpoint_gap_limit_s
        and maximum_invalid_window_s <= maximum_sample_gap_s
        and dual_owner_samples == 0
        and unowned_samples == 0
        and not inhibits
    )
    return report


def analyze(args) -> dict:
    interval_records = load_interval_records(args.events)
    intervals = {
        (direction, phase): (start, end)
        for direction, phase, start, end in interval_records
    }
    samples = load_samples(args.telemetry)
    config = json.loads(args.config.read_text(encoding="utf-8"))
    rated_output = 1000.0 * float(
        config["actuator_normalization"]["rated_thrust_command"]
    )
    observed_directions = []
    for direction, phase, _start, _end in interval_records:
        if phase == "extend" and direction not in observed_directions:
            observed_directions.append(direction)
    expected_argument = getattr(args, "expected_directions", None)
    if expected_argument is None:
        directions = observed_directions
    elif isinstance(expected_argument, str):
        directions = [
            item.strip() for item in expected_argument.split(",") if item.strip()
        ]
    else:
        directions = [str(item) for item in expected_argument]
    interval_keys = [(direction, phase) for direction, phase, _, _ in interval_records]
    duplicate_intervals = sorted(
        {key for key in interval_keys if interval_keys.count(key) > 1}
    )
    expected_interval_keys = [
        (direction, phase) for direction in directions for phase in FORMAL_PHASES
    ]
    unexpected_intervals = sorted(set(interval_keys) - set(expected_interval_keys))
    missing_intervals = sorted(set(expected_interval_keys) - set(interval_keys))
    observed_order_matches = observed_directions == directions
    interval_evidence_complete = bool(
        directions
        and not duplicate_intervals
        and not unexpected_intervals
        and not missing_intervals
        and observed_order_matches
        and len(interval_records) == len(expected_interval_keys)
    )
    any_failsafe = any(
        bool(sample.get("failsafe"))
        for sample in samples
        if sample.get("kind") == "vehicle_status"
    )
    stage_reports = []
    ownership_required = bool(
        getattr(args, "require_direct_xy_ownership", False)
    )
    handoff = (
        direct_xy_handoff_evidence(samples, maximum_transition_lag_s=0.10)
        if ownership_required
        else {"required": False, "pass": True}
    )
    all_pass = interval_evidence_complete and not any_failsafe and handoff["pass"]
    for direction in directions:
        for phase in ("extend", "hold", "retract"):
            interval = intervals.get((direction, phase))
            if interval is None:
                stage_reports.append(
                    {"direction": direction, "phase": phase, "pass": False}
                )
                all_pass = False
                continue
            start, end = interval
            window = [
                sample for sample in samples if in_interval(sample, start, end)
            ]
            odometry = [
                sample for sample in window if sample.get("kind") == "odometry"
            ]
            motors = [
                sample
                for sample in window
                if sample.get("kind") == "px4_actuator_outputs"
                and len(sample.get("outputs", [])) >= 8
            ]
            reallocator = [
                sample for sample in window if sample.get("kind") == "reallocator"
            ]
            if odometry:
                position = np.asarray(
                    [sample["position_world_enu_m"] for sample in odometry],
                    dtype=float,
                )
                xy_span = float(
                    max(np.ptp(position[:, 0]), np.ptp(position[:, 1]))
                )
                altitude_span = float(np.ptp(position[:, 2]))
                maximum_tilt = max(
                    quaternion_tilt_deg(
                        sample["quaternion_body_to_world_xyzw"]
                    )
                    for sample in odometry
                )
                stamps = np.asarray(
                    [sample["recorder_monotonic_s"] for sample in odometry],
                    dtype=float,
                )
                effective_rate_hz = (
                    float((len(stamps) - 1) / (stamps[-1] - stamps[0]))
                    if len(stamps) >= 2 and stamps[-1] > stamps[0]
                    else 0.0
                )
                maximum_odom_gap_s = (
                    float(np.max(np.diff(stamps))) if len(stamps) >= 2 else float("inf")
                )
            else:
                xy_span = altitude_span = maximum_tilt = float("inf")
                effective_rate_hz = 0.0
                maximum_odom_gap_s = float("inf")
            motor_saturation = sum(
                1
                for sample in motors
                if any(float(value) >= rated_output for value in sample["outputs"][:8])
            )
            allocator_saturation = sum(
                int(sample["state"].get("saturated", 0)) for sample in reallocator
            )
            maximum_residual = max(
                (
                    float(sample["state"].get("residual_norm", float("inf")))
                    for sample in reallocator
                ),
                default=float("inf"),
            )
            continuity = compensation_continuity(reallocator)
            ownership = (
                direct_xy_ownership_evidence(
                    [
                        sample
                        for sample in samples
                        if start - float(args.maximum_gap_s)
                        <= float(sample.get("recorder_monotonic_s", -1.0))
                        <= end
                    ],
                    start,
                    end,
                    validated_handoffs=handoff,
                    maximum_sample_gap_s=float(args.maximum_gap_s),
                )
                if ownership_required
                else {"required": False, "pass": True}
            )
            controller_intent = (
                direct_xy_controller_intent_evidence(window)
                if ownership_required
                else {"required": False, "pass": True}
            )
            report = {
                "direction": direction,
                "phase": phase,
                "duration_s": end - start,
                "odometry_samples": len(odometry),
                "effective_odometry_rate_hz": effective_rate_hz,
                "maximum_odometry_gap_s": maximum_odom_gap_s,
                "xy_peak_to_peak_m": xy_span,
                "altitude_peak_to_peak_m": altitude_span,
                "maximum_tilt_deg": maximum_tilt,
                "motor_samples": len(motors),
                "motor_saturation_samples": motor_saturation,
                "reallocator_samples": len(reallocator),
                "allocator_saturated_motor_count_sum": allocator_saturation,
                "maximum_allocation_residual_norm": maximum_residual,
                "compensation_continuity": continuity,
                "direct_xy_ownership": ownership,
                "direct_xy_controller_intent": controller_intent,
                "no_failsafe": not any_failsafe,
            }
            report["pass"] = bool(
                xy_span <= args.xy_limit
                and altitude_span <= args.z_limit
                and maximum_tilt <= args.tilt_limit
                and effective_rate_hz >= args.minimum_rate_hz
                and maximum_odom_gap_s <= args.maximum_gap_s
                and motor_saturation == 0
                and allocator_saturation == 0
                and maximum_residual <= args.residual_limit
                and len(motors) > 0
                and len(reallocator) > 0
                and continuity["allocation_failure_samples"] == 0
                and continuity["slew_violation_samples"] == 0
                and continuity["maximum_sample_gap_s"] <= args.maximum_gap_s
                and ownership["pass"]
                and controller_intent["pass"]
                and not any_failsafe
            )
            all_pass = all_pass and report["pass"]
            stage_reports.append(report)
    return {
        "schema": "my_drone.directional-high-rate-acceptance.v1",
        "telemetry": str(args.telemetry),
        "events": str(args.events),
        "directions": directions,
        "observed_directions": observed_directions,
        "interval_evidence": {
            "complete": interval_evidence_complete,
            "expected_stage_count": len(expected_interval_keys),
            "observed_stage_count": len(interval_records),
            "duplicate_intervals": [list(item) for item in duplicate_intervals],
            "missing_intervals": [list(item) for item in missing_intervals],
            "unexpected_intervals": [list(item) for item in unexpected_intervals],
            "observed_order_matches": observed_order_matches,
        },
        "any_failsafe": any_failsafe,
        "direct_xy_handoffs": handoff,
        "limits": {
            "xy_peak_to_peak_m": args.xy_limit,
            "altitude_peak_to_peak_m": args.z_limit,
            "maximum_tilt_deg": args.tilt_limit,
            "minimum_sample_rate_hz": args.minimum_rate_hz,
            "maximum_sample_gap_s": args.maximum_gap_s,
            "maximum_allocation_residual_norm": args.residual_limit,
            "maximum_controller_intent_gap_s": (
                CONTROLLER_INTENT_MAXIMUM_GAP_S
            ),
        },
        "stages": stage_reports,
        "pass": all_pass,
    }


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--telemetry", type=Path, required=True)
    parser.add_argument("--events", type=Path, required=True)
    parser.add_argument("--config", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--xy-limit", type=float, default=0.05)
    parser.add_argument("--z-limit", type=float, default=0.05)
    parser.add_argument("--tilt-limit", type=float, default=1.0)
    parser.add_argument("--minimum-rate-hz", type=float, default=50.0)
    parser.add_argument("--maximum-gap-s", type=float, default=0.10)
    parser.add_argument("--residual-limit", type=float, default=0.02)
    parser.add_argument(
        "--require-direct-xy-ownership",
        action="store_true",
        help="require unique mixed-axis PX4/direct-force XY ownership evidence",
    )
    parser.add_argument(
        "--expected-directions",
        default=",".join(FORMAL_DIRECTIONS),
        help=(
            "comma-separated directions that must appear exactly once and in "
            "this order; defaults to the complete ten-direction matrix"
        ),
    )
    args = parser.parse_args()
    report = analyze(args)
    args.output.write_text(
        json.dumps(report, indent=2, sort_keys=True), encoding="utf-8"
    )
    for stage in report["stages"]:
        print(
            "HIGH_RATE_STAGE_METRICS "
            f"direction={stage['direction']} phase={stage['phase']} "
            f"xy_m={stage.get('xy_peak_to_peak_m', float('inf')):.6f} "
            f"z_m={stage.get('altitude_peak_to_peak_m', float('inf')):.6f} "
            f"tilt_deg={stage.get('maximum_tilt_deg', float('inf')):.6f} "
            f"rate_hz={stage.get('effective_odometry_rate_hz', 0.0):.1f} "
            f"pass={stage['pass']}"
        )
    print(f"HIGH_RATE_DIRECTIONAL_ACCEPTANCE pass={report['pass']}")
    return 0 if report["pass"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
