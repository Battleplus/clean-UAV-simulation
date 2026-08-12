#!/usr/bin/env python3
"""Generate a machine-readable PX4 control-loop identification report.

The analyzer is deliberately read-only.  It uses the armed Offboard window in
one ULog and compares the setpoint and measured signals for the body-rate,
attitude and velocity layers.  A response is only called a step response when
the dominant command contains a long enough plateau; otherwise rise/settling
metrics are reported as ``null`` instead of manufacturing a tuning result from
a continuously changing outer-loop command.
"""

from __future__ import annotations

import argparse
import json
import math
from dataclasses import dataclass
from pathlib import Path
from typing import Any

import numpy as np


@dataclass(frozen=True)
class ResponseLimits:
    amplitude_floor: float
    plateau_fraction: float = 0.10
    plateau_min_s: float = 0.40
    settling_band_fraction: float = 0.10
    settling_hold_s: float = 0.30


def _finite_sorted(time_s, target, actual):
    time_s = np.asarray(time_s, dtype=float)
    target = np.asarray(target, dtype=float)
    actual = np.asarray(actual, dtype=float)
    valid = np.isfinite(time_s) & np.isfinite(target) & np.isfinite(actual)
    time_s, target, actual = time_s[valid], target[valid], actual[valid]
    if not len(time_s):
        return time_s, target, actual
    order = np.argsort(time_s)
    return time_s[order], target[order], actual[order]


def _first_held_index(mask: np.ndarray, time_s: np.ndarray, hold_s: float):
    start = None
    for index, is_inside in enumerate(mask):
        if is_inside and start is None:
            start = index
        elif not is_inside:
            start = None
        if start is not None and time_s[index] - time_s[start] >= hold_s:
            return start
    return None


def response_metrics(
    time_s,
    target,
    actual,
    limits: ResponseLimits,
) -> dict[str, Any]:
    """Measure the dominant signed step, rejecting non-step-like commands."""
    time_s, target, actual = _finite_sorted(time_s, target, actual)
    result: dict[str, Any] = {
        "sample_count": int(len(time_s)),
        "step_evaluable": False,
        "target_value": None,
        "actual_peak": None,
        "rise_time_s": None,
        "overshoot_fraction": None,
        "settling_time_s": None,
        "steady_error": None,
        "rms_tracking_error": None,
        "reason": "insufficient_samples",
    }
    if len(time_s) < 8 or time_s[-1] <= time_s[0]:
        return result

    duration_s = float(time_s[-1] - time_s[0])
    baseline_end = time_s[0] + min(0.50, 0.10 * duration_s)
    baseline_mask = time_s <= baseline_end
    target_baseline = float(np.median(target[baseline_mask]))
    actual_baseline = float(np.median(actual[baseline_mask]))
    target_delta = target - target_baseline
    actual_delta = actual - actual_baseline
    dominant_index = int(np.argmax(np.abs(target_delta)))
    dominant_value = float(target_delta[dominant_index])
    amplitude = abs(dominant_value)
    result["rms_tracking_error"] = float(np.sqrt(np.mean((actual - target) ** 2)))
    if amplitude < limits.amplitude_floor:
        result["reason"] = "command_amplitude_below_floor"
        return result

    sign = 1.0 if dominant_value >= 0.0 else -1.0
    signed_target = sign * target_delta
    signed_actual = sign * actual_delta
    plateau = np.abs(signed_target - amplitude) <= limits.plateau_fraction * amplitude

    # Select the contiguous plateau containing the dominant sample.
    plateau_start = dominant_index
    while plateau_start > 0 and plateau[plateau_start - 1]:
        plateau_start -= 1
    plateau_end = dominant_index
    while plateau_end + 1 < len(plateau) and plateau[plateau_end + 1]:
        plateau_end += 1
    plateau_duration_s = float(time_s[plateau_end] - time_s[plateau_start])
    result["target_value"] = float(target_baseline + sign * amplitude)
    result["command_amplitude"] = amplitude
    result["plateau_duration_s"] = plateau_duration_s
    if plateau_duration_s < limits.plateau_min_s:
        result["reason"] = "dominant_command_has_no_stable_plateau"
        return result

    # Walk back only within the current signed excursion.  Searching from the
    # beginning of a multi-key flight would incorrectly use an earlier W/R/Q
    # command as the onset of a later command with the same sign.
    onset_index = plateau_start
    while onset_index > 0 and signed_target[onset_index - 1] >= 0.10 * amplitude:
        onset_index -= 1
    analysis_end = min(len(time_s) - 1, plateau_end + max(1, plateau_end - plateau_start))
    response_slice = slice(onset_index, analysis_end + 1)
    response_actual = signed_actual[response_slice]
    actual_peak = float(np.max(response_actual))
    overshoot = max(0.0, actual_peak - amplitude) / amplitude

    rise_candidates = np.flatnonzero(
        signed_actual[onset_index : plateau_end + 1] >= 0.90 * amplitude
    )
    rise_index = (
        onset_index + int(rise_candidates[0]) if len(rise_candidates) else None
    )
    tolerance = max(limits.settling_band_fraction * amplitude, limits.amplitude_floor * 0.1)
    tracking_error = np.abs(actual - target)
    settle_index = None
    if rise_index is not None:
        settle_local = _first_held_index(
            tracking_error[rise_index : plateau_end + 1] <= tolerance,
            time_s[rise_index : plateau_end + 1],
            limits.settling_hold_s,
        )
        settle_index = rise_index + settle_local if settle_local is not None else None

    steady_start_time = max(
        float(time_s[plateau_start]),
        float(time_s[plateau_end]) - max(0.30, 0.25 * plateau_duration_s),
    )
    steady = (time_s >= steady_start_time) & (time_s <= time_s[plateau_end])
    result.update(
        {
            "step_evaluable": True,
            "actual_peak": float(actual_baseline + sign * actual_peak),
            "rise_time_s": (
                float(time_s[rise_index] - time_s[onset_index])
                if rise_index is not None
                else None
            ),
            "overshoot_fraction": float(overshoot),
            "settling_time_s": (
                float(time_s[settle_index] - time_s[onset_index])
                if settle_index is not None
                else None
            ),
            "steady_error": float(np.mean(actual[steady] - target[steady])),
            "reason": "ok",
        }
    )
    return result


def _dataset(ulog, name: str):
    matches = [item for item in ulog.data_list if item.name == name]
    return matches[0] if matches else None


def _matrix(data, prefix: str, count: int) -> np.ndarray:
    return np.column_stack([np.asarray(data[f"{prefix}[{index}]"], dtype=float) for index in range(count)])


def _euler_from_quaternion(q: np.ndarray) -> np.ndarray:
    q = np.asarray(q, dtype=float)
    q /= np.maximum(np.linalg.norm(q, axis=1, keepdims=True), 1.0e-12)
    w, x, y, z = q.T
    roll = np.arctan2(2.0 * (w * x + y * z), 1.0 - 2.0 * (x * x + y * y))
    pitch = np.arcsin(np.clip(2.0 * (w * y - z * x), -1.0, 1.0))
    yaw = np.unwrap(np.arctan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z)))
    return np.column_stack((roll, pitch, yaw))


def _interp_columns(source_t, values, target_t) -> np.ndarray:
    source_t = np.asarray(source_t, dtype=float)
    target_t = np.asarray(target_t, dtype=float)
    values = np.asarray(values, dtype=float)
    return np.column_stack(
        [np.interp(target_t, source_t, values[:, axis]) for axis in range(values.shape[1])]
    )


def _active_window(ulog) -> tuple[float, float]:
    mode = _dataset(ulog, "vehicle_control_mode")
    if mode is None:
        return float(ulog.start_timestamp), float(ulog.last_timestamp)
    data = mode.data
    t = np.asarray(data["timestamp"], dtype=float)
    active = np.asarray(data["flag_control_offboard_enabled"], dtype=bool)
    if "flag_armed" in data:
        active &= np.asarray(data["flag_armed"], dtype=bool)
    indices = np.flatnonzero(active)
    if not len(indices):
        return float(ulog.start_timestamp), float(ulog.last_timestamp)
    return float(t[indices[0]]), float(t[indices[-1]])


def _control_layer_mask(ulog, query_timestamp, layer: str) -> np.ndarray:
    """Select one armed Offboard layer using the preceding control mode."""
    query_timestamp = np.asarray(query_timestamp, dtype=float)
    mode = _dataset(ulog, "vehicle_control_mode")
    if mode is None:
        return np.ones(query_timestamp.shape, dtype=bool)
    data = mode.data
    mode_t = np.asarray(data["timestamp"], dtype=float)
    indices = np.searchsorted(mode_t, query_timestamp, side="right") - 1
    valid = indices >= 0
    indices = np.clip(indices, 0, max(0, len(mode_t) - 1))
    selected = valid.copy()
    selected &= np.asarray(data["flag_control_offboard_enabled"], dtype=bool)[indices]
    if "flag_armed" in data:
        selected &= np.asarray(data["flag_armed"], dtype=bool)[indices]
    position = np.asarray(data["flag_control_position_enabled"], dtype=bool)[indices]
    velocity = np.asarray(data["flag_control_velocity_enabled"], dtype=bool)[indices]
    attitude = np.asarray(data["flag_control_attitude_enabled"], dtype=bool)[indices]
    rates = np.asarray(data["flag_control_rates_enabled"], dtype=bool)[indices]
    if layer == "velocity":
        # PX4's position controller also enables every downstream controller.
        selected &= velocity & ~position
    elif layer == "attitude":
        selected &= attitude & ~velocity & ~position
    elif layer == "body_rate":
        selected &= rates & ~attitude & ~velocity & ~position
    elif layer == "direct_inner":
        selected &= rates & ~velocity & ~position
    else:
        raise ValueError(f"unknown control layer: {layer}")
    return selected


def _axis_report(time_s, target, actual, names, limits):
    return {
        name: response_metrics(time_s, target[:, index], actual[:, index], limits)
        for index, name in enumerate(names)
    }


def _segmented_axis_report(time_s, target, actual, names, limits, gap_s=0.50):
    """Analyze separated direct-control steps without joining recovery gaps."""
    time_s = np.asarray(time_s, dtype=float)
    target = np.asarray(target, dtype=float)
    actual = np.asarray(actual, dtype=float)
    if not len(time_s):
        return _axis_report(time_s, target, actual, names, limits)
    boundaries = np.flatnonzero(np.diff(time_s) > gap_s) + 1
    segments = np.split(np.arange(len(time_s)), boundaries)
    report = {}
    for axis, name in enumerate(names):
        candidates = []
        for segment in segments:
            if len(segment) < 8:
                continue
            metrics = response_metrics(
                time_s[segment], target[segment, axis], actual[segment, axis], limits
            )
            metrics["segment_start_s"] = float(time_s[segment[0]])
            metrics["segment_end_s"] = float(time_s[segment[-1]])
            candidates.append(metrics)
        if not candidates:
            report[name] = response_metrics([], [], [], limits)
            continue
        # The matching axis step has the largest command amplitude.  Prefer an
        # evaluable plateau only when amplitudes are effectively tied, so a
        # small incidental command cannot hide a malformed intended step.
        report[name] = max(
            candidates,
            key=lambda item: (
                float(item.get("command_amplitude") or 0.0),
                bool(item["step_evaluable"]),
            ),
        )
    return report


def _max_evaluable_overshoot(axes: dict[str, dict[str, Any]], selected=None):
    selected = selected or axes.keys()
    values = [
        axes[name]["overshoot_fraction"]
        for name in selected
        if axes[name]["step_evaluable"] and axes[name]["overshoot_fraction"] is not None
    ]
    return max(values) if values else None


def _complete_axis_overshoot(axes: dict[str, dict[str, Any]], selected=None):
    selected = tuple(selected or axes.keys())
    if not selected or any(
        name not in axes
        or not axes[name]["step_evaluable"]
        or axes[name]["overshoot_fraction"] is None
        for name in selected
    ):
        return None
    return max(float(axes[name]["overshoot_fraction"]) for name in selected)


def analyze_ulog(
    ulog,
    source: str,
    *,
    dedicated_inner_loop_step_protocol: bool = False,
) -> dict[str, Any]:
    window_start_us, window_end_us = _active_window(ulog)
    report: dict[str, Any] = {
        "schema": 1,
        "source_ulog": source,
        "window": {
            "start_log_s": (window_start_us - ulog.start_timestamp) / 1.0e6,
            "end_log_s": (window_end_us - ulog.start_timestamp) / 1.0e6,
            "duration_s": (window_end_us - window_start_us) / 1.0e6,
        },
        "loops": {},
    }

    rate_sp = _dataset(ulog, "vehicle_rates_setpoint")
    angular = _dataset(ulog, "vehicle_angular_velocity")
    if rate_sp is not None and angular is not None:
        sd, ad = rate_sp.data, angular.data
        st = np.asarray(sd["timestamp"], dtype=float)
        active = (
            (st >= window_start_us)
            & (st <= window_end_us)
            & _control_layer_mask(
                ulog,
                st,
                "body_rate" if dedicated_inner_loop_step_protocol else "velocity",
            )
        )
        target = np.column_stack([sd[name] for name in ("roll", "pitch", "yaw")])[active]
        actual_values = _matrix(ad, "xyz", 3)
        actual = _interp_columns(ad["timestamp"], actual_values, st[active])
        time_s = (st[active] - window_start_us) / 1.0e6
        reporter = (
            _segmented_axis_report
            if dedicated_inner_loop_step_protocol
            else _axis_report
        )
        report["loops"]["body_rate_rad_s"] = reporter(
            time_s,
            target,
            actual,
            ("roll", "pitch", "yaw"),
            ResponseLimits(0.05),
        )

    attitude_sp = _dataset(ulog, "vehicle_attitude_setpoint")
    attitude = _dataset(ulog, "vehicle_attitude")
    if attitude_sp is not None and attitude is not None:
        sd, ad = attitude_sp.data, attitude.data
        st = np.asarray(sd["timestamp"], dtype=float)
        active = (
            (st >= window_start_us)
            & (st <= window_end_us)
            & _control_layer_mask(
                ulog,
                st,
                "attitude" if dedicated_inner_loop_step_protocol else "velocity",
            )
        )
        target = _euler_from_quaternion(_matrix(sd, "q_d", 4))[active]
        actual_values = _euler_from_quaternion(_matrix(ad, "q", 4))
        actual = _interp_columns(ad["timestamp"], actual_values, st[active])
        time_s = (st[active] - window_start_us) / 1.0e6
        reporter = (
            _segmented_axis_report
            if dedicated_inner_loop_step_protocol
            else _axis_report
        )
        report["loops"]["attitude_rad"] = reporter(
            time_s,
            target,
            actual,
            ("roll", "pitch", "yaw"),
            ResponseLimits(math.radians(1.0)),
        )

    local_sp = _dataset(ulog, "vehicle_local_position_setpoint")
    local = _dataset(ulog, "vehicle_local_position")
    zero_velocity_segments: list[tuple[float, float]] = []
    if local_sp is not None and local is not None:
        sd, ld = local_sp.data, local.data
        st = np.asarray(sd["timestamp"], dtype=float)
        active = (
            (st >= window_start_us)
            & (st <= window_end_us)
            & _control_layer_mask(ulog, st, "velocity")
        )
        target = np.column_stack([sd[name] for name in ("vx", "vy", "vz")])[active]
        actual_values = np.column_stack([ld[name] for name in ("vx", "vy", "vz")])
        actual = _interp_columns(ld["timestamp"], actual_values, st[active])
        time_s = (st[active] - window_start_us) / 1.0e6
        report["loops"]["velocity_m_s"] = _axis_report(
            time_s, target, actual, ("north", "east", "down"), ResponseLimits(0.05)
        )

        finite_target = np.all(np.isfinite(target), axis=1)
        zero = finite_target & (np.linalg.norm(target, axis=1) <= 0.02)
        start = None
        for index, is_zero in enumerate(zero):
            if index > 0 and time_s[index] - time_s[index - 1] > 0.50:
                if start is not None and time_s[index - 1] - time_s[start] >= 2.0:
                    zero_velocity_segments.append((time_s[start], time_s[index - 1]))
                start = None
            if is_zero and start is None:
                start = index
            elif not is_zero and start is not None:
                if time_s[index - 1] - time_s[start] >= 2.0:
                    zero_velocity_segments.append((time_s[start], time_s[index - 1]))
                start = None
        if len(time_s) and start is not None and time_s[-1] - time_s[start] >= 2.0:
            zero_velocity_segments.append((time_s[start], time_s[-1]))

    actuator = _dataset(ulog, "actuator_motors")
    if actuator is not None:
        data = actuator.data
        t = np.asarray(data["timestamp"], dtype=float)
        active = (
            (t >= window_start_us)
            & (t <= window_end_us)
            & _control_layer_mask(
                ulog,
                t,
                "direct_inner" if dedicated_inner_loop_step_protocol else "velocity",
            )
        )
        motors = _matrix(data, "control", 8)[active]
        finite = np.isfinite(motors)
        saturated = finite & ((motors <= 0.001) | (motors >= 0.999))
        report["motors"] = {
            "sample_count": int(np.count_nonzero(finite)),
            "saturation_fraction": float(np.count_nonzero(saturated) / max(1, np.count_nonzero(finite))),
            "minimum": float(np.nanmin(np.where(finite, motors, np.nan))),
            "maximum": float(np.nanmax(np.where(finite, motors, np.nan))),
        }

    hover_tilt = None
    if zero_velocity_segments and attitude is not None:
        # The final qualifying zero-demand interval is the H/hover segment in
        # the deterministic WASD protocol; earlier position-takeoff intervals
        # contain NaN velocity setpoints and are excluded above.
        segment_start_s, segment_end_s = zero_velocity_segments[-1]
        ad = attitude.data
        at_s = (np.asarray(ad["timestamp"], dtype=float) - window_start_us) / 1.0e6
        euler = _euler_from_quaternion(_matrix(ad, "q", 4))
        inside = (at_s >= segment_start_s) & (at_s <= segment_end_s)
        if np.any(inside):
            tilt_deg = np.degrees(np.hypot(euler[inside, 0], euler[inside, 1]))
            hover_tilt = float(np.nanmax(tilt_deg))
            report["zero_velocity_hold"] = {
                "start_s": segment_start_s,
                "end_s": segment_end_s,
                "duration_s": segment_end_s - segment_start_s,
                "max_roll_pitch_tilt_deg": hover_tilt,
            }

    rate_axes = report["loops"].get("body_rate_rad_s", {})
    attitude_axes = report["loops"].get("attitude_rad", {})
    velocity_axes = report["loops"].get("velocity_m_s", {})
    rate_overshoot = (
        _complete_axis_overshoot(rate_axes)
        if dedicated_inner_loop_step_protocol
        else _max_evaluable_overshoot(rate_axes)
    )
    attitude_overshoot = (
        _complete_axis_overshoot(attitude_axes)
        if dedicated_inner_loop_step_protocol
        else _max_evaluable_overshoot(attitude_axes)
    )
    horizontal_overshoot = _max_evaluable_overshoot(velocity_axes, ("north", "east"))
    vertical_overshoot = _max_evaluable_overshoot(velocity_axes, ("down",))
    saturation = report.get("motors", {}).get("saturation_fraction")
    report["acceptance"] = {
        # Inner-loop setpoints generated naturally by the outer controller are
        # useful tracking diagnostics but are not independent identification
        # inputs.  Only a log explicitly produced by the bounded step protocol
        # may promote these two acceptance gates.
        "body_rate_overshoot_below_10_percent": (
            rate_overshoot < 0.10
            if dedicated_inner_loop_step_protocol and rate_overshoot is not None
            else None
        ),
        "attitude_overshoot_below_10_percent": (
            attitude_overshoot < 0.10
            if dedicated_inner_loop_step_protocol and attitude_overshoot is not None
            else None
        ),
        "horizontal_velocity_overshoot_below_15_percent": (
            horizontal_overshoot < 0.15 if horizontal_overshoot is not None else None
        ),
        "vertical_velocity_overshoot_below_10_percent": (
            vertical_overshoot < 0.10 if vertical_overshoot is not None else None
        ),
        "zero_velocity_tilt_below_2_deg": (
            hover_tilt < 2.0 if hover_tilt is not None else None
        ),
        "motor_saturation_near_zero": (
            saturation <= 1.0e-3 if saturation is not None else None
        ),
    }
    required = list(report["acceptance"].values())
    report["coverage"] = {
        "all_acceptance_metrics_evaluable": all(value is not None for value in required),
        "all_evaluable_metrics_pass": all(value for value in required if value is not None),
        "dedicated_inner_loop_step_protocol": dedicated_inner_loop_step_protocol,
        "interpretation": (
            "Inner-loop metrics use naturally generated setpoints from the outer-loop flight. "
            "Null step metrics require a dedicated bounded excitation before tuning."
        ),
    }
    return report


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("ulog", type=Path)
    parser.add_argument("--output", type=Path)
    parser.add_argument(
        "--dedicated-inner-loop-step-protocol",
        action="store_true",
        help="allow rate/attitude acceptance only for a bounded dedicated step log",
    )
    args = parser.parse_args()
    from pyulog import ULog

    report = analyze_ulog(
        ULog(str(args.ulog)),
        str(args.ulog),
        dedicated_inner_loop_step_protocol=args.dedicated_inner_loop_step_protocol,
    )
    rendered = json.dumps(report, indent=2, sort_keys=True)
    if args.output:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(rendered + "\n", encoding="utf-8")
        print(args.output)
    else:
        print(rendered)


if __name__ == "__main__":
    main()
