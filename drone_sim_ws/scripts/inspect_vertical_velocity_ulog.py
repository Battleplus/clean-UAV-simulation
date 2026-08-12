#!/usr/bin/env python3
"""Compare PX4 vertical velocity with Gazebo truth in velocity-only Offboard."""

from __future__ import annotations

import argparse
import json
from pathlib import Path

import numpy as np
from pyulog import ULog


def dataset(ulog: ULog, name: str):
    return next((item.data for item in ulog.data_list if item.name == name), None)


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("ulog", type=Path)
    parser.add_argument("--output", type=Path)
    args = parser.parse_args()

    ulog = ULog(str(args.ulog))
    trajectory = dataset(ulog, "trajectory_setpoint")
    estimated = dataset(ulog, "vehicle_local_position")
    truth = dataset(ulog, "vehicle_local_position_groundtruth")
    if trajectory is None or estimated is None or truth is None:
        raise SystemExit("ULog lacks trajectory/local-position/groundtruth datasets")

    setpoint_time = np.asarray(trajectory["timestamp"], dtype=np.int64)
    position = np.column_stack(
        [trajectory[f"position[{axis}]"] for axis in range(3)]
    )
    velocity = np.column_stack(
        [trajectory[f"velocity[{axis}]"] for axis in range(3)]
    )
    acceleration = np.column_stack(
        [trajectory[f"acceleration[{axis}]"] for axis in range(3)]
    )
    velocity_only = np.all(~np.isfinite(position), axis=1) & np.all(
        np.isfinite(velocity), axis=1
    )
    if not np.any(velocity_only):
        raise SystemExit("ULog has no velocity-only Offboard trajectory setpoints")

    start = int(setpoint_time[velocity_only][0])
    end = int(setpoint_time[velocity_only][-1])
    estimate_time = np.asarray(estimated["timestamp"], dtype=np.int64)
    truth_time = np.asarray(truth["timestamp"], dtype=np.int64)
    window = (estimate_time >= start) & (estimate_time <= end)
    estimate_vz = np.asarray(estimated["vz"], dtype=float)[window]
    aligned_truth_vz = np.interp(
        estimate_time[window], truth_time, np.asarray(truth["vz"], dtype=float)
    )
    truth_window = (truth_time >= start) & (truth_time <= end)
    native_truth_time = truth_time[truth_window]
    native_truth_vz = np.asarray(truth["vz"], dtype=float)[truth_window]
    truth_dt_s = np.diff(native_truth_time) / 1.0e6
    truth_dvz = np.diff(native_truth_vz)
    derivative_valid = (truth_dt_s > 1.0e-6) & (truth_dt_s < 0.05)
    truth_vertical_acceleration = np.divide(
        truth_dvz,
        truth_dt_s,
        out=np.full_like(truth_dvz, np.nan),
        where=derivative_valid,
    )
    error = estimate_vz - aligned_truth_vz
    reset_z = np.asarray(estimated["z_reset_counter"])[window]
    reset_vz = np.asarray(estimated["vz_reset_counter"])[window]
    command_vz = velocity[velocity_only, 2]
    command_az = acceleration[velocity_only, 2]

    result = {
        "schema": 2,
        "ulog": str(args.ulog),
        "velocity_only_start_log_s": (start - ulog.start_timestamp) / 1.0e6,
        "velocity_only_duration_s": (end - start) / 1.0e6,
        "setpoint_samples": int(np.count_nonzero(velocity_only)),
        "command_vz_min_m_s": float(np.min(command_vz)),
        "command_vz_max_m_s": float(np.max(command_vz)),
        "acceleration_feedforward_finite_fraction": float(
            np.mean(np.isfinite(command_az))
        ),
        "command_az_min_m_s2": (
            float(np.nanmin(command_az)) if np.any(np.isfinite(command_az)) else None
        ),
        "command_az_max_m_s2": (
            float(np.nanmax(command_az)) if np.any(np.isfinite(command_az)) else None
        ),
        "estimated_vz_min_m_s": float(np.min(estimate_vz)),
        "estimated_vz_max_m_s": float(np.max(estimate_vz)),
        "truth_vz_min_m_s": float(np.min(aligned_truth_vz)),
        "truth_vz_max_m_s": float(np.max(aligned_truth_vz)),
        "estimated_truth_vz_error_mean_m_s": float(np.mean(error)),
        "estimated_truth_vz_error_max_abs_m_s": float(np.max(np.abs(error))),
        "estimated_truth_vz_correlation": float(
            np.corrcoef(estimate_vz, aligned_truth_vz)[0, 1]
        ),
        "native_truth_samples": int(native_truth_vz.size),
        "native_truth_max_sample_interval_s": (
            float(np.max(truth_dt_s)) if truth_dt_s.size else None
        ),
        "native_truth_max_abs_vz_delta_m_s": (
            float(np.max(np.abs(truth_dvz))) if truth_dvz.size else None
        ),
        "native_truth_max_abs_vertical_acceleration_m_s2": (
            float(np.nanmax(np.abs(truth_vertical_acceleration)))
            if np.any(np.isfinite(truth_vertical_acceleration)) else None
        ),
        "native_truth_vertical_impulses_over_5m_s2": int(
            np.count_nonzero(np.abs(truth_vertical_acceleration) > 5.0)
        ),
        "z_reset_counters_in_window": sorted({int(value) for value in reset_z}),
        "vz_reset_counters_in_window": sorted({int(value) for value in reset_vz}),
        "z_reset_changes_in_window": int(np.count_nonzero(np.diff(reset_z))),
        "vz_reset_changes_in_window": int(np.count_nonzero(np.diff(reset_vz))),
    }
    rendered = json.dumps(result, indent=2, ensure_ascii=False)
    print(rendered)
    if args.output:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(rendered + "\n", encoding="utf-8")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
