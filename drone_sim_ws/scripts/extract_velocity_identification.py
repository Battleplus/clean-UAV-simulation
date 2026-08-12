#!/usr/bin/env python3
"""Extract auditable velocity-loop identification from one raw PTY log."""

from __future__ import annotations

import argparse
import hashlib
import json
import math
from pathlib import Path
from typing import Any


MARKER = "DDS_VELOCITY_WASD_METRICS "
REQUIRED_MARKERS = ("DDS_VELOCITY_WASD_PASS", "LANDING_DISARMED_CONFIRMED")


def _finite(metrics: dict[str, Any], keys: tuple[str, ...]) -> bool:
    return all(
        isinstance(metrics.get(key), (int, float))
        and math.isfinite(float(metrics[key]))
        for key in keys
    )


def extract(log_path: Path, expected_state_report_hz: float) -> dict[str, Any]:
    payload = log_path.read_bytes()
    text = payload.decode("utf-8", errors="replace")
    missing_markers = [marker for marker in REQUIRED_MARKERS if marker not in text]
    metric_lines = [line for line in text.splitlines() if MARKER in line]
    if not metric_lines:
        raise ValueError("raw log does not contain DDS_VELOCITY_WASD_METRICS")
    raw = json.loads(metric_lines[-1].split(MARKER, 1)[1])

    commanded = {
        "horizontal": ("W_0", "S_1", "A_2", "D_3"),
        "vertical": ("R_4", "F_5"),
        "yaw": ("Q_6", "E_7"),
    }
    response_groups = {
        "horizontal": ("phase_horizontal_response", "m_s"),
        "vertical": ("phase_vertical_response", "m_s"),
        "yaw": ("phase_yaw_response", "deg_s"),
    }
    minimum_samples = max(8, int(round(expected_state_report_hz * 1.5)))
    stage_audit: dict[str, Any] = {}
    all_stage_metrics_complete = True
    all_stage_samples_sufficient = True
    stage_overshoot_by_group: dict[str, list[float]] = {
        group: [] for group in commanded
    }
    for group, stages in commanded.items():
        source_key, suffix = response_groups[group]
        source = raw.get(source_key, {})
        for stage in stages:
            metrics = source.get(stage, {})
            required = (
                f"target_{suffix}",
                f"actual_peak_directed_{suffix}",
                "rise_time_s",
                "overshoot_fraction",
                "settling_time_s",
                f"steady_error_{suffix}",
            )
            complete = _finite(metrics, required)
            samples = int(metrics.get("samples") or 0)
            sufficient = samples >= minimum_samples
            all_stage_metrics_complete &= complete
            all_stage_samples_sufficient &= sufficient
            if complete:
                stage_overshoot_by_group[group].append(
                    float(metrics["overshoot_fraction"])
                )
            stage_audit[stage] = {
                "group": group,
                "samples": samples,
                "minimum_samples": minimum_samples,
                "samples_sufficient": sufficient,
                "metrics_complete": complete,
                "metrics": metrics,
            }

    h_quality = raw.get("phase_quality", {}).get("H_9", {})
    zero_tilt = h_quality.get("tail_1s_max_truth_roll_pitch_deg")
    reported_overshoot_keys = {
        "horizontal": "horizontal_speed_overshoot_fraction",
        "vertical": "vertical_speed_overshoot_fraction",
        "yaw": "yaw_rate_overshoot_fraction",
    }
    stage_maximum_overshoot = {
        group: (max(values) if values else None)
        for group, values in stage_overshoot_by_group.items()
    }
    aggregate_not_understated: dict[str, bool | None] = {}
    for group, raw_key in reported_overshoot_keys.items():
        reported = raw.get(raw_key)
        stage_maximum = stage_maximum_overshoot[group]
        aggregate_not_understated[group] = (
            float(reported) + 1.0e-6 >= float(stage_maximum)
            if isinstance(reported, (int, float))
            and math.isfinite(float(reported))
            and isinstance(stage_maximum, (int, float))
            and math.isfinite(float(stage_maximum))
            else None
        )
    acceptance = {
        "horizontal_overshoot_below_15_percent": (
            float(raw["horizontal_speed_overshoot_fraction"]) < 0.15
            if isinstance(raw.get("horizontal_speed_overshoot_fraction"), (int, float))
            else None
        ),
        "vertical_overshoot_below_10_percent": (
            float(raw["vertical_speed_overshoot_fraction"]) < 0.10
            if isinstance(raw.get("vertical_speed_overshoot_fraction"), (int, float))
            else None
        ),
        "yaw_overshoot_below_10_percent": (
            float(raw["yaw_rate_overshoot_fraction"]) < 0.10
            if isinstance(raw.get("yaw_rate_overshoot_fraction"), (int, float))
            else None
        ),
        "zero_velocity_tilt_below_2_deg": (
            float(zero_tilt) < 2.0 if isinstance(zero_tilt, (int, float)) else None
        ),
        "motor_saturation_near_zero": (
            float(raw["motor_saturation_fraction"]) <= 1.0e-3
            if isinstance(raw.get("motor_saturation_fraction"), (int, float))
            else None
        ),
    }
    acceptance_complete = all(value is not None for value in acceptance.values())
    acceptance_pass = acceptance_complete and all(acceptance.values())
    overshoot_consistency_complete = all(
        value is not None for value in aggregate_not_understated.values()
    )
    overshoot_consistency_pass = (
        overshoot_consistency_complete and all(aggregate_not_understated.values())
    )
    ready = bool(
        not missing_markers
        and all_stage_metrics_complete
        and all_stage_samples_sufficient
        and overshoot_consistency_pass
        and acceptance_pass
    )
    return {
        "schema": 1,
        "source_log": str(log_path.resolve()),
        "source_log_size_bytes": len(payload),
        "source_log_sha256": hashlib.sha256(payload).hexdigest(),
        "expected_state_report_hz": expected_state_report_hz,
        "required_markers_present": not missing_markers,
        "missing_markers": missing_markers,
        "stage_audit": stage_audit,
        "acceptance": acceptance,
        "stage_maximum_overshoot_fraction": stage_maximum_overshoot,
        "aggregate_overshoot_not_understated": aggregate_not_understated,
        "overshoot_consistency_pass": overshoot_consistency_pass,
        "all_stage_metrics_complete": all_stage_metrics_complete,
        "all_stage_samples_sufficient": all_stage_samples_sufficient,
        "ready_for_velocity_tuning": ready,
        "result": (
            "VELOCITY_IDENTIFICATION_EVIDENCE_PASS"
            if ready
            else "VELOCITY_IDENTIFICATION_EVIDENCE_INCOMPLETE"
        ),
        "raw_metrics": raw,
    }


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("log", type=Path)
    parser.add_argument("--state-report-hz", type=float, required=True)
    parser.add_argument("--output", type=Path)
    args = parser.parse_args()
    report = extract(args.log, args.state_report_hz)
    serialized = json.dumps(report, ensure_ascii=False, indent=2) + "\n"
    if args.output:
        args.output.write_text(serialized, encoding="utf-8")
    print(serialized, end="")
    return 0 if report["ready_for_velocity_tuning"] else 2


if __name__ == "__main__":
    raise SystemExit(main())
