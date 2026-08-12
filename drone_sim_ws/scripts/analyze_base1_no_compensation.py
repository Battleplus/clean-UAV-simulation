#!/usr/bin/env python3
"""Summarize deterministic Base 1 no-compensation flight logs."""

from __future__ import annotations

import argparse
import json
import math
import re
from pathlib import Path


METRIC = re.compile(r"([a-zA-Z0-9_]+)=([^ ]+)")


def parse_log(path: Path) -> dict:
    text = path.read_text(encoding="utf-8", errors="replace")
    metric_line = next(
        (line for line in text.splitlines() if line.startswith("ARM_FLIGHT_METRICS ")),
        None,
    )
    metrics: dict[str, float | int | str] = {}
    if metric_line:
        for key, raw in METRIC.findall(metric_line):
            if "/" in raw:
                metrics[key] = raw
                continue
            try:
                value = float(raw)
                metrics[key] = int(value) if value.is_integer() else value
            except ValueError:
                metrics[key] = raw
    failsafe_seen = re.search(r"(?<![A-Za-z0-9_])failsafe=True\b", text) is not None
    dynamic_gate_pass = (
        "ARM_FLIGHT_CYCLE_METRICS label=demo_extended" in text
        and "ARM_FLIGHT_CYCLE_METRICS label=retracted" in text
        and not failsafe_seen
        and "ARM_FLIGHT_INTERNAL_LAND_DETECTED" not in text
        and float(metrics.get("horizontal_drift_m", math.inf)) < 0.15
        and float(metrics.get("altitude_span_m", math.inf)) < 0.30
        and float(metrics.get("max_truth_tilt_deg", math.inf)) < 3.0
        and float(metrics.get("motor_saturation_rate", math.inf)) == 0.0
    )
    return {
        "log": str(path.resolve()),
        "pass": "DDS_ARM_FLIGHT_PASS" in text,
        "dynamic_gate_pass": dynamic_gate_pass,
        "failsafe_seen": failsafe_seen,
        "internal_land_seen": "ARM_FLIGHT_INTERNAL_LAND_DETECTED" in text,
        "metrics": metrics,
    }


def build_report(logs: list[Path]) -> dict:
    runs = [parse_log(path) for path in logs]
    numeric_keys = (
        "horizontal_drift_m",
        "altitude_span_m",
        "max_truth_tilt_deg",
        "rms_truth_tilt_deg",
        "max_arm_torque_nm",
        "motor_saturation_rate",
    )
    maxima = {}
    for key in numeric_keys:
        values = [
            float(run["metrics"][key])
            for run in runs
            if key in run["metrics"] and math.isfinite(float(run["metrics"][key]))
        ]
        maxima[key] = max(values) if values else None
    all_pass = bool(runs) and all(
        run["pass"]
        and not run["failsafe_seen"]
        and not run["internal_land_seen"]
        for run in runs
    )
    return {
        "schema": "my_drone.base1-no-compensation-baseline.v1",
        "scope": "Base 1 4kg only; 7.735kg not exercised",
        "compensation": {
            "acceleration_feedforward": False,
            "torque_feedforward": False,
            "static_com_gain": 0.0,
            "disturbance_observer": False,
        },
        "profile": "full_extend_slow_4kg",
        "runs_requested": len(logs),
        "runs_passed": sum(1 for run in runs if run["pass"]),
        "dynamic_runs_passed": sum(1 for run in runs if run["dynamic_gate_pass"]),
        "all_dynamic_runs_pass": bool(runs)
        and all(run["dynamic_gate_pass"] for run in runs),
        "all_runs_pass": all_pass,
        "maxima": maxima,
        "runs": runs,
    }


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("logs", nargs="+", type=Path)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    report = build_report(args.logs)
    args.output.write_text(
        json.dumps(report, ensure_ascii=False, indent=2) + "\n", encoding="utf-8"
    )
    print(json.dumps(report, ensure_ascii=False, indent=2))
    return 0 if report["all_runs_pass"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
