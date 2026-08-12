#!/usr/bin/env python3
"""Compare Base 1 hover with arm control disabled and enabled-but-static."""

from __future__ import annotations

import argparse
import json
import re
from pathlib import Path


METRICS_RE = re.compile(r"DDS_DYNAMIC_HOVER_METRICS (\{[^\r\n]+\})")


def parse_log(path: Path) -> dict:
    if not path.exists():
        return {
            "log": str(path.resolve()),
            "exists": False,
            "pass": False,
            "metrics": {},
        }
    text = path.read_text(encoding="utf-8", errors="replace")
    matches = METRICS_RE.findall(text)
    metrics = json.loads(matches[-1]) if matches else {}
    return {
        "log": str(path.resolve()),
        "exists": True,
        "pass": "DDS_DYNAMIC_HOVER_PASS" in text,
        "metrics": metrics,
    }


def compare(disabled: dict, static: dict) -> dict:
    d = disabled["metrics"]
    s = static["metrics"]

    def delta(name: str) -> float | None:
        if d.get(name) is None or s.get(name) is None:
            return None
        return float(s[name]) - float(d[name])

    deltas = {
        "max_horizontal_error_m": delta("max_horizontal_error_m"),
        "truth_height_peak_to_peak_m": delta("truth_height_peak_to_peak_m"),
        "px4_vertical_speed_abs_p90_m_s": delta("px4_vertical_speed_abs_p90_m_s"),
        "mean_motor_output": delta("mean_motor_output"),
    }
    gates = {
        "both_runtime_pass": bool(disabled["pass"] and static["pass"]),
        "horizontal_regression_le_0p10_m": (
            deltas["max_horizontal_error_m"] is not None
            and deltas["max_horizontal_error_m"] <= 0.10
        ),
        "height_p2p_regression_le_0p03_m": (
            deltas["truth_height_peak_to_peak_m"] is not None
            and deltas["truth_height_peak_to_peak_m"] <= 0.03
        ),
        "vertical_speed_p90_regression_le_0p03_m_s": (
            deltas["px4_vertical_speed_abs_p90_m_s"] is not None
            and deltas["px4_vertical_speed_abs_p90_m_s"] <= 0.03
        ),
        "mean_motor_regression_le_50": (
            deltas["mean_motor_output"] is not None
            and deltas["mean_motor_output"] <= 50.0
        ),
        "no_motor_saturation": (
            d.get("motor_saturation_fraction") == 0.0
            and s.get("motor_saturation_fraction") == 0.0
        ),
        "no_failsafe": not d.get("failsafe_seen", True) and not s.get("failsafe_seen", True),
    }
    return {
        "schema": "my_drone.base1-arm-state-hover.v1",
        "scope": "Base 1 4kg only; compensation off; 7.735kg not exercised",
        "disabled": disabled,
        "static": static,
        "static_minus_disabled": deltas,
        "gates": gates,
        "pass": all(gates.values()),
    }


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("disabled_log", type=Path)
    parser.add_argument("static_log", type=Path)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    report = compare(parse_log(args.disabled_log), parse_log(args.static_log))
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")
    print(json.dumps(report, indent=2))
    return 0 if report["pass"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
