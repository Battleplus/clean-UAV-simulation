#!/usr/bin/env python3
"""Compare one Base 1 single-channel arm-compensation A/B pair."""

from __future__ import annotations

import argparse
import json
import math
from pathlib import Path

from analyze_base1_no_compensation import parse_log


MARKER = "BASE1_COMPENSATION_STATE "
PRINCIPAL_METRICS = (
    "horizontal_drift_m",
    "altitude_span_m",
    "max_truth_tilt_deg",
    "rms_truth_tilt_deg",
)


def parse_reallocator_log(path: Path) -> dict:
    text = path.read_text(encoding="utf-8", errors="replace")
    states = []
    for line in text.splitlines():
        if MARKER not in line:
            continue
        try:
            state = json.loads(line.split(MARKER, 1)[1])
        except json.JSONDecodeError:
            continue
        wrench = state.get("compensation_wrench_frd")
        if isinstance(wrench, list) and len(wrench) == 6:
            try:
                values = [float(value) for value in wrench]
            except (TypeError, ValueError):
                continue
            if all(math.isfinite(value) for value in values):
                state["wrench_norm"] = math.sqrt(sum(value * value for value in values))
                states.append(state)
    active = [state for state in states if state["wrench_norm"] > 1.0e-8]
    return {
        "log": str(path.resolve()),
        "state_count": len(states),
        "active_state_count": len(active),
        "maximum_wrench_norm": max(
            (state["wrench_norm"] for state in active), default=0.0
        ),
        "maximum_residual_norm": max(
            (float(state.get("residual_norm", math.inf)) for state in active),
            default=None,
        ),
        "maximum_saturated_motors": max(
            (int(state.get("saturated", 99)) for state in active), default=None
        ),
        "all_active_states_flight_allowed": bool(active)
        and all(state.get("flight_allowed") is True for state in active),
        "all_active_states_source_fresh": bool(active)
        and all(state.get("source_fresh") is True for state in active),
        "all_active_states_headroom_ok": bool(active)
        and all(state.get("headroom_ok") is True for state in active),
        "runtime_proven_active": bool(active),
    }


def compare(
    off_flight: Path,
    on_flight: Path,
    on_reallocator: Path,
    channel: str,
    gain: float,
) -> dict:
    off = parse_log(off_flight)
    on = parse_log(on_flight)
    runtime = parse_reallocator_log(on_reallocator)
    delta = {}
    ratios = {}
    for metric in PRINCIPAL_METRICS:
        off_value = float(off["metrics"].get(metric, math.inf))
        on_value = float(on["metrics"].get(metric, math.inf))
        delta[metric] = on_value - off_value
        ratios[metric] = on_value / off_value if off_value > 1.0e-12 else None
    both_accepted = bool(off["accepted_pass"] and on["accepted_pass"])
    improves_drift_and_rms = bool(
        delta["horizontal_drift_m"] < 0.0 and delta["rms_truth_tilt_deg"] < 0.0
    )
    no_principal_worsening = all(value <= 0.0 for value in delta.values())
    candidate_accepted = bool(
        both_accepted
        and runtime["runtime_proven_active"]
        and runtime["all_active_states_flight_allowed"]
        and runtime["all_active_states_source_fresh"]
        and runtime["all_active_states_headroom_ok"]
        and improves_drift_and_rms
        and no_principal_worsening
    )
    return {
        "schema": "my_drone.base1-single-channel-compensation-ab.v1",
        "scope": "Base 1 4kg only; 7.735kg not exercised",
        "channel": channel,
        "gain": gain,
        "off": off,
        "on": on,
        "candidate_runtime": runtime,
        "on_minus_off": delta,
        "on_over_off": ratios,
        "both_runs_accepted": both_accepted,
        "improves_drift_and_rms": improves_drift_and_rms,
        "no_principal_metric_worsened": no_principal_worsening,
        "candidate_accepted_for_repeat": candidate_accepted,
        "interpretation": (
            "candidate may proceed to an independent repeat pair"
            if candidate_accepted
            else "candidate is not accepted; do not combine or increase gain"
        ),
    }


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--off-flight", type=Path, required=True)
    parser.add_argument("--on-flight", type=Path, required=True)
    parser.add_argument("--on-reallocator", type=Path, required=True)
    parser.add_argument(
        "--channel",
        choices=("reaction_force", "gravity_torque", "reaction_torque"),
        required=True,
    )
    parser.add_argument("--gain", type=float, required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    report = compare(
        args.off_flight,
        args.on_flight,
        args.on_reallocator,
        args.channel,
        args.gain,
    )
    args.output.write_text(
        json.dumps(report, ensure_ascii=False, indent=2) + "\n", encoding="utf-8"
    )
    print(json.dumps(report, ensure_ascii=False, indent=2))
    return 0 if report["candidate_accepted_for_repeat"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
