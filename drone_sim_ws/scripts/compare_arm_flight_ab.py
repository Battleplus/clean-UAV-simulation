#!/usr/bin/env python3
"""Compare two arm-flight PTY logs without treating a command-only pass as proof.

The script is intentionally log-based: the two runs must be performed with the
same model, PX4 airframe, profile and startup procedure.  It reports whether
feed-forward changed the measured drift/altitude window, but does not claim
that a change is an improvement unless the run also stayed inside the safety
gates.
"""

from __future__ import annotations

import argparse
import json
import re
from pathlib import Path


METRICS_RE = re.compile(
    r"ARM_FLIGHT_METRICS\s+"
    r"horizontal_drift_m=(?P<drift>[-+0-9.e]+)\s+"
    r"altitude_span_m=(?P<altitude>[-+0-9.e]+)\s+"
    r"samples=(?P<samples>\d+)"
)
PRESET_RE = re.compile(r"ARM_PRESET_REACHED preset=(\S+) max_error=([-+0-9.e]+)")
STATE_RE = re.compile(
    r"STATE arm=(\d+) nav=(\d+) NED=\(([-+0-9.e]+),([-+0-9.e]+),([-+0-9.e]+)\)"
)


def recover_metrics_from_state_window(text: str) -> tuple[float, float, int]:
    """Recover airborne metrics when a later LAND timeout hid the summary."""
    in_window = False
    positions: list[tuple[float, float, float]] = []
    for line in text.splitlines():
        if "ARM_FLIGHT_HOVER_READY" in line:
            in_window = True
            continue
        if in_window and "ARM_FLIGHT_SENT_LAND" in line:
            break
        if not in_window:
            continue
        match = STATE_RE.search(line)
        if match is not None and int(match.group(1)) == 2:
            positions.append(tuple(float(match.group(index)) for index in (3, 4, 5)))
    if not positions:
        raise ValueError("no armed STATE samples between HOVER_READY and SENT_LAND")
    north0, east0, _ = positions[0]
    drift = max(
        ((north - north0) ** 2 + (east - east0) ** 2) ** 0.5
        for north, east, _ in positions
    )
    down = [position[2] for position in positions]
    return drift, max(down) - min(down), len(positions)


def parse_log(path: Path) -> dict:
    text = path.read_text(encoding="utf-8", errors="replace")
    metrics = METRICS_RE.search(text)
    if metrics is None:
        try:
            drift, altitude, samples = recover_metrics_from_state_window(text)
        except ValueError as exc:
            raise ValueError(f"{path}: ARM_FLIGHT_METRICS not found: {exc}") from exc
        metrics_source = "recovered_state_window"
    else:
        drift = float(metrics.group("drift"))
        altitude = float(metrics.group("altitude"))
        samples = int(metrics.group("samples"))
        metrics_source = "summary_marker"
    presets = {
        name: float(error) for name, error in PRESET_RE.findall(text)
    }
    return {
        "log": str(path),
        "horizontal_drift_m": drift,
        "altitude_span_m": altitude,
        "samples": samples,
        "metrics_source": metrics_source,
        "pass_marker": "DDS_ARM_FLIGHT_PASS" in text,
        # Do not mistake the test summary's ``no_failsafe=True`` for an
        # observed PX4 failsafe state.
        "failsafe_seen": bool(re.search(r"(?<![A-Za-z_])failsafe=True", text)),
        "safety_abort": "ARM_FLIGHT_ABORTED" in text,
        "preset_max_error_rad": presets,
        "all_required_presets_reached": all(
            name in presets
            for name in ("flight_micro_a", "flight_micro_b", "retracted")
        ),
        "within_current_gate": (
            drift < 1.5
            and altitude < 1.5
            and not bool(re.search(r"(?<![A-Za-z_])failsafe=True", text))
        ),
    }


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--feedforward-off", type=Path, required=True)
    parser.add_argument("--feedforward-on", type=Path, required=True)
    parser.add_argument("--output", type=Path)
    args = parser.parse_args()

    off = parse_log(args.feedforward_off)
    on = parse_log(args.feedforward_on)
    report = {
        "feedforward_off": off,
        "feedforward_on": on,
        "delta_on_minus_off": {
            "horizontal_drift_m": on["horizontal_drift_m"]
            - off["horizontal_drift_m"],
            "altitude_span_m": on["altitude_span_m"]
            - off["altitude_span_m"],
        },
        "comparison": {
            "same_gate_pass": bool(
                off["within_current_gate"] and on["within_current_gate"]
            ),
            "feedforward_run_accepted": bool(
                on["within_current_gate"]
                and on["all_required_presets_reached"]
                and on["pass_marker"]
            ),
            "baseline_run_accepted": bool(
                off["within_current_gate"]
                and off["all_required_presets_reached"]
                and off["pass_marker"]
            ),
            "paired_runs_accepted": bool(
                off["within_current_gate"]
                and on["within_current_gate"]
                and off["all_required_presets_reached"]
                and on["all_required_presets_reached"]
                and off["pass_marker"]
                and on["pass_marker"]
            ),
            "interpretation": (
                "feed-forward reduced both measured windows"
                if on["horizontal_drift_m"] <= off["horizontal_drift_m"]
                and on["altitude_span_m"] <= off["altitude_span_m"]
                else "feed-forward did not reduce both measured windows"
            ),
        },
    }
    rendered = json.dumps(report, indent=2, ensure_ascii=False)
    print(rendered)
    if args.output:
        args.output.write_text(rendered + "\n", encoding="utf-8")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
