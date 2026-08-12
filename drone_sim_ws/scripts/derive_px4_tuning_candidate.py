#!/usr/bin/env python3
"""Gate and derive one bounded PX4 tuning candidate from identification data.

The controller hierarchy is tuned from the inside out: body rate, attitude,
then velocity.  Missing rise/settling/steady-state metrics are a hard hold, not
permission to guess.  This script never edits an airframe.
"""

from __future__ import annotations

import argparse
import json
import math
import re
from pathlib import Path
from typing import Any


PARAM_RE = re.compile(
    r"^\s*param\s+(?:set-default|set)\s+([A-Z0-9_]+)\s+([-+0-9.eE]+)\s*$"
)
REQUIRED_STEP_METRICS = (
    "target_value",
    "actual_peak",
    "rise_time_s",
    "overshoot_fraction",
    "settling_time_s",
    "steady_error",
)

LAYERS = [
    {
        "name": "body_rate",
        "report_key": "body_rate_rad_s",
        "axes": ("roll", "pitch", "yaw"),
        "overshoot_limit": 0.10,
        "parameters": {
            "roll": "MC_ROLLRATE_P",
            "pitch": "MC_PITCHRATE_P",
            "yaw": "MC_YAWRATE_P",
        },
    },
    {
        "name": "attitude",
        "report_key": "attitude_rad",
        "axes": ("roll", "pitch", "yaw"),
        "overshoot_limit": 0.10,
        "parameters": {
            "roll": "MC_ROLL_P",
            "pitch": "MC_PITCH_P",
            "yaw": "MC_YAW_P",
        },
    },
]


def parse_airframe_parameters(path: Path) -> dict[str, float]:
    parameters: dict[str, float] = {}
    for line in path.read_text(encoding="utf-8", errors="replace").splitlines():
        match = PARAM_RE.match(line)
        if match:
            parameters[match.group(1)] = float(match.group(2))
    return parameters


def _missing_metrics(axis_report: dict[str, Any]) -> list[str]:
    missing = []
    if axis_report.get("step_evaluable") is not True:
        missing.append("step_evaluable")
    for name in REQUIRED_STEP_METRICS:
        value = axis_report.get(name)
        if not isinstance(value, (int, float)) or not math.isfinite(float(value)):
            missing.append(name)
    return missing


def evaluate(
    report: dict[str, Any],
    parameters: dict[str, float],
    velocity_evidence: dict[str, Any] | None = None,
) -> dict[str, Any]:
    saturation = report.get("motors", {}).get("saturation_fraction")
    if not isinstance(saturation, (int, float)) or not math.isfinite(float(saturation)):
        return {
            "decision": "HOLD_INCOMPLETE_IDENTIFICATION",
            "active_layer": "global_safety",
            "reasons": ["motor saturation fraction is missing"],
            "parameter_changes": [],
        }
    if float(saturation) > 1.0e-3:
        return {
            "decision": "HOLD_MOTOR_SATURATION",
            "active_layer": "global_safety",
            "reasons": [f"motor saturation fraction {saturation:.6f} exceeds 0.001"],
            "parameter_changes": [],
        }

    loops = report.get("loops", {})
    passed_layers: list[str] = []
    for layer in LAYERS:
        name = str(layer["name"])
        axes_report = loops.get(layer["report_key"], {})
        incomplete: list[str] = []
        for axis in layer["axes"]:
            missing = _missing_metrics(axes_report.get(axis, {}))
            if missing:
                incomplete.append(f"{name}.{axis}: missing {', '.join(missing)}")
        if incomplete:
            return {
                "decision": "HOLD_INCOMPLETE_IDENTIFICATION",
                "active_layer": name,
                "passed_layers": passed_layers,
                "reasons": incomplete,
                "parameter_changes": [],
            }

        failing_axes: list[tuple[str, float, float]] = []
        for axis in layer["axes"]:
            metrics = axes_report[axis]
            limit = float(
                layer.get("overshoot_limit_by_axis", {}).get(
                    axis, layer.get("overshoot_limit")
                )
            )
            overshoot = float(metrics["overshoot_fraction"])
            if overshoot >= limit:
                failing_axes.append((axis, overshoot, limit))
        if failing_axes:
            changes_by_parameter: dict[str, dict[str, Any]] = {}
            reasons = []
            for axis, overshoot, limit in failing_axes:
                parameter = layer["parameters"][axis]
                if parameter not in parameters:
                    return {
                        "decision": "HOLD_MISSING_AIRFRAME_PARAMETER",
                        "active_layer": name,
                        "passed_layers": passed_layers,
                        "reasons": [f"airframe does not define {parameter}"],
                        "parameter_changes": [],
                    }
                # One conservative iteration only.  Re-identification is
                # mandatory before another adjustment.
                reduction = min(0.05, max(0.02, 0.5 * (overshoot - limit)))
                old = float(parameters[parameter])
                new = old * (1.0 - reduction)
                prior = changes_by_parameter.get(parameter)
                candidate = {
                    "parameter": parameter,
                    "old": old,
                    "candidate": new,
                    "relative_change": -reduction,
                    "axes": [axis],
                    "status": "EXPERIMENTAL_NOT_APPLIED",
                }
                if prior is None or reduction > abs(float(prior["relative_change"])):
                    changes_by_parameter[parameter] = candidate
                elif axis not in prior["axes"]:
                    prior["axes"].append(axis)
                reasons.append(
                    f"{name}.{axis} overshoot {overshoot:.3%} is not below {limit:.1%}"
                )
            return {
                "decision": "BOUNDED_CANDIDATE_REQUIRES_RETEST",
                "active_layer": name,
                "passed_layers": passed_layers,
                "reasons": reasons,
                "parameter_changes": list(changes_by_parameter.values()),
                "application_policy": "apply only to a disposable 4 kg test airframe, then rerun the same identification protocol",
            }
        passed_layers.append(name)

    if not velocity_evidence or velocity_evidence.get("ready_for_velocity_tuning") is not True:
        return {
            "decision": "HOLD_INCOMPLETE_IDENTIFICATION",
            "active_layer": "velocity",
            "passed_layers": passed_layers,
            "reasons": [
                "a SHA-256-backed 10 Hz WASD evidence report is missing or incomplete"
            ],
            "parameter_changes": [],
        }

    velocity_acceptance = velocity_evidence.get("acceptance", {})
    required_velocity_gates = (
        "horizontal_overshoot_below_15_percent",
        "vertical_overshoot_below_10_percent",
        "zero_velocity_tilt_below_2_deg",
        "motor_saturation_near_zero",
    )
    incomplete_velocity = [
        key for key in required_velocity_gates
        if not isinstance(velocity_acceptance.get(key), bool)
    ]
    if incomplete_velocity:
        return {
            "decision": "HOLD_INCOMPLETE_IDENTIFICATION",
            "active_layer": "velocity",
            "passed_layers": passed_layers,
            "reasons": [f"velocity evidence missing gates: {', '.join(incomplete_velocity)}"],
            "parameter_changes": [],
        }

    failed_velocity = [
        key for key in required_velocity_gates if velocity_acceptance[key] is False
    ]
    if failed_velocity:
        # Oscillation or saturation failures are not safely reduced to one P
        # change.  They require diagnosis rather than a guessed gain edit.
        if any(
            key in failed_velocity
            for key in ("zero_velocity_tilt_below_2_deg", "motor_saturation_near_zero")
        ):
            return {
                "decision": "HOLD_VELOCITY_SAFETY_GATE",
                "active_layer": "velocity",
                "passed_layers": passed_layers,
                "reasons": failed_velocity,
                "parameter_changes": [],
            }
        changes = []
        if "horizontal_overshoot_below_15_percent" in failed_velocity:
            parameter = "MPC_XY_VEL_P_ACC"
            if parameter not in parameters:
                return {
                    "decision": "HOLD_MISSING_AIRFRAME_PARAMETER",
                    "active_layer": "velocity",
                    "passed_layers": passed_layers,
                    "reasons": [f"airframe does not define {parameter}"],
                    "parameter_changes": [],
                }
            changes.append({
                "parameter": parameter,
                "old": parameters[parameter],
                "candidate": parameters[parameter] * 0.95,
                "relative_change": -0.05,
                "axes": ["horizontal"],
                "status": "EXPERIMENTAL_NOT_APPLIED",
            })
        if "vertical_overshoot_below_10_percent" in failed_velocity:
            parameter = "MPC_Z_VEL_P_ACC"
            if parameter not in parameters:
                return {
                    "decision": "HOLD_MISSING_AIRFRAME_PARAMETER",
                    "active_layer": "velocity",
                    "passed_layers": passed_layers,
                    "reasons": [f"airframe does not define {parameter}"],
                    "parameter_changes": [],
                }
            changes.append({
                "parameter": parameter,
                "old": parameters[parameter],
                "candidate": parameters[parameter] * 0.95,
                "relative_change": -0.05,
                "axes": ["vertical"],
                "status": "EXPERIMENTAL_NOT_APPLIED",
            })
        return {
            "decision": "BOUNDED_CANDIDATE_REQUIRES_RETEST",
            "active_layer": "velocity",
            "passed_layers": passed_layers,
            "reasons": failed_velocity,
            "parameter_changes": changes,
            "application_policy": "apply only to a disposable 4 kg test airframe, then rerun both identification protocols",
        }
    passed_layers.append("velocity")
    return {
        "decision": "ALL_LAYERS_PASS_NO_PARAMETER_CHANGE",
        "active_layer": None,
        "passed_layers": passed_layers,
        "reasons": [],
        "parameter_changes": [],
    }


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("report", type=Path)
    parser.add_argument("airframe", type=Path)
    parser.add_argument("--velocity-evidence", type=Path)
    parser.add_argument("--output", type=Path)
    args = parser.parse_args()
    report = json.loads(args.report.read_text(encoding="utf-8"))
    velocity_evidence = (
        json.loads(args.velocity_evidence.read_text(encoding="utf-8"))
        if args.velocity_evidence else None
    )
    result = {
        "schema": 1,
        "source_report": str(args.report.resolve()),
        "source_airframe": str(args.airframe.resolve()),
        "source_velocity_evidence": (
            str(args.velocity_evidence.resolve()) if args.velocity_evidence else None
        ),
        **evaluate(
            report,
            parse_airframe_parameters(args.airframe),
            velocity_evidence,
        ),
    }
    serialized = json.dumps(result, ensure_ascii=False, indent=2) + "\n"
    if args.output:
        args.output.write_text(serialized, encoding="utf-8")
    print(serialized, end="")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
