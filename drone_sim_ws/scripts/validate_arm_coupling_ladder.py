#!/usr/bin/env python3
"""Validate the 4 kg staged arm-coupling report against raw flight logs."""

from __future__ import annotations

import argparse
import hashlib
import json
import math
import re
from pathlib import Path
from typing import Any


EXPECTED_STAGES = [
    "gripper",
    "wrist_roll",
    "shoulder_pan_single_joint",
    "multi_joint_slow",
    "full_extend_retract",
    "full_extend_retract_with_rigid_payload",
]

METRICS = [
    "horizontal_drift_m",
    "altitude_span_m",
    "max_com_shift_m",
    "max_inertia_diag_change_kg_m2",
    "max_arm_force_n",
    "max_arm_torque_nm",
    "max_truth_tilt_deg",
    "motor_saturation_samples",
]

LIMIT_MAP = {
    "horizontal_drift_m": "horizontal_drift_m",
    "altitude_span_m": "altitude_span_m",
    "max_arm_torque_nm": "arm_reaction_torque_nm",
    "max_truth_tilt_deg": "truth_tilt_deg",
    "motor_saturation_samples": "motor_saturation_samples",
}

PAIR_RE = re.compile(r"([a-zA-Z0-9_]+)=([^\s]+)")


def parse_metrics(log_path: Path) -> dict[str, float]:
    text = log_path.read_text(encoding="utf-8", errors="replace")
    if "LANDING_DISARMED_CONFIRMED" not in text:
        raise ValueError(f"{log_path}: missing confirmed landing/disarm marker")
    if "DDS_ARM_FLIGHT_PASS" not in text:
        raise ValueError(f"{log_path}: missing accepted flight marker")

    lines = [line for line in text.splitlines() if "ARM_FLIGHT_METRICS " in line]
    if not lines:
        raise ValueError(f"{log_path}: missing ARM_FLIGHT_METRICS line")
    values: dict[str, float] = {}
    for key, raw in PAIR_RE.findall(lines[-1].split("ARM_FLIGHT_METRICS ", 1)[1]):
        if key == "motor_saturation_samples":
            raw = raw.split("/", 1)[0]
        try:
            values[key] = float(raw)
        except ValueError:
            continue
    missing = [metric for metric in METRICS if metric not in values]
    if missing:
        raise ValueError(f"{log_path}: missing metrics {missing}")
    return values


def stage_log_paths(workspace: Path, stage: dict[str, Any]) -> list[Path]:
    raw = stage.get("logs")
    if raw is None:
        raw = [stage.get("log")]
    if not isinstance(raw, list) or not raw or any(not item for item in raw):
        raise ValueError(f"stage {stage.get('stage')}: missing log evidence")
    return [(workspace / str(item)).resolve() for item in raw]


def validate_tensor_evidence(workspace: Path, evidence: dict[str, Any]) -> dict[str, Any]:
    report_path = (workspace / str(evidence.get("report", ""))).resolve()
    validation_path = (workspace / str(evidence.get("validation", ""))).resolve()
    if not report_path.is_file() or not validation_path.is_file():
        raise ValueError("full-tensor report or validation evidence is missing")
    payload = report_path.read_bytes()
    actual_sha256 = hashlib.sha256(payload).hexdigest()
    validation = json.loads(validation_path.read_text(encoding="utf-8"))
    if validation.get("result") != "ARM_COUPLING_TENSOR_REPORT_PASS":
        raise ValueError("full-tensor validation is not PASS")
    if not math.isclose(float(validation.get("base_mass_kg", math.nan)), 4.0, abs_tol=1e-9):
        raise ValueError("full-tensor validation is not for the 4 kg base model")
    if not math.isclose(float(validation.get("payload_mass_kg", math.nan)), 0.05, abs_tol=1e-9):
        raise ValueError("full-tensor validation is not for the 0.05 kg payload")
    if validation.get("source_sha256") != actual_sha256:
        raise ValueError("full-tensor report SHA-256 does not match its validation")
    if evidence.get("report_sha256") != actual_sha256:
        raise ValueError("ladder full-tensor SHA-256 reference is stale")
    return {
        "report": report_path.relative_to(workspace).as_posix(),
        "validation": validation_path.relative_to(workspace).as_posix(),
        "report_sha256": actual_sha256,
        "result": validation["result"],
    }


def validate(workspace: Path, report_path: Path) -> dict[str, Any]:
    report = json.loads(report_path.read_text(encoding="utf-8"))
    stages = report.get("stages", [])
    if [stage.get("stage") for stage in stages] != EXPECTED_STAGES:
        raise ValueError("coupling stages are missing, duplicated, or out of order")
    if report.get("overall_result") != "pass_for_4kg_debug_scope":
        raise ValueError("report is not marked as a 4 kg debug-scope pass")
    if report.get("feedforward_enabled") is not False:
        raise ValueError("accepted ladder must retain the rejected feed-forward as disabled")
    tensor_evidence = validate_tensor_evidence(
        workspace, report.get("full_tensor_model_evidence", {})
    )

    limits = report["acceptance_limits"]
    checked: list[dict[str, Any]] = []
    for stage in stages:
        stage_name = str(stage["stage"])
        if stage.get("result") != "pass":
            raise ValueError(f"stage {stage_name}: result is not pass")
        paths = stage_log_paths(workspace, stage)
        runs = [parse_metrics(path) for path in paths]
        conservative = {metric: max(run[metric] for run in runs) for metric in METRICS}

        for metric, expected in conservative.items():
            actual = float(stage.get(metric, math.nan))
            if not math.isclose(actual, expected, rel_tol=0.0, abs_tol=5e-10):
                raise ValueError(
                    f"stage {stage_name}: {metric}={actual} does not match "
                    f"conservative raw-log value {expected}"
                )
        if len(paths) > 1 and int(stage.get("repeat_count", 0)) != len(paths):
            raise ValueError(f"stage {stage_name}: repeat_count does not match logs")
        for metric, limit_key in LIMIT_MAP.items():
            if conservative[metric] > float(limits[limit_key]):
                raise ValueError(
                    f"stage {stage_name}: {metric}={conservative[metric]} "
                    f"exceeds {limit_key}={limits[limit_key]}"
                )
        checked.append(
            {
                "stage": stage_name,
                "accepted_runs": len(paths),
                "logs": [path.relative_to(workspace).as_posix() for path in paths],
                "conservative_metrics": conservative,
            }
        )

    payload = stages[-1]
    if payload.get("payload_status") != "generic simulation fixture; not measured cleaning-tool data":
        raise ValueError("payload evidence is not explicitly limited to a generic fixture")
    return {
        "result": "ARM_COUPLING_LADDER_EVIDENCE_PASS",
        "scope": "4 kg debug profile only",
        "stage_count": len(checked),
        "accepted_run_count": sum(row["accepted_runs"] for row in checked),
        "full_tensor_model_evidence": tensor_evidence,
        "stages": checked,
    }


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--workspace",
        type=Path,
        default=Path(__file__).resolve().parents[1],
    )
    parser.add_argument(
        "--report",
        type=Path,
        default=Path("analysis/arm_coupling_ladder_4kg_20260811.json"),
    )
    parser.add_argument("--output", type=Path)
    args = parser.parse_args()
    workspace = args.workspace.resolve()
    report_path = args.report if args.report.is_absolute() else workspace / args.report
    result = validate(workspace, report_path.resolve())
    serialized = json.dumps(result, indent=2, ensure_ascii=False) + "\n"
    if args.output:
        output = args.output if args.output.is_absolute() else workspace / args.output
        output.write_text(serialized, encoding="utf-8")
    print(serialized, end="")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
