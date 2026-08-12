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
import hashlib
import json
import math
import re
from pathlib import Path


METRICS_RE = re.compile(
    r"ARM_FLIGHT_METRICS[^\r\n]*?"
    r"horizontal_drift_m=(?P<drift>[-+0-9.e]+)\s+"
    r"altitude_span_m=(?P<altitude>[-+0-9.e]+)[^\r\n]*?"
    r"\ssamples=(?P<samples>\d+)"
)
METRICS_LINE_RE = re.compile(r"ARM_FLIGHT_METRICS\s+(?P<body>[^\r\n]+)")
METRIC_VALUE_RE = re.compile(r"(?P<name>[A-Za-z0-9_]+)=(?P<value>[-+0-9.e]+)")
SATURATION_RE = re.compile(r"motor_saturation_samples=(?P<count>\d+)/(?P<total>\d+)")
PRESET_RE = re.compile(r"ARM_PRESET_REACHED preset=(\S+) max_error=([-+0-9.e]+)")
STATE_RE = re.compile(
    r"STATE arm=(\d+) nav=(\d+) NED=\(([-+0-9.e]+),([-+0-9.e]+),([-+0-9.e]+)\)"
)


def _manifest_identity(path: Path, payload: bytes) -> dict:
    return {
        "path": str(path.resolve()),
        "size_bytes": len(payload),
        "sha256": hashlib.sha256(payload).hexdigest(),
    }


def compare_case_manifests(off_path: Path, on_path: Path) -> dict:
    """Require identical hashed setup with only observer_enabled toggled."""
    errors: list[str] = []
    manifests = []
    identities = []
    for label, path in (("off", off_path), ("on", on_path)):
        payload = path.read_bytes()
        identities.append(_manifest_identity(path, payload))
        try:
            manifest = json.loads(payload.decode("utf-8"))
        except (UnicodeDecodeError, json.JSONDecodeError) as exc:
            return {
                "valid": False,
                "errors": [f"{label} manifest is not valid UTF-8 JSON: {exc}"],
                "manifests": identities,
            }
        manifests.append(manifest)
    off, on = manifests
    if off.get("schema") != 1 or on.get("schema") != 1:
        errors.append("both case manifests must use schema 1")
    if off.get("observer_enabled") is not False:
        errors.append("OFF manifest does not disable the observer")
    if on.get("observer_enabled") is not True:
        errors.append("ON manifest does not enable the observer")
    if off.get("common") != on.get("common"):
        errors.append("OFF and ON common configuration manifests differ")

    common = off.get("common") if isinstance(off.get("common"), dict) else {}
    file_records = common.get("files") if isinstance(common, dict) else None
    if not isinstance(file_records, dict) or not file_records:
        errors.append("common configuration has no hashed files")
    else:
        for name, record in file_records.items():
            if not isinstance(record, dict):
                errors.append(f"file record {name} is invalid")
                continue
            path = Path(str(record.get("path", "")))
            if not path.is_file():
                errors.append(f"hashed source file is missing: {name}")
                continue
            payload = path.read_bytes()
            if len(payload) != record.get("size_bytes"):
                errors.append(f"hashed source file size changed: {name}")
            if hashlib.sha256(payload).hexdigest() != record.get("sha256"):
                errors.append(f"hashed source file content changed: {name}")
    return {
        "valid": not errors,
        "errors": errors,
        "manifests": identities,
        "common_configuration": common,
        "only_observer_enable_differs": bool(not errors),
    }


def parse_candidate_runtime(path: Path, required_marker: str) -> dict:
    """Prove that the candidate process stayed alive and published output."""
    payload = path.read_bytes()
    text = payload.decode("utf-8", errors="replace")
    marker_count = text.count(required_marker)
    active_marker_count = 0
    maximum_estimated_torque_nm = 0.0
    for line in text.splitlines():
        if required_marker not in line:
            continue
        payload_text = line.split(required_marker, 1)[1].strip()
        try:
            state = json.loads(payload_text)
            torque = state.get("estimated_disturbance_torque_frd_nm")
            if state.get("active") is True:
                active_marker_count += 1
            if isinstance(torque, list) and len(torque) == 3:
                values = [float(value) for value in torque]
                if all(math.isfinite(value) for value in values):
                    maximum_estimated_torque_nm = max(
                        maximum_estimated_torque_nm,
                        math.sqrt(sum(value * value for value in values)),
                    )
        except (TypeError, ValueError, json.JSONDecodeError):
            continue
    observer_died = bool(
        re.search(
            r"arm_disturbance_observer[^\r\n]*(?:process has died|exit code)",
            text,
            flags=re.IGNORECASE,
        )
    )
    cli_failure = bool(
        re.search(
            r"arm_disturbance_observer[^\r\n]*unrecognized arguments",
            text,
            flags=re.IGNORECASE,
        )
    )
    valid = bool(
        marker_count > 0
        and active_marker_count > 0
        and maximum_estimated_torque_nm > 1.0e-6
        and not observer_died
        and not cli_failure
    )
    return {
        "log": str(path.resolve()),
        "log_size_bytes": len(payload),
        "log_sha256": hashlib.sha256(payload).hexdigest(),
        "required_marker": required_marker,
        "marker_count": marker_count,
        "active_marker_count": active_marker_count,
        "maximum_estimated_torque_nm": maximum_estimated_torque_nm,
        "observer_process_died": observer_died,
        "cli_argument_failure": cli_failure,
        "valid": valid,
        "interpretation": (
            "candidate runtime published active, nonzero observer diagnostics"
            if valid
            else "candidate runtime was not proven active with nonzero output; flight deltas are not attributable to the observer"
        ),
    }


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


def parse_log(
    path: Path,
    required_presets: tuple[str, ...],
    horizontal_gate_m: float,
    altitude_gate_m: float,
    tilt_gate_deg: float,
    arm_torque_gate_nm: float,
) -> dict:
    payload = path.read_bytes()
    text = payload.decode("utf-8", errors="replace")
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
    metric_line_match = METRICS_LINE_RE.search(text)
    metric_values: dict[str, float] = {}
    saturation_count = None
    saturation_total = None
    if metric_line_match is not None:
        metric_body = metric_line_match.group("body")
        metric_values = {
            match.group("name"): float(match.group("value"))
            for match in METRIC_VALUE_RE.finditer(metric_body)
        }
        saturation = SATURATION_RE.search(metric_body)
        if saturation is not None:
            saturation_count = int(saturation.group("count"))
            saturation_total = int(saturation.group("total"))
    max_truth_tilt_deg = metric_values.get("max_truth_tilt_deg")
    rms_truth_tilt_deg = metric_values.get("rms_truth_tilt_deg")
    max_arm_torque_nm = metric_values.get("max_arm_torque_nm")
    strict_diagnostics_present = all(
        value is not None
        for value in (
            max_truth_tilt_deg,
            rms_truth_tilt_deg,
            max_arm_torque_nm,
            saturation_count,
            saturation_total,
        )
    )
    strict_diagnostics_ok = bool(
        strict_diagnostics_present
        and saturation_count == 0
        and max_truth_tilt_deg <= tilt_gate_deg
        and max_arm_torque_nm <= arm_torque_gate_nm
    )
    return {
        "log": str(path.resolve()),
        "log_size_bytes": len(payload),
        "log_sha256": hashlib.sha256(payload).hexdigest(),
        "horizontal_drift_m": drift,
        "altitude_span_m": altitude,
        "samples": samples,
        "metrics_source": metrics_source,
        "max_com_shift_m": metric_values.get("max_com_shift_m"),
        "max_inertia_diag_change_kg_m2": metric_values.get(
            "max_inertia_diag_change_kg_m2"
        ),
        "max_arm_force_n": metric_values.get("max_arm_force_n"),
        "max_arm_torque_nm": max_arm_torque_nm,
        "motor_saturation_samples": saturation_count,
        "motor_sample_count": saturation_total,
        "motor_saturation_rate": metric_values.get("motor_saturation_rate"),
        "max_truth_tilt_deg": max_truth_tilt_deg,
        "rms_truth_tilt_deg": rms_truth_tilt_deg,
        "strict_diagnostics_present": strict_diagnostics_present,
        "strict_diagnostics_ok": strict_diagnostics_ok,
        "pass_marker": "DDS_ARM_FLIGHT_PASS" in text,
        # Do not mistake the test summary's ``no_failsafe=True`` for an
        # observed PX4 failsafe state.
        "failsafe_seen": bool(re.search(r"(?<![A-Za-z_])failsafe=True", text)),
        "safety_abort": "ARM_FLIGHT_ABORTED" in text,
        "preset_max_error_rad": presets,
        "all_required_presets_reached": all(
            name in presets for name in required_presets
        ),
        "within_current_gate": (
            drift <= horizontal_gate_m
            and altitude <= altitude_gate_m
            and strict_diagnostics_ok
            and not bool(re.search(r"(?<![A-Za-z_])failsafe=True", text))
            and "ARM_FLIGHT_ABORTED" not in text
        ),
    }


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--feedforward-off", type=Path, required=True)
    parser.add_argument("--feedforward-on", type=Path, required=True)
    parser.add_argument(
        "--required-presets",
        nargs="+",
        default=("flight_micro_a", "flight_micro_b", "retracted"),
        help="preset markers required for this exact trajectory",
    )
    parser.add_argument("--horizontal-gate", type=float, default=1.5)
    parser.add_argument("--altitude-gate", type=float, default=1.5)
    parser.add_argument("--tilt-gate", type=float, default=3.0)
    parser.add_argument("--arm-torque-gate", type=float, default=0.5)
    parser.add_argument(
        "--candidate-runtime-log",
        type=Path,
        help="optional candidate backend log used to prove the experimental process ran",
    )
    parser.add_argument(
        "--candidate-runtime-required-marker",
        default="ARM_DOB_STATE",
    )
    parser.add_argument("--off-case-manifest", type=Path)
    parser.add_argument("--on-case-manifest", type=Path)
    parser.add_argument(
        "--require-improvement",
        action="store_true",
        help="return nonzero unless both runs pass and ON improves every principal metric",
    )
    parser.add_argument("--output", type=Path)
    args = parser.parse_args()

    required_presets = tuple(args.required_presets)
    off = parse_log(
        args.feedforward_off,
        required_presets,
        args.horizontal_gate,
        args.altitude_gate,
        args.tilt_gate,
        args.arm_torque_gate,
    )
    on = parse_log(
        args.feedforward_on,
        required_presets,
        args.horizontal_gate,
        args.altitude_gate,
        args.tilt_gate,
        args.arm_torque_gate,
    )
    principal_metrics = (
        "horizontal_drift_m",
        "altitude_span_m",
        "max_truth_tilt_deg",
        "rms_truth_tilt_deg",
    )
    improves_all_principal_metrics = all(
        off[name] is not None
        and on[name] is not None
        and on[name] <= off[name]
        for name in principal_metrics
    )
    runtime_evidence = (
        parse_candidate_runtime(
            args.candidate_runtime_log,
            args.candidate_runtime_required_marker,
        )
        if args.candidate_runtime_log is not None
        else None
    )
    candidate_runtime_valid = bool(
        runtime_evidence is None or runtime_evidence["valid"]
    )
    manifests_requested = bool(
        args.off_case_manifest is not None or args.on_case_manifest is not None
    )
    if manifests_requested and (
        args.off_case_manifest is None or args.on_case_manifest is None
    ):
        pair_configuration_evidence = {
            "valid": False,
            "errors": ["both OFF and ON case manifests are required"],
        }
    elif manifests_requested:
        pair_configuration_evidence = compare_case_manifests(
            args.off_case_manifest, args.on_case_manifest
        )
    else:
        pair_configuration_evidence = None
    pair_configuration_valid = bool(
        pair_configuration_evidence is None
        or pair_configuration_evidence["valid"]
    )
    report = {
        "feedforward_off": off,
        "feedforward_on": on,
        "delta_on_minus_off": {
            "horizontal_drift_m": on["horizontal_drift_m"]
            - off["horizontal_drift_m"],
            "altitude_span_m": on["altitude_span_m"]
            - off["altitude_span_m"],
            "max_truth_tilt_deg": (
                None
                if on["max_truth_tilt_deg"] is None
                or off["max_truth_tilt_deg"] is None
                else on["max_truth_tilt_deg"] - off["max_truth_tilt_deg"]
            ),
            "rms_truth_tilt_deg": (
                None
                if on["rms_truth_tilt_deg"] is None
                or off["rms_truth_tilt_deg"] is None
                else on["rms_truth_tilt_deg"] - off["rms_truth_tilt_deg"]
            ),
        },
        "candidate_runtime_evidence": runtime_evidence,
        "pair_configuration_evidence": pair_configuration_evidence,
        "comparison": {
            "required_presets": list(required_presets),
            "horizontal_gate_m": args.horizontal_gate,
            "altitude_gate_m": args.altitude_gate,
            "tilt_gate_deg": args.tilt_gate,
            "arm_torque_gate_nm": args.arm_torque_gate,
            "candidate_runtime_valid": candidate_runtime_valid,
            "pair_configuration_valid": pair_configuration_valid,
            "effect_evaluated": (
                candidate_runtime_valid and pair_configuration_valid
            ),
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
                candidate_runtime_valid
                and pair_configuration_valid
                and
                off["within_current_gate"]
                and on["within_current_gate"]
                and off["all_required_presets_reached"]
                and on["all_required_presets_reached"]
                and off["pass_marker"]
                and on["pass_marker"]
            ),
            "improves_all_principal_metrics": (
                improves_all_principal_metrics
                if candidate_runtime_valid and pair_configuration_valid
                else None
            ),
            "candidate_accepted": bool(
                candidate_runtime_valid
                and pair_configuration_valid
                and
                off["within_current_gate"]
                and on["within_current_gate"]
                and off["all_required_presets_reached"]
                and on["all_required_presets_reached"]
                and off["pass_marker"]
                and on["pass_marker"]
                and improves_all_principal_metrics
            ),
            "interpretation": (
                "candidate runtime or paired configuration invalid; effect was not evaluated"
                if not candidate_runtime_valid or not pair_configuration_valid
                else (
                    "candidate improved drift, altitude, maximum tilt and RMS tilt"
                    if improves_all_principal_metrics
                    else "candidate did not improve every principal metric"
                )
            ),
        },
    }
    rendered = json.dumps(report, indent=2, ensure_ascii=False)
    print(rendered)
    if args.output:
        args.output.write_text(rendered + "\n", encoding="utf-8")
    if args.require_improvement and not report["comparison"]["candidate_accepted"]:
        return 3
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
