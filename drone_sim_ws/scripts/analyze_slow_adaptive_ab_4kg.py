#!/usr/bin/env python3
"""Initialize and score the strict Base1 slow-adaptive flight campaign.

The campaign is deliberately larger than a convenient one-off comparison:
three fresh-PX4 OFF/ON pairs are required for both the nominal vehicle and a
physical 10 g gripper payload which is absent from the estimator URDF.  This
file never launches Gazebo.  It writes the immutable campaign manifest and,
after the runner has produced evidence, reduces the logs to one machine-
checkable verdict.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import math
import re
from datetime import datetime, timezone
from pathlib import Path
from statistics import mean


STATE_PREFIX = "BASE1_COMPENSATION_STATE "
ROS_TIME_RE = re.compile(r"\[(\d+(?:\.\d+)?)\].*BASE1_COMPENSATION_STATE ")
KEY_VALUE_RE = re.compile(r"([A-Za-z0-9_]+)=([-+0-9.e]+)")
SATURATION_RE = re.compile(r"motor_saturation_samples=(\d+)/(\d+)")
PRESET_RE = re.compile(r"ARM_PRESET_REACHED preset=(\S+)")
EXPECTED_GROUPS = ("nominal", "payload_10g_unmodeled")
EXPECTED_MODES = ("off", "on")
PAIR_COUNT = 3


def _sha256(path: Path) -> dict:
    payload = path.read_bytes()
    return {
        "path": str(path.resolve()),
        "size_bytes": len(payload),
        "sha256": hashlib.sha256(payload).hexdigest(),
    }


def expected_cases(campaign_id: str) -> list[dict]:
    cases = []
    ordinal = 0
    for group in EXPECTED_GROUPS:
        for pair in range(1, PAIR_COUNT + 1):
            for mode in EXPECTED_MODES:
                ordinal += 1
                cases.append(
                    {
                        "ordinal": ordinal,
                        "label": f"adaptive_ab_{campaign_id}_{group}_p{pair}_{mode}",
                        "group": group,
                        "pair": pair,
                        "adaptive_enabled": mode == "on",
                    }
                )
    return cases


def initialize_manifest(
    output: Path,
    campaign_id: str,
    workspace: Path,
    payload_urdf: Path,
) -> dict:
    nominal_urdf = workspace / "src/drone_arm_sim/urdf/my_drone_v3/my_drone_cad_debug_4kg.urdf"
    files = {
        "nominal_physical_and_estimator_urdf": nominal_urdf,
        "payload_10g_physical_urdf": payload_urdf,
        "flight_driver": workspace / "scripts/test_ros2_dds_arm_flight_pty.py",
        "backend_launcher": workspace / "scripts/wsl_start_ros2_dds_debug_4kg.sh",
        "overlay_launcher": workspace / "scripts/activate_base1_wrench_reallocator_overlay.sh",
        "reallocator_source": workspace / "src/drone_arm_sim/drone_arm_sim/base1_wrench_reallocator.py",
        "adaptive_source": workspace / "src/drone_arm_sim/drone_arm_sim/slow_adaptive_wrench.py",
        "payload_builder": workspace / "scripts/build_payload_urdf.py",
        "campaign_runner": workspace / "scripts/run_slow_adaptive_ab_4kg.sh",
        "campaign_analyzer": workspace / "scripts/analyze_slow_adaptive_ab_4kg.py",
        "motion_reference": workspace / "src/drone_arm_sim/config/so101_motion_reference_4kg.json",
        "flight_config": workspace / "src/drone_arm_sim/config/my_drone_v3_cad_debug_4kg.json",
        "flight_world": workspace / "src/drone_arm_sim/worlds/flight_world_debug_4kg.sdf",
        "px4_airframe": workspace / "px4/airframes/4027_gz_my_drone_octorotor_debug_4kg",
    }
    missing = [name for name, path in files.items() if not path.is_file()]
    if missing:
        raise ValueError(f"manifest source files missing: {missing}")
    manifest = {
        "schema": "my_drone.slow-adaptive-ab.v1",
        "campaign_id": campaign_id,
        "status": "NOT_RUN",
        "created_utc": datetime.now(timezone.utc).isoformat(),
        "common": {
            "model_mass_kg": 4.0,
            "gz_random_seed": 4027,
            "fresh_px4_workdir_each_case": True,
            "profile": "nose_forward_straight_4kg",
            "extension_s": 90.0,
            "hold_s": 8.0,
            "return_s": 120.0,
            "truth_hold_enabled": True,
            "truth_hold": {
                "xy_p": 0.80,
                "xy_d": 0.0,
                "z_p": 1.30,
                "z_d": 0.45,
                "velocity_filter_tau_s": 0.0,
                "xy_max_m_s": 0.08,
                "z_max_m_s": 0.12,
            },
            "predictive_torque_feedforward_enabled": False,
            "static_com_feedforward_gain": 0.0,
            "disturbance_observer_enabled": False,
            "base1_position_feedback_enabled": False,
            "adaptive_effort_residual": {
                "baseline_hold_s": 5.0,
                "integration_time_constant_s": 15.0,
                "leak_time_constant_s": 60.0,
                "warmup_s": 3.0,
                "force_deadband_n": 0.02,
                "torque_deadband_nm": 0.005,
                "horizontal_force_rate_n_s": 0.005,
                "vertical_force_rate_n_s": 0.003,
                "torque_rate_nm_s": 0.002,
                "eligibility_gates": {
                    "joint_velocity_rad_s": 0.005,
                    "joint_acceleration_rad_s2": 0.02,
                    "body_speed_m_s": 0.03,
                    "angular_rate_rad_s": 0.008726646,
                    "allocation_residual_norm": 0.005,
                },
            },
            "model_compensation": {
                "reaction_force_gain": 1.0,
                "reaction_torque_gain": 1.0,
                "gravity_torque_gain": 1.0,
                "gravity_torque_limit_nm": 1.35,
                "maximum_motor_delta_n": 1.60,
                "minimum_headroom_n": 0.25,
                "maximum_residual_norm": 0.02,
            },
            "adaptive_limits": {
                "horizontal_force_n": 0.06,
                "vertical_force_n": 0.04,
                "torque_nm": 0.02,
                "maximum_logged_regular_step_norm": 0.008,
            },
            "safety_gates": {
                "horizontal_m": 0.05,
                "altitude_m": 0.05,
                "tilt_deg": 1.0,
                "motor_saturation_samples": 0,
            },
            "acceptance": {
                "nominal_max_relative_degradation": 0.05,
                "payload_minimum_composite_improvement": 0.20,
                "composite": "mean of horizontal/0.05, altitude/0.05, tilt/1.0",
            },
        },
        "payload_fixture": {
            "physical_mass_kg": 0.010,
            "physical_attachment": "merged into gripper_link inertial and collision",
            "physical_urdf": str(payload_urdf.resolve()),
            "estimator_urdf": str(nominal_urdf.resolve()),
            "estimator_payload_mass_kg": 0.0,
            "interpretation": "the 10 g load exists in Gazebo but is deliberately absent from the estimator model",
        },
        "run_order": expected_cases(campaign_id),
        "files": {name: _sha256(path) for name, path in sorted(files.items())},
        "result": None,
    }
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(json.dumps(manifest, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")
    return manifest


def _vector(value: object) -> list[float] | None:
    if not isinstance(value, list) or len(value) != 6:
        return None
    try:
        result = [float(item) for item in value]
    except (TypeError, ValueError):
        return None
    return result if all(math.isfinite(item) for item in result) else None


def _norm(values: list[float]) -> float:
    return math.sqrt(sum(value * value for value in values))


def parse_trim_log(path: Path, adaptive: dict, limits: dict) -> dict:
    payload = path.read_bytes()
    states = []
    marker_count = 0
    for line in payload.decode("utf-8", errors="replace").splitlines():
        if STATE_PREFIX not in line:
            continue
        marker_count += 1
        try:
            state = json.loads(line.split(STATE_PREFIX, 1)[1])
        except json.JSONDecodeError:
            continue
        residual = _vector(state.get("adaptive_effort_residual_wrench_frd"))
        requested = _vector(state.get("adaptive_wrench_requested_frd"))
        delivered = _vector(state.get("adaptive_wrench_delivered_frd"))
        state_after = _vector(state.get("adaptive_wrench_state_after_backcalc_frd"))
        time_match = ROS_TIME_RE.search(line)
        if all(value is not None for value in (residual, requested, delivered, state_after)):
            states.append(
                {
                    "state": state,
                    "timestamp_s": float(time_match.group(1)) if time_match else None,
                    "residual": residual,
                    "requested": requested,
                    "delivered": delivered,
                    "state_after": state_after,
                }
            )

    instrumentation_present = bool(marker_count > 0 and len(states) == marker_count)
    output_norms = [_norm(item["state_after"]) for item in states]
    eligible = [bool(item["state"].get("adaptive_eligible")) for item in states]
    limited_indices = [
        index for index, item in enumerate(states)
        if bool(item["state"].get("allocation_limited"))
    ]
    backcalc_expected_indices = [
        index for index in limited_indices
        if _norm(states[index]["requested"]) > 1.0e-9
    ]
    backcalc_events = [
        index for index, item in enumerate(states)
        if item["state"].get("adaptive_backcalculated") is True
    ]
    delivery_errors = []
    backcalc_state_errors = []
    backcalc_semantics_errors = []
    for index, item in enumerate(states):
        state = item["state"]
        try:
            scale = float(state.get("feasibility_scale"))
        except (TypeError, ValueError):
            scale = math.nan
        expected_delivered = [scale * value for value in item["requested"]]
        delivery_errors.append(
            _norm([actual - expected for actual, expected in zip(item["delivered"], expected_delivered)])
            if math.isfinite(scale) else math.inf
        )
        expected_backcalc = index in backcalc_expected_indices
        actual_backcalc = state.get("adaptive_backcalculated") is True
        if expected_backcalc != actual_backcalc:
            backcalc_semantics_errors.append(index)
        if actual_backcalc:
            backcalc_state_errors.append(
                _norm([
                    actual - delivered
                    for actual, delivered in zip(item["state_after"], item["delivered"])
                ])
            )

    rate_violations = []
    regular_step_norms = []
    horizontal_rate = float(adaptive["horizontal_force_rate_n_s"])
    vertical_rate = float(adaptive["vertical_force_rate_n_s"])
    torque_rate = float(adaptive["torque_rate_nm_s"])
    for index in range(1, len(states)):
        previous = states[index - 1]
        current = states[index]
        # Immediate back-calculation is a commanded synchronisation to actual
        # delivery, not an integrator slew.  Validate it above and exclude it
        # from the ordinary residual-integrator rate test.
        if (
            previous["state"].get("adaptive_backcalculated") is True
            or current["state"].get("adaptive_backcalculated") is True
        ):
            continue
        if previous["timestamp_s"] is None or current["timestamp_s"] is None:
            rate_violations.append({"index": index, "reason": "missing_ros_timestamp"})
            continue
        dt_s = current["timestamp_s"] - previous["timestamp_s"]
        delta = [
            right - left
            for left, right in zip(previous["state_after"], current["state_after"])
        ]
        regular_step_norms.append(_norm(delta))
        tolerance = 1.0e-5
        if (
            dt_s <= 0.0
            or math.hypot(delta[0], delta[1]) > horizontal_rate * dt_s + tolerance
            or abs(delta[2]) > vertical_rate * dt_s + tolerance
            or _norm(delta[3:]) > torque_rate * dt_s + tolerance
        ):
            rate_violations.append({"index": index, "dt_s": dt_s, "delta": delta})
    zero_drops = 0
    for index in range(1, len(states)):
        if (
            eligible[index]
            and states[index]["state"].get("adaptive_backcalculated") is not True
            and output_norms[index - 1] > 1.0e-5
            and output_norms[index] < 1.0e-9
        ):
            zero_drops += 1

    horizontal_limit = float(limits["horizontal_force_n"])
    vertical_limit = float(limits["vertical_force_n"])
    torque_limit = float(limits["torque_nm"])
    limit_violations = 0
    for item in states:
        output = item["state_after"]
        if (
            math.hypot(output[0], output[1]) > horizontal_limit + 1.0e-6
            or abs(output[2]) > vertical_limit + 1.0e-6
            or _norm(output[3:]) > torque_limit + 1.0e-6
        ):
            limit_violations += 1
    maximum_delivery_error = max(delivery_errors, default=math.inf)
    maximum_backcalc_state_error = max(backcalc_state_errors, default=0.0)
    backcalc_pass = bool(
        instrumentation_present
        and maximum_delivery_error <= 1.0e-9
        and maximum_backcalc_state_error <= 1.0e-9
        and not backcalc_semantics_errors
    )
    return {
        "log": _sha256(path),
        "diagnostic_marker_count": marker_count,
        "required_instrumentation_present": instrumentation_present,
        "state_samples": len(states),
        "eligible_samples": sum(eligible),
        "ineligible_samples": len(eligible) - sum(eligible),
        "quasi_static_samples": sum(
            bool(item["state"].get("adaptive_quasi_static")) for item in states
        ),
        "baseline_ready_samples": sum(
            bool(item["state"].get("adaptive_baseline_ready")) for item in states
        ),
        "maximum_baseline_capture_s": max(
            (float(item["state"].get("adaptive_baseline_capture_s", 0.0)) for item in states),
            default=0.0,
        ),
        "maximum_warmup_s": max(
            (float(item["state"].get("adaptive_warmup_s", 0.0)) for item in states),
            default=0.0,
        ),
        "maximum_effort_residual_norm": max(
            (_norm(item["residual"]) for item in states), default=0.0
        ),
        "maximum_requested_norm": max(
            (_norm(item["requested"]) for item in states), default=0.0
        ),
        "maximum_delivered_norm": max(
            (_norm(item["delivered"]) for item in states), default=0.0
        ),
        "maximum_output_norm": max(output_norms, default=0.0),
        "maximum_output_components_abs": [
            max((abs(item["state_after"][index]) for item in states), default=0.0)
            for index in range(6)
        ],
        "limits": limits,
        "limit_violations": limit_violations,
        "continuity": {
            "maximum_regular_logged_step_norm": max(regular_step_norms, default=0.0),
            "component_rate_limits": {
                "horizontal_force_n_s": horizontal_rate,
                "vertical_force_n_s": vertical_rate,
                "torque_nm_s": torque_rate,
            },
            "rate_violation_count": len(rate_violations),
            "rate_violations": rate_violations,
            "eligible_zero_drop_events": zero_drops,
            "pass": instrumentation_present and zero_drops == 0 and not rate_violations,
        },
        "anti_windup_backcalculation": {
            "implementation": "immediate adaptive-share back-calculation to feasibility_scale * requested trim when allocation-limited",
            "allocation_limited_samples": len(limited_indices),
            "backcalculation_expected_samples": len(backcalc_expected_indices),
            "backcalculation_recorded_samples": len(backcalc_events),
            "maximum_requested_to_delivered_equation_error": maximum_delivery_error,
            "maximum_delivered_to_state_after_error": maximum_backcalc_state_error,
            "semantic_mismatch_indices": backcalc_semantics_errors,
            "status": "EXERCISED" if backcalc_expected_indices else "NOT_EXERCISED",
            "pass": backcalc_pass,
        },
    }


def parse_flight_log(path: Path, gates: dict) -> dict:
    payload = path.read_bytes()
    text = payload.decode("utf-8", errors="replace")
    metrics_line = next((line for line in text.splitlines() if "ARM_FLIGHT_METRICS " in line), "")
    values = {name: float(value) for name, value in KEY_VALUE_RE.findall(metrics_line)}
    saturation = SATURATION_RE.search(metrics_line)
    presets = set(PRESET_RE.findall(text))
    required = {"flight_straight_forward", "retracted"}
    horizontal = values.get("horizontal_drift_m", math.inf)
    altitude = values.get("altitude_span_m", math.inf)
    tilt = values.get("max_truth_tilt_deg", math.inf)
    saturation_count = int(saturation.group(1)) if saturation else -1
    safety_pass = bool(
        "DDS_ARM_FLIGHT_PASS" in text
        and required.issubset(presets)
        and "failsafe=True" not in text
        and horizontal <= float(gates["horizontal_m"])
        and altitude <= float(gates["altitude_m"])
        and tilt <= float(gates["tilt_deg"])
        and saturation_count == int(gates["motor_saturation_samples"])
    )
    composite = mean((
        horizontal / float(gates["horizontal_m"]),
        altitude / float(gates["altitude_m"]),
        tilt / float(gates["tilt_deg"]),
    ))
    return {
        "log": _sha256(path),
        "horizontal_m": horizontal,
        "altitude_m": altitude,
        "max_tilt_deg": tilt,
        "motor_saturation_samples": saturation_count,
        "motor_sample_count": int(saturation.group(2)) if saturation else -1,
        "required_presets_reached": sorted(required & presets),
        "failsafe_seen": "failsafe=True" in text,
        "dds_pass_marker": "DDS_ARM_FLIGHT_PASS" in text,
        "normalized_composite": composite,
        "safety_pass": safety_pass,
    }


def _group_means(cases: list[dict], group: str, enabled: bool) -> dict:
    selected = [case for case in cases if case["group"] == group and case["adaptive_enabled"] is enabled]
    return {
        key: mean(case["flight"][key] for case in selected)
        for key in ("horizontal_m", "altitude_m", "max_tilt_deg", "normalized_composite")
    }


def analyze_campaign(manifest_path: Path, campaign_dir: Path, output: Path) -> tuple[dict, int]:
    manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
    campaign_id = manifest.get("campaign_id")
    manifest_structure_valid = bool(
        manifest.get("schema") == "my_drone.slow-adaptive-ab.v1"
        and isinstance(campaign_id, str)
        and manifest.get("run_order") == expected_cases(campaign_id)
        and manifest.get("common", {}).get("gz_random_seed") == 4027
        and manifest.get("common", {}).get("fresh_px4_workdir_each_case") is True
        and manifest.get("common", {}).get("truth_hold", {}).get("xy_d") == 0.0
    )
    if not manifest_structure_valid:
        report = {
            "schema": "my_drone.slow-adaptive-ab-result.v1",
            "campaign_id": campaign_id,
            "status": "INVALID_EVIDENCE",
            "accepted": False,
            "reason": "campaign manifest structure/order/seed/P-only hold is invalid",
        }
        output.parent.mkdir(parents=True, exist_ok=True)
        output.write_text(json.dumps(report, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")
        return report, 2
    expected = manifest.get("run_order", [])
    missing = []
    for case in expected:
        label = case["label"]
        for suffix in ("flight.log", "base1_reallocator.log", "px4_workdir.path"):
            path = campaign_dir / f"{label}_{suffix}"
            if not path.is_file() or path.stat().st_size == 0:
                missing.append(str(path.resolve()))
    if missing:
        completed = sum(
            (campaign_dir / f"{case['label']}_flight.log").is_file()
            for case in expected
        )
        report = {
            "schema": "my_drone.slow-adaptive-ab-result.v1",
            "campaign_id": manifest.get("campaign_id"),
            "status": "NOT_RUN" if completed == 0 else "INCOMPLETE",
            "expected_case_count": len(expected),
            "completed_flight_log_count": completed,
            "missing_evidence": missing,
            "accepted": False,
            "reason": "all 12 fresh-PX4 cases must be physically run before adaptive trim can be judged",
        }
        output.parent.mkdir(parents=True, exist_ok=True)
        output.write_text(json.dumps(report, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")
        return report, 3

    changed_files = []
    for name, record in manifest.get("files", {}).items():
        path = Path(record.get("path", ""))
        if not path.is_file():
            changed_files.append(f"{name}:missing")
            continue
        current = _sha256(path)
        if (
            current["size_bytes"] != record.get("size_bytes")
            or current["sha256"] != record.get("sha256")
        ):
            changed_files.append(f"{name}:content_changed")
    if changed_files:
        report = {
            "schema": "my_drone.slow-adaptive-ab-result.v1",
            "campaign_id": manifest.get("campaign_id"),
            "status": "INVALID_EVIDENCE",
            "accepted": False,
            "changed_manifest_files": changed_files,
            "reason": "source or model content changed after campaign initialization",
        }
        output.write_text(json.dumps(report, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")
        return report, 2

    limits = manifest["common"]["adaptive_limits"]
    adaptive_config = manifest["common"]["adaptive_effort_residual"]
    gates = manifest["common"]["safety_gates"]
    cases = []
    workdirs = []
    for expected_case in expected:
        label = expected_case["label"]
        case = dict(expected_case)
        case["flight"] = parse_flight_log(campaign_dir / f"{label}_flight.log", gates)
        case["trim"] = parse_trim_log(
            campaign_dir / f"{label}_base1_reallocator.log", adaptive_config, limits
        )
        workdir = (campaign_dir / f"{label}_px4_workdir.path").read_text(encoding="utf-8").strip()
        case["px4_workdir"] = workdir
        workdirs.append(workdir)
        adaptive_enabled = case["adaptive_enabled"]
        if adaptive_enabled:
            trim_non_degenerate = bool(
                case["trim"]["state_samples"] >= 10
                and case["trim"]["eligible_samples"] >= 3
                and case["trim"]["baseline_ready_samples"] >= 3
                and case["trim"]["maximum_baseline_capture_s"]
                >= adaptive_config["baseline_hold_s"]
                and case["trim"]["maximum_warmup_s"] >= adaptive_config["warmup_s"]
                and case["trim"]["maximum_effort_residual_norm"] > 1.0e-5
                and case["trim"]["maximum_output_norm"] > 1.0e-5
            )
        else:
            trim_non_degenerate = bool(
                case["trim"]["eligible_samples"] == 0
                and case["trim"]["maximum_output_norm"] <= 1.0e-9
            )
        case["trim_non_degenerate"] = trim_non_degenerate
        case["case_pass"] = bool(
            case["flight"]["safety_pass"]
            and trim_non_degenerate
            and case["trim"]["required_instrumentation_present"]
            and case["trim"]["limit_violations"] == 0
            and case["trim"]["continuity"]["pass"]
            and case["trim"]["anti_windup_backcalculation"]["pass"]
        )
        cases.append(case)

    fresh_px4_proven = len(set(workdirs)) == len(workdirs) and all(
        item.startswith("/tmp/my_drone_px4_work.") for item in workdirs
    )
    nominal_off = _group_means(cases, "nominal", False)
    nominal_on = _group_means(cases, "nominal", True)
    payload_off = _group_means(cases, "payload_10g_unmodeled", False)
    payload_on = _group_means(cases, "payload_10g_unmodeled", True)
    eps = {"horizontal_m": 0.001, "altitude_m": 0.001, "max_tilt_deg": 0.05, "normalized_composite": 0.01}
    relative_limit = float(manifest["common"]["acceptance"]["nominal_max_relative_degradation"])
    nominal_non_degraded = all(
        nominal_on[key] <= nominal_off[key] * (1.0 + relative_limit) + eps[key]
        for key in nominal_off
    )
    payload_improvement = (
        0.0
        if payload_off["normalized_composite"] <= 1.0e-12
        else 1.0 - payload_on["normalized_composite"] / payload_off["normalized_composite"]
    )
    payload_components_non_degraded = all(
        payload_on[key] <= payload_off[key] * 1.05 + eps[key]
        for key in ("horizontal_m", "altitude_m", "max_tilt_deg")
    )
    payload_improved = bool(
        payload_improvement >= float(manifest["common"]["acceptance"]["payload_minimum_composite_improvement"])
        and payload_components_non_degraded
    )
    accepted = bool(
        all(case["case_pass"] for case in cases)
        and fresh_px4_proven
        and nominal_non_degraded
        and payload_improved
    )
    report = {
        "schema": "my_drone.slow-adaptive-ab-result.v1",
        "campaign_id": manifest.get("campaign_id"),
        "status": "ACCEPTED" if accepted else "REJECTED",
        "accepted": accepted,
        "case_count": len(cases),
        "same_gazebo_seed": manifest["common"]["gz_random_seed"],
        "fresh_unique_px4_workdirs_proven": fresh_px4_proven,
        "nominal": {
            "off_mean": nominal_off,
            "on_mean": nominal_on,
            "non_degraded": nominal_non_degraded,
        },
        "payload_10g_unmodeled": {
            "off_mean": payload_off,
            "on_mean": payload_on,
            "composite_improvement_fraction": payload_improvement,
            "minimum_required_improvement_fraction": manifest["common"]["acceptance"]["payload_minimum_composite_improvement"],
            "components_non_degraded": payload_components_non_degraded,
            "improvement_pass": payload_improved,
        },
        "non_degeneracy_pass": all(case["trim_non_degenerate"] for case in cases),
        "all_safety_and_trim_gates_pass": all(case["case_pass"] for case in cases),
        "cases": cases,
    }
    output.write_text(json.dumps(report, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")
    return report, 0 if accepted else 1


def main() -> int:
    parser = argparse.ArgumentParser()
    subparsers = parser.add_subparsers(dest="command", required=True)
    initialize = subparsers.add_parser("init")
    initialize.add_argument("--output", required=True, type=Path)
    initialize.add_argument("--campaign-id", required=True)
    initialize.add_argument("--workspace", required=True, type=Path)
    initialize.add_argument("--payload-urdf", required=True, type=Path)
    analyze = subparsers.add_parser("analyze")
    analyze.add_argument("--manifest", required=True, type=Path)
    analyze.add_argument("--campaign-dir", required=True, type=Path)
    analyze.add_argument("--output", required=True, type=Path)
    args = parser.parse_args()
    if args.command == "init":
        manifest = initialize_manifest(args.output, args.campaign_id, args.workspace, args.payload_urdf)
        print(f"ADAPTIVE_AB_INITIALIZED status={manifest['status']} cases={len(manifest['run_order'])}")
        return 0
    report, status = analyze_campaign(args.manifest, args.campaign_dir, args.output)
    print(f"ADAPTIVE_AB_RESULT status={report['status']} accepted={report['accepted']}")
    return status


if __name__ == "__main__":
    raise SystemExit(main())
