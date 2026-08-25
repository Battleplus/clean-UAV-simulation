#!/usr/bin/env python3
"""Pure tests for the strict adaptive A/B manifest and scoring gates."""

from __future__ import annotations

import importlib.util
import json
from pathlib import Path
import tempfile


SCRIPT = Path(__file__).with_name("analyze_slow_adaptive_ab_4kg.py")
SPEC = importlib.util.spec_from_file_location("adaptive_ab", SCRIPT)
MODULE = importlib.util.module_from_spec(SPEC)
assert SPEC.loader is not None
SPEC.loader.exec_module(MODULE)


def _flight(horizontal: float, altitude: float, tilt: float) -> str:
    return (
        "ARM_PRESET_REACHED preset=flight_straight_forward max_error=0.001\n"
        "ARM_PRESET_REACHED preset=retracted max_error=0.001\n"
        f"ARM_FLIGHT_METRICS horizontal_drift_m={horizontal} altitude_span_m={altitude} "
        f"max_truth_tilt_deg={tilt} motor_saturation_samples=0/200 samples=200\n"
        "DDS_ARM_FLIGHT_PASS\n"
    )


def _trim(enabled: bool) -> str:
    lines = []
    for index in range(12):
        eligible = enabled and index >= 2
        value = min(0.01, max(0, index - 2) * 0.001) if enabled else 0.0
        state = {
            "adaptive_eligible": eligible,
            "adaptive_quasi_static": True,
            "adaptive_baseline_ready": True,
            "adaptive_baseline_capture_s": 5.0,
            "adaptive_effort_residual_wrench_frd": [0.04, 0, 0, 0, 0, 0],
            "adaptive_wrench_frd": [value, 0, 0, 0, 0, 0],
            "adaptive_wrench_requested_frd": [value, 0, 0, 0, 0, 0],
            "adaptive_wrench_delivered_frd": [value, 0, 0, 0, 0, 0],
            "adaptive_wrench_state_after_backcalc_frd": [value, 0, 0, 0, 0, 0],
            "adaptive_backcalculated": False,
            "adaptive_warmup_s": float(max(0, index - 1)) if enabled else 0.0,
            "allocation_limited": False,
            "feasibility_scale": 1.0,
        }
        lines.append(f"[{float(index):.1f}] test " + MODULE.STATE_PREFIX + json.dumps(state))
    return "\n".join(lines) + "\n"


def _manifest(campaign: str) -> dict:
    return {
        "schema": "my_drone.slow-adaptive-ab.v1",
        "campaign_id": campaign,
        "common": {
            "gz_random_seed": 4027,
            "fresh_px4_workdir_each_case": True,
            "truth_hold": {"xy_d": 0.0},
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
            },
        },
        "run_order": MODULE.expected_cases(campaign),
    }


def test_missing_campaign_is_explicitly_not_run():
    with tempfile.TemporaryDirectory() as directory:
        root = Path(directory)
        manifest = root / "manifest.json"
        output = root / "result.json"
        manifest.write_text(json.dumps(_manifest("unit")), encoding="utf-8")
        report, status = MODULE.analyze_campaign(manifest, root, output)
        assert status == 3
        assert report["status"] == "NOT_RUN"
        assert report["accepted"] is False


def test_runner_uses_real_eight_second_nose_forward_hold():
    runner = Path(__file__).with_name("run_slow_adaptive_ab_4kg.sh").read_text(
        encoding="utf-8"
    )
    driver = Path(__file__).with_name("test_ros2_dds_arm_flight_pty.py").read_text(
        encoding="utf-8"
    )
    assert "ARM_NOSE_FORWARD_HOLD_S=8" in runner
    assert '"ARM_NOSE_FORWARD_HOLD_S", "5"' in driver
    assert "return_start_s = 5.0 + float(arm_duration) + float(arm_hold_duration)" in driver
    assert 'payload_10g_unmodeled' in runner
    assert '--mass-kg 0.010' in runner
    assert 'colcon build --symlink-install --packages-select drone_arm_sim px4_ros2_control' in runner
    assert "ADAPTIVE_AB_NOT_RUN reason=colcon_build_failed" in runner


def test_backcalculation_evidence_requires_delivered_state_match():
    with tempfile.TemporaryDirectory() as directory:
        path = Path(directory) / "trim.log"
        state = {
            "adaptive_eligible": False,
            "adaptive_quasi_static": True,
            "adaptive_baseline_ready": True,
            "adaptive_baseline_capture_s": 5.0,
            "adaptive_warmup_s": 3.0,
            "adaptive_effort_residual_wrench_frd": [0.04, 0, 0, 0, 0, 0],
            "adaptive_wrench_requested_frd": [0.02, 0, 0, 0, 0, 0],
            "adaptive_wrench_delivered_frd": [0.01, 0, 0, 0, 0, 0],
            "adaptive_wrench_state_after_backcalc_frd": [0.01, 0, 0, 0, 0, 0],
            "adaptive_backcalculated": True,
            "allocation_limited": True,
            "feasibility_scale": 0.5,
        }
        path.write_text(
            "[1.0] test " + MODULE.STATE_PREFIX + json.dumps(state) + "\n",
            encoding="utf-8",
        )
        report = MODULE.parse_trim_log(
            path,
            _manifest("unit")["common"]["adaptive_effort_residual"],
            _manifest("unit")["common"]["adaptive_limits"],
        )
        assert report["anti_windup_backcalculation"]["status"] == "EXERCISED"
        assert report["anti_windup_backcalculation"]["pass"] is True


def test_three_pair_campaign_accepts_only_real_non_degenerate_improvement():
    with tempfile.TemporaryDirectory() as directory:
        root = Path(directory)
        manifest = root / "manifest.json"
        output = root / "result.json"
        data = _manifest("unit")
        manifest.write_text(json.dumps(data), encoding="utf-8")
        for case in data["run_order"]:
            label = case["label"]
            enabled = case["adaptive_enabled"]
            if case["group"] == "nominal":
                metrics = (0.010, 0.010, 0.20) if not enabled else (0.0102, 0.0101, 0.20)
            else:
                metrics = (0.030, 0.020, 0.50) if not enabled else (0.020, 0.015, 0.35)
            (root / f"{label}_flight.log").write_text(_flight(*metrics), encoding="utf-8")
            (root / f"{label}_base1_reallocator.log").write_text(_trim(enabled), encoding="utf-8")
            (root / f"{label}_px4_workdir.path").write_text(
                f"/tmp/my_drone_px4_work.{case['ordinal']:06d}\n", encoding="utf-8"
            )
        report, status = MODULE.analyze_campaign(manifest, root, output)
        assert status == 0
        assert report["status"] == "ACCEPTED"
        assert report["fresh_unique_px4_workdirs_proven"] is True
        assert report["non_degeneracy_pass"] is True
        assert report["payload_10g_unmodeled"]["composite_improvement_fraction"] >= 0.20


def test_zero_output_on_group_is_rejected_as_degenerate():
    with tempfile.TemporaryDirectory() as directory:
        root = Path(directory)
        manifest = root / "manifest.json"
        output = root / "result.json"
        data = _manifest("unit")
        manifest.write_text(json.dumps(data), encoding="utf-8")
        for case in data["run_order"]:
            label = case["label"]
            enabled = case["adaptive_enabled"]
            metrics = (0.030, 0.020, 0.50) if not enabled else (0.020, 0.015, 0.35)
            (root / f"{label}_flight.log").write_text(_flight(*metrics), encoding="utf-8")
            (root / f"{label}_base1_reallocator.log").write_text(_trim(False), encoding="utf-8")
            (root / f"{label}_px4_workdir.path").write_text(
                f"/tmp/my_drone_px4_work.{case['ordinal']:06d}\n", encoding="utf-8"
            )
        report, status = MODULE.analyze_campaign(manifest, root, output)
        assert status == 1
        assert report["accepted"] is False
        assert report["non_degeneracy_pass"] is False
