import json
from pathlib import Path

from analyze_base1_arm_state_hover import compare, parse_log


def _case(horizontal=0.1, height=0.05, vz=0.02, motor=450.0):
    return {
        "pass": True,
        "metrics": {
            "max_horizontal_error_m": horizontal,
            "truth_height_peak_to_peak_m": height,
            "px4_vertical_speed_abs_p90_m_s": vz,
            "mean_motor_output": motor,
            "motor_saturation_fraction": 0.0,
            "failsafe_seen": False,
        },
    }


def test_parse_last_metrics_and_pass(tmp_path: Path):
    log = tmp_path / "hover.log"
    metrics = _case()["metrics"]
    log.write_text(
        "DDS_DYNAMIC_HOVER_METRICS " + json.dumps(metrics) + "\nDDS_DYNAMIC_HOVER_PASS\n",
        encoding="utf-8",
    )
    parsed = parse_log(log)
    assert parsed["pass"]
    assert parsed["exists"]
    assert parsed["metrics"] == metrics


def test_missing_log_is_reported_as_failure(tmp_path: Path):
    parsed = parse_log(tmp_path / "missing.log")
    assert not parsed["exists"]
    assert not parsed["pass"]
    assert parsed["metrics"] == {}


def test_equal_hover_states_pass():
    assert compare(_case(), _case())["pass"]


def test_height_regression_fails():
    report = compare(_case(height=0.04), _case(height=0.08))
    assert not report["pass"]
    assert not report["gates"]["height_p2p_regression_le_0p03_m"]


def test_runtime_failure_cannot_be_hidden_by_metrics():
    static = _case()
    static["pass"] = False
    report = compare(_case(), static)
    assert not report["pass"]
    assert not report["gates"]["both_runtime_pass"]
