from pathlib import Path

from analyze_base1_no_compensation import build_report


def test_pass_report_parses_metrics(tmp_path: Path) -> None:
    log = tmp_path / "run.log"
    log.write_text(
        "STATE arm=1 nav=4 NED=(0,0,0.47)\n"
        "STATE arm=2 nav=14 NED=(0,0,-0.53)\n"
        "OFFBOARD mode and ARM commands sent\n"
        "PX4 command ack: command=21 result=0\n"
        "LANDING_DISARMED_CONFIRMED\n"
        "ARM_FLIGHT_METRICS horizontal_drift_m=0.108 altitude_span_m=0.199 "
        "max_arm_torque_nm=0.156 motor_saturation_rate=0.000000 "
        "max_truth_tilt_deg=1.345 rms_truth_tilt_deg=0.500\n"
        "ARM_FLIGHT_CYCLE_METRICS label=demo_extended horizontal_drift_m=0.108\n"
        "ARM_FLIGHT_CYCLE_METRICS label=retracted horizontal_drift_m=0.080\n"
        "DDS_ARM_FLIGHT_PASS\n",
        encoding="utf-8",
    )
    report = build_report([log])
    assert report["all_runs_pass"] is True
    assert report["all_dynamic_runs_pass"] is True
    assert report["scope"].startswith("Base 1 4kg only")
    assert report["maxima"]["horizontal_drift_m"] == 0.108
    assert report["maxima"]["motor_saturation_rate"] == 0.0


def test_failsafe_invalidates_pass_marker(tmp_path: Path) -> None:
    log = tmp_path / "run.log"
    log.write_text("failsafe=True\nDDS_ARM_FLIGHT_PASS\n", encoding="utf-8")
    report = build_report([log])
    assert report["all_runs_pass"] is False


def test_no_failsafe_summary_text_is_not_a_failsafe(tmp_path: Path) -> None:
    log = tmp_path / "run.log"
    log.write_text("DDS_ARM_FLIGHT_FAIL no_failsafe=True\n", encoding="utf-8")
    report = build_report([log])
    assert report["runs"][0]["failsafe_seen"] is False


def test_obsolete_absolute_climb_failure_can_be_independently_reclassified(
    tmp_path: Path,
) -> None:
    log = tmp_path / "run.log"
    log.write_text(
        "STATE arm=1 nav=4 NED=(0,0,0.47)\n"
        "STATE arm=2 nav=14 NED=(0,0,-0.53)\n"
        "OFFBOARD mode and ARM commands sent\n"
        "PX4 command ack: command=21 result=0\n"
        "LANDING_DISARMED_CONFIRMED\n"
        "ARM_FLIGHT_CYCLE_METRICS label=demo_extended horizontal_drift_m=0.08\n"
        "ARM_FLIGHT_CYCLE_METRICS label=retracted horizontal_drift_m=0.07\n"
        "ARM_FLIGHT_METRICS horizontal_drift_m=0.08 altitude_span_m=0.12 "
        "max_truth_tilt_deg=1.0 rms_truth_tilt_deg=0.4 "
        "max_arm_torque_nm=0.035 motor_saturation_rate=0\n"
        "DDS_ARM_FLIGHT_FAIL missing=[] climbed=False no_failsafe=True stable=True\n",
        encoding="utf-8",
    )
    run = build_report([log])["runs"][0]
    assert run["pass"] is False
    assert run["relative_climb_pass"] is True
    assert run["full_sequence_gate_pass"] is True
    assert run["accepted_pass"] is True
