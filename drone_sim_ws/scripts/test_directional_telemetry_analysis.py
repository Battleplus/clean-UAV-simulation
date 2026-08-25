import importlib.util
import json
from pathlib import Path
from types import SimpleNamespace


SCRIPTS = Path(__file__).resolve().parent
SPEC = importlib.util.spec_from_file_location(
    "directional_telemetry", SCRIPTS / "analyze_directional_telemetry.py"
)
directional_telemetry = importlib.util.module_from_spec(SPEC)
assert SPEC.loader is not None
SPEC.loader.exec_module(directional_telemetry)
CONFIG = (
    SCRIPTS.parent
    / "src/drone_arm_sim/config/my_drone_v3_cad_candidate_1p3kg.json"
)


def build_evidence(
    tmp_path: Path,
    *,
    altitude_excursion: float = 0.01,
    expected_directions=None,
):
    events = tmp_path / "events.log"
    events.write_text(
        "\n".join(
            [
                "DIRECTIONAL_STAGE_INTERVAL direction=front phase=extend start=1 end=2",
                "DIRECTIONAL_STAGE_INTERVAL direction=front phase=hold start=2 end=3",
                "DIRECTIONAL_STAGE_INTERVAL direction=front phase=retract start=3 end=4",
            ]
        ),
        encoding="utf-8",
    )
    telemetry = tmp_path / "telemetry.jsonl"
    samples = [
        {
            "kind": "vehicle_status",
            "recorder_monotonic_s": 1.0,
            "failsafe": False,
        }
    ]
    sequence = 1
    for index in range(301):
        stamp = 1.0 + 0.01 * index
        z = altitude_excursion if index == 150 else 0.0
        samples.extend(
            [
                {
                    "sequence": sequence,
                    "kind": "odometry",
                    "recorder_monotonic_s": stamp,
                    "position_world_enu_m": [0.001, -0.001, z],
                    "quaternion_body_to_world_xyzw": [0.0, 0.0, 0.0, 1.0],
                },
                {
                    "sequence": sequence + 1,
                    "kind": "px4_actuator_outputs",
                    "recorder_monotonic_s": stamp,
                    "outputs": [200.0] * 8,
                },
                {
                    "sequence": sequence + 2,
                    "kind": "reallocator",
                    "recorder_monotonic_s": stamp,
                    "state": {
                        "event": "allocated",
                        "requested_compensation_wrench_frd": [
                            0.01, 0.0, 0.0, 0.0, 0.01, 0.0
                        ],
                        "delivered_compensation_wrench_frd": [
                            0.01, 0.0, 0.0, 0.0, 0.01, 0.0
                        ],
                        "feasibility_scale": 1.0,
                        "residual_norm": 1.0e-12,
                        "saturated": 0,
                    },
                },
            ]
        )
        sequence += 3
    telemetry.write_text(
        "\n".join(json.dumps(sample) for sample in samples), encoding="utf-8"
    )
    return SimpleNamespace(
        events=events,
        telemetry=telemetry,
        config=CONFIG,
        xy_limit=0.05,
        z_limit=0.05,
        tilt_limit=1.0,
        minimum_rate_hz=50.0,
        maximum_gap_s=0.10,
        residual_limit=0.02,
        expected_directions=expected_directions,
    )


def test_high_rate_directional_evidence_passes_every_stage(tmp_path):
    report = directional_telemetry.analyze(build_evidence(tmp_path))
    assert report["pass"]
    assert len(report["stages"]) == 3
    assert all(stage["pass"] for stage in report["stages"])


def test_altitude_excursion_over_limit_fails_stage(tmp_path):
    report = directional_telemetry.analyze(
        build_evidence(tmp_path, altitude_excursion=0.06)
    )
    assert not report["pass"]
    assert any(
        stage["altitude_peak_to_peak_m"] > 0.05 for stage in report["stages"]
    )


def test_partial_direction_cannot_pass_full_matrix_gate(tmp_path):
    report = directional_telemetry.analyze(
        build_evidence(
            tmp_path,
            expected_directions=directional_telemetry.FORMAL_DIRECTIONS,
        )
    )
    assert not report["pass"]
    assert not report["interval_evidence"]["complete"]
    assert report["interval_evidence"]["expected_stage_count"] == 30


def test_duplicate_stage_interval_fails_even_when_latest_value_is_valid(tmp_path):
    args = build_evidence(tmp_path, expected_directions=("front",))
    with args.events.open("a", encoding="utf-8") as stream:
        stream.write(
            "\nDIRECTIONAL_STAGE_INTERVAL direction=front phase=hold "
            "start=2 end=3\n"
        )
    report = directional_telemetry.analyze(args)
    assert not report["pass"]
    assert report["interval_evidence"]["duplicate_intervals"] == [
        ["front", "hold"]
    ]


def _direct_xy_window(
    *,
    finite_xy_velocity: bool = False,
    position_active: bool = True,
    sample_count: int = 50,
):
    samples = []
    sequence = 0
    for index in range(1, sample_count):
        stamp = 1.0 + index * 0.02
        px4_stamp = 1_000_000 + index
        samples.extend(
            [
                {
                    "sequence": sequence,
                    "kind": "reallocator",
                    "recorder_monotonic_s": stamp - 0.005,
                    "state": {
                        "position_feedback_enabled": True,
                        "position_feedback_active": position_active,
                        "position_target_latched": True,
                        "truth_fresh": True,
                        "allocation_limited": False,
                    },
                },
                {
                    "sequence": sequence + 1,
                    "kind": "arm_direct_xy_state",
                    "recorder_monotonic_s": stamp - 0.004,
                    "state": {
                        "state": "direct_xy",
                        "position_feedback_active": position_active,
                        "reallocator_fresh": True,
                    },
                },
                {
                    "sequence": sequence + 2,
                    "kind": "px4_offboard_control_mode_input",
                    "recorder_monotonic_s": stamp - 0.001,
                    "px4_timestamp_us": px4_stamp,
                    "position": False,
                    "velocity": True,
                    "acceleration": False,
                    "attitude": False,
                    "body_rate": False,
                    "thrust_and_torque": False,
                    "direct_actuator": False,
                },
                {
                    "sequence": sequence + 3,
                    "kind": "px4_trajectory_setpoint_input",
                    "recorder_monotonic_s": stamp,
                    "px4_timestamp_us": px4_stamp,
                    "position": [float("nan")] * 3,
                    "velocity": (
                        [0.0, 0.0, -0.01]
                        if finite_xy_velocity
                        else [float("nan"), float("nan"), -0.01]
                    ),
                    "acceleration": [0.0, 0.0, float("nan")],
                    "jerk": [float("nan")] * 3,
                    "yaw": 0.0,
                    "yawspeed": float("nan"),
                },
            ]
        )
        sequence += 4
    return samples


def _controller_intent_window(
    *,
    stamps=(1.0, 1.05, 1.10),
    epochs=(4, 4, 4),
    generations=(7, 7, 7),
    motion_active=True,
):
    samples = []
    for index, (stamp, epoch, generation) in enumerate(
        zip(stamps, epochs, generations)
    ):
        samples.append(
            {
                "sequence": index,
                "kind": "arm_direct_xy_controller_intent",
                "recorder_monotonic_s": stamp,
                "state": {
                    "schema": "my_drone.arm-direct-xy-controller-intent.v1",
                    "monotonic_s": stamp - 0.001,
                    "controller_session_id": "controller-v9",
                    "ownership_epoch": epoch,
                    "intent_generation": generation,
                    "state": "direct_xy",
                    "motion_active": motion_active,
                    "mixed_setpoint_active": True,
                    "force_requested": True,
                },
            }
        )
    return samples


def test_controller_intent_evidence_reports_rate_gap_and_identity_continuity():
    report = directional_telemetry.direct_xy_controller_intent_evidence(
        _controller_intent_window()
    )
    assert report["pass"]
    assert report["active_intent_samples"] == 3
    assert abs(report["effective_active_rate_hz"] - 20.0) < 1.0e-9
    assert abs(report["maximum_active_gap_s"] - 0.05) < 1.0e-9
    assert report["inconsistent_active_samples"] == 0
    assert report["ownership_epoch_regressions"] == 0
    assert report["intent_generation_regressions"] == 0


def test_controller_intent_gap_at_250ms_is_rejected_strictly():
    report = directional_telemetry.direct_xy_controller_intent_evidence(
        _controller_intent_window(stamps=(1.0, 1.25, 1.30))
    )
    assert not report["pass"]
    assert report["maximum_active_gap_s"] == 0.25
    assert report["maximum_allowed_gap_s"] == 0.250


def test_controller_intent_inconsistent_requested_level_is_rejected():
    report = directional_telemetry.direct_xy_controller_intent_evidence(
        _controller_intent_window(motion_active=False)
    )
    assert not report["pass"]
    assert report["inconsistent_active_samples"] == 3


def test_controller_intent_epoch_and_generation_regressions_are_rejected():
    report = directional_telemetry.direct_xy_controller_intent_evidence(
        _controller_intent_window(
            epochs=(4, 3, 3), generations=(7, 8, 6)
        )
    )
    assert not report["pass"]
    assert report["ownership_epoch_regressions"] == 1
    assert report["intent_generation_regressions"] == 1


def _window_with_validated_entry():
    """A finite-PX4 prefix and one 55 ms ordered direct-XY entry."""
    samples = [
        {
            "sequence": -8,
            "kind": "reallocator",
            "recorder_monotonic_s": 0.885,
            "state": {
                "position_feedback_enabled": True,
                "position_feedback_active": False,
                "position_target_latched": True,
                "truth_fresh": True,
                "allocation_limited": False,
            },
        },
        {
            "sequence": -7,
            "kind": "arm_direct_xy_state",
            "recorder_monotonic_s": 0.886,
            "state": {
                "state": "px4_xy",
                "position_feedback_active": False,
                "reallocator_fresh": True,
            },
        },
        {
            "sequence": -6,
            "kind": "px4_offboard_control_mode_input",
            "recorder_monotonic_s": 0.899,
            "px4_timestamp_us": 900_000,
            "position": False,
            "velocity": True,
            "acceleration": False,
            "attitude": False,
            "body_rate": False,
            "thrust_and_torque": False,
            "direct_actuator": False,
        },
        {
            "sequence": -5,
            "kind": "px4_trajectory_setpoint_input",
            "recorder_monotonic_s": 0.900,
            "px4_timestamp_us": 900_000,
            "position": [float("nan")] * 3,
            "velocity": [0.0, 0.0, -0.01],
            "acceleration": [float("nan")] * 3,
            "jerk": [float("nan")] * 3,
            "yaw": 0.0,
            "yawspeed": float("nan"),
        },
        {
            "sequence": -4,
            "kind": "px4_offboard_control_mode_input",
            "recorder_monotonic_s": 0.949,
            "px4_timestamp_us": 950_000,
            "position": False,
            "velocity": True,
            "acceleration": False,
            "attitude": False,
            "body_rate": False,
            "thrust_and_torque": False,
            "direct_actuator": False,
        },
        {
            "sequence": -3,
            "kind": "px4_trajectory_setpoint_input",
            "recorder_monotonic_s": 0.950,
            "px4_timestamp_us": 950_000,
            "position": [float("nan")] * 3,
            "velocity": [float("nan"), float("nan"), -0.01],
            "acceleration": [0.0, 0.0, float("nan")],
            "jerk": [float("nan")] * 3,
            "yaw": 0.0,
            "yawspeed": float("nan"),
        },
    ]
    samples.extend(_direct_xy_window(sample_count=205))
    handoffs = {
        "entry_handoffs": [
            {
                "mixed_setpoint_s": 0.950,
                "direct_force_on_s": 1.005,
                "lag_s": 0.055,
            }
        ],
        "exit_handoffs": [],
    }
    return samples, handoffs


def test_mixed_axis_ownership_evidence_proves_unique_owner():
    report = directional_telemetry.direct_xy_ownership_evidence(
        _direct_xy_window(), 1.0, 2.0
    )
    assert report["pass"]
    assert report["direct_owner_coverage"] == 1.0
    assert report["dual_owner_samples"] == 0
    assert report["unowned_xy_samples"] == 0


def test_setpoint_keepalive_uses_recent_mode_level_without_exact_timestamp():
    samples = _direct_xy_window()
    # The real 50 Hz keepalive refreshes TrajectorySetpoint timestamps between
    # authoritative 20 Hz flight-loop publications.  Keep one mode level for
    # every three setpoints to reproduce that legal wire pattern.
    samples = [
        sample
        for sample in samples
        if sample.get("kind") != "px4_offboard_control_mode_input"
        or ((int(sample["sequence"]) - 1) // 4) % 3 == 0
    ]
    report = directional_telemetry.direct_xy_ownership_evidence(
        samples, 1.0, 2.0
    )
    assert report["pass"]
    assert report["fresh_velocity_mode_coverage"] == 1.0
    assert report["mode_setpoint_timestamp_pair_rate"] < 0.5


def test_recent_mode_level_may_precede_stage_boundary():
    samples = [
        sample
        for sample in _direct_xy_window(sample_count=5)
        if sample.get("kind") != "px4_offboard_control_mode_input"
    ]
    samples.append(
        {
            "sequence": -1,
            "kind": "px4_offboard_control_mode_input",
            "recorder_monotonic_s": 0.99,
            "px4_timestamp_us": 990_000,
            "position": False,
            "velocity": True,
            "acceleration": False,
            "attitude": False,
            "body_rate": False,
            "thrust_and_torque": False,
            "direct_actuator": False,
        }
    )
    report = directional_telemetry.direct_xy_ownership_evidence(
        samples, 1.0, 1.08
    )
    assert report["pass"]
    assert report["mode_setpoint_timestamp_pair_rate"] == 0.0


def test_stale_mode_level_is_not_accepted_for_keepalive_setpoints():
    samples = [
        sample
        for sample in _direct_xy_window()
        if sample.get("kind") != "px4_offboard_control_mode_input"
    ]
    samples.append(
        {
            "sequence": -1,
            "kind": "px4_offboard_control_mode_input",
            "recorder_monotonic_s": 0.99,
            "px4_timestamp_us": 990_000,
            "position": False,
            "velocity": True,
            "acceleration": False,
            "attitude": False,
            "body_rate": False,
            "thrust_and_torque": False,
            "direct_actuator": False,
        }
    )
    report = directional_telemetry.direct_xy_ownership_evidence(
        samples, 1.0, 2.0
    )
    assert not report["pass"]
    assert report["fresh_velocity_mode_coverage"] < 0.99


def test_finite_px4_xy_velocity_is_detected_as_dual_ownership():
    report = directional_telemetry.direct_xy_ownership_evidence(
        _direct_xy_window(finite_xy_velocity=True), 1.0, 2.0
    )
    assert not report["pass"]
    assert report["dual_owner_samples"] > 0


def test_mixed_axis_without_position_force_owner_is_rejected():
    report = directional_telemetry.direct_xy_ownership_evidence(
        _direct_xy_window(position_active=False), 1.0, 2.0
    )
    assert not report["pass"]
    assert report["unowned_xy_samples"] > 0


def test_stage_entry_accepts_only_independently_validated_handoff_corridor():
    samples, handoffs = _window_with_validated_entry()
    without_evidence = directional_telemetry.direct_xy_ownership_evidence(
        samples, 0.88, 5.10
    )
    assert not without_evidence["pass"]
    assert without_evidence["unowned_xy_samples"] == 1

    report = directional_telemetry.direct_xy_ownership_evidence(
        samples, 0.88, 5.10, validated_handoffs=handoffs
    )
    assert report["pass"]
    assert report["validated_handoff_corridor_samples"] == 1
    assert report["unowned_xy_samples"] == 0
    assert report["maximum_ownership_anomaly_window_s"] < 0.10


def test_validated_handoff_never_exempts_real_setpoint_gap():
    samples, handoffs = _window_with_validated_entry()
    # Delete six consecutive 50 Hz setpoint/mode pairs.  State samples remain,
    # so this isolates a real 140 ms setpoint-publication gap.
    missing_px4_stamps = {1_000_000 + index for index in range(80, 86)}
    samples = [
        sample
        for sample in samples
        if not (
            sample.get("kind")
            in {
                "px4_offboard_control_mode_input",
                "px4_trajectory_setpoint_input",
            }
            and sample.get("px4_timestamp_us") in missing_px4_stamps
        )
    ]
    report = directional_telemetry.direct_xy_ownership_evidence(
        samples, 0.88, 5.10, validated_handoffs=handoffs
    )
    assert not report["pass"]
    assert report["unowned_xy_samples"] == 0
    assert report["maximum_setpoint_gap_s"] > 0.10


def _handoff_sample(stamp, sequence, kind, value):
    if kind == "setpoint":
        mixed = value == "mixed"
        return {
            "sequence": sequence,
            "kind": "px4_trajectory_setpoint_input",
            "recorder_monotonic_s": stamp,
            "position": [float("nan")] * 3,
            "velocity": (
                [float("nan"), float("nan"), -0.01]
                if mixed
                else [0.0, 0.0, -0.01]
            ),
            "acceleration": (
                [0.0, 0.0, float("nan")]
                if mixed
                else [float("nan")] * 3
            ),
            "jerk": [float("nan")] * 3,
            "yaw": 0.0,
            "yawspeed": float("nan"),
        }
    return {
        "sequence": sequence,
        "kind": "reallocator",
        "recorder_monotonic_s": stamp,
        "state": {"position_feedback_active": bool(value)},
    }


def _ordered_handoff_window(*, entry_force_first=False, exit_force_first=False):
    samples = [
        _handoff_sample(1.000, 1, "setpoint", "finite"),
        _handoff_sample(1.001, 2, "force", False),
    ]
    entry = [
        _handoff_sample(2.000, 3, "setpoint", "mixed"),
        _handoff_sample(2.020, 4, "force", True),
    ]
    if entry_force_first:
        entry.reverse()
        entry[0]["recorder_monotonic_s"] = 2.000
        entry[1]["recorder_monotonic_s"] = 2.020
    samples.extend(entry)
    samples.extend(
        [
            _handoff_sample(2.040, 5, "setpoint", "mixed"),
            _handoff_sample(2.041, 6, "force", True),
        ]
    )
    exit_events = [
        _handoff_sample(3.000, 7, "setpoint", "finite"),
        _handoff_sample(3.020, 8, "force", False),
    ]
    if exit_force_first:
        exit_events.reverse()
        exit_events[0]["recorder_monotonic_s"] = 3.000
        exit_events[1]["recorder_monotonic_s"] = 3.020
    samples.extend(exit_events)
    samples.extend(
        [
            _handoff_sample(3.040, 9, "setpoint", "finite"),
            _handoff_sample(3.041, 10, "force", False),
        ]
    )
    return samples


def test_formal_handoff_accepts_mixed_then_force_and_finite_then_force_off():
    report = directional_telemetry.direct_xy_handoff_evidence(
        _ordered_handoff_window()
    )
    assert report["pass"]
    assert report["entry_count"] == report["exit_count"] == 1
    assert abs(report["maximum_entry_lag_s"] - 0.020) < 1.0e-9
    assert abs(report["maximum_exit_lag_s"] - 0.020) < 1.0e-9
    assert report["dual_owner_outside_handoff_samples"] == 0
    assert report["unowned_xy_outside_handoff_samples"] == 0


def test_formal_handoff_rejects_direct_force_before_mixed_setpoint():
    report = directional_telemetry.direct_xy_handoff_evidence(
        _ordered_handoff_window(entry_force_first=True)
    )
    assert not report["pass"]
    assert report["entry_force_before_mixed_count"] == 1
    assert report["dual_owner_outside_handoff_samples"] > 0


def test_formal_handoff_rejects_force_off_before_finite_px4_xy():
    report = directional_telemetry.direct_xy_handoff_evidence(
        _ordered_handoff_window(exit_force_first=True)
    )
    assert not report["pass"]
    assert report["exit_force_off_before_finite_count"] == 1
    assert report["unowned_xy_outside_handoff_samples"] > 0


def test_formal_handoff_rejects_unfinished_transition_corridor():
    samples = _ordered_handoff_window()[:3]
    report = directional_telemetry.direct_xy_handoff_evidence(samples)
    assert not report["pass"]
    assert report["incomplete_handoff"]
