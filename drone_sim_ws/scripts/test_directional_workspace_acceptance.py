import importlib.util
import json
from pathlib import Path

import numpy as np


SCRIPTS = Path(__file__).resolve().parent
SPEC = importlib.util.spec_from_file_location(
    "directional_plan",
    SCRIPTS / "plan_directional_workspace_acceptance_4kg.py",
)
directional_plan = importlib.util.module_from_spec(SPEC)
assert SPEC.loader is not None
SPEC.loader.exec_module(directional_plan)

HARNESS_SPEC = importlib.util.spec_from_file_location(
    "directional_flight_harness",
    SCRIPTS / "test_ros2_dds_arm_flight_pty.py",
)
directional_flight_harness = importlib.util.module_from_spec(HARNESS_SPEC)
assert HARNESS_SPEC.loader is not None
HARNESS_SPEC.loader.exec_module(directional_flight_harness)


NAMES = (
    "shoulder_pan",
    "shoulder_lift",
    "elbow_flex",
    "wrist_flex",
    "wrist_roll",
    "gripper",
)


class FakeModel:
    def forward_kinematics(self, link, positions):
        assert link == "gripper_link"
        transform = np.eye(4)
        # Encode endpoint y/x/z in pan/lift/elbow.  This also exercises the
        # planner's shoulder-pan mirror fallback for front_left.
        transform[:3, 3] = (
            positions["shoulder_lift"],
            positions["shoulder_pan"],
            positions["elbow_flex"],
        )
        return transform, []


class FakeDynamics:
    model = FakeModel()


class FakePreflight:
    names = NAMES
    reference = {"presets": {"retracted": [0.0] * len(NAMES)}}
    dynamics = FakeDynamics()

    @staticmethod
    def collision_pairs_at(positions):
        return (
            [("synthetic_link_a", "synthetic_link_b")]
            if positions["wrist_flex"] == 99.0
            else []
        )

    @staticmethod
    def adapt_quintic(start, target, duration, **kwargs):
        # The envelope's direct front-left witness is deliberately rejected;
        # the mirrored front-right candidate must then be selected.
        rejected = target["wrist_flex"] == 99.0
        evaluation = {
            "failure_counts": {"collision_proxy": 1} if rejected else {},
            "maximum_gravity_torque_nm": 1.0,
            "maximum_reaction_force_n": 0.01,
            "maximum_reaction_torque_nm": 0.01,
            "minimum_physical_motor_headroom_n": 1.0,
            "minimum_overlay_delta_headroom_n": 0.2,
            "maximum_allocation_residual_norm": 1.0e-12,
        }
        selected = {
            "effective_duration_s": float(duration),
            "distance_scale": 1.0,
            "time_scale": 1.0,
            "evaluation": evaluation,
        }
        return {
            "accepted": not rejected,
            "decision": "accepted" if not rejected else "rejected",
            "selected": None if rejected else selected,
            "attempts": [{"evaluation": evaluation}],
        }


def _pose(x, y, z=0.0, *, sentinel=0.0):
    return {
        "positions_rad": {
            "shoulder_pan": y,
            "shoulder_lift": x,
            "elbow_flex": z,
            "wrist_flex": sentinel,
            "wrist_roll": 0.0,
            "gripper": 0.0,
        }
    }


def fake_envelope():
    horizontal = {
        "front": _pose(1.0, 0.0),
        "front_left": _pose(1.0, 1.0, sentinel=99.0),
        "left": _pose(0.0, 1.0),
        "rear_left": _pose(-1.0, 1.0),
        "rear": _pose(-1.0, 0.0),
        "rear_right": _pose(-1.0, -1.0),
        "right": _pose(0.0, -1.0),
        "front_right": _pose(1.0, -1.0),
    }
    return {
        "summary": {
            "vertical_classification_center_m": 0.0,
            "vertical_classification_deadband_m": 0.025,
        },
        "boundary_samples": {
            "maximum_radius_by_direction": horizontal,
            "highest_allowed": _pose(0.0, 0.0, 1.0),
            "lowest_allowed": _pose(0.0, 0.0, -1.0),
        },
    }


def test_schema2_height_reference_is_accepted():
    envelope = fake_envelope()
    envelope["summary"]["vertical_classification_reference_m"] = envelope[
        "summary"
    ].pop("vertical_classification_center_m")
    plan = directional_plan.build_plan(
        FakePreflight(), envelope, requested_duration_s=2.0,
        hold_s=1.0, settle_s=1.0, sample_count=5,
    )
    assert len(plan["legs"]) == 10


def test_plan_contains_every_direction_and_an_exact_return_for_each():
    plan = directional_plan.build_plan(
        FakePreflight(), fake_envelope(), requested_duration_s=2.0,
        hold_s=1.0, settle_s=1.0, sample_count=5,
    )
    assert tuple(plan["direction_order"]) == directional_plan.DIRECTION_ORDER
    assert len(plan["legs"]) == 10
    assert all(leg["return"]["distance_scale"] == 1.0 for leg in plan["legs"])
    assert all(leg["target_positions_rad"]["wrist_roll"] == 0.0 for leg in plan["legs"])
    assert all(leg["target_positions_rad"]["gripper"] == 0.0 for leg in plan["legs"])
    assert plan["acceptance_limits"] == {
        "xy_peak_to_peak_m": 0.05,
        "altitude_peak_to_peak_m": 0.05,
        "maximum_tilt_deg": 1.0,
        "motor_saturation_samples": 0,
        "failsafe": False,
        "return_required_after_every_direction": True,
    }


def test_collision_rejected_witness_uses_direction_checked_mirror():
    plan = directional_plan.build_plan(FakePreflight(), fake_envelope(), sample_count=5)
    front_left = next(leg for leg in plan["legs"] if leg["direction"] == "front_left")
    assert front_left["source"] == "mirrored_front_right"
    assert front_left["candidate_failures_before_selection"] == [
        {
            "source": "maximum_radius_by_direction.front_left",
            "failure_counts": {"collision_proxy": 1},
        }
    ]
    assert front_left["endpoint_body_flu_m"][0] > 0.0
    assert front_left["endpoint_body_flu_m"][1] > 0.0


def test_flight_harness_has_isolated_profile_and_per_stage_gates():
    source = (SCRIPTS / "test_ros2_dds_arm_flight_pty.py").read_text(encoding="utf-8")
    assert 'profile == "directional_workspace_4kg"' in source
    assert "DIRECTIONAL_STAGE_INTERVAL" in source
    assert "DIRECTIONAL_FLIGHT_STAGE_METRICS" in source
    assert "directional_stages_stable" in source
    assert "nose_forward_straight_4kg" in source
    launcher = (SCRIPTS / "run_directional_workspace_acceptance_4kg.sh").read_text(
        encoding="utf-8"
    )
    for value in ("0.05", "1.0", "ARM_DIRECTIONAL_PLAN"):
        assert value in launcher
    assert "DIRECTIONAL_RESTART_BACKEND" in launcher
    assert "colcon build --symlink-install --packages-select drone_arm_sim px4_ros2_control" in launcher
    sequence = (SCRIPTS / "directional_workspace_flight_sequence.py").read_text(
        encoding="utf-8"
    )
    assert "source_evidence" in sequence
    assert "directional plan is stale for the live model" in sequence
    assert "implementation_sha256" in sequence
    assert "directional plan is stale for the live preflight implementation" in sequence
    assert '"--directions"' in sequence
    assert "wait_for_direct_xy_ready()" in sequence
    assert "raise_if_motion_aborted()" in sequence
    assert "ARM_DIRECTIONAL_ONLY" in source
    assert "ARM_FLIGHT_CONTROLLER_SOURCE" in source
    assert 'src/px4_ros2_control/px4_ros2_control/dds_wasd_control.py' in source
    assert '[sys.executable, str(controller_source)]' in source
    assert '["ros2", "run", "px4_ros2_control", "dds_wasd_control"]' not in source
    candidate_launcher = (
        SCRIPTS / "run_directional_workspace_acceptance_1p3kg.sh"
    ).read_text(encoding="utf-8")
    assert "ARM_DIRECTIONAL_PREFLIGHT_SAMPLES" in candidate_launcher
    assert ':-81' in candidate_launcher
    front_launcher = (
        SCRIPTS / "run_front_retract_acceptance_1p3kg.sh"
    ).read_text(encoding="utf-8")
    assert "ARM_DIRECTIONAL_ONLY=front" in front_launcher
    assert "front_retract_flight_acceptance_1p3kg.log" in front_launcher


def test_directional_live_preflight_finishes_before_direct_xy_grant():
    source = (SCRIPTS / "directional_workspace_flight_sequence.py").read_text(
        encoding="utf-8"
    )
    loop = source.index("for leg in selected_legs:")
    outward = source.index("outward = preflight.adapt_quintic(", loop)
    return_leg = source.index("return_leg = preflight.adapt_quintic(", outward)
    grant = source.index("node.wait_for_direct_xy_ready()", return_leg)
    motion_rising = source.index("node.publish_motion_active(True)", grant)
    stability = source.index("_wait_px4_stable(", motion_rising)

    assert outward < return_leg < grant < motion_rising < stability
    # No collision/dynamics sweep may age either the ready lease before the
    # rising edge or the active motion heartbeat before the return command.
    assert "adapt_quintic(" not in source[grant:stability]


def test_directional_abort_recovery_requires_measured_retracted_success():
    command = directional_flight_harness.directional_abort_recovery_command()
    assert command[0].endswith("/scripts/run_with_cpu_role.sh")
    assert command[1] == "arm"
    assert command[2:] == [
        "ros2", "run", "drone_arm_sim", "arm_preset_control",
        "--preset", "retracted", "--duration", "20", "--wait",
        "--tolerance", "0.08", "--flight-preflight",
    ]
    reached = "ARM_PRESET_REACHED preset=retracted max_error=0.010000 rad"
    assert directional_flight_harness.directional_abort_recovery_reached(
        0, reached
    )
    assert not directional_flight_harness.directional_abort_recovery_reached(
        1, reached
    )
    assert not directional_flight_harness.directional_abort_recovery_reached(
        0, "trajectory process exited without measured endpoint evidence"
    )


def test_directional_abort_recovery_requires_fresh_armed_stable_hold_state():
    now = 100.0
    # timestamp, arm, nav, north, east, down, vx, vy, vz
    stable = (99.5, 2.0, 14.0, 0.0, 0.0, -1.2, 0.04, -0.03, 0.02)
    assert directional_flight_harness.directional_abort_recovery_stable(
        stable, now
    )
    for unsafe in (
        (98.0, *stable[1:]),
        (stable[0], 1.0, *stable[2:]),
        (*stable[:2], 18.0, *stable[3:]),
        (*stable[:6], 0.11, 0.0, 0.0),
        (*stable[:8], 0.09),
    ):
        assert not directional_flight_harness.directional_abort_recovery_stable(
            unsafe, now
        )


def test_directional_abort_path_never_lands_when_retraction_is_blocked():
    source = (SCRIPTS / "test_ros2_dds_arm_flight_pty.py").read_text(
        encoding="utf-8"
    )
    assert "ARM_FLIGHT_DIRECTIONAL_ERROR_LAND" not in source
    assert "ARM_FLIGHT_DIRECTIONAL_CHILD_ERROR_LAND" not in source
    assert "ARM_FLIGHT_DIRECTIONAL_RECOVERY_BLOCKED" in source
    assert "ARM_FLIGHT_DIRECTIONAL_RECOVERY_BLOCKED_LAND" not in source
    assert "DIRECTIONAL_EMERGENCY_RETRACT_COMPLETE" in source
    assert "DIRECTIONAL_EMERGENCY_RETRACT_BLOCKED" in source
    assert "and not directional_recovery_blocked" in source
    assert "and not directional_recovery_reached" in source
    assert "ARM_FLIGHT_DIRECTIONAL_RECOVERY_RETRACTED_REACHED" in source
    assert "ARM_FLIGHT_DIRECTIONAL_RECOVERY_STABLE_LAND" in source


def test_internal_emergency_return_reuses_preflight_without_bypassing_gates():
    source = (SCRIPTS / "directional_workspace_flight_sequence.py").read_text(
        encoding="utf-8"
    )
    helper = source.split(
        "def _attempt_preflighted_emergency_return(", 1
    )[1].split("\ndef main(", 1)[0]
    assert "TrajectoryPreflight" not in helper
    assert "node.publish_motion_active(False)" in helper
    assert (
        "node.wait_for_direct_xy_ready(timeout_s=EMERGENCY_REENTRY_TIMEOUT_S)"
        in helper
    )
    assert "EMERGENCY_REENTRY_TIMEOUT_S = 2.0" in source
    assert "node.clear_motion_abort_after_fresh_preauthorization()" in helper
    assert "node.publish_motion_active(True)" in helper
    assert "node.send(" in helper
    assert "DIRECTIONAL_EMERGENCY_RETRACT_BLOCKED" in helper
