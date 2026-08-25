from pathlib import Path

import numpy as np
import pytest

from drone_arm_sim.cartesian_velocity_profile import (
    COMMAND_DIRECTIONS,
    JerkLimitedVelocity3D,
    joint_velocity_sample_is_stable,
)
from drone_arm_sim.cartesian_velocity_kinematics import solve_velocity_step_ik
from drone_arm_sim.model_analysis import UrdfModel


ROOT = Path(__file__).resolve().parents[3]
PACKAGE = ROOT / "src/drone_arm_sim"
JOINT_NAMES = [
    "shoulder_pan",
    "shoulder_lift",
    "elbow_flex",
    "wrist_flex",
    "wrist_roll",
    "gripper",
]
TOOL_FORWARD_AXIS_LOCAL = np.array([0.999961256, -0.000000665, 0.008802682])
TOOL_FORWARD_AXIS_LOCAL /= np.linalg.norm(TOOL_FORWARD_AXIS_LOCAL)


def test_direction_map_covers_six_cartesian_directions():
    assert set(COMMAND_DIRECTIONS) == {
        "forward", "back", "left", "right", "up", "down"
    }
    np.testing.assert_array_equal(COMMAND_DIRECTIONS["forward"], [1.0, 0.0, 0.0])
    np.testing.assert_array_equal(COMMAND_DIRECTIONS["back"], [-1.0, 0.0, 0.0])
    np.testing.assert_array_equal(COMMAND_DIRECTIONS["left"], [0.0, 1.0, 0.0])
    np.testing.assert_array_equal(COMMAND_DIRECTIONS["right"], [0.0, -1.0, 0.0])
    np.testing.assert_array_equal(COMMAND_DIRECTIONS["up"], [0.0, 0.0, 1.0])
    np.testing.assert_array_equal(COMMAND_DIRECTIONS["down"], [0.0, 0.0, -1.0])


def test_one_velocity_command_latches_without_key_repeat():
    limiter = JerkLimitedVelocity3D(0.01, 0.01, 0.02)
    limiter.set_direction(COMMAND_DIRECTIONS["forward"])
    values = [limiter.step(0.05) for _ in range(200)]
    assert values[0][0] > 0.0
    assert values[-1][0] == pytest.approx(0.01)
    assert values[-1][1] == pytest.approx(0.0)
    assert values[-1][2] == pytest.approx(0.0)
    assert limiter.target[0] == pytest.approx(0.01)


def test_invalid_timer_delta_keeps_finite_velocity_state():
    limiter = JerkLimitedVelocity3D(0.01, 0.01, 0.02)
    limiter.set_direction(COMMAND_DIRECTIONS["forward"])
    np.testing.assert_array_equal(limiter.step(float("nan")), np.zeros(3))
    np.testing.assert_array_equal(limiter.step(None), np.zeros(3))


def test_stop_and_reverse_are_acceleration_and_jerk_limited():
    limiter = JerkLimitedVelocity3D(0.01, 0.01, 0.02)
    limiter.set_direction(COMMAND_DIRECTIONS["forward"])
    for _ in range(100):
        limiter.step(0.05)
    before = limiter.velocity.copy()
    limiter.set_direction(None)
    first_stop = limiter.step(0.05)
    assert 0.0 < first_stop[0] < before[0]
    assert abs(limiter.acceleration[0]) <= 0.01 + 1e-12
    previous_acceleration = limiter.acceleration.copy()
    for _ in range(200):
        limiter.step(0.05)
        assert np.linalg.norm(limiter.acceleration) <= 0.01 + 1e-12
        assert (
            np.linalg.norm(limiter.acceleration - previous_acceleration)
            <= 0.02 * 0.05 + 1e-12
        )
        previous_acceleration = limiter.acceleration.copy()
    assert limiter.stopped()

    limiter.set_direction(COMMAND_DIRECTIONS["back"])
    previous_acceleration = limiter.acceleration.copy()
    limiter.step(0.05)
    assert np.max(np.abs(limiter.acceleration - previous_acceleration)) <= 0.02 * 0.05 + 1e-12
    assert np.linalg.norm(limiter.velocity) <= 0.01 + 1e-12


def test_diagonal_motion_uses_vector_acceleration_and_jerk_limits():
    limiter = JerkLimitedVelocity3D(0.01, 0.01, 0.02)
    limiter.set_direction(np.asarray([1.0, 1.0, 1.0]))
    previous_acceleration = limiter.acceleration.copy()
    for _ in range(400):
        limiter.step(0.05)
        assert np.linalg.norm(limiter.velocity) <= 0.01 + 1e-12
        assert np.linalg.norm(limiter.acceleration) <= 0.01 + 1e-12
        assert (
            np.linalg.norm(limiter.acceleration - previous_acceleration)
            <= 0.02 * 0.05 + 1e-12
        )
        previous_acceleration = limiter.acceleration.copy()
    np.testing.assert_allclose(
        limiter.velocity,
        np.full(3, 0.01 / np.sqrt(3.0)),
        atol=1.0e-9,
    )


def test_candidate_retracted_pose_small_step_ik_moves_in_all_six_directions():
    model = UrdfModel(
        PACKAGE / "urdf/my_drone_v3/my_drone_cad_candidate_1p3kg.urdf"
    )
    reference = __import__("json").loads(
        (PACKAGE / "config/so101_motion_reference_4kg.json").read_text(
            encoding="utf-8"
        )
    )
    names = [item["name"] for item in reference["joints"]]
    retracted = dict(zip(names, reference["presets"]["retracted"], strict=True))
    seed = {name: retracted[name] for name in JOINT_NAMES[:-1]}
    start, _ = model.forward_kinematics("gripper_link", seed)
    forward = start[:3, :3] @ TOOL_FORWARD_AXIS_LOCAL
    forward /= np.linalg.norm(forward)

    for command, direction in COMMAND_DIRECTIONS.items():
        requested = 0.005 * direction
        solution, status = solve_velocity_step_ik(
            model,
            start[:3, 3] + requested,
            forward,
            seed,
            TOOL_FORWARD_AXIS_LOCAL,
            position_tolerance_m=1.0e-4,
        )
        achieved, _ = model.forward_kinematics("gripper_link", solution)
        displacement = achieved[:3, 3] - start[:3, 3]
        assert status["converged"], (command, status)
        assert np.linalg.norm(displacement) > 1.0e-5, command
        assert float(np.dot(displacement, direction)) > 1.0e-5, command


def test_joint_settle_gate_requires_complete_finite_low_velocity_sample():
    stopped = {name: 0.001 for name in JOINT_NAMES}
    assert joint_velocity_sample_is_stable(stopped, JOINT_NAMES)
    assert not joint_velocity_sample_is_stable(
        {name: value for name, value in stopped.items() if name != "gripper"},
        JOINT_NAMES,
    )
    assert not joint_velocity_sample_is_stable(
        {**stopped, "wrist_roll": 0.02}, JOINT_NAMES
    )
    assert not joint_velocity_sample_is_stable(
        {**stopped, "wrist_roll": np.nan}, JOINT_NAMES
    )


def test_persistent_server_preflights_every_segment_and_publishes_selected_target():
    source = (
        ROOT / "src/drone_arm_sim/drone_arm_sim/cartesian_velocity_control.py"
    ).read_text(encoding="utf-8")
    assert "TrajectoryPreflight" in source
    assert "allow_distance_scaling=True" in source
    assert 'time_scales=(1.0, 1.5, 2.0, 3.0, 4.0, 6.0, 8.0, 12.0)' in source
    assert 'distance_scales=(0.9, 0.8, 0.7, 0.6, 0.5)' in source
    assert "self._publish_segment(selected_target, duration)" in source
    assert "maximum_joint_step / 0.02" in source
    assert "sample_count=preflight_sample_count" in source
    assert 'self.motion_publisher.publish(Bool(data=False))' in source
    assert "self.settled_joint_samples >= JOINT_SETTLE_SAMPLE_COUNT" in source
    assert "self.motion_publisher.publish(Bool(data=True))" in source


def test_directional_cycle_keeps_one_motion_session_through_the_dwell():
    source = (
        ROOT / "scripts/directional_workspace_flight_sequence.py"
    ).read_text(encoding="utf-8")
    hold = source[source.index("def _hold("):source.index("def _wait_px4_stable(")]
    assert "publish_motion_active(True)" in hold
    assert "publish_motion_active(False)" not in hold


def test_4kg_startup_preloads_velocity_server_and_keyboard_uses_latched_commands():
    overlay = (ROOT / "scripts/activate_base1_wrench_reallocator_overlay.sh").read_text(encoding="utf-8")
    keyboard = (ROOT / "scripts/run_ros2_arm_keyboard.sh").read_text(encoding="utf-8")
    cleanup = (ROOT / "scripts/wsl_start_ros2_dds_noarm.sh").read_text(encoding="utf-8")
    assert "BASE1_CARTESIAN_VELOCITY_READY" in overlay
    assert "cartesian_arm_velocity_control" in overlay
    assert "/my_drone/arm_cartesian_velocity_command" in keyboard
    assert "run_cartesian_velocity_mode" in keyboard
    assert "SPACE: jerk-limited stop" in keyboard
    assert "[c]artesian_arm_velocity_control" in cleanup
