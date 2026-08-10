import math

import pytest

from drone_arm_sim.rule_supervisor import (
    RuleSupervisorPolicy,
    SupervisorInput,
    SupervisorState,
    evaluate_supervisor,
    quaternion_tilt_rad,
)


@pytest.mark.parametrize(
    ("sample", "state", "scale"),
    [
        (SupervisorInput(), SupervisorState.NORMAL, 1.0),
        (SupervisorInput(arm_torque_nm=0.16), SupervisorState.SLOW, 0.45),
        (SupervisorInput(horizontal_error_m=0.65), SupervisorState.PAUSE, 0.0),
        (SupervisorInput(tilt_rad=0.45), SupervisorState.RETRACT, 0.0),
        (SupervisorInput(motor_saturation_fraction=0.75), SupervisorState.LAND, 0.0),
    ],
)
def test_policy_boundaries(sample, state, scale):
    decision = evaluate_supervisor(sample)
    assert decision.state == state
    assert decision.arm_speed_scale == scale


def test_policy_recovery_requires_a_stable_hold():
    policy = RuleSupervisorPolicy(recovery_hold_s=2.0)
    assert policy.update(SupervisorInput(arm_torque_nm=0.35), 0.0).state == SupervisorState.PAUSE
    assert policy.update(SupervisorInput(), 1.0).state == SupervisorState.PAUSE
    assert policy.update(SupervisorInput(), 2.9).state == SupervisorState.PAUSE
    assert policy.update(SupervisorInput(), 3.1).state == SupervisorState.NORMAL


def test_invalid_samples_are_rejected():
    with pytest.raises(ValueError):
        evaluate_supervisor(SupervisorInput(horizontal_error_m=-1.0))
    with pytest.raises(ValueError):
        evaluate_supervisor(SupervisorInput(motor_saturation_fraction=1.1))


def test_quaternion_tilt_ignores_yaw_and_measures_roll():
    yaw = math.radians(60.0)
    assert quaternion_tilt_rad(0.0, 0.0, math.sin(yaw / 2), math.cos(yaw / 2)) == pytest.approx(0.0)
    roll = math.radians(30.0)
    assert quaternion_tilt_rad(math.sin(roll / 2), 0.0, 0.0, math.cos(roll / 2)) == pytest.approx(roll)
