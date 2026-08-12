from test_ros2_dds_arm_flight_pty import climbed_relative_to_takeoff_ground


def state(arm: int, down: float):
    return (0.0, arm, 4.0, 0.0, 0.0, down, 0.0, 0.0, 0.0)


def test_relative_climb_accepts_positive_ekf_ground_origin() -> None:
    states = [state(1, 0.47), state(1, 0.48), state(2, 0.46), state(2, -0.55)]
    assert climbed_relative_to_takeoff_ground(states) is True


def test_relative_climb_rejects_small_motion() -> None:
    states = [state(1, -0.10), state(1, -0.09), state(2, -0.12), state(2, -0.50)]
    assert climbed_relative_to_takeoff_ground(states) is False


def test_relative_climb_requires_armed_samples() -> None:
    assert climbed_relative_to_takeoff_ground([state(1, 0.0)]) is False
