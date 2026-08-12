from types import SimpleNamespace

import numpy as np
import pytest

from px4_ros2_control.dds_wasd_control import (
    DdsWasdControl,
    FlightControlState,
    TargetNed,
)


def make_controller(z: float, sign: float = -1.0):
    controller = object.__new__(DdsWasdControl)
    controller.local_z_sign = sign
    controller.local = SimpleNamespace(x=1.0, y=-2.0, z=z, heading=0.4)
    controller.target_initialized = True
    controller.xy_reset_counter = 0
    controller.z_reset_counter = 0
    controller.heading_reset_counter = 0
    controller.takeoff_ground_down = z
    controller.control_state = FlightControlState.POSITION_HOLD
    controller.active_velocity_key = None
    controller.last_velocity_key_monotonic = 0.0
    controller.velocity_command_ned = np.zeros(3)
    controller.acceleration_command_ned = np.zeros(3)
    controller.velocity_acceleration_feedforward_enabled = True
    controller.yaw_rate_command = 0.0
    controller.yaw_hold_rad = 0.4
    controller.yaw_hold_pending = False
    return controller


def test_local_position_resets_shift_hold_and_ground_references(monkeypatch):
    quiet_logger(monkeypatch)
    controller = make_controller(-1.2)
    controller.target = TargetNed(1.0, 2.0, -1.5, 0.4)
    reset = SimpleNamespace(
        x=1.3,
        y=1.8,
        z=-1.1,
        heading=0.5,
        xy_valid=True,
        z_valid=True,
        delta_xy=[0.3, -0.2],
        xy_reset_counter=1,
        delta_z=0.1,
        z_reset_counter=1,
        delta_heading=0.1,
        heading_reset_counter=1,
    )

    controller._local_cb(reset)

    assert controller.target.north == pytest.approx(1.3)
    assert controller.target.east == pytest.approx(1.8)
    assert controller.target.down == pytest.approx(-1.4)
    assert controller.takeoff_ground_down == pytest.approx(-1.1)
    assert controller.target.yaw == pytest.approx(0.5)


def test_disarmed_prestream_refresh_preserves_takeoff_height_while_falling():
    controller = make_controller(-1.5)
    controller._refresh_disarmed_takeoff_target()
    assert controller.target.down - controller.local.z == pytest.approx(-1.2)

    controller.local.z = -0.4
    controller._refresh_disarmed_takeoff_target()
    assert controller.takeoff_ground_down == pytest.approx(-0.4)
    assert controller.target.down - controller.local.z == pytest.approx(-1.2)


def test_disarmed_prestream_refresh_supports_positive_vertical_convention():
    controller = make_controller(0.7, sign=1.0)
    controller._refresh_disarmed_takeoff_target()
    assert controller.target.down - controller.local.z == pytest.approx(1.2)


def quiet_logger(monkeypatch):
    logger = SimpleNamespace(info=lambda *_args: None, warning=lambda *_args: None)
    monkeypatch.setattr(DdsWasdControl, "get_logger", lambda _self: logger)


def advance_velocity(controller, start: float = 10.0, steps: int = 100, dt: float = 0.05):
    for index in range(steps):
        controller.update_velocity_control(start + (index + 1) * dt, dt)


def test_r_commands_up_and_f_commands_down_velocity(monkeypatch):
    quiet_logger(monkeypatch)
    controller = make_controller(-1.2, sign=-1.0)

    controller.set_velocity_key("r", now=10.0)
    velocity, _ = controller._desired_velocity_ned(True)
    assert velocity[2] == pytest.approx(-0.15)
    controller.set_velocity_key("f", now=10.1)
    velocity, _ = controller._desired_velocity_ned(True)
    assert velocity[2] == pytest.approx(0.15)


def test_hover_key_commands_zero_velocity_without_position_switch(monkeypatch):
    quiet_logger(monkeypatch)
    controller = make_controller(-1.45, sign=-1.0)
    controller.target = TargetNed(8.0, 9.0, -4.0, -1.0)
    controller.control_state = FlightControlState.VELOCITY_CONTROL
    controller.velocity_command_ned[:] = [0.2, -0.1, 0.05]
    controller.yaw_rate_command = 0.1

    controller.hold_current_position()

    assert controller.target.north == pytest.approx(8.0)
    assert controller.target.east == pytest.approx(9.0)
    assert controller.target.down == pytest.approx(-4.0)
    assert controller.target.yaw == pytest.approx(-1.0)
    assert controller.control_state == FlightControlState.VELOCITY_CONTROL
    assert controller.active_velocity_key is None
    assert np.allclose(controller.velocity_command_ned, [0.2, -0.1, 0.05])
    advance_velocity(controller)
    assert np.allclose(controller.velocity_command_ned, 0.0)
    assert controller.yaw_rate_command == pytest.approx(0.0)


def test_key_commands_full_velocity_and_remains_latched(monkeypatch):
    quiet_logger(monkeypatch)
    controller = make_controller(-1.2)
    controller.target = TargetNed(0.0, 0.0, -1.2, 0.4)
    controller.local.heading = 0.0
    controller.set_velocity_key("w", now=10.0)

    controller.update_velocity_control(now=10.10, dt=0.10)
    assert controller.control_state == FlightControlState.VELOCITY_CONTROL
    assert 0.0 < np.linalg.norm(controller.velocity_command_ned[:2]) < 0.4
    assert controller.velocity_command_ned[1] == pytest.approx(0.0)

    advance_velocity(controller, start=10.10)
    assert controller.control_state == FlightControlState.VELOCITY_CONTROL
    assert controller.velocity_command_ned[0] == pytest.approx(0.4)
    assert controller.active_velocity_key == "w"
    assert controller.target.north == pytest.approx(0.0)
    assert controller.target.east == pytest.approx(0.0)
    assert controller.target.down == pytest.approx(-1.2)


def test_terminal_key_repeat_is_idempotent_and_does_not_relatch_heading(monkeypatch):
    quiet_logger(monkeypatch)
    controller = make_controller(-1.2)
    controller.control_state = FlightControlState.VELOCITY_CONTROL
    controller.local.heading = 0.25
    controller.set_velocity_key("w", now=10.0)
    controller.update_velocity_control(now=10.05, dt=0.05)

    first_velocity = controller.velocity_command_ned.copy()
    first_acceleration = controller.acceleration_command_ned.copy()
    first_yaw_hold = controller.yaw_hold_rad
    first_event_time = controller.last_velocity_key_monotonic

    # Simulate the auto-repeat characters produced by holding W in a terminal.
    controller.local.heading = 0.60
    controller.set_velocity_key("w", now=10.10)

    assert controller.active_velocity_key == "w"
    assert controller.last_velocity_key_monotonic == pytest.approx(first_event_time)
    assert controller.yaw_hold_rad == pytest.approx(first_yaw_hold)
    assert controller.yaw_hold_pending is False
    assert np.array_equal(controller.velocity_command_ned, first_velocity)
    assert np.array_equal(controller.acceleration_command_ned, first_acceleration)


def test_arm_coupling_state_callback_requires_complete_finite_vectors():
    controller = make_controller(-1.2)
    controller.arm_reaction_force_norm_n = float("nan")
    controller.arm_com_shift_norm_m = float("nan")
    controller.arm_inertia_diag_kg_m2 = np.full(3, float("nan"))
    controller.last_arm_coupling_state_monotonic = 0.0

    controller._arm_coupling_state_cb(SimpleNamespace(data="{}"))
    assert np.isnan(controller.arm_com_shift_norm_m)

    message = {
        "com_shift_m": [0.01, -0.02, 0.0],
        "inertia_diag_kg_m2": [0.20, 0.21, 0.22],
        "reaction_force_body_n": [0.3, 0.4, 0.0],
    }
    controller._arm_coupling_state_cb(SimpleNamespace(data=__import__("json").dumps(message)))

    assert controller.arm_com_shift_norm_m == pytest.approx(5.0 ** 0.5 / 100.0)
    assert controller.arm_reaction_force_norm_n == pytest.approx(0.5)
    assert controller.arm_inertia_diag_kg_m2 == pytest.approx([0.20, 0.21, 0.22])
    assert controller.last_arm_coupling_state_monotonic > 0.0


def test_h_is_the_only_manual_zero_velocity_command(monkeypatch):
    quiet_logger(monkeypatch)
    controller = make_controller(-1.2)
    controller.local.heading = 0.0
    controller.set_velocity_key("w", now=10.0)
    advance_velocity(controller)
    assert controller.velocity_command_ned[0] == pytest.approx(0.4)

    controller.hold_current_position()

    assert controller.control_state == FlightControlState.VELOCITY_CONTROL
    assert controller.active_velocity_key is None
    assert controller.velocity_command_ned[0] == pytest.approx(0.4)
    advance_velocity(controller, start=20.0)
    assert np.allclose(controller.velocity_command_ned, 0.0)


def test_vertical_velocity_remains_latched_without_automatic_release(monkeypatch):
    quiet_logger(monkeypatch)
    controller = make_controller(-1.2)
    controller.target = TargetNed(0.0, 0.0, -1.2, 0.4)
    controller.set_velocity_key("r", now=10.0)
    controller.velocity_command_ned[:] = 0.0
    controller.local.z = -1.5

    advance_velocity(controller)

    assert controller.control_state == FlightControlState.VELOCITY_CONTROL
    assert controller.velocity_command_ned[2] == pytest.approx(-0.15)
    assert controller.active_velocity_key == "r"
    assert controller.target.down == pytest.approx(-1.2)


def test_latched_velocity_is_independent_of_measured_speed(monkeypatch):
    quiet_logger(monkeypatch)
    controller = make_controller(-1.2)
    controller.target = TargetNed(0.0, 0.0, -1.2, 0.4)
    controller.set_velocity_key("w", now=10.0)
    controller.velocity_command_ned[:] = 0.0
    controller.local.vx = 0.20
    controller.local.vy = 0.0
    controller.local.vz = 0.0

    advance_velocity(controller)
    assert controller.control_state == FlightControlState.VELOCITY_CONTROL

    controller.local.vx = 0.04
    controller.update_velocity_control(now=15.05, dt=0.05)
    assert controller.control_state == FlightControlState.VELOCITY_CONTROL
    assert controller.active_velocity_key == "w"
    assert np.linalg.norm(controller.velocity_command_ned[:2]) == pytest.approx(0.4)


def test_horizontal_velocity_uses_current_heading(monkeypatch):
    quiet_logger(monkeypatch)
    controller = make_controller(-1.2)
    controller.local.heading = np.pi / 2.0
    controller.set_velocity_key("w", now=1.0)
    velocity, yaw_rate = controller._desired_velocity_ned(True)

    assert velocity[0] == pytest.approx(0.0, abs=1.0e-9)
    assert velocity[1] == pytest.approx(0.4)
    assert yaw_rate == 0.0


def test_yaw_to_translation_transition_ramps_to_zero_then_latches_heading(monkeypatch):
    quiet_logger(monkeypatch)
    controller = make_controller(-1.2)
    controller.control_state = FlightControlState.VELOCITY_CONTROL
    controller.local.heading = 0.20
    controller.set_velocity_key("q", now=1.0)
    for index in range(20):
        controller.update_velocity_control(now=1.05 + index * 0.05, dt=0.05)
    assert controller.yaw_rate_command < 0.0
    assert controller.yaw_hold_pending is False

    # A new translation key is the explicit "stop turning" event.  The yaw
    # rate must ramp, not jump, to zero.  Only once it reaches zero is the
    # measured heading frozen for finite-yaw setpoint publication.
    prior_rate = controller.yaw_rate_command
    controller.local.heading = 0.35
    controller.set_velocity_key("w", now=2.1)
    controller.update_velocity_control(now=2.15, dt=0.05)
    assert controller.yaw_rate_command > prior_rate
    assert controller.yaw_rate_command < 0.0
    assert controller.yaw_hold_pending is True

    for index in range(40):
        controller.local.heading = 0.35 + index * 0.001
        controller.update_velocity_control(now=2.20 + index * 0.05, dt=0.05)
        if not controller.yaw_hold_pending:
            break
    assert controller.yaw_rate_command == pytest.approx(0.0)
    assert controller.yaw_hold_pending is False
    assert controller.yaw_hold_rad == pytest.approx(controller.local.heading)
    desired, desired_yaw_rate = controller._desired_velocity_ned(True)
    assert desired_yaw_rate == pytest.approx(0.0)
    assert np.linalg.norm(desired[:2]) == pytest.approx(0.4)


def test_s_curve_respects_horizontal_acceleration_and_jerk_limits():
    velocity = np.zeros(2)
    acceleration = np.zeros(2)
    target = np.array([0.4, 0.0])
    previous_acceleration = acceleration.copy()
    dt = 0.05
    for _ in range(100):
        velocity, acceleration = DdsWasdControl._jerk_limited_vector_step(
            velocity, acceleration, target, 0.30, 0.60, dt
        )
        assert np.linalg.norm(acceleration) <= 0.30 + 1.0e-9
        assert np.linalg.norm(acceleration - previous_acceleration) <= 0.60 * dt + 1.0e-9
        assert velocity[0] <= 0.406
        previous_acceleration = acceleration.copy()
    assert velocity[0] == pytest.approx(0.4, abs=0.005)


def test_s_curve_reversal_does_not_cross_latched_speed_bound():
    velocity = np.zeros(1)
    acceleration = np.zeros(1)
    previous_acceleration = acceleration.copy()
    dt = 0.05
    limits = (0.18, 0.40)
    targets = [np.array([0.15])] * 60 + [np.array([-0.15])] * 120
    for target in targets:
        velocity, acceleration = DdsWasdControl._jerk_limited_vector_step(
            velocity, acceleration, target, *limits, dt
        )
        assert abs(velocity[0]) <= 0.151
        assert abs(acceleration[0]) <= limits[0] + 1.0e-9
        assert abs(acceleration[0] - previous_acceleration[0]) <= limits[1] * dt + 1.0e-9
        previous_acceleration = acceleration.copy()
    assert velocity[0] == pytest.approx(-0.15, abs=0.002)


def test_tuned_vertical_s_curve_defaults_preserve_speed_and_jerk_contract():
    assert DdsWasdControl.VERTICAL_SPEED_M_S == pytest.approx(0.15)
    assert DdsWasdControl.VERTICAL_ACCEL_LIMIT_M_S2 == pytest.approx(0.18)
    assert DdsWasdControl.VERTICAL_JERK_LIMIT_M_S3 == pytest.approx(0.40)
    assert DdsWasdControl.YAW_RATE_RAD_S == pytest.approx(np.deg2rad(15.0))
    assert DdsWasdControl.YAW_ACCEL_LIMIT_RAD_S2 == pytest.approx(np.deg2rad(20.0))


def test_position_and_velocity_setpoints_are_mutually_exclusive():
    class Recorder:
        def __init__(self):
            self.messages = []

        def publish(self, message):
            self.messages.append(message)

    controller = make_controller(-1.2)
    controller.target = TargetNed(1.0, 2.0, -1.2, 0.4)
    controller.mode_pub = Recorder()
    controller.setpoint_pub = Recorder()
    controller.now_us = lambda: 123
    controller.arm_feedforward_enabled = False
    controller.arm_motion_active = False

    controller.publish_hold()
    hold_mode = controller.mode_pub.messages[-1]
    hold = controller.setpoint_pub.messages[-1]
    assert hold_mode.position is True
    assert hold_mode.velocity is False
    assert np.all(np.isfinite(hold.position))
    assert np.all(np.isnan(hold.velocity))

    controller.velocity_command_ned[:] = [0.1, -0.2, 0.05]
    controller.yaw_rate_command = 0.2
    controller.publish_velocity()
    velocity_mode = controller.mode_pub.messages[-1]
    velocity = controller.setpoint_pub.messages[-1]
    assert velocity_mode.position is False
    assert velocity_mode.velocity is True
    assert np.all(np.isnan(velocity.position))
    assert np.all(np.isfinite(velocity.velocity))
    assert np.all(np.isfinite(velocity.acceleration))
    assert np.isnan(velocity.yaw)
    assert velocity.yawspeed == pytest.approx(0.2)

    controller.velocity_acceleration_feedforward_enabled = False
    controller.publish_velocity()
    no_acceleration_feedforward = controller.setpoint_pub.messages[-1]
    assert np.all(np.isnan(no_acceleration_feedforward.acceleration))

    controller.yaw_rate_command = 0.0
    controller.yaw_hold_rad = -0.3
    controller.publish_velocity()
    heading_hold = controller.setpoint_pub.messages[-1]
    assert heading_hold.yaw == pytest.approx(-0.3)
    assert np.isnan(heading_hold.yawspeed)
