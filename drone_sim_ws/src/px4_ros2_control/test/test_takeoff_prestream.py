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
    controller.yaw_rate_command = 0.0
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


def test_r_commands_up_and_f_commands_down_velocity(monkeypatch):
    quiet_logger(monkeypatch)
    controller = make_controller(-1.2, sign=-1.0)

    controller.set_velocity_key("r", now=10.0)
    velocity, _ = controller._desired_velocity_ned(True)
    assert velocity[2] == pytest.approx(-0.25)
    controller.set_velocity_key("f", now=10.1)
    velocity, _ = controller._desired_velocity_ned(True)
    assert velocity[2] == pytest.approx(0.25)


def test_hover_key_brakes_xy_but_preserves_commanded_altitude(monkeypatch):
    quiet_logger(monkeypatch)
    controller = make_controller(-1.45, sign=-1.0)
    controller.target = TargetNed(8.0, 9.0, -4.0, -1.0)
    controller.control_state = FlightControlState.VELOCITY_CONTROL
    controller.velocity_command_ned[:] = [0.2, -0.1, 0.05]
    controller.yaw_rate_command = 0.1

    controller.hold_current_position()

    assert controller.target.north == pytest.approx(1.0)
    assert controller.target.east == pytest.approx(-2.0)
    assert controller.target.down == pytest.approx(-4.0)
    assert controller.target.yaw == pytest.approx(0.4)
    assert controller.control_state == FlightControlState.POSITION_HOLD
    assert np.allclose(controller.velocity_command_ned, 0.0)
    assert controller.yaw_rate_command == 0.0


def test_key_heartbeat_ramps_velocity_then_releases_to_hold(monkeypatch):
    quiet_logger(monkeypatch)
    controller = make_controller(-1.2)
    controller.target = TargetNed(0.0, 0.0, -1.2, 0.4)
    controller.local.heading = 0.0
    controller.set_velocity_key("w", now=10.0)

    controller.update_velocity_control(now=10.10, dt=0.10)
    assert controller.control_state == FlightControlState.VELOCITY_CONTROL
    assert controller.velocity_command_ned[0] == pytest.approx(0.05)
    assert controller.velocity_command_ned[1] == pytest.approx(0.0)

    controller.update_velocity_control(now=10.31, dt=0.10)
    assert controller.control_state == FlightControlState.POSITION_HOLD
    assert np.allclose(controller.velocity_command_ned, 0.0)
    assert controller.target.north == pytest.approx(controller.local.x)
    assert controller.target.east == pytest.approx(controller.local.y)
    assert controller.target.down == pytest.approx(-1.2)


def test_vertical_velocity_release_holds_new_measured_altitude(monkeypatch):
    quiet_logger(monkeypatch)
    controller = make_controller(-1.2)
    controller.target = TargetNed(0.0, 0.0, -1.2, 0.4)
    controller.set_velocity_key("r", now=10.0)
    controller.velocity_command_ned[:] = 0.0
    controller.local.z = -1.5

    controller.update_velocity_control(now=10.31, dt=0.0)

    assert controller.control_state == FlightControlState.POSITION_HOLD
    assert controller.target.down == pytest.approx(-1.5)


def test_horizontal_velocity_uses_current_heading(monkeypatch):
    quiet_logger(monkeypatch)
    controller = make_controller(-1.2)
    controller.local.heading = np.pi / 2.0
    controller.set_velocity_key("w", now=1.0)
    velocity, yaw_rate = controller._desired_velocity_ned(True)

    assert velocity[0] == pytest.approx(0.0, abs=1.0e-9)
    assert velocity[1] == pytest.approx(0.4)
    assert yaw_rate == 0.0


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
    assert np.isnan(velocity.yaw)
    assert velocity.yawspeed == pytest.approx(0.2)
