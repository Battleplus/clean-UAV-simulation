from types import SimpleNamespace
import inspect
import threading
import time

import numpy as np
import pytest
import px4_ros2_control.dds_wasd_control as dds_module

from px4_ros2_control.dds_wasd_control import (
    DdsWasdControl,
    DirectXySafetyIngress,
    FlightControlState,
    RawTerminal,
    TargetNed,
    quaternion_wxyz_to_rotation_body_to_world,
    truth_hold_position_ned,
    truth_hold_velocity_ned,
)


def test_direct_xy_controller_safety_topics_use_reliable_latest_qos():
    ingress_source = inspect.getsource(DirectXySafetyIngress.__init__)
    controller_source = inspect.getsource(DdsWasdControl.__init__)

    for source in (ingress_source, controller_source):
        assert "reliability=ReliabilityPolicy.RELIABLE" in source
        assert "history=HistoryPolicy.KEEP_LAST" in source
        assert "depth=1" in source
    assert '"/my_drone/arm_direct_xy_guardian/state"' in ingress_source
    assert '"/my_drone/arm_direct_xy_force_command"' in ingress_source
    assert '"/my_drone/arm_direct_xy_controller_intent"' in controller_source


def test_direct_xy_safety_ingress_forwards_only_to_nonblocking_cache_callbacks():
    calls = []
    snapshot = (1.0, 1.001, {})
    controller = SimpleNamespace(
        ARM_DIRECT_XY_OWNERSHIP=False,
        _cache_arm_motion_active_cb=lambda msg: calls.append(("motion", msg)),
        _snapshot_arm_direct_xy_reallocator_state=lambda msg: (
            calls.append(("parse", msg)) or snapshot
        ),
        _cache_arm_direct_xy_reallocator_snapshot=lambda value: calls.append(
            ("reallocator", value)
        ),
    )
    ingress = SimpleNamespace(
        controller=controller,
        _observe_force_lease_snapshot=lambda value: calls.append(("lease", value)),
        _publish_active_receipt_ack_snapshot=lambda value: calls.append(
            ("ack", value)
        ),
    )
    motion = SimpleNamespace(data=True)
    state = SimpleNamespace(data="{}")

    DirectXySafetyIngress._arm_motion_cb(ingress, motion)
    DirectXySafetyIngress._reallocator_state_cb(ingress, state)

    assert calls == [
        ("motion", motion),
        ("parse", state),
        ("lease", snapshot),
        ("reallocator", snapshot),
        ("ack", snapshot),
    ]

    motion_source = inspect.getsource(DirectXySafetyIngress._arm_motion_cb)
    report_source = inspect.getsource(DirectXySafetyIngress._reallocator_state_cb)
    assert "_locked_arm_motion_active_cb" not in motion_source
    assert "_locked_arm_direct_xy_reallocator_state_cb" not in report_source
    assert report_source.count("_snapshot_arm_direct_xy_reallocator_state") == 1


def test_direct_xy_safety_ingress_publishes_only_actual_matching_epoch_ack():
    class Recorder:
        def __init__(self):
            self.messages = []

        def publish(self, message):
            self.messages.append(message)

    now = time.monotonic()
    controller = SimpleNamespace(
        ARM_DIRECT_XY_OWNERSHIP=True,
        ARM_DIRECT_XY_WATCHDOG_S=0.04,
        arm_direct_xy_ownership_epoch=4,
        arm_direct_xy_active=True,
        arm_direct_xy_abort_latched=False,
        arm_motion_is_active=lambda _now: True,
        _direct_xy_reallocator_report_is_healthy=(
            DdsWasdControl._direct_xy_reallocator_report_is_healthy
        ),
    )
    ingress = SimpleNamespace(
        controller=controller,
        owner_state_pub=Recorder(),
        last_active_receipt_ack_epoch=-1,
    )
    report = {
        "monotonic_s": now,
        "event": "allocated",
        "motion_active": True,
        "position_feedback_enabled": True,
        "position_feedback_ready": True,
        "position_feedback_prepared": True,
        "position_target_latched": True,
        "position_feedback_active": True,
        "direct_xy_force_command_fresh": True,
        "direct_xy_force_enabled": True,
        "direct_xy_force_epoch": 4,
        "source_fresh": True,
        "truth_fresh": True,
        "flight_allowed": True,
        "headroom_ok": True,
        "feasibility_scale": 1.0,
        "residual_norm": 0.0,
        "saturated": 0,
        "allocation_limited": False,
    }

    DirectXySafetyIngress._publish_active_receipt_ack_snapshot(
        ingress, (now, now + 0.001, report)
    )
    ack = __import__("json").loads(ingress.owner_state_pub.messages[-1].data)
    assert ack["state"] == "direct_xy"
    assert ack["ownership_epoch"] == 4
    assert ack["ack_source"] == "dedicated_safety_ingress"

    # A healthy report in the same epoch refreshes lease/cache elsewhere but
    # does not flood the reliable owner-state publisher.
    DirectXySafetyIngress._publish_active_receipt_ack_snapshot(
        ingress, (now + 0.002, now + 0.003, report)
    )
    assert len(ingress.owner_state_pub.messages) == 1

    report["direct_xy_force_epoch"] = 5
    controller.arm_direct_xy_ownership_epoch = 5
    DirectXySafetyIngress._publish_active_receipt_ack_snapshot(
        ingress, (now + 0.004, now + 0.005, report)
    )
    assert len(ingress.owner_state_pub.messages) == 2


def test_human_state_logging_is_not_in_the_control_tick():
    tick_source = inspect.getsource(DdsWasdControl._tick)
    report_source = inspect.getsource(DdsWasdControl._report_state)
    assert '"STATE "' not in tick_source
    assert '"STATE "' in report_source


def test_gazebo_body_velocity_rotates_to_world_enu():
    yaw_90_wxyz = [np.sqrt(0.5), 0.0, 0.0, np.sqrt(0.5)]
    rotation = quaternion_wxyz_to_rotation_body_to_world(yaw_90_wxyz)
    np.testing.assert_allclose(
        rotation @ np.asarray([1.0, 0.0, 0.0]),
        [0.0, 1.0, 0.0],
        atol=1e-12,
    )


def test_dds_watchdog_allows_slow_lockstep_status_period():
    assert DdsWasdControl.STATUS_TIMEOUT_S >= 5.0
    assert DdsWasdControl.LOCAL_POSITION_TIMEOUT_S <= 1.0
    assert (
        DdsWasdControl.LANDING_LOCAL_POSITION_TIMEOUT_S
        > DdsWasdControl.LOCAL_POSITION_TIMEOUT_S
    )
    assert DdsWasdControl.LANDING_LOCAL_POSITION_TIMEOUT_S <= 2.0
    assert DdsWasdControl.ACTUATOR_OUTPUT_TIMEOUT_S <= 1.0


def test_formal_touchdown_contact_allowance_defaults_to_zero():
    assert DdsWasdControl.TOUCHDOWN_CONTACT_ALLOWANCE_M == pytest.approx(0.0)


def test_dds_spin_callbacks_drains_bounded_ready_queue(monkeypatch):
    controller = object.__new__(DdsWasdControl)
    calls = []

    def spin_once(node, timeout_sec):
        assert node is controller
        calls.append(timeout_sec)

    monkeypatch.setattr(
        "px4_ros2_control.dds_wasd_control.rclpy.spin_once", spin_once
    )
    controller.spin_callbacks(0.02, drain_limit=4)
    assert calls == [0.02, 0.0, 0.0, 0.0, 0.0]


def test_terminal_reader_blocks_for_requested_poll_period(monkeypatch):
    calls = []

    def select_once(readers, writers, errors, timeout):
        calls.append((readers, writers, errors, timeout))
        return ([], [], [])

    monkeypatch.setattr("px4_ros2_control.dds_wasd_control.select.select", select_once)
    terminal = RawTerminal()
    assert terminal.read_key(0.02) is None
    assert calls[-1][3] == pytest.approx(0.02)


def test_executor_tick_serializes_on_shared_lock():
    controller = object.__new__(DdsWasdControl)
    controller.control_lock = threading.RLock()
    called = threading.Event()
    controller._consume_latest_arm_motion_snapshot = lambda: None
    controller._consume_latest_arm_direct_xy_report_snapshot = lambda: None
    controller._tick = lambda: called.set()

    with controller.control_lock:
        worker = threading.Thread(target=controller._locked_tick)
        worker.start()
        assert not called.wait(0.02)
    worker.join(timeout=1.0)
    assert not worker.is_alive()
    assert called.is_set()


def test_airborne_watchdog_separates_low_rate_status_from_local_position():
    controller = object.__new__(DdsWasdControl)
    now = time.monotonic()
    controller.status = SimpleNamespace()
    controller.local = SimpleNamespace(xy_valid=True, z_valid=True)
    controller.last_status_monotonic = now - DdsWasdControl.STATUS_TIMEOUT_S - 1.0
    controller.last_local_monotonic = now

    assert not controller.status_fresh()
    assert controller.local_position_fresh()
    assert not controller.state_fresh()


def make_controller(z: float, sign: float = -1.0):
    controller = object.__new__(DdsWasdControl)
    controller.local_z_sign = sign
    controller.local = SimpleNamespace(
        x=1.0, y=-2.0, z=z, heading=0.4, vx=0.0, vy=0.0, vz=0.0
    )
    controller.target_initialized = True
    controller.xy_reset_counter = 0
    controller.z_reset_counter = 0
    controller.heading_reset_counter = 0
    controller.takeoff_ground_down = z
    controller.control_state = FlightControlState.POSITION_HOLD
    controller.active_velocity_key = None
    controller.last_velocity_key_monotonic = 0.0
    controller.hover_transition_pending = False
    controller.velocity_command_ned = np.zeros(3)
    controller.acceleration_command_ned = np.zeros(3)
    controller.velocity_acceleration_feedforward_enabled = True
    controller.yaw_rate_command = 0.0
    controller.yaw_hold_rad = 0.4
    controller.yaw_hold_pending = False
    controller.gazebo_truth_enu = np.array([2.0, 1.0, 1.5])
    controller.gazebo_truth_velocity_enu = np.zeros(3)
    controller.last_gazebo_truth_monotonic = time.monotonic()
    controller.truth_hold_target_enu = None
    return controller


def test_truth_hold_outer_loop_uses_enu_to_ned_and_limits_speed():
    command = truth_hold_velocity_ned(
        np.array([1.0, 2.0, 3.0]),
        np.array([0.8, 2.1, 2.7]),
        np.array([0.1, -0.2, 0.05]),
        position_gain_xy=1.0,
        position_gain_z=1.0,
        velocity_damping_xy=0.5,
        velocity_damping_z=0.4,
        maximum_speed_xy=0.2,
        maximum_speed_z=0.12,
    )
    # ENU raw command is [0.15, 0.0, 0.28], so NED swaps XY and negates Z.
    np.testing.assert_allclose(command, [0.0, 0.15, -0.12], atol=1e-12)


def test_truth_hold_position_offset_cancels_ekf_origin_and_limits_error():
    target = truth_hold_position_ned(
        np.asarray([10.0, -4.0, -1.2]),
        np.asarray([1.0, 2.0, 3.0]),
        np.asarray([0.8, 2.1, 2.7]),
        position_gain=1.0,
        maximum_offset_xy_m=0.15,
        maximum_offset_z_m=0.12,
    )
    # Truth ENU correction [+.2,-.1,+.3] is bounded to 0.15 horizontally
    # and 0.12 vertically, then mapped to NED around the current EKF point.
    horizontal = np.asarray([-0.1, 0.2]) * (0.15 / np.hypot(0.1, 0.2))
    np.testing.assert_allclose(
        target, [10.0 + horizontal[0], -4.0 + horizontal[1], -1.32], atol=1e-12
    )


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
    logger = SimpleNamespace(
        info=lambda *_args: None,
        warning=lambda *_args: None,
        error=lambda *_args: None,
    )
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


def test_hover_key_brakes_then_locks_current_position(monkeypatch):
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
    assert controller.hover_transition_pending is True
    assert controller.active_velocity_key is None
    assert np.allclose(controller.velocity_command_ned, [0.2, -0.1, 0.05])
    advance_velocity(controller)
    assert np.allclose(controller.velocity_command_ned, 0.0)
    assert controller.yaw_rate_command == pytest.approx(0.0)
    assert controller.complete_hover_transition_if_ready() is True
    assert controller.control_state == FlightControlState.POSITION_HOLD
    assert controller.target.north == pytest.approx(controller.local.x)
    assert controller.target.east == pytest.approx(controller.local.y)
    assert controller.target.down == pytest.approx(controller.local.z)


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
    assert controller.complete_hover_transition_if_ready() is True
    assert controller.control_state == FlightControlState.POSITION_HOLD


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
    assert DdsWasdControl.YAW_ACCEL_LIMIT_RAD_S2 == pytest.approx(np.deg2rad(15.0))


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


def _make_direct_xy_controller(monkeypatch):
    quiet_logger(monkeypatch)

    events = []

    class Recorder:
        def __init__(self, name):
            self.name = name
            self.messages = []

        def publish(self, message):
            self.messages.append(message)
            events.append((self.name, message))

    controller = make_controller(-1.2)
    controller.target = TargetNed(1.0, -2.0, -1.2, 0.4)
    now = time.monotonic()
    controller.ARM_DIRECT_XY_OWNERSHIP = True
    controller.TRUTH_HOLD_ENABLED = True
    controller.TRUTH_HOLD_ARM_POSITION_OVERLAY = False
    controller.arm_feedforward_enabled = False
    controller.truth_hold_stale_reported = False
    controller.truth_hold_target_enu = controller.gazebo_truth_enu.copy()
    controller.gazebo_truth_velocity_filtered_enu = np.zeros(3)
    controller.gazebo_truth_rpy = np.zeros(3)
    controller.status = SimpleNamespace(
        arming_state=2,
        failsafe=False,
    )
    controller.last_status_monotonic = now
    controller.arm_motion_active = True
    controller.last_arm_motion_monotonic = now
    controller.arm_direct_xy_reallocator_healthy = True
    controller.arm_direct_xy_position_feedback_active = False
    controller.arm_direct_xy_position_feedback_prepared = True
    controller.arm_direct_xy_position_feedback_ready = True
    controller.arm_direct_xy_force_enabled_ack = False
    controller.arm_direct_xy_force_epoch_ack = -1
    controller.arm_direct_xy_force_commanded = False
    controller.arm_direct_xy_force_ever_ack = False
    controller.arm_direct_xy_exit_disable_pending = False
    controller.arm_direct_xy_health_since = now - 1.0
    controller.last_arm_direct_xy_state_monotonic = now
    controller.arm_direct_xy_stable_since = now - 1.0
    controller.arm_direct_xy_active = False
    controller.arm_direct_xy_abort_latched = False
    controller.arm_direct_xy_last_fault = ""
    controller.arm_direct_xy_ownership_epoch = 7
    controller.arm_direct_xy_preauthorized = True
    controller.arm_direct_xy_entry_deadline = now + 1.0
    controller.arm_direct_xy_prepared_monotonic = now
    controller.arm_direct_xy_force_ack_deadline = 0.0
    controller.arm_direct_xy_force_ack_monotonic = 0.0
    controller.last_arm_direct_xy_report_producer_monotonic = 0.0
    controller.latest_arm_direct_xy_report_queued_producer_monotonic = 0.0
    controller.arm_direct_xy_report_cache_lock = threading.Lock()
    controller.arm_direct_xy_pending_report_snapshot = None
    controller.arm_direct_xy_pending_fault_snapshot = None
    controller.last_arm_direct_xy_healthy_receipt_monotonic = now
    controller.arm_motion_report_cache_lock = threading.Lock()
    controller.arm_motion_pending_snapshot = None
    controller.arm_direct_xy_guardian_cache_lock = threading.Lock()
    controller.arm_direct_xy_guardian_pending_snapshot = None
    controller.control_lock = threading.RLock()
    controller.last_joint_state_monotonic = now
    controller.rl_joint_names = (
        "shoulder_pan", "shoulder_lift", "elbow_flex",
        "wrist_flex", "wrist_roll", "gripper",
    )
    controller.rl_joint_positions = np.arange(6, dtype=float) * 0.01
    controller.mode_pub = Recorder("mode")
    controller.setpoint_pub = Recorder("setpoint")
    controller.arm_direct_xy_ready_pub = Recorder("ready")
    controller.arm_motion_inhibit_pub = Recorder("inhibit")
    controller.arm_direct_xy_state_pub = Recorder("state")
    controller.arm_direct_xy_force_command_pub = Recorder("force_command")
    controller.rl_arm_pub = Recorder("arm")
    controller.rl_arm_motion_pub = Recorder("arm_motion")
    controller.landing_pub = Recorder("landing")
    controller.command_pub = Recorder("command")
    controller.offboard_requested = True
    controller.pending_takeoff = False
    controller.landing_requested = False
    controller.offboard_landing_active = False
    controller.test_publish_events = events
    controller.now_us = lambda: 123
    return controller


def test_external_guardian_mode_publishes_intent_not_physical_force(monkeypatch):
    controller = _make_direct_xy_controller(monkeypatch)
    recorder_type = type(controller.arm_direct_xy_state_pub)
    controller.ARM_DIRECT_XY_EXTERNAL_GUARDIAN = True
    controller.arm_direct_xy_guardian_session_id = "controller-boot-a"
    controller.arm_direct_xy_intent_pub = recorder_type("intent")
    controller.arm_direct_xy_force_lease_lock = threading.Lock()
    controller.arm_direct_xy_force_lease_generation = 3
    handoffs = []
    controller._start_event_driven_direct_xy_handoff = (
        lambda received: handoffs.append(received) or True
    )
    controller.arm_direct_xy_active = True
    controller.test_publish_events.clear()
    controller.setpoint_pub.publish(SimpleNamespace())
    controller._request_arm_direct_xy_force_after_mixed_setpoint(time.monotonic())
    intent_report = __import__("json").loads(
        controller.arm_direct_xy_intent_pub.messages[-1].data
    )
    assert intent_report["schema"] == "my_drone.arm-direct-xy-controller-intent.v1"
    assert intent_report["controller_session_id"] == "controller-boot-a"
    assert intent_report["ownership_epoch"] == 7
    assert intent_report["intent_generation"] == 4
    assert intent_report["force_requested"] is True
    assert controller.arm_direct_xy_force_command_pub.messages == []
    names = [name for name, _message in controller.test_publish_events]
    assert names.index("setpoint") < names.index("intent")


def test_external_guardian_abort_diagnostic_cannot_revoke_before_finite_xy(
    monkeypatch,
):
    controller = _make_direct_xy_controller(monkeypatch)
    recorder_type = type(controller.arm_direct_xy_state_pub)
    controller.ARM_DIRECT_XY_EXTERNAL_GUARDIAN = True
    controller.arm_direct_xy_guardian_session_id = "controller-boot-a"
    controller.arm_direct_xy_intent_pub = recorder_type("intent")
    controller.arm_direct_xy_force_lease_lock = threading.Lock()
    controller.arm_direct_xy_force_lease_generation = 3
    _activate_direct_xy_owner(controller)
    controller.test_publish_events.clear()

    controller._abort_arm_for_direct_xy_fault("guardian_test")
    assert controller.arm_direct_xy_intent_pub.messages == []

    controller._restore_finite_xy_and_disable_direct_force()
    finite_index = next(
        index
        for index, (name, message) in enumerate(controller.test_publish_events)
        if name == "setpoint" and np.all(np.isfinite(message.velocity[:2]))
    )
    intent_index = next(
        index
        for index, (name, message) in enumerate(controller.test_publish_events)
        if name == "intent"
        and __import__("json").loads(message.data)["force_requested"] is False
    )
    assert finite_index < intent_index


def test_external_guardian_exit_completes_only_on_exact_physical_disable_ack(
    monkeypatch,
):
    controller = _make_direct_xy_controller(monkeypatch)
    recorder_type = type(controller.arm_direct_xy_state_pub)
    controller.ARM_DIRECT_XY_EXTERNAL_GUARDIAN = True
    controller.arm_direct_xy_guardian_session_id = "controller-boot-a"
    controller.arm_direct_xy_intent_pub = recorder_type("intent")
    controller.arm_direct_xy_force_lease_lock = threading.Lock()
    controller.arm_direct_xy_force_lease_generation = 3
    controller.arm_direct_xy_force_lease_requested = True
    controller.arm_direct_xy_force_lease_epoch = 7
    _activate_direct_xy_owner(controller)

    controller.test_publish_events.clear()
    controller._arm_motion_active_cb(SimpleNamespace(data=False))

    assert controller.arm_direct_xy_exit_disable_pending is True
    assert controller.arm_direct_xy_force_enabled_ack is True
    exit_intent = __import__("json").loads(
        controller.arm_direct_xy_intent_pub.messages[-1].data
    )
    assert exit_intent["state"] == "exit"
    assert exit_intent["intent_generation"] == 4
    assert exit_intent["force_requested"] is False

    # Guardian exit is still only a pending physical handoff.
    exit_state = {
        "schema": "my_drone.arm-direct-xy-state.v1",
        "controller_session_id": "controller-boot-a",
        "ownership_epoch": 7,
        "intent_generation": 4,
        "lease_generation": 9,
        "state": "exit",
        "reallocator_fresh": False,
        "position_feedback_active": True,
        "position_feedback_prepared": False,
        "position_feedback_ready": False,
        "direct_xy_force_enabled_ack": True,
    }
    controller._cache_external_guardian_state(
        SimpleNamespace(data=__import__("json").dumps(exit_state))
    )
    controller._consume_external_guardian_state()
    assert controller.arm_direct_xy_exit_disable_pending is True
    assert controller.arm_direct_xy_force_enabled_ack is True

    # A queued pre-exit px4_xy level cannot acknowledge the newer intent.
    stale_ack = dict(exit_state)
    stale_ack.update(
        intent_generation=3,
        state="px4_xy",
        reallocator_fresh=True,
        position_feedback_active=False,
        direct_xy_force_enabled_ack=False,
    )
    controller._cache_external_guardian_state(
        SimpleNamespace(data=__import__("json").dumps(stale_ack))
    )
    controller._consume_external_guardian_state()
    assert controller.arm_direct_xy_exit_disable_pending is True

    physical_ack = dict(stale_ack)
    physical_ack["intent_generation"] = 4
    controller._cache_external_guardian_state(
        SimpleNamespace(data=__import__("json").dumps(physical_ack))
    )
    controller._consume_external_guardian_state()
    assert controller.arm_direct_xy_exit_disable_pending is False
    assert controller.arm_direct_xy_force_enabled_ack is False
    assert controller.arm_direct_xy_force_ever_ack is False


def test_external_guardian_exit_refresh_does_not_change_intent_generation(
    monkeypatch,
):
    controller = _make_direct_xy_controller(monkeypatch)
    recorder_type = type(controller.arm_direct_xy_state_pub)
    controller.ARM_DIRECT_XY_EXTERNAL_GUARDIAN = True
    controller.arm_direct_xy_guardian_session_id = "controller-boot-a"
    controller.arm_direct_xy_intent_pub = recorder_type("intent")
    controller.arm_direct_xy_force_lease_lock = threading.Lock()
    controller.arm_direct_xy_force_lease_generation = 3
    controller.arm_direct_xy_force_lease_requested = True
    controller.arm_direct_xy_force_lease_epoch = 7
    _activate_direct_xy_owner(controller)

    controller._arm_motion_active_cb(SimpleNamespace(data=False))
    first = __import__("json").loads(
        controller.arm_direct_xy_intent_pub.messages[-1].data
    )
    controller.publish_truth_hold()
    second = __import__("json").loads(
        controller.arm_direct_xy_intent_pub.messages[-1].data
    )
    assert first["state"] == second["state"] == "exit"
    assert first["intent_generation"] == second["intent_generation"] == 4
    assert controller.arm_direct_xy_exit_disable_pending is True


def test_motion_false_transition_excludes_stale_mixed_keepalive_intent(
    monkeypatch,
):
    """The old mixed repeater cannot observe a half-applied falling edge."""
    controller = _make_direct_xy_controller(monkeypatch)
    recorder_type = type(controller.arm_direct_xy_state_pub)
    controller.ARM_DIRECT_XY_EXTERNAL_GUARDIAN = True
    controller.arm_direct_xy_guardian_session_id = "controller-boot-a"
    controller.arm_direct_xy_intent_pub = recorder_type("intent")
    controller.arm_direct_xy_force_lease_lock = threading.Lock()
    controller.arm_direct_xy_intent_transition_lock = threading.RLock()
    controller.arm_direct_xy_force_lease_generation = 3
    controller.arm_direct_xy_force_lease_requested = True
    controller.arm_direct_xy_force_lease_epoch = 7
    _activate_direct_xy_owner(controller)
    controller.test_publish_events.clear()

    # Hold the transition authority, queue the isolated keepalive intent, then
    # apply motion=False recursively on the owning thread.  The queued writer
    # may run only after finite PX4 XY plus the exit intent are complete.
    with controller.arm_direct_xy_intent_transition_lock:
        worker = threading.Thread(
            target=lambda: controller._publish_external_guardian_intent(
                "direct_xy", time.monotonic()
            )
        )
        worker.start()
        time.sleep(0.01)
        assert worker.is_alive()
        controller._arm_motion_active_cb(SimpleNamespace(data=False))
    worker.join(timeout=0.2)
    assert not worker.is_alive()

    payloads = [
        __import__("json").loads(message.data)
        for message in controller.arm_direct_xy_intent_pub.messages
    ]
    assert any(payload["state"] == "exit" for payload in payloads)
    assert not any(
        payload["force_requested"] is True
        and (
            payload["motion_active"] is False
            or payload["mixed_setpoint_active"] is False
        )
        for payload in payloads
    )
    finite_index = next(
        index
        for index, (name, message) in enumerate(controller.test_publish_events)
        if name == "setpoint" and np.all(np.isfinite(message.velocity[:2]))
    )
    exit_index = next(
        index
        for index, (name, message) in enumerate(controller.test_publish_events)
        if name == "intent"
        and __import__("json").loads(message.data)["state"] == "exit"
    )
    assert finite_index < exit_index


def test_mixed_keepalive_intent_resamples_clock_after_motion_transition_lock(
    monkeypatch,
):
    """A newer motion heartbeat cannot look future-dated to a repeat intent."""
    controller = _make_direct_xy_controller(monkeypatch)
    recorder_type = type(controller.arm_direct_xy_state_pub)
    controller.ARM_DIRECT_XY_EXTERNAL_GUARDIAN = True
    controller.arm_direct_xy_guardian_session_id = "controller-boot-a"
    controller.arm_direct_xy_intent_pub = recorder_type("intent")
    controller.arm_direct_xy_force_lease_lock = threading.Lock()
    controller.arm_direct_xy_intent_transition_lock = threading.RLock()
    controller.arm_direct_xy_force_lease_generation = 3
    controller.arm_direct_xy_force_lease_requested = True
    controller.arm_direct_xy_force_lease_epoch = 7
    _activate_direct_xy_owner(controller)

    clock_called = threading.Event()

    def locked_clock():
        clock_called.set()
        return 100.101

    monkeypatch.setattr(dds_module.time, "monotonic", locked_clock)
    with controller.arm_direct_xy_intent_transition_lock:
        worker = threading.Thread(
            target=lambda: controller._publish_external_guardian_intent(
                "direct_xy", 100.000
            )
        )
        worker.start()
        time.sleep(0.01)
        assert worker.is_alive()
        assert not clock_called.is_set(), "intent clock must be read under transition lock"
        # Reproduce v8: a newer heartbeat is committed after the keepalive's
        # pre-publish sample but before its guardian-intent snapshot.
        controller._apply_arm_motion_active_level(
            SimpleNamespace(data=True), observed_monotonic=100.100
        )

    worker.join(timeout=0.2)
    assert not worker.is_alive()
    assert clock_called.is_set()
    intent = __import__("json").loads(
        controller.arm_direct_xy_intent_pub.messages[-1].data
    )
    assert intent["monotonic_s"] == pytest.approx(100.101)
    assert intent["force_requested"] is True
    assert intent["mixed_setpoint_active"] is True
    assert intent["motion_active"] is True


def test_external_guardian_owner_level_is_cache_only_until_control_tick(monkeypatch):
    controller = _make_direct_xy_controller(monkeypatch)
    controller.ARM_DIRECT_XY_EXTERNAL_GUARDIAN = True
    controller.arm_direct_xy_guardian_session_id = "controller-boot-a"
    controller.arm_direct_xy_guardian_cache_lock = threading.Lock()
    controller.arm_direct_xy_guardian_pending_snapshot = None
    controller.arm_direct_xy_force_lease_lock = threading.Lock()
    controller.arm_direct_xy_force_lease_generation = 3
    controller.arm_motion_active = False
    controller.arm_direct_xy_reallocator_healthy = False
    state = {
        "schema": "my_drone.arm-direct-xy-state.v1",
        "controller_session_id": "controller-boot-a",
        "ownership_epoch": 7,
        "intent_generation": 3,
        "lease_generation": 4,
        "state": "px4_xy",
        "reallocator_fresh": True,
        "position_feedback_active": False,
        "position_feedback_prepared": False,
        "position_feedback_ready": True,
        "direct_xy_force_enabled_ack": False,
    }
    controller._cache_external_guardian_state(
        SimpleNamespace(data=__import__("json").dumps(state))
    )
    assert controller.arm_direct_xy_reallocator_healthy is False
    controller._consume_external_guardian_state()
    assert controller.arm_direct_xy_reallocator_healthy is True
    assert controller.arm_direct_xy_position_feedback_ready is True


def test_external_guardian_prepared_level_wakes_handoff_immediately(monkeypatch):
    """v12 regression: do not wait another 20 Hz flight-control tick."""
    controller = _make_direct_xy_controller(monkeypatch)
    controller.ARM_DIRECT_XY_EXTERNAL_GUARDIAN = True
    controller.arm_direct_xy_guardian_session_id = "controller-boot-a"
    controller.arm_direct_xy_force_lease_lock = threading.Lock()
    controller.arm_direct_xy_force_lease_generation = 3
    controller.arm_direct_xy_position_feedback_prepared = False
    controller.arm_direct_xy_prepared_monotonic = 0.0
    handoffs = []
    controller._start_event_driven_direct_xy_handoff = (
        lambda received: handoffs.append(received) or True
    )
    state = {
        "schema": "my_drone.arm-direct-xy-state.v1",
        "controller_session_id": "controller-boot-a",
        "ownership_epoch": 7,
        "intent_generation": 3,
        "lease_generation": 4,
        "state": "px4_xy",
        "reallocator_fresh": True,
        "position_feedback_active": False,
        "position_feedback_prepared": True,
        "position_feedback_ready": True,
        "direct_xy_force_enabled_ack": False,
    }

    controller._locked_external_guardian_state_cb(
        SimpleNamespace(data=__import__("json").dumps(state))
    )

    assert len(handoffs) == 1
    assert controller.arm_direct_xy_guardian_pending_snapshot is None
    assert controller.arm_direct_xy_position_feedback_prepared is True


def test_external_guardian_event_never_waits_for_busy_control_lock(monkeypatch):
    controller = _make_direct_xy_controller(monkeypatch)
    controller.ARM_DIRECT_XY_EXTERNAL_GUARDIAN = True
    controller.arm_direct_xy_guardian_session_id = "controller-boot-a"
    controller.arm_direct_xy_force_lease_lock = threading.Lock()
    controller.arm_direct_xy_force_lease_generation = 3
    handoffs = []
    controller._start_event_driven_direct_xy_handoff = (
        lambda received: handoffs.append(received) or True
    )
    state = {
        "schema": "my_drone.arm-direct-xy-state.v1",
        "controller_session_id": "controller-boot-a",
        "ownership_epoch": 7,
        "intent_generation": 3,
        "lease_generation": 4,
        "state": "px4_xy",
        "reallocator_fresh": True,
        "position_feedback_active": False,
        "position_feedback_prepared": True,
        "position_feedback_ready": True,
        "direct_xy_force_enabled_ack": False,
    }
    message = SimpleNamespace(data=__import__("json").dumps(state))

    with controller.control_lock:
        worker = threading.Thread(
            target=lambda: controller._locked_external_guardian_state_cb(message)
        )
        worker.start()
        worker.join(timeout=0.1)
        assert not worker.is_alive(), "guardian ingress must not queue on flight work"
        assert controller.arm_direct_xy_guardian_pending_snapshot is not None

    controller._consume_external_guardian_state()
    assert controller.arm_direct_xy_guardian_pending_snapshot is None
    assert controller.arm_direct_xy_position_feedback_prepared is True
    assert len(handoffs) == 1


def test_external_guardian_is_consumed_just_before_runtime_watchdog(monkeypatch):
    controller = _make_direct_xy_controller(monkeypatch)
    recorder_type = type(controller.arm_direct_xy_state_pub)
    controller.ARM_DIRECT_XY_EXTERNAL_GUARDIAN = True
    controller.arm_direct_xy_guardian_session_id = "controller-boot-a"
    controller.arm_direct_xy_guardian_cache_lock = threading.Lock()
    controller.arm_direct_xy_guardian_pending_snapshot = None
    controller.arm_direct_xy_force_lease_lock = threading.Lock()
    controller.arm_direct_xy_force_lease_generation = 3
    controller.arm_direct_xy_intent_pub = recorder_type("intent")
    _activate_direct_xy_owner(controller)
    controller.last_arm_direct_xy_healthy_receipt_monotonic = (
        time.monotonic()
        - controller.ARM_DIRECT_XY_GUARDIAN_STATE_TIMEOUT_S
        - 0.01
    )
    state = {
        "schema": "my_drone.arm-direct-xy-state.v1",
        "controller_session_id": "controller-boot-a",
        "ownership_epoch": controller.arm_direct_xy_ownership_epoch,
        "intent_generation": 3,
        "lease_generation": 4,
        "state": "direct_xy",
        "reallocator_fresh": True,
        "position_feedback_active": True,
        "position_feedback_prepared": True,
        "position_feedback_ready": True,
        "direct_xy_force_enabled_ack": True,
    }
    controller._cache_external_guardian_state(
        SimpleNamespace(data=__import__("json").dumps(state))
    )

    controller.publish_truth_hold()

    assert controller.arm_direct_xy_guardian_pending_snapshot is None
    assert controller.arm_direct_xy_abort_latched is False
    assert controller.arm_direct_xy_active is True
    setpoint = controller.setpoint_pub.messages[-1]
    assert np.all(np.isnan(setpoint.velocity[:2]))


def test_external_guardian_identity_mismatch_cannot_mask_stale_runtime_owner(
    monkeypatch,
):
    controller = _make_direct_xy_controller(monkeypatch)
    recorder_type = type(controller.arm_direct_xy_state_pub)
    controller.ARM_DIRECT_XY_EXTERNAL_GUARDIAN = True
    controller.arm_direct_xy_guardian_session_id = "controller-boot-a"
    controller.arm_direct_xy_guardian_cache_lock = threading.Lock()
    controller.arm_direct_xy_guardian_pending_snapshot = None
    controller.arm_direct_xy_force_lease_lock = threading.Lock()
    controller.arm_direct_xy_force_lease_generation = 3
    controller.arm_direct_xy_intent_pub = recorder_type("intent")
    _activate_direct_xy_owner(controller)
    controller.last_arm_direct_xy_healthy_receipt_monotonic = (
        time.monotonic()
        - controller.ARM_DIRECT_XY_GUARDIAN_STATE_TIMEOUT_S
        - 0.01
    )
    state = {
        "schema": "my_drone.arm-direct-xy-state.v1",
        "controller_session_id": "wrong-session",
        "ownership_epoch": controller.arm_direct_xy_ownership_epoch,
        "intent_generation": 3,
        "lease_generation": 4,
        "state": "direct_xy",
        "reallocator_fresh": True,
        "position_feedback_active": True,
        "position_feedback_prepared": True,
        "position_feedback_ready": True,
        "direct_xy_force_enabled_ack": True,
    }
    controller._cache_external_guardian_state(
        SimpleNamespace(data=__import__("json").dumps(state))
    )

    controller.publish_truth_hold()

    assert controller.arm_direct_xy_abort_latched is True
    assert controller.arm_direct_xy_active is False
    setpoint = controller.setpoint_pub.messages[-1]
    assert np.all(np.isfinite(setpoint.velocity))


def test_direct_xy_runtime_fault_diagnostic_preserves_strict_40ms_boundary(
    monkeypatch,
):
    controller = _make_direct_xy_controller(monkeypatch)
    now = time.monotonic()
    controller.last_gazebo_truth_monotonic = now
    controller.last_status_monotonic = now
    controller.last_arm_direct_xy_healthy_receipt_monotonic = (
        now - controller.ARM_DIRECT_XY_WATCHDOG_S + 1.0e-6
    )
    assert controller._arm_direct_xy_runtime_fault_reason(now) is None

    controller.last_arm_direct_xy_healthy_receipt_monotonic = (
        now - controller.ARM_DIRECT_XY_WATCHDOG_S - 1.0e-6
    )
    assert (
        controller._arm_direct_xy_runtime_fault_reason(now)
        == "reallocator_heartbeat_stale"
    )


def test_external_guardian_transport_has_separate_bounded_deadline(monkeypatch):
    controller = _make_direct_xy_controller(monkeypatch)
    controller.ARM_DIRECT_XY_EXTERNAL_GUARDIAN = True
    now = time.monotonic()
    controller.last_gazebo_truth_monotonic = now
    controller.last_status_monotonic = now
    controller.last_arm_direct_xy_healthy_receipt_monotonic = (
        now - controller.ARM_DIRECT_XY_WATCHDOG_S - 0.001
    )

    # The physical producer remains protected by the independent guardian's
    # unchanged 40 ms gate.  This process judges only guardian-state transport.
    assert controller._arm_direct_xy_runtime_fault_reason(now) is None

    controller.last_arm_direct_xy_healthy_receipt_monotonic = (
        now - controller.ARM_DIRECT_XY_GUARDIAN_STATE_TIMEOUT_S - 1.0e-6
    )
    assert (
        controller._arm_direct_xy_runtime_fault_reason(now)
        == "guardian_state_stale"
    )
    assert controller.ARM_DIRECT_XY_WATCHDOG_S == pytest.approx(0.04)
    assert controller.ARM_DIRECT_XY_GUARDIAN_STATE_TIMEOUT_S == pytest.approx(0.15)
    assert (
        controller.ARM_DIRECT_XY_GUARDIAN_STATE_TIMEOUT_S
        + 1.0 / controller.RATE_HZ
        <= 0.200001
    )


def test_direct_xy_runtime_fault_diagnostic_prioritizes_pending_fault(
    monkeypatch,
):
    controller = _make_direct_xy_controller(monkeypatch)
    now = time.monotonic()
    controller.last_gazebo_truth_monotonic = now
    controller.last_status_monotonic = now
    controller.arm_direct_xy_pending_fault_snapshot = (now, now, {})

    assert (
        controller._arm_direct_xy_runtime_fault_reason(now)
        == "reallocator_pending_fault"
    )


def test_runtime_fault_clock_is_sampled_after_concurrent_truth_timestamps(
    monkeypatch,
):
    controller = _make_direct_xy_controller(monkeypatch)
    controller.last_arm_direct_xy_healthy_receipt_monotonic = 100.0
    controller.last_gazebo_truth_monotonic = 100.005
    controller.last_status_monotonic = 100.005
    monkeypatch.setattr(dds_module.time, "monotonic", lambda: 100.006)

    assert controller._arm_direct_xy_runtime_fault_reason(100.0) is None


def test_runtime_fault_rejects_timestamp_ahead_of_resampled_clock(monkeypatch):
    controller = _make_direct_xy_controller(monkeypatch)
    controller.last_arm_direct_xy_healthy_receipt_monotonic = 100.0
    controller.last_gazebo_truth_monotonic = 100.010
    controller.last_status_monotonic = 100.005
    monkeypatch.setattr(dds_module.time, "monotonic", lambda: 100.006)

    assert (
        controller._arm_direct_xy_runtime_fault_reason(100.0)
        == "truth_timestamp_future"
    )


def _activate_direct_xy_owner(controller):
    controller.arm_direct_xy_active = True
    controller.arm_direct_xy_force_commanded = True
    controller.arm_direct_xy_force_enabled_ack = True
    controller.arm_direct_xy_force_ever_ack = True
    controller.arm_direct_xy_force_epoch_ack = controller.arm_direct_xy_ownership_epoch


def _direct_xy_reallocator_report(controller, *, active=False, producer_time=None):
    return {
        "monotonic_s": time.monotonic() if producer_time is None else producer_time,
        "event": "allocated" if active else "zero_overlay",
        "source_fresh": True,
        "flight_allowed": True,
        "headroom_ok": True,
        "motion_active": True,
        "position_feedback_enabled": True,
        "position_feedback_ready": True,
        "position_feedback_active": bool(active),
        "position_feedback_prepared": True,
        "position_target_latched": True,
        "direct_xy_force_command_fresh": bool(active),
        "direct_xy_force_enabled": bool(active),
        "direct_xy_force_epoch": (
            controller.arm_direct_xy_ownership_epoch if active else -1
        ),
        "truth_fresh": True,
        "allocation_limited": False,
        "feasibility_scale": 1.0,
        "residual_norm": 0.0,
        "saturated": 0,
    }


def _deliver_direct_xy_report(controller, *, active=False, producer_time=None):
    report = _direct_xy_reallocator_report(
        controller, active=active, producer_time=producer_time
    )
    controller._arm_direct_xy_reallocator_state_cb(
        SimpleNamespace(data=__import__("json").dumps(report))
    )
    return report


def test_runtime_safety_cache_never_applies_owner_transition_when_main_lock_is_free(
    monkeypatch,
):
    controller = _make_direct_xy_controller(monkeypatch)
    report = _direct_xy_reallocator_report(controller, active=True)
    apply_calls = []

    def forbidden_apply(_snapshot):
        apply_calls.append(True)
        raise AssertionError("safety ingress must not apply an owner transition")

    controller._apply_arm_direct_xy_reallocator_snapshot = forbidden_apply
    controller._cache_arm_direct_xy_reallocator_state_cb(
        SimpleNamespace(data=__import__("json").dumps(report))
    )

    assert apply_calls == []
    assert controller.arm_direct_xy_active is False
    assert controller.arm_direct_xy_pending_report_snapshot is not None

    motion_calls = []
    controller._arm_motion_active_cb = lambda *_args, **_kwargs: motion_calls.append(True)
    controller._cache_arm_motion_active_cb(SimpleNamespace(data=True))
    assert motion_calls == []
    assert controller.arm_motion_pending_snapshot is not None


def _start_independent_force_lease(controller, start_s):
    controller.arm_direct_xy_active = True
    controller.arm_direct_xy_abort_latched = False
    controller.arm_motion_active = True
    controller.last_arm_motion_monotonic = float(start_s)
    controller._request_arm_direct_xy_force_after_mixed_setpoint(start_s)
    # Runtime bootstrap is emitted by the independent 10 ms safety timer, not
    # synchronously from the main 20 Hz handoff callback.
    assert controller._refresh_arm_direct_xy_force_lease(start_s + 0.0005)
    report = _direct_xy_reallocator_report(
        controller, active=True, producer_time=start_s + 0.001
    )
    assert controller._observe_arm_direct_xy_force_lease_report(
        report, received_monotonic=start_s + 0.002
    )
    return report


def _force_lease_commands(controller):
    return [
        __import__("json").loads(message.data)
        for message in controller.arm_direct_xy_force_command_pub.messages
    ]


def test_direct_xy_force_lease_refresh_is_independent_of_250ms_main_lock(
    monkeypatch,
):
    controller = _make_direct_xy_controller(monkeypatch)
    start_s = time.monotonic()
    _start_independent_force_lease(controller, start_s)
    controller.arm_direct_xy_force_command_pub.messages.clear()

    # Simulate a main control callback retaining control_lock for almost the
    # complete 0.25 s liveness lease.  Fresh reallocator receipts continue on
    # the independent safety executor and therefore never need this lock.
    controller.control_lock.acquire()
    try:
        for offset in (0.04, 0.08, 0.12, 0.16, 0.20, 0.24):
            report = _direct_xy_reallocator_report(
                controller, active=True, producer_time=start_s + offset
            )
            receipt_s = start_s + offset + 0.001
            assert controller._observe_arm_direct_xy_force_lease_report(
                report, received_monotonic=receipt_s
            )
            assert controller._refresh_arm_direct_xy_force_lease(
                receipt_s + 0.001
            )
    finally:
        controller.control_lock.release()

    commands = _force_lease_commands(controller)
    assert len(commands) == 6
    assert all(command["enabled"] is True for command in commands)
    assert all(command["ownership_epoch"] == 7 for command in commands)


def test_blocked_force_publish_does_not_block_fresh_report_receipt(monkeypatch):
    controller = _make_direct_xy_controller(monkeypatch)
    start_s = time.monotonic()
    _start_independent_force_lease(controller, start_s)

    publish_started = threading.Event()
    publish_release = threading.Event()

    class BlockingPublisher:
        def publish(self, _message):
            publish_started.set()
            assert publish_release.wait(timeout=1.0)

    with controller.arm_direct_xy_force_lease_lock:
        controller.arm_direct_xy_force_lease_publisher = BlockingPublisher()
        controller.arm_direct_xy_force_command_pub = (
            controller.arm_direct_xy_force_lease_publisher
        )

    refresh_result = []
    refresh_thread = threading.Thread(
        target=lambda: refresh_result.append(
            controller._refresh_arm_direct_xy_force_lease(start_s + 0.03)
        )
    )
    refresh_thread.start()
    assert publish_started.wait(timeout=1.0)

    newer = _direct_xy_reallocator_report(
        controller, active=True, producer_time=start_s + 0.04
    )
    receipt_started = time.monotonic()
    assert controller._observe_arm_direct_xy_force_lease_report(
        newer, received_monotonic=start_s + 0.041
    )
    assert time.monotonic() - receipt_started < 0.04

    publish_release.set()
    refresh_thread.join(timeout=1.0)
    assert not refresh_thread.is_alive()
    assert refresh_result == [True]


def test_force_lease_bootstrap_publish_runs_only_on_independent_safety_refresh(
    monkeypatch,
):
    controller = _make_direct_xy_controller(monkeypatch)
    start_s = time.monotonic()
    controller.arm_direct_xy_active = True
    controller.arm_direct_xy_abort_latched = False
    controller.arm_motion_active = True
    controller.last_arm_motion_monotonic = start_s

    publish_started = threading.Event()
    publish_release = threading.Event()

    class BlockingPublisher:
        def publish(self, _message):
            publish_started.set()
            assert publish_release.wait(timeout=1.0)

    controller._attach_arm_direct_xy_force_lease_publisher(BlockingPublisher())

    # This method runs in the main 20 Hz control path.  It may establish the
    # lease intent, but it must neither call the reliable publisher nor wait
    # for DDS backpressure.
    request_started = time.monotonic()
    controller._request_arm_direct_xy_force_after_mixed_setpoint(start_s)
    assert time.monotonic() - request_started < 0.04
    assert not publish_started.is_set()

    refresh_result = []
    refresh_thread = threading.Thread(
        target=lambda: refresh_result.append(
            controller._refresh_arm_direct_xy_force_lease(start_s + 0.0005)
        )
    )
    refresh_thread.start()
    assert publish_started.wait(timeout=1.0)
    assert refresh_thread.is_alive(), "the independent refresh owns the blocked publish"

    publish_release.set()
    refresh_thread.join(timeout=1.0)
    assert not refresh_thread.is_alive()
    assert refresh_result == [True]


def test_direct_xy_force_lease_fails_closed_when_main_setpoint_stops_for_250ms(
    monkeypatch,
):
    controller = _make_direct_xy_controller(monkeypatch)
    start_s = time.monotonic()
    _start_independent_force_lease(controller, start_s)
    fresh_report = _direct_xy_reallocator_report(
        controller, active=True, producer_time=start_s + 0.249
    )
    assert controller._observe_arm_direct_xy_force_lease_report(
        fresh_report, received_monotonic=start_s + 0.249
    )

    assert not controller._refresh_arm_direct_xy_force_lease(start_s + 0.250)
    commands = _force_lease_commands(controller)
    assert commands[-1]["enabled"] is False
    assert commands[-1]["lease_generation"] > commands[0]["lease_generation"]
    assert controller.arm_direct_xy_force_lease_requested is False


def test_direct_xy_force_lease_runtime_clock_is_sampled_after_lease_lock(
    monkeypatch,
):
    controller = _make_direct_xy_controller(monkeypatch)
    start_s = time.monotonic()
    _start_independent_force_lease(controller, start_s)
    controller.arm_direct_xy_force_command_pub.messages.clear()

    clock_called = threading.Event()
    result = []

    def locked_clock():
        clock_called.set()
        return start_s + 0.101

    monkeypatch.setattr(dds_module.time, "monotonic", locked_clock)
    controller.arm_direct_xy_force_lease_lock.acquire()
    try:
        refresh_thread = threading.Thread(
            target=lambda: result.append(
                controller._refresh_arm_direct_xy_force_lease()
            )
        )
        refresh_thread.start()
        time.sleep(0.01)
        assert not clock_called.is_set(), "runtime clock must be read under the lease lock"

        # Simulate the main controller publishing a newer mixed setpoint while
        # the refresh callback waits for the lock.  A clock sample taken before
        # this write would be older than the timestamp and falsely revoke.
        controller.arm_direct_xy_force_lease_mixed_setpoint_monotonic = (
            start_s + 0.100
        )
        controller.arm_direct_xy_force_lease_active_receipt_monotonic = (
            start_s + 0.100
        )
        controller.last_arm_motion_monotonic = start_s + 0.100
    finally:
        controller.arm_direct_xy_force_lease_lock.release()

    refresh_thread.join(1.0)
    assert not refresh_thread.is_alive()
    assert clock_called.is_set()
    assert result == [True]
    commands = _force_lease_commands(controller)
    assert commands[-1]["enabled"] is True
    assert controller.arm_direct_xy_force_lease_requested is True


def test_safety_ingress_does_not_presample_runtime_lease_clock():
    source = inspect.getsource(DirectXySafetyIngress._refresh_force_lease)
    assert "time.monotonic" not in source
    assert "_refresh_arm_direct_xy_force_lease()" in source


def test_safety_ingress_separates_report_and_lease_callback_groups():
    source = inspect.getsource(DirectXySafetyIngress.__init__)
    assert "self.report_callback_group = MutuallyExclusiveCallbackGroup()" in source
    assert "self.lease_callback_group = MutuallyExclusiveCallbackGroup()" in source
    # arm-motion plus either the internal reallocator report or the external
    # guardian owner level share the cache-only report group.
    assert source.count("callback_group=self.report_callback_group") == 3
    assert "callback_group=self.lease_callback_group" in source


def test_safety_executor_has_independent_report_and_lease_workers():
    source = inspect.getsource(dds_module.main)
    assert "safety_executor = MultiThreadedExecutor(num_threads=2)" in source
    assert "safety_executor = SingleThreadedExecutor()" not in source


def test_main_owner_adopts_matching_safety_lease_ack_when_report_cache_is_delayed(
    monkeypatch,
):
    controller = _make_direct_xy_controller(monkeypatch)
    start_s = time.monotonic()
    controller.arm_direct_xy_force_ack_deadline = (
        start_s + controller.ARM_DIRECT_XY_ENTRY_TIMEOUT_S
    )
    controller._request_arm_direct_xy_force_after_mixed_setpoint(start_s)
    active = _direct_xy_reallocator_report(
        controller, active=True, producer_time=start_s + 0.049
    )
    assert controller._observe_arm_direct_xy_force_lease_report(
        active, received_monotonic=start_s + 0.050
    )

    # No normal report snapshot is applied here.  The main owner must consume
    # the authoritative receipt/epoch saved by the safety ingress itself.
    assert controller.arm_direct_xy_force_enabled_ack is False
    assert controller._adopt_arm_direct_xy_force_lease_ack(start_s + 0.051)
    assert controller.arm_direct_xy_force_enabled_ack is True
    assert controller.arm_direct_xy_force_epoch_ack == 7
    assert controller.arm_direct_xy_force_ack_deadline == 0.0


@pytest.mark.parametrize(
    ("first_receipt_offset_s", "accepted"),
    [(0.149999, True), (0.150000, False)],
)
def test_safety_lease_ack_preserves_strict_150ms_owner_entry_boundary(
    monkeypatch, first_receipt_offset_s, accepted
):
    controller = _make_direct_xy_controller(monkeypatch)
    start_s = time.monotonic()
    controller.arm_direct_xy_force_ack_deadline = (
        start_s + controller.ARM_DIRECT_XY_ENTRY_TIMEOUT_S
    )
    controller._request_arm_direct_xy_force_after_mixed_setpoint(start_s)
    receipt_s = start_s + first_receipt_offset_s
    active = _direct_xy_reallocator_report(
        controller, active=True, producer_time=receipt_s - 0.001
    )
    assert controller._observe_arm_direct_xy_force_lease_report(
        active, received_monotonic=receipt_s
    )

    adopted = controller._adopt_arm_direct_xy_force_lease_ack(receipt_s)

    assert adopted is accepted
    assert controller.arm_direct_xy_force_enabled_ack is accepted


def test_direct_xy_force_lease_health_loss_revokes_and_cannot_restart_epoch(
    monkeypatch,
):
    controller = _make_direct_xy_controller(monkeypatch)
    start_s = time.monotonic()
    _start_independent_force_lease(controller, start_s)
    assert controller._refresh_arm_direct_xy_force_lease(start_s + 0.03)

    lost = _direct_xy_reallocator_report(
        controller, active=True, producer_time=start_s + 0.034
    )
    lost.update(
        position_feedback_active=False,
        direct_xy_force_enabled=False,
        direct_xy_force_command_fresh=False,
    )
    assert not controller._observe_arm_direct_xy_force_lease_report(
        lost, received_monotonic=start_s + 0.035
    )
    commands_after_loss = _force_lease_commands(controller)
    assert commands_after_loss[-1]["enabled"] is False
    revoked_generation = commands_after_loss[-1]["lease_generation"]

    delayed_positive = _direct_xy_reallocator_report(
        controller, active=True, producer_time=start_s + 0.006
    )
    assert not controller._observe_arm_direct_xy_force_lease_report(
        delayed_positive, received_monotonic=start_s + 0.007
    )
    assert not controller._refresh_arm_direct_xy_force_lease(start_s + 0.008)
    commands_final = _force_lease_commands(controller)
    assert commands_final[-1]["enabled"] is False
    assert commands_final[-1]["lease_generation"] == revoked_generation


def test_direct_xy_force_lease_serializes_exit_after_inflight_refresh(
    monkeypatch,
):
    controller = _make_direct_xy_controller(monkeypatch)
    events = []
    refresh_entered = threading.Event()
    release_refresh = threading.Event()

    class BlockingPublisher:
        def __init__(self):
            self.messages = []
            self.block_true = False

        def publish(self, message):
            command = __import__("json").loads(message.data)
            if self.block_true and command["enabled"]:
                refresh_entered.set()
                assert release_refresh.wait(1.0)
            self.messages.append(message)
            events.append(("force", command))

    publisher = BlockingPublisher()
    controller._attach_arm_direct_xy_force_lease_publisher(publisher)
    start_s = time.monotonic()
    _start_independent_force_lease(controller, start_s)
    publisher.block_true = True

    refresh_thread = threading.Thread(
        target=lambda: controller._refresh_arm_direct_xy_force_lease(
            start_s + 0.03
        )
    )
    refresh_thread.start()
    assert refresh_entered.wait(1.0)

    def finite_then_disable():
        events.append(("finite", None))
        controller._publish_arm_direct_xy_force_command(False)

    exit_thread = threading.Thread(target=finite_then_disable)
    exit_thread.start()
    time.sleep(0.01)
    assert exit_thread.is_alive(), "disable must wait behind the in-flight refresh"
    release_refresh.set()
    refresh_thread.join(1.0)
    exit_thread.join(1.0)
    assert not refresh_thread.is_alive()
    assert not exit_thread.is_alive()

    finite_index = next(index for index, event in enumerate(events) if event[0] == "finite")
    false_index = next(
        index
        for index, event in enumerate(events)
        if event[0] == "force" and event[1]["enabled"] is False
    )
    assert finite_index < false_index
    assert not any(
        event[0] == "force" and event[1]["enabled"] is True
        for event in events[false_index + 1 :]
    )
    assert events[-1][0] == "force"
    assert events[-1][1]["enabled"] is False


def test_reallocator_callback_never_waits_for_control_lock_and_latest_report_wins(
    monkeypatch,
):
    controller = _make_direct_xy_controller(monkeypatch)
    applied = []
    controller._apply_arm_direct_xy_reallocator_snapshot = (
        lambda snapshot: applied.append(snapshot[0])
    )

    newest = time.monotonic()
    older = newest - 0.01
    newest_report = _direct_xy_reallocator_report(
        controller, active=False, producer_time=newest
    )
    older_report = _direct_xy_reallocator_report(
        controller, active=False, producer_time=older
    )
    newest_message = SimpleNamespace(
        data=__import__("json").dumps(newest_report)
    )
    older_message = SimpleNamespace(
        data=__import__("json").dumps(older_report)
    )

    with controller.control_lock:
        older_worker = threading.Thread(
            target=lambda: controller._locked_arm_direct_xy_reallocator_state_cb(
                older_message
            )
        )
        older_worker.start()
        older_worker.join(timeout=0.1)
        assert not older_worker.is_alive()

        newest_worker = threading.Thread(
            target=lambda: controller._locked_arm_direct_xy_reallocator_state_cb(
                newest_message
            )
        )
        newest_worker.start()
        newest_worker.join(timeout=0.1)
        assert not newest_worker.is_alive()

        # A delayed duplicate carrying an older producer timestamp cannot
        # replace the already-cached newest state either.
        older_worker = threading.Thread(
            target=lambda: controller._locked_arm_direct_xy_reallocator_state_cb(
                older_message
            )
        )
        older_worker.start()
        older_worker.join(timeout=0.1)
        assert not older_worker.is_alive()
        assert applied == []

    with controller.control_lock:
        controller._consume_latest_arm_direct_xy_report_snapshot()
    assert applied == [newest]


def test_arm_motion_callback_never_waits_for_control_lock(monkeypatch):
    controller = _make_direct_xy_controller(monkeypatch)
    controller.arm_motion_active = False
    controller.arm_direct_xy_ownership_epoch = 3

    with controller.control_lock:
        worker = threading.Thread(
            target=lambda: controller._locked_arm_motion_active_cb(
                SimpleNamespace(data=True)
            )
        )
        worker.start()
        worker.join(timeout=0.1)
        assert not worker.is_alive()
        assert controller.arm_motion_active is False
        assert controller.arm_direct_xy_ownership_epoch == 3

    with controller.control_lock:
        controller._consume_latest_arm_motion_snapshot()
    assert controller.arm_motion_active is True
    assert controller.arm_direct_xy_ownership_epoch == 4


def test_cached_motion_epoch_is_committed_before_prepared_reallocator(monkeypatch):
    controller = _make_direct_xy_controller(monkeypatch)
    controller.arm_motion_active = False
    controller.arm_direct_xy_ownership_epoch = 4
    report = _direct_xy_reallocator_report(
        controller, active=False, producer_time=time.monotonic()
    )
    message = SimpleNamespace(data=__import__("json").dumps(report))

    with controller.control_lock:
        workers = [
            threading.Thread(
                target=lambda: controller._locked_arm_motion_active_cb(
                    SimpleNamespace(data=True)
                )
            ),
            threading.Thread(
                target=lambda: (
                    controller._locked_arm_direct_xy_reallocator_state_cb(message)
                )
            ),
        ]
        for worker in workers:
            worker.start()
            worker.join(timeout=0.1)
            assert not worker.is_alive()
        assert controller.arm_direct_xy_active is False

    with controller.control_lock:
        controller._consume_latest_arm_motion_snapshot()
        controller._consume_latest_arm_direct_xy_report_snapshot()

    assert controller.arm_direct_xy_ownership_epoch == 5
    assert controller.arm_direct_xy_active is True
    assert controller._refresh_arm_direct_xy_force_lease()
    command = __import__("json").loads(
        controller.arm_direct_xy_force_command_pub.messages[-1].data
    )
    assert command["enabled"] is True
    assert command["ownership_epoch"] == 5


def test_reallocator_snapshot_is_aged_at_receipt_not_after_control_lock_wait(
    monkeypatch,
):
    controller = _make_direct_xy_controller(monkeypatch)
    controller.arm_motion_active = False
    report = _direct_xy_reallocator_report(
        controller, active=False, producer_time=time.monotonic()
    )
    message = SimpleNamespace(data=__import__("json").dumps(report))

    with controller.control_lock:
        worker = threading.Thread(
            target=lambda: controller._locked_arm_direct_xy_reallocator_state_cb(
                message
            )
        )
        worker.start()
        worker.join(timeout=0.1)
        assert not worker.is_alive()
        # Reproduce the former failure: applying after a delay longer than the
        # watchdog must not turn a sample that arrived fresh into an unhealthy
        # report merely because the control lock was occupied.
        time.sleep(controller.ARM_DIRECT_XY_WATCHDOG_S + 0.01)

    with controller.control_lock:
        controller._consume_latest_arm_direct_xy_report_snapshot()
    assert controller.arm_direct_xy_reallocator_healthy is True


def test_healthy_receipt_heartbeat_remains_fresh_while_control_lock_is_busy(
    monkeypatch,
):
    controller = _make_direct_xy_controller(monkeypatch)
    controller.arm_direct_xy_reallocator_healthy = True

    with controller.control_lock:
        for _ in range(4):
            report = _direct_xy_reallocator_report(
                controller, active=False, producer_time=time.monotonic()
            )
            worker = threading.Thread(
                target=lambda current=report: (
                    controller._locked_arm_direct_xy_reallocator_state_cb(
                        SimpleNamespace(
                            data=__import__("json").dumps(current)
                        )
                    )
                )
            )
            worker.start()
            worker.join(timeout=0.1)
            assert not worker.is_alive()
            time.sleep(controller.ARM_DIRECT_XY_WATCHDOG_S / 2.0)

        assert controller._arm_direct_xy_health_fresh(time.monotonic()) is True


def test_pending_real_fault_invalidates_receipt_heartbeat_before_commit(monkeypatch):
    controller = _make_direct_xy_controller(monkeypatch)
    controller.arm_direct_xy_reallocator_healthy = True
    fault = _direct_xy_reallocator_report(
        controller, active=False, producer_time=time.monotonic()
    )
    fault["truth_fresh"] = False

    with controller.control_lock:
        worker = threading.Thread(
            target=lambda: controller._locked_arm_direct_xy_reallocator_state_cb(
                SimpleNamespace(data=__import__("json").dumps(fault))
            )
        )
        worker.start()
        worker.join(timeout=0.1)
        assert not worker.is_alive()
        assert controller._arm_direct_xy_health_fresh(time.monotonic()) is False


def test_real_reallocator_fault_is_not_hidden_by_newer_healthy_pending_level(
    monkeypatch,
):
    controller = _make_direct_xy_controller(monkeypatch)
    _activate_direct_xy_owner(controller)
    fault = _direct_xy_reallocator_report(
        controller, active=True, producer_time=time.monotonic()
    )
    fault["source_fresh"] = False
    healthy = _direct_xy_reallocator_report(
        controller, active=True, producer_time=time.monotonic()
    )
    fault_message = SimpleNamespace(data=__import__("json").dumps(fault))
    healthy_message = SimpleNamespace(data=__import__("json").dumps(healthy))

    with controller.control_lock:
        for message in (fault_message, healthy_message):
            worker = threading.Thread(
                target=lambda current=message: (
                    controller._locked_arm_direct_xy_reallocator_state_cb(current)
                )
            )
            worker.start()
            worker.join(timeout=0.1)
            assert not worker.is_alive()

    with controller.control_lock:
        controller._consume_latest_arm_direct_xy_report_snapshot()

    assert controller.arm_direct_xy_abort_latched is True
    assert controller.arm_direct_xy_reallocator_healthy is False
    assert controller.arm_direct_xy_position_feedback_active is False
    assert controller.arm_direct_xy_force_enabled_ack is False
    state = __import__("json").loads(
        controller.arm_direct_xy_state_pub.messages[-1].data
    )
    assert state["state"] == "aborting"
    assert state["watchdog_reason"] == "reallocator_unhealthy"


def test_prepared_snapshot_received_on_time_cannot_commit_handoff_after_deadline(
    monkeypatch,
):
    controller = _make_direct_xy_controller(monkeypatch)
    controller.arm_direct_xy_active = False
    controller.arm_direct_xy_force_commanded = False
    controller.arm_direct_xy_force_enabled_ack = False
    controller.arm_direct_xy_entry_deadline = time.monotonic() + 0.02
    report = _direct_xy_reallocator_report(
        controller, active=False, producer_time=time.monotonic()
    )
    message = SimpleNamespace(data=__import__("json").dumps(report))

    with controller.control_lock:
        worker = threading.Thread(
            target=lambda: controller._locked_arm_direct_xy_reallocator_state_cb(
                message
            )
        )
        worker.start()
        worker.join(timeout=0.1)
        assert not worker.is_alive()
        time.sleep(0.03)

    controller.test_publish_events.clear()
    with controller.control_lock:
        controller._consume_latest_arm_direct_xy_report_snapshot()

    assert controller.arm_direct_xy_abort_latched is True
    assert not any(
        name == "force_command"
        and __import__("json").loads(published.data)["enabled"] is True
        for name, published in controller.test_publish_events
    )
    state = __import__("json").loads(
        controller.arm_direct_xy_state_pub.messages[-1].data
    )
    assert state["watchdog_reason"] == "entry_gate_timeout"


def test_direct_xy_reallocator_health_rejects_stale_limited_or_saturated():
    healthy = {
        "event": "allocated",
        "source_fresh": True,
        "flight_allowed": True,
        "headroom_ok": True,
        "motion_active": True,
        "position_feedback_enabled": True,
        "position_feedback_ready": True,
        "position_feedback_active": True,
        "position_target_latched": True,
        "truth_fresh": True,
        "feasibility_scale": 1.0,
        "residual_norm": 1.0e-8,
        "saturated": 0,
    }
    assert DdsWasdControl._direct_xy_reallocator_report_is_healthy(healthy)
    for patch in (
        {"source_fresh": False},
        {"position_feedback_enabled": False},
        {"position_feedback_ready": False},
        {"truth_fresh": False},
        {"allocation_limited": True},
        {"feasibility_scale": 0.999},
        {"residual_norm": 0.01},
        {"saturated": 1},
        {"event": "allocation_failure"},
    ):
        report = dict(healthy)
        report.update(patch)
        assert not DdsWasdControl._direct_xy_reallocator_report_is_healthy(report)


def test_direct_xy_zero_overlay_can_pre_authorize_but_not_grant_ownership(monkeypatch):
    controller = _make_direct_xy_controller(monkeypatch)
    controller.arm_motion_active = False
    controller.last_arm_motion_monotonic = time.monotonic()
    report = {
        "event": "zero_overlay",
        "source_fresh": True,
        "flight_allowed": True,
        "headroom_ok": True,
        "motion_active": True,
        "position_feedback_enabled": True,
        "position_feedback_ready": True,
        "position_feedback_active": False,
        "position_target_latched": False,
        "truth_fresh": True,
        "allocation_limited": False,
        "feasibility_scale": 1.0,
        "residual_norm": 0.0,
        "saturated": 0,
    }
    controller._arm_direct_xy_reallocator_state_cb(
        SimpleNamespace(data=__import__("json").dumps(report))
    )
    assert controller.arm_direct_xy_reallocator_healthy is True
    assert controller.arm_direct_xy_position_feedback_active is False
    assert controller._arm_direct_xy_preentry_ready(time.monotonic()) is True
    controller.publish_truth_hold()
    assert controller.arm_direct_xy_ready_pub.messages[-1].data is True
    state = __import__("json").loads(controller.arm_direct_xy_state_pub.messages[-1].data)
    assert state["state"] == "px4_xy"
    finite_px4 = controller.setpoint_pub.messages[-1]
    assert np.all(np.isfinite(finite_px4.velocity))

    controller._arm_motion_active_cb(SimpleNamespace(data=True))
    active_report = dict(report)
    active_report.update({
        "motion_active": True,
        "position_feedback_prepared": True,
        "position_target_latched": True,
    })
    controller._arm_direct_xy_reallocator_state_cb(
        SimpleNamespace(data=__import__("json").dumps(active_report))
    )
    controller.publish_truth_hold()
    assert controller.arm_direct_xy_ready_pub.messages[-1].data is True
    handoff = controller.setpoint_pub.messages[-1]
    assert np.all(np.isnan(handoff.velocity[:2]))
    handoff_state = __import__("json").loads(
        controller.arm_direct_xy_state_pub.messages[-1].data
    )
    assert handoff_state["state"] == "handoff"

    enabled_report = dict(active_report)
    enabled_report.update({
        "event": "allocated",
        "position_feedback_active": True,
        "direct_xy_force_enabled": True,
        "direct_xy_force_epoch": controller.arm_direct_xy_ownership_epoch,
    })
    controller._arm_direct_xy_reallocator_state_cb(
        SimpleNamespace(data=__import__("json").dumps(enabled_report))
    )
    controller.publish_truth_hold()
    direct_state = __import__("json").loads(
        controller.arm_direct_xy_state_pub.messages[-1].data
    )
    assert direct_state["state"] == "direct_xy"


def test_premotion_inhibit_reproduces_unstable_entry_gate_not_receipt_ack(
    monkeypatch,
):
    """The receipt ACK is stage two; it cannot deadlock pre-authorisation.

    This reproduces the latest flight trace: allocator health is fresh and its
    position loop is ready, but the vehicle is still moving faster than the
    entry gate.  DDS must keep ``ready=False``/``inhibit=True`` without waiting
    for an active-owner ACK.  Once the *same* pre-motion state becomes stable,
    the grant must clear even though no force ACK has ever existed.
    """
    controller = _make_direct_xy_controller(monkeypatch)
    now = time.monotonic()
    controller.arm_motion_active = False
    controller.last_arm_motion_monotonic = now
    controller.arm_direct_xy_active = False
    controller.arm_direct_xy_preauthorized = False
    controller.arm_direct_xy_force_commanded = False
    controller.arm_direct_xy_force_enabled_ack = False
    controller.arm_direct_xy_force_ever_ack = False
    controller.arm_direct_xy_force_epoch_ack = -1
    controller.arm_direct_xy_reallocator_healthy = True
    controller.arm_direct_xy_position_feedback_ready = True
    controller.arm_direct_xy_health_since = now - 1.0
    controller.last_arm_direct_xy_healthy_receipt_monotonic = now
    controller.truth_hold_target_enu = controller.gazebo_truth_enu.copy()
    controller.gazebo_truth_rpy = np.zeros(3)
    controller.gazebo_truth_velocity_enu = np.asarray(
        [controller.ARM_DIRECT_XY_ENTRY_XY_SPEED_M_S + 0.01, 0.0, 0.0]
    )
    controller.gazebo_truth_velocity_filtered_enu = (
        controller.gazebo_truth_velocity_enu.copy()
    )

    controller.publish_truth_hold()

    assert controller.arm_direct_xy_ready_pub.messages[-1].data is False
    assert controller.arm_motion_inhibit_pub.messages[-1].data is True
    assert controller.arm_direct_xy_force_enabled_ack is False
    assert not any(
        __import__("json").loads(message.data)["enabled"] is True
        for message in controller.arm_direct_xy_force_command_pub.messages
    )

    # Satisfy the existing entry thresholds and their hold time without
    # fabricating a direct-owner ACK.  Pre-authorisation must now become true.
    now = time.monotonic()
    controller.gazebo_truth_velocity_enu = np.zeros(3)
    controller.gazebo_truth_velocity_filtered_enu = np.zeros(3)
    controller.arm_direct_xy_stable_since = (
        now - controller.ARM_DIRECT_XY_STABLE_HOLD_S - 0.01
    )
    controller.last_gazebo_truth_monotonic = now
    controller.last_status_monotonic = now
    controller.last_arm_direct_xy_healthy_receipt_monotonic = now
    controller.publish_truth_hold()

    assert controller.arm_direct_xy_ready_pub.messages[-1].data is True
    assert controller.arm_motion_inhibit_pub.messages[-1].data is False
    assert controller.arm_direct_xy_force_enabled_ack is False
    assert controller.arm_direct_xy_force_epoch_ack == -1


def test_direct_xy_epoch_increments_once_per_motion_rising_edge(monkeypatch):
    controller = _make_direct_xy_controller(monkeypatch)
    controller.arm_motion_active = False
    controller.arm_direct_xy_ownership_epoch = 2
    controller._arm_motion_active_cb(SimpleNamespace(data=True))
    assert controller.arm_direct_xy_ownership_epoch == 3
    controller._arm_motion_active_cb(SimpleNamespace(data=True))
    assert controller.arm_direct_xy_ownership_epoch == 3
    controller._arm_motion_active_cb(SimpleNamespace(data=False))
    controller._arm_motion_active_cb(SimpleNamespace(data=True))
    assert controller.arm_direct_xy_ownership_epoch == 4


@pytest.mark.parametrize("was_active", [False, True])
def test_direct_xy_idle_edge_never_publishes_false_authorization(
    monkeypatch, was_active
):
    """Idle clears an action epoch but cannot authorize the next one."""
    controller = _make_direct_xy_controller(monkeypatch)
    controller.arm_motion_active = was_active
    # Reproduce the dangerous stale state from the previous implementation:
    # a prior pre-authorisation must not leak through either a falling edge or
    # a repeated idle heartbeat while the new stability hold is unevaluated.
    controller.arm_direct_xy_preauthorized = True
    controller.arm_direct_xy_ready_pub.messages.clear()
    controller.arm_motion_inhibit_pub.messages.clear()

    controller._arm_motion_active_cb(SimpleNamespace(data=False))

    assert controller.arm_direct_xy_preauthorized is False
    assert controller.arm_direct_xy_ready_pub.messages
    assert controller.arm_motion_inhibit_pub.messages
    assert all(message.data is False for message in controller.arm_direct_xy_ready_pub.messages)
    assert all(message.data is True for message in controller.arm_motion_inhibit_pub.messages)


def test_direct_xy_entry_publishes_mixed_axis_before_force_enable(monkeypatch):
    controller = _make_direct_xy_controller(monkeypatch)
    before_publish = time.monotonic()
    _deliver_direct_xy_report(controller)

    mode = controller.mode_pub.messages[-1]
    setpoint = controller.setpoint_pub.messages[-1]
    assert mode.velocity is True
    assert mode.position is False
    assert np.all(np.isnan(setpoint.position))
    assert np.all(np.isnan(setpoint.velocity[:2]))
    assert np.isfinite(setpoint.velocity[2])
    np.testing.assert_allclose(setpoint.acceleration[:2], [0.0, 0.0])
    assert np.isnan(setpoint.acceleration[2])
    assert setpoint.yaw == pytest.approx(controller.target.yaw)
    assert controller.arm_direct_xy_active is True
    assert controller.arm_motion_inhibit_pub.messages[-1].data is False
    state = __import__("json").loads(controller.arm_direct_xy_state_pub.messages[-1].data)
    assert state["schema"] == "my_drone.arm-direct-xy-state.v1"
    assert state["ownership_epoch"] == 7
    assert state["state"] == "handoff"
    assert state["watchdog_reason"] == ""
    assert state["reallocator_fresh"] is True
    assert state["position_feedback_active"] is False
    # The independent safety timer, represented explicitly here, emits the
    # bootstrap after the main callback has returned from mixed-setpoint entry.
    assert controller._refresh_arm_direct_xy_force_lease()
    names = [name for name, _message in controller.test_publish_events]
    assert names.index("setpoint") < names.index("force_command")
    force = __import__("json").loads(
        controller.arm_direct_xy_force_command_pub.messages[-1].data
    )
    assert force["enabled"] is True
    assert force["ownership_epoch"] == 7
    assert controller.arm_direct_xy_force_ack_deadline >= (
        before_publish + controller.ARM_DIRECT_XY_ENTRY_TIMEOUT_S
    )

    # Heartbeat refreshes must not slide the acknowledgement deadline.
    first_ack_deadline = controller.arm_direct_xy_force_ack_deadline
    controller.publish_truth_hold()
    assert controller.arm_direct_xy_force_ack_deadline == first_ack_deadline


def test_direct_xy_entry_budget_matches_owner_contract_without_relaxing_runtime_gates(
    monkeypatch,
):
    controller = _make_direct_xy_controller(monkeypatch)

    assert controller.ARM_DIRECT_XY_ENTRY_TIMEOUT_S == pytest.approx(0.15)
    assert controller.ARM_DIRECT_XY_WATCHDOG_S == pytest.approx(0.04)
    assert controller.ARM_DIRECT_XY_MAIN_SETPOINT_LEASE_S == pytest.approx(0.25)


@pytest.mark.parametrize(
    ("receipt_offset_s", "accepted"),
    [(0.149, True), (0.151, False)],
)
def test_direct_xy_force_ack_uses_strict_150ms_entry_boundary(
    monkeypatch, receipt_offset_s, accepted
):
    controller = _make_direct_xy_controller(monkeypatch)
    start = time.monotonic()
    controller.arm_direct_xy_active = True
    controller.arm_direct_xy_force_commanded = True
    controller.arm_direct_xy_force_ack_deadline = (
        start + controller.ARM_DIRECT_XY_ENTRY_TIMEOUT_S
    )
    receipt = start + receipt_offset_s
    producer = receipt - 0.001
    report = _direct_xy_reallocator_report(
        controller, active=True, producer_time=producer
    )

    controller._apply_arm_direct_xy_reallocator_snapshot(
        (producer, receipt, report)
    )

    if accepted:
        assert controller.arm_direct_xy_abort_latched is False
        assert controller.arm_direct_xy_force_enabled_ack is True
        assert controller.arm_direct_xy_force_ack_deadline == 0.0
    else:
        assert controller.arm_direct_xy_abort_latched is True
        assert controller.arm_direct_xy_force_enabled_ack is False
        state = __import__("json").loads(
            controller.arm_direct_xy_state_pub.messages[-1].data
        )
        assert state["watchdog_reason"] == "direct_force_ack_timeout"


def test_direct_xy_late_prepare_fails_stage_one_without_force_enable(monkeypatch):
    controller = _make_direct_xy_controller(monkeypatch)
    now = time.monotonic()
    controller.arm_direct_xy_entry_deadline = now - 0.001
    controller.arm_direct_xy_prepared_monotonic = now
    controller.test_publish_events.clear()

    controller.publish_truth_hold()

    assert controller.arm_direct_xy_abort_latched is True
    assert not any(
        name == "force_command"
        and __import__("json").loads(message.data)["enabled"] is True
        for name, message in controller.test_publish_events
    )
    state = __import__("json").loads(
        controller.arm_direct_xy_state_pub.messages[-1].data
    )
    assert state["watchdog_reason"] == "entry_gate_timeout"


def test_direct_xy_on_time_prepare_cannot_handoff_on_late_control_tick(monkeypatch):
    controller = _make_direct_xy_controller(monkeypatch)
    now = time.monotonic()
    controller.arm_direct_xy_entry_deadline = now - 0.001
    controller.arm_direct_xy_prepared_monotonic = now - 0.01
    controller.test_publish_events.clear()

    controller.publish_truth_hold()

    assert controller.arm_direct_xy_abort_latched is True
    assert not any(
        name == "force_command"
        and __import__("json").loads(message.data)["enabled"] is True
        for name, message in controller.test_publish_events
    )
    assert controller.arm_direct_xy_entry_deadline == 0.0
    assert controller.arm_direct_xy_prepared_monotonic == 0.0
    assert controller.arm_direct_xy_force_ack_deadline == 0.0


def test_direct_xy_ack_transitions_handoff_to_owner(monkeypatch):
    controller = _make_direct_xy_controller(monkeypatch)
    _deliver_direct_xy_report(controller)
    _deliver_direct_xy_report(controller, active=True)
    state = __import__("json").loads(
        controller.arm_direct_xy_state_pub.messages[-1].data
    )
    assert state["state"] == "direct_xy"
    assert controller.arm_direct_xy_force_ack_deadline == 0.0


def test_direct_xy_exit_publishes_finite_px4_before_force_disable(monkeypatch):
    controller = _make_direct_xy_controller(monkeypatch)
    _deliver_direct_xy_report(controller)
    _deliver_direct_xy_report(controller, active=True)

    controller.test_publish_events.clear()
    controller._arm_motion_active_cb(SimpleNamespace(data=False))

    names = [name for name, _message in controller.test_publish_events]
    finite_index = next(
        index
        for index, (name, message) in enumerate(controller.test_publish_events)
        if name == "setpoint" and np.all(np.isfinite(message.velocity[:2]))
    )
    disable_index = next(
        index
        for index, (name, message) in enumerate(controller.test_publish_events)
        if name == "force_command"
        and __import__("json").loads(message.data)["enabled"] is False
    )
    assert finite_index < disable_index
    assert names[-1] == "state"
    state = __import__("json").loads(
        controller.arm_direct_xy_state_pub.messages[-1].data
    )
    assert state["state"] == "px4_xy"


def test_direct_xy_stale_prepared_snapshot_cannot_restart_handoff(monkeypatch):
    controller = _make_direct_xy_controller(monkeypatch)
    newest_time = time.monotonic()
    idle = _direct_xy_reallocator_report(controller, producer_time=newest_time)
    idle.update(
        motion_active=False,
        position_feedback_prepared=False,
        position_target_latched=False,
    )
    controller._arm_direct_xy_reallocator_state_cb(
        SimpleNamespace(data=__import__("json").dumps(idle))
    )
    controller.test_publish_events.clear()

    _deliver_direct_xy_report(
        controller,
        producer_time=newest_time - 0.01,
    )

    assert controller.arm_direct_xy_active is False
    assert not any(
        name == "force_command"
        and __import__("json").loads(message.data)["enabled"] is True
        for name, message in controller.test_publish_events
    )


def test_direct_xy_reallocator_idle_immediately_restores_before_disable(monkeypatch):
    controller = _make_direct_xy_controller(monkeypatch)
    _deliver_direct_xy_report(controller)
    _deliver_direct_xy_report(controller, active=True)
    controller.test_publish_events.clear()
    idle = _direct_xy_reallocator_report(controller)
    idle.update(
        motion_active=False,
        position_feedback_prepared=False,
        position_feedback_active=False,
        position_target_latched=False,
        direct_xy_force_enabled=False,
    )

    controller._arm_direct_xy_reallocator_state_cb(
        SimpleNamespace(data=__import__("json").dumps(idle))
    )

    finite_index = next(
        index
        for index, (name, message) in enumerate(controller.test_publish_events)
        if name == "setpoint" and np.all(np.isfinite(message.velocity[:2]))
    )
    disable_index = next(
        index
        for index, (name, message) in enumerate(controller.test_publish_events)
        if name == "force_command"
        and __import__("json").loads(message.data)["enabled"] is False
    )
    assert finite_index < disable_index
    assert controller.arm_direct_xy_active is False


def test_direct_xy_handoff_ack_timeout_restores_finite_before_disable(monkeypatch):
    controller = _make_direct_xy_controller(monkeypatch)
    _deliver_direct_xy_report(controller)
    controller.arm_direct_xy_force_ack_deadline = time.monotonic() - 0.001
    controller.test_publish_events.clear()
    _deliver_direct_xy_report(controller, active=True)

    assert controller.arm_direct_xy_abort_latched is True
    finite_index = next(
        index
        for index, (name, message) in enumerate(controller.test_publish_events)
        if name == "setpoint" and np.all(np.isfinite(message.velocity[:2]))
    )
    disable_index = next(
        index
        for index, (name, message) in enumerate(controller.test_publish_events)
        if name == "force_command"
        and __import__("json").loads(message.data)["enabled"] is False
    )
    assert finite_index < disable_index
    state = __import__("json").loads(
        controller.arm_direct_xy_state_pub.messages[-1].data
    )
    assert state["state"] == "aborting"
    assert state["watchdog_reason"] == "direct_force_ack_timeout"


def test_direct_xy_late_ack_callback_aborts_instead_of_granting_owner(monkeypatch):
    controller = _make_direct_xy_controller(monkeypatch)
    controller.publish_truth_hold()
    controller.arm_direct_xy_force_ack_deadline = time.monotonic() - 0.001
    report = {
        "event": "allocated",
        "source_fresh": True,
        "flight_allowed": True,
        "headroom_ok": True,
        "motion_active": True,
        "position_feedback_enabled": True,
        "position_feedback_ready": True,
        "position_feedback_active": True,
        "position_feedback_prepared": True,
        "position_target_latched": True,
        "direct_xy_force_enabled": True,
        "direct_xy_force_epoch": controller.arm_direct_xy_ownership_epoch,
        "truth_fresh": True,
        "allocation_limited": False,
        "feasibility_scale": 1.0,
        "residual_norm": 0.0,
        "saturated": 0,
    }

    controller._arm_direct_xy_reallocator_state_cb(
        SimpleNamespace(data=__import__("json").dumps(report))
    )

    assert controller.arm_direct_xy_abort_latched is True
    assert controller.arm_direct_xy_force_enabled_ack is False
    assert controller.arm_direct_xy_entry_deadline == 0.0
    assert controller.arm_direct_xy_prepared_monotonic == 0.0
    assert controller.arm_direct_xy_force_ack_deadline == 0.0
    assert controller.arm_direct_xy_force_ack_monotonic == 0.0
    state = __import__("json").loads(
        controller.arm_direct_xy_state_pub.messages[-1].data
    )
    assert state["state"] == "aborting"
    assert state["watchdog_reason"] == "direct_force_ack_timeout"


def test_direct_xy_watchdog_restores_finite_truth_hold_and_aborts_arm(monkeypatch):
    controller = _make_direct_xy_controller(monkeypatch)
    _deliver_direct_xy_report(controller)
    assert controller.arm_direct_xy_active is True

    stale = time.monotonic() - controller.ARM_DIRECT_XY_WATCHDOG_S - 0.001
    controller.last_arm_direct_xy_state_monotonic = stale
    controller.last_arm_direct_xy_healthy_receipt_monotonic = stale
    controller.publish_truth_hold()

    fallback = controller.setpoint_pub.messages[-1]
    assert np.all(np.isfinite(fallback.velocity))
    assert controller.arm_direct_xy_active is False
    assert controller.arm_direct_xy_abort_latched is True
    assert controller.arm_motion_inhibit_pub.messages[-1].data is True
    stop = controller.rl_arm_pub.messages[-1]
    np.testing.assert_allclose(stop.points[0].positions, controller.rl_joint_positions)
    np.testing.assert_allclose(stop.points[0].velocities, np.zeros(6))
    state = __import__("json").loads(controller.arm_direct_xy_state_pub.messages[-1].data)
    assert state["state"] == "aborting"
    assert state["watchdog_reason"] == "runtime_watchdog"
    assert state["watchdog_detail"] == "reallocator_heartbeat_stale"
    # 20 Hz publication plus the bounded 40 ms health timeout gives a
    # deterministic worst-case finite-XY recovery within 100 ms.
    assert controller.ARM_DIRECT_XY_WATCHDOG_S + 1.0 / controller.RATE_HZ <= 0.100001


def test_direct_xy_truth_stale_falls_back_to_finite_position_and_aborts(monkeypatch):
    controller = _make_direct_xy_controller(monkeypatch)
    controller.publish_truth_hold()
    controller.last_gazebo_truth_monotonic = (
        time.monotonic() - controller.TRUTH_HOLD_TIMEOUT_S - 0.01
    )
    controller.publish_hold()

    fallback = controller.setpoint_pub.messages[-1]
    assert np.all(np.isfinite(fallback.position))
    assert np.all(np.isnan(fallback.velocity))
    assert controller.arm_direct_xy_abort_latched is True
    state = __import__("json").loads(controller.arm_direct_xy_state_pub.messages[-1].data)
    assert state["state"] == "aborting"
    assert state["watchdog_reason"] == "truth_hold_stale"


@pytest.mark.parametrize("exit_action", ["land", "exit_offboard", "emergency_disarm"])
def test_direct_xy_mode_exit_restores_finite_before_force_disable(
    monkeypatch, exit_action
):
    controller = _make_direct_xy_controller(monkeypatch)
    _activate_direct_xy_owner(controller)
    controller.test_publish_events.clear()

    if exit_action == "land":
        controller.land(staged=False)
    elif exit_action == "exit_offboard":
        controller.exit_offboard()
    else:
        controller.emergency_disarm()

    finite_index = next(
        index
        for index, (name, message) in enumerate(controller.test_publish_events)
        if name == "setpoint" and (
            np.all(np.isfinite(message.velocity[:2]))
            or np.all(np.isfinite(message.position[:2]))
        )
    )
    disable_index = next(
        index
        for index, (name, message) in enumerate(controller.test_publish_events)
        if name == "force_command"
        and __import__("json").loads(message.data)["enabled"] is False
    )
    command_index = next(
        index
        for index, (name, _message) in enumerate(controller.test_publish_events)
        if name == "command"
    )
    assert finite_index < disable_index < command_index
    assert controller.arm_direct_xy_abort_latched is True
    assert controller.arm_direct_xy_force_commanded is False


def _prepare_tick_for_mode_exit(controller):
    now = time.monotonic()
    controller.last_control_tick_monotonic = now - 0.05
    controller.last_state_report = now
    controller.last_status_monotonic = now
    controller.last_local_monotonic = now
    controller.local.xy_valid = True
    controller.local.z_valid = True
    controller.status_stale_reported = False
    controller.arm_only_started = 0.0
    controller.pending_takeoff = False
    controller.prestream_started = 0.0
    controller.TOUCHDOWN_DISARM_ENABLED = False
    controller.publish_rl_observation = lambda: None
    return now


def test_direct_xy_local_timeout_revokes_force_before_stopping_offboard(monkeypatch):
    controller = _make_direct_xy_controller(monkeypatch)
    _activate_direct_xy_owner(controller)
    now = _prepare_tick_for_mode_exit(controller)
    controller.last_local_monotonic = now - controller.LOCAL_POSITION_TIMEOUT_S - 0.1
    controller.test_publish_events.clear()

    controller._tick()

    finite_index = next(
        index
        for index, (name, message) in enumerate(controller.test_publish_events)
        if name == "setpoint" and np.all(np.isfinite(message.velocity[:2]))
    )
    disable_index = next(
        index
        for index, (name, message) in enumerate(controller.test_publish_events)
        if name == "force_command"
        and __import__("json").loads(message.data)["enabled"] is False
    )
    assert finite_index < disable_index
    assert controller.offboard_requested is False


def test_direct_xy_native_land_handoff_revokes_before_nav_land(monkeypatch):
    controller = _make_direct_xy_controller(monkeypatch)
    _activate_direct_xy_owner(controller)
    now = _prepare_tick_for_mode_exit(controller)
    controller.control_state = FlightControlState.LANDING
    controller.landing_requested = True
    controller.offboard_landing_active = True
    controller.offboard_landing_started = now - 10.0
    controller.OFFBOARD_LAND_MIN_DURATION_S = 0.0
    controller.takeoff_ground_down = controller.local.z
    controller.test_publish_events.clear()

    controller._tick()

    finite_index = next(
        index
        for index, (name, message) in enumerate(controller.test_publish_events)
        if name == "setpoint" and (
            np.all(np.isfinite(message.velocity[:2]))
            or np.all(np.isfinite(message.position[:2]))
        )
    )
    disable_index = next(
        index
        for index, (name, message) in enumerate(controller.test_publish_events)
        if name == "force_command"
        and __import__("json").loads(message.data)["enabled"] is False
    )
    command_index = next(
        index
        for index, (name, _message) in enumerate(controller.test_publish_events)
        if name == "command"
    )
    assert finite_index < disable_index < command_index
    assert controller.offboard_requested is False
    assert controller.offboard_landing_active is False


def test_direct_xy_unhealthy_report_latches_across_following_healthy_report(monkeypatch):
    controller = _make_direct_xy_controller(monkeypatch)
    _activate_direct_xy_owner(controller)
    report = {
        "event": "allocated",
        "source_fresh": True,
        "flight_allowed": True,
        "headroom_ok": True,
        "motion_active": True,
        "position_feedback_enabled": True,
        "position_feedback_ready": True,
        "position_feedback_active": True,
        "position_feedback_prepared": True,
        "position_target_latched": True,
        "direct_xy_force_enabled": True,
        "direct_xy_force_epoch": controller.arm_direct_xy_ownership_epoch,
        "truth_fresh": True,
        "allocation_limited": False,
        "feasibility_scale": 1.0,
        "residual_norm": 0.0,
        "saturated": 0,
    }
    unhealthy = dict(report, source_fresh=False)

    controller._arm_direct_xy_reallocator_state_cb(
        SimpleNamespace(data=__import__("json").dumps(unhealthy))
    )
    controller._arm_direct_xy_reallocator_state_cb(
        SimpleNamespace(data=__import__("json").dumps(report))
    )

    assert controller.arm_direct_xy_abort_latched is True
    assert controller.arm_direct_xy_reallocator_healthy is False
    assert controller.arm_direct_xy_position_feedback_active is False
    assert controller.arm_direct_xy_force_enabled_ack is False
    assert controller.arm_direct_xy_ready_pub.messages[-1].data is False
    assert controller.arm_motion_inhibit_pub.messages[-1].data is True
