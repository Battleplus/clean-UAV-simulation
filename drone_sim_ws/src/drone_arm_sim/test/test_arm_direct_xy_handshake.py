import time
from types import SimpleNamespace
import json

import numpy as np

from drone_arm_sim.arm_preset_control import (
    ArmPresetCommander,
    DirectXyMotionGate,
    DirectXyOwnerStateIngress,
    JOINT_NAMES,
)


class Recorder:
    def __init__(self):
        self.messages = []

    def publish(self, message):
        self.messages.append(message)


def test_direct_xy_gate_is_a_noop_when_feature_is_disabled():
    gate = DirectXyMotionGate(False)
    assert gate.blocking_reason(100.0) is None


def test_direct_xy_gate_requires_continuous_fresh_ready_and_non_inhibit():
    gate = DirectXyMotionGate(True, ready_hold_s=0.10, ready_timeout_s=0.15)
    gate.update_inhibit(False)
    gate.update_ready(True, 10.0)
    assert gate.blocking_reason(10.09) == "direct_xy_ready_not_held"
    assert gate.blocking_reason(10.10) is None
    assert gate.blocking_reason(10.151) == "direct_xy_ready_stale"
    gate.update_ready(True, 10.16)
    assert gate.blocking_reason(10.20) == "direct_xy_ready_not_held"
    assert gate.blocking_reason(10.26) is None


def test_direct_xy_gate_rejects_missing_or_asserted_inhibit():
    gate = DirectXyMotionGate(True, ready_hold_s=0.0)
    gate.update_ready(True, 1.0)
    assert gate.blocking_reason(1.0) == "inhibit_state_missing"
    gate.update_inhibit(True)
    assert gate.blocking_reason(1.0) == "motion_inhibited"


def test_wait_handshake_announces_idle_before_waiting(monkeypatch):
    controller = object.__new__(ArmPresetCommander)
    controller.direct_xy_gate = DirectXyMotionGate(True, ready_hold_s=0.0)
    controller.direct_xy_gate.update_inhibit(False)
    controller.direct_xy_gate.update_ready(True, time.monotonic())
    controller.direct_xy_gate.update_owner_state(
        "px4_xy", 0, time.monotonic()
    )
    controller.motion_command_active = True
    controller.motion_abort_reason = None
    controller.motion_publisher = Recorder()

    ready_seen_before_refresh = []

    def spin_once(node, timeout_sec):
        assert node is controller
        ready_seen_before_refresh.append(node.direct_xy_gate.ready)
        controller.direct_xy_gate.update_inhibit(False)
        controller.direct_xy_gate.update_ready(True, time.monotonic())
        controller.direct_xy_gate.update_owner_state(
            "px4_xy", 0, time.monotonic()
        )

    monkeypatch.setattr(
        "drone_arm_sim.arm_preset_control.rclpy.spin_once", spin_once
    )
    controller.wait_for_direct_xy_ready(timeout_s=0.1)

    assert controller.motion_publisher.messages[0].data is False
    assert controller.motion_command_active is False
    assert ready_seen_before_refresh[0] is False


def test_wait_handshake_treats_premotion_inhibit_as_recoverable_level(
    monkeypatch,
):
    controller = object.__new__(ArmPresetCommander)
    controller.direct_xy_gate = DirectXyMotionGate(True, ready_hold_s=0.0)
    controller.motion_command_active = False
    controller.motion_abort_reason = None
    controller.motion_publisher = Recorder()
    calls = 0

    def spin_once(node, timeout_sec):
        nonlocal calls
        calls += 1
        if calls == 1:
            node.direct_xy_gate.update_inhibit(True)
            node.direct_xy_gate.update_ready(False, time.monotonic())
            return
        node.direct_xy_gate.update_inhibit(False)
        node.direct_xy_gate.update_ready(True, time.monotonic())
        node.direct_xy_gate.update_owner_state("px4_xy", 0, time.monotonic())

    monkeypatch.setattr(
        "drone_arm_sim.arm_preset_control.rclpy.spin_once", spin_once
    )

    controller.wait_for_direct_xy_ready(timeout_s=0.1)

    assert calls >= 2
    assert controller.motion_publisher.messages[0].data is False


def test_inhibit_during_motion_publishes_measured_hold_and_cancels_session():
    controller = object.__new__(ArmPresetCommander)
    controller.direct_xy_gate = DirectXyMotionGate(True, ready_hold_s=0.0)
    controller.direct_xy_gate.update_inhibit(False)
    controller.direct_xy_gate.update_ready(True, time.monotonic())
    controller.direct_xy_gate.update_owner_state(
        "px4_xy", 0, time.monotonic()
    )
    controller.motion_command_active = True
    controller.motion_abort_reason = None
    controller.latest_positions = {
        name: 0.1 * index for index, name in enumerate(JOINT_NAMES)
    }
    controller.publisher = Recorder()
    controller.motion_publisher = Recorder()
    controller.get_logger = lambda: SimpleNamespace(error=lambda message: None)

    controller.on_motion_inhibit(SimpleNamespace(data=True))

    assert controller.motion_command_active is False
    assert controller.motion_abort_reason == "direct_xy_motion_inhibited"
    assert controller.motion_publisher.messages[-1].data is False
    hold = controller.publisher.messages[-1]
    assert hold.joint_names == JOINT_NAMES
    np.testing.assert_allclose(
        hold.points[0].positions,
        [controller.latest_positions[name] for name in JOINT_NAMES],
    )
    np.testing.assert_allclose(hold.points[0].velocities, np.zeros(6))
    np.testing.assert_allclose(hold.points[0].accelerations, np.zeros(6))


def test_runtime_owner_state_replaces_ready_freshness_assertion():
    gate = DirectXyMotionGate(
        True, ready_hold_s=0.0, ready_timeout_s=0.15, owner_timeout_s=0.15
    )
    gate.update_inhibit(False)
    gate.update_ready(True, 10.0)
    gate.update_owner_state("px4_xy", 3, 10.0)
    gate.begin_motion(10.0)
    gate.update_owner_state("direct_xy", 4, 10.05)

    # The pre-authorisation is stale at 10.16, but the actual owner report is
    # fresh.  Runtime safety must not accidentally reuse the ready timestamp.
    assert gate.blocking_reason(10.16) == "direct_xy_ready_stale"
    assert gate.runtime_blocking_reason(10.16) is None


def test_motion_begin_requires_fresh_px4_owner_epoch_baseline():
    gate = DirectXyMotionGate(True, ready_hold_s=0.0, owner_timeout_s=0.15)
    gate.update_inhibit(False)
    gate.update_ready(True, 10.0)
    with np.testing.assert_raises_regex(RuntimeError, "preowner_state_missing"):
        gate.begin_motion(10.0)
    gate.update_owner_state("px4_xy", 5, 10.0)
    gate.begin_motion(10.01)
    assert gate.minimum_motion_epoch == 6


def test_handoff_state_keeps_executor_waiting_without_false_abort():
    gate = DirectXyMotionGate(
        True, ready_hold_s=0.0, owner_timeout_s=0.15
    )
    gate.update_inhibit(False)
    gate.update_ready(True, 10.0)
    gate.update_owner_state("px4_xy", 5, 10.0)
    gate.begin_motion(10.01)

    gate.update_owner_state("handoff", 6, 10.02)
    assert gate.phase == "awaiting_direct_xy"
    assert gate.runtime_blocking_reason(10.03) == "direct_xy_owner_pending"

    gate.update_owner_state("direct_xy", 6, 10.04)
    assert gate.phase == "direct_xy"
    assert gate.runtime_blocking_reason(10.05) is None


def test_same_epoch_handoff_cannot_regress_dedicated_active_ack():
    gate = DirectXyMotionGate(
        True, ready_hold_s=0.0, owner_timeout_s=0.15
    )
    gate.update_inhibit(False)
    gate.update_ready(True, 10.0)
    gate.update_owner_state("px4_xy", 5, 10.0)
    gate.begin_motion(10.01)
    gate.update_owner_state("direct_xy", 6, 10.02)

    gate.update_owner_state("handoff", 6, 10.03)

    assert gate.owner_state == "direct_xy"
    assert gate.phase == "direct_xy"
    assert gate.runtime_blocking_reason(10.04) is None


def test_publish_motion_rising_spins_until_new_epoch_direct_owner(monkeypatch):
    controller = object.__new__(ArmPresetCommander)
    controller.direct_xy_gate = DirectXyMotionGate(
        True, ready_hold_s=0.0, owner_timeout_s=0.15
    )
    controller.direct_xy_gate.update_inhibit(False)
    controller.direct_xy_gate.update_ready(True, time.monotonic())
    controller.direct_xy_gate.update_owner_state("px4_xy", 7, time.monotonic())
    controller.direct_xy_owner_entry_timeout_s = 0.1
    controller.motion_command_active = False
    controller.motion_abort_reason = None
    controller.motion_publisher = Recorder()

    spins = []

    def spin_once(node, timeout_sec):
        spins.append(timeout_sec)
        node.direct_xy_gate.update_owner_state(
            "direct_xy", 8, time.monotonic()
        )

    monkeypatch.setattr(
        "drone_arm_sim.arm_preset_control.rclpy.spin_once", spin_once
    )
    controller.publish_motion_active(True)

    assert spins
    assert controller.motion_command_active is True
    assert controller.direct_xy_gate.phase == "direct_xy"
    assert controller.motion_publisher.messages[0].data is True


def test_spin_callbacks_drains_bounded_ready_queue(monkeypatch):
    controller = object.__new__(ArmPresetCommander)
    calls = []

    def spin_once(node, timeout_sec):
        assert node is controller
        calls.append(timeout_sec)

    monkeypatch.setattr(
        "drone_arm_sim.arm_preset_control.rclpy.spin_once", spin_once
    )
    controller.spin_callbacks(0.02, drain_limit=4)
    assert calls == [0.02, 0.0, 0.0, 0.0, 0.0]


def test_dedicated_owner_ingress_is_consumed_despite_joint_callback_flood(
    monkeypatch,
):
    """Reproduce v5: the 8 ms ACK must bypass the always-ready joint queue."""
    controller = object.__new__(ArmPresetCommander)
    controller.direct_xy_gate = DirectXyMotionGate(
        True, ready_hold_s=0.0, owner_timeout_s=0.15
    )
    controller.direct_xy_gate.update_inhibit(False)
    controller.direct_xy_gate.update_ready(True, 10.0)
    controller.direct_xy_gate.update_owner_state("px4_xy", 1, 10.0)
    controller.direct_xy_gate.begin_motion(10.01)
    controller.motion_command_active = True
    controller.motion_abort_reason = None

    class FakeIngress:
        def __init__(self):
            self.snapshot = (
                10.02,
                json.dumps(
                    {
                        "schema": "my_drone.arm-direct-xy-state.v1",
                        "ownership_epoch": 2,
                        "state": "direct_xy",
                    }
                ),
            )

        def take_latest(self):
            result = self.snapshot
            self.snapshot = None
            return result

    controller.direct_xy_owner_ingress = FakeIngress()
    joint_callbacks = []

    def joint_only_spin(_node, timeout_sec):
        joint_callbacks.append(timeout_sec)

    monkeypatch.setattr(
        "drone_arm_sim.arm_preset_control.rclpy.spin_once", joint_only_spin
    )
    monkeypatch.setattr(
        "drone_arm_sim.arm_preset_control.time.monotonic", lambda: 10.03
    )

    controller.spin_callbacks(0.01, drain_limit=32)

    assert len(joint_callbacks) == 33
    assert controller.direct_xy_gate.phase == "direct_xy"
    assert controller.direct_xy_gate.runtime_blocking_reason(10.03) is None


def test_owner_ingress_callback_is_cache_only():
    import inspect

    source = inspect.getsource(DirectXyOwnerStateIngress._cache_owner)
    assert "direct_xy_gate" not in source
    assert ".publish(" not in source


def test_owner_ingress_uses_reliable_latest_qos():
    import inspect

    source = inspect.getsource(DirectXyOwnerStateIngress.__init__)
    assert "reliability=ReliabilityPolicy.RELIABLE" in source
    assert "history=HistoryPolicy.KEEP_LAST" in source
    assert "depth=1" in source
    assert "ReliabilityPolicy.BEST_EFFORT" not in source


def test_external_owner_ingress_lifecycle_is_shared_and_idempotent(monkeypatch):
    """Every ArmPresetCommander entry point must use the same owner ingress."""
    controller = object.__new__(ArmPresetCommander)
    controller.direct_xy_gate = DirectXyMotionGate(True)
    controller.direct_xy_owner_ingress = None
    controller._direct_xy_owner_executor = None
    controller._direct_xy_owner_executor_thread = None

    created_ingresses = []
    created_executors = []

    class FakeIngress:
        def __init__(self, topic):
            self.topic = topic
            self.destroy_count = 0
            created_ingresses.append(self)

        def destroy_node(self):
            self.destroy_count += 1

    class FakeExecutor:
        def __init__(self):
            self.nodes = []
            self.shutdown_count = 0
            created_executors.append(self)

        def add_node(self, node):
            self.nodes.append(node)

        def spin_once(self, timeout_sec):
            assert timeout_sec == 0.0

        def shutdown(self, timeout_sec):
            assert timeout_sec == 1.0
            self.shutdown_count += 1

    monkeypatch.setattr(
        "drone_arm_sim.arm_preset_control._environment_flag",
        lambda name, default=False: name == "ARM_DIRECT_XY_EXTERNAL_GUARDIAN",
    )
    monkeypatch.setattr(
        "drone_arm_sim.arm_preset_control.DirectXyOwnerStateIngress",
        FakeIngress,
    )
    monkeypatch.setattr(
        "drone_arm_sim.arm_preset_control.SingleThreadedExecutor", FakeExecutor
    )
    controller.start_direct_xy_owner_ingress()
    controller.start_direct_xy_owner_ingress()

    assert len(created_ingresses) == 1
    assert created_ingresses[0].topic == "/my_drone/arm_direct_xy_guardian/state"
    assert created_executors[0].nodes == created_ingresses
    assert controller._direct_xy_owner_executor_thread is None
    assert controller._direct_xy_owner_ingress_terminal_fault is None

    controller.stop_direct_xy_owner_ingress()
    controller.stop_direct_xy_owner_ingress()

    assert created_executors[0].shutdown_count == 1
    assert created_ingresses[0].destroy_count == 1
    assert controller.direct_xy_owner_ingress is None


def test_command_loop_drives_owner_ingress_before_150ms_stale_decision(
    monkeypatch,
):
    """A command poll refreshes owner state even without a background thread."""
    controller = object.__new__(ArmPresetCommander)
    controller.direct_xy_gate = DirectXyMotionGate(
        True, ready_hold_s=0.0, owner_timeout_s=0.15
    )
    controller.direct_xy_gate.update_inhibit(False)
    controller.direct_xy_gate.update_ready(True, 30.0)
    controller.direct_xy_gate.update_owner_state("px4_xy", 4, 30.0)
    controller.direct_xy_gate.begin_motion(30.01)
    controller.motion_command_active = True
    controller.motion_abort_reason = None
    controller._direct_xy_owner_executor_thread = None
    controller._direct_xy_owner_ingress_terminal_fault = None

    class FakeIngress:
        snapshot = None

        def take_latest(self):
            result = self.snapshot
            self.snapshot = None
            return result

    ingress = FakeIngress()
    controller.direct_xy_owner_ingress = ingress

    report = json.dumps(
        {
            "schema": "my_drone.arm-direct-xy-state.v1",
            "ownership_epoch": 5,
            "state": "direct_xy",
        }
    )

    class FakeExecutor:
        def __init__(self):
            self.calls = 0

        def spin_once(self, timeout_sec):
            assert timeout_sec == 0.0
            self.calls += 1
            ingress.snapshot = (30.20, report)

    executor = FakeExecutor()
    controller._direct_xy_owner_executor = executor
    monkeypatch.setattr(
        "drone_arm_sim.arm_preset_control.rclpy.spin_once",
        lambda _node, timeout_sec: None,
    )
    monkeypatch.setattr(
        "drone_arm_sim.arm_preset_control.time.monotonic", lambda: 30.21
    )

    # The prior 30.0 owner sample is 210 ms old.  Explicit ingress service
    # refreshes it at the same command boundary, without relaxing 150 ms.
    controller.spin_callbacks(0.0, drain_limit=0)

    assert executor.calls == 2
    assert controller.direct_xy_gate.owner_stamp_s == 30.20
    assert controller.direct_xy_gate.owner_epoch == 5
    assert controller.direct_xy_gate.runtime_blocking_reason(30.21) is None


def test_owner_ingress_executor_fault_is_explicit_and_aborts_motion():
    controller = object.__new__(ArmPresetCommander)
    controller.motion_command_active = True
    controller.motion_abort_reason = None
    controller._direct_xy_owner_executor_thread = None
    controller._direct_xy_owner_ingress_terminal_fault = None

    class BrokenExecutor:
        def spin_once(self, timeout_sec):
            raise RuntimeError("executor stopped")

    class Logger:
        def __init__(self):
            self.errors = []

        def error(self, message):
            self.errors.append(message)

    logger = Logger()
    aborted = []
    controller._direct_xy_owner_executor = BrokenExecutor()
    controller.get_logger = lambda: logger
    controller.emergency_hold = lambda reason: aborted.append(reason)

    controller._drive_direct_xy_owner_ingress()

    assert controller._direct_xy_owner_ingress_terminal_fault == (
        "executor_exception:RuntimeError:executor stopped"
    )
    assert aborted == ["direct_xy_owner_ingress_terminal_fault"]
    assert "ARM_DIRECT_XY_OWNER_INGRESS_TERMINAL_FAULT" in logger.errors[0]


def test_external_owner_report_preserves_guardian_attested_epoch(monkeypatch):
    """Consume the guardian's exact report without inventing a local epoch."""
    controller = object.__new__(ArmPresetCommander)
    controller.direct_xy_gate = DirectXyMotionGate(
        True, ready_hold_s=0.0, owner_timeout_s=0.15
    )
    controller.direct_xy_gate.update_inhibit(False)
    controller.direct_xy_gate.update_ready(True, 20.0)
    controller.motion_command_active = False
    controller.motion_abort_reason = None

    report = {
        "schema": "my_drone.arm-direct-xy-state.v1",
        "controller_session_id": "controller-session-v6",
        "ownership_epoch": 41,
        "lease_generation": 73,
        "state": "px4_xy",
        "reallocator_fresh": True,
        "direct_xy_force_enabled_ack": False,
        "direct_xy_force_epoch_ack": -1,
    }
    controller._apply_direct_xy_state_json(json.dumps(report), 20.01)

    assert controller.direct_xy_gate.owner_state == "px4_xy"
    assert controller.direct_xy_gate.owner_epoch == 41
    assert controller.direct_xy_gate.owner_stamp_s == 20.01
    controller.direct_xy_gate.begin_motion(20.02)
    assert controller.direct_xy_gate.minimum_motion_epoch == 42


def test_guardian_abort_logs_exact_identity_and_reason_once():
    controller = object.__new__(ArmPresetCommander)
    controller.direct_xy_gate = DirectXyMotionGate(True)
    controller.motion_command_active = False
    controller.motion_abort_reason = None
    controller._last_direct_xy_guardian_abort_key = None
    errors = []
    controller.get_logger = lambda: SimpleNamespace(error=errors.append)
    report = json.dumps(
        {
            "schema": "my_drone.arm-direct-xy-state.v1",
            "controller_session_id": "session-v7",
            "ownership_epoch": 3,
            "intent_generation": 5,
            "lease_generation": 8,
            "state": "aborting",
            "watchdog_reason": "main_intent_stale",
            "watchdog_detail": "",
        }
    )

    controller._apply_direct_xy_state_json(report, 30.0)
    controller._apply_direct_xy_state_json(report, 30.01)

    assert len(errors) == 1
    assert "ARM_DIRECT_XY_GUARDIAN_ABORT" in errors[0]
    assert '"controller_session_id": "session-v7"' in errors[0]
    assert '"ownership_epoch": 3' in errors[0]
    assert '"intent_generation": 5' in errors[0]
    assert '"lease_generation": 8' in errors[0]
    assert '"watchdog_reason": "main_intent_stale"' in errors[0]


def test_abort_can_clear_only_after_fresh_normal_preauthorization(monkeypatch):
    controller = object.__new__(ArmPresetCommander)
    controller.direct_xy_gate = DirectXyMotionGate(
        True, ready_hold_s=0.0, owner_timeout_s=0.15
    )
    controller.motion_command_active = False
    controller.motion_abort_reason = "direct_xy_owner_aborting"
    monkeypatch.setattr(
        "drone_arm_sim.arm_preset_control.time.monotonic", lambda: 40.0
    )

    with np.testing.assert_raises_regex(
        RuntimeError, "inhibit_state_missing"
    ):
        controller.clear_motion_abort_after_fresh_preauthorization()
    assert controller.motion_abort_reason == "direct_xy_owner_aborting"

    controller.direct_xy_gate.update_inhibit(False)
    controller.direct_xy_gate.update_ready(True, 40.0)
    controller.direct_xy_gate.update_owner_state("px4_xy", 9, 40.0)
    controller.clear_motion_abort_after_fresh_preauthorization()
    assert controller.motion_abort_reason is None


def test_owner_loss_callback_immediately_holds_measured_pose():
    controller = object.__new__(ArmPresetCommander)
    controller.direct_xy_gate = DirectXyMotionGate(
        True, ready_hold_s=0.0, owner_timeout_s=0.15
    )
    now = time.monotonic()
    controller.direct_xy_gate.update_inhibit(False)
    controller.direct_xy_gate.update_ready(True, now)
    controller.direct_xy_gate.update_owner_state("px4_xy", 2, now)
    controller.direct_xy_gate.begin_motion(now)
    controller.direct_xy_gate.update_owner_state("direct_xy", 3, now)
    controller.motion_command_active = True
    controller.motion_abort_reason = None
    controller.latest_positions = {
        name: 0.05 * index for index, name in enumerate(JOINT_NAMES)
    }
    controller.publisher = Recorder()
    controller.motion_publisher = Recorder()
    controller.get_logger = lambda: SimpleNamespace(error=lambda message: None)

    controller.on_direct_xy_state(SimpleNamespace(data=json.dumps({
        "schema": "my_drone.arm-direct-xy-state.v1",
        "ownership_epoch": 3,
        "state": "px4_xy",
    })))

    assert controller.motion_abort_reason == "direct_xy_owner_lost"
    assert controller.motion_command_active is False
    assert controller.motion_publisher.messages[-1].data is False
    assert controller.publisher.messages[-1].joint_names == JOINT_NAMES
