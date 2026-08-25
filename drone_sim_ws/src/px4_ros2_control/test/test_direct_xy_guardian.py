import inspect
import json
import math

import pytest

from px4_ros2_control.direct_xy_guardian import (
    ACTIVE_REFRESH_S,
    BOOTSTRAP_REFRESH_S,
    ENTRY_TIMEOUT_S,
    MAIN_INTENT_LEASE_S,
    PHYSICAL_LEASE_S,
    PRODUCER_WATCHDOG_S,
    DirectXyGuardianCore,
    DirectXyGuardianIngressNode,
    DirectXyGuardianNode,
    FORCE_SCHEMA,
    INTENT_SCHEMA,
    IngressFirstSingleThreadedExecutor,
    configure_guardian_runtime,
    ingress_first_node_order,
    owner_state_publish_evidence,
)


class FakeGc:
    def __init__(self, enabled=True):
        self.enabled = enabled
        self.disable_calls = 0

    def isenabled(self):
        return self.enabled

    def disable(self):
        self.disable_calls += 1
        self.enabled = False


def test_guardian_runtime_disables_only_injected_cyclic_gc():
    fake_gc = FakeGc(enabled=True)

    result = configure_guardian_runtime(fake_gc)

    assert result == {
        "cyclic_gc_was_enabled": True,
        "cyclic_gc_enabled": False,
    }
    assert fake_gc.disable_calls == 1


def intent(now, *, session="session-a", epoch=1, generation=1, requested=False):
    return json.dumps(
        {
            "schema": INTENT_SCHEMA,
            "monotonic_s": now,
            "controller_session_id": session,
            "ownership_epoch": epoch,
            "intent_generation": generation,
            "state": "handoff" if requested else "px4_xy",
            "motion_active": requested,
            "mixed_setpoint_active": requested,
            "force_requested": requested,
        }
    )


def report(
    now,
    *,
    session="session-a",
    epoch=1,
    generation=2,
    active=True,
    motion_active=True,
):
    return json.dumps(
        {
            "schema": "my_drone.base1-reallocator-state.v1",
            "monotonic_s": now,
            "event": "allocated" if active else "zero_overlay",
            "source_fresh": True,
            "flight_allowed": True,
            "headroom_ok": True,
            "position_feedback_enabled": True,
            "position_feedback_ready": True,
            "truth_fresh": True,
            "feasibility_scale": 1.0,
            "residual_norm": 0.0,
            "saturated": 0,
            "allocation_limited": False,
            "motion_active": motion_active,
            "position_feedback_prepared": True,
            "position_target_latched": True,
            "position_feedback_active": active,
            "direct_xy_force_command_fresh": active,
            "direct_xy_force_enabled": active,
            "direct_xy_force_session_id": session,
            "direct_xy_force_epoch": epoch,
            "direct_xy_force_lease_generation": generation,
        }
    )


def establish(core, start=10.0):
    assert core.cache_intent_json(intent(start, epoch=0, generation=0), start)
    output = core.step(start)
    assert output.force_command["enabled"] is False
    assert output.force_command["physical_lease_s"] == pytest.approx(PHYSICAL_LEASE_S)


def test_callbacks_are_cache_only_and_never_publish():
    assert ".publish(" not in inspect.getsource(
        DirectXyGuardianIngressNode._intent_callback
    )
    assert ".publish(" not in inspect.getsource(
        DirectXyGuardianIngressNode._report_callback
    )
    assert ".publish(" in inspect.getsource(DirectXyGuardianNode._timer_callback)
    assert "except Exception" in inspect.getsource(DirectXyGuardianNode._timer_callback)


def test_ros_wrapper_separates_ingress_and_explicitly_orders_ready_batch():
    ingress_constructor = inspect.getsource(DirectXyGuardianIngressNode.__init__)
    watchdog_constructor = inspect.getsource(DirectXyGuardianNode.__init__)
    executor_source = inspect.getsource(IngressFirstSingleThreadedExecutor)
    main_source = inspect.getsource(
        __import__(
            "px4_ros2_control.direct_xy_guardian", fromlist=["main"]
        ).main
    )
    assert "create_subscription" in ingress_constructor
    # Both controller intent and reallocator evidence are safety-critical
    # latest levels.  Neither endpoint may silently fall back to best effort.
    assert ingress_constructor.count(
        "reliability=ReliabilityPolicy.RELIABLE"
    ) == 2
    assert "ReliabilityPolicy.BEST_EFFORT" not in ingress_constructor
    assert "reallocator_report_qos" in ingress_constructor
    assert "create_subscription" not in watchdog_constructor
    assert "reliability=ReliabilityPolicy.RELIABLE" in watchdog_constructor
    assert "ReliabilityPolicy.BEST_EFFORT" not in watchdog_constructor
    assert '"/my_drone/arm_direct_xy_force_command", qos' in watchdog_constructor
    assert '"/my_drone/arm_direct_xy_guardian/state", qos' in watchdog_constructor
    assert "create_timer" in watchdog_constructor
    assert "super().get_nodes()" in executor_source
    assert "ingress_first_node_order" in executor_source
    assert "IngressFirstSingleThreadedExecutor" in main_source
    assert "executor.spin()" in main_source


def test_concurrently_ready_batch_is_ordered_ingress_before_watchdog():
    ingress = object()
    watchdog = object()
    unrelated = object()

    ordered = ingress_first_node_order(
        [watchdog, unrelated, ingress], ingress, watchdog
    )

    assert ordered == [ingress, watchdog, unrelated]


def test_thresholds_are_not_relaxed():
    assert PRODUCER_WATCHDOG_S == pytest.approx(0.040)
    assert ENTRY_TIMEOUT_S == pytest.approx(0.150)
    assert PHYSICAL_LEASE_S == pytest.approx(0.200)
    assert MAIN_INTENT_LEASE_S == pytest.approx(0.250)
    assert BOOTSTRAP_REFRESH_S == pytest.approx(0.010)
    assert ACTIVE_REFRESH_S == pytest.approx(0.025)


def test_owner_state_publish_evidence_is_additive_and_auditable():
    original = {"schema": "my_drone.arm-direct-xy-state.v1", "state": "direct_xy"}

    first = owner_state_publish_evidence(
        original,
        publish_monotonic_s=100.0,
        sequence=1,
        previous_publish_monotonic_s=None,
    )
    second = owner_state_publish_evidence(
        original,
        publish_monotonic_s=100.037,
        sequence=2,
        previous_publish_monotonic_s=100.0,
    )

    assert original == {
        "schema": "my_drone.arm-direct-xy-state.v1",
        "state": "direct_xy",
    }
    assert first["monotonic_s"] == pytest.approx(100.0)
    assert first["sequence"] == 1
    assert first["timer_lateness_s"] == pytest.approx(0.0)
    assert second["monotonic_s"] == pytest.approx(100.037)
    assert second["sequence"] == 2
    assert second["timer_lateness_s"] == pytest.approx(0.027)
    assert second["schema"] == original["schema"]
    assert second["state"] == original["state"]


def test_owner_state_publish_evidence_rejects_invalid_sequence_and_timestamp():
    with pytest.raises(ValueError, match="sequence must be positive"):
        owner_state_publish_evidence(
            {},
            publish_monotonic_s=1.0,
            sequence=0,
            previous_publish_monotonic_s=None,
        )
    with pytest.raises(ValueError, match="finite"):
        owner_state_publish_evidence(
            {},
            publish_monotonic_s=float("nan"),
            sequence=1,
            previous_publish_monotonic_s=None,
        )


def test_stale_report_detail_distinguishes_data_age_from_timer_lateness():
    core = DirectXyGuardianCore()
    establish(core, start=20.0)
    assert core.cache_intent_json(intent(20.001, requested=True), 20.001)
    requested = core.step(20.001)
    generation = requested.force_command["lease_generation"]
    assert core.cache_reallocator_json(
        report(20.011, generation=generation), 20.011
    )
    assert core.step(20.011).owner_state["state"] == "direct_xy"

    stale = core.step(20.052)

    assert stale.owner_state["watchdog_reason"] == "reallocator_report_stale"
    assert stale.owner_state["watchdog_detail"] == (
        "producer_age_ms=41.000,receipt_age_ms=41.000,"
        "timer_lateness_ms=31.000"
    )


def _active_core_with_report(report_stamp: float) -> tuple[DirectXyGuardianCore, int]:
    core = DirectXyGuardianCore()
    establish(core, start=report_stamp - 0.011)
    intent_stamp = report_stamp - 0.010
    assert core.cache_intent_json(
        intent(intent_stamp, requested=True), intent_stamp
    )
    requested = core.step(intent_stamp)
    generation = requested.force_command["lease_generation"]
    assert core.cache_reallocator_json(
        report(report_stamp, generation=generation), report_stamp
    )
    assert core.step(report_stamp).owner_state["state"] == "direct_xy"
    return core, generation


def test_queued_fresh_report_is_consumed_before_stale_watchdog_step():
    core, generation = _active_core_with_report(30.0)
    watchdog_now = 30.041
    queued_report_stamp = watchdog_now - 0.001

    # This is the production ingress-first batch order: consume the latest
    # KEEP_LAST report callback, then run the simultaneously-ready watchdog.
    assert core.cache_reallocator_json(
        report(queued_report_stamp, generation=generation),
        queued_report_stamp,
    )
    output = core.step(watchdog_now)

    assert output.owner_state["state"] == "direct_xy"
    assert output.owner_state["watchdog_reason"] == ""


def test_no_queued_fresh_report_still_revokes_at_40ms():
    core, _generation = _active_core_with_report(40.0)

    deadline = math.nextafter(40.0 + PRODUCER_WATCHDOG_S, math.inf)
    output = core.step(deadline)

    assert output.owner_state["state"] == "aborting"
    assert output.owner_state["watchdog_reason"] == "reallocator_report_stale"


def test_guardian_ready_requires_a_fresh_physical_producer_sample():
    core = DirectXyGuardianCore()
    assert core.producer_stream_fresh(1.0) is False
    assert core.cache_reallocator_json(report(1.0), 1.0)
    assert core.producer_stream_fresh(1.0 + PRODUCER_WATCHDOG_S - 1.0e-6)
    assert core.producer_stream_fresh(1.0 + PRODUCER_WATCHDOG_S) is False


def test_idle_handshake_then_strict_entry_and_active_refresh():
    core = DirectXyGuardianCore()
    establish(core)
    assert core.cache_intent_json(intent(10.01, requested=True), 10.01)
    bootstrap = core.step(10.01)
    assert bootstrap.force_command["schema"] == FORCE_SCHEMA
    assert bootstrap.force_command["enabled"] is True
    lease_generation = bootstrap.force_command["lease_generation"]
    assert core.cache_reallocator_json(
        report(10.02, generation=lease_generation), 10.02
    )
    active = core.step(10.02)
    assert active.owner_state["state"] == "direct_xy"
    assert active.owner_state["direct_xy_force_enabled_ack"] is True
    assert active.force_command is None
    refreshed = core.step(10.02 + ACTIVE_REFRESH_S + 1.0e-6)
    assert refreshed.force_command["enabled"] is True


def test_entry_boundary_is_strict_and_revokes():
    core = DirectXyGuardianCore()
    establish(core)
    core.cache_intent_json(intent(20.0, requested=True), 20.0)
    bootstrap = core.step(20.0)
    generation = bootstrap.force_command["lease_generation"]
    boundary = 20.0 + ENTRY_TIMEOUT_S
    core.cache_reallocator_json(report(boundary, generation=generation), boundary)
    failed = core.step(boundary)
    assert failed.force_command["enabled"] is False
    assert failed.owner_state["watchdog_reason"] == "owner_entry_timeout"


def test_continuous_physical_reports_use_40ms_watchdog():
    core = DirectXyGuardianCore()
    establish(core)
    core.cache_intent_json(intent(30.0, requested=True), 30.0)
    generation = core.step(30.0).force_command["lease_generation"]
    core.cache_reallocator_json(report(30.01, generation=generation), 30.01)
    assert core.step(30.01).owner_state["state"] == "direct_xy"
    core.cache_intent_json(intent(30.039, requested=True), 30.039)
    still_active = core.step(30.01 + PRODUCER_WATCHDOG_S - 1.0e-6)
    assert still_active.owner_state["state"] == "direct_xy"
    failed = core.step(30.01 + PRODUCER_WATCHDOG_S + 1.0e-9)
    assert failed.force_command["enabled"] is False
    assert failed.owner_state["watchdog_reason"] == "reallocator_report_stale"


def test_semantic_source_fault_is_not_misreported_as_transport_stale():
    core = DirectXyGuardianCore()
    establish(core)
    core.cache_intent_json(intent(31.0, requested=True), 31.0)
    generation = core.step(31.0).force_command["lease_generation"]
    core.cache_reallocator_json(report(31.01, generation=generation), 31.01)
    assert core.step(31.01).owner_state["state"] == "direct_xy"

    unhealthy = json.loads(report(31.02, generation=generation))
    unhealthy["source_fresh"] = False
    unhealthy["position_feedback_ready"] = False
    unhealthy["source_invalid_reasons"] = ["coupling_state"]
    assert core.cache_reallocator_json(json.dumps(unhealthy), 31.02)
    failed = core.step(31.02)
    assert failed.force_command["enabled"] is False
    assert (
        failed.owner_state["watchdog_reason"]
        == "reallocator_source_unhealthy:coupling_state"
    )


def test_normal_motion_false_edge_waits_for_controller_idle_intent():
    """The report may see completion before the controller publishes finite XY."""
    core = DirectXyGuardianCore()
    establish(core)
    core.cache_intent_json(intent(32.0, requested=True), 32.0)
    generation = core.step(32.0).force_command["lease_generation"]
    core.cache_reallocator_json(
        report(32.01, generation=generation, motion_active=True), 32.01
    )
    assert core.step(32.01).owner_state["state"] == "direct_xy"

    # Cross-process delivery can expose this edge first.  Keep the healthy
    # physical owner until the controller's ordered finite-XY -> idle intent.
    core.cache_reallocator_json(
        report(32.02, generation=generation, motion_active=False), 32.02
    )
    waiting = core.step(32.02)
    assert waiting.owner_state["state"] == "direct_xy"
    assert waiting.force_command is None

    core.cache_intent_json(
        intent(32.03, epoch=1, generation=2, requested=False), 32.03
    )
    released = core.step(32.03)
    assert released.force_command["enabled"] is False
    assert released.owner_state["state"] == "exit"
    release_generation = released.force_command["lease_generation"]
    assert released.owner_state["direct_xy_force_enabled_ack"] is True

    # Local False publication is not completion.  Only the exact physical
    # disabled echo may transfer authoritative ownership back to PX4.
    core.cache_reallocator_json(
        report(
            32.04,
            generation=release_generation,
            active=False,
            motion_active=False,
        ),
        32.04,
    )
    acknowledged = core.step(32.04)
    assert acknowledged.force_command is None
    assert acknowledged.owner_state["state"] == "px4_xy"
    assert acknowledged.owner_state["direct_xy_force_enabled_ack"] is False


def test_normal_exit_retries_same_generation_until_exact_disabled_ack():
    core = DirectXyGuardianCore()
    establish(core)
    core.cache_intent_json(intent(33.0, requested=True), 33.0)
    active_generation = core.step(33.0).force_command["lease_generation"]
    core.cache_reallocator_json(
        report(33.01, generation=active_generation), 33.01
    )
    assert core.step(33.01).owner_state["state"] == "direct_xy"

    core.cache_intent_json(
        intent(33.02, generation=2, requested=False), 33.02
    )
    release = core.step(33.02)
    release_generation = release.force_command["lease_generation"]
    assert release.owner_state["state"] == "exit"

    # Wrong generation, session and epoch are all rejected independently.
    mismatches = (
        report(33.03, generation=release_generation - 1, active=False),
        report(
            33.04,
            session="wrong-session",
            generation=release_generation,
            active=False,
        ),
        report(
            33.05,
            epoch=2,
            generation=release_generation,
            active=False,
        ),
    )
    for index, sample in enumerate(mismatches, start=3):
        stamp = 33.0 + index / 100.0
        core.cache_intent_json(
            intent(stamp, generation=2, requested=False), stamp
        )
        core.cache_reallocator_json(sample, stamp)
        waiting = core.step(stamp)
        assert waiting.owner_state["state"] == "exit"

    retry_time = 33.05 + ACTIVE_REFRESH_S + 1.0e-6
    core.cache_intent_json(
        intent(retry_time, generation=2, requested=False), retry_time
    )
    retry = core.step(retry_time)
    assert retry.force_command["enabled"] is False
    assert retry.force_command["lease_generation"] == release_generation
    assert retry.owner_state["state"] == "exit"

    core.cache_reallocator_json(
        report(
            retry_time + 0.005,
            generation=release_generation,
            active=False,
            motion_active=False,
        ),
        retry_time + 0.005,
    )
    done = core.step(retry_time + 0.005)
    assert done.owner_state["state"] == "px4_xy"


def test_requested_intent_with_false_motion_still_fails_closed():
    """Controller serialization must not weaken guardian consistency checks."""
    core = DirectXyGuardianCore()
    establish(core)
    core.cache_intent_json(intent(34.0, requested=True), 34.0)
    active_generation = core.step(34.0).force_command["lease_generation"]
    core.cache_reallocator_json(
        report(34.01, generation=active_generation), 34.01
    )
    assert core.step(34.01).owner_state["state"] == "direct_xy"

    inconsistent = json.loads(intent(34.02, requested=True))
    inconsistent["motion_active"] = False
    core.cache_intent_json(json.dumps(inconsistent), 34.02)
    failed = core.step(34.02)
    assert failed.force_command["enabled"] is False
    assert failed.owner_state["state"] == "aborting"
    assert failed.owner_state["watchdog_reason"] == "controller_intent_inconsistent"
    assert failed.owner_state["watchdog_detail"] == "motion_active=false"


def test_inconsistent_intent_detail_identifies_each_failed_field():
    core = DirectXyGuardianCore()
    establish(core)
    malformed = json.loads(intent(34.0, requested=True))
    malformed["mixed_setpoint_active"] = False
    malformed["motion_active"] = False
    malformed["state"] = "px4_xy"
    core.cache_intent_json(json.dumps(malformed), 34.0)

    failed = core.step(34.0)

    assert failed.owner_state["watchdog_reason"] == "controller_intent_inconsistent"
    assert failed.owner_state["watchdog_detail"] == (
        "mixed_setpoint_active=false,motion_active=false,state=px4_xy"
    )


def test_main_intent_lease_expires_at_250ms():
    core = DirectXyGuardianCore()
    establish(core)
    stale = core.step(10.0 + MAIN_INTENT_LEASE_S)
    assert stale.owner_state["watchdog_reason"] == "main_intent_stale"


def test_new_session_cannot_take_over_with_active_true():
    core = DirectXyGuardianCore()
    establish(core)
    core.cache_intent_json(
        intent(10.1, session="session-b", epoch=1, generation=1, requested=True),
        10.1,
    )
    rejected = core.step(10.1)
    assert rejected.owner_state["watchdog_reason"] == "controller_session_mismatch"


def test_faulted_intent_cannot_restart_without_new_generation():
    core = DirectXyGuardianCore()
    establish(core)
    core.cache_intent_json(intent(50.0, requested=True), 50.0)
    core.step(50.0)
    failed = core.step(50.0 + ENTRY_TIMEOUT_S)
    assert failed.force_command["enabled"] is False
    same_intent = core.step(50.0 + ENTRY_TIMEOUT_S + 0.01)
    assert same_intent.force_command is None
    assert same_intent.owner_state["state"] == "aborting"
    core.cache_intent_json(
        intent(50.17, generation=2, requested=True), 50.17
    )
    restarted = core.step(50.17)
    assert restarted.force_command["enabled"] is True


def test_new_session_requires_idle_handshake_then_can_replace_old_session():
    core = DirectXyGuardianCore()
    establish(core)
    core.cache_intent_json(
        intent(10.1, session="session-b", epoch=0, generation=0), 10.1
    )
    rebound = core.step(10.1)
    assert rebound.force_command["enabled"] is False
    assert rebound.force_command["controller_session_id"] == "session-b"


def test_report_must_echo_exact_session_epoch_and_generation():
    core = DirectXyGuardianCore()
    establish(core)
    core.cache_intent_json(intent(40.0, requested=True), 40.0)
    generation = core.step(40.0).force_command["lease_generation"]
    core.cache_reallocator_json(
        report(40.01, generation=generation - 1), 40.01
    )
    pending = core.step(40.01)
    assert pending.owner_state["state"] == "handoff"
    core.cache_reallocator_json(report(40.02, generation=generation), 40.02)
    assert core.step(40.02).owner_state["state"] == "direct_xy"


def test_new_idle_epoch_is_physically_handshaken_before_preentry_ready():
    core = DirectXyGuardianCore()
    establish(core)
    first_generation = core.lease_generation
    core.cache_reallocator_json(
        report(
            10.01,
            epoch=0,
            generation=first_generation,
            active=False,
        ),
        10.01,
    )
    initial_ready = core.step(10.01)
    assert initial_ready.owner_state["reallocator_fresh"] is True
    assert initial_ready.owner_state["position_feedback_ready"] is True

    assert core.cache_intent_json(
        intent(10.02, epoch=1, generation=0, requested=False), 10.02
    )
    new_epoch = core.step(10.02)
    assert new_epoch.force_command["enabled"] is False
    assert new_epoch.force_command["ownership_epoch"] == 1
    assert new_epoch.force_command["lease_generation"] > first_generation
    # The old physical report must not authorize the new epoch.
    assert new_epoch.owner_state["position_feedback_ready"] is False

    core.cache_reallocator_json(
        report(
            10.03,
            epoch=1,
            generation=new_epoch.force_command["lease_generation"],
            active=False,
        ),
        10.03,
    )
    synchronized = core.step(10.03)
    assert synchronized.owner_state["reallocator_fresh"] is True
    assert synchronized.owner_state["position_feedback_ready"] is True


def test_force_publish_failure_latches_fail_closed_revoke():
    core = DirectXyGuardianCore()
    establish(core)
    core.cache_intent_json(intent(60.0, requested=True), 60.0)
    enabled = core.step(60.0).force_command
    failed = core.force_publish_failed(60.001)
    assert failed.force_command["enabled"] is False
    assert failed.force_command["lease_generation"] > enabled["lease_generation"]
    assert failed.owner_state["state"] == "aborting"
    assert failed.owner_state["watchdog_reason"] == "force_publish_failed"
