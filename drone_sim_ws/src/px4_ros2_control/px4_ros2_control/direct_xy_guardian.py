#!/usr/bin/env python3
"""Process-isolated fail-closed guardian for direct-XY ownership.

The flight controller publishes intent only.  This node is the sole publisher
of the physical force lease and of the owner state consumed by the arm
executor.  Subscription callbacks only parse and cache latest values; all
state transitions and publications happen from the 100 Hz guardian timer.
"""

from __future__ import annotations

import gc
import json
import math
import threading
import time
from dataclasses import dataclass
from typing import Any

try:
    import rclpy
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.node import Node
    from rclpy.qos import (
        DurabilityPolicy,
        HistoryPolicy,
        QoSProfile,
        ReliabilityPolicy,
    )
    from std_msgs.msg import String
except ModuleNotFoundError:  # Pure protocol tests do not require ROS 2.
    rclpy = None
    SingleThreadedExecutor = object
    Node = object
    String = None


INTENT_SCHEMA = "my_drone.arm-direct-xy-controller-intent.v1"
FORCE_SCHEMA = "my_drone.arm-direct-xy-force-command.v1"
OWNER_SCHEMA = "my_drone.arm-direct-xy-state.v1"
REALLOCATOR_SCHEMA = "my_drone.base1-reallocator-state.v1"

PRODUCER_WATCHDOG_S = 0.040
ENTRY_TIMEOUT_S = 0.150
PHYSICAL_LEASE_S = 0.200
MAIN_INTENT_LEASE_S = 0.250
BOOTSTRAP_REFRESH_S = 0.010
ACTIVE_REFRESH_S = 0.025


def configure_guardian_runtime(gc_module: Any = gc) -> dict[str, bool]:
    """Remove nondeterministic cyclic-GC pauses from the watchdog process.

    CPython reference counting remains active, so the short-lived JSON and
    message objects used by the 100 Hz loop are still reclaimed immediately.
    The guardian deliberately owns only two nodes for the process lifetime;
    disabling the separate cyclic collector therefore trades a bounded,
    auditable process-lifetime risk for eliminating unbounded collector pauses
    inside the 40 ms safety watchdog.

    ``gc_module`` is injectable so tests do not alter the test runner's global
    collector state.
    """
    was_enabled = bool(gc_module.isenabled())
    gc_module.disable()
    return {
        "cyclic_gc_was_enabled": was_enabled,
        "cyclic_gc_enabled": bool(gc_module.isenabled()),
    }


@dataclass(frozen=True)
class CachedLevel:
    producer_monotonic_s: float
    received_monotonic_s: float
    payload: dict[str, Any]


@dataclass(frozen=True)
class GuardianOutput:
    force_command: dict[str, Any] | None
    owner_state: dict[str, Any]


def _finite_float(value: Any) -> float:
    result = float(value)
    if not math.isfinite(result):
        raise ValueError("value must be finite")
    return result


def owner_state_publish_evidence(
    owner_state: dict[str, Any],
    *,
    publish_monotonic_s: float,
    sequence: int,
    previous_publish_monotonic_s: float | None,
) -> dict[str, Any]:
    """Add publisher-side timing evidence without changing the state schema.

    The fields are additive so existing owner-state consumers remain
    compatible.  A skipped ``sequence`` identifies a failed publish attempt;
    a large ``timer_lateness_s`` identifies a guardian service gap independently
    of subscriber/DDS receipt time.
    """
    publish_now = _finite_float(publish_monotonic_s)
    publish_sequence = int(sequence)
    if publish_sequence < 1:
        raise ValueError("sequence must be positive")
    if previous_publish_monotonic_s is None:
        timer_lateness_s = 0.0
    else:
        previous = _finite_float(previous_publish_monotonic_s)
        timer_lateness_s = max(
            0.0, publish_now - previous - BOOTSTRAP_REFRESH_S
        )
    result = dict(owner_state)
    result.update(
        {
            "monotonic_s": publish_now,
            "sequence": publish_sequence,
            "timer_lateness_s": timer_lateness_s,
        }
    )
    return result


def parse_controller_intent(data: str, received_monotonic_s: float) -> CachedLevel | None:
    """Validate one controller level without changing guardian state."""
    try:
        payload = json.loads(data)
        if payload.get("schema") != INTENT_SCHEMA:
            return None
        producer = _finite_float(payload["monotonic_s"])
        session = str(payload["controller_session_id"])
        epoch = int(payload["ownership_epoch"])
        generation = int(payload["intent_generation"])
        state = str(payload["state"])
        for name in ("motion_active", "mixed_setpoint_active", "force_requested"):
            if type(payload[name]) is not bool:
                return None
    except (KeyError, TypeError, ValueError, json.JSONDecodeError):
        return None
    if (
        not session
        or len(session) > 128
        or epoch < 0
        or generation < 0
        or state not in {"px4_xy", "handoff", "direct_xy", "exit", "aborting"}
    ):
        return None
    return CachedLevel(producer, float(received_monotonic_s), payload)


def parse_reallocator_report(data: str, received_monotonic_s: float) -> CachedLevel | None:
    """Parse one physical report; semantic health is judged by ``step``."""
    try:
        payload = json.loads(data)
        if payload.get("schema") != REALLOCATOR_SCHEMA:
            return None
        producer = _finite_float(payload["monotonic_s"])
    except (KeyError, TypeError, ValueError, json.JSONDecodeError):
        return None
    return CachedLevel(producer, float(received_monotonic_s), payload)


def reallocator_contract_healthy(report: dict[str, Any]) -> bool:
    """Apply the unchanged allocation/flight contract to one report."""
    try:
        feasibility = _finite_float(report["feasibility_scale"])
        residual = _finite_float(report["residual_norm"])
        saturated = int(report["saturated"])
    except (KeyError, TypeError, ValueError):
        return False
    return bool(
        report.get("event") in {"allocated", "zero_overlay"}
        and report.get("source_fresh") is True
        and report.get("flight_allowed") is True
        and report.get("headroom_ok") is True
        and report.get("position_feedback_enabled") is True
        and report.get("position_feedback_ready") is True
        and report.get("truth_fresh") is True
        and feasibility >= 1.0 - 1.0e-6
        and residual <= 1.0e-3
        and saturated == 0
        and report.get("allocation_limited", False) is False
    )


def reallocator_contract_failure_reason(report: dict[str, Any]) -> str:
    """Return an actionable semantic fault, distinct from report transport."""
    if report.get("source_fresh") is not True:
        details = report.get("source_invalid_reasons")
        if isinstance(details, list) and details:
            return "reallocator_source_unhealthy:" + ",".join(
                str(item) for item in details
            )
        return "reallocator_source_unhealthy"
    boolean_contract = (
        ("flight_allowed", True),
        ("headroom_ok", True),
        ("position_feedback_enabled", True),
        ("position_feedback_ready", True),
        ("truth_fresh", True),
        ("allocation_limited", False),
    )
    for field, expected in boolean_contract:
        if report.get(field, False) is not expected:
            return "reallocator_contract_unhealthy:" + field
    try:
        feasibility = _finite_float(report["feasibility_scale"])
        residual = _finite_float(report["residual_norm"])
        saturated = int(report["saturated"])
    except (KeyError, TypeError, ValueError):
        return "reallocator_contract_unhealthy:invalid_numeric_field"
    if feasibility < 1.0 - 1.0e-6:
        return "reallocator_contract_unhealthy:feasibility_scale"
    if residual > 1.0e-3:
        return "reallocator_contract_unhealthy:residual_norm"
    if saturated != 0:
        return "reallocator_contract_unhealthy:saturated"
    if report.get("event") not in {"allocated", "zero_overlay"}:
        return "reallocator_contract_unhealthy:event"
    return ""


class DirectXyGuardianCore:
    """Deterministic timer-driven guardian with cache-only ingress."""

    def __init__(self) -> None:
        self._cache_lock = threading.Lock()
        self._intent: CachedLevel | None = None
        self._report: CachedLevel | None = None
        self.session_id = ""
        self.intent_epoch = 0
        self.intent_generation = 0
        self.lease_generation = 0
        self.lease_requested = False
        self.active = False
        # Normal exit is a two-phase handoff.  ``releasing`` starts only
        # after the controller has published finite PX4 XY and withdrawn its
        # request.  It remains set until the physical reallocator echoes this
        # exact session/epoch/lease generation with force disabled.
        self.releasing = False
        self.entry_started_s = 0.0
        self.first_active_receipt_s = 0.0
        self.last_active_receipt_s = 0.0
        self.last_force_publish_s = 0.0
        self.last_fault = ""
        self.last_fault_detail = ""
        self.faulted_intent_key: tuple[str, int, int] | None = None
        self.last_step_monotonic_s: float | None = None
        self.timer_lateness_s = 0.0

    def cache_intent_json(self, data: str, received_monotonic_s: float) -> bool:
        level = parse_controller_intent(data, received_monotonic_s)
        if level is None:
            return False
        with self._cache_lock:
            current = self._intent
            if current is not None and level.producer_monotonic_s <= current.producer_monotonic_s:
                return False
            self._intent = level
        return True

    def cache_reallocator_json(self, data: str, received_monotonic_s: float) -> bool:
        level = parse_reallocator_report(data, received_monotonic_s)
        if level is None:
            return False
        with self._cache_lock:
            current = self._report
            if current is not None and level.producer_monotonic_s <= current.producer_monotonic_s:
                return False
            self._report = level
        return True

    def _snapshots(self) -> tuple[CachedLevel | None, CachedLevel | None]:
        with self._cache_lock:
            return self._intent, self._report

    def producer_stream_fresh(self, now_s: float) -> bool:
        """Report whether the physical producer is arriving inside 40 ms."""
        _intent, report = self._snapshots()
        return self._fresh(report, _finite_float(now_s), PRODUCER_WATCHDOG_S)

    @staticmethod
    def _fresh(level: CachedLevel | None, now_s: float, timeout_s: float) -> bool:
        if level is None:
            return False
        producer_age = now_s - level.producer_monotonic_s
        receipt_age = now_s - level.received_monotonic_s
        return bool(0.0 <= producer_age < timeout_s and 0.0 <= receipt_age < timeout_s)

    def _force_command(self, enabled: bool) -> dict[str, Any]:
        return {
            "schema": FORCE_SCHEMA,
            "controller_session_id": self.session_id,
            "ownership_epoch": int(self.intent_epoch),
            "enabled": bool(enabled),
            "lease_generation": int(self.lease_generation),
            "physical_lease_s": PHYSICAL_LEASE_S,
        }

    def _owner_state(
        self,
        now_s: float,
        state: str,
        reason: str = "",
        detail: str = "",
    ) -> dict[str, Any]:
        intent, report = self._snapshots()
        try:
            report_epoch = int(
                -1 if report is None else report.payload.get(
                    "direct_xy_force_epoch", -1
                )
            )
            report_generation = int(
                -1 if report is None else report.payload.get(
                    "direct_xy_force_lease_generation", -1
                )
            )
        except (TypeError, ValueError):
            report_generation = -1
        report_fresh = bool(
            self._fresh(report, now_s, PRODUCER_WATCHDOG_S)
            and report is not None
            and reallocator_contract_healthy(report.payload)
            and str(report.payload.get("direct_xy_force_session_id", ""))
            == self.session_id
            and report_epoch == self.intent_epoch
            and report_generation == self.lease_generation
        )
        report_value = {} if report is None else report.payload
        prepared = bool(
            report_fresh
            and report_value.get("motion_active") is True
            and report_value.get("position_feedback_prepared") is True
            and report_value.get("position_target_latched") is True
        )
        ready = bool(
            report_fresh and report_value.get("position_feedback_ready") is True
        )
        intent_motion = bool(
            intent is not None and intent.payload.get("motion_active") is True
        )
        return {
            "schema": OWNER_SCHEMA,
            "controller_session_id": self.session_id,
            "ownership_epoch": int(self.intent_epoch),
            "intent_generation": int(self.intent_generation),
            "lease_generation": int(self.lease_generation),
            "state": state,
            "watchdog_reason": reason,
            "watchdog_detail": str(detail),
            "reallocator_fresh": report_fresh,
            "position_feedback_active": bool(self.active),
            "position_feedback_prepared": prepared,
            "position_feedback_ready": ready,
            "motion_active": intent_motion,
            "direct_xy_force_enabled_ack": bool(self.active),
            "direct_xy_force_epoch_ack": int(self.intent_epoch if self.active else -1),
            "ack_source": "direct_xy_guardian_process" if self.active else "",
        }

    def _revoke(
        self, now_s: float, reason: str, *, detail: str = ""
    ) -> GuardianOutput:
        command = None
        if self.lease_requested or self.active or self.releasing:
            self.lease_generation += 1
            self.lease_requested = False
            self.active = False
            self.releasing = False
            self.last_force_publish_s = now_s
            command = self._force_command(False)
        self.last_fault = reason
        self.last_fault_detail = str(detail)
        if self.session_id:
            self.faulted_intent_key = (
                self.session_id,
                self.intent_epoch,
                self.intent_generation,
            )
        return GuardianOutput(
            command,
            self._owner_state(now_s, "aborting", reason, self.last_fault_detail),
        )

    def _matching_disabled_report(
        self, report: CachedLevel | None, now_s: float
    ) -> bool:
        """Require physical proof for the exact normal-exit lease identity."""
        if not self._fresh(report, now_s, PRODUCER_WATCHDOG_S):
            return False
        assert report is not None
        value = report.payload
        if not reallocator_contract_healthy(value):
            return False
        try:
            report_epoch = int(value["direct_xy_force_epoch"])
            report_generation = int(value["direct_xy_force_lease_generation"])
            report_session = str(value["direct_xy_force_session_id"])
        except (KeyError, TypeError, ValueError):
            return False
        return bool(
            report_session == self.session_id
            and report_epoch == self.intent_epoch
            and report_generation == self.lease_generation
            and value.get("event") == "zero_overlay"
            and value.get("position_feedback_active") is False
            and value.get("direct_xy_force_command_fresh") is False
            and value.get("direct_xy_force_enabled") is False
        )

    def _orderly_release(
        self, now_s: float, report: CachedLevel | None
    ) -> GuardianOutput:
        command = None
        if not self.releasing:
            self.lease_generation += 1
            self.lease_requested = False
            self.releasing = True
            self.last_force_publish_s = now_s
            command = self._force_command(False)
        elif self._matching_disabled_report(report, now_s):
            # This is the only normal path that declares PX4 the completed
            # owner.  Until this exact physical ACK, owner_state remains
            # ``exit`` and reports the direct owner as potentially active.
            self.active = False
            self.releasing = False
            self.last_fault = ""
            self.last_fault_detail = ""
            self.faulted_intent_key = None
            return GuardianOutput(None, self._owner_state(now_s, "px4_xy"))
        elif now_s - self.last_force_publish_s >= ACTIVE_REFRESH_S:
            # Retry the same False generation; never manufacture a new ACK
            # identity on every timer tick.
            self.last_force_publish_s = now_s
            command = self._force_command(False)
        self.last_fault = ""
        self.last_fault_detail = ""
        self.faulted_intent_key = None
        return GuardianOutput(command, self._owner_state(now_s, "exit"))

    def force_publish_failed(self, now_s: float) -> GuardianOutput:
        """Latch a failed wire write without allowing the timer to die."""
        return self._revoke(_finite_float(now_s), "force_publish_failed")

    def _active_report_health(
        self, report: CachedLevel | None, now_s: float
    ) -> tuple[bool, str]:
        if not self._fresh(report, now_s, PRODUCER_WATCHDOG_S):
            return False, "reallocator_report_stale"
        assert report is not None
        value = report.payload
        contract_fault = reallocator_contract_failure_reason(value)
        if contract_fault:
            return False, contract_fault
        try:
            report_epoch = int(value["direct_xy_force_epoch"])
            report_generation = int(value["direct_xy_force_lease_generation"])
            report_session = str(value["direct_xy_force_session_id"])
        except (KeyError, TypeError, ValueError):
            return False, "reallocator_identity_invalid"
        active_contract = (
            ("event", "allocated"),
            # The controller intent is the authority for ownership lifetime.
            # At normal trajectory completion the reallocator can observe the
            # arm-motion False edge before the controller callback does.  It
            # deliberately keeps position feedback and the leased force owner
            # alive during that short handoff so the controller can publish a
            # finite PX4 XY setpoint before releasing the lease.  Treating the
            # report's motion_active flag as an independent kill switch creates
            # an unowned-XY interval.  All physical/source/identity watchdogs
            # below remain mandatory during this handoff.
            ("position_feedback_prepared", True),
            ("position_target_latched", True),
            ("position_feedback_active", True),
            ("direct_xy_force_command_fresh", True),
            ("direct_xy_force_enabled", True),
        )
        for field, expected in active_contract:
            if value.get(field) != expected:
                return False, "reallocator_active_contract_unhealthy:" + field
        if report_session != self.session_id:
            return False, "reallocator_identity_mismatch:session"
        if report_epoch != self.intent_epoch:
            return False, "reallocator_identity_mismatch:epoch"
        if report_generation != self.lease_generation:
            return False, "reallocator_identity_mismatch:lease_generation"
        return True, ""

    def _report_watchdog_detail(
        self, report: CachedLevel | None, now_s: float
    ) -> str:
        if report is None:
            producer_age_ms = receipt_age_ms = float("inf")
        else:
            producer_age_ms = 1000.0 * (now_s - report.producer_monotonic_s)
            receipt_age_ms = 1000.0 * (now_s - report.received_monotonic_s)
        return (
            f"producer_age_ms={producer_age_ms:.3f},"
            f"receipt_age_ms={receipt_age_ms:.3f},"
            f"timer_lateness_ms={1000.0 * self.timer_lateness_s:.3f}"
        )

    def _matching_active_report(self, report: CachedLevel | None, now_s: float) -> bool:
        healthy, _reason = self._active_report_health(report, now_s)
        return healthy

    def step(self, now_s: float) -> GuardianOutput:
        """Advance once; the caller may publish only the returned values."""
        now = _finite_float(now_s)
        if self.last_step_monotonic_s is None:
            self.timer_lateness_s = 0.0
        else:
            self.timer_lateness_s = max(
                0.0,
                now - self.last_step_monotonic_s - BOOTSTRAP_REFRESH_S,
            )
        self.last_step_monotonic_s = now
        intent, report = self._snapshots()
        intent_fresh = self._fresh(intent, now, MAIN_INTENT_LEASE_S)
        if not intent_fresh:
            if self.session_id:
                return self._revoke(now, "main_intent_stale")
            return GuardianOutput(None, self._owner_state(now, "aborting", "main_intent_missing"))

        assert intent is not None
        value = intent.payload
        session = str(value["controller_session_id"])
        epoch = int(value["ownership_epoch"])
        intent_generation = int(value["intent_generation"])
        requested = bool(value["force_requested"])
        mixed = bool(value["mixed_setpoint_active"])
        motion = bool(value["motion_active"])

        if not self.session_id:
            # Establish a boot/session only from an explicitly finite owner.
            # This prevents a queued True from a dead controller process from
            # becoming the first authority after guardian/reallocator restart.
            if requested or mixed or motion:
                return GuardianOutput(None, self._owner_state(now, "aborting", "session_not_established"))
            self.session_id = session
            self.intent_epoch = epoch
            self.intent_generation = intent_generation
            self.lease_generation += 1
            self.last_force_publish_s = now
            return GuardianOutput(self._force_command(False), self._owner_state(now, "px4_xy"))

        if session != self.session_id:
            if requested or mixed or motion:
                return self._revoke(now, "controller_session_mismatch")
            if self.lease_requested or self.active:
                return self._revoke(now, "controller_session_changed")
            # A controller restart must first advertise a finite idle owner.
            # Only then may it replace the prior boot/session and reset its
            # process-local epoch/generation counters.
            self.session_id = session
            self.intent_epoch = epoch
            self.intent_generation = intent_generation
            self.lease_generation += 1
            self.faulted_intent_key = None
            self.last_fault = ""
            self.last_fault_detail = ""
            self.last_force_publish_s = now
            return GuardianOutput(
                self._force_command(False), self._owner_state(now, "px4_xy")
            )
        if epoch < self.intent_epoch or intent_generation < self.intent_generation:
            return self._revoke(now, "controller_intent_regressed")

        new_intent = bool(
            epoch != self.intent_epoch or intent_generation != self.intent_generation
        )
        if new_intent:
            self.intent_epoch = epoch
            self.intent_generation = intent_generation

        valid_request = bool(
            requested
            and mixed
            and motion
            and value.get("state") in {"handoff", "direct_xy"}
        )
        if requested and not valid_request:
            inconsistent_fields = []
            if not mixed:
                inconsistent_fields.append("mixed_setpoint_active=false")
            if not motion:
                inconsistent_fields.append("motion_active=false")
            if value.get("state") not in {"handoff", "direct_xy"}:
                inconsistent_fields.append("state=" + str(value.get("state")))
            return self._revoke(
                now,
                "controller_intent_inconsistent",
                detail=",".join(inconsistent_fields),
            )
        if not requested:
            if self.lease_requested or self.active or self.releasing:
                return self._orderly_release(now, report)
            if new_intent:
                # Epoch/generation changes must also be established at the
                # physical reallocator while PX4 still owns finite XY.  If we
                # only update the guardian's local identity, the reallocator
                # continues echoing the preceding epoch and the controller's
                # pre-entry readiness gate can never become true.
                self.lease_generation += 1
                self.last_force_publish_s = now
                self.last_fault = ""
                self.last_fault_detail = ""
                self.faulted_intent_key = None
                return GuardianOutput(
                    self._force_command(False),
                    self._owner_state(now, "px4_xy"),
                )
            self.last_fault = ""
            self.last_fault_detail = ""
            self.faulted_intent_key = None
            return GuardianOutput(None, self._owner_state(now, "px4_xy"))

        intent_key = (session, epoch, intent_generation)
        if self.faulted_intent_key == intent_key:
            return GuardianOutput(
                None,
                self._owner_state(
                    now,
                    "aborting",
                    self.last_fault or "intent_fault_latched",
                    self.last_fault_detail,
                ),
            )

        if new_intent or not self.lease_requested:
            if self.releasing:
                return self._revoke(now, "exit_not_acknowledged")
            self.lease_generation += 1
            self.lease_requested = True
            self.active = False
            self.entry_started_s = intent.received_monotonic_s
            self.first_active_receipt_s = 0.0
            self.last_active_receipt_s = 0.0
            self.last_force_publish_s = 0.0
            self.last_fault = ""
            self.last_fault_detail = ""
            self.faulted_intent_key = None

        matching_active, active_report_fault = self._active_report_health(report, now)
        if matching_active:
            assert report is not None
            if not self.active:
                if not (
                    self.entry_started_s
                    <= report.received_monotonic_s
                    < self.entry_started_s + ENTRY_TIMEOUT_S
                ):
                    return self._revoke(now, "owner_entry_timeout")
                self.first_active_receipt_s = report.received_monotonic_s
            self.active = True
            self.last_active_receipt_s = report.received_monotonic_s
        elif self.active:
            return self._revoke(
                now,
                active_report_fault,
                detail=(
                    self._report_watchdog_detail(report, now)
                    if active_report_fault == "reallocator_report_stale"
                    else ""
                ),
            )
        elif now >= self.entry_started_s + ENTRY_TIMEOUT_S:
            return self._revoke(now, "owner_entry_timeout")

        refresh_period = ACTIVE_REFRESH_S if self.active else BOOTSTRAP_REFRESH_S
        command = None
        if self.last_force_publish_s <= 0.0 or now - self.last_force_publish_s >= refresh_period:
            self.last_force_publish_s = now
            command = self._force_command(True)
        state = "direct_xy" if self.active else "handoff"
        return GuardianOutput(command, self._owner_state(now, state))


def ingress_first_node_order(
    registered_nodes: list[Any], ingress_node: Any, watchdog_node: Any
) -> list[Any]:
    """Return an explicit ingress-before-watchdog executor node order."""
    ordered = [
        node
        for node in (ingress_node, watchdog_node)
        if node in registered_nodes
    ]
    ordered.extend(
        node
        for node in registered_nodes
        if node is not ingress_node and node is not watchdog_node
    )
    return ordered


class DirectXyGuardianIngressNode(Node):
    """DDS ingress only; callbacks validate/cache and never publish."""

    def __init__(self, core: DirectXyGuardianCore) -> None:
        super().__init__("my_drone_direct_xy_guardian_ingress")
        self.core = core
        intent_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        reallocator_report_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self.create_subscription(
            String,
            "/my_drone/arm_direct_xy_controller_intent",
            self._intent_callback,
            intent_qos,
        )
        self.create_subscription(
            String,
            "/my_drone/base1_reallocator/state",
            self._report_callback,
            reallocator_report_qos,
        )

    def _intent_callback(self, message) -> None:
        self.core.cache_intent_json(message.data, time.monotonic())

    def _report_callback(self, message) -> None:
        self.core.cache_reallocator_json(message.data, time.monotonic())


class DirectXyGuardianNode(Node):
    """Watchdog/publisher only; the ordered executor drains ingress first."""

    def __init__(self, core: DirectXyGuardianCore | None = None) -> None:
        super().__init__("my_drone_direct_xy_guardian")
        self.core = core if core is not None else DirectXyGuardianCore()
        self._producer_ready_streak = 0
        self._ready_reported = False
        self._last_abort_log_key = None
        self._owner_publish_sequence = 0
        self._last_owner_publish_monotonic_s: float | None = None
        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self.force_publisher = self.create_publisher(
            String, "/my_drone/arm_direct_xy_force_command", qos
        )
        self.owner_publisher = self.create_publisher(
            String, "/my_drone/arm_direct_xy_guardian/state", qos
        )
        self.create_timer(BOOTSTRAP_REFRESH_S, self._timer_callback)

    def _timer_callback(self) -> None:
        now = time.monotonic()
        if self.core.producer_stream_fresh(now):
            self._producer_ready_streak += 1
        else:
            self._producer_ready_streak = 0
        if self._producer_ready_streak >= 3 and not self._ready_reported:
            self._ready_reported = True
            self.get_logger().info(
                "DIRECT_XY_GUARDIAN_READY producer_watchdog_s=0.040"
            )
        output = self.core.step(now)
        if output.owner_state.get("state") == "aborting":
            abort_log_key = (
                str(output.owner_state.get("controller_session_id", "")),
                int(output.owner_state.get("ownership_epoch", -1)),
                int(output.owner_state.get("intent_generation", -1)),
                int(output.owner_state.get("lease_generation", -1)),
                str(output.owner_state.get("watchdog_reason", "")),
                str(output.owner_state.get("watchdog_detail", "")),
            )
            if abort_log_key != self._last_abort_log_key:
                self._last_abort_log_key = abort_log_key
                self.get_logger().error(
                    "DIRECT_XY_GUARDIAN_ABORT "
                    + json.dumps(
                        {
                            "controller_session_id": abort_log_key[0],
                            "ownership_epoch": abort_log_key[1],
                            "intent_generation": abort_log_key[2],
                            "lease_generation": abort_log_key[3],
                            "watchdog_reason": abort_log_key[4],
                            "watchdog_detail": abort_log_key[5],
                        },
                        sort_keys=True,
                    )
                )
        else:
            self._last_abort_log_key = None
        if output.force_command is not None:
            try:
                self.force_publisher.publish(
                    String(data=json.dumps(output.force_command, sort_keys=True))
                )
            except Exception as error:  # DDS failure must not kill the guardian.
                self.get_logger().error(f"DIRECT_XY_GUARDIAN_FORCE_PUBLISH_FAILED {error}")
                output = self.core.force_publish_failed(time.monotonic())
                if output.force_command is not None:
                    try:
                        self.force_publisher.publish(
                            String(data=json.dumps(output.force_command, sort_keys=True))
                        )
                    except Exception as revoke_error:
                        self.get_logger().error(
                            "DIRECT_XY_GUARDIAN_REVOKE_PUBLISH_FAILED "
                            + str(revoke_error)
                        )
        try:
            publish_now = time.monotonic()
            self._owner_publish_sequence += 1
            owner_state = owner_state_publish_evidence(
                output.owner_state,
                publish_monotonic_s=publish_now,
                sequence=self._owner_publish_sequence,
                previous_publish_monotonic_s=(
                    self._last_owner_publish_monotonic_s
                ),
            )
            self._last_owner_publish_monotonic_s = publish_now
            self.owner_publisher.publish(
                String(data=json.dumps(owner_state, sort_keys=True))
            )
        except Exception as error:  # Physical lease still expires at 200 ms.
            self.get_logger().error(f"DIRECT_XY_GUARDIAN_STATE_PUBLISH_FAILED {error}")


class IngressFirstSingleThreadedExecutor(SingleThreadedExecutor):
    """Serialize one ready batch with ingress nodes before watchdog timers.

    Jazzy's executor visits timers before subscriptions *inside one node*.
    Splitting entities across nodes and overriding ``get_nodes`` with an
    explicit order makes all ready ingress handlers run before the watchdog
    node's timer.  It does not rely on the base executor's unordered node set.
    """

    def __init__(self, ingress_node: Node, watchdog_node: Node) -> None:
        super().__init__()
        self._ingress_node = ingress_node
        self._watchdog_node = watchdog_node
        super().add_node(ingress_node)
        super().add_node(watchdog_node)

    def get_nodes(self) -> list[Node]:
        return ingress_first_node_order(
            super().get_nodes(), self._ingress_node, self._watchdog_node
        )


def main() -> None:
    if rclpy is None:
        raise SystemExit("ROS 2 Python packages are not available")
    runtime_config = configure_guardian_runtime()
    rclpy.init()
    core = DirectXyGuardianCore()
    ingress_node = DirectXyGuardianIngressNode(core)
    watchdog_node = DirectXyGuardianNode(core)
    watchdog_node.get_logger().info(
        "DIRECT_XY_GUARDIAN_RUNTIME "
        f"cyclic_gc_enabled={str(runtime_config['cyclic_gc_enabled']).lower()} "
        "cpython_reference_counting=enabled"
    )
    executor = IngressFirstSingleThreadedExecutor(ingress_node, watchdog_node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        ingress_node.destroy_node()
        watchdog_node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
