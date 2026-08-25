"""Command and verify SO101 arm presets through ros2_control."""

from __future__ import annotations

import argparse
import json
import os
from pathlib import Path
import sys
import threading
import time

from ament_index_python.packages import get_package_share_directory
from builtin_interfaces.msg import Duration
import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, String
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from drone_arm_sim.trajectory_preflight import TrajectoryPreflight


JOINT_NAMES = [
    "shoulder_pan",
    "shoulder_lift",
    "elbow_flex",
    "wrist_flex",
    "wrist_roll",
    "gripper",
]


def _environment_flag(name: str, default: bool = False) -> bool:
    value = os.environ.get(name)
    if value is None:
        return bool(default)
    return value.strip().lower() in {"1", "true", "yes", "on"}


class DirectXyMotionGate:
    """Two-stage, fail-closed gate for the direct horizontal-force owner.

    Before motion, the executor consumes the short-lived ``ready``
    pre-authorisation.  After the motion rising edge, readiness is no longer
    a runtime assertion: actual ownership is proven by a fresh, matching
    ``state=direct_xy`` report.  ``inhibit`` remains authoritative in both
    phases.
    """

    def __init__(
        self,
        enabled: bool,
        *,
        ready_hold_s: float = 0.10,
        ready_timeout_s: float = 0.15,
        owner_timeout_s: float = 0.15,
    ) -> None:
        self.enabled = bool(enabled)
        self.ready_hold_s = max(0.0, float(ready_hold_s))
        self.ready_timeout_s = max(0.01, float(ready_timeout_s))
        self.owner_timeout_s = max(0.01, float(owner_timeout_s))
        self.ready = False
        self.ready_stamp_s: float | None = None
        self.ready_since_s: float | None = None
        self.inhibit_received = False
        self.inhibited = True if self.enabled else False
        self.phase = "preauthorization"
        self.owner_state = "px4_xy"
        self.owner_stamp_s: float | None = None
        self.owner_epoch = -1
        self.minimum_motion_epoch = 0

    def update_ready(self, ready: bool, now_s: float) -> None:
        value = bool(ready)
        stamp = float(now_s)
        if value and not self.ready:
            self.ready_since_s = stamp
        elif not value:
            self.ready_since_s = None
        self.ready = value
        self.ready_stamp_s = stamp

    def update_inhibit(self, inhibited: bool) -> None:
        self.inhibit_received = True
        self.inhibited = bool(inhibited)

    def blocking_reason(self, now_s: float) -> str | None:
        """Return the pre-authorisation blocking reason."""
        if not self.enabled:
            return None
        if not self.inhibit_received:
            return "inhibit_state_missing"
        if self.inhibited:
            return "motion_inhibited"
        if not self.ready or self.ready_stamp_s is None:
            return "direct_xy_not_ready"
        age_s = float(now_s) - self.ready_stamp_s
        if age_s < 0.0 or age_s > self.ready_timeout_s:
            # A later True sample must establish a new continuous interval;
            # do not inherit readiness across a telemetry gap.
            self.ready = False
            self.ready_since_s = None
            return "direct_xy_ready_stale"
        if (
            self.ready_since_s is None
            or float(now_s) - self.ready_since_s + 1.0e-9 < self.ready_hold_s
        ):
            return "direct_xy_ready_not_held"
        return None

    def begin_motion(self, now_s: float) -> None:
        reason = self.preauthorization_blocking_reason(now_s)
        if reason is not None:
            raise RuntimeError("ARM_DIRECT_XY_NOT_READY reason=" + reason)
        self.phase = "awaiting_direct_xy"
        self.minimum_motion_epoch = self.owner_epoch + 1

    def preauthorization_blocking_reason(self, now_s: float) -> str | None:
        reason = self.blocking_reason(now_s)
        if reason is not None or not self.enabled:
            return reason
        if self.owner_stamp_s is None:
            return "direct_xy_preowner_state_missing"
        owner_age_s = float(now_s) - self.owner_stamp_s
        if owner_age_s < 0.0 or owner_age_s > self.owner_timeout_s:
            return "direct_xy_preowner_state_stale"
        if self.owner_state != "px4_xy":
            return "direct_xy_preowner_not_px4"
        return None

    def end_motion(self) -> None:
        self.phase = "preauthorization"

    def update_owner_state(self, state: str, epoch: int, now_s: float) -> None:
        value = str(state)
        sequence = int(epoch)
        if value not in {"px4_xy", "handoff", "direct_xy", "aborting"}:
            return
        # Reject delayed state from an older action/session.
        if sequence < self.owner_epoch:
            return
        if (
            sequence == self.owner_epoch
            and self.phase == "direct_xy"
            and self.owner_state == "direct_xy"
            and value == "handoff"
        ):
            # The dedicated safety ingress can observe the allocator's active
            # ACK while the main control callback is still finishing the
            # preceding handoff publication.  Do not let that same-epoch,
            # causally older handoff sample regress an established owner.
            return
        self.owner_state = value
        self.owner_epoch = sequence
        self.owner_stamp_s = float(now_s)
        if (
            self.phase == "awaiting_direct_xy"
            and value == "direct_xy"
            and sequence >= self.minimum_motion_epoch
        ):
            self.phase = "direct_xy"
        elif value == "aborting" and self.phase != "preauthorization":
            self.phase = "aborting"

    def runtime_blocking_reason(self, now_s: float) -> str | None:
        """Assert actual ownership after motion starts; never reuse ready."""
        if not self.enabled:
            return None
        if not self.inhibit_received:
            return "inhibit_state_missing"
        if self.inhibited:
            return "motion_inhibited"
        if self.phase == "awaiting_direct_xy":
            return "direct_xy_owner_pending"
        if self.phase == "aborting" or self.owner_state == "aborting":
            return "direct_xy_owner_aborting"
        if self.phase != "direct_xy" or self.owner_state != "direct_xy":
            return "direct_xy_owner_lost"
        if self.owner_epoch < self.minimum_motion_epoch:
            return "direct_xy_owner_epoch_stale"
        if self.owner_stamp_s is None:
            return "direct_xy_owner_state_missing"
        age_s = float(now_s) - self.owner_stamp_s
        if age_s < 0.0 or age_s > self.owner_timeout_s:
            return "direct_xy_owner_state_stale"
        return None


class DirectXyOwnerStateIngress(Node):
    """Cache the guardian owner level outside the joint-state executor.

    A flight preset node receives a continuously-ready joint-state stream.
    Sharing one ``spin_once`` queue allowed that stream to consume the whole
    150 ms owner-entry window even though the guardian published a valid
    direct_xy ACK after only a few milliseconds.  This single-subscription
    node runs on its own executor and exposes only the newest replaceable
    owner level; the command thread remains the sole mutator of the gate.
    """

    def __init__(self, topic: str) -> None:
        super().__init__("arm_direct_xy_owner_ingress")
        self._cache_lock = threading.Lock()
        self._latest: tuple[float, str] | None = None
        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self.create_subscription(String, str(topic), self._cache_owner, qos)

    def _cache_owner(self, message: String) -> None:
        snapshot = (time.monotonic(), str(message.data))
        with self._cache_lock:
            self._latest = snapshot

    def take_latest(self) -> tuple[float, str] | None:
        with self._cache_lock:
            snapshot = self._latest
            self._latest = None
        return snapshot


class ArmPresetCommander(Node):
    def __init__(self) -> None:
        super().__init__("arm_preset_control")
        self.publisher = self.create_publisher(
            JointTrajectory, "/arm_controller/joint_trajectory", 10
        )
        self.latest_positions: dict[str, float] = {}
        self.motion_publisher = self.create_publisher(
            Bool, "/my_drone/arm_motion_active", 10
        )
        self.direct_xy_gate = DirectXyMotionGate(
            _environment_flag("ARM_DIRECT_XY_OWNERSHIP"),
            ready_hold_s=float(
                os.environ.get("ARM_DIRECT_XY_COMMAND_READY_HOLD_S", "0.10")
            ),
            ready_timeout_s=float(
                os.environ.get("ARM_DIRECT_XY_COMMAND_READY_TIMEOUT_S", "0.15")
            ),
            owner_timeout_s=float(
                os.environ.get("ARM_DIRECT_XY_COMMAND_OWNER_TIMEOUT_S", "0.15")
            ),
        )
        latest_safety_level_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self.direct_xy_owner_entry_timeout_s = max(
            0.02,
            float(os.environ.get("ARM_DIRECT_XY_OWNER_ENTRY_TIMEOUT_S", "0.15")),
        )
        self.motion_command_active = False
        self.motion_abort_reason: str | None = None
        self._last_direct_xy_guardian_abort_key: tuple | None = None
        self.direct_xy_owner_ingress: DirectXyOwnerStateIngress | None = None
        self._direct_xy_owner_executor: SingleThreadedExecutor | None = None
        self._direct_xy_owner_executor_thread: threading.Thread | None = None
        self._direct_xy_owner_ingress_terminal_fault: str | None = None
        # These are state snapshots, not event journals.  Depth one prevents
        # a high-rate joint stream from building a callback backlog ahead of
        # the 20 Hz ownership watchdog in the single-threaded executor.
        self.create_subscription(JointState, "/joint_states", self.on_state, 1)
        self.create_subscription(
            Bool,
            "/my_drone/arm_direct_xy_ready",
            self.on_direct_xy_ready,
            1,
        )
        self.create_subscription(
            Bool,
            "/my_drone/arm_motion_inhibit",
            self.on_motion_inhibit,
            1,
        )
        if not _environment_flag("ARM_DIRECT_XY_EXTERNAL_GUARDIAN"):
            self.create_subscription(
                String,
                "/my_drone/arm_direct_xy_state",
                self.on_direct_xy_state,
                latest_safety_level_qos,
            )

    def start_direct_xy_owner_ingress(self) -> None:
        """Start the dedicated guardian-state path when external ownership is used.

        This is intentionally an explicit lifecycle operation rather than an
        implementation detail of the command-line ``main``.  Flight sequence
        and jog entry points also construct :class:`ArmPresetCommander`; if
        they miss this path, ready/inhibit can be healthy while the exact
        guardian owner acknowledgement is never consumed.
        """
        if (
            not self.direct_xy_gate.enabled
            or not _environment_flag("ARM_DIRECT_XY_EXTERNAL_GUARDIAN")
            or self.direct_xy_owner_ingress is not None
        ):
            return
        ingress = DirectXyOwnerStateIngress(
            "/my_drone/arm_direct_xy_guardian/state"
        )
        executor = SingleThreadedExecutor()
        executor.add_node(ingress)
        self.direct_xy_owner_ingress = ingress
        self._direct_xy_owner_executor = executor
        # The command loop drives this one-subscription executor explicitly.
        # A background Python thread can be starved by the trajectory loop/GIL
        # without producing a terminal error; the resulting cached state then
        # looks indistinguishable from a real 150 ms guardian outage.
        self._direct_xy_owner_executor_thread = None
        self._direct_xy_owner_ingress_terminal_fault = None

    def stop_direct_xy_owner_ingress(self) -> None:
        """Stop and detach the dedicated guardian-state path, idempotently."""
        executor = getattr(self, "_direct_xy_owner_executor", None)
        executor_thread = getattr(
            self, "_direct_xy_owner_executor_thread", None
        )
        ingress = getattr(self, "direct_xy_owner_ingress", None)
        if executor is not None:
            executor.shutdown(timeout_sec=1.0)
        if executor_thread is not None:
            executor_thread.join(timeout=1.0)
        if ingress is not None:
            ingress.destroy_node()
        self.direct_xy_owner_ingress = None
        self._direct_xy_owner_executor = None
        self._direct_xy_owner_executor_thread = None

    def spin_callbacks(
        self, timeout_sec: float = 0.02, *, drain_limit: int = 32
    ) -> None:
        """Wait once, then drain a bounded batch of already-ready callbacks.

        ``rclpy.spin_once`` executes one callback.  Calling it only once per
        50 ms trajectory poll lets a continuously-ready JointState callback
        delay the direct-owner watchdog.  Depth-one subscriptions plus this
        bounded zero-time drain keep the latest safety state observable while
        retaining a deterministic upper bound on work per control poll.
        """
        self._drive_direct_xy_owner_ingress()
        self._consume_direct_xy_owner_ingress()
        rclpy.spin_once(self, timeout_sec=max(0.0, float(timeout_sec)))
        for _ in range(max(0, int(drain_limit))):
            rclpy.spin_once(self, timeout_sec=0.0)
        # A direct_xy ACK can arrive while the bounded joint-state drain is
        # running.  Consume it at the same command-thread boundary before any
        # 150 ms entry decision is evaluated.
        self._drive_direct_xy_owner_ingress()
        self._consume_direct_xy_owner_ingress()

    def _set_direct_xy_owner_ingress_terminal_fault(self, detail: str) -> None:
        if getattr(self, "_direct_xy_owner_ingress_terminal_fault", None):
            return
        self._direct_xy_owner_ingress_terminal_fault = str(detail)
        self.get_logger().error(
            "ARM_DIRECT_XY_OWNER_INGRESS_TERMINAL_FAULT detail=" + str(detail)
        )
        if getattr(self, "motion_command_active", False):
            self.emergency_hold("direct_xy_owner_ingress_terminal_fault")

    def _direct_xy_owner_ingress_fault_reason(self) -> str | None:
        detail = getattr(
            self, "_direct_xy_owner_ingress_terminal_fault", None
        )
        if detail:
            return "direct_xy_owner_ingress_terminal_fault"
        # Kept as a diagnostic for older launchers/tests which may attach a
        # thread.  The normal command-driven path deliberately has no thread.
        executor_thread = getattr(
            self, "_direct_xy_owner_executor_thread", None
        )
        if executor_thread is not None and not executor_thread.is_alive():
            return "direct_xy_owner_ingress_thread_stopped"
        return None

    def _drive_direct_xy_owner_ingress(self) -> None:
        """Service one latest-level guardian callback without blocking."""
        executor = getattr(self, "_direct_xy_owner_executor", None)
        if executor is None:
            return
        thread_reason = self._direct_xy_owner_ingress_fault_reason()
        if thread_reason is not None:
            if thread_reason == "direct_xy_owner_ingress_thread_stopped":
                self._set_direct_xy_owner_ingress_terminal_fault(
                    "legacy_executor_thread_stopped"
                )
            return
        try:
            executor.spin_once(timeout_sec=0.0)
        except Exception as exc:  # executor failure must never look like stale DDS
            self._set_direct_xy_owner_ingress_terminal_fault(
                f"executor_exception:{type(exc).__name__}:{exc}"
            )

    def _consume_direct_xy_owner_ingress(self) -> None:
        ingress = getattr(self, "direct_xy_owner_ingress", None)
        if ingress is None:
            return
        snapshot = ingress.take_latest()
        if snapshot is None:
            return
        received_monotonic, data = snapshot
        self._apply_direct_xy_state_json(data, received_monotonic)

    def on_state(self, message: JointState) -> None:
        self.latest_positions.update(zip(message.name, message.position))

    def on_direct_xy_ready(self, message: Bool) -> None:
        self.direct_xy_gate.update_ready(bool(message.data), time.monotonic())

    def on_motion_inhibit(self, message: Bool) -> None:
        inhibited = bool(message.data)
        self.direct_xy_gate.update_inhibit(inhibited)
        if inhibited and self.motion_command_active:
            self.emergency_hold("direct_xy_motion_inhibited")

    def on_direct_xy_state(self, message: String) -> None:
        self._apply_direct_xy_state_json(message.data, time.monotonic())

    def _apply_direct_xy_state_json(
        self, data: str, received_monotonic: float
    ) -> None:
        try:
            report = json.loads(data)
            if report.get("schema") != "my_drone.arm-direct-xy-state.v1":
                return
            state = str(report["state"])
            epoch = int(report["ownership_epoch"])
        except (KeyError, TypeError, ValueError, json.JSONDecodeError):
            return
        if state == "aborting":
            try:
                intent_generation = int(report.get("intent_generation", -1))
                lease_generation = int(report.get("lease_generation", -1))
            except (TypeError, ValueError):
                intent_generation = -1
                lease_generation = -1
            abort_key = (
                str(report.get("controller_session_id", "")),
                epoch,
                intent_generation,
                lease_generation,
                str(report.get("watchdog_reason", "")),
                str(report.get("watchdog_detail", "")),
            )
            if abort_key != getattr(
                self, "_last_direct_xy_guardian_abort_key", None
            ):
                self._last_direct_xy_guardian_abort_key = abort_key
                self.get_logger().error(
                    "ARM_DIRECT_XY_GUARDIAN_ABORT "
                    + json.dumps(
                        {
                            "controller_session_id": abort_key[0],
                            "ownership_epoch": abort_key[1],
                            "intent_generation": abort_key[2],
                            "lease_generation": abort_key[3],
                            "watchdog_reason": abort_key[4],
                            "watchdog_detail": abort_key[5],
                        },
                        sort_keys=True,
                    )
                )
        self.direct_xy_gate.update_owner_state(
            state, epoch, float(received_monotonic)
        )
        if self.motion_command_active:
            reason = self.direct_xy_gate.runtime_blocking_reason(time.monotonic())
            if reason not in {None, "direct_xy_owner_pending"}:
                self.emergency_hold(reason)

    def wait_for_direct_xy_ready(self, timeout_s: float = 6.0) -> None:
        """Wait for a continuously healthy ownership hand-off before motion."""
        if not self.direct_xy_gate.enabled:
            return
        # Explicitly announce an idle arm session first.  This gives the DDS
        # owner a deterministic edge on which to clear any prior abort latch
        # and publish its non-inhibited state; waiting for an unsolicited
        # inhibit=False sample would otherwise deadlock the first command
        # after a clean startup.
        #
        # Discard any ready sample received before this idle announcement.
        # Otherwise the executor can consume that old grant and publish the
        # rising edge before DDS has processed motion=False; DDS then rejects
        # the edge with a one-sample inhibit pulse.  A post-announcement True
        # sample must establish a fresh continuous hold interval.
        self.direct_xy_gate.update_ready(False, time.monotonic())
        self.publish_motion_active(False)
        deadline = time.monotonic() + max(0.01, float(timeout_s))
        last_reason = "direct_xy_not_ready"
        while time.monotonic() < deadline:
            self.spin_callbacks(0.02)
            now_s = time.monotonic()
            reason = self.direct_xy_gate.preauthorization_blocking_reason(now_s)
            if reason is None:
                return
            last_reason = reason
            # Before a trajectory starts, inhibit is a level-triggered wait
            # condition rather than a terminal error.  A brief source-gap can
            # correctly revoke pre-authorisation; wait for it to clear and for
            # a brand-new continuous ready hold.  Once motion is active,
            # ``on_motion_inhibit`` still performs an immediate emergency hold.
        raise RuntimeError("ARM_DIRECT_XY_HANDSHAKE_TIMEOUT reason=" + last_reason)

    def clear_motion_abort_after_fresh_preauthorization(self) -> None:
        """Clear one stopped action only after the ordinary gate is healthy.

        This does not grant ownership and does not bypass any deadline.  It is
        used by an already-preflighted emergency return after the prior action
        published motion=False and observed the guardian's fresh physical
        ``px4_xy`` handback plus the normal ready/health/stability hold.
        """
        if self.motion_command_active:
            raise RuntimeError(
                "ARM_DIRECT_XY_RECOVERY_BLOCKED reason=motion_still_active"
            )
        reason = self.direct_xy_gate.preauthorization_blocking_reason(
            time.monotonic()
        )
        if reason is not None:
            raise RuntimeError(
                "ARM_DIRECT_XY_RECOVERY_BLOCKED reason=" + reason
            )
        self.motion_abort_reason = None

    def _assert_motion_permitted(self) -> None:
        if self.motion_abort_reason is not None:
            raise RuntimeError(
                "ARM_DIRECT_XY_MOTION_ABORTED reason=" + self.motion_abort_reason
            )
        ingress_reason = self._direct_xy_owner_ingress_fault_reason()
        if ingress_reason is not None:
            if self.motion_command_active:
                self.emergency_hold(ingress_reason)
            raise RuntimeError("ARM_DIRECT_XY_NOT_READY reason=" + ingress_reason)
        reason = (
            self.direct_xy_gate.runtime_blocking_reason(time.monotonic())
            if self.motion_command_active
            else self.direct_xy_gate.preauthorization_blocking_reason(
                time.monotonic()
            )
        )
        if reason is not None:
            if self.motion_command_active:
                self.emergency_hold(reason)
            raise RuntimeError("ARM_DIRECT_XY_NOT_READY reason=" + reason)

    def emergency_hold(self, reason: str) -> None:
        """Stop a running trajectory at the latest measured joint pose."""
        if self.motion_abort_reason is not None:
            return
        self.motion_abort_reason = str(reason)
        if all(name in self.latest_positions for name in JOINT_NAMES):
            message = JointTrajectory()
            message.joint_names = list(JOINT_NAMES)
            point = JointTrajectoryPoint()
            point.positions = [
                float(self.latest_positions[name]) for name in JOINT_NAMES
            ]
            point.velocities = [0.0] * len(JOINT_NAMES)
            point.accelerations = [0.0] * len(JOINT_NAMES)
            point.time_from_start = Duration(sec=0, nanosec=100_000_000)
            message.points = [point]
            self.publisher.publish(message)
        self.motion_command_active = False
        self.direct_xy_gate.end_motion()
        self.motion_publisher.publish(Bool(data=False))
        self.get_logger().error(
            "ARM_DIRECT_XY_EXECUTOR_ABORT reason=" + self.motion_abort_reason
        )

    def raise_if_motion_aborted(self) -> None:
        if self.motion_abort_reason is not None:
            raise RuntimeError(
                "ARM_DIRECT_XY_MOTION_ABORTED reason=" + self.motion_abort_reason
            )
        ingress_reason = self._direct_xy_owner_ingress_fault_reason()
        if ingress_reason is not None:
            if self.motion_command_active:
                self.emergency_hold(ingress_reason)
            raise RuntimeError(
                "ARM_DIRECT_XY_MOTION_ABORTED reason=" + ingress_reason
            )
        if self.motion_command_active and self.direct_xy_gate.enabled:
            reason = self.direct_xy_gate.runtime_blocking_reason(time.monotonic())
            if reason is not None:
                self.emergency_hold(reason)
                raise RuntimeError("ARM_DIRECT_XY_MOTION_ABORTED reason=" + reason)

    def _wait_for_direct_xy_owner(self) -> None:
        deadline = time.monotonic() + self.direct_xy_owner_entry_timeout_s
        while time.monotonic() < deadline:
            self.spin_callbacks(0.01)
            if self.motion_abort_reason is not None:
                raise RuntimeError(
                    "ARM_DIRECT_XY_MOTION_ABORTED reason="
                    + self.motion_abort_reason
                )
            reason = self.direct_xy_gate.runtime_blocking_reason(time.monotonic())
            if reason is None:
                return
            if reason != "direct_xy_owner_pending":
                self.emergency_hold(reason)
                raise RuntimeError("ARM_DIRECT_XY_MOTION_ABORTED reason=" + reason)
        self.emergency_hold("direct_xy_owner_entry_timeout")
        raise RuntimeError(
            "ARM_DIRECT_XY_MOTION_ABORTED reason=direct_xy_owner_entry_timeout"
        )

    def send(self, positions: list[float], duration_s: float) -> None:
        if self.direct_xy_gate.enabled:
            if not self.motion_command_active:
                raise RuntimeError(
                    "ARM_DIRECT_XY_NOT_READY reason=motion_session_not_active"
                )
            self._assert_motion_permitted()
        message = JointTrajectory()
        message.joint_names = JOINT_NAMES
        # A single far-away endpoint leaves the Gazebo joint controller to
        # choose its own interpolation/initial velocity.  During flight that
        # can create a large first acceleration even when the endpoint time is
        # long.  Publish a sampled quintic (smoothstep) trajectory instead:
        # position, velocity and acceleration are all continuous and zero at
        # both ends, while the requested preset and total duration are kept
        # unchanged.  The first sample is the measured current pose whenever
        # available, so an interrupted command cannot inject a position jump.
        start = [
            float(self.latest_positions.get(name, 0.0))
            for name in JOINT_NAMES
        ]
        segment_count = max(4, int(round(float(duration_s))))
        points = []
        for index in range(segment_count + 1):
            tau = index / segment_count
            smooth = 10.0 * tau**3 - 15.0 * tau**4 + 6.0 * tau**5
            smooth_rate = (30.0 * tau**2 - 60.0 * tau**3 + 30.0 * tau**4) / duration_s
            smooth_accel = (
                60.0 * tau - 180.0 * tau**2 + 120.0 * tau**3
            ) / (duration_s * duration_s)
            point = JointTrajectoryPoint()
            point.positions = [
                current + smooth * (target - current)
                for current, target in zip(start, positions)
            ]
            point.velocities = [
                smooth_rate * (target - current)
                for current, target in zip(start, positions)
            ]
            point.accelerations = [
                smooth_accel * (target - current)
                for current, target in zip(start, positions)
            ]
            elapsed = float(duration_s) * tau
            seconds = int(elapsed)
            point.time_from_start = Duration(
                sec=seconds,
                nanosec=int(round((elapsed - seconds) * 1_000_000_000)),
            )
            points.append(point)
        message.points = points
        self.publisher.publish(message)

    def publish_motion_active(self, active: bool) -> None:
        if active and self.direct_xy_gate.enabled:
            if self.motion_command_active:
                self._assert_motion_permitted()
            else:
                self._assert_motion_permitted()
                self.direct_xy_gate.begin_motion(time.monotonic())
        message = Bool()
        message.data = bool(active)
        self.motion_publisher.publish(message)
        self.motion_command_active = bool(active)
        if active and self.direct_xy_gate.enabled:
            if self.direct_xy_gate.phase == "awaiting_direct_xy":
                self._wait_for_direct_xy_owner()
        elif not active:
            self.direct_xy_gate.end_motion()

    def maximum_error(self, target: list[float]) -> float | None:
        if any(name not in self.latest_positions for name in JOINT_NAMES):
            return None
        return max(
            abs(self.latest_positions[name] - expected)
            for name, expected in zip(JOINT_NAMES, target)
        )


def load_presets() -> dict[str, list[float]]:
    config = motion_reference_path()
    return json.loads(config.read_text(encoding="utf-8"))["presets"]


def motion_reference_path() -> Path:
    override = os.environ.get("SO101_MOTION_REFERENCE", "").strip()
    if override:
        return Path(override).expanduser().resolve()
    return (
        Path(get_package_share_directory("drone_arm_sim"))
        / "config"
        / "so101_motion_reference.json"
    )


def load_motion_reference() -> dict:
    config = motion_reference_path()
    return json.loads(config.read_text(encoding="utf-8"))


def kinematics_urdf_path() -> Path:
    override = os.environ.get("SO101_KINEMATICS_URDF", "").strip()
    if override:
        return Path(override).expanduser().resolve()
    return (
        Path(get_package_share_directory("drone_arm_sim"))
        / "urdf/my_drone_v3/my_drone_cad_debug_4kg.urdf"
    )


def flight_config_path() -> Path:
    override = os.environ.get("MY_DRONE_FLIGHT_CONFIG", "").strip()
    if override:
        return Path(override).expanduser().resolve()
    return (
        Path(get_package_share_directory("drone_arm_sim"))
        / "config/my_drone_v3_cad_debug_4kg.json"
    )


def main(args=None) -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--preset",
        choices=(
            "retracted",
            "work_a",
            "work_b",
            "flight_work_a",
            "flight_work_b",
            "demo_extended",
            "flight_micro_a",
            "flight_micro_b",
            "gripper_open",
            "gripper_closed",
            "wrist_roll_test",
            "wrist_roll_home",
            "shoulder_pan_slow_test",
            "shoulder_pan_home",
            "flight_straight_forward",
        ),
        required=True,
    )
    parser.add_argument("--duration", type=float, default=3.0)
    parser.add_argument("--wait", action="store_true")
    parser.add_argument("--tolerance", type=float, default=0.04)
    parser.add_argument(
        "--flight-preflight",
        action="store_true",
        help="Evaluate the complete trajectory against the selected flight envelope before publishing.",
    )
    parser.add_argument(
        "--allow-distance-scaling",
        action="store_true",
        help="Allow a non-retraction target to be shortened after all time-scaling attempts fail.",
    )
    parser.add_argument(
        "--preflight-only",
        action="store_true",
        help="Run flight preflight from the measured joint state but do not publish a trajectory.",
    )
    parsed = parser.parse_args(args)
    if parsed.duration <= 0.0:
        parser.error("--duration must be positive")
    if parsed.preflight_only and not parsed.flight_preflight:
        parser.error("--preflight-only requires --flight-preflight")

    reference = load_motion_reference()
    target = reference["presets"][parsed.preset]
    limits = {item["name"]: item for item in reference.get("joints", [])}
    for name, value in zip(JOINT_NAMES, target):
        limit = limits.get(name)
        if limit is None:
            raise RuntimeError(f"no safety limit is defined for joint {name}")
        if not float(limit["lower_rad"]) <= float(value) <= float(limit["upper_rad"]):
            raise RuntimeError(f"preset {parsed.preset} exceeds limit for joint {name}")
    # A cubic trajectory can peak above average speed.  Reserve half of the
    # configured joint velocity limit for a conservative ground/flight command.
    min_duration = max(
        abs(float(value) - float(reference["presets"]["retracted"][index]))
        / max(1e-6, 0.5 * float(limits[name]["velocity_rad_s"]))
        for index, (name, value) in enumerate(zip(JOINT_NAMES, target))
    )
    if parsed.duration < min_duration:
        parser.error(
            f"{parsed.preset} needs duration >= {min_duration:.3f}s for the 0.5x velocity safety limit"
        )
    rclpy.init()
    node = ArmPresetCommander()
    node.start_direct_xy_owner_ingress()
    try:
        connection_deadline = time.monotonic() + 10.0
        while node.publisher.get_subscription_count() == 0:
            if time.monotonic() >= connection_deadline:
                raise RuntimeError("arm_controller trajectory subscriber not found")
            node.spin_callbacks(0.1)
        # Every preset invocation creates a fresh node.  Waiting only for the
        # controller subscriber is insufficient: /joint_states may not have
        # arrived yet, so send() would assume an all-zero start.  That is
        # harmless for the first extension from home but turns a later retract
        # into an instantaneous jump from the extended pose to zero.  Require
        # the real six-joint start state before constructing the trajectory.
        state_deadline = time.monotonic() + 5.0
        while any(name not in node.latest_positions for name in JOINT_NAMES):
            if time.monotonic() >= state_deadline:
                missing = [
                    name for name in JOINT_NAMES
                    if name not in node.latest_positions
                ]
                raise RuntimeError(
                    "joint state unavailable before trajectory start: "
                    + ", ".join(missing)
                )
            node.spin_callbacks(0.1)
        if parsed.flight_preflight:
            start = {
                name: float(node.latest_positions[name]) for name in JOINT_NAMES
            }
            requested_target = dict(zip(JOINT_NAMES, target, strict=True))
            preflight = TrajectoryPreflight(
                kinematics_urdf_path(),
                motion_reference_path(),
                flight_config_path(),
            )
            decision = preflight.adapt_quintic(
                start,
                requested_target,
                parsed.duration,
                # Retraction is a safety action and must always reach home.
                allow_distance_scaling=(
                    parsed.allow_distance_scaling and parsed.preset != "retracted"
                ),
            )
            if not decision["accepted"]:
                last = decision["attempts"][-1]["evaluation"]
                raise RuntimeError(
                    "ARM_TRAJECTORY_PREFLIGHT_REJECTED "
                    + json.dumps(
                        {
                            "preset": parsed.preset,
                            "attempt_count": len(decision["attempts"]),
                            "failure_counts": last["failure_counts"],
                        },
                        sort_keys=True,
                    )
                )
            selected = decision["selected"]
            distance_scale = float(selected["distance_scale"])
            parsed.duration = float(selected["effective_duration_s"])
            target = [
                start[name] + distance_scale * (requested_target[name] - start[name])
                for name in JOINT_NAMES
            ]
            node.get_logger().info(
                "ARM_TRAJECTORY_PREFLIGHT_ACCEPTED "
                + json.dumps(
                    {
                        "preset": parsed.preset,
                        "decision": decision["decision"],
                        "effective_duration_s": parsed.duration,
                        "distance_scale": distance_scale,
                        "maximum_gravity_torque_nm": selected["evaluation"][
                            "maximum_gravity_torque_nm"
                        ],
                        "maximum_reaction_force_n": selected["evaluation"][
                            "maximum_reaction_force_n"
                        ],
                        "maximum_reaction_torque_nm": selected["evaluation"][
                            "maximum_reaction_torque_nm"
                        ],
                        "minimum_overlay_delta_headroom_n": selected["evaluation"][
                            "minimum_overlay_delta_headroom_n"
                        ],
                    },
                    sort_keys=True,
                )
            )
            if parsed.preflight_only:
                node.get_logger().info("ARM_TRAJECTORY_PREFLIGHT_ONLY_COMPLETE")
                return
        node.wait_for_direct_xy_ready()
        node.publish_motion_active(True)
        node.send(target, parsed.duration)
        node.get_logger().info(
            f"Sent preset {parsed.preset} over /arm_controller/joint_trajectory"
        )
        if not parsed.wait and not node.direct_xy_gate.enabled:
            return
        deadline = time.monotonic() + parsed.duration + 5.0
        last_motion_heartbeat = 0.0
        while time.monotonic() < deadline:
            node.spin_callbacks(0.05)
            node.raise_if_motion_aborted()
            if time.monotonic() - last_motion_heartbeat >= 0.25:
                node.publish_motion_active(True)
                last_motion_heartbeat = time.monotonic()
            error = node.maximum_error(target)
            if error is not None and error <= parsed.tolerance:
                node.get_logger().info(
                    f"ARM_PRESET_REACHED preset={parsed.preset} max_error={error:.6f} rad"
                )
                return
        error = node.maximum_error(target)
        raise RuntimeError(
            f"preset {parsed.preset} was not reached; max_error={error} rad"
        )
    except RuntimeError as error:
        node.get_logger().error(str(error))
        sys.exit(1)
    finally:
        node.publish_motion_active(False)
        node.spin_callbacks(0.1)
        node.stop_direct_xy_owner_ingress()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
