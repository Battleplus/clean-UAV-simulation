#!/usr/bin/env python3
"""Execute one preflighted ten-direction arm sequence in a PX4 hover."""

from __future__ import annotations

import argparse
import hashlib
import json
import math
from pathlib import Path
import sys
import time

import rclpy
from px4_msgs.msg import VehicleLocalPosition
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy

from drone_arm_sim.arm_preset_control import (
    ArmPresetCommander,
    JOINT_NAMES,
    flight_config_path,
    kinematics_urdf_path,
    motion_reference_path,
)
from drone_arm_sim.trajectory_preflight import TrajectoryPreflight


EMERGENCY_REENTRY_TIMEOUT_S = 2.0


class DirectionalSequenceNode(ArmPresetCommander):
    def __init__(self) -> None:
        super().__init__()
        self.local: VehicleLocalPosition | None = None
        self.local_monotonic = 0.0
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self.create_subscription(
            VehicleLocalPosition,
            "/fmu/out/vehicle_local_position_v1",
            self._local_cb,
            qos,
        )

    def _local_cb(self, message: VehicleLocalPosition) -> None:
        self.local = message
        self.local_monotonic = time.monotonic()


def _wait_ready(node: DirectionalSequenceNode, timeout_s: float = 12.0) -> None:
    deadline = time.monotonic() + timeout_s
    while (
        node.publisher.get_subscription_count() == 0
        or any(name not in node.latest_positions for name in JOINT_NAMES)
        or node.local is None
    ):
        if time.monotonic() >= deadline:
            raise RuntimeError("arm controller, joint state, or PX4 local position unavailable")
        node.spin_callbacks(0.05)


def _wait_segment(
    node: DirectionalSequenceNode,
    target: dict[str, float],
    duration_s: float,
    *,
    tolerance_rad: float,
) -> None:
    start = time.monotonic()
    deadline = start + duration_s + 12.0
    stable_since = None
    last_heartbeat = 0.0
    while time.monotonic() < deadline:
        node.spin_callbacks(0.05)
        node.raise_if_motion_aborted()
        now = time.monotonic()
        if now - last_heartbeat >= 0.25:
            node.publish_motion_active(True)
            last_heartbeat = now
        error = node.maximum_error([target[name] for name in JOINT_NAMES])
        # Do not declare a trajectory complete merely because its first sample
        # equals the measured start pose.  Its timestamp must elapse and the
        # measured endpoint must then remain inside tolerance.
        if now >= start + duration_s and error is not None and error <= tolerance_rad:
            if stable_since is None:
                stable_since = now
            elif now - stable_since >= 0.5:
                return
        else:
            stable_since = None
    error = node.maximum_error([target[name] for name in JOINT_NAMES])
    raise RuntimeError(f"directional arm segment did not settle; max_error={error}")


def _hold(node: DirectionalSequenceNode, duration_s: float) -> None:
    """Keep one compensation session across extend, dwell, and retract."""
    node.publish_motion_active(True)
    deadline = time.monotonic() + duration_s
    while time.monotonic() < deadline:
        node.spin_callbacks(0.05)
        node.raise_if_motion_aborted()
        node.publish_motion_active(True)


def _wait_px4_stable(
    node: DirectionalSequenceNode,
    hold_s: float,
    timeout_s: float,
) -> None:
    stable_since = None
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        node.spin_callbacks(0.05)
        node.raise_if_motion_aborted()
        fresh = time.monotonic() - node.local_monotonic < 0.5
        stable = (
            fresh
            and node.local is not None
            and math.hypot(float(node.local.vx), float(node.local.vy)) < 0.10
            and abs(float(node.local.vz)) < 0.08
        )
        if stable:
            if stable_since is None:
                stable_since = time.monotonic()
            elif time.monotonic() - stable_since >= hold_s:
                return
        else:
            stable_since = None
        node.publish_motion_active(False)
    raise RuntimeError("PX4 did not restabilize after directional arm return")


def _attempt_preflighted_emergency_return(
    node: DirectionalSequenceNode,
    home: dict[str, float],
    return_duration_s: float,
    *,
    tolerance_rad: float,
    event,
) -> bool:
    """Re-enter through every normal gate and run the already-vetted return.

    A guardian abort has already frozen the trajectory.  First publish an
    explicit falling edge and wait until the guardian reports its physically
    acknowledged finite-PX4 owner and the existing stability/health gate is
    held.  Only then clear the stopped action and create a new ownership epoch.
    No trajectory planning is allowed in this time-critical recovery path.
    """
    node.publish_motion_active(False)
    try:
        # v7's extended PX4-only configuration began leaving its tight hold
        # in under four seconds.  Fail closed well before that point; this
        # shortens only the overall recovery wait and leaves every underlying
        # readiness/health/stability hold and watchdog unchanged.
        node.wait_for_direct_xy_ready(timeout_s=EMERGENCY_REENTRY_TIMEOUT_S)
        node.clear_motion_abort_after_fresh_preauthorization()
        node.publish_motion_active(True)
        node.send([home[name] for name in JOINT_NAMES], return_duration_s)
        _wait_segment(
            node,
            home,
            return_duration_s,
            tolerance_rad=tolerance_rad,
        )
        node.publish_motion_active(False)
        event("DIRECTIONAL_EMERGENCY_RETRACT_COMPLETE")
        return True
    except RuntimeError as recovery_error:
        # Fail closed at the latest measured pose.  In particular, do not
        # command LAND with the arm extended when the compensation owner could
        # not be safely re-established.
        node.publish_motion_active(False)
        event(
            "DIRECTIONAL_EMERGENCY_RETRACT_BLOCKED error="
            + str(recovery_error)
        )
        return False


def main(args=None) -> int:
    workspace = Path(__file__).resolve().parents[1]
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--plan",
        type=Path,
        default=workspace / "analysis/base1/directional_workspace_flight_plan_4kg.json",
    )
    parser.add_argument("--event-file", type=Path, required=True)
    parser.add_argument(
        "--directions",
        default="",
        help=(
            "optional comma-separated direction subset from the ten-leg plan; "
            "the default executes every direction"
        ),
    )
    parser.add_argument("--tolerance", type=float, default=0.06)
    parser.add_argument("--stability-timeout", type=float, default=35.0)
    parsed = parser.parse_args(args)
    plan = json.loads(parsed.plan.read_text(encoding="utf-8"))
    if plan.get("schema") != "my_drone.directional-workspace-flight-plan.v1":
        raise RuntimeError("unexpected directional flight-plan schema")
    if len(plan.get("legs", [])) != 10:
        raise RuntimeError("directional flight plan must contain ten legs")
    plan_directions = [str(leg["direction"]) for leg in plan["legs"]]
    requested_directions = [
        item.strip() for item in parsed.directions.split(",") if item.strip()
    ]
    if requested_directions:
        if len(set(requested_directions)) != len(requested_directions):
            raise RuntimeError("direction subset contains duplicates")
        unknown = [
            direction for direction in requested_directions
            if direction not in plan_directions
        ]
        if unknown:
            raise RuntimeError(
                "direction subset is not present in the plan: " + ",".join(unknown)
            )
        selected_directions = tuple(requested_directions)
    else:
        selected_directions = tuple(plan_directions)
    selected_legs = [
        leg for direction in selected_directions for leg in plan["legs"]
        if str(leg["direction"]) == direction
    ]
    evidence = plan.get("source_evidence", {})
    expected_hashes = evidence.get("model_sha256", {})
    live_inputs = {
        "urdf": kinematics_urdf_path(),
        "motion_reference": motion_reference_path(),
        "flight_config": flight_config_path(),
    }
    stale = {
        name: {
            "expected": expected_hashes.get(name),
            "actual": hashlib.sha256(Path(path).read_bytes()).hexdigest(),
        }
        for name, path in live_inputs.items()
        if expected_hashes.get(name)
        != hashlib.sha256(Path(path).read_bytes()).hexdigest()
    }
    if stale:
        raise RuntimeError(
            "directional plan is stale for the live model: "
            + json.dumps(stale, sort_keys=True)
        )
    expected_implementation_hashes = evidence.get("implementation_sha256", {})
    live_implementations = {
        "planner": workspace / "scripts/plan_directional_workspace_acceptance_4kg.py",
        "trajectory_preflight": workspace
        / "src/drone_arm_sim/drone_arm_sim/trajectory_preflight.py",
        "workspace_envelope": workspace
        / "src/drone_arm_sim/drone_arm_sim/workspace_envelope.py",
    }
    stale_implementations = {
        name: {
            "expected": expected_implementation_hashes.get(name),
            "actual": hashlib.sha256(path.read_bytes()).hexdigest(),
        }
        for name, path in live_implementations.items()
        if expected_implementation_hashes.get(name)
        != hashlib.sha256(path.read_bytes()).hexdigest()
    }
    if stale_implementations:
        raise RuntimeError(
            "directional plan is stale for the live preflight implementation: "
            + json.dumps(stale_implementations, sort_keys=True)
        )
    parsed.event_file.parent.mkdir(parents=True, exist_ok=True)
    parsed.event_file.write_text("", encoding="utf-8")

    def event(message: str) -> None:
        print(message, flush=True)
        with parsed.event_file.open("a", encoding="utf-8") as stream:
            stream.write(message + "\n")
            stream.flush()

    preflight = TrajectoryPreflight(
        kinematics_urdf_path(),
        motion_reference_path(),
        flight_config_path(),
    )
    home = {name: float(plan["home_positions_rad"][name]) for name in JOINT_NAMES}
    rclpy.init()
    node = DirectionalSequenceNode()
    node.start_direct_xy_owner_ingress()
    try:
        _wait_ready(node)
        for leg in selected_legs:
            direction = str(leg["direction"])
            target = {
                name: float(leg["target_positions_rad"][name]) for name in JOINT_NAMES
            }
            measured = {
                name: float(node.latest_positions[name]) for name in JOINT_NAMES
            }
            outward = preflight.adapt_quintic(
                measured,
                target,
                float(leg["outward"]["effective_duration_s"]),
                allow_distance_scaling=False,
            )
            if not outward["accepted"]:
                raise RuntimeError(
                    f"live outward preflight rejected {direction}: "
                    + json.dumps(
                        outward["attempts"][-1]["evaluation"]["failure_counts"],
                        sort_keys=True,
                    )
                )
            outward_duration = float(outward["selected"]["effective_duration_s"])

            # Complete both potentially expensive collision/dynamics sweeps
            # before accepting the short-lived direct-XY pre-authorisation.
            # The outward endpoint is required to settle at ``target`` within
            # the gate below, so it is the authoritative planned start for
            # the exact return preflight.  Running adapt_quintic after ready,
            # or while the motion heartbeat is active, can otherwise age the
            # ready sample and the one-second motion lease before any command
            # reaches the joint controller.
            return_leg = preflight.adapt_quintic(
                target,
                home,
                float(leg["return"]["effective_duration_s"]),
                allow_distance_scaling=False,
            )
            if not return_leg["accepted"]:
                raise RuntimeError(
                    f"live return preflight rejected {direction}: "
                    + json.dumps(
                        return_leg["attempts"][-1]["evaluation"]["failure_counts"],
                        sort_keys=True,
                    )
                )
            return_duration = float(return_leg["selected"]["effective_duration_s"])

            try:
                # Nothing computationally expensive is permitted between this
                # grant and the motion rising edge.
                node.wait_for_direct_xy_ready()
                extend_start = time.monotonic()
                node.publish_motion_active(True)
                node.send([target[name] for name in JOINT_NAMES], outward_duration)
                _wait_segment(
                    node, target, outward_duration, tolerance_rad=parsed.tolerance
                )
                extend_end = time.monotonic()
                event(
                    "DIRECTIONAL_STAGE_INTERVAL "
                    f"direction={direction} phase=extend "
                    f"start={extend_start:.6f} end={extend_end:.6f}"
                )

                hold_start = time.monotonic()
                _hold(node, float(leg["hold_s"]))
                hold_end = time.monotonic()
                event(
                    "DIRECTIONAL_STAGE_INTERVAL "
                    f"direction={direction} phase=hold "
                    f"start={hold_start:.6f} end={hold_end:.6f}"
                )

                retract_start = time.monotonic()
                node.publish_motion_active(True)
                node.send([home[name] for name in JOINT_NAMES], return_duration)
                _wait_segment(
                    node, home, return_duration, tolerance_rad=parsed.tolerance
                )
                retract_end = time.monotonic()
                event(
                    "DIRECTIONAL_STAGE_INTERVAL "
                    f"direction={direction} phase=retract "
                    f"start={retract_start:.6f} end={retract_end:.6f}"
                )
            except RuntimeError as motion_error:
                event(
                    "DIRECTIONAL_RUNTIME_ABORT direction="
                    f"{direction} error={motion_error}"
                )
                _attempt_preflighted_emergency_return(
                    node,
                    home,
                    return_duration,
                    tolerance_rad=parsed.tolerance,
                    event=event,
                )
                raise RuntimeError(
                    f"directional leg {direction} aborted: {motion_error}"
                ) from motion_error
            _wait_px4_stable(
                node,
                float(leg["settle_s"]),
                parsed.stability_timeout,
            )
            event(f"DIRECTIONAL_DIRECTION_COMPLETE direction={direction}")
        event(f"DIRECTIONAL_SEQUENCE_COMPLETE directions={len(selected_legs)}")
        return 0
    except RuntimeError as error:
        event(f"DIRECTIONAL_SEQUENCE_ERROR error={error}")
        node.get_logger().error(str(error))
        return 1
    finally:
        node.publish_motion_active(False)
        node.spin_callbacks(0.1)
        node.stop_direct_xy_owner_ingress()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    sys.exit(main())
