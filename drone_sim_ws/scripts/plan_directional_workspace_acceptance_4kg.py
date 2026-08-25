#!/usr/bin/env python3
"""Build the ten-direction Base1 arm/PX4 dynamic acceptance plan.

The static workspace envelope is only a source of candidate poses.  Every
outward leg and its exact return-to-home leg is passed through the same
trajectory preflight used by the live arm controller.  The resulting JSON is
therefore an executable plan, not a claim that flight acceptance has passed.
"""

from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path
from typing import Mapping

import numpy as np

from drone_arm_sim.trajectory_preflight import (
    DEFAULT_DISTANCE_SCALES,
    TrajectoryPreflight,
    quintic_joint_samples,
)
from drone_arm_sim.workspace_envelope import _horizontal_direction


DIRECTION_ORDER = (
    "front",
    "rear",
    "left",
    "right",
    "up",
    "down",
    "front_left",
    "front_right",
    "rear_left",
    "rear_right",
)
HORIZONTAL_DIRECTIONS = frozenset(DIRECTION_ORDER) - {"up", "down"}
MIRROR_SOURCE = {
    "front_left": "front_right",
    "front_right": "front_left",
    "rear_left": "rear_right",
    "rear_right": "rear_left",
    "left": "right",
    "right": "left",
}


def _summary(selected: Mapping[str, object]) -> dict:
    evaluation = selected["evaluation"]
    return {
        "effective_duration_s": float(selected["effective_duration_s"]),
        "distance_scale": float(selected["distance_scale"]),
        "time_scale": float(selected["time_scale"]),
        "maximum_gravity_torque_nm": float(
            evaluation["maximum_gravity_torque_nm"]
        ),
        "maximum_reaction_force_n": float(evaluation["maximum_reaction_force_n"]),
        "maximum_reaction_torque_nm": float(
            evaluation["maximum_reaction_torque_nm"]
        ),
        "minimum_physical_motor_headroom_n": float(
            evaluation["minimum_physical_motor_headroom_n"]
        ),
        "minimum_overlay_delta_headroom_n": float(
            evaluation["minimum_overlay_delta_headroom_n"]
        ),
        "maximum_allocation_residual_norm": float(
            evaluation["maximum_allocation_residual_norm"]
        ),
    }


def _candidate_records(envelope: Mapping[str, object], direction: str):
    boundary = envelope["boundary_samples"]
    if direction == "up":
        candidates = boundary.get("highest_candidates")
        if candidates:
            return [
                (f"highest_candidates[{index}]", candidate)
                for index, candidate in enumerate(candidates)
            ]
        return [("highest_allowed", boundary["highest_allowed"])]
    if direction == "down":
        candidates = boundary.get("lowest_candidates")
        if candidates:
            return [
                (f"lowest_candidates[{index}]", candidate)
                for index, candidate in enumerate(candidates)
            ]
        return [("lowest_allowed", boundary["lowest_allowed"])]
    horizontal = boundary["maximum_radius_by_direction"]
    candidates = [(f"maximum_radius_by_direction.{direction}", horizontal[direction])]
    mirror = MIRROR_SOURCE.get(direction)
    if mirror is not None:
        mirrored = json.loads(json.dumps(horizontal[mirror]))
        mirrored["positions_rad"]["shoulder_pan"] *= -1.0
        candidates.append((f"mirrored_{mirror}", mirrored))
    return candidates


def _actual_target(
    names: tuple[str, ...],
    start: Mapping[str, float],
    requested: Mapping[str, float],
    distance_scale: float,
) -> dict[str, float]:
    return {
        name: float(start[name])
        + float(distance_scale) * (float(requested[name]) - float(start[name]))
        for name in names
    }


def _collision_free_distance_scales(
    preflight: TrajectoryPreflight,
    start: Mapping[str, float],
    target: Mapping[str, float],
    duration_s: float,
    sample_count: int,
) -> tuple[float, ...]:
    """Return full/shortened scales whose complete geometric path is clear."""
    result = []
    for distance_scale in (1.0, *DEFAULT_DISTANCE_SCALES):
        samples = quintic_joint_samples(
            preflight.names,
            start,
            target,
            duration_s,
            distance_scale=float(distance_scale),
            sample_count=sample_count,
        )
        if not any(
            preflight.collision_pairs_at(sample["positions"])
            for sample in samples
        ):
            result.append(float(distance_scale))
    return tuple(result)


def build_plan(
    preflight: TrajectoryPreflight,
    envelope: Mapping[str, object],
    *,
    requested_duration_s: float = 20.0,
    hold_s: float = 3.0,
    settle_s: float = 2.0,
    sample_count: int = 41,
    shoulder_pan_speed_cap_rad_s: float = 0.0,
) -> dict:
    """Return a complete, preflighted, home-separated direction sequence."""
    if requested_duration_s <= 0.0 or hold_s < 0.0 or settle_s < 0.0:
        raise ValueError("duration must be positive; hold/settle cannot be negative")
    names = tuple(preflight.names)
    home_values = preflight.reference["presets"]["retracted"]
    home = dict(zip(names, home_values, strict=True))
    summary = envelope["summary"]
    # Schema 2 classifies height relative to the documented retracted tool
    # endpoint, instead of the midpoint of the sampled extrema used by the
    # older schema.  Accept the legacy field only so old reports fail in a
    # controlled, reproducible way rather than silently changing direction.
    center_key = (
        "vertical_classification_reference_m"
        if "vertical_classification_reference_m" in summary
        else "vertical_classification_center_m"
    )
    center_z = float(summary[center_key])
    vertical_deadband = float(envelope["summary"]["vertical_classification_deadband_m"])
    legs = []
    for direction in DIRECTION_ORDER:
        selected_candidate = None
        candidate_failures = []
        for source, candidate in _candidate_records(envelope, direction):
            requested = {
                name: float(candidate["positions_rad"][name]) for name in names
            }
            # Directional translation acceptance must not quietly combine a
            # tool-orientation change or a gripper stroke with the large arm
            # move.  Those are independent user controls and have their own
            # tests.  Holding these two joints at the documented home values
            # removes an unnecessary simultaneous disturbance and makes each
            # direction leg a test of endpoint translation only.
            for fixed_joint in ("wrist_roll", "gripper"):
                if fixed_joint in requested:
                    requested[fixed_joint] = float(home[fixed_joint])
            candidate_duration_s = float(requested_duration_s)
            if shoulder_pan_speed_cap_rad_s > 0.0 and "shoulder_pan" in requested:
                # A zero-endpoint-derivative quintic reaches 1.875 times its
                # average speed.  Size the segment so the heavy arm's base-yaw
                # joint never exceeds the candidate-specific dynamic cap.
                pan_distance = abs(
                    float(requested["shoulder_pan"])
                    - float(home["shoulder_pan"])
                )
                candidate_duration_s = max(
                    candidate_duration_s,
                    1.875 * pan_distance / shoulder_pan_speed_cap_rad_s,
                )
            collision_free_scales = _collision_free_distance_scales(
                preflight,
                home,
                requested,
                candidate_duration_s,
                sample_count,
            )
            if not collision_free_scales:
                candidate_failures.append(
                    {
                        "source": source,
                        "failure_counts": {"continuous_collision_proxy": 1},
                    }
                )
                continue
            outward = preflight.adapt_quintic(
                home,
                requested,
                candidate_duration_s,
                allow_distance_scaling=True,
                distance_scales=tuple(
                    scale for scale in collision_free_scales if scale < 1.0
                ) or DEFAULT_DISTANCE_SCALES,
                sample_count=sample_count,
                try_full_distance=1.0 in collision_free_scales,
            )
            if not outward["accepted"]:
                candidate_failures.append(
                    {
                        "source": source,
                        "failure_counts": outward["attempts"][-1]["evaluation"][
                            "failure_counts"
                        ],
                    }
                )
                continue
            outward_selected = outward["selected"]
            target = _actual_target(
                names, home, requested, outward_selected["distance_scale"]
            )
            transform, _ = preflight.dynamics.model.forward_kinematics(
                "gripper_link", target
            )
            endpoint = np.asarray(transform[:3, 3], dtype=float)
            if direction in HORIZONTAL_DIRECTIONS:
                actual_direction = _horizontal_direction(endpoint)
                correct_direction = actual_direction == direction
            elif direction == "up":
                actual_direction = "up" if endpoint[2] > center_z + vertical_deadband else "not_up"
                correct_direction = actual_direction == "up"
            else:
                actual_direction = "down" if endpoint[2] < center_z - vertical_deadband else "not_down"
                correct_direction = actual_direction == "down"
            if not correct_direction:
                candidate_failures.append(
                    {"source": source, "failure_counts": {"direction_mismatch": 1}}
                )
                continue
            return_leg = preflight.adapt_quintic(
                target,
                home,
                candidate_duration_s,
                allow_distance_scaling=False,
                sample_count=sample_count,
            )
            if not return_leg["accepted"]:
                candidate_failures.append(
                    {
                        "source": source,
                        "failure_counts": return_leg["attempts"][-1]["evaluation"][
                            "failure_counts"
                        ],
                        "phase": "retract",
                    }
                )
                continue
            selected_candidate = {
                "direction": direction,
                "source": source,
                "endpoint_body_flu_m": endpoint.tolist(),
                "target_positions_rad": target,
                "outward": {
                    "decision": outward["decision"],
                    **_summary(outward_selected),
                },
                "return": {
                    "decision": return_leg["decision"],
                    **_summary(return_leg["selected"]),
                },
                "hold_s": float(hold_s),
                "settle_s": float(settle_s),
                "candidate_failures_before_selection": candidate_failures,
            }
            break
        if selected_candidate is None:
            raise RuntimeError(
                f"no complete outward-and-return trajectory for {direction}: "
                + json.dumps(candidate_failures, sort_keys=True)
            )
        legs.append(selected_candidate)
    total_motion_s = sum(
        leg["outward"]["effective_duration_s"]
        + leg["hold_s"]
        + leg["return"]["effective_duration_s"]
        + leg["settle_s"]
        for leg in legs
    )
    return {
        "schema": "my_drone.directional-workspace-flight-plan.v1",
        "status": "PREFLIGHTED_NOT_FLIGHT_ACCEPTED",
        "frame": "base_link ROS FLU (+X front, +Y left, +Z up)",
        "direction_order": list(DIRECTION_ORDER),
        "home_positions_rad": home,
        "requested_duration_s": float(requested_duration_s),
        "shoulder_pan_speed_cap_rad_s": float(shoulder_pan_speed_cap_rad_s),
        "total_motion_s": float(total_motion_s),
        "acceptance_limits": {
            "xy_peak_to_peak_m": 0.05,
            "altitude_peak_to_peak_m": 0.05,
            "maximum_tilt_deg": 1.0,
            "motor_saturation_samples": 0,
            "failsafe": False,
            "return_required_after_every_direction": True,
        },
        "legs": legs,
    }


def main() -> int:
    workspace = Path(__file__).resolve().parents[1]
    package = workspace / "src/drone_arm_sim"
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--envelope",
        type=Path,
        default=workspace / "analysis/base1/arm_workspace_envelope_4kg.json",
    )
    parser.add_argument(
        "--output",
        type=Path,
        default=workspace / "analysis/base1/directional_workspace_flight_plan_4kg.json",
    )
    parser.add_argument(
        "--urdf",
        type=Path,
        default=package / "urdf/my_drone_v3/my_drone_cad_debug_4kg.urdf",
    )
    parser.add_argument(
        "--motion-reference",
        type=Path,
        default=package / "config/so101_motion_reference_4kg.json",
    )
    parser.add_argument(
        "--flight-config",
        type=Path,
        default=package / "config/my_drone_v3_cad_debug_4kg.json",
    )
    parser.add_argument("--target-mass-kg", type=float, default=4.0)
    parser.add_argument("--gravity-torque-limit-nm", type=float, default=1.35)
    parser.add_argument("--maximum-motor-delta-n", type=float, default=1.60)
    parser.add_argument("--duration", type=float, default=20.0)
    parser.add_argument("--hold", type=float, default=3.0)
    parser.add_argument("--settle", type=float, default=2.0)
    parser.add_argument("--sample-count", type=int, default=41)
    parser.add_argument(
        "--shoulder-pan-speed-cap-rad-s",
        type=float,
        default=0.0,
        help="Optional quintic peak-speed cap for shoulder_pan; zero keeps the Base1 default.",
    )
    args = parser.parse_args()
    envelope = json.loads(args.envelope.read_text(encoding="utf-8"))
    preflight = TrajectoryPreflight(
        args.urdf,
        args.motion_reference,
        args.flight_config,
        target_mass_kg=args.target_mass_kg,
        gravity_torque_limit_nm=args.gravity_torque_limit_nm,
        maximum_motor_delta_n=args.maximum_motor_delta_n,
    )
    plan = build_plan(
        preflight,
        envelope,
        requested_duration_s=args.duration,
        hold_s=args.hold,
        settle_s=args.settle,
        sample_count=args.sample_count,
        shoulder_pan_speed_cap_rad_s=args.shoulder_pan_speed_cap_rad_s,
    )
    plan["source_evidence"] = {
        "envelope_sha256": hashlib.sha256(args.envelope.read_bytes()).hexdigest(),
        "model_sha256": dict(envelope.get("inputs", {}).get("sha256", {})),
        "implementation_sha256": {
            "planner": hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
            "trajectory_preflight": hashlib.sha256(
                (package / "drone_arm_sim/trajectory_preflight.py").read_bytes()
            ).hexdigest(),
            "workspace_envelope": hashlib.sha256(
                (package / "drone_arm_sim/workspace_envelope.py").read_bytes()
            ).hexdigest(),
        },
    }
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(
        json.dumps(plan, ensure_ascii=False, indent=2) + "\n", encoding="utf-8"
    )
    print(
        "DIRECTIONAL_WORKSPACE_PLAN_PASS "
        + json.dumps(
            {
                "directions": plan["direction_order"],
                "total_motion_s": plan["total_motion_s"],
                "output": str(args.output),
            },
            sort_keys=True,
        )
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
