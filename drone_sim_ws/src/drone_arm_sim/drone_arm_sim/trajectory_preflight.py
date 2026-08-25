"""Predict arm/rotor feasibility before publishing a joint trajectory."""

from __future__ import annotations

import json
import math
from pathlib import Path
from typing import Iterable, Mapping, Sequence

import numpy as np

from drone_arm_sim.base1_wrench_reallocator import (
    allocate_total_wrench,
    compensation_wrench_frd,
)
from drone_arm_sim.coupled_dynamics import CoupledArmDynamics
from drone_arm_sim.workspace_envelope import (
    STRUCTURAL_PROXY_EXCLUSIONS,
    _adjacent_link_pairs,
    _box_collision_proxies,
    _hover_baseline,
    _rotor_swept_volume_proxies,
    calibrated_proxy_exclusions,
    collision_pairs,
)


DEFAULT_TIME_SCALES = (1.0, 1.5, 2.0, 3.0, 4.0, 6.0, 8.0, 12.0)
DEFAULT_DISTANCE_SCALES = (0.9, 0.8, 0.7, 0.6, 0.5)


def quintic_joint_samples(
    names: Sequence[str],
    start: Mapping[str, float],
    target: Mapping[str, float],
    duration_s: float,
    *,
    distance_scale: float = 1.0,
    sample_count: int = 41,
) -> list[dict]:
    """Return position through jerk samples of one minimum-jerk segment."""
    duration = float(duration_s)
    if not math.isfinite(duration) or duration <= 0.0:
        raise ValueError("duration_s must be finite and positive")
    scale = float(distance_scale)
    if not 0.0 < scale <= 1.0:
        raise ValueError("distance_scale must be in (0, 1]")
    count = max(5, int(sample_count))
    q0 = np.asarray([float(start[name]) for name in names], dtype=float)
    requested = np.asarray([float(target[name]) for name in names], dtype=float)
    delta = scale * (requested - q0)
    result = []
    for tau in np.linspace(0.0, 1.0, count):
        position_shape = 10.0 * tau**3 - 15.0 * tau**4 + 6.0 * tau**5
        velocity_shape = 30.0 * tau**2 - 60.0 * tau**3 + 30.0 * tau**4
        acceleration_shape = 60.0 * tau - 180.0 * tau**2 + 120.0 * tau**3
        jerk_shape = 60.0 - 360.0 * tau + 360.0 * tau**2
        q = q0 + position_shape * delta
        qd = velocity_shape * delta / duration
        qdd = acceleration_shape * delta / (duration * duration)
        qddd = jerk_shape * delta / (duration * duration * duration)
        result.append(
            {
                "time_s": float(tau * duration),
                "positions": dict(zip(names, q.tolist(), strict=True)),
                "velocities": dict(zip(names, qd.tolist(), strict=True)),
                "accelerations": dict(zip(names, qdd.tolist(), strict=True)),
                "jerks": dict(zip(names, qddd.tolist(), strict=True)),
            }
        )
    return result


class TrajectoryPreflight:
    """Evaluate and adapt one trajectory against the selected flight envelope."""

    def __init__(
        self,
        urdf: Path,
        motion_reference: Path,
        flight_config: Path,
        *,
        target_mass_kg: float | None = None,
        gravity_torque_limit_nm: float | None = None,
        reaction_force_limit_n: float = 1.0,
        reaction_torque_limit_nm: float = 0.10,
        maximum_motor_delta_n: float | None = None,
        minimum_motor_headroom_n: float | None = None,
        minimum_delta_headroom_n: float | None = None,
        allocation_residual_limit: float = 1.0e-6,
        force_slew_rate_n_s: float = 1.0,
        torque_slew_rate_nm_s: float = 0.10,
        maximum_joint_acceleration_rad_s2: float = 4.0,
        maximum_joint_jerk_rad_s3: float = 20.0,
    ) -> None:
        self.reference = json.loads(Path(motion_reference).read_text(encoding="utf-8"))
        self.config = json.loads(Path(flight_config).read_text(encoding="utf-8"))
        configured_mass_kg = float(
            self.config.get(
                "estimated_all_up_mass_kg",
                self.config.get("temporary_fixed_mass_kg", 4.0),
            )
        )
        selected_mass_kg = (
            configured_mass_kg
            if target_mass_kg is None
            else float(target_mass_kg)
        )
        if not np.isfinite(selected_mass_kg) or selected_mass_kg <= 0.0:
            raise ValueError("flight preflight target mass must be finite and positive")
        candidate_limits = self.config.get("candidate_compensation_limits", {})
        if not isinstance(candidate_limits, Mapping):
            candidate_limits = {}
        self.dynamics = CoupledArmDynamics(
            Path(urdf), self.reference, target_mass_kg=selected_mass_kg
        )
        self.names = self.dynamics.active_joint_names
        self.gravity = float(self.config.get("gravity_m_s2", 9.80665))
        self.baseline = _hover_baseline(
            self.config, selected_mass_kg, self.gravity
        )
        urdf_boxes = _box_collision_proxies(Path(urdf))
        self.boxes = urdf_boxes + _rotor_swept_volume_proxies(
            Path(urdf), clearance_m=0.005
        )
        self.ignored_collision_pairs = _adjacent_link_pairs(
            self.dynamics.model
        ) | set(STRUCTURAL_PROXY_EXCLUSIONS)
        retracted_values = self.reference.get("presets", {}).get("retracted")
        self.retracted_anchor_collision_overrides: set[tuple[str, str]] = set()
        if isinstance(retracted_values, list) and len(retracted_values) == len(self.names):
            retracted = dict(zip(self.names, retracted_values, strict=True))
            self.retracted_anchor_collision_overrides = (
                calibrated_proxy_exclusions(self.dynamics, urdf_boxes, (retracted,))
                - self.ignored_collision_pairs
            )
        # The CAD-valid folded anchor has one known coarse-box overlap.  Keep
        # that calibration local to its small folded neighbourhood; never leak it
        # into the rest of the workspace or into rotor swept-volume checks.
        self.retracted_override_tolerance_rad = 0.20
        self.target_mass_kg = selected_mass_kg
        self.gravity_torque_limit_nm = float(
            candidate_limits.get("gravity_torque_limit_nm", 1.35)
            if gravity_torque_limit_nm is None
            else gravity_torque_limit_nm
        )
        self.reaction_force_limit_n = float(reaction_force_limit_n)
        self.reaction_torque_limit_nm = float(reaction_torque_limit_nm)
        self.maximum_motor_delta_n = float(
            candidate_limits.get("maximum_motor_delta_n", 1.60)
            if maximum_motor_delta_n is None
            else maximum_motor_delta_n
        )
        self.minimum_motor_headroom_n = float(
            candidate_limits.get("minimum_motor_headroom_n", 0.25)
            if minimum_motor_headroom_n is None
            else minimum_motor_headroom_n
        )
        self.minimum_delta_headroom_n = float(
            candidate_limits.get("minimum_delta_headroom_n", 0.05)
            if minimum_delta_headroom_n is None
            else minimum_delta_headroom_n
        )
        self.allocation_residual_limit = float(allocation_residual_limit)
        self.force_slew_rate_n_s = float(force_slew_rate_n_s)
        self.torque_slew_rate_nm_s = float(torque_slew_rate_nm_s)
        self.maximum_joint_acceleration_rad_s2 = float(
            maximum_joint_acceleration_rad_s2
        )
        self.maximum_joint_jerk_rad_s3 = float(maximum_joint_jerk_rad_s3)
        self.joint_velocity_limits_rad_s = {
            str(item["name"]): float(item.get("velocity_rad_s", float("inf")))
            for item in self.reference.get("joints", [])
            if isinstance(item, dict) and "name" in item
        }

    def is_retraction(
        self,
        start: Mapping[str, float],
        target: Mapping[str, float],
        *,
        tolerance_rad: float = 1.0e-6,
    ) -> bool:
        """Return true only for a monotonic joint-space move toward safe home.

        A safety retraction must never be shortened: doing so can leave the arm
        at the very extended pose that made the requested path difficult.  The
        test is deliberately conservative.  Every joint must get no farther
        from the documented retracted pose and the total distance must shrink.
        """
        home = np.asarray(
            [float(self.dynamics.home_positions[name]) for name in self.names],
            dtype=float,
        )
        q0 = np.asarray([float(start[name]) for name in self.names], dtype=float)
        q1 = np.asarray([float(target[name]) for name in self.names], dtype=float)
        if not np.all(np.isfinite(q0)) or not np.all(np.isfinite(q1)):
            return False
        start_error = np.abs(q0 - home)
        target_error = np.abs(q1 - home)
        tolerance = max(0.0, float(tolerance_rad))
        return bool(
            np.linalg.norm(target_error) < np.linalg.norm(start_error) - tolerance
            and np.all(target_error <= start_error + tolerance)
        )

    def collision_pairs_at(self, positions: Mapping[str, float]):
        """Return the exact proxy pairs used by preflight for one pose.

        This lightweight public check lets higher-level planners discard an
        unsafe kinematic branch before running every time/distance adaptation
        attempt.  The folded-anchor override is intentionally identical to
        :meth:`evaluate`; it does not weaken collision policy.
        """
        sample_ignored_pairs = self.ignored_collision_pairs
        if all(
            abs(float(positions[name]) - float(self.dynamics.home_positions[name]))
            <= self.retracted_override_tolerance_rad
            for name in self.names
        ):
            sample_ignored_pairs = (
                sample_ignored_pairs | self.retracted_anchor_collision_overrides
            )
        return collision_pairs(
            self.dynamics,
            self.boxes,
            positions,
            sample_ignored_pairs,
        )

    def evaluate(self, samples: Sequence[dict]) -> dict:
        if len(samples) < 2:
            raise ValueError("trajectory preflight needs at least two samples")
        previous_time = None
        previous_wrench = None
        sample_reports = []
        failure_counts: dict[str, int] = {}
        maximum = float(self.config["maximum_thrust_n"])
        base_thrust = np.asarray(self.baseline["thrust_config_order_n"], dtype=float)
        for index, sample in enumerate(samples):
            time_s = float(sample["time_s"])
            positions = sample["positions"]
            velocities = sample["velocities"]
            accelerations = sample["accelerations"]
            jerks = sample.get("jerks", {})
            reasons = []
            maximum_joint_velocity = 0.0
            maximum_joint_acceleration = 0.0
            maximum_joint_jerk = 0.0
            for name in self.names:
                lower, upper = self.dynamics.model.joint_limits(name)
                value = float(positions[name])
                velocity = abs(float(velocities.get(name, 0.0)))
                acceleration = abs(float(accelerations.get(name, 0.0)))
                jerk = abs(float(jerks.get(name, 0.0)))
                if not all(
                    math.isfinite(item)
                    for item in (value, velocity, acceleration, jerk)
                ):
                    reasons.append("non_finite_joint_state")
                    continue
                maximum_joint_velocity = max(maximum_joint_velocity, velocity)
                maximum_joint_acceleration = max(
                    maximum_joint_acceleration, acceleration
                )
                maximum_joint_jerk = max(maximum_joint_jerk, jerk)
                if value < lower - 1.0e-9 or value > upper + 1.0e-9:
                    reasons.append("joint_limit")
                velocity_limit = self.joint_velocity_limits_rad_s.get(
                    name, float("inf")
                )
                if velocity > velocity_limit + 1.0e-9:
                    reasons.append("joint_velocity_limit")
                if (
                    self.maximum_joint_acceleration_rad_s2 > 0.0
                    and acceleration
                    > self.maximum_joint_acceleration_rad_s2 + 1.0e-9
                ):
                    reasons.append("joint_acceleration_limit")
                if (
                    self.maximum_joint_jerk_rad_s3 > 0.0
                    and jerk > self.maximum_joint_jerk_rad_s3 + 1.0e-9
                ):
                    reasons.append("joint_jerk_limit")
            collisions = self.collision_pairs_at(positions)
            if collisions:
                reasons.append("collision_proxy")
            state = self.dynamics.state(positions, velocities, accelerations)
            gravity_force = np.asarray([0.0, 0.0, -state.mass_kg * self.gravity])
            gravity_torque = np.cross(state.com_shift_m, gravity_force)
            gravity_norm = float(np.linalg.norm(gravity_torque))
            reaction_force_norm = float(np.linalg.norm(state.reaction_force_body_n))
            reaction_torque_norm = float(np.linalg.norm(state.reaction_torque_body_nm))
            if gravity_norm > self.gravity_torque_limit_nm + 1.0e-9:
                reasons.append("gravity_torque_limit")
            if reaction_force_norm > self.reaction_force_limit_n + 1.0e-9:
                reasons.append("reaction_force_limit")
            if reaction_torque_norm > self.reaction_torque_limit_nm + 1.0e-9:
                reasons.append("reaction_torque_limit")
            reaction = np.concatenate(
                (state.reaction_force_body_n, state.reaction_torque_body_nm)
            )
            gravity_wrench = np.concatenate((np.zeros(3), gravity_torque))
            requested = compensation_wrench_frd(
                reaction,
                gravity_wrench,
                reaction_force_gain=1.0,
                reaction_torque_gain=1.0,
                gravity_torque_gain=1.0,
                force_limit_n=self.reaction_force_limit_n,
                reaction_torque_limit_nm=self.reaction_torque_limit_nm,
                gravity_torque_limit_nm=self.gravity_torque_limit_nm,
            )
            if previous_time is not None:
                dt = time_s - previous_time
                if dt <= 0.0:
                    reasons.append("non_monotonic_time")
                else:
                    rate = np.abs(requested - previous_wrench) / dt
                    if np.any(rate[:3] > self.force_slew_rate_n_s + 1.0e-9):
                        reasons.append("force_slew_rate")
                    if np.any(rate[3:] > self.torque_slew_rate_nm_s + 1.0e-9):
                        reasons.append("torque_slew_rate")
            previous_time = time_s
            previous_wrench = requested
            allocation = allocate_total_wrench(
                self.config,
                self.baseline["commands_motor_order"],
                requested,
                maximum_motor_delta_n=self.maximum_motor_delta_n,
            )
            thrust = np.asarray(allocation["thrust_config_order_n"], dtype=float)
            physical_headroom = float(min(np.min(thrust), np.min(maximum - thrust)))
            delta_headroom = float(
                self.maximum_motor_delta_n - np.max(np.abs(thrust - base_thrust))
            )
            residual = float(allocation["residual_norm"])
            if not allocation["success"] or residual > self.allocation_residual_limit:
                reasons.append("allocation_residual")
            if physical_headroom < self.minimum_motor_headroom_n:
                reasons.append("physical_motor_headroom")
            if delta_headroom < self.minimum_delta_headroom_n:
                reasons.append("overlay_delta_headroom")
            reasons = sorted(set(reasons))
            for reason in reasons:
                failure_counts[reason] = failure_counts.get(reason, 0) + 1
            sample_reports.append(
                {
                    "index": index,
                    "time_s": time_s,
                    "gravity_torque_norm_nm": gravity_norm,
                    "reaction_force_norm_n": reaction_force_norm,
                    "reaction_torque_norm_nm": reaction_torque_norm,
                    "maximum_joint_velocity_rad_s": maximum_joint_velocity,
                    "maximum_joint_acceleration_rad_s2": maximum_joint_acceleration,
                    "maximum_joint_jerk_rad_s3": maximum_joint_jerk,
                    "requested_compensation_wrench_frd": requested.tolist(),
                    "minimum_physical_motor_headroom_n": physical_headroom,
                    "remaining_overlay_delta_headroom_n": delta_headroom,
                    "allocation_residual_norm": residual,
                    "collision_proxy_pairs": [list(pair) for pair in collisions],
                    "accepted": not reasons,
                    "reasons": reasons,
                }
            )
        accepted = not failure_counts
        return {
            "accepted": accepted,
            "sample_count": len(samples),
            "failure_counts": failure_counts,
            "maximum_gravity_torque_nm": max(
                item["gravity_torque_norm_nm"] for item in sample_reports
            ),
            "maximum_reaction_force_n": max(
                item["reaction_force_norm_n"] for item in sample_reports
            ),
            "maximum_reaction_torque_nm": max(
                item["reaction_torque_norm_nm"] for item in sample_reports
            ),
            "maximum_joint_velocity_rad_s": max(
                item["maximum_joint_velocity_rad_s"] for item in sample_reports
            ),
            "maximum_joint_acceleration_rad_s2": max(
                item["maximum_joint_acceleration_rad_s2"]
                for item in sample_reports
            ),
            "maximum_joint_jerk_rad_s3": max(
                item["maximum_joint_jerk_rad_s3"] for item in sample_reports
            ),
            "minimum_physical_motor_headroom_n": min(
                item["minimum_physical_motor_headroom_n"] for item in sample_reports
            ),
            "minimum_overlay_delta_headroom_n": min(
                item["remaining_overlay_delta_headroom_n"] for item in sample_reports
            ),
            "maximum_allocation_residual_norm": max(
                item["allocation_residual_norm"] for item in sample_reports
            ),
            "first_failed_sample": next(
                (item for item in sample_reports if not item["accepted"]), None
            ),
            "samples": sample_reports,
        }

    def adapt_quintic(
        self,
        start: Mapping[str, float],
        target: Mapping[str, float],
        requested_duration_s: float,
        *,
        allow_distance_scaling: bool,
        time_scales: Iterable[float] = DEFAULT_TIME_SCALES,
        distance_scales: Iterable[float] = DEFAULT_DISTANCE_SCALES,
        sample_count: int = 41,
        try_full_distance: bool = True,
    ) -> dict:
        """Adapt a segment in the required safety order.

        The full requested distance is tried at progressively longer durations
        first.  Time scaling reduces peak velocity, acceleration and jerk by
        ``1/s``, ``1/s**2`` and ``1/s**3`` respectively.  Only after every
        full-distance attempt fails may a non-retraction target be shortened.
        A move monotonically toward the documented retracted pose always keeps
        distance scale 1.0, even if a caller accidentally enables shortening.
        """
        attempts = []
        is_retraction = self.is_retraction(start, target)
        distance_scaling_allowed = bool(allow_distance_scaling and not is_retraction)
        requested_time_scales = tuple(float(value) for value in time_scales)
        if any(
            not np.isfinite(value) or value < 1.0
            for value in requested_time_scales
        ):
            raise ValueError("time scales must be finite and >= 1")
        normalized_time_scales = tuple(sorted({1.0, *requested_time_scales}))
        requested_distance_scales = tuple(float(value) for value in distance_scales)
        if any(
            not np.isfinite(value) or not 0.0 < value < 1.0
            for value in requested_distance_scales
        ):
            raise ValueError("distance scales must be finite and in (0, 1)")
        normalized_distance_scales = tuple(
            sorted(set(requested_distance_scales), reverse=True)
        )
        if not try_full_distance and not distance_scaling_allowed:
            raise ValueError(
                "full-distance precheck may only be skipped when shortening is allowed"
            )

        def attempt(distance_scale: float, time_scale: float) -> dict:
            effective_duration = float(requested_duration_s) * float(time_scale)
            samples = quintic_joint_samples(
                self.names,
                start,
                target,
                effective_duration,
                distance_scale=distance_scale,
                sample_count=sample_count,
            )
            evaluation = self.evaluate(samples)
            record = {
                "phase": (
                    "full_distance"
                    if abs(float(distance_scale) - 1.0) <= 1.0e-12
                    else "shortened_distance"
                ),
                "distance_scale": float(distance_scale),
                "time_scale": float(time_scale),
                "effective_duration_s": effective_duration,
                "velocity_scale": 1.0 / float(time_scale),
                "acceleration_scale": 1.0 / (float(time_scale) ** 2),
                "jerk_scale": 1.0 / (float(time_scale) ** 3),
                "selected_target": dict(samples[-1]["positions"]),
                "evaluation": evaluation,
            }
            attempts.append(record)
            return record

        if try_full_distance:
            for time_scale in normalized_time_scales:
                record = attempt(1.0, time_scale)
                if record["evaluation"]["accepted"]:
                    return {
                        "accepted": True,
                        "decision": "accepted" if time_scale == 1.0 else "slowed",
                        "selected": record,
                        "attempts": attempts,
                        "is_retraction": is_retraction,
                        "distance_scaling_allowed": distance_scaling_allowed,
                    }
        if distance_scaling_allowed:
            for distance_scale in normalized_distance_scales:
                for time_scale in normalized_time_scales:
                    record = attempt(float(distance_scale), time_scale)
                    if record["evaluation"]["accepted"]:
                        return {
                            "accepted": True,
                            "decision": (
                                "shortened"
                                if time_scale == 1.0
                                else "shortened_and_slowed"
                            ),
                            "selected": record,
                            "attempts": attempts,
                            "is_retraction": is_retraction,
                            "distance_scaling_allowed": distance_scaling_allowed,
                        }
        return {
            "accepted": False,
            "decision": "rejected",
            "selected": None,
            "attempts": attempts,
            "is_retraction": is_retraction,
            "distance_scaling_allowed": distance_scaling_allowed,
        }
