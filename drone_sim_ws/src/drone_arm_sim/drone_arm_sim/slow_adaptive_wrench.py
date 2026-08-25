"""Slow bounded *effort-residual* adaptation for Base1 arm flight.

The rigid-body model remains the primary feed-forward path.  This module only
integrates the small, quasi-static PX4 wrench-effort residual that is left
after the model has acted.  Position, velocity, attitude and angular-rate
errors deliberately do not enter :class:`SlowAdaptiveWrenchTrim`: those loops
already belong to PX4 and duplicating them here previously caused oscillation.

The trim is deliberately slow, motion-gated, rate-limited and
allocation-aware.  Its state survives action boundaries; only :meth:`reset`
clears a learned trim.
"""

from __future__ import annotations

import numpy as np


def _bound_norm(values: np.ndarray, maximum: float) -> np.ndarray:
    vector = np.asarray(values, dtype=float).copy()
    limit = max(0.0, float(maximum))
    norm = float(np.linalg.norm(vector))
    if not np.isfinite(norm) or limit == 0.0:
        return np.zeros_like(vector)
    if norm > limit:
        vector *= limit / norm
    return vector


def _deadband_scalar(value: float, width: float) -> float:
    """Return a continuous scalar dead-zone (no jump at the boundary)."""
    magnitude = abs(float(value))
    threshold = max(0.0, float(width))
    if magnitude <= threshold:
        return 0.0
    return float(np.copysign(magnitude - threshold, value))


def _deadband_norm(values: np.ndarray, width: float) -> np.ndarray:
    """Return a continuous radial dead-zone for a vector group."""
    vector = np.asarray(values, dtype=float).copy()
    threshold = max(0.0, float(width))
    magnitude = float(np.linalg.norm(vector))
    if not np.isfinite(magnitude) or magnitude <= threshold:
        return np.zeros_like(vector)
    return vector * ((magnitude - threshold) / magnitude)


def _slew_norm(current: np.ndarray, target: np.ndarray, maximum_delta: float) -> np.ndarray:
    """Move a vector toward ``target`` with a radial delta limit."""
    present = np.asarray(current, dtype=float)
    desired = np.asarray(target, dtype=float)
    delta = desired - present
    limit = max(0.0, float(maximum_delta))
    magnitude = float(np.linalg.norm(delta))
    if not np.isfinite(magnitude) or limit == 0.0:
        return present.copy()
    if magnitude > limit:
        delta *= limit / magnitude
    return present + delta


class SlowAdaptiveWrenchTrim:
    """Leaky-integrate a raw PX4 effort residual behind strict safety gates.

    The update law, after deadband, is::

        d(trim)/dt = residual / time_constant - trim / leak_time_constant

    ``eligible`` must mean that the arm/vehicle state is quasi-static.  When
    the gate drops, the learned trim is retained (rather than being silently
    cleared between actions) and warm-up starts over.  If the allocator cannot
    deliver the request, learning freezes and an optional delivered wrench is
    back-calculated into the internal state to prevent wind-up.
    """

    def __init__(
        self,
        *,
        time_constant_s: float = 15.0,
        leak_time_constant_s: float | None = None,
        decay_time_constant_s: float | None = None,
        warmup_s: float = 3.0,
        force_deadband_n: float = 0.02,
        torque_deadband_nm: float = 0.005,
        horizontal_force_limit_n: float = 0.06,
        vertical_force_limit_n: float = 0.04,
        torque_limit_nm: float = 0.02,
        horizontal_force_rate_n_s: float = 0.005,
        vertical_force_rate_n_s: float = 0.003,
        torque_rate_nm_s: float = 0.002,
    ) -> None:
        # A time constant below three seconds can follow the measured Base1
        # attitude oscillation and is therefore refused by construction.
        self.time_constant_s = max(3.0, float(time_constant_s))
        # ``decay_time_constant_s`` is accepted as a compatibility alias.  It
        # used to decay state whenever eligibility dropped; it now controls
        # only the intentional leaky-integrator term while eligible.
        if leak_time_constant_s is None:
            leak_time_constant_s = (
                60.0 if decay_time_constant_s is None else decay_time_constant_s
            )
        self.leak_time_constant_s = max(1.0, float(leak_time_constant_s))
        self.decay_time_constant_s = self.leak_time_constant_s
        self.warmup_s = max(0.0, float(warmup_s))
        self.force_deadband_n = max(0.0, float(force_deadband_n))
        self.torque_deadband_nm = max(0.0, float(torque_deadband_nm))
        self.horizontal_force_limit_n = max(0.0, float(horizontal_force_limit_n))
        self.vertical_force_limit_n = max(0.0, float(vertical_force_limit_n))
        self.torque_limit_nm = max(0.0, float(torque_limit_nm))
        self.horizontal_force_rate_n_s = max(0.0, float(horizontal_force_rate_n_s))
        self.vertical_force_rate_n_s = max(0.0, float(vertical_force_rate_n_s))
        self.torque_rate_nm_s = max(0.0, float(torque_rate_nm_s))
        self.value = np.zeros(6)
        self.eligible_duration_s = 0.0

    def reset(self) -> None:
        self.value[:] = 0.0
        self.eligible_duration_s = 0.0

    def _bounded(self, candidate: np.ndarray) -> np.ndarray:
        values = np.asarray(candidate, dtype=float).copy()
        if values.shape != (6,) or not np.all(np.isfinite(values)):
            return np.zeros(6)
        values[:2] = _bound_norm(values[:2], self.horizontal_force_limit_n)
        values[2] = float(np.clip(values[2], -self.vertical_force_limit_n, self.vertical_force_limit_n))
        values[3:] = _bound_norm(values[3:], self.torque_limit_nm)
        return values

    def _deadbanded(self, residual: np.ndarray) -> np.ndarray:
        values = np.asarray(residual, dtype=float).copy()
        values[:2] = _deadband_norm(values[:2], self.force_deadband_n)
        values[2] = _deadband_scalar(values[2], self.force_deadband_n)
        values[3:] = _deadband_norm(values[3:], self.torque_deadband_nm)
        return values

    def _rate_limited(self, current: np.ndarray, target: np.ndarray, dt_s: float) -> np.ndarray:
        dt = max(0.0, float(dt_s))
        result = np.asarray(current, dtype=float).copy()
        desired = np.asarray(target, dtype=float)
        result[:2] = _slew_norm(
            result[:2], desired[:2], self.horizontal_force_rate_n_s * dt
        )
        result[2] += float(
            np.clip(
                desired[2] - result[2],
                -self.vertical_force_rate_n_s * dt,
                self.vertical_force_rate_n_s * dt,
            )
        )
        result[3:] = _slew_norm(
            result[3:], desired[3:], self.torque_rate_nm_s * dt
        )
        return result

    def back_calculate(self, delivered_wrench_frd: np.ndarray) -> np.ndarray:
        """Synchronise state to the allocator's actually delivered overlay.

        Back-calculation is intentionally immediate: while allocation-limited,
        the physical output has already changed and retaining a larger hidden
        integrator state would create wind-up and a later discontinuity.
        """
        try:
            delivered = np.asarray(delivered_wrench_frd, dtype=float)
        except (TypeError, ValueError):
            return self.value.copy()
        if delivered.shape != (6,) or not np.all(np.isfinite(delivered)):
            return self.value.copy()
        self.value = self._bounded(delivered)
        return self.value.copy()

    def step(
        self,
        candidate_wrench_frd: np.ndarray,
        dt_s: float,
        *,
        eligible: bool,
        allocation_limited: bool = False,
        delivered_wrench_frd: np.ndarray | None = None,
        quasi_static: bool = True,
    ) -> np.ndarray:
        """Advance the trim from a raw PX4 wrench-effort residual.

        ``candidate_wrench_frd`` retains the historical argument name for API
        compatibility; it is now an effort residual, not a pose-controller
        output.  ``delivered_wrench_frd`` is the last overlay actually realised
        by allocation and is used for anti-windup back-calculation.
        """
        try:
            raw_dt = float(dt_s)
        except (TypeError, ValueError):
            raw_dt = 0.0
        dt = 0.0 if not np.isfinite(raw_dt) else max(0.0, min(raw_dt, 0.1))

        if delivered_wrench_frd is not None:
            self.back_calculate(delivered_wrench_frd)

        try:
            candidate = np.asarray(candidate_wrench_frd, dtype=float)
        except (TypeError, ValueError):
            candidate = np.empty(0)
        valid = candidate.shape == (6,) and np.all(np.isfinite(candidate))
        if not eligible or not quasi_static or not valid:
            self.eligible_duration_s = 0.0
            # Preserve calibration across action boundaries.  Clearing it is
            # an explicit operator/supervisor decision via reset().
            return self.value.copy()

        self.eligible_duration_s += dt
        if self.eligible_duration_s + 1.0e-12 < self.warmup_s:
            return self.value.copy()
        if allocation_limited:
            # Never integrate against a motor limit.  If the allocator passed
            # its delivered result above, state has already been back-calculated.
            return self.value.copy()

        residual = self._deadbanded(candidate)
        derivative = (
            residual / self.time_constant_s
            - self.value / self.leak_time_constant_s
        )
        target = self._bounded(self.value + derivative * dt)
        self.value = self._bounded(self._rate_limited(self.value, target, dt))
        return self.value.copy()
