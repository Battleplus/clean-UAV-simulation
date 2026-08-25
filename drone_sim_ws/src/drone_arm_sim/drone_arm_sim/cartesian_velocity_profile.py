"""ROS-independent jerk-limited Cartesian velocity profile."""

from __future__ import annotations

import numpy as np


COMMAND_DIRECTIONS = {
    "forward": np.asarray([1.0, 0.0, 0.0]),
    "back": np.asarray([-1.0, 0.0, 0.0]),
    "left": np.asarray([0.0, 1.0, 0.0]),
    "right": np.asarray([0.0, -1.0, 0.0]),
    "up": np.asarray([0.0, 0.0, 1.0]),
    "down": np.asarray([0.0, 0.0, -1.0]),
}


def joint_velocity_sample_is_stable(
    velocities: dict[str, float],
    joint_names,
    *,
    threshold_rad_s: float = 0.01,
) -> bool:
    """Require a complete, finite measured velocity sample below the limit."""
    names = tuple(joint_names)
    return bool(
        all(name in velocities for name in names)
        and all(np.isfinite(float(velocities[name])) for name in names)
        and max(abs(float(velocities[name])) for name in names)
        <= float(threshold_rad_s)
    )


def _limit_vector_norm(vector: np.ndarray, maximum_norm: float) -> np.ndarray:
    """Return ``vector`` with a Euclidean, rather than per-axis, limit."""
    value = np.asarray(vector, dtype=float)
    limit = max(0.0, float(maximum_norm))
    norm = float(np.linalg.norm(value))
    if norm > limit > 0.0:
        return value * (limit / norm)
    if limit == 0.0:
        return np.zeros_like(value)
    return value.copy()


class JerkLimitedVelocity3D:
    """Latch a 3-D velocity target and approach it with vector a/j limits."""

    def __init__(
        self,
        maximum_speed_m_s: float,
        maximum_acceleration_m_s2: float,
        maximum_jerk_m_s3: float,
    ) -> None:
        self.maximum_speed = max(0.0, float(maximum_speed_m_s))
        self.maximum_acceleration = max(0.0, float(maximum_acceleration_m_s2))
        self.maximum_jerk = max(0.0, float(maximum_jerk_m_s3))
        self.target = np.zeros(3)
        self.velocity = np.zeros(3)
        self.acceleration = np.zeros(3)

    def set_direction(self, direction: np.ndarray | None) -> None:
        if direction is None:
            self.target[:] = 0.0
            return
        vector = np.asarray(direction, dtype=float)
        if vector.shape != (3,) or not np.all(np.isfinite(vector)):
            raise ValueError("velocity direction must be a finite 3-vector")
        norm = float(np.linalg.norm(vector))
        if norm <= 1.0e-12:
            self.target[:] = 0.0
        else:
            self.target = self.maximum_speed * vector / norm

    def step(self, dt_s: float) -> np.ndarray:
        try:
            raw_dt = float(dt_s)
        except (TypeError, ValueError):
            raw_dt = 0.0
        dt = max(0.0, min(raw_dt, 0.2)) if np.isfinite(raw_dt) else 0.0
        if dt == 0.0:
            return self.velocity.copy()
        previous_acceleration = self.acceleration.copy()
        velocity_error = self.target - self.velocity
        error_norm = float(np.linalg.norm(velocity_error))
        if error_norm <= 1.0e-12:
            desired_acceleration = np.zeros(3)
        else:
            direction = velocity_error / error_norm
            closing_acceleration = max(
                0.0, float(np.dot(self.acceleration, direction))
            )
            braking_delta_velocity = (
                closing_acceleration * closing_acceleration
                / (2.0 * self.maximum_jerk)
                if self.maximum_jerk > 0.0
                else float("inf")
            )
            braking_margin = (
                2.0 * closing_acceleration * dt
                + self.maximum_jerk * dt * dt
            )
            desired_acceleration = (
                np.zeros(3)
                if error_norm <= braking_delta_velocity + braking_margin
                else direction * self.maximum_acceleration
            )
        acceleration_delta = _limit_vector_norm(
            desired_acceleration - self.acceleration,
            self.maximum_jerk * dt,
        )
        self.acceleration = _limit_vector_norm(
            self.acceleration + acceleration_delta,
            self.maximum_acceleration,
        )
        terminal_acceleration = velocity_error / dt
        if (
            np.linalg.norm(terminal_acceleration)
            <= self.maximum_acceleration + 1.0e-12
            and np.linalg.norm(terminal_acceleration - previous_acceleration)
            <= self.maximum_jerk * dt + 1.0e-12
            and np.linalg.norm(terminal_acceleration)
            <= self.maximum_jerk * dt + 1.0e-12
            and np.linalg.norm(previous_acceleration)
            <= self.maximum_jerk * dt + 1.0e-12
        ):
            self.velocity = self.target.copy()
            self.acceleration[:] = 0.0
        else:
            self.velocity = self.velocity + self.acceleration * dt
        return self.velocity.copy()

    def stopped(self) -> bool:
        return bool(
            np.linalg.norm(self.target) < 1.0e-7
            and np.linalg.norm(self.velocity) < 1.0e-5
            and np.linalg.norm(self.acceleration) < 1.0e-4
        )
