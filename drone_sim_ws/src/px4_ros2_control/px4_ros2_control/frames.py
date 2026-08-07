"""Explicit PX4 NED/FRD to ROS ENU/FLU vector conversions."""

from __future__ import annotations

from collections.abc import Sequence


def ned_to_enu(vector: Sequence[float]) -> tuple[float, float, float]:
    """Convert a world vector [north, east, down] to [east, north, up]."""
    north, east, down = (float(value) for value in vector)
    return east, north, -down


def enu_to_ned(vector: Sequence[float]) -> tuple[float, float, float]:
    """Convert a world vector [east, north, up] to [north, east, down]."""
    east, north, up = (float(value) for value in vector)
    return north, east, -up


def frd_to_flu(vector: Sequence[float]) -> tuple[float, float, float]:
    """Convert a body vector [forward, right, down] to [forward, left, up]."""
    forward, right, down = (float(value) for value in vector)
    return forward, -right, -down


def flu_to_frd(vector: Sequence[float]) -> tuple[float, float, float]:
    """Convert a body vector [forward, left, up] to [forward, right, down]."""
    forward, left, up = (float(value) for value in vector)
    return forward, -left, -up
