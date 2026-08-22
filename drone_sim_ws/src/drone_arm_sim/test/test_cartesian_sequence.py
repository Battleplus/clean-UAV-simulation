import math
import time
from types import SimpleNamespace

import numpy as np

from drone_arm_sim.cartesian_arm_demo import ARM_JOINTS, _trajectory_points
from drone_arm_sim.cartesian_arm_sequence import CartesianSequenceNode


def test_velocity_filter_rejects_one_sample_spike_without_changing_limit():
    node = object.__new__(CartesianSequenceNode)
    node.filtered_velocity = [0.02, 0.01, 0.01]
    node.last_velocity_filter_monotonic = time.monotonic() - 0.02
    node.last_local_monotonic = 0.0

    node._local_cb(SimpleNamespace(vx=0.02, vy=0.01, vz=0.24))

    assert math.hypot(*node.filtered_velocity[:2]) < 0.10
    assert abs(node.filtered_velocity[2]) < 0.08
    assert node.local.vz == 0.24


def test_cartesian_demo_supplies_quintic_boundary_conditions():
    outward = [
        {name: 0.0 for name in ARM_JOINTS},
        {name: 0.05 for name in ARM_JOINTS},
        {name: 0.10 for name in ARM_JOINTS},
    ]
    points = _trajectory_points(outward, gripper=0.2, out_duration_s=90.0, hold_s=8.0)

    assert len(points) == 6
    assert all(len(point.velocities) == 6 for point in points)
    assert all(len(point.accelerations) == 6 for point in points)
    for index in (0, 2, 3, 5):
        assert np.allclose(points[index].velocities, 0.0)
        assert np.allclose(points[index].accelerations, 0.0)

    times = [
        point.time_from_start.sec + point.time_from_start.nanosec * 1.0e-9
        for point in points
    ]
    assert all(math.isfinite(value) for value in times)
    assert all(later > earlier for earlier, later in zip(times, times[1:]))
    assert all(np.all(np.isfinite(point.positions)) for point in points)
    assert all(np.all(np.isfinite(point.velocities)) for point in points)
    assert all(np.all(np.isfinite(point.accelerations)) for point in points)


def test_ninety_second_demo_reduces_joint_derivatives_by_time_scaling():
    outward = [
        {name: 0.00 for name in ARM_JOINTS},
        {name: 0.04 for name in ARM_JOINTS},
        {name: 0.10 for name in ARM_JOINTS},
        {name: 0.15 for name in ARM_JOINTS},
    ]
    fast = _trajectory_points(outward, gripper=0.2, out_duration_s=30.0, hold_s=8.0)
    slow = _trajectory_points(outward, gripper=0.2, out_duration_s=90.0, hold_s=8.0)

    fast_velocity = max(np.max(np.abs(point.velocities)) for point in fast)
    slow_velocity = max(np.max(np.abs(point.velocities)) for point in slow)
    fast_acceleration = max(np.max(np.abs(point.accelerations)) for point in fast)
    slow_acceleration = max(np.max(np.abs(point.accelerations)) for point in slow)

    assert slow_velocity <= fast_velocity / 3.0 + 1.0e-12
    assert slow_acceleration <= fast_acceleration / 9.0 + 1.0e-12
