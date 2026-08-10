import math
import time
from types import SimpleNamespace

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
