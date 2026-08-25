import math
import threading

from px4_msgs.msg import TrajectorySetpoint

from px4_ros2_control.dds_wasd_control import TrajectorySetpointKeepalive


class Recorder:
    def __init__(self):
        self.messages = []

    def publish(self, message):
        self.messages.append(message)


def make_setpoint(*, mixed_xy: bool, timestamp: int = 1_000_000):
    nan = float("nan")
    message = TrajectorySetpoint()
    message.timestamp = timestamp
    message.position = [nan, nan, nan] if mixed_xy else [1.0, 2.0, -3.0]
    message.velocity = [nan, nan, 0.1] if mixed_xy else [nan, nan, nan]
    message.acceleration = [0.0, 0.0, nan] if mixed_xy else [nan, nan, nan]
    message.jerk = [nan, nan, nan]
    message.yaw = 0.2
    message.yawspeed = nan
    return message


def test_keepalive_refreshes_wire_timestamp_without_changing_command():
    recorder = Recorder()
    clock = [10.0]
    pump = TrajectorySetpointKeepalive(
        recorder,
        active_callback=lambda: True,
        period_s=0.02,
        maximum_source_age_s=0.18,
        monotonic_callback=lambda: clock[0],
    )
    source = make_setpoint(mixed_xy=False)

    pump.publish(source)
    assert len(recorder.messages) == 1
    clock[0] = 10.021
    assert pump.pump_once()

    repeated = recorder.messages[-1]
    assert repeated is not source
    assert repeated.timestamp == 1_021_000
    assert list(repeated.position) == [1.0, 2.0, -3.0]
    assert all(math.isnan(value) for value in repeated.velocity)


def test_keepalive_stops_when_inactive_or_source_lease_expires():
    recorder = Recorder()
    active = [True]
    clock = [20.0]
    pump = TrajectorySetpointKeepalive(
        recorder,
        active_callback=lambda: active[0],
        period_s=0.02,
        maximum_source_age_s=0.18,
        monotonic_callback=lambda: clock[0],
    )
    pump.publish(make_setpoint(mixed_xy=False))

    active[0] = False
    clock[0] = 20.04
    assert not pump.pump_once()
    active[0] = True
    clock[0] = 20.181
    assert not pump.pump_once()
    assert len(recorder.messages) == 1


def test_new_finite_sample_atomically_replaces_mixed_sample_for_repeats():
    recorder = Recorder()
    repeated_mixed = []
    clock = [30.0]
    pump = TrajectorySetpointKeepalive(
        recorder,
        active_callback=lambda: True,
        mixed_xy_repeat_callback=lambda stamp: repeated_mixed.append(stamp),
        period_s=0.02,
        maximum_source_age_s=0.18,
        monotonic_callback=lambda: clock[0],
    )
    pump.publish(make_setpoint(mixed_xy=True, timestamp=2_000_000))
    clock[0] = 30.02
    assert pump.pump_once()
    assert repeated_mixed == [30.02]

    clock[0] = 30.03
    pump.publish(make_setpoint(mixed_xy=False, timestamp=3_000_000))
    clock[0] = 30.05
    assert pump.pump_once()
    repeated = recorder.messages[-1]
    assert list(repeated.position[:2]) == [1.0, 2.0]
    assert repeated_mixed == [30.02]


def test_authoritative_publish_and_repeat_share_one_wire_order_lock():
    class BlockingRecorder(Recorder):
        def __init__(self):
            super().__init__()
            self.first_repeat_entered = threading.Event()
            self.release_repeat = threading.Event()

        def publish(self, message):
            if int(message.timestamp) == 1_020_000:
                self.first_repeat_entered.set()
                self.release_repeat.wait(timeout=1.0)
            super().publish(message)

    recorder = BlockingRecorder()
    clock = [40.0]
    pump = TrajectorySetpointKeepalive(
        recorder,
        active_callback=lambda: True,
        period_s=0.02,
        monotonic_callback=lambda: clock[0],
    )
    pump.publish(make_setpoint(mixed_xy=True, timestamp=1_000_000))
    clock[0] = 40.02
    repeat_thread = threading.Thread(target=pump.pump_once)
    repeat_thread.start()
    assert recorder.first_repeat_entered.wait(timeout=1.0)

    clock[0] = 40.03
    finite_thread = threading.Thread(
        target=lambda: pump.publish(
            make_setpoint(mixed_xy=False, timestamp=2_000_000)
        )
    )
    finite_thread.start()
    recorder.release_repeat.set()
    repeat_thread.join(timeout=1.0)
    finite_thread.join(timeout=1.0)

    assert [int(message.timestamp) for message in recorder.messages] == [
        1_000_000,
        1_020_000,
        2_000_000,
    ]
    clock[0] = 40.05
    assert pump.pump_once()
    assert list(recorder.messages[-1].position[:2]) == [1.0, 2.0]


def test_keepalive_bridges_measured_132_ms_authoritative_source_gap():
    recorder = Recorder()
    clock = [50.0]
    pump = TrajectorySetpointKeepalive(
        recorder,
        active_callback=lambda: True,
        period_s=0.02,
        maximum_source_age_s=0.18,
        monotonic_callback=lambda: clock[0],
    )
    pump.publish(make_setpoint(mixed_xy=False, timestamp=10_000_000))
    for offset in (0.02, 0.04, 0.06, 0.08, 0.10, 0.12):
        clock[0] = 50.0 + offset
        assert pump.pump_once()
    # This is the observed worst-case source interval from the formal v3 run.
    clock[0] = 50.132
    pump.publish(make_setpoint(mixed_xy=False, timestamp=10_132_000))

    timestamps = [int(message.timestamp) for message in recorder.messages]
    gaps_s = [
        (newer - older) / 1_000_000.0
        for older, newer in zip(timestamps, timestamps[1:])
    ]
    assert max(gaps_s) <= 0.020001


def test_delayed_authoritative_sample_cannot_move_wire_timestamp_backwards():
    recorder = Recorder()
    clock = [60.0]
    wire_timestamp = [5_000_000]
    pump = TrajectorySetpointKeepalive(
        recorder,
        active_callback=lambda: True,
        period_s=0.02,
        monotonic_callback=lambda: clock[0],
        timestamp_callback=lambda: wire_timestamp[0],
    )
    pump.publish(make_setpoint(mixed_xy=True, timestamp=4_900_000))
    clock[0] = 60.02
    assert pump.pump_once()
    # This finite command was computed before it waited for the wire lock, so
    # its source timestamp is older than the repeat already on the wire.
    delayed_finite = make_setpoint(mixed_xy=False, timestamp=4_950_000)
    clock[0] = 60.021
    pump.publish(delayed_finite)

    timestamps = [int(message.timestamp) for message in recorder.messages]
    assert timestamps == [4_900_000, 5_000_000, 5_000_001]
    assert list(recorder.messages[-1].position[:2]) == [1.0, 2.0]
