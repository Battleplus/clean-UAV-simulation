"""Relay Gazebo sensor protobufs after an explicit simulation-time latency."""

from __future__ import annotations

import argparse
from collections import deque
from dataclasses import dataclass
import math
import threading

try:
    from gz.transport13 import Node as GzNode
    from gz.msgs10.fluid_pressure_pb2 import FluidPressure
    from gz.msgs10.imu_pb2 import IMU
    from gz.msgs10.magnetometer_pb2 import Magnetometer
    from gz.msgs10.navsat_pb2 import NavSat
    import rclpy
    from rclpy.node import Node
    from rosgraph_msgs.msg import Clock
except ModuleNotFoundError:
    GzNode = None
    FluidPressure = IMU = Magnetometer = NavSat = None
    rclpy = None
    Node = object
    Clock = None


BASE_TOPIC = "/world/flight_world/model/my_drone/link/base_link/sensor"


@dataclass(frozen=True)
class RelaySpec:
    name: str
    message_type: object
    raw_topic: str
    output_topic: str
    delay_s: float


def message_stamp_seconds(message, fallback_s: float = 0.0) -> float:
    try:
        stamp = message.header.stamp
        return float(stamp.sec) + 1e-9 * float(stamp.nsec)
    except (AttributeError, TypeError, ValueError):
        return float(fallback_s)


class SensorDelayRelay(Node):
    def __init__(self, delays: dict[str, float]):
        super().__init__("my_drone_gazebo_sensor_delay")
        self.gz_node = GzNode()
        self.simulation_time_s = 0.0
        self.lock = threading.Lock()
        definitions = [
            ("imu", IMU, "/my_drone/raw/imu", f"{BASE_TOPIC}/imu_sensor/imu"),
            ("magnetometer", Magnetometer, "/my_drone/raw/magnetometer", f"{BASE_TOPIC}/magnetometer_sensor/magnetometer"),
            ("air_pressure", FluidPressure, "/my_drone/raw/air_pressure", f"{BASE_TOPIC}/air_pressure_sensor/air_pressure"),
            ("navsat", NavSat, "/my_drone/raw/navsat", f"{BASE_TOPIC}/navsat_sensor/navsat"),
        ]
        self.specs = [
            RelaySpec(name, msg_type, raw, output, max(0.0, delays[name]))
            for name, msg_type, raw, output in definitions
        ]
        self.queues = {spec.name: deque(maxlen=5000) for spec in self.specs}
        self.zero_delay_pending = {spec.name: None for spec in self.specs}
        self.last_published_source_time = {spec.name: -math.inf for spec in self.specs}
        self.last_published_clock_time = {spec.name: -math.inf for spec in self.specs}
        self.gz_publishers = {
            spec.name: self.gz_node.advertise(spec.output_topic, spec.message_type)
            for spec in self.specs
        }
        self.callbacks = []
        for spec in self.specs:
            callback = self._make_callback(spec)
            self.callbacks.append(callback)
            if not self.gz_node.subscribe(spec.message_type, spec.raw_topic, callback):
                raise RuntimeError(f"Cannot subscribe Gazebo topic {spec.raw_topic}")
            self.get_logger().info(
                f"sensor delay {spec.name}: {spec.delay_s * 1000.0:.1f} ms"
            )
        self.create_subscription(Clock, "/clock", self.on_clock, 50)

    def _make_callback(self, spec: RelaySpec):
        def receive(message):
            if rclpy is None or not rclpy.ok():
                return
            copied = spec.message_type()
            copied.CopyFrom(message)
            # PX4's Gazebo IMU bridge timestamps a sample with
            # hrt_absolute_time() when this publication is received; it does
            # not consume the protobuf header stamp.  Releasing multiple
            # queued IMU messages from one /clock callback consequently gives
            # them the same PX4 timestamp and vehicle_imu rejects the second
            # sample.  For a zero-delay stream, retain only the newest source
            # sample until the next /clock callback.  on_clock publishes at
            # most one sample per simulation tick, preserving the 250 Hz
            # source bandwidth without ever releasing a same-tick batch.
            if spec.delay_s <= 0.0:
                source_time = message_stamp_seconds(message, self.simulation_time_s)
                with self.lock:
                    pending = self.zero_delay_pending[spec.name]
                    if pending is None or source_time > pending[0]:
                        self.zero_delay_pending[spec.name] = (source_time, copied)
                return
            with self.lock:
                source_time = message_stamp_seconds(message, self.simulation_time_s)
                self.queues[spec.name].append((source_time + spec.delay_s, copied))
        return receive

    def on_clock(self, message) -> None:
        if rclpy is None or not rclpy.ok():
            return
        now = float(message.clock.sec) + 1e-9 * float(message.clock.nanosec)
        due = []
        with self.lock:
            self.simulation_time_s = now
            for spec in self.specs:
                if spec.delay_s <= 0.0:
                    pending = self.zero_delay_pending[spec.name]
                    if (
                        pending is not None
                        and pending[0] > self.last_published_source_time[spec.name]
                        and (
                            spec.name != "imu"
                            or now - self.last_published_clock_time[spec.name] >= 0.0035
                        )
                    ):
                        source_time, sensor_message = pending
                        due.append((spec.name, sensor_message))
                        self.last_published_source_time[spec.name] = source_time
                        self.last_published_clock_time[spec.name] = now
                        self.zero_delay_pending[spec.name] = None
                    continue
                queue = self.queues[spec.name]
                newest_due = None
                while queue and queue[0][0] <= now:
                    newest_due = queue.popleft()[1]
                if newest_due is not None:
                    due.append((spec.name, newest_due))
        for name, sensor_message in due:
            if rclpy is None or not rclpy.ok():
                return
            try:
                self.gz_publishers[name].publish(sensor_message)
            except Exception as exc:  # Gazebo callbacks can race ROS shutdown.
                if rclpy is None or not rclpy.ok() or "context is invalid" in str(exc).lower():
                    return
                raise


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--imu-delay-ms", type=float, default=4.0)
    parser.add_argument("--mag-delay-ms", type=float, default=10.0)
    parser.add_argument("--baro-delay-ms", type=float, default=20.0)
    parser.add_argument("--navsat-delay-ms", type=float, default=50.0)
    parsed, ros_arguments = parser.parse_known_args()
    if rclpy is None or GzNode is None:
        raise SystemExit("ROS 2 and Gazebo Transport Python packages are required")
    delays = {
        "imu": parsed.imu_delay_ms * 0.001,
        "magnetometer": parsed.mag_delay_ms * 0.001,
        "air_pressure": parsed.baro_delay_ms * 0.001,
        "navsat": parsed.navsat_delay_ms * 0.001,
    }
    rclpy.init(args=ros_arguments)
    node = SensorDelayRelay(delays)
    try:
        rclpy.spin(node)
    except Exception as exc:
        # Normal launch shutdown raises ExternalShutdownException on some
        # Jazzy/rclpy combinations and returns normally on others.
        if "shutdown" not in str(exc).lower() and "context is invalid" not in str(exc).lower():
            raise
    finally:
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()


if __name__ == "__main__":
    main()
