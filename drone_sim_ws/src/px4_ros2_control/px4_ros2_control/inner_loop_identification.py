#!/usr/bin/env python3
"""Bounded 4 kg PX4 attitude/body-rate identification flight.

This diagnostic owns the Offboard stream and must never run beside the WASD
controller.  It takes off in position mode, executes one small direct step at
a time, returns to position recovery after every step, and lands on completion
or on the first safety-gate violation.
"""

from __future__ import annotations

import argparse
import json
import math
import time
from pathlib import Path
from typing import Callable, Optional

import numpy as np
import rclpy
from px4_msgs.msg import (
    ActuatorOutputs,
    OffboardControlMode,
    TrajectorySetpoint,
    VehicleAttitudeSetpoint,
    VehicleCommand,
    VehicleLocalPosition,
    VehicleOdometry,
    VehicleRatesSetpoint,
    VehicleStatus,
)
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy


def euler_to_quaternion_wxyz(roll: float, pitch: float, yaw: float) -> list[float]:
    cr, sr = math.cos(roll * 0.5), math.sin(roll * 0.5)
    cp, sp = math.cos(pitch * 0.5), math.sin(pitch * 0.5)
    cy, sy = math.cos(yaw * 0.5), math.sin(yaw * 0.5)
    return [
        cr * cp * cy + sr * sp * sy,
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
    ]


def quaternion_wxyz_to_rpy(q) -> np.ndarray:
    values = np.asarray(q, dtype=float)
    values /= max(float(np.linalg.norm(values)), 1.0e-12)
    w, x, y, z = values
    return np.array(
        [
            math.atan2(2.0 * (w * x + y * z), 1.0 - 2.0 * (x * x + y * y)),
            math.asin(float(np.clip(2.0 * (w * y - z * x), -1.0, 1.0))),
            math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z)),
        ]
    )


class IdentificationAbort(RuntimeError):
    pass


class InnerLoopIdentification(Node):
    RATE_HZ = 20.0
    TAKEOFF_HEIGHT_M = 1.0
    HOVER_THRUST = 0.4811
    MAX_HORIZONTAL_ERROR_M = 0.75
    MAX_VERTICAL_ERROR_M = 0.40
    MAX_TILT_DEG = 8.0
    MOTOR_LOW = 5.0
    MOTOR_HIGH = 995.0
    # Direct-control mode changes are asynchronous inside PX4.  A 0.5 s
    # baseline was too short in V2: the ULog sometimes began the body-rate
    # segment after the zero baseline and the analyzer then mistook the return
    # to zero for the commanded step.  Keep a measured zero plateau before
    # every excitation and a long enough command plateau to observe rise and
    # settling without increasing the 8 degree safety envelope.
    DIRECT_BASELINE_S = 0.80
    ATTITUDE_STEP_S = 1.40
    ATTITUDE_ZERO_S = 0.60
    RATE_ROLL_PITCH_STEP_S = 1.20
    RATE_YAW_STEP_S = 1.50
    RATE_ZERO_S = 0.20

    def __init__(self, output: Path):
        super().__init__("my_drone_inner_loop_identification")
        self.output = output
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self.mode_pub = self.create_publisher(
            OffboardControlMode, "/fmu/in/offboard_control_mode", 10
        )
        self.trajectory_pub = self.create_publisher(
            TrajectorySetpoint, "/fmu/in/trajectory_setpoint", 10
        )
        self.attitude_pub = self.create_publisher(
            VehicleAttitudeSetpoint, "/fmu/in/vehicle_attitude_setpoint_v1", 10
        )
        self.rates_pub = self.create_publisher(
            VehicleRatesSetpoint, "/fmu/in/vehicle_rates_setpoint", 10
        )
        self.command_pub = self.create_publisher(
            VehicleCommand, "/fmu/in/vehicle_command", 10
        )
        self.create_subscription(
            VehicleStatus, "/fmu/out/vehicle_status_v4", self._status_cb, qos
        )
        self.create_subscription(
            VehicleLocalPosition,
            "/fmu/out/vehicle_local_position_v1",
            self._local_cb,
            qos,
        )
        self.create_subscription(
            VehicleOdometry, "/fmu/out/vehicle_odometry", self._odometry_cb, qos
        )
        self.create_subscription(
            ActuatorOutputs, "/fmu/out/actuator_outputs", self._actuator_cb, qos
        )
        self.status: Optional[VehicleStatus] = None
        self.local: Optional[VehicleLocalPosition] = None
        self.odometry: Optional[VehicleOdometry] = None
        self.actuator: Optional[ActuatorOutputs] = None
        self.last_status = 0.0
        self.last_local = 0.0
        self.run_started = time.monotonic()
        self.events: list[dict] = []
        self.samples: list[dict] = []
        self.result = "NOT_STARTED"
        self.abort_reason: Optional[str] = None
        self.ground_ned: Optional[np.ndarray] = None
        self.hover_ned: Optional[np.ndarray] = None
        self.hover_yaw = 0.0
        self.max_horizontal_error_m = 0.0
        self.max_vertical_error_m = 0.0
        self.max_tilt_deg = 0.0
        self.motor_saturation_samples = 0
        self.motor_samples = 0

    def now_us(self) -> int:
        return self.get_clock().now().nanoseconds // 1000

    def elapsed(self) -> float:
        return time.monotonic() - self.run_started

    def _status_cb(self, message: VehicleStatus) -> None:
        self.status = message
        self.last_status = time.monotonic()

    def _local_cb(self, message: VehicleLocalPosition) -> None:
        self.local = message
        self.last_local = time.monotonic()

    def _odometry_cb(self, message: VehicleOdometry) -> None:
        self.odometry = message

    def _actuator_cb(self, message: ActuatorOutputs) -> None:
        self.actuator = message

    def state_fresh(self) -> bool:
        now = time.monotonic()
        return (
            self.status is not None
            and self.local is not None
            and self.odometry is not None
            and now - self.last_status < 1.0
            and now - self.last_local < 1.0
            and bool(self.local.xy_valid)
            and bool(self.local.z_valid)
        )

    def publish_command(self, command: int, **parameters: float) -> None:
        message = VehicleCommand()
        message.timestamp = self.now_us()
        for index in range(1, 8):
            setattr(message, f"param{index}", float(parameters.get(f"param{index}", 0.0)))
        message.command = int(command)
        message.target_system = 1
        message.target_component = 1
        message.source_system = 1
        message.source_component = 1
        message.from_external = True
        self.command_pub.publish(message)

    def publish_position(self, target: np.ndarray, yaw: float) -> None:
        mode = OffboardControlMode()
        mode.timestamp = self.now_us()
        mode.position = True
        self.mode_pub.publish(mode)
        message = TrajectorySetpoint()
        message.timestamp = mode.timestamp
        nan = float("nan")
        message.position = np.asarray(target, dtype=float).tolist()
        message.velocity = [nan, nan, nan]
        message.acceleration = [nan, nan, nan]
        message.jerk = [nan, nan, nan]
        message.yaw = float(yaw)
        message.yawspeed = nan
        self.trajectory_pub.publish(message)

    def publish_attitude(self, roll: float, pitch: float, yaw: float) -> None:
        mode = OffboardControlMode()
        mode.timestamp = self.now_us()
        mode.attitude = True
        self.mode_pub.publish(mode)
        message = VehicleAttitudeSetpoint()
        message.timestamp = mode.timestamp
        message.q_d = euler_to_quaternion_wxyz(roll, pitch, yaw)
        message.thrust_body = [0.0, 0.0, -self.HOVER_THRUST]
        message.yaw_sp_move_rate = 0.0
        self.attitude_pub.publish(message)

    def publish_rates(self, roll: float, pitch: float, yaw: float) -> None:
        mode = OffboardControlMode()
        mode.timestamp = self.now_us()
        mode.body_rate = True
        self.mode_pub.publish(mode)
        message = VehicleRatesSetpoint()
        message.timestamp = mode.timestamp
        message.roll = float(roll)
        message.pitch = float(pitch)
        message.yaw = float(yaw)
        message.thrust_body = [0.0, 0.0, -self.HOVER_THRUST]
        message.reset_integral = False
        self.rates_pub.publish(message)

    def _record(self, phase: str, target: dict) -> None:
        if not self.state_fresh():
            return
        rpy = quaternion_wxyz_to_rpy(self.odometry.q)
        rates = np.asarray(self.odometry.angular_velocity, dtype=float)
        motors = []
        if self.actuator is not None:
            motors = [float(value) for value in self.actuator.output[: min(8, self.actuator.noutputs)]]
        self.samples.append(
            {
                "time_s": self.elapsed(),
                "phase": phase,
                "target": target,
                "position_ned_m": [float(self.local.x), float(self.local.y), float(self.local.z)],
                "velocity_ned_m_s": [float(self.local.vx), float(self.local.vy), float(self.local.vz)],
                "attitude_rpy_rad": rpy.tolist(),
                "body_rate_frd_rad_s": rates.tolist(),
                "motors": motors,
            }
        )

    def _check_safety(self, phase: str, direct_control: bool) -> None:
        if not self.state_fresh():
            raise IdentificationAbort("PX4 state became stale")
        if self.status.failsafe:
            raise IdentificationAbort("PX4 failsafe")
        if self.hover_ned is None:
            return
        position = np.array([self.local.x, self.local.y, self.local.z], dtype=float)
        horizontal = float(np.linalg.norm(position[:2] - self.hover_ned[:2]))
        vertical = abs(float(position[2] - self.hover_ned[2]))
        tilt = math.degrees(float(np.linalg.norm(quaternion_wxyz_to_rpy(self.odometry.q)[:2])))
        if direct_control:
            self.max_horizontal_error_m = max(self.max_horizontal_error_m, horizontal)
            self.max_vertical_error_m = max(self.max_vertical_error_m, vertical)
            self.max_tilt_deg = max(self.max_tilt_deg, tilt)
        if direct_control and horizontal > self.MAX_HORIZONTAL_ERROR_M:
            raise IdentificationAbort(f"horizontal gate {horizontal:.3f} m")
        if direct_control and vertical > self.MAX_VERTICAL_ERROR_M:
            raise IdentificationAbort(f"vertical gate {vertical:.3f} m")
        if tilt > self.MAX_TILT_DEG:
            raise IdentificationAbort(f"tilt gate {tilt:.3f} deg")
        if direct_control and self.actuator is not None:
            values = np.asarray(self.actuator.output[: min(8, self.actuator.noutputs)], dtype=float)
            if len(values) == 8 and np.all(np.isfinite(values)):
                self.motor_samples += 1
                if np.any(values <= self.MOTOR_LOW) or np.any(values >= self.MOTOR_HIGH):
                    self.motor_saturation_samples += 1
                    raise IdentificationAbort("motor saturation gate")

    def _run_for(
        self,
        duration_s: float,
        phase: str,
        target: dict,
        publish: Callable[[], None],
        *,
        direct_control: bool,
    ) -> None:
        self.events.append({"phase": phase, "event": "begin", "time_s": self.elapsed(), "target": target})
        end = time.monotonic() + duration_s
        period = 1.0 / self.RATE_HZ
        while rclpy.ok() and time.monotonic() < end:
            started = time.monotonic()
            rclpy.spin_once(self, timeout_sec=0.01)
            publish()
            self._record(phase, target)
            self._check_safety(phase, direct_control)
            time.sleep(max(0.0, period - (time.monotonic() - started)))
        self.events.append({"phase": phase, "event": "end", "time_s": self.elapsed(), "target": target})

    def _wait_stable(self, timeout_s: float, hold_s: float = 2.0) -> None:
        assert self.hover_ned is not None
        stable_since = None
        end = time.monotonic() + timeout_s
        while rclpy.ok() and time.monotonic() < end:
            self._run_for(
                0.05,
                "position_recovery",
                {"position_ned_m": self.hover_ned.tolist(), "yaw_rad": self.hover_yaw},
                lambda: self.publish_position(self.hover_ned, self.hover_yaw),
                direct_control=False,
            )
            position = np.array([self.local.x, self.local.y, self.local.z], dtype=float)
            velocity = np.array([self.local.vx, self.local.vy, self.local.vz], dtype=float)
            stable = (
                np.linalg.norm(position[:2] - self.hover_ned[:2]) <= 0.12
                and abs(position[2] - self.hover_ned[2]) <= 0.12
                and np.linalg.norm(velocity[:2]) <= 0.10
                and abs(velocity[2]) <= 0.08
                and math.degrees(np.linalg.norm(quaternion_wxyz_to_rpy(self.odometry.q)[:2])) <= 2.0
            )
            if stable:
                stable_since = stable_since or time.monotonic()
                if time.monotonic() - stable_since >= hold_s:
                    return
            else:
                stable_since = None
        raise IdentificationAbort("position recovery stability timeout")

    def _direct_attitude_step(self, name: str, roll: float, pitch: float, yaw: float) -> None:
        self._wait_stable(15.0, 1.5)
        self._run_for(
            self.DIRECT_BASELINE_S,
            f"attitude_{name}_baseline",
            {"rpy_rad": [0.0, 0.0, self.hover_yaw]},
            lambda: self.publish_attitude(0.0, 0.0, self.hover_yaw),
            direct_control=True,
        )
        self._run_for(
            self.ATTITUDE_STEP_S,
            f"attitude_{name}_step",
            {"rpy_rad": [roll, pitch, yaw]},
            lambda: self.publish_attitude(roll, pitch, yaw),
            direct_control=True,
        )
        self._run_for(
            self.ATTITUDE_ZERO_S,
            f"attitude_{name}_zero",
            {"rpy_rad": [0.0, 0.0, self.hover_yaw]},
            lambda: self.publish_attitude(0.0, 0.0, self.hover_yaw),
            direct_control=True,
        )

    def _direct_rate_step(
        self,
        name: str,
        roll: float,
        pitch: float,
        yaw: float,
        *,
        step_s: float,
    ) -> None:
        self._wait_stable(15.0, 1.5)
        self._run_for(
            self.DIRECT_BASELINE_S,
            f"rate_{name}_baseline",
            {"rates_rad_s": [0.0, 0.0, 0.0]},
            lambda: self.publish_rates(0.0, 0.0, 0.0),
            direct_control=True,
        )
        self._run_for(
            step_s,
            f"rate_{name}_step",
            {"rates_rad_s": [roll, pitch, yaw]},
            lambda: self.publish_rates(roll, pitch, yaw),
            direct_control=True,
        )
        self._run_for(
            self.RATE_ZERO_S,
            f"rate_{name}_zero",
            {"rates_rad_s": [0.0, 0.0, 0.0]},
            lambda: self.publish_rates(0.0, 0.0, 0.0),
            direct_control=True,
        )

    def _wait_initial_state(self) -> None:
        end = time.monotonic() + 45.0
        while rclpy.ok() and time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=0.05)
            if (
                self.state_fresh()
                and self.status.pre_flight_checks_pass
                and not self.status.failsafe
            ):
                return
        raise IdentificationAbort("PX4 preflight state unavailable")

    def _land(self) -> None:
        if self.status is None:
            return
        self.publish_command(VehicleCommand.VEHICLE_CMD_NAV_LAND)
        end = time.monotonic() + 30.0
        ground_stable_since = None
        while rclpy.ok() and time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=0.05)
            if self.status and self.status.arming_state == VehicleStatus.ARMING_STATE_DISARMED:
                self.events.append({"phase": "land", "event": "disarmed", "time_s": self.elapsed()})
                return
            # The canted 4 kg debug mixer can remain armed after native LAND.
            # Permit the PX4 force token only after the vehicle is physically
            # back at the recorded ground height with low 3-D velocity.  This
            # mirrors the manual debug controller and is never enabled for the
            # formal 7.735 kg profile.
            if self.local is not None and self.ground_ned is not None:
                on_ground = (
                    abs(float(self.local.z) - float(self.ground_ned[2])) <= 0.07
                    and math.hypot(float(self.local.vx), float(self.local.vy)) < 0.20
                    and abs(float(self.local.vz)) < 0.15
                )
                if on_ground:
                    ground_stable_since = ground_stable_since or time.monotonic()
                    if time.monotonic() - ground_stable_since >= 0.7:
                        self.publish_command(
                            VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM,
                            param1=0.0,
                            param2=21196.0,
                        )
                        self.events.append(
                            {
                                "phase": "land",
                                "event": "ground_gated_force_disarm_sent",
                                "time_s": self.elapsed(),
                            }
                        )
                        ground_stable_since = time.monotonic() + 1000.0
                else:
                    ground_stable_since = None
        raise IdentificationAbort("LAND disarm timeout")

    def run(self) -> None:
        try:
            self._wait_initial_state()
            self.ground_ned = np.array([self.local.x, self.local.y, self.local.z], dtype=float)
            self.hover_ned = self.ground_ned.copy()
            self.hover_ned[2] -= self.TAKEOFF_HEIGHT_M
            self.hover_yaw = float(self.local.heading)
            self._run_for(
                2.2,
                "takeoff_prestream",
                {"position_ned_m": self.hover_ned.tolist(), "yaw_rad": self.hover_yaw},
                lambda: self.publish_position(self.hover_ned, self.hover_yaw),
                direct_control=False,
            )
            self.publish_command(
                VehicleCommand.VEHICLE_CMD_DO_SET_MODE, param1=1.0, param2=6.0
            )
            self.publish_command(
                VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=1.0
            )
            self._wait_stable(50.0, 3.0)

            self._direct_attitude_step("roll", math.radians(3.0), 0.0, self.hover_yaw)
            self._direct_attitude_step("pitch", 0.0, math.radians(3.0), self.hover_yaw)
            self._direct_attitude_step("yaw", 0.0, 0.0, self.hover_yaw + math.radians(5.0))
            # Lower roll/pitch rate for a longer plateau: the accumulated
            # commanded angle remains 4.8 degrees while rise and 0.3 s
            # settling can be measured.  Yaw does not consume the tilt margin.
            self._direct_rate_step(
                "roll",
                math.radians(4.0),
                0.0,
                0.0,
                step_s=self.RATE_ROLL_PITCH_STEP_S,
            )
            self._direct_rate_step(
                "pitch",
                0.0,
                math.radians(4.0),
                0.0,
                step_s=self.RATE_ROLL_PITCH_STEP_S,
            )
            self._direct_rate_step(
                "yaw",
                0.0,
                0.0,
                math.radians(8.0),
                step_s=self.RATE_YAW_STEP_S,
            )
            self._wait_stable(20.0, 2.0)
            self._land()
            self.result = "INNER_LOOP_IDENTIFICATION_PASS"
        except IdentificationAbort as error:
            self.abort_reason = str(error)
            self.result = "INNER_LOOP_IDENTIFICATION_ABORT"
            self.get_logger().error(self.abort_reason)
            try:
                self._land()
            except IdentificationAbort as land_error:
                self.abort_reason += f"; {land_error}"
        finally:
            self.write_report()

    def write_report(self) -> None:
        payload = {
            "schema": 1,
            "profile": "4kg_debug_only",
            "result": self.result,
            "abort_reason": self.abort_reason,
            "limits": {
                "horizontal_error_m": self.MAX_HORIZONTAL_ERROR_M,
                "vertical_error_m": self.MAX_VERTICAL_ERROR_M,
                "tilt_deg": self.MAX_TILT_DEG,
                "hover_thrust": self.HOVER_THRUST,
            },
            "protocol": {
                "direct_baseline_s": self.DIRECT_BASELINE_S,
                "attitude_step_s": self.ATTITUDE_STEP_S,
                "attitude_zero_s": self.ATTITUDE_ZERO_S,
                "rate_roll_pitch_deg_s": 4.0,
                "rate_roll_pitch_step_s": self.RATE_ROLL_PITCH_STEP_S,
                "rate_yaw_deg_s": 8.0,
                "rate_yaw_step_s": self.RATE_YAW_STEP_S,
                "rate_zero_s": self.RATE_ZERO_S,
            },
            "observed": {
                "max_horizontal_error_m": self.max_horizontal_error_m,
                "max_vertical_error_m": self.max_vertical_error_m,
                "max_tilt_deg": self.max_tilt_deg,
                "motor_saturation_samples": self.motor_saturation_samples,
                "motor_sample_count": self.motor_samples,
            },
            "events": self.events,
            "samples": self.samples,
        }
        self.output.parent.mkdir(parents=True, exist_ok=True)
        self.output.write_text(json.dumps(payload, indent=2) + "\n", encoding="utf-8")
        print(f"INNER_LOOP_IDENTIFICATION_REPORT {self.output}", flush=True)
        print(self.result, flush=True)


def parse_args(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument(
        "--confirm-4kg-debug",
        action="store_true",
        help="required acknowledgement that this is not the formal 7.735 kg model",
    )
    return parser.parse_args(argv)


def main(args=None) -> None:
    options = parse_args(args)
    if not options.confirm_4kg_debug:
        raise SystemExit("refusing to arm without --confirm-4kg-debug")
    rclpy.init(args=None)
    node = InnerLoopIdentification(options.output)
    try:
        node.run()
    finally:
        succeeded = node.result == "INNER_LOOP_IDENTIFICATION_PASS"
        node.destroy_node()
        rclpy.shutdown()
    if not succeeded:
        raise SystemExit(1)


if __name__ == "__main__":
    main()
