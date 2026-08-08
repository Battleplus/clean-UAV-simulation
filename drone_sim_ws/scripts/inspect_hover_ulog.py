#!/usr/bin/env python3
"""Print a compact, reproducible summary of a PX4 hover ULog.

This is intentionally read-only and is used to correlate the first position,
attitude, thrust, and actuator divergence during Gazebo regression runs.
"""

from __future__ import annotations

import argparse
import json
import math
from pathlib import Path

import numpy as np
from pyulog import ULog


def _dataset(ulog: ULog, name: str):
    matches = [item for item in ulog.data_list if item.name == name]
    return matches[0] if matches else None


def _field(data, *names: str):
    for name in names:
        if name in data:
            return np.asarray(data[name])
    return None


def _first_crossing(timestamp, values, threshold: float):
    indices = np.flatnonzero(np.asarray(values) > threshold)
    if not len(indices):
        return None
    return float(timestamp[indices[0]] / 1e6)


def _euler_from_quaternion(q):
    w, x, y, z = q.T
    roll = np.arctan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y))
    pitch = np.arcsin(np.clip(2 * (w * y - z * x), -1, 1))
    yaw = np.arctan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))
    return np.column_stack((roll, pitch, yaw))


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("ulog", type=Path)
    parser.add_argument("--config", type=Path)
    parser.add_argument("--plot", type=Path, help="write an armed-window diagnostic PNG")
    parser.add_argument("--list", action="store_true", help="list datasets and fields")
    parser.add_argument("--window-start-s", type=float)
    parser.add_argument("--window-end-s", type=float)
    args = parser.parse_args()
    ulog = ULog(str(args.ulog))
    print(f"file={args.ulog}")
    print(f"duration_s={(ulog.last_timestamp - ulog.start_timestamp) / 1e6:.3f}")
    gain_prefixes = ("MC_ROLL", "MC_PITCH", "MC_YAW", "IMU_GYRO_CUTOFF")
    selected_parameters = {
        key: value for key, value in ulog.initial_parameters.items()
        if key.startswith(gain_prefixes)
    }
    if selected_parameters:
        print("control_parameters=" + json.dumps(selected_parameters, sort_keys=True))

    if args.list:
        for item in sorted(ulog.data_list, key=lambda value: (value.name, value.multi_id)):
            print(f"{item.name}[{item.multi_id}]: {','.join(item.data.keys())}")
        return

    armed = _dataset(ulog, "actuator_armed")
    armed_start = ulog.start_timestamp
    armed_end = ulog.last_timestamp
    if armed is not None:
        armed_values = np.asarray(armed.data["armed"], dtype=bool)
        armed_indices = np.flatnonzero(armed_values)
        if len(armed_indices):
            armed_times = np.asarray(armed.data["timestamp"])
            armed_start = armed_times[armed_indices[0]]
            later_disarmed = np.flatnonzero(
                (armed_times > armed_start) & ~armed_values
            )
            if len(later_disarmed):
                armed_end = armed_times[later_disarmed[0]]
            print(f"armed_start_log_s={(armed_start - ulog.start_timestamp) / 1e6:.3f}")
            print(f"armed_duration_s={(armed_end - armed_start) / 1e6:.3f}")

    local = _dataset(ulog, "vehicle_local_position")
    if local is not None:
        d = local.data
        t = np.asarray(d["timestamp"])
        x, y, z = (_field(d, key) for key in ("x", "y", "z"))
        valid = (
            np.isfinite(x) & np.isfinite(y) & np.isfinite(z)
            & (t >= armed_start) & (t <= armed_end)
        )
        if np.any(valid):
            first = np.flatnonzero(valid)[0]
            x0, y0, z0 = x[first], y[first], z[first]
            horizontal = np.hypot(x - x0, y - y0)
            vertical = np.abs(z - z0)
            horizontal = np.where(valid, horizontal, np.nan)
            vertical = np.where(valid, vertical, np.nan)
            relative_t = t - armed_start
            print(f"position_max_horizontal_m={np.nanmax(horizontal):.6f}")
            print(f"position_max_vertical_m={np.nanmax(vertical):.6f}")
            for threshold in (0.1, 0.3, 1.0):
                print(f"horizontal_gt_{threshold:g}m_s={_first_crossing(relative_t, horizontal, threshold)}")
                print(f"vertical_gt_{threshold:g}m_s={_first_crossing(relative_t, vertical, threshold)}")

    attitude = _dataset(ulog, "vehicle_attitude")
    if attitude is not None:
        d = attitude.data
        t = np.asarray(d["timestamp"])
        q = np.column_stack([d[f"q[{index}]"] for index in range(4)])
        euler = np.degrees(_euler_from_quaternion(q))
        tilt = np.hypot(euler[:, 0], euler[:, 1])
        active = (t >= armed_start) & (t <= armed_end)
        tilt = np.where(active, tilt, np.nan)
        relative_t = t - armed_start
        print(f"attitude_max_abs_roll_deg={np.nanmax(np.where(active, np.abs(euler[:, 0]), np.nan)):.6f}")
        print(f"attitude_max_abs_pitch_deg={np.nanmax(np.where(active, np.abs(euler[:, 1]), np.nan)):.6f}")
        for threshold in (5.0, 10.0, 20.0):
            print(f"tilt_gt_{threshold:g}deg_s={_first_crossing(relative_t, tilt, threshold)}")
        groundtruth_attitude = _dataset(ulog, "vehicle_attitude_groundtruth")
        if groundtruth_attitude is not None:
            gd = groundtruth_attitude.data
            gt_t = np.asarray(gd["timestamp"])
            gt_q = np.column_stack([gd[f"q[{index}]"] for index in range(4)])
            gt_euler = np.degrees(_euler_from_quaternion(gt_q))
            estimate_index = int(np.argmin(np.abs(t - armed_start)))
            groundtruth_index = int(np.argmin(np.abs(gt_t - armed_start)))
            yaw_error = (
                euler[estimate_index, 2] - gt_euler[groundtruth_index, 2] + 180.0
            ) % 360.0 - 180.0
            print(f"yaw_estimate_at_arm_deg={euler[estimate_index, 2]:.6f}")
            print(f"yaw_groundtruth_at_arm_deg={gt_euler[groundtruth_index, 2]:.6f}")
            print(f"yaw_estimate_error_at_arm_deg={yaw_error:.6f}")

    actuator = _dataset(ulog, "actuator_motors")
    if actuator is not None:
        d = actuator.data
        controls = []
        for index in range(12):
            field = _field(d, f"control[{index}]")
            if field is not None:
                controls.append(field)
        if controls:
            values = np.column_stack(controls[:8])
            active = (
                (np.asarray(d["timestamp"]) >= armed_start)
                & (np.asarray(d["timestamp"]) <= armed_end)
            )
            values = values[active]
            finite = np.isfinite(values)
            values = np.where(finite, values, np.nan)
            print(f"actuator_min={np.nanmin(values):.6f}")
            print(f"actuator_max={np.nanmax(values):.6f}")
            print(f"actuator_saturation_fraction={np.nanmean((values <= 0.001) | (values >= 0.999)):.6f}")

    thrust = _dataset(ulog, "vehicle_thrust_setpoint")
    if thrust is not None:
        d = thrust.data
        xyz = np.column_stack([d[f"xyz[{index}]"] for index in range(3)])
        active = (
            (np.asarray(d["timestamp"]) >= armed_start)
            & (np.asarray(d["timestamp"]) <= armed_end)
        )
        xyz = xyz[active]
        print(f"thrust_setpoint_norm_min={np.nanmin(np.linalg.norm(xyz, axis=1)):.6f}")
        print(f"thrust_setpoint_norm_max={np.nanmax(np.linalg.norm(xyz, axis=1)):.6f}")

    local_setpoint = _dataset(ulog, "vehicle_local_position_setpoint")
    if local_setpoint is not None and local is not None:
        sd, ld = local_setpoint.data, local.data
        st = np.asarray(sd["timestamp"])
        lt = np.asarray(ld["timestamp"])
        acceleration_sp = np.column_stack([
            sd[f"acceleration[{index}]"] for index in range(2)
        ])
        acceleration_measured = np.column_stack([ld["ax"], ld["ay"]])
        measured_at_setpoint = np.column_stack([
            np.interp(st, lt, acceleration_measured[:, axis]) for axis in range(2)
        ])
        identification = (
            (st >= armed_start + 4e6)
            & (st <= min(armed_end, armed_start + 12e6))
            & np.all(np.isfinite(acceleration_sp), axis=1)
            & np.all(np.isfinite(measured_at_setpoint), axis=1)
        )
        if np.count_nonzero(identification) > 5:
            for axis, label in enumerate(("north", "east")):
                correlation = np.corrcoef(
                    acceleration_sp[identification, axis],
                    measured_at_setpoint[identification, axis],
                )[0, 1]
                print(f"acceleration_sp_to_measured_corr_{label}={correlation:.6f}")

        control_mode = _dataset(ulog, "vehicle_control_mode")
        if control_mode is not None:
            cd = control_mode.data
            ct = np.asarray(cd["timestamp"])
            offboard = np.asarray(cd["flag_control_offboard_enabled"], dtype=bool)
            offboard &= (ct >= armed_start) & (ct <= armed_end)
            if np.any(offboard):
                offboard_start = int(ct[np.flatnonzero(offboard)[0]])
                offboard_end = int(ct[np.flatnonzero(offboard)[-1]])
                if args.window_start_s is not None or args.window_end_s is not None:
                    window_start_s = args.window_start_s or 0.0
                    window_end_s = args.window_end_s
                    steady_start = offboard_start + int(window_start_s * 1e6)
                    steady_end = (
                        offboard_end if window_end_s is None
                        else min(offboard_end, offboard_start + int(window_end_s * 1e6))
                    )
                else:
                    steady_start = max(offboard_start, offboard_end - int(10e6))
                    steady_end = offboard_end
                steady = (st >= steady_start) & (st <= steady_end)
                if np.count_nonzero(steady) > 5:
                    actual_z = np.interp(st[steady], lt, np.asarray(ld["z"]))
                    actual_vz = np.interp(st[steady], lt, np.asarray(ld["vz"]))
                    setpoint_z = np.asarray(sd["z"])[steady]
                    setpoint_vz = np.asarray(sd["vz"])[steady]
                    thrust_z = np.asarray(sd["thrust[2]"])[steady]
                    finite_z = np.isfinite(actual_z) & np.isfinite(setpoint_z)
                    print(f"offboard_duration_s={(offboard_end - offboard_start) / 1e6:.6f}")
                    if np.any(finite_z):
                        error_z = actual_z[finite_z] - setpoint_z[finite_z]
                        print(f"steady_z_actual_mean_m={np.mean(actual_z[finite_z]):.6f}")
                        print(f"steady_z_setpoint_mean_m={np.mean(setpoint_z[finite_z]):.6f}")
                        print(f"steady_z_error_mean_m={np.mean(error_z):.6f}")
                        print(f"steady_z_error_max_abs_m={np.max(np.abs(error_z)):.6f}")
                    finite_vz = np.isfinite(actual_vz) & np.isfinite(setpoint_vz)
                    if np.any(finite_vz):
                        print(f"steady_vz_actual_mean_m_s={np.mean(actual_vz[finite_vz]):.6f}")
                        print(f"steady_vz_setpoint_mean_m_s={np.mean(setpoint_vz[finite_vz]):.6f}")
                    finite_thrust = np.isfinite(thrust_z)
                    if np.any(finite_thrust):
                        print(f"steady_thrust_z_mean={np.mean(thrust_z[finite_thrust]):.6f}")
                        print(f"steady_thrust_z_range=[{np.min(thrust_z[finite_thrust]):.6f},{np.max(thrust_z[finite_thrust]):.6f}]")

    torque = _dataset(ulog, "vehicle_torque_setpoint")
    angular = _dataset(ulog, "vehicle_angular_velocity_groundtruth")
    if torque is not None and angular is not None:
        td, ad = torque.data, angular.data
        tt = np.asarray(td["timestamp"])
        at = np.asarray(ad["timestamp"])
        torque_xyz = np.column_stack([td[f"xyz[{index}]"] for index in range(3)])
        omega = np.column_stack([ad[f"xyz[{index}]"] for index in range(3)])
        alpha = np.gradient(omega, at / 1e6, axis=0)
        early = (tt >= armed_start + 0.5e6) & (tt <= min(armed_end, armed_start + 8e6))
        if np.count_nonzero(early) > 5:
            alpha_at_torque = np.column_stack([
                np.interp(tt[early], at, alpha[:, axis]) for axis in range(3)
            ])
            for axis, label in enumerate(("roll", "pitch", "yaw")):
                requested = torque_xyz[early, axis]
                measured = alpha_at_torque[:, axis]
                correlation = np.corrcoef(requested, measured)[0, 1]
                print(f"torque_to_alpha_corr_{label}={correlation:.6f}")

    # PX4's estimated body-rate topic is unambiguously FRD.  Keep it beside
    # ground truth so a simulator-specific ground-truth frame convention does
    # not get misdiagnosed as an actuator sign error.
    angular_estimated = _dataset(ulog, "vehicle_angular_velocity")
    if torque is not None and angular_estimated is not None:
        td, ad = torque.data, angular_estimated.data
        tt = np.asarray(td["timestamp"])
        at = np.asarray(ad["timestamp"])
        torque_xyz = np.column_stack([td[f"xyz[{index}]"] for index in range(3)])
        omega = np.column_stack([ad[f"xyz[{index}]"] for index in range(3)])
        alpha = np.gradient(omega, at / 1e6, axis=0)
        early = (tt >= armed_start + 0.5e6) & (tt <= min(armed_end, armed_start + 8e6))
        if np.count_nonzero(early) > 5:
            alpha_at_torque = np.column_stack([
                np.interp(tt[early], at, alpha[:, axis]) for axis in range(3)
            ])
            for axis, label in enumerate(("roll", "pitch", "yaw")):
                correlation = np.corrcoef(
                    torque_xyz[early, axis], alpha_at_torque[:, axis]
                )[0, 1]
                print(f"torque_to_estimated_alpha_corr_{label}={correlation:.6f}")

    if args.config is not None and actuator is not None and angular is not None:
        config = json.loads(args.config.read_text(encoding="utf-8"))
        matrix_columns = []
        moment_ratio = float(config.get("reaction_moment_ratio_m") or 0.0)
        for rotor in sorted(config["rotors"], key=lambda value: int(value["motor"])):
            position = np.asarray(rotor["position_m"], dtype=float)
            axis = np.asarray(rotor.get("thrust_axis_body", rotor["axis_body"]), dtype=float)
            axis /= np.linalg.norm(axis)
            moment = np.cross(position, axis) - float(rotor["direction"]) * moment_ratio * axis
            matrix_columns.append(np.concatenate((axis, moment)))
        matrix = np.column_stack(matrix_columns)
        normalization = config["actuator_normalization"]
        hover_command = float(normalization["px4_hover_command"])
        hover_thrust = float(normalization["physical_hover_thrust_n"])
        maximum_thrust = float(normalization["maximum_thrust_n"])
        ad = actuator.data
        actuator_t = np.asarray(ad["timestamp"])
        command = np.column_stack([ad[f"control[{index}]"] for index in range(8)])
        command = np.clip(command, 0.0, 1.0)
        if config.get("actuator_input_model") == "hover_scaled_linear_thrust_with_rated_cap":
            gain = float(normalization["linear_thrust_per_command_n"])
            thrust_n = np.minimum(maximum_thrust, command * gain)
        else:
            thrust_n = np.where(
                command <= hover_command,
                command * hover_thrust / hover_command,
                hover_thrust + (command - hover_command) * (maximum_thrust - hover_thrust) / (1.0 - hover_command),
            )
        predicted_wrench = thrust_n @ matrix.T
        angular_t = np.asarray(angular.data["timestamp"])
        omega = np.column_stack([angular.data[f"xyz[{index}]"] for index in range(3)])
        alpha = np.gradient(omega, angular_t / 1e6, axis=0)
        alpha_at_actuator = np.column_stack([
            np.interp(actuator_t, angular_t, alpha[:, axis]) for axis in range(3)
        ])
        early = (
            (actuator_t >= armed_start + 0.5e6)
            & (actuator_t <= min(armed_end, armed_start + 8e6))
        )
        for axis, label in enumerate(("roll", "pitch", "yaw")):
            correlation = np.corrcoef(
                predicted_wrench[early, 3 + axis], alpha_at_actuator[early, axis]
            )[0, 1]
            print(f"predicted_wrench_to_alpha_corr_{label}={correlation:.6f}")
        if torque is not None:
            torque_t = np.asarray(torque.data["timestamp"])
            torque_xyz = np.column_stack([
                torque.data[f"xyz[{index}]"] for index in range(3)
            ])
            torque_at_actuator = np.column_stack([
                np.interp(actuator_t, torque_t, torque_xyz[:, axis])
                for axis in range(3)
            ])
            for axis, label in enumerate(("roll", "pitch", "yaw")):
                correlation = np.corrcoef(
                    torque_at_actuator[early, axis], predicted_wrench[early, 3 + axis]
                )[0, 1]
                print(f"torque_to_predicted_wrench_corr_{label}={correlation:.6f}")

        if args.plot is not None:
            import matplotlib.pyplot as plt

            figure, axes = plt.subplots(5, 1, figsize=(13, 13), sharex=True)
            if local is not None:
                local_t = (np.asarray(local.data["timestamp"]) - armed_start) / 1e6
                local_position = np.column_stack([
                    local.data[key] for key in ("x", "y", "z")
                ])
                local_position -= local_position[np.argmin(np.abs(local_t))]
                axes[0].plot(local_t, local_position)
                axes[0].legend(("x", "y", "z"), ncol=3)
                axes[0].set_ylabel("position (m)")
            if attitude is not None:
                attitude_t = (np.asarray(attitude.data["timestamp"]) - armed_start) / 1e6
                q = np.column_stack([
                    attitude.data[f"q[{index}]"] for index in range(4)
                ])
                attitude_deg = np.degrees(_euler_from_quaternion(q))
                axes[1].plot(attitude_t, attitude_deg)
                axes[1].legend(("roll", "pitch", "yaw"), ncol=3)
                axes[1].set_ylabel("attitude (deg)")
            actuator_relative_t = (actuator_t - armed_start) / 1e6
            axes[2].plot(actuator_relative_t, command)
            axes[2].set_ylabel("motor command")
            axes[2].set_ylim(-0.05, 1.05)
            axes[3].plot(actuator_relative_t, predicted_wrench[:, 3:])
            axes[3].legend(("roll", "pitch", "yaw"), ncol=3)
            axes[3].set_ylabel("predicted torque (Nm)")
            angular_relative_t = (angular_t - armed_start) / 1e6
            axes[4].plot(angular_relative_t, omega)
            axes[4].legend(("p", "q", "r"), ncol=3)
            axes[4].set_ylabel("body rate (rad/s)")
            axes[4].set_xlabel("seconds after arm")
            for axis in axes:
                axis.grid(True, alpha=0.3)
                axis.set_xlim(0.0, min(20.0, (armed_end - armed_start) / 1e6))
            figure.tight_layout()
            args.plot.parent.mkdir(parents=True, exist_ok=True)
            figure.savefig(args.plot, dpi=150)
            print(f"plot={args.plot}")


if __name__ == "__main__":
    main()
