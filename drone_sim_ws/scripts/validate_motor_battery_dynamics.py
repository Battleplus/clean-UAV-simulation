#!/usr/bin/env python3
"""Offline acceptance check for the provisional motor/ESC/battery dynamics.

This deliberately uses the measured normalized static thrust table and the
provisional first-order motor model.  It does not turn the model into an RPM
model; the report keeps that limitation explicit.
"""

from __future__ import annotations

import argparse
import copy
import json
from pathlib import Path

import numpy as np

from drone_arm_sim.allocation_analysis import allocation_matrix, allocate_bounded_wrench
from drone_arm_sim.gazebo_direct_motor_model import (
    battery_step,
    command_to_thrust_n,
    first_order_motor_step,
)


def _crossing_time(times: list[float], values: list[float], level: float) -> float | None:
    for index, value in enumerate(values):
        if value >= level:
            return times[index]
    return None


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--config",
        type=Path,
        default=Path(__file__).resolve().parents[1]
        / "src/drone_arm_sim/config/my_drone_v3_cad_7p735_flight.json",
    )
    parser.add_argument("--duration", type=float, default=5.0)
    parser.add_argument("--battery-resistance-ohm", type=float, default=0.0005)
    parser.add_argument("--battery-capacity-ah", type=float, default=20.0)
    args = parser.parse_args()

    config = json.loads(args.config.read_text(encoding="utf-8"))
    dynamic = copy.deepcopy(config)
    battery = dynamic.setdefault("battery_dynamics", {})
    battery.update(
        enabled=True,
        pack_internal_resistance_ohm=args.battery_resistance_ohm,
        capacity_ah=args.battery_capacity_ah,
    )

    dt = 0.001
    rise_times: list[float] = []
    rise_values: list[float] = []
    command = np.zeros(8)
    target = np.ones(8)
    for index in range(int(args.duration / dt)):
        command = first_order_motor_step(command, target, dt, dynamic)
        rise_times.append((index + 1) * dt)
        rise_values.append(float(command[0]))
    rise_63 = _crossing_time(rise_times, rise_values, 1.0 - np.exp(-1.0))
    rise_95 = _crossing_time(rise_times, rise_values, 0.95)

    fall_times: list[float] = []
    fall_values: list[float] = []
    command = np.ones(8)
    for index in range(int(args.duration / dt)):
        command = first_order_motor_step(command, np.zeros(8), dt, dynamic)
        fall_times.append((index + 1) * dt)
        fall_values.append(float(1.0 - command[0]))
    fall_63 = _crossing_time(fall_times, fall_values, 1.0 - np.exp(-1.0))

    # Exercise the enabled battery model at a representative hover command.
    soc = 1.0
    voltage = float(battery.get("reference_voltage_v", 14.8))
    scales: list[float] = []
    currents: list[float] = []
    for _ in range(int(args.duration / dt)):
        soc, voltage, scale, current = battery_step(dynamic, np.full(8, 0.86), soc, dt)
        scales.append(scale)
        currents.append(current)

    mass = float(dynamic.get("estimated_all_up_mass_kg", dynamic.get("temporary_fixed_mass_kg", 7.735)))
    gravity = float(dynamic.get("gravity_m_s2", 9.81))
    hover = np.array([0.0, 0.0, -mass * gravity, 0.0, 0.0, 0.0])
    allocation = allocate_bounded_wrench(dynamic, hover)
    matrix = allocation_matrix(dynamic)
    # Battery scale applies to the available thrust cap, not to the geometry.
    available = allocation_matrix(dynamic) @ np.full(8, float(dynamic["maximum_thrust_n"]))
    available *= float(min(scales))
    vertical_margin_n = float(abs(available[2]) - abs(hover[2]))

    report = {
        "model_boundary": "normalized static thrust table; no RPM/C_T/C_Q",
        "rise_tau_config_s": float(dynamic["motor_dynamics"]["rise_time_constant_s"]),
        "fall_tau_config_s": float(dynamic["motor_dynamics"]["fall_time_constant_s"]),
        "rise_t63_s": rise_63,
        "rise_t95_s": rise_95,
        "fall_t63_s": fall_63,
        "battery_enabled": True,
        "battery_resistance_ohm": float(args.battery_resistance_ohm),
        "battery_capacity_ah": float(args.battery_capacity_ah),
        "battery_min_voltage_v": float(min(
            battery_step(dynamic, np.full(8, 0.86), 1.0, 0.0)[1], voltage
        )),
        "battery_final_voltage_v": float(voltage),
        "battery_min_thrust_scale": float(min(scales)),
        "battery_max_current_a": float(max(currents)),
        "battery_max_scale_step": float(max(
            abs(scales[i] - scales[i - 1]) for i in range(1, len(scales))
        )),
        "hover_allocation_feasible_at_reference": bool(allocation["feasible"]),
        "hover_vertical_margin_after_sag_n": vertical_margin_n,
        "allocation_rank": int(np.linalg.matrix_rank(matrix)),
    }
    print("MOTOR_BATTERY_DYNAMICS " + json.dumps(report, sort_keys=True))

    checks = [
        rise_63 is not None,
        rise_95 is not None,
        fall_63 is not None,
        report["battery_max_scale_step"] < 0.02,
        report["hover_allocation_feasible_at_reference"],
    ]
    if not all(checks):
        print("MOTOR_BATTERY_DYNAMICS_FAIL", flush=True)
        return 1
    print("MOTOR_BATTERY_DYNAMICS_PASS", flush=True)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
