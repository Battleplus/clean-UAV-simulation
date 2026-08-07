#!/usr/bin/env python3
"""Compare retracted, expanded, moving and payload coupling states."""

from __future__ import annotations

import argparse
import json
from pathlib import Path

import numpy as np

from drone_arm_sim.coupled_dynamics import CoupledArmDynamics, JOINT_NAMES, Payload


def record(state) -> dict:
    return {
        "mass_kg": state.mass_kg,
        "center_of_mass_body_flu_m": state.center_of_mass_m.tolist(),
        "com_shift_from_matching_home_m": state.com_shift_m.tolist(),
        "com_shift_norm_m": float(np.linalg.norm(state.com_shift_m)),
        "inertia_at_com_kg_m2": state.inertia_at_com_kg_m2.tolist(),
        "inertia_eigenvalues_kg_m2": np.linalg.eigvalsh(state.inertia_at_com_kg_m2).tolist(),
        "reaction_force_body_n": state.reaction_force_body_n.tolist(),
        "reaction_force_norm_n": float(np.linalg.norm(state.reaction_force_body_n)),
        "reaction_torque_body_nm": state.reaction_torque_body_nm.tolist(),
        "reaction_torque_norm_nm": float(np.linalg.norm(state.reaction_torque_body_nm)),
        "joint_resisting_torque_nm": state.joint_resisting_torque_nm.tolist(),
    }


def main() -> int:
    workspace = Path(__file__).resolve().parents[1]
    package = workspace / "src/drone_arm_sim"
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--urdf",
        type=Path,
        default=package / "urdf/my_drone_v3/my_drone_cad_formal_dynamic.urdf",
    )
    parser.add_argument(
        "--motion-reference",
        type=Path,
        default=package / "config/so101_motion_reference.json",
    )
    parser.add_argument("--payload-mass-kg", type=float, default=0.25)
    parser.add_argument(
        "--output",
        type=Path,
        default=workspace / "analysis/arm_coupling_comparison.json",
    )
    args = parser.parse_args()
    reference = json.loads(args.motion_reference.read_text(encoding="utf-8"))
    dynamics = CoupledArmDynamics(args.urdf, reference, target_mass_kg=7.735)

    def positions(preset: str) -> dict[str, float]:
        return dict(zip(JOINT_NAMES, reference["presets"][preset]))

    retracted = dynamics.state(positions("retracted"))
    expanded = dynamics.state(positions("work_a"))
    velocity = dict(zip(JOINT_NAMES, (0.25, -0.20, 0.30, -0.20, 0.15, 0.10)))
    acceleration = dict(zip(JOINT_NAMES, (0.50, -0.40, 0.60, -0.40, 0.30, 0.20)))
    working = dynamics.state(positions("work_a"), velocity, acceleration)
    payload = Payload(args.payload_mass_kg)
    loaded = dynamics.state(positions("work_a"), velocity, acceleration, payload)
    contact = dynamics.contact_delta_twist(
        positions("work_a"), np.array([0.0, 0.0, -1.0]), payload
    )
    report = {
        "schema": 1,
        "urdf": str(args.urdf),
        "model_mass_without_payload_kg": 7.735,
        "payload_mass_kg": args.payload_mass_kg,
        "states": {
            "retracted": record(retracted),
            "expanded_static_work_a": record(expanded),
            "moving_work_a": record(working),
            "moving_work_a_with_payload": record(loaded),
        },
        "one_ns_downward_payload_contact_delta_twist_body": contact.tolist(),
        "provisional_parameters": {
            "joint_damping_and_coulomb_friction": "from so101_motion_reference.json; not servo-bench calibrated",
            "payload_inertia": "1e-4 kg m^2 diagonal placeholder",
            "contact": "impulse response calculation; Gazebo arm collision/contact validation remains required",
        },
    }
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(
        json.dumps(report, ensure_ascii=False, indent=2) + "\n", encoding="utf-8"
    )
    print("ARM_COUPLING_ANALYSIS " + json.dumps({
        "expanded_com_shift_m": record(expanded)["com_shift_norm_m"],
        "moving_reaction_force_n": record(working)["reaction_force_norm_n"],
        "moving_reaction_torque_nm": record(working)["reaction_torque_norm_nm"],
        "loaded_mass_kg": loaded.mass_kg,
        "contact_delta_twist_norm": float(np.linalg.norm(contact)),
    }, sort_keys=True))
    print(f"ARM_COUPLING_ANALYSIS_PASS output={args.output}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
