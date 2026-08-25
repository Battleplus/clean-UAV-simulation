import json
from pathlib import Path
import re
import xml.etree.ElementTree as ET

import numpy as np

from drone_arm_sim.allocation_analysis import allocation_matrix
from drone_arm_sim.model_analysis import UrdfModel
from drone_arm_sim.trajectory_preflight import TrajectoryPreflight


WORKSPACE = Path(__file__).resolve().parents[3]
PACKAGE = WORKSPACE / "src/drone_arm_sim"
URDF = PACKAGE / "urdf/my_drone_v3/my_drone_cad_candidate_1p3kg.urdf"
CONFIG = PACKAGE / "config/my_drone_v3_cad_candidate_1p3kg.json"
REFERENCE = PACKAGE / "config/so101_motion_reference_4kg.json"
AIRFRAME = WORKSPACE / "px4/airframes/4028_gz_my_drone_octorotor_candidate_1p3kg"
LAUNCHER = WORKSPACE / "scripts/wsl_start_ros2_dds_candidate_1p3kg.sh"
DIRECTIONAL_RUNNER = WORKSPACE / "scripts/run_directional_workspace_acceptance_1p3kg.sh"
WASD_RUNNER = WORKSPACE / "scripts/run_wasd_acceptance_1p3kg.sh"
FULL_RUNNER = WORKSPACE / "scripts/run_full_acceptance_1p3kg.sh"
STATUS_REPORT = WORKSPACE / "analysis/base1/candidate_1p3kg_acceptance_status.json"
ARM_LINKS = {
    "arm_base_link",
    "shoulder_link",
    "upper_arm_link",
    "lower_arm_link",
    "wrist_link",
    "gripper_link",
    "moving_jaw_link",
}


def _mass_partition():
    root = ET.parse(URDF).getroot()
    rows = {
        link.get("name"): float(link.find("inertial/mass").get("value"))
        for link in root.findall("link")
        if link.find("inertial/mass") is not None
    }
    arm = sum(value for name, value in rows.items() if name in ARM_LINKS)
    airframe = sum(value for name, value in rows.items() if name not in ARM_LINKS)
    return root, arm, airframe


def test_candidate_mass_partition_and_inertia_are_physical():
    root, arm, airframe = _mass_partition()
    assert root.get("name") == "my_drone_v3_cad_candidate_1p3kg"
    assert np.isclose(arm, 0.6, atol=1.0e-10, rtol=0.0)
    assert np.isclose(airframe, 0.7, atol=1.0e-10, rtol=0.0)
    reference = json.loads(REFERENCE.read_text(encoding="utf-8"))
    names = [row["name"] for row in reference["joints"]]
    folded = dict(zip(names, reference["presets"]["retracted"], strict=True))
    mass, center, inertia = UrdfModel(URDF).mass_properties(folded)
    assert np.isclose(mass, 1.3, atol=1.0e-9, rtol=0.0)
    assert np.all(np.isfinite(center))
    assert np.all(np.linalg.eigvalsh(inertia) > 0.0)


def test_candidate_hover_allocation_is_com_relative_and_feasible():
    config = json.loads(CONFIG.read_text(encoding="utf-8"))
    assert config["estimated_all_up_mass_kg"] == 1.3
    assert config["mass_partition"]["arm_mass_kg"] == 0.6
    matrix = allocation_matrix(config)
    assert np.linalg.matrix_rank(matrix) == 6
    hover = np.asarray(config["bounded_hover_thrust_n"], dtype=float)
    np.testing.assert_allclose(
        matrix @ hover,
        [0.0, 0.0, -1.3 * 9.80665, 0.0, 0.0, 0.0],
        atol=1.0e-8,
    )
    assert np.all(hover > 0.0)
    assert np.all(hover < float(config["maximum_thrust_n"]))
    center_frd = np.asarray(config["mass_partition"]["center_of_mass_frd_m"])
    for rotor in config["rotors"]:
        np.testing.assert_allclose(
            np.asarray(rotor["wrench_position_m"])
            - np.asarray(rotor["position_m"]),
            center_frd,
            atol=1.0e-10,
        )


def test_candidate_does_not_restore_obsolete_high_landing_support():
    config = json.loads(CONFIG.read_text(encoding="utf-8"))
    assert config["takeoff_support_release"]["restore_on_land"] is False


def test_candidate_airframe_matches_config_and_is_not_base1_airframe():
    config = json.loads(CONFIG.read_text(encoding="utf-8"))
    text = AIRFRAME.read_text(encoding="utf-8")
    assert "candidate 1.3kg arm 0.6kg" in text
    assert "require fresh PX4/Gazebo identification" in text
    hover_match = re.search(r"MPC_THR_HOVER\s+(\S+)", text)
    assert hover_match is not None
    assert abs(
        float(hover_match.group(1))
        - float(config["actuator_normalization"]["px4_hover_command"])
    ) <= 5.0e-5
    expected_rate_gains = {
        "MC_ROLLRATE_P": 0.030,
        "MC_PITCHRATE_P": 0.030,
        "MC_YAWRATE_P": 0.040,
        "MC_ROLLRATE_I": 0.018,
        "MC_PITCHRATE_I": 0.018,
        "MC_YAWRATE_I": 0.010,
        "MC_ROLLRATE_D": 0.0009,
        "MC_PITCHRATE_D": 0.0009,
    }
    for name, expected in expected_rate_gains.items():
        match = re.search(rf"{name}\s+(\S+)", text)
        assert match is not None
        assert np.isclose(float(match.group(1)), expected, atol=5.0e-7)
    for index, rotor in enumerate(sorted(config["rotors"], key=lambda row: row["motor"])):
        for axis, expected in zip("XYZ", rotor["position_m"]):
            match = re.search(rf"CA_ROTOR{index}_P{axis}\s+(\S+)", text)
            assert match is not None
            assert np.isclose(float(match.group(1)), expected, atol=5.0e-10)


def test_candidate_keeps_base1_geometry_and_joint_topology():
    source = ET.parse(
        PACKAGE / "urdf/my_drone_v3/my_drone_cad_debug_4kg.urdf"
    ).getroot()
    candidate = ET.parse(URDF).getroot()
    assert [link.get("name") for link in candidate.findall("link")] == [
        link.get("name") for link in source.findall("link")
    ]
    assert [
        (joint.get("name"), joint.get("type"))
        for joint in candidate.findall("joint")
    ] == [
        (joint.get("name"), joint.get("type"))
        for joint in source.findall("joint")
    ]
    for source_joint, candidate_joint in zip(
        source.findall("joint"), candidate.findall("joint"), strict=True
    ):
        assert ET.tostring(source_joint) == ET.tostring(candidate_joint)


def test_candidate_launcher_selects_candidate_and_strict_static_limits():
    text = LAUNCHER.read_text(encoding="utf-8")
    assert "AIRFRAME_ID=4028" in text
    assert "my_drone_cad_candidate_1p3kg.urdf" in text
    assert "my_drone_v3_cad_candidate_1p3kg.json" in text
    assert "ARM_COUPLING_TARGET_MASS_KG=1.3" in text
    assert "BASE1_GRAVITY_TORQUE_LIMIT_NM=0.90" in text
    assert "BASE1_COMP_MAX_MOTOR_DELTA_N=1.25" in text
    assert 'BASE1_POSITION_FEEDBACK_ENABLED="${BASE1_POSITION_FEEDBACK_ENABLED:-true}"' in text
    assert 'BASE1_POSITION_GAIN_N_M="${BASE1_POSITION_GAIN_N_M:-1.9}"' in text
    assert 'BASE1_VELOCITY_GAIN_N_S_M="${BASE1_VELOCITY_GAIN_N_S_M:-2.8}"' in text
    assert 'PX4_TRUTH_HOLD_ARM_POSITION_OVERLAY="${PX4_TRUTH_HOLD_ARM_POSITION_OVERLAY:-false}"' in text
    assert 'ARM_DIRECT_XY_OWNERSHIP="${ARM_DIRECT_XY_OWNERSHIP:-true}"' in text
    assert 'ARM_DIRECT_XY_WATCHDOG_S="${ARM_DIRECT_XY_WATCHDOG_S:-0.04}"' in text
    assert 'ARM_DIRECT_XY_ENTRY_TIMEOUT_S="${ARM_DIRECT_XY_ENTRY_TIMEOUT_S:-0.15}"' in text
    assert 'BASE1_REACTION_FORCE_GAIN="${BASE1_REACTION_FORCE_GAIN:-0}"' in text
    assert 'BASE1_REACTION_TORQUE_GAIN="${BASE1_REACTION_TORQUE_GAIN:-0}"' in text


def test_candidate_preflight_infers_mass_and_limits_from_selected_config():
    preflight = TrajectoryPreflight(URDF, REFERENCE, CONFIG)
    assert preflight.target_mass_kg == 1.3
    assert preflight.gravity_torque_limit_nm == 0.90
    assert preflight.maximum_motor_delta_n == 1.25
    assert preflight.minimum_motor_headroom_n == 0.25
    assert preflight.minimum_delta_headroom_n == 0.30


def test_candidate_dynamic_acceptance_entry_points_are_isolated_and_strict():
    directional = DIRECTIONAL_RUNNER.read_text(encoding="utf-8")
    assert "arm_workspace_envelope_1p3kg.json" in directional
    assert "directional_workspace_flight_plan_1p3kg.json" in directional
    assert "my_drone_cad_candidate_1p3kg.urdf" in directional
    assert "my_drone_v3_cad_candidate_1p3kg.json" in directional
    assert "ARM_CANDIDATE_MASS_KG=1.3" in directional
    assert "ARM_CANDIDATE_GRAVITY_TORQUE_LIMIT_NM=0.90" in directional
    assert "ARM_CANDIDATE_MAXIMUM_MOTOR_DELTA_N=1.25" in directional
    wasd = WASD_RUNNER.read_text(encoding="utf-8")
    assert "ENABLE_ARM_CONTROL=false" in wasd
    assert "wsl_start_ros2_dds_candidate_1p3kg.sh" in wasd
    full = FULL_RUNNER.read_text(encoding="utf-8")
    assert "run_wasd_acceptance_1p3kg.sh" in full
    assert "run_directional_workspace_acceptance_1p3kg.sh" in full
    assert "CANDIDATE_1P3KG_FULL_ACCEPTANCE_PASS" in full


def test_candidate_machine_audit_never_confuses_offline_with_dynamic_acceptance():
    report = json.loads(STATUS_REPORT.read_text(encoding="utf-8"))
    if report["dynamic_accepted"]:
        assert report["offline_ready"] is True
        assert report["status"] == "DYNAMIC_ACCEPTED"
        assert all(report["details"]["dynamic_checks"].values())
    elif report["offline_ready"]:
        assert report["status"] == "OFFLINE_READY_DYNAMIC_NOT_ACCEPTED"
    else:
        assert report["status"] == "OFFLINE_EVIDENCE_STALE_OR_FAILED"
        assert report["dynamic_accepted"] is False
        assert not all(report["checks"].values())
