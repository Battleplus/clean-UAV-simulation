from pathlib import Path

from build_base1_freeze_manifest import BASE1_COMMIT, BASE1_REF, build_manifest


ROOT = Path(__file__).resolve().parents[2]


def test_base1_reference_and_current_core_are_identical() -> None:
    manifest = build_manifest(ROOT, BASE1_REF)
    assert manifest["reference_commit"].startswith(BASE1_COMMIT)
    assert manifest["scope"].startswith("4kg Base 1")
    assert manifest["all_core_files_match"] is True
    assert manifest["mismatches"] == []
    assert all(item["matches_base1"] for item in manifest["files"])


def test_base1_compensation_defaults_are_all_off() -> None:
    defaults = build_manifest(ROOT, BASE1_REF)["default_compensation"]
    assert defaults == {
        "ARM_FEEDFORWARD_ENABLED": False,
        "ARM_TORQUE_FEEDFORWARD_ENABLED": False,
        "ARM_STATIC_COM_FEEDFORWARD_GAIN": 0.0,
        "ARM_DISTURBANCE_OBSERVER_ENABLED": False,
    }
    backend = (ROOT / "drone_sim_ws/scripts/wsl_start_ros2_dds_noarm.sh").read_text()
    wasd = (
        ROOT
        / "drone_sim_ws/src/px4_ros2_control/px4_ros2_control/dds_wasd_control.py"
    ).read_text()
    launch = (
        ROOT / "drone_sim_ws/src/drone_arm_sim/launch/cad_direct_thrust.launch.py"
    ).read_text()
    assert '${ARM_TORQUE_FEEDFORWARD_ENABLED:-false}' in backend
    assert '${ARM_STATIC_COM_FEEDFORWARD_GAIN:-0.0}' in backend
    assert '${ARM_DISTURBANCE_OBSERVER_ENABLED:-false}' in backend
    assert '"ARM_FEEDFORWARD_ENABLED", "false"' in wasd
    assert '"arm_torque_feedforward_enabled", default_value="false"' in launch
    assert '"arm_static_com_feedforward_gain", default_value="0.0"' in launch
    assert '"arm_disturbance_observer_enabled", default_value="false"' in launch
