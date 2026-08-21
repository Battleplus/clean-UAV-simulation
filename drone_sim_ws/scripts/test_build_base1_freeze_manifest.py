from pathlib import Path

from build_base1_freeze_manifest import (
    BASE1_COMMIT,
    BASE1_REF,
    INTENTIONAL_DIFFERENCES,
    build_manifest,
)


ROOT = Path(__file__).resolve().parents[2]


def test_candidate_manifest_schema_and_reference() -> None:
    manifest = build_manifest(ROOT, BASE1_REF)
    assert manifest["schema"] == "my_drone.base1-armcomp-candidate.v1"
    assert manifest["reference_commit"].startswith(BASE1_COMMIT)
    assert manifest["scope"].startswith("4kg armcomp candidate")


def test_all_mismatches_are_intentional_or_accounted() -> None:
    """Every core-file mismatch must appear in INTENTIONAL_DIFFERENCES.

    If this test fails, a core file was changed without being documented
    in the intentional-differences table.  Either add it to the table with
    a change_reason, or revert the change.
    """
    manifest = build_manifest(ROOT, BASE1_REF)
    unaccounted = manifest.get("unaccounted_mismatches", [])
    assert unaccounted == [], (
        f"Unaccounted core-file changes: {unaccounted}. "
        f"Add them to INTENTIONAL_DIFFERENCES in build_base1_freeze_manifest.py."
    )


def test_intentional_change_count_matches() -> None:
    manifest = build_manifest(ROOT, BASE1_REF)
    mismatches = set(manifest["mismatches"])
    intentional_in_manifest = set(manifest.get("intentional_differences", {}).keys())
    assert intentional_in_manifest == mismatches & set(INTENTIONAL_DIFFERENCES.keys())


def test_compensation_defaults_are_all_off() -> None:
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
