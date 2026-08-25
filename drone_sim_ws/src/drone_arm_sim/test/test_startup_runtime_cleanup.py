from pathlib import Path


ROOT = Path(__file__).resolve().parents[3]


def test_estimator_restart_terminates_ros2_process_group_and_orphan_child():
    source = (
        ROOT / "scripts/run_base1_readonly_estimator_overlay.sh"
    ).read_text(encoding="utf-8")

    assert "stop_previous_estimator" in source
    assert 'kill -TERM -- "-${pgid}"' in source
    assert "estimator_child_pattern=" in source
    assert 'pkill -TERM -f "${estimator_child_pattern}"' in source
    assert "base1_arm_coupling_estimator_100hz" in source
    assert '"${#estimator_children[@]}" -eq 1' in source


def test_estimator_readiness_uses_one_persistent_dds_sample_gate():
    source = (
        ROOT / "scripts/run_base1_readonly_estimator_overlay.sh"
    ).read_text(encoding="utf-8")

    launch_index = source.index("setsid ros2 run drone_arm_sim arm_coupling_monitor")
    readiness = source[launch_index:]
    assert "wait_base1_ros_samples.py" in readiness
    assert "--coupling-samples 3" in readiness
    assert "if ros2 topic list" not in readiness
    assert "timeout \"${startup_sample_timeout_s}\" ros2 topic echo" not in readiness


def test_arm_startup_uses_persistent_joint_sample_gate_for_default_topic():
    source = (ROOT / "scripts/wsl_start_ros2_dds_noarm.sh").read_text(
        encoding="utf-8"
    )

    assert "arm_joint_sample_args=(--joint-samples 3" in source
    assert "wait_base1_ros_samples.py" in source
    assert "ARM_INIT_RETRY_TIMEOUT_S" in source
    # A custom diagnostic topic remains supported, but the old twenty-process
    # retry loop must not return for the normal /joint_states path.
    arm_block = source[source.index('arm_joint_state_topic='):]
    assert "for _ in $(seq 1 20)" not in arm_block


def test_ground_arm_initialisation_bypasses_airborne_xy_ownership_handshake():
    source = (ROOT / "scripts/wsl_start_ros2_dds_noarm.sh").read_text(
        encoding="utf-8"
    )
    command = "ros2 run drone_arm_sim arm_preset_control"
    command_index = source.index(command)
    prefix = source[max(0, command_index - 160):command_index]
    assert "ARM_DIRECT_XY_OWNERSHIP=false" in prefix


def test_overlay_saved_pid_cleanup_validates_command_and_kills_process_group():
    source = (
        ROOT / "scripts/activate_base1_wrench_reallocator_overlay.sh"
    ).read_text(encoding="utf-8")

    assert "stop_saved_ros2_session" in source
    assert '[[ "${command_line}" == *"${required_fragment}"* ]]' in source
    assert 'kill -TERM -- "-${pgid}"' in source
    assert "ros2 run drone_arm_sim base1_wrench_reallocator" in source
    assert "ros2 run drone_arm_sim cartesian_arm_velocity_control" in source


def test_external_guardian_uses_the_current_workspace_source():
    source = (
        ROOT / "scripts/activate_base1_wrench_reallocator_overlay.sh"
    ).read_text(encoding="utf-8")

    expected = (
        '${workspace_dir}/src/px4_ros2_control/'
        'px4_ros2_control/direct_xy_guardian.py'
    )
    assert f'guardian_source="{expected}"' in source
    assert '[[ ! -f "${guardian_source}" ]]' in source
    assert 'setsid python3 "${guardian_source}"' in source


def test_clean_restart_targets_estimator_child_and_removes_stale_pidfiles():
    source = (ROOT / "scripts/wsl_start_ros2_dds_noarm.sh").read_text(
        encoding="utf-8"
    )

    assert "/[a]rm_coupling_monitor .*__node:=base1_arm_coupling_estimator_100hz" in source
    for name in (
        "agent.pid",
        "gazebo.pid",
        "px4.pid",
        "base1_estimator.pid",
        "base1_reallocator.pid",
        "base1_overlay_motor.pid",
        "cartesian_velocity.pid",
    ):
        assert f'"${{runtime_dir}}/{name}"' in source
