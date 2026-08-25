from __future__ import annotations

import ast
import os
from pathlib import Path
import subprocess


SCRIPTS = Path(__file__).resolve().parent
RUNNER = SCRIPTS / "run_with_cpu_role.sh"


def _affinity(
    role: str,
    enabled: str = "true",
    affinity_override: str = "0-11",
) -> set[int]:
    environment = os.environ.copy()
    environment.update(
        {
            "MY_DRONE_CPU_ISOLATION_ENABLED": enabled,
            "MY_DRONE_CPU_AFFINITY_OVERRIDE": affinity_override,
        }
    )
    result = subprocess.run(
        [
            "bash",
            str(RUNNER),
            role,
            "python3",
            "-c",
            "import os; print(sorted(os.sched_getaffinity(0)))",
        ],
        check=True,
        capture_output=True,
        text=True,
        env=environment,
    )
    return set(ast.literal_eval(result.stdout.strip()))


def test_role_mapping_reserves_four_low_numbered_pairs():
    assert _affinity("guardian") == {0, 1}
    assert _affinity("controller") == {2, 3}
    assert _affinity("arm") == {4, 5}
    assert _affinity("support") == {6, 7}
    assert _affinity("bulk") == {8, 9, 10, 11}


def test_twenty_cpu_formal_mapping_keeps_timing_roles_off_high_tail():
    affinity = "0-19"
    assert _affinity("guardian", affinity_override=affinity) == {0, 1}
    assert _affinity("controller", affinity_override=affinity) == {2, 3}
    assert _affinity("arm", affinity_override=affinity) == {4, 5}
    assert _affinity("support", affinity_override=affinity) == {6, 7}
    assert _affinity("bulk", affinity_override=affinity) == set(range(8, 20))


def test_noncontiguous_allowed_cpu_list_is_partitioned_without_overlap():
    affinity = "0,2,4,6,8,10,12,14,16,18"
    assert _affinity("guardian", affinity_override=affinity) == {0, 2}
    assert _affinity("controller", affinity_override=affinity) == {4, 6}
    assert _affinity("arm", affinity_override=affinity) == {8, 10}
    assert _affinity("support", affinity_override=affinity) == {12, 14}
    assert _affinity("bulk", affinity_override=affinity) == {16, 18}


def test_five_to_eight_cpu_machine_uses_single_core_safe_fallback():
    affinity = "2,4,6,8,10,12"
    assert _affinity("guardian", affinity_override=affinity) == {2}
    assert _affinity("controller", affinity_override=affinity) == {4}
    assert _affinity("arm", affinity_override=affinity) == {6}
    assert _affinity("support", affinity_override=affinity) == {8}
    assert _affinity("bulk", affinity_override=affinity) == {10, 12}


def test_fewer_than_five_cpus_preserves_parent_affinity():
    expected = set(os.sched_getaffinity(0))
    assert _affinity("guardian", affinity_override="0-3") == expected


def test_disabled_isolation_preserves_parent_affinity():
    expected = set(os.sched_getaffinity(0))
    assert _affinity("guardian", enabled="false") == expected


def test_formal_candidate_enables_isolation_without_threshold_changes():
    candidate = (SCRIPTS / "run_directional_workspace_acceptance_1p3kg.sh").read_text()
    assert 'MY_DRONE_CPU_ISOLATION_ENABLED="${MY_DRONE_CPU_ISOLATION_ENABLED:-true}"' in candidate
    assert 'ARM_DIRECT_XY_WATCHDOG_S' not in candidate
    assert 'ARM_DIRECT_XY_ENTRY_TIMEOUT_S' not in candidate


def test_launch_chain_assigns_each_runtime_role():
    backend = (SCRIPTS / "wsl_start_ros2_dds_noarm.sh").read_text()
    overlay = (SCRIPTS / "activate_base1_wrench_reallocator_overlay.sh").read_text()
    estimator = (SCRIPTS / "run_base1_readonly_estimator_overlay.sh").read_text()
    acceptance = (SCRIPTS / "run_directional_workspace_acceptance_4kg.sh").read_text()
    harness = (SCRIPTS / "test_ros2_dds_arm_flight_pty.py").read_text()
    assert '"${cpu_role_runner}" bulk' in backend
    assert '"${cpu_role_runner}" support' in overlay
    assert '"${cpu_role_runner}" guardian' in overlay
    assert overlay.count('"${cpu_role_runner}" bulk') >= 2
    assert '"${cpu_role_runner}" bulk' in estimator
    assert 'run_with_cpu_role.sh" bulk' in acceptance
    assert 'cpu_role_command("controller"' in harness
    assert 'cpu_role_command("arm"' in harness
