#!/usr/bin/env bash
set -euo pipefail

# Non-root CPU isolation for the 1.3 kg formal acceptance path.  Linux CPU
# affinity is inherited by child processes and their threads, so wrapping the
# launch roots also confines Gazebo/PX4 descendants without changing their
# command lines or requiring real-time scheduling privileges.

role="${1:-}"
if [[ -z "${role}" || "${role}" == "-h" || "${role}" == "--help" ]]; then
  echo "usage: $0 {bulk|support|arm|controller|guardian} command [args...]" >&2
  exit 2
fi
shift
if [[ "$#" -eq 0 ]]; then
  echo "run_with_cpu_role: missing command" >&2
  exit 2
fi

if [[ "${MY_DRONE_CPU_ISOLATION_ENABLED:-false}" != "true" ]] || \
   ! command -v taskset >/dev/null 2>&1; then
  exec "$@"
fi

affinity_spec="${MY_DRONE_CPU_AFFINITY_OVERRIDE:-}"
if [[ -z "${affinity_spec}" ]]; then
  affinity_spec="$(taskset -pc $$ 2>/dev/null | sed -E 's/.*: *//')"
fi

expand_cpu_list() {
  local spec="$1" part first last cpu
  local -a result=()
  IFS=',' read -r -a parts <<<"${spec}"
  for part in "${parts[@]}"; do
    if [[ "${part}" == *-* ]]; then
      first="${part%-*}"
      last="${part#*-}"
      for ((cpu=first; cpu<=last; cpu++)); do result+=("${cpu}"); done
    elif [[ -n "${part}" ]]; then
      result+=("${part}")
    fi
  done
  printf '%s\n' "${result[@]}"
}

mapfile -t cpus < <(expand_cpu_list "${affinity_spec}")
count="${#cpus[@]}"
# Do not assume the allowed CPUs are contiguous.  WSL/container affinity can
# expose lists such as 2,4,6-9; the order returned by taskset is the only
# stable input available here.
#
# On the formal 20-logical-CPU machine, the high-numbered 16-19 tail is the
# suspected slow-core group. Reserving that tail correlated with simultaneous
# guardian/controller service gaps. Prefer four low-numbered *pairs*: each Python ROS
# process then has a sibling CPU available for DDS/executor helper threads.
# Bulk Gazebo/PX4 work is strictly excluded from every reserved CPU.
#
# Nine allowed CPUs are the minimum for four pairs plus a non-overlapping bulk
# pool.  With 5-8 CPUs, safely fall back to one low-numbered CPU per critical
# role.  With fewer than five, retaining the parent scheduler is safer than
# forcing bulk work to overlap a watchdog CPU.
if (( count >= 9 )); then
  guardian_cpus="${cpus[0]},${cpus[1]}"
  controller_cpus="${cpus[2]},${cpus[3]}"
  arm_cpus="${cpus[4]},${cpus[5]}"
  support_cpus="${cpus[6]},${cpus[7]}"
  bulk_cpus="$(IFS=,; echo "${cpus[*]:8}")"
elif (( count >= 5 )); then
  guardian_cpus="${cpus[0]}"
  controller_cpus="${cpus[1]}"
  arm_cpus="${cpus[2]}"
  support_cpus="${cpus[3]}"
  bulk_cpus="$(IFS=,; echo "${cpus[*]:4}")"
else
  exec "$@"
fi

case "${role}" in
  guardian) selected="${guardian_cpus}" ;;
  controller) selected="${controller_cpus}" ;;
  arm) selected="${arm_cpus}" ;;
  support) selected="${support_cpus}" ;;
  bulk) selected="${bulk_cpus}" ;;
  *)
    echo "run_with_cpu_role: unknown role '${role}'" >&2
    exit 2
    ;;
esac

echo "CPU_ROLE_AFFINITY role=${role} cpus=${selected} pid=$$" >&2
exec taskset -c "${selected}" "$@"
