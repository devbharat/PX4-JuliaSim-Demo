#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
LOCKSTEP_SCRIPTS_DIR=$(cd "${SCRIPT_DIR}/../scripts" && pwd)
# shellcheck source=Tools/px4_lockstep_julia/scripts/_lockstep_common.sh
source "${LOCKSTEP_SCRIPTS_DIR}/_lockstep_common.sh"

# Optional: regen uORBGenerated.jl if PX4 headers exist.
ensure_julia_deps
ensure_uorb_codegen

# Prepare sysimage: rebuild if PX4_LOCKSTEP_SYSIMAGE=1, otherwise auto-use if current.
prepare_sysimage

# Default to PX4 root build path if not provided.
if [[ -z "${PX4_LOCKSTEP_LIB:-}" ]]; then
  export PX4_LOCKSTEP_LIB="${REPO_ROOT}/build/px4_sitl_lockstep/src/lib/px4_lockstep/libpx4_lockstep.${LIB_EXT}"
fi

TEST_FILE="${REPO_ROOT}/Tools/px4_lockstep_julia/test/runtests.jl"
PROJECT="${REPO_ROOT}/Tools/px4_lockstep_julia"

# Default: run everything sequentially in separate processes for integration tiers.
# Use --parallel to run unit shards + integration tiers in parallel.
PARALLEL=0
for arg in "$@"; do
  case "${arg}" in
    --parallel)
      PARALLEL=1
      ;;
    *)
      echo "Unknown argument: ${arg}" >&2
      exit 1
      ;;
  esac
done

fail=0

color_enabled() {
  [[ -t 1 ]] && [[ -z "${NO_COLOR:-}" ]]
}

if color_enabled; then
  COLOR_RED=$'\033[0;31m'
  COLOR_GREEN=$'\033[0;32m'
  COLOR_YELLOW=$'\033[0;33m'
  COLOR_RESET=$'\033[0m'
else
  COLOR_RED=""
  COLOR_GREEN=""
  COLOR_YELLOW=""
  COLOR_RESET=""
fi

print_status() {
  local name="$1"
  local status="$2"
  if [[ "${status}" -eq 0 ]]; then
    echo "${COLOR_GREEN}PASS${COLOR_RESET} ${name}"
  else
    echo "${COLOR_RED}FAIL${COLOR_RESET} ${name}"
  fi
}

REQUEST_GROUP="${PX4_LOCKSTEP_TEST_GROUP:-all}"
case "${REQUEST_GROUP}" in
  all)
    RUN_UNIT=1
    RUN_INTEGRATION=1
    ;;
  unit)
    RUN_UNIT=1
    RUN_INTEGRATION=0
    ;;
  integration)
    RUN_UNIT=0
    RUN_INTEGRATION=1
    ;;
  *)
    echo "Unknown PX4_LOCKSTEP_TEST_GROUP=${REQUEST_GROUP}. Expected: all|unit|integration." >&2
    exit 1
    ;;
esac

split_csv() {
  local raw="$1"
  local var="$2"
  local -a parts trimmed
  IFS=',' read -r -a parts <<<"${raw}"
  trimmed=()
  for part in "${parts[@]}"; do
    part="${part#"${part%%[![:space:]]*}"}"
    part="${part%"${part##*[![:space:]]}"}"
    if [[ -n "${part}" ]]; then
      trimmed+=("${part}")
    fi
  done
  eval "${var}=(\"\${trimmed[@]}\")"
}

TIER_LIST=(tier0 tier1 tier2 tier3 tier4 tier5)
if [[ -n "${PX4_LOCKSTEP_INTEGRATION_FILTER:-}" && "${PX4_LOCKSTEP_INTEGRATION_FILTER}" != "all" ]]; then
  split_csv "${PX4_LOCKSTEP_INTEGRATION_FILTER}" TIER_LIST
fi

UNIT_GROUPS=(verification runtime aircraft battery)
if [[ -n "${PX4_LOCKSTEP_UNIT_GROUP:-}" && "${PX4_LOCKSTEP_UNIT_GROUP}" != "all" ]]; then
  split_csv "${PX4_LOCKSTEP_UNIT_GROUP}" UNIT_GROUPS
fi

run_julia_test() {
  local group="$1"
  local filter="${2:-}"
  local unit_group="${3:-}"
  if [[ -n "${filter}" ]]; then
    JULIA_DEPOT_PATH="${JULIA_DEPOT_PATH}" \
    PX4_LOCKSTEP_TEST_GROUP="${group}" \
    PX4_LOCKSTEP_INTEGRATION_FILTER="${filter}" \
    run_julia --startup-file=no --project="${PROJECT}" "${TEST_FILE}"
  elif [[ -n "${unit_group}" ]]; then
    JULIA_DEPOT_PATH="${JULIA_DEPOT_PATH}" \
    PX4_LOCKSTEP_TEST_GROUP="${group}" \
    PX4_LOCKSTEP_UNIT_GROUP="${unit_group}" \
    run_julia --startup-file=no --project="${PROJECT}" "${TEST_FILE}"
  else
    JULIA_DEPOT_PATH="${JULIA_DEPOT_PATH}" \
    PX4_LOCKSTEP_TEST_GROUP="${group}" \
    run_julia --startup-file=no --project="${PROJECT}" "${TEST_FILE}"
  fi
}

if [[ "${PARALLEL}" -eq 1 ]]; then
  LOG_DIR=$(mktemp -d)
  JOB_NAMES=()
  JOB_LOGS=()
  JOB_PIDS=()

  # Unit shards in parallel.
  if [[ "${RUN_UNIT}" -eq 1 ]]; then
    for group in "${UNIT_GROUPS[@]}"; do
      log="${LOG_DIR}/unit_${group}.log"
      JOB_NAMES+=("unit:${group}")
      JOB_LOGS+=("${log}")
      run_julia_test "unit" "" "${group}" >"${log}" 2>&1 &
      JOB_PIDS+=("$!")
    done
  fi

  # All integration tiers in parallel.
  if [[ "${RUN_INTEGRATION}" -eq 1 ]]; then
    for tier in "${TIER_LIST[@]}"; do
      log="${LOG_DIR}/integration_${tier}.log"
      JOB_NAMES+=("integration:${tier}")
      JOB_LOGS+=("${log}")
      run_julia_test "integration" "${tier}" "" >"${log}" 2>&1 &
      JOB_PIDS+=("$!")
    done
  fi

  # Dump logs after completion so output is readable (no interleaving).
  JOB_STATUS=()
  for i in "${!JOB_PIDS[@]}"; do
    if wait "${JOB_PIDS[$i]}"; then
      JOB_STATUS[$i]=0
    else
      JOB_STATUS[$i]=1
      fail=1
    fi
  done
  for log in "${JOB_LOGS[@]}"; do
    cat "${log}"
  done
  for i in "${!JOB_NAMES[@]}"; do
    print_status "${JOB_NAMES[$i]}" "${JOB_STATUS[$i]}"
  done

  rm -rf "${LOG_DIR}"
else
  # Sequential: unit tests in one process, integration tiers in separate processes.
  if [[ "${RUN_UNIT}" -eq 1 ]]; then
    if ! run_julia_test "unit" "" ""; then
      fail=1
      print_status "unit" 1
    else
      print_status "unit" 0
    fi
  fi

  if [[ "${RUN_INTEGRATION}" -eq 1 ]]; then
    for tier in "${TIER_LIST[@]}"; do
      if ! run_julia_test "integration" "${tier}" ""; then
        fail=1
        print_status "integration:${tier}" 1
      else
        print_status "integration:${tier}" 0
      fi
    done
  fi
fi

if [[ "${fail}" -eq 0 ]]; then
  echo "${COLOR_GREEN}ALL TESTS PASSED${COLOR_RESET}"
else
  echo "${COLOR_RED}TESTS FAILED${COLOR_RESET}"
fi

exit ${fail}
