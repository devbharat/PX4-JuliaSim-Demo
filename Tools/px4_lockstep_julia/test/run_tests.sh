#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
LOCKSTEP_SCRIPTS_DIR=$(cd "${SCRIPT_DIR}/../scripts" && pwd)
# shellcheck source=Tools/px4_lockstep_julia/scripts/_lockstep_common.sh
source "${LOCKSTEP_SCRIPTS_DIR}/_lockstep_common.sh"

# Optional: regen uORBGenerated.jl if PX4 headers exist.
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
  UNIT_LOGS=()
  TIER_LOGS=()
  UNIT_PIDS=()
  TIER_PIDS=()

  # Unit shards in parallel.
  if [[ "${RUN_UNIT}" -eq 1 ]]; then
    for group in "${UNIT_GROUPS[@]}"; do
      log="${LOG_DIR}/unit_${group}.log"
      UNIT_LOGS+=("${log}")
      run_julia_test "unit" "" "${group}" >"${log}" 2>&1 &
      UNIT_PIDS+=("$!")
    done
  fi

  # All integration tiers in parallel.
  if [[ "${RUN_INTEGRATION}" -eq 1 ]]; then
    for tier in "${TIER_LIST[@]}"; do
      log="${LOG_DIR}/integration_${tier}.log"
      TIER_LOGS+=("${log}")
      run_julia_test "integration" "${tier}" "" >"${log}" 2>&1 &
      TIER_PIDS+=("$!")
    done
  fi

  for pid in "${UNIT_PIDS[@]}"; do
    if ! wait "${pid}"; then
      fail=1
    fi
  done
  for pid in "${TIER_PIDS[@]}"; do
    if ! wait "${pid}"; then
      fail=1
    fi
  done

  # Dump logs after completion so output is readable (no interleaving).
  for log in "${UNIT_LOGS[@]}"; do
    cat "${log}"
  done
  for log in "${TIER_LOGS[@]}"; do
    cat "${log}"
  done

  rm -rf "${LOG_DIR}"
else
  # Sequential: unit tests in one process, integration tiers in separate processes.
  if [[ "${RUN_UNIT}" -eq 1 ]]; then
    if ! run_julia_test "unit" "" ""; then
      fail=1
    fi
  fi

  if [[ "${RUN_INTEGRATION}" -eq 1 ]]; then
    for tier in "${TIER_LIST[@]}"; do
      if ! run_julia_test "integration" "${tier}" ""; then
        fail=1
      fi
    done
  fi
fi

exit ${fail}
