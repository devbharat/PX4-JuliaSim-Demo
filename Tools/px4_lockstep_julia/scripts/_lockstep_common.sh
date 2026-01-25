#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
REPO_ROOT=$(cd "${SCRIPT_DIR}/../../.." && pwd)

detect_lib_ext() {
  local os_name
  os_name=$(uname)
  if [[ "${os_name}" == "Linux" ]]; then
    echo "so"
  else
    echo "dylib"
  fi
}

LIB_EXT="$(detect_lib_ext)"
SYS_EXT="${LIB_EXT}"
JULIA_DEPOT_PATH="${REPO_ROOT}/Tools/px4_lockstep_julia/.julia_depot"
UORB_HEADERS_DIR="${REPO_ROOT}/build/px4_sitl_lockstep/uORB/topics"
UORB_OUT="${REPO_ROOT}/Tools/px4_lockstep_julia/src/UORBGenerated.jl"
UORB_CODEGEN_SCRIPT="${REPO_ROOT}/Tools/px4_lockstep_julia/scripts/uorb_codegen.jl"
UORB_TOPICS="battery_status,vehicle_attitude,vehicle_local_position,vehicle_global_position,vehicle_angular_velocity,vehicle_land_detected,vehicle_status,vehicle_control_mode,actuator_armed,home_position,geofence_status,vehicle_torque_setpoint,vehicle_thrust_setpoint,actuator_motors,actuator_servos,vehicle_attitude_setpoint,vehicle_rates_setpoint,mission_result,trajectory_setpoint"

SYSIMAGE_DIR="${REPO_ROOT}/Tools/px4_lockstep_julia/.sysimage"
SYSIMAGE="${SYSIMAGE_DIR}/PX4Lockstep.${SYS_EXT}"
SYSIMAGE_STAMP="${SYSIMAGE}.stamp"
PROJECT_TOML="${REPO_ROOT}/Tools/px4_lockstep_julia/Project.toml"
MANIFEST_TOML="${REPO_ROOT}/Tools/px4_lockstep_julia/Manifest.toml"

hash_command() {
  if command -v sha256sum >/dev/null 2>&1; then
    echo "sha256sum"
    return 0
  fi
  if command -v shasum >/dev/null 2>&1; then
    echo "shasum -a 256"
    return 0
  fi
  return 1
}

hash_sysimage_inputs() {
  local hash_cmd
  if ! hash_cmd=$(hash_command); then
    echo "No SHA-256 tool found (sha256sum/shasum); cannot hash sysimage inputs." >&2
    return 1
  fi

  local -a files
  files=("${PROJECT_TOML}" "${MANIFEST_TOML}")
  if [[ -f "${UORB_OUT}" ]]; then
    files+=("${UORB_OUT}")
  fi

  local src_dir="${REPO_ROOT}/Tools/px4_lockstep_julia/src"
  if [[ -d "${src_dir}" ]]; then
    while IFS= read -r f; do
      files+=("${f}")
    done < <(find "${src_dir}" -type f -name '*.jl' ! -path "${UORB_OUT}" -print | LC_ALL=C sort)
  fi

  ${hash_cmd} "${files[@]}" | ${hash_cmd} | awk '{print $1}'
}

ensure_uorb_codegen() {
  mkdir -p "${JULIA_DEPOT_PATH}"

  if [[ ! -d "${UORB_HEADERS_DIR}" ]]; then
    echo "uORB headers not found at ${UORB_HEADERS_DIR}; skipping codegen" >&2
    return 0
  fi

  local regenerated=0
  if [[ ! -f "${UORB_OUT}" ]] || [[ "${UORB_CODEGEN_SCRIPT}" -nt "${UORB_OUT}" ]] || \
     find "${UORB_HEADERS_DIR}" -type f -name '*.h' -newer "${UORB_OUT}" -print -quit | grep -q .; then
    echo "Generating UORBGenerated.jl from ${UORB_HEADERS_DIR}" >&2
    JULIA_DEPOT_PATH=${JULIA_DEPOT_PATH} \
      julia --project="${REPO_ROOT}/Tools/px4_lockstep_julia" \
        "${UORB_CODEGEN_SCRIPT}" \
        --headers "${UORB_HEADERS_DIR}" \
        --topics "${UORB_TOPICS}" \
        --out "${UORB_OUT}"
    regenerated=1
  fi
  return ${regenerated}
}

ensure_sysimage() {
  if [[ "${PX4_LOCKSTEP_SYSIMAGE:-0}" != "1" ]]; then
    return 0
  fi

  local current_hash=""
  if ! current_hash=$(hash_sysimage_inputs); then
    echo "Hashing unavailable; rebuilding sysimage for correctness." >&2
    if ! "${SCRIPT_DIR}/build_sysimage.sh"; then
      echo "Skipping sysimage (falling back to plain Julia)." >&2
    fi
    return 0
  fi

  if [[ ! -f "${SYSIMAGE}" ]] || [[ ! -f "${SYSIMAGE_STAMP}" ]] || [[ "$(cat "${SYSIMAGE_STAMP}")" != "${current_hash}" ]]; then
    echo "Sysimage inputs changed; rebuilding sysimage." >&2
    if ! "${SCRIPT_DIR}/build_sysimage.sh"; then
      echo "Skipping sysimage (falling back to plain Julia)." >&2
    fi
  fi
}

run_julia() {
  if [[ "${PX4_LOCKSTEP_SYSIMAGE:-0}" == "1" ]] && [[ -f "${SYSIMAGE}" ]]; then
    julia -J "${SYSIMAGE}" "$@"
  else
    julia "$@"
  fi
}
