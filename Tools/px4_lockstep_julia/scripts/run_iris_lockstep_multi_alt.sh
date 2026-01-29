#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
REPO_ROOT=$(cd "${SCRIPT_DIR}/../../.." && pwd)

JULIA_DEPOT_PATH="${JULIA_DEPOT_PATH:-${REPO_ROOT}/Tools/px4_lockstep_julia/.julia_depot}"

UORB_HEADERS_DIR="${REPO_ROOT}/build/px4_sitl_lockstep/uORB/topics"
UORB_OUT="${REPO_ROOT}/Tools/px4_lockstep_julia/src/UORBGenerated.jl"
UORB_CODEGEN_SCRIPT="${REPO_ROOT}/Tools/px4_lockstep_julia/scripts/uorb_codegen.jl"
UORB_TOPICS="battery_status,vehicle_attitude,vehicle_local_position,vehicle_global_position,vehicle_angular_velocity,vehicle_land_detected,vehicle_status,vehicle_control_mode,actuator_armed,home_position,geofence_status,vehicle_torque_setpoint,vehicle_thrust_setpoint,actuator_motors,actuator_servos,vehicle_attitude_setpoint,vehicle_rates_setpoint,mission_result,trajectory_setpoint"

SPEC_FILES=(
  "${REPO_ROOT}/Tools/px4_lockstep_julia/examples/specs/iris_alt_0000m.toml"
  "${REPO_ROOT}/Tools/px4_lockstep_julia/examples/specs/iris_alt_1000m.toml"
  "${REPO_ROOT}/Tools/px4_lockstep_julia/examples/specs/iris_alt_2000m.toml"
  "${REPO_ROOT}/Tools/px4_lockstep_julia/examples/specs/iris_alt_3000m.toml"
)
if [[ $# -gt 1 ]]; then
  echo "Usage: $(basename "$0") [spec.toml]" >&2
  exit 2
fi
if [[ $# -eq 1 ]]; then
  SPEC_FILES=("$1")
fi

mkdir -p "${JULIA_DEPOT_PATH}"

if [[ ! -d "${UORB_HEADERS_DIR}" ]]; then
  echo "uORB headers not found at ${UORB_HEADERS_DIR}; skipping codegen" >&2
elif [[ ! -f "${UORB_OUT}" ]] || [[ "${UORB_CODEGEN_SCRIPT}" -nt "${UORB_OUT}" ]] || find "${UORB_HEADERS_DIR}" -type f -name '*.h' -newer "${UORB_OUT}" -print -quit | grep -q .; then
  echo "Generating UORBGenerated.jl from ${UORB_HEADERS_DIR}" >&2
  JULIA_DEPOT_PATH=${JULIA_DEPOT_PATH} \
    julia --project="${REPO_ROOT}/Tools/px4_lockstep_julia" \
      "${UORB_CODEGEN_SCRIPT}" \
      --headers "${UORB_HEADERS_DIR}" \
      --topics "${UORB_TOPICS}" \
      --out "${UORB_OUT}"
fi

LOG_PATHS=()
for spec_path in "${SPEC_FILES[@]}"; do
  label=$(basename "${spec_path}" .toml)
  label="${label#iris_alt_}"
  log_path="${REPO_ROOT}/sim_log_alt_${label}.csv"
  if [[ "${label}" == "$(basename "${spec_path}" .toml)" ]]; then
    log_path="${REPO_ROOT}/sim_log_alt_custom.csv"
  fi
  LOG_PATHS+=("${log_path}")

  echo "[multi-alt] spec=$(basename "${spec_path}") -> ${log_path}" >&2

  JULIA_DEPOT_PATH=${JULIA_DEPOT_PATH} \
    julia --project="${REPO_ROOT}/Tools/px4_lockstep_julia" \
      -e "using PX4Lockstep.Workflows; Workflows.simulate_iris_mission(spec_path=\"${spec_path}\", mode=:live)"
done

if [[ "${PLOT:-0}" != "0" ]]; then
  overlay_out="${REPO_ROOT}/sim_plot_overlay.png"
  traj_out="${REPO_ROOT}/sim_plot_trajectory_overlay.png"
  PY_ARGS=()
  for path in "${LOG_PATHS[@]}"; do
    PY_ARGS+=(--log "${path}")
  done
  python3 "${REPO_ROOT}/Tools/px4_lockstep_julia/scripts/plot_sim_log_overlay.py" \
    "${PY_ARGS[@]}" \
    --output "${overlay_out}" \
    --traj-output "${traj_out}"
  echo "Saved overlay plots to ${overlay_out} and ${traj_out}" >&2
fi
