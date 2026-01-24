#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
REPO_ROOT=$(cd "${SCRIPT_DIR}/../../.." && pwd)

T_END_S=${1:-${IRIS_T_END_S:-20}}

OS_NAME=$(uname)
LIB_EXT="dylib"
if [[ "${OS_NAME}" == "Linux" ]]; then
  LIB_EXT="so"
fi

LIB_PATH_DEFAULT="${REPO_ROOT}/build/px4_sitl_lockstep/src/lib/px4_lockstep/libpx4_lockstep.${LIB_EXT}"
MISSION_DEFAULT="${REPO_ROOT}/Tools/px4_lockstep_julia/examples/simple_mission.waypoints"
DEPOT_DEFAULT="${REPO_ROOT}/Tools/px4_lockstep_julia/.julia_depot"
UORB_HEADERS_DEFAULT="${REPO_ROOT}/build/px4_sitl_lockstep/uORB/topics"
UORB_OUT_DEFAULT="${REPO_ROOT}/Tools/px4_lockstep_julia/src/UORBGenerated.jl"

PX4_LOCKSTEP_LIB=${PX4_LOCKSTEP_LIB:-${LIB_PATH_DEFAULT}}
PX4_LOCKSTEP_MISSION=${PX4_LOCKSTEP_MISSION:-${MISSION_DEFAULT}}
JULIA_DEPOT_PATH=${JULIA_DEPOT_PATH:-${DEPOT_DEFAULT}}
UORB_HEADERS_DIR=${UORB_HEADERS_DIR:-${UORB_HEADERS_DEFAULT}}
UORB_OUT=${UORB_OUT:-${UORB_OUT_DEFAULT}}
UORB_TOPICS=${UORB_TOPICS:-"battery_status,vehicle_attitude,vehicle_local_position,vehicle_global_position,vehicle_angular_velocity,vehicle_land_detected,vehicle_status,vehicle_control_mode,actuator_armed,home_position,geofence_status,vehicle_torque_setpoint,vehicle_thrust_setpoint,actuator_motors,actuator_servos,vehicle_attitude_setpoint,vehicle_rates_setpoint,mission_result,trajectory_setpoint"}

# Create the depot on demand (we don't keep it in the repo).
mkdir -p "${JULIA_DEPOT_PATH}"

if [[ ! -d "${UORB_HEADERS_DIR}" ]]; then
  echo "uORB headers not found at ${UORB_HEADERS_DIR}; skipping codegen" >&2
elif [[ ! -f "${UORB_OUT}" ]] || find "${UORB_HEADERS_DIR}" -type f -name '*.h' -newer "${UORB_OUT}" -print -quit | grep -q .; then
  echo "Generating UORBGenerated.jl from ${UORB_HEADERS_DIR}" >&2
  JULIA_DEPOT_PATH=${JULIA_DEPOT_PATH} \
    julia --project="${REPO_ROOT}/Tools/px4_lockstep_julia" \
      "${REPO_ROOT}/Tools/px4_lockstep_julia/scripts/uorb_codegen.jl" \
      --headers "${UORB_HEADERS_DIR}" \
      --topics "${UORB_TOPICS}" \
      --out "${UORB_OUT}"
fi

IRIS_T_END_S=${T_END_S} \
PX4_LOCKSTEP_LIB=${PX4_LOCKSTEP_LIB} \
PX4_LOCKSTEP_MISSION=${PX4_LOCKSTEP_MISSION} \
JULIA_DEPOT_PATH=${JULIA_DEPOT_PATH} \
julia --project="${REPO_ROOT}/Tools/px4_lockstep_julia" \
  "${REPO_ROOT}/Tools/px4_lockstep_julia/examples/replay/iris_integrator_compare.jl"
