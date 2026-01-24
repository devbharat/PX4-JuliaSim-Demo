#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
# shellcheck source=Tools/px4_lockstep_julia/scripts/_lockstep_common.sh
source "${SCRIPT_DIR}/_lockstep_common.sh"

T_END_S=${1:-${IRIS_T_END_S:-20}}

ensure_uorb_codegen
ensure_sysimage

JULIA_DEPOT_PATH=${JULIA_DEPOT_PATH} \
run_julia --project="${REPO_ROOT}/Tools/px4_lockstep_julia" \
  -e "ENV[\"IRIS_T_END_S\"]=\"${T_END_S}\"; \
      ENV[\"PX4_LOCKSTEP_LIB\"]=\"${PX4_LOCKSTEP_LIB}\"; \
      ENV[\"PX4_LOCKSTEP_MISSION\"]=\"${PX4_LOCKSTEP_MISSION}\"; \
      include(\"${REPO_ROOT}/Tools/px4_lockstep_julia/examples/replay/iris_integrator_compare.jl\")"
