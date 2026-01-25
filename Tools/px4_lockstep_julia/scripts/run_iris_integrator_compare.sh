#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
# shellcheck source=Tools/px4_lockstep_julia/scripts/_lockstep_common.sh
source "${SCRIPT_DIR}/_lockstep_common.sh"

ensure_uorb_codegen
ensure_sysimage

SPEC_PATH="${REPO_ROOT}/Tools/px4_lockstep_julia/examples/specs/iris_compare.toml"
if [[ $# -gt 1 ]]; then
  echo "Usage: $(basename "$0") [spec.toml]" >&2
  exit 2
fi
if [[ $# -eq 1 ]]; then
  SPEC_PATH="$1"
fi

JULIA_DEPOT_PATH=${JULIA_DEPOT_PATH} \
run_julia --project="${REPO_ROOT}/Tools/px4_lockstep_julia" \
  "${REPO_ROOT}/Tools/px4_lockstep_julia/examples/replay/iris_integrator_compare.jl" \
  "${SPEC_PATH}"
