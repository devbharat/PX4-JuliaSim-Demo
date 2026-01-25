#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
# shellcheck source=Tools/px4_lockstep_julia/scripts/_lockstep_common.sh
source "${SCRIPT_DIR}/_lockstep_common.sh"

SPEC_PATH="${REPO_ROOT}/Tools/px4_lockstep_julia/examples/specs/iris_lockstep.toml"
if [[ $# -gt 1 ]]; then
  echo "Usage: $(basename "$0") [spec.toml]" >&2
  exit 2
fi
if [[ $# -eq 1 ]]; then
  SPEC_PATH="$1"
fi

ensure_uorb_codegen
ensure_sysimage

JULIA_DEPOT_PATH=${JULIA_DEPOT_PATH} \
run_julia --project="${REPO_ROOT}/Tools/px4_lockstep_julia" \
  -e "using PX4Lockstep.Workflows; \
      Workflows.simulate_iris_mission( \
        spec_path=\"${SPEC_PATH}\", \
        mode=:live)"
