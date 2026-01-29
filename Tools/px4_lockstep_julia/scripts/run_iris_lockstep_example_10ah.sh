#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
# shellcheck source=Tools/px4_lockstep_julia/scripts/_lockstep_common.sh
source "${SCRIPT_DIR}/_lockstep_common.sh"

SPEC_PATH="${REPO_ROOT}/Tools/px4_lockstep_julia/examples/specs/iris_example_8s_10ah_lockstep.toml"

exec "${SCRIPT_DIR}/run_iris_lockstep.sh" "${SPEC_PATH}"
