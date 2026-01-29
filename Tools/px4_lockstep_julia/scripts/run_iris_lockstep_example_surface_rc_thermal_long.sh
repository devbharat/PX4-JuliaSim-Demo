#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
# shellcheck source=Tools/px4_lockstep_julia/scripts/_lockstep_common.sh
source "${SCRIPT_DIR}/_lockstep_common.sh"

SPEC_PATH="${REPO_ROOT}/Tools/px4_lockstep_julia/examples/specs/iris_example_3s2p_10ah_surface_rc_thermal_lockstep.toml"

"${SCRIPT_DIR}/run_iris_lockstep.sh" "${SPEC_PATH}"

echo "To plot results:"
echo "  python \"${SCRIPT_DIR}/plot_sim_log.py\" \\"
echo "    --log \"${REPO_ROOT}/Tools/sim_log.csv\" \\"
echo "    --output \"${REPO_ROOT}/Tools/sim_plot.png\" \\"
echo "    --battery-output \"${REPO_ROOT}/Tools/sim_battery_plot.png\""
