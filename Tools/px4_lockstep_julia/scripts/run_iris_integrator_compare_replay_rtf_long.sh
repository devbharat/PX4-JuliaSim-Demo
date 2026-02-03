#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
# shellcheck source=Tools/px4_lockstep_julia/scripts/_lockstep_common.sh
source "${SCRIPT_DIR}/_lockstep_common.sh"

SPEC_PATH="${REPO_ROOT}/Tools/px4_lockstep_julia/examples/specs/iris_example_3s2p_10ah_surface_rc_thermal_lockstep.toml"
OUT_DIR="${REPO_ROOT}/Tools/px4_lockstep_julia/examples/replay/out_long"
RUN_NAME="iris_long"

RECORDING_PATH="${OUT_DIR}/${RUN_NAME}_tier0.jls"
SUMMARY_CSV="${OUT_DIR}/${RUN_NAME}_summary.csv"
RTF_CSV="${OUT_DIR}/${RUN_NAME}_rtf.csv"
LOG_DIR="${OUT_DIR}/logs"

ensure_julia_deps
ensure_uorb_codegen
ensure_sysimage

mkdir -p "${OUT_DIR}"

if [[ ! -f "${RECORDING_PATH}" ]]; then
  echo "[record] PX4 live run to create Tier-0 recording"
  JULIA_DEPOT_PATH=${JULIA_DEPOT_PATH} \
  run_julia --project="${REPO_ROOT}/Tools/px4_lockstep_julia" \
    -e "using PX4Lockstep.Workflows; \
        Workflows.simulate_iris_mission( \
          spec_path=\"${SPEC_PATH}\", \
          mode=:record, \
          recording_out=\"${RECORDING_PATH}\")"
fi

echo "[replay] Integrator sweep (replay-only; no PX4 calls)"
JULIA_DEPOT_PATH=${JULIA_DEPOT_PATH} \
run_julia --project="${REPO_ROOT}/Tools/px4_lockstep_julia" \
  -e "using PX4Lockstep.Workflows; \
      using PX4Lockstep.Sim.Aircraft; \
      spec_path=\"${SPEC_PATH}\"; \
      rec_path=\"${RECORDING_PATH}\"; \
      rows = compare_integrators_iris_mission( \
        recording_in=rec_path, \
        spec_path=spec_path, \
        out_csv=\"${SUMMARY_CSV}\", \
        log_dir=\"${LOG_DIR}\", \
        log_prefix=\"${RUN_NAME}\", \
        print_table=true); \
      spec = Aircraft.load_spec(spec_path; strict=true); \
      t_end = spec.timeline.t_end_s; \
      open(\"${RTF_CSV}\", \"w\") do io; \
        println(io, \"solver,wall_s,rtf\"); \
        for r in rows; \
          rtf = t_end / r.wall_s; \
          println(io, string(r.solver), \",\", r.wall_s, \",\", rtf); \
        end; \
      end; \
      println(\"Wrote RTF CSV → ${RTF_CSV}\");"

echo "To plot RTF bar chart:"
echo "  MPLCONFIGDIR=/tmp/mplconfig python - <<'PY'"
echo "  import csv"
echo "  import matplotlib.pyplot as plt"
echo "  from pathlib import Path"
echo "  rows = []"
echo "  with open('${RTF_CSV}') as f:"
echo "      r = csv.DictReader(f)"
echo "      rows = list(r)"
echo "  solvers = [row['solver'] for row in rows]"
echo "  rtf = [float(row['rtf']) for row in rows]"
echo "  fig, ax = plt.subplots(figsize=(5.0, 3.2), dpi=150)"
echo "  ax.bar(solvers, rtf, color='tab:green', alpha=0.85)"
echo "  ax.set_ylabel('Realtime factor (sim_s / wall_s)')"
echo "  ax.set_title('Replay-only integrator RTF (no PX4 calls)')"
echo "  ax.grid(True, axis='y', alpha=0.3)"
echo "  fig.tight_layout()"
echo "  out = Path('Tools/px4_lockstep_julia/docs/Report/Latex/figs/replay_rtf_long.png')"
echo "  fig.savefig(out)"
echo "  print(f'Saved {out}')"
echo "  PY"
