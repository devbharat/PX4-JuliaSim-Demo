# Python analysis helpers

This folder contains lightweight scripts for post-processing. The default log
output from the Julia sim is `sim_log.csv`.

## Setup

```bash
python3 -m venv .venv
source .venv/bin/activate
pip install -r requirements.txt
```

## Plotting

```bash
python Tools/px4_lockstep_julia/scripts/plot_sim_log.py --log sim_log.csv --output sim_plot.png
```

Add `--show` to open an interactive window.

## Integrator comparison plots

If the integrator comparison is run with replay logs enabled (`IRIS_LOG_DIR`),
summary and trajectory/error plots can be generated:

```bash
python Tools/px4_lockstep_julia/scripts/plot_integrator_compare.py \
  --summary Tools/px4_lockstep_julia/examples/replay/out/iris_YYYYMMDD_HHMMSS_summary.csv \
  --log-dir Tools/px4_lockstep_julia/examples/replay/out
```

## uORB Julia struct generation (experimental)

Generate Julia `struct` definitions from PX4's generated uORB headers:

```bash
julia --project=Tools/px4_lockstep_julia Tools/px4_lockstep_julia/scripts/uorb_codegen.jl \
  --headers build/px4_sitl_lockstep/uORB/topics \
  --topics "battery_status,vehicle_attitude" \
  --out Tools/px4_lockstep_julia/src/UORBGenerated.jl
```

Pass multiple `--topic` flags or a comma-separated `--topics` list.

## Iris lockstep run helper

Run the live Iris mission with uORB injection flags (defaults to 70s):

```bash
Tools/px4_lockstep_julia/scripts/run_iris_lockstep.sh 70
```

Override paths/flags by exporting `PX4_LOCKSTEP_LIB`, `PX4_LOCKSTEP_MISSION`,
`JULIA_DEPOT_PATH`, or any of the `PX4_LOCKSTEP_UORB_*` env vars.
