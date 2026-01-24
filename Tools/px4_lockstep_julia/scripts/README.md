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

The helper assumes the standard PX4 tree layout. For custom library or mission
paths, run Julia directly and set `PX4_LOCKSTEP_LIB` / `PX4_LOCKSTEP_MISSION`
in your environment.

### Sysimage (optional, opt-in)

To speed up Julia startup, enable sysimage building and usage:

```bash
PX4_LOCKSTEP_SYSIMAGE=1 Tools/px4_lockstep_julia/scripts/run_iris_lockstep.sh 70
```

The first build can take about a minute. The sysimage is rebuilt automatically when
Julia sources, `Project.toml`, `Manifest.toml`, or `UORBGenerated.jl` change, so
active development will trigger slow rebuilds.

This is best for CI or analysis workflows that run many times on the same build,
not for rapid local iteration.

## Iris multi-altitude sweep helper

Run multiple missions at increasing home altitude offsets and write a CSV per run:

```bash
Tools/px4_lockstep_julia/scripts/run_iris_lockstep_multi_alt.sh 70 1000 4 .
```

Args: `T_END_S`, `ALT_STEP_M`, `COUNT`, `OUT_DIR`, `PARALLEL`, `PLOT`
(defaults: `70`, `1000`, `4`, repo root, `0`, `0`).

To generate overlay plots automatically, pass a nonzero `PLOT` arg:

```bash
Tools/px4_lockstep_julia/scripts/run_iris_lockstep_multi_alt.sh 70 1000 4 . 1 1
```

## Overlay plot helper

Overlay multiple logs (e.g. from the sweep) to visualize trends:

```bash
python Tools/px4_lockstep_julia/scripts/plot_sim_log_overlay.py \
  --log sim_log_alt_488m.csv \
  --log sim_log_alt_1488m.csv \
  --log sim_log_alt_2488m.csv \
  --log sim_log_alt_3488m.csv \
  --output sim_plot_overlay.png
```

Add `--traj-output` to also generate a trajectory/altitude overlay plot:

```bash
python Tools/px4_lockstep_julia/scripts/plot_sim_log_overlay.py \
  --log sim_log_alt_488m.csv \
  --log sim_log_alt_1488m.csv \
  --log sim_log_alt_2488m.csv \
  --log sim_log_alt_3488m.csv \
  --output sim_plot_overlay.png \
  --traj-output sim_plot_trajectory_overlay.png
```
