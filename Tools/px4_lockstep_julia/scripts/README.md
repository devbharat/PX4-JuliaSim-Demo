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
