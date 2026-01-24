# Workflows

This page collects the practical “how do I…” tasks.

## Run Iris live (PX4 in the loop)

From your PX4 root:

```bash
Tools/px4_lockstep_julia/scripts/run_iris_lockstep.sh 70
```

This helper:

1. Locates `libpx4_lockstep` in the standard PX4 build output.
2. Regenerates `src/UORBGenerated.jl` when uORB headers have changed.
3. Optionally builds/uses a sysimage when `PX4_LOCKSTEP_SYSIMAGE=1`.
4. Runs `PX4Lockstep.Sim.Workflows.simulate_iris_mission(...; mode=:live)`.

Note: sysimage builds can take around a minute the first time, and any Julia code
change triggers a rebuild. It’s intended for CI or analysis runs with many repeats,
not for rapid edit‑run iteration.

Output:

- `sim_log.csv` in your current directory.

## Record → replay integrator sweep (recommended)

Closed-loop “run the sim twice and compare” is often misleading once the trajectories diverge and PX4 issues different commands.

Instead, this workflow:

1. Runs one baseline Iris mission with PX4 in the loop.
2. Records the minimum Tier‑0 streams needed for deterministic replay.
3. Replays those exact streams open-loop while sweeping integrators.

Run:

```bash
Tools/px4_lockstep_julia/scripts/run_iris_integrator_compare.sh 20
```

Outputs:

- Summary CSVs under `Tools/px4_lockstep_julia/examples/replay/out/`
- Optional per-integrator replay logs if you set `IRIS_LOG_DIR` (see below).

Useful environment variables:

- `IRIS_T_END_S`: duration (seconds)
- `IRIS_SWEEP_SOLVERS`: comma-separated solver list (e.g. `RK4,RK23,RK45`)
- `IRIS_LOG_DIR`: write replay logs for plotting
- `IRIS_LOG_PREFIX`: prefix for replay log filenames

See also: `Tools/px4_lockstep_julia/examples/replay/README.md`.

## Determinism check (replay same integrator repeatedly)

```bash
PX4_LOCKSTEP_MISSION=Tools/px4_lockstep_julia/examples/simple_mission.waypoints \
IRIS_DETERMINISM_SOLVER=RK4 IRIS_DETERMINISM_N=3 \
IRIS_LOG_DIR=Tools/px4_lockstep_julia/examples/replay/out \
  julia --project=Tools/px4_lockstep_julia \
    Tools/px4_lockstep_julia/examples/replay/iris_integrator_determinism.jl
```

## Verification problems (no PX4 required)

These are deterministic reference problems (analytic solutions and/or invariants) intended to catch numerical regressions and validate integrator behavior.

```bash
julia --project=Tools/px4_lockstep_julia Tools/px4_lockstep_julia/examples/verification/sho.jl
julia --project=Tools/px4_lockstep_julia Tools/px4_lockstep_julia/examples/verification/pendulum.jl
```

More scripts and details: `Tools/px4_lockstep_julia/examples/verification/README.md`.

## Plotting

The default Iris run writes `sim_log.csv`. To plot:

```bash
python Tools/px4_lockstep_julia/scripts/plot_sim_log.py \
  --log sim_log.csv \
  --output sim_plot.png
```

For integrator sweep plots:

```bash
python Tools/px4_lockstep_julia/scripts/plot_integrator_compare.py \
  --summary Tools/px4_lockstep_julia/examples/replay/out/iris_YYYYMMDD_HHMMSS_summary.csv \
  --log-dir Tools/px4_lockstep_julia/examples/replay/out
```

Setup instructions: `Tools/px4_lockstep_julia/scripts/README.md`.

## Running without the helper scripts

You can call the workflow directly from Julia (useful for debugging):

```bash
IRIS_T_END_S=20 \
PX4_LOCKSTEP_LIB=/path/to/libpx4_lockstep.(so|dylib) \
PX4_LOCKSTEP_MISSION=Tools/px4_lockstep_julia/examples/simple_mission.waypoints \
  julia --project=Tools/px4_lockstep_julia \
    -e 'using PX4Lockstep.Sim; Sim.simulate_iris_mission(mode=:live)'
```

If you’re using this repo standalone (not inside a PX4 tree), use `--project=.` and set `PX4_LOCKSTEP_LIB` explicitly.
