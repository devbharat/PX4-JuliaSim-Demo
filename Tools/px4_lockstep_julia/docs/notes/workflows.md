# Workflows

This page collects practical, task-oriented workflows.

## Run Iris live (PX4 in the loop)

From your PX4 root:

```bash
Tools/px4_lockstep_julia/scripts/run_iris_lockstep.sh
```

This helper:

1. Locates `libpx4_lockstep` in the standard PX4 build output.
2. Regenerates `src/UORBGenerated.jl` when uORB headers have changed.
3. Optionally builds/uses a sysimage when `PX4_LOCKSTEP_SYSIMAGE=1`.
4. Runs `PX4Lockstep.Workflows.simulate_iris_mission(...; mode=:live)`.

`simulate_iris_mission` requires an explicit spec via `spec_path` or `spec_name`.
The helper uses `examples/specs/iris_lockstep.toml`, which extends the built-in
defaults in `src/Workflows/assets/aircraft/iris_default.toml`.

Note: sysimage builds can take around a minute the first time, and any Julia code
change triggers a rebuild. It’s intended for CI or analysis runs with many repeats,
not for rapid edit‑run iteration.

Output:

- `sim_log.csv` at the path configured in the spec (see `examples/specs/iris_lockstep.toml`).

## Record → replay integrator sweep (recommended)

Closed-loop “run the sim twice and compare” is often misleading once the trajectories diverge and PX4 issues different commands.

Instead, this workflow:

1. Runs one baseline Iris mission with PX4 in the loop.
2. Records the minimum Tier‑0 streams needed for deterministic replay.
3. Replays those exact streams open-loop while sweeping integrators.

Run:

```bash
Tools/px4_lockstep_julia/scripts/run_iris_integrator_compare.sh
```

This helper uses `examples/specs/iris_compare.toml`, which extends the built-in defaults.

Outputs:

- Summary CSVs under `Tools/px4_lockstep_julia/examples/replay/out/`
- Optional per-integrator replay logs if you pass `log_dir` to the workflow.

See also: `Tools/px4_lockstep_julia/examples/replay/README.md`.

## Determinism check (replay same integrator repeatedly)

```bash
julia --project=Tools/px4_lockstep_julia \
  Tools/px4_lockstep_julia/examples/replay/iris_integrator_determinism.jl \
  RK4 3 /path/to/spec.toml
```

## Verification problems (no PX4 required)

These are deterministic reference problems (analytic solutions and/or invariants) intended to catch numerical regressions and validate integrator behavior.

```bash
julia --project=Tools/px4_lockstep_julia Tools/px4_lockstep_julia/examples/verification/sho.jl
julia --project=Tools/px4_lockstep_julia Tools/px4_lockstep_julia/examples/verification/pendulum.jl
```

More scripts and details: `Tools/px4_lockstep_julia/examples/verification/README.md`.

## Plotting

The default Iris run writes `sim_log.csv` (path from the spec). To plot:

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
julia --project=Tools/px4_lockstep_julia \
  -e 'using PX4Lockstep.Workflows; Workflows.simulate_iris_mission(spec_path="/path/to/spec.toml", mode=:live)'
```

Your spec must set `px4.libpath` for live/record runs (and `px4.mission_path` for live PX4 missions).

To run a custom spec, pass `spec_path`:

```bash
julia --project=Tools/px4_lockstep_julia \
  -e 'using PX4Lockstep.Workflows; Workflows.simulate_iris_mission(spec_path="path/to/spec.toml", mode=:live)'
```

To use the built-in Iris default, pass `spec_name` explicitly:

```bash
julia --project=Tools/px4_lockstep_julia \
  -e 'using PX4Lockstep.Workflows; Workflows.simulate_iris_mission(spec_name=:iris_default, mode=:live)'
```
```

If you’re using this repo standalone (not inside a PX4 tree), use `--project=.` and set `px4.libpath` in your spec.
