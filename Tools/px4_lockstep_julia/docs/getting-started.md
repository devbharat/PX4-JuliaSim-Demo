# Getting started

This repo is typically used from inside a PX4 tree at:

```
PX4-Autopilot/Tools/px4_lockstep_julia
```

The helper scripts assume that layout (they walk up to the PX4 root and locate build artifacts).

## Prerequisites

- **PX4 lockstep SITL** built (produces `libpx4_lockstep`).
- **Julia 1.12** (see `Project.toml` `compat`). This is the version used in CI and
  validated in the lockstep workflows; older versions are not currently supported.
- Optional: Python 3 (only for plotting scripts under `scripts/`).

## Build PX4 lockstep SITL

From your PX4 root:

```bash
make px4_sitl_lockstep
```

The helper scripts expect to find the lockstep library at:

```
build/px4_sitl_lockstep/src/lib/px4_lockstep/libpx4_lockstep.(so|dylib)
```

If your build output is elsewhere, edit the example specs and update `px4.libpath`.

## Instantiate the Julia environment

From the PX4 root:

```bash
julia --project=Tools/px4_lockstep_julia -e 'using Pkg; Pkg.instantiate()'
```

## Run an end-to-end Iris mission

The easiest path is the run helper (it also regenerates `src/UORBGenerated.jl` when uORB headers change):

```bash
Tools/px4_lockstep_julia/scripts/run_iris_lockstep.sh
```

This produces `sim_log.csv` at the path configured in the spec (see `examples/specs/iris_lockstep.toml`,
which defaults to `Tools/sim_log.csv` from the PX4 root).
On Linux, update the `.dylib` extension in the example specs to `.so`.

Example output:

```
WARN  [px4] param not found: SYS_CTRL_ALLOC
INFO  [px4] px4_lockstep created
INFO  [px4] mission loaded: 8 items
INFO  [navigator] Executing Mission
INFO  [navigator] Climb to 12.0 meters above home
```

Plot the resulting log:

```bash
python Tools/px4_lockstep_julia/scripts/plot_sim_log.py --log Tools/sim_log.csv
```

Example output:

```
Samples: 7001
Duration: 70.00 s
Altitude: min -0.00 m, max 12.03 m
Horizontal speed: max 5.44 m/s
Mission progress: 7/8
Battery voltage: min 10.56 V, max 12.60 V
Battery current: max 64.14 A
Battery remaining: min 0.82
Saved plot to sim_plot.png
```

<img width="1800" height="1500" alt="sim_plot" src="https://github.com/user-attachments/assets/adc3ce6e-48ba-4d09-8159-56af8c11ea75" />


## Compare integrators (record + replay)

This helper records one PX4 live run, then replays the plant with multiple integrators:

```bash
Tools/px4_lockstep_julia/scripts/run_iris_integrator_compare.sh
```

Example output:

```
Iris integrator compare (record + replay)
  t_end=60.0s, dt_ap=0.004s, dt_wind=0.001s, dt_log=0.01s
  solvers: RK4, RK23, RK45
[compare_integrators_iris_mission] RECORD (PX4 live)
WARN  [px4] param not found: SYS_CTRL_ALLOC
INFO  [px4] px4_lockstep created
INFO  [px4] mission loaded: 8 items
INFO  [navigator] Executing Mission
INFO  [navigator] Climb to 12.0 meters above home
  recording_out: /Users/you/PX4-Autopilot/Tools/px4_lockstep_julia/examples/replay/out/iris_20260125_164710_tier0.jls
[compare_integrators_iris_mission] REPLAY (plant-only sweep)

Summary (max error vs reference replay)
solver       wall_s     pos_max     vel_max     att_max        ω_max     rotor_max       |ΔV|       |ΔI|
RK4           0.184      0.0063    0.000183    2.67e-06     1.34e-06      0.000859   3.68e-07    1.8e-05
RK23          0.235      0.0271    0.000807    6.49e-06      2.9e-06       0.00207   2.02e-06   9.95e-05
RK45          0.086    2.84e-06    3.58e-07    5.16e-08     7.77e-10      1.14e-06    1.1e-09   5.25e-08
Wrote summary CSV → /Users/you/PX4-Autopilot/Tools/px4_lockstep_julia/examples/replay/out/iris_20260125_164710_summary.csv
```

## Sysimage (optional, opt-in)

To speed up Julia startup, the helper scripts can build and use a sysimage.
Enable it explicitly:

```bash
PX4_LOCKSTEP_SYSIMAGE=1 Tools/px4_lockstep_julia/scripts/run_iris_lockstep.sh
```

The first run will be slower while the sysimage is built (often around a minute);
subsequent runs are faster. The sysimage is rebuilt automatically when Julia sources,
`Project.toml`, `Manifest.toml`, or `UORBGenerated.jl` change, so frequent edits will
trigger slow rebuilds.

Use this primarily for CI or analysis workflows that run the same build many times,
not for rapid edit‑run iteration during development.

If the sysimage build is skipped, install `PackageCompiler` in this environment:

```bash
julia --project=Tools/px4_lockstep_julia -e 'using Pkg; Pkg.add("PackageCompiler")'
```

## Configuration notes

The helper scripts assume the **standard PX4 tree layout**. You can pass a custom
spec path to override defaults; for custom library or mission paths, create a spec
that extends the built-in defaults under `src/Workflows/assets/aircraft/` and set
`px4.libpath` / `px4.mission_path` there.

## Troubleshooting

### “PX4 lockstep library not found”

If you are running **without** the helper scripts, set `px4.libpath` in your spec.

### uORB struct mismatch errors

If PX4’s uORB headers changed, regenerate `src/UORBGenerated.jl`:

```bash
Tools/px4_lockstep_julia/scripts/run_iris_lockstep.sh
```

The run scripts check timestamps and regenerate automatically when needed.
