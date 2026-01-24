# Getting started

This repo is typically used from inside a PX4 tree at:

```
PX4-Autopilot/Tools/px4_lockstep_julia
```

The helper scripts assume that layout (they walk up to the PX4 root and locate build artifacts).

## Prerequisites

- **PX4 lockstep SITL** built (produces `libpx4_lockstep`).
- **Julia 1.12** (see `Project.toml` `compat`).
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

If your build output is elsewhere, set `PX4_LOCKSTEP_LIB` explicitly.

## Instantiate the Julia environment

From the PX4 root:

```bash
julia --project=Tools/px4_lockstep_julia -e 'using Pkg; Pkg.instantiate()'
```

## Run an end-to-end Iris mission

The easiest path is the run helper (it also regenerates `src/UORBGenerated.jl` when uORB headers change):

```bash
Tools/px4_lockstep_julia/scripts/run_iris_lockstep.sh 70
```

This produces `sim_log.csv` in your current working directory.

## Sysimage (optional, opt-in)

To speed up Julia startup, the helper scripts can build and use a sysimage.
Enable it explicitly:

```bash
PX4_LOCKSTEP_SYSIMAGE=1 Tools/px4_lockstep_julia/scripts/run_iris_lockstep.sh 70
```

The first run will be slower while the sysimage is built (often around a minute);
subsequent runs are faster. The sysimage is rebuilt automatically when Julia sources,
`Project.toml`, `Manifest.toml`, or `UORBGenerated.jl` change, so frequent edits will
trigger slow rebuilds.

Use this primarily for CI or analysis workflows that run the same build many times,
not for rapid edit‑run iteration during development.

## Configuration notes

The helper scripts assume the **standard PX4 tree layout** and do not accept path overrides.
If you need custom paths (e.g., a different `libpx4_lockstep` location), run Julia directly
and set `PX4_LOCKSTEP_LIB` / `PX4_LOCKSTEP_MISSION` yourself (see below).

## Troubleshooting

### “PX4 lockstep library not found”

If you are running **without** the helper scripts, set `PX4_LOCKSTEP_LIB` to the built shared library:

```bash
export PX4_LOCKSTEP_LIB=/absolute/path/to/libpx4_lockstep.so
```

### uORB struct mismatch errors

If PX4’s uORB headers changed, regenerate `src/UORBGenerated.jl`:

```bash
Tools/px4_lockstep_julia/scripts/run_iris_lockstep.sh 1
```

The run scripts check timestamps and regenerate automatically when needed.
