# PX4Lockstep.jl

Deterministic PX4 lockstep SITL **with the plant, scheduling, and logging owned by Julia**.

This repo has two layers:

1. **`PX4Lockstep`** — a small, dependency-light Julia wrapper around `libpx4_lockstep` (C ABI).
2. **`PX4Lockstep.Sim`** — a single-threaded, event-driven simulation framework (truth dynamics, environment, powertrain, scenarios, logs) designed to run PX4 *in the loop*.

The guiding idea: keep the C boundary minimal and deterministic, and build everything else (models + runtime semantics) as reviewable Julia code.

## What you can do with it

- Run PX4 SITL in **lockstep** with a deterministic Julia-owned plant, environment, powertrain, and contacts.
- Define aircraft, uORB boundaries, timelines, and missions in **TOML** (eg. [`Iris TOML`](src/Workflows/assets/aircraft/iris_default.toml)), then run them via clean workflows.
- Record a PX4 run once, then **replay the exact same inputs** to compare integrators/models without closed-loop divergence.
- Inject deterministic scenarios and faults (arm/mission, wind disturbances, motor/battery faults) and capture ordered logs.
- Sweep integrators or model variants in replay and export summary CSVs for regression tracking.
- Run standalone **verification problems** (analytic + invariants) without PX4 to catch numerical regressions.

## Quickstart (recommended: from a PX4 tree)

These scripts assume this repo lives at `PX4-Autopilot/Tools/px4_lockstep_julia`.

0) Ensure Julia is installed. If not, follow the install guide:

```
https://julialang.org/downloads/
```

1) Build lockstep SITL in your PX4 tree (example):

```bash
cd /path/to/PX4-Autopilot
make px4_sitl_lockstep
```

2) Instantiate the Julia environment:

```bash
julia --project=Tools/px4_lockstep_julia -e 'using Pkg; Pkg.instantiate()'
```

3) Run the Iris mission (auto-regenerates `src/UORBGenerated.jl` if needed):

```bash
Tools/px4_lockstep_julia/scripts/run_iris_lockstep.sh
```

This produces `sim_log.csv` at the path configured in the spec (defaults to the PX4 repo root).
On Linux, update the `.dylib` extension in the example specs to `.so`.

4) Do a **record → replay** integrator sweep:

```bash
Tools/px4_lockstep_julia/scripts/run_iris_integrator_compare.sh
```

Outputs are written under `Tools/px4_lockstep_julia/examples/replay/out/`.

## Architecture in one picture

![runtime_flowchart](https://github.com/user-attachments/assets/54fa6ff8-6389-4aa0-b0b5-f22c3393631f)

Key invariants:

- **Single authoritative run loop**: `PX4Lockstep.Sim.Runtime.Engine`.
- **Integer microsecond timebase** everywhere (`UInt64`) to avoid drift.
- No RNG inside the ODE RHS; randomness only at discrete boundaries.

## Documentation

- **Docs home:** `docs/README.md`
- **Getting started:** `docs/getting-started.md`
- **Architecture & design:** `docs/architecture.md`
- **Workflows (Iris, record/replay, verification):** `docs/notes/workflows.md`
- **Frames & sign conventions:** `docs/reference/conventions.md`
- **Reference API entrypoints:** `docs/reference/api.md`

## Standalone usage (not in a PX4 tree)

If you check this repo out standalone, create a small spec that extends the built-in
Iris defaults and sets `px4.libpath`:

```bash
cat > /tmp/iris_spec.toml <<EOF
schema_version = 1
extends = ["/path/to/Tools/px4_lockstep_julia/src/Workflows/assets/aircraft/iris_default.toml"]

[px4]
libpath = "/path/to/libpx4_lockstep.(so|dylib)"
EOF

julia --project=. -e 'using Pkg; Pkg.instantiate()'
julia --project=. -e 'using PX4Lockstep.Workflows; Workflows.simulate_iris_mission(spec_path="/tmp/iris_spec.toml", mode=:live)'
```

## Verification problems (integrator correctness)

Run deterministic reference problems without PX4:

```bash
julia --project=Tools/px4_lockstep_julia Tools/px4_lockstep_julia/examples/verification/sho.jl
julia --project=Tools/px4_lockstep_julia Tools/px4_lockstep_julia/examples/verification/pendulum.jl
```

More: `Tools/px4_lockstep_julia/examples/verification/README.md`.

## Plotting & analysis

Python helpers live in `scripts/`.

```bash
python Tools/px4_lockstep_julia/scripts/plot_sim_log.py --log sim_log.csv --output sim_plot.png
```

See `scripts/README.md` for setup and additional plotting options.

## Development

```bash
julia --project=Tools/px4_lockstep_julia -e 'using Pkg; Pkg.test()'
```

Formatting, static analysis, and uORB codegen are documented in `docs/development.md`.

## Notes / constraints

- `libpx4_lockstep` is not re-entrant by default; run Monte Carlo in separate processes, or pass `allow_multiple_handles=true` (unsafe).
- The world frame is **NED** to match PX4 conventions.
