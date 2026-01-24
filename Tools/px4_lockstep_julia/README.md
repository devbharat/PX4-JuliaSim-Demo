# PX4Lockstep.jl

Deterministic PX4 lockstep SITL **with the plant, scheduling, and logging owned by Julia**.

This repo has two layers:

1. **`PX4Lockstep`** — a small, dependency-light Julia wrapper around `libpx4_lockstep` (C ABI).
2. **`PX4Lockstep.Sim`** — a single-threaded, event-driven simulation framework (truth dynamics, environment, powertrain, scenarios, logs) designed to run PX4 *in the loop*.

The guiding idea: keep the C boundary minimal and deterministic, and build everything else (models + runtime semantics) as reviewable Julia code.

## What you can do with it

- Run PX4 SITL in **lockstep** with a deterministic Julia plant.
- Record a PX4 run, then **replay the exact same inputs** to compare integrators/models without closed-loop divergence.
- Run standalone **verification problems** (analytic + invariants) to catch numerical regressions.

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
Tools/px4_lockstep_julia/scripts/run_iris_lockstep.sh 70
```

This produces `sim_log.csv` in your current directory.

4) (Highly recommended) Do a **record → replay** integrator sweep:

```bash
Tools/px4_lockstep_julia/scripts/run_iris_integrator_compare.sh 20
```

Outputs are written under `Tools/px4_lockstep_julia/examples/replay/out/`.

## Architecture in one picture

```mermaid
flowchart LR
  subgraph Discrete[Discrete-time boundary updates (event-driven)]
    Scn[Scenario] --> Bus[(SimBus)]
    Wind[Wind source] --> Bus
    Est[Estimator source] --> Bus
    AP[PX4 lockstep autopilot] --> Bus
  end

  subgraph Continuous[Continuous-time plant integration]
    Bus -->|sample-and-hold inputs| Int[Integrator]
    Int --> X[PlantState]
    X -->|derived outputs| Bus
  end

  Bus --> Log[Log sinks / recorder]
```

Key invariants:

- **Single authoritative run loop**: `PX4Lockstep.Sim.Runtime.Engine`.
- **Integer microsecond timebase** everywhere (`UInt64`) to avoid drift.
- No RNG inside the ODE RHS; randomness only at discrete boundaries.

## Documentation

- **Docs home:** `docs/README.md`
- **Getting started:** `docs/getting-started.md`
- **Architecture & design:** `docs/architecture.md`
- **Workflows (Iris, record/replay, verification):** `docs/workflows.md`
- **Frames & sign conventions:** `docs/reference/conventions.md`
- **Reference API entrypoints:** `docs/reference/api.md`

## Standalone usage (not in a PX4 tree)

If you check this repo out standalone, set the library path explicitly:

```bash
export PX4_LOCKSTEP_LIB=/path/to/libpx4_lockstep.(so|dylib)
export PX4_LOCKSTEP_MISSION=/path/to/simple_mission.waypoints

julia --project=. -e 'using Pkg; Pkg.instantiate()'
julia --project=. -e 'using PX4Lockstep.Sim; Sim.simulate_iris_mission(mode=:live)'
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
