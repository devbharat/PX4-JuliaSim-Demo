# Record & replay

Record/replay exists to make simulation comparisons scientific.

Instead of comparing two closed-loop runs (which quickly diverge and cause PX4 to issue different commands), you:

1. Run PX4 in the loop once and record the **minimum** streams required to drive the plant deterministically.
2. Replay those exact streams open-loop while varying one thing (integrator settings, model changes, etc.).

## What gets recorded

The canonical workflow records a Tier‑0 snapshot set that is designed to be:

- small
- deterministic
- forward-evolvable via a schema version

Conceptually, Tier‑0 captures “everything the continuous plant needs between event boundaries”:

- autopilot commands
- wind samples
- scenario outputs (including faults)
- optional boundary-time derived outputs (battery telemetry, etc.)

The streams are versioned by `BUS_SCHEMA_VERSION` in `src/sim/Runtime/Bus.jl`.

## How to run it

From the PX4 root:

```bash
Tools/px4_lockstep_julia/scripts/run_iris_integrator_compare.sh
```

That script runs `examples/replay/iris_integrator_compare.jl`, which uses:

- `PX4Lockstep.Workflows.simulate_iris_mission(...; mode=:record)`
- `PX4Lockstep.Workflows.compare_integrators_iris_mission(...)`

Outputs are written under `examples/replay/out/`.

## Determinism expectations

If you replay the same recording with the same solver and configuration, you should get bitwise-identical results.

The usual failure modes when this breaks are:

- accidental RNG use inside an ODE RHS
- coupling that bypasses the bus (something reads mutable state instead of `PlantInput`)
- time represented as floating-point seconds instead of integer microseconds

Related docs:

- [`../components/record-replay.md`](../components/record-replay.md)
- [`../components/simulation-engine.md`](../components/simulation-engine.md)
