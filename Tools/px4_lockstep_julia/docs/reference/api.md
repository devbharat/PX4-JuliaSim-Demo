# Public API index

This repository has a fairly modular internal architecture (runtime engine, plant models, sources, record/replay, etc.).
That modularity is intentional, but most internal modules are *not* intended as stable entrypoints.

This page lists the **recommended public entrypoints** that are expected to remain stable.

## Stable entrypoints

### `Sim.simulate`

The single “canonical” entrypoint used by other workflows.

*Location*: `src/sim/API.jl`

Typical use:

```julia
eng = Sim.simulate(
    mode       = :live,  # :live | :record | :replay
    timeline   = timeline,
    plant0     = plant0,
    dynfun     = dynfun,
    integrator = integrator,
    autopilot  = autopilot,
    wind       = wind,
    scenario   = scenario,
    estimator  = estimator,
)
```

### `Sim.record_live_px4`

Convenience wrapper for a record run. Equivalent to `Sim.simulate(mode=:record, ...)`.

*Location*: `src/sim/API.jl`

Typical use:

```julia
eng = Sim.record_live_px4(
    timeline   = timeline,
    plant0     = plant0,
    dynfun     = dynfun,
    integrator = integrator,
    autopilot  = autopilot,
    wind       = wind,
    scenario   = scenario,
    recorder   = Sim.Recording.InMemoryRecorder(),
)
```

### `Sim.replay_recording`

Convenience wrapper for a replay run. Equivalent to `Sim.simulate(mode=:replay, ...)`.

*Location*: `src/sim/API.jl`

This expects you to construct replay sources explicitly (wind, scenario, autopilot) from traces.
See [`../components/record-replay.md`](../components/record-replay.md) for the current record/replay workflow.

### `Sim.Workflows.simulate_iris_mission`

End-to-end Iris mission workflow (high level wrapper).

*Location*: `src/sim/Workflows/Iris.jl`

This is the recommended starting point if you want “a working run” without assembling all components.

## Not considered stable

These are useful internal building blocks but are not yet promised as stable API:

- `Sim.Runtime.*` (engine internals, scheduler details)
- `Sim.Sources.*` (source protocol may evolve)
- `Sim.PlantModels.*` (plant RHS functors are still being iterated)
- `Sim.Logging.*` (log schema may change; see `CSV_SCHEMA_VERSION` in `Sim.Logging`)
