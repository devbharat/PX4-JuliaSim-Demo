# Simulation Engine (Runtime.Engine)

> **Canonical engine:** `PX4Lockstep.Sim.Runtime.Engine`

## Role

`src/sim/Runtime/Engine.jl` implements the single authoritative hybrid simulation loop.

It is responsible for:

- traversing the event timeline (autopilot/wind/scenario/log/optional phys)
- enforcing a canonical boundary ordering
- stepping discrete sources (live or replay)
- integrating the continuous plant state over each interval
- recording/logging deterministic snapshots

The engine runs in one of three modes:

- **Live**: PX4-in-the-loop
- **Record**: live + record Tier-0 streams
- **Replay**: open-loop plant replay for integrator comparisons

## Key decisions and rationale

- **Microsecond clock (`UInt64`)**: all scheduling and all step subdivision is derived from integer microseconds. This prevents long-run drift and makes lockstep timestamps exact.

- **Boundary semantics define input changes**: the plant input is held constant on `[t_k, t_{k+1})` and can only change at event boundaries.

- **Enforced canonical ordering** (see `Runtime/BoundaryProtocol.jl`): scenario → wind → derived outputs → estimator → telemetry → autopilot → plant discontinuities → log/record → integrate.

- **Bus-driven orchestration**: sources publish into `SimBus`; the plant consumes a held `PlantInput` derived from the bus. This is the core mechanism enabling record/replay and independent submodel replay.

- **Fixed-step is a configuration, not a separate engine**: if a traditional "physics dt" is required, populate `timeline.phys` with a uniform axis. Those ticks are included in the global boundary union so exactly one interval is integrated per physics tick while PX4/wind/log continue on their own cadences.

## Integration contracts

- `dt_*` values must be representable as integer microseconds.
- Randomness is only sampled at boundaries (never inside the ODE RHS).
- Autopilot commands are treated as sample-and-hold within an interval.
- The plant RHS is pure (no mutation, no IO). Any hard-bounds correction is done via `plant_project(...)` after an accepted interval.

## Extension points

- **Stage hooks** via `Runtime.Telemetry`: additional deterministic logic can run in a dedicated stage without changing the canonical ordering.
- **Recorder backends**: `Recording.InMemoryRecorder` is the default; larger runs can add an HDF5 backend later.
- **Log sinks**: attach `Sim.Logging.AbstractLogSink` instances via the `log_sinks` kw on `Sim.simulate` / `Runtime.Engine` (e.g. `Sim.Logging.CSVLogSink`).

## Caveats

- Contact modeling is penalty-force; there is no continuous-time root finding.
- Scenario `When` conditions are evaluated only at event boundaries.
- Tier-0 recordings do not embed model parameters; replay assumes the model configuration matches the recording context.
