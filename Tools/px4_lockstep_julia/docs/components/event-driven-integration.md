# Event-Driven Integration (Runtime.Engine)

## Role

The canonical event-driven run loop lives in `src/sim/Runtime/Engine.jl`.

It integrates the continuous plant state between discrete event boundaries (autopilot,
wind, scenario, logging) while preserving determinism.

## Key Decisions and Rationale

- **One canonical run loop:** avoids drift between multiple engines implementing
  subtly different boundary ordering and stop-time semantics.
- **Event boundaries define inputs:** scenario events, wind updates, and autopilot
  ticks are the only points where inputs may change; logging/recording samples are
  scheduled as boundaries for deterministic snapshots.
- **Optional fixed-step physics boundaries:** the timeline supports an optional
  `timeline.phys` axis. When present, its ticks are included in the global event union
  so you can force a fixed integration step size (e.g. to emulate a fixed-step
  simulator) while still stepping PX4 at its own cadence.
- **Microsecond clock:** all schedules use integer microseconds (`UInt64`) to align
  with lockstep time and avoid floating-point drift.
- **Single coupled RHS:** plant models (e.g., `PlantModels.CoupledMultirotorModel`)
  evaluate actuators + propulsion + battery + rigid body in one RHS so adaptive error
  control sees the full plant.
- **Deterministic bus coupling:** discrete sources publish into a typed, versioned
  `SimBus` and the engine derives held `PlantInput` from the bus over each interval.
- **Optional plant projection:** after each accepted interval, the engine calls
  `plant_project(f, x)` if implemented by the model to enforce hard bounds
  deterministically (e.g., rotor ω ≥ 0, SOC ∈ [0,1]).

## Boundary Ordering Contract

At each boundary time `t_k` (in microseconds):

1. Scenario updates bus (`faults`, `ap_cmd`, `landed`)
2. Wind updates bus (`wind_ned`)
3. Plant-derived telemetry updates bus (`battery`, etc) via `plant_outputs`
4. Estimator updates bus (`est`)
5. Telemetry hooks run (optional, read-only)
6. Autopilot updates bus (`cmd`)
7. Boundary-time plant discontinuities run (e.g. direct actuator snaps)
8. Recorder/logging samples bus + plant (mode-dependent)
9. Plant integrates over `[t_k, t_{k+1})` with held `PlantInput`

This ordering must be implemented in exactly one place: `Runtime.Engine`.

## Caveats

- Scenario `When(...)` conditions are evaluated at existing boundaries; if scenarios
  add new AtTime events dynamically, the runtime timeline will need dynamic boundary
  insertion (tracked in `docs/engine_unification_todo.md`).
- Contact event detection (root-finding) is not implemented; penalty contact is
  treated as continuous forcing.
