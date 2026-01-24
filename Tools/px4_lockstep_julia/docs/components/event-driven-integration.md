# Event-Driven Integration (Runtime.Engine)

## Role

The canonical event-driven run loop lives in `src/sim/Runtime/Engine.jl`.

It advances a hybrid system:

- **Discrete** sources publish into a shared `SimBus` at scheduled boundaries.
- The **continuous** plant is integrated over each interval using sample-and-hold
  inputs derived from the bus.

This is the single place where boundary ordering, stop-time semantics, and recording
rules are defined.

## Key decisions and rationale

- **One canonical run loop:** avoids drift between multiple engines implementing subtly
  different boundary ordering or stop-time semantics.
- **Event boundaries define input changes:** wind, scenario outputs, and autopilot
  commands are only allowed to change at discrete boundaries. Between boundaries, the
  plant input is held constant.
- **Microsecond clock:** all schedules use integer microseconds (`UInt64`) to align
  with lockstep time and avoid floating-point drift.
- **Optional fixed-step physics boundaries:** the timeline supports an optional
  `timeline.phys` axis. When present, those ticks are included in the global event union
  to enforce a fixed integration step size while still stepping PX4 at its own cadence.
- **Single coupled RHS:** plant models (e.g. `PlantModels.CoupledMultirotorModel`) can
  evaluate actuators + propulsion + battery + rigid body in one RHS so adaptive error
  control sees the full coupled plant.
- **Protocol hooks, not engine forks:** optional `plant_outputs`, `plant_project`, and
  `plant_on_autopilot_tick` hooks allow models to expose deterministic algebraic outputs
  and boundary-time discontinuities without duplicating run loops.

## Boundary ordering contract

At each global boundary time `t_k` (in microseconds), the engine applies a fixed stage
order (see `src/sim/Runtime/BoundaryProtocol.jl`). Individual stages are gated by axis
membership (e.g. wind updates only when `t_k ∈ timeline.wind`); otherwise the previous
bus sample is held.

Order at `t_k`:

1. Scenario updates bus (`faults`, `ap_cmd`, `landed`) if `t_k ∈ timeline.scn`
2. Wind updates bus (`wind_ned`) if `t_k ∈ timeline.wind`
3. Plant-derived telemetry updates bus (battery vector, env cache) via `plant_outputs`
4. Estimator updates bus (`est`) if `t_k ∈ timeline.ap`
5. Telemetry hooks run (optional, read-only) if `t_k ∈ timeline.ap`
6. Autopilot updates bus (`cmd`) if `t_k ∈ timeline.ap`
7. Boundary-time plant discontinuities run via `plant_on_autopilot_tick` (optional)
8. Recorder/logging samples bus + plant if `t_k ∈ timeline.log` (mode-dependent)
9. Plant integrates over `[t_k, t_{k+1})` with held `PlantInput`

This ordering is implemented in exactly one place: `Runtime.Engine`.

## Caveats

- Scenario `When(...)` conditions are evaluated only when the scenario is stepped
  (typically at `timeline.scn` boundaries). If you need higher-rate evaluation, include
  additional scenario boundaries in the timeline (via `event_times_us`) or implement a
  scenario source that publishes at the desired cadence.
- Dynamic insertion of new explicit boundaries at runtime is not supported; timeline
  boundaries are computed up-front.
- Contact event detection (root-finding) is not implemented; penalty contact is treated
  as continuous forcing.
