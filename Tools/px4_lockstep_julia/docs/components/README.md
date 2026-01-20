# Component Design Docs

This directory collects component-level design notes for `PX4Lockstep.jl`. This page
adds a **system architecture overview** and then links to the deeper component docs.

## System Architecture Overview

### Layering

There are two primary layers:

1. **`PX4Lockstep` ABI wrapper** (`src/PX4Lockstep.jl`)
   - Loads `libpx4_lockstep` and exposes `create`, `step_uorb!`, `load_mission`, etc.
   - Ensures ABI compatibility and enforces the single-handle default.
2. **`PX4Lockstep.Sim` simulation framework** (`src/sim/*`)
   - Owns truth dynamics, environment, powertrain, logging, and scenarios.
   - Provides the deterministic event-driven runtime used for live, record, and replay runs.

### Core data flow

```
truth plant state ─┐
                   ├─> estimator source ─> bus.est ─┐
scenario outputs ──┤                                ├─> autopilot source ─> bus.cmd
wind source ───────┤                                │
                   └─> derived outputs (battery) ───┘

bus.cmd + bus.wind + bus.faults  →  plant integrator  →  new plant state
```

- All coupling between components goes through `Runtime.SimBus` (`src/sim/Runtime/Bus.jl`).
- Inputs are **sample-and-hold** between event boundaries to preserve determinism.

### Runtime engine

`Runtime.Engine` (`src/sim/Runtime/Engine.jl`) is the **only** authoritative run loop.

- Time advances on the union event axis (`timeline.evt`), built from
  autopilot, wind, log, scenario, and optional physics axes.
- At each boundary, the engine runs a fixed stage order
  (`Runtime/BoundaryProtocol.jl`):

  1. scenario → 2. wind → 3. derived outputs → 4. estimator → 5. telemetry →
  6. autopilot → 7. plant discontinuities → 8. logging

- The plant is integrated over `[t_k, t_{k+1})` using the selected integrator.

### Timebase and scheduling

- All scheduling uses **integer microseconds** (`UInt64`) via `Runtime.Timeline`.
- `dt_to_us` enforces exact microsecond representability (no float drift).
- Optional `timeline.phys` provides fixed-step boundaries without a separate engine.

### Plant + integrators

- `PlantState` / `PlantDeriv` (`src/sim/Plant.jl`) hold the **full** continuous state
  (rigid body, actuators, rotors, battery), enabling adaptive solvers.
- Integrators (`src/sim/Integrators.jl`) include:
  - Fixed-step Euler and RK4.
  - Adaptive RK23 and RK45 with optional microsecond-quantized substeps.

### Record & replay

- Record/replay is **bus-driven** and versioned by `BUS_SCHEMA_VERSION`.
- Tier‑0 recordings capture the minimum set of streams needed for deterministic replay
  (commands, wind, plant, battery), stored by `Recording.InMemoryRecorder`.
- Scenario streams can be recorded on the event axis (`*_evt`) to preserve dynamic
  transitions for replay.

### Logging

- Logging is separate from recording and uses `Sim.Logging.AbstractLogSink`.
- `CSVLogSink` writes a structured schema with `time_us` and setpoint data for plotting.

### Determinism guarantees

- No RNG in continuous RHS; randomness only at discrete boundaries.
- All time and scheduling decisions are integer microseconds.
- Inputs are piecewise constant between boundaries.

<img width="1958" height="1760" alt="runtime_flowchart" src="https://github.com/user-attachments/assets/44527cd1-1d53-43a7-b70c-4f738cb028ea" />

### uORB bridge

The lockstep ABI is **uORB-only**. Julia queues uORB messages by topic name and flushes
them inside `px4_lockstep_step_uorb()` after time is updated. This keeps determinism
while avoiding legacy input/output structs.

Key notes:

- Inputs are published via the uORB bridge (see env flags in
  `Tools/px4_lockstep_julia/scripts/run_iris_lockstep.sh`).
- Outputs are read via uORB subscriptions (e.g. `actuator_motors`,
  `vehicle_attitude_setpoint`, `mission_result`).
- Julia message layouts must match the generated uORB C structs; validate against
  headers under `build/px4_sitl_lockstep/uORB/topics`.

## Component Docs Index

- [x] `PX4Lockstep` C ABI wrapper (`src/PX4Lockstep.jl`)
- [x] Fixed-step semantics via `Sim.Runtime.Engine` + `timeline.phys`
- [x] Event-driven integration (`src/sim/Runtime/Engine.jl`)
- [x] Integrators (`src/sim/Integrators.jl`)
- [x] Plant state model (`src/sim/Plant.jl`)
- [x] Rigid-body core (`src/sim/RigidBody.jl`)
- [x] Environment models (`src/sim/Environment.jl`)
- [x] Vehicles (`src/sim/Vehicles.jl`)
- [x] Propulsion (`src/sim/Propulsion.jl`)
- [x] Powertrain (`src/sim/Powertrain.jl`)
- [x] Scenario/events (`src/sim/Scenario.jl`, `src/sim/Events.jl`)
- [x] Scheduling (`src/sim/Runtime/Timeline.jl`, `src/sim/Runtime/Scheduler.jl`)
- [x] Estimators (`src/sim/Estimators.jl`)
- [x] Noise utilities (`src/sim/Noise.jl`)
- [x] Contacts (`src/sim/Contacts.jl`)
- [x] Logging (`src/sim/Logging.jl`)
- [x] Verification utilities (`src/sim/Verification.jl`)
- [x] Autopilot bridge (`src/sim/Autopilots.jl`)
- [x] Fault signals (`src/sim/Faults.jl`)
- [x] Plant output protocol (`src/sim/PlantInterface.jl`)
- [x] Record/replay primitives (`src/sim/Recording/`, `src/sim/Sources/`, `src/sim/Runtime/`)
