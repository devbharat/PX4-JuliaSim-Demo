# Component Design Docs

This directory collects component-level design notes for `PX4Lockstep.jl`. This page
adds a **system architecture overview** and then links to the deeper component docs.

## System Architecture Overview

### Layering

There are two primary layers:

1. **`PX4Lockstep` ABI wrapper** (`src/PX4Lockstep.jl`)
   - Loads `libpx4_lockstep` and exposes `create`, `step!`, `load_mission`, etc.
   - Ensures ABI compatibility and enforces the single-handle default.
2. **`PX4Lockstep.Sim` simulation framework** (`src/sim/*`)
   - Owns truth dynamics, environment, powertrain, logging, and scenarios.
   - Provides the deterministic event-driven runtime used for live, record, and replay runs.

### Core data flow

```
truth plant state ─┐
                  ├─> estimator source ─> bus.est ─┐
scenario outputs ─┤                              ├─> autopilot source ─> bus.cmd
wind source ──────┤                              │
                  └─> derived outputs (battery) ─┘

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

### uORB bridge migration (experimental)

The lockstep ABI now exposes a generic uORB pub/sub bridge. It allows Julia to queue
uORB messages by topic name and flush them inside `px4_lockstep_step()` after time is
updated. This keeps determinism while avoiding new fields in `px4_lockstep_inputs_t`.

Current opt-in migration (staged):

- `battery_status`, `vehicle_attitude`, `vehicle_local_position`,
  `vehicle_global_position`, `vehicle_angular_velocity`, `vehicle_land_detected`,
  `vehicle_status`, `vehicle_control_mode`, `actuator_armed`, `home_position`, and
  `geofence_status` can be published via the uORB bridge from Julia instead of
  `publish_inputs()`.
- Enable with `PX4_LOCKSTEP_UORB_BATTERY=1`, `PX4_LOCKSTEP_UORB_ATTITUDE=1`,
  `PX4_LOCKSTEP_UORB_LOCAL_POSITION=1`, `PX4_LOCKSTEP_UORB_GLOBAL_POSITION=1`,
  `PX4_LOCKSTEP_UORB_RATES=1`, `PX4_LOCKSTEP_UORB_LAND_DETECTED=1`,
  `PX4_LOCKSTEP_UORB_VEHICLE_STATUS=1`, `PX4_LOCKSTEP_UORB_VEHICLE_CONTROL_MODE=1`,
  `PX4_LOCKSTEP_UORB_ACTUATOR_ARMED=1`, `PX4_LOCKSTEP_UORB_HOME_POSITION=1`, and/or
  `PX4_LOCKSTEP_UORB_GEOFENCE_STATUS=1`. The C++ harness will skip those legacy
  publishes when the flags are set.
- Set `PX4_LOCKSTEP_UORB_OUTPUTS=1` to read outputs from uORB subscriptions
  (`vehicle_torque_setpoint`, `vehicle_thrust_setpoint`, `actuator_motors`,
  `actuator_servos`, `vehicle_attitude_setpoint`, `vehicle_rates_setpoint`,
  `mission_result`, `vehicle_status`, `battery_status`, `trajectory_setpoint`).
- Set `PX4_LOCKSTEP_UORB_ONLY=1` to use the uORB-only ABI entrypoint
  (`px4_lockstep_step_uorb`) and avoid `px4_lockstep_inputs_t`/`px4_lockstep_outputs_t`.
- Julia message layouts must match the generated uORB C structs; until autogeneration
  exists, validate against headers under `build/px4_sitl_lockstep/uORB/topics`.

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
