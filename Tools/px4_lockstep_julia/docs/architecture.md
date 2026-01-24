# PX4Lockstep.jl Simulation Design

## Scope

`PX4Lockstep.jl` runs PX4 through the lockstep C ABI while keeping all dynamics,
scheduling, and logging on the Julia side. This document covers the simulation
framework under `Tools/px4_lockstep_julia`.

## Architectural Goals

- **Deterministic lockstep:** identical inputs yield identical outputs, with exact
  microsecond time injection into PX4.
- **Reviewable numerical core:** in-house integrators and models remain dependency-light
  and easy to audit.
- **Strict separation of concerns:** ABI bindings are isolated from simulation logic.

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
                   └─> derived outputs (battery telemetry) ───┘

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
  (rigid body, actuators, rotors, batteries), enabling adaptive solvers.
- Integrators (`src/sim/Integrators.jl`) include:
  - Fixed-step Euler and RK4.
  - Adaptive RK23 and RK45 with optional microsecond-quantized substeps.

### Record & replay

- Record/replay is **bus-driven** and versioned by `BUS_SCHEMA_VERSION`.
- Tier‑0 recordings capture the minimum set of streams needed for deterministic replay
  (commands, wind, plant, battery telemetry), stored by `Recording.InMemoryRecorder`.
- Scenario streams can be recorded on the event axis (`*_evt`) to preserve dynamic
  transitions for replay.

### Logging

- Logging is separate from recording and uses `Sim.Logging.AbstractLogSink`.
- `CSVLogSink` writes a structured schema with `time_us` and setpoint data for plotting.

### Determinism guarantees

- No RNG in continuous RHS; randomness only at discrete boundaries.
- All time and scheduling decisions are integer microseconds.
- Inputs are piecewise constant between boundaries.

![runtime_flowchart](https://github.com/user-attachments/assets/a4b0ac3b-ae83-48e9-9cd0-82cdcd7b168e)

## System Boundary

- **PX4 ABI wrapper:** loads `libpx4_lockstep`, validates ABI layout, and steps PX4.
- **Simulation framework:** integrates the plant, applies disturbances, and logs data.

The wrapper remains intentionally small; modeling and scheduling live in the simulation
layer so they can evolve without ABI risk.

## uORB Bridge (Julia ⇄ PX4)

The lockstep ABI is **uORB-only**. Julia publishes uORB inputs and reads uORB outputs
through the generic pub/sub API exposed by `libpx4_lockstep`.

### Data flow

1. **Queue publishes:** Julia builds uORB messages and calls
   `PX4Lockstep.publish!(...)` on typed publishers.
2. **Lockstep step:** `PX4Lockstep.step_uorb!` advances PX4 time; queued messages are
   published on the C side inside `px4_lockstep_step_uorb`.
3. **Read outputs:** Julia polls uORB subscriptions (e.g. `actuator_motors`,
   `vehicle_attitude_setpoint`, `mission_result`) and updates `UORBOutputs`.

This keeps PX4 time injection deterministic while allowing multi-rate sensor injection
without expanding a fixed input struct.

### Julia implementation

- **Bridge helpers:** `src/sim/Autopilots/UORBBridge.jl` defines the topic registry,
  message builders, and the `UORBBridge` helper used by `PX4LockstepAutopilot`.
- **Autopilot integration:** `src/sim/Autopilots.jl` uses `UORBBridge` to publish inputs
  and sample outputs on every `autopilot_step`.
- **Size validation:** `create_publisher` / `create_subscriber` validate Julia struct
  sizes against uORB metadata at init time (fail fast on mismatch).

### Code generation

Julia uORB struct definitions are generated from the PX4 uORB headers using:

```
Tools/px4_lockstep_julia/scripts/uorb_codegen.jl
```

This produces `src/UORBGenerated.jl`, which must match the active PX4 build
(`build/px4_sitl_lockstep/uORB/topics`). The run scripts automatically regenerate the
file when headers are newer:

- `Tools/px4_lockstep_julia/scripts/run_iris_lockstep.sh`
- `Tools/px4_lockstep_julia/scripts/run_iris_integrator_compare.sh`

### Interface configuration

The uORB boundary is now configured explicitly via `PX4UORBInterfaceConfig` presets
(`iris_state_injection_interface`, `minimal_actuator_only_interface`). This keeps the
published/subscribed topics reproducible and removes reliance on environment flags.

## Data Ownership and Frames

- **Truth state** is owned by the simulator; **estimated state** is produced by
  estimator models and is the only state PX4 observes.
- **World frame** is NED with body axes in the aircraft frame.
- **Global position** is derived from the shared `WorldOrigin` using a spherical Earth
  approximation suited to local missions.

## Core Architectural Decisions

### Execution Engine

Historically, the codebase accumulated multiple "engines" with overlapping
responsibilities.

This is being refactored to converge on **one canonical engine**:

- **Canonical engine (`Sim.Runtime.Engine`)**: bus-driven hybrid engine that supports
  live PX4-in-the-loop, recording, and replay.

The goal is that only `Sim.Runtime.Engine` contains a real run loop. Legacy harnesses
either become thin wrappers or are removed.

See [`notes/runtime_engine.md`](notes/runtime_engine.md) for the engine architecture and invariants.

### Verification Strategy

Verification is treated as a first-class design concern:

* **Tier 1**: analytic/invariant unit tests (fast, always-on)
* **Tier 2**: full-plant contract tests (frames/sign/coupling checks; still fast)
* **Tier 3**: optional system regression (record/replay + solver envelope)

See [`notes/verification_coverage.md`](notes/verification_coverage.md) for current coverage.

### Full-Plant State for Adaptive Integration

Adaptive solvers require a single continuous state that captures actuator outputs, rotor
speeds, and battery states. `PlantState` collects these to keep the RHS pure and to allow
error control beyond rigid-body motion while still syncing legacy components after each
interval.

### Microsecond Timebase

All cadences (autopilot, wind, logging, scenario) are quantized to integer microseconds.
This forbids arbitrary `dt` values but guarantees exact lockstep timestamps and avoids
long-run drift.

### Pure RHS + Sample-and-Hold Inputs

ODE substeps are pure: no RNG, no mutation, and inputs held constant between discrete
events. Randomness and command updates are injected only at boundaries.

### Explicit RNG Partitioning

Independent RNG streams are used for wind, estimator noise, and miscellaneous processes
to prevent cross-coupling of random sequences.

### Signal Bus + Faults

The record/replay architecture uses a typed `SimBus` with a schema version. Discrete
components publish into the bus, while the plant consumes held values between event
boundaries. Faults are modeled as a bus-level `FaultState` (motor disable, battery
disconnect, sensor fault mask) so they can be recorded and replayed without mutating
model objects.

### Plant Output Protocol

`plant_outputs(...)` is a shared protocol (in `PlantInterface.jl`) for querying
algebraic outputs (battery telemetry, rotor outputs) at boundary times. Engines call
this hook to keep battery injection and logs deterministic without embedding engine
logic inside plant models.

## Fixed-step configuration

There is no separate "fixed-step engine". If a traditional fixed physics step is
required (for example to match an older simulator or to run simple regressions),
configure the canonical engine with a physics axis:

- populate `timeline.phys` with a uniform `0:dt_phys:t_end` microsecond-quantized axis
- choose a fixed-step integrator (Euler or RK4)

The runtime engine will then integrate exactly one interval per physics tick, while
still allowing autopilot (`timeline.ap`), wind (`timeline.wind`), and logging
(`timeline.log`) to run on their own cadences.

## Execution Model (Runtime Engine)

The canonical event-driven run loop is `Sim.Runtime.Engine`.

At each boundary time `t_k` (in microseconds):

1. Scenario publishes (`faults`, `ap_cmd`, `landed`).
2. Wind publishes (`wind_ned`).
3. Plant-derived telemetry updates the bus (`battery`, etc) via `plant_outputs`.
4. Estimator publishes (`est`).
5. Telemetry hooks run (optional, read-only).
6. Autopilot publishes (`cmd`).
7. Boundary-time plant discontinuities run (e.g. direct actuator snaps).
8. Logging/recording samples bus + plant (mode-dependent).
9. Plant integrates over `[t_k, t_{k+1})` with held `PlantInput`.

After each accepted interval, the engine may call `plant_project(f, x)` (if implemented)
to apply deterministic hard bounds (e.g., rotor ω ≥ 0, SOC ∈ [0,1]).

Modes
-----
The same engine supports:

* **Live**: live sources publishing into the bus.
* **Record**: live sources + recording sinks attached.
* **Replay**: replay sources driven by recorded traces (integrator comparisons).

## Key Invariants

- Time is derived from integer microsecond counters only.
- RHS evaluation is pure; RNG and mutation occur at boundaries.
- Controller outputs are sample-and-hold within each integration interval.
- ABI-facing structs match the C layout exactly.
- Recording/replay streams are versioned by `BUS_SCHEMA_VERSION`.

## Constraints and Tradeoffs

- `libpx4_lockstep` is not re-entrant by default; Monte Carlo runs should use separate
  processes.
- Contact modeling is penalty-force only; there is no general event detection.
- Adaptive integrators quantize substeps to microseconds, which is deterministic but
  slightly more conservative than unconstrained step selection.
- Closed-loop integrator comparisons require command replay to isolate plant error.
- Recordings do not embed model parameters; replay assumes the current model matches
  the recording context.

## Caveats and Limitations

- Microsecond quantization is mandatory: `dt`, event times, and delays must be exact
  integer microseconds or errors are raised.
- The event-driven plant engine currently targets multirotor plants (`QuadRotorSet`) and
  approximates contact handling with a penalty model and simple crossing split.
- Scenario `When` events are evaluated only at discrete event boundaries, not
  continuously.
- CSV logging is synchronous and can become IO-bound for long runs; plan for storage
  and throughput.
- `ScriptedScenario` uses floating-point time comparisons; align scenario times to
  `dt` or use `EventScenario` + `AtTime` for exact scheduling.
- Serialization uses Julia's built-in serializer; recordings are not a stable
  long-term archival format.

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
