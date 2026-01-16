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

## System Boundary

- **PX4 ABI wrapper:** loads `libpx4_lockstep`, validates ABI layout, and steps PX4.
- **Simulation framework:** integrates the plant, applies disturbances, and logs data.

The wrapper remains intentionally small; modeling and scheduling live in the simulation
layer so they can evolve without ABI risk.

## Data Ownership and Frames

- **Truth state** is owned by the simulator; **estimated state** is produced by
  estimator models and is the only state PX4 observes.
- **World frame** is NED with body axes in the aircraft frame.
- **Global position** is derived from the shared `WorldOrigin` using a spherical Earth
  approximation suited to local missions.

## Core Architectural Decisions

### Execution Engines

- **Fixed-step engine (`Simulation.jl`):** deterministic lockstep loop that integrates
  rigid-body dynamics while stepping actuators/propulsion/battery discretely.
- **Event-driven plant engine (`PlantSimulation.jl`):** integrates a coupled full-plant
  state between discrete event boundaries to enable adaptive solvers.
- **Record/replay bus engine (`RecordReplay.BusEngine`):** drives a plant from a typed
  signal bus and event timeline, enabling deterministic recording and replay.

The fixed-step engine remains the baseline for PX4 regressions. The event-driven plant
engine and bus engine are used to validate adaptive solvers and perform integrator
comparisons using recorded inputs.

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

## Execution Model (Fixed-Step Engine)

1. Snapshot the pre-step state for logging.
2. Step wind/turbulence and evaluate the scenario.
3. Sample wind once per tick and, on the autopilot cadence, run estimator → PX4.
4. Apply actuator dynamics and propulsion, then integrate the rigid body.
5. Update the battery and log the pre-step snapshot on the log cadence.

The fixed-step engine is the default for lockstep regression testing.

## Execution Model (Event-Driven Plant Engine)

1. At the current time, process scenario events, wind updates, autopilot ticks, and
   logging snapshots in a deterministic order.
2. Select the next boundary (wind/autopilot/log/scenario/`t_end`).
3. Integrate the full `PlantState` over the interval with held inputs, splitting on
   ground crossings when contact is enabled.
4. Clamp physical bounds and sync legacy component mirrors for logging and tooling.

This engine enables adaptive integration without sacrificing determinism.

## Execution Model (Record/Replay Bus Engine)

1. At each event boundary, update scenario, wind, estimator, and autopilot sources in
   deterministic order.
2. Update battery telemetry via `plant_outputs` and record bus streams when in record
   mode.
3. Integrate the plant from `t_k` to `t_{k+1}` using held bus inputs.
4. Record plant/battery snapshots on the log axis.

The bus engine decouples component output generation from plant integration and provides
deterministic replay using recorded streams.

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
- The fixed-step engine advances actuators/propulsion/battery discretely; it does not
  represent their continuous dynamics.
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

## Component Design Docs

Component-level notes live in `docs/components/README.md`.
Record/replay and fault semantics are detailed in `docs/record_replay.md` and
`docs/faults.md`.
