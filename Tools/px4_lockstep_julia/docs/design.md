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

## uORB Bridge (Julia ⇄ PX4)

The lockstep ABI is **uORB-only**. Julia publishes uORB inputs and reads uORB outputs
through the generic pub/sub API exposed by `libpx4_lockstep`.

### Data flow

1. **Queue publishes:** Julia builds uORB messages and calls
   `PX4Lockstep.queue_uorb_publish!(...)`.
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
- **Size validation:** `create_uorb_publisher_checked` validates Julia struct sizes
  against uORB metadata at init time (fail fast on mismatch).

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

### Environment controls

Input topic publishers are **enabled by default**. Set any of the `PX4_LOCKSTEP_UORB_*`
flags to `0` to disable a specific publisher (e.g.
`PX4_LOCKSTEP_UORB_LOCAL_POSITION=0`). Output subscriptions are always created so PX4
outputs remain available to the simulator.

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

See `docs/engine_unification.md` and `docs/engine_unification_todo.md`.

### Verification Strategy

Verification is treated as a first-class design concern:

* **Tier 1**: analytic/invariant unit tests (fast, always-on)
* **Tier 2**: full-plant contract tests (frames/sign/coupling checks; still fast)
* **Tier 3**: optional system regression (record/replay + solver envelope)

See:

* `docs/verification_plan.md` (canonical plan)
* `docs/verification_todo.md` (implementation progress)

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

## Component Design Docs

Component-level notes live in `docs/components/README.md`.
Record/replay and fault semantics are detailed in `docs/record_replay.md` and
`docs/faults.md`.
