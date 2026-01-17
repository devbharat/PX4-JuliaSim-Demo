# Record & Replay Architecture (Option A: Event-sourced Signal Bus)

This design adds **first-class record/replay** to `PX4Lockstep.Sim`.

The intent is **not** “log more things”. The intent is to make the simulator
**scientifically comparable** across integrators/model changes and to enable **component-level
replay** for debugging and model iteration.

Key outcomes:

- Deterministic **plant-only** replay (for integrator sweeps) without closed-loop divergence.
- Ability to replay **any component** (wind, battery, propulsion, estimator, scenario) from
  recorded inputs.
- A consistent, versioned data model that scales from CSV to **HDF5**.

---

## 1. Non-negotiable contracts

### 1.1 Timebase contract

- The only authoritative time is `time_us::UInt64`.
- All schedules, event times, and recorded sample timestamps must be integer microseconds.
- Any `Float64 t_s` is derived: `t_s = time_us * 1e-6`.

### 1.2 Hybrid (event + continuous) contract

The simulation is a hybrid system:

- **Discrete events** occur at scheduled times (autopilot tick, wind tick, scenario events, log tick).
- Between events, the plant integrates continuously over `[t_k, t_{k+1})`.

Inputs between events are defined by explicit interpolation rules:

- PX4 motor/servo commands: **ZOH** (hold last command until next autopilot tick).
- Wind realization: **sample-and-hold** (hold constant between wind ticks).
- Scenario parameter changes (faults): piecewise constant, applied at their event times.

Fault semantics are defined in `docs/faults.md`.

### 1.3 Determinism contract

- No RNG use in continuous RHS evaluation.
- RNG use is only allowed in discrete components (wind update, noise/estimator update) and must be recordable.
- Replay must not depend on iteration order of hash tables or non-deterministic collection ordering.

### 1.4 Record/Replay contract

A replay run must be fully determined by:

- (A) a **run configuration** + model parameters (versioned/hashes),
- (B) initial states for components,
- (C) a set of **recorded streams** aligned to integer time axes,
- (D) explicit interpolation rules.

---

## 2. Architectural model

### 2.1 The Bus

Option A uses a typed **signal bus** as the *only* way components exchange information.

- Components *read* a subset of bus fields as inputs.
- Components *write* a subset of bus fields as outputs.

The bus is versioned so traces are stable across time:

- `bus_schema_version::Int`

The bus is **not** meant to contain “everything”; it contains signals that form the coupling
between components.

### 2.2 Components

Components are independent subsystems that can run in one of two modes:

- `Live` (compute outputs from physics/models)
- `Replay` (publish outputs from a recorded stream)

Each component has:

- immutable parameters (config)
- internal state (continuous or discrete)
- update function(s)

#### Discrete component interface

A discrete component updates at its event times:

```julia
step_discrete!(comp, bus, t_us)::Nothing
```

Examples:

- autopilot tick (PX4 lockstep)
- wind tick (OU/gust update)
- scenario tick (fault injections)
- estimator tick

#### Continuous contribution interface

Continuous components contribute to the plant RHS (pure, no RNG):

```julia
eval_continuous(comp, x::PlantState, bus, t_us)::ComponentContribution
```

Examples:

- propulsion/battery coupling (algebraic + ODE states)
- aerodynamic forces
- rigid-body dynamics

**Note:** for the “full-plant variable-step” system, the canonical continuous state is
`PlantState`.

### 2.3 Scheduler / Timeline

A `Timeline` is the union of event times from all discrete components:

- `T_ap_us`   autopilot ticks
- `T_wind_us` wind ticks
- `T_log_us`  logging sample times
- `T_scn_us`  scenario events (AtTime)
- `T_evt_us`  union of all above plus `t_end_us`

The engine advances by:

1) processing all discrete events at `t = T_evt[k]` (in deterministic priority order)
2) integrating plant from `T_evt[k]` to `T_evt[k+1]`

Implementation note: the engine uses per-axis cursors so membership checks are
O(1) per boundary (avoids repeated binary searches on long timelines).

---

## 3. Record/Replay design

### 3.1 Streams and time axes

A “stream” is a time-indexed sequence of values sampled on a specific axis.

Examples:

- `cmd` stream on `T_ap_us`
- `est` stream on `T_ap_us` (optional; `record_estimator=true`)
- `wind_ned` stream on `T_wind_us`
- `plant_state` stream on `T_log_us`
- `ap_cmd`, `landed`, `faults` streams on `T_scn_us`
- `ap_cmd_evt`, `landed_evt`, `faults_evt` streams on `T_evt_us` for dynamic scenarios

Scenario outputs that couple into the rest of the sim (e.g. `ap_cmd`, `landed`, `faults`) are
recorded as **bus streams**, not as opaque event payloads.

Important note for dynamic scenarios:

* If scenarios can change outputs based on state (e.g. `When(...)`), record those bus
  outputs on **`T_evt_us`** so replay cannot miss a transition.
  - In code, these are recorded as `:ap_cmd_evt`, `:landed_evt`, and `:faults_evt` streams
    aligned to `timeline.evt`.
* See `docs/faults.md` for the exact recording semantics.

Interpolation rules are stream-specific and explicit:

- commands: ZOH
- wind: sample-and-hold
- logs: sampled (no interpolation)

### 3.2 Record tiers

Tiered recordings are supported to prevent trace bloat.

#### Tier 0: Integrator comparison (default goal)

Record the minimum required to replay the plant deterministically:

- timeline axes `T_ap_us`, `T_wind_us`, `T_log_us`, `T_scn_us`, `T_phys_us`, `T_evt_us`
- `cmd` on `T_ap_us`
- `wind_ned` on `T_wind_us`
- `PlantState` (or selected outputs) on `T_log_us`
- `battery` on `T_log_us` (telemetry snapshot)

Optional (for estimator replay):

- `est` on `T_ap_us` when `record_estimator=true`

This enables:

- plant-only replay under different integrators (no PX4 involved)

#### Tier 1: Component replay

Record component inputs/outputs required to replay each component in isolation:

- propulsion: duty, rho, wind/air-relative quantities, bus voltage/current, ω
- battery: bus current/voltage, SOC/V1
- estimator output: `est` on `T_ap_us` when `record_estimator=true`
- estimator/noise: injected sensor streams

#### Tier 2: RHS trace (debug)

Record per-RHS evaluation details (adaptive solvers):

- `t_rhs_us[i]`
- state subset
- bus solve residuals and intermediate values

This is expensive and opt-in only.

### 3.3 Replay modes

1) **Full replay**: replay everything (useful for deterministic regression reproduction)
2) **Plant-only replay**: replay `cmd(t)` and `wind(t)` and re-run plant
3) **Component-only replay**: replay one component given its recorded inputs
4) **Mixed**: replay wind + autopilot live; or replay autopilot + wind live, etc.

---

## 4. Iris integrator comparison workflow (target use case)

Closed-loop comparisons drift because tiny differences change PX4 decisions.

The correct workflow is:

1) Run once with PX4 live, record Tier 0:
   - `cmd(t)` on autopilot ticks
   - `wind(t)` on wind ticks
   - plant outputs on log ticks
2) Replay the plant with:
   - `AutopilotReplaySource` (publishing recorded `cmd`)
   - `WindReplaySource`
3) Sweep integrators/tolerances and compare against a reference replay:
   - reference = RK45 tight (plant-aware error control ON)
   - tests = Euler/RK4/RK23/RK45

This isolates plant integration error from controller divergence.

---

## 5. Storage format and schema

### 5.1 Minimum requirement

The record/replay API should support a simple in-memory recorder for tests.

For lightweight persistence and sharing, the repository provides a minimal
`Tier0Recording` container that can be saved/loaded via Julia's built-in
`Serialization` (no extra dependencies). Tier‑0 files serialize a deterministic,
ordered representation of streams so recordings are byte‑stable across runs when
inputs are identical.

For production runs, an HDF5 backend is expected.

### 5.2 HDF5 layout (proposed)

One file per run.

```
/meta/
  schema_version
  bus_schema_version
  git_commit
  julia_version
  config_json
  param_hashes/

/time/
  T_evt_us
  T_ap_us
  T_wind_us
  T_log_us
  T_scn_us

/signals/
  /cmd/ motors[12, N_ap], servos[8, N_ap]
  /wind/ wind_ned[3, N_wind]
  /plant/ pos_ned[3, N_log], vel_ned[3, N_log], q_bn[4, N_log], ...
  /battery/ bus_v[N_log], bus_i[N_log], soc[N_log], v1[N_log]
  /scenario/ event_type[N_scn], payload[...]

(optional)
/rhs_trace/
  T_rhs_us
  ...
```

Chunking: chunk along time dimension; compression on.

---

## 6. Implementation plan (high-level)

### Stage 1: Foundational types

- Define `SimBus` schema and version.
- Define `Timeline` and axis builder.
- Define `Trace` abstraction for streams (ZOH, sample/hold).

### Stage 2: Sources

- `AutopilotSource`: Live + Replay
- `WindSource`: Live + Replay
- `ScenarioSource`: Live + Replay
- `EstimatorSource`: Live + Replay (truth pass-through + optional noisy estimator)

### Stage 3: Engine

- `Engine` executes: process events at `t_k`, integrate plant to `t_{k+1}`.
- Integrates using the full-plant RHS from `PlantModels` (e.g. `CoupledMultirotorModel`) but routed through the bus.

### Stage 4: Record/replay

- `Recorder`: Null + InMemory, with schema validation on load
- `Replay`: Trace readers (InMemory)
- HDF5 recorder/replayer (optional, later)

### Stage 5: Examples + tests

- `examples/replay/iris_integrator_compare.jl`
- `examples/replay/minimal_record_replay_demo.jl`
- Tests that replay matches baseline within tolerance.

---

## 7. Design choices (explicit)

- Design choices prioritize **clarity** over minimal changes. Backward compatibility is not a hard constraint.
- Recording at adaptive substep resolution is avoided by default.
- Stream interpolation rules are explicit (no implicit “nearest sample”).
- Component boundaries remain explicit so submodels can be replayed independently.
