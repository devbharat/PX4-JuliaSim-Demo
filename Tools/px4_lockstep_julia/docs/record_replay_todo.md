# Record/Replay TODO (living list)

This file is the **authoritative checklist** for the Option A (event-sourced bus) record/replay
work. Keep it updated as we implement.

Conventions:
- `[ ]` not started
- `[~]` in progress
- `[x]` done

---

## Phase 0 — Design + scaffolding

- [x] Write design doc: `docs/record_replay.md`
- [x] Add this TODO tracker
- [x] Create `Sim.RecordReplay` module skeleton and include it from `Sim.jl`
- [x] Add placeholder example scripts under `examples/replay/`

Acceptance criteria:
- Repo loads with the new module (no runtime errors unless you call TODO functions).

---

## Phase 1 — Core data model (bus + timeline + streams)

- [x] Define `Sim.RecordReplay.SimBus` schema (versioned)
  - [x] Decide which signals are required for Tier-0 integrator replay (cmd, wind, battery)
  - [x] Add first-class **fault state** signal on the bus (motor disable, battery disconnect, sensor fail masks)
  - [x] Decide and document fault semantics + consumption map (`docs/faults.md`)
  - [x] Document interpolation rules per signal (ZOH vs sample/hold)
  - [x] Reuse `Sim.Powertrain.BatteryStatus` for `bus.battery` (avoid duplicate schema drift)

- [x] Define `Timeline` API
  - [x] Build periodic axes (`T_ap_us`, `T_wind_us`, `T_log_us`)
  - [x] Integrate scenario `AtTime` events into `T_scn_us`
  - [x] Provide `T_evt_us = union(...)` builder with deterministic ordering

- [x] Define `Trace` abstractions
  - [x] `ZOHTrace` (commands)
  - [x] `SampleHoldTrace` (wind)
  - [x] `SampledTrace` (logs)
  - [x] Deterministic `sample(trace, t_us)` semantics

Acceptance criteria:
- Can build a timeline for a given `(t0_us, t_end_us, dt_*)`.
- Can evaluate traces deterministically at arbitrary `t_us`.

---

## Phase 2 — Sources (live + replay)

- [x] Autopilot source
  - [x] `LiveAutopilotSource` (wrap PX4 lockstep handle)
  - [x] `ReplayAutopilotSource` (reads recorded cmd trace)

- [x] Wind source
  - [x] `LiveWindSource` (wrap existing OU/gust model)
  - [x] `ReplayWindSource` (reads recorded wind trace)

- [x] Scenario source
  - [x] `LiveScenarioSource` (wrap existing scenario/events)
  - [x] `ReplayScenarioSource` (replays recorded `bus.ap_cmd` + `bus.landed` + `bus.faults`)

- [x] Refactor plant-affecting scenario faults to publish bus-level fault state (no mutation)
  - [x] motor disable (duty forced to 0 via fault mask)
  - [x] battery disconnect (bus voltage -> 0, `BatteryStatus.connected=false`)
  - [x] sensor failure mask (consumed by estimator source; minimal freeze hook)

- [x] Estimator source
  - [x] `NullEstimatorSource` publishes truth-as-estimate to `bus.est`
  - [x] `LiveEstimatorSource` wraps `Sim.Estimators` with a dedicated RNG and dt
  - [x] `ReplayEstimatorSource` replays recorded estimator output as a ZOH bus stream

Acceptance criteria:
- Each source can be run standalone on its axis and publish into a `SimBus`.

---

## Phase 3 — Engine: bus-driven hybrid simulation

- [x] Implement `BusEngine` that can run in two modes:
  - [x] `:replay` (replay sources)
  - [x] `:record` (live sources + recorder)

- [x] Deterministic event processing order at each `t_us`:
  - [x] scenario → wind → (derived bus outputs) → estimator → autopilot → logging

- [x] Plant integration between event boundaries
  - [x] Hold bus inputs constant over interval (`PlantInput(cmd, wind)`)
  - [x] Reset integrator state at each interval (matches current PlantSimulation semantics)
  - [x] Optional derived outputs update via `plant_outputs(...)` when applicable

Acceptance criteria:
- With `ReplayAutopilotSource + ReplayWindSource`, the plant can integrate open-loop without PX4.

---

## Phase 4 — Recording backends

- [x] Implement `InMemoryRecorder` for tests
  - [x] Add trace builders: `zoh_trace`, `samplehold_trace`, `sampled_trace`
  - [x] Add helper: `tier0_traces(rec, timeline)`
- [x] Add a minimal Tier-0 persistence format
  - [x] `Tier0Recording` wrapper (timeline + plant0 + InMemoryRecorder)
  - [x] `save_recording` / `load_recording` via `Serialization`
- [ ] Define HDF5 schema and implement `HDF5Recorder` (optional dependency)
  - [ ] chunking + compression
  - [ ] metadata and schema versions

Acceptance criteria:
- Tier-0 record file supports: cmd trace, wind trace, plant log outputs.
- Tier-0 also records scenario outputs (`ap_cmd`, `landed`, `faults`) and **faults on `timeline.evt`** (so `When(...)`-driven transitions cannot be missed).

---

## Phase 5 — Iris integrator sweep tool (the payoff)

- [x] `examples/replay/iris_record_run.jl`
  - [x] run PX4 live, record Tier-0 traces

- [x] `examples/replay/iris_replay_integrator_sweep.jl`
  - [x] replay plant under different integrators
  - [x] compare to reference replay using `Sim.Verification` utilities

- [x] Clean UX wrapper: `examples/replay/iris_integrator_compare.jl`
  - [x] record baseline run (PX4 live) **and** replay sweep in one command
  - [x] write a summary CSV under `examples/replay/out/`
  - [x] keep Iris defaults centralized via `examples/replay/iris_common.jl`

Acceptance criteria:
- Integrator comparison is stable and not dominated by PX4 closed-loop divergence.

---

## Phase 6 — Component-level replay (Tier-1)

- [ ] Battery replay harness (input = recorded bus current)
- [ ] Propulsion replay harness (inputs = duty, rho, airspeed, bus voltage)
- [ ] Estimator injection replay harness

Acceptance criteria:
- Each component can be replayed independently and matches recorded outputs on the chosen grid.

