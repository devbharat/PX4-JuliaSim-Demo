# TODO 03 — Weird / Unexpected / Technical Debt (Julia sim side)

Scope: **PX4Lockstep.Sim (Julia)** only.

These items are not necessarily “wrong” today, but they are the kinds of surprises that:
- slow onboarding,
- create hidden coupling,
- or become correctness problems when you add VTOL/fixed-wing.

---

## P0 — High-impact debt (will cause user pain)

### [ ] W0.1 — `write_recording(...)` is exported but currently errors

**What I saw**
- `src/sim/Recording/Recorder.jl` exports `write_recording` but the default method is:
  - `error("write_recording not implemented yet; use save_recording")`

**Why it’s bad**
- Public APIs that intentionally throw are a footgun.

**Plan**
- Option A (preferred): implement `write_recording(io_or_path, rec::Tier0Recording)` properly.
- Option B: stop exporting it; keep it internal until implemented.
- Ensure there is exactly one “blessed” persistence API (`save_recording` vs `write_recording`) and it is documented.

**Definition of done**
- Calling the exported persistence API works, and tests cover round-trip save/load.

---

### [ ] W0.2 — Two scheduling concepts exist, but only one is used

**What I saw**
- `src/sim/Scheduling.jl` defines trigger/schedule types (`StepTrigger`, etc.), but nothing else references them.
- The canonical engine uses `src/sim/Runtime/Scheduler.jl`.

**Why it’s weird**
- It looks like a half-removed subsystem. New contributors will waste time trying to understand which scheduler matters.

**Plan**
- Either delete `Scheduling.jl` (if truly dead), or wire it into Runtime and remove duplication.

**Definition of done**
- There is a single scheduling API surface; dead code removed.

---

### [ ] W0.3 — Events API includes `AtStep`, but `Runtime.Engine` has no `step` field

**What I saw**
- `src/sim/Events.jl`: `AtStep(step)` expects `sim.step` in `_sim_step(sim)`.
- `Runtime.Engine` doesn’t define `step`.

**Why it’s weird**
- The API suggests a supported event type that fails at runtime for the canonical engine.

**Plan**
- Decide what “step” means in the new architecture:
  - event boundary count,
  - physics tick count,
  - or remove step-based scheduling entirely.

**Definition of done**
- Either `AtStep` works with the canonical engine, or it is impossible to misuse.

---

## P1 — Medium debt (confusing / brittle)

### [ ] W1.1 — `MotorPropUnit` stores `ω_rad_s` but plant integrates `rotor_ω` separately

**What I saw**
- `src/sim/Propulsion.jl`: `MotorPropUnit` has `ω_rad_s::Float64`.
- `src/sim/PlantModels/CoupledMultirotor.jl`: the plant state has `rotor_ω`, and `_motorprop_unit_eval` uses the `ω` passed in, not `unit.ω_rad_s`.

**Why it’s weird**
- Two “copies” of ω exist; one is redundant.

**Plan**
- Remove `ω_rad_s` from `MotorPropUnit`, or move it fully into the integrated plant state (and make `MotorPropUnit` param-only).

---

### [ ] W1.2 — Battery model objects are mutable and hold state that is duplicated in `PlantState`

**What I saw**
- `src/sim/Powertrain.jl`: `IdealBattery` and `TheveninBattery` are mutable with internal SOC/V1.
- `PlantState` contains `batt_soc` and `batt_v1`.

**Why it’s weird**
- It invites accidental mixing: some code may read from the battery object while other code uses plant state.

**Plan**
- Split into `BatteryParams` + integrated state.
- Keep mutable stateful battery objects only for “legacy stepping” if you still need them.

---

### [ ] W1.3 — Log extraction uses runtime reflection (`hasproperty`) in multiple places

**What I saw**
- `src/sim/Runtime/Engine.jl`: `_autopilot_log_fields` checks `hasproperty(out, ...)` repeatedly.

**Why it’s weird**
- It hides interface drift (fields renamed, missing data) and makes log schema fragile.

**Plan**
- Define a small typed struct (or NamedTuple contract) for autopilot telemetry.
- Convert autopilot “raw output” → typed telemetry at the source boundary.

---

### [ ] W1.4 — Rotor logging is hard-coded to 4 rotors

**What I saw**
- `src/sim/Runtime/Engine.jl`: `_rotor_omega_tuple` and `_rotor_thrust_tuple` emit only 4 values.

**Why it’s weird**
- For hex/octo/VTOL, logs silently drop information.

**Plan**
- See Architecture TODO A1.4.

---

## P2 — Style / cleanup (nice to fix as you touch areas)

### [ ] W2.1 — Remove unused imports

**Example**
- `src/sim/Propulsion.jl` imports `Random` but doesn’t use it.

### [ ] W2.2 — Add a “public API index” doc

**Why**
- The code is modular (good), but it’s not obvious which entrypoints are stable.

**Plan**
- A simple `docs/API.md` (or README section) listing the intended public entrypoints:
  - `Sim.simulate`
  - `Sim.record_live_px4`
  - `Sim.replay_recording`
  - `Workflows.simulate_iris_mission`

