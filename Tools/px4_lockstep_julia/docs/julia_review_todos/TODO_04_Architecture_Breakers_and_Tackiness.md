# TODO 04 — Architecture Breakers / “Tackiness” (Julia sim side)

Scope: **PX4Lockstep.Sim (Julia)** only.

These are the places where the code currently works, but the *mechanism* fights the intended architecture:
- bus-first coupling,
- deterministic record/replay,
- pure plant RHS,
- explicit discontinuities at boundaries.

If left as-is, these patterns will make VTOL/fixed-wing support much harder (because every new subsystem adds new hidden mutation paths).

---

## P0 — Must-fix architecture breakers

### [ ] T0.1 — Remove `ScenarioSimView` (scenario should not get a backdoor reference to the sim)

**What I saw**
- `src/sim/Sources/Scenario.jl` defines `ScenarioSimView` containing:
  - `env`, `battery`, `plant` and other internals
- `scenario_step(...)` passes this view into the scenario.

**Why this breaks the architecture**
- It allows scenario code to mutate *anything* (environment, battery, propulsion, plant state) without those effects being represented on the bus or recorded as streams.
- It turns the scenario interface into “here’s the entire sim, do what you want,” which is the opposite of a clean coupling contract.

**Plan (recommended)**
- Replace the scenario interface with a pure boundary update:
  - inputs: `(t_us, bus_in, plant_state_snapshot, maybe env_snapshot)`
  - outputs: `(faults, high_level_ap_cmd, landed, optional wind_injection_command, optional estimator_faults, etc.)`
- If a scenario needs to influence wind, have it output a **bus-level wind disturbance request**, and let the wind source incorporate it.
- Keep a *temporary* legacy adapter that wraps existing scenarios, but make it opt-in.

**Definition of done**
- No scenario code receives mutable references to engine internals.
- All scenario effects flow through explicit outputs that can be recorded/replayed.

---

### [ ] T0.2 — Eliminate mutation-based failures outside `FaultState`

**What I saw**
- `Propulsion.MotorPropUnit` has `enabled::Bool` and a mutating `set_motor_enabled!`.
- Scenario helpers can mutate environment or battery directly.

**Why this breaks the architecture**
- Fault semantics should be **data on the bus**, sample-and-hold, recordable.
- Direct mutation of model objects bypasses the bus and creates hidden state.

**Plan**
- Treat `FaultState` as the only supported fault injection mechanism for:
  - motor disable/stuck/scale (extend the fault schema if needed)
  - battery disconnect
  - estimator freeze / sensor fault masks
- Remove or quarantine mutating failure hooks like `set_motor_enabled!`.

**Definition of done**
- Disabling a motor can only be done by setting a bit in `FaultState` (or equivalent explicit bus field).

---

### [ ] T0.3 — Stop synchronizing component state into the plant (“sync shims” should not be part of the normal path)

**What I saw**
- `src/sim/Plant.jl` includes compatibility hooks to sync component objects from `PlantState`.

**Why this breaks the architecture**
- Any future code that reads component state instead of `PlantState` will silently become dependent on call ordering.

**Plan**
- Make the canonical path: **PlantState is the only truth**.
- If you need legacy compatibility for visualization/debugging, put it behind an explicit function call (off the critical path) and clearly label it `Legacy`.

---

### [ ] T0.4 — Restrict `Events.When(...)` closures (or formally bless them as “unsafe”)

**What I saw**
- `Events.When(condition, action)` can execute arbitrary user code with access to the sim object.

**Why this breaks the architecture**
- It makes it impossible to reason about what state can change at boundaries, and impossible to guarantee record/replay unless you record *everything*.

**Plan (pick one)**
- Option A: Keep `When`, but restrict actions to returning a typed “scenario output” (faults, commands, etc.) rather than mutating sim.
- Option B: Keep it but label it explicitly as `UnsafeWhen` and keep it out of core workflows.

**Definition of done**
- Either the event system is pure (preferred) or its unsafe nature is explicit and quarantined.

---

## P1 — “Tacky” but survivable (cleanup as you extend)

### [ ] T1.1 — Remove reflection-driven logging (`hasproperty`) from the runtime engine

**Why**
- Logging shouldn’t be the place where “unknown payload shapes” are handled.

**Plan**
- Convert source outputs into typed structs at the source boundary, then log those.

---

### [ ] T1.2 — Decide whether the engine supports true hybrid event detection (zero-crossing) or discrete sampling semantics

**Why**
- Right now, `When` conditions are evaluated at boundaries. That is fine if you define it as the semantics.
- If you want “true” hybrid semantics later, you’ll need event detection + step revision.

**Plan**
- Write a short decision doc:
  - If discrete semantics: document it; add tests that enforce it.
  - If true hybrid: add groundwork for event insertion + interval subdivision.

