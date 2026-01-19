# TODO 02 — Correctness / Determinism (Julia sim side)

Scope: **PX4Lockstep.Sim (Julia)** only.

This TODO list is strictly about *bugs*, *invariants*, and *determinism*. Anything here that’s not fixed eventually becomes a “why did record/replay diverge?” debugging session.

---

## P0 — Fix before trusting record/replay

### [ ] C0.1 — Fix wind being applied inconsistently (propulsion uses bus wind, drag uses env wind)

**What’s wrong (confirmed in code)**
- Propulsion air-relative velocity uses **`u.wind_ned`** (bus forcing):
  - `src/sim/PlantModels/CoupledMultirotor.jl` → `_eval_propulsion_and_bus(...)`:
    - `v_air_ned = u.wind_ned - x.rb.vel_ned`
- Rigid-body drag uses **`env.wind`** (generator), not bus forcing:
  - `src/sim/Vehicles.jl` → `dynamics(model, env, t, x, ...)`:
    - `wind_ned = wind_velocity(env.wind, x.pos_ned, t)`
    - `v_air_ned = wind_ned - x.vel_ned`

**Why this is a correctness bug**
- In replay workflows, the environment is intentionally built with `NoWind()` while wind is replayed from the recording.
- Result: propulsion sees the recorded wind but drag sees zero wind, so replay is physically inconsistent.

**Recommended fix**
- Make wind a strict **input** to the dynamics path:
  - Either:
    1) Add `wind_ned` as an argument to `Vehicles.dynamics(...)`, or
    2) Move the drag term out of `Vehicles.dynamics(...)` and compute it in the plant RHS using `u.wind_ned`.
- Ensure **all** air-relative calculations in the plant use the same `u.wind_ned`.

**Definition of done**
- There is no call site in any plant RHS that uses `env.wind` to compute air-relative velocity.
- Replay with recorded wind produces the same trajectory as record (within integrator tolerance).

**Regression tests (must add)**
- Add a unit test that constructs a minimal plant + environment, records a short run with a non-zero wind trace, replays it with `env.wind = NoWind()`, and asserts:
  - the rigid-body state matches within a small tolerance,
  - the computed drag term matches between record and replay.

---

### [ ] C0.2 — Add a Tier-0 “record then replay” determinism test for the canonical engine

**Goal**: Catch *any* future hidden dependency on non-recorded inputs.

**Plan**
- Use a deterministic scenario (or `NullScenarioSource`) and a known wind trace.
- Record Tier-0 streams (`cmd`, `wind_ned`, `faults_evt` optionally) + log axis.
- Replay open-loop (ReplayAutopilotSource + ReplayWindSource + ReplayScenarioSource).

**Assertions**
- Plant state at each log time matches within epsilon.
- If you expect bit-exact determinism for a specific integrator + quantization mode, add a stricter “exact match” mode.

**Where to implement**
- Tests can be placed under `test/` using the existing Verification utilities.

---

### [ ] C0.3 — Enforce “plant_outputs is pure and consistent”

**Why**
- `plant_outputs(...)` is used for boundary-time telemetry (battery) and logging.
- If it mutates any model object or disagrees with the integrated RHS assumptions, you get subtle divergence.

**Risk areas**
- Battery models and propulsion sets are mutable objects, even though the integrated state lives in `PlantState`.

**Plan**
- Add a test that calls `plant_outputs(dynfun, t, x, u)` twice and asserts:
  - `x` is unchanged,
  - `dynfun`’s internal component objects (battery/propulsion) are unchanged (or, better, make them immutable params).
- Add a consistency check test that compares `plant_outputs(...).battery_status` against what the RHS computes internally (within tolerance).

---

### [ ] C0.4 — Validate + clamp actuator commands at the engine boundary

**Why**
- A single NaN or out-of-range command can poison the integrator and create “random” divergence.

**Plan**
- Implement `validate(cmd::ActuatorCommand)` / `sanitize(cmd)` that:
  - checks `isfinite` for all channels,
  - clamps motors/servos to `[0,1]` (or the intended range),
  - optionally errors in `MODE_RECORD` to fail fast.
- Call it in `Runtime.Engine.process_events_at!` immediately after `update!(autopilot, ...)` and before recording `:cmd`.

**Definition of done**
- Bad commands fail fast (or are clamped deterministically) with a clear error.

---

### [ ] C0.5 — Remove float-time edge cases in gust duration handling (use microsecond end-times)

**Why**
- Core scheduling is microsecond-quantized, but some wind models store step end-times in Float64 seconds.
- This can create off-by-one-tick behavior at long durations or on boundary-aligned gusts.

**Where**
- `src/sim/Environment.jl`: `OUWind.step_until_s`, `GustStep.t_on/t_off`

**Plan**
- Internally store gust start/end as `UInt64` microseconds.
- Evaluate gust activation using the engine’s authoritative `t_us` (or wind source’s `t_us`).

**Definition of done**
- Gust activation/deactivation is strictly aligned to the same microsecond grid used everywhere else.

---

## P1 — Important correctness edge cases

### [ ] C1.1 — Ensure log axis always captures a final snapshot (or make the behavior explicit)

**Issue**
- If `t_end_us` is not exactly on the periodic log axis, the final plant state won’t be logged.

**Plan (pick one)**
- Option A: Always include `t_end_us` in `timeline.log` (even if it breaks strict periodicity), or
- Option B: Keep periodicity, but add an explicit final snapshot record after the run.

**Definition of done**
- The behavior is deterministic and documented, and Tier-0 recordings have a predictable “final state” story.

---

### [ ] C1.2 — Make `Events.AtStep` semantics either supported or impossible to misuse

**Issue**
- `Events.AtStep` expects `sim.step`, but `Runtime.Engine` does not expose `step`.
- That means `AtStep` silently works in some contexts and errors in others.

**Plan (pick one)**
- Option A: Add a step counter to `Runtime.Engine` and define what “step” means (event boundary count? physics dt count?).
- Option B: Remove `AtStep` or restrict it to specific sim types.

**Definition of done**
- No user can accidentally schedule `AtStep` on the canonical engine and get a runtime error mid-flight.

---

### [ ] C1.3 — Add ordering test: scenario → wind → derived outputs → autopilot

**Why**
- The engine relies on stage ordering so a scenario can inject a gust/fault at boundary time `t_us` and have wind/autopilot see it on that same boundary.

**Plan**
- Create a minimal “test scenario” that sets a fault and/or modifies wind at a boundary.
- Assert that:
  - wind source sees it within the same `process_events_at!` call,
  - autopilot sees the derived telemetry consistent with that fault.

---

## P2 — Long-term verification

### [ ] C2.1 — Property-based tests for timeline/scheduler invariants

**Goal**: Kill entire classes of “off by one boundary” bugs.

**Examples**
- `evt.t_us` strictly increasing.
- `evt` always contains `t0_us` and `t_end_us`.
- Every axis member time appears in `evt`.
- `consume_boundary!` / `advance_evt!` never skip due events.

