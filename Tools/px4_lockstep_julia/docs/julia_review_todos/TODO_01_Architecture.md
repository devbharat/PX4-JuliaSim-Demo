# TODO 01 — Architecture (Julia sim side)

Scope: **PX4Lockstep.Sim (Julia)** only.

This TODO list is about structural changes that improve determinism, composability, and maintainability, with an eye toward supporting **VTOL/fixed-wing** plant models later.

## Guiding principles (keep these invariant)

1. **Single authoritative timebase:** `UInt64` microseconds everywhere. Float seconds are derived/for convenience.
2. **Explicit coupling contract:** the **bus** is the only coupling surface between discrete-time sources and the continuous-time plant.
3. **Deterministic hybrid semantics:** all discrete updates happen at explicit event boundaries; continuous dynamics integrate between boundaries under piecewise-constant inputs.
4. **No hidden mutable state:** continuous-time behavior must be a function of `(t, x, u, params)`; if anything is time-varying and sampled, it should be in `u`.

---

## P0 — Structural issues that will block scaling

### [ ] A0.1 — Make “bus as the contract” a hard rule (remove remaining hidden side channels)

**Problem**: Some plant dynamics still read environment state directly (not through the bus), which breaks the “record bus → replay open-loop” model.

**Where**
- `src/sim/Vehicles.jl`: `dynamics(model, env, t, x, ...)` computes drag using `env.wind`.
- `src/sim/PlantModels/CoupledMultirotor.jl`: propulsion uses `u.wind_ned` (bus).

**Plan**
- Update the vehicle/plant-side APIs so *all sampled disturbances* (wind, injected gusts, etc.) come through `PlantInput`.
- Treat `EnvironmentModel` as **parameters / slowly varying fields** (atmosphere model, gravity model, reference origin), not as a sampled disturbance source.

**Definition of done**
- No plant RHS calls `wind_velocity(env.wind, ...)` (or equivalent) anywhere.
- A Tier-0 record/replay produces identical plant evolution (modulo integrator tolerances) when replay uses recorded `wind_ned`.

**Notes**
- This overlaps with correctness TODO C0.1 (wind mismatch) but is fundamentally an architecture invariant.

---

### [ ] A0.2 — Remove duplicated state ownership (stop “syncing” mutable components from plant state)

**Problem**: State lives in both `PlantState` and mutable component objects, kept consistent via `sync_components_from_plant!` and other shims. This creates multiple sources of truth and makes it easy to accidentally introduce record/replay divergence.

**Where**
- `src/sim/Plant.jl`: `sync_components_from_plant!` / `sync_components_to_plant!`
- `src/sim/Propulsion.jl`: `MotorPropUnit` contains `ω_rad_s` and `enabled`, but the plant integrates `rotor_ω` separately.
- `src/sim/Powertrain.jl`: battery models are mutable and store SOC/V1, but plant integrates `power.soc`/`power.v1`.

**Plan (recommended)**
- Split each “component model” into:
  - **Params (immutable)**: geometry/coefficients
  - **State (in PlantState)**: the continuous state vector
- Make the plant RHS depend on params + plant state only.
- Keep legacy “mutable component state” only behind explicit compatibility layers (or remove entirely).

**Definition of done**
- A `PlantModel` can be evaluated without mutating any component objects.
- The only mutable runtime state is the engine’s `bus`, `plant`, and *sources* (scenario/autopilot/wind/etc.).

---

### [ ] A0.3 — Decompose `CoupledMultirotorModel` into explicit sub-models (forces/moments + power + actuators)

**Problem**: `CoupledMultirotorModel` currently mixes:
- actuator dynamics
- rotor electro-mechanics
- bus voltage solving
- battery dynamics
- rigid-body forces/moments
- (minimal) aero drag
- (future) contacts

This makes it hard to swap any one subsystem (e.g., a better aero model, a different rotor model, VTOL tilt mechanics).

**Where**
- `src/sim/PlantModels/CoupledMultirotor.jl`

**Plan**
- Introduce clear internal interfaces (not necessarily public) with narrow responsibilities:
  - `ActuatorDynamics` (motors/servos command lag)
  - `PropulsionModel` (rotor thrust/torque + ω dynamics)
  - `ElectricalBusModel` (bus voltage/current solve; includes battery coupling)
  - `AeroModel` (forces/moments from air-relative velocity + surfaces)
  - `RigidBodyModel` (EoM integration)
  - `ContactModel` (ground reaction, landing gear)
- Make the coupled RHS “wire” these pieces in one place.

**Definition of done**
- You can swap (at construction) the aero model and propulsion model without editing the rigid-body or battery code.
- Adding a **fixed-wing aero model** does not require editing the multirotor propulsion code.

---

### [ ] A0.4 — Strengthen typing across the Runtime engine (eliminate `Any` in hot interfaces)

**Problem**: Several runtime-facing structs store `Any`, which makes type stability and performance harder to reason about and tends to hide interface drift.

**Where**
- `src/sim/Runtime/Engine.jl`: `EngineOutputs.plant_y::Any`, `EngineStats.last_integrator_stats::Any`
- `src/sim/Sources/Autopilot.jl`: `LiveAutopilotSource.last_out::Any`

**Plan**
- Parameterize these on concrete types:
  - `EngineOutputs{Y}` where `Y` is the plant output type (or `Union{Nothing,Y}`)
  - `EngineStats{S}` where `S` is `IntegratorStats`
  - `LiveAutopilotSource{AP,Out}` (or store a small typed “telemetry struct”)

**Definition of done**
- `@code_warntype` (or JET) on `Runtime.run!` and `plant_rhs` does not show `Any` from these paths.

---

## P1 — Architecture improvements that simplify future features

### [ ] A1.1 — Formalize a “vehicle definition” layer (data + mapping)

**Problem**: To support VTOL/fixed-wing, the system needs a first-class way to define:
- geometry (mass/inertia, rotor locations, control surface axes)
- actuator mapping (PX4 output indices → plant actuators)
- propulsion groups (tilt rotors vs lift rotors vs pusher)

**Plan**
- Introduce a `VehicleDefinition` (or similar) that owns:
  - parameter structs
  - indexing/mapping of `ActuatorCommand` into plant inputs
- Ensure the mapping is explicit and versionable.

**Definition of done**
- A VTOL can define: 4 lift rotors + 1 pusher + 2 tilt servos using the existing `ActuatorCommand` packet without ad-hoc indexing scattered across code.

---

### [ ] A1.2 — Make logging sinks schema-aware (avoid hard-coded “first 4 rotors”)

**Problem**: Current log emission extracts only 4 rotor ω / thrust values.

**Where**
- `src/sim/Runtime/Engine.jl`: `_rotor_omega_tuple`, `_rotor_thrust_tuple`

**Plan**
- Either:
  - log `Vector`/`SVector` fields (if sinks can handle), or
  - log an explicit `N` + packed arrays, or
  - define per-vehicle “log view” mapping.

**Definition of done**
- A vehicle with `N != 4` doesn’t silently drop propulsion telemetry.

---

### [ ] A1.3 — Reduce reflection in telemetry (`hasproperty` patterns)

**Problem**: `_autopilot_log_fields` uses `hasproperty` and ad-hoc field probing. This is brittle.

**Plan**
- Define an explicit `AutopilotTelemetry` struct populated by autopilot sources.
- Log sinks consume this struct.

**Definition of done**
- Adding/removing fields from the autopilot output does not require editing `Runtime.Engine`.

---

### [ ] A1.4 — Decide how “scenario events” should work long-term (pure outputs vs mutation)

**Problem**: Today, events can mutate `sim` (e.g., changing wind model in-place). This is flexible, but it’s a side channel.

**Plan options**
- Option A (keep mutation but sandbox it):
  - allow scenario to mutate only *scenario-owned* state (faults/cmd) and publish requests on the bus
  - create a dedicated “disturbance request” channel for wind steps, sensor faults, etc.
- Option B (make scenarios pure):
  - scenario returns `{ap_cmd, landed, faults, disturbance_requests}`
  - engine applies requests at canonical stages

**Definition of done**
- Scenario actions that affect the plant are visible as explicit bus signals (recordable), not hidden object mutations.

---

## P2 — Longer-term refactors (only after P0/P1)

### [ ] A2.1 — Add a first-class “AeroModel” interface

Deliverables:
- `aero_forces_moments(model, env, t, rb_state, air_state, control_state) -> (F_ned, M_body)`
- A baseline fixed-wing implementation (lift/drag/moment coefficients + control surfaces)

### [ ] A2.2 — Add a first-class “PropulsorModel” interface

Deliverables:
- Support both:
  - multirotor rotors (vertical thrust)
  - forward propulsors (pusher/puller)
  - tilt rotors (variable thrust axis)

### [ ] A2.3 — Add a “ground contact” subsystem with explicit discontinuities

Deliverables:
- contact forces computed in a module that can be swapped
- any discontinuities are applied via `plant_on_autopilot_tick` or a new boundary hook
