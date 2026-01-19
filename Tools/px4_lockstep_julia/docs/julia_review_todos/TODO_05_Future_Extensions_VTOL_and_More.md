# TODO 05 — Future Extensions: VTOL, Fixed-wing, Other Drones (Julia sim side)

Scope: **PX4Lockstep.Sim (Julia)** only.

This file is about what the Julia architecture needs so the *same engine* can simulate:
- multirotors (today)
- tilt-rotor / tailsitter VTOL
- conventional fixed-wing
- other drone variants with different aero/propulsion models

The intent is to keep the **canonical Runtime engine** unchanged, and make the plant model + mappings swappable.

---

## P0 — Preconditions for adding non-multirotor vehicles

### [ ] F0.1 — Define a formal “vehicle definition” layer (geometry + actuator mapping + model composition)

**What you need for VTOL/FW**
- A way to describe, in data and code:
  - mass/inertia
  - aerodynamic reference areas and coefficient models
  - propulsion units (which bus motor index drives which propulsor)
  - control surfaces (which bus servo index drives which surface)
  - rotor locations and axes (tilt mechanisms)

**Deliverable**
- `VehicleDefinition` (or `PlantConfig`) struct that contains:
  - `rigidbody::RigidBodyModel`
  - `aero::AbstractAeroModel` (possibly `nothing` for pure multirotor)
  - `propulsion::AbstractPropulsionModel`
  - `actuators::AbstractActuatorModel`
  - `contact::AbstractContactModel`
  - `mapping::ActuatorMapping` (bus → local commands)

**Definition of done**
- You can instantiate Iris and a “toy fixed-wing” from two different definitions without touching `Runtime.Engine`.

---

### [ ] F0.2 — Introduce an `ActuatorMapping` that decouples PX4 output array indices from plant internals

**Why**
- PX4 gives you arrays (`motors[1:12]`, `servos[1:8]`).
- VTOL/fixed-wing needs semantic meaning: e.g. `left_aileron`, `right_elevon`, `tilt_servo_1`, `pusher_motor`, etc.

**Plan**
- Define:
  - `map_actuators(mapping, cmd::ActuatorCommand, x, t) -> LocalActuatorCommands`
- For multirotor: mapping is mostly identity.
- For fixed-wing: mapping assigns `servos` to named surfaces.
- For tilt-rotor: mapping routes both thrust + tilt commands.

**Definition of done**
- Plant models never directly index into `cmd.motors[i]` except through the mapping.

---

### [ ] F0.3 — Add an `AbstractAeroModel` interface (component build-up style)

**Why**
- Multirotor today uses a simple linear drag. Fixed-wing requires lift/drag/moment models.

**Plan (minimal viable)**
- Define an interface:
  - `aero_forces_moments(model, env, t, x_rb, v_air_body, ω_body, control_surfaces) -> (F_body, M_body)`
- Start with a toy model:
  - coefficient-based lift/drag (CL(α), CD(α), Cm(α))
  - control-surface deflection terms

**Definition of done**
- A fixed-wing plant can fly a trim condition with PX4 in SITL (even roughly) using the same engine.

---

### [ ] F0.4 — Generalize propulsion beyond “QuadRotorSet”

**Why**
- VTOL may have multiple prop groups (lift rotors + pusher).
- Fixed-wing may have 1–2 propulsors.

**Plan**
- Replace `QuadRotorSet` with a more general `PropulsorSet`:
  - each propulsor has:
    - axis (body frame)
    - location (for moments)
    - direction (reaction torque sign)
    - motor/prop model params
- Ensure `RotorOutput` generalizes to `PropulsorOutput`.

**Definition of done**
- The same propulsion code can evaluate N propulsors with arbitrary axes/positions.

---

## P1 — VTOL-specific features

### [ ] F1.1 — Add tilt mechanisms as explicit states (or explicit algebraic constraints)

**Why**
- Tilt-rotors have servo-driven tilt angles that affect prop axis and moment arm.

**Plan**
- Decide:
  - treat tilt angles as first-order actuator states (in `PlantState`), or
  - treat them as direct (snap-to-command at autopilot tick).
- Include tilt angles in the force/moment calculation:
  - prop axis rotates with tilt.

**Definition of done**
- A tilt-rotor plant model can transition from hover to forward flight with reasonable forces.

---

### [ ] F1.2 — Support mixed control allocation (PX4 “VTOL” mixers) cleanly

**Why**
- PX4 changes actuator output semantics across VTOL modes.

**Plan**
- Keep the bus fixed-size arrays, but make `ActuatorMapping` mode-aware:
  - it can look at `bus.ap_cmd` / nav state / or a “mode” field provided by autopilot telemetry.

**Definition of done**
- The mapping can interpret PX4 outputs correctly in each mode without changing the engine.

---

## P2 — Quality and usability

### [ ] F2.1 — Data-driven vehicle configs (TOML) + validation

**Plan**
- Provide a `vehicle.toml` schema and a constructor.
- Add validation for:
  - consistent rotor counts,
  - inertia positive-definite,
  - actuator indices in range,
  - surface definitions.

### [ ] F2.2 — Add a “model library” folder

**Goal**
- Keep reusable model components (aero tables, vehicle configs) out of the core runtime code.

