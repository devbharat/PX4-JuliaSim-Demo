# Propulsion Models

## Role

See also: **[Frames and sign conventions](../reference/conventions.md)**.

`src/sim/Propulsion.jl` maps actuator duties (PWM/normalized commands) into per-rotor
states and forces:

- rotor speed `ω`
- thrust and reaction torque
- motor/bus current draw

It is the primary coupling between controller commands and rigid-body forces for the
multirotor plant models.

## Key decisions and rationale

- **Quasi-static motor electrical model:** ignores inductance and solves current
  algebraically. This matches flight-dynamics time scales and remains deterministic.
- **Quadratic propeller model:** stable baseline for multirotors without table-driven
  aero dependencies; optional inflow correction accounts for axial inflow.
- **Separation of geometry vs. dynamics:**
  - vehicle models own rotor positions and axes
  - propulsion owns motor/prop parameters and rotor spin direction for yaw reaction
- **Calibrated defaults:** `default_multirotor_set` provides a reasonable generic
  multirotor baseline intended for closed-loop testing (not high-fidelity aero).
- **TOML-driven parameters:** the default motor/prop set is configured via
  `[airframe.propulsion]` with optional `[airframe.propulsion.esc]` and
  `[airframe.propulsion.motor]` sub-tables in the aircraft spec.

## Integration contracts

- `MotorPropUnit` and `QuadRotorSet` are **parameter containers**; failure injection is
  expressed via the bus-level `Faults.FaultState` and applied by the plant model
  (e.g. duty clamping, battery disconnect), not by mutating propulsion objects.
- The primary output is `RotorOutput{N}`:
  - `thrust_n`, `shaft_torque_nm`, `ω_rad_s`, `ω_dot_rad_s2`
  - `motor_current_a` (per rotor)
  - `bus_current_a` (total draw, sign convention: positive = discharge)
- Rotor direction for yaw reaction torque is encoded in `QuadRotorSet.rotor_dir`.
  Rotor **spin** direction is opposite this sign (reaction torque is on the body),
  which is what vehicle dynamics use for gyroscopic coupling.
- Rotor axial inertia (kg·m²) is exposed via `rotor_inertia_kgm2(unit)` and is used
  by vehicle dynamics for rotor gyroscopic coupling.

## Caveats

- Electrical inductance and ESC internal dynamics are ignored; current is solved
  algebraically.
- Motor current is clamped to `BLDCMotorParams.max_current_a`, and duties below the ESC
  deadzone produce zero bus current.
- The quadratic prop model is calibrated for hover-like conditions and can be inaccurate
  at high advance ratios.
- Inflow correction only accounts for axial velocity; cross-flow effects are ignored.
