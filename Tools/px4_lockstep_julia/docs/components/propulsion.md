# Propulsion Models

## Role

`src/sim/Propulsion.jl` converts actuator duties into rotor speed, thrust, torque, and
bus current. It is the primary coupling between controller commands and rigid-body
forces.

## Key Decisions and Rationale

- **Quasi-static motor model:** ignores inductance and solves current algebraically;
  this matches flight-dynamics time scales and keeps the RHS deterministic.
- **Quadratic propeller model:** stable baseline for multirotors without table-driven
  aero dependencies; optional inflow correction captures axial inflow effects.
- **Separated geometry:** rotor positions and directions live in the vehicle/rotor set
  so propulsion can be reused across airframes.
- **Calibrated defaults:** `default_iris_quadrotor_set` tunes prop constants to hit a
  reasonable hover thrust at nominal voltage/density.

## Integration Contracts

- Propulsion outputs are consumed by vehicle dynamics and battery models.
- Rotor direction for yaw torque is encoded in `QuadRotorSet.rotor_dir`.
- Motor enable flags are handled per `MotorPropUnit` to support failure injection.

## Caveats

- Electrical inductance and ESC dynamics are ignored; current is solved algebraically.
- Motor current is clamped to `max_current_a`, and duties below the ESC deadzone produce
  zero bus current.
- The quadratic prop model is calibrated for hover-like conditions and can be inaccurate
  at high advance ratios.
- Inflow correction only accounts for axial velocity; cross-flow effects are ignored.
