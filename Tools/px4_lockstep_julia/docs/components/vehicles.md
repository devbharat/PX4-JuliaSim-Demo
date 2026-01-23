# Vehicles and Actuators

## Role

`src/sim/Vehicles.jl` defines the vehicle model interface, actuator dynamics, and the
baseline Iris quadrotor dynamics used for PX4-in-the-loop testing.

## Key Decisions and Rationale

- **Fixed-size actuator commands:** aligns with the lockstep ABI and keeps engine code
  airframe-agnostic.
- **Actuator dynamics spectrum:** direct, first-order (exact discretization), and
  second-order (semi-implicit Euler with optional rate limits) cover common actuator
  behaviors without heavy modeling overhead.
- **Minimal Iris model:** prioritizes stability and closed-loop signal over high-fidelity
  aero, enabling fast iteration on PX4 integration.

## Integration Contracts

- `dynamics(model, env, t, state, u)` returns `RigidBodyDeriv` in NED/body frames.
- Rotor torque sign is supplied by the propulsion model, not the vehicle model.

## Caveats

- The Iris model is intentionally low fidelity (simple drag and damping) and is not a
  validated aero model.
- Rigid-body inertia is modeled as **diagonal only** (`inertia_diag`); off-diagonal products
  of inertia are not represented.
- Rotor inertia is modeled inside the propulsion units (`BLDCMotorParams.J`) for rotor
  speed dynamics, but **gyroscopic coupling into the rigid body is not modeled** (no rotor
  angular momentum effects on body rates).
- Second-order actuator dynamics use a semi-implicit Euler step; large `dt` can reduce
  accuracy or stability.
- Non-quadrotor or aerodynamic models require new `dynamics` implementations and
  parameter sets.
