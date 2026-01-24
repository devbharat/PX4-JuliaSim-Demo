# Vehicles and Actuators

## Role

See also: **[Frames and sign conventions](../reference/conventions.md)**.

`src/sim/Vehicles.jl` defines:

- the vehicle model interface (`mass`, `inertia_diag`, `dynamics`)
- reusable actuator dynamics (direct/first-order/second-order)
- a minimal multirotor rigid-body model (including the Iris defaults)

This layer is intentionally propulsion-agnostic: propulsion models compute thrust and
shaft torques; vehicle models consume those and produce rigid-body derivatives.

## Key decisions and rationale

- **Fixed-size actuator commands:** aligns with the lockstep ABI and keeps engine code
  airframe-agnostic.
- **Actuator dynamics spectrum:** direct, first-order (exact discretization), and
  second-order (semi-implicit Euler with optional rate limits) cover common actuator
  behaviors without heavyweight modeling dependencies.
- **Minimal multirotor model:** prioritizes stable, high-signal closed-loop behavior for
  PX4-in-the-loop runs over high-fidelity aerodynamic effects.

## Integration contracts

- `mass(model) -> Float64` exposes vehicle mass so external forces (e.g. contact)
  can be applied without hard-coding airframe types.
- `inertia_diag(model) -> Vec3` returns the diagonal body-frame inertia used by the
  rigid-body integrators.
- `dynamics(model, env, t_s, x, u, wind_ned) -> RigidBodyDeriv` computes the 6DOF
  rigid-body derivative.

  - `x` is a `RigidBodyState` (NED position/velocity, body→NED quaternion, body rates).
  - `u` is vehicle-specific. For multirotors it is typically `Propulsion.RotorOutput{N}`
    (thrust + shaft torques).
  - `wind_ned` must come from the **runtime bus** (sample-and-hold). Vehicle dynamics
    should not read `env.wind` directly, so record/replay uses identical forcing.

## Caveats

- The multirotor model is intentionally low fidelity (simple drag and damping) and is
  not a validated aero model.
- Rigid-body inertia is modeled as **diagonal only** (`inertia_diag`); off-diagonal
  products of inertia are not represented.
- Rotor inertia is modeled inside propulsion units (`BLDCMotorParams.J`) for rotor-speed
  dynamics, but **gyroscopic coupling into the rigid body is not modeled** (no rotor
  angular momentum effects on body rates).
- Second-order actuator dynamics use a semi-implicit Euler step; large `dt` can reduce
  accuracy or stability.
