# Rigid-Body Core

## Role

See also: **[Frames and sign conventions](../reference/conventions.md)**.

`src/sim/RigidBody.jl` defines the minimal 6DOF state and update helpers shared across
all vehicle models and integrators.

## Key Decisions and Rationale

- **Quaternion attitude (body → NED):** avoids Euler singularities and matches PX4
  conventions.
- **Normalization on update:** `rb_add` and `rb_scale_add` renormalize once per update to
  contain drift without expensive orthogonalization.
- **Body-rate kinematics:** quaternion derivatives use body rates with a body→NED
  convention, keeping frame semantics explicit.
- **Model-agnostic design:** forces and moments are delegated to vehicle models, keeping
  the core state independent of airframe details.

## Integration Contracts

- Vehicle models must return `RigidBodyDeriv` consistent with NED and body-frame
  conventions.

## Caveats

- Quaternion normalization on every update can slightly damp energy; it is chosen for
  stability over exact energy conservation.
- No constraints are enforced beyond normalization; integration accuracy depends on the
  chosen integrator and timestep.
