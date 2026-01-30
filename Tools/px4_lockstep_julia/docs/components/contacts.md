# Contacts

## Role

`src/sim/Contacts.jl` defines optional contact forces (currently a flat ground plane).
It exists to keep contact handling explicit and deterministic without hard-coding it
into the plant dynamics.

## Key Decisions and Rationale

- **Two contact styles are supported:**
  - **Penalty-force model** (`FlatGroundContact`): compliant spring/damper normal force
    with smooth Coulomb friction.
  - **Constraint + hybrid contact model** (`FlatGroundConstraintContact`): parameters for
    a non-stiff unilateral normal reaction *and* a hybrid event/impulse contact protocol:
    - **Impact map (Phase 3+)**: restitution + optional tangential impact friction.
    - **Grounded time-stepping (Phase 4+)**: fixed substeps + a 1-contact impulse solve
      for stable rest and stick-slip-like behavior.
- **Force-at-COM only:** contact is applied at the center of mass to avoid modeling
  moments or landing-gear geometry; this keeps the model deterministic but limits
  touchdown fidelity.

## Integration Contracts

- **Penalty contact** (`FlatGroundContact`) is pure continuous forcing: it returns an
  external force in NED for the rigid-body integrator to apply.
- **Constraint/hybrid contact** (`FlatGroundConstraintContact`) has two usage modes:
  - *Continuous grounded-mode reaction:* a non-stiff unilateral normal reaction computed
    from the unconstrained acceleration (`a_free`). This keeps explicit adaptive solvers
    from shrinking steps due to stiff penalty forces.
  - *Hybrid grounded time-stepping:* plant models may override integration using the
    `plant_integrate_interval` protocol and apply end-of-substep contact impulses instead
    of continuous forces. In this mode, the plant integrates the **smooth** dynamics with
    `NoContact()` and applies a deterministic impulse solve in grounded mode.
  - *Safe airborne stepping:* when approaching the guard, plant models may also
    cap the next airborne integration chunk to O(time-to-impact) based on the current
    gap and closing speed. TOI localization (lookahead + bisection) should always use
    the *free* dynamics (`NoContact()`), not the contact-enabled RHS.
- Flat-ground contact assumes NED down-positive with the ground plane at `z = 0`.

## Impact telemetry and acceleration estimates

For the hybrid/constraint contact path, a plant may return **interval metadata** from
`plant_integrate_interval(...)` describing impacts that occurred *inside* the integration
interval (i.e., at a time-of-impact localized to integer microseconds).

The runtime currently latches and exposes the following bus signals for logging:

- `bus.impact_dv_ned` — the maximum impact **Δv** (NED, m/s) since the last log sample.
- `bus.impact_time_us` — the timestamp (us) of that max-Δv impact.
- `bus.impact_count` — number of impacts since the last log sample.
- `bus.impact_accel_est_ned` — a **1-tick equivalent acceleration estimate** (NED, m/s^2):
  `impact_accel_est_ned ≈ impact_dv_ned / dt_autopilot`.

Why an estimate?
Impacts are impulse-like (discontinuous velocity changes), so they are not representable as
a continuous-time acceleration without introducing stiffness. The `impact_accel_est_ned`
signal exists so that low-rate logs (e.g., 50–100 Hz) can still capture landing spikes
in a single sample.

`dt_autopilot` is derived from the sim's autopilot axis period (`timeline.ap`).

## Caveats

- Contact forces are applied at the center of mass only; no moments or landing-gear
  geometry are modeled.
- Penalty methods require sufficiently small `dt` to limit penetration; there is no
  general event detection.
- The constraint contact makes **resting contact** non-stiff, but by itself it does not
  localize touchdown time. Hybrid/event-based impact handling (TOI + impact map) is
  required for high-fidelity landings at coarse `dt`.
- The penalty friction model is smooth (no stick-slip). Grounded time-stepping can
  approximate stiction via a tangential impulse bound (Coulomb cone over each substep).
