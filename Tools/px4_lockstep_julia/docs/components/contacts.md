# Contacts

## Role

`src/sim/Contacts.jl` defines optional contact forces (currently a flat ground plane).
It exists to keep contact handling explicit and deterministic without hard-coding it
into the plant dynamics.

## Key Decisions and Rationale

- **Penalty-force model:** compliant spring/damper normal force with smooth Coulomb
  friction keeps integration stable without event detection.
- **Force-at-COM only:** contact is applied at the center of mass to avoid modeling
  moments or landing-gear geometry; this keeps the model deterministic but limits
  touchdown fidelity.

## Integration Contracts

- Contact models return an external force in NED for the rigid-body integrator to apply.
- Flat-ground contact assumes NED down-positive with the ground plane at `z = 0`.

## Caveats

- Contact forces are applied at the center of mass only; no moments or landing-gear
  geometry are modeled.
- Penalty methods require sufficiently small `dt` to limit penetration; there is no
  general event detection.
- Friction is a smooth Coulomb approximation and does not model stick-slip.
