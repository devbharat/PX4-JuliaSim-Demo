# Noise Utilities

## Role

`src/sim/Noise.jl` provides deterministic noise and bias processes used by estimators
and wind models.

## Key Decisions and Rationale

- **Explicit RNG input:** callers supply RNGs so streams can be partitioned and
  reproduced.
- **Exact discrete OU/AR(1):** bias processes use closed-form discrete updates with
  cached coefficients so variance remains stable as `dt` changes.

## Integration Contracts

- Noise is injected only at discrete update boundaries, not inside ODE substeps.

## Caveats

- AR(1)/OU processes are discrete-time; changing `dt` changes the realized statistics.
- Non-finite or non-positive time constants disable the process evolution.
