# Plant Output Protocol

## Role

`src/sim/PlantInterface.jl` defines the `plant_outputs` protocol used by engines to
query algebraic plant outputs at event boundaries.

## Key Decisions and Rationale

- **Protocol function (no fallback):** `plant_outputs` is a generic function without a
  default implementation, so engines must check `applicable` before calling.
- **Engine‑agnostic:** keeps battery/rotor telemetry logic inside the plant RHS instead
  of duplicating it across engines.

## Integration Contracts

- Implementations must be pure and deterministic.
- Engines should call `plant_outputs` only at event boundaries.
- `PlantModels.CoupledMultirotorModel` supplies the default implementation.

Additional protocol
-------------------
`plant_project(f, x)` may be implemented by a plant model to enforce deterministic
hard bounds (e.g., ω ≥ 0, SOC ∈ [0,1]) after each accepted integration interval.

## Caveats

- Not all RHS functors implement the protocol; callers must guard with `applicable`.
