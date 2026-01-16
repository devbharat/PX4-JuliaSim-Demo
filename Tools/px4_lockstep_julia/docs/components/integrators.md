# Integrators

## Role

`src/sim/Integrators.jl` provides fixed-step and adaptive ODE solvers for both rigid
body and full plant states. The implementations are explicit to keep behavior
deterministic and reviewable.

## Key Decisions and Rationale

- **In-house RK methods:** avoids dependency on large ODE frameworks and keeps solver
  behavior explicit and reviewable.
- **Deterministic error norms:** adaptive solvers use a max/∞-norm over scaled
  position/velocity/ω plus a geodesic quaternion angle error to avoid mixing units.
- **Quantized adaptive steps:** optional microsecond quantization prevents long-run
  drift and aligns with lockstep time; remaining-time snapping avoids extra 1 µs tails.
- **Opt-in plant error control:** additional actuator/rotor/battery tolerances are
  ignored by default to preserve legacy rigid-body behavior.

## Integration Contracts

- Inputs are piecewise constant across each integration interval.
- RHS functions must be pure (no RNG, no mutation).
- `reset!` clears adaptive step history and should be called when reusing integrators
  for deterministic comparisons.

## Caveats

- Adaptive solvers default to 1 µs quantization; for sub-microsecond dynamics, disable
  `quantize_us` and manage drift explicitly.
- `max_substeps` and `h_min` guardrails can trigger errors if the interval is too long
  or tolerances are too tight.
- Full-plant error control requires finite `atol_*` values and `plant_error_control=true`;
  otherwise non-rigid-body states are ignored.

## Extension Notes

New integrators should implement `step_integrator` for `RigidBodyState` and
`PlantState` if full-plant integration is required.
