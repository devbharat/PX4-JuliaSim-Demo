# Verification Utilities

## Role

`src/sim/Verification.jl` provides deterministic reference problems and comparison tools
used to guard integrator correctness without PX4 in the loop.

## Key Decisions and Rationale

- **Uniform trajectory grid:** avoids explicit time vectors so resampling is exact and
  drift-free.
- **RK45 reference path:** provides a repeatable numerical “truth” using the same
  in-house integrator with microsecond-quantized substeps by default (`quantize_us`).
- **PlantState comparisons:** extend beyond rigid-body metrics to rotor and battery
  states where adaptive solvers are exercised, while leaving actuator errors opt-in.

## Integration Contracts

- Reference and test trajectories must share exact uniform grids.
- `rk45_reference` is expected to be run with `reset!` for deterministic comparisons.
- Closed-loop comparisons should replay commands to avoid conflating controller
  divergence with integrator error.

## Caveats

- `rk45_reference` provides a high-accuracy numerical reference, not an analytic truth.
- The reference uses the same in-house integrator implementation, so it is not an
  independent solver check.
- Resampling requires exact integer ratios between grids; otherwise comparisons are
  invalid.
- Closed-loop comparisons can diverge quickly without command replay or state resets.
