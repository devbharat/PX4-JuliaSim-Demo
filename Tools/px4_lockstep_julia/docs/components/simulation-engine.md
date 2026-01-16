# Fixed-Step Simulation Engine

## Role

`src/sim/Simulation.jl` implements the fixed-step lockstep engine used for most PX4
regression runs. It is the canonical “PX4-in-the-loop at constant dt” path.

## Key Decisions and Rationale

- **Integer-step scheduling:** multi-rate tasks are scheduled by integer step counters,
  not floating-point time, eliminating cadence jitter from FP drift.
- **Microsecond dt enforcement:** `dt` must be representable as integer microseconds,
  which guarantees exact lockstep time injection at the cost of forbidding arbitrary
  `dt` values.
- **Wind sample-and-hold:** wind is sampled once per tick (via `SampledWind`) and held
  constant for RK4 stages to preserve determinism.
- **Discrete subsystem updates:** actuator dynamics, propulsion, and battery are stepped
  once per physics tick while only the rigid body is integrated continuously.
- **Pre-step logging:** log snapshots represent the state at the start of the interval,
  keeping logs aligned with PX4 time and controller outputs.

## Integration Contracts

- Controller outputs are treated as piecewise constant across `dt`.
- The autopilot is stepped on a deterministic cadence derived from `dt_autopilot`.
- Logging cadence must be an integer multiple of `dt`.
- `strict_lockstep_rates` enforces that `dt_autopilot` does not undersample PX4 tasks.
- Scenario faults are applied to motor duties and battery connection status.

## Extension Notes

The fixed-step engine is deliberately conservative; more advanced integration strategies
belong in the event-driven plant engine.

## Caveats

- Actuator/propulsion/battery dynamics are discretized at `dt`; use the plant engine if
  their continuous dynamics must be captured.
- `dt`, `dt_autopilot`, and `dt_log` must be microsecond-quantized and integer-related;
  violations raise errors.
- `strict_lockstep_rates` can be disabled, but doing so risks running PX4 tasks slower
  than intended.
- `strict_lockstep_rates` is only enforced when the autopilot implements
  `max_internal_rate_hz` (e.g., PX4 lockstep).
