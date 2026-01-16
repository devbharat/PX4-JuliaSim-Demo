# Scheduling

## Role

`src/sim/Scheduling.jl` implements integer-step periodic triggers for the fixed-step
engine.

## Key Decisions and Rationale

- **Step-based triggers:** cadence is computed with integer modulo arithmetic (with
  optional offsets) to avoid floating-point jitter.

## Integration Contracts

- Task periods must be integer multiples of the physics `dt`.

## Caveats

- `StepTrigger` assumes a fixed `dt` and is not suitable for variable-step simulation.
- Periods must be converted to integer step counts before constructing triggers; the
  helper assumes validation happened upstream.
- Offsets are expressed in steps and can shift phase; choose them deliberately.
