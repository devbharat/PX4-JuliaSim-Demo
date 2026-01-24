# Environment Models

## Role

See also: **[Frames and sign conventions](../reference/conventions.md)**.

`src/sim/Environment.jl` provides atmosphere, wind, and gravity models with deterministic
step and sampling APIs.

## Key Decisions and Rationale

- **Sample-and-hold wind wrapper:** `SampledWind` freezes wind across RK stages so
  fixed-step integration sees a constant disturbance.
- **OU turbulence model:** `OUWind` uses an exact discrete update with an explicit RNG
  input and optional step gusts for deterministic scenarios.
- **Simple ISA1976 atmosphere + gravity choices:** ISA1976 density is adequate for
  low-altitude missions; `UniformGravity` is the default with `SphericalGravity`
  available for long-range/high-altitude cases.

## Integration Contracts

- Wind models must be stepped explicitly via `step_wind!` at event boundaries.
- Sampling occurs via `sample_wind!` and should not mutate during ODE substeps.
- Scenario gusts are injected via `add_step_gust!` on supported wind types.

## Caveats

- `SampledWind` must be refreshed with `sample_wind!` each tick or it will return stale
  disturbances.
- `OUWind` statistics depend on the `dt` passed to `step_wind!`; varying the update
  cadence changes the process.
- `SphericalGravity` assumes local down is radial and is only an approximation for
  local NED frames.
