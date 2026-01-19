# Powertrain Models

## Role

`src/sim/Powertrain.jl` supplies battery models and PX4-facing `BatteryStatus`
injection. The interface is intentionally small to keep the plant RHS deterministic.

## Key Decisions and Rationale

- **Ideal battery baseline:** constant voltage with coulomb counting provides a stable,
  deterministic reference for regression tests.
- **Thevenin model:** adds OCV sag and RC polarization without thermal dynamics, which
  captures the dominant effects PX4 uses for battery logic.
- **Explicit warning mapping:** SOC thresholds map to PX4 warning levels so status
  semantics remain consistent across battery models.

## Integration Contracts

- Battery model objects (`IdealBattery`, `TheveninBattery`) are **parameter-only**.
- `step!(model, state, I_bus_a, dt)` advances SOC/polarization in a provided
  `Powertrain.BatteryState` (stand-alone stepping helper).
- In the event-driven plant engine, SOC and polarization voltage are integrated as part
  of `PlantState` while the battery model supplies OCV/parameterization.
- `status(model, state)` provides the exact fields expected by the lockstep ABI.

## Caveats

- Regenerative or negative bus currents are clamped to zero in `step!`.
- `status` reflects the provided state. In the canonical engine, battery telemetry is
  derived via `plant_outputs(...)` and published on the bus.
- The Thevenin model omits thermal effects and aging; it should not be used for
  high-fidelity energy studies.
- Invalid battery parameters can break the bus solve in the plant engine; expect
  explicit errors in that case.
