# Powertrain Models

## Role

`src/sim/Powertrain.jl` supplies battery models and PX4-facing `BatteryStatus`
telemetry. Multi-battery and multi-bus wiring is handled by
`src/sim/PlantModels/PowerNetwork.jl` and is integrated in the coupled plant
model (`PlantModels.CoupledMultirotorModel`).

The interface is intentionally small to keep the plant RHS deterministic.

## Key Decisions and Rationale

- **Ideal battery baseline:** constant voltage with coulomb counting provides a stable,
  deterministic reference for regression tests.
- **Thevenin model:** adds OCV sag and RC polarization with an optional lumped thermal
  hook (temperature state integrated in the plant). `R0` can be a constant, a
  temperature-only curve, or a SOC+temperature surface (CSV) while keeping the
  plant RHS deterministic.
- **Explicit warning mapping:** SOC thresholds map to PX4 warning levels so status
  semantics remain consistent across battery models.

## Integration Contracts

- Battery model objects (`IdealBattery`, `TheveninBattery`) are **parameter-only**.
- Canonical construction is TOML/spec-driven; for programmatic use, build batteries
  via `Aircraft.BatterySpec` + `Aircraft.build_battery` (direct constructors are deprecated).
- `step!(model, state, I_bus_a, dt)` advances SOC/polarization in a provided
  `Powertrain.BatteryState` (stand-alone stepping helper).
- In the event-driven plant engine, SOC and polarization voltage are integrated as part
  of `PlantState` while the battery model supplies OCV/parameterization.
- `PlantModels.PowerNetwork{N,B,K}` wires the power system:
  - `bus_for_motor[i]` maps physical motor `i` to a bus `1..K`.
  - `bus_for_battery[j]` maps battery `j` to a bus `1..K`.
  - `avionics_load_w[k]` is a constant power load on bus `k`.
  - `share_mode` controls how multiple batteries on a bus share current.
- The coupled plant solves one bus voltage per bus, then distributes bus load current
  across batteries on that bus (rule: `:inv_r0` or `:equal`).
- `PlantOutputs{N,B,K}` carries per-bus `bus_voltage_v` / `bus_current_a` and
  per-battery `battery_statuses` (deterministic order, length `B`).
- `status(model, state)` provides the exact fields expected by the lockstep ABI.

## Caveats

- Regenerative or negative bus currents are clamped to zero in `step!`.
- `status` reflects the provided state. In the canonical engine, battery telemetry is
  derived via `plant_outputs(...)` and published on the bus.
- Stand-alone `status(model, state)` uses the model's `temp_c`; thermal state only
  exists in the integrated plant.
- The Thevenin model includes a simple thermal hook but omits aging, OCV(T), and
  capacity(T); it should not be used for high-fidelity energy studies.
- Dense OCV curves (e.g. cell characterization tables) are supported: the OCV helper
  uses a binary search and aircraft specs can load curves from CSV at build time.
- Invalid battery parameters can break the bus solve in the plant engine; expect
  explicit errors in that case.
- PowerNetwork models a single-bus assignment per battery; cross-feed or diode OR-ing
  is intentionally out of scope.
