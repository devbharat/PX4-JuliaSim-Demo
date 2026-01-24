# Plant Interface Protocols

## Role

`src/sim/PlantInterface.jl` defines protocol functions used by the runtime engine and
offline tools to interact with plant models **without** coupling to a particular engine
implementation.

The canonical engine (`Sim.Runtime.Engine`) uses these hooks to:

- compute deterministic algebraic outputs at boundary times (battery telemetry, rotor
  outputs, environment samples)
- enforce deterministic hard bounds after integration steps
- apply explicit discontinuities exactly at autopilot tick boundaries

## Protocol functions

### `plant_outputs(f, t_s, x, u)`

Evaluates algebraic outputs of the plant at a boundary time.

- **File:** `src/sim/PlantInterface.jl`
- **Expected return:** `Plant.PlantOutputs`

Typical fields provided by `PlantOutputs` (see `src/sim/Plant.jl`):

- `rotors`: `Propulsion.RotorOutput{N}` (thrust, shaft torque, rotor ω, currents)
- `bus_voltage_v`, `bus_current_a`: per-bus electrical telemetry
- `rho_kgm3`, `temp_k`, `air_vel_body`: atmosphere + relative flow samples
- `battery_statuses`: per-battery `Powertrain.BatteryStatus` vector (deterministic order)

The engine primarily uses `battery_statuses` to populate `bus.batteries` **before**
stepping PX4, and uses the remaining fields for deterministic logging and analysis.

### `plant_project(f, x)`

Optional projection hook called after each accepted integration interval.

Use this to enforce hard physical bounds deterministically (e.g. `ω ≥ 0`,
`SOC ∈ [0,1]`).

Implementation requirements:

- deterministic and side-effect free (no RNG, no IO)
- returns a new state or mutates only owned state

### `plant_on_autopilot_tick(f, x, cmd)`

Optional boundary-time discontinuity hook called **only** at autopilot tick boundaries
(after the autopilot has published `cmd`, and before the next integration interval).

This is intended for hybrid discontinuities that must occur exactly at the boundary,
for example:

- direct actuators snapping `PlantState.motors_y/servos_y` to the newly published
  `ActuatorCommand`

## Integration contracts

- These protocols are generic functions with **no fallback method**. The runtime engine
  checks capability using `hasmethod` and only calls hooks that are implemented.
- Protocol implementations must be fast and allocation-free in the hot path.

## Caveats

- Not all RHS functors implement `plant_outputs`; callers must guard accordingly.
- `plant_outputs` is evaluated only at discrete boundaries. If you need continuous-time
  monitoring, log plant states and post-process offline.
