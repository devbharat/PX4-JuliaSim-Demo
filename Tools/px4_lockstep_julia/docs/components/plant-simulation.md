# Event-Driven Plant Simulation

## Role

`src/sim/PlantSimulation.jl` integrates the full continuous plant state between
discrete event boundaries. It enables adaptive solvers while preserving lockstep
determinism.

## Key Decisions and Rationale

- **Event boundaries define inputs:** scenario events, wind updates, and autopilot ticks
  are the only points where inputs may change; logging is scheduled as a boundary for
  deterministic snapshots.
- **Microsecond event clock:** all triggers use integer microseconds to align with
  lockstep time and avoid floating-point drift.
- **Single coupled RHS:** actuators, propulsion, rotor dynamics, battery SOC/polarization
  voltage, and rigid body dynamics are evaluated in one RHS so adaptive error control
  sees the full plant.
- **Deterministic bus solve:** the battery/bus equation uses analytic region checks and
  fixed-iteration bisection for deterministic convergence.
- **Post-step projection:** rotor speeds, SOC, and actuator bounds are clamped after
  integration to prevent numerical overshoot from destabilizing later steps.

## Integration Contracts

- `PlantInput` is held constant between event boundaries.
- `PlantOutputs` supplies battery status for PX4 injection and rotor outputs for logging.
- `AtTime` events are treated as true boundaries; `When` events are evaluated at
  boundaries.
- Direct actuators are snapped at boundaries; dynamic actuators remain continuous.
- Faults in `PlantInput` (motor disable, battery disconnect) are applied inside the RHS.

## Extension Notes

The engine currently targets multirotor plants (`QuadRotorSet`). Supporting other
propulsion types requires extending the RHS and algebraic coupling helpers.

## Caveats

- `When` events are evaluated only at event boundaries, so trigger timing is limited by
  the wind/autopilot/log cadence.
- Contact handling is approximate; crossings are detected by sign changes in `z` and
  split only once per interval.
- Post-step projection clamps rotor speeds and SOC, which can hide instability if
  tolerances are too loose.
- `strict_lockstep_rates` relies on `max_internal_rate_hz`; custom autopilots that do
  not implement it skip the guard.
