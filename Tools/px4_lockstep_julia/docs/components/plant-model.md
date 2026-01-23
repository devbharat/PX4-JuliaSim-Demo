# Plant State Model

## Role

`src/sim/Plant.jl` defines the full continuous plant state used by adaptive integrators
and the event-driven engine. It captures the states that must evolve continuously for
deterministic variable-step integration.

## Key Decisions and Rationale

- **Fixed-size actuator channels:** motor/servo arrays match the lockstep ABI and keep
  allocations out of hot integration loops.
- **Rotor + battery count as type parameters:** `PlantState{N,B}` specializes on rotor
  and battery counts to keep adaptive integrators allocation-free.
- **Explicit input/output split:** `PlantInput` holds commands, wind, and faults;
  `PlantOutputs` carries algebraic couplings (rotor outputs, per-bus voltage/current,
  and battery telemetry) so the RHS stays pure.
- **Single source of truth:** subsystem model objects are treated as **parameter-only**;
  `PlantState` is canonical during integration.

## Integration Contracts

- `PlantInput` must be updated only at event boundaries.
- The plant RHS must be pure: no RNG and no mutation of shared state.
- No component objects are synchronized from `PlantState` in the canonical engine path.

## Caveats

- Actuator and servo channels are fixed-size to match the lockstep ABI; additional
  channels require interface changes.
- `PlantInput` currently includes actuator commands, wind, and faults; additional
  couplings must be added explicitly.
