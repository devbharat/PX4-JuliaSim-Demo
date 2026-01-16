# Plant State Model

## Role

`src/sim/Plant.jl` defines the full continuous plant state used by adaptive integrators
and the event-driven engine. It captures the states that must evolve continuously for
deterministic variable-step integration.

## Key Decisions and Rationale

- **Fixed-size actuator channels:** motor/servo arrays match the lockstep ABI and keep
  allocations out of hot integration loops.
- **Rotor count as a type parameter:** `PlantState{N}` specializes on rotor count to
  keep adaptive integrators allocation-free.
- **Explicit input/output split:** `PlantInput` holds commands, wind, and faults;
  `PlantOutputs` carries algebraic couplings (rotor outputs, bus voltage/current,
  battery status) so the RHS stays pure.
- **Compatibility shims:** `init_plant_state` and `sync_components_from_plant!` bridge
  legacy mutable components during the transition to full-plant integration.

## Integration Contracts

- `PlantInput` must be updated only at event boundaries.
- `PlantState` is the single source of truth during integration.
- Sync shims are allowed after integration, not during RHS evaluation.

## Caveats

- Actuator and servo channels are fixed-size to match the lockstep ABI; additional
  channels require interface changes.
- `PlantInput` currently includes actuator commands, wind, and faults; additional
  couplings must be added explicitly.
- Legacy component shims must be synchronized after integration; mutating them inside
  the RHS breaks determinism.
