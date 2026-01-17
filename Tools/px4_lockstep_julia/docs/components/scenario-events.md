# Scenarios and Events

## Role

`src/sim/Scenario.jl` and `src/sim/Events.jl` provide a deterministic mechanism for
high-level mission inputs and discrete event injection.

## Key Decisions and Rationale

- **Scenario abstraction:** separates “what to command” from the simulation engine so
  the same plant can be exercised under many conditions.
- **Faults as signals:** scenarios publish `FaultState` instead of mutating propulsion
  or battery objects, enabling record/replay.
- **Microsecond `AtTime` events:** time triggers are stored as integer microseconds to
  align with lockstep time and avoid drift.
- **Multiple trigger types:** `AtTime`, `AtStep`, and `When` cover time, step count, and
  state-dependent events without extra dependencies.
- **One-shot semantics:** each event fires at most once, simplifying reasoning about
  side effects and ensuring idempotency.

## Integration Contracts

- `process_events!` may mutate the simulation instance at event boundaries only.
- `next_event_us` is used by the event-driven engine to schedule true boundaries.
- `When` events are evaluated only when boundaries are processed, so their timing
  resolution is limited to the event cadence.
- `scenario_faults` returns the held fault state for the current boundary time.

## Caveats

- `AtTime` requires microsecond-quantized times; non-quantized values raise errors.
- `AtStep` events are not currently supported by the canonical engine (`Runtime.Engine`). Prefer `AtTime` (possibly derived from `timeline.phys`), or extend the runtime to expose a deterministic step counter.
- `When` events can fire late by up to one event interval in the event-driven engine.
- `ScriptedScenario` uses float time thresholds; align them to `dt` for deterministic
  tick boundaries.
