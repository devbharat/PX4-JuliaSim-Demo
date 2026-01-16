# Record/Replay Bus Engine

## Role

`src/sim/RecordReplay.jl` and `src/sim/RecordReplay/*` implement the event-sourced bus
architecture for deterministic record/replay. The `BusEngine` integrates the plant
between timeline boundaries while discrete sources publish bus fields.

## Key Decisions and Rationale

- **Typed bus + schema version:** `SimBus` defines the coupling contract with explicit
  units and a `BUS_SCHEMA_VERSION` to force conscious changes.
- **Timeline as the authority:** `Timeline` aggregates periodic axes and scenario
  `AtTime` events into a single boundary axis used for integration.
- **Record tiers:** Tier‑0 recordings capture only what is needed for deterministic
  plant replay (commands, wind, plant, battery), keeping files small.
- **Deterministic ordering:** sources update in a fixed order and record
  `ap_cmd_evt`, `landed_evt`, and `faults_evt` on the event axis to avoid missing
  dynamic transitions.

## Integration Contracts

- Sources publish into the bus only at their scheduled boundary times.
- `BusEngine` must hold `bus.cmd`, `bus.wind_ned`, and `bus.faults` constant across the
  integration interval.
- `plant_outputs` is queried at boundaries to update battery telemetry deterministically.
- Recordings enforce `BUS_SCHEMA_VERSION` on load.

## Caveats

- Recordings are tied to current model parameters; replay does not embed model hashes.
- Julia serialization is used for Tier‑0 and is not a long‑term archival format.
- Dynamic `AtTime` events created during replay are not yet reflected in the timeline.
