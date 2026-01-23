# Record/Replay (Option A)

The record/replay stack is now *first-class* and lives in three places:

* `Sim.Runtime` — canonical hybrid engine (timeline traversal + plant integration)
* `Sim.Sources` — live and replay sources that publish bus signals
* `Sim.Recording` — trace + recorder types, persistence helpers

## Role

The runtime integrates the plant between timeline boundaries while discrete sources
publish bus fields.

## Key Decisions and Rationale

- **Typed bus + schema version:** `SimBus` defines the coupling contract with explicit
  units and a `BUS_SCHEMA_VERSION` to force conscious changes.
- **Timeline as the authority:** `Timeline` aggregates periodic axes and scenario
  `AtTime` events into a single boundary axis used for integration.
- **Record tiers:** Tier‑0 recordings capture only what is needed for deterministic
  plant replay (commands, wind, plant, battery telemetry), keeping files small.
- **Optional estimator stream:** enable `record_estimator=true` to capture `est` on
  the autopilot axis for estimator replay.
- **Deterministic ordering:** sources update in a fixed order and record
  `ap_cmd_evt`, `landed_evt`, `faults_evt`, and `wind_dist_evt` on the event axis to
  avoid missing dynamic transitions.

## Integration Contracts

- Sources publish into the bus only at their scheduled boundary times.
- `Engine` must hold `bus.cmd`, `bus.wind_ned`, and `bus.faults` constant across the
  integration interval.
- `plant_outputs` is queried at boundaries to update battery telemetry deterministically.
- `read_recording` validates schema version, axes, and required streams.

## Caveats

- Recordings are tied to current model parameters; replay does not embed model hashes.
- Julia serialization is used for Tier‑0 and is not a long‑term archival format.
- Dynamic `AtTime` events created during replay are not yet reflected in the timeline.
