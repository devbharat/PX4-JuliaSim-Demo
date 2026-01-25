# Record and Replay

## Role

Closed-loop “run twice and compare” quickly becomes misleading once trajectories diverge
and PX4 issues different commands.

The record/replay workflow avoids that failure mode:

1. Run PX4 once **live** (lockstep).
2. Record the minimal streams needed to reproduce the plant’s inputs deterministically.
3. Replay those streams **open-loop** while sweeping integrators, model changes, or
   analysis hooks.

The implementation spans:

- **Runtime** (`src/sim/Runtime/*`): the canonical hybrid engine and timeline.
- **Sources** (`src/sim/Sources/*`): live and replay sources that publish bus signals.
- **Recording** (`src/sim/Recording/*`): recorders, traces, and persistence helpers.

## What is recorded (Tier‑0)

A Tier‑0 recording (`Sim.Recording.Tier0Recording`) bundles:

- `BUS_SCHEMA_VERSION` (compatibility guard)
- the `Runtime.Timeline` used for the run
- the initial plant state `plant0`
- an in-memory recorder (`Recording.InMemoryRecorder`) holding the required streams

Required streams (sufficient for deterministic plant replay):

- `:cmd` — actuator commands (`ActuatorCommand`), **ZOH** on the autopilot axis
- `:wind_base_ned` — base wind sample, **sample/hold** on the wind axis
- `:plant` — plant state snapshots, **sampled** on the log axis
- `:battery` and `:batteries` — battery telemetry snapshots, **sampled** on the log axis
  - `:batteries` is the full vector (deterministic order)
  - `:battery` is a legacy primary snapshot (`batteries[1]`)

Optional streams:

- scenario outputs (faults, high-level autopilot commands, and wind disturbance), recorded either:
  - on the **event axis** (`*_evt`) when `record_faults_evt=true` (recommended), or
  - on the **scenario axis** otherwise
- estimator output (`:est`) on the autopilot axis when `record_estimator=true`

Tier‑0 is intentionally small. It is designed for integrator/model comparisons and
short regression runs, not for long-duration logging.

## How replay is wired

Replays are driven by typed traces and replay sources:

- `Recording.tier0_traces(rec)` builds the standard Tier‑0 traces (`cmd`, `wind_base_ned`, ...)
- `Sources.ReplayAutopilotSource(trace)` publishes `bus.cmd`
- `Sources.ReplayWindSource(trace)` publishes `bus.wind_ned`
- `Sources.ReplayScenarioSource(...)` publishes `bus.ap_cmd`, `bus.landed`, `bus.faults`

Example (schematic):

```julia
using PX4Lockstep.Sim

rec = Sim.Recording.read_recording("iris_recording.bin")
tr0 = Sim.Recording.tier0_traces(rec)

wind      = Sim.Sources.ReplayWindSource(tr0.wind_base_ned)
autopilot = Sim.Sources.ReplayAutopilotSource(tr0.cmd)

# Optional: replay scenario outputs (faults + high-level commands) if they were recorded.
scenario = try
    scn = Sim.Recording.scenario_traces(rec)
    wind_dist = haskey(scn, :wind_dist) ? scn.wind_dist : nothing
    Sim.Sources.ReplayScenarioSource(scn.ap_cmd, scn.landed, scn.faults; wind_dist=wind_dist)
catch
    Sim.Sources.NullScenarioSource()
end

eng = Sim.simulate(
    mode       = :replay,
    timeline   = rec.timeline,
    plant0     = rec.plant0,
    dynfun     = dynfun,
    integrator = integrator,
    autopilot  = autopilot,
    wind       = wind,
    scenario   = scenario,
)
```

## Guarantees and limitations

- **Deterministic scheduling:** all axes are integer microseconds; stream times are
  validated against the timeline when loading.
- **Sample-and-hold inputs:** `bus.cmd`, `bus.wind_ned`, and `bus.faults` are held
  constant over each integration interval.
- **Schema guard:** recordings are versioned by `BUS_SCHEMA_VERSION` and rejected if
  incompatible.

Caveats:

- Tier‑0 recordings do **not** embed model parameters or hashes; replay assumes the
  current model configuration matches the recording context.
- Persistence uses Julia `Serialization` (fast and dependency-light), but it is not a
  stable long-term archival format across Julia versions. If long-lived recordings
  become a requirement, add a versioned, stable format (e.g. HDF5/JLD2/Arrow) and keep
  `Serialization` as the developer‑fast path.
- Timeline boundaries are computed up-front. Dynamic insertion of new explicit
  boundaries at runtime is not supported.
