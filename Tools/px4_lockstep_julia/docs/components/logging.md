# Logging

The `Sim.Logging` module provides a *structured logging* interface decoupled from record/replay:

- **Record/replay** is handled by `Sim.Recording` (Tier-0 traces stored by a recorder).
- **Logging** is handled by `Sim.Logging` (write “sim logs” to memory or a file sink).

## What gets logged

A log entry is intended to represent a deterministic, pre-step snapshot of the sim at a boundary time `t_k`:

- rigid body state (`RigidBodyState`) at `t_k`
- actuator command applied at `t_k`
- wind sample at `t_k`
- (optionally) propulsion outputs (rotor thrust/omega, up to 12 channels)
- (optionally) battery telemetry

### Contact and landing diagnostics

When available, the runtime also logs a small set of **contact/landing diagnostics** on the
log axis (see `Sim.Logging.csv_schema()` for exact column names):

- `landed_phy` — physics-derived landed flag
- `acc_x/y/z` — inertial acceleration sample in NED at the log boundary
- `spec_bx/by/bz` — accelerometer-like specific force in body/FRD at the log boundary
- `impact_dv_*` — max impact Δv since the last log sample (NED)
- `impact_acc_est_*` — estimated 1-tick impact acceleration computed as `impact_dv / dt_autopilot`
- `impact_time_us`, `impact_count` — timestamp and count for impacts since the last log

The impact acceleration values are **estimates** (impulses are discontinuous), but they are
useful for not missing touchdown spikes when logging at a lower rate than the contact
substep rate.

## Log sinks

A *log sink* receives log entries and decides how to store them.

Built-in sinks:

- `Sim.Logging.SimLog` — an in-memory log (useful for debugging / quick plots)
- `Sim.Logging.CSVLogSink` — writes the standard sim log schema to a CSV file

Both implement `Sim.Logging.AbstractLogSink`.

## Log schema

The CSV output is defined by a single versioned schema object:

- `Sim.Logging.csv_schema() -> LogSchema`

The schema contains ordered columns with basic metadata:

- `name` (CSV column name)
- `unit`
- `desc`

CSV files written by `CSVLogSink` and `write_csv` start with:

1. `# schema_version=<int>`
2. `# schema_name=<string>`
3. `<comma-separated header line>`

This makes logs self-describing without adding any heavy dependencies.

## Runtime integration

`Sim.Runtime.Engine` supports log sinks via the `log_sinks` keyword.

At each `timeline.log` boundary, the engine:

1. records Tier-0 signals to the recorder (if one is present)
2. emits a log entry to each configured log sink

This keeps logging usable in *live* runs even when a Tier-0 trace is not recorded.

### Example: attach a CSV log sink

```julia
using PX4Lockstep
const SIM = PX4Lockstep.Sim

sink = SIM.Logging.CSVLogSink("out/sim_log.csv")

sim = SIM.simulate(
    mode = :live,
    timeline = timeline,
    plant0 = plant0,
    dynfun = dynfun,
    integrator = SIM.Integrators.RK4Integrator(),
    autopilot = autopilot,
    wind = wind,
    scenario = scenario,
    estimator = estimator,
    telemetry = SIM.Runtime.NullTelemetry(),
    recorder = nothing,
    log_sinks = sink,   # <- can also pass a tuple/vector of sinks
)

SIM.Runtime.run!(sim)
```

## Adding new sinks (HDF5 later)

To add a new sink (e.g., HDF5), implement:

- `log!(sink, t_s, rb_state, actuator_cmd; kwargs...)`
- `close!(sink)`

and pass it via `log_sinks = ...`.
