# Logging

## Role

`src/sim/Logging.jl` provides deterministic logging with both in-memory and streaming
CSV sinks.

## Key Decisions and Rationale

- **Columnar `SimLog`:** minimizes allocations and enables `reserve!` to size buffers for
  long runs.
- **Schema versioning:** `CSV_SCHEMA_VERSION` lets downstream tooling detect breaking
  column changes.
- **Pre-step snapshots:** logs represent the state at the start of each interval,
  matching PX4 time semantics and controller sample-and-hold.
- **Streaming option:** `CSVLogSink` writes incrementally to avoid large in-memory logs.

## Integration Contracts

- Logging must not mutate simulation state.
- Rotor outputs may be `NaN` when not available (non-plant or non-rotor cases).

## Caveats

- Logs are pre-step snapshots; interpreting them as post-step states will shift
  controller/plant timing.
- `CSVLogSink` writes synchronously on each `log!` call and can become IO-bound for long
  runs.
- `time_us` defaults to rounding `t`; provide exact microsecond time for strict
  lockstep analysis.
