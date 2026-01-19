# Scheduling

The canonical simulator (`Sim.Runtime.Engine`) uses **explicit integer-microsecond time axes** and advances the simulation on the **union** of those axes.

This replaces older “fixed-step + modular step counters” patterns and avoids float-time drift/jitter entirely.

## Where scheduling lives

- `src/sim/Runtime/Timeline.jl`
  - Builds periodic axes in **integer microseconds**.
  - Merges axes into a single strictly-increasing event axis (`timeline.evt`).
  - Enforces exact microsecond representability via `dt_to_us`.

- `src/sim/Runtime/Scheduler.jl`
  - Maintains per-axis indices.
  - Reports whether each axis is **due** at the current event boundary.

- `src/sim/Runtime/Engine.jl`
  - Defines the canonical stage order.
  - Uses the scheduler `due_*` flags to decide which sources/loggers fire at each boundary.

## Axes

A `Runtime.Timeline` contains:

- `timeline.ap`   : autopilot tick axis
- `timeline.wind` : wind update axis
- `timeline.log`  : logging/sampling axis
- `timeline.scn`  : scenario axis (typically sparse, plus `t0`)
- `timeline.phys` : optional extra integration boundary axis
- `timeline.evt`  : union axis used for integration boundaries

All times are stored as `UInt64` microseconds. Because axes are generated from integer periods, there is no cadence drift.

## Scheduler contract

At each event boundary, `Runtime.boundary_event(sched)` returns a struct with:

- `time_us::UInt64`
- `due_ap`, `due_wind`, `due_log`, `due_scn`, `due_phys` (booleans)

The engine processes sources/loggers in a fixed order and calls `Runtime.consume_boundary!(sched, ev)` once the boundary has been fully processed.
