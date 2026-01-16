# Record/Replay Examples (Option A)

These examples are **scaffolding** for the new first-class record/replay architecture
(`PX4Lockstep.Sim.RecordReplay`).

Status: **partially functional**.

* `minimal_record_replay_demo.jl` demonstrates the core bus + timeline + traces + engine loop (no PX4).
* Iris PX4-live **record** and plant-only **replay** are working.

The goal is to make the intended workflows explicit and then fill in the implementation
in small, verifiable steps.

## Quick start: Iris integrator comparison (recommended)

This is the clean UX wrapper that records a baseline PX4 run and then sweeps
plant integrators via replay:

```bash
PX4_LOCKSTEP_LIB=/path/to/libpx4_lockstep.(so|dylib) \
PX4_LOCKSTEP_MISSION=Tools/px4_lockstep_julia/examples/simple_mission.waypoints \
  julia --project=Tools/px4_lockstep_julia -O3 Tools/px4_lockstep_julia/examples/replay/iris_integrator_compare.jl
```

It will write a Tier-0 recording and a summary CSV under `examples/replay/out/`.

## Intended workflows

### 1) Iris record run (PX4 live)
- Run PX4 lockstep live
- Record Tier-0 streams:
  - `cmd(t)` on autopilot ticks
  - `wind(t)` on wind ticks
  - `plant(t)` + `battery(t)` on log ticks
  - scenario outputs (including `faults`) on scenario/event ticks

### 2) Iris plant-only replay (integrator sweep)
- Replace PX4 with `ReplayAutopilotSource` using recorded `cmd(t)`
- Replace wind model with `ReplayWindSource` using recorded `wind(t)`
- Replay scenario outputs so **plant-affecting faults** are applied identically
- Sweep integrators and compare to a reference replay (RK45 tight, plant-aware)

## Files

- `minimal_record_replay_demo.jl`: tiny deterministic demo (no PX4)
- `iris_common.jl`: shared Iris defaults (to keep scripts consistent)
- `iris_record_run.jl`: PX4-live record → Tier-0 `.jls`
- `iris_replay_integrator_sweep.jl`: plant-only replay sweep (needs `IRIS_RECORD_IN`)
- `iris_integrator_compare.jl`: **one-shot** record + replay + summary (recommended)

See `docs/record_replay.md` for the design.
