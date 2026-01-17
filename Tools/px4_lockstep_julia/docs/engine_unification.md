# Engine Unification

This document captures the rationale and the resulting architecture of the engine
unification refactor in **PX4Lockstep.Sim**.

## Historical context

Historically, the repository had multiple run loops with overlapping responsibilities
(event traversal, discrete source stepping, plant integration):

- a legacy fixed-step tick loop
- an event-driven variable-step loop
- a record/replay loop

Even with discipline, multiple engines inevitably drift in:

- boundary ordering
- stop-time semantics
- time quantization
- recording/replay behavior

That drift is a long-term correctness and determinism risk.

**Current state:** there is now exactly **one canonical run loop**.

## Canonical engine

The authoritative engine is:

- `PX4Lockstep.Sim.Runtime.Engine`

It supports:

- **Live**: PX4-in-the-loop (lockstep cadence)
- **Record**: live + deterministic stream recording (Tier-0)
- **Replay**: open-loop plant-only replay (integrator comparisons)
- Optional **fixed-physics dt** via a physics axis `timeline.phys`

The engine is **bus-driven**: all discrete components communicate via a typed,
versioned `SimBus`. The plant model consumes a held `PlantInput` derived from the
bus over each integration interval.

## Module layering

The stable dependency direction is:

- `Sim.Runtime` — the one engine (timeline traversal, enforced boundary protocol, interval integration)
- `Sim.Sources` — discrete sources that publish into the bus (live + replay)
- `Sim.Recording` — recorders, traces, persistence, schema checks
- `Sim.PlantModels` — coupled plant RHS + algebraic outputs + projection hooks
- `Sim.Integrators` — numerical methods only (no domain logic)

## Boundary ordering contract

At each event boundary time `t_k` (microseconds):

1. Scenario updates the bus (`faults`, `ap_cmd`, `landed`)
2. Wind updates the bus (`wind_ned`)
3. Plant-derived telemetry updates the bus (`battery`, optional env sample, etc)
4. Estimator updates the bus (`est`)
5. Telemetry hooks run (optional, read-only)
6. Autopilot updates the bus (`cmd`)
7. Boundary-time plant discontinuities run (e.g. direct actuator snaps)
8. Sampling hooks run:
   - **Tier-0 recorder** samples record/replay streams (cmd/wind/scenario in *record* mode; plant/battery on the log axis)
   - **Log sinks** emit structured logs (CSV sink, etc) when configured
9. Plant integrates over `[t_k, t_{k+1})` with held `PlantInput`

This ordering is enforced by the runtime boundary protocol and must not be duplicated
anywhere else.

## Fixed-step semantics

If a traditional fixed physics dt is required (for example to match an older simulator),
set `timeline.phys` to a uniform axis. Those ticks are included in the global boundary
union, so the engine performs exactly one integration interval per physics tick.

PX4 (autopilot) ticks, wind sampling, and logging can still run at their own cadences.
