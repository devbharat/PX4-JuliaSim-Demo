# Scenario Events

The scenario layer is the intended place to express **hybrid-system** behavior:

- arm/disarm
- start a mission / request RTL
- inject step gusts or wind disturbances
- fail a motor or disconnect the battery
- trigger actions when a condition becomes true

The `Sim.Events` module provides a small, deterministic one-shot scheduler that can be embedded inside a scenario.

## What the scheduler provides

- **Multiple trigger types:**
  - `AtTime(t_s, action)` fires once when simulation time reaches or exceeds `t_s`.
    Internally, `t_s` is quantized to integer microseconds.
  - `AtStep(step, action)` fires once when the scenario has been *stepped* `step` times.
    “Stepped” means invocations of `scenario_outputs(...)` by the active scenario source.
  - `When(cond, action)` fires once when `cond(state, ctx, t_s)` becomes true.

- **One-shot semantics:** every event fires at most once.

- **Deterministic timebase:** time triggers are tracked in `UInt64` microseconds to avoid float drift.

## How scenarios are stepped in the runtime

In the canonical runtime (`Sim.Runtime.Engine`), scenarios are driven by a scenario source
(`Sim.Sources.AbstractScenarioSource`). The source is stepped at **every event boundary**
(`timeline.evt`) so hybrid conditions are evaluated deterministically.

- For `Sim.Sources.LiveScenarioSource`, `event_times_us(...)` discovers `AtTime` events
  in the scenario scheduler and adds those times (plus `t0_us`) to `timeline.scn`.
  These times are included in the global boundary union so `AtTime` triggers are exact.
- Between boundaries, scenario outputs (`bus.ap_cmd`, `bus.faults`, `bus.landed`,
  `bus.wind_dist_ned`) are treated as **ZOH** (sample-and-hold).

Practical guidance:

- Prefer `AtTime` for most hybrid events that must occur at specific times.
- `AtStep` is useful for test scripts where you want an event after a deterministic number
  of *scenario* updates. In the canonical engine, that means **event boundaries**.
- `When` conditions are evaluated at every event boundary. If you need higher-rate
  evaluation, add more boundaries (e.g. a smaller `timeline.phys` or extra times via
  `event_times_us`).

## Caveats

- Scenario stepping at **every event boundary** favors correctness and determinism, but it
  can be more expensive than a dedicated scenario cadence if your scenario logic is heavy.
- `AtStep(k)` counts **event boundaries**, not `timeline.scn` ticks. This is intentional
  in the canonical engine but could be revisited if a distinct scenario cadence is added.
- Only `AtTime` events are currently discoverable up-front for inclusion as explicit timeline
  boundaries (`Runtime.build_timeline_for_run`).
- Dynamic insertion of new explicit scenario boundaries at runtime is not supported.
