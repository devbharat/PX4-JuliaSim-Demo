# Scenario events

The scenario layer is a convenient place to express **hybrid-system** behavior:

- arm / disarm
- start a mission / pulse RTL
- inject a step gust
- fail a motor or disconnect the battery
- trigger actions when some condition becomes true

The `Sim.Events` module provides a small, deterministic one-shot event scheduler that can be embedded inside a scenario.

## What the scheduler provides

- **Multiple trigger types:**
  - `AtTime(t_s, action)` fires once when `ctx.t_us >= t_s`.
  - `AtStep(step, action)` fires once when `ctx.step >= step`.
  - `When(cond, action)` fires once when `cond(state, ctx, t_s)` becomes true.

- **Deterministic timebase:** `AtTime` uses integer microseconds internally to avoid float drift.

- **One-shot semantics:** every event fires at most once.

## Canonical engine support

The canonical simulator (`Sim.Runtime.Engine`) exposes a deterministic boundary counter:

- `Engine.step` is a **0-based** count of processed event-axis boundaries.
- Live scenarios executed through `Sim.Sources.LiveScenarioSource` receive a `ScenarioContext` snapshot that exposes `t_us` and `step` for use by `AtTime` / `AtStep` / `When`.

If you want a specific physical time (e.g. “at exactly 3.275 s”), prefer `AtTime`. `AtStep` is most useful when you want to trigger after a deterministic number of engine boundaries (e.g. for scripted tests).

## Caveats

- Only `AtTime` events are currently discoverable up-front for inclusion as explicit timeline boundaries (`Runtime.build_timeline_for_run`).
  - This is intentional: `AtTime` represents “hard” hybrid event times.
  - `AtStep` events are evaluated when the scenario is stepped at boundaries.

- Dynamic insertion of new `AtTime` boundaries at runtime is not yet supported.
