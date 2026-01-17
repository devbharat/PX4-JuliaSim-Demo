"""Runtime.BoundaryProtocol

This file codifies the **canonical hybrid boundary ordering** for the simulator.

Why this exists
---------------
Deterministic hybrid simulators are extremely sensitive to subtle ordering drift.
If boundary processing order is only a convention, two “engines” (or two code paths)
will inevitably diverge over time.

To prevent drift, the canonical engine (`Sim.Runtime.Engine`) processes all boundary
work through a single typed `BoundaryEvent` and an explicit stage order.

Boundary event semantics
-----------------------
A boundary event occurs at an integer-microsecond time `t_us` on the union axis
`timeline.evt`. At a given `t_us`, one or more sub-axes may also be due:

- autopilot tick (`timeline.ap`)
- wind tick (`timeline.wind`)
- log tick (`timeline.log`)
- scenario AtTime boundary (`timeline.scn`)
- fixed physics tick (`timeline.phys`)  (adds boundaries only)

The engine must process discrete sources and derived outputs in a canonical order
before integrating the plant over `[t_k, t_{k+1})`.

Canonical stage order
---------------------
Stages are processed in the following order:

1. `:scenario`
   Scenario publishes faults and high-level commands.
2. `:wind`
   Wind source samples wind/turbulence (no RNG inside plant RHS).
3. `:derived_outputs`
   Evaluate algebraic plant outputs (battery telemetry) for autopilot consumption.
4. `:estimator`
   Estimator publishes bus.est (truth injection, noise/bias/delay).
5. `:telemetry`
   Optional deterministic hooks for metrics/inspection (must not mutate bus/plant).
6. `:autopilot`
   Autopilot consumes bus.est/battery/faults and publishes bus.cmd.
7. `:plant_discontinuities`
   Apply boundary-time plant updates at autopilot ticks (e.g. DirectActuators snap).
8. `:logging`
   Record plant/bus streams at log ticks.

Notes
-----
- Scenario is currently stepped at *every* boundary, not only `timeline.scn`, so
  hybrid `When(...)` logic can update faults as soon as possible. `due_scn` is still
  computed for debugging and potential future “scenario-only-on-scn-axis” modes.
- `due_phys` is included to support fixed-step semantics via added boundaries.
"""

"""Typed information about what is due at a given boundary time."""
struct BoundaryEvent
    time_us::UInt64
    due_scn::Bool
    due_wind::Bool
    due_ap::Bool
    due_log::Bool
    due_phys::Bool
end

"""Canonical stage ordering used by the engine."""
const CANONICAL_STAGE_ORDER = (
    :scenario,
    :wind,
    :derived_outputs,
    :estimator,
    :telemetry,
    :autopilot,
    :plant_discontinuities,
    :logging,
)

"""Return a human-readable list of due axes at a boundary."""
function due_axes(ev::BoundaryEvent)
    a = Symbol[]
    ev.due_scn && push!(a, :scenario_axis)
    ev.due_wind && push!(a, :wind)
    ev.due_ap && push!(a, :autopilot)
    ev.due_log && push!(a, :log)
    ev.due_phys && push!(a, :physics)
    return a
end

export BoundaryEvent, CANONICAL_STAGE_ORDER, due_axes
