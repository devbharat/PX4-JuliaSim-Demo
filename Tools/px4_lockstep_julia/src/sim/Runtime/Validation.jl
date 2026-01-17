"""Runtime.Validation

Development-time validation hooks for determinism and scheduling invariants.

Determinism bugs in hybrid simulation loops tend to be silent (the sim still
"works" but is no longer replayable). These checks are designed to fail loudly
when invariants are violated.

Design goals
------------
- Must be deterministic (no logging/printing side effects unless failing)
- Must be cheap enough to run by default in tests
- Must be disableable in production runs

NOTE
----
This module intentionally does *not* attempt to detect RNG use inside the plant
RHS; Julia does not provide a clean, low-overhead global hook for that.
The core rule remains:
  "no RNG in plant RHS" — sample all randomness in sources.
"""

Base.@kwdef struct EngineValidator
    enabled::Bool = true

    # Schema invariants
    check_bus_schema::Bool = true

    # Timebase invariants
    check_time_monotonic::Bool = true
    check_time_alignment::Bool = true

    # Scheduler invariants
    check_axis_ordering::Bool = true
end

"""Validate invariants at a boundary time.

`sim` is expected to have fields:
- `bus`, `timeline`, `sched`, `stats`, `t_us`

This is intentionally duck-typed to keep this file independent of Engine's type
parameters.
"""
function validate_boundary!(v::EngineValidator, sim, ev::BoundaryEvent)
    v.enabled || return nothing

    # 1) Time alignment invariants
    if v.check_time_alignment
        sim.t_us == ev.time_us ||
            error("boundary time mismatch: sim.t_us=$(sim.t_us) ev.time_us=$(ev.time_us)")
        sim.timeline.t0_us <= ev.time_us <= sim.timeline.t_end_us ||
            error("boundary time out of timeline bounds")
    end

    # 2) Time monotonicity
    if v.check_time_monotonic
        if getproperty(sim.stats, :n_boundaries) > 0
            last = getproperty(sim.stats, :last_boundary_us)
            ev.time_us > last ||
                error("non-monotonic boundary: t=$(ev.time_us) <= last=$(last)")
        end
    end

    # 3) Bus schema
    if v.check_bus_schema
        bus = sim.bus
        bus.schema_version == BUS_SCHEMA_VERSION || error(
            "BUS_SCHEMA_VERSION mismatch: bus has $(bus.schema_version) expected $(BUS_SCHEMA_VERSION)",
        )
    end

    # 4) Axis ordering consistency: axis next times should never be < current time.
    if v.check_axis_ordering
        s = sim.sched
        t = ev.time_us

        # Helper: if idx is in-bounds, assert next axis time is >= t.
        function _chk(axis::TimeAxis, idx::Int, name::Symbol)
            if idx <= length(axis.t_us)
                axis.t_us[idx] >= t ||
                    error("axis $(name) skipped an event: next=$(axis.t_us[idx]) < t=$t")
            end
        end

        _chk(sim.timeline.ap, s.ap_idx, :autopilot)
        _chk(sim.timeline.wind, s.wind_idx, :wind)
        _chk(sim.timeline.log, s.log_idx, :log)
        _chk(sim.timeline.scn, s.scn_idx, :scenario)
        _chk(sim.timeline.phys, s.phys_idx, :physics)
    end

    return nothing
end

export EngineValidator, validate_boundary!
