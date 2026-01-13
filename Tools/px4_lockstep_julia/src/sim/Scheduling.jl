"""PX4Lockstep.Sim.Scheduling

Small deterministic scheduling helpers (multi-rate stepping).

The sim is fundamentally a fixed-step integrator for the continuous-time plant, but
controllers, estimators, logging, etc. are *discrete-time* tasks that often run at lower
rates.

This module provides a minimal PeriodicTrigger you can use to implement sample-and-hold
logic without introducing threads or wall-clock time.
"""
module Scheduling

export PeriodicTrigger, due!, reset!

"""A periodic trigger for deterministic multi-rate simulation.

`due!(tr, t)` returns true when `t` has reached the next trigger time, and advances the
internal `t_next` so it can be called repeatedly.

Notes:
- `t` is simulation time (seconds), not wall clock.
- If the simulation ever runs slower than the trigger (e.g. you jump time), `due!` will
  catch up by advancing multiple periods.
"""
mutable struct PeriodicTrigger
    period::Float64
    t_next::Float64

    function PeriodicTrigger(period::Float64, t0::Float64=0.0)
        period > 0 || throw(ArgumentError("period must be > 0"))
        return new(period, t0)
    end
end

@inline function reset!(tr::PeriodicTrigger, t0::Float64=0.0)
    tr.t_next = t0
    return tr
end

@inline function due!(tr::PeriodicTrigger, t::Float64; eps::Float64=1e-12)
    if t + eps >= tr.t_next
        # Advance at least once, then catch up if we lagged.
        while t + eps >= tr.t_next
            tr.t_next += tr.period
        end
        return true
    end
    return false
end

end # module Scheduling
