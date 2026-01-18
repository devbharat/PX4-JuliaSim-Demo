"""PX4Lockstep.Sim.Events

Deterministic event scheduling for scenarios.

The simulator is a hybrid system:
* continuous rigid-body dynamics integrated at fixed dt
* discrete events (arming, mode changes, failures, wind steps, etc.)

The original `ScriptedScenario` supported a few hard-coded times. This module
provides a composable event scheduler with:

* time-triggered events (`AtTime`)
* condition-triggered events (`When`)
* one-shot semantics (each event fires at most once)

Events operate by mutating the simulation instance (`sim`). This keeps the mechanism
general: events can modify scenario commands, environment, vehicle failures, etc.

This is intentionally lightweight and avoids external dependencies.
"""
module Events

export AbstractEvent, AtTime, When, EventScheduler, step_events!, next_at_time_us

abstract type AbstractEvent end

"""Time-triggered one-shot event.

To avoid float comparison drift, fire times are stored as **integer microseconds**.

Semantics: the event fires once when `time_us(sim) >= t_fire`.
"""
struct AtTime{F} <: AbstractEvent
    t_fire_us::UInt64
    action::F  # action(sim, t)
end

function AtTime(t_fire_s::Real, action::F) where {F}
    t_s = Float64(t_fire_s)
    t_s >= 0.0 || error("AtTime requires t_fire_s >= 0")
    t_us_f = t_s * 1e6
    t_us = Int(round(t_us_f))
    # Enforce microsecond quantization so scheduling is stable and explicit.
    if abs(t_us_f - t_us) > 1e-6
        error("AtTime requires microsecond-quantized time; got $t_fire_s s")
    end
    return AtTime{F}(UInt64(t_us), action)
end

"""Condition-triggered one-shot event."""
struct When{C,F} <: AbstractEvent
    condition::C  # condition(sim, t)::Bool
    action::F     # action(sim, t)
end

"""A simple one-shot event scheduler."""
mutable struct EventScheduler
    events::Vector{AbstractEvent}
    fired::BitVector
end

function EventScheduler(events::Vector{AbstractEvent} = AbstractEvent[])
    return EventScheduler(events, falses(length(events)))
end


"""Return the next scheduled `AtTime` event time (microseconds) that is strictly in the future.

This is intended for hybrid/event-driven simulators to include scenario events as true
event boundaries.

* Returns `nothing` if no unfired future `AtTime` events exist.
* Only considers `AtTime` events (not `When`).

Notes
-----
* The scheduler requires `t_fire_us > now_us` (strictly in the future). Events that
  are due *at* the current time should be handled by calling `step_events!` first.
"""
function next_at_time_us(sched::EventScheduler, now_us::UInt64)::Union{UInt64,Nothing}
    tmin = typemax(UInt64)
    found = false
    @inbounds for i = 1:length(sched.events)
        if !sched.fired[i]
            e = sched.events[i]
            if e isa AtTime
                t = e.t_fire_us
                if t > now_us && t < tmin
                    tmin = t
                    found = true
                end
            end
        end
    end
    return found ? tmin : nothing
end

"""Convenience overload: infer `now_us` from the simulation instance."""
@inline function next_at_time_us(sched::EventScheduler, sim)::Union{UInt64,Nothing}
    return next_at_time_us(sched, _sim_now_us(sim))
end


@inline function _sim_now_us(sim)::UInt64
    T = typeof(sim)
    # Engine-style timebase.
    if Base.hasfield(T, :t_us)
        return getfield(sim, :t_us)
    end

    # SimulationInstance-style timebase (fixed dt * step).
    if Base.hasfield(T, :t0_us) && Base.hasfield(T, :dt_us) && Base.hasfield(T, :step)
        t0_us = getfield(sim, :t0_us)
        dt_us = getfield(sim, :dt_us)
        step = getfield(sim, :step)
        return t0_us + UInt64(step) * UInt64(dt_us)
    end

    error("sim type $(T) does not provide a compatible timebase for AtTime events")
end

"""Add an event to the scheduler."""
function Base.push!(s::EventScheduler, e::AbstractEvent)
    push!(s.events, e)
    push!(s.fired, false)
    return s
end

@inline function _should_fire(e::AtTime, sim, t::Float64)
    now_us = _sim_now_us(sim)
    return now_us >= e.t_fire_us
end

@inline function _should_fire(e::When, sim, t::Float64)
    return e.condition(sim, t)
end

@inline function _apply!(e::AtTime, sim, t::Float64)
    e.action(sim, t)
    return nothing
end

@inline function _apply!(e::When, sim, t::Float64)
    e.action(sim, t)
    return nothing
end

"""Evaluate and apply any events that should fire this tick."""
function step_events!(sched::EventScheduler, sim, t::Float64)
    @inbounds for i = 1:length(sched.events)
        if !sched.fired[i]
            e = sched.events[i]
            if _should_fire(e, sim, t)
                _apply!(e, sim, t)
                sched.fired[i] = true
            end
        end
    end
    return nothing
end

end # module Events
