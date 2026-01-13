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

export AbstractEvent, AtTime, When, EventScheduler, step_events!

abstract type AbstractEvent end

"""Time-triggered one-shot event."""
struct AtTime{F} <: AbstractEvent
    t_fire::Float64
    action::F  # action(sim, t)
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

"""Add an event to the scheduler."""
function Base.push!(s::EventScheduler, e::AbstractEvent)
    push!(s.events, e)
    push!(s.fired, false)
    return s
end

@inline function _should_fire(e::AtTime, sim, t::Float64)
    return t >= e.t_fire
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
