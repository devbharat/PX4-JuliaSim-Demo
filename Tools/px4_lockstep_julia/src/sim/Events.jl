"""PX4Lockstep.Sim.Events

Deterministic one-shot event scheduling.

The simulator is a hybrid system:
* continuous dynamics integrated between event boundaries
* discrete events (arming, mode changes, failures, wind steps, etc.) applied at boundaries

Architecture contract
---------------------
Events are treated as **pure boundary updates**:

* Conditions and actions are evaluated only at boundaries.
* Actions are expected to be **side-effect free** w.r.t. simulator internals.
* Instead of mutating a `sim` object, actions **transform caller-owned state**.

This makes the event mechanism compatible with:
* deterministic record/replay (effects are explicit)
* bus-first coupling (effects can be published as bus fields)

In practice, the runtime passes an immutable boundary context (e.g. `ScenarioContext`) as
`sim` and the caller owns the state being updated (e.g. an `EventScenarioState`).

Notes
-----
Julia cannot *enforce* purity for arbitrary closures. This module makes the safe path
idiomatic by:

* passing a restricted boundary context rather than the full engine
* threading an explicit `state` value through the scheduler
"""
module Events

export AbstractEvent, AtTime, AtStep, When, EventScheduler, step_events!, next_at_time_us

abstract type AbstractEvent end

"""Time-triggered one-shot event.

To avoid float comparison drift, fire times are stored as **integer microseconds**.

Semantics: the event fires once when `time_us(sim) >= t_fire`.

Action signature:
```
action(state, sim, t_s)::state2
```
"""
struct AtTime{F} <: AbstractEvent
    t_fire_us::UInt64
    action::F
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

"""Step-triggered one-shot event.

Semantics: the event fires once when `step(sim) >= step_fire`.

The meaning of `step(sim)` is simulator-defined. For the canonical `Runtime.Engine`,
`step` counts processed event-axis boundaries and is deterministic.

Action signature:
```
action(state, sim, t_s)::state2
```
"""
struct AtStep{F} <: AbstractEvent
    step_fire::Int
    action::F
end

function AtStep(step_fire::Integer, action::F) where {F}
    s = Int(step_fire)
    s >= 0 || error("AtStep requires step_fire >= 0")
    return AtStep{F}(s, action)
end

"""Condition-triggered one-shot event.

Condition signature:
```
condition(state, sim, t_s)::Bool
```

Action signature:
```
action(state, sim, t_s)::state2
```
"""
struct When{C,F} <: AbstractEvent
    condition::C
    action::F
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

This is intended for hybrid/event-driven simulators to include scenario `AtTime` events as
true event boundaries.

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

@inline function _sim_now_us(sim)::UInt64
    T = typeof(sim)
    # Engine-style timebase.
    if Base.hasfield(T, :t_us)
        return getfield(sim, :t_us)
    end
    error("sim type $(T) does not provide a compatible timebase for AtTime events")
end

@inline function _sim_step(sim)::Int
    T = typeof(sim)
    if Base.hasfield(T, :step)
        return Int(getfield(sim, :step))
    end
    error("sim type $(T) does not provide a compatible step counter for AtStep events")
end


"""Add an event to the scheduler."""
function Base.push!(s::EventScheduler, e::AbstractEvent)
    push!(s.events, e)
    push!(s.fired, false)
    return s
end


# ------------------------------------------------------------
# Calling helpers
# ------------------------------------------------------------

@inline function _call_condition(cond, state, sim, t::Float64)::Bool
    applicable(cond, state, sim, t) ||
        error("Event condition must accept (state, sim, t). Got: " * string(typeof(cond)))
    return Bool(cond(state, sim, t))
end

@inline function _call_action(act, state, sim, t::Float64)
    applicable(act, state, sim, t) ||
        error("Event action must accept (state, sim, t). Got: " * string(typeof(act)))
    return act(state, sim, t)
end


@inline _should_fire(e::AtTime, state, sim, t::Float64) = (_sim_now_us(sim) >= e.t_fire_us)
@inline _should_fire(e::AtStep, state, sim, t::Float64) = (_sim_step(sim) >= e.step_fire)
@inline _should_fire(e::When, state, sim, t::Float64) =
    _call_condition(e.condition, state, sim, t)

@inline _apply!(e::AtTime, state, sim, t::Float64) = _call_action(e.action, state, sim, t)
@inline _apply!(e::AtStep, state, sim, t::Float64) = _call_action(e.action, state, sim, t)
@inline _apply!(e::When, state, sim, t::Float64) = _call_action(e.action, state, sim, t)


"""Evaluate any events that should fire at this boundary and return the updated `state`.

Parameters
----------
- `sched`: event scheduler
- `state`: caller-owned state threaded through actions
- `sim`: immutable boundary context (e.g. `ScenarioContext`)
- `t`: time in seconds (convenience float)

Returns
-------
Updated state value after applying all due events in scheduler order.
"""
function step_events!(sched::EventScheduler, state, sim, t::Float64)
    st = state
    @inbounds for i = 1:length(sched.events)
        if !sched.fired[i]
            e = sched.events[i]
            if _should_fire(e, st, sim, t)
                st = _apply!(e, st, sim, t)
                sched.fired[i] = true
            end
        end
    end
    return st
end


end # module Events
