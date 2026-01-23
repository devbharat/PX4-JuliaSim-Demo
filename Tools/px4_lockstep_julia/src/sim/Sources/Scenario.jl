"""Sources.Scenario

Scenario sources publish *high-level* autopilot commands and faults into the bus.

Scenario outputs are treated as ZOH between scenario boundaries.

This is the primary place where hybrid-system events enter the sim:
- scripted arm/mission/rtl commands
- plant-affecting faults (motor disable, battery disconnect, sensor faults)

Scheduling
----------
For **live** scenarios with a static scheduler, `AtTime` events can be discovered up front.
These event times should be treated as true engine boundaries (hybrid correctness),
independent of autopilot rate.

For dynamic scenarios that create new events on the fly, a fixed pre-built timeline
is not sufficient (planned future extension).
"""

using ..Autopilots: AutopilotCommand
using ..Faults: FaultState
using ..Types: vec3

using ..Scenario
using ..Events: AtTime
using ..Recording:
    AbstractTrace,
    ZOHTrace,
    ZOHCursor,
    SampleHoldTrace,
    SampleHoldCursor,
    sample

"""Base type for scenario sources."""
abstract type AbstractScenarioSource <: AbstractSource end

"""No-op scenario source.

Publishes default autopilot command (all false), `landed=false`, and no faults.
"""
struct NullScenarioSource <: AbstractScenarioSource end

function update!(::NullScenarioSource, bus::SimBus, plant_state, t_us::UInt64)
    bus.ap_cmd = AutopilotCommand()
    bus.landed = false
    bus.faults = FaultState()
    bus.wind_dist_ned = vec3(0.0, 0.0, 0.0)
    return nothing
end

"""Replay scenario source.

Samples scenario output traces and publishes into:
- `bus.ap_cmd`
- `bus.landed`
- `bus.faults`
- `bus.wind_dist_ned` (optional)

All traces are expected to be ZOH semantics.
"""
struct ReplayScenarioSource{
    TA<:AbstractTrace{AutopilotCommand},
    TL<:AbstractTrace{Bool},
    TF<:AbstractTrace{FaultState},
    TW,
} <: AbstractScenarioSource
    ap_cmd::TA
    landed::TL
    faults::TF
    wind_dist::TW
end

function ReplayScenarioSource(ap_cmd, landed, faults; wind_dist = nothing)
    ap_cmd = ap_cmd isa ZOHTrace ? ZOHCursor(ap_cmd) : ap_cmd
    landed = landed isa ZOHTrace ? ZOHCursor(landed) : landed
    faults = faults isa ZOHTrace ? ZOHCursor(faults) : faults
    if wind_dist !== nothing && wind_dist isa ZOHTrace
        wind_dist = ZOHCursor(wind_dist)
    end
    return ReplayScenarioSource(ap_cmd, landed, faults, wind_dist)
end

function update!(src::ReplayScenarioSource, bus::SimBus, plant_state, t_us::UInt64)
    bus.ap_cmd = sample(src.ap_cmd, t_us)
    bus.landed = sample(src.landed, t_us)
    bus.faults = sample(src.faults, t_us)
    if src.wind_dist === nothing
        bus.wind_dist_ned = vec3(0.0, 0.0, 0.0)
    else
        bus.wind_dist_ned = sample(src.wind_dist, t_us)
    end
    return nothing
end

"""Live scenario source.

This bridges the existing `Sim.Scenario` interface into the runtime bus.

Publishes:
- `bus.ap_cmd`
- `bus.landed`
- `bus.faults`

Scenarios receive a **read-only context snapshot** and publish explicit outputs.
"""
mutable struct LiveScenarioSource{S} <: AbstractScenarioSource
    scenario::S
    step::Int
end

LiveScenarioSource(scenario) = LiveScenarioSource{typeof(scenario)}(scenario, 0)

function update!(src::LiveScenarioSource, bus::SimBus, plant_state, t_us::UInt64)
    rb = _rb_state(plant_state)
    t_s = Float64(t_us) * 1e-6

    ctx = Scenario.ScenarioContext(
        t_us = t_us,
        t_s = t_s,
        step = src.step,
        plant = plant_state,
        rb = rb,
    )
    out = Scenario.scenario_outputs(src.scenario, ctx)
    bus.ap_cmd = out.ap_cmd
    bus.landed = out.landed
    bus.faults = out.faults
    bus.wind_dist_ned = out.wind_dist_ned

    # Deterministic boundary counter for AtStep scenario events.
    src.step += 1
    return nothing
end

# ------------------------------------------------------------
# Scheduling helpers (static AtTime discovery)
# ------------------------------------------------------------

"""Return discrete *AtTime* event times (microseconds) for a scenario source.

Used by `Runtime.build_timeline_for_run` so scenario events become true boundaries.

Notes
-----
- This is a static discovery mechanism; dynamic event insertion is not yet supported.
"""
function event_times_us(::AbstractScenarioSource, t0_us::UInt64, t_end_us::UInt64)
    return UInt64[]
end

function event_times_us(src::LiveScenarioSource, t0_us::UInt64, t_end_us::UInt64)
    s = src.scenario
    if hasproperty(s, :scheduler)
        sched = getfield(s, :scheduler)
        if hasproperty(sched, :events)
            times = UInt64[]
            for e in getfield(sched, :events)
                if e isa AtTime
                    t = getfield(e, :t_fire_us)
                    if t >= t0_us && t <= t_end_us
                        push!(times, t)
                    end
                end
            end
            sort!(times)
            unique!(times)
            return times
        end
    end
    return UInt64[]
end


"""Event times for replay scenario sources.

Replay trace axes are used as the scenario axes.
"""
function event_times_us(src::ReplayScenarioSource, t0_us::UInt64, t_end_us::UInt64)
    ts = vcat(src.ap_cmd.axis.t_us, src.landed.axis.t_us, src.faults.axis.t_us)
    out = UInt64[t for t in ts if (t >= t0_us && t <= t_end_us)]
    sort!(out)
    unique!(out)
    return out
end

export AbstractScenarioSource, NullScenarioSource, ReplayScenarioSource, LiveScenarioSource
