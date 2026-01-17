"""Sources.Scenario

Scenario sources publish *high-level* autopilot commands and faults into the bus.

Scenario outputs are treated as ZOH between scenario boundaries.

This is the primary place where hybrid-system events enter the sim:
- scripted arm/mission/rtl commands
- plant-affecting faults (motor disable, battery disconnect, sensor faults)

Scheduling
----------
For **live** scenarios with a static scheduler, we can discover `AtTime` events up front.
These event times should be treated as true engine boundaries (hybrid correctness),
independent of autopilot rate.

For dynamic scenarios that create new events on the fly, a fixed pre-built timeline
is not sufficient (planned future extension).
"""

using ..Autopilots: AutopilotCommand
using ..Faults: FaultState

using ..Scenario
using ..Events: AtTime

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
    return nothing
end

"""Replay scenario source.

Samples scenario output traces and publishes into `bus.ap_cmd`, `bus.landed`, `bus.faults`.

All traces are expected to be ZOH semantics.
"""
struct ReplayScenarioSource{
    TA<:ZOHTrace{AutopilotCommand},
    TL<:ZOHTrace{Bool},
    TF<:ZOHTrace{FaultState},
} <: AbstractScenarioSource
    ap_cmd::TA
    landed::TL
    faults::TF
end

function update!(src::ReplayScenarioSource, bus::SimBus, plant_state, t_us::UInt64)
    bus.ap_cmd = sample(src.ap_cmd, t_us)
    bus.landed = sample(src.landed, t_us)
    bus.faults = sample(src.faults, t_us)
    return nothing
end

"""Adapter that provides a scenario with access to environment/vehicle/battery objects.

This is a compatibility shim for pre-record/replay scenario code.
Long term, scenarios should publish bus-level commands/faults only (no mutation).
"""
mutable struct ScenarioSimView{E,V,B}
    t_us::UInt64
    env::E
    vehicle::V
    battery::B
end

"""Live scenario source.

This bridges the existing `Sim.Scenario` interface into the runtime bus.

Publishes:
- `bus.ap_cmd`
- `bus.landed`
- `bus.faults`

May also trigger scenario-side mutations via the provided `simview` (legacy).
"""
mutable struct LiveScenarioSource{S,SV} <: AbstractScenarioSource
    scenario::S
    simview::SV
end

function LiveScenarioSource(scenario; env, vehicle, battery)
    simview = ScenarioSimView(UInt64(0), env, vehicle, battery)
    return LiveScenarioSource{typeof(scenario),typeof(simview)}(scenario, simview)
end

function update!(src::LiveScenarioSource, bus::SimBus, plant_state, t_us::UInt64)
    rb = _rb_state(plant_state)
    src.simview.t_us = t_us
    t_s = Float64(t_us) * 1e-6

    ap_cmd, landed = Scenario.scenario_step(src.scenario, t_s, rb, src.simview)
    bus.faults = Scenario.scenario_faults(src.scenario, t_s, rb, src.simview)
    bus.ap_cmd = ap_cmd
    bus.landed = landed
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

We treat the replay trace axes as the scenario axes.
"""
function event_times_us(src::ReplayScenarioSource, t0_us::UInt64, t_end_us::UInt64)
    ts = vcat(src.ap_cmd.axis.t_us, src.landed.axis.t_us, src.faults.axis.t_us)
    out = UInt64[t for t in ts if (t >= t0_us && t <= t_end_us)]
    sort!(out)
    unique!(out)
    return out
end

export AbstractScenarioSource,
    NullScenarioSource,
    ReplayScenarioSource,
    LiveScenarioSource,
    ScenarioSimView
