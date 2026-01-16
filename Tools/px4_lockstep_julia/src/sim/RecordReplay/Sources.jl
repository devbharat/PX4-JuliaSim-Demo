"""Component sources (live + replay).

A **source** publishes one or more bus fields at its scheduled times.

Sources exist in two modes:
- *Live*: compute outputs from models (PX4 lockstep, OU wind, scenario logic)
- *Replay*: publish recorded outputs from traces

This file defines the abstract interfaces and a few basic replay sources.
Live sources are left as TODO shells in this first pass.
"""

using ..Types: Vec3
using ..Vehicles: ActuatorCommand
using ..RigidBody: RigidBodyState
using ..Environment: AbstractWind, step_wind!, sample_wind!, wind_velocity
using ..Autopilots: AutopilotCommand, autopilot_step
using ..Estimators: EstimatedState, AbstractEstimator, estimate!
using ..Faults: FaultState, SENSOR_FAULT_EST_FREEZE
import ..Scenario
using ..Events: AtTime

using StaticArrays: SVector

# Use trace sampling utilities.
# (Types are defined in Traces.jl inside this same module.)

# ----------------
# Abstract source interfaces
# ----------------

abstract type AbstractSource end

abstract type AbstractAutopilotSource <: AbstractSource end
abstract type AbstractWindSource <: AbstractSource end
abstract type AbstractScenarioSource <: AbstractSource end
abstract type AbstractEstimatorSource <: AbstractSource end

"""No-op scenario source.

Useful for plant-only replay runs where no scenario events exist.
"""
struct NullScenarioSource <: AbstractScenarioSource end

function update!(::NullScenarioSource, bus::SimBus, plant_state, t_us::UInt64)
    bus.ap_cmd = AutopilotCommand()
    bus.landed = false
    bus.faults = FaultState()
    return nothing
end

"""No-op estimator source.

Useful for replay runs that do not inject estimator noise.
"""
struct NullEstimatorSource <: AbstractEstimatorSource end

function update!(::NullEstimatorSource, bus::SimBus, plant_state, t_us::UInt64)
    # "Null" estimator = truth-as-estimate. This keeps the bus contract simple:
    # autopilot always consumes `bus.est`.
    rb = _rb_state(plant_state)
    bus.est = EstimatedState(
        pos_ned = rb.pos_ned,
        vel_ned = rb.vel_ned,
        q_bn = rb.q_bn,
        ω_body = rb.ω_body,
    )
    return nothing
end

"""Update a source at time `t_us`.

The engine calls sources at deterministic event boundaries. For periodic sources
(autopilot, wind, logging), the engine typically calls `update!` only when
`t_us` lies on the corresponding axis. For non-periodic sources (scenario), the
engine may call `update!` at *every* boundary. Concrete sources should be robust
to being called when no work is due (i.e. return quickly).

Arguments
---------
- `bus`: the mutable `SimBus` shared between components
- `plant_state`: current plant continuous state (type depends on engine)

Returns
-------
`Nothing`. Sources mutate `bus` (and possibly scenario state).
"""
function update!(::AbstractSource, ::SimBus, plant_state, t_us::UInt64)
    error("TODO: update! must be implemented by concrete sources")
end

# ----------------
# Replay sources
# ----------------

"""Replay autopilot command source.

Publishes `bus.cmd` at autopilot axis times.

Interpolation
-------------
Commands are ZOH between autopilot ticks.
"""
struct ReplayAutopilotSource <: AbstractAutopilotSource
    cmd::ZOHTrace{ActuatorCommand}
end

function update!(src::ReplayAutopilotSource, bus::SimBus, plant_state, t_us::UInt64)
    bus.cmd = sample(src.cmd, t_us)
    return nothing
end

"""Replay wind source.

Publishes `bus.wind_ned` at wind axis times.

Interpolation
-------------
Wind is sample-and-hold between wind ticks.
"""
struct ReplayWindSource <: AbstractWindSource
    wind_ned::SampleHoldTrace{Vec3}
end

function update!(src::ReplayWindSource, bus::SimBus, plant_state, t_us::UInt64)
    bus.wind_ned = sample(src.wind_ned, t_us)
    return nothing
end

"""Replay estimator source.

Publishes `bus.est` at autopilot axis times.

Interpolation
-------------
The estimator output is treated as ZOH between ticks.
"""
struct ReplayEstimatorSource <: AbstractEstimatorSource
    est::ZOHTrace{EstimatedState}
end

function update!(src::ReplayEstimatorSource, bus::SimBus, plant_state, t_us::UInt64)
    bus.est = sample(src.est, t_us)
    return nothing
end

"""Replay scenario source.

This first-pass replay source replays the *scenario outputs* that couple into the
rest of the sim through the bus:

* `bus.ap_cmd`
* `bus.landed`

Those signals are assumed ZOH between scenario axis samples.

Why not replay "events"?
------------------------
Replaying event payloads (motor failures, gust injections) requires a stable, versioned
event schema and a clear contract for how faults couple into the plant.

Option A's long-term direction is: scenario publishes faults onto the bus (as typed
signals) rather than mutating mutable component objects. Once that exists, replaying
scenario behavior reduces to replaying bus streams.
"""
struct ReplayScenarioSource <: AbstractScenarioSource
    ap_cmd::ZOHTrace{AutopilotCommand}
    landed::ZOHTrace{Bool}
    faults::ZOHTrace{FaultState}
end

function update!(src::ReplayScenarioSource, bus::SimBus, plant_state, t_us::UInt64)
    bus.ap_cmd = sample(src.ap_cmd, t_us)
    bus.landed = sample(src.landed, t_us)
    bus.faults = sample(src.faults, t_us)
    return nothing
end

# ----------------
# Live sources (TODO shells)
# ----------------

@inline function _rb_state(plant_state)
    if plant_state isa RigidBodyState
        return plant_state
    elseif hasproperty(plant_state, :rb)
        # PlantState{N} has a concrete field `rb`.
        return getfield(plant_state, :rb)
    end
    error(
        "RecordReplay: cannot extract rigid-body state from plant_state of type $(typeof(plant_state)).",
    )
end

@inline _sanitize01(x::Real) = isfinite(x) ? clamp(Float64(x), 0.0, 1.0) : 0.0
@inline _sanitize11(x::Real) = isfinite(x) ? clamp(Float64(x), -1.0, 1.0) : 0.0

"""Live PX4 lockstep autopilot source.

This wraps the existing `Sim.Autopilots.autopilot_step` interface.

Notes
-----
* The autopilot consumes:
  - rigid-body state (currently truth-as-estimate)
  - `bus.ap_cmd` (high-level arm/mission request)
  - `bus.landed`
  - `bus.battery` (telemetry)
* It publishes `bus.cmd` (motor/servo commands) as a **ZOH** stream.
"""
mutable struct LiveAutopilotSource{A} <: AbstractAutopilotSource
    ap::A
    last_out::Any
    sanitize::Bool
end

LiveAutopilotSource(ap; sanitize::Bool = true) = LiveAutopilotSource(ap, nothing, sanitize)

function update!(src::LiveAutopilotSource, bus::SimBus, plant_state, t_us::UInt64)
    # Autopilot consumes the *estimated* state from the bus.
    # `NullEstimatorSource` ensures this is truth-as-estimate when no estimator is configured.
    est = bus.est
    out = autopilot_step(
        src.ap,
        t_us,
        est.pos_ned,
        est.vel_ned,
        est.q_bn,
        est.ω_body,
        bus.ap_cmd;
        landed = bus.landed,
        battery = bus.battery,
    )

    src.last_out = out

    # PX4 lockstep provides 16 motors and 8 servos. We use the sim's canonical
    # sizes (12 motors, 8 servos) and clamp to a safe range.
    motors_raw = out.actuator_motors
    servos_raw = out.actuator_servos

    if src.sanitize
        motors = SVector{12,Float64}(ntuple(i -> _sanitize01(motors_raw[i]), 12))
        servos = SVector{8,Float64}(ntuple(i -> _sanitize11(servos_raw[i]), 8))
    else
        motors = SVector{12,Float64}(motors_raw[1:12])
        servos = SVector{8,Float64}(servos_raw[1:8])
    end

    bus.cmd = ActuatorCommand(motors = motors, servos = servos)
    return nothing
end

"""Live wind source.

Advances the deterministic wind model once per wind tick and publishes the sampled
wind velocity into `bus.wind_ned`.

Determinism rules
-----------------
* The RNG MUST be dedicated to this wind source.
* No RNG usage is allowed outside of `update!`.
"""
mutable struct LiveWindSource{W,R} <: AbstractWindSource
    wind::W
    rng::R
    dt_wind_s::Float64

    function LiveWindSource(wind::W, rng::R, dt_wind_s::Real) where {W<:AbstractWind,R}
        return new{W,R}(wind, rng, Float64(dt_wind_s))
    end
end

function update!(src::LiveWindSource, bus::SimBus, plant_state, t_us::UInt64)
    rb = _rb_state(plant_state)
    t_s = Float64(t_us) * 1e-6
    pos = rb.pos_ned

    step_wind!(src.wind, pos, t_s, src.dt_wind_s, src.rng)
    sample_wind!(src.wind, pos, t_s)
    bus.wind_ned = wind_velocity(src.wind, pos, t_s)
    return nothing
end

mutable struct ScenarioSimView{E,V,B}
    t_us::UInt64
    env::E
    vehicle::V
    battery::B
end

"""Live scenario source.

This is an adapter around the existing `Sim.Scenario` API.

It is responsible for producing:
* `bus.ap_cmd`
* `bus.landed`

and for applying scenario events that mutate component state (e.g., wind gust
injection, motor disable) through the provided `simview`.
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
    # Scenario-controlled fault state is a first-class bus signal.
    bus.faults = Scenario.scenario_faults(src.scenario, t_s, rb, src.simview)
    bus.ap_cmd = ap_cmd
    bus.landed = landed
    return nothing
end

"""Live estimator/noise injection source.

TODO: implement by reusing `Sim.Estimators` in discrete-time update mode.
"""
struct LiveEstimatorSource <: AbstractEstimatorSource
    est::AbstractEstimator
    rng::Any
    dt_est_s::Float64
end

function update!(src::LiveEstimatorSource, bus::SimBus, plant_state, t_us::UInt64)
    # Optional: allow scenarios to freeze the estimator output via a bus-level fault
    # signal. This avoids mutating estimator internals and is replayable.
    if (bus.faults.sensor_fault_mask & SENSOR_FAULT_EST_FREEZE) != 0
        return nothing
    end
    rb = _rb_state(plant_state)
    t_s = Float64(t_us) * 1e-6
    bus.est = estimate!(src.est, src.rng, t_s, rb, src.dt_est_s)
    return nothing
end

# ----------------
# Scheduling helpers
# ----------------

"""Return the set of discrete *AtTime* scenario event times (microseconds).

This is used to build `timeline.scn` so scenario AtTime events are treated as true
event boundaries.

Notes
-----
* This inspects the scenario's scheduler for `Events.AtTime` entries.
* Dynamically-added AtTime events (e.g., from `When(...)` actions) are not yet
  supported by the fixed timeline implementation.
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

We treat the replay trace's axis as the scenario axis.
"""
function event_times_us(src::ReplayScenarioSource, t0_us::UInt64, t_end_us::UInt64)
    # Defensive: keep event boundaries aligned with *all* scenario-replayed outputs.
    # (In normal usage these all share the same axis: `timeline.scn`.)
    ts = vcat(src.ap_cmd.axis.t_us, src.landed.axis.t_us, src.faults.axis.t_us)
    # Filter defensively to the requested window.
    out = UInt64[t for t in ts if (t >= t0_us && t <= t_end_us)]
    sort!(out)
    unique!(out)
    return out
end
