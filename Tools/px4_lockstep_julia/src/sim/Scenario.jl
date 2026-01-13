"""PX4Lockstep.Sim.Scenario

Scenario drivers: provide events and inputs over time.

The sim engine calls the scenario each tick to determine:

* high-level autopilot commands (arm, start mission, request RTL, ...)
* whether the vehicle is considered 'landed'
* (optionally) environment disturbances or external forces

Keeping scenarios as a separate layer makes it easy to test the same vehicle+world+autopilot under many conditions.
"""
module Scenario

using ..Types: Vec3
using ..RigidBody: RigidBodyState
using ..Autopilots: AutopilotCommand
using ..Events: EventScheduler, AbstractEvent, AtTime, When, step_events!
using ..Environment: add_step_gust!
using ..Propulsion: set_motor_enabled!
import ..Powertrain

export AbstractScenario,
    ScriptedScenario,
    EventScenario,
    scenario_step,
    # Convenience helpers for EventScenario
    arm_at!,
    disarm_at!,
    mission_start_at!,
    mission_pulse_at!,
    rtl_pulse_at!,
    wind_step_at!,
    fail_motor_at!,
    when_soc_below!

abstract type AbstractScenario end

"""Fallback scenario interface that can optionally see the full simulation.

Most scenarios only depend on time and vehicle state. Event-driven scenarios benefit from
access to the simulation object (to inject wind changes, failures, etc.).

The sim engine calls the 4-arg method:

    scenario_step(scenario, t, state, sim)

and by default this forwards to the legacy 3-arg method.
"""
scenario_step(s::AbstractScenario, t::Float64, x::RigidBodyState, sim) =
    scenario_step(s, t, x)

"""A simple time-driven scenario.

This is a pragmatic replacement for the logic embedded in the old `basic_step.jl` example.
It is intentionally minimal and deterministic.
"""
Base.@kwdef mutable struct ScriptedScenario <: AbstractScenario
    arm_time_s::Float64 = 1.0
    mission_time_s::Float64 = 2.0
    rtl_time_s::Union{Nothing,Float64} = nothing
    disarm_time_s::Union{Nothing,Float64} = nothing
    # When set, force landed=false after mission start to ensure takeoff in pure lockstep.
    takeoff_override_delay_s::Union{Nothing,Float64} = 0.5
    # Land detection thresholds (best effort). In many PX4 stacks, landed status affects arming + takeoff.
    land_z_thresh_m::Float64 = 0.05
    land_vz_thresh_mps::Float64 = 0.2
end

"""Compute scenario outputs for this tick.

Returns:
* `cmd::AutopilotCommand`
* `landed::Bool`
"""
function scenario_step(s::ScriptedScenario, t::Float64, x::RigidBodyState)
    armed = t >= s.arm_time_s
    request_mission = t >= s.mission_time_s
    request_rtl = (!isnothing(s.rtl_time_s)) && (t >= s.rtl_time_s)
    if (!isnothing(s.disarm_time_s)) && (t >= s.disarm_time_s)
        armed = false
    end

    landed =
        (x.pos_ned[3] >= -s.land_z_thresh_m) && (abs(x.vel_ned[3]) < s.land_vz_thresh_mps)
    if !isnothing(s.takeoff_override_delay_s)
        if armed && request_mission && t >= (s.mission_time_s + s.takeoff_override_delay_s)
            landed = false
        end
    end

    cmd = AutopilotCommand(
        armed = armed,
        request_mission = request_mission,
        request_rtl = request_rtl,
    )
    return cmd, landed
end

############################
# Event-driven scenario
############################

"""A small event-driven scenario with an explicit scheduler.

This replaces the "hard-coded times" pattern with composable events:
* arm at time t
* pulse mission/RTL requests
* inject wind gusts
* fail motors
* trigger actions on SOC thresholds

Events are one-shot and deterministic.
"""
Base.@kwdef mutable struct EventScenario <: AbstractScenario
    cmd::AutopilotCommand = AutopilotCommand()
    scheduler::EventScheduler = EventScheduler()

    # Land detection thresholds (best effort).
    land_z_thresh_m::Float64 = 0.05
    land_vz_thresh_mps::Float64 = 0.2

    # Optional: once mission has started, force landed=false after a delay.
    takeoff_override_delay_s::Union{Nothing,Float64} = 0.5
    _mission_started::Bool = false
    _mission_start_time_s::Float64 = NaN
end

@inline function _set_cmd!(
    s::EventScenario;
    armed::Bool = s.cmd.armed,
    request_mission::Bool = s.cmd.request_mission,
    request_rtl::Bool = s.cmd.request_rtl,
)
    s.cmd = AutopilotCommand(
        armed = armed,
        request_mission = request_mission,
        request_rtl = request_rtl,
    )
    return nothing
end

function scenario_step(s::EventScenario, t::Float64, x::RigidBodyState, sim)
    # Apply any pending events.
    step_events!(s.scheduler, sim, t)

    # Latch mission start to support takeoff override even when mission request is pulsed.
    if s.cmd.request_mission && !s._mission_started
        s._mission_started = true
        s._mission_start_time_s = t
    end

    landed =
        (x.pos_ned[3] >= -s.land_z_thresh_m) && (abs(x.vel_ned[3]) < s.land_vz_thresh_mps)
    if !isnothing(s.takeoff_override_delay_s)
        if s.cmd.armed &&
           s._mission_started &&
           t >= (s._mission_start_time_s + s.takeoff_override_delay_s)
            landed = false
        end
    end

    return s.cmd, landed
end

############################
# Convenience helpers
############################

"""Arm at time `t_arm`."""
function arm_at!(s::EventScenario, t_arm::Float64)
    push!(s.scheduler, AtTime(t_arm, (sim, t)->_set_cmd!(s; armed = true)))
    return s
end

"""Disarm at time `t_disarm`."""
function disarm_at!(s::EventScenario, t_disarm::Float64)
    push!(s.scheduler, AtTime(t_disarm, (sim, t)->_set_cmd!(s; armed = false)))
    return s
end

"""Set mission request high at time `t_start` (latched until cleared)."""
function mission_start_at!(s::EventScenario, t_start::Float64)
    push!(s.scheduler, AtTime(t_start, (sim, t)->_set_cmd!(s; request_mission = true)))
    return s
end

"""Pulse mission request for `pulse_s` seconds starting at `t_start`."""
function mission_pulse_at!(s::EventScenario, t_start::Float64; pulse_s::Float64 = 0.2)
    push!(s.scheduler, AtTime(t_start, (sim, t)->_set_cmd!(s; request_mission = true)))
    push!(
        s.scheduler,
        AtTime(
            t_start + max(0.0, pulse_s),
            (sim, t)->_set_cmd!(s; request_mission = false),
        ),
    )
    return s
end

"""Pulse RTL request for `pulse_s` seconds starting at `t_start`."""
function rtl_pulse_at!(s::EventScenario, t_start::Float64; pulse_s::Float64 = 0.2)
    push!(s.scheduler, AtTime(t_start, (sim, t)->_set_cmd!(s; request_rtl = true)))
    push!(
        s.scheduler,
        AtTime(t_start + max(0.0, pulse_s), (sim, t)->_set_cmd!(s; request_rtl = false)),
    )
    return s
end

"""Inject a step gust into an OUWind for `duration_s` seconds at time `t_start`."""
function wind_step_at!(
    s::EventScenario,
    t_start::Float64,
    dv_ned::Vec3;
    duration_s::Float64 = 2.0,
)
    push!(
        s.scheduler,
        AtTime(t_start, (sim, t)->add_step_gust!(sim.env.wind, dv_ned, t, duration_s)),
    )
    return s
end

"""Disable a motor at time `t_start`.

This assumes the simulation's vehicle instance has a `propulsion` object compatible with
`Propulsion.QuadRotorSet`.
"""
function fail_motor_at!(s::EventScenario, t_start::Float64, motor_index::Int)
    push!(
        s.scheduler,
        AtTime(t_start, (sim, t)->begin
            if !hasproperty(sim.vehicle, :propulsion)
                return
            end
            p = getfield(sim.vehicle, :propulsion)
            if p === nothing
                return
            end
            if motor_index < 1 || motor_index > length(p.units)
                return
            end
            set_motor_enabled!(p.units[motor_index], false)
        end),
    )
    return s
end

"""Trigger `action(sim, t)` when SOC (battery remaining) drops below `threshold`.

Example:

    when_soc_below!(scenario, 0.12) do sim, t
        rtl_pulse_at!(scenario, t)
    end
"""
function when_soc_below!(s::EventScenario, threshold::Float64, action::Function)
    push!(
        s.scheduler,
        When(
            (sim, t)->Powertrain.status(sim.battery).remaining <= threshold,
            (sim, t)->action(sim, t),
        ),
    )
    return s
end

end # module Scenario
