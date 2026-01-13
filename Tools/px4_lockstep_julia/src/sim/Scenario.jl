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

export AbstractScenario, ScriptedScenario, scenario_step

abstract type AbstractScenario end

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

end # module Scenario
