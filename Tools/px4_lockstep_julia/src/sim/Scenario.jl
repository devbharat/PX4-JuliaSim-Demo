"""PX4Lockstep.Sim.Scenario

Scenario drivers: provide events and inputs over time.

The sim engine calls the scenario each tick to determine:

* high-level autopilot commands (arm, start mission, request RTL, ...)
* whether the vehicle is considered 'landed'
* (optionally) environment disturbances or external forces

Keeping scenarios as a separate layer makes it easy to test the same vehicle+world+autopilot under many conditions.
"""
module Scenario

using ..Types: Vec3, vec3
using ..RigidBody: RigidBodyState
using ..Autopilots: AutopilotCommand
using ..Events:
    EventScheduler, AbstractEvent, AtTime, AtStep, When, step_events!, next_at_time_us
using ..Faults:
    FaultState,
    disable_motor,
    enable_motor,
    set_battery_connected,
    add_sensor_faults,
    clear_sensor_faults

export AbstractScenario,
    ScriptedScenario,
    EventScenario,
    ScenarioContext,
    ScenarioOutput,
    scenario_outputs,
    scenario_step,
    scenario_faults,
    scenario_wind_dist,
    process_events!,
    next_event_us,
    # Convenience helpers for EventScenario
    arm_at!,
    disarm_at!,
    mission_start_at!,
    mission_pulse_at!,
    rtl_pulse_at!,
    wind_step_at!,
    fail_motor_at!,
    restore_motor_at!,
    battery_disconnect_at!,
    battery_reconnect_at!,
    sensor_fail_at!,
    sensor_restore_at!,
    when_soc_below!

abstract type AbstractScenario end

"""Immutable boundary-time scenario context.

This is the *only* thing a scenario receives from the runtime.

Design intent
-------------
- No scenario code should receive mutable references to engine internals (env, vehicle, battery objects).
- Anything a scenario wants to influence must flow through explicit outputs that can be recorded/replayed.

Fields
------
- `t_us`: authoritative integer microseconds at the current boundary.
- `t_s`: convenience float seconds derived from `t_us`.
- `step`: deterministic boundary counter (0-based).
- `plant`: plant state snapshot (immutable).
- `rb`: rigid-body snapshot (duplicated for convenience).
"""
Base.@kwdef struct ScenarioContext{PS}
    t_us::UInt64
    t_s::Float64
    step::Int
    plant::PS
    rb::RigidBodyState
end

"""Scenario outputs published into the runtime bus.

All fields are treated as **sample-and-hold** between scenario boundaries.
"""
Base.@kwdef struct ScenarioOutput
    ap_cmd::AutopilotCommand = AutopilotCommand()
    landed::Bool = false
    faults::FaultState = FaultState()
    # Additive wind disturbance request (NED, m/s). Applied by the wind source at wind ticks.
    wind_dist_ned::Vec3 = vec3(0.0, 0.0, 0.0)
end

"""Compute all scenario outputs for the current boundary.

Scenarios can implement:
- `scenario_step(s, t_s, rb, ctx)`
- `scenario_faults(s, t_s, rb, ctx)`
- `scenario_wind_dist(s, t_s, rb, ctx)`
"""
function scenario_outputs(s::AbstractScenario, ctx::ScenarioContext)::ScenarioOutput
    cmd, landed = scenario_step(s, ctx.t_s, ctx.rb, ctx)
    faults = scenario_faults(s, ctx.t_s, ctx.rb, ctx)
    wind_dist = scenario_wind_dist(s, ctx.t_s, ctx.rb, ctx)
    return ScenarioOutput(
        ap_cmd = cmd,
        landed = landed,
        faults = faults,
        wind_dist_ned = wind_dist,
    )
end

"""Scenario interface: return autopilot command + landed flag."""
scenario_step(::AbstractScenario, ::Float64, ::RigidBodyState, ::ScenarioContext) =
    error("scenario_step must be implemented for the scenario type")

"""Return the current fault state published by the scenario (default: none)."""
scenario_faults(::AbstractScenario, ::Float64, ::RigidBodyState, ::ScenarioContext) =
    FaultState()

"""Return an additive wind disturbance request (default: zero)."""
scenario_wind_dist(::AbstractScenario, ::Float64, ::RigidBodyState, ::ScenarioContext) =
    vec3(0.0, 0.0, 0.0)

"""Process any discrete events owned by the scenario at time `t`.

Default: no-op.

This hook exists so event-driven simulators can treat scenario `AtTime` events as true
event boundaries (and not merely as part of an autopilot tick).
"""
process_events!(::AbstractScenario, ctx, t::Float64) = nothing

"""Return the next discrete scenario event time (microseconds) strictly in the future.

Default: `nothing`.

Event-driven simulators can include this time as an event boundary.
"""
next_event_us(::AbstractScenario, ctx)::Union{UInt64,Nothing} = nothing

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
function scenario_step(
    s::ScriptedScenario,
    t::Float64,
    x::RigidBodyState,
    ctx::ScenarioContext,
)
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

Architecture contract
---------------------
Event actions are expected to be **pure boundary updates**: they return a new
`EventScenarioState` rather than mutating simulation internals.
"""
Base.@kwdef struct EventScenarioState
    cmd::AutopilotCommand = AutopilotCommand()
    # Scenario-controlled fault state (sample-and-hold).
    faults::FaultState = FaultState()
    # Additive wind disturbance request (sample-and-hold).
    wind_dist_ned::Vec3 = vec3(0.0, 0.0, 0.0)
end

Base.@kwdef mutable struct EventScenario <: AbstractScenario
    state::EventScenarioState = EventScenarioState()
    scheduler::EventScheduler = EventScheduler()

    # Land detection thresholds (best effort).
    land_z_thresh_m::Float64 = 0.05
    land_vz_thresh_mps::Float64 = 0.2

    # Optional: once mission has started, force landed=false after a delay.
    takeoff_override_delay_s::Union{Nothing,Float64} = 0.5
    _mission_started::Bool = false
    _mission_start_time_s::Float64 = NaN
    _last_process_us::UInt64 = typemax(UInt64)
end

"""Return the current fault state for an `EventScenario`."""
scenario_faults(s::EventScenario, t::Float64, x::RigidBodyState, ctx::ScenarioContext) =
    s.state.faults

"""Return the current wind disturbance request for an `EventScenario`."""
scenario_wind_dist(s::EventScenario, t::Float64, x::RigidBodyState, ctx::ScenarioContext) =
    s.state.wind_dist_ned

@inline function _set_cmd(
    st::EventScenarioState;
    armed::Bool = st.cmd.armed,
    request_mission::Bool = st.cmd.request_mission,
    request_rtl::Bool = st.cmd.request_rtl,
)
    cmd = AutopilotCommand(
        armed = armed,
        request_mission = request_mission,
        request_rtl = request_rtl,
    )
    return EventScenarioState(
        cmd = cmd,
        faults = st.faults,
        wind_dist_ned = st.wind_dist_ned,
    )
end

@inline _set_faults(st::EventScenarioState, faults::FaultState) =
    EventScenarioState(cmd = st.cmd, faults = faults, wind_dist_ned = st.wind_dist_ned)

@inline _set_wind(st::EventScenarioState, wind_dist_ned::Vec3) =
    EventScenarioState(cmd = st.cmd, faults = st.faults, wind_dist_ned = wind_dist_ned)

function scenario_step(
    s::EventScenario,
    t::Float64,
    x::RigidBodyState,
    ctx::ScenarioContext,
)
    # Apply any pending events (pure state update).
    process_events!(s, ctx, t)

    landed =
        (x.pos_ned[3] >= -s.land_z_thresh_m) && (abs(x.vel_ned[3]) < s.land_vz_thresh_mps)
    if !isnothing(s.takeoff_override_delay_s)
        if s.state.cmd.armed &&
           s._mission_started &&
           t >= (s._mission_start_time_s + s.takeoff_override_delay_s)
            landed = false
        end
    end

    return s.state.cmd, landed
end

function process_events!(s::EventScenario, ctx::ScenarioContext, t::Float64)
    now_us = ctx.t_us
    if now_us == s._last_process_us
        return nothing
    end
    s._last_process_us = now_us

    # Thread a caller-owned state value through the scheduler.
    s.state = step_events!(s.scheduler, s.state, ctx, t)

    # Latch mission start to support takeoff override even when mission request is pulsed.
    if s.state.cmd.request_mission && !s._mission_started
        s._mission_started = true
        s._mission_start_time_s = t
    end
    return nothing
end

@inline function next_event_us(
    s::EventScenario,
    ctx::ScenarioContext,
)::Union{UInt64,Nothing}
    # Only `AtTime` events are treated as true time boundaries. `When` is evaluated when
    # `process_events!` is called at event boundaries.
    return next_at_time_us(s.scheduler, ctx.t_us)
end

############################
# Convenience helpers
############################

"""Arm at time `t_arm`."""
function arm_at!(s::EventScenario, t_arm::Float64)
    push!(s.scheduler, AtTime(t_arm, (st, ctx, t)->_set_cmd(st; armed = true)))
    return s
end

"""Disarm at time `t_disarm`."""
function disarm_at!(s::EventScenario, t_disarm::Float64)
    push!(s.scheduler, AtTime(t_disarm, (st, ctx, t)->_set_cmd(st; armed = false)))
    return s
end

"""Set mission request high at time `t_start` (latched until cleared)."""
function mission_start_at!(s::EventScenario, t_start::Float64)
    push!(s.scheduler, AtTime(t_start, (st, ctx, t)->_set_cmd(st; request_mission = true)))
    return s
end

"""Pulse mission request for `pulse_s` seconds starting at `t_start`."""
function mission_pulse_at!(s::EventScenario, t_start::Float64; pulse_s::Float64 = 0.2)
    push!(s.scheduler, AtTime(t_start, (st, ctx, t)->_set_cmd(st; request_mission = true)))
    push!(
        s.scheduler,
        AtTime(
            t_start + max(0.0, pulse_s),
            (st, ctx, t)->_set_cmd(st; request_mission = false),
        ),
    )
    return s
end

"""Pulse RTL request for `pulse_s` seconds starting at `t_start`."""
function rtl_pulse_at!(s::EventScenario, t_start::Float64; pulse_s::Float64 = 0.2)
    push!(s.scheduler, AtTime(t_start, (st, ctx, t)->_set_cmd(st; request_rtl = true)))
    push!(
        s.scheduler,
        AtTime(
            t_start + max(0.0, pulse_s),
            (st, ctx, t)->_set_cmd(st; request_rtl = false),
        ),
    )
    return s
end

"""Inject a step gust (additive wind disturbance) for `duration_s` seconds at time `t_start`.

This publishes into the runtime bus as `wind_dist_ned`.
"""
function wind_step_at!(
    s::EventScenario,
    t_start::Float64,
    dv_ned::Vec3;
    duration_s::Float64 = 2.0,
)
    push!(s.scheduler, AtTime(t_start, (st, ctx, t)->_set_wind(st, dv_ned)))
    push!(
        s.scheduler,
        AtTime(
            t_start + max(0.0, duration_s),
            (st, ctx, t)->_set_wind(st, vec3(0.0, 0.0, 0.0)),
        ),
    )
    return s
end

"""Disable motor `motor_index` at time `t_start`.

This updates a bus-level `FaultState` rather than mutating component objects.
Plant dynamics interpret this as "effective duty forced to 0".
"""
function fail_motor_at!(s::EventScenario, t_start::Float64, motor_index::Int)
    push!(
        s.scheduler,
        AtTime(
            t_start,
            (st, ctx, t)->_set_faults(st, disable_motor(st.faults, motor_index)),
        ),
    )
    return s
end

"""Re-enable motor `motor_index` at time `t_start`."""
function restore_motor_at!(s::EventScenario, t_start::Float64, motor_index::Int)
    push!(
        s.scheduler,
        AtTime(
            t_start,
            (st, ctx, t)->_set_faults(st, enable_motor(st.faults, motor_index)),
        ),
    )
    return s
end

"""Disconnect the battery (bus voltage forced to 0) at time `t_start`."""
function battery_disconnect_at!(s::EventScenario, t_start::Float64)
    push!(
        s.scheduler,
        AtTime(
            t_start,
            (st, ctx, t)->_set_faults(st, set_battery_connected(st.faults, false)),
        ),
    )
    return s
end

"""Reconnect the battery at time `t_start`."""
function battery_reconnect_at!(s::EventScenario, t_start::Float64)
    push!(
        s.scheduler,
        AtTime(
            t_start,
            (st, ctx, t)->_set_faults(st, set_battery_connected(st.faults, true)),
        ),
    )
    return s
end

"""Add sensor fault bits at time `t_start`.

This publishes into `faults.sensor_fault_mask`. Specific interpretation is
component-dependent (estimator sources may freeze output, drop sensors, etc.).
"""
function sensor_fail_at!(s::EventScenario, t_start::Float64, mask::UInt64)
    push!(
        s.scheduler,
        AtTime(t_start, (st, ctx, t)->_set_faults(st, add_sensor_faults(st.faults, mask))),
    )
    return s
end

"""Clear sensor fault bits at time `t_start`."""
function sensor_restore_at!(s::EventScenario, t_start::Float64, mask::UInt64)
    push!(
        s.scheduler,
        AtTime(
            t_start,
            (st, ctx, t)->_set_faults(st, clear_sensor_faults(st.faults, mask)),
        ),
    )
    return s
end

"""Trigger `action(state, ctx, t)` when SOC (battery remaining) drops below `threshold`.

The `action` must be a **pure boundary update** returning an updated
`EventScenarioState`.

Example (request RTL when SOC <= 12%):

```julia
when_soc_below!(scenario, 0.12) do st, ctx, t
    EventScenarioState(
        cmd = AutopilotCommand(
            armed = st.cmd.armed,
            request_mission = st.cmd.request_mission,
            request_rtl = true,
        ),
        faults = st.faults,
        wind_dist_ned = st.wind_dist_ned,
    )
end
```
"""
function when_soc_below!(s::EventScenario, threshold::Float64, action::Function)
    push!(
        s.scheduler,
        When(
            (st, ctx, t)->(ctx.plant.batt_soc <= threshold),
            (st, ctx, t)->action(st, ctx, t),
        ),
    )
    return s
end

end # module Scenario
