"""PX4Lockstep.Sim.Autopilots

Autopilot interfaces.

Right now, the primary autopilot in the loop is PX4 running through your lockstep shared
library.

The design goal is:

* Keep the PX4 bridge thin and deterministic.
* Keep all "simulation truth" on the Julia side.
* Inject only the uORB-equivalent state PX4 expects (local position, attitude, body rates,
  GPS-like global position, battery status, landed status, etc.).

This allows swapping PX4 in/out without rewriting the sim.
"""
module Autopilots

using ..Types: Vec3, Quat, vec3, yaw_from_quat
using ..Powertrain: BatteryStatus

# C ABI wrapper lives in the top-level `PX4Lockstep` module.
using PX4Lockstep: LockstepHandle, LockstepInputs, LockstepOutputs
using PX4Lockstep: create, destroy, load_mission, step!

export HomeLocation,
    AutopilotCommand,
    AbstractAutopilot,
    PX4LockstepAutopilot,
    init!,
    close!,
    load_mission!,
    autopilot_step

const EARTH_RADIUS_M = 6.378137e6

"""Home location used to convert local NED to lat/lon/alt.

We use a spherical Earth approximation for now. This is more than adequate for the local
missions/trajectories we care about in early SITL.
"""
Base.@kwdef struct HomeLocation
    lat_deg::Float64 = 47.397742
    lon_deg::Float64 = 8.545594
    alt_msl_m::Float64 = 488.0
end

"""High-level commands into the autopilot.

These are intended to remain *small* — most behavior is triggered from PX4 internal logic.
"""
Base.@kwdef struct AutopilotCommand
    armed::Bool = false
    request_mission::Bool = false
    request_rtl::Bool = false
end

abstract type AbstractAutopilot end

"""PX4 autopilot driven through the lockstep shared library."""
mutable struct PX4LockstepAutopilot <: AbstractAutopilot
    handle::LockstepHandle
    home::HomeLocation
    edge_trigger::Bool
    last_cmd::AutopilotCommand
end

"""Create and initialize the PX4 lockstep autopilot."""
function init!(;
    config = nothing,
    libpath = nothing,
    home::HomeLocation = HomeLocation(),
    edge_trigger::Bool = false,
)
    h = isnothing(config) ? create(; libpath = libpath) : create(config; libpath = libpath)
    return PX4LockstepAutopilot(h, home, edge_trigger, AutopilotCommand())
end

"""Destroy the lockstep handle."""
function close!(ap::PX4LockstepAutopilot)
    destroy(ap.handle)
    return nothing
end

"""Load a QGroundControl mission file into PX4."""
function load_mission!(ap::PX4LockstepAutopilot, path::AbstractString)
    rc = load_mission(ap.handle, path)
    rc == 0 || @warn "Mission load failed" rc=rc path=path
    return rc
end

@inline function _ned_to_lla(pos_ned::Vec3, home::HomeLocation)
    x, y, z = pos_ned
    dlat = x / EARTH_RADIUS_M
    dlon = y / (EARTH_RADIUS_M * cosd(home.lat_deg))
    lat = home.lat_deg + rad2deg(dlat)
    lon = home.lon_deg + rad2deg(dlon)
    alt = home.alt_msl_m - z
    return lat, lon, alt
end

"""Step PX4 once.

Inputs:
* `t`         : simulation time (s)
* `state`     : rigid body state (pos/vel/quaternion/body rates)
* `cmd`       : high-level autopilot command (arm/mission/rtl)
* `landed`    : landed flag (best-effort; affects arming + some logic)
* `battery`   : battery status injected into PX4

Returns:
* `LockstepOutputs` from the PX4 lockstep library
"""
function autopilot_step(
    ap::PX4LockstepAutopilot,
    t::Float64,
    state_pos_ned::Vec3,
    state_vel_ned::Vec3,
    q_bn::Quat,
    ω_body::Vec3,
    cmd::AutopilotCommand;
    landed::Bool = false,
    battery::BatteryStatus = BatteryStatus(),
)::LockstepOutputs

    time_us = UInt64(round(Int, t * 1e6))
    yaw = yaw_from_quat(q_bn)
    lat, lon, alt = _ned_to_lla(state_pos_ned, ap.home)

    req_mission =
        ap.edge_trigger ? (cmd.request_mission && !ap.last_cmd.request_mission) :
        cmd.request_mission
    req_rtl =
        ap.edge_trigger ? (cmd.request_rtl && !ap.last_cmd.request_rtl) : cmd.request_rtl

    inputs = LockstepInputs(
        time_us = time_us,
        armed = cmd.armed ? 1 : 0,
        nav_auto_mission = req_mission ? 1 : 0,
        nav_auto_rtl = req_rtl ? 1 : 0,
        landed = landed ? 1 : 0,
        x = Float32(state_pos_ned[1]),
        y = Float32(state_pos_ned[2]),
        z = Float32(state_pos_ned[3]),
        vx = Float32(state_vel_ned[1]),
        vy = Float32(state_vel_ned[2]),
        vz = Float32(state_vel_ned[3]),
        yaw = Float32(yaw),
        lat_deg = lat,
        lon_deg = lon,
        alt_msl_m = Float32(alt),
        q = (Float32(q_bn[1]), Float32(q_bn[2]), Float32(q_bn[3]), Float32(q_bn[4])),
        rates_xyz = (Float32(ω_body[1]), Float32(ω_body[2]), Float32(ω_body[3])),
        battery_connected = battery.connected ? 1 : 0,
        battery_voltage_v = Float32(battery.voltage_v),
        battery_current_a = Float32(battery.current_a),
        battery_remaining = Float32(battery.remaining),
        battery_warning = battery.warning,
    )

    out = step!(ap.handle, inputs)
    ap.last_cmd = cmd
    return out
end

end # module Autopilots
