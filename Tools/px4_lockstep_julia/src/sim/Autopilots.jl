"""PX4Lockstep.Sim.Autopilots

Autopilot interfaces.

The primary autopilot in the loop is PX4 running through the lockstep shared library.

Design goals:

* Keep the PX4 bridge thin and deterministic.
* Keep all "simulation truth" on the Julia side.
* Inject only the uORB-equivalent state PX4 expects (local position, attitude, body rates,
  GPS-like global position, battery status, landed status, etc.).

This allows swapping PX4 in/out without rewriting the sim.
"""
module Autopilots

using ..Types: Vec3, Quat, WorldOrigin, vec3, yaw_from_quat
using ..Powertrain: BatteryStatus

# C ABI wrapper lives in the top-level `PX4Lockstep` module.
using PX4Lockstep: LockstepHandle
using PX4Lockstep: create, destroy, load_mission, step_uorb!
using PX4Lockstep: UORBPublisher, UORBSubscriber, UORBMsg
using PX4Lockstep: create_publisher, create_subscriber, publish!
using PX4Lockstep: uorb_check, uorb_copy, uorb_unsubscribe!
using PX4Lockstep: BatteryStatusMsg, VehicleAttitudeMsg, VehicleLocalPositionMsg
using PX4Lockstep:
    VehicleGlobalPositionMsg, VehicleAngularVelocityMsg, VehicleLandDetectedMsg
using PX4Lockstep: VehicleStatusMsg, VehicleControlModeMsg, ActuatorArmedMsg
using PX4Lockstep: HomePositionMsg, GeofenceStatusMsg
using PX4Lockstep: VehicleTorqueSetpointMsg, VehicleThrustSetpointMsg
using PX4Lockstep: ActuatorMotorsMsg, ActuatorServosMsg
using PX4Lockstep: VehicleAttitudeSetpointMsg, VehicleRatesSetpointMsg
using PX4Lockstep: MissionResultMsg, TrajectorySetpointMsg

export HomeLocation,
    WorldOrigin,
    AutopilotCommand,
    AbstractAutopilot,
    UORBOutputs,
    PX4UORBInterfaceConfig,
    UORBPubSpec,
    UORBSubSpec,
    iris_state_injection_interface,
    minimal_actuator_only_interface,
    PX4LockstepAutopilot,
    autopilot_output_type,
    max_internal_rate_hz,
    init!,
    close!,
    load_mission!,
    autopilot_step

const EARTH_RADIUS_M = 6.378137e6

"""Home location used to convert local NED to lat/lon/alt.

Alias for `WorldOrigin` so the simulation and PX4 share a single origin definition.
The conversion uses a spherical Earth approximation (adequate for local missions).
"""
const HomeLocation = WorldOrigin

"""High-level commands into the autopilot.

These are intended to remain *small* — most behavior is triggered from PX4 internal logic.
"""
Base.@kwdef struct AutopilotCommand
    armed::Bool = false
    request_mission::Bool = false
    request_rtl::Bool = false
end

abstract type AbstractAutopilot end

include("Autopilots/UORBBridge.jl")

"""Return the fastest internal control/navigation loop rate (Hz) for an autopilot.

For PX4 lockstep, this is derived from the lockstep config used to build the library.
If `nothing` is returned, the simulator will not issue dt/rate warnings.
"""
max_internal_rate_hz(::AbstractAutopilot) = nothing

"""Return the concrete output type produced by `autopilot_step`.

The simulator uses this to keep the sample-and-hold path type-stable.

For custom autopilots used in tests or prototyping, define:

    autopilot_output_type(::MyAutopilot) = MyOutputType
"""
autopilot_output_type(::AbstractAutopilot) = Any

"""PX4 autopilot driven through the lockstep shared library."""
mutable struct PX4LockstepAutopilot <: AbstractAutopilot
    handle::LockstepHandle
    home::HomeLocation
    edge_trigger::Bool
    last_cmd::AutopilotCommand
    uorb::UORBBridge
    uorb_outputs::UORBOutputs
    home_update_count::UInt32
end

autopilot_output_type(::PX4LockstepAutopilot) = UORBOutputs

function max_internal_rate_hz(ap::PX4LockstepAutopilot)
    cfg = ap.handle.config
    max_hz = Int32(0)
    # Only consider enabled modules. Rate <= 0 means "every tick", so it does not
    # constrain dt_autopilot.
    if cfg.enable_commander != 0 && cfg.commander_rate_hz > 0
        max_hz = max(max_hz, cfg.commander_rate_hz)
    end
    if cfg.navigator_rate_hz > 0
        max_hz = max(max_hz, cfg.navigator_rate_hz)
    end
    if cfg.mc_pos_control_rate_hz > 0
        max_hz = max(max_hz, cfg.mc_pos_control_rate_hz)
    end
    if cfg.mc_att_control_rate_hz > 0
        max_hz = max(max_hz, cfg.mc_att_control_rate_hz)
    end
    if cfg.mc_rate_control_rate_hz > 0
        max_hz = max(max_hz, cfg.mc_rate_control_rate_hz)
    end
    if cfg.enable_control_allocator != 0 && cfg.control_allocator_rate_hz > 0
        max_hz = max(max_hz, cfg.control_allocator_rate_hz)
    end

    return max_hz > 0 ? Int(max_hz) : nothing
end

"""Create and initialize the PX4 lockstep autopilot.

Only one lockstep handle is supported per process by default. Use
`allow_multiple_handles=true` only when the PX4 lockstep runtime is known
to be re-entrant.
"""
function init!(;
    config = nothing,
    libpath = nothing,
    home::HomeLocation = HomeLocation(),
    edge_trigger::Bool = false,
    allow_multiple_handles::Bool = false,
    uorb_cfg::PX4UORBInterfaceConfig = iris_state_injection_interface(),
)
    h =
        isnothing(config) ?
        create(; libpath = libpath, allow_multiple_handles = allow_multiple_handles) :
        create(config; libpath = libpath, allow_multiple_handles = allow_multiple_handles)
    uorb = _init_uorb_bridge(h, uorb_cfg)
    uorb_outputs = UORBOutputs()
    return PX4LockstepAutopilot(
        h,
        home,
        edge_trigger,
        AutopilotCommand(),
        uorb,
        uorb_outputs,
        UInt32(0),
    )
end

"""Destroy the lockstep handle."""
function close!(ap::PX4LockstepAutopilot)
    _close_uorb_bridge!(ap.uorb)
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

@inline function _update_uorb_outputs!(ap::PX4LockstepAutopilot)
    return _update_uorb_outputs!(ap.uorb, ap.uorb_outputs)
end

"""Step PX4 once.

Inputs:
* `t`         : simulation time (s)
* `state`     : rigid body state (pos/vel/quaternion/body rates)
* `cmd`       : high-level autopilot command (arm/mission/rtl)
* `landed`    : landed flag (best-effort; affects arming + some logic)
* `battery`   : battery status injected into PX4

Returns:
* `UORBOutputs` from the PX4 uORB subscriptions
"""
function autopilot_step(
    ap::PX4LockstepAutopilot,
    time_us::UInt64,
    state_pos_ned::Vec3,
    state_vel_ned::Vec3,
    q_bn::Quat,
    ω_body::Vec3,
    cmd::AutopilotCommand;
    landed::Bool = false,
    battery::BatteryStatus = BatteryStatus(),
)

    yaw = yaw_from_quat(q_bn)
    lat, lon, alt = _ned_to_lla(state_pos_ned, ap.home)

    req_mission =
        ap.edge_trigger ? (cmd.request_mission && !ap.last_cmd.request_mission) :
        cmd.request_mission
    req_rtl =
        ap.edge_trigger ? (cmd.request_rtl && !ap.last_cmd.request_rtl) : cmd.request_rtl
    auto_mode = req_mission || req_rtl
    nav_state =
        req_rtl ? NAV_STATE_AUTO_RTL :
        req_mission ? NAV_STATE_AUTO_MISSION : NAV_STATE_MANUAL
    bridge = ap.uorb
    arming_state = cmd.armed ? ARMING_STATE_ARMED : ARMING_STATE_DISARMED
    use_home = ap.handle.config.enable_commander == 0
    ref_lat = use_home ? ap.home.lat_deg : lat
    ref_lon = use_home ? ap.home.lon_deg : lon
    ref_alt = use_home ? ap.home.alt_msl_m : alt

    if haskey(bridge.pubs, :battery_status)
        _publish_uorb!(bridge, :battery_status, _battery_status_msg(time_us, battery))
    end

    if haskey(bridge.pubs, :vehicle_attitude)
        _publish_uorb!(bridge, :vehicle_attitude, _vehicle_attitude_msg(time_us, q_bn))
    end

    if haskey(bridge.pubs, :vehicle_local_position)
        msg = _vehicle_local_position_msg(
            time_us,
            state_pos_ned,
            state_vel_ned,
            yaw,
            ref_lat,
            ref_lon,
            ref_alt,
        )
        _publish_uorb!(bridge, :vehicle_local_position, msg)
    end

    if haskey(bridge.pubs, :vehicle_global_position)
        _publish_uorb!(
            bridge,
            :vehicle_global_position,
            _vehicle_global_position_msg(time_us, lat, lon, alt),
        )
    end

    if haskey(bridge.pubs, :vehicle_angular_velocity)
        _publish_uorb!(
            bridge,
            :vehicle_angular_velocity,
            _vehicle_angular_velocity_msg(time_us, ω_body),
        )
    end

    if haskey(bridge.pubs, :vehicle_land_detected)
        _publish_uorb!(
            bridge,
            :vehicle_land_detected,
            _vehicle_land_detected_msg(time_us, landed),
        )
    end

    if haskey(bridge.pubs, :vehicle_status)
        _publish_uorb!(
            bridge,
            :vehicle_status,
            _vehicle_status_msg(time_us, nav_state, arming_state),
        )
    end

    if haskey(bridge.pubs, :vehicle_control_mode)
        _publish_uorb!(
            bridge,
            :vehicle_control_mode,
            _vehicle_control_mode_msg(
                time_us,
                cmd,
                auto_mode,
                nav_state,
                ap.handle.config.enable_control_allocator != 0,
            ),
        )
    end

    if haskey(bridge.pubs, :actuator_armed)
        _publish_uorb!(bridge, :actuator_armed, _actuator_armed_msg(time_us, cmd))
    end

    if haskey(bridge.pubs, :geofence_status)
        _publish_uorb!(bridge, :geofence_status, _geofence_status_msg(time_us))
    end

    if haskey(bridge.pubs, :home_position)
        has_home =
            isfinite(ap.home.lat_deg) &&
            isfinite(ap.home.lon_deg) &&
            isfinite(ap.home.alt_msl_m)
        if has_home
            ap.home_update_count += UInt32(1)
            _publish_uorb!(
                bridge,
                :home_position,
                _home_position_msg(
                    time_us,
                    ap.home.lat_deg,
                    ap.home.lon_deg,
                    ap.home.alt_msl_m,
                    ap.home_update_count,
                ),
            )
        end
    end

    step_uorb!(ap.handle, time_us)
    ap.last_cmd = cmd
    _update_uorb_outputs!(ap)
    return ap.uorb_outputs
end

end # module Autopilots
