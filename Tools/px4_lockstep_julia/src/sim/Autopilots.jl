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

export HomeLocation,
    WorldOrigin,
    AutopilotCommand,
    AbstractAutopilot,
    UORBOutputs,
    PX4UORBInterfaceConfig,
    UORBPubSpec,
    UORBSubSpec,
    PX4LockstepAutopilot,
    # uORB injection scheduling helpers
    PX4StepContext,
    AbstractUORBInjectionSource,
    PeriodicUORBInjection,
    HomePositionUORBInjection,
    PX4UORBInjector,
    add_injection_source!,
    injection_periods_us,
    recommended_step_dt_us,
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
include("Autopilots/UORBInjection.jl")

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
    mission_rearm_pending::Bool
    mission_rearm_until_us::UInt64
    uorb::UORBBridge
    uorb_outputs::UORBOutputs
    injector::PX4UORBInjector
end

# -----------------
# Injection helpers
# -----------------

"""Return the configured uORB injection periods in microseconds.

For autopilots that do not use uORB injection scheduling, this returns an empty vector.
"""
injection_periods_us(::AbstractAutopilot) = UInt64[]

"""Return the recommended PX4 step dt in microseconds from the injection schedule.

If no periodic injection sources are configured (or the autopilot does not expose
an injection schedule), returns `nothing`.
"""
recommended_step_dt_us(::AbstractAutopilot) = nothing

injection_periods_us(ap::PX4LockstepAutopilot) = injection_periods_us(ap.injector)
recommended_step_dt_us(ap::PX4LockstepAutopilot) = recommended_step_dt_us(ap.injector)

"""Add an injection source to a running PX4 lockstep autopilot."""
function add_injection_source!(ap::PX4LockstepAutopilot, src::AbstractUORBInjectionSource)
    add_source!(ap.injector, src)
    return ap
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

`uorb_cfg` is required and must be provided by the caller (typically from TOML).
"""
function init!(;
    config = nothing,
    libpath = nothing,
    home::HomeLocation = HomeLocation(),
    edge_trigger::Bool = false,
    allow_multiple_handles::Bool = false,
    uorb_cfg::PX4UORBInterfaceConfig,
)
    h =
        isnothing(config) ?
        create(; libpath = libpath, allow_multiple_handles = allow_multiple_handles) :
        create(config; libpath = libpath, allow_multiple_handles = allow_multiple_handles)
    uorb = _init_uorb_bridge(h, uorb_cfg)
    uorb_outputs = UORBOutputs()

    # Default injector publishes the state-injection topics that are configured
    # as publishers in `uorb_cfg`.
    injector = build_state_injection_injector(uorb, home)

    return PX4LockstepAutopilot(
        h,
        home,
        edge_trigger,
        AutopilotCommand(),
        false,
        UInt64(0),
        uorb,
        uorb_outputs,
        injector,
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
    # Full battery vector (deterministic order).
    batteries::Vector{BatteryStatus} = BatteryStatus[battery],
)

    yaw = yaw_from_quat(q_bn)
    lat, lon, alt = _ned_to_lla(state_pos_ned, ap.home)

    req_mission =
        ap.edge_trigger ? (cmd.request_mission && !ap.last_cmd.request_mission) :
        cmd.request_mission
    req_rtl =
        ap.edge_trigger ? (cmd.request_rtl && !ap.last_cmd.request_rtl) : cmd.request_rtl

    if !ap.edge_trigger
        if cmd.request_mission
            mission_valid = ap.uorb_outputs.mission_valid != 0
            if !mission_valid
                ap.mission_rearm_pending = true
            elseif ap.mission_rearm_pending
                # Hold manual for at least one navigator tick so activation re-triggers.
                nav_rate = ap.handle.config.navigator_rate_hz
                hold_us = nav_rate > 0 ? UInt64(cld(1_000_000, nav_rate)) : UInt64(1)
                ap.mission_rearm_until_us = time_us + hold_us
                ap.mission_rearm_pending = false
            end
            if ap.mission_rearm_until_us != 0
                if time_us < ap.mission_rearm_until_us
                    req_mission = false
                else
                    ap.mission_rearm_until_us = 0
                end
            end
        else
            ap.mission_rearm_pending = false
            ap.mission_rearm_until_us = 0
        end
    end

    auto_mode = req_mission || req_rtl
    nav_state =
        req_rtl ? NAV_STATE_AUTO_RTL :
        req_mission ? NAV_STATE_AUTO_MISSION : NAV_STATE_MANUAL
    arming_state = cmd.armed ? ARMING_STATE_ARMED : ARMING_STATE_DISARMED

    use_home = ap.handle.config.enable_commander == 0
    ref_lat = use_home ? ap.home.lat_deg : lat
    ref_lon = use_home ? ap.home.lon_deg : lon
    ref_alt = use_home ? ap.home.alt_msl_m : alt

    ctx = PX4StepContext(
        time_us = time_us,
        pos_ned = state_pos_ned,
        vel_ned = state_vel_ned,
        q_bn = q_bn,
        ω_body = ω_body,
        cmd = cmd,
        landed = landed,
        battery = battery,
        batteries = batteries,
        yaw_rad = yaw,
        lat_deg = lat,
        lon_deg = lon,
        alt_msl_m = alt,
        ref_lat_deg = ref_lat,
        ref_lon_deg = ref_lon,
        ref_alt_m = ref_alt,
        auto_mode = auto_mode,
        nav_state = nav_state,
        arming_state = arming_state,
        control_allocator_enabled = ap.handle.config.enable_control_allocator != 0,
    )

    # Queue any uORB publishes that are due at this time.
    inject_due!(ap.injector, ap.handle, ctx)

    step_uorb!(ap.handle, time_us)
    ap.last_cmd = cmd
    _update_uorb_outputs!(ap)
    return ap.uorb_outputs
end

end # module Autopilots
