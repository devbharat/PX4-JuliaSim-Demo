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
using PX4Lockstep: LockstepHandle, LockstepInputs, LockstepOutputs
using PX4Lockstep: create, destroy, load_mission, step!, step_uorb!
using PX4Lockstep: UORBPublisher, UORBSubscriber
using PX4Lockstep: create_uorb_publisher_checked, create_uorb_subscriber
using PX4Lockstep: queue_uorb_publish!, uorb_check, uorb_copy!, uorb_unsubscribe!
using PX4Lockstep: BatteryStatusMsg, VehicleAttitudeMsg, VehicleLocalPositionMsg
using PX4Lockstep: VehicleGlobalPositionMsg, VehicleAngularVelocityMsg, VehicleLandDetectedMsg
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
    PX4LockstepAutopilot,
    autopilot_output_type,
    max_internal_rate_hz,
    init!,
    close!,
    load_mission!,
    autopilot_step

const EARTH_RADIUS_M = 6.378137e6
const UORB_BATTERY_ENV = "PX4_LOCKSTEP_UORB_BATTERY"
const UORB_ATTITUDE_ENV = "PX4_LOCKSTEP_UORB_ATTITUDE"
const UORB_LOCAL_POSITION_ENV = "PX4_LOCKSTEP_UORB_LOCAL_POSITION"
const UORB_GLOBAL_POSITION_ENV = "PX4_LOCKSTEP_UORB_GLOBAL_POSITION"
const UORB_RATES_ENV = "PX4_LOCKSTEP_UORB_RATES"
const UORB_LAND_ENV = "PX4_LOCKSTEP_UORB_LAND_DETECTED"
const UORB_VEHICLE_STATUS_ENV = "PX4_LOCKSTEP_UORB_VEHICLE_STATUS"
const UORB_CONTROL_MODE_ENV = "PX4_LOCKSTEP_UORB_VEHICLE_CONTROL_MODE"
const UORB_ACTUATOR_ARMED_ENV = "PX4_LOCKSTEP_UORB_ACTUATOR_ARMED"
const UORB_HOME_POSITION_ENV = "PX4_LOCKSTEP_UORB_HOME_POSITION"
const UORB_GEOFENCE_STATUS_ENV = "PX4_LOCKSTEP_UORB_GEOFENCE_STATUS"
const UORB_OUTPUTS_ENV = "PX4_LOCKSTEP_UORB_OUTPUTS"
const UORB_ONLY_ENV = "PX4_LOCKSTEP_UORB_ONLY"
const ZERO_VEC3_F32 = (0.0f0, 0.0f0, 0.0f0)
const ZERO_VEC2_F32 = (0.0f0, 0.0f0)
const ZERO_Q_F32 = (0.0f0, 0.0f0, 0.0f0, 0.0f0)
const NAN_F32 = Float32(NaN)
const ZERO_CELL_V_F32 = (
    0.0f0,
    0.0f0,
    0.0f0,
    0.0f0,
    0.0f0,
    0.0f0,
    0.0f0,
    0.0f0,
    0.0f0,
    0.0f0,
    0.0f0,
    0.0f0,
    0.0f0,
    0.0f0,
)
const ZERO_PAD_U8 = (UInt8(0), UInt8(0), UInt8(0), UInt8(0))
const ZERO_PAD_U8_1 = (UInt8(0),)
const ZERO_PAD_U8_3 = (UInt8(0), UInt8(0), UInt8(0))
const ZERO_PAD_U8_6 = (
    UInt8(0),
    UInt8(0),
    UInt8(0),
    UInt8(0),
    UInt8(0),
    UInt8(0),
)
const ZERO_PAD_U8_7 = (
    UInt8(0),
    UInt8(0),
    UInt8(0),
    UInt8(0),
    UInt8(0),
    UInt8(0),
    UInt8(0),
)
const ZERO_PAD_U8_2 = (UInt8(0), UInt8(0))
const ZERO_PAD_U8_5 = (UInt8(0), UInt8(0), UInt8(0), UInt8(0), UInt8(0))
const ZERO_CONTROLS_8 = ntuple(_ -> 0.0f0, 8)
const ZERO_CONTROLS_12 = ntuple(_ -> 0.0f0, 12)
const NAN_CONTROLS_8 = ntuple(_ -> NAN_F32, 8)
const NAN_CONTROLS_12 = ntuple(_ -> NAN_F32, 12)
const ARMING_STATE_DISARMED = UInt8(1)
const ARMING_STATE_ARMED = UInt8(2)
const NAV_STATE_MANUAL = UInt8(0)
const NAV_STATE_AUTO_MISSION = UInt8(3)
const NAV_STATE_AUTO_RTL = UInt8(5)
const VEHICLE_TYPE_ROTARY_WING = UInt8(1)

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

Base.@kwdef mutable struct UORBOutputs
    actuator_controls::NTuple{8,Float32} = ZERO_CONTROLS_8
    actuator_motors::NTuple{12,Float32} = NAN_CONTROLS_12
    actuator_servos::NTuple{8,Float32} = NAN_CONTROLS_8
    attitude_setpoint_q::NTuple{4,Float32} = ZERO_Q_F32
    rates_setpoint_xyz::NTuple{3,Float32} = ZERO_VEC3_F32
    thrust_setpoint_body::NTuple{3,Float32} = ZERO_VEC3_F32
    mission_seq::Int32 = 0
    mission_count::Int32 = 0
    mission_finished::Int32 = 0
    nav_state::Int32 = 0
    arming_state::Int32 = 0
    battery_warning::Int32 = 0
    trajectory_setpoint_position::NTuple{3,Float32} = ZERO_VEC3_F32
    trajectory_setpoint_velocity::NTuple{3,Float32} = ZERO_VEC3_F32
    trajectory_setpoint_acceleration::NTuple{3,Float32} = ZERO_VEC3_F32
    trajectory_setpoint_yaw::Float32 = 0.0f0
    trajectory_setpoint_yawspeed::Float32 = 0.0f0
end

struct UORBOutputSubscriptions
    torque_sp::UORBSubscriber
    thrust_sp::UORBSubscriber
    actuator_motors::UORBSubscriber
    actuator_servos::UORBSubscriber
    attitude_sp::UORBSubscriber
    rates_sp::UORBSubscriber
    mission_result::UORBSubscriber
    vehicle_status::UORBSubscriber
    battery_status::UORBSubscriber
    trajectory_setpoint::UORBSubscriber
end

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
    uorb_battery_pub::Union{Nothing,UORBPublisher}
    uorb_attitude_pub::Union{Nothing,UORBPublisher}
    uorb_local_position_pub::Union{Nothing,UORBPublisher}
    uorb_global_position_pub::Union{Nothing,UORBPublisher}
    uorb_rates_pub::Union{Nothing,UORBPublisher}
    uorb_land_pub::Union{Nothing,UORBPublisher}
    uorb_vehicle_status_pub::Union{Nothing,UORBPublisher}
    uorb_control_mode_pub::Union{Nothing,UORBPublisher}
    uorb_actuator_armed_pub::Union{Nothing,UORBPublisher}
    uorb_home_position_pub::Union{Nothing,UORBPublisher}
    uorb_geofence_status_pub::Union{Nothing,UORBPublisher}
    uorb_output_subs::Union{Nothing,UORBOutputSubscriptions}
    uorb_outputs::UORBOutputs
    uorb_only::Bool
    home_update_count::UInt32
end

autopilot_output_type(ap::PX4LockstepAutopilot) =
    (ap.uorb_only || ap.uorb_output_subs !== nothing) ? UORBOutputs : LockstepOutputs

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
)
    h =
        isnothing(config) ?
        create(; libpath = libpath, allow_multiple_handles = allow_multiple_handles) :
        create(config; libpath = libpath, allow_multiple_handles = allow_multiple_handles)
    uorb_battery_pub = nothing
    if _env_flag_enabled(UORB_BATTERY_ENV)
        uorb_battery_pub, _ =
            create_uorb_publisher_checked(h, "battery_status", BatteryStatusMsg)
    end
    uorb_attitude_pub = nothing
    if _env_flag_enabled(UORB_ATTITUDE_ENV)
        uorb_attitude_pub, _ =
            create_uorb_publisher_checked(h, "vehicle_attitude", VehicleAttitudeMsg)
    end
    uorb_local_position_pub = nothing
    if _env_flag_enabled(UORB_LOCAL_POSITION_ENV)
        uorb_local_position_pub, _ = create_uorb_publisher_checked(
            h,
            "vehicle_local_position",
            VehicleLocalPositionMsg,
        )
    end
    uorb_global_position_pub = nothing
    if _env_flag_enabled(UORB_GLOBAL_POSITION_ENV)
        uorb_global_position_pub, _ = create_uorb_publisher_checked(
            h,
            "vehicle_global_position",
            VehicleGlobalPositionMsg,
        )
    end
    uorb_rates_pub = nothing
    if _env_flag_enabled(UORB_RATES_ENV)
        uorb_rates_pub, _ = create_uorb_publisher_checked(
            h,
            "vehicle_angular_velocity",
            VehicleAngularVelocityMsg,
        )
    end
    uorb_land_pub = nothing
    if _env_flag_enabled(UORB_LAND_ENV)
        uorb_land_pub, _ = create_uorb_publisher_checked(
            h,
            "vehicle_land_detected",
            VehicleLandDetectedMsg,
        )
    end
    uorb_vehicle_status_pub = nothing
    if _env_flag_enabled(UORB_VEHICLE_STATUS_ENV)
        uorb_vehicle_status_pub, _ = create_uorb_publisher_checked(
            h,
            "vehicle_status",
            VehicleStatusMsg,
        )
    end
    uorb_control_mode_pub = nothing
    if _env_flag_enabled(UORB_CONTROL_MODE_ENV)
        uorb_control_mode_pub, _ = create_uorb_publisher_checked(
            h,
            "vehicle_control_mode",
            VehicleControlModeMsg,
        )
    end
    uorb_actuator_armed_pub = nothing
    if _env_flag_enabled(UORB_ACTUATOR_ARMED_ENV)
        uorb_actuator_armed_pub, _ = create_uorb_publisher_checked(
            h,
            "actuator_armed",
            ActuatorArmedMsg,
        )
    end
    uorb_home_position_pub = nothing
    if _env_flag_enabled(UORB_HOME_POSITION_ENV)
        uorb_home_position_pub, _ = create_uorb_publisher_checked(
            h,
            "home_position",
            HomePositionMsg,
        )
    end
    uorb_geofence_status_pub = nothing
    if _env_flag_enabled(UORB_GEOFENCE_STATUS_ENV)
        uorb_geofence_status_pub, _ = create_uorb_publisher_checked(
            h,
            "geofence_status",
            GeofenceStatusMsg,
        )
    end
    uorb_only = _env_flag_enabled(UORB_ONLY_ENV)
    uorb_output_subs = nothing
    if _env_flag_enabled(UORB_OUTPUTS_ENV)
        uorb_output_subs = UORBOutputSubscriptions(
            create_uorb_subscriber(h, "vehicle_torque_setpoint"),
            create_uorb_subscriber(h, "vehicle_thrust_setpoint"),
            create_uorb_subscriber(h, "actuator_motors"),
            create_uorb_subscriber(h, "actuator_servos"),
            create_uorb_subscriber(h, "vehicle_attitude_setpoint"),
            create_uorb_subscriber(h, "vehicle_rates_setpoint"),
            create_uorb_subscriber(h, "mission_result"),
            create_uorb_subscriber(h, "vehicle_status"),
            create_uorb_subscriber(h, "battery_status"),
            create_uorb_subscriber(h, "trajectory_setpoint"),
        )
    end
    if uorb_only && uorb_output_subs === nothing
        error("uORB-only mode requires PX4_LOCKSTEP_UORB_OUTPUTS=1")
    end
    uorb_outputs = UORBOutputs()
    return PX4LockstepAutopilot(
        h,
        home,
        edge_trigger,
        AutopilotCommand(),
        uorb_battery_pub,
        uorb_attitude_pub,
        uorb_local_position_pub,
        uorb_global_position_pub,
        uorb_rates_pub,
        uorb_land_pub,
        uorb_vehicle_status_pub,
        uorb_control_mode_pub,
        uorb_actuator_armed_pub,
        uorb_home_position_pub,
        uorb_geofence_status_pub,
        uorb_output_subs,
        uorb_outputs,
        uorb_only,
        UInt32(0),
    )
end

"""Destroy the lockstep handle."""
function close!(ap::PX4LockstepAutopilot)
    if ap.uorb_output_subs !== nothing
        subs = ap.uorb_output_subs
        uorb_unsubscribe!(ap.handle, subs.torque_sp)
        uorb_unsubscribe!(ap.handle, subs.thrust_sp)
        uorb_unsubscribe!(ap.handle, subs.actuator_motors)
        uorb_unsubscribe!(ap.handle, subs.actuator_servos)
        uorb_unsubscribe!(ap.handle, subs.attitude_sp)
        uorb_unsubscribe!(ap.handle, subs.rates_sp)
        uorb_unsubscribe!(ap.handle, subs.mission_result)
        uorb_unsubscribe!(ap.handle, subs.vehicle_status)
        uorb_unsubscribe!(ap.handle, subs.battery_status)
        uorb_unsubscribe!(ap.handle, subs.trajectory_setpoint)
    end
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

@inline function _update_controls_torque(
    controls::NTuple{8,Float32},
    torque::VehicleTorqueSetpointMsg,
)
    return (
        torque.xyz[1],
        torque.xyz[2],
        torque.xyz[3],
        controls[4],
        controls[5],
        controls[6],
        controls[7],
        controls[8],
    )
end

@inline function _update_controls_thrust(
    controls::NTuple{8,Float32},
    thrust::VehicleThrustSetpointMsg,
)
    return (
        controls[1],
        controls[2],
        controls[3],
        thrust.xyz[3],
        controls[5],
        controls[6],
        controls[7],
        controls[8],
    )
end

function _update_uorb_outputs!(ap::PX4LockstepAutopilot)
    subs = ap.uorb_output_subs
    subs === nothing && return ap.uorb_outputs
    out = ap.uorb_outputs

    controls = out.actuator_controls
    if uorb_check(ap.handle, subs.torque_sp)
        torque_msg = Ref{VehicleTorqueSetpointMsg}()
        uorb_copy!(ap.handle, subs.torque_sp, torque_msg)
        controls = _update_controls_torque(controls, torque_msg[])
    end
    if uorb_check(ap.handle, subs.thrust_sp)
        thrust_msg = Ref{VehicleThrustSetpointMsg}()
        uorb_copy!(ap.handle, subs.thrust_sp, thrust_msg)
        controls = _update_controls_thrust(controls, thrust_msg[])
    end
    out.actuator_controls = controls

    if uorb_check(ap.handle, subs.actuator_motors)
        motors_msg = Ref{ActuatorMotorsMsg}()
        uorb_copy!(ap.handle, subs.actuator_motors, motors_msg)
        out.actuator_motors = motors_msg[].control
    end

    if uorb_check(ap.handle, subs.actuator_servos)
        servos_msg = Ref{ActuatorServosMsg}()
        uorb_copy!(ap.handle, subs.actuator_servos, servos_msg)
        out.actuator_servos = servos_msg[].control
    end

    if uorb_check(ap.handle, subs.attitude_sp)
        att_msg = Ref{VehicleAttitudeSetpointMsg}()
        uorb_copy!(ap.handle, subs.attitude_sp, att_msg)
        out.attitude_setpoint_q = att_msg[].q_d
        out.thrust_setpoint_body = att_msg[].thrust_body
    end

    if uorb_check(ap.handle, subs.rates_sp)
        rates_msg = Ref{VehicleRatesSetpointMsg}()
        uorb_copy!(ap.handle, subs.rates_sp, rates_msg)
        out.rates_setpoint_xyz = (rates_msg[].roll, rates_msg[].pitch, rates_msg[].yaw)
    end

    if uorb_check(ap.handle, subs.mission_result)
        mission_msg = Ref{MissionResultMsg}()
        uorb_copy!(ap.handle, subs.mission_result, mission_msg)
        out.mission_seq = Int32(mission_msg[].seq_current)
        out.mission_count = Int32(mission_msg[].seq_total)
        out.mission_finished = mission_msg[].finished ? Int32(1) : Int32(0)
    end

    if uorb_check(ap.handle, subs.vehicle_status)
        vstatus_msg = Ref{VehicleStatusMsg}()
        uorb_copy!(ap.handle, subs.vehicle_status, vstatus_msg)
        out.nav_state = Int32(vstatus_msg[].nav_state)
        out.arming_state = Int32(vstatus_msg[].arming_state)
    end

    if uorb_check(ap.handle, subs.battery_status)
        battery_msg = Ref{BatteryStatusMsg}()
        uorb_copy!(ap.handle, subs.battery_status, battery_msg)
        out.battery_warning = Int32(battery_msg[].warning)
    end

    if uorb_check(ap.handle, subs.trajectory_setpoint)
        traj_msg = Ref{TrajectorySetpointMsg}()
        uorb_copy!(ap.handle, subs.trajectory_setpoint, traj_msg)
        out.trajectory_setpoint_position = traj_msg[].position
        out.trajectory_setpoint_velocity = traj_msg[].velocity
        out.trajectory_setpoint_acceleration = traj_msg[].acceleration
        out.trajectory_setpoint_yaw = traj_msg[].yaw
        out.trajectory_setpoint_yawspeed = traj_msg[].yawspeed
    end

    return out
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
* `UORBOutputs` when `PX4_LOCKSTEP_UORB_OUTPUTS=1` or `PX4_LOCKSTEP_UORB_ONLY=1`
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
    req_rtl = ap.edge_trigger ? (cmd.request_rtl && !ap.last_cmd.request_rtl) : cmd.request_rtl
    auto_mode = req_mission || req_rtl
    nav_state = req_rtl ? NAV_STATE_AUTO_RTL : req_mission ? NAV_STATE_AUTO_MISSION : NAV_STATE_MANUAL

    inputs = nothing
    if !ap.uorb_only
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
    end

    if ap.uorb_battery_pub !== nothing
        battery_msg = BatteryStatusMsg(
            time_us,
            Float32(battery.voltage_v),
            Float32(battery.current_a),
            0.0f0,
            0.0f0,
            Float32(battery.remaining),
            0.0f0,
            0.0f0,
            0.0f0,
            ZERO_CELL_V_F32,
            0.0f0,
            0.0f0,
            0.0f0,
            0.0f0,
            0.0f0,
            0.0f0,
            0.0f0,
            0.0f0,
            0.0f0,
            0.0f0,
            0.0f0,
            UInt16(0),
            UInt16(0),
            UInt16(0),
            UInt16(0),
            UInt16(0),
            UInt16(0),
            UInt16(0),
            UInt16(0),
            UInt16(0),
            battery.connected,
            UInt8(0),
            UInt8(0),
            UInt8(0),
            UInt8(0),
            false,
            false,
            UInt8(battery.warning),
            ZERO_PAD_U8_2,
        )
        queue_uorb_publish!(ap.handle, ap.uorb_battery_pub, battery_msg)
    end

    if ap.uorb_attitude_pub !== nothing
        att_msg = VehicleAttitudeMsg(
            time_us,
            0,
            (Float32(q_bn[1]), Float32(q_bn[2]), Float32(q_bn[3]), Float32(q_bn[4])),
            ZERO_Q_F32,
            UInt8(0),
            ZERO_PAD_U8_7,
        )
        queue_uorb_publish!(ap.handle, ap.uorb_attitude_pub, att_msg)
    end

    if ap.uorb_local_position_pub !== nothing
        use_home = ap.handle.config.enable_commander == 0
        ref_lat = use_home ? ap.home.lat_deg : lat
        ref_lon = use_home ? ap.home.lon_deg : lon
        ref_alt = use_home ? ap.home.alt_msl_m : alt
        has_ref = isfinite(ref_lat) && isfinite(ref_lon) && isfinite(ref_alt)
        lpos_msg = VehicleLocalPositionMsg(
            time_us,
            time_us,
            time_us,
            ref_lat,
            ref_lon,
            Float32(state_pos_ned[1]),
            Float32(state_pos_ned[2]),
            Float32(state_pos_ned[3]),
            ZERO_VEC2_F32,
            0.0f0,
            Float32(state_vel_ned[1]),
            Float32(state_vel_ned[2]),
            Float32(state_vel_ned[3]),
            Float32(state_vel_ned[3]),
            ZERO_VEC2_F32,
            0.0f0,
            0.0f0,
            0.0f0,
            0.0f0,
            Float32(yaw),
            0.0f0,
            0.0f0,
            0.0f0,
            0.0f0,
            Float32(ref_alt),
            0.0f0,
            0.0f0,
            0.0f0,
            0.0f0,
            0.0f0,
            0.0f0,
            0.0f0,
            NAN_F32,
            NAN_F32,
            NAN_F32,
            NAN_F32,
            NAN_F32,
            true,
            true,
            true,
            true,
            UInt8(0),
            UInt8(0),
            UInt8(0),
            UInt8(0),
            UInt8(0),
            true,
            has_ref,
            has_ref,
            false,
            UInt8(0),
            UInt8(0),
            false,
        )
        queue_uorb_publish!(ap.handle, ap.uorb_local_position_pub, lpos_msg)
    end

    if ap.uorb_global_position_pub !== nothing
        gpos_msg = VehicleGlobalPositionMsg(
            time_us,
            time_us,
            lat,
            lon,
            Float32(alt),
            0.0f0,
            0.0f0,
            0.0f0,
            1.0f0,
            1.0f0,
            0.0f0,
            true,
            true,
            UInt8(0),
            UInt8(0),
            UInt8(0),
            false,
            false,
            ZERO_PAD_U8_5,
        )
        queue_uorb_publish!(ap.handle, ap.uorb_global_position_pub, gpos_msg)
    end

    if ap.uorb_rates_pub !== nothing
        rates_msg = VehicleAngularVelocityMsg(
            time_us,
            0,
            (Float32(ω_body[1]), Float32(ω_body[2]), Float32(ω_body[3])),
            ZERO_VEC3_F32,
        )
        queue_uorb_publish!(ap.handle, ap.uorb_rates_pub, rates_msg)
    end

    if ap.uorb_land_pub !== nothing
        land_msg = VehicleLandDetectedMsg(
            time_us,
            false,
            landed,
            landed,
            landed,
            false,
            false,
            false,
            false,
            false,
            false,
            false,
            false,
            ZERO_PAD_U8,
        )
        queue_uorb_publish!(ap.handle, ap.uorb_land_pub, land_msg)
    end

    if ap.uorb_vehicle_status_pub !== nothing
        arming_state = cmd.armed ? ARMING_STATE_ARMED : ARMING_STATE_DISARMED
        vstatus_msg = VehicleStatusMsg(
            time_us,
            UInt64(0),
            UInt64(0),
            UInt64(0),
            UInt32(0),
            UInt32(0),
            UInt16(0),
            arming_state,
            UInt8(0),
            UInt8(0),
            nav_state,
            nav_state,
            UInt8(0),
            UInt8(0),
            VEHICLE_TYPE_ROTARY_WING,
            false,
            false,
            UInt8(0),
            false,
            UInt8(0),
            false,
            false,
            false,
            false,
            false,
            UInt8(0),
            UInt8(0),
            UInt8(0),
            false,
            false,
            false,
            false,
            false,
            false,
            false,
            false,
            false,
            false,
            false,
            ZERO_PAD_U8_6,
        )
        queue_uorb_publish!(ap.handle, ap.uorb_vehicle_status_pub, vstatus_msg)
    end

    if ap.uorb_control_mode_pub !== nothing
        vcm_msg = VehicleControlModeMsg(
            time_us,
            cmd.armed,
            auto_mode,
            !auto_mode,
            auto_mode,
            false,
            true,
            true,
            true,
            true,
            false,
            true,
            true,
            ap.handle.config.enable_control_allocator != 0,
            false,
            nav_state,
            ZERO_PAD_U8_1,
        )
        queue_uorb_publish!(ap.handle, ap.uorb_control_mode_pub, vcm_msg)
    end

    if ap.uorb_actuator_armed_pub !== nothing
        armed_msg = ActuatorArmedMsg(
            time_us,
            cmd.armed,
            cmd.armed,
            true,
            false,
            false,
            false,
            false,
            ZERO_PAD_U8_1,
        )
        queue_uorb_publish!(ap.handle, ap.uorb_actuator_armed_pub, armed_msg)
    end

    if ap.uorb_geofence_status_pub !== nothing
        geofence_msg = GeofenceStatusMsg(
            time_us,
            UInt32(0),
            UInt8(1),
            ZERO_PAD_U8_3,
        )
        queue_uorb_publish!(ap.handle, ap.uorb_geofence_status_pub, geofence_msg)
    end

    if ap.uorb_home_position_pub !== nothing
        has_home =
            isfinite(ap.home.lat_deg) && isfinite(ap.home.lon_deg) &&
            isfinite(ap.home.alt_msl_m)
        if has_home
            ap.home_update_count += UInt32(1)
            home_msg = HomePositionMsg(
                time_us,
                ap.home.lat_deg,
                ap.home.lon_deg,
                Float32(ap.home.alt_msl_m),
                0.0f0,
                0.0f0,
                0.0f0,
                0.0f0,
                0.0f0,
                0.0f0,
                ap.home_update_count,
                true,
                true,
                true,
                false,
                ZERO_PAD_U8,
            )
            queue_uorb_publish!(ap.handle, ap.uorb_home_position_pub, home_msg)
        end
    end

    if ap.uorb_only
        step_uorb!(ap.handle, time_us)
        ap.last_cmd = cmd
        _update_uorb_outputs!(ap)
        return ap.uorb_outputs
    end

    out = step!(ap.handle, inputs::LockstepInputs)
    ap.last_cmd = cmd
    if ap.uorb_output_subs === nothing
        return out
    end
    _update_uorb_outputs!(ap)
    return ap.uorb_outputs
end

function _env_flag_enabled(name::AbstractString)
    val = get(ENV, name, "")
    return !(isempty(val) || val == "0")
end

end # module Autopilots
