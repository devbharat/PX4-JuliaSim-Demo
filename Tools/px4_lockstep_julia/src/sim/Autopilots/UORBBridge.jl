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
const ZERO_PAD_U8_2 = (UInt8(0), UInt8(0))
const ZERO_PAD_U8_3 = (UInt8(0), UInt8(0), UInt8(0))
const ZERO_PAD_U8_5 = (UInt8(0), UInt8(0), UInt8(0), UInt8(0), UInt8(0))
const ZERO_PAD_U8_6 = (UInt8(0), UInt8(0), UInt8(0), UInt8(0), UInt8(0), UInt8(0))
const ZERO_PAD_U8_7 = (UInt8(0), UInt8(0), UInt8(0), UInt8(0), UInt8(0), UInt8(0), UInt8(0))
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

# -----------------------------------------------------------------------------
# uORB interface configuration
# -----------------------------------------------------------------------------

"""Spec for a uORB publisher created by the Julia simulator.

Fields
------
- `key`       : Symbol used internally by the bridge (not a uORB topic name).
- `type`      : Generated uORB message type (subtype of `PX4Lockstep.UORBMsg`).
- `instance`  : uORB instance (-1 = auto).
- `priority`  : publisher priority passed to PX4 (uORB advertise).
- `queue_size`: uORB queue length (nothing -> use `uorb_queue_length(type)`).
"""
Base.@kwdef struct UORBPubSpec
    key::Symbol
    type::DataType
    instance::Int32 = -1
    priority::Int32 = 0
    queue_size::Union{Nothing,Int32} = nothing
end

"""Spec for a uORB subscriber created by the Julia simulator."""
Base.@kwdef struct UORBSubSpec
    key::Symbol
    type::DataType
    instance::UInt32 = UInt32(0)
end

"""Explicit uORB boundary contract between Julia and PX4."""
Base.@kwdef struct PX4UORBInterfaceConfig
    # publishers: what Julia will inject into PX4
    pubs::Vector{UORBPubSpec} = UORBPubSpec[]
    # subscribers: what Julia will read from PX4
    subs::Vector{UORBSubSpec} = UORBSubSpec[]
end


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
    mission_valid::Int32 = 0
    nav_state::Int32 = 0
    arming_state::Int32 = 0
    battery_warning::Int32 = 0
    trajectory_setpoint_position::NTuple{3,Float32} = ZERO_VEC3_F32
    trajectory_setpoint_velocity::NTuple{3,Float32} = ZERO_VEC3_F32
    trajectory_setpoint_acceleration::NTuple{3,Float32} = ZERO_VEC3_F32
    trajectory_setpoint_yaw::Float32 = 0.0f0
    trajectory_setpoint_yawspeed::Float32 = 0.0f0
end

mutable struct UORBBridge
    handle::LockstepHandle

    # Allow multiple uORB instances per logical key.
    # Each entry is (handle, instance).
    pubs::Dict{Symbol,Vector{Tuple{UORBPublisher,Int32}}}
    subs::Dict{Symbol,Vector{Tuple{UORBSubscriber,Int32}}}
end

function _init_uorb_bridge(handle::LockstepHandle, cfg::PX4UORBInterfaceConfig)
    pubs = Dict{Symbol,Vector{Tuple{UORBPublisher,Int32}}}()
    for spec in cfg.pubs
        spec.type <: UORBMsg ||
            error("UORBPubSpec type must be a uORB message type (got $(spec.type))")
        pub, inst = create_publisher(
            handle,
            spec.type;
            priority = spec.priority,
            queue_size = spec.queue_size,
            instance = spec.instance,
        )
        v = get!(pubs, spec.key) do
            Vector{Tuple{UORBPublisher,Int32}}()
        end
        push!(v, (pub, Int32(inst)))
    end

    subs = Dict{Symbol,Vector{Tuple{UORBSubscriber,Int32}}}()
    for spec in cfg.subs
        spec.type <: UORBMsg ||
            error("UORBSubSpec type must be a uORB message type (got $(spec.type))")
        sub = create_subscriber(handle, spec.type; instance = spec.instance)
        v = get!(subs, spec.key) do
            Vector{Tuple{UORBSubscriber,Int32}}()
        end
        push!(v, (sub, Int32(spec.instance)))
    end

    return UORBBridge(handle, pubs, subs)
end



function _close_uorb_bridge!(bridge::UORBBridge)
    for entries in values(bridge.subs)
        for (sub, _inst) in entries
            uorb_unsubscribe!(bridge.handle, sub)
        end
    end
    return nothing
end

"""Publish a message to all configured publishers under `key` (if any)."""
function _publish_uorb!(bridge::UORBBridge, key::Symbol, msg)
    entries = get(bridge.pubs, key, nothing)
    entries === nothing && return nothing
    for (pub, _inst) in entries
        publish!(bridge.handle, pub, msg)
    end
    return nothing
end

"""Read (copy) the first updated subscriber under `key`.

If multiple instances are configured, this returns the first updated one in the
configuration order.
"""
function _read_uorb(bridge::UORBBridge, key::Symbol)
    entries = get(bridge.subs, key, nothing)
    entries === nothing && return nothing
    for (sub, _inst) in entries
        uorb_check(bridge.handle, sub) || continue
        return uorb_copy(bridge.handle, sub)
    end
    return nothing
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

function _update_uorb_outputs!(bridge::UORBBridge, out::UORBOutputs)
    controls = out.actuator_controls
    torque_msg = _read_uorb(bridge, :torque_sp)
    if torque_msg !== nothing
        controls = _update_controls_torque(controls, torque_msg)
    end
    thrust_msg = _read_uorb(bridge, :thrust_sp)
    if thrust_msg !== nothing
        controls = _update_controls_thrust(controls, thrust_msg)
    end
    out.actuator_controls = controls

    motors_msg = _read_uorb(bridge, :actuator_motors)
    if motors_msg !== nothing
        out.actuator_motors = motors_msg.control
    end

    servos_msg = _read_uorb(bridge, :actuator_servos)
    if servos_msg !== nothing
        out.actuator_servos = servos_msg.control
    end

    att_msg = _read_uorb(bridge, :attitude_sp)
    if att_msg !== nothing
        out.attitude_setpoint_q = att_msg.q_d
        out.thrust_setpoint_body = att_msg.thrust_body
    end

    rates_msg = _read_uorb(bridge, :rates_sp)
    if rates_msg !== nothing
        out.rates_setpoint_xyz = (rates_msg.roll, rates_msg.pitch, rates_msg.yaw)
    end

    mission_msg = _read_uorb(bridge, :mission_result)
    if mission_msg !== nothing
        out.mission_seq = Int32(mission_msg.seq_current)
        out.mission_count = Int32(mission_msg.seq_total)
        out.mission_finished = mission_msg.finished ? Int32(1) : Int32(0)
        out.mission_valid = mission_msg.valid ? Int32(1) : Int32(0)
    end

    vstatus_msg = _read_uorb(bridge, :vehicle_status)
    if vstatus_msg !== nothing
        out.nav_state = Int32(vstatus_msg.nav_state)
        out.arming_state = Int32(vstatus_msg.arming_state)
    end

    battery_msg = _read_uorb(bridge, :battery_status)
    if battery_msg !== nothing
        out.battery_warning = Int32(battery_msg.warning)
    end

    traj_msg = _read_uorb(bridge, :trajectory_setpoint)
    if traj_msg !== nothing
        out.trajectory_setpoint_position = traj_msg.position
        out.trajectory_setpoint_velocity = traj_msg.velocity
        out.trajectory_setpoint_acceleration = traj_msg.acceleration
        out.trajectory_setpoint_yaw = traj_msg.yaw
        out.trajectory_setpoint_yawspeed = traj_msg.yawspeed
    end

    return out
end

@inline function _battery_status_msg(
    time_us::UInt64,
    battery::BatteryStatus;
    id::UInt8 = UInt8(0),
)
    return BatteryStatusMsg(
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
        id,
        false,
        false,
        UInt8(battery.warning),
        ZERO_PAD_U8_2,
    )
end

@inline function _vehicle_attitude_msg(time_us::UInt64, q_bn::Quat)
    return VehicleAttitudeMsg(
        time_us,
        time_us,
        (Float32(q_bn[1]), Float32(q_bn[2]), Float32(q_bn[3]), Float32(q_bn[4])),
        ZERO_Q_F32,
        UInt8(0),
        ZERO_PAD_U8_7,
    )
end

@inline function _vehicle_local_position_msg(
    time_us::UInt64,
    state_pos_ned::Vec3,
    state_vel_ned::Vec3,
    yaw::Float64,
    ref_lat::Float64,
    ref_lon::Float64,
    ref_alt::Float64,
)
    has_ref = isfinite(ref_lat) && isfinite(ref_lon) && isfinite(ref_alt)
    return VehicleLocalPositionMsg(
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
end

@inline function _vehicle_global_position_msg(
    time_us::UInt64,
    lat::Float64,
    lon::Float64,
    alt::Float64,
)
    t = time_us == 0 ? UInt64(1) : time_us
    return VehicleGlobalPositionMsg(
        t,
        t,
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
end

@inline function _vehicle_angular_velocity_msg(time_us::UInt64, ω_body::Vec3)
    return VehicleAngularVelocityMsg(
        time_us,
        time_us,
        (Float32(ω_body[1]), Float32(ω_body[2]), Float32(ω_body[3])),
        ZERO_VEC3_F32,
    )
end

@inline function _vehicle_land_detected_msg(time_us::UInt64, landed::Bool)
    return VehicleLandDetectedMsg(
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
end

@inline function _vehicle_status_msg(time_us::UInt64, nav_state::UInt8, arming_state::UInt8)
    return VehicleStatusMsg(
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
end

@inline function _vehicle_control_mode_msg(
    time_us::UInt64,
    cmd::AutopilotCommand,
    auto_mode::Bool,
    _nav_state::UInt8,
    control_allocator_enabled::Bool,
)
    return VehicleControlModeMsg(
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
        control_allocator_enabled,
        false,
        UInt8(0),
        ZERO_PAD_U8_1,
    )
end

@inline function _actuator_armed_msg(time_us::UInt64, cmd::AutopilotCommand)
    return ActuatorArmedMsg(
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
end

@inline function _geofence_status_msg(time_us::UInt64)
    t = time_us == 0 ? UInt64(1) : time_us
    return GeofenceStatusMsg(t, UInt32(0), UInt8(1), ZERO_PAD_U8_3)
end

@inline function _home_position_msg(
    time_us::UInt64,
    lat_deg::Float64,
    lon_deg::Float64,
    alt_msl_m::Float64,
    update_count::UInt32,
)
    t = time_us == 0 ? UInt64(1) : time_us
    return HomePositionMsg(
        t,
        lat_deg,
        lon_deg,
        Float32(alt_msl_m),
        0.0f0,
        0.0f0,
        0.0f0,
        0.0f0,
        0.0f0,
        0.0f0,
        update_count,
        true,
        true,
        true,
        false,
        ZERO_PAD_U8,
    )
end
