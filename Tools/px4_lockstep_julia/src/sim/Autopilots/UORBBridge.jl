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

"""Default uORB interface for the Iris workflow.

Injects estimated state + a few status topics into PX4, and subscribes to actuator and
setpoint outputs.

This mirrors the previous `UORB_INPUT_SPECS` / `UORB_OUTPUT_SPECS` default behavior,
but is now explicit and reproducible.
"""
function iris_state_injection_interface()
    pubs = UORBPubSpec[
        UORBPubSpec(key = :battery_status, type = BatteryStatusMsg),
        UORBPubSpec(key = :vehicle_attitude, type = VehicleAttitudeMsg),
        UORBPubSpec(key = :vehicle_local_position, type = VehicleLocalPositionMsg),
        UORBPubSpec(key = :vehicle_global_position, type = VehicleGlobalPositionMsg),
        UORBPubSpec(key = :vehicle_angular_velocity, type = VehicleAngularVelocityMsg),
        UORBPubSpec(key = :vehicle_land_detected, type = VehicleLandDetectedMsg),
        UORBPubSpec(key = :vehicle_status, type = VehicleStatusMsg),
        UORBPubSpec(key = :vehicle_control_mode, type = VehicleControlModeMsg),
        UORBPubSpec(key = :actuator_armed, type = ActuatorArmedMsg),
        UORBPubSpec(key = :home_position, type = HomePositionMsg),
        UORBPubSpec(key = :geofence_status, type = GeofenceStatusMsg),
    ]

    subs = UORBSubSpec[
        UORBSubSpec(key = :torque_sp, type = VehicleTorqueSetpointMsg),
        UORBSubSpec(key = :thrust_sp, type = VehicleThrustSetpointMsg),
        UORBSubSpec(key = :actuator_motors, type = ActuatorMotorsMsg),
        UORBSubSpec(key = :actuator_servos, type = ActuatorServosMsg),
        UORBSubSpec(key = :attitude_sp, type = VehicleAttitudeSetpointMsg),
        UORBSubSpec(key = :rates_sp, type = VehicleRatesSetpointMsg),
        UORBSubSpec(key = :mission_result, type = MissionResultMsg),
        UORBSubSpec(key = :vehicle_status, type = VehicleStatusMsg),
        UORBSubSpec(key = :battery_status, type = BatteryStatusMsg),
        UORBSubSpec(key = :trajectory_setpoint, type = TrajectorySetpointMsg),
    ]

    return PX4UORBInterfaceConfig(pubs = pubs, subs = subs)
end

"""Minimal uORB interface used by unit tests or debugging.

This does **not** inject state into PX4, so it is not suitable for live mission runs.
"""
function minimal_actuator_only_interface()
    subs = UORBSubSpec[
        UORBSubSpec(key = :actuator_motors, type = ActuatorMotorsMsg),
        UORBSubSpec(key = :actuator_servos, type = ActuatorServosMsg),
    ]
    return PX4UORBInterfaceConfig(pubs = UORBPubSpec[], subs = subs)
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
    nav_state::Int32 = 0
    arming_state::Int32 = 0
    battery_warning::Int32 = 0
    trajectory_setpoint_position::NTuple{3,Float32} = ZERO_VEC3_F32
    trajectory_setpoint_velocity::NTuple{3,Float32} = ZERO_VEC3_F32
    trajectory_setpoint_acceleration::NTuple{3,Float32} = ZERO_VEC3_F32
    trajectory_setpoint_yaw::Float32 = 0.0f0
    trajectory_setpoint_yawspeed::Float32 = 0.0f0
end

struct UORBSubSlot{T}
    sub::UORBSubscriber{T}
    buf::Base.RefValue{T}
end

UORBSubSlot(sub::UORBSubscriber{T}) where {T} = UORBSubSlot{T}(sub, Ref{T}())

@inline function read_updated!(h::LockstepHandle, slot::UORBSubSlot{T}) where {T}
    uorb_check(h, slot.sub) || return false
    uorb_copy!(h, slot.sub, slot.buf)
    return true
end

mutable struct UORBBridge
    handle::LockstepHandle
    pubs::Dict{Symbol,UORBPublisher}
    subs::Dict{Symbol,UORBSubSlot}

    # Fast-path publishers (avoid Dict lookups in hot loop).
    pub_battery_status::Union{Nothing,UORBPublisher{BatteryStatusMsg}}
    pub_vehicle_attitude::Union{Nothing,UORBPublisher{VehicleAttitudeMsg}}
    pub_vehicle_local_position::Union{Nothing,UORBPublisher{VehicleLocalPositionMsg}}
    pub_vehicle_global_position::Union{Nothing,UORBPublisher{VehicleGlobalPositionMsg}}
    pub_vehicle_angular_velocity::Union{Nothing,UORBPublisher{VehicleAngularVelocityMsg}}
    pub_vehicle_land_detected::Union{Nothing,UORBPublisher{VehicleLandDetectedMsg}}
    pub_vehicle_status::Union{Nothing,UORBPublisher{VehicleStatusMsg}}
    pub_vehicle_control_mode::Union{Nothing,UORBPublisher{VehicleControlModeMsg}}
    pub_actuator_armed::Union{Nothing,UORBPublisher{ActuatorArmedMsg}}
    pub_home_position::Union{Nothing,UORBPublisher{HomePositionMsg}}
    pub_geofence_status::Union{Nothing,UORBPublisher{GeofenceStatusMsg}}

    # Fast-path subscribers (avoid Dict lookups + allocations in hot loop).
    sub_torque_sp::Union{Nothing,UORBSubSlot{VehicleTorqueSetpointMsg}}
    sub_thrust_sp::Union{Nothing,UORBSubSlot{VehicleThrustSetpointMsg}}
    sub_actuator_motors::Union{Nothing,UORBSubSlot{ActuatorMotorsMsg}}
    sub_actuator_servos::Union{Nothing,UORBSubSlot{ActuatorServosMsg}}
    sub_attitude_sp::Union{Nothing,UORBSubSlot{VehicleAttitudeSetpointMsg}}
    sub_rates_sp::Union{Nothing,UORBSubSlot{VehicleRatesSetpointMsg}}
    sub_mission_result::Union{Nothing,UORBSubSlot{MissionResultMsg}}
    sub_vehicle_status::Union{Nothing,UORBSubSlot{VehicleStatusMsg}}
    sub_battery_status::Union{Nothing,UORBSubSlot{BatteryStatusMsg}}
    sub_trajectory_setpoint::Union{Nothing,UORBSubSlot{TrajectorySetpointMsg}}
end

function _init_uorb_bridge(handle::LockstepHandle, cfg::PX4UORBInterfaceConfig)
    pubs = Dict{Symbol,UORBPublisher}()
    pub_battery_status = nothing
    pub_vehicle_attitude = nothing
    pub_vehicle_local_position = nothing
    pub_vehicle_global_position = nothing
    pub_vehicle_angular_velocity = nothing
    pub_vehicle_land_detected = nothing
    pub_vehicle_status = nothing
    pub_vehicle_control_mode = nothing
    pub_actuator_armed = nothing
    pub_home_position = nothing
    pub_geofence_status = nothing
    for spec in cfg.pubs
        spec.type <: UORBMsg ||
            error("UORBPubSpec type must be a uORB message type (got $(spec.type))")
        pub, _ = create_publisher(
            handle,
            spec.type;
            priority = spec.priority,
            queue_size = spec.queue_size,
            instance = spec.instance,
        )
        pubs[spec.key] = pub
        if spec.key === :battery_status
            pub_battery_status = pub::UORBPublisher{BatteryStatusMsg}
        elseif spec.key === :vehicle_attitude
            pub_vehicle_attitude = pub::UORBPublisher{VehicleAttitudeMsg}
        elseif spec.key === :vehicle_local_position
            pub_vehicle_local_position = pub::UORBPublisher{VehicleLocalPositionMsg}
        elseif spec.key === :vehicle_global_position
            pub_vehicle_global_position = pub::UORBPublisher{VehicleGlobalPositionMsg}
        elseif spec.key === :vehicle_angular_velocity
            pub_vehicle_angular_velocity = pub::UORBPublisher{VehicleAngularVelocityMsg}
        elseif spec.key === :vehicle_land_detected
            pub_vehicle_land_detected = pub::UORBPublisher{VehicleLandDetectedMsg}
        elseif spec.key === :vehicle_status
            pub_vehicle_status = pub::UORBPublisher{VehicleStatusMsg}
        elseif spec.key === :vehicle_control_mode
            pub_vehicle_control_mode = pub::UORBPublisher{VehicleControlModeMsg}
        elseif spec.key === :actuator_armed
            pub_actuator_armed = pub::UORBPublisher{ActuatorArmedMsg}
        elseif spec.key === :home_position
            pub_home_position = pub::UORBPublisher{HomePositionMsg}
        elseif spec.key === :geofence_status
            pub_geofence_status = pub::UORBPublisher{GeofenceStatusMsg}
        end
    end

    subs = Dict{Symbol,UORBSubSlot}()
    sub_torque_sp = nothing
    sub_thrust_sp = nothing
    sub_actuator_motors = nothing
    sub_actuator_servos = nothing
    sub_attitude_sp = nothing
    sub_rates_sp = nothing
    sub_mission_result = nothing
    sub_vehicle_status = nothing
    sub_battery_status = nothing
    sub_trajectory_setpoint = nothing
    for spec in cfg.subs
        spec.type <: UORBMsg ||
            error("UORBSubSpec type must be a uORB message type (got $(spec.type))")
        sub = create_subscriber(handle, spec.type; instance = spec.instance)
        slot = UORBSubSlot(sub)
        subs[spec.key] = slot
        if spec.key === :torque_sp
            sub_torque_sp = slot::UORBSubSlot{VehicleTorqueSetpointMsg}
        elseif spec.key === :thrust_sp
            sub_thrust_sp = slot::UORBSubSlot{VehicleThrustSetpointMsg}
        elseif spec.key === :actuator_motors
            sub_actuator_motors = slot::UORBSubSlot{ActuatorMotorsMsg}
        elseif spec.key === :actuator_servos
            sub_actuator_servos = slot::UORBSubSlot{ActuatorServosMsg}
        elseif spec.key === :attitude_sp
            sub_attitude_sp = slot::UORBSubSlot{VehicleAttitudeSetpointMsg}
        elseif spec.key === :rates_sp
            sub_rates_sp = slot::UORBSubSlot{VehicleRatesSetpointMsg}
        elseif spec.key === :mission_result
            sub_mission_result = slot::UORBSubSlot{MissionResultMsg}
        elseif spec.key === :vehicle_status
            sub_vehicle_status = slot::UORBSubSlot{VehicleStatusMsg}
        elseif spec.key === :battery_status
            sub_battery_status = slot::UORBSubSlot{BatteryStatusMsg}
        elseif spec.key === :trajectory_setpoint
            sub_trajectory_setpoint = slot::UORBSubSlot{TrajectorySetpointMsg}
        end
    end

    return UORBBridge(
        handle,
        pubs,
        subs,
        pub_battery_status,
        pub_vehicle_attitude,
        pub_vehicle_local_position,
        pub_vehicle_global_position,
        pub_vehicle_angular_velocity,
        pub_vehicle_land_detected,
        pub_vehicle_status,
        pub_vehicle_control_mode,
        pub_actuator_armed,
        pub_home_position,
        pub_geofence_status,
        sub_torque_sp,
        sub_thrust_sp,
        sub_actuator_motors,
        sub_actuator_servos,
        sub_attitude_sp,
        sub_rates_sp,
        sub_mission_result,
        sub_vehicle_status,
        sub_battery_status,
        sub_trajectory_setpoint,
    )
end

# Convenience: build the default Iris interface.
_init_uorb_bridge(handle::LockstepHandle) =
    _init_uorb_bridge(handle, iris_state_injection_interface())

function _close_uorb_bridge!(bridge::UORBBridge)
    for slot in values(bridge.subs)
        uorb_unsubscribe!(bridge.handle, slot.sub)
    end
    return nothing
end

function _publish_uorb!(bridge::UORBBridge, key::Symbol, msg)
    pub = get(bridge.pubs, key, nothing)
    pub === nothing && return nothing
    publish!(bridge.handle, pub, msg)
    return nothing
end

function _read_uorb(bridge::UORBBridge, key::Symbol)
    slot = get(bridge.subs, key, nothing)
    slot === nothing && return nothing
    read_updated!(bridge.handle, slot) || return nothing
    return slot.buf[]
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
    slot = bridge.sub_torque_sp
    if slot !== nothing && read_updated!(bridge.handle, slot)
        controls = _update_controls_torque(controls, slot.buf[])
    end
    slot = bridge.sub_thrust_sp
    if slot !== nothing && read_updated!(bridge.handle, slot)
        controls = _update_controls_thrust(controls, slot.buf[])
    end
    out.actuator_controls = controls

    slot = bridge.sub_actuator_motors
    if slot !== nothing && read_updated!(bridge.handle, slot)
        out.actuator_motors = slot.buf[].control
    end

    slot = bridge.sub_actuator_servos
    if slot !== nothing && read_updated!(bridge.handle, slot)
        out.actuator_servos = slot.buf[].control
    end

    slot = bridge.sub_attitude_sp
    if slot !== nothing && read_updated!(bridge.handle, slot)
        msg = slot.buf[]
        out.attitude_setpoint_q = msg.q_d
        out.thrust_setpoint_body = msg.thrust_body
    end

    slot = bridge.sub_rates_sp
    if slot !== nothing && read_updated!(bridge.handle, slot)
        msg = slot.buf[]
        out.rates_setpoint_xyz = (msg.roll, msg.pitch, msg.yaw)
    end

    slot = bridge.sub_mission_result
    if slot !== nothing && read_updated!(bridge.handle, slot)
        msg = slot.buf[]
        out.mission_seq = Int32(msg.seq_current)
        out.mission_count = Int32(msg.seq_total)
        out.mission_finished = msg.finished ? Int32(1) : Int32(0)
    end

    slot = bridge.sub_vehicle_status
    if slot !== nothing && read_updated!(bridge.handle, slot)
        msg = slot.buf[]
        out.nav_state = Int32(msg.nav_state)
        out.arming_state = Int32(msg.arming_state)
    end

    slot = bridge.sub_battery_status
    if slot !== nothing && read_updated!(bridge.handle, slot)
        out.battery_warning = Int32(slot.buf[].warning)
    end

    slot = bridge.sub_trajectory_setpoint
    if slot !== nothing && read_updated!(bridge.handle, slot)
        msg = slot.buf[]
        out.trajectory_setpoint_position = msg.position
        out.trajectory_setpoint_velocity = msg.velocity
        out.trajectory_setpoint_acceleration = msg.acceleration
        out.trajectory_setpoint_yaw = msg.yaw
        out.trajectory_setpoint_yawspeed = msg.yawspeed
    end

    return out
end

@inline function _battery_status_msg(time_us::UInt64, battery::BatteryStatus)
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
        UInt8(0),
        false,
        false,
        UInt8(battery.warning),
        ZERO_PAD_U8_2,
    )
end

@inline function _vehicle_attitude_msg(time_us::UInt64, q_bn::Quat)
    return VehicleAttitudeMsg(
        time_us,
        0,
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
    return VehicleGlobalPositionMsg(
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
end

@inline function _vehicle_angular_velocity_msg(time_us::UInt64, ω_body::Vec3)
    return VehicleAngularVelocityMsg(
        time_us,
        0,
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
    nav_state::UInt8,
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
        nav_state,
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
    return GeofenceStatusMsg(time_us, UInt32(0), UInt8(1), ZERO_PAD_U8_3)
end

@inline function _home_position_msg(
    time_us::UInt64,
    lat_deg::Float64,
    lon_deg::Float64,
    alt_msl_m::Float64,
    update_count::UInt32,
)
    return HomePositionMsg(
        time_us,
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
