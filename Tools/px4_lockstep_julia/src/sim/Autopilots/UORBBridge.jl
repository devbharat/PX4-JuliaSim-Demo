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

const UORB_INPUT_SPECS = (
    (key = :battery_status, topic = "battery_status", env = UORB_BATTERY_ENV, type = BatteryStatusMsg),
    (key = :vehicle_attitude, topic = "vehicle_attitude", env = UORB_ATTITUDE_ENV, type = VehicleAttitudeMsg),
    (
        key = :vehicle_local_position,
        topic = "vehicle_local_position",
        env = UORB_LOCAL_POSITION_ENV,
        type = VehicleLocalPositionMsg,
    ),
    (
        key = :vehicle_global_position,
        topic = "vehicle_global_position",
        env = UORB_GLOBAL_POSITION_ENV,
        type = VehicleGlobalPositionMsg,
    ),
    (
        key = :vehicle_angular_velocity,
        topic = "vehicle_angular_velocity",
        env = UORB_RATES_ENV,
        type = VehicleAngularVelocityMsg,
    ),
    (
        key = :vehicle_land_detected,
        topic = "vehicle_land_detected",
        env = UORB_LAND_ENV,
        type = VehicleLandDetectedMsg,
    ),
    (
        key = :vehicle_status,
        topic = "vehicle_status",
        env = UORB_VEHICLE_STATUS_ENV,
        type = VehicleStatusMsg,
    ),
    (
        key = :vehicle_control_mode,
        topic = "vehicle_control_mode",
        env = UORB_CONTROL_MODE_ENV,
        type = VehicleControlModeMsg,
    ),
    (
        key = :actuator_armed,
        topic = "actuator_armed",
        env = UORB_ACTUATOR_ARMED_ENV,
        type = ActuatorArmedMsg,
    ),
    (
        key = :home_position,
        topic = "home_position",
        env = UORB_HOME_POSITION_ENV,
        type = HomePositionMsg,
    ),
    (
        key = :geofence_status,
        topic = "geofence_status",
        env = UORB_GEOFENCE_STATUS_ENV,
        type = GeofenceStatusMsg,
    ),
)

const UORB_OUTPUT_SPECS = (
    (
        key = :torque_sp,
        topic = "vehicle_torque_setpoint",
        type = VehicleTorqueSetpointMsg,
    ),
    (
        key = :thrust_sp,
        topic = "vehicle_thrust_setpoint",
        type = VehicleThrustSetpointMsg,
    ),
    (key = :actuator_motors, topic = "actuator_motors", type = ActuatorMotorsMsg),
    (key = :actuator_servos, topic = "actuator_servos", type = ActuatorServosMsg),
    (
        key = :attitude_sp,
        topic = "vehicle_attitude_setpoint",
        type = VehicleAttitudeSetpointMsg,
    ),
    (key = :rates_sp, topic = "vehicle_rates_setpoint", type = VehicleRatesSetpointMsg),
    (key = :mission_result, topic = "mission_result", type = MissionResultMsg),
    (key = :vehicle_status, topic = "vehicle_status", type = VehicleStatusMsg),
    (key = :battery_status, topic = "battery_status", type = BatteryStatusMsg),
    (
        key = :trajectory_setpoint,
        topic = "trajectory_setpoint",
        type = TrajectorySetpointMsg,
    ),
)

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

mutable struct UORBBridge
    handle::LockstepHandle
    pubs::Dict{Symbol,UORBPublisher}
    subs::Dict{Symbol,UORBSubscriber}
end

function _init_uorb_bridge(handle::LockstepHandle)
    pubs = Dict{Symbol,UORBPublisher}()
    for spec in UORB_INPUT_SPECS
        if _env_flag_enabled(spec.env)
            pub, _ = create_uorb_publisher_checked(handle, spec.topic, spec.type)
            pubs[spec.key] = pub
        end
    end
    subs = Dict{Symbol,UORBSubscriber}()
    for spec in UORB_OUTPUT_SPECS
        subs[spec.key] = create_uorb_subscriber(handle, spec.topic)
    end
    return UORBBridge(handle, pubs, subs)
end

function _close_uorb_bridge!(bridge::UORBBridge)
    for sub in values(bridge.subs)
        uorb_unsubscribe!(bridge.handle, sub)
    end
    return nothing
end

function _publish_uorb!(bridge::UORBBridge, key::Symbol, msg)
    pub = get(bridge.pubs, key, nothing)
    pub === nothing && return nothing
    queue_uorb_publish!(bridge.handle, pub, msg)
    return nothing
end

function _read_uorb(bridge::UORBBridge, key::Symbol, ::Type{T}) where {T}
    sub = get(bridge.subs, key, nothing)
    sub === nothing && return nothing
    uorb_check(bridge.handle, sub) || return nothing
    msg = Ref{T}()
    uorb_copy!(bridge.handle, sub, msg)
    return msg[]
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
    torque_msg = _read_uorb(bridge, :torque_sp, VehicleTorqueSetpointMsg)
    if torque_msg !== nothing
        controls = _update_controls_torque(controls, torque_msg)
    end
    thrust_msg = _read_uorb(bridge, :thrust_sp, VehicleThrustSetpointMsg)
    if thrust_msg !== nothing
        controls = _update_controls_thrust(controls, thrust_msg)
    end
    out.actuator_controls = controls

    motors_msg = _read_uorb(bridge, :actuator_motors, ActuatorMotorsMsg)
    if motors_msg !== nothing
        out.actuator_motors = motors_msg.control
    end

    servos_msg = _read_uorb(bridge, :actuator_servos, ActuatorServosMsg)
    if servos_msg !== nothing
        out.actuator_servos = servos_msg.control
    end

    att_msg = _read_uorb(bridge, :attitude_sp, VehicleAttitudeSetpointMsg)
    if att_msg !== nothing
        out.attitude_setpoint_q = att_msg.q_d
        out.thrust_setpoint_body = att_msg.thrust_body
    end

    rates_msg = _read_uorb(bridge, :rates_sp, VehicleRatesSetpointMsg)
    if rates_msg !== nothing
        out.rates_setpoint_xyz = (rates_msg.roll, rates_msg.pitch, rates_msg.yaw)
    end

    mission_msg = _read_uorb(bridge, :mission_result, MissionResultMsg)
    if mission_msg !== nothing
        out.mission_seq = Int32(mission_msg.seq_current)
        out.mission_count = Int32(mission_msg.seq_total)
        out.mission_finished = mission_msg.finished ? Int32(1) : Int32(0)
    end

    vstatus_msg = _read_uorb(bridge, :vehicle_status, VehicleStatusMsg)
    if vstatus_msg !== nothing
        out.nav_state = Int32(vstatus_msg.nav_state)
        out.arming_state = Int32(vstatus_msg.arming_state)
    end

    battery_msg = _read_uorb(bridge, :battery_status, BatteryStatusMsg)
    if battery_msg !== nothing
        out.battery_warning = Int32(battery_msg.warning)
    end

    traj_msg = _read_uorb(bridge, :trajectory_setpoint, TrajectorySetpointMsg)
    if traj_msg !== nothing
        out.trajectory_setpoint_position = traj_msg.position
        out.trajectory_setpoint_velocity = traj_msg.velocity
        out.trajectory_setpoint_acceleration = traj_msg.acceleration
        out.trajectory_setpoint_yaw = traj_msg.yaw
        out.trajectory_setpoint_yawspeed = traj_msg.yawspeed
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

@inline function _vehicle_status_msg(
    time_us::UInt64,
    nav_state::UInt8,
    arming_state::UInt8,
)
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

function _env_flag_enabled(name::AbstractString)
    val = get(ENV, name, "1")
    return !(isempty(val) || val == "0")
end
