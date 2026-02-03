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
        Float32(battery.temperature_c),
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
