# Auto-generated from PX4 uORB headers
# headers: build/px4_sitl_lockstep/uORB/topics
# topics: battery_status, vehicle_attitude, vehicle_local_position, vehicle_global_position, vehicle_angular_velocity, vehicle_land_detected, vehicle_status, vehicle_control_mode, actuator_armed, home_position, geofence_status, vehicle_torque_setpoint, vehicle_thrust_setpoint, actuator_motors, actuator_servos, vehicle_attitude_setpoint, vehicle_rates_setpoint, mission_result, trajectory_setpoint

struct BatteryStatusMsg
    timestamp::UInt64
    voltage_v::Float32
    current_a::Float32
    current_average_a::Float32
    discharged_mah::Float32
    remaining::Float32
    scale::Float32
    time_remaining_s::Float32
    temperature::Float32
    voltage_cell_v::NTuple{14,Float32}
    max_cell_voltage_delta::Float32
    full_charge_capacity_wh::Float32
    remaining_capacity_wh::Float32
    nominal_voltage::Float32
    internal_resistance_estimate::Float32
    ocv_estimate::Float32
    ocv_estimate_filtered::Float32
    volt_based_soc_estimate::Float32
    voltage_prediction::Float32
    prediction_error::Float32
    estimation_covariance_norm::Float32
    capacity::UInt16
    cycle_count::UInt16
    average_time_to_empty::UInt16
    manufacture_date::UInt16
    state_of_health::UInt16
    max_error::UInt16
    interface_error::UInt16
    faults::UInt16
    over_discharge_count::UInt16
    connected::Bool
    cell_count::UInt8
    source::UInt8
    priority::UInt8
    id::UInt8
    is_powering_off::Bool
    is_required::Bool
    warning::UInt8
    _padding0::NTuple{2,UInt8}
end

struct VehicleAttitudeMsg
    timestamp::UInt64
    timestamp_sample::UInt64
    q::NTuple{4,Float32}
    delta_q_reset::NTuple{4,Float32}
    quat_reset_counter::UInt8
    _padding0::NTuple{7,UInt8}
end

struct VehicleLocalPositionMsg
    timestamp::UInt64
    timestamp_sample::UInt64
    ref_timestamp::UInt64
    ref_lat::Float64
    ref_lon::Float64
    x::Float32
    y::Float32
    z::Float32
    delta_xy::NTuple{2,Float32}
    delta_z::Float32
    vx::Float32
    vy::Float32
    vz::Float32
    z_deriv::Float32
    delta_vxy::NTuple{2,Float32}
    delta_vz::Float32
    ax::Float32
    ay::Float32
    az::Float32
    heading::Float32
    heading_var::Float32
    unaided_heading::Float32
    delta_heading::Float32
    tilt_var::Float32
    ref_alt::Float32
    dist_bottom::Float32
    dist_bottom_var::Float32
    delta_dist_bottom::Float32
    eph::Float32
    epv::Float32
    evh::Float32
    evv::Float32
    vxy_max::Float32
    vz_max::Float32
    hagl_min::Float32
    hagl_max_z::Float32
    hagl_max_xy::Float32
    xy_valid::Bool
    z_valid::Bool
    v_xy_valid::Bool
    v_z_valid::Bool
    xy_reset_counter::UInt8
    z_reset_counter::UInt8
    vxy_reset_counter::UInt8
    vz_reset_counter::UInt8
    heading_reset_counter::UInt8
    heading_good_for_control::Bool
    xy_global::Bool
    z_global::Bool
    dist_bottom_valid::Bool
    dist_bottom_reset_counter::UInt8
    dist_bottom_sensor_bitfield::UInt8
    dead_reckoning::Bool
end

struct VehicleGlobalPositionMsg
    timestamp::UInt64
    timestamp_sample::UInt64
    lat::Float64
    lon::Float64
    alt::Float32
    alt_ellipsoid::Float32
    delta_alt::Float32
    delta_terrain::Float32
    eph::Float32
    epv::Float32
    terrain_alt::Float32
    lat_lon_valid::Bool
    alt_valid::Bool
    lat_lon_reset_counter::UInt8
    alt_reset_counter::UInt8
    terrain_reset_counter::UInt8
    terrain_alt_valid::Bool
    dead_reckoning::Bool
    _padding0::NTuple{5,UInt8}
end

struct VehicleAngularVelocityMsg
    timestamp::UInt64
    timestamp_sample::UInt64
    xyz::NTuple{3,Float32}
    xyz_derivative::NTuple{3,Float32}
end

struct VehicleLandDetectedMsg
    timestamp::UInt64
    freefall::Bool
    ground_contact::Bool
    maybe_landed::Bool
    landed::Bool
    in_ground_effect::Bool
    in_descend::Bool
    has_low_throttle::Bool
    vertical_movement::Bool
    horizontal_movement::Bool
    rotational_movement::Bool
    close_to_ground_or_skipped_check::Bool
    at_rest::Bool
    _padding0::NTuple{4,UInt8}
end

struct VehicleStatusMsg
    timestamp::UInt64
    armed_time::UInt64
    takeoff_time::UInt64
    nav_state_timestamp::UInt64
    valid_nav_states_mask::UInt32
    can_set_nav_states_mask::UInt32
    failure_detector_status::UInt16
    arming_state::UInt8
    latest_arming_reason::UInt8
    latest_disarming_reason::UInt8
    nav_state_user_intention::UInt8
    nav_state::UInt8
    executor_in_charge::UInt8
    hil_state::UInt8
    vehicle_type::UInt8
    failsafe::Bool
    failsafe_and_user_took_over::Bool
    failsafe_defer_state::UInt8
    gcs_connection_lost::Bool
    gcs_connection_lost_counter::UInt8
    high_latency_data_link_lost::Bool
    is_vtol::Bool
    is_vtol_tailsitter::Bool
    in_transition_mode::Bool
    in_transition_to_fw::Bool
    system_type::UInt8
    system_id::UInt8
    component_id::UInt8
    safety_button_available::Bool
    safety_off::Bool
    power_input_valid::Bool
    usb_connected::Bool
    open_drone_id_system_present::Bool
    open_drone_id_system_healthy::Bool
    parachute_system_present::Bool
    parachute_system_healthy::Bool
    rc_calibration_in_progress::Bool
    calibration_enabled::Bool
    pre_flight_checks_pass::Bool
    _padding0::NTuple{6,UInt8}
end

struct VehicleControlModeMsg
    timestamp::UInt64
    flag_armed::Bool
    flag_multicopter_position_control_enabled::Bool
    flag_control_manual_enabled::Bool
    flag_control_auto_enabled::Bool
    flag_control_offboard_enabled::Bool
    flag_control_position_enabled::Bool
    flag_control_velocity_enabled::Bool
    flag_control_altitude_enabled::Bool
    flag_control_climb_rate_enabled::Bool
    flag_control_acceleration_enabled::Bool
    flag_control_attitude_enabled::Bool
    flag_control_rates_enabled::Bool
    flag_control_allocation_enabled::Bool
    flag_control_termination_enabled::Bool
    source_id::UInt8
    _padding0::NTuple{1,UInt8}
end

struct ActuatorArmedMsg
    timestamp::UInt64
    armed::Bool
    prearmed::Bool
    ready_to_arm::Bool
    lockdown::Bool
    kill::Bool
    termination::Bool
    in_esc_calibration_mode::Bool
    _padding0::NTuple{1,UInt8}
end

struct HomePositionMsg
    timestamp::UInt64
    lat::Float64
    lon::Float64
    alt::Float32
    x::Float32
    y::Float32
    z::Float32
    roll::Float32
    pitch::Float32
    yaw::Float32
    update_count::UInt32
    valid_alt::Bool
    valid_hpos::Bool
    valid_lpos::Bool
    manual_home::Bool
    _padding0::NTuple{4,UInt8}
end

struct GeofenceStatusMsg
    timestamp::UInt64
    geofence_id::UInt32
    status::UInt8
    _padding0::NTuple{3,UInt8}
end

struct VehicleTorqueSetpointMsg
    timestamp::UInt64
    timestamp_sample::UInt64
    xyz::NTuple{3,Float32}
    _padding0::NTuple{4,UInt8}
end

struct VehicleThrustSetpointMsg
    timestamp::UInt64
    timestamp_sample::UInt64
    xyz::NTuple{3,Float32}
    _padding0::NTuple{4,UInt8}
end

struct ActuatorMotorsMsg
    timestamp::UInt64
    timestamp_sample::UInt64
    control::NTuple{12,Float32}
    reversible_flags::UInt16
    _padding0::NTuple{6,UInt8}
end

struct ActuatorServosMsg
    timestamp::UInt64
    timestamp_sample::UInt64
    control::NTuple{8,Float32}
end

struct VehicleAttitudeSetpointMsg
    timestamp::UInt64
    yaw_sp_move_rate::Float32
    q_d::NTuple{4,Float32}
    thrust_body::NTuple{3,Float32}
end

struct VehicleRatesSetpointMsg
    timestamp::UInt64
    roll::Float32
    pitch::Float32
    yaw::Float32
    thrust_body::NTuple{3,Float32}
    reset_integral::Bool
    _padding0::NTuple{7,UInt8}
end

struct MissionResultMsg
    timestamp::UInt64
    mission_id::UInt32
    geofence_id::UInt32
    home_position_counter::UInt32
    seq_reached::Int32
    seq_current::UInt16
    seq_total::UInt16
    item_changed_index::UInt16
    item_do_jump_remaining::UInt16
    valid::Bool
    warning::Bool
    finished::Bool
    failure::Bool
    item_do_jump_changed::Bool
    execution_mode::UInt8
    _padding0::NTuple{2,UInt8}
end

struct TrajectorySetpointMsg
    timestamp::UInt64
    position::NTuple{3,Float32}
    velocity::NTuple{3,Float32}
    acceleration::NTuple{3,Float32}
    jerk::NTuple{3,Float32}
    yaw::Float32
    yawspeed::Float32
end
