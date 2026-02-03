# Auto-generated from PX4 uORB headers
# headers: build/px4_sitl_lockstep/uORB/topics
# topics: battery_status, vehicle_attitude, vehicle_local_position, vehicle_global_position, vehicle_angular_velocity, vehicle_land_detected, vehicle_status, vehicle_control_mode, actuator_armed, home_position, geofence_status, vehicle_torque_setpoint, vehicle_thrust_setpoint, actuator_motors, actuator_servos, vehicle_attitude_setpoint, vehicle_rates_setpoint, mission_result, trajectory_setpoint

abstract type UORBMsg end

uorb_topic(::Type{T}) where {T<:UORBMsg} = error("uorb_topic not defined for " * string(T))
uorb_queue_length(::Type{T}) where {T<:UORBMsg} = 1
uorb_fields_hash(::Type{T}) where {T<:UORBMsg} = 0x0000000000000000
uorb_message_hash(::Type{T}) where {T<:UORBMsg} = 0x00000000
uorb_fields(::Type{T}) where {T<:UORBMsg} = ""

struct BatteryStatusMsg <: UORBMsg
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

uorb_topic(::Type{BatteryStatusMsg}) = "battery_status"
uorb_queue_length(::Type{BatteryStatusMsg}) = 1
uorb_fields_hash(::Type{BatteryStatusMsg}) = 0x5754359606b9fa8e
uorb_message_hash(::Type{BatteryStatusMsg}) = 0xfd330964
uorb_fields(
    ::Type{BatteryStatusMsg},
) = "uint64_t timestamp;float voltage_v;float current_a;float current_average_a;float discharged_mah;float remaining;float scale;float time_remaining_s;float temperature;float[14] voltage_cell_v;float max_cell_voltage_delta;float full_charge_capacity_wh;float remaining_capacity_wh;float nominal_voltage;float internal_resistance_estimate;float ocv_estimate;float ocv_estimate_filtered;float volt_based_soc_estimate;float voltage_prediction;float prediction_error;float estimation_covariance_norm;uint16_t capacity;uint16_t cycle_count;uint16_t average_time_to_empty;uint16_t manufacture_date;uint16_t state_of_health;uint16_t max_error;uint16_t interface_error;uint16_t faults;uint16_t over_discharge_count;bool connected;uint8_t cell_count;uint8_t source;uint8_t priority;uint8_t id;bool is_powering_off;bool is_required;uint8_t warning"

struct VehicleAttitudeMsg <: UORBMsg
    timestamp::UInt64
    timestamp_sample::UInt64
    q::NTuple{4,Float32}
    delta_q_reset::NTuple{4,Float32}
    quat_reset_counter::UInt8
    _padding0::NTuple{7,UInt8}
end

uorb_topic(::Type{VehicleAttitudeMsg}) = "vehicle_attitude"
uorb_queue_length(::Type{VehicleAttitudeMsg}) = 1
uorb_fields_hash(::Type{VehicleAttitudeMsg}) = 0xb93173293a570a66
uorb_message_hash(::Type{VehicleAttitudeMsg}) = 0x0ac1f7cd
uorb_fields(
    ::Type{VehicleAttitudeMsg},
) = "uint64_t timestamp;uint64_t timestamp_sample;float[4] q;float[4] delta_q_reset;uint8_t quat_reset_counter"

struct VehicleLocalPositionMsg <: UORBMsg
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

uorb_topic(::Type{VehicleLocalPositionMsg}) = "vehicle_local_position"
uorb_queue_length(::Type{VehicleLocalPositionMsg}) = 1
uorb_fields_hash(::Type{VehicleLocalPositionMsg}) = 0x9d02f99b80089def
uorb_message_hash(::Type{VehicleLocalPositionMsg}) = 0x5141d2f7
uorb_fields(
    ::Type{VehicleLocalPositionMsg},
) = "uint64_t timestamp;uint64_t timestamp_sample;uint64_t ref_timestamp;double ref_lat;double ref_lon;float x;float y;float z;float[2] delta_xy;float delta_z;float vx;float vy;float vz;float z_deriv;float[2] delta_vxy;float delta_vz;float ax;float ay;float az;float heading;float heading_var;float unaided_heading;float delta_heading;float tilt_var;float ref_alt;float dist_bottom;float dist_bottom_var;float delta_dist_bottom;float eph;float epv;float evh;float evv;float vxy_max;float vz_max;float hagl_min;float hagl_max_z;float hagl_max_xy;bool xy_valid;bool z_valid;bool v_xy_valid;bool v_z_valid;uint8_t xy_reset_counter;uint8_t z_reset_counter;uint8_t vxy_reset_counter;uint8_t vz_reset_counter;uint8_t heading_reset_counter;bool heading_good_for_control;bool xy_global;bool z_global;bool dist_bottom_valid;uint8_t dist_bottom_reset_counter;uint8_t dist_bottom_sensor_bitfield;bool dead_reckoning"

struct VehicleGlobalPositionMsg <: UORBMsg
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

uorb_topic(::Type{VehicleGlobalPositionMsg}) = "vehicle_global_position"
uorb_queue_length(::Type{VehicleGlobalPositionMsg}) = 1
uorb_fields_hash(::Type{VehicleGlobalPositionMsg}) = 0x9a21194e115e9cc2
uorb_message_hash(::Type{VehicleGlobalPositionMsg}) = 0x923ca365
uorb_fields(
    ::Type{VehicleGlobalPositionMsg},
) = "uint64_t timestamp;uint64_t timestamp_sample;double lat;double lon;float alt;float alt_ellipsoid;float delta_alt;float delta_terrain;float eph;float epv;float terrain_alt;bool lat_lon_valid;bool alt_valid;uint8_t lat_lon_reset_counter;uint8_t alt_reset_counter;uint8_t terrain_reset_counter;bool terrain_alt_valid;bool dead_reckoning"

struct VehicleAngularVelocityMsg <: UORBMsg
    timestamp::UInt64
    timestamp_sample::UInt64
    xyz::NTuple{3,Float32}
    xyz_derivative::NTuple{3,Float32}
end

uorb_topic(::Type{VehicleAngularVelocityMsg}) = "vehicle_angular_velocity"
uorb_queue_length(::Type{VehicleAngularVelocityMsg}) = 1
uorb_fields_hash(::Type{VehicleAngularVelocityMsg}) = 0x504cdc5c34d66ba5
uorb_message_hash(::Type{VehicleAngularVelocityMsg}) = 0x48cb9568
uorb_fields(
    ::Type{VehicleAngularVelocityMsg},
) = "uint64_t timestamp;uint64_t timestamp_sample;float[3] xyz;float[3] xyz_derivative"

struct VehicleLandDetectedMsg <: UORBMsg
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

uorb_topic(::Type{VehicleLandDetectedMsg}) = "vehicle_land_detected"
uorb_queue_length(::Type{VehicleLandDetectedMsg}) = 1
uorb_fields_hash(::Type{VehicleLandDetectedMsg}) = 0x65d2467c593573a0
uorb_message_hash(::Type{VehicleLandDetectedMsg}) = 0x4c30e23f
uorb_fields(
    ::Type{VehicleLandDetectedMsg},
) = "uint64_t timestamp;bool freefall;bool ground_contact;bool maybe_landed;bool landed;bool in_ground_effect;bool in_descend;bool has_low_throttle;bool vertical_movement;bool horizontal_movement;bool rotational_movement;bool close_to_ground_or_skipped_check;bool at_rest"

struct VehicleStatusMsg <: UORBMsg
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

uorb_topic(::Type{VehicleStatusMsg}) = "vehicle_status"
uorb_queue_length(::Type{VehicleStatusMsg}) = 1
uorb_fields_hash(::Type{VehicleStatusMsg}) = 0xef9369299929c0f5
uorb_message_hash(::Type{VehicleStatusMsg}) = 0xf779fa65
uorb_fields(
    ::Type{VehicleStatusMsg},
) = "uint64_t timestamp;uint64_t armed_time;uint64_t takeoff_time;uint64_t nav_state_timestamp;uint32_t valid_nav_states_mask;uint32_t can_set_nav_states_mask;uint16_t failure_detector_status;uint8_t arming_state;uint8_t latest_arming_reason;uint8_t latest_disarming_reason;uint8_t nav_state_user_intention;uint8_t nav_state;uint8_t executor_in_charge;uint8_t hil_state;uint8_t vehicle_type;bool failsafe;bool failsafe_and_user_took_over;uint8_t failsafe_defer_state;bool gcs_connection_lost;uint8_t gcs_connection_lost_counter;bool high_latency_data_link_lost;bool is_vtol;bool is_vtol_tailsitter;bool in_transition_mode;bool in_transition_to_fw;uint8_t system_type;uint8_t system_id;uint8_t component_id;bool safety_button_available;bool safety_off;bool power_input_valid;bool usb_connected;bool open_drone_id_system_present;bool open_drone_id_system_healthy;bool parachute_system_present;bool parachute_system_healthy;bool rc_calibration_in_progress;bool calibration_enabled;bool pre_flight_checks_pass"

struct VehicleControlModeMsg <: UORBMsg
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

uorb_topic(::Type{VehicleControlModeMsg}) = "vehicle_control_mode"
uorb_queue_length(::Type{VehicleControlModeMsg}) = 1
uorb_fields_hash(::Type{VehicleControlModeMsg}) = 0x02d9a65bb7015a29
uorb_message_hash(::Type{VehicleControlModeMsg}) = 0x25a90612
uorb_fields(
    ::Type{VehicleControlModeMsg},
) = "uint64_t timestamp;bool flag_armed;bool flag_multicopter_position_control_enabled;bool flag_control_manual_enabled;bool flag_control_auto_enabled;bool flag_control_offboard_enabled;bool flag_control_position_enabled;bool flag_control_velocity_enabled;bool flag_control_altitude_enabled;bool flag_control_climb_rate_enabled;bool flag_control_acceleration_enabled;bool flag_control_attitude_enabled;bool flag_control_rates_enabled;bool flag_control_allocation_enabled;bool flag_control_termination_enabled;uint8_t source_id"

struct ActuatorArmedMsg <: UORBMsg
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

uorb_topic(::Type{ActuatorArmedMsg}) = "actuator_armed"
uorb_queue_length(::Type{ActuatorArmedMsg}) = 1
uorb_fields_hash(::Type{ActuatorArmedMsg}) = 0x0b7e9d8276720dc7
uorb_message_hash(::Type{ActuatorArmedMsg}) = 0x77773c05
uorb_fields(
    ::Type{ActuatorArmedMsg},
) = "uint64_t timestamp;bool armed;bool prearmed;bool ready_to_arm;bool lockdown;bool kill;bool termination;bool in_esc_calibration_mode"

struct HomePositionMsg <: UORBMsg
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

uorb_topic(::Type{HomePositionMsg}) = "home_position"
uorb_queue_length(::Type{HomePositionMsg}) = 1
uorb_fields_hash(::Type{HomePositionMsg}) = 0x9d7c8e4fdf3e3e5d
uorb_message_hash(::Type{HomePositionMsg}) = 0x159bab7e
uorb_fields(
    ::Type{HomePositionMsg},
) = "uint64_t timestamp;double lat;double lon;float alt;float x;float y;float z;float roll;float pitch;float yaw;uint32_t update_count;bool valid_alt;bool valid_hpos;bool valid_lpos;bool manual_home"

struct GeofenceStatusMsg <: UORBMsg
    timestamp::UInt64
    geofence_id::UInt32
    status::UInt8
    _padding0::NTuple{3,UInt8}
end

uorb_topic(::Type{GeofenceStatusMsg}) = "geofence_status"
uorb_queue_length(::Type{GeofenceStatusMsg}) = 1
uorb_fields_hash(::Type{GeofenceStatusMsg}) = 0xfd44f1dd8785a29d
uorb_message_hash(::Type{GeofenceStatusMsg}) = 0x5e9910da
uorb_fields(
    ::Type{GeofenceStatusMsg},
) = "uint64_t timestamp;uint32_t geofence_id;uint8_t status"

struct VehicleTorqueSetpointMsg <: UORBMsg
    timestamp::UInt64
    timestamp_sample::UInt64
    xyz::NTuple{3,Float32}
    _padding0::NTuple{4,UInt8}
end

uorb_topic(::Type{VehicleTorqueSetpointMsg}) = "vehicle_torque_setpoint"
uorb_queue_length(::Type{VehicleTorqueSetpointMsg}) = 1
uorb_fields_hash(::Type{VehicleTorqueSetpointMsg}) = 0x0bedf2b08526d772
uorb_message_hash(::Type{VehicleTorqueSetpointMsg}) = 0x67088e65
uorb_fields(
    ::Type{VehicleTorqueSetpointMsg},
) = "uint64_t timestamp;uint64_t timestamp_sample;float[3] xyz"

struct VehicleThrustSetpointMsg <: UORBMsg
    timestamp::UInt64
    timestamp_sample::UInt64
    xyz::NTuple{3,Float32}
    _padding0::NTuple{4,UInt8}
end

uorb_topic(::Type{VehicleThrustSetpointMsg}) = "vehicle_thrust_setpoint"
uorb_queue_length(::Type{VehicleThrustSetpointMsg}) = 1
uorb_fields_hash(::Type{VehicleThrustSetpointMsg}) = 0x0bedf2b08526d772
uorb_message_hash(::Type{VehicleThrustSetpointMsg}) = 0x67088e65
uorb_fields(
    ::Type{VehicleThrustSetpointMsg},
) = "uint64_t timestamp;uint64_t timestamp_sample;float[3] xyz"

struct ActuatorMotorsMsg <: UORBMsg
    timestamp::UInt64
    timestamp_sample::UInt64
    control::NTuple{12,Float32}
    reversible_flags::UInt16
    _padding0::NTuple{6,UInt8}
end

uorb_topic(::Type{ActuatorMotorsMsg}) = "actuator_motors"
uorb_queue_length(::Type{ActuatorMotorsMsg}) = 1
uorb_fields_hash(::Type{ActuatorMotorsMsg}) = 0xadc2b527fbb92c8e
uorb_message_hash(::Type{ActuatorMotorsMsg}) = 0xcabc4d45
uorb_fields(
    ::Type{ActuatorMotorsMsg},
) = "uint64_t timestamp;uint64_t timestamp_sample;float[12] control;uint16_t reversible_flags"

struct ActuatorServosMsg <: UORBMsg
    timestamp::UInt64
    timestamp_sample::UInt64
    control::NTuple{8,Float32}
end

uorb_topic(::Type{ActuatorServosMsg}) = "actuator_servos"
uorb_queue_length(::Type{ActuatorServosMsg}) = 1
uorb_fields_hash(::Type{ActuatorServosMsg}) = 0x71978c93f3352e79
uorb_message_hash(::Type{ActuatorServosMsg}) = 0xc5d59a9e
uorb_fields(
    ::Type{ActuatorServosMsg},
) = "uint64_t timestamp;uint64_t timestamp_sample;float[8] control"

struct VehicleAttitudeSetpointMsg <: UORBMsg
    timestamp::UInt64
    yaw_sp_move_rate::Float32
    q_d::NTuple{4,Float32}
    thrust_body::NTuple{3,Float32}
end

uorb_topic(::Type{VehicleAttitudeSetpointMsg}) = "vehicle_attitude_setpoint"
uorb_queue_length(::Type{VehicleAttitudeSetpointMsg}) = 1
uorb_fields_hash(::Type{VehicleAttitudeSetpointMsg}) = 0x07e58c36d4e5c5f7
uorb_message_hash(::Type{VehicleAttitudeSetpointMsg}) = 0x0591bb5a
uorb_fields(
    ::Type{VehicleAttitudeSetpointMsg},
) = "uint64_t timestamp;float yaw_sp_move_rate;float[4] q_d;float[3] thrust_body"

struct VehicleRatesSetpointMsg <: UORBMsg
    timestamp::UInt64
    roll::Float32
    pitch::Float32
    yaw::Float32
    thrust_body::NTuple{3,Float32}
    reset_integral::Bool
    _padding0::NTuple{7,UInt8}
end

uorb_topic(::Type{VehicleRatesSetpointMsg}) = "vehicle_rates_setpoint"
uorb_queue_length(::Type{VehicleRatesSetpointMsg}) = 1
uorb_fields_hash(::Type{VehicleRatesSetpointMsg}) = 0xbd6244bc0d3fcb0d
uorb_message_hash(::Type{VehicleRatesSetpointMsg}) = 0x79992167
uorb_fields(
    ::Type{VehicleRatesSetpointMsg},
) = "uint64_t timestamp;float roll;float pitch;float yaw;float[3] thrust_body;bool reset_integral"

struct MissionResultMsg <: UORBMsg
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

uorb_topic(::Type{MissionResultMsg}) = "mission_result"
uorb_queue_length(::Type{MissionResultMsg}) = 1
uorb_fields_hash(::Type{MissionResultMsg}) = 0xb192f0a8df4e74b2
uorb_message_hash(::Type{MissionResultMsg}) = 0x70caef92
uorb_fields(
    ::Type{MissionResultMsg},
) = "uint64_t timestamp;uint32_t mission_id;uint32_t geofence_id;uint32_t home_position_counter;int32_t seq_reached;uint16_t seq_current;uint16_t seq_total;uint16_t item_changed_index;uint16_t item_do_jump_remaining;bool valid;bool warning;bool finished;bool failure;bool item_do_jump_changed;uint8_t execution_mode"

struct TrajectorySetpointMsg <: UORBMsg
    timestamp::UInt64
    position::NTuple{3,Float32}
    velocity::NTuple{3,Float32}
    acceleration::NTuple{3,Float32}
    jerk::NTuple{3,Float32}
    yaw::Float32
    yawspeed::Float32
end

uorb_topic(::Type{TrajectorySetpointMsg}) = "trajectory_setpoint"
uorb_queue_length(::Type{TrajectorySetpointMsg}) = 1
uorb_fields_hash(::Type{TrajectorySetpointMsg}) = 0x3e78725f04131b37
uorb_message_hash(::Type{TrajectorySetpointMsg}) = 0xf614acde
uorb_fields(
    ::Type{TrajectorySetpointMsg},
) = "uint64_t timestamp;float[3] position;float[3] velocity;float[3] acceleration;float[3] jerk;float yaw;float yawspeed"

const UORB_ALL_TYPES = (
    BatteryStatusMsg,
    VehicleAttitudeMsg,
    VehicleLocalPositionMsg,
    VehicleGlobalPositionMsg,
    VehicleAngularVelocityMsg,
    VehicleLandDetectedMsg,
    VehicleStatusMsg,
    VehicleControlModeMsg,
    ActuatorArmedMsg,
    HomePositionMsg,
    GeofenceStatusMsg,
    VehicleTorqueSetpointMsg,
    VehicleThrustSetpointMsg,
    ActuatorMotorsMsg,
    ActuatorServosMsg,
    VehicleAttitudeSetpointMsg,
    VehicleRatesSetpointMsg,
    MissionResultMsg,
    TrajectorySetpointMsg,
)
