struct UORBSubSlot{T,S}
    sub::S
    inst::Int32
    buf::Base.RefValue{T}
    updated::Base.RefValue{Int32}
end

Base.@kwdef mutable struct UORBReadSlotsReal
    torque_sp::Union{
        Nothing,
        UORBSubSlot{VehicleTorqueSetpointMsg,UORBSubscriber{VehicleTorqueSetpointMsg}},
    } = nothing
    thrust_sp::Union{
        Nothing,
        UORBSubSlot{VehicleThrustSetpointMsg,UORBSubscriber{VehicleThrustSetpointMsg}},
    } = nothing
    actuator_motors::Union{
        Nothing,
        UORBSubSlot{ActuatorMotorsMsg,UORBSubscriber{ActuatorMotorsMsg}},
    } = nothing
    actuator_servos::Union{
        Nothing,
        UORBSubSlot{ActuatorServosMsg,UORBSubscriber{ActuatorServosMsg}},
    } = nothing
    attitude_sp::Union{
        Nothing,
        UORBSubSlot{VehicleAttitudeSetpointMsg,UORBSubscriber{VehicleAttitudeSetpointMsg}},
    } = nothing
    rates_sp::Union{
        Nothing,
        UORBSubSlot{VehicleRatesSetpointMsg,UORBSubscriber{VehicleRatesSetpointMsg}},
    } = nothing
    mission_result::Union{
        Nothing,
        UORBSubSlot{MissionResultMsg,UORBSubscriber{MissionResultMsg}},
    } = nothing
    vehicle_status::Union{
        Nothing,
        UORBSubSlot{VehicleStatusMsg,UORBSubscriber{VehicleStatusMsg}},
    } = nothing
    battery_status::Union{
        Nothing,
        UORBSubSlot{BatteryStatusMsg,UORBSubscriber{BatteryStatusMsg}},
    } = nothing
    trajectory_setpoint::Union{
        Nothing,
        UORBSubSlot{TrajectorySetpointMsg,UORBSubscriber{TrajectorySetpointMsg}},
    } = nothing
end

Base.@kwdef mutable struct UORBReadSlotsMock
    torque_sp::Union{
        Nothing,
        UORBSubSlot{VehicleTorqueSetpointMsg,MockSubscriber{VehicleTorqueSetpointMsg}},
    } = nothing
    thrust_sp::Union{
        Nothing,
        UORBSubSlot{VehicleThrustSetpointMsg,MockSubscriber{VehicleThrustSetpointMsg}},
    } = nothing
    actuator_motors::Union{
        Nothing,
        UORBSubSlot{ActuatorMotorsMsg,MockSubscriber{ActuatorMotorsMsg}},
    } = nothing
    actuator_servos::Union{
        Nothing,
        UORBSubSlot{ActuatorServosMsg,MockSubscriber{ActuatorServosMsg}},
    } = nothing
    attitude_sp::Union{
        Nothing,
        UORBSubSlot{VehicleAttitudeSetpointMsg,MockSubscriber{VehicleAttitudeSetpointMsg}},
    } = nothing
    rates_sp::Union{
        Nothing,
        UORBSubSlot{VehicleRatesSetpointMsg,MockSubscriber{VehicleRatesSetpointMsg}},
    } = nothing
    mission_result::Union{
        Nothing,
        UORBSubSlot{MissionResultMsg,MockSubscriber{MissionResultMsg}},
    } = nothing
    vehicle_status::Union{
        Nothing,
        UORBSubSlot{VehicleStatusMsg,MockSubscriber{VehicleStatusMsg}},
    } = nothing
    battery_status::Union{
        Nothing,
        UORBSubSlot{BatteryStatusMsg,MockSubscriber{BatteryStatusMsg}},
    } = nothing
    trajectory_setpoint::Union{
        Nothing,
        UORBSubSlot{TrajectorySetpointMsg,MockSubscriber{TrajectorySetpointMsg}},
    } = nothing
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

mutable struct UORBBridge{B<:AbstractUORBBackend,RS}
    backend::B
    handle::LockstepHandle

    # Allow multiple uORB instances per logical key.
    # Each entry is (handle, instance).
    pubs::Dict{Symbol,Vector{Tuple{Any,Int32}}}
    subs::Dict{Symbol,Vector{Tuple{Any,Int32}}}

    read_slots::RS
end
