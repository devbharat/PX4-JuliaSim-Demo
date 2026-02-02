using PX4Lockstep: LockstepConfig
using PX4Lockstep: UORBPublisher, UORBSubscriber, UORBMsg
using PX4Lockstep: create_publisher, create_subscriber, publish!
using PX4Lockstep: uorb_check!, uorb_copy!, uorb_unsubscribe!
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

abstract type AbstractUORBBackend end

struct RealUORBBackend <: AbstractUORBBackend end

mutable struct MockUORBBackend <: AbstractUORBBackend
    latest::Dict{Tuple{Symbol,Int32},Any}
    gen::Dict{Tuple{Symbol,Int32},UInt64}
end

MockUORBBackend() =
    MockUORBBackend(Dict{Tuple{Symbol,Int32},Any}(), Dict{Tuple{Symbol,Int32},UInt64}())

mutable struct MockPublisher
    key::Symbol
    instance::Int32
end

mutable struct MockSubscriber{T<:UORBMsg}
    key::Symbol
    instance::Int32
    last_gen::UInt64
end

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

@inline function _backend_create_publisher(
    ::RealUORBBackend,
    handle::LockstepHandle,
    _key::Symbol,
    ::Type{T};
    priority::Int32,
    queue_size,
    instance::Int32,
) where {T<:UORBMsg}
    return create_publisher(
        handle,
        T;
        priority = priority,
        queue_size = queue_size,
        instance = instance,
    )
end

@inline function _backend_create_subscriber(
    ::RealUORBBackend,
    handle::LockstepHandle,
    _key::Symbol,
    ::Type{T};
    instance::Int32,
) where {T<:UORBMsg}
    return create_subscriber(handle, T; instance = instance)
end

@inline function _backend_publish!(
    ::RealUORBBackend,
    handle::LockstepHandle,
    pub::UORBPublisher,
    msg,
)
    return publish!(handle, pub, msg)
end

@inline function _backend_check!(
    ::RealUORBBackend,
    handle::LockstepHandle,
    sub::UORBSubscriber,
    updated::Base.RefValue{Int32},
)::Bool
    return uorb_check!(handle, sub, updated)
end

@inline function _backend_copy!(
    ::RealUORBBackend,
    handle::LockstepHandle,
    sub::UORBSubscriber{T},
    buf::Base.RefValue{T},
) where {T<:UORBMsg}
    return uorb_copy!(handle, sub, buf)
end

@inline function _backend_unsubscribe!(
    ::RealUORBBackend,
    handle::LockstepHandle,
    sub::UORBSubscriber,
)
    return uorb_unsubscribe!(handle, sub)
end

@inline function _backend_create_publisher(
    ::MockUORBBackend,
    _handle::LockstepHandle,
    key::Symbol,
    ::Type{T};
    priority::Int32,
    queue_size,
    instance::Int32,
) where {T<:UORBMsg}
    return MockPublisher(key, instance), instance
end

@inline function _backend_create_subscriber(
    ::MockUORBBackend,
    _handle::LockstepHandle,
    key::Symbol,
    ::Type{T};
    instance::Int32,
) where {T<:UORBMsg}
    return MockSubscriber{T}(key, instance, UInt64(0))
end

@inline function _backend_publish!(
    backend::MockUORBBackend,
    _handle::LockstepHandle,
    pub::MockPublisher,
    msg,
)
    key = (pub.key, pub.instance)
    backend.latest[key] = msg
    backend.gen[key] = get(backend.gen, key, UInt64(0)) + UInt64(1)
    return nothing
end

@inline function _backend_check!(
    backend::MockUORBBackend,
    _handle::LockstepHandle,
    sub::MockSubscriber,
    updated::Base.RefValue{Int32},
)::Bool
    key = (sub.key, sub.instance)
    gen = get(backend.gen, key, UInt64(0))
    updated[] = gen > sub.last_gen ? Int32(1) : Int32(0)
    return updated[] != 0
end

@inline function _backend_copy!(
    backend::MockUORBBackend,
    _handle::LockstepHandle,
    sub::MockSubscriber{T},
    buf::Base.RefValue{T},
) where {T<:UORBMsg}
    key = (sub.key, sub.instance)
    msg = get(backend.latest, key, nothing)
    msg isa T || error("Mock uORB missing message for $(sub.key)[$(sub.instance)]")
    buf[] = msg
    sub.last_gen = get(backend.gen, key, sub.last_gen)
    return nothing
end

@inline function _backend_unsubscribe!(
    ::MockUORBBackend,
    _handle::LockstepHandle,
    _sub::MockSubscriber,
)
    return nothing
end

"""Publish a mock uORB message by key (for tests)."""
function mock_publish!(
    backend::MockUORBBackend,
    key::Symbol,
    msg;
    instance::Int32 = Int32(0),
)
    pub = MockPublisher(key, instance)
    _backend_publish!(
        backend,
        LockstepHandle(Ptr{Cvoid}(0), Ptr{Cvoid}(0), LockstepConfig()),
        pub,
        msg,
    )
    return nothing
end

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


function _init_uorb_bridge(
    handle::LockstepHandle,
    cfg::PX4UORBInterfaceConfig;
    backend::AbstractUORBBackend = RealUORBBackend(),
)
    pubs = Dict{Symbol,Vector{Tuple{Any,Int32}}}()
    for spec in cfg.pubs
        spec.type <: UORBMsg ||
            error("UORBPubSpec type must be a uORB message type (got $(spec.type))")
        pub, inst = _backend_create_publisher(
            backend,
            handle,
            spec.key,
            spec.type;
            priority = spec.priority,
            queue_size = spec.queue_size,
            instance = Int32(spec.instance),
        )
        v = get!(pubs, spec.key) do
            Vector{Tuple{Any,Int32}}()
        end
        push!(v, (pub, Int32(inst)))
    end

    subs = Dict{Symbol,Vector{Tuple{Any,Int32}}}()
    for spec in cfg.subs
        spec.type <: UORBMsg ||
            error("UORBSubSpec type must be a uORB message type (got $(spec.type))")
        sub = _backend_create_subscriber(
            backend,
            handle,
            spec.key,
            spec.type;
            instance = Int32(spec.instance),
        )
        v = get!(subs, spec.key) do
            Vector{Tuple{Any,Int32}}()
        end
        push!(v, (sub, Int32(spec.instance)))
    end

    read_slots = _build_read_slots(backend, subs)
    return UORBBridge(backend, handle, pubs, subs, read_slots)
end

@inline function _make_slot(
    ::RealUORBBackend,
    subs::Dict{Symbol,Vector{Tuple{Any,Int32}}},
    key::Symbol,
    ::Type{T},
) where {T<:UORBMsg}
    entries = get(subs, key, nothing)
    entries === nothing && return nothing
    sub_any, inst = entries[1]
    sub_any isa UORBSubscriber{T} ||
        error("uORB subscriber type mismatch for $key: expected $(UORBSubscriber{T})")
    sub = sub_any::UORBSubscriber{T}
    return UORBSubSlot{T,UORBSubscriber{T}}(sub, inst, Ref{T}(), Ref{Int32}(0))
end

@inline function _make_slot(
    ::MockUORBBackend,
    subs::Dict{Symbol,Vector{Tuple{Any,Int32}}},
    key::Symbol,
    ::Type{T},
) where {T<:UORBMsg}
    entries = get(subs, key, nothing)
    entries === nothing && return nothing
    sub_any, inst = entries[1]
    sub_any isa MockSubscriber{T} ||
        error("mock uORB subscriber type mismatch for $key: expected $(MockSubscriber{T})")
    sub = sub_any::MockSubscriber{T}
    return UORBSubSlot{T,MockSubscriber{T}}(sub, inst, Ref{T}(), Ref{Int32}(0))
end

function _build_read_slots(backend::RealUORBBackend, subs)
    return UORBReadSlotsReal(
        torque_sp = _make_slot(backend, subs, :torque_sp, VehicleTorqueSetpointMsg),
        thrust_sp = _make_slot(backend, subs, :thrust_sp, VehicleThrustSetpointMsg),
        actuator_motors = _make_slot(backend, subs, :actuator_motors, ActuatorMotorsMsg),
        actuator_servos = _make_slot(backend, subs, :actuator_servos, ActuatorServosMsg),
        attitude_sp = _make_slot(backend, subs, :attitude_sp, VehicleAttitudeSetpointMsg),
        rates_sp = _make_slot(backend, subs, :rates_sp, VehicleRatesSetpointMsg),
        mission_result = _make_slot(backend, subs, :mission_result, MissionResultMsg),
        vehicle_status = _make_slot(backend, subs, :vehicle_status, VehicleStatusMsg),
        battery_status = _make_slot(backend, subs, :battery_status, BatteryStatusMsg),
        trajectory_setpoint = _make_slot(
            backend,
            subs,
            :trajectory_setpoint,
            TrajectorySetpointMsg,
        ),
    )
end

function _build_read_slots(backend::MockUORBBackend, subs)
    return UORBReadSlotsMock(
        torque_sp = _make_slot(backend, subs, :torque_sp, VehicleTorqueSetpointMsg),
        thrust_sp = _make_slot(backend, subs, :thrust_sp, VehicleThrustSetpointMsg),
        actuator_motors = _make_slot(backend, subs, :actuator_motors, ActuatorMotorsMsg),
        actuator_servos = _make_slot(backend, subs, :actuator_servos, ActuatorServosMsg),
        attitude_sp = _make_slot(backend, subs, :attitude_sp, VehicleAttitudeSetpointMsg),
        rates_sp = _make_slot(backend, subs, :rates_sp, VehicleRatesSetpointMsg),
        mission_result = _make_slot(backend, subs, :mission_result, MissionResultMsg),
        vehicle_status = _make_slot(backend, subs, :vehicle_status, VehicleStatusMsg),
        battery_status = _make_slot(backend, subs, :battery_status, BatteryStatusMsg),
        trajectory_setpoint = _make_slot(
            backend,
            subs,
            :trajectory_setpoint,
            TrajectorySetpointMsg,
        ),
    )
end

function _close_uorb_bridge!(bridge::UORBBridge)
    for entries in values(bridge.subs)
        for (sub, _inst) in entries
            _backend_unsubscribe!(bridge.backend, bridge.handle, sub)
        end
    end
    return nothing
end

"""Publish a message to all configured publishers under `key` (if any)."""
function _publish_uorb!(bridge::UORBBridge, key::Symbol, msg)
    entries = get(bridge.pubs, key, nothing)
    entries === nothing && return nothing
    for (pub, _inst) in entries
        _backend_publish!(bridge.backend, bridge.handle, pub, msg)
    end
    return nothing
end

@inline _read_slot!(::UORBBridge, ::Nothing) = nothing

@inline function _read_slot!(bridge::UORBBridge{B}, slot::UORBSubSlot{T,S}) where {B,T,S}
    _backend_check!(bridge.backend, bridge.handle, slot.sub, slot.updated) || return nothing
    _backend_copy!(bridge.backend, bridge.handle, slot.sub, slot.buf)
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
    slots = bridge.read_slots
    controls = out.actuator_controls
    torque_msg = _read_slot!(bridge, slots.torque_sp)
    if torque_msg !== nothing
        controls = _update_controls_torque(controls, torque_msg)
    end
    thrust_msg = _read_slot!(bridge, slots.thrust_sp)
    if thrust_msg !== nothing
        controls = _update_controls_thrust(controls, thrust_msg)
    end
    out.actuator_controls = controls

    motors_msg = _read_slot!(bridge, slots.actuator_motors)
    if motors_msg !== nothing
        out.actuator_motors = motors_msg.control
    end

    servos_msg = _read_slot!(bridge, slots.actuator_servos)
    if servos_msg !== nothing
        out.actuator_servos = servos_msg.control
    end

    att_msg = _read_slot!(bridge, slots.attitude_sp)
    if att_msg !== nothing
        out.attitude_setpoint_q = att_msg.q_d
        out.thrust_setpoint_body = att_msg.thrust_body
    end

    rates_msg = _read_slot!(bridge, slots.rates_sp)
    if rates_msg !== nothing
        out.rates_setpoint_xyz = (rates_msg.roll, rates_msg.pitch, rates_msg.yaw)
    end

    mission_msg = _read_slot!(bridge, slots.mission_result)
    if mission_msg !== nothing
        out.mission_seq = Int32(mission_msg.seq_current)
        out.mission_count = Int32(mission_msg.seq_total)
        out.mission_finished = mission_msg.finished ? Int32(1) : Int32(0)
        out.mission_valid = mission_msg.valid ? Int32(1) : Int32(0)
    end

    vstatus_msg = _read_slot!(bridge, slots.vehicle_status)
    if vstatus_msg !== nothing
        out.nav_state = Int32(vstatus_msg.nav_state)
        out.arming_state = Int32(vstatus_msg.arming_state)
    end

    battery_msg = _read_slot!(bridge, slots.battery_status)
    if battery_msg !== nothing
        out.battery_warning = Int32(battery_msg.warning)
    end

    traj_msg = _read_slot!(bridge, slots.trajectory_setpoint)
    if traj_msg !== nothing
        out.trajectory_setpoint_position = traj_msg.position
        out.trajectory_setpoint_velocity = traj_msg.velocity
        out.trajectory_setpoint_acceleration = traj_msg.acceleration
        out.trajectory_setpoint_yaw = traj_msg.yaw
        out.trajectory_setpoint_yawspeed = traj_msg.yawspeed
    elseif out.nav_state != Int32(NAV_STATE_AUTO_MISSION) &&
           out.nav_state != Int32(NAV_STATE_AUTO_RTL)
        out.trajectory_setpoint_position = ZERO_VEC3_F32
        out.trajectory_setpoint_velocity = ZERO_VEC3_F32
        out.trajectory_setpoint_acceleration = ZERO_VEC3_F32
        out.trajectory_setpoint_yaw = 0.0f0
        out.trajectory_setpoint_yawspeed = 0.0f0
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
