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
