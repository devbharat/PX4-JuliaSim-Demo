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
    end

    return out
end
