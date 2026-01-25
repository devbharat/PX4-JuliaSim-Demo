"""Sources.Autopilot

Autopilot sources publish actuator commands into the runtime bus.

Two common modes:
- **Live**: drive PX4 (or another autopilot) through the lockstep ABI at `timeline.ap`.
- **Replay**: sample a recorded actuator-command trace and publish it deterministically.

Autopilot commands (`bus.cmd`) are treated as **ZOH between autopilot ticks**.
"""

using StaticArrays: SVector

using ..Autopilots: autopilot_step, autopilot_output_type, UORBOutputs
import ..Runtime: AutopilotTelemetry, autopilot_telemetry

"""Base type for autopilot sources."""
abstract type AbstractAutopilotSource <: AbstractSource end


"""Live autopilot source.

This wraps `Sim.Autopilots.autopilot_step`.

Consumes:
- `bus.est` (estimated state)
- `bus.ap_cmd` (arm/mission/rtl)
- `bus.landed`
- `bus.batteries` (battery 1 is the primary)

Publishes:
- `bus.cmd` (actuator command)
"""
mutable struct LiveAutopilotSource{A,O} <: AbstractAutopilotSource
    ap::A
    last_out::Union{Nothing,O}
    telemetry::AutopilotTelemetry
end

function LiveAutopilotSource(ap::A) where {A}
    O = autopilot_output_type(ap)
    return LiveAutopilotSource{A,O}(ap, nothing, AutopilotTelemetry())
end

@inline function _to_ntuple3_float(v)
    return (Float64(v[1]), Float64(v[2]), Float64(v[3]))
end

@inline _telemetry_from_out(::Any) = AutopilotTelemetry()

@inline function _telemetry_from_out(out::UORBOutputs)
    return AutopilotTelemetry(
        pos_sp = _to_ntuple3_float(out.trajectory_setpoint_position),
        vel_sp = _to_ntuple3_float(out.trajectory_setpoint_velocity),
        acc_sp = _to_ntuple3_float(out.trajectory_setpoint_acceleration),
        yaw_sp = Float64(out.trajectory_setpoint_yaw),
        yawspeed_sp = Float64(out.trajectory_setpoint_yawspeed),
        nav_state = Int32(out.nav_state),
        arming_state = Int32(out.arming_state),
        mission_seq = Int32(out.mission_seq),
        mission_count = Int32(out.mission_count),
        mission_finished = Int32(out.mission_finished),
    )
end

function update!(src::LiveAutopilotSource, bus::SimBus, plant_state, t_us::UInt64)
    est = bus.est
    out = autopilot_step(
        src.ap,
        t_us,
        est.pos_ned,
        est.vel_ned,
        est.q_bn,
        est.ω_body,
        bus.ap_cmd;
        landed = bus.landed,
        battery = bus.batteries[1],
        batteries = bus.batteries,
    )

    src.last_out = out
    src.telemetry = _telemetry_from_out(out)

    motors_raw = out.actuator_motors
    servos_raw = out.actuator_servos

    motors = SVector{12,Float64}(ntuple(i -> Float64(motors_raw[i]), 12))
    servos = SVector{8,Float64}(ntuple(i -> Float64(servos_raw[i]), 8))

    bus.cmd = ActuatorCommand(motors = motors, servos = servos)
    return nothing
end

@inline autopilot_telemetry(src::LiveAutopilotSource) = src.telemetry

"""Replay autopilot source.

Samples a ZOH command trace and publishes into `bus.cmd`.
"""
struct ReplayAutopilotSource{T<:ZOHTrace{ActuatorCommand}} <: AbstractAutopilotSource
    cmd_trace::T
end

function update!(src::ReplayAutopilotSource, bus::SimBus, plant_state, t_us::UInt64)
    bus.cmd = sample(src.cmd_trace, t_us)
    return nothing
end

@inline autopilot_telemetry(::ReplayAutopilotSource) = AutopilotTelemetry()

export AbstractAutopilotSource, LiveAutopilotSource, ReplayAutopilotSource
