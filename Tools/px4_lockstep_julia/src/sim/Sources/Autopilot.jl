"""Sources.Autopilot

Autopilot sources publish actuator commands into the runtime bus.

Two common modes:
- **Live**: drive PX4 (or another autopilot) through the lockstep ABI at `timeline.ap`.
- **Replay**: sample a recorded actuator-command trace and publish it deterministically.

Autopilot commands (`bus.cmd`) are treated as **ZOH between autopilot ticks**.
"""

using StaticArrays: SVector

using ..Autopilots: autopilot_step

"""Base type for autopilot sources."""
abstract type AbstractAutopilotSource <: AbstractSource end


"""Live autopilot source.

This wraps `Sim.Autopilots.autopilot_step`.

Consumes:
- `bus.est` (estimated state)
- `bus.ap_cmd` (arm/mission/rtl)
- `bus.landed`
- `bus.battery`

Publishes:
- `bus.cmd` (actuator command)
"""
mutable struct LiveAutopilotSource{A} <: AbstractAutopilotSource
    ap::A
    last_out::Any
end

LiveAutopilotSource(ap) = LiveAutopilotSource(ap, nothing)

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
        battery = bus.battery,
    )

    src.last_out = out

    motors_raw = out.actuator_motors
    servos_raw = out.actuator_servos

    motors = SVector{12,Float64}(ntuple(i -> Float64(motors_raw[i]), 12))
    servos = SVector{8,Float64}(ntuple(i -> Float64(servos_raw[i]), 8))

    bus.cmd = ActuatorCommand(motors = motors, servos = servos)
    return nothing
end

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

export AbstractAutopilotSource, LiveAutopilotSource, ReplayAutopilotSource
