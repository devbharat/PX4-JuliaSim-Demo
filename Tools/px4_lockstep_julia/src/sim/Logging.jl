"""PX4Lockstep.Sim.Logging

Deterministic logging utilities.

This module provides two logger backends:

1) `SimLog` (in-memory, columnar, low-dependency)
2) `CSVLogSink` (streaming to disk during simulation)

Design goals:
- Logging should be explicit and easy to extend.
- Logging should not mutate simulation state (only record it).
- The log format should be stable enough for external tools/scripts.
"""
module Logging

using ..Types: Vec3
using ..RigidBody: RigidBodyState
using ..Vehicles: ActuatorCommand
using ..Powertrain: BatteryStatus
using Printf

export AbstractLogSink, SimLog, CSVLogSink, close!, log!, write_csv
export reserve!

# Bump this when you add/remove/rename columns in the CSV output.
const CSV_SCHEMA_VERSION = 1

############################
# Log sinks
############################

abstract type AbstractLogSink end

"""In-memory columnar log.

This keeps allocations low while still being easy to export to CSV.
"""
mutable struct SimLog <: AbstractLogSink
    t::Vector{Float64}

    # truth state
    pos_ned::Vector{NTuple{3,Float64}}
    vel_ned::Vector{NTuple{3,Float64}}
    q_bn::Vector{NTuple{4,Float64}}
    ω_body::Vector{NTuple{3,Float64}}

    # setpoints (from PX4)
    pos_sp_ned::Vector{NTuple{3,Float64}}
    vel_sp_ned::Vector{NTuple{3,Float64}}
    acc_sp_ned::Vector{NTuple{3,Float64}}
    yaw_sp::Vector{Float64}
    yawspeed_sp::Vector{Float64}

    # actuator commands
    motor_cmd4::Vector{NTuple{4,Float64}}
    motor_cmd12::Vector{NTuple{12,Float64}}

    # propulsion outputs (optional)
    rotor_omega_rad_s::Vector{NTuple{4,Float64}}
    rotor_thrust_n::Vector{NTuple{4,Float64}}

    # environment (sampled at vehicle position)
    wind_ned::Vector{NTuple{3,Float64}}
    air_density::Vector{Float64}

    # air-relative velocity in body (wind - vel, rotated)
    air_vel_body::Vector{NTuple{3,Float64}}

    # battery
    batt_voltage_v::Vector{Float64}
    batt_current_a::Vector{Float64}
    batt_remaining::Vector{Float64}
    batt_warning::Vector{Int32}

    # PX4 status
    nav_state::Vector{Int32}
    arming_state::Vector{Int32}
    mission_seq::Vector{Int32}
    mission_count::Vector{Int32}
    mission_finished::Vector{Int32}
end

function SimLog()
    return SimLog(
        Float64[],
        NTuple{3,Float64}[],
        NTuple{3,Float64}[],
        NTuple{4,Float64}[],
        NTuple{3,Float64}[],
        NTuple{3,Float64}[],
        NTuple{3,Float64}[],
        NTuple{3,Float64}[],
        Float64[],
        Float64[],
        NTuple{4,Float64}[],
        NTuple{12,Float64}[],
        NTuple{4,Float64}[],
        NTuple{4,Float64}[],
        NTuple{3,Float64}[],
        Float64[],
        NTuple{3,Float64}[],
        Float64[],
        Float64[],
        Float64[],
        Int32[],
        Int32[],
        Int32[],
        Int32[],
        Int32[],
        Int32[],
    )
end

"""Hint the expected number of log samples.

This is a convenience to reduce allocations in long runs when using the in-memory
`SimLog` backend.
"""
function reserve!(log::SimLog, n::Int)
    n <= 0 && return log
    sizehint!(log.t, n)
    sizehint!(log.pos_ned, n)
    sizehint!(log.vel_ned, n)
    sizehint!(log.q_bn, n)
    sizehint!(log.ω_body, n)
    sizehint!(log.pos_sp_ned, n)
    sizehint!(log.vel_sp_ned, n)
    sizehint!(log.acc_sp_ned, n)
    sizehint!(log.yaw_sp, n)
    sizehint!(log.yawspeed_sp, n)
    sizehint!(log.motor_cmd4, n)
    sizehint!(log.motor_cmd12, n)
    sizehint!(log.rotor_omega_rad_s, n)
    sizehint!(log.rotor_thrust_n, n)
    sizehint!(log.wind_ned, n)
    sizehint!(log.air_density, n)
    sizehint!(log.air_vel_body, n)
    sizehint!(log.batt_voltage_v, n)
    sizehint!(log.batt_current_a, n)
    sizehint!(log.batt_remaining, n)
    sizehint!(log.batt_warning, n)
    sizehint!(log.nav_state, n)
    sizehint!(log.arming_state, n)
    sizehint!(log.mission_seq, n)
    sizehint!(log.mission_count, n)
    sizehint!(log.mission_finished, n)
    return log
end

"""Streaming CSV sink.

This is useful for long runs where in-memory storage is not desired.
"""
mutable struct CSVLogSink <: AbstractLogSink
    io::IO
    wrote_header::Bool
end

function CSVLogSink(path::AbstractString; append::Bool = false)
    io = open(path, append ? "a" : "w")
    sink = CSVLogSink(io, false)
    return sink
end

function close!(sink::CSVLogSink)
    try
        close(sink.io)
    catch
    end
    return nothing
end

close!(::SimLog) = nothing

############################
# Logging API
############################

const _CSV_HEADER = join(
    [
        "time_s",
        # truth
        "pos_x",
        "pos_y",
        "pos_z",
        "vel_x",
        "vel_y",
        "vel_z",
        "q_w",
        "q_x",
        "q_y",
        "q_z",
        "p",
        "q",
        "r",
        # setpoints
        "pos_sp_x",
        "pos_sp_y",
        "pos_sp_z",
        "vel_sp_x",
        "vel_sp_y",
        "vel_sp_z",
        "acc_sp_x",
        "acc_sp_y",
        "acc_sp_z",
        "yaw_sp",
        "yawspeed_sp",
        # motor cmd
        "m1",
        "m2",
        "m3",
        "m4",
        "m5",
        "m6",
        "m7",
        "m8",
        "m9",
        "m10",
        "m11",
        "m12",
        # rotor outputs (optional)
        "rotor_w1",
        "rotor_w2",
        "rotor_w3",
        "rotor_w4",
        "rotor_T1",
        "rotor_T2",
        "rotor_T3",
        "rotor_T4",
        # environment
        "wind_x",
        "wind_y",
        "wind_z",
        "rho",
        "air_bx",
        "air_by",
        "air_bz",
        # battery
        "batt_v",
        "batt_a",
        "batt_rem",
        "batt_warn",
        # px4
        "nav_state",
        "arming_state",
        "mission_seq",
        "mission_count",
        "mission_finished",
    ],
    ",",
)

@inline function _motor12_from_cmd(cmd::ActuatorCommand)
    m = cmd.motors
    return (m[1], m[2], m[3], m[4], m[5], m[6], m[7], m[8], m[9], m[10], m[11], m[12])
end

@inline function _motor4_from_cmd(cmd::ActuatorCommand)
    m = cmd.motors
    return (m[1], m[2], m[3], m[4])
end

"""Record one sample (generic entry point).

`wind_ned` and `rho` should be the environment sampled at the vehicle position/time.
"""
function log!(
    sink::SimLog,
    t::Float64,
    x::RigidBodyState,
    cmd::ActuatorCommand;
    wind_ned::Vec3,
    rho::Float64,
    air_vel_body::NTuple{3,Float64} = (NaN, NaN, NaN),
    battery::BatteryStatus = BatteryStatus(),
    nav_state::Int32 = Int32(-1),
    arming_state::Int32 = Int32(-1),
    mission_seq::Int32 = Int32(0),
    mission_count::Int32 = Int32(0),
    mission_finished::Int32 = Int32(0),
    pos_sp::NTuple{3,Float64} = (NaN, NaN, NaN),
    vel_sp::NTuple{3,Float64} = (NaN, NaN, NaN),
    acc_sp::NTuple{3,Float64} = (NaN, NaN, NaN),
    yaw_sp::Float64 = NaN,
    yawspeed_sp::Float64 = NaN,
    rotor_omega::NTuple{4,Float64} = (NaN, NaN, NaN, NaN),
    rotor_thrust::NTuple{4,Float64} = (NaN, NaN, NaN, NaN),
)
    push!(sink.t, t)
    push!(sink.pos_ned, (x.pos_ned[1], x.pos_ned[2], x.pos_ned[3]))
    push!(sink.vel_ned, (x.vel_ned[1], x.vel_ned[2], x.vel_ned[3]))
    push!(sink.q_bn, (x.q_bn[1], x.q_bn[2], x.q_bn[3], x.q_bn[4]))
    push!(sink.ω_body, (x.ω_body[1], x.ω_body[2], x.ω_body[3]))

    push!(sink.pos_sp_ned, pos_sp)
    push!(sink.vel_sp_ned, vel_sp)
    push!(sink.acc_sp_ned, acc_sp)
    push!(sink.yaw_sp, yaw_sp)
    push!(sink.yawspeed_sp, yawspeed_sp)

    push!(sink.motor_cmd4, _motor4_from_cmd(cmd))
    push!(sink.motor_cmd12, _motor12_from_cmd(cmd))

    push!(sink.rotor_omega_rad_s, rotor_omega)
    push!(sink.rotor_thrust_n, rotor_thrust)

    push!(sink.wind_ned, (wind_ned[1], wind_ned[2], wind_ned[3]))
    push!(sink.air_density, rho)
    push!(sink.air_vel_body, air_vel_body)

    push!(sink.batt_voltage_v, battery.voltage_v)
    push!(sink.batt_current_a, battery.current_a)
    push!(sink.batt_remaining, battery.remaining)
    push!(sink.batt_warning, Int32(battery.warning))

    push!(sink.nav_state, nav_state)
    push!(sink.arming_state, arming_state)
    push!(sink.mission_seq, mission_seq)
    push!(sink.mission_count, mission_count)
    push!(sink.mission_finished, mission_finished)
    return nothing
end

function log!(
    sink::CSVLogSink,
    t::Float64,
    x::RigidBodyState,
    cmd::ActuatorCommand;
    wind_ned::Vec3,
    rho::Float64,
    air_vel_body::NTuple{3,Float64} = (NaN, NaN, NaN),
    battery::BatteryStatus = BatteryStatus(),
    nav_state::Int32 = Int32(-1),
    arming_state::Int32 = Int32(-1),
    mission_seq::Int32 = Int32(0),
    mission_count::Int32 = Int32(0),
    mission_finished::Int32 = Int32(0),
    pos_sp::NTuple{3,Float64} = (NaN, NaN, NaN),
    vel_sp::NTuple{3,Float64} = (NaN, NaN, NaN),
    acc_sp::NTuple{3,Float64} = (NaN, NaN, NaN),
    yaw_sp::Float64 = NaN,
    yawspeed_sp::Float64 = NaN,
    rotor_omega::NTuple{4,Float64} = (NaN, NaN, NaN, NaN),
    rotor_thrust::NTuple{4,Float64} = (NaN, NaN, NaN, NaN),
)
    io = sink.io
    if !sink.wrote_header
        println(io, "# schema_version=$(CSV_SCHEMA_VERSION)")
        println(io, _CSV_HEADER)
        sink.wrote_header = true
    end

    m = cmd.motors

    # Write a single row. We use printf-style formatting for floats to keep file size manageable.
    @printf(io, "%.6f,", t)
    # truth pos/vel
    @printf(io, "%.6f,%.6f,%.6f,", x.pos_ned[1], x.pos_ned[2], x.pos_ned[3])
    @printf(io, "%.6f,%.6f,%.6f,", x.vel_ned[1], x.vel_ned[2], x.vel_ned[3])
    # quat
    @printf(io, "%.8f,%.8f,%.8f,%.8f,", x.q_bn[1], x.q_bn[2], x.q_bn[3], x.q_bn[4])
    # body rates
    @printf(io, "%.6f,%.6f,%.6f,", x.ω_body[1], x.ω_body[2], x.ω_body[3])
    # setpoints
    @printf(io, "%.6f,%.6f,%.6f,", pos_sp[1], pos_sp[2], pos_sp[3])
    @printf(io, "%.6f,%.6f,%.6f,", vel_sp[1], vel_sp[2], vel_sp[3])
    @printf(io, "%.6f,%.6f,%.6f,", acc_sp[1], acc_sp[2], acc_sp[3])
    @printf(io, "%.6f,%.6f,", yaw_sp, yawspeed_sp)
    # motors 1..12
    @printf(
        io,
        "%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,",
        m[1],
        m[2],
        m[3],
        m[4],
        m[5],
        m[6],
        m[7],
        m[8],
        m[9],
        m[10],
        m[11],
        m[12]
    )
    # rotor outputs
    @printf(
        io,
        "%.6f,%.6f,%.6f,%.6f,",
        rotor_omega[1],
        rotor_omega[2],
        rotor_omega[3],
        rotor_omega[4]
    )
    @printf(
        io,
        "%.6f,%.6f,%.6f,%.6f,",
        rotor_thrust[1],
        rotor_thrust[2],
        rotor_thrust[3],
        rotor_thrust[4]
    )
    # environment
    @printf(io, "%.6f,%.6f,%.6f,%.6f,", wind_ned[1], wind_ned[2], wind_ned[3], rho)
    @printf(io, "%.6f,%.6f,%.6f,", air_vel_body[1], air_vel_body[2], air_vel_body[3])
    # battery
    @printf(
        io,
        "%.6f,%.6f,%.6f,%d,",
        battery.voltage_v,
        battery.current_a,
        battery.remaining,
        battery.warning
    )
    # px4
    @printf(
        io,
        "%d,%d,%d,%d,%d\n",
        nav_state,
        arming_state,
        mission_seq,
        mission_count,
        mission_finished
    )

    return nothing
end

"""Write the full in-memory log to CSV.

This is mostly for convenience; for long runs, consider using `CSVLogSink`.
"""
function write_csv(log::SimLog, path::AbstractString)
    open(path, "w") do io
        println(io, "# schema_version=$(CSV_SCHEMA_VERSION)")
        println(io, _CSV_HEADER)
        n = length(log.t)
        for i = 1:n
            p = log.pos_ned[i]
            v = log.vel_ned[i]
            q = log.q_bn[i]
            w = log.ω_body[i]
            psp = log.pos_sp_ned[i]
            vsp = log.vel_sp_ned[i]
            asp = log.acc_sp_ned[i]
            m12 = log.motor_cmd12[i]
            wind = log.wind_ned[i]
            airb = log.air_vel_body[i]
            ωr = log.rotor_omega_rad_s[i]
            Tr = log.rotor_thrust_n[i]

            @printf(io, "%.6f,", log.t[i])
            @printf(io, "%.6f,%.6f,%.6f,", p[1], p[2], p[3])
            @printf(io, "%.6f,%.6f,%.6f,", v[1], v[2], v[3])
            @printf(io, "%.8f,%.8f,%.8f,%.8f,", q[1], q[2], q[3], q[4])
            @printf(io, "%.6f,%.6f,%.6f,", w[1], w[2], w[3])
            @printf(io, "%.6f,%.6f,%.6f,", psp[1], psp[2], psp[3])
            @printf(io, "%.6f,%.6f,%.6f,", vsp[1], vsp[2], vsp[3])
            @printf(io, "%.6f,%.6f,%.6f,", asp[1], asp[2], asp[3])
            @printf(io, "%.6f,%.6f,", log.yaw_sp[i], log.yawspeed_sp[i])

            @printf(
                io,
                "%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,",
                m12[1],
                m12[2],
                m12[3],
                m12[4],
                m12[5],
                m12[6],
                m12[7],
                m12[8],
                m12[9],
                m12[10],
                m12[11],
                m12[12]
            )

            @printf(io, "%.6f,%.6f,%.6f,%.6f,", ωr[1], ωr[2], ωr[3], ωr[4])
            @printf(io, "%.6f,%.6f,%.6f,%.6f,", Tr[1], Tr[2], Tr[3], Tr[4])

            @printf(
                io,
                "%.6f,%.6f,%.6f,%.6f,",
                wind[1],
                wind[2],
                wind[3],
                log.air_density[i]
            )
            @printf(io, "%.6f,%.6f,%.6f,", airb[1], airb[2], airb[3])

            @printf(
                io,
                "%.6f,%.6f,%.6f,%d,",
                log.batt_voltage_v[i],
                log.batt_current_a[i],
                log.batt_remaining[i],
                log.batt_warning[i]
            )

            @printf(
                io,
                "%d,%d,%d,%d,%d\n",
                log.nav_state[i],
                log.arming_state[i],
                log.mission_seq[i],
                log.mission_count[i],
                log.mission_finished[i]
            )
        end
    end
    return nothing
end

end # module Logging
